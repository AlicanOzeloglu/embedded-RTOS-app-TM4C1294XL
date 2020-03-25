/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== httpget.c ========
 *  HTTP Client GET example application
 */
#include <string.h>

/* XDCtools Header files */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>

#include <ti/drivers/I2C.h>

/* Example/Board Header file */
#include "Board.h"

extern Event_Handle event0;

#include <sys/socket.h>

#define HOSTNAME          "api.openweathermap.org"
#define REQUEST_URI       "/data/2.5/forecast/?id=315202&APPID=b9bdaf75a7b1e96362a172ec83cb9303"
#define USER_AGENT        "HTTPCli (ARM; TI-RTOS)"
//#define SOCKETTEST_IP     "192.168.1.36"
#define SOCKETTEST_IP     "10.31.2.70"

#define HOSTNAME2          "api.weatherbit.io"
#define REQUEST_URI2       "/v2.0/current?city=Eskisehir,TR&key=2e614d998b32454ba6ad7e281204af25"


#define TASKSTACKSIZE 4096

extern Semaphore_Handle semaphore0;

Semaphore_Handle semaphore_clock;
Semaphore_Handle semaphore_http;

char   tempstr[20];
char   presstr[20];
char   tempstr2[20];
char   presstr2[20];
char temp_sensor[20];
char pres_sensor[20];
char   temp_openweather[20];
char   temp_weatherbit[20];
char    press_openweather[20];
char    press_weatherbit[20];

char    socketDataSensor[120];
char    socketDataOpenweather[120];
char    socketDataWeatherbit[120];

uint8_t txBuffer[4];
uint8_t rxBuffer[30];
uint8_t rxBufferLong[22];
I2C_Handle i2c;
I2C_Params i2cParams;
I2C_Transaction i2cTransaction;

/*
 *  ======== printError ========
 */



void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}

short AC1, AC2, AC3, B1, B2, MB, MC, MD; // calibration variables
unsigned short AC4, AC5, AC6; // calibration variables
long UT, UP; //uncompensated temperature and pressure
float B3, B4, B6, B7, X1t, X1p, X2t, X2p, X3p, B5t, B5p, Altitude;

void BMP180_getPressureCalibration(void)
{

    txBuffer[0] = 0xAA;
    i2cTransaction.slaveAddress = Board_BMP180_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 22;
    if (I2C_transfer(i2c, &i2cTransaction)) {

    //System_printf("Calibration data acquired\n");
    AC1 = rxBuffer[0]<<8 | rxBuffer[1];
    AC2 = rxBuffer[2]<<8 | rxBuffer[3];
    AC3 = rxBuffer[4]<<8 | rxBuffer[5];
    AC4 = rxBuffer[6]<<8 | rxBuffer[7];
    AC5 = rxBuffer[8]<<8 | rxBuffer[9];
    AC6 = rxBuffer[10]<<8 | rxBuffer[11];
    B1 = rxBuffer[12]<<8 | rxBuffer[13];
    B2 = rxBuffer[14]<<8 | rxBuffer[15];
    MB = rxBuffer[16]<<8 | rxBuffer[17];
    MC = rxBuffer[18]<<8 | rxBuffer[19];
    MD = rxBuffer[20]<<8 | rxBuffer[21];

    }
}

void BMP180_startTemperatureAcquisition(void)
{
    txBuffer[0] = 0xf4; // control register
    txBuffer[1] = 0x2e; // conversion command
    i2cTransaction.slaveAddress = Board_BMP180_ADDR; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 2; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 0; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
    //System_printf("Temperature acquisition initiated\n");
    }
}

float BMP180_getTemperature(void)
{
    float temp;
    txBuffer[0] = 0xf6; // temp register
    i2cTransaction.slaveAddress = Board_BMP180_ADDR; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 1; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 2; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
    //System_printf("Temperature value acquired\n");
    }
    UT = rxBuffer[0]<<8 | rxBuffer[1]; //UT = raw temperature data
    //System_printf("Uncompansated Temperature : %d\n", UT);
    //compute temperature
    X1t = ((UT - AC6) * AC5) >> 15;
    X2t = (MC << 11) / (X1t + MD);
    B5t = X1t + X2t;
    temp = ((B5t + 8) / 16) / 10;
    return temp;
}

void BMP180_startPressureAcquisition(void)
{
    txBuffer[0] = 0xf4; // control register
    txBuffer[1] = 0x34; // conversion command
    i2cTransaction.slaveAddress = Board_BMP180_ADDR; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 2; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 0; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
    //System_printf("Pressure acquisition initiated\n");
    }
}

float BMP180_getPressure(void)
{
    float pressure;
    txBuffer[0] = 0xf6; // temp register
    i2cTransaction.slaveAddress = Board_BMP180_ADDR; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 1; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 2; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
        //System_printf("Pressure value acquired\n");
    }
    UP = rxBuffer[0]<<8 | rxBuffer[1]; //UT = raw pressure data
    //System_printf("Uncompansated Pressure : %d\n", UP);
    B6 = B5t - 4000;
    X1p = (B2 * (B6 * B6 / 4096)) / 2048;
    X2p = AC2 * B6 / 2048;
    X3p = X1p = X2p;
    B3 = ((((long)AC1 * 4 + X3p)) + 2) / 4;
    X1p = AC3 * B6 / 8192;
    X2p = (B1 * (B6 * B6 / 4096)) / 65536;
    X3p = ((X1p + X2p) + 2) / 4;
    B4 = AC4 * (unsigned long)(X3p + 32768) / 32768;
    B7 = ((unsigned long)UP - B3) * (50000);
    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    }
    else {
        pressure = (B7 / B4) * 2;
    }
    X1p = (pressure / 256) * (pressure / 256);
    X1p = (X1p * 3038) / 65536;
    X2p = (-7357 * pressure) / 65536;
    pressure = pressure + (X1p + X2p + 3791) / 16;
    return pressure;
}

float BMP180_calculateAltitude(float pressure)
{
    float alt;
    // compute altitude; uses default sea level pressure; altitude will vary
    // depending on difference between default sea level pressure
    // (101325 Pascal) and the actual pressure
    //
    alt = 44330.0f * (1.0f - powf(pressure / 101325.0f, 1 / 5.255f));
    //System_printf("Altitude calculated\n");
    return alt;
}

short AC1, AC2, AC3, B1, B2, MB, MC, MD; // calibration variables
unsigned short AC4, AC5, AC6; // calibration variables
void getPressureCalibration(void)
{
    txBuffer[0] = 0xAA;
    i2cTransaction.slaveAddress = Board_BMP180_ADDR; // 0x77
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 22;
    I2C_transfer(i2c, &i2cTransaction);
    AC1 = rxBufferLong[0]<<8 | rxBufferLong[1];
    AC2 = rxBufferLong[2]<<8 | rxBufferLong[3];
    AC3 = rxBufferLong[4]<<8 | rxBufferLong[5];
    AC4 = rxBufferLong[6]<<8 | rxBufferLong[7];
    AC5 = rxBufferLong[8]<<8 | rxBufferLong[9];
    AC6 = rxBufferLong[10]<<8 | rxBufferLong[11];
    B1 = rxBufferLong[12]<<8 | rxBufferLong[13];
    B2 = rxBufferLong[14]<<8 | rxBufferLong[15];
    MB = rxBufferLong[16]<<8 | rxBufferLong[17];
    MC = rxBufferLong[18]<<8 | rxBufferLong[19];
    MD = rxBufferLong[20]<<8 | rxBufferLong[21];
}

void initializeI2C()
{
// Create I2C interface for sensor usage
//
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz; // It can be I2C_400kHz orI2C_100kHz
    // Let's open the I2C interface
    //
    i2c = I2C_open(Board_I2C0, &i2cParams); // actually I2C7
    if (i2c == NULL) {
        // error initializing IIC
        //
        System_abort("Error Initializing I2C\n");
    }
    //System_printf("I2C Initialized!\n");
}
void closeI2C(void)
{
    // close the interface
    //
    I2C_close(i2c);
    //System_printf("I2C interface closed\n");
}

void sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        BIOS_exit(-1);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  /* clear serverAddr structure */
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     /* convert port # to network order */
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    int connStat = connect(sockfd, (struct sockaddr *)&serverAddr, /* connecting….*/
                  sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("Error while connecting to server\n");
        if (sockfd > 0)
            close(sockfd);
        BIOS_exit(-1);
    }

    int numSend = send(sockfd, data, size, 0);       /* send data to the server*/
    if(numSend < 0) {
        System_printf("Error while sending data to server\n");
        if (sockfd > 0) close(sockfd);
        BIOS_exit(-1);
    }

    if (sockfd > 0) close(sockfd);
}

Void ClockFunc()
{
    Semaphore_post(semaphore_clock);
}


Void sensorTask()
{
    while(1){
            Semaphore_pend(semaphore_clock, BIOS_WAIT_FOREVER);
            unsigned int i;
            uint16_t temperature;
            initializeI2C();

            /* Point to the T ambient register and read its 2 bytes */
            txBuffer[0] = 0x01; // register=0x01 (temperature)
            i2cTransaction.slaveAddress = Board_TMP006_ADDR; // 0x41
            i2cTransaction.writeBuf = txBuffer;
            i2cTransaction.writeCount = 1;
            i2cTransaction.readBuf = rxBuffer;
            i2cTransaction.readCount = 2;
            if (I2C_transfer(i2c, &i2cTransaction)) {
            temperature = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);
            if (rxBuffer[0] & 0x80) { // if the nunber is negative
            temperature |= 0xF000;
            }
            temperature /= 32;


            snprintf(temp_sensor, sizeof(temp_sensor), "%d", temperature);
            //System_printf("char temp: %s\n", temp_sensor);

            //System_printf("Temperature from tmp: %d (C)\n", temperature);
            }
            else {
            System_printf("I2C Bus fault\n");
            }
            System_flush();
            //I2C_close(i2c);

            float temp, press, alt;
            //initializeI2C();
            BMP180_getPressureCalibration();
            BMP180_startTemperatureAcquisition();
            System_flush();
            Task_sleep(5);
            temp = BMP180_getTemperature();
            BMP180_startPressureAcquisition();
            System_flush();
            Task_sleep(5);
            press = BMP180_getPressure();
            alt = BMP180_calculateAltitude(press);
            //System_flush();
            closeI2C();

            //System_flush();
            press = (int)press / 100;
            //System_printf("Pressure from bmp: %d\n",(int)press);
            snprintf(pres_sensor, sizeof(pres_sensor), "%d", (int)press);

            Event_post(event0, Event_Id_00); // post Event_Id_00
        }
}

Void socketTask(UArg arg0, UArg arg1)
{
    while(1) {
        // wait for the semaphore that httpTask() will signal
        // when temperature string is retrieved from api.openweathermap.org site
        //
        Event_pend(event0,Event_Id_00 + Event_Id_01 , Event_Id_NONE,  BIOS_WAIT_FOREVER);

        GPIO_write(Board_LED0, 1); // turn on the LED

        // connect to SocketTest program on the system with given IP/port
        // send hello message whihc has a length of 5.
        //
/*
        float t = strtof(temp_openweather);
        t = t - 273;
        snprintf(temp_openweather, sizeof(temp_openweather), "%d", (int)t);

*/
        //System_printf("Sending data to Socket!\n");

        //strcpy(socketDataSensor, " Sensor Temperature: ");
        strcpy(socketDataSensor, temp_sensor);
        strcat(socketDataSensor, "C, ");
        strcat(socketDataSensor, pres_sensor);
        strcat(socketDataSensor, "hPa     --     ");
        //System_printf("Sensor: '%s'\n", socketDataSensor);

        //strcpy(socketDataOpenweather, " Open weather Temperature: ");
        strcpy(socketDataOpenweather, temp_openweather);
        strcat(socketDataOpenweather, "K, ");
        strcat(socketDataOpenweather, press_openweather);
        strcat(socketDataOpenweather, "hPa     --     ");

        //strcpy(socketDataWeatherbit, " Weatherbit Temperature: ");
        strcpy(socketDataWeatherbit, temp_weatherbit);
        strcat(socketDataWeatherbit, "C, ");
        strcat(socketDataWeatherbit, press_weatherbit);
        strcat(socketDataWeatherbit, "hPa ");



        strcat(socketDataSensor, socketDataOpenweather);
        strcat(socketDataSensor, socketDataWeatherbit);



        sendData2Server(SOCKETTEST_IP, 5011, socketDataSensor, strlen(socketDataSensor));
        //sendData2Server(SOCKETTEST_IP, 5011, socketDataSensor, strlen(socketDataSensor));
        //sendData2Server(SOCKETTEST_IP, 5011, socketDataOpenweather, strlen(socketDataOpenweather));
        //sendData2Server(SOCKETTEST_IP, 5011, socketDataWeatherbit, strlen(socketDataWeatherbit));
        //sendData2Server(SOCKETTEST_IP, 5011, temp_sensor, strlen(temp_sensor));

        GPIO_write(Board_LED0, 0);  // turn off the LED

        // wait for 5 seconds (5000 ms)
        //
        Task_sleep(1000);
    }
}

/*
 *  ======== httpTask ========
 *  Makes a HTTP GET request
 */
Void httpTask(UArg arg0, UArg arg1)
{
    bool moreFlag = false;
    char data[64], *s1, *s2, *s3, *s4;
    int ret, temp_received=0, len;
    struct sockaddr_in addr;
    int press_received = 0;

    HTTPCli_Struct cli;
    HTTPCli_Struct cli2;

    HTTPCli_Field fields[3] = {
        { HTTPStd_FIELD_NAME_HOST, HOSTNAME },
        { HTTPStd_FIELD_NAME_USER_AGENT, USER_AGENT },
        { NULL, NULL }
    };

    HTTPCli_Field fields2[3] = {
            { HTTPStd_FIELD_NAME_HOST, HOSTNAME2 },
            { HTTPStd_FIELD_NAME_USER_AGENT, USER_AGENT },
            { NULL, NULL }
        };

    while(1) {

        Semaphore_pend(semaphore_clock, BIOS_WAIT_FOREVER);
        System_printf("Sending a HTTP GET request to '%s'\n", HOSTNAME);
        System_flush();

        temp_received = 0;
        press_received = 0;
        moreFlag = false;

        HTTPCli_construct(&cli);

        HTTPCli_setRequestFields(&cli, fields);

        ret = HTTPCli_initSockAddr((struct sockaddr *)&addr, HOSTNAME, 0);
        if (ret < 0) {
            printError("httpTask: address resolution failed", ret);
        }

        ret = HTTPCli_connect(&cli, (struct sockaddr *)&addr, 0, NULL);
        if (ret < 0) {
            printError("httpTask: connect failed", ret);
        }

        ret = HTTPCli_sendRequest(&cli, HTTPStd_GET, REQUEST_URI, false);
        if (ret < 0) {
            printError("httpTask: send failed", ret);
        }

        ret = HTTPCli_getResponseStatus(&cli);
        if (ret != HTTPStd_OK) {
            printError("httpTask: cannot get status", ret);
        }

        //System_printf("HTTP Response Status Code: %d\n", ret);

        ret = HTTPCli_getResponseField(&cli, data, sizeof(data), &moreFlag);
        if (ret != HTTPCli_FIELD_ID_END) {
            printError("httpTask: response field processing failed", ret);
        }

        len = 0;
        do {    //openweather
            ret = HTTPCli_readResponseBody(&cli, data, sizeof(data), &moreFlag);
            //System_printf("All data : %s\n", data);
            if (ret < 0) {
                printError("httpTask: response body processing failed", ret);
            }
            else {
                // string is read correctly
                // find "temp:" string
                //
                s1=strstr(data, "temp");
                if(s1) {
                    if(temp_received) continue;     // temperature is retrieved before, continue
                    // is s1 is not null i.e. "temp" string is found
                    // search for comma
                    s2=strstr(s1, ",");
                    if(s2) {
                        *s2=0;                      // put end of string
                        strcpy(tempstr, s1+6);      // copy the string
                        temp_received = 1;
                    }
                }

                s3=strstr(data, "grnd_level");
                if(s3) {
                    if(press_received) continue;     // temperature is retrieved before, continue
                    // is s1 is not null i.e. "temp" string is found
                    // search for comma
                    s4=strstr(s3, ",");
                    if(s4) {
                        *s4=0;                      // put end of string
                        strcpy(presstr, s3+12);      // copy the string
                        press_received = 1;
                    }
                }


            }

            len += ret;     // update the total string length received so far
        } while (moreFlag);
        strcpy(temp_openweather, tempstr);
        strcpy(press_openweather, presstr);
        moreFlag = false;
        temp_received = 0;
        press_received = 0;

        //System_printf("Temperature from Openweather: %s\n", tempstr);
        //System_printf("Pressure from Openweather: %s\n", presstr);
        //strcpy(press_openweather, presstr);

        //System_printf("press : %s\n", presstr);

        HTTPCli_disconnect(&cli);
        //Task_sleep(1000);
        System_printf("Sending a HTTP GET request to '%s'\n", HOSTNAME2);
        System_flush();

        temp_received = 0;
        moreFlag = false;
        HTTPCli_construct(&cli2);

                HTTPCli_setRequestFields(&cli2, fields2);

                ret = HTTPCli_initSockAddr((struct sockaddr *)&addr, HOSTNAME2, 0);
                if (ret < 0) {
                    printError("httpTask: address resolution failed", ret);
                }

                ret = HTTPCli_connect(&cli2, (struct sockaddr *)&addr, 0, NULL);
                if (ret < 0) {
                    printError("httpTask: connect failed", ret);
                }

                ret = HTTPCli_sendRequest(&cli2, HTTPStd_GET, REQUEST_URI2, false);
                if (ret < 0) {
                    printError("httpTask: send failed", ret);
                }

                ret = HTTPCli_getResponseStatus(&cli2);
                if (ret != HTTPStd_OK) {
                    printError("httpTask: cannot get status", ret);
                }

                //System_printf("HTTP Response Status Code: %d\n", ret);

                ret = HTTPCli_getResponseField(&cli2, data, sizeof(data), &moreFlag);
                if (ret != HTTPCli_FIELD_ID_END) {
                    printError("httpTask: response field processing failed", ret);
                }

                len = 0;
                do {    //weatherbit
                    ret = HTTPCli_readResponseBody(&cli2, data, sizeof(data), &moreFlag);
                    //System_printf("All data : %s\n", data);
                    if (ret < 0) {
                        printError("httpTask: response body processing failed", ret);
                    }
                    else {
                        // string is read correctly
                        // find "temp:" string
                        //
                        s1=strstr(data, "app_temp");
                        if(s1) {
                            if(temp_received) continue;     // temperature is retrieved before, continue
                            // is s1 is not null i.e. "temp" string is found
                            // search for comma
                            s2=strstr(s1, "}");
                            if(s2) {
                                *s2=0;                      // put end of string
                                strcpy(tempstr, s1+10);      // copy the string
                                temp_received = 1;
                            }
                        }

                        s3=strstr(data, "pres");
                        if(s3) {
                            if(press_received) continue;     // temperature is retrieved before, continue
                            // is s1 is not null i.e. "temp" string is found
                            // search for comma
                            s4=strstr(s3, ",");
                            if(s4) {
                                *s4=0;                      // put end of string
                                strcpy(presstr, s3 + 6);      // copy the string
                                press_received = 1;
                            }
                        }

                    }

                    len += ret;     // update the total string length received so far
                } while (moreFlag);
        strcpy(temp_weatherbit, tempstr);
        strcpy(press_weatherbit, presstr);

        //System_printf("Temperature from Openweather: %s\n", tempstr);
        //System_printf("Pressure from Openweather: %s\n", presstr);

        //System_printf("Temperature from weatherbit : %s\n", tempstr);
        //System_printf("Temperature from weatherbit : %s\n", presstr);
        //System_printf("Recieved %d bytes of payload\n", len);

        System_flush();                                         // write logs to console

        HTTPCli_disconnect(&cli2);                               // disconnect from openweathermap

        //Semaphore_post(semaphore0);                             // activate socketTask

        Event_post(event0, Event_Id_01); // post Event_Id_00

        Task_sleep(1000);

    }
    HTTPCli_destruct(&cli);

}


/*
 *  ======== netIPAddrHook ========
 *  This function is called when IP Addr is added/deleted
 */
void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
       static Task_Handle taskHandle1, taskHandle2, taskHandle3;
       Task_Params taskParams;
       Error_Block eb;

       // Create a HTTP task when the IP address is added
       if (fAdd && !taskHandle1 && !taskHandle2 && !taskHandle3) {
          Error_init(&eb);

       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle1 = Task_create((Task_FuncPtr)httpTask, &taskParams, &eb);
/*
       Task_Params_init(&taskParams);
          taskParams.stackSize = TASKSTACKSIZE;
          taskParams.priority = 1;
          taskHandle3 = Task_create((Task_FuncPtr)httpTask2, &taskParams, &eb);
*/
       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle2 = Task_create((Task_FuncPtr)socketTask, &taskParams, &eb);

       if (taskHandle1 == NULL || taskHandle2 == NULL) {
           printError("netIPAddrHook: Failed to create HTTP and Socket Tasks\n", -1);
       }
   }
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    semaphore_clock=Semaphore_create(0,NULL,NULL);
    semaphore_http=Semaphore_create(0,NULL,NULL);

    System_printf("Starting the HTTP GET example\nSystem provider is set to "
            "SysMin. Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();


    /* Start BIOS */
    BIOS_start();

    return (0);
}
