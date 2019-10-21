/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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

/* Standard includes                                                          */
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* Hardware includes                                                          */
#include <ti/devices/cc32xx/inc/hw_types.h>
#include <ti/devices/cc32xx/inc/hw_ints.h>
#include <ti/devices/cc32xx/inc/hw_memmap.h>

/* Driverlib includes                                                         */
#include <ti/devices/cc32xx/driverlib/rom.h>
#include <ti/devices/cc32xx/driverlib/rom_map.h>
#include <ti/devices/cc32xx/driverlib/timer.h>

/* TI-Driver includes                                                         */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/gpio/GPIOCC32XX.h>
#include <ti/drivers/PWM.h>

/* Simplelink includes                                                        */
#include <ti/drivers/net/wifi/simplelink.h>

/* Common interface includes                                                  */
#include "network_if.h"
#include "uart_term.h"

/* Application includes                                                       */
#include "Board.h"
#include "FreeRTOS.h"

#include "pthread.h"
#include "time.h"
#include "unistd.h"


extern int sprintf(char *str, const char *format, ...);
extern int vsprintf(char *str, const char *format, va_list ap);

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************)

#define APPLICATION_VERSION     "0.0.1"
#define APPLICATION_NAME        "babot"

/* Spawn task priority and Task and Thread Stack Size                         */
#define TASKSTACKSIZE           2048
#define RXTASKSIZE              4096
#define THREADSIZE              2048
#define SPAWN_TASK_PRIORITY     9

#define MESSAGE_COMMAND_BUTTON_PRESSED_1 211

typedef struct QueueMessages {
    uint8_t eventType;
    uint32_t reserved;
} QueueMessage;


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
void pushButtonInterruptHandler2(uint_least8_t index);
void pushButtonInterruptHandler3(uint_least8_t index);
void TimerPeriodicIntHandler(sigval val);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
static void DisplayBanner(char * AppName);
void *BabotClient(void *pvParameters);

void Babot_ClientStop(uint8_t disconnect);
void Babot_ServerStop();
void Babot_Stop();
void Babot_start();
int32_t Babot_IF_Connect();
int32_t BabotServer_start();
int32_t BabotClient_start();
char * generate_get_request(char *host, char *path);
char * bsprintf(const char *format, ...);
void babot_other_task_spawn();
void * babot_other_task(void *pvParameters);

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

uint32_t socketConn = 0;

uint32_t gInitState = 0;
int32_t gApConnectionState = -1;
uint32_t memPtrCounterAlloc = 0;
uint32_t memPtrCounterfree = 0;
uint32_t gWaitForExternalRestart = 0;
_SlLockObj_t reportLockObj;
unsigned short g_usTimerInts;

PWM_Handle activePwmA;
PWM_Handle activePwmB;

/* Receive task handle                                                        */
pthread_t g_rx_task_hndl = (pthread_t) NULL;
pthread_t g_server_task_hndl = (pthread_t) NULL;
uint32_t gUiConnFlag = 0;

/* AP Security Parameters                                                     */
SlWlanSecParams_t SecurityParams = { 0 };

_i16 activeSocketId  = -1;
char *pendingSendCommand = NULL;
int pendingSendCommandSize = 0;

/* Client ID,User Name and Password                                           */
char ClientId[12];

struct BridgeHandle
{
    /* Handle to the client context                                           */
    void *ClientCtxHndl;
} BridgeHndl;

/* Message Queue                                                              */
mqd_t g_PBQueue;

pthread_t babotThread = (pthread_t) NULL;
pthread_t appThread = (pthread_t) NULL;

pthread_t babotThread = (pthread_t) NULL;

timer_t g_timer;

uint8_t otherTask = 1;

//*****************************************************************************
//                 Banner VARIABLES
//*****************************************************************************
char lineBreak[]                = "\n\r";

//*****************************************************************************
//
//! Push Button Handler2(GPIOSW3). Press push button3 Whenever user wants to
//! disconnect from the remote broker. Write message into message queue
//! indicating disconnect from broker.
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void pushButtonInterruptHandler3(uint_least8_t index) {
    portYIELD();
}

void pushButtonInterruptHandler2(uint_least8_t index) {
    //there used to be code here which queued the button press, the magic itself was done in the babot thread run loop.
    portYIELD();
}

//*****************************************************************************
//
//! Periodic Timer Interrupt Handler
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void TimerPeriodicIntHandler(sigval val)
{
    unsigned long ulInts;

    /* Clear all pending interrupts from the timer we are currently using.    */
    ulInts = MAP_TimerIntStatus(TIMERA0_BASE, true);
    MAP_TimerIntClear(TIMERA0_BASE, ulInts);

    /* Increment our interrupt counter.                                       */
    g_usTimerInts++;

    if (!(g_usTimerInts & 0x1))
    {
        /* Turn Led Off                                                       */
        GPIO_write(Board_LED0, Board_LED_OFF);
    }
    else
    {
        /* Turn Led On                                                        */
        GPIO_write(Board_LED0, Board_LED_ON);
    }
}

//*****************************************************************************
//
//! Function to configure and start timer to blink the LED while device is
//! trying to connect to an AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerConfigNStart()
{
    struct itimerspec value;
    sigevent sev;

    /* Create Timer                                                           */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_notify_function = &TimerPeriodicIntHandler;
    timer_create(2, &sev, &g_timer);

    /* start timer                                                            */
    value.it_interval.tv_sec = 0;
    value.it_interval.tv_nsec = 100 * 1000000;
    value.it_value.tv_sec = 0;
    value.it_value.tv_nsec = 0;

    timer_settime(g_timer, 0, &value, NULL);
}

//*****************************************************************************
//
//! Disable the LED blinking Timer as Device is connected to AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerDeinitStop()
{ 
    /* Disable the LED blinking Timer as Device is connected to AP.           */
    timer_delete(g_timer);
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t    CC3220 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}

void * BabotClient(void *pvParameters)
{
    UART_PRINT("BabotClient thread starting,.,.\n\r");
    //this is the connection thread.
    _i16 socketId;

    while (socketConn == 0) {

        SlSockAddrIn_t  Addr;
        _i16 AddrSize = sizeof(SlSockAddrIn_t);
        _i16 connState;

        Addr.sin_family = SL_AF_INET;
        Addr.sin_port = sl_Htons(8080);
        Addr.sin_addr.s_addr = sl_Htonl(SL_IPV4_VAL(0,0,0,0));

        socketId = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_IPPROTO_TCP);

        char *request = generate_get_request("host", "/babot");
        UART_PRINT("requst dump\r\n%s\r\n------\r\n", request);

        connState = sl_Connect(socketId, (SlSockAddr_t *)&Addr, AddrSize);

        _i16 state = -1;
        if (connState < 0) {
            UART_PRINT("Failed to connect to host\n\r", connState);
        } else {
            state = sl_Send(socketId, request, strlen(request), 0);
        }

        free(request);
        if (state > 0) {
            //conn success;
            activeSocketId = socketId;
            int c = 0;
            while (activeSocketId >= 0) {
                c++;

                SlFdSet_t readFd;
                SlFdSet_t writeFd;

                SL_SOCKET_FD_SET(activeSocketId, &readFd);

                SlTimeval_t timeout;
                timeout.tv_sec = 0;
                timeout.tv_usec = 10000; // 100 microseconds;

                _i16 selectResult = sl_Select(1, &readFd, NULL, NULL, &timeout);

                if (selectResult > 0) {
                    if (SL_SOCKET_FD_ISSET(activeSocketId, &readFd)) {
                        UART_PRINT("sl_Select, data avaiable, reading\n\r", connState);
                        uint8_t *incoming = malloc(sizeof(uint8_t) * 512);
                        int rec = sl_Recv(socketId, incoming, 512, 0);

                        if (rec > 0) {
                            uint8_t *defbytes = read_websocket_frame(incoming, rec);
                            UART_PRINT("got reply \r\n%s\r\n--------parsed\r\n%s\r\n----------\n\r", incoming, defbytes);
                            uint8_t command = 0;
                            uint8_t pwmSpeed = 1;

                            int temp = 0;
                            for (temp = 0; temp < 2; temp++) {
                                char currentCommand = defbytes[command];
                                char currentPwmSpeen = defbytes[pwmSpeed];

                                if (currentCommand == 'f') {
                                    if (temp == 0) GPIO_write(CC3220SF_LAUNCHXL_GPIO_STEPPER_ONE, 1);
                                    else GPIO_write(CC3220SF_LAUNCHXL_GPIO__STEPPER_THREE, 1);
                                    if (temp == 0) GPIO_write(CC3220SF_LAUNCHXL_GPIO_STEPPER_TWO, 0);
                                    else GPIO_write(CC3220SF_LAUNCHXL_GPIO__STEPPER_FOUR, 0);
                                } else if (currentCommand == 's') {
                                    if (temp == 0) GPIO_write(CC3220SF_LAUNCHXL_GPIO_STEPPER_ONE, 0);
                                    else GPIO_write(CC3220SF_LAUNCHXL_GPIO__STEPPER_THREE, 0);
                                    if (temp == 0) GPIO_write(CC3220SF_LAUNCHXL_GPIO_STEPPER_TWO, 0);
                                    else GPIO_write(CC3220SF_LAUNCHXL_GPIO__STEPPER_FOUR, 0);
                                } else if (currentCommand == 'r') {
                                    if (temp == 0) GPIO_write(CC3220SF_LAUNCHXL_GPIO_STEPPER_ONE, 0);
                                    else GPIO_write(CC3220SF_LAUNCHXL_GPIO__STEPPER_THREE, 0);
                                    if (temp == 0) GPIO_write(CC3220SF_LAUNCHXL_GPIO_STEPPER_TWO, 1);
                                    else GPIO_write(CC3220SF_LAUNCHXL_GPIO__STEPPER_FOUR, 1);
                                }

                                int speed = 0;

                                uint8_t controlChar = defbytes[pwmSpeed];

                                if (controlChar == '1') speed = 1000;
                                else if (controlChar == '2') speed = 2000;
                                else if (controlChar == '3') speed = 3000;
                                else if (controlChar == '4') speed = 4000;
                                else if (controlChar == '5') speed = 5000;
                                else if (controlChar == '6') speed = 6000;
                                else if (controlChar == '7') speed = 7000;
                                else if (controlChar == '8') speed = 8000;
                                else if (controlChar == '9') speed = 9000;

                                PWM_setDuty(temp == 0 ? activePwmA : activePwmB, speed);

                                command+=2;
                                pwmSpeed+=2;
                            }

                            free(defbytes);
                            free(incoming);
                        } else {
                            //error occured. reset the socket
                            sl_Close(activeSocketId);
                            activeSocketId = -1;
                            state = -1;
                            UART_PRINT("no connection available, resetting\r\n");
                        }
                    } else {
                        UART_PRINT("sl_Select reports a connection is ready\n\r");
                    }
                } else {
                    UART_PRINT("sl_Select returned a negative value\n\r", connState);
                }
                UART_PRINT(".");
                usleep(100);
            }
        }
        sleep(2);
    }

    UART_PRINT("Run loop ended unexpectedly\n\r");
    sl_Close(socketId);

    return NULL;

}

char * bsprintf(const char *format, ...) {
    char *buffer = malloc(sizeof(char) * 128);
    va_list va;
    va_start(va, format);
    int sz = vsprintf(buffer, format, va);
    if (sz > 0) {
        char *trimmed = malloc((sizeof(char) * sz) + 1); //last char is terminating null
        int i;
        for (i = 0; i < sz; i++) {
            trimmed[i] = buffer[i];
        }
        trimmed[sz] = '\0';
        free(buffer);
        va_end(va);
        return trimmed;
    }
    free(buffer);
    va_end(va);
    return NULL;
}

char * generate_get_request(char *host, char *path) {
    char *request = malloc(sizeof(char) * 312);

    char *method = bsprintf("GET %s HTTP/1.1\r\n", path);

    strcpy(request, method);
    free(method);

    char * hostPath = bsprintf("Host: %s\r\n", host);
    strcat(request, hostPath);
    free(hostPath);
    strcat(request, "Upgrade: websocket\r\n");
    strcat(request, "Pragma: no-cache\r\nCache-Control: no-cache\r\n");
    strcat(request, "Connection: Upgrade\r\n");
    strcat(request, "Sec-WebSocket-Version: 13\r\n");
    strcat(request, "User-Agent: Babot/0.1 FreeRTOS\r\n");
    strcat(request, "Accept-Encoding: identity\r\n");
    strcat(request, "Accept-Language: *\r\n");
    strcat(request, "Content-Length: 0\r\n");
    strcat(request, "sec-websocket-key: ");

    char *key = malloc(sizeof(char) * 16);
    int i;
    for (i = 0; i < 16; i++) {
        char randNum = (char)(rand() % 255);
        key[i] = randNum;
    }

    UART_PRINT("Generating base64\n\r");

    char *b64key = b64_encode(key, 16);

    char *scKey = bsprintf("%s\r\n\r\n", b64key);
    strcat(request, scKey);
    free(scKey);

    return request;
}

//this is called from mainThread
int32_t Babot_IF_Connect()
{
    int32_t lRetVal;
    char SSID_Remote_Name[32];
    int8_t Str_Length;

    memset(SSID_Remote_Name, '\0', sizeof(SSID_Remote_Name));
    Str_Length = strlen(SSID_NAME);

    if (Str_Length)
    {
        /* Copy the Default SSID to the local variable                        */
        strncpy(SSID_Remote_Name, SSID_NAME, Str_Length);
    }

    /* Display Application Banner                                             */
    DisplayBanner(APPLICATION_NAME);

    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_OFF);
    GPIO_write(Board_LED2, Board_LED_OFF);

    /* Reset The state of the machine                                         */
    Network_IF_ResetMCUStateMachine();

    /* Start the driver                                                       */
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device\n\r", lRetVal);
        return -1;
    }

    /* switch on Green LED to indicate Simplelink is properly up.             */
    GPIO_write(Board_LED2, Board_LED_ON);

    /* Start Timer to blink Red LED till AP connection                        */
    LedTimerConfigNStart();

    /* Initialize AP security params                                          */
    SecurityParams.Key = (signed char *) SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /* Connect to the Access Point                                            */
    lRetVal = Network_IF_ConnectAP(SSID_Remote_Name, SecurityParams);
    if (lRetVal < 0)
    {
        UART_PRINT("Connection to an AP failed\n\r");
        return -1;
    }

    /* Disable the LED blinking Timer as Device is connected to AP.           */
    LedTimerDeinitStop();

    /* Switch ON RED LED to indicate that Device acquired an IP.              */
    GPIO_write(Board_LED0, Board_LED_ON);

    sleep(1);

    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_OFF);
    GPIO_write(Board_LED2, Board_LED_OFF);

    return 0;
}

void Babot_start()
{
    int32_t threadArg = 100;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int32_t retc = 0;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;
    retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, THREADSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);

    if (retc != 0)
    {
        UART_PRINT("babot thread create fail\n\r");
        return;
    }

    retc = pthread_create(&babotThread, &pAttrs, BabotClient, (void *) &threadArg);
    if (retc != 0)
    {
        UART_PRINT("babot thread create fail\n\r");
        return;
    }

}

void Babot_Stop()
{

}

int32_t BabotClient_start()
{
    return 0;
}

void Babot_ClientStop(uint8_t disconnect)
{

}

void printBorder(char ch, int n)
{
    int        i = 0;

    for(i=0; i<n; i++)    putch(ch);
}

int32_t DisplayAppBanner(char* appName, char* appVersion)
{
     int32_t            ret = 0;
     uint8_t            macAddress[SL_MAC_ADDR_LEN];
     uint16_t           macAddressLen = SL_MAC_ADDR_LEN;
     uint16_t           ConfigSize = 0;
     uint8_t            ConfigOpt = SL_DEVICE_GENERAL_VERSION;
     SlDeviceVersion_t ver = {0};
     uint8_t            Index;

     srand(199999);

     ConfigSize = sizeof(SlDeviceVersion_t);

     /* Print device version info. */
     ret = sl_DeviceGet(SL_DEVICE_GENERAL, &ConfigOpt, &ConfigSize, (uint8_t*)(&ver));

     /* Print device Mac address */
     ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen, &macAddress[0]);

     UART_PRINT(lineBreak);
     UART_PRINT("\t");
     printBorder('=', 44);
     UART_PRINT(lineBreak);
     UART_PRINT("\t   %s Example Ver: %s",appName, appVersion);
     UART_PRINT(lineBreak);
     UART_PRINT("\t");
     printBorder('=', 44);
     UART_PRINT(lineBreak);
     UART_PRINT(lineBreak);
     UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
     UART_PRINT(lineBreak);
     UART_PRINT("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],ver.FwVersion[2],ver.FwVersion[3]);
     UART_PRINT(lineBreak);
     UART_PRINT("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],ver.PhyVersion[2],ver.PhyVersion[3]);
     UART_PRINT(lineBreak);
     UART_PRINT("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3]);
     UART_PRINT(lineBreak);
     UART_PRINT("\t ROM:  %d",ver.RomVersion);
     UART_PRINT(lineBreak);
     UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
     UART_PRINT(lineBreak);
     UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
     UART_PRINT(lineBreak);
     UART_PRINT(lineBreak);
     UART_PRINT("\t");
     printBorder('=', 44);
     UART_PRINT(lineBreak);
     UART_PRINT(lineBreak);

     for (Index = 0; Index < 6; Index++)
     {
         ClientId[Index] = (char)macAddress[Index];
     }

     return ret;
}

void babot_other_task_spawn() {


}

void * babot_other_task(void *pvParameters) {





    UART_PRINT("other task loop starting\n\r");
    while (otherTask == 1) {
        //do nothing;

    }

    pthread_exit(0);

    return NULL;

}



void mainThread(void * args)
{

    uint32_t count = 0;
    pthread_t spawn_thread = (pthread_t) NULL;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;
    int32_t retc = 0;
    UART_Handle tUartHndl;

    Board_initSPI();

    /* Configure the UART                                                     */
    tUartHndl = InitTerm();
    /* remove uart receive from LPDS dependency                               */
    UART_control(tUartHndl, UART_CMD_RXDISABLE, NULL);

    /* Create the sl_Task                                                     */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASKSTACKSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs_spawn, PTHREAD_CREATE_DETACHED);

    retc = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if (retc != 0)
    {
        UART_PRINT("could not create simplelink task\n\r");
        while (1);
    }

    retc = sl_Start(0, 0, 0);
    if (retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Start failed\n");
        while(1);
    }

    /* Output device information to the UART terminal */
    retc = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    retc = sl_Stop(SL_STOP_TIMEOUT );
    if (retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Stop failed\n");
        while(1);
    }

    if(retc < 0)
    {
        /* Handle Error */
        UART_PRINT("Babot_client - Unable to retrieve device information \n");
        while (1);
    }

    //create a thread for the interrupts.

    unsigned mode = 0;

    GPIO_setCallback(Board_BUTTON1, pushButtonInterruptHandler3);
    GPIO_enableInt(Board_BUTTON1); // SW3

    GPIO_setCallback(Board_BUTTON0, pushButtonInterruptHandler2);
    GPIO_enableInt(Board_BUTTON0); // SW3


    PWM_init();
    PWM_Params params;
    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = 10000;
    activePwmA = PWM_open(Board_PWM0, &params);
    activePwmB = PWM_open(Board_PWM1, &params);

    if (activePwmA == NULL || activePwmB == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }
    PWM_start(activePwmA);
    PWM_start(activePwmB);
    PWM_setDuty(activePwmA, 100);
    PWM_setDuty(activePwmB, 100);

    while (1)
    {
        gWaitForExternalRestart = 0;


        /* Connect to AP                                                      */
        gApConnectionState = Babot_IF_Connect();

        Babot_start();

        /* Wait for Init to be completed!!!                                   */
        while (gInitState != 0)
        {
            UART_PRINT(".");
            sleep(1);
        }
        UART_PRINT(".\r\n");

        while (gWaitForExternalRestart == 0);

        UART_PRINT("TO Complete - Closing all threads and resources\r\n");

        /* Stop the Babot Process                                              */
        Babot_Stop();

        UART_PRINT("reopen Babot # %d  \r\n", ++count);

    }
}
