/*
 * main.c - UDP socket sample application
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/*
 * Application Name     -   UDP socket
 * Application Overview -   This is a sample application demonstrating how to
 *                          open and use a standard UDP socket with CC3100.
 * Application Details  -   http://processors.wiki.ti.com/index.php/CC31xx_UDP_Socket_Application
 *                          doc\examples\udp_socket.pdf
 */


//use stty -F /dev/ttyACM0 115200 -cstopb and cat /dev/ttyACM0 to view cli_uart s
#include <stdint.h>
#include "simplelink.h"
#include "sl_common.h"
#include "driverlib/adc.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include <inc/hw_gpio.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include <stdio.h>
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"


#include <stdbool.h>
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

#define APPLICATION_VERSION "1.2.0"

#define SL_STOP_TIMEOUT        0xFF

/* IP addressed of server side socket.
 * Should be in long format, E.g: 0xc0a8010a == 192.168.1.10
 * 0x0a6B0278 == 10.107.2.120
 * 0x0A6B4FAF == 10.107.79.175  -- this is my laps ip in wel
 * #define IP_HR "10.107.95.115"
 * #define IP_HR "10.107.79.167"
 * #define IP_ADDR         0x0A6B4FA7     //laptops ip 
 * #define IP_HR "10.107.13.186"
 * #define IP_ADDR         0x0A6B0DBA     //laptops ip .
 * #define IP_HR "10.42.0.1"
 * #define IP_ADDR         0x0A2A0001     //laptops ip .

 * #define IP_HR "10.1.96.112"
 * #define IP_ADDR         0x0A016070     //laptops ip .
 */
#define IP_HR "192.168.43.183"		//using mobile hotspot
#define IP_ADDR         0xC0A82BB7     //laptops ip .
#define PORT_NUM        50001            /* Port number to be used */

#define BUF_SIZE        1400
#define NO_OF_PACKETS   2

#define NO_OF_SAMPLES 4096
#define PACKET_SIZE 1024
#define CTRL_WIDTH 12

#define SAMPLING_FREQ 512000 //50 khz

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap w/ host-driver's error codes */
    BSD_UDP_CLIENT_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

/*
 * GLOBAL VARIABLES -- Start
 */
_u8 g_Status = 0;

union
{
    _u8 BsdBuf[BUF_SIZE];
    _u32 demobuf[BUF_SIZE/4];
} uBuf;


/*---------GLOBAL variables for ADC and triggering logic---------------------*/
_u8  adcBuf[PACKET_SIZE];
_u32 ADCoutput[NO_OF_SAMPLES];
_u16 in_buf[NO_OF_SAMPLES];
_u8 out_buf[PACKET_SIZE];
_u8 recv_buf[CTRL_WIDTH];
int diff_buf[NO_OF_SAMPLES];
_u16 send_buf[NO_OF_SAMPLES];
_u16 index_in =0,index_out = 0,sample_number = 0;
int count_unsend = 0,count_untriggered = 0;
_u16 level_trig = 530; /*---static level trigger ----*/
_u16  trig_detected = 0,trig_index=0;
char tstr[100];

uint32_t SAMPLING_N = 1;
/*____________implementing circular buffer using index_in_______________*/


uint32_t ADCvalue;
uint32_t size_ram = 0  ;
/*
 * GLOBAL VARIABLES -- End
 */

/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 configureSimpleLinkToDefaultState();
static _i32 establishConnectionWithAP();
static _i32 initializeAppVariables();
static _i32 BsdUdpServer(_u16 Port);
static _i32 BsdUdpClient(_u16 Port);
static _i32 ADC_Push(_u16 Port);
static void displayBanner();
static void ADCInit(void);
void ADC_ReInit(void);
/*
 * STATIC FUNCTION DEFINITIONS -- End
 */

/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */
/*!
    \brief This function handles WLAN events
    \param[in]      pWlanEvent is the event passed to the handler
    \return         None
    \note
    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");
        return;
    }
    
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is
             * SL_USER_INITIATED_DISCONNECTION */
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication
    \param[in]      pNetAppEvent is the event passed to the handler
    \return         None
    \note
    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");
        return;
    }
 
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connection (like IP, gateway address etc)
             * will be available in 'SlIpV4AcquiredAsync_t'
             * Applications can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events
    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information
    \return         None
    \note
    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application */
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication
    \param[in]      pDevEvent is the event passed to the handler
    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication
    \param[in]      pSock is the event passed to the handler
    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");
        return;
    }

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            /*
             * TX Failed
             *
             * Information about the socket descriptor and status will be
             * available in 'SlSockEventData_t' - Applications can use it if
             * required
             *
             * SlSockEventData_u *pEventData = NULL;
             * pEventData = & pSock->socketAsyncEvent;
             */
            switch( pSock->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                    break;
                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                    break;
            }
            break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
            break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */

/*
 * Application's entry point
 */
int main(int argc, char** argv)
{
    _i32 retVal = -1;

    int i;


    // configuration of A port to get control of gain
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x0f);
    //end of assignment

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    /* Stop WDT and initialize the system-clock of the MCU
       These functions needs to be implemented in PAL */
    stopWDT();
    initClk();

    /* Configure command line interface */

    CLI_Configure();

    displayBanner();
    


    /*
     * Following function configures the device to default state by cleaning
     * the persistent settings stored in NVMEM (viz. connection profiles &
     * policies, power policy etc)
     *
     * Applications may choose to skip this step if the developer is sure
     * that the device is in its default state at start of application
     *
     * Note that all profiles and persistent settings that were done on the
     * device will be lost
     */
    retVal = configureSimpleLinkToDefaultState();
    if(retVal < 0)
    {
        CLI_Write(" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    CLI_Write(" Device is configured in default state \n\r");

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    /* Initializing the CC3100 device */
    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");

    /* Connecting to WLAN AP - Set with static parameters defined at the top
       After this call we will be connected and have IP address */
    retVal = establishConnectionWithAP();
    if(retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

//    CLI_Write(" Started sending data to UDP server \n\r");

//   retVal = BsdUdpClient(PORT_NUM);
//    if(retVal < 0)
//        CLI_Write(" Failed to send data to UDP sevrer\n\r");
//    else
//        CLI_Write(" successfully sent data to UDP server \n\r");

   ADCInit();
   while(count_unsend<NO_OF_SAMPLES);
   CLI_Write("Collected samples\n");
   //CLI_Write("pushing _ADC Data \n");
   //retVal = ADC_Push(PORT_NUM);
  //  if(retVal < 0)
  //      CLI_Write(" Failed to push ADC data to UDP sevrer\n\r");
  //  else
  //      CLI_Write("pushing ADC data succesful \n\r");
  //    CLI_Write(" Waiting for data from UDP client \n\r");
  //
  //  retVal = BsdUdpServer(PORT_NUM);
  //  if(retVal < 0)
  //      CLI_Write(" Failed to read data from the UDP client \n\r");
  //  else
  //      CLI_Write(" Successfully received data from UDP client \n\r");

    while(1){
    //SysCtlSRAMSizeGet(void);
	while(count_unsend < NO_OF_SAMPLES);
	if(trig_detected){
	     index_out = 0;
	     i = trig_index;
	     count_unsend = 512 * SAMPLING_N; /*_____for throwing away N samples___*/
	     while(count_unsend > 0){ 
	     	out_buf[index_out] = in_buf[i] & 0x0000FF;
		index_out ++;
		out_buf[index_out] = (in_buf[i] >> 8) & 0x0003;
	     	i = (i + SAMPLING_N ) & 0x00000000FFF;
	     	count_unsend = count_unsend - SAMPLING_N;
		index_out ++;
		//sprintf(tstr,"trig index %d \t\t count_unsend %d\t\t index_out %d \t\t index_in %d \n\n",trig_index,count_unsend,index_out,i);
		//CLI_Write(tstr);
	     }

	     trig_detected = 0;
	     CLI_Write("Pushing ADC_Data \n");
	     retVal = ADC_Push(PORT_NUM);
	     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){
	     	}
	    // size_ram ++;
	     //if (size_ram > 100){void SysCtlReset(void);size_ram = 0;}
	     ADC_ReInit();
	     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){
		}
	     }
    }

    /* Stop the CC3100 device */
    retVal = sl_Stop(SL_STOP_TIMEOUT);
    if(retVal < 0)
    {
        LOOP_FOREVER();
    }

    return 0;
}

/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters
    \param[in]      none
    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return SUCCESS;
}

/*!
    \brief Connecting to a WLAN Access point
    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address
    \param[in]  None
    \return     0 on success, negative error-code on error
    \note
    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = pal_Strlen(PASSKEY);
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief Opening a UDP client side socket and sending data
    This function opens a UDP socket and tries to send data to a UDP server
    IP_ADDR waiting on port PORT_NUM.
    Then the function will send 1000 UDP packets to the server.
    \param[in]      port number on which the server will be listening on
    \return         0 on success, -1 on Error.
    \note
    \warning
*/
static _i32 BsdUdpClient(_u16 Port)
{
    SlSockAddrIn_t  Addr;
    _u16            idx = 0;
    _u16            AddrSize = 0;
    _i16            SockID = 0;
    _i16            Status = 0;
    _u32            LoopCount = 0;

    for (idx=0 ; idx<BUF_SIZE ; idx++)
    {
        uBuf.BsdBuf[idx] = (_u8)(idx % 10);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons((_u16)Port);
    Addr.sin_addr.s_addr = sl_Htonl((_u32)IP_ADDR);

    AddrSize = sizeof(SlSockAddrIn_t);

    SockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( SockID < 0 )
    {
        ASSERT_ON_ERROR(SockID);
    }

    while (LoopCount < NO_OF_PACKETS)
    {
//        uBuf.BsdBuf[0] = LoopCount >> 24 & 0xFF;
//        uBuf.BsdBuf[1] = LoopCount >> 16 & 0xFF;
//        uBuf.BsdBuf[2] = LoopCount >> 8 & 0xFF;
//        uBuf.BsdBuf[3] = LoopCount  & 0xFF;

        uBuf.BsdBuf[0] ='E';
               uBuf.BsdBuf[1] = 'E';
            uBuf.BsdBuf[2] ='7';




        Status = sl_SendTo(SockID, uBuf.BsdBuf, BUF_SIZE, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);
        if( Status <= 0 )
        {
            Status = sl_Close(SockID);
            ASSERT_ON_ERROR(BSD_UDP_CLIENT_FAILED);
        }

        LoopCount++;
    }

    Status = sl_Close(SockID);
    ASSERT_ON_ERROR(Status);

    return SUCCESS;
}


static _i32 ADC_Push(_u16 Port)
{
    //CLI_Write("entered ADCPUSH\n");
    SlSockAddrIn_t  Addr;
    _u16            AddrSize = 0;
    _i16            SockID = 0;
    _i16            Status = 0,recvSize;
    _u32            LoopCount = 0;

_u8 tststr[2048];
    // CLI_Write("from ADC PUSH\n \r ");
    //memcpy(tststr, "this is my kingdom come\n",23);

    /*______________Disable adc sampling while sending data________________*/
    		//	ADCIntDisable(ADC0_BASE,3);
    		//	IntDisable(INT_TIMER1A);
    		//	IntDisable(INT_ADC0SS3);
    		//	TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
    /*______________________________________________________________________*/
    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons((_u16)Port);
    Addr.sin_addr.s_addr = sl_Htonl((_u32)IP_ADDR);

    AddrSize = sizeof(SlSockAddrIn_t);

    SockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( SockID < 0 )
    {
        ASSERT_ON_ERROR(SockID);
    }
    while (LoopCount < NO_OF_PACKETS)
    {
       /*Status = sl_SendTo(SockID, tststr, PACKET_SIZE, 0,
                               (SlSockAddr_t *)&Addr, AddrSize);*/
        Status = sl_SendTo(SockID, out_buf, PACKET_SIZE, 0,
                                       (SlSockAddr_t *)&Addr, AddrSize);
        LoopCount++;
    }
	
	   // Status = sl_RecvFrom(SockID, &uBuf.BsdBuf[BUF_SIZE - CTRL_WIDTH] , CTRL_WIDTH, 0,
		//		(SlSockAddr_t *)&Addr, (SlSocklen_t*)&AddrSize );
	    Status = sl_RecvFrom(SockID, recv_buf, CTRL_WIDTH, 0,
			    (SlSockAddr_t *)&Addr, (SlSocklen_t*)&AddrSize );
	    if(Status < 0)
	    {
		sl_Close(SockID);
		ASSERT_ON_ERROR(Status);
	    }
    
	    recvSize -= Status;
   level_trig = recv_buf[2] << 3;

   // voltage/div changes
   if (recv_buf[0]== 1) {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x0f);}
   else if (recv_buf[0] == 2) {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x8f);}
   else if (recv_buf[0] == 3) {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x4f);}
   else if (recv_buf [0]== 4) {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x2f);}
   else if (recv_buf[0] == 5) {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x1f);}
   else {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x0f);}
   //ends here volt/div

   // time/div changes
   if (recv_buf[1]== 1)       {SAMPLING_N = 8;}
   else  if (recv_buf[1]== 2) {SAMPLING_N = 6;}
   else  if (recv_buf[1]== 3) {SAMPLING_N = 4;}
   else  if (recv_buf[1]== 4) {SAMPLING_N = 2;}
   else  if (recv_buf[1]== 5) {SAMPLING_N = 1;}
   else			      {SAMPLING_N = 1;}
   // ends here time/div
/*__________re enabling interupt for ADC sampling ______________*/
		//ADCInit();
/*______________________________________________________________*/

    Status = sl_Close(SockID);
    ASSERT_ON_ERROR(Status);

    return SUCCESS;
}

/*!
    \brief Opening a UDP server side socket and receiving data
    This function opens a UDP socket in Listen mode and waits for incoming
    UDP packets from the connected client.
    \param[in]      port number on which the server will be listening on
    \return         0 on success, Negative value on Error.
    \note
    \warning
*/
static _i32 BsdUdpServer(_u16 Port)
{
    SlSockAddrIn_t  Addr;
    SlSockAddrIn_t  LocalAddr;
    _u16          idx = 0;
    _u16          AddrSize = 0;
    _i16          SockID = 0;
    _i16          Status = 0;
    _u16          LoopCount = 0;
    _u16          recvSize = 0;

    for (idx=0 ; idx<BUF_SIZE ; idx++)
    {
        uBuf.BsdBuf[idx] = (_u8)(idx % 10);
    }

    LocalAddr.sin_family = SL_AF_INET;
    LocalAddr.sin_port = sl_Htons((_u16)Port);
    LocalAddr.sin_addr.s_addr = 0;

    AddrSize = sizeof(SlSockAddrIn_t);

    SockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( SockID < 0 )
    {
        ASSERT_ON_ERROR(SockID);
    }

    Status = sl_Bind(SockID, (SlSockAddr_t *)&LocalAddr, AddrSize);
    if( Status < 0 )
    {
        Status = sl_Close(SockID);
        ASSERT_ON_ERROR(Status);
    }

    Status = sl_RecvFrom(SockID, &uBuf.BsdBuf[BUF_SIZE - recvSize], recvSize, 0,
                                (SlSockAddr_t *)&Addr, (SlSocklen_t*)&AddrSize );
    Status = sl_Close(SockID);

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables
    \param[in]  None
    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(uBuf.BsdBuf, 0, sizeof(uBuf));

    return SUCCESS;
}

/*!
    \brief This function displays the application's banner
    \param      None
    \return     None
*/
static void displayBanner()
{
    CLI_Write("\n\r\n\r");

    CLI_Write("Wifi DSO debug interface -- server ");
    CLI_Write(IP_HR);
    CLI_Write("\n\r*******************************************************************************\n\r");
}
void ADCInit(void){
	//CLI_Write("from ADC INIT");

    //SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    // *** Peripheral Enable
   SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
   {
   }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // *** ADC0
    ADCHardwareOversampleConfigure(ADC0_BASE,4);
    ADCReferenceSet(ADC0_BASE,ADC_REF_INT);
    ADCSequenceConfigure(ADC0_BASE,3,ADC_TRIGGER_TIMER,0);
    ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH4|ADC_CTL_IE|ADC_CTL_END); // changed to take value from temp sensor
    ADCSequenceEnable(ADC0_BASE,3);

    // *** Timer0
  TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC);

   TimerLoadSet(TIMER1_BASE,TIMER_A,SysCtlClockGet()/SAMPLING_FREQ);
   TimerControlTrigger(TIMER1_BASE,TIMER_A,true);

    // *** GPIO
    GPIOPinTypeADC(GPIO_PORTD_BASE,GPIO_PIN_0);
    //CLI_Write("before interrupts");

    // *** Interrupts
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
    IntEnable(INT_ADC0SS3);
    ADCIntEnable(ADC0_BASE,3);
    TimerEnable(TIMER1_BASE,TIMER_A);
    CLI_Write("IDK");
    IntMasterEnable();
}
void ADC_ReInit(void){



    IntEnable(INT_TIMER1A);
    ADCIntEnable(ADC0_BASE,3);
    IntEnable(INT_ADC0SS3);
    TimerEnable(TIMER1_BASE,TIMER_A);
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
    CLI_Write("NIK:");
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    IntMasterEnable();
    return;
}
void Timer1IntHandler(void){
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    //CLI_Write("from TIMer");
}

 void ADC0SS3IntHandler(void){
   static int j=0;
   ADCIntClear(ADC0_BASE,3);
   ADCSequenceDataGet(ADC0_BASE,3,&ADCvalue);
   //CLI_Write("from ADCIN");
   //adcBuf[sample_number] = (ADCvalue >> 2) & 0x00FF;
   //sample_number++;
   //adcBuf[sample_number] = (ADCvalue >> 10) & 0x0003;
   ADCvalue = ADCvalue >> 2;  
   in_buf[index_in] = ADCvalue & 0x000000000000003FF;
   /*______first order difference to implement edge trigger______*/
   diff_buf[index_in] = in_buf[index_in] - in_buf[((index_in -4) & 0x000000000FFF)];
   
  // if((in_buf[index_in] >= (level_trig - 1)) && (in_buf[index_in] <= level_trig + 1) && (trig_detected == 0) && (diff_buf[index_in] > 0) ){
  if((trig_detected == 0)&& (in_buf[index_in - 1] < level_trig) && (in_buf[index_in] > level_trig) && (diff_buf[index_in] > 0) ){
	
   	trig_detected = 1;
	count_untriggered = 0;
   	trig_index = index_in;
	count_unsend ++;
   }else if(trig_detected){
   	count_unsend ++;
   }
   else{
   	count_untriggered ++;
	if (count_untriggered > 4096){
		count_unsend = NO_OF_SAMPLES + 1;
		trig_detected = 1; // modification to check the arbritary sending of data when trigger not detected
		count_untriggered = 0;
	}
   }

   
   /*_________implementing a circular buffer of size 512________*/
    index_in = (index_in +1 ) & 0x000FFF;

   /*__________counter for unsend data__________________________*/

   /*_________static supplied level trig________________________*/

   if (count_unsend >= NO_OF_SAMPLES){
       ADCIntDisable(ADC0_BASE,3);
       IntDisable(INT_TIMER1A);
       IntDisable(INT_ADC0SS3);
       TimerIntDisable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
       IntMasterDisable();
       }
  // if(i<1024c)
   //{temp_val[i] = ADC0Value; }
   // Get Data from ADC and store it in ADC0Value
   //CLI_Write("from ADC");
   return;
}
