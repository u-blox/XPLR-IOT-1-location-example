/*
 * Copyright 2020 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


/** 
 * @file main.c
 * @brief Example code for XPLR-IoT-1 Kit to demonstrate CloudLocate usage with M10 receiver.
 */


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ubxlib.h"
#include "u_cfg_os_platform_specific.h"
#include "u_cfg_app_platform_specific.h"

#include "u_mqtt_common.h"
#include "u_mqtt_client.h"

#include <zephyr.h>
#include <sys/printk.h>
#include <console/console.h>

#include "module_config.h"

/* ----------------------------------------------------------------
 * ERROR CHECKING
 * -------------------------------------------------------------- */

#define VERIFY(cond, fail_msg) \
    if (!(cond)) {\
        failed(fail_msg); \
    }

void failed(const char *msg)
{
    uPortLog(msg);
    while(1);
}

/*! \enum messageType_t
 * @brief Enum containing all supported message types
 */
typedef enum {
    MEASX,
    MEAS50,
    MEAS20,
    NAVPVT
} messageType_t;


/*! \struct measCfg_t
 * @brief A struct containing messageId and KeyId for meas message configuration
 * @typedef messageId
 * @typedef keyId
 */
typedef struct {
    uint32_t messageId;
    uint32_t keyId;
} measCfg_t; 


/*! 
 * @brief A struct array containing messageIds and KeyIds for all meas messages
 * Reference: U-blox M10 Spg 5.10 Document (https://www.u-blox.com/docs/UBX-21035062)
 */
measCfg_t msgInfo[] =  {
    { 0x0214, U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_RXM_MEASX_UART1_U1},
    { 0x0286, U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_RXM_MEAS50_UART1_U1},
    { 0x0284, U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_RXM_MEAS20_UART1_U1},
    { 0x0107, U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_NAV_PVT_UART1_U1}
};

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

//Flags to Enable/Disable Messages 
#define U_GNSS_CFG_ENABLE_MSG 1

#define U_GNSS_CFG_DISABLE_MSG 0

/* ----------------------------------------------------------------
 * CONFIGURATION PARAMETERS
 * -------------------------------------------------------------- */

// ------------------------- CONFIGURATIONS START ---------------------------------

#define APN "<network-apn>"

#define BROKER_NAME "mqtt.thingstream.io"

#define CLIENT_ID "<mqtt-thing-client-id>"

#define USERNAME "<mqtt-thing-username>"

#define PASSWORD "<mqtt-thing-passowrd>"

#define PUB_TOPIC "CloudLocate/GNSS/request"

#define SUB_TOPIC "<mqtt-thing-subscribe-topic>"

const messageType_t MSG_TYPE = MEASX;
const int32_t NUM_OF_SECONDS_TO_WAIT_FOR_FIRST_MESSAGE = 10;
const int32_t COMPACT_MSG_TIMEOUT_IN_SECS = 30;
bool FALLBACK_NAVPVT_ENABLED = true;
const int32_t FALLBACK_TIMEOUT_IN_SECS = 60; 
// ------------------------- CONFIGURATIONS END ---------------------------------


/* ----------------------------------------------------------------
 * GLOBAL VARIABLES
 * -------------------------------------------------------------- */

const int32_t cellRegistrationTimeout = 50; 
int32_t cellSearchstartTimeMs;
bool isCellConnectAborted = false;



/* ------------------------------------------------------------------------------
 * STATIC FUNCTION DECLERATION
 * -----------------------------------------------------------------------------*/

/** Callback for unread message indications.
*/
static void messageIndicationCallback(int32_t, void *);

/** Callback to check if CellConnect should continue or not.
*/
static bool continueCellSearch(uDeviceHandle_t deviceHandle);



/* ------------------------------------------------------------------------------
 * STATIC FUNCTION IMPLEMENTATION
 * -----------------------------------------------------------------------------*/

/** \fn static void messageIndicationCallback(int32_t numUnread, void *pParam)
 * @brief Indication for unread message indications.
 * @param[in] numUnread count of unread characters
 * @param[in] pParam pointer to an unread incoming parameter
*/

static void messageIndicationCallback(int32_t numUnread, void *pParam)
{
    bool *pMessagesAvailable = (bool *) pParam;
    printk("The broker says there are %d message(s) unread.\n", numUnread);
    *pMessagesAvailable = true;
}
/** \fn static bool continueCellSearch(uDeviceHandle_t deviceHandle)
 * @brief Indication to stop or continue CellConnect.
 * @param[in] deviceHandle device handle
*/
static bool continueCellSearch(uDeviceHandle_t deviceHandle)
{
    bool shouldCellSearchContinue = (uPortGetTickTimeMs() - cellSearchstartTimeMs < (cellRegistrationTimeout*1000));
	if (!shouldCellSearchContinue){
        isCellConnectAborted = true;
    }
    return shouldCellSearchContinue;
}

/* ------------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -----------------------------------------------------------------------------*/


/** \fn void printUBXMessageinHex(char *pBuffer, int32_t bufferLenght)
 * @brief Function to print a string of defined length.
 * @param[in] pBuffer incoming string - char array
 * @param[in] bufferLenght length of the string to be printed
*/
void printUBXMessageinHex(char *pBuffer, int32_t bufferLenght)
{
    for(int i=0; i<bufferLenght; i++)
    {
        printk("%02x",pBuffer[i]);
    }
    printk("\n");
}

/** \fn int32_t getMeasMessageFromGNSS(char *pBuffer, int32_t bufferLength)
 * @brief Getting meas message from GNSS receiver
 * @param[in] pBuffer buffer to be filled with meas message
 * @param[in] bufferLEngth length of the buffer
*/
int32_t getMeasMessageFromGNSS(char *pBuffer, int32_t bufferLength)
{
    int32_t uartHandle;
    uGnssTransportHandle_t gnssUartHandle;
    uDeviceHandle_t gnssDeviceHandle;
    
    int count = 0;
    int32_t length = 0;
    uGnssMessageId_t messageId = {0};
    bool validMessage = false;
    int32_t startTimeMs;

    max10Enable();
    max10SafeBootDisable();
    max10BackupSupplyDisable();
    max10NoraCommEnable();

    uGnssInit();   

    uartHandle = uPortUartOpen(U_CFG_APP_GNSS_UART,
                               U_GNSS_UART_BAUD_RATE, NULL,
                               U_GNSS_UART_BUFFER_LENGTH_BYTES,
                               -1,
                               -1,
                               -1,
                               -1);

    
    gnssUartHandle.uart = uartHandle;

    if (uGnssAdd(U_GNSS_MODULE_TYPE_M10,
                 U_GNSS_TRANSPORT_UART, gnssUartHandle,
                 EN_MAX_PIN, false, &gnssDeviceHandle) == U_ERROR_COMMON_SUCCESS) 
    {
        uGnssSetUbxMessagePrint(gnssDeviceHandle, false);

        if (uGnssPwrOn(gnssDeviceHandle) == 0) {
            printk("Gnss Powered on\r\n");
            
            // For the selected message type in configuration, enabling that message on GNSS receiver 
            if(uGnssCfgValSet(gnssDeviceHandle, msgInfo[MSG_TYPE].keyId, U_GNSS_CFG_ENABLE_MSG, U_GNSS_CFG_VAL_TRANSACTION_NONE,U_GNSS_CFG_VAL_LAYER_RAM) == 0) {
                startTimeMs = uPortGetTickTimeMs();     
                printk("Enabled compact message.\r\n");


                messageId.type = U_GNSS_PROTOCOL_UBX;
                messageId.id.ubx = msgInfo[MSG_TYPE].messageId; 
                printk("Waiting for compact message. Timer values NUM_OF_SECONDS_TO_WAIT_FOR_FIRST_MESSAGE: %d, COMPACT_MSG_TIMEOUT_IN_SECS: %d\r\n", NUM_OF_SECONDS_TO_WAIT_FOR_FIRST_MESSAGE,COMPACT_MSG_TIMEOUT_IN_SECS);
                while(!validMessage && ((uPortGetTickTimeMs() - startTimeMs) < (COMPACT_MSG_TIMEOUT_IN_SECS*1000))  ) //timer is expired
                {
                    length = uGnssMsgReceive(gnssDeviceHandle, &messageId, &pBuffer, bufferLength, COMPACT_MSG_TIMEOUT_IN_SECS*1000, NULL);
                         
                    if (length > 0 && ((uPortGetTickTimeMs() - startTimeMs) >= (NUM_OF_SECONDS_TO_WAIT_FOR_FIRST_MESSAGE*1000)) ) //added check for the first message wait
                    {       
                        // MEASX is generated even if there is no satellite information so adding a length check
                        // so we know that whatever we are sending have atleast some satellite information in it
                        if(MSG_TYPE == MEASX && length > 300)
                        {
                            validMessage = true;        
                        }

                        //CloudLocate service expects compact message(MEAS50 and MEAS20) without header and checksum so stripping down the message
                        if(MSG_TYPE == MEAS20 || MSG_TYPE == MEAS50 )
                        {
                            unsigned char strippedMessage[50];
                            memcpy(strippedMessage,pBuffer+6,length-8);
                            memset(pBuffer,0,length);
                            length  = length - 8; // removing the length for header and checksum
                            memcpy(pBuffer, strippedMessage, length);
                            validMessage = true;
                            printk("Compact message found\n");   
                        }
                        
                    }    
                }
                if (validMessage == false && FALLBACK_NAVPVT_ENABLED == true ){
                    printk("No compact message found. FallBack configuration is enabled so looking for NAVPVT msg.. \n");
                    startTimeMs = uPortGetTickTimeMs();
                    uGnssCfgValSet(gnssDeviceHandle, msgInfo[(messageType_t)NAVPVT].keyId, U_GNSS_CFG_ENABLE_MSG, U_GNSS_CFG_VAL_TRANSACTION_NONE,U_GNSS_CFG_VAL_LAYER_RAM);
                    
                    messageId.id.ubx = msgInfo[(messageType_t)NAVPVT].messageId;

                    while(!validMessage && ((uPortGetTickTimeMs() - startTimeMs) < (FALLBACK_TIMEOUT_IN_SECS*1000))){
                            length = uGnssMsgReceive(gnssDeviceHandle, &messageId, &pBuffer, bufferLength, FALLBACK_TIMEOUT_IN_SECS*1000, NULL);
                            if (length > 0 ){
                                if ((pBuffer[27] & 0x01) && (pBuffer[26] == 0x02 || pBuffer[26] == 0x03)) // Fix only valid when it is 2d or 3d fix and also GNSSFixOk flag is set 
                                {
                                    printk("Valid NAVPVT message found\n"); 
                                    validMessage = true;
                                }
                            }

                    }   
                }

                if(validMessage)
                {
                    printk("Final message :    ");        
					printUBXMessageinHex(pBuffer, length) ;
					count = length;
                }
            }
            else
            {
                printk("Error in enabling MeasX message\r\n");
            }
        }
        else{
            printk("Could not power on GNSS\r\n");
        }
    }
	uGnssDeinit();
    return count;

}


/** \fn void getLocationFromCloudLocate(const char *message, int32_t messageLength)
 * @brief Implementation of the "CloudLocate" API for XPLR-IoT-1 Kit.
 * @param[in] message meas message received from GNSS receiver
 * @param[in] messageLength length of the meas message
*/
void getLocationFromCloudLocate(const char *message, int32_t messageLength){
    uDeviceHandle_t devHandle;
    int32_t cellResponse;
    int32_t uartHandle;
    uAtClientHandle_t atClientHandle;
    uMqttClientContext_t *pContext = NULL;
    uMqttClientConnection_t connection = U_MQTT_CLIENT_CONNECTION_DEFAULT;
    char buffer[1024];
    size_t bufferSize;
    char subTopic[200];
    volatile bool messagesAvailable = false;
    int32_t startTimeMs;
    int32_t numofRetries = 2;
    int32_t errorCode = -1;

    saraR5InitPower();
    printk("SARA-R5 Powered on \r\n");
    setUartConfig(SARAuart);
    
    uDeviceInit();
    uAtClientInit();
    uCellInit();

    uartHandle = uPortUartOpen(2,
                               115200, NULL,
                               U_CELL_UART_BUFFER_LENGTH_BYTES,
                               -1,
                               -1,
                               -1,
                               -1);

    atClientHandle = uAtClientAdd(uartHandle,
                                  U_AT_CLIENT_STREAM_TYPE_UART,
                                  NULL,
                                  U_CELL_AT_BUFFER_LENGTH_BYTES);

    cellResponse = uCellAdd(U_CELL_MODULE_TYPE_SARA_R5,
                          atClientHandle,
                          -1,
                          -1,
                          -1, false, &devHandle);

    
    uAtClientPrintAtSet(atClientHandle, true);
    uAtClientLock(atClientHandle);
    
    //  AT command To disable echo
    uAtClientCommandStart(atClientHandle, "ATE0");
    uAtClientCommandStopReadResponse(atClientHandle);

    // AT+CMEE AT command enable or disable the use of result code
    // Enable +CME ERROR: result code and use verbose values
    uAtClientCommandStart(atClientHandle, "AT+CMEE=2");
    uAtClientCommandStopReadResponse(atClientHandle);
    uAtClientUnlock(atClientHandle);    
    
    // Bring up the network interface
    printk("Bringing up the network...\n");
	cellSearchstartTimeMs = uPortGetTickTimeMs();
    for (int i= 0; i< numofRetries && errorCode!= 0; i++){
        errorCode = uCellNetConnect(devHandle, NULL, APN, NULL, NULL, continueCellSearch);
        uPortTaskBlock (500);
    }
	if (isCellConnectAborted)
	{
		printk("Network registration aborted because it took more than cellRegistrationTimeout(s): %d. Please check if you have good network coverage \r\n", cellRegistrationTimeout );
	}

    if (errorCode == 0) {
        pContext = pUMqttClientOpen(devHandle, NULL);
        if (pContext != NULL) {
            connection.pBrokerNameStr = BROKER_NAME;
            connection.pClientIdStr = CLIENT_ID;
            connection.pUserNameStr = USERNAME;
            connection.pPasswordStr = PASSWORD;

            // Connect to the MQTT broker
            printk("Connecting to MQTT broker \"%s\"...\n", BROKER_NAME);
            if (uMqttClientConnect(pContext, &connection) == 0) {

                // Set up a callback to be called when new messages are available
                uMqttClientSetMessageCallback(pContext,
                                              messageIndicationCallback,
                                              (void *) &messagesAvailable);

                // Subscribe to the topic on the broker
                printk("Subscribing to topic \"%s\"...\n", SUB_TOPIC);
                if (uMqttClientSubscribe(pContext, SUB_TOPIC,
                                         U_MQTT_QOS_EXACTLY_ONCE)) {

                    // Publish our message to our topic on the MQTT broker
                    printk("Publishing \"%s\" to topic \"%s\"...\n",
                             message, PUB_TOPIC);
                    startTimeMs = uPortGetTickTimeMs();
                    if (uMqttClientPublish(pContext, PUB_TOPIC, message,
                                           messageLength,
                                           U_MQTT_QOS_EXACTLY_ONCE,
                                           false) == 0) {

                        // Wait for us to be notified that our new
                        // message is available on the broker
                        while (!messagesAvailable &&
                               (uPortGetTickTimeMs() - startTimeMs < 10000)) {
                            uPortTaskBlock(1000);
                        }

                        // Read the new message from the broker
                        while (uMqttClientGetUnread(pContext) > 0) {
                            bufferSize = sizeof(buffer);
                            if (uMqttClientMessageRead(pContext, subTopic,
                                                       sizeof(subTopic),
                                                       buffer, &bufferSize,
                                                       NULL) == 0) {
                                printk("CloudLocate response:  \"%.*s\"\n",bufferSize,buffer);
                            }
                        }
                    } else {
                        printk("Unable to publish our message \"%s\"!\n",
                                 message);
                    }
                } else {
                    printk("Unable to subscribe to topic \"%s\"!\n", SUB_TOPIC);
                }
                uMqttClientDisconnect(pContext);
            } else {
                printk("Unable to connect to MQTT broker \"%s\"!\n",BROKER_NAME);
            }
        } else {
            printk("Unable to create MQTT instance!\n");
        }

        uMqttClientClose(pContext);
        printk("Taking down network...\n");
    } 
    else {
      printk("Unable to bring up the network!\n");
    }
    
    uCellDeinit();
    uAtClientDeinit();
    uDeviceDeinit();
    printk("Done.\n");
}

/** \fn void main(void)
 * @brief Main function of the code 
 */
/* ----------------------------------------------------------------
 * MAIN FUNCTION
 * -------------------------------------------------------------- */

void main(void)
{
    unsigned char buffer[1000];
    int32_t msgSize;
	VERIFY(uPortInit() == 0, "uPortInit failed\n");
    msgSize = getMeasMessageFromGNSS(buffer, sizeof(buffer));
    if (msgSize > 0 )
    {
        getLocationFromCloudLocate(buffer, msgSize);   
    }
    else {
         printk("Unable to get message from GNSS. Please adjust timer values in configuration parameters\n");
    }
	uPortDeinit();
    	
}
