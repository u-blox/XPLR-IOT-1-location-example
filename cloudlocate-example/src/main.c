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
#include <stdlib.h>
#include <stdio.h>

#include "ubxlib.h"
#include "u_cfg_os_platform_specific.h"
#include "u_cfg_app_platform_specific.h"

#include "u_mqtt_common.h"
#include "u_mqtt_client.h"

#include <zephyr.h>
#include <sys/printk.h>
#include <console/console.h>

#include "module_config.h"

#include <shell/shell.h>
#include <shell/shell_uart.h> 

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

//Flags to Enable/Disable Messages 
#define U_GNSS_CFG_ENABLE_MSG 1
#define U_GNSS_CFG_DISABLE_MSG 0

// Thingstream Broker URL
#define BROKER_NAME "mqtt.thingstream.io"

// Topic to publish GNSS measurements for CloudLocate
#define PUB_TOPIC "CloudLocate/GNSS/request"

// Input Parameters' Max Lengths
#define APN_MAXLEN 50
#define CLIENT_ID_MAXLEN 50
#define USERNAME_MAXLEN 25
#define PASSWORD_MAXLEN 50
#define SUB_TOPIC_MAXLEN 100

#define VERIFY(cond, fail_msg) \
    if (!(cond)) {\
        failed(fail_msg); \
    }


/* ----------------------------------------------------------------
 * TYPE DEFINITIONS
 * -------------------------------------------------------------- */

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


/* ------------------------------------------------------------------------------
 * CALLBACK DECLERATIONS
 * -----------------------------------------------------------------------------*/

/** \fn static void messageIndicationCallback(int32_t numUnread, void *pParam)
 * @brief Indication for unread message indications.
 * @param[in] numUnread count of unread characters
 * @param[in] pParam pointer to an unread incoming parameter
*/
static void messageIndicationCallback(int32_t, void *);

/** \fn static bool continueCellSearchCallback(uDeviceHandle_t deviceHandle)
 * @brief Indication to stop or continue CellConnect.
 * @param[in] deviceHandle device handle
*/
static bool continueCellSearchCallback(uDeviceHandle_t deviceHandle);


/* ------------------------------------------------------------------------------
 * HELPER FUNCTION DECLERATIONS
 * -----------------------------------------------------------------------------*/

/** \fn void failed(const char *msg)
 * @brief Function to log failed messages.
 * @param[in] msg incoming string - const char array
*/
void failed(const char *msg);

/** \fn void printUBXMessageinHex(char *pBuffer, int32_t bufferLenght)
 * @brief Function to print a string of defined length.
 * @param[in] pBuffer incoming string - char array
 * @param[in] bufferLenght length of the string to be printed
*/
void printUBXMessageinHex(char *pBuffer, int32_t bufferLenght);

/** \fn int32_t getMeasMessageFromGNSS(char *pBuffer, int32_t bufferLength, messageType_t msgType)
 * @brief Getting meas message from GNSS receiver
 * @param[in] pBuffer buffer to be filled with meas message
 * @param[in] bufferLength length of the buffer
 * @param[in] msgType type of compact message
*/
int32_t getMeasMessageFromGNSS(char *pBuffer, int32_t bufferLength, messageType_t msgType);

/** \fn int32_t getLocationFromCloudLocate(const struct shell *shell, size_t argc, char **argv)
 * @brief Function to request position from CloudLocate service
 * @param[in] shell pointer to shell instance
 * @param[in] argc count of arguments
 * @param[in] argv pointer to array of arguments passed.
*/
int32_t getLocationFromCloudLocate(const struct shell *shell, size_t argc, char **argv);

/** \fn static int getConfigParameters(const struct shell *shell, size_t argc, char **argv)
 * @brief Function to read configuration parameters
 * @param[in] shell pointer to shell instance
 * @param[in] argc count of arguments
 * @param[in] argv pointer to array of arguments passed. 
*/
static int getConfigParameters(const struct shell *shell, size_t argc, char **argv);

/** \fn static int setConfigParameters(const struct shell *shell, size_t argc, char **argv)
 * @brief Function to set configuration parameters
 * @param[in] shell pointer to shell instance
 * @param[in] argc count of arugments
 * @param[in] argv pointer to array of arguments passed. 
*/
static int setConfigParameters(const struct shell *shell, size_t argc, char **argv);


/* ------------------------------------------------------------------------------
 * GLOBALS
 * -----------------------------------------------------------------------------*/

// MQTT Credentials
char username[USERNAME_MAXLEN+1];
char password[PASSWORD_MAXLEN+1];
char subTopic[SUB_TOPIC_MAXLEN+1];
char clientId[CLIENT_ID_MAXLEN+1];

// APN name to set for the network
char APN[APN_MAXLEN+1]="tsiot";
// Timeout for Cell Registration in seconds
int32_t cellRegistrationTimeout = 40;
int32_t numOfSecondsToWaitForFirstMessage = 10;
int32_t compactMsgTimeoutInSecs = 200;
bool fallbackNavpvtEnabled = true;
int32_t fallbackTimeoutInSecs = 80;  

int32_t cellSearchstartTimeMs;
bool isCellConnectAborted = false;
// flag to indicate whether the configuration is done or not 
bool configurationDone = false;

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


/* ------------------------------------------------------------------------------
 * CALLBACK IMPLEMENTATION
 * -----------------------------------------------------------------------------*/

static void messageIndicationCallback(int32_t numUnread, void *pParam)
{
    bool *pMessagesAvailable = (bool *) pParam;
    printk("The broker says there are %d message(s) unread.\n", numUnread);
    *pMessagesAvailable = true;
}

static bool continueCellSearchCallback(uDeviceHandle_t deviceHandle)
{
    bool shouldCellSearchContinue = (uPortGetTickTimeMs() - cellSearchstartTimeMs < (cellRegistrationTimeout*1000));
	if (!shouldCellSearchContinue){
        isCellConnectAborted = true;
    }
    return shouldCellSearchContinue;
}

/* ------------------------------------------------------------------------------
 * HELPER FUNCTION IMPLEMENTATION
 * -----------------------------------------------------------------------------*/

void failed(const char *msg)
{
    uPortLog(msg);
    while(1);
}

void printUBXMessageinHex(char *pBuffer, int32_t bufferLenght)
{
    for(int i=0; i<bufferLenght; i++)
    {
        printk("%02x",pBuffer[i]);
    }
    printk("\n");
}

int32_t getMeasMessageFromGNSS(char *pBuffer, int32_t bufferLength, messageType_t msgType)
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
            if(uGnssCfgValSet(gnssDeviceHandle, msgInfo[msgType].keyId, U_GNSS_CFG_ENABLE_MSG, U_GNSS_CFG_VAL_TRANSACTION_NONE,U_GNSS_CFG_VAL_LAYER_RAM) == 0) {
                startTimeMs = uPortGetTickTimeMs();     
                printk("Enabled compact message.\r\n");


                messageId.type = U_GNSS_PROTOCOL_UBX;
                messageId.id.ubx = msgInfo[msgType].messageId; 
                printk("Waiting for compact message. Timer values TimeToWaitForFirstMessage: %d, CompactMessageTimeout: %d\r\n", numOfSecondsToWaitForFirstMessage,compactMsgTimeoutInSecs);
                
                // Waiting for compact message within the given timer values
                while(!validMessage && ((uPortGetTickTimeMs() - startTimeMs) < (compactMsgTimeoutInSecs*1000))  ) //timer is expired
                {
                    length = uGnssMsgReceive(gnssDeviceHandle, &messageId, &pBuffer, bufferLength, compactMsgTimeoutInSecs*1000, NULL);
                         
                    if (length > 0 && ((uPortGetTickTimeMs() - startTimeMs) >= (numOfSecondsToWaitForFirstMessage*1000)) ) //added check for the first message wait
                    {       
                        // MEASX is generated even if there is no satellite information so adding a length check
                        // so we know that whatever we are sending have atleast some satellite information in it
                        if(msgType == MEASX && length > 300)
                        {
                            validMessage = true;        
                        }

                        //CloudLocate service expects compact message(MEAS50 and MEAS20) without header and checksum so stripping down the message
                        if(msgType == MEAS20 || msgType == MEAS50 )
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
                if (validMessage == false && fallbackNavpvtEnabled == true ){
                    printk("No compact message found. FallBack configuration is enabled so looking for NAVPVT msg.. \n");
                    startTimeMs = uPortGetTickTimeMs();
                    uGnssCfgValSet(gnssDeviceHandle, msgInfo[(messageType_t)NAVPVT].keyId, U_GNSS_CFG_ENABLE_MSG, U_GNSS_CFG_VAL_TRANSACTION_NONE,U_GNSS_CFG_VAL_LAYER_RAM);
                    
                    messageId.id.ubx = msgInfo[(messageType_t)NAVPVT].messageId;
                    
                    //Waiting for a valid NAVPVT message within the given fallback timeout
                    while(!validMessage && ((uPortGetTickTimeMs() - startTimeMs) < (fallbackTimeoutInSecs*1000))){
                            length = uGnssMsgReceive(gnssDeviceHandle, &messageId, &pBuffer, bufferLength, fallbackTimeoutInSecs*1000, NULL);
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
                printk("Error in enabling meas message\r\n");
            }
        }
        else{
            printk("Could not power on GNSS\r\n");
        }
    }
    uGnssRemove(gnssDeviceHandle);
    uPortUartClose(uartHandle);
    uGnssPwrOff(gnssDeviceHandle);
	return count;

}

int32_t getLocationFromCloudLocate(const struct shell *shell, size_t argc, char **argv){
    
    if (configurationDone == false){
        shell_print(shell, "Before requesting location please complete the parameter configurtion using config command\r\n");
        return 1;
    }
    unsigned char gnssCompactMessage[1000];
    int32_t gnssCompactMessageLength;
    uDeviceHandle_t devHandle;
    int32_t cellResponse;
    int32_t uartHandle;
    uAtClientHandle_t atClientHandle;
    uMqttClientContext_t *pContext = NULL;
    uMqttClientConnection_t connection = U_MQTT_CLIENT_CONNECTION_DEFAULT;
    char receivedMsg[250];
    size_t receivedMsgSize;
    char receivedMsgTopic[200];
    volatile bool messagesAvailable = false;
    int32_t startTimeMs;
    int32_t numofRetries = 2;
    int32_t errorCode = -1;
    messageType_t msgType; 

    if (strcmp("measx", argv[1]) == 0)
    {
        msgType = MEASX;
    }
    else if (strcmp("meas50", argv[1]) == 0)
    {
        msgType = MEAS50;
    }
    else if ((strcmp("meas20", argv[1]) == 0))
    {
        msgType = MEAS20;
    }
    else {
        printk("Invalid message type: %s\n", argv[1]);
        return 1;
    }

    gnssCompactMessageLength = getMeasMessageFromGNSS(gnssCompactMessage, sizeof(gnssCompactMessage), msgType );
    if (gnssCompactMessageLength <= 0 )
    {
        printk("Unable to get message from GNSS. Please adjust timer values in configuration parameters\n");  
        return 1;
    }

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
        errorCode = uCellNetConnect(devHandle, NULL, APN, NULL, NULL, continueCellSearchCallback);
        uPortTaskBlock (500);
    }
    //Refers to cellRegistrationTimeout
	if (isCellConnectAborted)
	{
		printk("Network registration aborted because it took more than cellRegistrationTimeout(s): %d. Please check if you have good network coverage \r\n", cellRegistrationTimeout );
	}

    if (errorCode == 0) {
        pContext = pUMqttClientOpen(devHandle, NULL);
        if (pContext != NULL) {
            connection.pBrokerNameStr = BROKER_NAME;
            connection.pClientIdStr = clientId;
            connection.pUserNameStr = username;
            connection.pPasswordStr = password;

            // Connect to the MQTT broker
            printk("Connecting to MQTT broker \"%s\"...\n", BROKER_NAME);
            if (uMqttClientConnect(pContext, &connection) == 0) {

                // Set up a callback to be called when new messages are available
                uMqttClientSetMessageCallback(pContext,
                                              messageIndicationCallback,
                                              (void *) &messagesAvailable);

                // Subscribe to the topic on the broker
                printk("Subscribing to topic \"%s\"...\n", subTopic);
                if (uMqttClientSubscribe(pContext, subTopic,
                                         U_MQTT_QOS_EXACTLY_ONCE)) {

                    // Publish our message to our topic on the MQTT broker
                    printk("Publishing \"%s\" to topic \"%s\"...\n",
                             gnssCompactMessage, PUB_TOPIC);
                    startTimeMs = uPortGetTickTimeMs();
                    if (uMqttClientPublish(pContext, PUB_TOPIC, gnssCompactMessage,
                                           gnssCompactMessageLength,
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
                            receivedMsgSize = sizeof(receivedMsg);
                            if (uMqttClientMessageRead(pContext, receivedMsgTopic,
                                                       sizeof(receivedMsgTopic),
                                                       receivedMsg, &receivedMsgSize,
                                                       NULL) == 0) {
                                printk("CloudLocate response:  \"%.*s\"\n",receivedMsgSize, receivedMsg);
                            }
                        }
                    } else {
                        printk("Unable to publish our message \"%s\"!\n",
                                 gnssCompactMessage);
                    }
                } else {
                    printk("Unable to subscribe to topic \"%s\"!\n", subTopic);
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
    
    uCellRemove(devHandle);
    uAtClientRemove(atClientHandle);
    uPortUartClose(uartHandle);
    return 0;
}

static int getConfigParameters(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "MqttUsername: %s,\r\nMqttPassword: %s,\r\nDeviceId: %s,\r\nAPN: %s,\r\nCellRegistrationTimeout: %d,\r\nTimeToWaitForFirstMessage: %d,\r\nCompactMessageTimeout: %d,\r\nFallbackNavpvtStatus: %d,\r\nFallbackTimeout: %d \r\n",\
    username, \
    password,\
    clientId,\
    APN,\
    cellRegistrationTimeout, \
    numOfSecondsToWaitForFirstMessage,\
    compactMsgTimeoutInSecs,\
    fallbackNavpvtEnabled,\
    fallbackTimeoutInSecs);
    
    return 0;

}

static int setConfigParameters(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 10){
        bool invalid = false;
        uint32_t integerParameter;
        // checks parameters validity
        if( strlen( argv[1] ) >= USERNAME_MAXLEN ){
            shell_error( shell," MqttUsername length cannot be greater than %d\r\n", USERNAME_MAXLEN );    
            invalid = true;
        }
        if( strlen( argv[2] )  >= PASSWORD_MAXLEN ){
            shell_error( shell,"MqttPassword length cannot be greater than %d\r\n", PASSWORD_MAXLEN );    
            invalid = true;
        }
        if( strlen( argv[3] ) >= CLIENT_ID_MAXLEN ){
            shell_error( shell,"DeviceId length cannot be greater than %d\r\n", CLIENT_ID_MAXLEN );    
            invalid = true;
        }
        if( strlen( argv[4] ) >= APN_MAXLEN ){
            shell_error( shell,"APN length cannot be greater than %d\r\n", APN_MAXLEN );    
            invalid = true;
        }
        integerParameter = atoi(argv[5]);
        if(integerParameter < 0 && integerParameter >= 300 ){
            shell_error( shell,"CellRegistrationTimeout should be in between 1-300 seconds\r\n");    
            invalid = true;
        }
        integerParameter = atoi(argv[6]);
        if(integerParameter < 0 && integerParameter > 60 ){
            shell_error( shell,"TimeToWaitForFirstMessage should be in between 0-60 seconds\r\n");    
            invalid = true;
        }
        integerParameter = atoi(argv[7]);
        if(integerParameter < 0 && integerParameter > 300 ){
            shell_error( shell,"CompactMessageTimeout should be in between 0-300 seconds\r\n");    
            invalid = true;
        }
        integerParameter = atoi(argv[8]);
        if( integerParameter != 1 && integerParameter != 0 ){
            shell_error( shell,"Enter valid FallbackNavpvtStatus\r\n");    
            invalid = true;
        }
        integerParameter = atoi(argv[9]);
        if(integerParameter < 0 && integerParameter > 60 ){
            shell_error( shell,"FallbackTimeout should be in between 0-60 seconds\r\n");    
            invalid = true;
        }
        if( invalid ){
            return 1;
        }
        strcpy(username, argv[1]);
        strcpy(password, argv[2]);
        strcpy(clientId, argv[3]);
        sprintf(subTopic, "CloudLocate/%s/GNSS/response", argv[3]);
        strcpy(APN, argv[4]);
        cellRegistrationTimeout = atoi(argv[5]);
        numOfSecondsToWaitForFirstMessage = atoi(argv[6]);
        compactMsgTimeoutInSecs = atoi(argv[7]);
        fallbackNavpvtEnabled = atoi(argv[8]);
        fallbackTimeoutInSecs = atoi(argv[9]);
        configurationDone = true;
        getConfigParameters(shell, argc, argv);

    }
    else {
        shell_print(shell, "Missing params. Please enter all parameters: <MqttUsername> <MqttPassword> <DeviceId> <APN> <CellRegistrationTimeout(s)> <TimeToWaitForFirstMessage(s)> <CompactMessageTimeout(s)> <FallbackNavpvtStatus> <FallbackTimeout(s)>\r\n");
    }
    return 0;
}


/* ------------------------------------------------------------------------------
 * SHELL COMMANDS
 * 1- config
 * 	1a- config set <MqttUsername> <MqttPassword> <DeviceId> <APN> <CellRegistrationTimeout> <TimeToWaitForFirstMessage> <CompactMessageTimeout> <FallbackNavpvtStatus> <FallbackTimeout>
 * 	1b- config get
 * 2- location MsgType 
 * MsgType refers to meas20, meas50 or measx 
 * -----------------------------------------------------------------------------*/

//2nd Level of options - Config Subcommands
SHELL_STATIC_SUBCMD_SET_CREATE(config_sub_cmd,
        //Command to get configuration parameters configured using set command
        SHELL_CMD(get, NULL, "read configuration parameters",
                                               getConfigParameters),
        //Command to set configuration parameters
        SHELL_CMD(set,   NULL, "set configuration parameters: <MqttUsername> <MqttPassword> <DeviceId> <APN> <CellRegistrationTimeout(s)> <TimeToWaitForFirstMessage(s)> <CompactMessageTimeout(s)> <FallbackNavpvtStatus> <FallbackTimeout(s)>", setConfigParameters),
        SHELL_SUBCMD_SET_END
);
// Command to get the location based on the configured parameters
SHELL_CMD_REGISTER(location, NULL, "Get location from CloudLocate using measx/meas20/meas50", getLocationFromCloudLocate);
// 1st level of options - Configuration of parameters
SHELL_CMD_REGISTER(config, &config_sub_cmd, "Configuration of parameters", NULL);


/* ----------------------------------------------------------------
 * MAIN FUNCTION
 * -------------------------------------------------------------- */

void main(void)
{
	VERIFY(uPortInit() == 0, "uPortInit failed\n");
    uDeviceInit();
    uAtClientInit();
    uCellInit();
    uGnssInit(); 
    
    // Cellular network is required to publish gnss measurements to CloudLocate Service
    printk("Turning on SARA-R5..\r\n");
    saraR5InitPower();
    printk("SARA-R5 Powered on \r\n");
    setUartConfig(SARAuart);
    printk("Enter your required shell commands. Type help for further details");  
}
