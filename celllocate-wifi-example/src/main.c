/*
 * Copyright 2020 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


/** 
 * @file main.c
 * @brief Example code for XPLR-IoT-1 Kit to demonstrate CellLocate usage with Sara R5 and Nina-W15 modules.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "ubxlib.h"
#include "u_cfg_os_platform_specific.h"

#include <zephyr.h>
#include <console/console.h>
#include <shell/shell.h>
#include <device.h>
#include <devicetree.h>
#include <shell/shell_uart.h> 

#include "module_config.h"


/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

#define CELL_LOCATE_TOKEN_MAXLEN   25
#define CELL_LOCATE_SERVER_URL_MAXLEN  100
#define APN_MAXLEN 50
#define U_WIFI_UART_BUFFER_LENGTH_BYTES 600
#define VERIFY(cond, fail_msg) \
    if (!(cond)) {\
        failed(fail_msg); \
    }

/* ------------------------------------------------------------------------------
 * CALLBACK DECLERATIONS
 * -----------------------------------------------------------------------------*/

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

/** \fn char latLongToBits(int32_t thingX1e7, int32_t *pWhole, int32_t *pFraction)
 * @brief Convert a lat/long into a whole number and a bit-after-the-decimal-point that can be printed by a version of printf() 
 * that does not support floating point operations, returning the prefix (either "+" or "-").
 * The result should be printed with printf() format specifiers %c%d.%07d, e.g. something like:
 * int32_t whole;
 * int32_t fraction;
 * printf("%c%d.%07d/%c%d.%07d", latLongToBits(latitudeX1e7, &whole, &fraction),
 *                               whole, fraction,
 *                               latLongToBits(longitudeX1e7, &whole, &fraction),
 *                               whole, fraction);
 */
char latLongToBits(int32_t thingX1e7, int32_t *pWhole, int32_t *pFraction);

/** \fn void printLocation(int32_t latitudeX1e7, int32_t longitudeX1e7)
 * @brief Print lat/long location as a clickable link.
 * @param[in] latitudeX1e7 Latitude upto 7 decimal points
 * @param[in] longitudeX1e7 Longitude upto 7 decimal points
*/
void printLocation(int32_t latitudeX1e7, int32_t longitudeX1e7);

/** \fn int32_t getWifiScanPayload(char *pBuffer, int32_t bufferLength)
 * @brief Get Wi-Fi Access Points 
 * @param[in] pBuffer char array to be filled with APs information
 * @param[in] bufferLength maximum length of the buffer
 * @param[out] count length of the string received after Wi-Fi scan 
*/
int32_t getWifiScanPayload(char *pBuffer, int32_t bufferLength);


/** \fn uint8_t getPosition(char *pBuffer, uLocation_t *location, bool useWifiPayload)
 * @brief Function to get location using CellLocate service
 * @param[in] pBuffer input buffer containing wifi payload if set to USE_WIFI
 * @param[out] location To be filled when location is calculated
 * @param[in] useWifiPayload A flag that indicates to use WIFI location if set to 1. 
*/
uint8_t getPosition(char *pBuffer, uLocation_t *location);

/** \fn static int locationWifiHandler(const struct shell *shell, size_t argc, char **argv)
 * @brief WiFi command handler function to get location using CellLocate service with WiFi
 * @param[in] shell pointer to shell instance
 * @param[in] argc count of arugments
 * @param[in] argv pointer to array of arguments passed. 
*/
static int locationWifiHandler(const struct shell *shell, size_t argc, char **argv);

/** \fn static int locationCellHandler(const struct shell *shell, size_t argc, char **argv)
 * @brief Cell command handler function to get location using CellLocate service
 * @param[in] shell pointer to shell instance
 * @param[in] argc count of arugments
 * @param[in] argv pointer to array of arguments passed. 
*/
static int locationCellHandler(const struct shell *shell, size_t argc, char **argv);

/** \fn static int getConfigParameters(const struct shell *shell, size_t argc, char **argv)
 * @brief Function to read configuration parameters
 * @param[in] shell pointer to shell instance
 * @param[in] argc count of arugments
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

int wifiApSignalStrength = -90;
int numWifiAP = 15;
// CELL LOCATE Service Token 
char cellLocateToken[CELL_LOCATE_TOKEN_MAXLEN+1];
// CELL LOCATE Server address
char cellLocateServerUrl[CELL_LOCATE_SERVER_URL_MAXLEN+1]="cell-live1.services.u-blox.com";
// APN name to set for the network
char aPN[APN_MAXLEN+1]="tsiot";
// Timeout for Cell Registration in seconds
int32_t cellRegistrationTimeout = 40; 
// variable to keep connection attempt time
uint32_t cellNetConnectStartTime;
// flag to indicate whether the configuration is done or not 
bool configurationDone = false;

/* ------------------------------------------------------------------------------
 * CALLBACK IMPLEMENTATION
 * -----------------------------------------------------------------------------*/

static bool continueCellSearchCallback(uDeviceHandle_t deviceHandle)
{
    return (k_uptime_get() - cellNetConnectStartTime < (cellRegistrationTimeout*1000));
}

/* ------------------------------------------------------------------------------
 * HELPER FUNCTION IMPLEMENTATION
 * -----------------------------------------------------------------------------*/

void failed(const char *msg)
{
    uPortLog(msg);
    while(1);
}

char latLongToBits(int32_t thingX1e7, int32_t *pWhole, int32_t *pFraction)
{
    char prefix = '+';

    // Deal with the sign
    if (thingX1e7 < 0) {
        thingX1e7 = -thingX1e7;
        prefix = '-';
    }
    *pWhole = thingX1e7 / 10000000;
    *pFraction = thingX1e7 % 10000000;

    return prefix;
}

void printLocation(int32_t latitudeX1e7, int32_t longitudeX1e7)
{
    char prefixLat ;
    char prefixLong;
    int32_t wholeLat = 0;
    int32_t wholeLong = 0;
    int32_t fractionLat = 0;
    int32_t fractionLong = 0;
    prefixLat = latLongToBits(latitudeX1e7, &wholeLat, &fractionLat);
    prefixLong = latLongToBits(longitudeX1e7, &wholeLong, &fractionLong);
    printk("Position Lat: %c%d.%07d, Lon: %c%d.%07d\n",
             prefixLat, wholeLat, fractionLat, prefixLong, wholeLong,
             fractionLong);
    printk("Map URL: https://maps.google.com/?q=%c%d.%07d,%c%d.%07d\n",
             prefixLat, wholeLat, fractionLat, prefixLong, wholeLong,
             fractionLong);
}

int32_t getWifiScanPayload(char *pBuffer, int32_t bufferLength)
{
    printk("Preparing to get WiFi Scan Payload..\r\n");
    int32_t uartHandle;
    uAtClientHandle_t atClientHandle;
    int count = 0;

    memset(pBuffer,0,bufferLength);
    ninaNoraCommEnable();
    setUartConfig(NORAuart);
    printk("Nora UART configured \r\n");
    
    // Open a UART with the recommended buffer length
    // on your chosen UART HW block
    uartHandle = uPortUartOpen(2,
                               115200, NULL,
                               U_WIFI_UART_BUFFER_LENGTH_BYTES,
                               -1,
                               -1,
                               -1,
                               -1);

    // Add an AT client on the UART with the recommended
    // default buffer size.
    atClientHandle = uAtClientAdd(uartHandle,
                                  U_AT_CLIENT_STREAM_TYPE_UART,
                                  NULL,
                                  U_WIFI_UART_BUFFER_LENGTH_BYTES);

    
    // Set printing of AT commands by the cellular driver,
    // which can be useful while debugging.
    uAtClientPrintAtSet(atClientHandle, true);
    uAtClientLock(atClientHandle);
    printk("Setting AT client timeout \r\n");
    uAtClientTimeoutSet(atClientHandle, 5000);
	uAtClientFlush(atClientHandle);
    // Command to perform a Wi-Fi scan operation and outputs Wi-Fi Location fingerprint
    // AT+ULOCWIFIFMT=<numAPs>,<rssiFilter>,<format>
    printk("Requesting Wi-Fi location fingerprint \r\n");
    uAtClientCommandStart(atClientHandle, "AT+ULOCWIFIFMT=");
    uAtClientWriteInt(atClientHandle, numWifiAP);
    uAtClientWriteInt(atClientHandle, wifiApSignalStrength);
    uAtClientWriteInt(atClientHandle, 0);
    uAtClientCommandStop(atClientHandle);

    //Waiting for the response
    if (uAtClientResponseStart(atClientHandle, "+ULOCWIFIFMT:") == 0)
    {
        count = uAtClientReadString(atClientHandle, pBuffer, bufferLength, false);
    }
    uAtClientResponseStop(atClientHandle);
    uAtClientFlush(atClientHandle);
    uAtClientUnlock(atClientHandle);

    uAtClientRemove(atClientHandle);
    uPortUartClose(uartHandle);
    ninaNoraCommDisable();
    printk("NINA Payload(%d) : %s\r\n",count, pBuffer);
    return count;

}

uint8_t getPosition(char *pBuffer, uLocation_t *location)
{
    int32_t uartHandle;
    uAtClientHandle_t atClientHandle;
    uDeviceHandle_t cellHandle;
    
    int32_t numofRetries = 2;
    int32_t errorCode = -1;
    uint8_t locationReceived = U_ERROR_COMMON_UNKNOWN;
    printk("Turning on SARA-R5..\r\n");
    saraR5InitPower();
    setUartConfig(SARAuart);
    
    
    // Open a UART with the recommended buffer length
    // on your chosen UART HW block
    uartHandle = uPortUartOpen(2,
                               115200, NULL,
                               U_CELL_UART_BUFFER_LENGTH_BYTES,
                               -1,
                               -1,
                               -1,
                               -1);

    // Add an AT client on the UART with the recommended
    // default buffer size.
    atClientHandle = uAtClientAdd(uartHandle,
                                  U_AT_CLIENT_STREAM_TYPE_UART,
                                  NULL,
                                  U_CELL_AT_BUFFER_LENGTH_BYTES);

    uCellAdd(U_CELL_MODULE_TYPE_SARA_R5,
                          atClientHandle,
                          -1,
                          -1,
                          -1, false, &cellHandle);

    // Set printing of AT commands by the cellular driver,
    // which can be useful while debugging.                      
    uAtClientPrintAtSet(atClientHandle, true);
    uAtClientLock(atClientHandle);
    
    // AT command To disable echo
    uAtClientCommandStart(atClientHandle, "ATE0");
    uAtClientCommandStopReadResponse(atClientHandle);
    
    // AT+CMEE AT command enable or disable the use of result code
    // Enable +CME ERROR: result code and use verbose values
    uAtClientCommandStart(atClientHandle, "AT+CMEE=2");
    uAtClientCommandStopReadResponse(atClientHandle);
    uAtClientUnlock(atClientHandle);
    cellNetConnectStartTime = k_uptime_get();
    for (int i= 0; i< numofRetries &&  errorCode!= 0; i++){
        errorCode = uCellNetConnect(cellHandle, NULL, aPN, NULL, NULL, continueCellSearchCallback);
        uPortTaskBlock (500);
    }
    if (errorCode == 0) {
        // Configure Cell Locate service
        if (uCellLocSetServer(cellHandle, cellLocateToken, cellLocateServerUrl, NULL)==0){ 
            
            if (pBuffer != NULL){
                // feed wifi scan info to cellular module
                uAtClientLock(atClientHandle);

                // Configures the external location sensor used with the +ULOC command
                // Observation tags message in hexadecimal format
                // Set AT+ULOCEXT=<ULOCEXT_string>
                uAtClientCommandStart(atClientHandle, "AT+ULOCEXT=");
                uAtClientWriteString(atClientHandle, pBuffer, true);
                uAtClientCommandStopReadResponse(atClientHandle);
                uAtClientUnlock(atClientHandle);
            }
            
            // Now get location using Cell Locate
            if (uLocationGet(cellHandle, U_LOCATION_TYPE_CLOUD_CELL_LOCATE,
                            NULL, NULL,
                            location, NULL) == 0) {
                locationReceived = U_ERROR_COMMON_SUCCESS;

            } 
            
        }
    }
    else {
        printk("Cellular module not able to connect to network.\r\n");
    }
    
    uCellRemove(cellHandle);
    uAtClientRemove(atClientHandle);
    uPortUartClose(uartHandle);
    saraR5Disable();
    printk("SARA-R5 powered off \r\n");
    return locationReceived;
}

static int locationWifiHandler(const struct shell *shell, size_t argc, char **argv)
{
    if (configurationDone == false){
        shell_print(shell, "Before requesting location please complete the parameter configuration using config command\r\n");
        return 1;
    }
    unsigned char buffer[1023];
    uLocation_t location;
    int32_t rc = U_ERROR_COMMON_NOT_INITIALISED;
    if (getWifiScanPayload(buffer, sizeof(buffer)) > 0 )
    {
        if (getPosition(buffer, &location) == U_ERROR_COMMON_SUCCESS) {
            printLocation((int32_t)location.latitudeX1e7, (int32_t)location.longitudeX1e7);
            rc = U_ERROR_COMMON_SUCCESS;
        }
        else {
            shell_print(shell, "Could not get location.\r\n");
        }
    }
    else {
        shell_print(shell, "No WiFi AP information available. Please adjust filter conditions according to the environment\r\n");
    }
    
    return 0;
    
}

static int locationCellHandler(const struct shell *shell, size_t argc, char **argv)
{
    if (configurationDone == false){
        shell_print(shell, "Before requesting location please complete the parameter configurtion using config command\r\n");
        return 1;
    }
    int32_t rc = U_ERROR_COMMON_UNKNOWN;
    uLocation_t location;
    if (getPosition(NULL, &location) == U_ERROR_COMMON_SUCCESS) {
        printLocation((int32_t)location.latitudeX1e7, (int32_t)location.longitudeX1e7);
        rc = U_ERROR_COMMON_SUCCESS;
    }
    else {
        printk("Could not get location.\r\n");
    }
    return 0;
}

static int getConfigParameters(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "CellLocateServerURL: %s, Token: %s, APN: %s, CellRegistrationTimeout: %d, NumWifiAp: %d, WifiApSignalStrength: %d\r\n",\
    cellLocateServerUrl, \
    cellLocateToken,\
    aPN,\
    cellRegistrationTimeout, \
    numWifiAP,\
    wifiApSignalStrength);
    
    return 0;

}

static int setConfigParameters(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 7){
        bool invalid = false;
        uint32_t integerParameter;
        // checks parameters validity
        if( strlen( argv[1] ) >= CELL_LOCATE_SERVER_URL_MAXLEN ){
            shell_error( shell," CellLocateServerURL length cannot be greater than %d\r\n", CELL_LOCATE_SERVER_URL_MAXLEN );    
            invalid = true;
        }
        if( strlen( argv[2] )  >= CELL_LOCATE_TOKEN_MAXLEN ){
            shell_error( shell,"Token length cannot be greater than %d\r\n", CELL_LOCATE_TOKEN_MAXLEN );    
            invalid = true;
        }
        if( strlen( argv[3] ) >= APN_MAXLEN ){
            shell_error( shell,"APN length cannot be greater than %d\r\n", APN_MAXLEN );    
            invalid = true;
        }
        integerParameter = atoi(argv[4]);
        if(integerParameter < 0 && integerParameter >= 300 ){
            shell_error( shell,"CellRegistrationTimeout should be in between 1-300 seconds\r\n");    
            invalid = true;
        }
        integerParameter = atoi(argv[5]);
        if(integerParameter < 5 && integerParameter > 15 ){
            shell_error( shell,"NumWifiAP should be in between 5-15\r\n");    
            invalid = true;
        }
        integerParameter = atoi(argv[6]);
        if(integerParameter < -100 && integerParameter > 0 ){
            shell_error( shell,"WifiApSignalStrength should be in between -100-0\r\n");    
            invalid = true;
        }
        if( invalid ){
            return 1;
        }

        strcpy(cellLocateServerUrl, argv[1]);
        strcpy(cellLocateToken, argv[2]);
        strcpy(aPN, argv[3]);
        cellRegistrationTimeout = atoi(argv[4]);
        numWifiAP = atoi(argv[5]);
        wifiApSignalStrength = atoi(argv[6]);
        configurationDone = true;
        getConfigParameters(shell, argc, argv);

    }
    else {
        shell_print(shell, "Missing params. Please enter all parameters: <CellLocateServerURL> <Token> <APN> <CellRegistrationTimeout(s)> <NumWifiAp> <WifiApSignalStrength(dbm)>\r\n");
    }
    return 0;
}

/* ------------------------------------------------------------------------------
 * SHELL COMMANDS
 * 1- config
 * 	1a- config set <CellLocateServerURL> <Token> <APN> <CellRegistrationTimeout(s)> <NumWifiAp> <WifiApSignalStrength(dbm)
 * 	1b- config get
 * 2- location LocationType 
 * LocationType refers to cell or wifi 
 * -----------------------------------------------------------------------------*/

//2nd level of options 
SHELL_STATIC_SUBCMD_SET_CREATE(location_type,
        SHELL_CMD(wifi, NULL, "Use wifi access points information to get location",
                                               locationWifiHandler),
        SHELL_CMD(cell,   NULL, "Use cellular scan information to get location", locationCellHandler),
        SHELL_SUBCMD_SET_END
);
SHELL_STATIC_SUBCMD_SET_CREATE(config_sub_cmd,
        SHELL_CMD(get, NULL, "read configuration parameters",
                                               getConfigParameters),
        SHELL_CMD(set,   NULL, "set configuration parameters: <CellLocateServerURL> <Token> <APN> <CellRegistrationTimeout(s)> <NumWifiAp> <WifiApSignalStrength(dbm)>", setConfigParameters),
        SHELL_SUBCMD_SET_END
);

//1st level of options
SHELL_CMD_REGISTER(location, &location_type, "location command", NULL);
SHELL_CMD_REGISTER(config, &config_sub_cmd, "Configuration of parameters", NULL);
/* ----------------------------------------------------------------
 * MAIN FUNCTION
 * -------------------------------------------------------------- */

void main(void)
{   
    nina15InitPower();
    printk("NINA-W15 powered on \r\n");
    VERIFY(uPortInit() == 0, "uPortInit failed\r\n");
    uAtClientInit();
    uDeviceInit(); 
    uCellInit();
    printk("Enter your required shell commands. Type help for further details");
	
}
