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
#include <string.h>

#include "ubxlib.h"
#include "u_cfg_os_platform_specific.h"

#include <zephyr.h>
#include <sys/printk.h>
#include <console/console.h>

#include "module_config.h"

/* ----------------------------------------------------------------
 * ERROR CHECKING
 * -------------------------------------------------------------- */

#define U_WIFI_UART_BUFFER_LENGTH_BYTES 600

#define VERIFY(cond, fail_msg) \
    if (!(cond)) {\
        failed(fail_msg); \
    }

void failed(const char *msg)
{
    uPortLog(msg);
    while(1);
}

/* ----------------------------------------------------------------
 * CONFIGURATION PARAMETERS
 * -------------------------------------------------------------- */

// ------------------------- CONFIGURATIONS START ---------------------------------
// WiFi signal strength
// -30 dBm	: Maximum signal strength.
// -50 dBm	: Excellent signal strength.
// -60 dBm	: Still good, reliable signal strength.
// -67 dBm	: Minimum value for all services that require smooth and reliable data traffic.
// -70 dBm	: Signal is not very strong
// -80 dBm	: Minimum value required to make a connection.
// -90 dBm	: Unlikely able to connect or make use of any services

// Allowed signal strength = -100dBm to 0dBm
// Set mininum wifi signal strength for WiFi position payload in dBm

#define MIN_WIFI_SIGNALSTRENGTH -90


// WiFi valid Access Points
// - Mininum required Access Points = 5 
// - Maximum required Access Points = 16
// Set Boundary Conditions for MAX valid Access Points to send to CellLocate service
#define NUM_WIFI_ACCESSPOINT 15

// CELL LOCATE Service Token 
#define CELL_LOCATE_TOKEN "<your-cellLocate-token>"
// CELL LOCATE Server address
#define CELL_LOCATE_SERVER_ADDRESS "cell-live1.services.u-blox.com"
// APN name to set for the network
#define APN "<network-apn>"

// Set it to 0 if you dont want to use wifi scan 
#define USE_WIFI 1


// ------------------------- CONFIGURATIONS END ---------------------------------

// Check configuration Parameters

#if NUM_WIFI_ACCESSPOINT < 5
#error Mininum five access points are required for getting position using CellLocate Wi-Fi
#endif



/* ------------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -----------------------------------------------------------------------------*/


/** \fn void max10NoraCommEnable(void)
 * @brief Enabling communication between M10 and Nora-B1
*/

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
char latLongToBits(int32_t thingX1e7,
                          int32_t *pWhole,
                          int32_t *pFraction)
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

/** \fn void printLocation(int32_t latitudeX1e7, int32_t longitudeX1e7)
 * @brief Print lat/long location as a clickable link.
 * @param[in] latitudeX1e7 Latitude upto 7 decimal points
 * @param[in] longitudeX1e7 Longitude upto 7 decimal points
*/
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

/** \fn int32_t getWifiScanPayload(char *pBuffer, int32_t bufferLength)
 * @brief Get Wi-Fi Access Points 
 * @param[in] pBuffer char array to be filled with APs information
 * @param[in] bufferLength maximum length of the buffer
 * @param[out] count length of the string received after Wi-Fi scan 
*/
int32_t getWifiScanPayload(char *pBuffer, int32_t bufferLength)
{
    printk("Preparing to get WiFi Scan Payload");
    int32_t uartHandle;
    uAtClientHandle_t atClientHandle;
    int count = 0;

    nina15InitPower();
    printk("NINA-W15 powered on \r\n");
    ninaNoraCommEnable();
    printk("NINA-Nora communication enabled \r\n");
    setUartConfig(NORAuart);
    printk("Nora UART configured \r\n");
    VERIFY(uPortInit() == 0, "uPortInit failed\n");
    uAtClientInit();
    uWifiInit();
    printk("Wi-Fi client initialized \r\n");


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
    uAtClientTimeoutSet(atClientHandle, 2000);
    // Command to perform a Wi-Fi scan operation and outputs Wi-Fi Location fingerprint
    // AT+ULOCWIFIFMT=<numAPs>,<rssiFilter>,<format>
    printk("Requesting Wi-Fi location fingerprint \r\n");
    uAtClientCommandStart(atClientHandle, "AT+ULOCWIFIFMT=");
    uAtClientWriteInt(atClientHandle, NUM_WIFI_ACCESSPOINT);
    uAtClientWriteInt(atClientHandle, MIN_WIFI_SIGNALSTRENGTH);
    uAtClientWriteInt(atClientHandle, 0);
    uAtClientCommandStop(atClientHandle);

    if (uAtClientResponseStart(atClientHandle, "+ULOCWIFIFMT:") == 0)
    {
        count = uAtClientReadString(atClientHandle, pBuffer, bufferLength, false);
    }
    uAtClientResponseStop(atClientHandle);

    uAtClientUnlock(atClientHandle);

    uWifiDeinit();
    uAtClientDeinit();
    uPortDeinit();

    ninaNoraCommDisable();
    nina15Disable();
    printk("NINA-W15 Powered off \r\n");
    printk("NINA Payload(%d) : %s\n",count, pBuffer);
    return count;

}

/** \fn uint8_t getPosition(char *pBuffer, uLocation_t *location, bool useWifiPayload)
 * @brief Function to get location using CellLocate service
 * @param[in] pBuffer input buffer containing wifi payload if set to USE_WIFI
 * @param[out] location To be filled when location is calculated
 * @param[in] useWifiPayload A flag that indicates to use WIFI location if set to 1. 
*/
uint8_t getPosition(char *pBuffer, uLocation_t *location, bool useWifiPayload)
{
    int32_t uartHandle;
    uAtClientHandle_t atClientHandle;
    uDeviceHandle_t cellHandle;
    
    int32_t numofRetries = 2;
    int32_t errorCode = -1;
    uint8_t locationReceived = U_ERROR_COMMON_UNKNOWN;
    saraR5InitPower();
    printk("SARA-R5 Powered on \r\n");
    setUartConfig(SARAuart);
    VERIFY(uPortInit() == 0, "uPortInit failed\n");
    uDeviceInit(); 
    uAtClientInit();
    uCellInit();
    

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
    for (int i= 0; i< numofRetries &&  errorCode!= 0; i++){
        errorCode = uCellNetConnect(cellHandle, NULL, APN, NULL, NULL, NULL);
        uPortTaskBlock (500);
    }
    if (errorCode == 0) {
        // Configure Cell Locate service
        if (uCellLocSetServer(cellHandle, CELL_LOCATE_TOKEN, CELL_LOCATE_SERVER_ADDRESS, NULL)==0){ 
            if (useWifiPayload){
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
        printk("Cellular module not able to connect to network.\n");
    }
    uCellDeinit();
    uAtClientDeinit();
    uDeviceDeinit();
    uPortDeinit();
    saraR5Disable();
    printk("SARA-R5 Powered off \r\n");
    return locationReceived;
}

/** \fn void main(void)
 * @brief Main function of the code 
 */
/* ----------------------------------------------------------------
 * MAIN FUNCTION
 * -------------------------------------------------------------- */

void main(void)
{
    unsigned char buffer[1023]; // MAX string length for AT command 1023
    uLocation_t location;

    if(USE_WIFI){
        if (getWifiScanPayload(buffer, sizeof(buffer)) == 0 ){
            printk("No WiFi AP information availble. Please adjust filter conditions according to the environment\n");
            return;
        }
    }
    
    if (getPosition(buffer, &location, USE_WIFI) == U_ERROR_COMMON_SUCCESS) {
        printLocation(location.latitudeX1e7, location.longitudeX1e7);
    }
    else {
        printk("Could not get location.\n");
    }
	
}
