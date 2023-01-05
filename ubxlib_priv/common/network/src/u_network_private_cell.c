/*
 * Copyright 2019-2022 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Only #includes of u_* and the C standard library are allowed here,
 * no platform stuff and no OS stuff.  Anything required from
 * the platform/OS must be brought in through u_port* to maintain
 * portability.
 */

/** @file
 * @brief Implementation of the cellular portion of the network API. The
 * contents of this file aren't any more "private" than the other
 * sources files but the associated header file should be private and
 * this is simply named to match.
 */

#ifdef U_CFG_OVERRIDE
# include "u_cfg_override.h" // For a customer's configuration override
#endif

#include "stddef.h"    // NULL, size_t etc.
#include "stdint.h"    // int32_t etc.
#include "stdbool.h"

#include "u_error_common.h"

#include "u_port.h"
#include "u_port_uart.h"

#include "u_at_client.h"

#include "u_device.h"
#include "u_device_shared.h"
#include "u_device_shared_cell.h"

#include "u_network_shared.h"

#include "u_cell_module_type.h"
#include "u_cell.h"
#include "u_cell_net.h"
#include "u_cell_pwr.h"

#include "u_network.h"
#include "u_network_config_cell.h"
#include "u_network_private_cell.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Call-back for connect/disconnect timeout.
static bool keepGoingCallback(uDeviceHandle_t devHandle)
{
    uDeviceCellContext_t *pContext;
    uDeviceInstance_t *pDevInstance = NULL;
    bool keepGoing = false;

    if (uDeviceGetInstance(devHandle, &pDevInstance) == 0) {
        pContext = (uDeviceCellContext_t *) pDevInstance->pContext;
        if ((pContext == NULL) ||
            (uPortGetTickTimeMs() < pContext->stopTimeMs)) {
            keepGoing = true;
        }
    }

    return keepGoing;
}

// Call-back for status changes.
//lint -esym(818, pParameter) Suppress "could be declared as pointing to const",
// gotta follow function signature
static void statusCallback(uCellNetRegDomain_t domain,
                           uCellNetStatus_t status,
                           void *pParameter)
{
    uDeviceInstance_t *pInstance = (uDeviceInstance_t *) pParameter;
    uDeviceNetworkData_t *pNetworkData;
    uNetworkStatusCallbackData_t *pStatusCallbackData;
    bool isUp;
    uNetworkStatus_t networkStatus;

    // Note: can't lock the device API here since we may collide
    // with a network up/down call that will have already locked
    // it and then may, internally, be waiting on something to pass
    // up the event queue that we are currently blocking (since
    // the same event queue is used for most things).
    // We rely on the fact that the various network down calls
    // are well behaved and will not pull the rug out from under
    // one of their callbacks.
    if (uDeviceIsValidInstance(pInstance)) {
        pNetworkData = pUNetworkGetNetworkData(pInstance, U_NETWORK_TYPE_CELL);
        if (pNetworkData != NULL) {
            pStatusCallbackData = (uNetworkStatusCallbackData_t *) pNetworkData->pStatusCallbackData;
            if ((pStatusCallbackData != NULL) &&
                (pStatusCallbackData->pCallback)) {
                isUp = U_CELL_NET_STATUS_MEANS_REGISTERED(status);
                networkStatus.cell.domain = (int32_t) domain;
                networkStatus.cell.status = (int32_t) status;
                pStatusCallbackData->pCallback((uDeviceHandle_t) pInstance,
                                               U_NETWORK_TYPE_CELL, isUp, &networkStatus,
                                               pStatusCallbackData->pCallbackParameter);
            }
        }
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Bring a cellular interface up or take it down.
int32_t uNetworkPrivateChangeStateCell(uDeviceHandle_t devHandle,
                                       const uNetworkCfgCell_t *pCfg,
                                       bool upNotDown)
{
    uDeviceCellContext_t *pContext;
    uDeviceInstance_t *pDevInstance;
    int32_t errorCode = uDeviceGetInstance(devHandle, &pDevInstance);
    bool (*pKeepGoingCallback)(uDeviceHandle_t devHandle) = keepGoingCallback;

    if (errorCode == 0) {
        errorCode = (int32_t) U_ERROR_COMMON_INVALID_PARAMETER;
        pContext = (uDeviceCellContext_t *) pDevInstance->pContext;
        if ((pCfg != NULL) && (pCfg->version == 0) &&
            (pCfg->type == U_NETWORK_TYPE_CELL) && (pContext != NULL)) {
            if (pCfg->pKeepGoingCallback != NULL) {
                // The user has given us a keep-going callback, so use it
                pKeepGoingCallback = pCfg->pKeepGoingCallback;
            } else {
                // Set the stop time for the connect/disconnect calls
                pContext->stopTimeMs = uPortGetTickTimeMs() +
                                       (((int64_t) pCfg->timeoutSeconds) * 1000);
            }
            if (upNotDown) {
                // Connect using automatic selection,
                // default no user name or password for the APN
                errorCode = uCellNetConnect(devHandle, NULL,
                                            pCfg->pApn,
                                            NULL, NULL,
                                            pKeepGoingCallback);
            } else {
                // Disconnect
                errorCode = uCellNetDisconnect(devHandle, pKeepGoingCallback);
            }
        }
    }

    return errorCode;
}

// Set a call-back to be called when the cellular network
// status changes.
int32_t uNetworkSetStatusCallbackCell(uDeviceHandle_t devHandle)
{
    return uCellNetSetRegistrationStatusCallback(devHandle, statusCallback,
                                                 (void *) devHandle);
}

// End of file
