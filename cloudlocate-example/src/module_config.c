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
 * @file module_config.c
 * @brief Functions for different configurations of all the modules on XPLR-IOT-1 Kit
 */


#include "module_config.h"
#include <zephyr.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_uarte.h>
#include <drivers/uart.h>

/*!
 * @brief Sara uart configuration 
 */
UARTE_PSEL_Type SARA ={
    SARA_UART_RTS,
    SARA_UART_TX,
    SARA_UART_CTS,
    SARA_UART_RX
};

/*!
 * @brief Nora uart configuration 
 */
UARTE_PSEL_Type NORA ={
    NORA_UART_RTS,
    NORA_UART_TX,
    NORA_UART_CTS,
    NORA_UART_RX
};


/** \fn nina15ResetEnable(void)
 * @brief Enabling reset on Nina-W15
*/
void nina15ResetEnable(void)
{
    // Apply VCC to NINA
    nrf_gpio_cfg_output((uint32_t) NINA_RESET_PIN);
    nrf_gpio_pin_set((uint32_t) NINA_RESET_PIN);
}

/** \fn nina15ResetDisable(void)
 * @brief Disabling reset on Nina-W15
*/
void nina15ResetDisable(void)
{
    // Disable VCC to NINA
    nrf_gpio_cfg_output((uint32_t) NINA_RESET_PIN);
    nrf_gpio_pin_clear((uint32_t) NINA_RESET_PIN);
}

/** \fn void nina15Enable(void)
 * @brief Nina-W15 enabled
*/
void nina15Enable(void)
{
    // Apply VCC to NINA
    nrf_gpio_cfg_output((uint32_t) NINA_EN_PIN);
    nrf_gpio_pin_set((uint32_t) NINA_EN_PIN);
}

/** \fn void nina15Disable(void)
 * @brief Nina-W15 disabled
*/
void nina15Disable(void)
{
    // Disable VCC to NINA
    nrf_gpio_cfg_output((uint32_t) NINA_EN_PIN);
    nrf_gpio_pin_clear((uint32_t) NINA_EN_PIN);
}

/** \fn void ninaNoraCommEnable(void)
 * @brief Enabling communication between Nina-W15 and Nora-B1
*/
void ninaNoraCommEnable(void)
{
    // Select UARTE Comm, UART-BRIDGE comm de-select
    nrf_gpio_cfg_output((uint32_t) NORA_NINA_COMM_EN_PIN);
    nrf_gpio_pin_set((uint32_t) NORA_NINA_COMM_EN_PIN);
}

/** \fn void ninaNoraCommDisable(void)
 * @brief Disabling communication between Nina-W15 and Nora-B1
*/
void ninaNoraCommDisable(void)
{
    nrf_gpio_cfg_output((uint32_t) NORA_NINA_COMM_EN_PIN);
    nrf_gpio_pin_clear((uint32_t) NORA_NINA_COMM_EN_PIN);

}

/** \fn void nina15InitPower(void)
 * @brief Nina-W15 powered ON
*/
void nina15InitPower(void)
{
    nina15Enable();

    // Initialize the NINA module, we will hold it in reset until it is powered toggle reset
    nina15ResetEnable();
    k_sleep(K_MSEC(10));
    nina15ResetDisable();

    // t_Startup time after release of reset UBX-18006647 - R10 (page 23)
    k_sleep(K_MSEC(2600));
}

/** \fn void saraR5InitPower(void)
 * @brief SARA R5 powered ON
*/
void saraR5InitPower(void)
{
    
    nrf_gpio_cfg_output((uint32_t) NORA_EN_SARA_PIN);
    nrf_gpio_cfg_output((uint32_t) SARA_PWR_ON_PIN);
    

    // Power on sequence

    // Apply power to the module and wait a little time
    nrf_gpio_pin_set((uint32_t) NORA_EN_SARA_PIN);
    k_sleep(K_MSEC(500));
    
    //assert POWER_ON pin for a valid time
    nrf_gpio_pin_set((uint32_t) SARA_PWR_ON_PIN);
    k_sleep(K_MSEC(2100));
    //deassert POWER_ON pin
    nrf_gpio_pin_clear((uint32_t) SARA_PWR_ON_PIN);
    
}

/** \fn void saraR5Disable(void)
 * @brief SARA R5 powered OFF
*/
void saraR5Disable(void)
{
    // disable VCC to SaraR5
    nrf_gpio_cfg_output((uint32_t) NORA_EN_SARA_PIN);
    nrf_gpio_pin_clear((uint32_t) NORA_EN_SARA_PIN);
}

/** \fn void max10Enable(void)
 * @brief M10 GNSS receiver enabled
*/
void max10Enable(void)
{
    // Apply VCC to max10
    nrf_gpio_cfg_output((uint32_t) EN_MAX_PIN);
    nrf_gpio_pin_set((uint32_t) EN_MAX_PIN);
}

/** \fn void max10Disable(void)
 * @brief M10 GNSS receiver disabled
*/
void max10Disable(void)
{
    // Apply VCC to
    nrf_gpio_cfg_output((uint32_t) EN_MAX_PIN);
    nrf_gpio_pin_clear((uint32_t) EN_MAX_PIN);
}

/** \fn void max10SafeBootEnable(void)
 * @brief M10 GNSS receiver Safe Boot mode enabled
*/
void max10SafeBootEnable(void)
{
    // Apply VCC to max10
    nrf_gpio_cfg_output((uint32_t) MAX_SAFEBOOT_NOT_PIN);
    nrf_gpio_pin_set((uint32_t) MAX_SAFEBOOT_NOT_PIN);
}

/** \fn void max10SafeBootDisable(void)
 * @brief M10 GNSS receiver safe boot mode disabled
*/
void max10SafeBootDisable(void)
{
    // Apply VCC to
    nrf_gpio_cfg_output((uint32_t) MAX_SAFEBOOT_NOT_PIN);
    nrf_gpio_pin_clear((uint32_t) MAX_SAFEBOOT_NOT_PIN);
}

/** \fn void max10BackupSupplyEnable(void)
 * @brief M10 GNSS receiver backup supply enabled
*/
void max10BackupSupplyEnable(void)
{
    // Apply VCC to max10
    nrf_gpio_cfg_output((uint32_t) MAX_BACKUP_ENABLE_PIN);
    nrf_gpio_pin_set((uint32_t) MAX_BACKUP_ENABLE_PIN);
}

/** \fn void max10BackupSupplyDisable(void)
 * @brief M10 GNSS receiver backup supply disabled
*/
void max10BackupSupplyDisable(void)
{
    // Apply VCC to
    nrf_gpio_cfg_output((uint32_t) MAX_BACKUP_ENABLE_PIN);
    nrf_gpio_pin_clear((uint32_t) MAX_BACKUP_ENABLE_PIN);
}

/** \fn void max10NoraCommEnable(void)
 * @brief Enabling communication between M10 and Nora-B1
*/
void max10NoraCommEnable(void)
{
    // Apply VCC to
    nrf_gpio_cfg_output((uint32_t) MAX_COM_EN_PIN);
    nrf_gpio_pin_set((uint32_t) MAX_COM_EN_PIN);
}

/** \fn void max10NoraCommDisable(void)
 * @brief Disabling communication between M10 and Nora-B1
*/
void max10NoraCommDisable(void)
{
    // Apply VCC to
    nrf_gpio_cfg_output((uint32_t) MAX_COM_EN_PIN);
    nrf_gpio_pin_clear((uint32_t) MAX_COM_EN_PIN);
}


/** \fn void setUartConfig(uartype type)
 * @brief Setting UART configurations
*/
void setUartConfig(uartype type){

    UARTE_PSEL_Type pins;

    if(type == SARAuart){
        pins = SARA;
    }
    else if(type == NORAuart){
        pins = NORA;
    }
    else{
        return;
    }

    nrf_uarte_disable(NRF_UARTE2_S);
    
    // Set up TX and RX pins.
    nrf_gpio_pin_set(pins.TXD);
    nrf_gpio_cfg_output(pins.TXD);
    nrf_gpio_cfg_input(pins.RXD, NRF_GPIO_PIN_NOPULL);
    nrf_uarte_txrx_pins_set(NRF_UARTE2_S, pins.TXD, pins.RXD);

    // Set up CTS and RTS pins.
    nrf_gpio_cfg_input(pins.CTS, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_set(pins.RTS);
    nrf_gpio_cfg_output(pins.RTS);
    nrf_uarte_hwfc_pins_set(NRF_UARTE2_S, pins.RTS, pins.CTS);

    // enable UARTE
    nrf_uarte_enable(NRF_UARTE2_S);
    // kick-start Rx
    nrf_uarte_task_trigger(NRF_UARTE2_S, NRF_UARTE_TASK_STARTRX);

}