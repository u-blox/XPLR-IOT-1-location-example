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
 * @file module_config.h
 * @brief Macros definitions and header of the functions for different configurations of all the modules on XPLR-IOT-1 Kit
 */




/* ------------------------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -----------------------------------------------------------------------------*/


#ifndef _MODULE_CONFIG_H_
#define _MODULE_CONFIG_H_

#define NINA_RESET_PIN          41    /* active low */
#define NINA_EN_PIN              8    /* Applies voltage railt to NINA module */
#define NORA_NINA_COMM_EN_PIN   42    /* Uart routes to NORA */


#define SARA_RESET_PIN 21           /* RESET */
#define SARA_PWR_ON_PIN 9           /* Applies POWER_ON Signal */
#define NORA_EN_SARA_PIN 10         /* Applies voltage rail to  module */
//#define SARA_INT_PIN 33             /* INT Pin Signal */

#define SARA_UART_RX    40
#define SARA_UART_TX    36
#define SARA_UART_CTS   38
#define SARA_UART_RTS   19

#define NORA_UART_RX    43
#define NORA_UART_TX    31
#define NORA_UART_CTS   30
#define NORA_UART_RTS   20

#define EN_MAX_PIN              4    /* enables M10 voltage rail */
#define MAX_BACKUP_ENABLE_PIN   37   /* Applies backup voltage rail to  module */
#define MAX_SAFEBOOT_NOT_PIN    44   /* ! during reset puts device in safeboot mode */ 


/*!
 * @brief SARA and NORA Uart
 */
typedef enum{
    SARAuart,
    NORAuart
}uartype;


/* ------------------------------------------------------------------------------
 * CONFIGURATION FUNCTIONS DECLARATIONS
 * -----------------------------------------------------------------------------*/


void nina15ResetEnable(void);

void nina15ResetDisable(void);

void nina15Enable(void);

void nina15Disable(void);

void ninaNoraCommEnable(void);

void ninaNoraCommDisable(void);


void nina15InitPower(void);

void saraR5InitPower(void);

void saraR5Disable(void);

void max10Enable(void);

void max10Disable(void);

void max10SafeBootEnable(void);

void max10SafeBootDisable(void);

void max10BackupSupplyEnable(void);

void max10BackupSupplyDisable(void);


void setUartConfig(uartype type);

	
#endif // _MODULE_CONFIG_H_