/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _ZYNQ7000_H_
#define _ZYNQ7000_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "zynq7000_gic.h"
#include "zynq7000_gpio.h"
#include "zynq7000_i2c.h"
#include "zynq7000_prv_timer.h"
#include "zynq7000_prv_wdt.h"
#include "zynq7000_spi.h"
#include "zynq7000_ttc.h"
#include "zynq7000_uart.h"

/* Peripherals */
#define GIC_ICD_BASE 0xF8F01000U
#define GIC_ICD ((gic_icd_t *)GIC_ICD_BASE)

#define GIC_ICC_BASE 0xF8F00100U
#define GIC_ICC ((gic_icc_t *)GIC_ICC_BASE)

#define PRV_TIMER_BASE 0xF8F00600U
#define PRV_TIMER ((prv_timer_t *)PRV_TIMER_BASE)

#define PRV_WDT_BASE 0xF8F00620U
#define PRV_WDT ((prv_wdt_t *)PRV_WDT_BASE)

#define GPIO_BASE 0xE000A000U
#define GPIO ((gpio_t *)GPIO_BASE)

#define UART0_BASE 0xE0000000U
#define UART0 ((uart_t *)UART0_BASE)

#define UART1_BASE 0xE0001000U
#define UART1 ((uart_t *)UART1_BASE)

#define SPI0_BASE 0xE0006000U
#define SPI0 ((spi_t *)SPI0_BASE)

#define SPI1_BASE 0xE0007000U
#define SPI1 ((spi_t *)SPI1_BASE)

#define I2C0_BASE 0xE0004000U
#define I2C0 ((i2c_t *)I2C0_BASE)

#define I2C1_BASE 0xE0005000U
#define I2C1 ((i2c_t *)I2C1_BASE)

#define TTC0_BASE 0xF8001000U
#define TTC0 ((ttc_t *)TTC0_BASE)

#define TTC1_BASE 0xF8002000U
#define TTC1 ((ttc_t *)TTC1_BASE)

/* IRQ IDs */
typedef enum {
  /* Software Generated Interrupts */
  IRQ_ID_SW0 =                 0,
  IRQ_ID_SW1 =                 1,
  IRQ_ID_SW2 =                 2,
  IRQ_ID_SW3 =                 3,
  IRQ_ID_SW4 =                 4,
  IRQ_ID_SW5 =                 5,
  IRQ_ID_SW6 =                 6,
  IRQ_ID_SW7 =                 7,
  IRQ_ID_SW8 =                 8,
  IRQ_ID_SW9 =                 9,
  IRQ_ID_SW10 =               10,
  IRQ_ID_SW11 =               11,
  IRQ_ID_SW12 =               12,
  IRQ_ID_SW13 =               13,
  IRQ_ID_SW14 =               14,
  IRQ_ID_SW15 =               15,

  /* Private Peripheral Interrupts */
  /* IRQ IDs 16-26 are reserved */
  IRQ_ID_GBL_TIMER =          27,
  IRQ_ID_FIQ =                28,
  IRQ_ID_PRV_TIMER =          29,
  IRQ_ID_PRV_WDT =            30,
  IRQ_ID_IRQ =                31,

  /* Shared Peripheral Interrupts */
  IRQ_ID_PAR_CPU0 =           32,
  IRQ_ID_PAR_CPU1 =           33,
  IRQ_ID_PAR_L2CC =           34,
  IRQ_ID_OCM =                35,
  /* IRQ IDs 36 is reserved */
  IRQ_ID_PMU_CPU0 =           37,
  IRQ_ID_PMU_CPU1 =           38,
  IRQ_ID_XADC =               39,
  IRQ_ID_DEVC =               40,
  IRQ_ID_SWDT =               41,
  IRQ_ID_TTC0_0 =             42,
  IRQ_ID_TTC0_1 =             43,
  IRQ_ID_TTC0_2 =             44,
  IRQ_ID_DMAC_ABT =           45,
  IRQ_ID_DMAC_0 =             46,
  IRQ_ID_DMAC_1 =             47,
  IRQ_ID_DMAC_2 =             48,
  IRQ_ID_DMAC_3 =             49,
  IRQ_ID_SMC =                50,
  IRQ_ID_QSPI =               51,
  IRQ_ID_GPIO =               52,
  IRQ_ID_USB0 =               53,
  IRQ_ID_GEM0 =               54,
  IRQ_ID_GEM0_WAKE =          55,
  IRQ_ID_SDIO0 =              56,
  IRQ_ID_I2C0 =               57,
  IRQ_ID_SPI0 =               58,
  IRQ_ID_UART0 =              59,
  IRQ_ID_CAN0 =               60,
  IRQ_ID_FPGA0 =              61,
  IRQ_ID_FPGA1 =              62,
  IRQ_ID_FPGA2 =              63,
  IRQ_ID_FPGA3 =              64,
  IRQ_ID_FPGA4 =              65,
  IRQ_ID_FPGA5 =              66,
  IRQ_ID_FPGA6 =              67,
  IRQ_ID_FPGA7 =              68,
  IRQ_ID_TTC1_0 =             69,
  IRQ_ID_TTC1_1 =             70,
  IRQ_ID_TTC1_2 =             71,
  IRQ_ID_DMAC_4 =             72,
  IRQ_ID_DMAC_5 =             73,
  IRQ_ID_DMAC_6 =             74,
  IRQ_ID_DMAC_7 =             75,
  IRQ_ID_USB1 =               76,
  IRQ_ID_GEM1 =               77,
  IRQ_ID_GEM1_WAKE =          78,
  IRQ_ID_SDIO1 =              79,
  IRQ_ID_I2C1 =               80,
  IRQ_ID_SPI1 =               81,
  IRQ_ID_UART1 =              82,
  IRQ_ID_CAN1 =               83,
  IRQ_ID_FPGA8 =              84,
  IRQ_ID_FPGA9 =              85,
  IRQ_ID_FPGA10 =             86,
  IRQ_ID_FPGA11 =             87,
  IRQ_ID_FPGA12 =             88,
  IRQ_ID_FPGA13 =             89,
  IRQ_ID_FPGA14 =             90,
  IRQ_ID_FPGA15 =             91,
  IRQ_ID_PARITY =             92,
  /* IRQ IDs 93-95 are reserved */

  IRQ_ID__COUNT =             96 /* Maximum number of IRQ IDs */
} irq_id_t;

#endif /* _ZYNQ7000_H_ */
