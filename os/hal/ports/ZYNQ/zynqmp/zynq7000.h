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

#ifndef _ZYNQMP_H_
#define _ZYNQMP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "pl390_gic.h"
#include "zynq_gpio.h"
#include "zynq_i2c.h"
#include "zynq_spi.h"
#include "zynq_ttc.h"
#include "zynq_uart.h"

/* Peripherals */
#define GIC_ICD_BASE 0xF9000000U
#define GIC_ICD ((gic_icd_t *)GIC_ICD_BASE)

#define GIC_ICC_BASE 0xF9001000U
#define GIC_ICC ((gic_icc_t *)GIC_ICC_BASE)

#define GPIO_BASE 0xFF0A0000U
#define GPIO ((gpio_t *)GPIO_BASE)

#define UART0_BASE 0xFF000000U
#define UART0 ((uart_t *)UART0_BASE)

#define UART1_BASE 0xFF010000U
#define UART1 ((uart_t *)UART1_BASE)

#define SPI0_BASE 0xFF040000U
#define SPI0 ((spi_t *)SPI0_BASE)

#define SPI1_BASE 0xFF050000U
#define SPI1 ((spi_t *)SPI1_BASE)

#define I2C0_BASE 0xFF020000U
#define I2C0 ((i2c_t *)I2C0_BASE)

#define I2C1_BASE 0xFF030000U
#define I2C1 ((i2c_t *)I2C1_BASE)

#define TTC0_BASE 0xFF110000U
#define TTC0 ((ttc_t *)TTC0_BASE)

#define TTC1_BASE 0xFF120000U
#define TTC1 ((ttc_t *)TTC1_BASE)

#define TTC2_BASE 0xFF130000U
#define TTC2 ((ttc_t *)TTC2_BASE)

#define TTC3_BASE 0xFF140000U
#define TTC3 ((ttc_t *)TTC3_BASE)

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
/*
  IRQ_ID_GBL_TIMER =          27,
  IRQ_ID_FIQ =                28,
  IRQ_ID_PRV_TIMER =          29,
  IRQ_ID_PRV_WDT =            30,
  IRQ_ID_IRQ =                31,
*/
  /* Shared Peripheral Interrupts */
  IRQ_ID_RPU0_PERF_MON =      40,
  IRQ_ID_RPU1_PERF_MON =      41,
  IRQ_ID_OCM =                42,
  IRQ_ID_LPD_APB =            43,
  IRQ_ID_RPU0_ECC =           44,
  IRQ_ID_RPU1_ECC =           45,
  IRQ_ID_NAND =               46,
  IRQ_ID_QSPI =               47,
  IRQ_ID_GPIO =               48,
  IRQ_ID_I2C0 =               49,
  IRQ_ID_I2C1 =               50,
  IRQ_ID_SPI0 =               51,
  IRQ_ID_SPI1 =               52,
  IRQ_ID_UART0 =              53,
  IRQ_ID_UART1 =              54,
  IRQ_ID_CAN0 =               55,
  IRQ_ID_CAN1 =               56,
  IRQ_ID_LPD_APM =            57,
  IRQ_ID_RTC_ALARM =          58,
  IRQ_ID_RTC_SECONDS =        59,
  IRQ_ID_RTC_CLK_MON =        60,
  IRQ_ID_IPI_CH7 =            61,
  IRQ_ID_IPI_CH8 =            62,
  IRQ_ID_IPI_CH9 =            63,
  IRQ_ID_IPI_CH10 =           64,
  IRQ_ID_IPI_CH1 =            65,
  IRQ_ID_IPI_CH2 =            66,
  IRQ_ID_IPI_CH0 =            67,
  IRQ_ID_TTC0_0 =             68,
  IRQ_ID_TTC0_1 =             69,
  IRQ_ID_TTC0_2 =             70,
  IRQ_ID_TTC1_0 =             71,
  IRQ_ID_TTC1_1 =             72,
  IRQ_ID_TTC1_2 =             73,
  IRQ_ID_TTC2_0 =             74,
  IRQ_ID_TTC2_1 =             75,
  IRQ_ID_TTC2_2 =             76,
  IRQ_ID_TTC3_0 =             77,
  IRQ_ID_TTC3_1 =             78,
  IRQ_ID_TTC3_2 =             79,
  IRQ_ID_SDIO0 =              80,
  IRQ_ID_SDIO1 =              81,
  IRQ_ID_SDIO0_WAKEUP =       82,
  IRQ_ID_SDIO1_WAKEUP =       83,
  IRQ_ID_LPD_SWDT =           84,
  IRQ_ID_CSU_SWDT =           85,
  IRQ_ID_LPD_ATB =            86,
  IRQ_ID_AIB =                87,
  IRQ_ID_SYS_MON =            88,
  IRQ_ID_GEM0 =               89,
  IRQ_ID_GEM0_WAKEUP =        90,
  IRQ_ID_GEM1 =               91,
  IRQ_ID_GEM1_WAKEUP =        92,
  IRQ_ID_GEM2 =               93,
  IRQ_ID_GEM2_WAKEUP =        94,
  IRQ_ID_GEM3 =               95,
  IRQ_ID_GEM3_WAKEUP =        96,
  IRQ_ID_USB0_ENDPOINT0 =     97,
  IRQ_ID_USB0_ENDPOINT1 =     98,
  IRQ_ID_USB0_ENDPOINT2 =     99,
  IRQ_ID_USB0_ENDPOINT3 =    100,
  IRQ_ID_USB0_OTG       =    101,
  IRQ_ID_USB1_ENDPOINT0 =    102,
  IRQ_ID_USB1_ENDPOINT1 =    103,
  IRQ_ID_USB1_ENDPOINT2 =    104,
  IRQ_ID_USB1_ENDPOINT3 =    105,
  IRQ_ID_USB1_OTG       =    106,
  IRQ_ID_USB0_WAKEUP =       107,
  IRQ_ID_USB1_WAKEUP =       108,
  IRQ_ID_LPD_DMA0 =          109,
  IRQ_ID_LPD_DMA1 =          110,
  IRQ_ID_LPD_DMA2 =          111,
  IRQ_ID_LPD_DMA3 =          112,
  IRQ_ID_LPD_DMA4 =          113,
  IRQ_ID_LPD_DMA5 =          114,
  IRQ_ID_LPD_DMA6 =          115,
  IRQ_ID_LPD_DMA7 =          116,
  IRQ_ID_CSU =               117,
  IRQ_ID_CSU_DMA =           118,
  IRQ_ID_EFUSE =             119,
  IRQ_ID_LPD_XMPU_XPPU =     120,
  IRQ_ID_FPGA0 =             121,
  IRQ_ID_FPGA1 =             122,
  IRQ_ID_FPGA2 =             123,
  IRQ_ID_FPGA3 =             124,
  IRQ_ID_FPGA4 =             125,
  IRQ_ID_FPGA5 =             126,
  IRQ_ID_FPGA6 =             127,
  IRQ_ID_FPGA7 =             128,
  /* Reserved 129:135 */
  IRQ_ID_FPGA8 =             136,
  IRQ_ID_FPGA9 =             137,
  IRQ_ID_FPGA10 =            138,
  IRQ_ID_FPGA11 =            139,
  IRQ_ID_FPGA12 =            140,
  IRQ_ID_FPGA13 =            141,
  IRQ_ID_FPGA14 =            142,
  IRQ_ID_FPGA15 =            143,
  IRQ_ID_DDR =               144,

  IRQ_ID__COUNT =            188 /* Maximum number of IRQ IDs */
} irq_id_t;

#endif /* _ZYNQ7000_H_ */
