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

#ifndef _ZYNQ7000_SPI_H_
#define _ZYNQ7000_SPI_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t ISR;
  volatile uint32_t IER;
  volatile uint32_t IDR;
  volatile uint32_t IMR;
  volatile uint32_t ER;
  volatile uint32_t DR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;
  volatile uint32_t SICR;
  volatile uint32_t TXWM;
  volatile uint32_t RXWM;
  volatile uint32_t RESERVED0[51];
  volatile uint32_t MODID;
} spi_t;

/* Bitfields */
#define SPI_CR_MSTREN_Pos (0U)
#define SPI_CR_MSTREN_Msk (0x1U << SPI_CR_MSTREN_Pos)

#define SPI_CR_CPOL_Pos (1U)
#define SPI_CR_CPOL_Msk (0x1U << SPI_CR_CPOL_Pos)

#define SPI_CR_CPHA_Pos (2U)
#define SPI_CR_CPHA_Msk (0x1U << SPI_CR_CPHA_Pos)

#define SPI_CR_BAUDDIV_Pos (3U)
#define SPI_CR_BAUDDIV_Msk (0x7U << SPI_CR_BAUDDIV_Pos)
#define SPI_CR_BAUDDIV_4 (0x1U)
#define SPI_CR_BAUDDIV_8 (0x2U)
#define SPI_CR_BAUDDIV_16 (0x3U)
#define SPI_CR_BAUDDIV_32 (0x4U)
#define SPI_CR_BAUDDIV_64 (0x5U)
#define SPI_CR_BAUDDIV_128 (0x6U)
#define SPI_CR_BAUDDIV_256 (0x7U)

#define SPI_CR_REFCLK_Pos (8U)
#define SPI_CR_REFCLK_Msk (0x1U << SPI_CR_REFCLK_Pos)
#define SPI_CR_REFCLK_SPIREFCLK (0x0U)

#define SPI_CR_PERISEL_Pos (9U)
#define SPI_CR_PERISEL_Msk (0x1U << SPI_CR_PERISEL_Pos)

#define SPI_CR_CS_Pos (10U)
#define SPI_CR_CS_Msk (0xFU << SPI_CR_CS_Pos)

#define SPI_CR_MANCS_Pos (14U)
#define SPI_CR_MANCS_Msk (0x1U << SPI_CR_MANCS_Pos)

#define SPI_CR_MANSTARTEN_Pos (15U)
#define SPI_CR_MANSTARTEN_Msk (0x1U << SPI_CR_MANSTARTEN_Pos)

#define SPI_CR_MANSTART_Pos (16U)
#define SPI_CR_MANSTART_Msk (0x1U << SPI_CR_MANSTART_Pos)

#define SPI_CR_MODEFAILEN_Pos (17U)
#define SPI_CR_MODEFAILEN_Msk (0x1U << SPI_CR_MODEFAILEN_Pos)


#define SPI_INT_RXOVER_Pos (0U)
#define SPI_INT_RXOVER_Msk (0x1U << SPI_INT_RXOVER_Pos)

#define SPI_INT_MODEFAIL_Pos (1U)
#define SPI_INT_MODEFAIL_Msk (0x1U << SPI_INT_MODEFAIL_Pos)

#define SPI_INT_TXTRIG_Pos (2U)
#define SPI_INT_TXTRIG_Msk (0x1U << SPI_INT_TXTRIG_Pos)

#define SPI_INT_TXFULL_Pos (3U)
#define SPI_INT_TXFULL_Msk (0x1U << SPI_INT_TXFULL_Pos)

#define SPI_INT_RXTRIG_Pos (4U)
#define SPI_INT_RXTRIG_Msk (0x1U << SPI_INT_RXTRIG_Pos)

#define SPI_INT_RXFULL_Pos (5U)
#define SPI_INT_RXFULL_Msk (0x1U << SPI_INT_RXFULL_Pos)

#define SPI_INT_TXUNDER_Pos (6U)
#define SPI_INT_TXUNDER_Msk (0x1U << SPI_INT_TXUNDER_Pos)


#define SPI_ER_ENABLE_Pos (0U)
#define SPI_ER_ENABLE_Msk (0x1U << SPI_ER_ENABLE_Pos)


#define SPI_MODID (0x00090106U)

#endif /* _ZYNQ7000_SPI_H_ */
