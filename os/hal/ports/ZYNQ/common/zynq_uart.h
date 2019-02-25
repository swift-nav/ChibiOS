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

#ifndef _ZYNQ7000_UART_H_
#define _ZYNQ7000_UART_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t MR;
  volatile uint32_t IER;
  volatile uint32_t IDR;
  volatile uint32_t IMR;
  volatile uint32_t ISR;
  volatile uint32_t BAUDGEN;
  volatile uint32_t RXTOUT;
  volatile uint32_t RXWM;
  volatile uint32_t MODEMCR;
  volatile uint32_t MODEMSR;
  volatile uint32_t SR;
  volatile uint32_t FIFO;
  volatile uint32_t BAUDDIV;
  volatile uint32_t FCDR;
  volatile uint32_t RESERVED0[2];
  volatile uint32_t TXWM;
} uart_t;

/* Bitfields */
#define UART_CR_RXRST_Pos (0U)
#define UART_CR_RXRST_Msk (0x1U << UART_CR_RXRST_Pos)

#define UART_CR_TXRST_Pos (1U)
#define UART_CR_TXRST_Msk (0x1U << UART_CR_TXRST_Pos)

#define UART_CR_RXEN_Pos (2U)
#define UART_CR_RXEN_Msk (0x1U << UART_CR_RXEN_Pos)

#define UART_CR_RXDIS_Pos (3U)
#define UART_CR_RXDIS_Msk (0x1U << UART_CR_RXDIS_Pos)

#define UART_CR_TXEN_Pos (4U)
#define UART_CR_TXEN_Msk (0x1U << UART_CR_TXEN_Pos)

#define UART_CR_TXDIS_Pos (5U)
#define UART_CR_TXDIS_Msk (0x1U << UART_CR_TXDIS_Pos)

#define UART_CR_TORST_Pos (6U)
#define UART_CR_TORST_Msk (0x1U << UART_CR_TORST_Pos)


#define UART_MR_CLKSEL_Pos (0U)
#define UART_MR_CLKSEL_Msk (0x1U << UART_MR_CLKSEL_Pos)
#define UART_MR_CLKSEL_REFCLK (0x0U)
#define UART_MR_CLKSEL_REFCLKDIV8 (0x1U)

#define UART_MR_CHRL_Pos (1U)
#define UART_MR_CHRL_Msk (0x3U << UART_MR_CHRL_Pos)
#define UART_MR_CHRL_8BIT (0x0U)
#define UART_MR_CHRL_7BIT (0x2U)
#define UART_MR_CHRL_6BIT (0x3U)

#define UART_MR_PAR_Pos (3U)
#define UART_MR_PAR_Msk (0x7U << UART_MR_PAR_Pos)
#define UART_MR_PAR_EVEN (0x0U)
#define UART_MR_PAR_ODD (0x1U)
#define UART_MR_PAR_ZERO (0x2U)
#define UART_MR_PAR_ONE (0x3U)
#define UART_MR_PAR_NONE (0x4U)

#define UART_MR_NBSTOP_Pos (6U)
#define UART_MR_NBSTOP_Msk (0x3U << UART_MR_NBSTOP_Pos)
#define UART_MR_NBSTOP_ONE (0x0U)
#define UART_MR_NBSTOP_ONE_FIVE (0x1U)
#define UART_MR_NBSTOP_TWO (0x2U)

#define UART_MR_CHMODE_Pos (8U)
#define UART_MR_CHMODE_Msk (0x3U << UART_MR_CHMODE_Pos)
#define UART_MR_CHMODE_NORMAL (0x0U)
#define UART_MR_CHMODE_ECHO (0x1U)
#define UART_MR_CHMODE_LOCAL_LOOPBACK (0x2U)
#define UART_MR_CHMODE_REMOTE_LOOPBACK (0x3U)


#define UART_INT_RXTRIG_Pos (0U)
#define UART_INT_RXTRIG_Msk (0x1U << UART_INT_RXTRIG_Pos)

#define UART_INT_RXEMPTY_Pos (1U)
#define UART_INT_RXEMPTY_Msk (0x1U << UART_INT_RXEMPTY_Pos)

#define UART_INT_RXFULL_Pos (2U)
#define UART_INT_RXFULL_Msk (0x1U << UART_INT_RXFULL_Pos)

#define UART_INT_TXEMPTY_Pos (3U)
#define UART_INT_TXEMPTY_Msk (0x1U << UART_INT_TXEMPTY_Pos)

#define UART_INT_TXFULL_Pos (4U)
#define UART_INT_TXFULL_Msk (0x1U << UART_INT_TXFULL_Pos)

#define UART_INT_RXOVER_Pos (5U)
#define UART_INT_RXOVER_Msk (0x1U << UART_INT_RXOVER_Pos)

#define UART_INT_RXFRAMING_Pos (6U)
#define UART_INT_RXFRAMING_Msk (0x1U << UART_INT_RXFRAMING_Pos)

#define UART_INT_RXPARITY_Pos (7U)
#define UART_INT_RXPARITY_Msk (0x1U << UART_INT_RXPARITY_Pos)

#define UART_INT_RXTOUT_Pos (8U)
#define UART_INT_RXTOUT_Msk (0x1U << UART_INT_RXTOUT_Pos)

#define UART_INT_DMS_Pos (9U)
#define UART_INT_DMS_Msk (0x1U << UART_INT_DMS_Pos)

#define UART_INT_TXTRIG_Pos (10U)
#define UART_INT_TXTRIG_Msk (0x1U << UART_INT_TXTRIG_Pos)

#define UART_INT_TXNFULL_Pos (11U)
#define UART_INT_TXNFULL_Msk (0x1U << UART_INT_TXNFULL_Pos)

#define UART_INT_TXOVER_Pos (12U)
#define UART_INT_TXOVER_Msk (0x1U << UART_INT_TXOVER_Pos)


#define UART_SR_RXTRIG_Pos (0U)
#define UART_SR_RXTRIG_Msk (0x1U << UART_SR_RXTRIG_Pos)

#define UART_SR_RXEMPTY_Pos (1U)
#define UART_SR_RXEMPTY_Msk (0x1U << UART_SR_RXEMPTY_Pos)

#define UART_SR_RXFULL_Pos (2U)
#define UART_SR_RXFULL_Msk (0x1U << UART_SR_RXFULL_Pos)

#define UART_SR_TXEMPTY_Pos (3U)
#define UART_SR_TXEMPTY_Msk (0x1U << UART_SR_TXEMPTY_Pos)

#define UART_SR_TXFULL_Pos (4U)
#define UART_SR_TXFULL_Msk (0x1U << UART_SR_TXFULL_Pos)

#define UART_SR_RXACTIVE_Pos (10U)
#define UART_SR_RXACTIVE_Msk (0x1U << UART_SR_RXACTIVE_Pos)

#define UART_SR_TXACTIVE_Pos (11U)
#define UART_SR_TXACTIVE_Msk (0x1U << UART_SR_TXACTIVE_Pos)

#define UART_SR_FLOWDEL_Pos (12U)
#define UART_SR_FLOWDEL_Msk (0x1U << UART_SR_FLOWDEL_Pos)

#define UART_SR_TXTRIG_Pos (13U)
#define UART_SR_TXTRIG_Msk (0x1U << UART_SR_TXTRIG_Pos)

#define UART_SR_TXNFULL_Pos (14U)
#define UART_SR_TXNFULL_Msk (0x1U << UART_SR_TXNFULL_Pos)


#endif /* _ZYNQ7000_UART_H_ */
