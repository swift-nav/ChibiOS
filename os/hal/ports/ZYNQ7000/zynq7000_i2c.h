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

#ifndef _ZYNQ7000_I2C_H_
#define _ZYNQ7000_I2C_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t ADDR;
  volatile uint32_t DATA;
  volatile uint32_t ISR;
  volatile uint32_t TRANS_SIZE;
  volatile uint32_t SLV_PAUSE;
  volatile uint32_t TIMEOUT;
  volatile uint32_t IMR;
  volatile uint32_t IER;
  volatile uint32_t IDR;
} i2c_t;

/* Bitfields */
#define I2C_CR_RW_Pos (0U)
#define I2C_CR_RW_Msk (0x1U << I2C_CR_RW_Pos)
#define I2C_CR_RW_WRITE (0x0U)
#define I2C_CR_RW_READ (0x1U)

#define I2C_CR_MS_Pos (1U)
#define I2C_CR_MS_Msk (0x1U << I2C_CR_MS_Pos)
#define I2C_CR_MS_SLAVE (0x0U)
#define I2C_CR_MS_MASTER (0x1U)

#define I2C_CR_NEA_Pos (2U)
#define I2C_CR_NEA_Msk (0x1U << I2C_CR_NEA_Pos)
#define I2C_CR_NEA_NORMAL (0x1U)

#define I2C_CR_ACK_EN_Pos (3U)
#define I2C_CR_ACK_EN_Msk (0x1U << I2C_CR_ACK_EN_Pos)

#define I2C_CR_HOLD_Pos (4U)
#define I2C_CR_HOLD_Msk (0x1U << I2C_CR_HOLD_Pos)

#define I2C_CR_SLV_MON_Pos (5U)
#define I2C_CR_SLV_MON_Msk (0x1U << I2C_CR_SLV_MON_Pos)

#define I2C_CR_FIFO_CLR_Pos (6U)
#define I2C_CR_FIFO_CLR_Msk (0x1U << I2C_CR_FIFO_CLR_Pos)

#define I2C_CR_DIV_B_Pos (8U)
#define I2C_CR_DIV_B_Msk (0x3FU << I2C_CR_DIV_B_Pos)

#define I2C_CR_DIV_A_Pos (14U)
#define I2C_CR_DIV_A_Msk (0x3U << I2C_CR_DIV_A_Pos)


#define I2C_SR_RX_RW_Pos (3U)
#define I2C_SR_RX_RW_Msk (0x1U << I2C_SR_RX_RW_Pos)

#define I2C_SR_RX_DV_Pos (5U)
#define I2C_SR_RX_DV_Msk (0x1U << I2C_SR_RX_DV_Pos)

#define I2C_SR_TX_DV_Pos (6U)
#define I2C_SR_TX_DV_Msk (0x1U << I2C_SR_TX_DV_Pos)

#define I2C_SR_RX_OVF_Pos (7U)
#define I2C_SR_RX_OVF_Msk (0x1U << I2C_SR_RX_OVF_Pos)

#define I2C_SR_BA_Pos (8U)
#define I2C_SR_BA_Msk (0x1U << I2C_SR_BA_Pos)


#define I2C_INT_COMP_Pos (0U)
#define I2C_INT_COMP_Msk (0x1U << I2C_INT_COMP_Pos)

#define I2C_INT_DATA_Pos (1U)
#define I2C_INT_DATA_Msk (0x1U << I2C_INT_DATA_Pos)

#define I2C_INT_NACK_Pos (2U)
#define I2C_INT_NACK_Msk (0x1U << I2C_INT_NACK_Pos)

#define I2C_INT_TO_Pos (3U)
#define I2C_INT_TO_Msk (0x1U << I2C_INT_TO_Pos)

#define I2C_INT_SLV_RDY_Pos (4U)
#define I2C_INT_SLV_RDY_Msk (0x1U << I2C_INT_SLV_RDY_Pos)

#define I2C_INT_RX_OVF_Pos (5U)
#define I2C_INT_RX_OVF_Msk (0x1U << I2C_INT_RX_OVF_Pos)

#define I2C_INT_TX_OVF_Pos (6U)
#define I2C_INT_TX_OVF_Msk (0x1U << I2C_INT_TX_OVF_Pos)

#define I2C_INT_RX_UNF_Pos (7U)
#define I2C_INT_RX_UNF_Msk (0x1U << I2C_INT_RX_UNF_Pos)

#define I2C_INT_ARB_LOST_Pos (9U)
#define I2C_INT_ARB_LOST_Msk (0x1U << I2C_INT_ARB_LOST_Pos)

#endif /* _ZYNQ7000_I2C_H_ */
