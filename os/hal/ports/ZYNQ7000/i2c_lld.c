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

/**
 * @file    i2c_lld.c
 * @brief   PLATFORM I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "hal.h"

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

static void i2c_irq_handler(void *context);

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   I2C1 driver identifier.
 */
#if (PLATFORM_I2C_USE_I2C1 == TRUE) || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Initializes interrupts for the SPI peripheral.
 *
 * @param[in] icip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
static void interrupts_init(I2CDriver *icip) {
  gic_handler_register(icip->irq_id, i2c_irq_handler, icip);
  gic_irq_sensitivity_set(icip->irq_id, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(icip->irq_id, icip->irq_priority);
}

static u32 send_data(I2CDriver *icip) {
  u8 AvailBytes;
  s32 LoopCnt;
  s32 NumBytesToSend;

  /*
   * Determine number of bytes to write to FIFO.
   */
  AvailBytes = (u8)I2C_FIFO_DEPTH - (u8)icip->ic2->TRANS_SIZE;

  NumBytesToSend = icip->count - icip->txidx;

  if (NumBytesToSend > (s32)AvailBytes) {
    NumBytesToSend = (s32)AvailBytes;
  }

  /*
   * Fill FIFO with amount determined above.
   */
  for (LoopCnt = 0; LoopCnt < NumBytesToSend; LoopCnt++) {
    if (icip->txbuf != NULL) {
      icip->ic2->DATA = icip->txbuf[icip->txidx];
    } else {
      icip->ic2->DATA = 0;
    }
    icip->txidx++;
  }

  if (icip->txidx == icip->count) {
    icip->ic2->CR = icip->ic2->CR & ~I2C_CR_HOLD_Msk;
  }
  
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Handles a I2C IRQ.
 * @details Reads bytes from the RX FIFO and writes bytes to the TX FIFO
 *          until the current transaction is completed.
 *
 * @param[in] context     IRQ context, pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_irq_handler(void *context) {

  I2CDriver *icip = (I2CDriver *)context;
  i2c_t *i2c = (i2c_t *)icip->i2c;
  uint32_t i;

  /* Read and clear interrupt flags */
  uint32_t isr = i2c->ISR;
  i2c->ISR = isr;

  /*
   * Use the Mask register AND with the Interrupt Status register so
   * disabled interrupts are not processed.
   */
  isr &= ~i2c->IMR;

  if (isr & I2C_IXR_COMP_Msk) {
    send_data(icip);


    if (isr & I2C_IXR_DATA_MASK) {
      while (i2c->SR & I2C_SR_RXDV_Msk) {
	if (icip->rxbuf != NULL) {
	  icip->rxbuf[icip->rxidx] = icip->ic2->DATA;
	}
	icip->rxidx++
      }
    }
    
  }

  



  
  /** OLD SPI CODE **/
  
  if (isr & I2C_IXR_COMP_MASK && isr & I2C_IXR_DATA_MASK) {
    /* If there are no more bytes to send, then this must be the final RX.
     * Read out FIFO_RXWM_MAX - FIFO_HEADROOM bytes each interrupt until the
     * final interrupt. */
    bool final_rx = (icip->txidx >= icip->count);
    uint32_t bytes_to_read = final_rx ? (icip->count - icip->rxidx) :
                                        (FIFO_RXWM_MAX - FIFO_HEADROOM);

    /* Read from RX FIFO */
    for (i = 0; i < bytes_to_read; i++) {
      uint8_t rx = i2c->RXD;
      if (icip->rxbuf) {
        icip->rxbuf[icip->rxidx] = rx;
      }
      icip->rxidx++;
    }

    if (!final_rx) {
      /* Up until the final RX (when there are no bytes to send), RXWM will be
       * set to FIFO_RXWM_MAX - FIFO_HEADROOM, so there will be FIFO_RXWM_MAX -
       * FIFO_HEADROOM bytes free in the TX FIFO. If the remaining TX data
       * can fit into that space, send everything and interrupt when all bytes
       * are received. Otherwise send FIFO_RXWM_MAX - FIFO_HEADROOM bytes and
       * leave RXWM unchanged.
       */
      uint32_t bytes_to_write = icip->count - icip->txidx;
      if (bytes_to_write <= FIFO_RXWM_MAX - FIFO_HEADROOM) {
        i2c->RXWM = bytes_to_write + FIFO_HEADROOM;
      } else {
        bytes_to_write = FIFO_RXWM_MAX - FIFO_HEADROOM;
      }

      /* Write to TX FIFO */
      for (i = 0; i < bytes_to_write; i++) {
        if (icip->txbuf) {
          i2c->TXD = icip->txbuf[icip->txidx++];
        } else {
          i2c->TXD = 0;
          icip->txidx++;
        }
      }
    }

    /* Clean up if finished */
    if (final_rx) {
      osalDbgAssert(icip->rxidx == icip->count, "invalid rxidx");
      osalDbgAssert(icip->txidx == icip->count, "invalid txidx");

      /* Disable RXTRIG interrupt */
      i2c->IDR = I2C_INT_RXTRIG_Msk;
      _i2c_isr_code(icip);
    }
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if PLATFORM_I2C_USE_I2C1 == TRUE
  i2cObjectInit(&I2CD1);
  I2CD1.i2c = I2C1;
  I2CD1.irq_id = IRQ_ID_I2C1;
  I2CD1.irq_priority = ZYNQ7000_I2C_I2C1_IRQ_PRIORITY;
  interrupts_init(&I2CD1);
#endif
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {

  if (i2cp->state == I2C_STOP) {
    /* Enables the peripheral.*/
#if PLATFORM_I2C_USE_I2C1 == TRUE
    if (&I2CD1 == i2cp) {

    }
#endif
  }

}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  if (i2cp->state != I2C_STOP) {

    /* Disables the peripheral.*/
#if PLATFORM_I2C_USE_I2C1 == TRUE
    if (&I2CD1 == i2cp) {

    }
#endif
  }
}

/**
 * @brief   Receives data via the I2C bus as master.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     systime_t timeout) {

  (void)i2cp;
  (void)addr;
  (void)rxbuf;
  (void)rxbytes;
  (void)timeout;

  return MSG_OK;
}

/**
 * @brief   Transmits data via the I2C bus as master.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout) {

  (void)i2cp;
  (void)addr;
  (void)txbuf;
  (void)txbytes;
  (void)rxbuf;
  (void)rxbytes;
  (void)timeout;

  return MSG_OK;
}

#endif /* HAL_USE_I2C == TRUE */

/** @} */
