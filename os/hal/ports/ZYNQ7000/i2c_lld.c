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
 * @file    ZYNQ7000/i2c_lld.c
 * @brief   ZYNQ7000 I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "gic.h"
#include "zynq7000.h"
#include "hal.h"


#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

#define FIFO_SIZE     16
#define RX_SIZE_MAX  255

#define DIV_N         22
#define DIV_A_MAX      3
#define DIV_B_MAX     63

#define DIV_ROUND_UNSIGNED(num, div) (((num) + (div) / 2U) / (div))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

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
#if (ZYNQ7000_I2C_USE_I2C0 == TRUE) || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/**
 * @brief   I2C2 driver identifier.
 */
#if (ZYNQ7000_I2C_USE_I2C1 == TRUE) || defined(__DOXYGEN__)
I2CDriver I2CD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Computes the optimal dividers to achieve the specified
 *          clock frequency.
 *
 * @param[in] clk           target clock frequency (Hz)
 * @param[out] r_div_a      pointer to the output div_a value
 * @param[out] r_div_b      pointer to the output div_b value
 *
 * @notapi
 */
static void clk_calc(uint32_t clk, uint32_t *r_div_a, uint32_t *r_div_b) {
  uint32_t ref_clk = ZYNQ7000_CPU_1x_FREQUENCY_Hz;
  uint32_t best_error = UINT32_MAX;
  uint32_t best_div_a = 0;
  uint32_t best_div_b = 0;

  uint32_t div_a;
  for (div_a = 0; div_a <= DIV_A_MAX; div_a++) {
    uint32_t div_b = DIV_ROUND_UNSIGNED(ref_clk, DIV_N * (div_a + 1) * clk) - 1;

    /* Make sure div_b is in range */
    if (div_b > DIV_B_MAX)
      div_b = DIV_B_MAX;

    uint32_t clk_actual = ref_clk / (DIV_N * (div_a + 1) * (div_b + 1));
    uint32_t error = clk > clk_actual ? (clk - clk_actual) :
                                        (clk_actual - clk);
    if (error < best_error) {
      best_div_a = div_a;
      best_div_b = div_b;
      best_error = error;
    }
  }

  *r_div_a = best_div_a;
  *r_div_b = best_div_b;
}

/**
 * @brief   Initializes interrupts for the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void interrupts_init(I2CDriver *i2cp) {
  gic_handler_register(i2cp->irq_id, i2c_irq_handler, i2cp);
  gic_irq_sensitivity_set(i2cp->irq_id, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(i2cp->irq_id, i2cp->irq_priority);
}

/**
 * @brief   Starts the I2C peripheral.
 *
 * @param[in] i2c       pointer to the hardware I2C registers
 *
 * @notapi
 */
static void i2c_start(i2c_t *i2c, uint32_t clk) {

  /* Disable interrupts */
  i2c->IDR = I2C_INT_COMP_Msk | I2C_INT_DATA_Msk | I2C_INT_NACK_Msk |
             I2C_INT_TO_Msk | I2C_INT_SLV_RDY_Msk | I2C_INT_RX_OVF_Msk |
             I2C_INT_TX_OVF_Msk | I2C_INT_RX_UNF_Msk | I2C_INT_ARB_LOST_Msk;

  /* Clear interrupt flags */
  uint32_t isr = i2c->ISR;
  i2c->ISR = isr;

  /* Configure mode and clear FIFO */
  uint32_t div_a;
  uint32_t div_b;
  clk_calc(clk, &div_a, &div_b);
  i2c->CR = (I2C_CR_MS_MASTER << I2C_CR_MS_Pos) |
            (I2C_CR_NEA_NORMAL << I2C_CR_NEA_Pos) |
            (1 << I2C_CR_ACK_EN_Pos) |
            (1 << I2C_CR_FIFO_CLR_Pos) |
            (div_b << I2C_CR_DIV_B_Pos) |
            (div_a << I2C_CR_DIV_A_Pos);

  i2c->TIMEOUT = 0xff;

  /* Enable interrupts */
  i2c->IER = I2C_INT_COMP_Msk | I2C_INT_DATA_Msk | I2C_INT_NACK_Msk |
             I2C_INT_TO_Msk | I2C_INT_SLV_RDY_Msk | I2C_INT_RX_OVF_Msk |
             I2C_INT_TX_OVF_Msk | I2C_INT_RX_UNF_Msk | I2C_INT_ARB_LOST_Msk;
}

/**
 * @brief   Stops the I2C peripheral.
 *
 * @param[in] i2c       pointer to the hardware I2C registers
 *
 * @notapi
 */
static void i2c_stop(i2c_t *i2c) {

  /* Disable interrupts */
  i2c->IDR = I2C_INT_COMP_Msk | I2C_INT_DATA_Msk | I2C_INT_NACK_Msk |
             I2C_INT_TO_Msk | I2C_INT_SLV_RDY_Msk | I2C_INT_RX_OVF_Msk |
             I2C_INT_TX_OVF_Msk | I2C_INT_RX_UNF_Msk | I2C_INT_ARB_LOST_Msk;
}

/**
 * @brief   Configure registers for TX.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void tx_configure(I2CDriver *i2cp) {

  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  /* Configure write mode */
  i2c->CR = (i2c->CR & ~I2C_CR_RW_Msk) |
            (I2C_CR_RW_WRITE << I2C_CR_RW_Pos);
}

/**
 * @brief   Configure registers for RX.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void rx_configure(I2CDriver *i2cp) {

  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  /* Configure read mode */
  i2c->CR = (i2c->CR & ~I2C_CR_RW_Msk) |
            (I2C_CR_RW_READ << I2C_CR_RW_Pos);

  /* Set transfer size */
  i2c->TRANS_SIZE = i2cp->rxbytes;
}

/**
 * @brief   Fill the TX FIFO to capacity or until no data remains to be sent.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void tx_fifo_fill(I2CDriver *i2cp) {

  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  /* Refill TX FIFO */
  while ((i2cp->txidx < i2cp->txbytes) &&
         (i2c->TRANS_SIZE < FIFO_SIZE)) {
    i2c->DATA = i2cp->txbuf[i2cp->txidx++];
  }
}

/**
 * @brief   Update the HOLD bit according to the current
 *          txidx and rxidx values.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void hold_update(I2CDriver *i2cp) {

  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  /* Hold until
   * 1) All TX bytes have been written to the FIFO
   * 2) There is space for all remaining RX bytes in the FIFO
   */
  bool hold = (i2cp->txidx < i2cp->txbytes) ||
              (i2cp->rxidx + FIFO_SIZE < i2cp->rxbytes);

  i2c->CR = (i2c->CR & ~I2C_CR_HOLD_Msk) |
            ((hold ? 1 : 0) << I2C_CR_HOLD_Pos);
}

/**
 * @brief   Update the HOLD bit according to the current
 *          txidx and rxidx values.
 *
 * @param[in] isr       value of the I2C ISR register
 *
 * @return              the associated error flags
 *
 * @notapi
 */
static i2cflags_t isr_errors_parse(uint32_t isr) {

  i2cflags_t flags = I2C_NO_ERROR;

  if (isr & I2C_INT_NACK_Msk) {
    flags |= I2C_ACK_FAILURE;
  }

  if (isr & I2C_INT_TO_Msk) {
    flags |= I2C_TIMEOUT;
  }

  if (isr & (I2C_INT_RX_OVF_Msk | I2C_INT_TX_OVF_Msk | I2C_INT_RX_UNF_Msk)) {
    flags |= I2C_OVERRUN;
  }

  if (isr & I2C_INT_ARB_LOST_Msk) {
    flags |= I2C_ARBITRATION_LOST;
  }

  return flags;
}

/**
 * @brief   Execute an I2C transaction.
 * @note    txbuf, txbytes, txidx, rxbuf, rxbytes, rxidx must be initialized
 *          prior to calling this function.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static msg_t txn_execute(I2CDriver *i2cp, i2caddr_t addr, systime_t timeout) {

  if ((i2cp->txbytes == 0) && (i2cp->rxbytes == 0)) {
    osalDbgAssert(0, "txbytes and rxbytes == 0");
    return MSG_RESET;
  }

  if (i2cp->rxbytes > RX_SIZE_MAX) {
    osalDbgAssert(0, "rxbytes > 255");
    return MSG_RESET;
  }

  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  /* Reset error flags */
  i2cp->errors = I2C_NO_ERROR;

  /* Calculate bus busy timeout */
  systime_t start = osalOsGetSystemTimeX();
  systime_t end = start + OSAL_MS2ST(ZYNQ7000_I2C_BUSY_TIMEOUT_ms);

  while (true) {
    /* If the bus is not busy then the operation can continue, note, the
       loop is exited in the locked state */
    if (!(i2c->SR & I2C_SR_BA_Msk))
      break;

    /* If the system time went outside the allowed window then a timeout
       condition is returned */
    if (!osalOsIsTimeWithinX(osalOsGetSystemTimeX(), start, end)) {
      return MSG_TIMEOUT;
    }

    /* Note: This function is called in the kernel lock state from the
       high level driver */
    osalThreadSleepS(1);
  }

  /* Clear FIFO */
  i2c->CR |= I2C_CR_FIFO_CLR_Msk;

  /* Set up TX or RX as required */
  if (i2cp->txbytes > 0) {
    tx_configure(i2cp);
    tx_fifo_fill(i2cp);
  } else if (i2cp->rxbytes > 0) {
    rx_configure(i2cp);
  }

  hold_update(i2cp);

  /* Set slave address to kick off transfer */
  i2c->ADDR = addr;

  /* Wait for completion or a timeout */
  msg_t msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);

  /* Try to recover from a timeout */
  if (msg == MSG_TIMEOUT) {
    i2c->CR |= I2C_CR_FIFO_CLR_Msk;
    i2c->CR &= ~I2C_CR_HOLD_Msk;
  }

  return msg;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Handles an I2C IRQ.
 *
 * @param[in] context     IRQ context, pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_irq_handler(void *context) {

  I2CDriver *i2cp = (I2CDriver *)context;
  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  /* Clear interrupt flags */
  uint32_t isr = i2c->ISR;
  i2c->ISR = isr;

  if (isr & ~(I2C_INT_COMP_Msk | I2C_INT_DATA_Msk)) {
    /* Error interrupt */
    i2cp->errors |= isr_errors_parse(isr);
    _i2c_wakeup_error_isr(i2cp);
  } else {
    /* COMP or DATA interrupt */

    if (((i2c->CR & I2C_CR_RW_Msk) >> I2C_CR_RW_Pos) == I2C_CR_RW_WRITE) {
      /* Write in progress */

      if (i2cp->txidx < i2cp->txbytes) {
        /* Continue filling FIFO */
        tx_fifo_fill(i2cp);
        hold_update(i2cp);
      } else if (isr & I2C_INT_COMP_Msk) {
        /* All bytes written to FIFO and COMP received - finished with TX */

        /* Start RX if necessary */
        if (i2cp->rxbytes > 0) {
          rx_configure(i2cp);
          hold_update(i2cp);

          /* Set slave address to kick off transfer */
          uint32_t addr = i2c->ADDR;
          i2c->ADDR = addr;

        } else {
          /* No RX - finished */
          _i2c_wakeup_isr(i2cp);
        }
      }
    } else {
      /* Read in progress */

      /* Drain RX FIFO */
      while ((i2cp->rxidx < i2cp->rxbytes) &&
             (i2c->SR & I2C_SR_RX_DV_Msk)) {
        i2cp->rxbuf[i2cp->rxidx++] = i2c->DATA;
      }

      hold_update(i2cp);

      if (i2cp->rxidx == i2cp->rxbytes) {
        /* All bytes read from FIFO - finished */
        _i2c_wakeup_isr(i2cp);
      }
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

#if ZYNQ7000_I2C_USE_I2C0 == TRUE
  i2cObjectInit(&I2CD1);
  I2CD1.i2c = I2C0;
  I2CD1.irq_id = IRQ_ID_I2C0;
  I2CD1.irq_priority = ZYNQ7000_I2C_I2C0_IRQ_PRIORITY;
  interrupts_init(&I2CD1);
#endif

#if ZYNQ7000_I2C_USE_I2C1 == TRUE
  i2cObjectInit(&I2CD2);
  I2CD2.i2c = I2C1;
  I2CD2.irq_id = IRQ_ID_I2C1;
  I2CD2.irq_priority = ZYNQ7000_I2C_I2C1_IRQ_PRIORITY;
  interrupts_init(&I2CD2);
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
    i2c_start(i2cp->i2c, i2cp->config->clk);
    gic_irq_enable(i2cp->irq_id);
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
    gic_irq_disable(i2cp->irq_id);
    i2c_stop(i2cp->i2c);
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

  i2cp->txbuf = 0;
  i2cp->txbytes = 0;
  i2cp->rxbuf = rxbuf;
  i2cp->rxbytes = rxbytes;
  i2cp->txidx = i2cp->rxidx = 0;

  return txn_execute(i2cp, addr, timeout);
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

  i2cp->txbuf = txbuf;
  i2cp->txbytes = txbytes;
  i2cp->rxbuf = rxbuf;
  i2cp->rxbytes = rxbytes;
  i2cp->txidx = i2cp->rxidx = 0;

  return txn_execute(i2cp, addr, timeout);
}

#endif /* HAL_USE_I2C == TRUE */

/** @} */
