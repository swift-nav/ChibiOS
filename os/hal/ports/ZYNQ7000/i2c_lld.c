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
#if (ZYNQ7000_I2C_USE_I2C1 == TRUE) || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/
/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*****************************************************************************/
/**
*
* This function sets the serial clock rate for the IIC device. The device
* must be idle rather than busy transferring data before setting these device
* options.
*
* The data rate is set by values in the control register. The formula for
* determining the correct register values is:
* Fscl = Fpclk/(22 x (divisor_a+1) x (divisor_b+1))
* See the hardware data sheet for a full explanation of setting the serial
* clock rate.
*
* @param	InstancePtr is a pointer to the XIicPs instance.
* @param	FsclHz is the clock frequency in Hz. The two most common clock
*		rates are 100KHz and 400KHz.
*
* @return
*		- XST_SUCCESS if options are successfully set.
*		- XST_DEVICE_IS_STARTED if the device is currently transferring
*		data. The transfer must complete or be aborted before setting
*		options.
*		- XST_FAILURE if the Fscl frequency can not be set.
*
* @note		The clock can not be faster than the input clock divide by 22.
*
******************************************************************************/
static void set_clk(I2CDriver *InstancePtr, uint32_t FsclHz)
{
  uint32_t Div_a;
  uint32_t Div_b;
  uint32_t ActualFscl;
  uint32_t Temp;
  uint32_t TempLimit;
  uint32_t LastError;
  uint32_t BestError;
  uint32_t CurrentError;
  uint32_t ControlReg;
  uint32_t CalcDivA;
  uint32_t CalcDivB;
  uint32_t BestDivA = 0;
  uint32_t BestDivB = 0;
  uint32_t FsclHzVar = FsclHz;
  i2c_t *i2c = (i2c_t *)InstancePtr->i2c;


  /*
   * Assume Div_a is 0 and calculate (divisor_a+1) x (divisor_b+1).
   */
  Temp = (ZYNQ7000_I2C_I2C_REFCLK_FREQUENCY_Hz) / ((uint32_t)22U * FsclHzVar);

  /*
   * If frequency 400KHz is selected, 384.6KHz should be set.
   * If frequency 100KHz is selected, 90KHz should be set.
   * This is due to a hardware limitation.
   */
  if(FsclHzVar > 384600U) {
    FsclHzVar = 384600U;
  }

  if((FsclHzVar <= 100000U) && (FsclHzVar > 90000U)) {
    FsclHzVar = 90000U;
  }

  /*
   * TempLimit helps in iterating over the consecutive value of Temp to
   * find the closest clock rate achievable with divisors.
   * Iterate over the next value only if fractional part is involved.
   */
  TempLimit = (((ZYNQ7000_I2C_I2C_REFCLK_FREQUENCY_Hz) %
		((uint32_t)22 * FsclHzVar)) != 	(uint32_t)0x0U) ?
    Temp + (uint32_t)1U : Temp;
  BestError = FsclHzVar;

  BestDivA = 0U;
  BestDivB = 0U;
  for ( ; Temp <= TempLimit ; Temp++)
    {
      LastError = FsclHzVar;
      CalcDivA = 0U;
      CalcDivB = 0U;

      for (Div_b = 0U; Div_b < 64U; Div_b++) {

	Div_a = Temp / (Div_b + 1U);

	if (Div_a != 0U){
	  Div_a = Div_a - (uint32_t)1U;
	}
	if (Div_a > 3U){
	  continue;
	}
	ActualFscl = (ZYNQ7000_I2C_I2C_REFCLK_FREQUENCY_Hz) /
	  (22U * (Div_a + 1U) * (Div_b + 1U));

	if (ActualFscl > FsclHzVar){
	  CurrentError = (ActualFscl - FsclHzVar);}
	else{
	  CurrentError = (FsclHzVar - ActualFscl);}

	if (LastError > CurrentError) {
	  CalcDivA = Div_a;
	  CalcDivB = Div_b;
	  LastError = CurrentError;
	}
      }

      /*
       * Used to capture the best divisors.
       */
      if (LastError < BestError) {
	BestError = LastError;
	BestDivA = CalcDivA;
	BestDivB = CalcDivB;
      }
    }


  /*
   * Read the control register and mask the Divisors.
   */
  ControlReg = i2c->CR;
  ControlReg &= ~((uint32_t)I2C_CR_DIV_A_Msk | (uint32_t)I2C_CR_DIV_B_Msk);
  ControlReg |= (BestDivA << I2C_CR_DIV_A_Pos) |
    (BestDivB << I2C_CR_DIV_B_Pos);

  i2c->CR = ControlReg;
}

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


static void recieve_data(I2CDriver *icip) {
  uint8_t bytes_left;
  i2c_t *i2c = (i2c_t*)icip->i2c;
  osalDbgAssert(icip->rxbuf != NULL, "NULL buffer");
  while (i2c->SR & I2C_SR_RXDV_Msk) {
    osalDbgAssert(icip->rxidx < icip->count, "Too many bytes Rx");
    icip->rxbuf[icip->rxidx] = i2c->DATA;
    icip->rxidx++;
  }
  bytes_left = icip->count - icip->rxidx;
  if (bytes_left <= I2C_FIFO_DEPTH) {
     i2c->CR = i2c->CR & ~I2C_CR_HOLD_Msk;
  }
  i2c->TRANS_SIZE=bytes_left;
  if (bytes_left == 0) {
    i2c->IDR = I2C_IXR_ALL_INTR_Msk;
    icip->state = I2C_READY;
  }
}

static void send_data(I2CDriver *icip) {
  uint8_t AvailBytes;
  int32_t LoopCnt;
  int32_t NumBytesToSend;

  i2c_t *i2c = (i2c_t*)icip->i2c;

  /*
   * Determine number of bytes to write to FIFO.
   */
  AvailBytes = I2C_FIFO_DEPTH - i2c->TRANS_SIZE;

  NumBytesToSend = icip->count - icip->txidx;

  if (NumBytesToSend == 0) {
    i2c->IDR = I2C_IXR_ALL_INTR_Msk;
    icip->state = I2C_READY;
    return;
  }

  if (NumBytesToSend > (int32_t)AvailBytes) {
    NumBytesToSend = (int32_t)AvailBytes;
  }

  /*
   * Fill FIFO with amount determined above.
   */
  for (LoopCnt = 0; LoopCnt < NumBytesToSend; LoopCnt++) {
    i2c->DATA = icip->txbuf[icip->txidx];
    icip->txidx++;
  }

  if (icip->txidx == icip->count) {
    i2c->CR = i2c->CR & ~I2C_CR_HOLD_Msk;
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


  /* Read and clear interrupt flags */
  uint32_t isr = i2c->ISR;
  i2c->ISR = isr;

  /*
   * Use the Mask register AND with the Interrupt Status register so
   * disabled interrupts are not processed.
   */
  isr &= ~i2c->IMR;

  if (isr & I2C_IXR_COMP_Msk && icip->txbuf != NULL ) {
    send_data(icip);
  } else if (i2c->SR & I2C_SR_RXDV_Msk) {
    recieve_data(icip);
  } else if (isr & ~(I2C_IXR_COMP_Msk | I2C_IXR_DATA_Msk)) {
    osalDbgAssert(isr == 0, "I2C error IRQ");
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
#if ZYNQ7000_I2C_USE_I2C1 == TRUE
  i2cObjectInit(&I2CD1);
  I2CD1.i2c = I2C1;
  I2CD1.irq_id = IRQ_ID_I2C1;
  I2CD1.irq_priority = ZYNQ7000_I2C_I2C1_IRQ_PRIORITY;
  interrupts_init(&I2CD1);
  I2CD1.state = I2C_STOP;
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
#if ZYNQ7000_I2C_USE_I2C1 == TRUE
    if (&I2CD1 == i2cp) {
    	I2C1->CR = 0xF;
    	set_clk(i2cp,400000U);
    }
    i2cp->state = I2C_READY;
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
#if ZYNQ7000_I2C_USE_I2C1 == TRUE
    if (&I2CD1 == i2cp) {
    	I2C1->CR = 0;
    }
#endif
    i2cp->state = I2C_STOP;
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

  if (i2cp->state != I2C_READY) {
	  i2cp->errors = I2C_TIMEOUT;
	  return MSG_TIMEOUT;
  }

  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  i2cp->rxidx = 0;
  i2cp->count = rxbytes;
  i2cp->rxbuf = rxbuf;
  i2cp->txbuf = NULL;
  i2cp->state = I2C_ACTIVE_RX;
  i2cp->errors = I2C_NO_ERROR;

  /** Set Direction of transfer as write and Clear the FIFO's **/
  i2c->CR |= I2C_CR_CLR_FIFO_Msk;
  i2c->CR |= I2C_CR_RD_WR_Msk;
  /** Clear Interrupts **/
  uint32_t isr = i2c->ISR;
  i2c->ISR = isr;
  /** Enable Timeout, NACK, Rx overflow, Arbitration lost, DATA, Completion interrupts **/
  i2c->IER = I2C_IXR_ALL_INTR_Msk;

  //If the data is too much for the FIFO enable HOLD
  if (rxbytes > I2C_FIFO_DEPTH) {
	  i2c->CR |= I2C_CR_HOLD_Msk;
  }
  i2c->TRANS_SIZE = rxbytes;
  i2c->ADDR = addr;

  /** TODO: add mutex logic **/
  /** TODO: add timeout logic **/
  (void)timeout;
   while (i2cp->state == I2C_ACTIVE_RX) {
    //TODO: fix since ISR not being called
    i2c_irq_handler(i2cp);
    chThdSleepMilliseconds(1);
  }

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
  //TODO: receive during transmit not supported
  osalDbgAssert(rxbuf == NULL, "receive during transmit not supported");
  (void)rxbytes;
  if (i2cp->state != I2C_READY) {
    i2cp->errors = I2C_TIMEOUT;
    return MSG_TIMEOUT;
  }

  i2c_t *i2c = (i2c_t *)i2cp->i2c;

  i2cp->txidx = 0;
  i2cp->count = txbytes;
  i2cp->rxbuf = NULL;
  i2cp->txbuf = txbuf;
  i2cp->state = I2C_ACTIVE_TX;
  i2cp->errors = I2C_NO_ERROR;

  /** Set Direction of transfer as write and Clear the FIFO's **/
  i2c->CR |= I2C_CR_CLR_FIFO_Msk;
  i2c->CR &= ~I2C_CR_RD_WR_Msk;
  /** Clear Interrupts **/
  uint32_t isr = i2c->ISR;
  i2c->ISR = isr;
  /** Enable Timeout, NACK, Rx overflow, Arbitration lost, DATA, Completion interrupts **/
  //i2c->IER = I2C_IXR_ALL_INTR_Msk;
  i2c->IER = 1;
  
  //If the data is too much for the FIFO enable HOLD
  if (txbytes > I2C_FIFO_DEPTH) {
    i2c->CR |= I2C_CR_HOLD_Msk;
  }
  send_data(i2cp);
  i2c->ADDR = addr;

  /** TODO: add mutex logic **/
  /** TODO: add timeout logic **/
  (void)timeout;
  while (i2cp->state == I2C_ACTIVE_TX) {
    //TODO: fix since ISR not being called
    i2c_irq_handler(i2cp);
    chThdSleepMilliseconds(1);
   }
  return MSG_OK;
}

#endif /* HAL_USE_I2C == TRUE */

/** @} */
