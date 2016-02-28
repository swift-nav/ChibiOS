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
 * @file    ZYNQ7000/serial_lld.c
 * @brief   ZYNQ7000 serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "gic.h"
#include "zynq7000.h"
#include "hal.h"


#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

#define RX_WATERMARK_BYTES  32 /* 1-63 */
#define RX_TIMEOUT_BYTES     8 /* 1-128 */

#define CD_MIN 1
#define CD_MAX 65535
#define BD_MIN 4
#define BD_MAX 255

#define DIV_ROUND_UNSIGNED(num, div) (((num) + (div) / 2U) / (div))
#define ABS(n) ((n) > 0 ? (n) : -(n))

static void uart_irq_handler(void *context);

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

static void onotify(io_queue_t *qp, uart_t *uart, irq_id_t irq_id) {
  (void)qp;
  if (uart->SR & UART_SR_TXEMPTY_Msk) {
    gic_irq_pending_set(irq_id);
  }
}

#if (ZYNQ7000_SERIAL_USE_UART0 == TRUE) || defined(__DOXYGEN__)
static void onotify1(io_queue_t *qp) {
  onotify(qp, UART0, IRQ_ID_UART0);
}
#endif

#if (ZYNQ7000_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
static void onotify2(io_queue_t *qp) {
  onotify(qp, UART1, IRQ_ID_UART1);
}
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SD1 driver identifier.*/
#if (ZYNQ7000_SERIAL_USE_UART0 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief SD2 driver identifier.*/
#if (ZYNQ7000_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Sets the error flags for a serial driver
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] isr       value of the UART ISR register
 *
 * @notapi
 */
static void error_set(SerialDriver *sdp, uint32_t isr) {
  eventflags_t sts = 0;

  if (isr & UART_INT_RXOVER_Msk)
    sts |= SD_OVERRUN_ERROR;
  if (isr & UART_INT_RXFRAMING_Msk)
    sts |= SD_FRAMING_ERROR;
  if (isr & UART_INT_RXPARITY_Msk)
    sts |= SD_PARITY_ERROR;

  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

/**
 * @brief   Computes the optimal dividers to achieve the specified baud rate.
 *
 * @param[in] baudrate      target baud rate (Hz)
 * @param[out] rbd          pointer to the output BD value
 * @param[out] rcd          pointer to the output CD value
 *
 * @notapi
 */
static void baud_calc(uint32_t baudrate, uint32_t *rbd, uint32_t *rcd) {
  uint32_t ref_clk = ZYNQ7000_SERIAL_UART_REFCLK_FREQUENCY_Hz;
  uint32_t best_error = UINT32_MAX;
  uint32_t best_bd = 0;
  uint32_t best_cd = 0;

  uint32_t bd;
  for (bd = BD_MIN; bd <= BD_MAX; bd++) {
    uint32_t cd = DIV_ROUND_UNSIGNED(ref_clk, (bd + 1) * baudrate);

    /* Make sure cd is in range */
    if (cd < CD_MIN)
      cd = CD_MIN;
    else if (cd > CD_MAX)
      cd = CD_MAX;

    uint32_t baudrate_actual = ref_clk / ((bd + 1) * cd);
    uint32_t error = baudrate > baudrate_actual ? (baudrate - baudrate_actual) :
                                                  (baudrate_actual - baudrate);
    if (error < best_error) {
      best_bd = bd;
      best_cd = cd;
      best_error = error;
    }
  }

  *rbd = best_bd;
  *rcd = best_cd;
}

/**
 * @brief   Initializes interrupts for the UART peripheral.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
static void interrupts_init(SerialDriver *sdp) {
  gic_handler_register(sdp->irq_id, uart_irq_handler, sdp);
  gic_irq_sensitivity_set(sdp->irq_id, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(sdp->irq_id, sdp->irq_priority);
}

/**
 * @brief   Starts the UART peripheral.
 *
 * @param[in] uart          pointer to the hardware UART registers
 * @param[in] baudrate      desired baud rate (Hz)
 *
 * @notapi
 */
static void uart_start(uart_t *uart, uint32_t baudrate) {

  /* Disable RX and TX paths */
  uart->CR = UART_CR_RXDIS_Msk | UART_CR_TXDIS_Msk;

  /* Configure baud rate */
  uint32_t bd;
  uint32_t cd;
  baud_calc(baudrate, &bd, &cd);
  uart->BAUDDIV = bd;
  uart->BAUDGEN = cd;

  /* Reset RX and TX paths */
  uart->CR |= UART_CR_RXRST_Msk | UART_CR_TXRST_Msk;

  /* Clear any pending interrupts */
  uint32_t isr = uart->ISR;
  uart->ISR = isr;

  /* Configure mode */
  uart->MR = (UART_MR_CLKSEL_REFCLK << UART_MR_CLKSEL_Pos) |
             (UART_MR_CHRL_8BIT << UART_MR_CHRL_Pos) |
             (UART_MR_PAR_NONE << UART_MR_PAR_Pos) |
             (UART_MR_NBSTOP_ONE << UART_MR_NBSTOP_Pos) |
             (UART_MR_CHMODE_NORMAL << UART_MR_CHMODE_Pos);

  /* Configure FIFO watermarks and RX timeout */
  uart->RXWM = RX_WATERMARK_BYTES;
  uart->RXTOUT = (8 * RX_TIMEOUT_BYTES / 4) - 1;
  uart->TXWM = 0; /* TX watermark is not used */

  /* Enable RX and TX paths */
  uart->CR = UART_CR_RXEN_Msk | UART_CR_TXEN_Msk;

  /* Enable interrupts */
  uart->IER = UART_INT_RXTRIG_Msk |
              UART_INT_RXTOUT_Msk |
              UART_INT_TXEMPTY_Msk;
}

/**
 * @brief   Stops the UART peripheral.
 *
 * @param[in] uart          pointer to the hardware UART registers
 *
 * @notapi
 */
static void uart_stop(uart_t *uart) {

  /* Disable RX and TX paths */
  uart->CR = UART_CR_RXDIS_Msk | UART_CR_TXDIS_Msk;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Handles a UART IRQ.
 *
 * @param[in] context     IRQ context, pointer to a @p SerialDriver object
 *
 * @notapi
 */
static void uart_irq_handler(void *context) {

  SerialDriver *sdp = (SerialDriver *)context;
  uart_t *uart = (uart_t *)sdp->uart;

  /* Read and clear interrupt flags */
  uint32_t isr = uart->ISR;
  uart->ISR = isr;

  /* Check for errors */
  if (isr & (UART_INT_RXOVER_Msk |
             UART_INT_RXFRAMING_Msk |
             UART_INT_RXPARITY_Msk)) {
    error_set(sdp, isr);
  }

  /* Receive from RX FIFO */
  osalSysLockFromISR();
  while (!(uart->SR & UART_SR_RXEMPTY_Msk)) {
    msg_t b = uart->FIFO;
    if (iqIsEmptyI(&sdp->iqueue)) {
      chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
    }
    if (iqPutI(&sdp->iqueue, b) < Q_OK) {
      chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
      break;
    }
  }
  osalSysUnlockFromISR();

  /* Refill TX FIFO */
  osalSysLockFromISR();
  while (!(uart->SR & UART_SR_TXFULL_Msk)) {
    msg_t b = oqGetI(&sdp->oqueue);
    if (b < Q_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      break;
    } else {
      uart->FIFO = b;
    }
  }
  osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if ZYNQ7000_SERIAL_USE_UART0 == TRUE
  sdObjectInit(&SD1, NULL, onotify1);
  SD1.uart = UART0;
  SD1.irq_id = IRQ_ID_UART0;
  SD1.irq_priority = ZYNQ7000_SERIAL_UART0_IRQ_PRIORITY;
  interrupts_init(&SD1);
#endif

#if ZYNQ7000_SERIAL_USE_UART1 == TRUE
  sdObjectInit(&SD2, NULL, onotify2);
  SD2.uart = UART1;
  SD2.irq_id = IRQ_ID_UART1;
  SD2.irq_priority = ZYNQ7000_SERIAL_UART1_IRQ_PRIORITY;
  interrupts_init(&SD2);
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL) {
    config = &default_config;
  }

  if (sdp->state == SD_STOP) {
    uart_start(sdp->uart, config->speed);
    gic_irq_enable(sdp->irq_id);
  }
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    gic_irq_disable(sdp->irq_id);
    uart_stop(sdp->uart);
  }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
