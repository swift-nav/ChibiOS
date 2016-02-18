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
 * @file    ZYNQ7000/spi_lld.c
 * @brief   ZYNQ7000 SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "gic.h"
#include "zynq7000.h"
#include "hal.h"


#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

#define FIFO_SIZE         128
#define FIFO_RXWM_MAX     (FIFO_SIZE - 1)
#define FIFO_HEADROOM     8 /* Number of bytes before FIFO exhaustion to
                               generate interrupt */

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static void spi_irq_handler(void *context);

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SPID1 driver identifier.
 */
#if (ZYNQ7000_SPI_USE_SPI0 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/**
 * @brief   SPID2 driver identifier.
 */
#if (ZYNQ7000_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint32_t bauddiv_get(uint8_t clkdiv) {

  switch(clkdiv) {
  case SPI_CLK_DIV_4:
    return SPI_CR_BAUDDIV_4;
  case SPI_CLK_DIV_8:
    return SPI_CR_BAUDDIV_8;
  case SPI_CLK_DIV_16:
    return SPI_CR_BAUDDIV_16;
  case SPI_CLK_DIV_32:
    return SPI_CR_BAUDDIV_32;
  case SPI_CLK_DIV_64:
    return SPI_CR_BAUDDIV_64;
  case SPI_CLK_DIV_128:
    return SPI_CR_BAUDDIV_128;
  case SPI_CLK_DIV_256:
    return SPI_CR_BAUDDIV_256;
  default:
    osalDbgAssert(0, "invalid clkdiv");
    return SPI_CR_BAUDDIV_256;
  }
}

static void interrupts_init(SPIDriver *spip) {
  gic_handler_register(spip->irq_id, spi_irq_handler, spip);
  gic_irq_sensitivity_set(spip->irq_id, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(spip->irq_id, spip->irq_priority);
}

static void spi_start(spi_t *spi, uint8_t mode, uint8_t clkdiv) {

  /* Disable SPI */
  spi->ER = 0;

  /* Disable interrupts */
  spi->IDR = SPI_INT_RXOVER_Msk | SPI_INT_MODEFAIL_Msk | SPI_INT_TXTRIG_Msk |
             SPI_INT_TXFULL_Msk | SPI_INT_RXTRIG_Msk | SPI_INT_RXFULL_Msk |
             SPI_INT_TXUNDER_Msk;

  /* Configure mode */
  spi->CR = (1 << SPI_CR_MSTREN_Pos) |
            ((mode & SPI_MODE_FLAG_CPOL1) ? 1 : 0) << SPI_CR_CPOL_Pos |
            ((mode & SPI_MODE_FLAG_CPHA1) ? 1 : 0) << SPI_CR_CPHA_Pos |
            (bauddiv_get(clkdiv) << SPI_CR_BAUDDIV_Pos) |
            (SPI_CR_REFCLK_SPIREFCLK << SPI_CR_REFCLK_Pos) |
            (1 << SPI_CR_MANCS_Pos);

  /* Enable SPI */
  spi->ER = (1 << SPI_ER_ENABLE_Pos);

  /* Clear RX FIFO */
  spi->RXWM = 1;
  while (spi->ISR & SPI_INT_RXTRIG_Msk) {
    spi->RXD;
  }
}

static void spi_stop(spi_t *spi) {

  /* Disable SPI */
  spi->ER = 0;
}

static void txn_begin(SPIDriver *spip) {

  spi_t *spi = (spi_t *)spip->spi;

  /* If the full transaction can fit in the FIFO, interrupt after all bytes
  * are received. Otherwise interrupt FIFO_HEADROOM bytes before filling the
  * RX FIFO */

  /* Set RX watermark level */
  if (spip->count <= FIFO_RXWM_MAX) {
    spi->RXWM = spip->count;
  } else {
    spi->RXWM = FIFO_RXWM_MAX - FIFO_HEADROOM;
  }

  /* Write to TX FIFO */
  uint32_t bytes_to_write = MIN(spip->count, FIFO_RXWM_MAX);
  while (spip->txidx < bytes_to_write) {
    if (spip->txbuf) {
      spi->TXD = spip->txbuf[spip->txidx++];
    } else {
      spi->TXD = 0;
      spip->txidx++;
    }
  }

  /* Enable RXTRIG interrupt */
  spi->IER = SPI_INT_RXTRIG_Msk;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

static void spi_irq_handler(void *context) {

  SPIDriver *spip = (SPIDriver *)context;
  spi_t *spi = (spi_t *)spip->spi;
  uint32_t i;

  /* Read and clear interrupt flags */
  uint32_t isr = spi->ISR;
  spi->ISR = isr;

  if (isr & SPI_INT_RXTRIG_Msk) {
    /* If there are no more bytes to send, then this must be the final RX.
     * Read out FIFO_RXWM_MAX - FIFO_HEADROOM bytes each interrupt until the
     * final interrupt. */
    bool final_rx = (spip->txidx >= spip->count);
    uint32_t bytes_to_read = final_rx ? (spip->count - spip->rxidx) :
                                        (FIFO_RXWM_MAX - FIFO_HEADROOM);

    /* Read from RX FIFO */
    for (i = 0; i < bytes_to_read; i++) {
      uint8_t rx = spi->RXD;
      if (spip->rxbuf) {
        spip->rxbuf[spip->rxidx] = rx;
      }
      spip->rxidx++;
    }

    if (!final_rx) {
      /* Up until the final RX (when there are no bytes to send), RXWM will be
       * set to FIFO_RXWM_MAX - FIFO_HEADROOM, so there will be FIFO_RXWM_MAX -
       * FIFO_HEADROOM bytes free in the TX FIFO. If the remaining TX data
       * can fit into that space, send everything and interrupt when all bytes
       * are received. Otherwise send FIFO_RXWM_MAX - FIFO_HEADROOM bytes and
       * leave RXWM unchanged.
       */
      uint32_t bytes_to_write = spip->count - spip->txidx;
      if (bytes_to_write <= FIFO_RXWM_MAX - FIFO_HEADROOM) {
        spi->RXWM = bytes_to_write + FIFO_HEADROOM;
      } else {
        bytes_to_write = FIFO_RXWM_MAX - FIFO_HEADROOM;
      }

      /* Write to TX FIFO */
      for (i = 0; i < bytes_to_write; i++) {
        if (spip->txbuf) {
          spi->TXD = spip->txbuf[spip->txidx++];
        } else {
          spi->TXD = 0;
          spip->txidx++;
        }
      }
    }

    /* Clean up if finished */
    if (final_rx) {
      osalDbgAssert(spip->rxidx == spip->count, "invalid rxidx");
      osalDbgAssert(spip->txidx == spip->count, "invalid txidx");

      /* Disable RXTRIG interrupt */
      spi->IDR = SPI_INT_RXTRIG_Msk;
      _spi_isr_code(spip);
    }
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if ZYNQ7000_SPI_USE_SPI0 == TRUE
  spiObjectInit(&SPID1);
  SPID1.spi = SPI0;
  SPID1.irq_id = IRQ_ID_SPI0;
  SPID1.irq_priority = ZYNQ7000_SPI_SPI0_IRQ_PRIORITY;
  interrupts_init(&SPID1);
#endif

#if ZYNQ7000_SPI_USE_SPI1 == TRUE
  spiObjectInit(&SPID2);
  SPID2.spi = SPI1;
  SPID2.irq_id = IRQ_ID_SPI1;
  SPID2.irq_priority = ZYNQ7000_SPI_SPI1_IRQ_PRIORITY;
  interrupts_init(&SPID2);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  if (spip->state == SPI_STOP) {
    spi_start(spip->spi, spip->config->mode, spip->config->clkdiv);
    gic_irq_enable(spip->irq_id);
  }
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  if (spip->state == SPI_READY) {
    gic_irq_disable(spip->irq_id);
    spi_stop(spip->spi);
  }
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  palClearLine(spip->config->ssline);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  palSetLine(spip->config->ssline);
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  spip->txbuf = spip->rxbuf = 0;
  spip->txidx = spip->rxidx = 0;
  spip->count = n;

  txn_begin(spip);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  spip->txbuf = txbuf;
  spip->rxbuf = rxbuf;
  spip->txidx = spip->rxidx = 0;
  spip->count = n;

  txn_begin(spip);
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  spip->txbuf = txbuf;
  spip->rxbuf = 0;
  spip->txidx = spip->rxidx = 0;
  spip->count = n;

  txn_begin(spip);
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  spip->txbuf = 0;
  spip->rxbuf = rxbuf;
  spip->txidx = spip->rxidx = 0;
  spip->count = n;

  txn_begin(spip);
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint8_t spi_lld_polled_exchange(SPIDriver *spip, uint8_t frame) {

  spi_t *spi = (spi_t *)spip->spi;

  /* Set RX watermark level */
  spi->RXWM = 1;

  spi->TXD = frame;
  while (!(spi->ISR & SPI_INT_RXTRIG_Msk)) ;
  uint8_t rx = spi->RXD;

  return rx;
}

#endif /* HAL_USE_SPI == TRUE */

/** @} */
