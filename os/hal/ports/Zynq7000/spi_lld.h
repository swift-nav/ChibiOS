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
 * @file    spi_lld.h
 * @brief   Zynq7000 SPI subsystem low level driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPI_LLD_H_
#define _SPI_LLD_H_

#if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/** @brief SPI Mode (Polarity/Phase) */
#define SPI_MODE_FLAG_CPHA0   (0x00)
#define SPI_MODE_FLAG_CPHA1   (0x01)
#define SPI_MODE_FLAG_CPOL0   (0x00)
#define SPI_MODE_FLAG_CPOL1   (0x02)

#define SPI_CPOL0_CPHA0       (SPI_MODE_FLAG_CPOL0 | SPI_MODE_FLAG_CPHA0)
#define SPI_CPOL0_CPHA1       (SPI_MODE_FLAG_CPOL0 | SPI_MODE_FLAG_CPHA1)
#define SPI_CPOL1_CPHA0       (SPI_MODE_FLAG_CPOL1 | SPI_MODE_FLAG_CPHA0)
#define SPI_CPOL1_CPHA1       (SPI_MODE_FLAG_CPOL1 | SPI_MODE_FLAG_CPHA1)

#define SPI_MODE_0            SPI_CPOL0_CPHA0
#define SPI_MODE_1            SPI_CPOL0_CPHA1
#define SPI_MODE_2            SPI_CPOL1_CPHA0
#define SPI_MODE_3            SPI_CPOL1_CPHA1

/** @brief SPI clock divider */
#define SPI_CLK_DIV_4         0
#define SPI_CLK_DIV_8         1
#define SPI_CLK_DIV_16        2
#define SPI_CLK_DIV_32        3
#define SPI_CLK_DIV_64        4
#define SPI_CLK_DIV_128       5
#define SPI_CLK_DIV_256       6

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Zynq7000 configuration options
 * @{
 */
/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for SPI1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_SPI_USE_SPI1) || defined(__DOXYGEN__)
#define ZYNQ7000_SPI_USE_SPI1                  TRUE
#endif

/**
 * @brief   SPI2 driver enable switch.
 * @details If set to @p TRUE the support for SPI1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_SPI_USE_SPI2) || defined(__DOXYGEN__)
#define ZYNQ7000_SPI_USE_SPI2                  TRUE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an SPI driver.
 */
typedef struct SPIDriver SPIDriver;

/**
 * @brief   SPI notification callback type.
 *
 * @param[in] spip      pointer to the @p SPIDriver object triggering the
 *                      callback
 */
typedef void (*spicallback_t)(SPIDriver *spip);

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {
  /**
   * @brief Operation complete callback or @p NULL.
   */
  spicallback_t             end_cb;
  /* End of the mandatory fields.*/

  /**
   * @brief Polarity/Phase mode
   */
  uint8_t                   mode;

  /**
   * @brief Clock divider
   */
  uint8_t                   clkdiv;

  /**
   * @brief GPIO line used for Slave Select
   */
  ioline_t                  ssline;
} SPIConfig;

/**
 * @brief   Structure representing an SPI driver.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
struct SPIDriver {
  /**
   * @brief Driver state.
   */
  spistate_t                state;
  /**
   * @brief Current configuration data.
   */
  const SPIConfig           *config;
#if (SPI_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Waiting thread.
   */
  thread_reference_t        thread;
#endif
#if (SPI_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif
#if defined(SPI_DRIVER_EXT_FIELDS)
  SPI_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Pointer to hardware SPI registers.
   */
  void                      *spi;
  /**
   * @brief   Pointer to the buffer with data to send.
   */
  const uint8_t             *txbuf;
  /**
   * @brief   Current index in buffer when sending data.
   */
  size_t                    txidx;
  /**
   * @brief   Pointer to the buffer to put received data.
   */
  uint8_t                   *rxbuf;
  /**
   * @brief   Current index in buffer when receiving data.
   */
  size_t                    rxidx;
  /**
   * @brief   Number of bytes of data to send / receive.
   */
  size_t                    count;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (ZYNQ7000_SPI_USE_SPI1 == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif

#if (ZYNQ7000_SPI_USE_SPI2 == TRUE) && !defined(__DOXYGEN__)
extern SPIDriver SPID2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  void spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
  void spi_lld_ignore(SPIDriver *spip, size_t n);
  void spi_lld_exchange(SPIDriver *spip, size_t n,
                        const void *txbuf, void *rxbuf);
  void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
  uint8_t spi_lld_polled_exchange(SPIDriver *spip, uint8_t frame);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI == TRUE */

#endif /* _SPI_LLD_H_ */

/** @} */
