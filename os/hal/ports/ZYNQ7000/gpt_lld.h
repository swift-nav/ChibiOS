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
 * @file    ZYNQ7000/gpt_lld.h
 * @brief   ZYNQ7000 GPT subsystem low level driver header.
 *
 * @addtogroup GPT
 * @{
 */

#ifndef _GPT_LLD_H_
#define _GPT_LLD_H_

#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   GPTD1 driver enable switch.
 * @details If set to @p TRUE the support for GPTD1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_GPT_USE_TTC0_0) || defined(__DOXYGEN__)
#define ZYNQ7000_GPT_USE_TTC0_0             FALSE
#endif

/**
 * @brief   GPTD2 driver enable switch.
 * @details If set to @p TRUE the support for GPTD2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_GPT_USE_TTC0_1) || defined(__DOXYGEN__)
#define ZYNQ7000_GPT_USE_TTC0_1             FALSE
#endif

/**
 * @brief   GPTD3 driver enable switch.
 * @details If set to @p TRUE the support for GPTD3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_GPT_USE_TTC0_2) || defined(__DOXYGEN__)
#define ZYNQ7000_GPT_USE_TTC0_2             FALSE
#endif

/**
 * @brief   GPTD4 driver enable switch.
 * @details If set to @p TRUE the support for GPTD4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_GPT_USE_TTC1_0) || defined(__DOXYGEN__)
#define ZYNQ7000_GPT_USE_TTC1_0             FALSE
#endif

/**
 * @brief   GPTD5 driver enable switch.
 * @details If set to @p TRUE the support for GPTD5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_GPT_USE_TTC1_1) || defined(__DOXYGEN__)
#define ZYNQ7000_GPT_USE_TTC1_1             FALSE
#endif

/**
 * @brief   GPTD6 driver enable switch.
 * @details If set to @p TRUE the support for GPTD6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(ZYNQ7000_GPT_USE_TTC1_2) || defined(__DOXYGEN__)
#define ZYNQ7000_GPT_USE_TTC1_2             FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GPT frequency type.
 */
typedef uint32_t gptfreq_t;

/**
 * @brief   GPT counter type.
 */
typedef uint16_t gptcnt_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  gptfreq_t                 frequency;
  /**
   * @brief   Timer callback pointer.
   * @note    This callback is invoked on GPT counter events.
   */
  gptcallback_t             callback;
  /* End of the mandatory fields.*/
} GPTConfig;

/**
 * @brief   Structure representing a GPT driver.
 */
struct GPTDriver {
  /**
   * @brief Driver state.
   */
  gptstate_t                state;
  /**
   * @brief Current configuration data.
   */
  const GPTConfig           *config;
#if defined(GPT_DRIVER_EXT_FIELDS)
  GPT_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Pointer to hardware TTC registers.
   */
  void                      *ttc;
  /**
   * @brief   Timer/counter index within TTC module
   */
  uint8_t                   tc_index;
  /**
   * @brief   IRQ ID
   */
  uint8_t                   irq_id;
  /**
   * @brief   IRQ priority
   */
  uint8_t                   irq_priority;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (ZYNQ7000_GPT_USE_TTC0_0 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD1;
#endif

#if (ZYNQ7000_GPT_USE_TTC0_1 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD2;
#endif

#if (ZYNQ7000_GPT_USE_TTC0_2 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD3;
#endif

#if (ZYNQ7000_GPT_USE_TTC1_0 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD4;
#endif

#if (ZYNQ7000_GPT_USE_TTC1_1 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD5;
#endif

#if (ZYNQ7000_GPT_USE_TTC1_2 == TRUE) && !defined(__DOXYGEN__)
extern GPTDriver GPTD6;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void gpt_lld_init(void);
  void gpt_lld_start(GPTDriver *gptp);
  void gpt_lld_stop(GPTDriver *gptp);
  void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval);
  void gpt_lld_stop_timer(GPTDriver *gptp);
  void gpt_lld_change_interval(GPTDriver *gptp, gptcnt_t interval);
  gptcnt_t gpt_lld_get_interval(GPTDriver *gptp);
  gptcnt_t gpt_lld_get_counter(GPTDriver *gptp);
  void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_GPT == TRUE */

#endif /* _GPT_LLD_H_ */

/** @} */
