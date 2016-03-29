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
 * @file    ZYNQ7000/gpt_lld.c
 * @brief   ZYNQ7000 GPT subsystem low level driver source.
 *
 * @addtogroup GPT
 * @{
 */

#include "gic.h"
#include "zynq7000.h"
#include "hal.h"


#if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)

static void ttc_irq_handler(void *context);

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   GPTD1 driver identifier.
 */
#if (ZYNQ7000_GPT_USE_TTC0_0 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD1;
#endif

/**
 * @brief   GPTD2 driver identifier.
 */
#if (ZYNQ7000_GPT_USE_TTC0_1 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD2;
#endif

/**
 * @brief   GPTD3 driver identifier.
 */
#if (ZYNQ7000_GPT_USE_TTC0_2 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD3;
#endif

/**
 * @brief   GPTD4 driver identifier.
 */
#if (ZYNQ7000_GPT_USE_TTC1_0 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD4;
#endif

/**
 * @brief   GPTD5 driver identifier.
 */
#if (ZYNQ7000_GPT_USE_TTC1_1 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD5;
#endif

/**
 * @brief   GPTD6 driver identifier.
 */
#if (ZYNQ7000_GPT_USE_TTC1_2 == TRUE) || defined(__DOXYGEN__)
GPTDriver GPTD6;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Computes the prescaler required to achieve the specified frequency.
 * @note    This function asserts that the target frequency equals
 *          ZYNQ7000_CPU_1x_FREQUENCY_Hz / 2^N.
 *
 *
 * @param[in] frequency_hz    target frequency (Hz)
 * @return                    the computed prescaler value [0, 16],
 *                            corresponding to divide by 2^N
 *
 * @notapi
 */
static uint32_t prescaler_get(uint32_t frequency_hz) {

  uint32_t prescaler = 0;
  uint32_t prescaled_freq_hz = ZYNQ7000_CPU_1x_FREQUENCY_Hz;

  /* Compute prescaler [0, 16] corresponding to divide by 2^N */
  while ((prescaled_freq_hz > frequency_hz) &&
         (prescaler < 16)) {
    prescaler++;
    prescaled_freq_hz /= 2;
  }

  /* Provided frequency must be CPU_1x_FREQUENCY_Hz / 2^N */
  osalDbgAssert(prescaled_freq_hz == frequency_hz,
                "invalid frequency");

  return prescaler;
}

/**
 * @brief   Initializes interrupts for the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
static void interrupts_init(GPTDriver *gptp) {
  gic_handler_register(gptp->irq_id, ttc_irq_handler, gptp);
  gic_irq_sensitivity_set(gptp->irq_id, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(gptp->irq_id, gptp->irq_priority);
}

/**
 * @brief   Initializes a timer/counter within the TTC module.
 *
 * @param[in] ttc       pointer to the hardware TTC registers
 * @param[in] tc_index  index of the timer/counter within the TTC module
 *
 * @notapi
 */
static void timer_init(ttc_t *ttc, uint8_t tc_index) {

  /* Disable counter */
  ttc->CNTCTRL[tc_index] = (1 << TTC_CNTCTRL_DISABLE_Pos);

  /* Disable and clear interrupts */
  ttc->IEN[tc_index] = 0;
  ttc->ISR[tc_index];
}

/**
 * @brief   Configures a timer/counter within the TTC module.
 *
 * @param[in] ttc           pointer to the hardware TTC registers
 * @param[in] tc_index      index of the timer/counter within the TTC module
 * @param[in] frequency_hz  timer frequency (Hz)
 *
 * @notapi
 */
static void timer_configure(ttc_t *ttc, uint8_t tc_index,
                            uint32_t frequency_hz) {

  /* Configure prescaler */
  uint32_t prescaler = prescaler_get(frequency_hz);
  ttc->CLKCTRL[tc_index] =
      (TTC_CLKCTRL_SRC_PCLK << TTC_CLKCTRL_SRC_Pos) |
      ((prescaler > 0 ? (prescaler - 1) : 0) << TTC_CLKCTRL_PSVAL_Pos) |
      ((prescaler > 0 ? 1 : 0) << TTC_CLKCTRL_PSEN_Pos);

  /* Enable interval interrupt */
  ttc->IEN[tc_index] = (1 << TTC_INT_INTERVAL_Pos);
}

/**
 * @brief   Disables a timer/counter within the TTC module.
 *
 * @param[in] ttc       pointer to the hardware TTC registers
 * @param[in] tc_index  index of the timer/counter within the TTC module
 *
 * @notapi
 */
static void timer_disable(ttc_t *ttc, uint8_t tc_index) {

  /* Disable counter */
  ttc->CNTCTRL[tc_index] = (1 << TTC_CNTCTRL_DISABLE_Pos);

  /* Disable and clear interrupts */
  ttc->IEN[tc_index] = 0;
  ttc->ISR[tc_index];
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Handles a TTC IRQ.
 *
 * @param[in] context     IRQ context, pointer to the @p GPTDriver object
 *
 * @notapi
 */
static void ttc_irq_handler(void *context) {

  GPTDriver *gptp = (GPTDriver *)context;
  ttc_t *ttc = (ttc_t *)gptp->ttc;
  uint8_t tc_index = gptp->tc_index;

  uint32_t isr = ttc->ISR[tc_index];

  if (isr & TTC_INT_INTERVAL_Msk) {
    if (gptp->state == GPT_ONESHOT) {
      gptp->state = GPT_READY;                /* Back in GPT_READY state.     */
      gpt_lld_stop_timer(gptp);               /* Timer automatically stopped. */
    }
    gptp->config->callback(gptp);
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level GPT driver initialization.
 *
 * @notapi
 */
void gpt_lld_init(void) {

#if ZYNQ7000_GPT_USE_TTC0_0 == TRUE
  gptObjectInit(&GPTD1);
  GPTD1.ttc = TTC0;
  GPTD1.tc_index = 0;
  GPTD1.irq_id = IRQ_ID_TTC0_0;
  GPTD1.irq_priority = ZYNQ7000_GPT_TTC0_0_IRQ_PRIORITY;
  timer_init(GPTD1.ttc, GPTD1.tc_index);
  interrupts_init(&GPTD1);
#endif

#if ZYNQ7000_GPT_USE_TTC0_1 == TRUE
  gptObjectInit(&GPTD2);
  GPTD2.ttc = TTC0;
  GPTD2.tc_index = 1;
  GPTD2.irq_id = IRQ_ID_TTC0_1;
  GPTD2.irq_priority = ZYNQ7000_GPT_TTC0_1_IRQ_PRIORITY;
  timer_init(GPTD2.ttc, GPTD2.tc_index);
  interrupts_init(&GPTD2);
#endif

#if ZYNQ7000_GPT_USE_TTC0_2 == TRUE
  gptObjectInit(&GPTD3);
  GPTD3.ttc = TTC0;
  GPTD3.tc_index = 2;
  GPTD3.irq_id = IRQ_ID_TTC0_2;
  GPTD3.irq_priority = ZYNQ7000_GPT_TTC0_2_IRQ_PRIORITY;
  timer_init(GPTD3.ttc, GPTD3.tc_index);
  interrupts_init(&GPTD3);
#endif

#if ZYNQ7000_GPT_USE_TTC1_0 == TRUE
  gptObjectInit(&GPTD4);
  GPTD4.ttc = TTC1;
  GPTD4.tc_index = 0;
  GPTD4.irq_id = IRQ_ID_TTC1_0;
  GPTD4.irq_priority = ZYNQ7000_GPT_TTC1_0_IRQ_PRIORITY;
  timer_init(GPTD4.ttc, GPTD4.tc_index);
  interrupts_init(&GPTD4);
#endif

#if ZYNQ7000_GPT_USE_TTC1_1 == TRUE
  gptObjectInit(&GPTD5);
  GPTD5.ttc = TTC1;
  GPTD5.tc_index = 1;
  GPTD5.irq_id = IRQ_ID_TTC1_1;
  GPTD5.irq_priority = ZYNQ7000_GPT_TTC1_1_IRQ_PRIORITY;
  timer_init(GPTD5.ttc, GPTD5.tc_index);
  interrupts_init(&GPTD5);
#endif

#if ZYNQ7000_GPT_USE_TTC1_2 == TRUE
  gptObjectInit(&GPTD6);
  GPTD6.ttc = TTC1;
  GPTD6.tc_index = 2;
  GPTD6.irq_id = IRQ_ID_TTC1_2;
  GPTD6.irq_priority = ZYNQ7000_GPT_TTC1_2_IRQ_PRIORITY;
  timer_init(GPTD6.ttc, GPTD6.tc_index);
  interrupts_init(&GPTD6);
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_start(GPTDriver *gptp) {

  if (gptp->state == GPT_STOP) {
    timer_configure(gptp->ttc, gptp->tc_index, gptp->config->frequency);
    gic_irq_enable(gptp->irq_id);
  }
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop(GPTDriver *gptp) {

  if (gptp->state == GPT_READY) {
    timer_disable(gptp->ttc, gptp->tc_index);
    gic_irq_disable(gptp->irq_id);
  }
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void gpt_lld_start_timer(GPTDriver *gptp, gptcnt_t interval) {

  ttc_t *ttc = (ttc_t *)gptp->ttc;
  uint8_t tc_index = gptp->tc_index;

  /* Set interval */
  ttc->INTERVAL[tc_index] = interval - 1;

  /* Start counter */
  ttc->CNTCTRL[tc_index] = (1 << TTC_CNTCTRL_RESET_Pos) |
                           (1 << TTC_CNTCTRL_INTERVAL_Pos);
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void gpt_lld_stop_timer(GPTDriver *gptp) {

  ttc_t *ttc = (ttc_t *)gptp->ttc;
  uint8_t tc_index = gptp->tc_index;

  /* Disable counter */
  ttc->CNTCTRL[tc_index] = (1 << TTC_CNTCTRL_DISABLE_Pos);

  /* Clear any pending interrupts */
  ttc->ISR[tc_index];
}

/**
 * @brief   Changes the interval of GPT peripheral.
 * @details This function changes the interval of a running GPT unit.
 * @pre     The GPT unit must be running in continuous mode.
 * @post    The GPT unit interval is changed to the new value.
 * @note    This function simply stops and restarts the GPT peripheral.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @param[in] interval  new cycle time in timer ticks
 *
 * @notapi
 */
void gpt_lld_change_interval(GPTDriver *gptp, gptcnt_t interval) {

  /* If the new interval value happens to be less than the current counter
   * value, then the counter will not reset until it overflows. To prevent
   * this, simply stop and restart the timer with the new interval.
   */

  gpt_lld_stop_timer(gptp);
  gpt_lld_start_timer(gptp, interval);
}

/**
 * @brief   Returns the interval of GPT peripheral.
 * @pre     The GPT unit must be running in continuous mode.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @return              The current interval.
 *
 * @notapi
 */
gptcnt_t gpt_lld_get_interval(GPTDriver *gptp) {

  ttc_t *ttc = (ttc_t *)gptp->ttc;
  uint8_t tc_index = gptp->tc_index;

  return ttc->INTERVAL[tc_index] + 1;
}

/**
 * @brief   Returns the counter value of GPT peripheral.
 * @pre     The GPT unit must be running in continuous mode.
 * @note    The nature of the counter is not defined, it may count upward
 *          or downward, it could be continuously running or not.
 *
 * @param[in] gptp      pointer to a @p GPTDriver object
 * @return              The current counter value.
 *
 * @notapi
 */
gptcnt_t gpt_lld_get_counter(GPTDriver *gptp) {

  ttc_t *ttc = (ttc_t *)gptp->ttc;
  uint8_t tc_index = gptp->tc_index;

  return ttc->COUNTER[tc_index];
}

/**
 * @brief   Starts the timer in one shot mode and waits for completion.
 * @details This function specifically polls the timer waiting for completion
 *          in order to not have extra delays caused by interrupt servicing,
 *          this function is only recommended for short delays.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  time interval in ticks
 *
 * @notapi
 */
void gpt_lld_polled_delay(GPTDriver *gptp, gptcnt_t interval) {

  ttc_t *ttc = (ttc_t *)gptp->ttc;
  uint8_t tc_index = gptp->tc_index;

  /* Disable interrupts */
  gic_irq_disable(gptp->irq_id);

  gpt_lld_start_timer(gptp, interval);

  /* Wait for interval event */
  while (!ttc->ISR[tc_index] & TTC_INT_INTERVAL_Msk) ;

  gpt_lld_stop_timer(gptp);

  /* Restore interrupts */
  gic_irq_enable(gptp->irq_id);
}

#endif /* HAL_USE_GPT == TRUE */

/** @} */
