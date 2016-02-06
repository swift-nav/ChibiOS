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
 * @file    templates/wdg_lld.c
 * @brief   WDG Driver subsystem low level driver source template.
 *
 * @addtogroup WDG
 * @{
 */

#include "hal.h"

#include "zynq7000.h"

#if HAL_USE_WDG || defined(__DOXYGEN__)

#define DIV_CEIL_UNSIGNED(num, div) (1U + ((num) - 1U) / (div))

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if (ZYNQ7000_WDG_USE_WDG1 == TRUE) || defined(__DOXYGEN__)
WDGDriver WDGD1;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void period_calc(uint32_t period_ms, uint8_t *r_prescaler, uint32_t *r_load) {

  uint32_t wdg_clk_freq_khz = DIV_CEIL_UNSIGNED(CPU_FREQUENCY_Hz, 2*1000);

  /* Maximum prescaled frequency to avoid overflow in load value calculation */
  uint32_t prescaled_freq_khz_max = UINT32_MAX / period_ms;
  uint32_t prescaler = DIV_CEIL_UNSIGNED(wdg_clk_freq_khz,
                                         prescaled_freq_khz_max) - 1;
  if (prescaler > UINT8_MAX) {
    *r_prescaler = UINT8_MAX;
    *r_load = UINT32_MAX;
  } else {
    *r_prescaler = prescaler;
    *r_load = period_ms * DIV_CEIL_UNSIGNED(wdg_clk_freq_khz, prescaler + 1);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level WDG driver initialization.
 *
 * @notapi
 */
void wdg_lld_init(void) {

#if ZYNQ7000_WDG_USE_WDG1 == TRUE
  WDGD1.state = WDG_STOP;
#endif
}

/**
 * @brief   Configures and activates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_start(WDGDriver *wdgp) {

  uint8_t prescaler;
  uint32_t load;
  period_calc(wdgp->config->period_ms, &prescaler, &load);
  wdgp->load = load;

  PRV_WDT->LOAD = wdgp->load;
  PRV_WDT->CONTROL = (1 << PRV_WDT_CONTROL_ENABLE_Pos) |
                     (PRV_WDT_CONTROL_MODE_WDT << PRV_WDT_CONTROL_MODE_Pos) |
                     (prescaler << PRV_WDT_CONTROL_PRESCALER_Pos);
}

/**
 * @brief   Deactivates the WDG peripheral.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @api
 */
void wdg_lld_stop(WDGDriver *wdgp) {

  (void)wdgp;
  PRV_WDT->CONTROL &= ~PRV_WDT_CONTROL_ENABLE_Msk;
}

/**
 * @brief   Reloads WDG's counter.
 *
 * @param[in] wdgp      pointer to the @p WDGDriver object
 *
 * @notapi
 */
void wdg_lld_reset(WDGDriver * wdgp) {

  PRV_WDT->LOAD = wdgp->load;
}

#endif /* HAL_USE_WDG */

/** @} */
