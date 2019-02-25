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
 * @file    ZYNQ7000/st_lld.c
 * @brief   ZYNQ7000 ST Driver subsystem low level driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "gic.h"
#include "zynq7000.h"
#include "hal.h"


#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Handles the private timer IRQ. Calls the OS tick handler.
 *
 * @param[in] context     IRQ context
 *
 * @notapi
 */
static void systick_irq_handler(void *context) {
  (void)context;
  uint32_t isr = TTC3->ISR[2];
  if (isr & TTC_INT_INTERVAL_Msk) {
    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {
  TTC3->CLKCTRL[2] = (TTC_CLKCTRL_SRC_PCLK << TTC_CLKCTRL_SRC_Pos);
  TTC3->INTERVAL[2] = (100000000 / OSAL_ST_FREQUENCY) - 1;
  TTC3->CNTCTRL[2] =
      (1 << TTC_CNTCTRL_RESET_Pos) | (1 << TTC_CNTCTRL_INTERVAL_Pos);
  TTC3->IEN[2] = (1 << TTC_INT_INTERVAL_Pos);
  gic_handler_register(ZYNQMP_SYSTICK_IRQ, systick_irq_handler, 0);
  gic_irq_sensitivity_set(ZYNQMP_SYSTICK_IRQ, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(ZYNQMP_SYSTICK_IRQ, ZYNQMP_SYSTICK_IRQ_PRIORITY);
  gic_irq_enable(ZYNQMP_SYSTICK_IRQ);
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
