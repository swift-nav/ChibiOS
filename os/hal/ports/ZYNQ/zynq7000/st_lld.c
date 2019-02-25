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
static void prv_timer_irq_handler(void *context) {

  (void)context;
  osalSysLockFromISR();
  osalOsTimerHandlerI();
  osalSysUnlockFromISR();

  /* Write 1 to clear event flag */
  PRV_TIMER->INT_STATUS = (1 << PRV_TIMER_INTSTATUS_EVENTFLAG_Pos);
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

  PRV_TIMER->LOAD = ZYNQ7000_CPU_3x2x_FREQUENCY_Hz / OSAL_ST_FREQUENCY;
  PRV_TIMER->CONTROL = (1 << PRV_TIMER_CONTROL_ENABLE_Pos) |
                       (1 << PRV_TIMER_CONTROL_AUTORELOAD_Pos) |
                       (1 << PRV_TIMER_CONTROL_IRQENABLE_Pos) |
                       (0 << PRV_TIMER_CONTROL_PRESCALER_Pos);

  gic_handler_register(IRQ_ID_PRV_TIMER, prv_timer_irq_handler, 0);
  gic_irq_sensitivity_set(IRQ_ID_PRV_TIMER, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_PRV_TIMER, ZYNQ7000_ST_PRV_TIMER_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_PRV_TIMER);
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
