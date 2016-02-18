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
 * @file    ZYNQ7000/hal_lld.c
 * @brief   ZYNQ7000 HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "gic.h"
#include "zynq7000.h"
#include "hal.h"


static OSAL_IRQ_HANDLER(irq_handler);
void * const irq_handler_addr = irq_handler; /* must be externally visible */

/*===========================================================================*/
/* Driver exported variables.                                                */
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
 * @brief   Top-level IRQ handler.
 *
 * @notapi
 */
static OSAL_IRQ_HANDLER(irq_handler) {

  OSAL_IRQ_PROLOGUE();
  gic_handle_irq();
  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  gic_init();
}

/**
 * @brief   Returns the ID of the currently executing CPU
 *
 * @notapi
 */
uint8_t hal_lld_cpu_id_get(void) {

  uint8_t cpu_id;
  asm volatile (
      "mrc p15, 0, %0, c0, c0, 5"
      : "=r" (cpu_id)
      :
      :
  );
  return cpu_id;
}

/* Early init hook */
void __early_init(void) {

  /* Write vector table address to VBAR */
  extern void _start(void);
  asm volatile (
      "mcr p15, 0, %0, c12, c0, 0"
      :
      : "r" (_start)
      :
  );

  /* TODO: Set up caches, MMU, SCU */
}

/** @} */
