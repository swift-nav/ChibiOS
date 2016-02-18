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
 * @file    ZYNQ7000/gic.h
 * @brief   ZYNQ7000 GIC peripheral support header.
 *
 * @addtogroup ZYNQ7000_GIC
 * @{
 */

#ifndef _GIC_H_
#define _GIC_H_

#include "zynq7000.h"

/**
 * @brief   IRQ priority type
 */
typedef uint8_t irq_priority_t;

/**
 * @brief   IRQ sensitivity type
 */
typedef enum {
  IRQ_SENSITIVITY_EDGE,
  IRQ_SENSITIVITY_LEVEL
} irq_sensitivity_t;

/**
 * @brief   IRQ handler type
 */
typedef void (*irq_handler_t)(void *context);

#ifdef __cplusplus
extern "C" {
#endif
  void gic_init(void);
  void gic_handle_irq(void);
  void gic_handler_register(irq_id_t irq_id, irq_handler_t handler,
                            void *context);
  void gic_irq_priority_set(irq_id_t irq_id, irq_priority_t priority);
  void gic_irq_sensitivity_set(irq_id_t irq_id, irq_sensitivity_t sensitivity);
  void gic_irq_enable(irq_id_t irq_id);
  void gic_irq_disable(irq_id_t irq_id);
  void gic_irq_pending_set(irq_id_t irq_id);
  void gic_irq_pending_clear(irq_id_t irq_id);
#ifdef __cplusplus
}
#endif

#endif /* _GIC_H_ */

/** @} */
