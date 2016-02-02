/*
 * gic.h
 *
 *  Created on: Jan 24, 2016
 *      Author: jacob
 */

#ifndef _GIC_H_
#define _GIC_H_

#include "zynq7000.h"

typedef enum {
  IRQ_SENSITIVITY_EDGE,
  IRQ_SENSITIVITY_LEVEL
} irq_sensitivity_t;

typedef void (*irq_handler_t)(void *context);

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

#endif /* _GIC_H_ */
