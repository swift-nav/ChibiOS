/*
 * gic.c
 *
 *  Created on: Jan 24, 2016
 *      Author: jacob
 */

#include "gic.h"
#include <assert.h>
#include <string.h>

#define CPU_ID 0 /* TODO: fix this */

typedef struct {
  irq_handler_t handler;
  void *context;
} irq_handler_table_entry_t;

static irq_handler_table_entry_t irq_handler_table[IRQ_ID__COUNT];

void gic_init(void)
{
  /* Configure CPU interface */
  GIC_ICC->ICCICR = (1 << GIC_ICC_ICCICR_ENABLES_Pos) |
                    (1 << GIC_ICC_ICCICR_ENABLENS_Pos) |
                    (1 << GIC_ICC_ICCICR_ACKCTL_Pos);

  /* Set priority mask */
  GIC_ICC->ICCPMR = 0xFF;

  /* Enable distributor */
  GIC_ICD->ICDDCR = (1 << GIC_ICD_ICDDCR_ENABLE_Pos);

  /* TODO: initialize edge/level sensitivities */

  /* Clear handler table */
  memset(irq_handler_table, 0, sizeof(irq_handler_table));
}

void gic_handle_irq(void)
{
  /* Read pending interrupt ID */
  uint32_t int_id_full = GIC_ICC->ICCIAR;
  irq_id_t irq_id = (int_id_full & GIC_ICC_ICCIAR_ACKINTID_Msk) >>
                        GIC_ICC_ICCIAR_ACKINTID_Pos;

  /* Execute handler */
  if (irq_id < IRQ_ID__COUNT) {
    const irq_handler_table_entry_t *entry = &irq_handler_table[irq_id];
    if (entry->handler != 0)
      entry->handler(entry->context);
  }

  /* Write interrupt ID back to end of interrupt register */
  GIC_ICC->ICCEOIR = int_id_full;
}

void gic_handler_register(irq_id_t irq_id, irq_handler_t handler,
                          void *context)
{
  assert(irq_id < IRQ_ID__COUNT);

  /* Add to table */
  irq_handler_table[irq_id].handler = handler;
  irq_handler_table[irq_id].context = context;

  /* Target this CPU */
  GIC_ICD->ICDIPTR[irq_id] |= (1 << GIC_ICD_ICDIPTR_CPUTARGETn_Pos(CPU_ID));
}

void gic_irq_priority_set(irq_id_t irq_id, irq_priority_t priority)
{
  assert(irq_id < IRQ_ID__COUNT);

  GIC_ICD->ICDIPR[irq_id] = priority;
}

void gic_irq_sensitivity_set(irq_id_t irq_id, irq_sensitivity_t sensitivity)
{
  assert(irq_id < IRQ_ID__COUNT);

  switch (sensitivity) {
  case IRQ_SENSITIVITY_EDGE: {
    GIC_ICD->ICDICR[GIC_ICD_ICDICR_EDGETRIG_Reg(irq_id)] |=
        GIC_ICD_ICDICR_EDGETRIG_Msk(irq_id);
  }
  break;

  case IRQ_SENSITIVITY_LEVEL: {
    GIC_ICD->ICDICR[GIC_ICD_ICDICR_EDGETRIG_Reg(irq_id)] &=
        ~GIC_ICD_ICDICR_EDGETRIG_Msk(irq_id);
  }
  break;

  default:
    assert("invalid irq sensitivity");
  }
}

void gic_irq_enable(irq_id_t irq_id)
{
  assert(irq_id < IRQ_ID__COUNT);

  GIC_ICD->ICDISER[GIC_ICD_ICDISER_SET_Reg(irq_id)] =
      GIC_ICD_ICDISER_SET_Msk(irq_id);
}

void gic_irq_disable(irq_id_t irq_id)
{
  assert(irq_id < IRQ_ID__COUNT);

  GIC_ICD->ICDICER[GIC_ICD_ICDICER_CLEAR_Reg(irq_id)] =
      GIC_ICD_ICDICER_CLEAR_Msk(irq_id);
}
