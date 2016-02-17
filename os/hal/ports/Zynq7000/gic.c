/*
 * gic.c
 *
 *  Created on: Jan 24, 2016
 *      Author: jacob
 */

#include "gic.h"

#include <string.h>
#include "hal.h"

#define IRQ_ID_CHECK(id) osalDbgAssert(id < IRQ_ID__COUNT, "invalid IRQ ID")

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
  IRQ_ID_CHECK(irq_id);

  /* Add to table */
  irq_handler_table[irq_id].handler = handler;
  irq_handler_table[irq_id].context = context;

  /* Target this CPU */
  uint8_t cpu_id = hal_lld_cpu_id_get();
  GIC_ICD->ICDIPTR[irq_id] |= (1 << GIC_ICD_ICDIPTR_CPUTARGETn_Pos(cpu_id));
}

void gic_irq_priority_set(irq_id_t irq_id, irq_priority_t priority)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDIPR[irq_id] = priority;
}

void gic_irq_sensitivity_set(irq_id_t irq_id, irq_sensitivity_t sensitivity)
{
  IRQ_ID_CHECK(irq_id);

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
    osalDbgAssert(0, "invalid irq sensitivity");
  }
}

void gic_irq_enable(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDISER[GIC_ICD_ICDISER_SET_Reg(irq_id)] =
      GIC_ICD_ICDISER_SET_Msk(irq_id);
}

void gic_irq_disable(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDICER[GIC_ICD_ICDICER_CLEAR_Reg(irq_id)] =
      GIC_ICD_ICDICER_CLEAR_Msk(irq_id);
}

void gic_irq_pending_set(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDISPR[GIC_ICD_ICDISPR_SET_Reg(irq_id)] =
      GIC_ICD_ICDISPR_SET_Msk(irq_id);
}

void gic_irq_pending_clear(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDICPR[GIC_ICD_ICDICPR_CLEAR_Reg(irq_id)] =
      GIC_ICD_ICDICPR_CLEAR_Msk(irq_id);
}

