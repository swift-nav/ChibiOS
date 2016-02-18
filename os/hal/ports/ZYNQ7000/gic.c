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
 * @file    ZYNQ7000/gic.c
 * @brief   ZYNQ7000 GIC peripheral support code.
 *
 * @addtogroup ZYNQ7000_GIC
 * @{
 */

#include "gic.h"

#include <string.h>
#include "hal.h"

/**
 * @brief   Macro for IRQ ID verification
 */
#define IRQ_ID_CHECK(id) osalDbgAssert(id < IRQ_ID__COUNT, "invalid IRQ ID")

/**
 * @brief   Element in the IRQ handler table
 */
typedef struct {
  irq_handler_t handler;
  void *context;
} irq_handler_table_entry_t;

/**
 * @brief   Table of IRQ handlers
 */
static irq_handler_table_entry_t irq_handler_table[IRQ_ID__COUNT];

/**
 * @brief   Initializes the GIC
 *
 * @notapi
 */
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

/**
 * @brief   Handles a CPU IRQ
 * @details This function should be called from the IRQ vector to handle
 *          and distribute the pending interrupt.
 *
 * @notapi
 */
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

/**
 * @brief   Installs an IRQ handler
 * @details Sets a vector for an interrupt source and enables it.
 *
 * @param[in] irq_id    the IRQ number
 * @param[in] handler   the pointer to the IRQ handler
 * @param[in] context   the data to be passed to the IRQ handler
 *
 * @api
 */
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

/**
 * @brief   Sets the priority level for an IRQ
 * @note    Lower priority values correspond to higher logical priority.
 *
 * @param[in] irq_id    the IRQ number
 * @param[in] priority  the priority level [0, 30]
 *
 * @api
 */
void gic_irq_priority_set(irq_id_t irq_id, irq_priority_t priority)
{
  IRQ_ID_CHECK(irq_id);
  osalDbgAssert(priority <= GIC_ICD_ICDIPR_PRIORITY_MAX,
      "invalid irq priority");

  GIC_ICD->ICDIPR[irq_id] = (priority << GIC_ICD_ICDIPR_PRIORITY_Pos);
}

/**
 * @brief   Sets the internal sensitivity for an IRQ
 * @note    Sensitivities are often fixed for specific IRQs. See product
 *          datasheet for specifics.
 *
 * @param[in] irq_id        the IRQ number
 * @param[in] sensitivity   the sensitivity to configure
 *
 * @api
 */
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

/**
 * @brief   Enables handling of an IRQ
 *
 * @param[in] irq_id        the IRQ number
 *
 * @api
 */
void gic_irq_enable(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDISER[GIC_ICD_ICDISER_SET_Reg(irq_id)] =
      GIC_ICD_ICDISER_SET_Msk(irq_id);
}

/**
 * @brief   Disables handling of an IRQ
 *
 * @param[in] irq_id        the IRQ number
 *
 * @api
 */
void gic_irq_disable(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDICER[GIC_ICD_ICDICER_CLEAR_Reg(irq_id)] =
      GIC_ICD_ICDICER_CLEAR_Msk(irq_id);
}

/**
 * @brief   Sets an IRQ pending
 *
 * @param[in] irq_id        the IRQ number
 *
 * @api
 */
void gic_irq_pending_set(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDISPR[GIC_ICD_ICDISPR_SET_Reg(irq_id)] =
      GIC_ICD_ICDISPR_SET_Msk(irq_id);
}

/**
 * @brief   Clears a pending IRQ
 *
 * @param[in] irq_id        the IRQ number
 *
 * @api
 */
void gic_irq_pending_clear(irq_id_t irq_id)
{
  IRQ_ID_CHECK(irq_id);

  GIC_ICD->ICDICPR[GIC_ICD_ICDICPR_CLEAR_Reg(irq_id)] =
      GIC_ICD_ICDICPR_CLEAR_Msk(irq_id);
}

