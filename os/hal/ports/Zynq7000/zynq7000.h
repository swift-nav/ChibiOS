/*
 * zynq7000.h
 *
 *  Created on: Jan 23, 2016
 *      Author: jacob
 */

#ifndef _ZYNQ7000_H_
#define _ZYNQ7000_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "zynq7000_gic.h"
#include "zynq7000_prv_timer.h"
#include "zynq7000_gpio.h"

/* Peripherals */
#define GIC_ICD_BASE 0xF8F01000U
#define GIC_ICD ((gic_icd_t *)GIC_ICD_BASE)

#define GIC_ICC_BASE 0xF8F00100U
#define GIC_ICC ((gic_icc_t *)GIC_ICC_BASE)

#define PRV_TIMER_BASE 0xF8F00600U
#define PRV_TIMER ((prv_timer_t *)PRV_TIMER_BASE)

#define GPIO_BASE 0xE000A000U
#define GPIO ((gpio_t *)GPIO_BASE)

#define GPIO_MIO_BASE 0xF8000700U
#define GPIO_MIO ((gpio_mio_t *)GPIO_MIO_BASE)

/* IRQ IDs */
typedef enum {
  IRQ_ID_PRV_TIMER =          29,
  IRQ_ID__COUNT =             94 /* Maximum number of IRQ IDs */
} irq_id_t;

/* IRQ Priorities */
typedef uint8_t irq_priority_t;

#endif /* _ZYNQ7000_H_ */
