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
#include "zynq7000_gpio.h"
#include "zynq7000_prv_timer.h"
#include "zynq7000_prv_wdt.h"
#include "zynq7000_spi.h"
#include "zynq7000_ttc.h"
#include "zynq7000_uart.h"

/* Peripherals */
#define GIC_ICD_BASE 0xF8F01000U
#define GIC_ICD ((gic_icd_t *)GIC_ICD_BASE)

#define GIC_ICC_BASE 0xF8F00100U
#define GIC_ICC ((gic_icc_t *)GIC_ICC_BASE)

#define PRV_TIMER_BASE 0xF8F00600U
#define PRV_TIMER ((prv_timer_t *)PRV_TIMER_BASE)

#define PRV_WDT_BASE 0xF8F00620U
#define PRV_WDT ((prv_wdt_t *)PRV_WDT_BASE)

#define GPIO_BASE 0xE000A000U
#define GPIO ((gpio_t *)GPIO_BASE)

#define GPIO_MIO_BASE 0xF8000700U
#define GPIO_MIO ((gpio_mio_t *)GPIO_MIO_BASE)

#define UART0_BASE 0xE0000000U
#define UART0 ((uart_t *)UART0_BASE)

#define UART1_BASE 0xE0001000U
#define UART1 ((uart_t *)UART1_BASE)

#define SPI0_BASE 0xE0006000U
#define SPI0 ((spi_t *)SPI0_BASE)

#define SPI1_BASE 0xE0007000U
#define SPI1 ((spi_t *)SPI1_BASE)

#define TTC0_BASE 0xF8001000U
#define TTC0 ((ttc_t *)TTC0_BASE)

#define TTC1_BASE 0xF8002000U
#define TTC1 ((ttc_t *)TTC1_BASE)

/* IRQ IDs */
typedef enum {
  IRQ_ID_PRV_TIMER =          29,
  IRQ_ID_TTC0_0 =             42,
  IRQ_ID_TTC0_1 =             43,
  IRQ_ID_TTC0_2 =             44,
  IRQ_ID_GPIO =               52,
  IRQ_ID_SPI0 =               58,
  IRQ_ID_UART0 =              59,
  IRQ_ID_TTC1_0 =             69,
  IRQ_ID_TTC1_1 =             70,
  IRQ_ID_TTC1_2 =             71,
  IRQ_ID_SPI1 =               81,
  IRQ_ID_UART1 =              82,
  IRQ_ID__COUNT =             94 /* Maximum number of IRQ IDs */
} irq_id_t;

#endif /* _ZYNQ7000_H_ */
