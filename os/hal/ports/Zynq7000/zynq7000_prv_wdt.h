/*
 * zynq7000_prv_wdt.h
 *
 *  Created on: Feb 5, 2016
 *      Author: jacob
 */

#ifndef _ZYNQ7000_PRV_WDT_H_
#define _ZYNQ7000_PRV_WDT_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t LOAD;
  volatile uint32_t COUNTER;
  volatile uint32_t CONTROL;
  volatile uint32_t INT_STATUS;
  volatile uint32_t RST_STATUS;
  volatile uint32_t DISABLE;
} prv_wdt_t;

/* Bitfields */
#define PRV_WDT_CONTROL_ENABLE_Pos (0U)
#define PRV_WDT_CONTROL_ENABLE_Msk (0x1U << PRV_WDT_CONTROL_ENABLE_Pos)

#define PRV_WDT_CONTROL_AUTORELOAD_Pos (1U)
#define PRV_WDT_CONTROL_AUTORELOAD_Msk (0x1U << PRV_WDT_CONTROL_AUTORELOAD_Pos)

#define PRV_WDT_CONTROL_IRQENABLE_Pos (2U)
#define PRV_WDT_CONTROL_IRQENABLE_Msk (0x1U << PRV_WDT_CONTROL_IRQENABLE_Pos)

#define PRV_WDT_CONTROL_MODE_Pos (3U)
#define PRV_WDT_CONTROL_MODE_Msk (0x1U << PRV_WDT_CONTROL_MODE_Pos)
#define PRV_WDT_CONTROL_MODE_TIMER (0U)
#define PRV_WDT_CONTROL_MODE_WDT (1U)

#define PRV_WDT_CONTROL_PRESCALER_Pos (8U)
#define PRV_WDT_CONTROL_PRESCALER_Msk (0xFFU << PRV_WDT_CONTROL_PRESCALER_Pos)

#define PRV_WDT_INTSTATUS_EVENTFLAG_Pos (0U)
#define PRV_WDT_INTSTATUS_EVENTFLAG_Msk (0x1U << PRV_WDT_INTSTATUS_EVENTFLAG_Pos)

#define PRV_WDT_RSTSTATUS_RSTFLAG_Pos (0U)
#define PRV_WDT_RSTSTATUS_RSTFLAG_Msk (0x1U << PRV_WDT_RSTSTATUS_RSTFLAG_Pos)

#define PRV_WDT_DISABLE_VAL_1 (0x12345678U)
#define PRV_WDT_DISABLE_VAL_2 (0x87654321U)

#endif /* _ZYNQ7000_PRV_WDT_H_ */
