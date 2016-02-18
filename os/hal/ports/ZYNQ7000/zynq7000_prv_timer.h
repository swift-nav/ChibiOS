/*
 * zynq7000_prv_timer.h
 *
 *  Created on: Jan 24, 2016
 *      Author: jacob
 */

#ifndef _ZYNQ7000_PRV_TIMER_H_
#define _ZYNQ7000_PRV_TIMER_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t LOAD;
  volatile uint32_t COUNTER;
  volatile uint32_t CONTROL;
  volatile uint32_t INT_STATUS;
} prv_timer_t;

/* Bitfields */
#define PRV_TIMER_CONTROL_ENABLE_Pos (0U)
#define PRV_TIMER_CONTROL_ENABLE_Msk (0x1U << PRV_TIMER_CONTROL_ENABLE_Pos)

#define PRV_TIMER_CONTROL_AUTORELOAD_Pos (1U)
#define PRV_TIMER_CONTROL_AUTORELOAD_Msk (0x1U << PRV_TIMER_CONTROL_AUTORELOAD_Pos)

#define PRV_TIMER_CONTROL_IRQENABLE_Pos (2U)
#define PRV_TIMER_CONTROL_IRQENABLE_Msk (0x1U << PRV_TIMER_CONTROL_IRQENABLE_Pos)

#define PRV_TIMER_CONTROL_PRESCALER_Pos (8U)
#define PRV_TIMER_CONTROL_PRESCALER_Msk (0xFFU << PRV_TIMER_CONTROL_PRESCALER_Pos)

#define PRV_TIMER_INTSTATUS_EVENTFLAG_Pos (0U)
#define PRV_TIMER_INTSTATUS_EVENTFLAG_Msk (0x1U << PRV_TIMER_INTSTATUS_EVENTFLAG_Pos)

#endif /* _ZYNQ7000_PRV_TIMER_H_ */
