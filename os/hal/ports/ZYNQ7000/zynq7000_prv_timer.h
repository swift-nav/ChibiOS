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
