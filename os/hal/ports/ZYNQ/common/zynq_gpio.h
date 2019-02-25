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

#ifndef _ZYNQ7000_GPIO_H_
#define _ZYNQ7000_GPIO_H_

#include <stdint.h>

/* Registers */
typedef struct {
  struct {
    volatile uint32_t LSW;
    volatile uint32_t MSW;
  } MASK_DATA[6];
  volatile uint32_t RESERVED0[4];
  volatile uint32_t DATA[6];
  volatile uint32_t RESERVED1[2];
  volatile uint32_t DATA_RO[6];
  volatile uint32_t RESERVED2[99];
  struct {
    volatile uint32_t DIRM;
    volatile uint32_t OEN;
    volatile uint32_t INT_MASK;
    volatile uint32_t INT_EN;
    volatile uint32_t INT_DIS;
    volatile uint32_t INT_STAT;
    volatile uint32_t INT_TYPE;
    volatile uint32_t INT_POLARITY;
    volatile uint32_t INT_ANY;
    volatile uint32_t RESERVED3[7];
  } CFG[6];
} gpio_t;

/* Bitfields */
#define GPIO_PIN_Pos(pin) (pin)
#define GPIO_PIN_Msk(pin) (0x1U << GPIO_PIN_Pos(pin))

#define GPIO_CFG_INT_TYPE_LEVEL (0U)
#define GPIO_CFG_INT_TYPE_EDGE (1U)

#define GPIO_CFG_INT_POLARITY_LOW_FALLING (0U)
#define GPIO_CFG_INT_POLARITY_HIGH_RISING (1U)

#endif /* _ZYNQ7000_GPIO_H_ */
