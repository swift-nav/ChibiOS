/*
 * zynq7000_gpio.h
 *
 *  Created on: Jan 27, 2016
 *      Author: jacob
 */

#ifndef _ZYNQ7000_GPIO_H_
#define _ZYNQ7000_GPIO_H_

#include <stdint.h>

/* Registers */
typedef struct {
  struct {
    volatile uint32_t LSW;
    volatile uint32_t MSW;
  } MASK_DATA[4];
  volatile uint32_t RESERVED0[8];
  volatile uint32_t DATA[4];
  volatile uint32_t RESERVED1[4];
  volatile uint32_t DATA_RO[4];
  volatile uint32_t RESERVED2[101];
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
  } CFG[4];
} gpio_t;

typedef struct {
  volatile uint32_t MIO_PIN[54];
  volatile uint32_t RESERVED0[11];
  volatile uint32_t MIO_LOOPBACK;
  volatile uint32_t MIO_MST_TRI[2];
  volatile uint32_t SD_WP_CD_SEL[2];
} gpio_mio_t;

/* Bitfields */
#define GPIO_PIN_Pos(pin) (pin)
#define GPIO_PIN_Msk(pin) (0x1U << GPIO_PIN_Pos(pin))

#define GPIO_CFG_INT_TYPE_LEVEL (0U)
#define GPIO_CFG_INT_TYPE_EDGE (1U)

#define GPIO_CFG_INT_POLARITY_LOW_FALLING (0U)
#define GPIO_CFG_INT_POLARITY_HIGH_RISING (1U)

#define GPIO_MIO_PIN_TRI_ENABLE_Pos (0U)
#define GPIO_MIO_PIN_TRI_ENABLE_Msk (0x1U << GPIO_MIO_PIN_TRI_ENABLE_Pos)

#define GPIO_MIO_PIN_L0_SEL_Pos (1U)
#define GPIO_MIO_PIN_L0_SEL_Msk (0x1U << GPIO_MIO_PIN_L0_SEL_Pos)

#define GPIO_MIO_PIN_L1_SEL_Pos (2U)
#define GPIO_MIO_PIN_L1_SEL_Msk (0x1U << GPIO_MIO_PIN_L1_SEL_Pos)

#define GPIO_MIO_PIN_L2_SEL_Pos (3U)
#define GPIO_MIO_PIN_L2_SEL_Msk (0x3U << GPIO_MIO_PIN_L2_SEL_Pos)

#define GPIO_MIO_PIN_L3_SEL_Pos (5U)
#define GPIO_MIO_PIN_L3_SEL_Msk (0x7U << GPIO_MIO_PIN_L3_SEL_Pos)

#define GPIO_MIO_PIN_SPEED_Pos (8U)
#define GPIO_MIO_PIN_SPEED_Msk (0x1U << GPIO_MIO_PIN_SPEED_Pos)

#define GPIO_MIO_PIN_IO_TYPE_Pos (9U)
#define GPIO_MIO_PIN_IO_TYPE_Msk (0x7U << GPIO_MIO_PIN_IO_TYPE_Pos)

#define GPIO_MIO_PIN_PULLUP_Pos (12U)
#define GPIO_MIO_PIN_PULLUP_Msk (0x1U << GPIO_MIO_PIN_PULLUP_Pos)

#endif /* _ZYNQ7000_GPIO_H_ */
