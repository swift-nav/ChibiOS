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
 * @file    pal_lld.c
 * @brief   PLATFORM PAL subsystem low level driver source.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#include "zynq7000.h"

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void _pal_lld_init(const PALConfig *config) {

  (void)config;

  /* TODO: Set initial MIO config and GPIO state */
}

ioportmask_t _pal_lld_readport(ioportid_t port) {
  return GPIO->DATA_RO[port];
}

ioportmask_t _pal_lld_readlatch(ioportid_t port) {
  return GPIO->DATA[port];
}

void _pal_lld_writeport(ioportid_t port, ioportmask_t bits) {
  GPIO->DATA[port] = bits;
}

void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {

  uint32_t function = (mode & 0xffff);
  bool output = (mode & PAL_DIR_OUTPUT);
  bool pullup = (mode & PAL_PULL_UP);
  bool fast = (mode & PAL_SPEED_FAST);
  bool mio = (port == GPIO0) || (port == GPIO1);

  if (function == PAL_PIN_FUNCTION_GPIO) {
    if (output) {
      /* Set DIRM and OEN bits */
      GPIO->CFG[port].DIRM |= mask;
      GPIO->CFG[port].OEN |= mask;
    } else {
      /* Clear DIRM and OEN bits */
      GPIO->CFG[port].DIRM &= ~mask;
      GPIO->CFG[port].OEN &= ~mask;
    }
  }

  uint32_t clear_mask = (GPIO_MIO_PIN_PULLUP_Msk |
                         GPIO_MIO_PIN_SPEED_Msk |
                         GPIO_MIO_PIN_L3_SEL_Msk |
                         GPIO_MIO_PIN_L2_SEL_Msk |
                         GPIO_MIO_PIN_L1_SEL_Msk |
                         GPIO_MIO_PIN_L0_SEL_Msk |
                         GPIO_MIO_PIN_TRI_ENABLE_Msk);

  uint32_t set_mask = (function << GPIO_MIO_PIN_L0_SEL_Pos);
  if (!output)
    set_mask |= GPIO_MIO_PIN_TRI_ENABLE_Msk;
  if (fast)
    set_mask |= GPIO_MIO_PIN_SPEED_Msk;
  if (pullup)
    set_mask |= GPIO_MIO_PIN_PULLUP_Msk;

  if (mio) {
    uint32_t mio_pin_number = port * PAL_IOPORTS_WIDTH;
    do {
      if (mask & 1) {
        uint32_t mio_pin_reg = GPIO_MIO->MIO_PIN[mio_pin_number];
        mio_pin_reg &= ~clear_mask;
        mio_pin_reg |= set_mask;
        GPIO_MIO->MIO_PIN[mio_pin_number] = mio_pin_reg;
      }

      mio_pin_number++;
      mask >>= 1;
    } while (mask);
  }
}

#endif /* HAL_USE_PAL == TRUE */

/** @} */
