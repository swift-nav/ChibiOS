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
 * @file    ZYNQ7000/ext_lld.c
 * @brief   ZYNQ7000 EXT subsystem low level driver source.
 *
 * @addtogroup EXT
 * @{
 */

#include "gic.h"
#include "zynq7000.h"
#include "hal.h"


#if (HAL_USE_EXT == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EXT1 driver identifier.
 */
EXTDriver EXTD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Executes the callback for an EXT event.
 *
 * @param[in] port      GPIO port
 * @param[in] port      GPIO pin
 *
 * @notapi
 */
static void execute_callback(uint32_t port, uint32_t pin) {

  uint32_t channel;
  for (channel = 0; channel < EXT_MAX_CHANNELS; channel++) {

    const EXTChannelConfig *c = &EXTD1.config->channels[channel];
    uint32_t edges = c->mode & EXT_CH_MODE_EDGES_MASK;

    if ((edges != EXT_CH_MODE_DISABLED) &&
        (c->port == port) &&
        (c->pin == pin)) {

      if (c->cb != 0)
        c->cb(&EXTD1, channel);

      break;
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   Handles the GPIO IRQ.
 * @details Checks all GPIO ports for pins on which an event has occurred
 *          and is not masked. Executes callback functions where required.
 *
 * @param[in] context     IRQ context
 *
 * @notapi
 */
static void gpio_irq_handler(void *context) {

  (void)context;

  uint32_t port;
  for (port = 0; port < 4; port++) {
    /* Interrupts can only be generated for pins which are
     * not masked (INT_MASK) and on which an event has occurred (INT_STAT)
     */
    uint32_t int_sources = GPIO->CFG[port].INT_STAT &
                             ~GPIO->CFG[port].INT_MASK;

    if (!int_sources)
      continue;

    uint32_t pin;
    for (pin = 0; pin < 32; pin++) {
      uint32_t pin_mask = GPIO_PIN_Msk(pin);
      if (int_sources & pin_mask) {
        execute_callback(port, pin);
        /* Clear interrupt status */
        GPIO->CFG[port].INT_STAT = pin_mask;
      }
    }
  }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level EXT driver initialization.
 *
 * @notapi
 */
void ext_lld_init(void) {

  /* Driver initialization.*/
  extObjectInit(&EXTD1);

  /* Disable interrupts for all pins */
  uint32_t port;
  for (port = 0; port < 4; port++) {
    uint32_t pin;
    for (pin = 0; pin < 32; pin++) {
      GPIO->CFG[port].INT_DIS = GPIO_PIN_Msk(pin);
    }
  }

  /* Configure interrupts */
  gic_handler_register(IRQ_ID_GPIO, gpio_irq_handler, 0);
  gic_irq_sensitivity_set(IRQ_ID_GPIO, IRQ_SENSITIVITY_LEVEL);
  gic_irq_priority_set(IRQ_ID_GPIO, ZYNQ7000_EXT_GPIO_IRQ_PRIORITY);
}

/**
 * @brief   Configures and activates the EXT peripheral.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_start(EXTDriver *extp) {

  if (extp->state == EXT_STOP) {
    if (extp == &EXTD1) {
      /* Configuration of automatic channels.*/
      uint32_t channel;
      for (channel = 0; channel < EXT_MAX_CHANNELS; channel++) {
        if (extp->config->channels[channel].mode & EXT_CH_MODE_AUTOSTART)
          ext_lld_channel_enable(extp, channel);
      }

      gic_irq_enable(IRQ_ID_GPIO);
    }
  }
}

/**
 * @brief   Deactivates the EXT peripheral.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 *
 * @notapi
 */
void ext_lld_stop(EXTDriver *extp) {

  if (extp->state == EXT_ACTIVE) {
    if (extp == &EXTD1) {
      gic_irq_disable(IRQ_ID_GPIO);
    }
  }
}

/**
 * @brief   Enables an EXT channel.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 * @param[in] channel   channel to be enabled
 *
 * @notapi
 */
void ext_lld_channel_enable(EXTDriver *extp, expchannel_t channel) {

  const EXTChannelConfig *c = &extp->config->channels[channel];
  uint32_t port = c->port;
  uint32_t pin = c->pin;
  uint32_t edges = c->mode & EXT_CH_MODE_EDGES_MASK;

  if (edges == EXT_CH_MODE_DISABLED)
    return;

  /* Polarity */
  GPIO->CFG[port].INT_POLARITY &= ~GPIO_PIN_Msk(pin);
  GPIO->CFG[port].INT_ANY &= ~GPIO_PIN_Msk(pin);
  if (edges == EXT_CH_MODE_BOTH_EDGES) {
    GPIO->CFG[port].INT_ANY |= GPIO_PIN_Msk(pin);
  } else if (edges == EXT_CH_MODE_RISING_EDGE) {
    GPIO->CFG[port].INT_POLARITY |= (GPIO_CFG_INT_POLARITY_HIGH_RISING <<
                                      GPIO_PIN_Pos(pin));
  } else if (edges == EXT_CH_MODE_FALLING_EDGE) {
    GPIO->CFG[port].INT_POLARITY |= (GPIO_CFG_INT_POLARITY_LOW_FALLING <<
                                      GPIO_PIN_Pos(pin));
  }

  /* Sensitivity */
  GPIO->CFG[port].INT_TYPE &= ~GPIO_PIN_Msk(pin);
  GPIO->CFG[port].INT_TYPE |= (GPIO_CFG_INT_TYPE_EDGE << GPIO_PIN_Pos(pin));

  /* Clear status and enable interrupt */
  GPIO->CFG[port].INT_STAT = GPIO_PIN_Msk(pin);
  GPIO->CFG[port].INT_EN = GPIO_PIN_Msk(pin);
}

/**
 * @brief   Disables an EXT channel.
 *
 * @param[in] extp      pointer to the @p EXTDriver object
 * @param[in] channel   channel to be disabled
 *
 * @notapi
 */
void ext_lld_channel_disable(EXTDriver *extp, expchannel_t channel) {

  const EXTChannelConfig *c = &extp->config->channels[channel];
  uint32_t port = c->port;
  uint32_t pin = c->pin;

  GPIO->CFG[port].INT_DIS = GPIO_PIN_Msk(pin);
}

#endif /* HAL_USE_EXT == TRUE */

/** @} */
