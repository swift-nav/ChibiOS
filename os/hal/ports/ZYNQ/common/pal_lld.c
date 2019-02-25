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
 * @file    ZYNQ7000/pal_lld.c
 * @brief   ZYNQ7000 PAL subsystem low level driver source.
 *
 * @addtogroup PAL
 * @{
 */

#include "zynq7000.h"
#include "hal.h"


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

static void mask_write(ioportid_t port, ioportmask_t mask, ioportmask_t bits) {

  if (mask & 0xFFFFU) {
    GPIO->MASK_DATA[port].LSW = ((~(mask & 0xFFFFU) & 0xFFFFU) << 16) |
                                ((bits & 0xFFFFU)              << 0);
  }

  if ((mask >> 16) & 0xFFFFU) {
    GPIO->MASK_DATA[port].MSW = ((~((mask >> 16) & 0xFFFFU) & 0xFFFFU) << 16) |
                                (((bits >> 16) & 0xFFFFU)              << 0);
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config    architecture-dependent ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {

  (void)config;

  /* TODO: Set initial MIO config and GPIO state */
}

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
ioportmask_t _pal_lld_readport(ioportid_t port) {
  return GPIO->DATA_RO[port];
}

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
ioportmask_t _pal_lld_readlatch(ioportid_t port) {
  return GPIO->DATA[port];
}

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
void _pal_lld_writeport(ioportid_t port, ioportmask_t bits) {
  GPIO->DATA[port] = bits;
}

/**
 * @brief   Sets a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
void _pal_lld_setport(ioportid_t port, ioportmask_t bits) {
  mask_write(port, bits, 0xFFFFFFFFU);
}

/**
 * @brief   Clears a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
void _pal_lld_clearport(ioportid_t port, ioportmask_t bits) {
  mask_write(port, bits, 0x0U);
}

/**
 * @brief   Toggles a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be XORed on the specified port
 *
 * @notapi
 */
void _pal_lld_toggleport(ioportid_t port, ioportmask_t bits) {
  mask_write(port, bits, ~GPIO->DATA[port]);
}

/**
 * @brief   Writes a group of bits.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask, a logic AND is performed on the
 *                      output data
 * @param[in] bits      bits to be written. Values exceeding the group
 *                      width are masked.
 *
 * @notapi
 */
void _pal_lld_writegroup(ioportid_t port,
                         ioportmask_t mask,
                         ioportmask_t bits) {
  mask_write(port, mask, bits);
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {
  if (mode == PAL_MODE_OUTPUT) {
    /* Set DIRM and OEN bits */
    GPIO->CFG[port].DIRM |= mask;
    GPIO->CFG[port].OEN |= mask;
  } else {
    /* Clear DIRM and OEN bits */
    GPIO->CFG[port].DIRM &= ~mask;
    GPIO->CFG[port].OEN &= ~mask;
  }
}

/**
 * @brief   Writes a logic state on an output pad.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] bit       logic value, the value must be @p PAL_LOW or
 *                      @p PAL_HIGH
 *
 * @notapi
 */
void _pal_lld_writepad(ioportid_t port, uint8_t pad, uint8_t bit) {
    mask_write(port, PAL_PORT_BIT(pad), (bit & 1U) << pad);
}

/**
 * @brief   Toggles a pad logic state.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
void _pal_lld_togglepad(ioportid_t port, uint8_t pad) {

  uint8_t bit = (GPIO->DATA[port] & GPIO_PIN_Msk(pad)) ? 0 : 1;
  _pal_lld_writepad(port, pad, bit);
}

#endif /* HAL_USE_PAL == TRUE */

/** @} */
