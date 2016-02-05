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
 * @file    pal_lld.h
 * @brief   PLATFORM PAL subsystem low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef _PAL_LLD_H_
#define _PAL_LLD_H_

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

#include "gpio.h"

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

#undef PAL_MODE_RESET
#undef PAL_MODE_UNCONNECTED
#undef PAL_MODE_INPUT
#undef PAL_MODE_INPUT_PULLUP
#undef PAL_MODE_INPUT_PULLDOWN
#undef PAL_MODE_INPUT_ANALOG
#undef PAL_MODE_OUTPUT_PUSHPULL
#undef PAL_MODE_OUTPUT_OPENDRAIN

/* Direction */
#define PAL_DIR_INPUT                 (0U)
#define PAL_DIR_OUTPUT                (1U << 16U)
#define PAL_DIR_PERICTRL              (PAL_DIR_OUTPUT)

/* Pull */
#define PAL_PULL_NONE                 (0U)
#define PAL_PULL_UP                   (1U << 17U)

/* Speed */
#define PAL_SPEED_SLOW                (0U)
#define PAL_SPEED_FAST                (1U << 18U)

/* Pin Function */
#define PAL_PIN_FUNCTION_GPIO         (0U)
#define PAL_PIN_FUNCTION(n)           (n)

/* Composite Modes */
#define PAL_MODE_INPUT                (PAL_DIR_INPUT  |                       \
                                       PAL_PULL_NONE  |                       \
                                       PAL_SPEED_SLOW |                       \
                                       PAL_PIN_FUNCTION_GPIO)

#define PAL_MODE_UNCONNECTED          (PAL_MODE_INPUT)
#define PAL_MODE_RESET                (PAL_MODE_INPUT)

#define PAL_MODE_INPUT_PULLUP         (PAL_DIR_INPUT  |                       \
                                       PAL_PULL_UP    |                       \
                                       PAL_SPEED_SLOW |                       \
                                       PAL_PIN_FUNCTION_GPIO)

#define PAL_MODE_OUTPUT_PUSHPULL      (PAL_DIR_OUTPUT |                       \
                                       PAL_PULL_NONE  |                       \
                                       PAL_SPEED_SLOW |                       \
                                       PAL_PIN_FUNCTION_GPIO)

#define PAL_MODE_CUSTOM(dir, pull, speed, func)                               \
                                      ((dir)          |                       \
                                       (pull)         |                       \
                                       (speed)        |                       \
                                       (func))

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @name    Port related definitions
 * @{
 */
/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH           (32U)

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT              ((ioportmask_t)0xFFFFFFFFU)
/** @} */

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 */
#define PAL_LINE(port, pad)                                                 \
  ((ioline_t)(((uint32_t)(port)) << 8U) | ((uint32_t)(pad)))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                                                      \
  ((ioportid_t)((((uint32_t)(line)) >> 8U) & 0xFFU))

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                                                       \
  ((uint32_t)((uint32_t)(line) & 0xFFU))

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                  (0xFFFFFFFFU)
/** @} */

/**
 * @brief   Generic I/O ports static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {

} PALConfig;

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Type of an I/O line.
 */
typedef uint32_t ioline_t;

/**
 * @brief   Port Identifier.
 * @details This type can be a scalar or some kind of pointer, do not make
 *          any assumption about it, use the provided macros when populating
 *          variables of this type.
 */
typedef uint32_t ioportid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @param[in] config    architecture-dependent ports configuration
 *
 * @notapi
 */
#define pal_lld_init(config) _pal_lld_init(config)

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) _pal_lld_readport(port)

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
#define pal_lld_readlatch(port) _pal_lld_readlatch(port)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits) _pal_lld_writeport(port, bits)

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] mode      group mode
 *
 * @notapi
 */
#define pal_lld_setgroupmode(port, mask, offset, mode)                      \
  _pal_lld_setgroupmode(port, mask << offset, mode)

#if !defined(__DOXYGEN__)
extern const PALConfig pal_default_config;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(const PALConfig *config);
  ioportmask_t _pal_lld_readport(ioportid_t port);
  ioportmask_t _pal_lld_readlatch(ioportid_t port);
  void _pal_lld_writeport(ioportid_t port, ioportmask_t bits);
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL == TRUE */

#endif /* _PAL_LLD_H_ */

/** @} */
