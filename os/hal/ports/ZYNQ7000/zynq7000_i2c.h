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

#ifndef _ZYNQ7000_I2C_H_
#define _ZYNQ7000_I2C_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t CR;         /**< 32-bit Control */
  volatile uint32_t SR;         /**< Status */
  volatile uint32_t ADDR;       /**< IIC Address */
  volatile uint32_t DATA;       /**< IIC FIFO Data */
  volatile uint32_t ISR;        /**< Interrupt Status */
  volatile uint32_t TRANS_SIZE; /**< Transfer Size */
  volatile uint32_t SLV_PAUSE;  /**< Slave monitor pause */
  volatile uint32_t TIME_OUT;   /**< Time Out */
  volatile uint32_t IMR;        /**< Interrupt Enabled Mask */
  volatile uint32_t IER;        /**< Interrupt Enable */
  volatile uint32_t IDR;        /**< Interrupt Disable */
} i2c_t;

/* Bitfields */

/** @name Control Register
 *
 * This register contains various control bits that
 * affects the operation of the IIC controller. Read/Write.
 * @{
 */

#define I2C_CR_DIV_A_Msk	0x0000C000U /**< Clock Divisor A */
#define I2C_CR_DIV_A_Pos			14U /**< Clock Divisor A shift */
#define I2C_DIV_A_MAX				 4U /**< Maximum value of Divisor A */
#define I2C_CR_DIV_B_Msk	0x00003F00U /**< Clock Divisor B */
#define I2C_CR_DIV_B_Pos			 8U /**< Clock Divisor B shift */
#define I2C_CR_CLR_FIFO_Msk	0x00000040U /**< Clear FIFO, auto clears*/
#define I2C_CR_SLVMON_Msk	0x00000020U /**< Slave monitor mode */
#define I2C_CR_HOLD_Msk		0x00000010U /**<  Hold bus 1=Hold scl,
												0=terminate transfer */
#define I2C_CR_ACKEN_Msk	0x00000008U /**< Enable TX of ACK when
												Master receiver*/
#define I2C_CR_NEA_Msk		0x00000004U /**< Addressing Mode 1=7 bit,
												0=10 bit */
#define I2C_CR_MS_Msk		0x00000002U /**< Master mode bit 1=Master,
												0=Slave */
#define I2C_CR_RD_WR_Msk	0x00000001U /**< Read or Write Master
												transfer  0=Transmitter,
												1=Receiver*/
#define I2C_CR_RESET_VALUE			 0U /**< Reset value of the Control
												register */
/* @} */

/** @name IIC Status Register
 *
 * This register is used to indicate status of the IIC controller. Read only
 * @{
 */
#define I2C_SR_BA_Msk		0x00000100U  /**< Bus Active Mask */
#define I2C_SR_RXOVF_Msk	0x00000080U  /**< Receiver Overflow Mask */
#define I2C_SR_TXDV_Msk		0x00000040U  /**< Transmit Data Valid Mask */
#define I2C_SR_RXDV_Msk		0x00000020U  /**< Receiver Data Valid Mask */
#define I2C_SR_RXRW_Msk		0x00000008U  /**< Receive read/write Mask */
/* @} */

/** @name IIC Address Register
 *
 * Normal addressing mode uses add[6:0]. Extended addressing mode uses add[9:0].
 * A write access to this register always initiates a transfer if the IIC is in
 * master mode. Read/Write
 * @{
 */
#define I2C_ADDR_Msk	0x000003FF  /**< IIC Address Mask */
/* @} */

/** @name IIC Data Register
 *
 * When written to, the data register sets data to transmit. When read from, the
 * data register reads the last received byte of data. Read/Write
 * @{
 */
#define I2C_DATA_Msk	0x000000FF  /**< IIC Data Mask */
/* @} */

/** @name IIC Interrupt Registers
 *
 * <b>IIC Interrupt Status Register</b>
 *
 * This register holds the interrupt status flags for the IIC controller. Some
 * of the flags are level triggered
 * - i.e. are set as long as the interrupt condition exists.  Other flags are
 *   edge triggered, which means they are set one the interrupt condition occurs
 *   then remain set until they are cleared by software.
 *   The interrupts are cleared by writing a one to the interrupt bit position
 *   in the Interrupt Status Register. Read/Write.
 *
 * <b>IIC Interrupt Enable Register</b>
 *
 * This register is used to enable interrupt sources for the IIC controller.
 * Writing a '1' to a bit in this register clears the corresponding bit in the
 * IIC Interrupt Mask register.  Write only.
 *
 * <b>IIC Interrupt Disable Register </b>
 *
 * This register is used to disable interrupt sources for the IIC controller.
 * Writing a '1' to a bit in this register sets the corresponding bit in the
 * IIC Interrupt Mask register. Write only.
 *
 * <b>IIC Interrupt Mask Register</b>
 *
 * This register shows the enabled/disabled status of each IIC controller
 * interrupt source. A bit set to 1 will ignore the corresponding interrupt in
 * the status register. A bit set to 0 means the interrupt is enabled.
 * All mask bits are set and all interrupts are disabled after reset. Read only.
 *
 * All four registers have the same bit definitions. They are only defined once
 * for each of the Interrupt Enable Register, Interrupt Disable Register,
 * Interrupt Mask Register, and Interrupt Status Register
 * @{
 */

#define I2C_IXR_ARB_LOST_Msk  0x00000200U	 /**< Arbitration Lost Interrupt
													mask */
#define I2C_IXR_RX_UNF_Msk    0x00000080U	 /**< FIFO Recieve Underflow
													Interrupt mask */
#define I2C_IXR_TX_OVR_Msk    0x00000040U	 /**< Transmit Overflow
													Interrupt mask */
#define I2C_IXR_RX_OVR_Msk    0x00000020U	 /**< Receive Overflow Interrupt
													mask */
#define I2C_IXR_SLV_RDY_Msk   0x00000010U	 /**< Monitored Slave Ready
													Interrupt mask */
#define I2C_IXR_TO_Msk        0x00000008U	 /**< Transfer Time Out
													Interrupt mask */
#define I2C_IXR_NACK_Msk      0x00000004U	 /**< NACK Interrupt mask */
#define I2C_IXR_DATA_Msk      0x00000002U	 /**< Data Interrupt mask */
#define I2C_IXR_COMP_Msk      0x00000001U	 /**< Transfer Complete
													Interrupt mask */
#define I2C_IXR_DEFAULT_Msk   0x000002FFU	 /**< Default ISR Mask */
#define I2C_IXR_ALL_INTR_Msk  0x000002FFU	 /**< All ISR Mask */
/* @} */


/** @name IIC Transfer Size Register
*
* The register's meaning varies according to the operating mode as follows:
*   - Master transmitter mode: number of data bytes still not transmitted minus
*     one
*   - Master receiver mode: number of data bytes that are still expected to be
*     received
*   - Slave transmitter mode: number of bytes remaining in the FIFO after the
*     master terminates the transfer
*   - Slave receiver mode: number of valid data bytes in the FIFO
*
* This register is cleared if CLR_FIFO bit in the control register is set.
* Read/Write
* @{
*/
#define I2C_TRANS_SIZE_Msk  0x0000003F /**< IIC Transfer Size Mask */
#define I2C_FIFO_DEPTH          16	  /**< Number of bytes in the FIFO */
#define I2C_DATA_INTR_DEPTH     14    /**< Number of bytes at DATA intr */
/* @} */


/** @name IIC Slave Monitor Pause Register
*
* This register is associated with the slave monitor mode of the I2C interface.
* It is meaningful only when the module is in master mode and bit SLVMON in the
* control register is set.
*
* This register defines the pause interval between consecutive attempts to
* address the slave once a write to an I2C address register is done by the
* host. It represents the number of sclk cycles minus one between two attempts.
*
* The reset value of the register is 0, which results in the master repeatedly
* trying to access the slave immediately after unsuccessful attempt.
* Read/Write
* @{
*/
#define I2C_SLV_PAUSE_Msk    0x0000000F  /**< Slave monitor pause mask */
/* @} */


/** @name IIC Time Out Register
*
* The value of time out register represents the time out interval in number of
* sclk cycles minus one.
*
* When the accessed slave holds the sclk line low for longer than the time out
* period, thus prohibiting the I2C interface in master mode to complete the
* current transfer, an interrupt is generated and TO interrupt flag is set.
*
* The reset value of the register is 0x1f.
* Read/Write
* @{
 */
#define I2C_TIME_OUT_Msk    0x000000FFU    /**< IIC Time Out mask */
#define I2C_TO_RESET_VALUE   0x000000FFU    /**< IIC Time Out reset value */
/* @} */

#endif /* _ZYNQ7000_I2C_H_ */
