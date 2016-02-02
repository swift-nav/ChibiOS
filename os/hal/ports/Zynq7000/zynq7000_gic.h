/*
 * zynq7000_gic.h
 *
 *  Created on: Jan 24, 2016
 *      Author: jacob
 */

#ifndef _ZYNQ7000_GIC_H_
#define _ZYNQ7000_GIC_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t ICDDCR;
  volatile uint32_t ICDICTR;
  volatile uint32_t ICDIIDR;
  volatile uint32_t RESERVED0[29];
  volatile uint32_t ICDISR[32];
  volatile uint32_t ICDISER[32];
  volatile uint32_t ICDICER[32];
  volatile uint32_t ICDISPR[32];
  volatile uint32_t ICDICPR[32];
  volatile uint32_t ICDABR[32];
  volatile uint32_t RESERVED1[32];
  volatile uint8_t  ICDIPR[1020];
  volatile uint32_t RESERVED2[1];
  volatile uint8_t  ICDIPTR[1020];
  volatile uint32_t RESERVED3[1];
  volatile uint32_t ICDICR[64];
  volatile uint32_t PPI_STAT;
  volatile uint32_t SPI_STAT[31];
  volatile uint32_t RESERVED4[96];
  volatile uint32_t ICDSGIR;
  volatile uint32_t RESERVED5[51];
  volatile uint32_t PERIPHID[8];
  volatile uint32_t COMPONENTID[4];
} gic_icd_t;

typedef struct {
  volatile uint32_t ICCICR;
  volatile uint32_t ICCPMR;
  volatile uint32_t ICCBPR;
  volatile uint32_t ICCIAR;
  volatile uint32_t ICCEOIR;
  volatile uint32_t ICCRPR;
  volatile uint32_t ICCHPIR;
  volatile uint32_t ICCABPR;
  volatile uint32_t RESERVED0[55];
  volatile uint32_t ICCIIDR;
} gic_icc_t;

/* Bitfields */
#define GIC_ICD_ICDDCR_ENABLE_Pos (0U)
#define GIC_ICD_ICDDCR_ENABLE_Msk (0x1U << GIC_ICD_ICDDCR_ENABLE_Pos)

#define GIC_ICD_ICDISER_SET_Reg(id) ((id) / 32U)
#define GIC_ICD_ICDISER_SET_Pos(id) ((id) % 32U)
#define GIC_ICD_ICDISER_SET_Msk(id) (0x1U << GIC_ICD_ICDISER_SET_Pos(id))

#define GIC_ICD_ICDICER_CLEAR_Reg(id) ((id) / 32U)
#define GIC_ICD_ICDICER_CLEAR_Pos(id) ((id) % 32U)
#define GIC_ICD_ICDICER_CLEAR_Msk(id) (0x1U << GIC_ICD_ICDICER_CLEAR_Pos(id))

#define GIC_ICD_ICDISPR_SET_Reg(id) ((id) / 32U)
#define GIC_ICD_ICDISPR_SET_Pos(id) ((id) % 32U)
#define GIC_ICD_ICDISPR_SET_Msk(id) (0x1U << GIC_ICD_ICDISPR_SET_Pos(id))

#define GIC_ICD_ICDICPR_CLEAR_Reg(id) ((id) / 32U)
#define GIC_ICD_ICDICPR_CLEAR_Pos(id) ((id) % 32U)
#define GIC_ICD_ICDICPR_CLEAR_Msk(id) (0x1U << GIC_ICD_ICDICPR_CLEAR_Pos(id))

#define GIC_ICD_ICDIPTR_CPUTARGETn_Pos(cpu) (cpu)
#define GIC_ICD_ICDIPTR_CPUTARGETn_Msk(cpu) (0x1U << GIC_ICD_ICDIPTR_CPUTARGETn_Pos(cpu))

#define GIC_ICD_ICDICR_EDGETRIG_Reg(id) ((id) / 16U)
#define GIC_ICD_ICDICR_EDGETRIG_Pos(id) (2U * ((id) % 16U) + 1U)
#define GIC_ICD_ICDICR_EDGETRIG_Msk(id) (0x1U << GIC_ICD_ICDICR_EDGETRIG_Pos(id))

#define GIC_ICC_ICCICR_ENABLES_Pos (0U)
#define GIC_ICC_ICCICR_ENABLES_Msk (0x1U << GIC_ICC_ICCICR_ENABLES_Pos)

#define GIC_ICC_ICCICR_ENABLENS_Pos (1U)
#define GIC_ICC_ICCICR_ENABLENS_Msk (0x1U << GIC_ICC_ICCICR_ENABLENS_Pos)

#define GIC_ICC_ICCICR_ACKCTL_Pos (2U)
#define GIC_ICC_ICCICR_ACKCTL_Msk (0x1U << GIC_ICC_ICCICR_ACKCTL_Pos)

#define GIC_ICC_ICCIAR_ACKINTID_Pos (0U)
#define GIC_ICC_ICCIAR_ACKINTID_Msk (0x3FF << GIC_ICC_ICCIAR_ACKINTID_Pos)

#endif /* _ZYNQ7000_GIC_H_ */
