/*
 * zynq7000_ttc.h
 *
 *  Created on: Feb 6, 2016
 *      Author: jacob
 */

#ifndef _ZYNQ7000_TTC_H_
#define _ZYNQ7000_TTC_H_

#include <stdint.h>

/* Registers */
typedef struct {
  volatile uint32_t CLKCTRL[3];
  volatile uint32_t CNTCTRL[3];
  volatile uint32_t COUNTER[3];
  volatile uint32_t INTERVAL[3];
  struct {
    volatile uint32_t MATCH0;
    volatile uint32_t MATCH1;
    volatile uint32_t MATCH2;
  } MATCH[3];
  volatile uint32_t ISR[3];
  volatile uint32_t IEN[3];
  volatile uint32_t EVTCTRL[3];
  volatile uint32_t EVENT[3];
} ttc_t;

/* Bitfields */
#define TTC_CLKCTRL_PSEN_Pos (0U)
#define TTC_CLKCTRL_PSEN_Msk (0x1U << TTC_CLKCTRL_PSEN_Pos)

#define TTC_CLKCTRL_PSVAL_Pos (1U)
#define TTC_CLKCTRL_PSVAL_Msk (0xFU << TTC_CLKCTRL_PSVAL_Pos)

#define TTC_CLKCTRL_SRC_Pos (5U)
#define TTC_CLKCTRL_SRC_Msk (0x1U << TTC_CLKCTRL_SRC_Pos)
#define TTC_CLKCTRL_SRC_PCLK (0U)
#define TTC_CLKCTRL_SRC_EXTCLK (1U)

#define TTC_CLKCTRL_EXTEDGE_Pos (6U)
#define TTC_CLKCTRL_EXTEDGE_Msk (0x1U << TTC_CLKCTRL_EXTEDGE_Pos)
#define TTC_CLKCTRL_EXTEDGE_RISING (0U)
#define TTC_CLKCTRL_EXTEDGE_FALLING (1U)


#define TTC_CNTCTRL_DISABLE_Pos (0U)
#define TTC_CNTCTRL_DISABLE_Msk (0x1U << TTC_CNTCTRL_DISABLE_Pos)

#define TTC_CNTCTRL_INTERVAL_Pos (1U)
#define TTC_CNTCTRL_INTERVAL_Msk (0x1U << TTC_CNTCTRL_INTERVAL_Pos)

#define TTC_CNTCTRL_DECREMENT_Pos (2U)
#define TTC_CNTCTRL_DECREMENT_Msk (0x1U << TTC_CNTCTRL_DECREMENT_Pos)

#define TTC_CNTCTRL_MATCH_Pos (3U)
#define TTC_CNTCTRL_MATCH_Msk (0x1U << TTC_CNTCTRL_MATCH_Pos)

#define TTC_CNTCTRL_RESET_Pos (4U)
#define TTC_CNTCTRL_RESET_Msk (0x1U << TTC_CNTCTRL_RESET_Pos)

#define TTC_CNTCTRL_WAVEEN_Pos (5U)
#define TTC_CNTCTRL_WAVEEN_Msk (0x1U << TTC_CNTCTRL_WAVEEN_Pos)

#define TTC_CNTCTRL_WAVEPOL_Pos (6U)
#define TTC_CNTCTRL_WAVEPOL_Msk (0x1U << TTC_CNTCTRL_WAVEPOL_Pos)


#define TTC_INT_INTERVAL_Pos (0U)
#define TTC_INT_INTERVAL_Msk (0x1U << TTC_INT_INTERVAL_Pos)

#define TTC_INT_MATCH0_Pos (1U)
#define TTC_INT_MATCH0_Msk (0x1U << TTC_INT_MATCH0_Pos)

#define TTC_INT_MATCH1_Pos (2U)
#define TTC_INT_MATCH1_Msk (0x1U << TTC_INT_MATCH1_Pos)

#define TTC_INT_MATCH2_Pos (3U)
#define TTC_INT_MATCH2_Msk (0x1U << TTC_INT_MATCH2_Pos)

#define TTC_INT_CNTOVR_Pos (4U)
#define TTC_INT_CNTOVR_Msk (0x1U << TTC_INT_CNTOVR_Pos)

#define TTC_INT_EVTOVR_Pos (5U)
#define TTC_INT_EVTOVR_Msk (0x1U << TTC_INT_EVTOVR_Pos)


#define TTC_EVTCTRL_ENABLE_Pos (0U)
#define TTC_EVTCTRL_ENABLE_Msk (0x1U << TTC_EVTCTRL_ENABLE_Pos)

#define TTC_EVTCTRL_POLARITY_Pos (1U)
#define TTC_EVTCTRL_POLARITY_Msk (0x1U << TTC_EVTCTRL_POLARITY_Pos)
#define TTC_EVTCTRL_POLARITY_HIGH (0U)
#define TTC_EVTCTRL_POLARITY_LOW (1U)

#define TTC_EVTCTRL_OVERFLOW_Pos (2U)
#define TTC_EVTCTRL_OVERFLOW_Msk (0x1U << TTC_EVTCTRL_OVERFLOW_Pos)
#define TTC_EVTCTRL_OVERFLOW_DISABLE (0U)
#define TTC_EVTCTRL_OVERFLOW_CONTINUE (1U)

#endif /* _ZYNQ7000_TTC_H_ */
