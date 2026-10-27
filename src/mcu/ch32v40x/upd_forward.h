/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The downstream gate (port 2), shared by the application and the bootloader: FWD_EN (PA3) is TIM2 CH4's output
 * compare (default mapping, TIM2_RM 00: RM p.132-135; PA15 stays the ARGB output's). TIM2 counts DIN's falling edges:
 * external clock mode 1 on ITR2 = TIM3 (RM p.244 table 15-2), TIM3 CH1 capturing them (its TRGO per capture, MMS 011,
 * the same trigger that clocks the delay line's TIM4 in the app). Three modes (OC4M, RM p.254): forced LOW = closed
 * (reset, STREAM, a window, flash work), forced HIGH = open (update transparency), toggle at CNT == 256 = the slot plan
 * (closed for this board's slot, open from bit 257 on). No interrupt, no DMA: the edge-to-pin path is the timer's own.
 * DE_OUT (PA10) drives port 2 (push-pull HIGH) except in a relay window (hw.c rc_hw_relay). The caller writes the gate
 * only inside a quiet LOW (it qualifies the input) and keeps PA9 floating outside its replies. */
#ifndef MCU_UPD_FORWARD_H
#define MCU_UPD_FORWARD_H
#include "ch32v40x.h"

/* The gate opens only after UPD_FORWARD_QUIET_US (ultra_update.h, 350 us) of LOW with no edge, and the slot plan re-arms
 * only after ULTRA_RW_REARM_US (ultra_protocol.h, 200 us), both measured on the nominal clock: the real clock is never
 * faster, so the real LOW is never shorter. */
#define GATE_OCM_TOGGLE 3u
#define GATE_OCM_LO     4u
#define GATE_OCM_HI     5u
#define GATE_SLOT_BITS  256u                             /* ULTRA_SLOT_BITS: the 256th falling edge ends this board's slot */

static inline void gate_mode(uint32_t ocm)
{
    TIM2->CHCTLR2 = (uint16_t)(ocm << 12);               /* OC4M; CH3 unused (its bits 0) */
}

/* the gate closed and counting, DE_OUT driving: before the pin leaves its reset input (pull-down: closed) */
static inline void upd_forward_init(void)
{
    RCC->PB1PCENR |= RCC_TIM2EN;
    TIM2->CTLR1 = 0;
    TIM2->PSC = 0;
    TIM2->ATRLR = 0xFFFFu;
    TIM2->SMCFGR = (uint16_t)(2u << 4);                  /* TS = ITR2 (TIM3) first: TS only while SMS = 0 */
    TIM2->SMCFGR = (uint16_t)((2u << 4) | 7u);           /* SMS = 111: external clock mode 1 */
    gate_mode(GATE_OCM_LO);
    TIM2->CCER = TIM_CC4E;
    TIM2->CTLR1 = TIM_CEN;
    GPIOA->BSHR = 1u << 10;
    gpio_cfg_set(&GPIOA->CFGLR, 12u, 0xBu);              /* PA3 AF push-pull (TIM2 CH4) */
    gpio_cfg_set(&GPIOA->CFGHR, 8u, 0x2u);               /* PA10 push-pull */
}

/* closed / open: forced levels */
static inline void upd_forward_set(uint32_t open)
{
    gate_mode(open ? GATE_OCM_HI : GATE_OCM_LO);
}

/* the slot plan: closed now, open at the slot's last falling edge (the next frame's first 256 bits stay here) */
static inline void gate_slot_arm(void)
{
    gate_mode(GATE_OCM_LO);
    TIM2->CNT = 0;
    TIM2->CH4CVR = GATE_SLOT_BITS;
    gate_mode(GATE_OCM_TOGGLE);
}

/* the falling edges since the last re-arm (16 bits) */
static inline uint32_t gate_edges(void)
{
    return TIM2->CNT;
}

/* the slot plan is armed and no edge came since: nothing to re-arm */
static inline uint32_t gate_slot_armed(void)
{
    return TIM2->CNT == 0u && (TIM2->CHCTLR2 >> 12 & 7u) == GATE_OCM_TOGGLE;
}
#endif
