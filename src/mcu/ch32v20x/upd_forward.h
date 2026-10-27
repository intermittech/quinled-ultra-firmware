/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The downstream gate of the Diff-Power-UR-Digital-M application (BOARD 3; its port 2): FWD_EN (PA1) is TIM2 CH2's output
 * compare (default mapping: BOARD 3 has no remap), as in the bootloader (bl_hw.c). TIM2 counts DIN's falling edges: external
 * clock mode 1 on ITR2 = TIM3, TIM3 CH1 capturing them (its TRGO per capture, MMS 011, the same trigger that clocks the
 * delay line's TIM4: hw_dline.c). Three modes (OC2M): forced LOW = closed (reset, STREAM, a window, flash work), forced
 * HIGH = open (update transparency), toggle at CNT == 256 = the slot plan (closed for this board's slot, open from bit 257
 * on). No interrupt, no DMA: the edge-to-pin path is the timer's own. DE_OUT (PA10) drives port 2 (push-pull HIGH) except
 * in a relay window (hw.c rc_hw_relay). The caller writes the gate only inside a quiet LOW (it qualifies the input). */
#ifndef MCU_UPD_FORWARD_H
#define MCU_UPD_FORWARD_H
#include "ch32v20x.h"

/* The gate opens only after UPD_FORWARD_QUIET_US (ultra_update.h, 350 us) of LOW with no edge, and the slot plan re-arms
 * only after ULTRA_RW_REARM_US (ultra_protocol.h, 200 us), both measured on the nominal clock: the real clock is never
 * faster, so the real LOW is never shorter. */
#define GATE_OCM_TOGGLE 3u
#define GATE_OCM_LO     4u
#define GATE_OCM_HI     5u
#define GATE_SLOT_BITS  256u                             /* ULTRA_SLOT_BITS: the 256th falling edge ends this board's slot */

static inline void gate_mode(uint32_t ocm)
{
    TIM2->CHCTLR1 = (uint16_t)(ocm << 12);               /* OC2M; CH1 unused (its bits 0) */
}

/* the gate starts with the delay line's counters (hw_dline.c dl_counter_start: TIM2 beside TIM4, closed, then PA1 and
 * DE_OUT; TIM2's clock: hw_cold.c hw_init); another board's image (dl_hw_start not called) keeps the pin table's closed
 * LOW output */
static inline void upd_forward_init(void)
{
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
    TIM2->CH2CVR = GATE_SLOT_BITS;
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
    return TIM2->CNT == 0u && (TIM2->CHCTLR1 >> 12 & 7u) == GATE_OCM_TOGGLE;
}
#endif
