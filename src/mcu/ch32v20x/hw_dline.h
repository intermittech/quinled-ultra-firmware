/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Diff-Power-UR-Digital-M's delay line (BOARD 3; docs/ur_digital_m-firmware.md section 3), the hardware half: one
 * 512-word ring that SPI1 writes (RX, DMA1 ch2) and replays (TX, DMA1 ch3) 227.6 us later, and the bit counter (TIM3
 * captures PA6's falling edges, TIM4 counts them). Long HIGHs, latches and the quiet gap are found in the ring by the
 * 20 kHz tick (core/dline.c; design review 2026-10-15, #3 / #10: no TIM3 thresholds, which a flash operation's SRAM
 * tick couldn't serve); this file only starts, reads and steers the hardware.
 *
 * The primitives are inline (always): the SRAM tick (main_ur_digital_m.c flash_poll_isr -> dl_ram_tick, .ramtext) uses them
 * while the flash is busy, when no call into the flash may happen. dl_ring is in RAM. */
#ifndef HW_DLINE_H
#define HW_DLINE_H

#include <stdint.h>
#include "ch32v20x.h"
#include "dma_map.h"
#include "../profile.h"

#define DL_RING_WORDS   MCU_DLINE_RING_WORDS        /* 8,192 samples at 36 MHz: D = 227.6 us (a power of two: the positions' mask) */
#define DL_RING_GUARD   MCU_DLINE_GUARD_WORDS         /* words after the ring the local-effects encoder may store into (core/ur_digital_m/wsenc.h WS_GUARD):
                                       the DMA circles the first 512 only */

extern uint16_t dl_ring[DL_RING_WORDS + DL_RING_GUARD];

/* the line at boot: the ring zeroed, SPI1 full duplex at 36 MHz with both DMA channels circular over it, PA7 (MOSI)
 * from GPIO LOW to SPI1 once it runs (LOW: the zeroed ring); TIM3 / TIM4 the bit counter; SPI1's error interrupt on.
 * The mux stays as it is (hw_init: direct). After hw_init. */
void dl_hw_start(void);
/* a fault (SPI OVR, a DMA transfer error, the RX / TX distance off): SPI1 reset and the DMA stopped, then the ring
 * zeroed and started again (dl_hw_start's SPI half; the counters run on). The caller holds BLANK_N LOW (core/dline.c:
 * polls never show on the strip): the mux stays as it is, delayed replays the zeroed ring. From one context at a time
 * (it rewrites PA7's mode: a read-modify-write of GPIOA CFGLR, which nothing else writes after boot). */
void dl_hw_restart(void);

/* the app's hook (main_ur_digital_m.c): SPI1's error interrupt (OVR: RX missed a word, the ring is stale), priority above the
 * tick. One call per fault: the interrupt is off (ERRIE 0) until the next dl_hw_start / dl_hw_restart */
void dl_isr_fault(void);

#define DL_INLINE static inline __attribute__((always_inline))
/* the next word the RX DMA writes, 0..511 (its count runs 512 .. 1, then reloads) */
DL_INLINE uint32_t dl_hw_rx_pos(void)
{
    return (DL_RING_WORDS - DMA1_Channel2->CNTR) & (DL_RING_WORDS - 1u);
}
/* the next word the TX DMA reads (in step 2 ahead of RX: the SPI's TX buffer and shift register; 1-3 between reads) */
DL_INLINE uint32_t dl_hw_tx_pos(void)
{
    return (DL_RING_WORDS - DMA1_Channel3->CNTR) & (DL_RING_WORDS - 1u);
}
/* MUX_SEL (PB1): 1 = the strip gets the delayed copy (PA7), 0 = direct (RO) */
DL_INLINE void dl_hw_mux(uint32_t delayed)
{
    GPIOB->BSHR = delayed ? 1u << 1 : 1u << (1 + 16);
}
/* BLANK_N (PA8): 1 = the strip line held LOW, 0 = pass */
DL_INLINE void dl_hw_blank(uint32_t on)
{
    GPIOA->BSHR = on ? 1u << (8 + 16) : 1u << 8;
}
/* the line now (PA6, RO): 1 HIGH */
DL_INLINE uint32_t dl_hw_din(void)
{
    return (GPIOA->INDR >> 6) & 1u;
}
/* PA6's falling edges so far (TIM4, wraps at 65536): one per bit of a WS28xx frame */
DL_INLINE uint16_t dl_hw_edges(void)
{
    return (uint16_t)TIM4->CNT;
}
/* CHAIN: TIM4 back to 0 in the gap before a frame (core/dline.c DLINE_A_EDGES_ZERO): its count is then the frame's bits. TIM4's only
 * writer: a hardware gate (a later phase) re-arms its compare from the same place */
DL_INLINE void dl_hw_edges_zero(void)
{
    TIM4->CNT = 0;
}
/* the live check's wait between reading PA6 and TIM4 (main_ur_digital_m.c): >= 0.1 us (16 cycles, 0.11 us at 144 MHz) */
#define DL_LIVE_DELAY() __asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop" ::: "memory")

#endif
