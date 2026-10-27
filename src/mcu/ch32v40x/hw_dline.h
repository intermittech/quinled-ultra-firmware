/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Diff-Power-UR-Digital-M's delay line on the CH32V407, the hardware half (docs/ur_digital_m-firmware.md section 3):
 * one 512-word ring that SPI1 writes (RX, DMA1 ch2) and replays (TX, DMA1 ch3) 512 words later (16 samples a word at
 * PCLK2 / 2 = 50 MHz: 163.84 us), and the bit counter (TIM3 CH1 captures RO's falling edges on PB4, TIM4 counts them).
 * The strip's source: the mux (MUX_SEL PB6: 0 = RO direct, 1 = PX_SRC) after the source selector (SOURCE_SEL PB7:
 * 0 = the replay on PB5, 1 = ARGB_OUT on PA15), then the AND with BLANK_N (PA8). The window logic is core/dline.c; this file starts, reads and
 * steers the hardware. The primitives are inline: the SRAM tick uses them while the flash is busy. */
#ifndef HW_DLINE_H
#define HW_DLINE_H

#include <stdint.h>
#include "ch32v40x.h"
#include "dma_map.h"
#include "../profile.h"

#define DL_RING_WORDS   MCU_DLINE_RING_WORDS
#define DL_RING_GUARD   MCU_DLINE_GUARD_WORDS

extern uint16_t dl_ring[DL_RING_WORDS + DL_RING_GUARD];

void dl_hw_start(void);
void dl_hw_restart(void);
void dl_isr_fault(void);

#define DL_INLINE static inline __attribute__((always_inline))
DL_INLINE uint32_t dl_hw_rx_pos(void)
{
    return (DL_RING_WORDS - MCU_DMA_SPI_RX->CNTR) & (DL_RING_WORDS - 1u);
}
/* the free-running word count now, from a count w of this tick (less than a ring ago), one word ahead: the word in
 * progress counts as received, so the result is never earlier than the moment of the read */
DL_INLINE uint32_t argb_hw_now(uint32_t w)
{
    return w + ((dl_hw_rx_pos() - w) & (DL_RING_WORDS - 1u)) + 1u;
}
DL_INLINE uint32_t dl_hw_tx_pos(void)
{
    return (DL_RING_WORDS - MCU_DMA_SPI_TX->CNTR) & (DL_RING_WORDS - 1u);
}
/* MUX_SEL (PB6): 1 = the strip gets PX_SRC (the replay or the ARGB), 0 = direct (RO) */
DL_INLINE void dl_hw_mux(uint32_t delayed)
{
    GPIOB->BSHR = delayed ? 1u << 6 : 1u << (6 + 16);
}
/* SOURCE_SEL (PB7): 1 = ARGB_OUT (local effects), 0 = the replay */
DL_INLINE void dl_hw_source(uint32_t argb)
{
    GPIOB->BSHR = argb ? 1u << 7 : 1u << (7 + 16);
}
/* BLANK_N (PA8): 1 = the strip line held LOW, 0 = pass */
DL_INLINE void dl_hw_blank(uint32_t on)
{
    GPIOA->BSHR = on ? 1u << (8 + 16) : 1u << 8;
}
/* the line now (PB4, RO): 1 HIGH */
DL_INLINE uint32_t dl_hw_din(void)
{
    return (GPIOB->INDR >> 4) & 1u;
}
DL_INLINE uint16_t dl_hw_edges(void)
{
    return (uint16_t)TIM4->CNT;
}
DL_INLINE void dl_hw_edges_zero(void)
{
    TIM4->CNT = 0;
}
/* The ARGB serialiser (RM ch.28 p.507-513) on PA15, fed by DMA1 ch10 (8-bit, RM p.147-148, 508). Timing in HCLK
 * cycles (DAT_ARR bit period, CCRL / CCRH high times; "system clock cycles" = HCLK here: SYSCLK = HCLK). MODE 0: a
 * short RESET code (LOW) leads the data; the cycle ends with the last bit (FRHIF, polled by the tick: it also runs in
 * the flash operation's SRAM tick, where no other interrupt may enter). PA15 is the ARGB's only during a frame and a
 * GPIO LOW otherwise, so the idle level is defined whatever the serialiser's (BENCH: its idle / reset level, whether
 * the counts are N or N + 1 cycles). */
#define ARGB_LEAD_CYC 200u                               /* 1 us LOW before the data */
/* DMA1 ch10's flags in EXTEM_INTFR / EXTEM_INTFCR (RM p.153-156: channel x >= 8 at bits 4 (x - 8)): GIF, TCIF, HTIF,
 * TEIF at 8..11 */
#define ARGB_DMA_FLAGS (0xFu << 8)
#define ARGB_DMA_TEIF  (1u << 11)
/* argb_hw_status */
#define ARGB_ST_RUN  0u
#define ARGB_ST_OK   1u
#define ARGB_ST_FAIL 2u
DL_INLINE void argb_hw_start(const uint8_t *buf, uint32_t n, uint32_t period, uint32_t t0h, uint32_t t1h)
{
    MCU_DMA_ARGB->CFGR = 0;
    ARGB->CTLR = 0;
    ARGB->STATR = 0x1Fu;                                 /* FRHIF..HALFIF: write 1 to clear */
    DMA1_EXT->INTFCR = ARGB_DMA_FLAGS;                   /* (a previous frame's transfer error) */
    ARGB->DAT_ARR = period;
    ARGB->CCRL = t0h;
    ARGB->CCRH = t1h;
    ARGB->RST_ARR = ARGB_LEAD_CYC;
    ARGB->DAT_CYC = n - 1u;
    MCU_DMA_ARGB->PADDR = (uint32_t)&ARGB->DATAR;
    MCU_DMA_ARGB->MADDR = (uint32_t)buf;
    MCU_DMA_ARGB->CNTR = n;
    MCU_DMA_ARGB->CFGR = DMA_CFGR1_DIR | DMA_CFGR1_MINC | DMA_CFGR1_PL_0 | DMA_CFGR1_EN;   /* 8-bit, medium priority */
    ARGB->CTLR = ARGB_CTLR_DMAEN;                        /* MODE 0, MSB first */
    gpio_cfg_set(&GPIOA->CFGHR, 28u, 0xBu);              /* PA15: AF push-pull (atomic: PA9 shares the word) */
    ARGB->CTLR = ARGB_CTLR_DMAEN | ARGB_CTLR_TE;
}
/* the frame's bytes the DMA has not read yet */
DL_INLINE uint32_t argb_hw_left(void)
{
    return MCU_DMA_ARGB->CNTR;
}
/* The frame: running, sent, or failed. The end of cycle (FRHIF) alone is no success (RM p.513: it marks the cycle's
 * end; p.507-508: without new bytes the serialiser repeats stale ones): every byte read (CNTR = 0) and no transfer
 * error (TEIF: the DMA disables the channel, RM p.142-143). A transfer error ends the frame at once. */
DL_INLINE uint32_t argb_hw_status(void)
{
    if (DMA1_EXT->INTFR & ARGB_DMA_TEIF) {
        return ARGB_ST_FAIL;
    }
    if (!(ARGB->STATR & ARGB_STATR_FRHIF)) {
        return ARGB_ST_RUN;
    }
    return MCU_DMA_ARGB->CNTR == 0u ? ARGB_ST_OK : ARGB_ST_FAIL;
}
DL_INLINE void argb_hw_stop(void)
{
    gpio_cfg_set(&GPIOA->CFGHR, 28u, 0x3u);              /* PA15: GPIO output (its OUTDR bit is 0: LOW) */
    ARGB->CTLR = 0;
    MCU_DMA_ARGB->CFGR = 0;
    DMA1_EXT->INTFCR = ARGB_DMA_FLAGS;
}

/* the live check's wait between reading the pin and TIM4 (main_ur_digital_m.c): >= 0.1 us, past TIM3's input filter
 * (2 x 5 ns) and synchroniser: 24 cycles at 200 MHz = 0.12 us */
#define DL_LIVE_DELAY() __asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; " \
                                       "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop" ::: "memory")

#endif
