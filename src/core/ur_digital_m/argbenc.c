/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The local-effects encoder of the Diff-Power-UR-Digital-M on the CH32V40x: wsenc.h's interface and time base (ring
 * words: the SPI's free-running RX count, 64 HCLK cycles each) with the ARGB serialiser as the output instead of ring
 * words. The strip gets it through the source selector (SOURCE_SEL = 1); the replay keeps the ring (core/dline.c)
 * unchanged.
 *
 * A frame starts at the first tick (or SRAM tick) at or after its start word: a whole reset of LOW behind the last frame
 * and the frame grid's word (period_w), the renderer's frame ready (as wsenc.c). Its bytes go out by DMA. The times are
 * the hardware's, not the tick's snapshot (limit, taken before the scan: the serialiser starts later in the tick, by a
 * varying amount): the start is the RX count read just after the start (argb_hw_now: never early), the end the later of
 * start + the exact duration (the lead + bytes x 8 x bit period, in HCLK cycles = words x 64) and the moment the
 * serialiser's end of cycle is seen (with every byte read and no DMA error: else the frame is lost, as one that never
 * ends: the renderer makes it again, after a whole reset); the LOW counts from that end to each tick's snapshot, so it never overstates the
 * line's real LOW (a frame starts, and udfx switches the source back, only on a LOW at least the chip's reset). A cut
 * frame (ws_enc_init, a lost one) counts its LOW from the moment of the stop. Bit timing: the chip's 10 ns symbols
 * (udfx's table) at the clock estimate ws_argb_q16, applied at each frame start (a clock change takes effect between
 * frames). Bytes the DMA
 * has read are free for the renderer (e->i). Integer only, no division in the per-frame path but the grid's (divu);
 * ws_ring_fill is SRAM code. */
#include <stddef.h>
#include "wsenc.h"
#ifdef WS_ARGB_HW
#include WS_ARGB_HW                                /* a model of the serialiser (test/test_argbenc_v407.py) */
#else
#include "hw_dline.h"
#endif

#ifndef WS_RAMCODE
#if defined(__GNUC__) && defined(__riscv)
#define WS_RAMCODE MCU_RAMCODE
#else
#define WS_RAMCODE
#endif
#endif

/* HCLK cycles per 10 ns, Q16, at the boot clock: nominal 200 MHz lowered by the boot trim (clock.h) */
#define WS_ARGB_Q16_BOOT ((uint32_t)((uint64_t)(MCU_HCLK_HZ / 100000u) * 65536u * \
                                     (1000000u - MCU_HSI_BOOT_TRIM_STEPS * MCU_HSI_TRIM_PPM_STEP) / 1000000000u))
volatile uint32_t ws_argb_q16 = WS_ARGB_Q16_BOOT;
#define WS_CYC_PER_WORD (MCU_HCLK_HZ / WS_SAMPLE_HZ * MCU_SAMPLE_BITS)   /* 64: SPI1 at HCLK / 4, 16 samples a word */
#define WS_DONE_LATE_W  MCU_WORDS_MS                     /* no end-of-cycle 1 ms after the computed end: a fault */

void ws_lut_build(ws_lut_t *L, const ws_chip_t *c)
{
    L->s = c->s;
    L->t0h = c->t0h;
#if defined(__GNUC__)
    __asm volatile("" ::: "memory");               /* (two byte copies, never one halfword access at an odd address) */
#endif
    L->t1h = c->t1h;
}

#if defined(__GNUC__)
__attribute__((noinline))
#endif
void ws_enc_init(ws_enc_t *e, const ws_chip_t *c, const uint8_t *fb, uint32_t npx, uint32_t w0, uint32_t period_w)
{
    argb_hw_stop();                                      /* (a frame still running is cut: its pin LOW) */
    e->end_w = argb_hw_now(w0);                          /* (the LOW from the stop: w0 is this tick's count) */
    e->fb = fb;
    e->fb_next = fb;
    e->in_flight = 0;
    e->nbytes = npx * c->bpp;
    e->i = 0;
    e->w = w0;
    e->low_w = 0;                                        /* (the first frame waits a whole reset: a clean latch) */
    e->next_w = w0 + c->reset_w;
    e->period_w = period_w;
    e->frames = 0;
    e->start_w = w0;
    e->reset_w = c->reset_w;
    e->state = WS_LOW;
    e->ready = 0;
    e->late = 0;
    e->lost = 0;
}

static inline __attribute__((always_inline)) void ws_sat8(uint8_t *c)
{
    if (*c != 0xFFu) {
        (*c)++;
    }
}

static inline __attribute__((always_inline)) uint32_t ws_cyc(uint32_t sym, uint32_t q16)
{
    return (sym * q16 + 32768u) >> 16;
}

WS_RAMCODE uint32_t ws_ring_fill(ws_enc_t *e, const ws_lut_t *L, uint16_t *buf, uint32_t limit)
{
    const uint32_t w0 = e->w;
    (void)buf;
    if (e->state == WS_FRAME) {
        const uint32_t st = argb_hw_status();
        e->i = e->nbytes - argb_hw_left();               /* (the DMA's read bytes: their pool bytes are free) */
        if (st == ARGB_ST_OK) {
            const uint32_t now = argb_hw_now(limit);     /* (seen now: the real end is no later) */
            argb_hw_stop();
            e->in_flight = 0;                            /* (its frame buffer is free for the renderer) */
            e->state = WS_LOW;
            e->i = e->nbytes;
            if ((int32_t)(now - e->end_w) > 0) {
                e->end_w = now;
            }
            e->low_w = 0;
        } else if (st == ARGB_ST_FAIL || (int32_t)(limit - e->end_w) > (int32_t)WS_DONE_LATE_W) {
            argb_hw_stop();                              /* failed, or no end of cycle: lost, a full reset follows */
            e->in_flight = 0;
            ws_sat8(&e->lost);
            e->state = WS_LOW;
            e->end_w = argb_hw_now(limit);
            e->low_w = 0;
            e->i = 0;
            e->ready = 0;
        } else {
            e->w = limit;
            return 0;
        }
    }
    /* the LOW up to this snapshot, from end_w (an upper bound of the real end); saturating */
    if (e->low_w != 0u) {
        const uint32_t d = limit - w0;
        e->low_w = e->low_w + d < e->low_w ? 0xFFFFFFFFu : e->low_w + d;
    } else if ((int32_t)(limit - e->end_w) > 0) {
        e->low_w = limit - e->end_w;
    }
    if (e->ready && e->nbytes != 0u && e->low_w >= e->reset_w &&
        (e->period_w == 0u || (int32_t)(limit - e->next_w) >= 0)) {
        const uint32_t q = ws_argb_q16, per = ws_cyc(L->s, q);
        if (e->period_w != 0u) {                         /* the grid: the next frame's word (late: the next one) */
            if ((int32_t)(limit - e->next_w) > (int32_t)(WS_W_PER_MS / 2u)) {
                ws_sat8(&e->late);
            }
            e->next_w += ((limit - e->next_w) / e->period_w + 1u) * e->period_w;
        } else {
            e->next_w = limit;
        }
        e->fb = e->fb_next;                          /* (the handed-over frame: ready was set after fb_next) */
        e->in_flight = e->fb;
        argb_hw_start(e->fb, e->nbytes, per, ws_cyc(L->t0h, q), ws_cyc(L->t1h, q));
        e->start_w = argb_hw_now(limit);                 /* (read after the start: never earlier than it) */
        e->ready = 0;
        e->state = WS_FRAME;
        e->i = 0;
        e->end_w = e->start_w + (ARGB_LEAD_CYC + e->nbytes * 8u * per + WS_CYC_PER_WORD - 1u) / WS_CYC_PER_WORD + 1u;
        e->frames++;
    }
    e->w = limit;
    return limit - w0;
}
