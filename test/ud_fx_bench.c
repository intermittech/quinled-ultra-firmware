/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Entry points for test/test_ud_fx_cycles.py: the Ultra-Digital receiver's local pixel effects (core/ud/udfx.c, wsenc.c, the
 * effect engine) built for the CH32V203 (RV32IMC, xPack GCC 15.2, -Os as the firmware) and run in tools/rv32emu.py (an
 * instruction-class cycle model of the QingKe V4). Static buffers, no start-up code: the emulator calls each function
 * directly. The delay line (dline.c) is not part of it: dline_t is only a place for udfx_tick to read. */
#include <stddef.h>
#include "../src/core/ud/udfx.h"

static dline_t D;
static udfx_t U;
static uint16_t ring[UX_RING_WORDS] __attribute__((aligned(4)));

void *memset(void *d, int c, unsigned n);
void *memcpy(void *d, const void *s, unsigned n);
void *memset(void *d, int c, unsigned n) { uint8_t *p = d; while (n--) *p++ = (uint8_t)c; return d; }
void *memcpy(void *d, const void *s, unsigned n) { uint8_t *p = d; const uint8_t *q = s; while (n--) *p++ = *q++; return d; }

static void fx_slot(uint8_t fx, uint8_t bri, uint8_t seq)
{
    ultra_pixel_fx_t f;
    uint8_t s[32];
    memset(&f, 0, sizeof f);
    f.effect = fx;
    f.speed = 180;
    f.intensity = 160;
    f.palette = ULTRA_PX_PAL_DEFAULT;
    f.c[0][0] = 255; f.c[0][1] = 80;
    f.c[1][2] = 40;
    f.c[2][1] = 200;
    f.brightness = bri;
    ultra_slot_pixel_fx(s, 0, seq, &f);
    udfx_pixel_fx(&U, s, 1000);
}

/* chip, pixels, effect: configured, started, and switched to LOCAL (the line LOW in PASS), the first frame ready */
uint32_t b_setup(uint32_t chip, uint32_t npx, uint32_t fx, uint32_t fps)
{
    static uint8_t seq;
    udfx_init(&U);
    (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, chip);
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, npx);
    (void)udfx_config(&U, ULTRA_CFG_PX_FPS, fps);
    dline_init(&D, 0);
    D.direct = 0;
    D.rec = 0;
    D.wpos = D.zs = D.os = D.zpos = 1000;
    udfx_anchor(&U, 0, 0);
    udfx_service(&U, &D, 1, 0, 256);                 /* (derives the tables) */
    fx_slot((uint8_t)fx, 255, ++seq);
    udfx_service(&U, &D, 2, 0, 256);
    D.wpos += 700;                                   /* the line has been LOW for a latch (the chip's: <= 650 words) */
    udfx_tick(&U, &D, ring);
    return U.local;
}

/* n ticks of 112 / 113 words; the frame repeated (ready kept set). Returns the encoder's bytes taken */
uint32_t b_ticks(uint32_t n)
{
    uint32_t k;
    for (k = 0; k < n; k++) {
        D.wpos += 112u + (k & 1u);
        D.zs = D.wpos - 200u;                        /* (a LOW line) */
        U.e.ready = 1;
        udfx_tick(&U, &D, ring);
    }
    return U.e.frames;
}

/* one LOW tick (the encoder between frames) */
uint32_t b_low_tick(void)
{
    U.e.ready = 0;
    U.e.state = WS_LOW;
    U.e.low_w = 0;
    D.wpos += 113u;
    udfx_tick(&U, &D, ring);
    return U.e.w;
}

/* the whole next frame rendered (the encoder idle: every pixel free); returns the pixels rendered */
uint32_t b_render_frame(void)
{
    uint32_t chunks = 0;
    U.e.ready = 0;
    U.e.state = WS_LOW;
    U.begun = 0;
    U.rendered = 0;
    while (udfx_render(&U)) {
        chunks++;
    }
    return chunks;
}

/* the renderer as main_ud.c's loop calls it (ud-fx-verify's rv_race.c): b_race_tick is one 20 kHz tick of 112 / 113 words
 * (udfx_tick, and udfx_service every 20th), b_race_render the main loop's passes until udfx_render returns 0. In LOCAL the
 * renderer races the beam: it renders pixel j of the next frame once the encoder has taken pixel j of this one */
static uint32_t race_tickno, race_calls;

uint32_t b_race_reset(void)
{
    race_tickno = race_calls = 0;
    return 0;
}

uint32_t b_race_tick(void)
{
    D.wpos += 112u + (race_tickno++ & 1u);
    D.zs = D.wpos - 200u;
    udfx_tick(&U, &D, ring);
    if ((race_tickno % 20u) == 0u) {
        udfx_service(&U, &D, 3u + race_tickno / 20u, 0, 256);
    }
    return U.e.frames;
}

uint32_t b_race_render(void)
{
    uint32_t n = 0;
    while (udfx_render(&U)) {
        n++;
    }
    race_calls += n;                                 /* (the chunks rendered; the pass that finds nothing to do is not counted) */
    return n;
}

uint32_t b_race_frames(void) { return U.frames_done; }
uint32_t b_race_calls(void) { return race_calls; }

/* udfx_service once (a ms) */
uint32_t b_service(uint32_t ms)
{
    D.wpos += 2250u;
    udfx_service(&U, &D, ms, 1200, 256);
    return U.ms;
}

uint32_t b_anchor(uint32_t k)
{
    udfx_anchor(&U, 1000u + k * 1000000u, 100000u + k * 2250000u);
    return U.wq24;
}

uint32_t b_page(uint32_t part)
{
    uint8_t d[24];
    memset(d, 0, sizeof d);
    return (uint32_t)udfx_page(&U, part, d) + d[0];
}

uint32_t b_pixel_fx(uint32_t fx)
{
    static uint8_t seq;
    fx_slot((uint8_t)fx, 255, (uint8_t)(0x40u + ++seq));
    return U.seg[0].run;
}

uint32_t b_fx_count(void)
{
    return FX_COUNT;
}
