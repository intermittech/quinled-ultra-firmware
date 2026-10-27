/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Entry points for test/test_ur_digital_m_fx_cycles.py: the Diff-Power-UR-Digital-M's local pixel effects (core/ur_digital_m/udfx.c, wsenc.c, the
 * effect engine) built for the CH32V203 (RV32IMC, xPack GCC 15.2, -Os as the firmware) and run in tools/rv32emu.py (an
 * instruction-class cycle model of the QingKe V4). Static buffers, no start-up code: the emulator calls each function
 * directly. The delay line (dline.c) is not part of it: dline_t is only a place for udfx_tick to read. */
#include <stddef.h>
#include "../src/core/ur_digital_m/udfx.h"

static dline_t D;
static uint16_t CAP[DLINE_CAP_WORDS];              /* dline's capture buffer (dline_init) */
static udfx_t U;
static uint16_t ring[UX_RING_WORDS] __attribute__((aligned(4)));

/* Skipped-grid regression: include a long producer stall and a word-count wrap. */
uint32_t b_grid_setup(uint32_t chip, uint32_t npx, uint32_t fps, uint32_t lag);
uint32_t b_grid_fill(void)
{
    (void)ws_ring_fill(&U.e, &ux_lut, ring, U.e.w + 24u);
    return U.e.next_w;
}

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
    ultra_slot_pixel_fx(s,0, 0, seq, &f);
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
    dline_init(&D, 0, CAP);
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

uint32_t b_grid_setup(uint32_t chip, uint32_t npx, uint32_t fps, uint32_t lag)
{
    b_setup(chip, npx, 1, fps);
    U.e.next_w = 0xFFFF0000u;
    U.e.w = U.e.next_w + lag;
    U.e.low_w = U.e.reset_w;
    U.e.state = WS_LOW;
    U.e.ready = 1;
    return U.npx;
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

/* the renderer as main_ur_digital_m.c's loop calls it (ud-fx-verify's rv_race.c): b_race_tick is one 20 kHz tick of 112 / 113 words
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

/* ---- CHAIN mode: the chain scanner in dline_tick (core/dline.c dl_ch_scan) over a 1-slot Ultra frame (320 bits of 45 samples),
 * ticks of 112 / 113 words: the idle line, the frame's ticks, the tick that finds its end. The stream scanner runs beside it. */
static uint16_t cw[1100];
static uint32_t cw_n, ch_k;

uint32_t b_ch_setup(void)
{
    uint32_t i, b, s = 0, x = 0x1234567u;
    for (i = 0; i < sizeof cw / sizeof cw[0]; i++) {
        cw[i] = 0;
    }
    for (b = 0; b < 320u; b++) {
        uint32_t hi, j;
        x = x * 1103515245u + 12345u;
        hi = (x >> 16) & 1u ? 25u : 12u;
        for (j = 0; j < 45u; j++, s++) {
            if (j < hi) {
                cw[s >> 4] |= (uint16_t)(0x8000u >> (s & 15u));
            }
        }
    }
    cw_n = (s + 15u) / 16u;
    dline_init(&D, 0, CAP);
    D.direct = 0;
    D.rec = 0;
    D.ch_on = 1;
    D.ch_st = DLINE_CH_IDLE;
    D.ch_lz = 200;
    D.local = 1;
    ch_k = 0;
    return cw_n;
}

/* one tick: the next 112 / 113 words are cw[off ..] (or LOW past the frame); returns the tick's actions */
uint32_t b_ch_tick(uint32_t off)
{
    const uint32_t n = 112u + (ch_k++ & 1u);
    uint32_t k, rx, a;
    for (k = 0; k < n; k++) {
        ring[(D.wpos + k) & DLINE_RING_MASK] = off + k < cw_n ? cw[off + k] : 0;
    }
    rx = (D.wpos + n) & DLINE_RING_MASK;
    a = dline_tick(&D, ring, rx, (rx + 2u) & DLINE_RING_MASK, 320u);
    if (a & DLINE_A_CH_READY) {
        D.ch_ready = 0;
    }
    return a;
}

uint32_t b_ch_words(void) { return cw_n; }

uint32_t b_ch_on(uint32_t on)
{
    D.ch_on = (uint8_t)(on != 0u);
    return D.ch_on;
}

/* one udfx_render call (the main loop's step): its cycle count is the latency a decode or a poll answer can wait behind */
uint32_t b_render_once(void)
{
    return udfx_render(&U);
}

/* Queue four scene commands before a frame, so setup latency is measured too. */
uint32_t b_scene_commands(uint32_t fx)
{
    ultra_pixel_fx_t f = {0};
    uint8_t s[32];
    uint32_t i;
    f.effect = (uint8_t)fx; f.palette = ULTRA_PX_PAL_COLORS; f.brightness = 255;
    f.c[0][0] = 255; f.c[1][1] = 255; f.c[2][2] = 255;
    for (i = 0; i < UX_SEGS; i++) {
        ultra_slot_pixel_fx(s,0, (uint8_t)i, 99, &f);
        udfx_pixel_fx(&U, s, 1000);
    }
    U.e.ready = 0; U.e.state = WS_LOW; U.begun = 0;
    return 0;
}

uint32_t b_preparing(void) { return U.begun != 1u; }

/* Fire -> Rainbow, Rainbow -> Fire, Fire palette, or Fire -> Pacifica.
 * Queue through the controller so the measured calls include command setup. */
uint32_t b_fire_change(uint32_t mode)
{
    ultra_pixel_fx_t f = {0};
    uint8_t s[32];
    f.effect = (uint8_t)(mode == 1 ? FX_RAINBOW : mode == 4 ? FX_PACIFICA : FX_FIRE);
    f.palette = mode == 3 ? PAL_LAVA : PAL_DEFAULT;
    f.speed = 255; f.intensity = 255; f.custom1 = 55; f.brightness = 255;
    f.transition_ms = 2000;
    f.start_ms = -500;                         /* four-step catch-up on entry */
    ultra_slot_pixel_fx(s,0, 0, (uint8_t)(100u + mode), &f);
    udfx_pixel_fx(&U, s, 1000);
    return 0;
}

uint32_t b_fire_four(void)
{
    fx_params_t p = {0};
    uint32_t i;
    p.id = FX_FIRE; p.speed = 255; p.intensity = 255; p.custom1 = 55;
    p.palette = PAL_HEAT; p.bri = 255;
    for (i = 0; i < FX_MAX_SEG; i++) {
        fx_engine_set(&ux_eng, i, &p, ux_eng.now, ux_eng.now);
        while (fx_engine_step(&ux_eng)) { }
    }
    return 0;
}

uint32_t b_scene_at(uint32_t ms)
{
    U.e.ready = 0; U.e.state = WS_LOW; U.begun = 0;
    U.e.next_w = U.a_w = 0; U.a_host = ms * 1000u;
    return 0;
}

/* Both slots occupied; the next controller command must queue, not replace either context. */
uint32_t b_scene_fades(uint32_t fx)
{
    fx_params_t p = {0};
    uint32_t i;
    p.bri = 255; p.palette = PAL_COLORS; p.c[0].r = 255;
    for (i = 0; i < 2; i++) {
        p.id = FX_SOLID; p.trans_ms = 0;
        fx_engine_set(&ux_eng, i, &p, 0, 0);
        p.id = (uint8_t)fx; p.trans_ms = 65535;
        fx_engine_set(&ux_eng, i, &p, 0, 0);
    }
    fx_engine_frame(&ux_eng, 0);
    return 0;
}
