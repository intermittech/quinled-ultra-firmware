/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Ultra-Digital receiver's local pixel effects (core/ud/udfx.c + wsenc.c + the real core/dline.c, BOARD 3) on a word-exact
 * model of the delay ring: RX DMA writes word k into ring[k % 512] at time k, TX DMA reads slot (k + 2) % 512 at time k, so
 * word W leaves at time W + 510; the 20 kHz tick (112.5 words apart, late by a jitter) runs dline_tick and udfx_tick
 * exactly as main_ud.c's dl_tick does; the main loop between ticks renders, services and decodes. The strip's wire is
 * decoded sample by sample: every frame must be a whole one with exact pulses.
 *
 *   test\build_udfx.cmd [outdir] [scenario-prefix]     (MSVC; `*` or none: all)
 *
 * Checks (per scenario): the strip gets exactly the rendered frames (bytes, pulse widths, resets >= the chip's); the encoder
 * never writes a word RX hasn't written, never touches a slot outside [w0, rx); every word the encoder replaces was read raw by
 * dline's scan first; frames start on the host grid; stream <-> local only at latches (whole frames, hub or local); a marker +
 * poll + release in local mode never shows; a flash operation (SRAM tick only) loses nothing; faults recover. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/dline.h"
#include "../src/core/ud/udfx.h"
#include "../src/core/ultra_protocol.h"

#define M        DLINE_RING_MASK
#define MAXW     (2250u * 400u)
#define MAXW_LONG (2250u * 2600u)
#define MAXFR    1024
#define MAXB     (1024 * 5)

static int g_fail, g_quiet;
static long g_checks;
static const char *g_name = "";
static int g_printed;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; if (g_printed++ < 14) { printf("  FAIL %s:%d [%s] ", __FILE__, \
    __LINE__, g_name); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static uint32_t rng = 12345u;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* ------------------------------------------------------------------------------------------------ the world */
#if defined(_MSC_VER)
__declspec(align(4)) static uint16_t ring[UX_RING_WORDS];
#else
static uint16_t ring[UX_RING_WORDS] __attribute__((aligned(4)));
#endif
static uint16_t *line;                  /* the hub's drive of the data line (what RX samples when DE is low) */
static uint16_t *out;                   /* the strip's wire, word W as it leaves at time W + 510 */
static uint32_t *owned_t;               /* the time the encoder wrote this word (0: never) */
static uint32_t cap_w;                  /* the words the three hold (less 64) */
static uint8_t  exp_fb[MAXFR][MAXB];    /* the frames as they were handed to the encoder (fb at `ready`) */
static uint32_t n_exp;
static uint32_t enc_from, enc_to;       /* the words the encoder owned: [from, to) */

static dline_t D;
static udfx_t U;

/* the glue's pins and the hub's side of the window */
static uint32_t g_blank, g_mux, g_de;
static uint32_t de_end_w, arm_at, g_ready_pending;
static uint32_t g_answer;               /* the window's armed answer (1 yes) */
static uint32_t n_sends, n_ready, sum_blank_local;

/* ------------------------------------------------------------------------------------------------ the line */
static uint32_t ln;
static void run(int level, uint32_t len)
{
    while (len--) {
        if (level) {
            line[ln >> 4] |= (uint16_t)(0x8000u >> (ln & 15u));
        }
        ln++;
    }
}
static void hub_frame(const uint8_t *b, uint32_t n, uint32_t s, uint32_t t0h, uint32_t t1h)
{
    uint32_t i, k;
    for (i = 0; i < n; i++) {
        for (k = 0; k < 8u; k++) {
            uint32_t h = (b[i] >> (7u - k)) & 1u ? t1h : t0h;
            run(1, h);
            run(0, s - h);
        }
    }
}
static uint32_t g_dec_latch = 104u * 16u;   /* the strip's latch in samples (the default: dline's 46 us; a V5 strip's is 281 us) */
static uint32_t g_hub_period = 2250u * 3u;   /* the controller's frame period, words */
static uint32_t g_line_ms = 120;        /* build_switch_line's length */
static uint32_t g_hole_us;              /* a controller's frame holds a LOW this long after its 60th byte (an RMT refill), 0: none */
/* a controller's frame of 120 bytes, with the refill's LOW in the middle when g_hole_us is set */
static void ctl_frame(const uint8_t *b)
{
    if (g_hole_us != 0u) {
        hub_frame(b, 60, 45, 14, 29);
        run(0, g_hole_us * 36u);
        hub_frame(b + 60, 60, 45, 14, 29);
    } else {
        hub_frame(b, 120, 45, 14, 29);
    }
}
static void low_until_word(uint32_t w) { if (ln < w * 16u) run(0, w * 16u - ln); }
static void line_clear(void) { memset(line, 0, (cap_w + 64u) * sizeof *line); ln = 0; }
/* room for `words` ring words (a long scenario reserves more, then goes back to the default) */
static void world_reserve(uint32_t words)
{
    free(line); free(out); free(owned_t);
    line = (uint16_t *)calloc(words + 64u, sizeof *line);
    out = (uint16_t *)calloc(words + 64u, sizeof *out);
    owned_t = (uint32_t *)calloc(words + 64u, sizeof *owned_t);
    if (!line || !out || !owned_t) {
        printf("out of memory\n");
        exit(3);
    }
    cap_w = words;
}

/* ------------------------------------------------------------------------------------------------ the decoder */
typedef struct {
    uint32_t start_w, nbits, low_before, exact, closed, sw;   /* sw: start sample; exact: bit i = every pulse of timing i */
    uint8_t  b[MAXB];
} dfr_t;
static dfr_t fr[MAXFR];
typedef struct { uint32_t s, t0h, t1h; } tim_t;
static tim_t g_tim[2];                  /* the timings a pulse may be exact to (bit 0, bit 1 of `exact`) */
static uint32_t g_ntim;

static uint32_t decode_all(const uint16_t *w, uint32_t w0, uint32_t w1, uint32_t s, uint32_t t0h, uint32_t t1h)
{
    if (g_ntim == 0u) {
        g_tim[0].s = s; g_tim[0].t0h = t0h; g_tim[0].t1h = t1h; g_ntim = 1;
    }
    const uint32_t latch = g_dec_latch;
    uint32_t n = 0, x, lvl = 0, len = 0, low = latch, hi = 0, inframe = 0, i;
    dfr_t *f = 0;
    for (x = w0 * 16u; x <= w1 * 16u; x++) {
        uint32_t v = x < w1 * 16u ? (w[x >> 4] >> (15u - (x & 15u))) & 1u : 2u;
        if (v == lvl) {
            len++;
            continue;
        }
        if (lvl == 1u) {
            hi = len;
            if (!inframe) {
                if (n == MAXFR) {
                    break;
                }
                f = &fr[n++];
                memset(f, 0, sizeof *f);
                f->start_w = (x - len) >> 4;
                f->sw = x - len;
                f->low_before = low;
                f->exact = (1u << g_ntim) - 1u;
                inframe = 1;
            }
            if (f->nbits < MAXB * 8u && hi > (g_ntim != 0u ? (g_tim[0].t0h + g_tim[0].t1h) / 2u : (t0h + t1h) / 2u)) {
                f->b[f->nbits >> 3] |= (uint8_t)(0x80u >> (f->nbits & 7u));
            }
            for (i = 0; i < g_ntim; i++) {
                if (!(hi == g_tim[i].t0h || hi == g_tim[i].t1h)) {
                    f->exact &= ~(1u << i);
                }
            }
            f->nbits++;
        } else if (lvl == 0u && inframe) {
            if (len >= latch || v == 2u) {
                inframe = 0;
                f->closed = len >= latch;
            } else {
                for (i = 0; i < g_ntim; i++) {
                    if (len != g_tim[i].s - hi) {
                        f->exact &= ~(1u << i);
                    }
                }
            }
        }
        if (lvl == 0u) {
            low = len;
        }
        lvl = v;
        len = 1;
    }
    return n;
}
static uint32_t decode(const uint16_t *w, uint32_t w0, uint32_t w1, uint32_t s, uint32_t t0h, uint32_t t1h)
{
    uint32_t n;
    g_ntim = 0;
    n = decode_all(w, w0, w1, s, t0h, t1h);
    while (n > 0u && !fr[n - 1u].closed) {
        n--;
    }
    return n;
}

/* ------------------------------------------------------------------------------------------------ the chips (oracle) */
typedef struct { const char *name; uint32_t s, t0h, t1h, bpp, reset_us; } chip_o_t;
/* the study's presets (STUDY.md 1.2, tools/ud_timing_check.py): bit period, HIGH of a 0 / a 1 in samples, and the least
 * LOW a latch needs, us */
static const chip_o_t CHIP[ULTRA_PXC_COUNT] = {
    { "WS2812B V5", 48, 13, 25, 3, 281 },
    { "WS2812B classic", 48, 13, 28, 3, 100 },
    { "SK6812 RGB", 48, 12, 26, 3, 201 },
    { "SK6812 RGBW", 48, 12, 26, 4, 201 },
    { "WS2814 RGBW", 48, 13, 25, 4, 281 },
    { "WS2805 RGBCCT", 48, 13, 25, 5, 281 },
    { "WS2815", 48, 13, 25, 3, 281 },
    { "WS2811 400k", 90, 18, 43, 3, 100 },
};

/* ------------------------------------------------------------------------------------------------ the receiver */
typedef struct {
    uint32_t T;                         /* words simulated */
    uint32_t gap_max;                   /* the tick's lateness, 0 .. gap_max words (113 + 112: up to 60 us) */
    uint32_t long_gap_at, long_gap;     /* one long gap between ticks (words) at this word */
    uint32_t flash_from, flash_to;      /* a flash operation: the SRAM tick only, the main loop stalled */
    int      steps;                     /* render steps per tick (the main loop's share) */
    int      no_main;                   /* the main loop never runs */
    uint32_t main_div;                  /* the main loop runs every this many ticks (0, 1: every tick) */
    double   hsi;                       /* the receiver's clock fast by this fraction (host us per word shrinks) */
    uint32_t phase;                     /* the ring's position at host time 0 (words) */
    void   (*cmd)(uint32_t k);          /* the main loop's scripted work, once a ms */
} sim_t;
static const sim_t *SIM;
static uint32_t g_fault_pend, g_fault_at;  /* an SPI OVR injected at this word (the glue's dl_fault_pend) */
static uint32_t g_step;                 /* a scenario's script position (reset by simulate) */

/* the host clock (us, 32 bit) at ring word w */
static uint32_t host_of_word(uint32_t w)
{
    return (uint32_t)(int64_t)floor(((double)(int32_t)(w - SIM->phase)) * (16.0 / 36.0) / (1.0 + SIM->hsi) + 0.5);
}
static uint32_t word_of_host(uint32_t us)
{
    return SIM->phase + (uint32_t)(int64_t)floor((double)(int32_t)us * 2.25 * (1.0 + SIM->hsi) + 0.5);
}

static void apply(uint32_t a, uint32_t k)
{
    if (a & DLINE_A_BLANK_ON) {
        g_blank = 1;
    }
    if (a & DLINE_A_MUX_DELAYED) {
        g_mux = 1;
    }
    if ((a & DLINE_A_BLANK_OFF) && !D.restart) {
        g_blank = 0;
    }
    if (a & DLINE_A_FRAME_READY) {
        arm_at = k + 150u + (rnd() % 300u);               /* the main loop decodes 70 .. 200 us later */
        g_ready_pending = 1;
        n_ready++;
    }
    if (a & DLINE_A_SEND) {
        if (udfx_answer_blank(&U)) {                      /* (main_ud.c: rc_hw_dir pulls BLANK_N with DE; in local mode DE only) */
            g_blank = 1;
        }
        g_de = 1;
        de_end_w = k + 1u + 1440u;                        /* the burst: ~640 us */
        n_sends++;
    }
}

static void restart_ring(uint32_t k)
{
    memset(ring, 0, sizeof ring);
    dline_restart(&D, (k + 1u) & M);
    udfx_restarted(&U, &D);
}

/* one 20 kHz tick at time k (RX has written the words below k + 1) */
static void tick(uint32_t k, uint32_t ram)
{
    const uint32_t rx = (k + 1u) & M;
    uint32_t a, w0 = U.e.w, local = U.local;
    uint16_t shadow[UX_RING_WORDS];
    memcpy(shadow, ring, sizeof ring);
    if (g_fault_pend && !ram) {                           /* main_ud.c dl_tick: the OVR path restarts the ring and returns */
        g_fault_pend = 0;
        g_blank = 1;
        a = dline_fault(&D);
        apply(a, k);
        if (a & DLINE_A_RESTART) {
            restart_ring(k);
        }
        return;
    }
    if (ram) {
        a = dline_ram_tick(&D, ring, rx, (rx + 2u) & M, 0);
        apply(a, k);
        if (!D.restart) {
            udfx_ram_tick(&U, ring, D.wpos);
        }
    } else {
        a = dline_tick(&D, ring, rx, (rx + 2u) & M, 0);
        apply(a, k);
        if (a & DLINE_A_RESTART) {
            restart_ring(k);
        }
        udfx_tick(&U, &D, ring);
    }
    if (!local && U.local) {
        enc_from = D.wpos;                                /* the encoder owns the words from here ... */
        enc_to = 0xFFFFFFFFu;
    }
    if (local && !U.local) {
        enc_to = D.wpos;                                  /* ... to here */
    }
    if (local && U.local) {                               /* the encoder wrote [w0, e.w): check what it touched */
        uint32_t j, s, e_w = U.e.w;
        CHECK((int32_t)(D.wpos - e_w) >= 0, "the encoder ran ahead of RX: e.w %u, rx %u", e_w, D.wpos);
        for (j = D.wpos; j != w0 + 512u && (int32_t)(w0 + 512u - j) > 0; j++) {   /* slots of words outside [w0, rx) */
            s = j & M;
            CHECK(ring[s] == shadow[s], "overrun: the slot of word %u written (e.w %u, rx %u)", j, w0, D.wpos);
            if (ring[s] != shadow[s]) {
                break;
            }
        }
        for (j = w0; (int32_t)(e_w + 3u - j) > 0 && (int32_t)(D.wpos - j) > 0; j++) {
            if (owned_t[j] == 0u) {
                owned_t[j] = k + (D.wpos - w0) / 10u + 25u;   /* (the scan before the fill: ~0.1 word a word of the tick, the interrupt ~25) */
            }
        }
    }
}

static void main_loop(uint32_t k, uint32_t ms_tick)
{
    uint32_t i;
    if (g_ready_pending && k >= arm_at) {                 /* the capture decoded: arm the window's answer */
        dline_arm(&D, (int)g_answer);
        g_ready_pending = 0;
    }
    for (i = 0; i < (uint32_t)SIM->steps; i++) {
        uint32_t was_ready = U.e.ready;
        uint32_t was_begun = U.begun, more;
#ifdef UDFX_SCENE_TEST
        scene_hook(0);
#endif
        more = udfx_render(&U);
#ifdef UDFX_SCENE_TEST
        scene_hook(1);
#endif
        if (!was_ready && U.e.ready && n_exp < MAXFR) {   /* a frame handed over: keep its bytes */
            memcpy(exp_fb[n_exp++], ux_pool, U.npx * U.chip.bpp);
        }
        /* steps is a pixel-chunk quota, not a CPU model. Drain separate setup
         * yields; the compiled chain-budget test charges every call's cycles. */
        if (was_begun != 1u && more) i--;
    }
    if (ms_tick) {
        udfx_service(&U, &D, k / 2250u, 0, 256u);
        if (SIM->cmd) {
            SIM->cmd(k);
        }
    }
}

/* runs the model over line[0 .. T): the strip's words in out[] */
static void simulate(const sim_t *sc)
{
    uint32_t k, next_tick = 100u, ntk = 0, next_ms = 0;
    if (sc->T > cap_w) {
        printf("scenario of %u words, the world holds %u\n", sc->T, cap_w);
        exit(3);
    }
    SIM = sc;
    memset(ring, 0, sizeof ring);
    memset(owned_t, 0, (cap_w + 64u) * sizeof *owned_t);
    memset(out, 0, (cap_w + 64u) * sizeof *out);
    g_blank = 1;
    g_mux = 0;
    g_de = 0;
    g_ready_pending = 0;
    n_sends = n_ready = sum_blank_local = 0;
    g_step = 0;
    n_exp = 0;
    enc_from = 0;
    enc_to = 0;
    dline_init(&D, 0);
    for (k = 0; k < sc->T; k++) {
        const uint32_t in_flash = sc->flash_to != 0u && k >= sc->flash_from && k < sc->flash_to;
        ring[k & M] = g_de ? 0u : line[k];                /* RX DMA */
        sum_blank_local += U.local && g_blank;
        if (g_de && k >= de_end_w) {                      /* the burst is done: DE low, RO back */
            g_de = 0;
            dline_burst_done(&D, (k + 1u) & M);
        }
        if (k >= 510u) {                                  /* TX DMA: word k - 510 */
            out[k - 510u] = g_blank ? 0u : ring[(k + 2u) & M];
        }
        if (g_fault_at != 0u && k == g_fault_at) {
            g_fault_pend = 1;
        }
        if (k + 1u >= next_tick) {                        /* a tick: RX has written words < k + 1 */
            uint32_t jit;
            tick(k, in_flash);
            ntk++;
            jit = sc->gap_max > 113u ? rnd() % (sc->gap_max - 112u) : 0u;
            next_tick = 100u + (ntk * 225u) / 2u + jit;
            if (sc->long_gap && k >= sc->long_gap_at && k < sc->long_gap_at + 113u) {
                next_tick = k + sc->long_gap;
            }
            if (!in_flash && !sc->no_main && (sc->main_div < 2u || ntk % sc->main_div == 0u)) {
                main_loop(k, k >= next_ms);
                if (k >= next_ms) {
                    next_ms = k + 2250u;
                }
            }
        }
    }
}

/* the words the encoder owned that TX read before it wrote them (TX reads word W at time W + 510) */
static uint32_t deadline_misses(uint32_t T, uint32_t unused)
{
    uint32_t W, miss = 0;
    (void)unused;
    for (W = enc_from; W < enc_to && W + 512u < T; W++) {
        miss += owned_t[W] == 0u || owned_t[W] > W + 509u;
    }
    return miss;
}

/* ------------------------------------------------------------------------------------------------ helpers */
static uint32_t ux_max_px_test(uint32_t bpp) { return bpp == 3u ? 600u : bpp == 4u ? 500u : 400u; }
static void make_rx(uint32_t chip, uint32_t order, uint32_t npx, uint32_t fps, double hsi)
{
    udfx_init(&U);
    (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, chip);
    if (order) {
        (void)udfx_config(&U, ULTRA_CFG_PX_ORDER, order);
    }
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, npx);
    (void)udfx_config(&U, ULTRA_CFG_PX_FPS, fps);
    (void)hsi;
}

static void send_fx(uint8_t seg, uint8_t seq, uint8_t effect, int32_t start_ms, const uint8_t c[9], uint8_t bri,
                    uint16_t trans, uint16_t a, uint16_t len, uint8_t flags, uint32_t hdr_us)
{
    ultra_pixel_fx_t f;
    uint8_t s[32];
    uint32_t i;
    memset(&f, 0, sizeof f);
    f.effect = effect;
    f.start_ms = start_ms;
    f.speed = 150;
    f.intensity = 128;
    f.palette = ULTRA_PX_PAL_DEFAULT;
    for (i = 0; i < 9u; i++) {
        f.c[i / 3u][i % 3u] = c[i];
    }
    f.brightness = bri;
    f.transition_ms = trans;
    f.seg_start = a;
    f.seg_len = len;
    f.flags = flags;
    ultra_slot_pixel_fx(s, seg, seq, &f);
    udfx_pixel_fx(&U, s, hdr_us);
}

static const uint8_t COL[9] = { 255, 80, 0, 0, 0, 40, 0, 200, 90 };

static void check_frames(uint32_t n, uint32_t first, uint32_t bytes, const chip_o_t *c, const char *what)
{
    uint32_t k, ok = 0;
    for (k = 0; k < n; k++) {
        const dfr_t *f = &fr[k];
        const uint32_t j = first + k;
        const uint32_t resets = c->reset_us * 36u;
        int good = f->nbits == bytes * 8u && f->exact && f->low_before >= resets - 36u && j < n_exp &&
                   memcmp(f->b, exp_fb[j], bytes) == 0;
        CHECK(f->nbits == bytes * 8u, "%s frame %u: %u bits, not %u", what, k, f->nbits, bytes * 8u);
        CHECK(f->exact, "%s frame %u: a pulse off the chip's timing", what, k);
        CHECK(f->low_before >= resets - 36u, "%s frame %u: reset %u samples (%u needed)", what, k, f->low_before, resets);
        CHECK(j < n_exp && memcmp(f->b, exp_fb[j], bytes) == 0, "%s frame %u: bytes differ from the rendered ones", what, k);
        ok += good;
    }
    printf("  %-18s %3u frames decoded, %3u exact and equal to the rendered ones\n", what, n, ok);
}

/* the independent oracle for a Solid pixel: scale8 by the brightness, gamma 2.8, the white mode, the order */
static uint8_t o_scale8(uint32_t i, uint32_t s) { return (uint8_t)((i * (s + 1u)) >> 8); }

/* ------------------------------------------------------------------------------------------------ scenarios */
static uint32_t g_fx = 3;               /* the effect a scenario starts */
static uint8_t  g_bri = 255;
static uint16_t g_trans;
static uint8_t  g_flags;

static void cmd_start(uint32_t k)       /* ms 1: the hub's clock anchor and the effect */
{
    if (g_step == 0u && k / 2250u >= 1u) {
        g_step = 1;
        udfx_anchor(&U, host_of_word(D.wpos), D.wpos);
        send_fx(0, 1, (uint8_t)g_fx, 0, COL, g_bri, g_trans, 0, 0, g_flags, host_of_word(D.wpos));
    }
}

static void test_chips(void)
{
    uint32_t c, T = 2250u * 60u;
    sim_t sc;
    printf("[1] local frames per chip (60 px, 60 ms, ticks up to 60 us late)\n");
    for (c = 0; c < ULTRA_PXC_COUNT; c++) {
        static const uint8_t fxs[8] = { FX_PALETTE, FX_TWINKLEFOX, FX_FIRE, FX_SINELON, FX_PLASMA, FX_GRADIENT, FX_NOISE, FX_FLOW };
        uint32_t n;
        memset(&sc, 0, sizeof sc);
        sc.T = T; sc.gap_max = 135u + 112u; sc.steps = 2; sc.cmd = cmd_start;
        line_clear();
        make_rx(c, 0, 60, 0, 0);
        g_fx = fxs[c]; g_bri = 255; g_trans = 0; g_flags = 0;
        g_name = CHIP[c].name;
        simulate(&sc);
        n = decode(out, 0, T - 600u, CHIP[c].s, CHIP[c].t0h, CHIP[c].t1h);
        check_frames(n, 0, 60u * CHIP[c].bpp, &CHIP[c], CHIP[c].name);
        CHECK(deadline_misses(T, 700u) == 0u, "%s: deadline missed", CHIP[c].name);
        CHECK(n > 8u, "%s: only %u frames", CHIP[c].name, n);
        CHECK(U.e.lost == 0u, "%s: the encoder lost a frame", CHIP[c].name);
    }
}

/* the oracle: a Solid c1 pixel through brightness, gamma 2.8, the auto-white mode, the white split and the wire order */
static void oracle_px(const uint8_t *c, uint32_t bri, uint32_t bpp, const uint8_t *ord, uint32_t mode, uint32_t split,
                      uint8_t *wire)
{
    uint8_t ch[5];
    uint32_t w = 0, k;
    ch[0] = fx_gamma(o_scale8(c[0], bri));
    ch[1] = fx_gamma(o_scale8(c[1], bri));
    ch[2] = fx_gamma(o_scale8(c[2], bri));
    ch[3] = ch[4] = 0;
    if (bpp >= 4u && mode != 1u) {
        w = ch[0] < ch[1] ? ch[0] : ch[1];
        w = w < ch[2] ? w : ch[2];
        if (mode != 3u) {
            ch[0] = (uint8_t)(ch[0] - w); ch[1] = (uint8_t)(ch[1] - w); ch[2] = (uint8_t)(ch[2] - w);
        }
    }
    if (bpp == 4u) {
        ch[3] = (uint8_t)w;
    } else if (bpp == 5u) {
        ch[3] = o_scale8(w, 255u - split);
        ch[4] = o_scale8(w, split);
    }
    for (k = 0; k < bpp; k++) {
        wire[k] = ch[ord[k]];
    }
}

/* the 16-bit oracle (CONFIG 141 = ROUND): gamma and brightness at 16 bits, auto-white, the split, then (value + 128) >> 8; written out again here, not shared with fx_pack */
extern uint32_t fx_gamma16_of(uint32_t v);
static void oracle_px16(const uint8_t *c, uint32_t bri, uint32_t bpp, const uint8_t *ord, uint32_t mode, uint32_t split, uint8_t *wire)
{
    uint32_t v[5] = { 0, 0, 0, 0, 0 }, w, k, g = fx_gamma16_of(bri), mb = g + (g >> 8);
    for (k = 0; k < 3u; k++) {
        v[k] = fx_gamma16_of(c[k]);
        if (bri != 255u) {
            v[k] = (v[k] * mb + 32768u) >> 16;
        }
    }
    if (bpp >= 4u && mode != 1u) {
        w = v[0] < v[1] ? v[0] : v[1];
        w = w < v[2] ? w : v[2];
        if (mode != 3u) {
            v[0] -= w; v[1] -= w; v[2] -= w;
        }
        if (bpp == 4u) {
            v[3] = w;
        } else {
            v[3] = (w * (256u - split)) >> 8;
            v[4] = (w * (split + 1u)) >> 8;
        }
    }
    for (k = 0; k < bpp; k++) {
        wire[k] = (uint8_t)((v[ord[k]] + 128u) >> 8);
    }
}

static void orders_pass(uint32_t smooth)
{
    struct { uint32_t chip, order, white_cfg; uint8_t flags; const char *what; } t[] = {
        { 0, 0, 0, 0, "WS2812B default GRB" },
        { 0, ULTRA_PX_ORDER(ULTRA_PXCH_B, ULTRA_PXCH_G, ULTRA_PXCH_R, 0, 0), 0, 0, "WS2812B as BGR" },
        { 7, 0, 0, 0, "WS2811 default RGB" },
        { 3, 0, 0, 0, "SK6812 RGBW, accurate white" },
        { 3, 0, 1, 0, "SK6812 RGBW, white off (config)" },
        { 3, 0, 0, (uint8_t)(ULTRA_PXW_BRIGHT << ULTRA_PXF_WHITE_SHIFT), "SK6812 RGBW, bright (flag over config)" },
        { 4, 0, 0, 0, "WS2814 default RGBW" },
        { 4, ULTRA_PX_ORDER(ULTRA_PXCH_W, ULTRA_PXCH_R, ULTRA_PXCH_G, ULTRA_PXCH_B, 0), 0, 0, "WS2814 as WRGB" },
        { 5, 0, 0, 0, "WS2805 RGB WW CW, even split" },
        { 5, 0, (200u << 8) | 3u, 0, "WS2805 bright, split 200" },
        { 5, ULTRA_PX_ORDER(ULTRA_PXCH_CW, ULTRA_PXCH_W, ULTRA_PXCH_B, ULTRA_PXCH_G, ULTRA_PXCH_R), 0, 0, "WS2805 reversed" },
        { 6, 0, 0, 0, "WS2815 GRB" },
    };
    uint32_t i, j, T = 2250u * 12u;
    printf("[2] colour orders, auto-white modes and the white split (Solid c1 = 255,80,0 at 200/255), smoothing %s\n", smooth == ULTRA_PXS_OFF ? "off (8 bits)" : "round (16 bits)");
    for (i = 0; i < sizeof t / sizeof t[0]; i++) {
        sim_t sc;
        uint8_t ord[5], exp[5], mode, split;
        const uint32_t chip = t[i].chip, bpp = CHIP[chip].bpp;
        uint32_t n, good = 0;
        static const uint8_t def[ULTRA_PXC_COUNT][5] = { {1,0,2,0,0}, {1,0,2,0,0}, {1,0,2,0,0}, {1,0,2,3,0}, {0,1,2,3,0},
                                                          {0,1,2,3,4}, {1,0,2,0,0}, {0,1,2,0,0} };
        memset(&sc, 0, sizeof sc);
        sc.T = T; sc.gap_max = 135u; sc.steps = 2; sc.cmd = cmd_start;
        line_clear();
        make_rx(chip, t[i].order, 20, 0, 0);
        if (t[i].white_cfg) {
            (void)udfx_config(&U, ULTRA_CFG_PX_WHITE, t[i].white_cfg);
        }
        (void)udfx_config(&U, ULTRA_CFG_PX_SMOOTH, smooth);
        g_fx = FX_SOLID; g_bri = 200; g_trans = 0; g_flags = t[i].flags;
        g_name = t[i].what;
        simulate(&sc);
        n = decode(out, 0, T - 600u, CHIP[chip].s, CHIP[chip].t0h, CHIP[chip].t1h);
        CHECK(n >= 1u, "%s: no frame", t[i].what);
        for (j = 0; j < 5u; j++) {
            ord[j] = t[i].order ? (uint8_t)((t[i].order >> (3u * j)) & 7u) : def[chip][j];
        }
        mode = (uint8_t)((t[i].flags & ULTRA_PXF_WHITE_MASK) >> ULTRA_PXF_WHITE_SHIFT);
        if (mode == 0u) {
            mode = (uint8_t)((t[i].white_cfg & 3u) ? (t[i].white_cfg & 3u) : 2u);
        }
        split = (uint8_t)(((t[i].white_cfg >> 8) & 0xFFu) ? (t[i].white_cfg >> 8) & 0xFFu : 128u);
        (smooth == ULTRA_PXS_OFF ? oracle_px : oracle_px16)(COL, 200u, bpp, ord, mode, split, exp);
        if (n >= 1u) {
            int ok = fr[0].nbits == 20u * bpp * 8u;
            for (j = 0; j < 20u && ok; j++) {
                ok = memcmp(fr[0].b + j * bpp, exp, bpp) == 0;
            }
            CHECK(ok, "%s: pixel bytes %02x %02x %02x %02x %02x, expected %02x %02x %02x %02x %02x", t[i].what,
                  fr[0].b[0], fr[0].b[1], fr[0].b[2], fr[0].b[3], fr[0].b[4], exp[0], exp[1], exp[2], exp[3], exp[4]);
            good = ok;
        }
        printf("  %-40s %s  wire %02x %02x %02x%s\n", t[i].what, good ? "ok" : "WRONG", exp[0], exp[1], exp[2],
               bpp > 3u ? " ..." : "");
    }
}

static void test_orders(void)
{
    orders_pass(ULTRA_PXS_OFF);
    orders_pass(ULTRA_PXS_ROUND);
}

/* prime the clock: two anchors 1.5 s apart before the simulation's first word (the rate is measured) */
static void prime_clock(void)
{
    const uint32_t w1 = 0u - 3400000u, w2 = 0u - 50u;
    udfx_anchor(&U, host_of_word(w1), w1);
    udfx_anchor(&U, host_of_word(w2), w2);
}

static uint32_t g_anchor_every = 100u, g_jitter_us = 4u;
static void cmd_grid(uint32_t k)
{
    const uint32_t ms = k / 2250u;
    static uint32_t last_anchor;
    if (g_step == 0u && ms >= 1u) {
        send_fx(0, 1, (uint8_t)g_fx, 0, COL, 255, 0, 0, 0, 0, host_of_word(D.wpos));
        g_step = 1;
        last_anchor = 0;
    }
    if (ms >= 1u && ms - last_anchor >= g_anchor_every) {
        const int32_t j = (int32_t)(rnd() % (2u * g_jitter_us + 1u)) - (int32_t)g_jitter_us;
        udfx_anchor(&U, host_of_word(D.wpos) + (uint32_t)j, D.wpos);
        last_anchor = ms;
    }
}

static void test_grid(void)
{
    static const struct { double hsi; uint32_t phase, fps, npx; } cases[] = { { 0.0, 0, 60, 300 }, { 0.022, 1234, 60, 300 },
                                                                      { -0.022, 99999, 30, 300 }, { 0.011, 777, 100, 200 } };
    uint32_t c;
    printf("[3] frames on the host-clock grid (WS2812B, 300 px at 30 / 60 fps, 200 at 100; the receiver's HSI off by up to 2.2 %%, anchors every 100 ms +-4 us)\n");
    for (c = 0; c < 4u; c++) {
        sim_t sc;
        uint32_t T = 2250u * 330u, n, k, per_us = 1000000u / cases[c].fps, bad = 0, maxd = 0, spacing_bad = 0;
        memset(&sc, 0, sizeof sc);
        sc.T = T; sc.gap_max = 135u + 112u; sc.steps = 2; sc.cmd = cmd_grid; sc.hsi = cases[c].hsi; sc.phase = cases[c].phase;
        line_clear();
        make_rx(0, 0, cases[c].npx, cases[c].fps, cases[c].hsi);
        prime_clock();
        g_fx = FX_TWINKLEFOX;
        g_name = "grid";
        simulate(&sc);
        n = decode(out, 0, T - 600u, 48, 13, 25);
        for (k = 2; k < n; k++) {
            /* the frame's first rising edge is its start: host time of that word against the grid */
            const uint32_t h = host_of_word(fr[k].start_w);
            const uint32_t r = (h + per_us / 2u) % per_us;
            const uint32_t d = r > per_us / 2u ? per_us - r : r;       /* hmm: distance to the nearest grid point */
            const uint32_t dist = (h % per_us) < per_us / 2u ? h % per_us : per_us - h % per_us;
            (void)d;
            if (dist > maxd) {
                maxd = dist;
            }
            bad += dist > g_jitter_us + 3u;
            if (k > 2u) {
                spacing_bad += (fr[k].start_w - fr[k - 1].start_w) < 1000u;
            }
        }
        CHECK(n >= 4u, "grid %u fps: %u frames", cases[c].fps, n);
        CHECK(bad == 0u, "grid %u fps (hsi %+.1f %%): %u frames off the grid by more than the anchor jitter + 3 us, the worst %u us",
              cases[c].fps, cases[c].hsi * 100.0, bad, maxd);
        printf("  %3u fps, hsi %+.1f %%: %u frames, the worst start %u us off the host grid, late frames %u\n", cases[c].fps,
               cases[c].hsi * 100.0, n, maxd, U.e.late);
        (void)spacing_bad;
    }
}

/* ------------------------------------------------------------------------------------------------ stream <-> local */
static uint32_t g_t_local_ms = 20, g_t_stream_ms = 72, g_poll_answer, g_policy, g_fxflags;
static void cmd_switch(uint32_t k)
{
    const uint32_t ms = k / 2250u;
    if (g_step == 0u && ms >= 1u) {
        udfx_anchor(&U, host_of_word(D.wpos), D.wpos);
        if (g_policy) {
            (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, g_policy);
        }
        g_step = 1;
    }
    if (g_step == 1u && ms >= g_t_local_ms) {
        send_fx(0, 1, FX_SINELON, 0, COL, 255, 0, 0, 0, (uint8_t)g_fxflags, host_of_word(D.wpos));
        g_step = 2;
    }
    if (g_step == 2u && ms >= g_t_stream_ms) {
        send_fx(0, 2, 0, 0, COL, 0, 0, 0, 0, 0, host_of_word(D.wpos));
        g_step = 3;
    }
}

/* a hub: WLED-like frames (40 px, 80 us reset) every 3 ms, shifted by `shift` words; a marker + RW frame + release at 45 ms */
static void build_switch_line(uint32_t shift, uint8_t hb[120], int with_poll)
{
    uint32_t k;
    line_clear();
    for (k = 0; k < 120u; k++) {
        hb[k] = (uint8_t)(k * 37u + 11u);
    }
    for (k = 0; ln < 2250u * 16u * 19u + shift * 16u; k++) {
        low_until_word(g_hub_period * k + 300u + shift);
        hb[0] = (uint8_t)k;
        ctl_frame(hb);
        run(0, 80u * 36u);
    }
    if (with_poll) {
        low_until_word(2250u * 45u);
        run(1, 20u * 36u);                               /* the marker */
        run(0, 10u * 36u);
        for (k = 0; k < 320u; k++) {
            uint32_t h = (rnd() & 1u) ? 25u : 12u;
            run(1, h);
            run(0, 45u - h);
        }
        run(0, 150u * 36u);                              /* t_end .. the release */
        run(1, 800u * 36u);                              /* the released line */
    }
    for (k = 0; ln < 2250u * 16u * g_line_ms; k++) {     /* the hub streams again from 70 ms */
        low_until_word(2250u * 70u + g_hub_period * k + shift);
        hb[0] = (uint8_t)(100u + k);
        ctl_frame(hb);
        run(0, 80u * 36u);
    }
}

static void check_switch(uint32_t n, const uint8_t hb[120], uint32_t *hub, uint32_t *loc, uint32_t *other, const char *what)
{
    uint32_t k, phase = 0;
    *hub = *loc = *other = 0;
    for (k = 0; k < n; k++) {
        const dfr_t *f = &fr[k];
        const uint32_t lb = U.npx * U.chip.bpp;
        if (f->nbits == 120u * 8u && memcmp(f->b + 1, hb + 1, 119) == 0 && (f->exact & 1u)) {
            (*hub)++;
            if (phase == 1u) {
                phase = 2;
            }
        } else if (f->nbits == lb * 8u && *loc < n_exp && memcmp(f->b, exp_fb[*loc], lb) == 0 && (f->exact & 2u)) {
            (*loc)++;
            CHECK(phase != 2u, "%s: a local frame after the hub took over (frame %u)", what, k);
            phase = 1;
        } else {
            (*other)++;
            if (!g_quiet) {
                printf("  %s: frame %u at word %u: %u bits (not a whole hub or local frame)\n", what, k, f->start_w, f->nbits);
            }
        }
    }
}

static void test_switch(uint32_t chip, uint32_t shifts, int with_poll, uint32_t answer, const char *title)
{
    uint8_t hb[120];
    uint32_t sh, bad = 0, minhub = 1000, minloc = 1000, hub = 0, loc = 0, other = 0, T = 2250u * 125u, sends = 0;
    printf("[4] %s\n", title);
    for (sh = 0; sh < shifts; sh++) {
        sim_t sc;
        uint32_t n, f0 = (uint32_t)g_fail;
        memset(&sc, 0, sizeof sc);
        sc.T = T; sc.gap_max = 135u + 112u; sc.steps = 2; sc.cmd = cmd_switch;
        build_switch_line(sh * (shifts > 1u ? 6750u / shifts : 0u) + (sh % 7u) * 3u, hb, with_poll);
        make_rx(chip, 0, 50, 0, 0);
        g_answer = answer;
        g_name = title;
        g_quiet = shifts > 1u;
        simulate(&sc);
        g_ntim = 2;
        g_tim[0].s = 45; g_tim[0].t0h = 14; g_tim[0].t1h = 29;
        g_tim[1].s = CHIP[chip].s; g_tim[1].t0h = CHIP[chip].t0h; g_tim[1].t1h = CHIP[chip].t1h;
        n = decode_all(out, 0, T - 600u, 0, 14, 29);
        g_ntim = 0;
        while (n > 0u && !fr[n - 1u].closed) {
            n--;
        }
        check_switch(n, hb, &hub, &loc, &other, title);
        CHECK(other == 0u, "%s (shift %u): %u cut or foreign frames on the strip", title, sh, other);
        CHECK(U.switches == 2u, "%s (shift %u): %u switches, not 2 (local %u want_stream %u want_local %u e.state %u low_w %u "
              "d.state %u d.zs %u d.wpos %u frames %u)", title, sh, U.switches, U.local, U.want_stream, U.want_local,
              U.e.state, U.e.low_w, D.state, D.zs, D.wpos, U.e.frames);
        CHECK(hub > 5u && loc > 5u, "%s (shift %u): hub %u, local %u", title, sh, hub, loc);
        CHECK(sum_blank_local == 0u, "%s (shift %u): BLANK_N was LOW during local mode", title, sh);
        if (hub < minhub) minhub = hub;
        if (loc < minloc) minloc = loc;
        sends += n_sends;
        bad += (uint32_t)g_fail != f0;
        g_quiet = 0;
    }
    printf("  %u runs (hub frames shifted over a 3 ms period): every frame on the strip a whole hub or local one; hub >= %u, local >= %u frames, %u answers sent, %u runs with a failure\n",
           shifts, minhub, minloc, sends, bad);
}


/* ------------------------------------------------------------------------------------------------ more scenarios */
static uint32_t frame_sum(const dfr_t *f)
{
    uint32_t i, sum = 0;
    for (i = 0; i < f->nbits / 8u; i++) {
        sum += f->b[i];
    }
    return sum;
}

static void test_flash(void)
{
    sim_t sc;
    const uint32_t T = 2250u * 90u, f0 = 2250u * 30u, f1 = 2250u * 46u;
    uint32_t n, k, in_stall = 0, after = 0, nobump = 1;
    printf("[5] a flash operation (16 ms: the SRAM tick only, the main loop stalled) during a lit effect\n");
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u + 112u; sc.steps = 2; sc.cmd = cmd_start; sc.flash_from = f0; sc.flash_to = f1;
    line_clear();
    make_rx(0, 0, 100, 0, 0);
    g_fx = FX_PLASMA; g_bri = 255; g_trans = 0; g_flags = 0;
    g_name = "flash";
    simulate(&sc);
    n = decode(out, 0, T - 600u, 48, 13, 25);
    check_frames(n, 0, 100u * 3u, &CHIP[0], "flash stall");
    for (k = 0; k < n; k++) {
        in_stall += fr[k].start_w >= f0 + 700u && fr[k].start_w < f1;
        after += fr[k].start_w >= f1;
        nobump &= fr[k].low_before < 700u * 16u || fr[k].low_before >= 680u * 16u;   /* (resets are LOW: any length is fine) */
    }
    CHECK(deadline_misses(T, 0) == 0u, "flash: the SRAM tick missed the deadline");
    CHECK(after >= 5u, "flash: only %u frames after the operation", after);
    CHECK(in_stall <= 1u, "flash: %u frames started inside the stall (the one in progress at most)", in_stall);
    printf("  frames started during the stall: %u (the render waits), after it: %u; none cut, the SRAM tick kept encoding\n",
           in_stall, after);
    (void)nobump;
}

static void test_deadline(void)
{
    uint32_t g, last_ok = 0, T = 2250u * 36u;
    printf("[6] the deadline: one long gap between ticks (an interrupt held off), 300 px\n");
    for (g = 200u; g <= 580u; g += 20u) {
        sim_t sc;
        uint32_t n, k, bad, f0 = (uint32_t)g_fail, p0 = (uint32_t)g_printed;
        memset(&sc, 0, sizeof sc);
        sc.T = T; sc.gap_max = 113u; sc.steps = 2; sc.cmd = cmd_start; sc.long_gap_at = 2250u * 16u; sc.long_gap = g;
        line_clear();
        make_rx(0, 0, 300, 0, 0);
        g_fx = FX_PLASMA; g_bri = 255; g_trans = 0; g_flags = 0;
        g_name = "deadline";
        simulate(&sc);
        n = decode(out, 0, T - 600u, 48, 13, 25);
        bad = deadline_misses(T, 0) + (n == 0u);
        for (k = 0; k < n; k++) {
            bad += fr[k].nbits != 300u * 24u || !(fr[k].exact & 1u) || k >= n_exp || memcmp(fr[k].b, exp_fb[k], 900) != 0;
        }
        g_fail = (int)f0;                                /* (the sweep looks for the edge: failures expected) */
        g_printed = (int)p0;
        if (bad == 0u) {
            last_ok = g;
        }
        if (g <= 420u) {
            CHECK(bad == 0u, "a tick gap of %u words (%u us) must pass", g, g * 4u / 9u);
        }
    }
    printf("  a gap of %u words (%u us) between ticks still passes (the encoder drops its frame past %u words)\n", last_ok,
           last_ok * 4u / 9u, WS_LATE_W);
    CHECK(last_ok >= 420u && last_ok <= WS_LATE_W, "the deadline is %u words, not 420 .. %u", last_ok, (uint32_t)WS_LATE_W);
}

static void test_fault(void)
{
    sim_t sc;
    const uint32_t T = 2250u * 70u;
    uint32_t n, k, whole = 0, cut = 0, j = 0, bad_after = 0;
    printf("[7] a delay-line fault (SPI OVR) in local mode: BLANK_N, the ring restarts, the effect resumes\n");
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u; sc.steps = 2; sc.cmd = cmd_start;
    g_fault_at = 2250u * 30u + 37u;
    line_clear();
    make_rx(0, 0, 100, 0, 0);
    g_fx = FX_SINELON; g_bri = 255; g_trans = 0; g_flags = 0;
    g_name = "fault";
    g_fault_pend = 0;
    simulate(&sc);
    CHECK(g_blank == 0u, "BLANK_N still LOW at the end");
    n = decode(out, 0, T - 600u, 48, 13, 25);
    for (k = 0; k < n; k++) {
        uint32_t m = j;
        if (fr[k].nbits != 100u * 24u) {
            cut++;
            continue;
        }
        while (m < n_exp && memcmp(fr[k].b, exp_fb[m], 300) != 0) {
            m++;
        }
        if (m == n_exp) {
            if (fr[k].start_w > g_fault_at + 2250u) {
                bad_after++;
            } else {
                cut++;
            }
        } else {
            j = m + 1u;
            whole++;
        }
    }
    CHECK(cut <= 1u, "%u cut frames around the fault", cut);
    CHECK(bad_after == 0u, "%u frames after the fault are not rendered ones", bad_after);
    CHECK(whole > 10u && U.local, "only %u whole frames, local %u", whole, U.local);
    printf("  %u whole frames, %u cut at the fault, BLANK_N back up, faults seen by dline %u\n", whole, cut, D.st.faults);
}

/* ------------------------------------------------------------------------------------------------ the policy, the limiter */
static void build_rec(uint8_t rec[UX_REC_BYTES], uint32_t policy, int stream_wins_only)
{
    (void)stream_wins_only;
    udfx_init(&U);
    (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, ULTRA_PXC_WS2812B_V5);
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, 40);
    (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, policy);
    send_fx(0, 1, FX_PALETTE, 0, COL, 200, 300, 0, 0, ULTRA_PXF_STORE, 0);
    udfx_rec_pack(&U, 5, rec);
}

static void test_standalone(void)
{
    uint8_t rec[UX_REC_BYTES], hb[120];
    sim_t sc;
    const uint32_t T = 2250u * 2450u;
    uint32_t n, k, loc = 0, hub = 0, other = 0, first_local = 0, sum0 = 0, sumN = 0, phase = 0;
    printf("[8] power-on: the stored effect after 2 s of a quiet line, then a controller's stream takes the strip back\n");
    world_reserve(MAXW_LONG);
    build_rec(rec, ULTRA_PXP_POWER_ON | ULTRA_PXP_STREAM_WINS, 0);
    CHECK(udfx_rec_valid(rec), "the record is not valid");
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u; sc.steps = 2; sc.cmd = 0;
    line_clear();
    for (k = 0; k < 120u; k++) {
        hb[k] = (uint8_t)(k * 29u + 5u);
    }
    for (k = 0; k < 40u; k++) {                          /* a controller boots at 2.3 s */
        low_until_word(2250u * 2300u + 2250u * 3u * k);
        hb[0] = (uint8_t)k;
        hub_frame(hb, 120, 45, 14, 29);
        run(0, 80u * 36u);
    }
    udfx_init(&U);
    udfx_load(&U, rec);
    CHECK(U.cfg[ULTRA_CFG_PX_PIXELS - ULTRA_CFG_PX_FIRST] == 40u && U.stored_have == 1u, "the record did not load");
    g_name = "standalone";
    simulate(&sc);
    g_ntim = 2;
    g_tim[0].s = 45; g_tim[0].t0h = 14; g_tim[0].t1h = 29;
    g_tim[1].s = 48; g_tim[1].t0h = 13; g_tim[1].t1h = 25;
    n = decode_all(out, 0, T - 600u, 0, 14, 29);
    g_ntim = 0;
    while (n > 0u && !fr[n - 1u].closed) {
        n--;
    }
    for (k = 0; k < n; k++) {
        const dfr_t *f = &fr[k];
        if (f->nbits == 120u * 8u && memcmp(f->b + 1, hb + 1, 119) == 0 && (f->exact & 1u)) {
            hub++;
            phase = 2;
        } else if (f->nbits == 40u * 24u && (f->exact & 2u) && loc < n_exp && memcmp(f->b, exp_fb[loc], 120) == 0) {
            if (loc == 0u) {
                first_local = f->start_w;
                sum0 = frame_sum(f);
            }
            sumN = frame_sum(f);
            CHECK(phase != 2u, "a local frame after the hub took over");
            loc++;
            phase = 1;
        } else {
            other++;
            printf("  frame %u at word %u: %u bits (not a whole hub or local frame)\n", k, f->start_w, f->nbits);
        }
    }
    CHECK(other == 0u, "%u cut or foreign frames", other);
    CHECK(first_local > 2250u * 2000u && first_local < 2250u * 2100u, "the stored effect started at %.1f ms, not at 2.0-2.1 s",
          first_local / 2250.0);
    CHECK(loc >= 17u && loc <= 21u, "nominal 60 fps standalone: %u local frames", loc);
    CHECK(hub > 20u, "only %u hub frames after the controller booted", hub);
    CHECK(sum0 * 3u < sumN, "no fade-in: the first frame's sum %u, a later one's %u", sum0, sumN);
    CHECK(U.switches == 2u && !U.local, "switches %u, local %u", U.switches, U.local);
    printf("  stored effect from %.0f ms (fade-in: frame sums %u -> %u), %u local frames, then %u whole hub frames\n",
           first_local / 2250.0, sum0, sumN, loc, hub);
    world_reserve(MAXW);
}


/* a scripted main loop: timed actions, run in order once their ms has come */
enum { A_END = 0, A_ANCHOR, A_FX, A_CFG };
typedef struct {
    uint32_t ms;
    int kind;
    uint8_t seg, seq, fx, bri, flags;
    int32_t start;
    uint16_t trans, a, len;
    const uint8_t *col;
    uint32_t id, v;
} act_t;
static const act_t *g_script;
static uint32_t g_sc_i;
static void cmd_script(uint32_t k)
{
    const uint32_t ms = k / 2250u;
    while (g_script[g_sc_i].kind != A_END && ms >= g_script[g_sc_i].ms) {
        const act_t *x = &g_script[g_sc_i++];
        if (x->kind == A_ANCHOR) {
            udfx_anchor(&U, host_of_word(D.wpos), D.wpos);
        } else if (x->kind == A_FX) {
            send_fx(x->seg, x->seq, x->fx, x->start, x->col ? x->col : COL, x->bri, x->trans, x->a, x->len, x->flags,
                    host_of_word(D.wpos));
        } else if (x->kind == A_CFG) {
            (void)udfx_config(&U, x->id, x->v);
        }
    }
}
static void run_script(sim_t *sc, const act_t *script)
{
    g_script = script;
    g_sc_i = 0;
    sc->cmd = cmd_script;
    simulate(sc);
}

static const uint8_t WHITE[9] = { 255, 255, 255, 0, 0, 0, 0, 0, 0 };

static void test_limiter(void)
{
    sim_t sc;
    const uint32_t T = 2250u * 120u;
    uint32_t n, k, mA0 = 0, mA_end = 0;
    const act_t script[] = {
        { 1, A_ANCHOR }, { 1, A_CFG, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ULTRA_CFG_PX_LIMIT_MA, 2000 },
        { 1, A_FX, 0, 1, FX_SOLID, 255, 0, 0, 0, 0, 0, WHITE }, { 0, A_END } };
    printf("[9] the current limiter: a white 100-pixel strip against a 2 A limit (20 mA a channel)\n");
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u; sc.steps = 2;
    line_clear();
    make_rx(0, 0, 100, 0, 0);
    g_name = "limiter";
    run_script(&sc, script);
    n = decode(out, 0, T - 600u, 48, 13, 25);
    for (k = 0; k < n; k++) {
        const uint32_t mA = frame_sum(&fr[k]) * 200u / 2550u;
        if (k == 0u) {
            mA0 = mA;
        }
        mA_end = mA;
    }
    CHECK(n > 10u, "%u frames", n);
    CHECK(mA0 > 4000u, "the first frame draws %u mA (nothing known yet: full white)", mA0);
    CHECK(mA_end >= 1750u && mA_end <= 2100u, "the limited strip draws %u mA of a 2000 mA limit", mA_end);
    printf("  first frame %u mA, after %u frames %u mA (brightness scale %u/255)\n", mA0, n, mA_end, (uint32_t)U.lim_bri);
    /* Persistent measured overload must reach the documented trim floor, even
     * after the integer correction becomes less than one Q8 unit per frame. */
    udfx_init(&U);
    U.local = 1;
    U.npx = 1;
    U.meas_ma = 3000;
    for (k = 0; k < 200u; k++) {
        U.begun = 1;
        U.rendered = 1;
        U.e.ready = 0;
        U.fsum = 255;
        (void)udfx_render(&U);
        CHECK(U.cl_q8 >= 8u && U.cl_q8 <= 256u, "limiter trim out of bounds: %u", U.cl_q8);
    }
    CHECK(U.cl_q8 == 8u, "persistent overload stopped trimming at %u/256 instead of 8/256", U.cl_q8);
}

static void test_ramp(void)
{
    sim_t sc;
    const uint32_t T = 2250u * 360u;
    uint32_t n, k, mono = 1, prev = 0xFFFFFFFFu, started_dark = 0, dark_frames = 0, after_wake = 0, min_fade = 0, nxf = 0, xf_sum[64],
             first_sum = 0, dark_first = 0;
    const act_t script[] = {
        { 1, A_ANCHOR },
        { 1, A_FX, 0, 1, FX_SOLID, 255, 0, 0, 0, 0, 0, WHITE },
        { 40, A_FX, 0, 2, FX_SOLID, 0, 0, 0, 100, 0, 0, WHITE },      /* to dark over 100 ms */
        { 230, A_FX, 0, 3, FX_SOLID, 255, 0, 0, 0, 0, 0, WHITE },     /* wake */
        { 300, A_FX, 0, 4, FX_SINELON, 255, 0, 0, 120, 0, 0, COL },     /* another effect: fade out 60 ms, in 60 ms */
        { 0, A_END } };
    printf("[10] transitions: a brightness ramp to dark (then the line rests), a wake, an effect change\n");
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u; sc.steps = 2;
    line_clear();
    make_rx(0, 0, 30, 0, 0);
    g_name = "ramp";
    run_script(&sc, script);
    n = decode(out, 0, T - 600u, 48, 13, 25);
    for (k = 0; k < n; k++) {
        const uint32_t sum = frame_sum(&fr[k]), ms = fr[k].start_w / 2250u;
        if (k == 0u) {
            first_sum = sum;
        }
        if (ms >= 42u && ms < 142u) {                    /* the ramp down: never up */
            mono &= sum <= prev;
            prev = sum;
        }
        if (ms >= 138u && ms < 160u) {
            started_dark |= sum == 0u;                   /* the black frame that ends the ramp */
        }
        if (ms >= 160u && ms < 228u) {
            dark_frames++;                               /* frames while resting: none */
            if (dark_first == 0u) {
                dark_first = ms;
            }
        }
        if (ms >= 236u && ms < 296u) {
            after_wake += sum > first_sum / 2u;
        }
        if (ms >= 300u && ms < 420u) {                   /* the crossfade from Solid white (first_sum) to the new effect */
            xf_sum[nxf < 64u ? nxf : 63u] = sum;
            nxf++;
        }
    }
    CHECK(n > 20u, "%u frames", n);
    CHECK(mono, "the ramp down is not monotonic");
    CHECK(started_dark && dark_frames == 0u, "the dark: %u frames from %u ms (sum 0 seen %u)", dark_frames, dark_first, started_dark);
    CHECK(after_wake > 5u, "after the wake only %u bright frames", after_wake);
    {
        const uint32_t cnt = nxf < 64u ? nxf : 64u;
        uint32_t last = cnt ? xf_sum[cnt - 1u] : 0u, between = 0, j;
        for (j = 0; j < cnt; j++) {
            between += xf_sum[j] > last + last / 8u && xf_sum[j] < first_sum - first_sum / 8u;
        }
        CHECK(last < first_sum / 2u && between >= 3u, "the effect change is not a crossfade: %u frames between %u and %u", between, last, first_sum);
        min_fade = between;
        printf("  ramp down monotonic, %u frames while dark (the black one), %u bright frames after the wake, a crossfade of %u frames (sums %u -> %u)\n",
               dark_frames, after_wake, between, first_sum, last);
    }
}

/* ------------------------------------------------------------------------------------------------ the clock */
static double t_hsi;
static uint32_t true_host(uint32_t w) { return (uint32_t)(int64_t)floor((double)(int32_t)w * (16.0 / 36.0) / (1.0 + t_hsi) + 0.5); }
static void test_clock(void)
{
    uint32_t c;
    printf("[11] the host clock: anchors against the ring's word count (HSI off, jitter, a jump, the TIME wrap)\n");
    for (c = 0; c < 4u; c++) {
        static const double hsi[4] = { 0.022, -0.022, 0.0, 0.013 };
        static const uint32_t base[4] = { 0u, 0u, 0xFFFFFFFFu - 4000000u, 12345u };   /* c = 2: the 32-bit TIME wraps */
        uint32_t i, w = 100000u, worst = 0, worst_slew = 0, jump_fixed = 0;
        int32_t off = 0;
        uint32_t prev_map = 0, prev_true = 0;
        t_hsi = hsi[c];
        udfx_init(&U);
        for (i = 0; i < 45u; i++) {
            uint32_t host, probe, m, t, tb;
            int32_t e;
            w += 2250000u;                               /* a second of words */
            if (i == 20u) {
                off = 120000;                            /* the hub's clock jumps by 120 ms */
            }
            if (i == 32u) {
                off = 120000 + 8000;                     /* ... and by 8 ms: slewed */
            }
            host = true_host(w) + (uint32_t)off + base[c] + (uint32_t)((int32_t)(rnd() % 41u) - 20);
            tb = 0;
            udfx_anchor(&U, host, w);
            probe = w + 1000u;                           /* just after the anchor, and the next second */
            m = udfx_host_us(&U, probe);
            t = true_host(probe) + (uint32_t)off + base[c];
            tb = true_host(probe) + base[c];             /* (the hub's clock without its jumps) */
            e = (int32_t)(m - t);
            if (i >= 5u && (i < 20u || i >= 22u) && i != 32u && i != 33u && i != 34u) {
                worst = (uint32_t)(e < 0 ? -e : e) > worst ? (uint32_t)(e < 0 ? -e : e) : worst;
            }
            if (i == 21u) {
                jump_fixed = (uint32_t)(e < 0 ? -e : e);
            }
            if (i >= 32u && i <= 36u) {                  /* the 8 ms: corrected at 0.5 % of the time since: <= 5 ms a second */
                const int32_t dm = (int32_t)(m - prev_map), dt = (int32_t)(tb - prev_true);
                const uint32_t slew = (uint32_t)(dm - dt < 0 ? dt - dm : dm - dt);
                worst_slew = slew > worst_slew ? slew : worst_slew;
            }
            prev_map = m;
            prev_true = tb;
        }
        CHECK(worst < 100u, "hsi %+.1f %%: the mapping is %u us off after the rate is known", hsi[c] * 100.0, worst);
        CHECK(jump_fixed < 100u, "hsi %+.1f %%: a 120 ms jump (past the 50 ms step) is still %u us off one anchor later", hsi[c] * 100.0, jump_fixed);
        CHECK(worst_slew <= 5200u, "hsi %+.1f %%: slewing an 8 ms jump moved the clock %u us in a second (0.5 %% = 5000)", hsi[c] * 100.0, worst_slew);
        CHECK(worst_slew >= 4700u, "hsi %+.1f %%: the 8 ms jump was not slewed (%u us in a second)", hsi[c] * 100.0, worst_slew);
        printf("  hsi %+.1f %%%s: mapping within %u us; a 120 ms jump is stepped (%u us left); an 8 ms jump slews %u us a second\n",
               hsi[c] * 100.0, c == 2u ? ", TIME wraps" : "", worst, jump_fixed, worst_slew);
    }
}

/* ------------------------------------------------------------------------------------------------ the record */
static void test_record(void)
{
    uint8_t rec[UX_REC_BYTES], bad[UX_REC_BYTES];
    udfx_t a;
    uint32_t i, hits = 0, ok;
    printf("[12] the stored record: pack / load, corruption, a CRC that would read as erased flash\n");
    build_rec(rec, ULTRA_PXP_POWER_ON | ULTRA_PXP_ON_LOSS, 0);
    CHECK(udfx_rec_valid(rec), "a fresh record is not valid");
    memcpy(&a, &U, sizeof a);
    udfx_init(&U);
    udfx_load(&U, rec);
    ok = memcmp(U.cfg, a.cfg, sizeof U.cfg) == 0 && U.stored_have == a.stored_have && U.stored[0].effect == a.stored[0].effect &&
         U.stored[0].brightness == a.stored[0].brightness && U.stored[0].transition_ms == a.stored[0].transition_ms &&
         memcmp(U.stored[0].c, a.stored[0].c, 9) == 0 && U.rec_seq == 5u && !U.save_due;
    CHECK(ok, "the record did not load back as it was stored");
    for (i = 0; i < UX_REC_BYTES; i++) {                /* every single bit flip is caught */
        uint32_t b;
        for (b = 0; b < 8u; b++) {
            memcpy(bad, rec, sizeof bad);
            bad[i] ^= (uint8_t)(1u << b);
            hits += !udfx_rec_valid(bad);
        }
    }
    CHECK(hits == UX_REC_BYTES * 8u, "%u of %u bit flips were caught", hits, UX_REC_BYTES * 8u);
    memset(bad, 0xFF, sizeof bad);
    CHECK(!udfx_rec_valid(bad), "erased flash (0xFF) reads as a record");
    for (i = 0; i < UX_REC_BYTES; i += 2u) {
        bad[i] = 0x39; bad[i + 1u] = 0xE3;              /* (0xE339: the other erased value) */
    }
    CHECK(!udfx_rec_valid(bad), "erased flash (0xE339) reads as a record");
    {                                                    /* a CRC that would equal an erased value never goes out */
        uint32_t n, seen = 0, would = 0;
        const int f_start = g_fail;
        for (n = 0; n < 1000000u; n++) {
            udfx_t t;
            uint32_t crc;
            memset(&t, 0, sizeof t);
            t.cfg[2] = (uint16_t)n;
            t.cfg[3] = (uint16_t)(n >> 8);
            udfx_rec_pack(&t, n, rec);
            crc = (uint32_t)rec[UX_REC_BYTES - 2u] | (uint32_t)rec[UX_REC_BYTES - 1u] << 8;
            seen += rec[UX_REC_MASK + 1u] > 1u;
            {
                uint8_t t1[UX_REC_BYTES];
                uint32_t c1;
                memcpy(t1, rec, sizeof t1);
                t1[UX_REC_MASK + 1u] = 1;
                c1 = ultra_crc16(t1, UX_REC_BYTES - 2u);
                would += c1 == 0xFFFFu || c1 == 0xE339u;
            }
            CHECK(crc != 0xFFFFu && crc != 0xE339u, "a record with an erased CRC (%u)", n);
            CHECK(udfx_rec_valid(rec), "a record does not validate (%u)", n);
            if (g_fail != f_start) {
                break;
            }
        }
        CHECK(would > 0u && would == seen, "records whose first CRC would read erased: %u, the spare count used %u", would, seen);
        printf("  1,000,000 records: %u would have had an erased-looking CRC, all took the spare count; all valid\n", seen);
    }
}

/* ------------------------------------------------------------------------------------------------ the commands */
static void slot(uint8_t *s, uint8_t seg, uint8_t seq, uint8_t fx, int32_t start, uint8_t bri, uint16_t trans, uint8_t flags)
{
    ultra_pixel_fx_t f;
    memset(&f, 0, sizeof f);
    f.effect = fx; f.start_ms = start; f.brightness = bri; f.transition_ms = trans; f.flags = flags; f.speed = 100;
    f.palette = ULTRA_PX_PAL_DEFAULT;
    ultra_slot_pixel_fx(s, seg, seq, &f);
}

/* Command-only tests explicitly advance the renderer to a new frame boundary. */
static void command_boundary(void)
{
    uint32_t local = U.local;
    U.local = 1; U.e.ready = 0; U.e.state = WS_LOW; U.begun = 0;
    while (udfx_render(&U)) {}
    U.local = (uint8_t)local;
}

static void test_cmds(void)
{
    uint8_t s[32], d[24];
    uint32_t i, x;
    printf("[13] PIXEL_FX / CONFIG semantics and page 0x81\n");
    g_name = "cmds";
    udfx_init(&U);
    D.wpos = 0; D.edge_w = 0;
    /* CONFIG ranges */
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_CHIP, ULTRA_PXC_COUNT), "chip %u accepted", ULTRA_PXC_COUNT);
    CHECK(udfx_config(&U, ULTRA_CFG_PX_CHIP, ULTRA_PXC_WS2805_RGBCCT), "chip WS2805 refused");
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_CHIP, ULTRA_PXC_WS2805_RGBCCT), "an unchanged value reports a change");
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_ORDER, ULTRA_PX_ORDER(0, 0, 1, 2, 3)), "an order with a channel twice accepted");
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_ORDER, ULTRA_PX_ORDER(0, 1, 2, 3, 5)), "an order with channel 5 accepted");
    CHECK(udfx_config(&U, ULTRA_CFG_PX_ORDER, ULTRA_PX_ORDER(4, 3, 2, 1, 0)), "a valid 5-channel order refused");
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_FPS, 1001), "fps 1001 accepted");
    CHECK(U.cfg[ULTRA_CFG_PX_FPS - ULTRA_CFG_PX_FIRST] == 60u, "default fps is not 60");
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_FPS, 60), "unchanged fps reports a change");
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_LOSS_TIMEOUT, 4) && !udfx_config(&U, ULTRA_CFG_PX_LOSS_TIMEOUT, 601), "loss timeout out of range accepted");
    CHECK(udfx_config(&U, ULTRA_CFG_PX_LOSS_TIMEOUT, 5) && udfx_config(&U, ULTRA_CFG_PX_LOSS_TIMEOUT, 0), "loss timeout 5 / 0 refused");
    CHECK(udfx_config(&U, ULTRA_CFG_PX_POLICY, 0xFFu) && U.cfg[ULTRA_CFG_PX_POLICY - ULTRA_CFG_PX_FIRST] == 7u, "policy bits not masked");
    CHECK(!udfx_config(&U, 128, 1) && !udfx_config(&U, 142, 1) && !udfx_config(&U, 5, 1), "foreign ids taken");
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_SMOOTH, 4) && udfx_config(&U, ULTRA_CFG_PX_SMOOTH, ULTRA_PXS_DITHER) && udfx_config(&U, ULTRA_CFG_PX_SMOOTH, ULTRA_PXS_OFF) &&
          udfx_config(&U, ULTRA_CFG_PX_SMOOTH, ULTRA_PXS_AUTO) && !udfx_config(&U, ULTRA_CFG_PX_SMOOTH, ULTRA_PXS_AUTO), "CONFIG 141: 0..3 only, an unchanged value reports no change");
    {
        const uint8_t r0 = U.reconf, b0 = U.rebegin;
        CHECK(udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_CHAIN) && !udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, 3) &&
              U.cfg[ULTRA_CFG_PX_LINE_MODE - ULTRA_CFG_PX_FIRST] == ULTRA_PXLM_CHAIN && U.reconf == r0 && U.rebegin == b0, "CONFIG 140: 0..2, no re-configure");
    }
    (void)udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_AUTO);
    CHECK(!udfx_config(&U, ULTRA_CFG_PX_PIXELS, 70000), "a 17-bit value accepted");
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, 5000);
    /* the chip changes the order's meaning: an order for 5 channels on a 3-byte chip falls back to the chip's own */
    (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, ULTRA_PXC_WS2812B_V5);
    {                                                    /* (derived by the service when not local) */
        D.wpos = 100;
        udfx_service(&U, &D, 1, 0, 256);
    }
    CHECK(U.npx == ux_max_px_test(3), "5000 pixels on a 3-byte chip became %u, not %u", U.npx, ux_max_px_test(3));
    CHECK(U.pack.ord[0] == 1 && U.pack.ord[1] == 0 && U.pack.ord[2] == 2, "an order that doesn't fit the chip: %u %u %u", U.pack.ord[0], U.pack.ord[1], U.pack.ord[2]);
    CHECK(U.max_px == ux_max_px_test(3), "max pixels %u", U.max_px);
    (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, ULTRA_PXC_SK6812_RGBW);
    CHECK(UX_POOL_BYTES == 2000u && ux_max_px_test(4) == 500u && ux_max_px_test(3) == 600u && ux_max_px_test(5) == 400u, "the format caps");
    /* PIXEL_FX */
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, 50);
    udfx_service(&U, &D, 2, 0, 256);
    slot(s, 0, 7, FX_RAINBOW, 0, 200, 0, 0);
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(U.seg[0].run && U.seg[0].p.effect == FX_RAINBOW && !U.want_local, "a PIXEL_FX did not start the effect, or asked for LOCAL before the service did");
    udfx_service(&U, &D, 3, 0, 256);                     /* (the service asks for LOCAL, in the call that computes the grid's first word) */
    CHECK(U.want_local, "the service did not ask for LOCAL"); 
    slot(s, 0, 7, FX_FIRE, 0, 200, 0, 0);
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(U.seg[0].p.effect == FX_RAINBOW, "the same SEQ applied twice");
    slot(s, 0, 8, FX_FIRE, 0, 200, 0, 0);
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(U.seg[0].p.effect == FX_FIRE, "a new SEQ was not applied");
    slot(s, 4, 1, FX_SINELON, 0, 200, 0, 0);               /* LIGHT 4: reserved */
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(!U.seg[1].run && !U.seg[0].run == 0, "LIGHT 4 was applied");
    s[1] = (uint8_t)(0xEu << 4); ultra_slot_seal(s);     /* LIGHT 14 */
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(U.seg[0].p.effect == FX_FIRE, "LIGHT 14 was applied");
    slot(s, 1, 1, 200, 0, 255, 0, 0);                    /* an unknown effect id (200): Solid */
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(U.seg[1].run && U.eng.seg[1].cur.id == FX_SOLID, "an unknown effect id is %u, not Solid", U.eng.seg[1].cur.id);
    slot(s, ULTRA_LIGHT_BOARD, 9, FX_SINELON, 0, 255, 0, 0);   /* the whole strip: segment 0, the others stopped */
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(U.seg[0].run && U.seg[0].p.effect == FX_SINELON && !U.seg[1].run, "LIGHT 15: segment 1 still runs");
    slot(s, 2, 2, FX_SOLID, 5000, 255, 0, 0);            /* a START 5 s ahead waits */
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(!U.seg[2].run && U.seg[2].pend_have && U.seg[2].pend_at == 1000u + 5000u * 1000u, "a future START did not wait");
    slot(s, 0, 10, FX_SOLID, 0, 255, 0, ULTRA_PXF_STORE | ULTRA_PXF_LOCK);
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(U.stored_have == 1u && U.save_due && U.lock, "STORE / LOCK");
    slot(s, 0, 11, 0, 0, 0, 0, ULTRA_PXF_STORE);         /* effect 0 + STORE: back to the stream, the stored one cleared */
    udfx_pixel_fx(&U, s, 1000);
    command_boundary();
    CHECK(!U.seg[0].run && !U.seg[2].pend_have && U.want_stream && U.stored_have == 0u, "effect 0 + STORE");
    /* page 0x81 */
    memset(d, 0, sizeof d);
    CHECK(udfx_page(&U, 0, d) == 0, "page 0x81 part 0");
    CHECK(ultra_get16(d + ULTRA_PXG_CATALOGUE) == UX_CATALOGUE_VERSION && d[ULTRA_PXG_SEGMENTS] == UX_SEGS &&
          d[ULTRA_PXG_CHIPS] == ULTRA_PXC_COUNT && d[ULTRA_PXG_MODE] == ULTRA_PXM_STREAM, "page 0x81 part 0 fields");
    CHECK(ultra_get16(d + ULTRA_PXG_MAX_PIXELS) == UX_POOL_BYTES / 4u, "the page's max pixels %u", ultra_get16(d + ULTRA_PXG_MAX_PIXELS));
    CHECK(ultra_get16(d + ULTRA_PXG_CONFIG_CRC) == ultra_px_config_crc(U.cfg), "the page's CONFIG CRC");
    memset(d, 0, sizeof d);
    CHECK(udfx_page(&U, 1, d) == 0, "part 1");
    for (i = 0, x = 0; i < 192u; i++) {
        x += (d[i >> 3] >> (i & 7u)) & 1u;
    }
    CHECK(x == FX_COUNT - 1u && !(d[0] & 1u), "part 1 names %u effects, the engine has %u (without the stream, id 0)", x, (uint32_t)FX_COUNT - 1u);
    memset(d, 0, sizeof d);
    CHECK(udfx_page(&U, 2, d) == 0, "part 2");
    for (i = 0, x = 0; i < 192u; i++) {
        x += (d[i >> 3] >> (i & 7u)) & 1u;
    }
    CHECK(x == PAL_COUNT, "part 2 names %u palettes, the engine has %u", x, (uint32_t)PAL_COUNT);
    CHECK(udfx_page(&U, 3, d) == -1, "part 3 answered");
    printf("  config ranges, SEQ once, LIGHT 4..14 ignored, LIGHT 15, unknown effect = Solid, future START waits, STORE / LOCK, page 0x81\n");
}

/* a slow main loop: the render gets a chunk every N ticks; frames late, never torn */
static void test_slow(void)
{
    sim_t sc;
    const uint32_t T = 2250u * 150u;
    uint32_t n, k, ok = 0;
    const act_t script[] = { { 1, A_ANCHOR }, { 1, A_FX, 0, 1, FX_PLASMA, 255, 0, 0, 0, 0, 0, COL }, { 0, A_END } };
    printf("[14] a slow renderer (a chunk of 32 px every 60 ticks, 3 ms) on a 60 fps grid: late frames, never a torn one\n");
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u; sc.steps = 1; sc.main_div = 60;
    line_clear();
    make_rx(0, 0, 200, 60, 0);
    g_name = "slow";
    run_script(&sc, script);
    n = decode(out, 0, T - 600u, 48, 13, 25);
    for (k = 0; k < n; k++) {
        ok += fr[k].nbits == 200u * 24u && (fr[k].exact & 1u) && k < n_exp && memcmp(fr[k].b, exp_fb[k], 600) == 0;
    }
    CHECK(n >= 3u && ok == n, "%u of %u frames whole and equal to the rendered ones", ok, n);
    CHECK(U.e.late > 0u, "no late frame counted");
    printf("  %u frames, all whole; late frames counted: %u\n", n, U.e.late);
}

/* STREAM_WINS: a controller's stream takes the strip back from an unlocked effect (at its second frame, inside a latch); LOCK
 * keeps the effect; the hub's own frames before the effect started don't count as a controller taking over */
#define HOLE_PHASES 12u
static void test_wins(void)
{
    uint8_t hb[120];
    uint32_t pass, hub_after, loc, hub_before, k, T = 2250u * 125u;
    printf("[15] STREAM_WINS and LOCK: a controller's stream against an effect the hub started\n");
    for (pass = 0; pass < 3u + HOLE_PHASES; pass++) {
        sim_t sc;
        uint32_t n, local_end_w = 0, f0 = (uint32_t)g_fail;
        g_hole_us = pass >= 3u ? 60u : 0u;                          /* passes 3 ..: a controller at full rate whose frames hold a 60 us LOW */
        g_hub_period = pass >= 3u ? 3600u : 2250u * 3u;             /* ... (a refill) every 1.6 ms, and a V5 strip, which does not latch on it */
        g_dec_latch = pass >= 3u ? 281u * 36u : 104u * 16u;
        memset(&sc, 0, sizeof sc);
        sc.T = T; sc.gap_max = 135u + 112u; sc.steps = 2; sc.cmd = cmd_switch;
        build_switch_line(pass >= 3u ? (pass - 3u) * 300u : pass * 700u, hb, 0);
        make_rx(0, 0, 50, 0, 0);
        g_policy = pass == 0u ? 0u : ULTRA_PXP_STREAM_WINS;       /* 0: no policy: the effect stays; 1: wins; 2: wins but locked */
        g_fxflags = pass == 2u ? ULTRA_PXF_LOCK : 0u;
        g_t_stream_ms = 100000u;
        g_name = "wins";
        simulate(&sc);
        g_ntim = 2;
        g_tim[0].s = 45; g_tim[0].t0h = 14; g_tim[0].t1h = 29;
        g_tim[1].s = 48; g_tim[1].t0h = 13; g_tim[1].t1h = 25;
        n = decode_all(out, 0, T - 600u, 0, 14, 29);
        g_ntim = 0;
        while (n > 0u && !fr[n - 1u].closed) {
            n--;
        }
        hub_before = hub_after = loc = 0;
        for (k = 0; k < n; k++) {
            if (fr[k].nbits == 120u * 8u && memcmp(fr[k].b + 1, hb + 1, 119) == 0 && ((fr[k].exact & 1u) || g_hole_us != 0u)) {   /* (the hole is no exact bit) */
                if (loc == 0u) hub_before++; else hub_after++;
            } else if (fr[k].nbits == 50u * 24u && (fr[k].exact & 2u) && loc < n_exp && memcmp(fr[k].b, exp_fb[loc], 150) == 0) {
                loc++;
                local_end_w = fr[k].start_w;
            } else {
                CHECK(0, "pass %u: frame %u at word %u: %u bits (not a whole hub or local frame)", pass, k, fr[k].start_w, fr[k].nbits);
            }
        }
        (void)local_end_w;
        CHECK(hub_before >= 5u && loc > 10u, "pass %u: hub %u before the effect, %u local frames", pass, hub_before, loc);
        if (pass == 1u || pass >= 3u) {
            CHECK(hub_after >= 14u && U.switches == 2u && !U.local, "STREAM_WINS: %u hub frames after, %u switches", hub_after, U.switches);
        } else {
            CHECK(hub_after == 0u && U.switches == 1u && U.local, "%s: %u hub frames after the effect, %u switches", pass ? "LOCK" : "no policy",
                  hub_after, U.switches);
        }
        if (pass <= 3u || g_fail != (int)f0) {
            printf("  %-34s hub frames before %2u, local %3u, hub frames after the effect: %2u%s\n",
                   pass == 0u ? "no policy (the effect stays)" : pass == 1u ? "STREAM_WINS (the stream returns)" :
                   pass == 2u ? "STREAM_WINS + LOCK (stays)" : "full rate, frames with a 60 us hole",
                   hub_before, loc, hub_after, g_fail == (int)f0 ? "" : "  FAILED");
        }
    }
    g_policy = g_fxflags = 0;
    printf("  (the full-rate hole runs: %u hub phases, every frame on the strip a whole hub or local one)\n", HOLE_PHASES);
    g_hole_us = 0;
    g_hub_period = 2250u * 3u;
    g_dec_latch = 104u * 16u;
    g_t_stream_ms = 72;
}

/* a hand-back (PIXEL_FX 0) to a stream whose gaps are only the datasheet's reset: an SK6812 (80 us) hub back to back never gives
 * the chip's latch (190 words), so the receiver took dline's after 100 ms (ud-fx-verify's "handback": 10 of 30 phases stuck) */
static void test_handback(void)
{
    uint8_t hb[120];
    sim_t sc;
    const uint32_t T = 2250u * 230u;
    uint32_t n, k, hub = 0, loc = 0, foreign = 0, first_hub_ms = 0;
    printf("[19] a hand-back to a stream with the datasheet's exact reset (SK6812, 80 us, frames back to back)\n");
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u + 112u; sc.steps = 2; sc.cmd = cmd_switch;
    g_hub_period = 2880u;                                          /* 1.2 ms of frame + 80 us */
    g_dec_latch = 75u * 36u;                                       /* the strip latches at 80 us */
    g_line_ms = 230u;
    build_switch_line(0, hb, 0);
    make_rx(2, 0, 50, 0, 0);
    g_policy = 0;
    g_fxflags = 0;
    g_t_stream_ms = 72;
    g_name = "handback";
    simulate(&sc);
    g_ntim = 2;
    g_tim[0].s = 45; g_tim[0].t0h = 14; g_tim[0].t1h = 29;
    g_tim[1].s = 48; g_tim[1].t0h = 12; g_tim[1].t1h = 26;
    n = decode_all(out, 0, T - 600u, 0, 14, 29);
    g_ntim = 0;
    while (n > 0u && !fr[n - 1u].closed) {
        n--;
    }
    for (k = 0; k < n; k++) {
        if (fr[k].nbits == 120u * 8u && memcmp(fr[k].b + 1, hb + 1, 119) == 0 && (fr[k].exact & 1u)) {
            if (loc > 0u && hub == 0u) {
                first_hub_ms = fr[k].start_w / 2250u;
            }
            hub++;
        } else if (fr[k].nbits == 50u * 24u && (fr[k].exact & 2u) && loc < n_exp && memcmp(fr[k].b, exp_fb[loc], 150) == 0) {
            loc++;
        } else if (hub == 0u && loc == 0u) {
            hub = 0;                                               /* (the hub's frames before the effect: not counted) */
        } else {
            foreign++;
            CHECK(0, "frame %u at word %u: %u bits (not a whole hub or local frame)", k, fr[k].start_w, fr[k].nbits);
        }
    }
    CHECK(!U.local && U.switches == 2u, "still LOCAL after the hand-back (%u switches)", U.switches);
    CHECK(hub >= 30u && loc >= 20u && foreign == 0u, "%u hub frames after the hand-back, %u local frames, %u foreign", hub, loc, foreign);
    CHECK(first_hub_ms > 72u + 99u && first_hub_ms < 72u + 160u, "the stream came back at %u ms (the fallback is 100 ms after the encoder is idle)", first_hub_ms);
    printf("  local frames %u, then the stream from %u ms: %u whole hub frames, %u foreign\n", loc, first_hub_ms, hub, foreign);
    g_hub_period = 2250u * 3u;
    g_dec_latch = 104u * 16u;
    g_line_ms = 120;
    g_t_stream_ms = 72;
}

/* the protocol's vectors (review-r4/ud-fx/HANDOFF-protocol.md): the slot an independent encoder made, the CONFIG builder */
static void test_vectors(void)
{
    static const char want[] = "0b204d09fffffa240102fe0a0b0c0d0e0f101112c802bc000c012c1e050660d2";
    ultra_pixel_fx_t f, g;
    uint8_t s[32], ids[5] = { ULTRA_CFG_PX_CHIP, 5, 142, ULTRA_CFG_PX_VLENGTH, ULTRA_CFG_PX_LINE_MODE };
    uint16_t vals[5] = { 3, 7, 9, 1234, 2 }, zeros[ULTRA_CFG_PX_COUNT] = { 0 };
    char hex[65];
    uint32_t i;
    printf("[16] the protocol's vectors\n");
    memset(&f, 0, sizeof f);
    f.effect = 9; f.speed = 1; f.intensity = 2; f.palette = ULTRA_PX_PAL_COLORS;
    for (i = 0; i < 9u; i++) f.c[i / 3u][i % 3u] = (uint8_t)(10u + i);
    f.brightness = 200; f.flags = ULTRA_PXF_STORE | ULTRA_PXF_MIRROR | (ULTRA_PXW_BRIGHT << ULTRA_PXF_WHITE_SHIFT);
    f.start_ms = -1500; f.transition_ms = 700; f.seg_start = 12; f.seg_len = 300; f.custom1 = 5; f.custom2 = 6;
    ultra_slot_pixel_fx(s, 2, 77, &f);
    for (i = 0; i < 32u; i++) sprintf(hex + 2u * i, "%02x", s[i]);
    CHECK(strcmp(hex, want) == 0, "the PIXEL_FX vector: %s", hex);
    CHECK(ultra_pixel_fx_parse(s, &g) == 2 && g.effect == 9 && g.start_ms == -1500 && g.seg_len == 300 && g.flags == f.flags &&
          g.c[2][2] == 18 && g.transition_ms == 700 && g.custom2 == 6, "the parse");
    f.flags = ULTRA_PXF_RESTART | ULTRA_PXF_LOCK;
    ultra_slot_pixel_fx(s, 2, 78, &f);
    CHECK(s[ULTRA_PX_FLAGS] == 0x60u && ultra_pixel_fx_parse(s, &g) == 2 && g.flags == 0x60u, "the RESTART bit on the wire: %02x", s[ULTRA_PX_FLAGS]);
    f.flags = 0xFFu;
    ultra_slot_pixel_fx(s, 2, 79, &f);
    CHECK(s[ULTRA_PX_FLAGS] == 0x7Fu, "bit 7 sent: %02x", s[ULTRA_PX_FLAGS]);
    s[1] = (uint8_t)(0x4u << 4);
    CHECK(ultra_pixel_fx_parse(s, &g) == -1, "LIGHT 4 parsed");
    s[1] = (uint8_t)(ULTRA_LIGHT_BOARD << 4);
    CHECK(ultra_pixel_fx_parse(s, &g) == (int)ULTRA_LIGHT_BOARD, "LIGHT 15 not parsed");
    ultra_slot_config_px(s, 3, ids, vals, 5);
    CHECK(ultra_slot_check(s) == 0 && s[3] == 129 && s[6] == 137 && ultra_get16(s + 7) == 1234 && s[9] == 140 && ultra_get16(s + 10) == 2 && s[12] == 0,
          "CONFIG for 129-141: only those ids");
    ultra_slot_config(s, 3, ids, vals, 5);
    CHECK(s[3] == 5 && s[6] == 0 && ultra_get16(s + 4) == 7, "the official CONFIG builder passed a hardware-specific id");
    CHECK(ULTRA_CFG_PX_COUNT == 13u && ultra_px_config_crc(zeros) != 0xECDDu, "CONFIG CRC of thirteen zeros %04x", ultra_px_config_crc(zeros));
    CHECK(ULTRA_PXG_CONFIG_CRC + 2u == 24u && ULTRA_PXG_LATE < ULTRA_PXG_CONFIG_CRC && ULTRA_HWCAP_PIXEL_FX == 0x80u &&
          ULTRA_PIXEL_FX == 0x0B && ULTRA_TYPES == 0x0B, "constants");
    printf("  PIXEL_FX slot, parse, CONFIG 129-141 builder, CRC\n");
}

/* a stored effect running on the free-running clock when the hub's first header arrives: no jump, no stop */
static void test_rebase(void)
{
    uint8_t rec[UX_REC_BYTES];
    uint32_t t_before, t_after, i;
    printf("[17] the first anchor while a stored effect runs on the free-running clock\n");
    g_name = "rebase";
    build_rec(rec, ULTRA_PXP_POWER_ON, 0);
    udfx_init(&U);
    udfx_load(&U, rec);
    memset(&D, 0, sizeof D);
    dline_init(&D, 0);
    D.direct = 0; D.rec = 0;
    D.wpos = 1000;
    U.npx = 40;
    udfx_service(&U, &D, 1, 0, 256);                         /* the clock starts at word 1000 */
    D.st.frames = 0; D.st.marks = 0;
    D.edge_w = 0;
    D.wpos = 2250u * 2100u;                                  /* 2.1 s of a quiet line */
    udfx_service(&U, &D, 2100, 0, 256);
    command_boundary();
    CHECK(U.seg[0].run && U.want_local, "the stored effect did not start");
    U.local = 1;                                             /* (entered) */
    U.e.w = D.wpos; U.e.state = WS_LOW; U.e.low_w = 1000; U.e.next_w = D.wpos; U.e.reset_w = 680;
    for (i = 0; i < 3u; i++) {                               /* a few frames on the free clock */
        D.wpos += 2250u * 20u;
        U.begun = 0;
        U.e.ready = 0;
        U.e.w = D.wpos; U.e.next_w = D.wpos;
        udfx_service(&U, &D, 2100u + 20u * (i + 1u), 0, 256);
        (void)udfx_render(&U);
    }
    t_before = U.eng.seg[0].cur.t;
    D.wpos += 2250u * 20u;
    udfx_anchor(&U, 0x7FFFFF00u, D.wpos - 3u);               /* the hub's clock is somewhere else entirely */
    U.begun = 0; U.e.ready = 0; U.e.w = D.wpos; U.e.next_w = D.wpos;
    D.wpos += 2250u * 20u;
    udfx_service(&U, &D, 2200, 0, 256);
    U.e.w = D.wpos; U.e.next_w = D.wpos;
    while (udfx_render(&U)) {
    }
    t_after = U.eng.seg[0].cur.t;
    CHECK(t_after > t_before && t_after - t_before < 120u, "the effect time went from %u to %u ms over ~40 ms of line time", t_before, t_after);
    printf("  effect time %u -> %u ms across the first anchor\n", t_before, t_after);
}

/* the hub renews an old START before its signed 32-bit ms runs out: the same slot with a new START changes nothing */
static void test_renewal(void)
{
    uint8_t s[32];
    uint32_t a0;
    printf("[18] a START renewal keeps the running phase\n");
    g_name = "renewal";
    udfx_init(&U);
    D.wpos = 0; D.edge_w = 0;
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, 50);
    udfx_service(&U, &D, 2, 0, 256);
    slot(s, 0, 1, FX_RAINBOW, -5000, 255, 0, 0);
    udfx_pixel_fx(&U, s, 1000000u);
    command_boundary();
    a0 = U.seg[0].anchor;
    CHECK(U.seg[0].run && a0 == U.eng.seg[0].anchor, "the effect did not start with its anchor");
    slot(s, 0, 2, FX_RAINBOW, -4000, 255, 0, 0);         /* the same slot, a later zero: a renewal */
    udfx_pixel_fx(&U, s, 1000000u);
    command_boundary();
    CHECK(U.seg[0].anchor == a0 && U.eng.seg[0].anchor == a0, "a renewal moved the anchor (%u -> %u, engine %u)", a0, U.seg[0].anchor, U.eng.seg[0].anchor);
    slot(s, 0, 3, FX_RAINBOW, -4000, 200, 0, 0);         /* another brightness with it: the new anchor is taken */
    udfx_pixel_fx(&U, s, 1000000u);
    command_boundary();
    CHECK(U.seg[0].anchor != a0 && U.seg[0].anchor == U.eng.seg[0].anchor, "a change with a new START kept the old anchor");
    {                                                   /* RESTART: not a renewal */
        const uint32_t a1 = U.seg[0].anchor, sd = U.eng.seg[0].cur.seed;
        slot(s, 0, 4, FX_RAINBOW, -4000, 200, 0, ULTRA_PXF_RESTART);   /* the same START: the random effects re-seed, no time restarts */
        udfx_pixel_fx(&U, s, 1000000u);
    command_boundary();
        CHECK(U.seg[0].anchor == a1 && U.eng.seg[0].cur.seed != sd && U.seg[0].restarts == 1u, "RESTART with the same START: anchor %u -> %u, seed %08x -> %08x",
              a1, U.seg[0].anchor, sd, U.eng.seg[0].cur.seed);
        CHECK(!(U.seg[0].p.flags & ULTRA_PXF_RESTART), "RESTART kept in the running flags");
        slot(s, 0, 5, FX_RAINBOW, -3000, 200, 0, ULTRA_PXF_RESTART | ULTRA_PXF_STORE);   /* a new START too: the time begins at it; STORE keeps no RESTART */
        udfx_pixel_fx(&U, s, 1000000u);
    command_boundary();
        CHECK(U.seg[0].anchor == a1 + 1000u && U.seg[0].restarts == 2u, "RESTART with a new START: anchor %u -> %u", a1, U.seg[0].anchor);
        CHECK(U.stored_have == 1u && !(U.stored[0].flags & ULTRA_PXF_RESTART) && (U.stored[0].flags & ULTRA_PXF_STORE), "the stored effect kept RESTART");
    }
    printf("  a renewal keeps the anchor, any other change takes the new one\n");
}

/* ------------------------------------------------------------------------------------------------ smoothing (CONFIG 141) */
/* a short run on the host grid (Solid c1 = 255,80,0): the frame indexes and the bytes of the frames it sent */
static uint32_t smooth_run(uint32_t fps, uint32_t mode, double hsi, uint32_t *idx, uint8_t (*bytes)[60], uint32_t maxn)
{
    sim_t sc;
    uint32_t T = 2250u * 400u, n, k, got = 0, per_us = 1000000u / fps;
    memset(&sc, 0, sizeof sc);
    sc.T = T; sc.gap_max = 135u + 112u; sc.steps = 2; sc.cmd = cmd_grid; sc.hsi = hsi; sc.phase = 1234;
    line_clear();
    make_rx(0, 0, 20, fps, hsi);
    (void)udfx_config(&U, ULTRA_CFG_PX_SMOOTH, mode);
    prime_clock();
    g_fx = FX_SOLID; g_bri = 255; g_trans = 0; g_flags = 0;
    g_name = "smoothing";
    simulate(&sc);
    n = decode(out, 0, T - 600u, 48, 13, 25);
    for (k = 2; k < n && got < maxn; k++) {
        if (fr[k].nbits == 20u * 24u) {
            idx[got] = (host_of_word(fr[k].start_w) + per_us / 2u) / per_us;
            memcpy(bytes[got], fr[k].b, 60);
            got++;
        }
    }
    return got;
}

static void test_smooth(void)
{
    static const struct { uint32_t fps, mode, want; } a[] = { { 60, ULTRA_PXS_AUTO, FX_SM_DITHER }, { 30, ULTRA_PXS_AUTO, FX_SM_ROUND },
        { 50, ULTRA_PXS_AUTO, FX_SM_DITHER }, { 45, ULTRA_PXS_AUTO, FX_SM_ROUND }, { 100, ULTRA_PXS_AUTO, FX_SM_DITHER },
        { 60, ULTRA_PXS_ROUND, FX_SM_ROUND }, { 30, ULTRA_PXS_DITHER, FX_SM_DITHER }, { 60, ULTRA_PXS_OFF, FX_SM_8BIT },
        { 30, ULTRA_PXS_OFF, FX_SM_8BIT } };
    static uint32_t ia[200], ib[200];
    static uint8_t ba[200][60], bb[200][60];
    uint32_t c, na, nb, i, j, same = 0, bad = 0, lo = 255, hi = 0;
    double mean = 0.0;
    printf("[17] pixel smoothing (CONFIG 141): the mode at each frame rate, the dither identical on receivers whose clocks differ, its average\n");
    for (c = 0; c < sizeof a / sizeof a[0]; c++) {
        (void)smooth_run(a[c].fps, a[c].mode, 0.0, ia, ba, 4);
        CHECK(U.pack.smooth == a[c].want, "smoothing %u at %u fps: the packer runs mode %u, expected %u", a[c].mode, a[c].fps, U.pack.smooth, a[c].want);
        CHECK(U.pack.dmin == (a[c].mode == ULTRA_PXS_AUTO ? ULTRA_PXS_AUTO_MIN_LEVEL : 0u), "smoothing %u: dmin %u (AUTO: the header's %u, the others 0)", a[c].mode, U.pack.dmin, (unsigned)ULTRA_PXS_AUTO_MIN_LEVEL);
    }
    na = smooth_run(200, ULTRA_PXS_DITHER, 0.0, ia, ba, 200);
    nb = smooth_run(200, ULTRA_PXS_DITHER, 0.022, ib, bb, 200);
    CHECK(na > 40u && nb > 40u, "smoothing: %u / %u frames", na, nb);
    for (i = 0; i < na; i++) {
        for (j = 0; j < nb; j++) {
            if (ia[i] == ib[j]) {                      /* the same host frame on both receivers: the same bytes */
                same++;
                bad += memcmp(ba[i], bb[j], 60) != 0;
            }
        }
    }
    CHECK(same >= 30u && bad == 0u, "two receivers (hsi 0 / +2.2 %%): %u common frames, %u differ", same, bad);
    for (i = 0; i < na; i++) {                          /* the green channel (GRB: byte 0): dithered between two levels around the 16-bit value */
        lo = ba[i][0] < lo ? ba[i][0] : lo;
        hi = ba[i][0] > hi ? ba[i][0] : hi;
        mean += ba[i][0];
    }
    mean /= na;
    {
        const double exact = fx_gamma16_of(80) / 256.0;
        CHECK(hi - lo == 1u && fabs(mean - exact) < 0.2, "dither: green %u..%u, mean %.3f, the 16-bit value %.3f", lo, hi, mean, exact);
        {                                               /* the record (format 3): CONFIG 140 and 141 share a slot and come back */
            uint8_t rec[UX_REC_BYTES];
            udfx_t b;
            udfx_init(&U);
            (void)udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_CHAIN);
            (void)udfx_config(&U, ULTRA_CFG_PX_SMOOTH, ULTRA_PXS_OFF);
            (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, 300);
            udfx_rec_pack(&U, 9, rec);
            memcpy(&b, &U, sizeof b);
            udfx_init(&U);
            CHECK(udfx_rec_valid(rec) && rec[2] == UX_REC_FORMAT && UX_REC_FORMAT == 3u, "record format 3 is not valid");
            udfx_load(&U, rec);
            CHECK(memcmp(U.cfg, b.cfg, sizeof U.cfg) == 0 && U.cfg[ULTRA_CFG_PX_SMOOTH - ULTRA_CFG_PX_FIRST] == ULTRA_PXS_OFF &&
                  U.cfg[ULTRA_CFG_PX_LINE_MODE - ULTRA_CFG_PX_FIRST] == ULTRA_PXLM_CHAIN, "CONFIG 140 / 141 did not come back from the record");
            rec[2] = 2;                                 /* a record of the old format is not taken (and its CRC no longer matches either) */
            CHECK(!udfx_rec_valid(rec), "a format 2 record is valid");
        }
        printf("  200 fps dither: green %u..%u over %u frames, mean %.3f (the 16-bit value %.3f); %u common frames on two receivers, identical\n", lo, hi, na, mean, exact, same);
    }
}

/* ------------------------------------------------------------------------------------------------ main */
static void test_caps(void)
{
    uint32_t c, j;
    printf("[caps] CONFIG boundaries, format changes, record reload, page maximum\n");
    for (c = 0; c < ULTRA_PXC_COUNT; c++) {
        const uint32_t cap = ux_max_px_test(CHIP[c].bpp);
        const uint32_t counts[] = { 0, 1, cap - 1u, cap, cap + 1u, 65535u };
        for (j = 0; j < sizeof counts / sizeof counts[0]; j++) {
            uint8_t page[24], rec[UX_REC_BYTES];
            const uint32_t want_px = counts[j] < cap ? counts[j] : cap;
            make_rx(c, 0, counts[j], 1000, 0);
            D.wpos = 100; D.edge_w = 0;
            udfx_service(&U, &D, 1, 0, 256);
            CHECK(U.npx == want_px && U.max_px == cap && U.eng.npx == want_px,
                  "chip %u request %u: pixels %u max %u", c, counts[j], U.npx, U.max_px);
            CHECK(U.npx * U.chip.bpp <= 2000u, "frame exceeds 2000 wire bytes");
            memset(page, 0, sizeof page); udfx_page(&U, 0, page);
            CHECK(ultra_get16(page + ULTRA_PXG_MAX_PIXELS) == cap, "page maximum chip %u", c);
            udfx_rec_pack(&U, 1, rec);
            udfx_init(&U); udfx_load(&U, rec);
            CHECK(U.npx == want_px && U.max_px == cap, "reload bypasses cap chip %u", c);
        }
    }
    /* The raw count survives chip changes; each derived format has its own cap. */
    make_rx(0, 0, 65535, 60, 0);
    for (c = 0; c < ULTRA_PXC_COUNT; c++) {
        (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, c);
        D.wpos = 100; udfx_service(&U, &D, c + 1u, 0, 256);
        CHECK(U.npx == ux_max_px_test(CHIP[c].bpp), "chip change bypasses cap %u", c);
    }
}

static void test_cap_effects(void)
{
    uint32_t ci, fx, stall;
    printf("[caps] every effect on all eight chips at 600 / 500 / 400, normal + 16 ms producer stall\n");
    for (ci = 0; ci < ULTRA_PXC_COUNT; ci++) for (fx = 1; fx < FX_COUNT; fx++) for (stall = 0; stall < 2u; stall++) {
        const uint32_t c = ci, np = ux_max_px_test(CHIP[c].bpp);
        const int before = g_fail;
        sim_t sc;
        uint32_t n;
        memset(&sc, 0, sizeof sc); sc.T = 2250u * 220u; sc.steps = 64; sc.cmd = cmd_start;
        sc.flash_from = 2250u * 30u; sc.flash_to = stall ? sc.flash_from + 2250u * 16u : 0;
        line_clear(); make_rx(c, 0, np, 60, 0);
        g_fx = fx; g_bri = 255; g_trans = 0; g_flags = 0; g_name = "caps"; g_printed = 0;
        simulate(&sc); n = decode(out, 0, sc.T - 600u, CHIP[c].s, CHIP[c].t0h, CHIP[c].t1h);
        check_frames(n, 0, np * CHIP[c].bpp, &CHIP[c], "caps");
        CHECK(n >= 4u, "no sustained capped output chip %u effect %u", c, fx);
        printf("  cap chip=%u npx=%u fx=%u stall=%u frames=%u failed=%d\n", c, np, fx, stall * 16u, n, g_fail - before);
    }
}

static void test_cap_rates(void)
{
    static const uint32_t resets[] = { 680, 450, 480, 480, 680, 680, 680, 450 };
    uint32_t c, mode;
    printf("[rates] decoded wire: cap of each chip and 320 RGBW; FPS 0 / default 60 / 1000\n");
    for (c = 0; c < ULTRA_PXC_COUNT + 2u; c++) for (mode = 0; mode < 3u; mode++) {
        const uint32_t chip = c < ULTRA_PXC_COUNT ? c : c == ULTRA_PXC_COUNT ? 3u : 4u;
        const uint32_t np = c < ULTRA_PXC_COUNT ? ux_max_px_test(CHIP[chip].bpp) : 320u;
        const uint32_t fps = mode == 0u ? 0u : mode == 1u ? 60u : 1000u;
        const uint32_t wire_w = np * CHIP[chip].bpp * CHIP[chip].s / 2u + resets[chip];
        sim_t sc;
        uint32_t n, k;
        memset(&sc, 0, sizeof sc); sc.T = MAXW; sc.steps = 64; sc.cmd = cmd_start;
        line_clear(); make_rx(chip, 0, np, fps, 0);
        g_fx = FX_SOLID; g_bri = 255; g_trans = 0; g_flags = 0; g_name = "rates"; g_printed = 0;
        simulate(&sc); n = decode(out, 0, sc.T - 600u, CHIP[chip].s, CHIP[chip].t0h, CHIP[chip].t1h);
        check_frames(n, 0, np * CHIP[chip].bpp, &CHIP[chip], "rates");
        CHECK(n >= 5u, "too few rate samples");
        for (k = 2; k < n; k++) {
            CHECK(fr[k].start_w - fr[k-1].start_w >= wire_w, "wire/reset shortened");
            CHECK(fps != 60u || fr[k].start_w - fr[k-1].start_w >= 37496u, "exceeds nominal target");
        }
        if (n >= 5u) printf("  rate chip=%u npx=%u request=%u wire_max=%.6f observed=%.6f\n", chip, np, fps,
                            2250000.0 / wire_w, (n - 3u) * 2250000.0 / (fr[n-1].start_w - fr[2].start_w));
    }
}

static int want(const char *sel, const char *name)
{
    return sel[0] == '*' || strncmp(sel, name, strlen(sel)) == 0;
}

int main(int argc, char **argv)
{
    const char *sel = argc > 1 ? argv[1] : "*";
    world_reserve(MAXW);
    if (want(sel, "caps")) { test_caps(); test_cap_effects(); }
    if (want(sel, "rates")) test_cap_rates();
    if (want(sel, "chips")) test_chips();
    if (want(sel, "orders")) test_orders();
    if (want(sel, "grid")) test_grid();
    if (want(sel, "switch")) {
        test_switch(0, 1, 1, 0, "stream -> local -> stream at latches, a poll in local mode (WS2812B V5)");
        test_switch(0, 60, 1, 0, "the same, the hub's frame phase swept");
    }
    if (want(sel, "poll")) test_switch(0, 12, 1, 1, "answered polls in local mode (the burst's DE high, the retake) over 12 hub phases");
    if (want(sel, "flash")) test_flash();
    if (want(sel, "deadline")) test_deadline();
    if (want(sel, "fault")) { test_fault(); g_fault_at = 0; }
    if (want(sel, "standalone")) test_standalone();
    if (want(sel, "limiter")) test_limiter();
    if (want(sel, "ramp")) test_ramp();
    if (want(sel, "clock")) test_clock();
    if (want(sel, "record")) test_record();
    if (want(sel, "cmds")) test_cmds();
    if (want(sel, "slow")) test_slow();
    if (want(sel, "wins")) test_wins();
    if (want(sel, "handback")) test_handback();
    if (want(sel, "vectors")) test_vectors();
    if (want(sel, "rebase")) test_rebase();
    if (want(sel, "renewal")) test_renewal();
    if (want(sel, "smooth")) test_smooth();
    printf("udfx: %ld checks, %d failed\n", g_checks, g_fail);
    return g_fail != 0;
}
