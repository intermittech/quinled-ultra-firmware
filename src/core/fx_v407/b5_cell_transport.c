/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Batch B5 "cell_transport": cellular automata, transport, piles and sorting. Ids 64, 69, 71, 72,
 * 73, 74, 84, 88, 89, 91, 105; manifest/b5_cell_transport.json, JS fx/v407/b5_cell_transport.js (same names, same bytes).
 *
 * Every effect runs a model on the layer's simulation ticks (tick_ms 10). Each tick adds the speed's share of a model step
 * (Q16) to h[H_ACC]; a whole step is one job sliced over the cells, reading one plane and writing the other (ping-pong:
 * any slicing gives the same planes). Rendering blends the last two steps by the step's fraction (u[0]), so a model that
 * steps a few times a second still moves smoothly at 100 fps. The layer's clock runs at the speed's rate; b5_inc divides
 * it back out, so the catalogue's speed labels (cells, steps or seconds) hold; speed 0 stands still (the layer's rule).
 * Arena: a 256-byte header (64 words), then the effect's planes; objects live in the arena too (no particle slab).
 * B5-local candidates for B0: the model clock on ticks (b5_inc / b5_next / b5_frac) and the geometric control map b5_log. */
#include "fxv_gen.h"

#define B5_HDR   256u            /* header bytes: h[0..63] */
#define B5_TICK  10u             /* effect ms a simulation tick (the manifest's tick_ms) */
#define H_ACC    0u              /* Q16 progress towards the next model step */
#define H_INC    1u              /* Q16 progress a tick (this frame's speed) */
#define H_GEN    2u              /* model steps done (wraps) */
#define H_BANK   3u              /* the current plane of a ping-pong pair (0 / 1) */

static uint32_t *b5_h(const fxv_ctx_t *x) { return (uint32_t *)x->arena; }
static uint16_t *b5_w(const fxv_ctx_t *x, uint32_t plane) { return (uint16_t *)(x->arena + B5_HDR) + plane * x->n; }
static uint32_t b5_min(uint32_t a, uint32_t b) { return a < b ? a : b; }

/* lo at s = 0 .. hi at s = 255, geometric between (octaves in Q8, the last one linear); 1 <= lo <= hi < 2^24 */
static uint32_t b5_log(uint32_t lo, uint32_t hi, uint32_t s)
{
    uint32_t oct = 0u, base = lo, q, v;
    while (base * 2u <= hi) { base *= 2u; oct += 256u; }
    oct += ((hi - base) * 256u) / base;
    q = (s * oct) / 255u;
    base = lo << (q >> 8);
    v = base + ((base * (q & 255u)) >> 8);
    return v > hi ? hi : v;
}

/* Q16 model steps a tick for mhz / 1000 steps a real second: the layer's clock runs at rate / 256 of real time
 * (fxv_clock), 100 ticks a second at 1x; at most 4 steps a tick */
static uint32_t b5_inc(const fxv_ctx_t *x, uint32_t mhz)
{
    const uint32_t rate = x->speed ? 8u + ((x->speed * 15u) >> 3) : 0u;
    uint32_t v;
    if (!rate) return 0u;
    v = (mhz * 671u) / (rate * 4u);
    return v > 262144u ? 262144u : v;
}

/* 1: run a model step now (h[H_GEN] counts it); 0: this frame's ticks are spent */
static uint32_t b5_next(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x);
    while (h[H_ACC] < 65536u) {
        if (!fxv_tick_next(x)) return 0u;
        h[H_ACC] += h[H_INC];
    }
    h[H_ACC] -= 65536u;
    h[H_GEN]++;
    return 1u;
}

/* the share of the next step done, 0..65535: whole ticks plus the layer's ms into the next one */
static uint32_t b5_frac(const fxv_ctx_t *x)
{
    const uint32_t *h = b5_h(x), f = h[H_ACC] + (h[H_INC] * x->tick_acc) / B5_TICK;
    return f > 65535u ? 65535u : f;
}

/* ph / span in Q16 for ph < span x 65536, inv = 65536 / span (no division per pixel) */
static uint32_t b5_part(uint32_t ph, uint32_t inv)
{
    const uint32_t t = ((ph >> 4) * inv) >> 12;
    return t > 65535u ? 65535u : t;
}

/* palette mode (pal_own 0): the position opposite u (0..65535): half a turn on a cyclic palette, the mirror image on an
 * open one (no jump from one end to the other) */
static uint32_t b5_opp(const fxv_ctx_t *x, uint32_t u)
{
    return x->pal_wrap ? (u + 32768u) & 0xFFFFu : 65535u - u;
}

static uint32_t b5_digest(const fxv_ctx_t *x)
{
    uint32_t i, v = 0u;
    for (i = 0u; i < x->need; i++) v = fxv_hash32(v ^ x->arena[i]);
    return v;
}

static void b5_init(fxv_ctx_t *x) { (void)x; }

/* ------------------------------------------------------------------------------- 88 Excitable Wire
 * Greenberg-Hastings on the strip: a resting cell fires when a neighbour is excited, stays excited E steps (the front
 * width), then refractory R steps (the recovery time at the propagation speed) and rests again. A pacemaker at a random
 * cell fires a train of 2-4 pulses (each as soon as its cell rests again); the pulses run both ways, and two fronts that
 * meet annihilate (each runs into the other's refractory wake). Ends absorb. Colours: c1 the front fading into c3 (the
 * recovering wake), c3 easing back to c2 (the resting wire). A resting cell next to a fresh front (or the pacemaker
 * about to fire) rises with the step's fraction, so fronts glide instead of stepping. Two u16 planes: 4 bytes a pixel.
 * h[8] steps to the next pacemaker, h[9] its cell + 1 (0: none), h[10] pulses left, h[11] arming steps; u[1] E, u[2] R,
 * u[3] 65536 / E, u[4] 65536 / R, u[5] the spark interval in steps. */
static uint32_t ew_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + 4u * n; }

static void ew_advance(fxv_ctx_t *x)
{
    const uint32_t mhz = b5_log(5000u, 80000u, x->speed);             /* 5 .. 80 cells a second */
    const uint32_t e = 1u + (x->intensity * 11u) / 255u;              /* front width 1 .. 12 cells */
    uint32_t r = (b5_log(500u, 8000u, x->custom1) / 10u) * (mhz / 100u) / 1000u;   /* 0.5 .. 8 s of steps */
    if (r < 1u) r = 1u;
    b5_h(x)[H_INC] = b5_inc(x, mhz);
    x->u[1] = e;
    x->u[2] = r;
    x->u[3] = 65536u / e;
    x->u[4] = 65536u / r;
    x->u[5] = (b5_log(2000u, 20000u, 255u - x->custom2) / 10u) * (mhz / 100u) / 1000u;   /* 20 .. 2 s of steps */
    x->job[0] = 0u;
}

static void ew_start(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x);
    if (h[8]) h[8]--;
    if (h[11]) h[11]--;
    if (!h[8]) {                                         /* a new pacemaker (it replaces one still firing) */
        const uint32_t r = fxv_rand(x->seed, h[H_GEN], 88u);
        h[9] = x->n ? 1u + (r >> 8) % x->n : 0u;
        h[10] = 2u + (r & 255u) % 3u;
        h[11] = 1u;                                      /* fires from the next step: its cell rises first */
        h[8] = 1u + ((x->u[5] * (154u + (((r >> 24) * 205u) >> 8))) >> 8);   /* 0.6 .. 1.4 x the interval */
    }
}

static uint32_t ew_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x);
    (void)quota;
    if (!x->job[0]) {
        if (!b5_next(x)) { x->u[0] = b5_frac(x); return 0u; }
        ew_start(x);
        x->job[0] = 1u;
        x->job[1] = 0u;
    }
    {
        const uint32_t n = x->n, e = x->u[1], top = e + x->u[2], pace = h[11] ? 0u : h[9];
        const uint16_t *o = b5_w(x, h[H_BANK]);
        uint16_t *d = b5_w(x, 1u - h[H_BANK]);
        uint32_t i = x->job[1];
        const uint32_t end = b5_min(n, i + 128u);
        for (; i < end; i++) {
            const uint32_t c = o[i];
            uint32_t v;
            if (c == 0u) {
                const uint32_t l = i ? o[i - 1u] : 0u, r = i + 1u < n ? o[i + 1u] : 0u;
                v = l - 1u < e || r - 1u < e ? 1u : 0u;
                if (!v && i + 1u == pace) {
                    v = 1u;
                    if (!--h[10]) h[9] = 0u;
                }
            } else {
                v = c >= top ? 0u : c + 1u;
            }
            d[i] = (uint16_t)v;
        }
        x->job[1] = end;
        if (end < n) return 1u;
        h[H_BANK] ^= 1u;
        x->job[0] = 0u;
        return 1u;
    }
}

static void ew_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b5_h(x);
    const uint32_t n = x->n, e = x->u[1], f = x->u[0], rise = fxv_smooth16(f), pace = h[9];
    const uint16_t *s = b5_w(x, h[H_BANK]);
    uint32_t j;
    if (x->pal_own) {
        const fxv_px_t act = fxv_rgb16(x->c[0]), rest = fxv_rgb16(x->c[1]), rec = fxv_rgb16(x->c[2]);
        for (j = 0u; j < count; j++) {
            const uint32_t i = from + j, c = s[i];
            fxv_px_t col = rest;
            if (c == 0u) {
                const uint32_t l = i ? s[i - 1u] : 0u, r = i + 1u < n ? s[i + 1u] : 0u;
                if (l == 1u || r == 1u || i + 1u == pace) col = fxv_blend16(rest, act, rise);
            } else if (c <= e) {
                const uint32_t t = b5_part(((c - 1u) << 16) + f, x->u[3]);
                col = fxv_blend16(act, rec, fxv_smooth16(t));
            } else if (c - e <= x->u[2]) {               /* (a cell past a shortened recovery rests next step) */
                const uint32_t t = b5_part(((c - e - 1u) << 16) + f, x->u[4]);
                col = fxv_blend16(rec, rest, fxv_smooth16(t));
            }
            o[j] = col;
        }
    } else {
        /* palette: a cell's age since it fired picks the colour: the front at the palette's end (most palettes end
         * bright), the recovering wake running back through the rest of it to the resting wire (the palette's start,
         * dark); front, wake and wire keep the default slots' brightness (203, 89 and 14 / 255) */
        const fxv_px_t act = fxv_scale_px(fxv_pal(x, 65535u), 52171u), rest = fxv_scale_px(fxv_pal(x, 0u), 3598u);
        for (j = 0u; j < count; j++) {
            const uint32_t i = from + j, c = s[i];
            fxv_px_t col = rest;
            if (c == 0u) {
                const uint32_t l = i ? s[i - 1u] : 0u, r = i + 1u < n ? s[i + 1u] : 0u;
                if (l == 1u || r == 1u || i + 1u == pace) col = fxv_blend16(rest, act, rise);
            } else if (c <= e) {
                const uint32_t t = fxv_smooth16(b5_part(((c - 1u) << 16) + f, x->u[3]));
                col = fxv_scale_px(fxv_pal(x, 65535u - ((t * 21845u) >> 16)), fxv_lerp16(52171u, 22873u, t));
            } else if (c - e <= x->u[2]) {
                const uint32_t t = fxv_smooth16(b5_part(((c - e - 1u) << 16) + f, x->u[4]));
                col = fxv_scale_px(fxv_pal(x, 43690u - ((t * 43690u) >> 16)), fxv_lerp16(22873u, 3598u, t));
            }
            o[j] = col;
        }
    }
}

const fxv_desc_t fxv_d_excitable_wire = { ew_bytes, b5_init, ew_advance, ew_slice, ew_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 72 Cyclic Automaton
 * Griffeath's cyclic automaton on a ring of cells: N states (intensity: 3 .. 16 colours round the palette); a cell moves
 * to s + 1 when a neighbour holds s + 1, so each colour eats the one before it and the fronts between them travel. It
 * starts from colour noise (a random walk of states, every step +-1 or 0) that organises into travelling bands; in 1-D
 * a pure automaton then freezes or settles, so a cell also advances on its own now and then (custom1, the threshold:
 * 1 in 150 .. 1 in 20,000 cells a generation), each such seed growing into a new domain whose fronts chase the others.
 * 1 .. 40 generations a second. A cell's colour moves along the palette from its old state to the new one as the
 * generation progresses (custom2: the share of the generation the cross-fade takes, 0: a hard step). Two u8 planes
 * (2 bytes a pixel); h[8] fill done, h[9] the fill's running state, h[10] seeds (Q16) accumulator; u[1] N, u[3] seeds
 * a generation (Q16), u[4] 65536 / N, u[5] the cross-fade weight. */
static uint32_t ca_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + 2u * n; }
static uint8_t *ca_p(const fxv_ctx_t *x, uint32_t plane) { return x->arena + B5_HDR + plane * x->n; }

static void ca_advance(fxv_ctx_t *x)
{
    const uint32_t ns = 3u + (x->intensity * 13u) / 255u;
    b5_h(x)[H_INC] = b5_inc(x, b5_log(1000u, 40000u, x->speed));
    x->u[1] = ns;
    x->u[3] = (x->n * 65536u) / b5_log(150u, 20000u, x->custom1);   /* seeds a generation, Q16 */
    x->u[4] = 65536u / ns;
    x->job[0] = 0u;
}

static uint32_t ca_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x);
    const uint32_t n = x->n, ns = x->u[1];
    (void)quota;
    if (!h[8]) {                                         /* the start: a random walk of states in both planes */
        uint8_t *a = ca_p(x, 0u), *b = ca_p(x, 1u);
        uint32_t i = x->job[1], s = h[9];
        const uint32_t end = b5_min(n, i + 64u);
        for (; i < end; i++) {
            s += ns - 1u + (((fxv_hash2(x->seed, i) >> 16) * 3u) >> 16);
            if (s >= ns) s -= ns;
            if (s >= ns) s -= ns;
            a[i] = b[i] = (uint8_t)s;
        }
        h[9] = s;
        x->job[1] = end;
        if (end < n) return 1u;
        h[8] = 1u;
        x->job[1] = 0u;
    }
    if (!x->job[0]) {
        if (!b5_next(x)) {
            const uint32_t span = (x->custom2 + 1u) << 8, f = b5_frac(x);
            x->u[0] = f;
            x->u[5] = f >= span ? 65535u : fxv_smooth16((f << 8) / (x->custom2 + 1u));
            return 0u;
        }
        x->job[0] = 1u;
        x->job[1] = 0u;
        x->job[2] = 0u;
    }
    {
        const uint8_t *o = ca_p(x, h[H_BANK]);
        uint8_t *d = ca_p(x, 1u - h[H_BANK]);
        uint32_t i = x->job[1];
        const uint32_t end = b5_min(n, i + 64u);
        for (; i < end; i++) {
            const uint32_t c = o[i] < ns ? o[i] : 0u, nx = c + 1u >= ns ? 0u : c + 1u;
            d[i] = (uint8_t)(o[i ? i - 1u : n - 1u] == nx || o[i + 1u < n ? i + 1u : 0u] == nx ? nx : c);
        }
        x->job[1] = end;
        if (end < n || !x->job[2]) { x->job[2] = end >= n; return 1u; }
        /* seeds (a slice of its own): cells that did not move this generation advance on their own (at most 32) */
        x->job[2] = 0u;
        h[10] += x->u[3];
        if (n) {
            uint32_t k;
            const uint32_t seeds = b5_min(h[10] >> 16, 32u);
            for (k = 0u; k < seeds; k++) {
                const uint32_t c = ((fxv_rand(x->seed, h[H_GEN], k) >> 16) * n) >> 16;
                if (d[c] == o[c] && o[c] < ns) d[c] = (uint8_t)(o[c] + 1u >= ns ? 0u : o[c] + 1u);
            }
        }
        h[10] &= 0xFFFFu;
        h[H_BANK] ^= 1u;
        x->job[0] = 0u;
        return 1u;
    }
}

static void ca_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b5_h(x), ns = x->u[1], step = x->u[4], w = x->u[5];   /* (states past a lowered N read 0) */
    const uint8_t *cur = ca_p(x, h[H_BANK]), *old = ca_p(x, 1u - h[H_BANK]);
    uint32_t j;
    for (j = 0u; j < count; j++) {
        const uint32_t i = from + j, c = cur[i] < ns ? cur[i] : 0u, b = old[i] < ns ? old[i] : 0u;
        if (c == b) o[j] = fxv_pal(x, c * step);
        else if (c == b + 1u || (c == 0u && b + 1u == ns)) o[j] = fxv_pal(x, (b * step + ((step * w) >> 16)) & 0xFFFFu);
        else o[j] = fxv_blend16(fxv_pal(x, b * step), fxv_pal(x, c * step), w);
    }
}

const fxv_desc_t fxv_d_cyclic_automaton = { ca_bytes, b5_init, ca_advance, ca_slice, ca_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 73 Rule Garden
 * An elementary automaton (custom1: Wolfram's rule) on a ring of cells, 0.5 .. 10 generations a second. A live cell
 * takes the ink of its birth (a palette colour drifting with the generations, a gentle gradient along the strip) and
 * rises over its first generation; a dead cell keeps its colour and fades (custom2: 0.5 .. 24 generations). Seeding
 * (intensity) flips random cells, up to ~1 cell a second per 8 pixels, so even a dying rule stays alive. Rules that turn
 * an empty neighbourhood on (odd rules) show every other generation complemented (Wolfram's convention), so the strip
 * never blinks as a whole. Planes: two u8 states (bit 0 the cell, bit 1 shown), an age and an ink byte: 4 bytes a pixel.
 * h[8] first fill done, h[9] seeding (Q16 cells a generation) accumulator; u[1] the fade span (1/16 generation),
 * u[2] 2^20 / it, u[3] seeds a generation (Q16), u[4] the ink step a generation. */
static uint32_t rg_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + 4u * n; }

static void rg_advance(fxv_ctx_t *x)
{
    const uint32_t mhz = b5_log(500u, 10000u, x->speed), fade = 8u + (x->custom2 * 376u) / 255u;
    b5_h(x)[H_INC] = b5_inc(x, mhz);
    x->u[1] = fade;
    x->u[2] = 1048576u / fade;
    x->u[3] = (x->intensity * x->n * 1024u) / (mhz / 32u);
    x->u[4] = x->n ? 4194304u / x->n : 0u;               /* a quarter of the palette along the strip */
    x->job[0] = 0u;
}

/* cell i's states after its rule (or a seed's flip) gave s at generation gen: shown, age and ink */
static void rg_set(fxv_ctx_t *x, uint32_t i, uint32_t s, uint32_t was, uint32_t gen)
{
    uint8_t *age = ca_p(x, 2u), *ink = ca_p(x, 3u), *d = ca_p(x, 1u - b5_h(x)[H_BANK]);
    const uint32_t shown = s ^ (((x->custom1 & 0x81u) == 1u) & gen);   /* 000 -> 1, 111 -> 0: complement odd ones */
    if (shown != was) {
        age[i] = 0u;
        if (shown) ink[i] = (uint8_t)(gen * 5u + ((i * x->u[4]) >> 16));
    } else if (age[i] < 255u) {
        age[i]++;
    }
    d[i] = (uint8_t)(s | shown << 1);
}

static uint32_t rg_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x);
    const uint32_t n = x->n, rule = x->custom1;
    (void)quota;
    if (!h[8]) {                                         /* sparse seeds and the middle cell */
        uint8_t *a = ca_p(x, 0u), *age = ca_p(x, 2u), *ink = ca_p(x, 3u);
        uint32_t i = x->job[1];
        const uint32_t end = b5_min(n, i + 48u);    /* (64 a slice: 4,901) */
        for (; i < end; i++) {
            const uint32_t s = (fxv_hash2(x->seed, i) & 15u) == 0u || i == n / 2u ? 3u : 0u;
            a[i] = (uint8_t)s;
            age[i] = s ? 1u : 255u;
            ink[i] = (uint8_t)((i * x->u[4]) >> 16);
        }
        x->job[1] = end;
        if (end < n) return 1u;
        h[8] = 1u;
        x->job[1] = 0u;
    }
    if (!x->job[0]) {
        if (!b5_next(x)) { x->u[0] = b5_frac(x); return 0u; }
        x->job[0] = 1u;
        x->job[1] = 0u;
        x->job[2] = 0u;
        return 1u;                                       /* (the generation's start a slice of its own: with its first
                                                            cells one slice reached 5,024 cycles) */
    }
    {
        const uint8_t *o = ca_p(x, h[H_BANK]);
        const uint32_t gen = h[H_GEN];
        uint32_t i = x->job[1];
        const uint32_t end = b5_min(n, i + 40u);        /* (48 cells a slice: 5,024 cycles) */
        for (; i < end; i++) {
            const uint32_t l = o[i ? i - 1u : n - 1u] & 1u, c = o[i] & 1u, r = o[i + 1u < n ? i + 1u : 0u] & 1u;
            rg_set(x, i, (rule >> (l << 2 | c << 1 | r)) & 1u, (o[i] >> 1) & 1u, gen);
        }
        x->job[1] = end;
        if (end < n || !x->job[2]) { x->job[2] = end >= n; return 1u; }
        /* seeding (a slice of its own): flip random cells of the new generation (at most 32) */
        x->job[2] = 0u;
        h[9] += x->u[3];
        if (n) {
            uint8_t *d = ca_p(x, 1u - h[H_BANK]);
            uint32_t k;
            const uint32_t seeds = b5_min(h[9] >> 16, 32u);
            for (k = 0u; k < seeds; k++) {
                const uint32_t c = ((fxv_rand(x->seed, gen, k) >> 16) * n) >> 16;
                rg_set(x, c, (d[c] & 1u) ^ 1u, (d[c] >> 1) & 1u, gen);
            }
        }
        h[9] &= 0xFFFFu;
        h[H_BANK] ^= 1u;
        x->job[0] = 0u;
        return 1u;
    }
}

static void rg_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b5_h(x), f = x->u[0], rise = fxv_smooth16(f), fade = x->u[1], inv = x->u[2];
    const uint8_t *cur = ca_p(x, h[H_BANK]), *age = ca_p(x, 2u), *ink = ca_p(x, 3u);
    uint32_t j;
    for (j = 0u; j < count; j++) {
        const uint32_t i = from + j, a = age[i];
        const fxv_px_t c = fxv_pal(x, (uint32_t)ink[i] << 8);
        uint32_t b = 0u;
        if (cur[i] & 2u) {
            b = a ? 65535u : rise;
        } else if (a * 16u < fade + 16u) {
            const uint32_t t = b5_part(((a << 16) + f), inv);
            b = t >= 65535u ? 0u : fxv_scale16(65535u - fxv_smooth16(t), 40000u);
        }
        o[j] = fxv_scale_px(c, b);
    }
}

const fxv_desc_t fxv_d_rule_garden = { rg_bytes, b5_init, rg_advance, rg_slice, rg_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 105 Zipper Exchange
 * Two seams of teeth along the strip (tooth size: intensity, 1 .. 8 pixels), A and B alternating. Open, each tooth is a
 * dim narrow tooth on the backing; zipped, the teeth widen until they interlock with no gaps. A cycle (speed: 20 .. 2 s,
 * custom2 adds a zipped hold of up to half a cycle): sliders run out from 1 .. 4 origins (custom1) zipping the teeth
 * together, the zipped seams hold, an exchange front runs out from the origins swapping every tooth's colour, then the
 * sliders run back and unzip. The next cycle starts from the exchanged arrangement with the origins moved, so the
 * zipper never repeats. Analytic on the virtual strip (fractional fronts and tooth edges); the cycle runs on ticks: 256
 * model steps a cycle. u[1..4] phase boundaries (Q16 of the cycle: zip end, hold end, exchange end, unzip end), u[5]
 * the cycle's phase, u[6] the origin spacing (Q8 px), u[7] the origins' offset (Q8 px). */
static uint32_t zx_bytes(uint32_t n, uint32_t tier) { (void)n; (void)tier; return B5_HDR; }

static void zx_advance(fxv_ctx_t *x)
{
    const uint32_t hold = x->custom2 >> 1, d = 256u + hold, per = b5_log(2000u, 20000u, 255u - x->speed);
    b5_h(x)[H_INC] = b5_inc(x, ((65536000u / d) * 1000u) / per);
    x->u[1] = (90u << 16) / d;
    x->u[2] = x->u[1] + (hold << 16) / d;
    x->u[3] = x->u[2] + (51u << 16) / d;
    x->u[4] = x->u[3] + (90u << 16) / d;
    x->u[6] = ((uint32_t)x->vlen << 8) / (1u + (x->custom1 * 3u) / 255u);
}

static uint32_t zx_slice(fxv_ctx_t *x, uint32_t quota)
{
    const uint32_t *h = b5_h(x);
    (void)quota;
    if (b5_next(x)) return 1u;
    x->u[0] = b5_frac(x);
    x->u[5] = ((h[H_GEN] & 255u) << 8) + (x->u[0] >> 8);
    x->u[7] = ((fxv_rand(x->seed, h[H_GEN] >> 8, 105u) >> 8) % x->vlen) << 8;
    return 0u;
}

/* 0 .. 65535: how far a front at r (signed, Q8 px) has passed a point at distance d (Q8), soft over +-1.5 px */
static uint32_t zx_edge(int32_t r, uint32_t d)
{
    const int32_t v = r - (int32_t)d + 384;
    return v <= 0 ? 0u : v >= 768 ? 65535u : fxv_smooth16((uint32_t)v * 85u);
}

/* a front's position (Q8, signed) at progress f: from 1.5 px before the origin to 1.5 px past the half spacing */
static int32_t zx_front(uint32_t half, uint32_t f)
{
    return (int32_t)(((half + 768u) * (fxv_smooth16(f) >> 4)) >> 12) - 384;
}

static void zx_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t cyc = b5_h(x)[H_GEN] >> 8, ph = x->u[5], sp = x->u[6], half = sp >> 1;
    const uint32_t w = 1u + (x->intensity * 7u) / 255u;
    int32_t reach = -384, swap = -384;
    uint32_t amp = 0u, xamp = 0u, j;
    uint32_t p = x->vbase + from, slot = p / w, u = p - slot * w;
    uint32_t q = ((p << 8) + x->vlen * 256u - x->u[7]) % sp;
    if (ph < x->u[1]) {                                  /* zipping out from the origins */
        const uint32_t f = (ph << 16) / x->u[1];
        reach = zx_front(half, f);
        amp = (uint32_t)fxv_sin16(f >> 1);
    } else if (ph < x->u[3]) {                           /* zipped; the exchange front runs */
        reach = (int32_t)half + 1024;
        if (ph >= x->u[2]) {
            const uint32_t f = ((ph - x->u[2]) << 16) / (x->u[3] - x->u[2]);
            swap = zx_front(half, f);
            xamp = (uint32_t)fxv_sin16(f >> 1) >> 1;
        }
    } else if (ph < x->u[4]) {                           /* unzipping back to the origins */
        const uint32_t f = ((ph - x->u[3]) << 16) / (x->u[4] - x->u[3]);
        reach = zx_front(half, 65535u - f);
        swap = (int32_t)half + 1024;
        amp = (uint32_t)fxv_sin16(f >> 1);
    } else {
        swap = (int32_t)half + 1024;
    }
    if (x->pal_own) {
        const fxv_px_t sa = fxv_rgb16(x->c[0]), back = fxv_rgb16(x->c[1]), sb = fxv_rgb16(x->c[2]);
        const fxv_px_t mid = fxv_blend16(sa, sb, 32768u), glint = fxv_add_px(mid, fxv_scale_px(mid, 40000u));
        for (j = 0u; j < count; j++) {
            const uint32_t dq = q + 128u, d = dq < half ? dq : dq < sp ? sp - dq : dq - sp;   /* to the nearest origin, Q8 */
            const uint32_t close = zx_edge(reach, d), ex = zx_edge(swap, d);
            const uint32_t tw = ((w << 8) * (29491u + fxv_scale16(36044u, close))) >> 16;  /* 45 .. 100 % of the slot */
            const uint32_t a8 = ((w << 8) - tw) >> 1, b8 = a8 + tw, lo = u << 8, hi = lo + 256u;
            const uint32_t e0 = a8 > lo ? a8 : lo, e1 = b8 < hi ? b8 : hi, cov = e1 > e0 ? e1 - e0 : 0u;
            const uint32_t par = (slot ^ cyc) & 1u;
            fxv_px_t col = fxv_scale_px(fxv_blend16(par ? sb : sa, par ? sa : sb, ex), 36044u + fxv_scale16(29491u, close));
            col = fxv_blend16(back, col, cov >= 256u ? 65535u : cov << 8);
            if (amp) {
                const uint32_t dd = (int32_t)d > reach ? d - (uint32_t)reach : (uint32_t)reach - d;
                col = fxv_add_px(col, fxv_scale_px(glint, fxv_scale16(zx_edge(384, dd), amp)));
            }
            if (xamp) {
                const uint32_t dd = (int32_t)d > swap ? d - (uint32_t)swap : (uint32_t)swap - d;
                col = fxv_add_px(col, fxv_scale_px(glint, fxv_scale16(zx_edge(384, dd), xamp)));
            }
            o[j] = col;
            if (++u >= w) { u = 0u; slot++; }
            q += 256u;
            while (q >= sp) q -= sp;
        }
    } else {
        /* palette: a tooth's place along the strip picks seam A's colour (ua) and seam B takes the opposite palette
         * position (b5_opp), so the teeth run through the whole palette and the exchange swaps each pair; teeth at the
         * default seams' levels (200 and 229 / 255), the glints in the tooth's A colour at full light, the backing the
         * palette's start at the default backing's darkness (9 / 255). Only the colours a pixel shows are looked up. */
        const uint32_t pst = (65536u << 8) / (x->vlen ? x->vlen : 1u);
        const fxv_px_t back = fxv_scale_px(fxv_pal(x, 0u), 2313u);
        fxv_px_t pa = back, pb = back;
        uint32_t pslot = 0xFFFFFFFFu, ua = 0u, got = 0u;
        for (j = 0u; j < count; j++) {
            const uint32_t dq = q + 128u, d = dq < half ? dq : dq < sp ? sp - dq : dq - sp;
            const uint32_t close = zx_edge(reach, d), ex = zx_edge(swap, d);
            const uint32_t tw = ((w << 8) * (29491u + fxv_scale16(36044u, close))) >> 16;
            const uint32_t a8 = ((w << 8) - tw) >> 1, b8 = a8 + tw, lo = u << 8, hi = lo + 256u;
            const uint32_t e0 = a8 > lo ? a8 : lo, e1 = b8 < hi ? b8 : hi, cov = e1 > e0 ? e1 - e0 : 0u;
            const uint32_t par = (slot ^ cyc) & 1u, lvl = 36044u + fxv_scale16(29491u, close);
            const uint32_t need = (ex < 65535u ? (par ? 2u : 1u) : 0u) | (ex ? (par ? 1u : 2u) : 0u);   /* 1: A, 2: B */
            uint32_t ga = 0u, gx = 0u;
            fxv_px_t col;
            if (slot != pslot) {
                const uint32_t um = (slot * w * pst + ((w * pst) >> 1)) >> 8;
                ua = um > 65535u ? 65535u : um;
                pslot = slot;
                got = 0u;
            }
            if (amp) {
                const uint32_t dd = (int32_t)d > reach ? d - (uint32_t)reach : (uint32_t)reach - d;
                ga = fxv_scale16(zx_edge(384, dd), amp);
            }
            if (xamp) {
                const uint32_t dd = (int32_t)d > swap ? d - (uint32_t)swap : (uint32_t)swap - d;
                gx = fxv_scale16(zx_edge(384, dd), xamp);
            }
            if ((need & 1u || ga || gx) && !(got & 1u)) { pa = fxv_pal(x, ua); got |= 1u; }
            if (need & 2u && !(got & 2u)) { pb = fxv_pal(x, b5_opp(x, ua)); got |= 2u; }
            {
                const fxv_px_t ca = need & 1u ? fxv_scale_px(pa, (lvl * 51400u) >> 16) : back;
                const fxv_px_t cb = need & 2u ? fxv_scale_px(pb, (lvl * 58853u) >> 16) : back;
                col = ex == 0u ? (par ? cb : ca) : ex >= 65535u ? (par ? ca : cb) : fxv_blend16(par ? cb : ca, par ? ca : cb, ex);
            }
            col = fxv_blend16(back, col, cov >= 256u ? 65535u : cov << 8);
            if (ga) col = fxv_add_px(col, fxv_scale_px(pa, ga));
            if (gx) col = fxv_add_px(col, fxv_scale_px(pa, gx));
            o[j] = col;
            if (++u >= w) { u = 0u; slot++; }
            q += 256u;
            while (q >= sp) q -= sp;
        }
    }
}

const fxv_desc_t fxv_d_zipper_exchange = { zx_bytes, b5_init, zx_advance, zx_slice, zx_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 69 Estuary Locks
 * A row of 3 .. 12 pools (custom1) behind low gates (one bank pixel each, c2). Each pool's water (c1, a slow shimmer)
 * lies from its middle out to its level, so a fuller pool is wider and a little brighter; the dry floor is a dim c2.
 * A tide cycle (speed: 8 min .. 30 s, 1024 model steps): for its first half an inflow draws every pool towards its own
 * level (intensity: how far apart the levels are; a new random set each cycle), then it fades and the gates (custom2:
 * free .. slow) bring neighbours to a shared level. Where a pool stands above its neighbour, its gate glows (c3) and
 * ripples of light run from the gate into the lower pool, both in proportion to the difference. Levels: h[16..27]
 * (Q16), the cycle's targets h[28..39], h[9] the cycle they belong to, h[8] set up; u[1] pools a pixel (Q16),
 * u[2] 2^24 / the soft water edge u[5], u[3] pixels a half pool (Q8), u[4] pools. */
static uint32_t el_bytes(uint32_t n, uint32_t tier) { (void)n; (void)tier; return B5_HDR; }

static void el_targets(fxv_ctx_t *x, uint32_t cyc)
{
    uint32_t *h = b5_h(x), k;
    const uint32_t base = 18350u + ((fxv_rand(x->seed, cyc, 690u) >> 16) * 26214u >> 16);   /* 0.28 .. 0.68 */
    const uint32_t range = 6554u + x->intensity * 206u;                                     /* 0.1 .. 0.9 */
    for (k = 0u; k < 12u; k++) {
        const int32_t v = (int32_t)base + (int32_t)(((fxv_rand(x->seed, cyc, k) >> 16) * range) >> 16) - (int32_t)(range >> 1);
        h[28u + k] = (uint32_t)(v < 3277 ? 3277 : v > 62259 ? 62259 : v);
    }
    h[9] = cyc;
}

static void el_advance(fxv_ctx_t *x)
{
    const uint32_t pools = 3u + (x->custom1 * 9u) / 255u, n = x->n ? x->n : 1u;
    b5_h(x)[H_INC] = b5_inc(x, 1024000000u / b5_log(30000u, 480000u, 255u - x->speed));
    x->u[1] = (pools << 16) / n;
    x->u[5] = 3u * x->u[1] + 256u;
    x->u[2] = 16777216u / x->u[5];
    x->u[3] = (n << 8) / (2u * pools);
    x->u[4] = pools;
    if (!x->pal_own) x->u[6] = x->pal_wrap ? 65536u / pools : 65535u / (pools - 1u);   /* palette: pool to pool */
}

/* palette mode: each pool's colour once a frame into h[40..63] (r | g << 16, b), from its palette position: its place
 * in the row (the whole palette, first pool = its start, last = its end on an open palette), swept 5/8 of a pool either
 * way over each tide cycle (so every entry shows in every cycle, whatever the pool count) plus up to 1/8 of a pool for
 * its level (a fuller pool moves on along the palette; pools at a shared level read as one gradient) */
static void el_ptab(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x), k;
    const int32_t sp = (int32_t)x->u[6], sw = (fxv_sin16((h[H_GEN] & 1023u) << 6) * ((5 * sp) >> 3)) >> 15;
    for (k = 0u; k < x->u[4]; k++) {
        int32_t v = (int32_t)k * sp + sw + ((((int32_t)h[16u + k] - 32768) * sp) >> 18);
        fxv_px_t c;
        if (x->pal_wrap) v &= 0xFFFF;
        else v = v < 0 ? 0 : v > 65535 ? 65535 : v;
        c = fxv_pal(x, (uint32_t)v);
        h[40u + 2u * k] = (uint32_t)c.r | (uint32_t)c.g << 16;
        h[41u + 2u * k] = c.b;
    }
}

static fxv_px_t el_pcol(const uint32_t *h, uint32_t k, uint32_t s16)   /* pool k's colour at brightness s16 */
{
    fxv_px_t c;
    c.r = (uint16_t)fxv_scale16(h[40u + 2u * k] & 0xFFFFu, s16);
    c.g = (uint16_t)fxv_scale16(h[40u + 2u * k] >> 16, s16);
    c.b = (uint16_t)fxv_scale16(h[41u + 2u * k], s16);
    c.q = 0;
    c.k = 0;
    return c;
}

static uint32_t el_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x), k;
    int32_t flow[12];
    (void)quota;
    if (!h[8]) {                                         /* the pools start at the first cycle's levels */
        el_targets(x, 0u);
        for (k = 0u; k < 12u; k++) h[16u + k] = h[28u + k];
        h[8] = 1u;
    }
    if (!b5_next(x)) {
        x->u[0] = b5_frac(x);
        if (!x->pal_own) el_ptab(x);
        return 0u;
    }
    {
        const uint32_t g = h[H_GEN], ph = g & 1023u, pools = x->u[4];
        const int32_t gate = (int32_t)b5_log(32u, 1600u, 255u - x->custom2);
        const int32_t in = ph < 512u ? 384 : ph < 768u ? (int32_t)((768u - ph) * 3u >> 1) : 0;
        if (g >> 10 != h[9]) el_targets(x, g >> 10);
        for (k = 0u; k + 1u < pools; k++) flow[k] = (gate * ((int32_t)h[16u + k] - (int32_t)h[17u + k])) >> 16;
        for (k = 0u; k < pools; k++) {
            int32_t v = (int32_t)h[16u + k];
            v += (in * ((int32_t)h[28u + k] - v)) >> 16;
            if (k + 1u < pools) v -= flow[k];
            if (k) v += flow[k - 1u];
            h[16u + k] = (uint32_t)(v < 0 ? 0 : v > 65535 ? 65535 : v);
        }
    }
    return 1u;
}

static void el_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b5_h(x), per = x->u[1], pools = x->u[4], px2 = 2u * per;
    const uint32_t tw = x->et * 105u, ts = x->et * 7u;
    uint32_t j;
    if (x->pal_own) {
        const fxv_px_t water = fxv_rgb16(x->c[0]), bank = fxv_rgb16(x->c[1]), spill = fxv_rgb16(x->c[2]);
        const fxv_px_t floor_ = fxv_scale_px(bank, 22000u);
        for (j = 0u; j < count; j++) {
            const uint32_t i = from + j, pc = ((2u * i + 1u) * per) >> 1, k = b5_min(pc >> 16, pools - 1u);
            const uint32_t pos = pc & 0xFFFFu, d = pos < 32768u ? (32767u - pos) * 2u : (pos - 32768u) * 2u;
            const uint32_t lv = h[16u + k];
            fxv_px_t col;
            if (per <= 16384u && d + px2 > 65535u) {     /* the bank pixel; it glows while the pools differ */
                const uint32_t other = pos < 32768u ? (k ? h[15u + k] : lv) : (k + 1u < pools ? h[17u + k] : lv);
                const uint32_t diff = lv > other ? lv - other : other - lv;
                col = fxv_blend16(bank, spill, b5_min(65535u, diff * 6u) * 13u >> 4);
            } else {
                const int32_t e = (int32_t)lv - (int32_t)d;
                const uint32_t wet = e <= 0 ? 0u : (uint32_t)e >= x->u[5] ? 65535u : b5_min(65535u, ((uint32_t)e * x->u[2]) >> 8);
                const uint32_t sh = (uint32_t)(fxv_sin16(i * 2400u + ts) + 32767) >> 3;
                col = fxv_blend16(floor_, fxv_scale_px(water, 32000u + (lv >> 2) + sh), fxv_smooth16(wet));
                {   /* light spilling in from a fuller neighbour */
                    const uint32_t other = pos < 32768u ? (k ? h[15u + k] : 0u) : (k + 1u < pools ? h[17u + k] : 0u);
                    if (other > lv) {
                        const uint32_t gp = ((65535u - d) * x->u[3]) >> 16;            /* px from the gate, Q8 */
                        if (gp < 1536u) {
                            const int32_t r = fxv_sin16(gp * 51u - tw);
                            const uint32_t rip = r > 0 ? ((uint32_t)r * (uint32_t)r) >> 14 : 0u;
                            const uint32_t s = b5_min(65535u, (other - lv) * 6u);
                            col = fxv_add_px(col, fxv_scale_px(spill, fxv_scale16(fxv_scale16(rip, s), (1536u - gp) * 14u)));
                        }
                    }
                }
            }
            o[j] = col;
        }
    } else {
        /* palette: each pool's place in the row and its level pick its colour (el_ptab): the water, its dry floor and
         * its gate at the default slots' levels (110, 27 / 255); the spill glow and the ripples carry the fuller
         * neighbour's colour (170 / 255) */
        for (j = 0u; j < count; j++) {
            const uint32_t i = from + j, pc = ((2u * i + 1u) * per) >> 1, k = b5_min(pc >> 16, pools - 1u);
            const uint32_t pos = pc & 0xFFFFu, d = pos < 32768u ? (32767u - pos) * 2u : (pos - 32768u) * 2u;
            const uint32_t lv = h[16u + k], ko = pos < 32768u ? (k ? k - 1u : k) : (k + 1u < pools ? k + 1u : k);
            fxv_px_t col;
            if (per <= 16384u && d + px2 > 65535u) {     /* the gate */
                const uint32_t other = h[16u + ko], diff = lv > other ? lv - other : other - lv;
                col = fxv_blend16(el_pcol(h, k, 6939u), el_pcol(h, ko, 43690u), b5_min(65535u, diff * 6u) * 13u >> 4);
            } else {
                const int32_t e = (int32_t)lv - (int32_t)d;
                const uint32_t wet = e <= 0 ? 0u : (uint32_t)e >= x->u[5] ? 65535u : b5_min(65535u, ((uint32_t)e * x->u[2]) >> 8);
                const uint32_t sh = (uint32_t)(fxv_sin16(i * 2400u + ts) + 32767) >> 3;
                const uint32_t other = ko != k ? h[16u + ko] : 0u;
                col = fxv_blend16(el_pcol(h, k, 2329u), el_pcol(h, k, ((32000u + (lv >> 2) + sh) * 28270u) >> 16), fxv_smooth16(wet));
                if (other > lv) {                        /* light spilling in from a fuller neighbour */
                    const uint32_t gp = ((65535u - d) * x->u[3]) >> 16;
                    if (gp < 1536u) {
                        const int32_t r = fxv_sin16(gp * 51u - tw);
                        const uint32_t rip = r > 0 ? ((uint32_t)r * (uint32_t)r) >> 14 : 0u;
                        const uint32_t s = b5_min(65535u, (other - lv) * 6u);
                        col = fxv_add_px(col, el_pcol(h, ko, fxv_scale16(fxv_scale16(fxv_scale16(rip, s), (1536u - gp) * 14u), 43690u)));
                    }
                }
            }
            o[j] = col;
        }
    }
}

const fxv_desc_t fxv_d_estuary_locks = { el_bytes, b5_init, el_advance, el_slice, el_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 91 Chromatic Sort
 * K chips (intensity: 4 .. 128, at least 4 pixels each) spread evenly over the palette, separated by thin gaps (c2).
 * An odd-even transposition sort: each round compares every other neighbouring pair and swaps the ones out of order,
 * the two chips sliding past each other over the swap duration (speed: 2 .. 0.15 s), the active pair tinted c3; a round
 * with nothing to swap takes no time. The key is the chip's palette order (hue), its luminance, or a blend (custom1).
 * Sorted, the spectrum holds (custom2: 2 .. 30 s), then the chips scramble gently, by the same sort towards a random
 * order (a new one each time), and sort again. 64 model steps a round. Arena after the header (Km = the chips n
 * allows, stride S = Km rounded up to 4): perm[S] (slot -> chip), swap[S] (this round: slot and slot + 1 swap),
 * key[S] u16, colour[S] x 3 u16, the scramble's random keys rkey[S] u16. h[9] phase (0 sort, 1 hold, 2 scramble),
 * h[10] parity, h[11] step in the round, h[12] hold steps left, h[13] scramble number, h[14] K of perm, h[15] a round
 * to evaluate (one a slice; it takes no time), h[17] empty rounds in a row, h[18] rkey filled up to, h[19] the
 * shuffle's position; u[1] chips a pixel
 * (Q16), u[2] K, u[3] S, u[4] hold steps. */
static uint32_t cs_km(uint32_t n) { const uint32_t k = n >> 2; return k < 1u ? 1u : k > 128u ? 128u : k; }
static uint32_t cs_stride(uint32_t n) { return (cs_km(n) + 3u) & ~3u; }
static uint32_t cs_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + 12u * cs_stride(n); }
static uint8_t *cs_perm(const fxv_ctx_t *x) { return x->arena + B5_HDR; }
static uint8_t *cs_swap(const fxv_ctx_t *x) { return x->arena + B5_HDR + x->u[3]; }
static uint16_t *cs_key(const fxv_ctx_t *x) { return (uint16_t *)(x->arena + B5_HDR + 2u * x->u[3]); }
static uint16_t *cs_col(const fxv_ctx_t *x) { return (uint16_t *)(x->arena + B5_HDR + 4u * x->u[3]); }
static uint16_t *cs_rkey(const fxv_ctx_t *x) { return (uint16_t *)(x->arena + B5_HDR + 10u * x->u[3]); }

static void cs_advance(fxv_ctx_t *x)
{
    const uint32_t n = x->n ? x->n : 1u, k = b5_min(cs_km(n), 4u + (x->intensity * 124u) / 255u);
    const uint32_t dur = b5_log(150u, 2000u, 255u - x->speed);
    b5_h(x)[H_INC] = b5_inc(x, 64000000u / dur);
    x->u[1] = (k << 16) / n;
    x->u[2] = k;
    x->u[3] = cs_stride(n);
    x->u[4] = (b5_log(2000u, 30000u, x->custom2) * 64u) / dur;
    x->job[0] = 0u;
    x->job[1] = 0u;
}

/* the sort key of chip j: its key (sorting), or the scramble's random rank */
static uint32_t cs_k(const fxv_ctx_t *x, uint32_t j)
{
    return b5_h(x)[9] == 2u ? cs_rkey(x)[j] : cs_key(x)[j];
}

/* a new round: mark the pairs to swap; 0: nothing to swap */
static uint32_t cs_round(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x), s, any = 0u;
    const uint32_t k = x->u[2];
    const uint8_t *perm = cs_perm(x);
    uint8_t *sw = cs_swap(x);
    for (s = 0u; s < k; s++) sw[s] = 0u;
    for (s = h[10]; s + 1u < k; s += 2u)
        if (cs_k(x, perm[s]) > cs_k(x, perm[s + 1u])) { sw[s] = 1u; any = 1u; }
    h[11] = 0u;
    return any;
}

/* evaluate one round: swaps start it; two empty rounds in a row end the phase (sorted: hold; scrambled: sort) */
static void cs_eval(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x);
    if (cs_round(x)) { h[15] = 0u; h[17] = 0u; return; }
    h[10] ^= 1u;
    if (++h[17] < 2u) return;
    h[17] = 0u;
    if (h[9] == 0u) { h[9] = 1u; h[12] = x->u[4]; h[15] = 0u; }
    else h[9] = 0u;
}

static uint32_t cs_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x), j;
    const uint32_t k = x->u[2];
    (void)quota;
    if (!x->job[0]) {                                    /* this frame's chip colours and keys, 16 chips a slice */
        uint16_t *key = cs_key(x), *col = cs_col(x);
        const uint32_t end = b5_min(k, x->job[1] + 16u), step = 65536u / k;
        for (j = x->job[1]; j < end; j++) {
            const fxv_px_t c = fxv_pal(x, (j * step + (step >> 1)) & 0xFFFFu);
            const uint32_t lum = (c.r * 54u + c.g * 183u + c.b * 19u) >> 8;
            col[3u * j] = c.r;
            col[3u * j + 1u] = c.g;
            col[3u * j + 2u] = c.b;
            key[j] = (uint16_t)(((255u - x->custom1) * j * step + x->custom1 * lum) >> 8);
        }
        x->job[1] = end;
        if (end < k) return 1u;
        x->job[0] = 1u;
        if (h[14] != k) { h[14] = 0u; h[19] = k; }      /* (a new K: shuffle the chips, then sort from there) */
        return 1u;
    }
    if (!h[14]) {                                        /* the shuffle (Fisher-Yates from the top), 32 chips a slice */
        uint8_t *perm = cs_perm(x);
        if (h[19] == k) for (j = 0u; j < k; j++) perm[j] = (uint8_t)j;
        for (j = h[19]; j > 1u && j + 32u > h[19]; j--) {
            const uint32_t r = ((fxv_hash2(x->seed, j) >> 16) * j) >> 16, t = perm[j - 1u];
            perm[j - 1u] = perm[r];
            perm[r] = (uint8_t)t;
        }
        h[19] = j;
        if (j > 1u) return 1u;
        h[14] = k;
        h[9] = 0u;
        h[10] = 0u;
        h[15] = 1u;
        h[17] = 0u;
        return 1u;
    }
    if (h[9] == 2u && h[18] < k) {                       /* the scramble's random keys, 32 a slice */
        uint16_t *rk = cs_rkey(x);
        const uint32_t end = b5_min(k, h[18] + 32u);
        for (j = h[18]; j < end; j++) rk[j] = (uint16_t)(fxv_hash2(x->seed ^ h[13], j) >> 16);
        h[18] = end;
        return 1u;
    }
    if (h[15]) { cs_eval(x); return 1u; }
    if (!b5_next(x)) { x->u[0] = b5_frac(x); return 0u; }
    if (h[9] == 1u) {                                    /* the sorted hold */
        if (h[12]) h[12]--;
        else { h[9] = 2u; h[13]++; h[10] = 0u; h[15] = 1u; h[17] = 0u; h[18] = 0u; }
        return 1u;
    }
    if (++h[11] >= 64u) {                                /* the round's swaps are done */
        uint8_t *perm = cs_perm(x), *sw = cs_swap(x);
        uint32_t s;
        for (s = 0u; s + 1u < k; s++)
            if (sw[s]) { const uint8_t t = perm[s]; perm[s] = perm[s + 1u]; perm[s + 1u] = t; }
        h[10] ^= 1u;
        h[15] = 1u;
    }
    return 1u;
}

static fxv_px_t cs_chip(const fxv_ctx_t *x, uint32_t chip)
{
    const uint16_t *c = cs_col(x) + 3u * chip;
    fxv_px_t p;
    p.r = c[0];
    p.g = c[1];
    p.b = c[2];
    p.q = 0u;
    p.k = 0u;
    return p;
}

/* coverage 0..65535 of a pixel centred at v (Q16 slot units, pixel width w <= 32768) by the interval [a, b) */
static uint32_t cs_cover(int32_t v, int32_t a, int32_t b, uint32_t w)
{
    const int32_t hw = (int32_t)(w >> 1), lo = v - hw > a ? v - hw : a, hi = v + hw < b ? v + hw : b;
    return hi <= lo ? 0u : b5_min(65535u, ((uint32_t)(hi - lo) << 16) / w);
}

static void cs_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b5_h(x), k = x->u[2], per = x->u[1], moving = h[9] != 1u;
    const uint32_t t = moving ? fxv_smooth16((((h[11] << 16) + x->u[0]) >> 6) & 0xFFFFu) : 0u;
    const uint32_t lift = moving ? (uint32_t)fxv_sin16(t >> 1) >> 2 : 0u;
    const int32_t g = per >> 1 < 4096u ? (int32_t)(per >> 1) : 4096;   /* half a gap: half a pixel, <= 1/16 chip */
    const uint8_t *perm = cs_perm(x), *sw = cs_swap(x);
    const fxv_px_t gap = fxv_rgb16(x->c[1]), act = fxv_rgb16(x->c[2]);
    uint32_t j;
    for (j = 0u; j < count; j++) {
        const uint32_t pc = ((2u * (from + j) + 1u) * per) >> 1, s = b5_min(pc >> 16, k - 1u);
        fxv_px_t col;
        if (moving && ((sw[s] && s + 1u < k) || (s && sw[s - 1u]))) {
            /* a swapping pair [a, a + 2) slots: chip A slides right over B, B slides left beneath it */
            const uint32_t a = sw[s] ? s : s - 1u;
            const int32_t v = (int32_t)(pc - (a << 16)), d = (int32_t)t;
            const uint32_t ca = cs_cover(v, d + g, d + 65536 - g, per);
            const uint32_t cb = cs_cover(v, 65536 - d + g, 131072 - d - g, per);
            const fxv_px_t pa = fxv_blend16(cs_chip(x, perm[a]), act, lift);
            const fxv_px_t pb = fxv_blend16(cs_chip(x, perm[a + 1u]), act, lift);
            col = fxv_blend16(fxv_blend16(gap, pb, cb), pa, ca);
            col = fxv_add_px(col, fxv_scale_px(act, fxv_scale16(65535u - (ca > cb ? ca : cb), lift)));
        } else {
            col = fxv_blend16(gap, cs_chip(x, perm[s]), cs_cover((int32_t)(pc & 0xFFFFu), g, 65536 - g, per));
        }
        o[j] = col;
    }
}

const fxv_desc_t fxv_d_chromatic_sort = { cs_bytes, b5_init, cs_advance, cs_slice, cs_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 74 Night Traffic
 * Nagel-Schreckenberg traffic on a ring road of 2-pixel cells: each step every car speeds up by one (at most 5 cells a
 * step), slows to the gap ahead, and dawdles one cell slower at random, so jams form out of nothing
 * and creep backwards (custom1: 0 .. 62 %). Density (intensity): 6 .. 35 % of the cells, at most 120 cars a lane. Two ways (custom2 >= 128):
 * white headlights (c1) flowing one way, red tail lights (c2) the other, each lane its own road; one way: each car a
 * headlight in front and a tail light behind. Tail lights brighten while the car brakes. Traffic speed: 1 .. 8 steps a
 * second; cars glide between their step positions. Headlights carry a natural-white share (q) for W / CW; tail lights
 * stay pure red. Arena after the header (S = the cars a lane n allows, rounded to 4): per lane pos[S], prev[S] u16,
 * v[S], fl[S] (bit 0 braking, bit 1 braking the step before); then the head and tail light planes, u16 a pixel
 * (rebuilt each frame from the interpolated cars). h[10 + lane] cars placed; u[1] cars a lane, u[2] lanes, u[3] S,
 * u[4] cells, u[5] dawdle (Q16). */
static uint32_t nt_cap(uint32_t n) { const uint32_t c = n >> 1; return c < 1u ? 1u : c > 120u ? 120u : c; }
static uint32_t nt_stride(uint32_t n) { return (nt_cap(n) + 3u) & ~3u; }
static uint32_t nt_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + 12u * nt_stride(n) + 4u * n; }
static uint16_t *nt_pos(const fxv_ctx_t *x, uint32_t l) { return (uint16_t *)(x->arena + B5_HDR + l * 6u * x->u[3]); }
static uint16_t *nt_prev(const fxv_ctx_t *x, uint32_t l) { return nt_pos(x, l) + x->u[3]; }
static uint8_t *nt_v(const fxv_ctx_t *x, uint32_t l) { return x->arena + B5_HDR + l * 6u * x->u[3] + 4u * x->u[3]; }
static uint8_t *nt_fl(const fxv_ctx_t *x, uint32_t l) { return nt_v(x, l) + x->u[3]; }
static uint16_t *nt_plane(const fxv_ctx_t *x, uint32_t k) { return (uint16_t *)(x->arena + B5_HDR + 12u * x->u[3]) + k * x->n; }

static void nt_advance(fxv_ctx_t *x)
{
    const uint32_t cells = x->n >> 1 ? x->n >> 1 : 1u;
    uint32_t cars = (cells * (3932u + x->intensity * 75u)) >> 16;
    b5_h(x)[H_INC] = b5_inc(x, b5_log(1000u, 8000u, x->speed));
    if (cars < 1u) cars = 1u;
    x->u[1] = b5_min(cars, nt_cap(x->n));
    x->u[2] = x->custom2 >= 128u ? 2u : 1u;
    x->u[3] = nt_stride(x->n);
    x->u[4] = cells;
    x->u[5] = x->custom1 * 160u;
    x->job[0] = 0u;
    x->job[1] = 0u;
    x->job[2] = 0u;
}

/* one lane's step for cars [from, to): speed up, keep the gap, dawdle, move (gaps from the old positions) */
static void nt_step(fxv_ctx_t *x, uint32_t l, uint32_t from, uint32_t to)
{
    uint16_t *pos = nt_pos(x, l), *prev = nt_prev(x, l);
    uint8_t *v = nt_v(x, l), *fl = nt_fl(x, l);
    const uint32_t cars = x->u[1], cells = x->u[4], gen = b5_h(x)[H_GEN];
    uint32_t i;
    for (i = from; i < to; i++) {
        const uint32_t ahead = i + 1u < cars ? pos[i + 1u] : i ? prev[0] : pos[0];
        uint32_t gap = ahead + cells - pos[i] - 1u, s = v[i] + 1u, p;
        if (gap >= cells) gap -= cells;
        if (s > 5u) s = 5u;
        if (s > gap) s = gap;
        if (s && (fxv_rand(x->seed + l, gen, i) & 0xFFFFu) < x->u[5]) s--;
        fl[i] = (uint8_t)((fl[i] << 1 & 2u) | (s < v[i] ? 1u : 0u));
        v[i] = (uint8_t)s;
        prev[i] = pos[i];
        p = pos[i] + s;
        pos[i] = (uint16_t)(p >= cells ? p - cells : p);
    }
}

/* pixel jj of the road (a ring of `road` pixels) */
static uint32_t nt_wrap(int32_t jj, uint32_t road)
{
    while (jj < 0) jj += (int32_t)road;
    while (jj >= (int32_t)road) jj -= (int32_t)road;
    return (uint32_t)jj;
}

/* add a triangle of radius r8 (Q8 px, 256 or 320) and peak amp at x8 into plane p, round the road of `road` pixels */
static void nt_splat(uint16_t *p, uint32_t road, int32_t x8, int32_t r8, uint32_t amp)
{
    const uint32_t inv = r8 == 256 ? 256u : 205u;
    int32_t jj;
    for (jj = (x8 - r8) >> 8; jj <= (x8 + r8) >> 8; jj++) {
        const int32_t c = jj * 256 + 128, dd = c > x8 ? c - x8 : x8 - c;
        if (dd < r8) {
            const uint32_t w = b5_min(65535u, (uint32_t)(r8 - dd) * inv);
            const uint32_t at = nt_wrap(jj, road), s = p[at] + ((amp * w) >> 16);
            p[at] = (uint16_t)(s > 65535u ? 65535u : s);
        }
    }
}

/* a headlight's beam: a ramp from amp at x8 to 0 six pixels ahead (dir +1 / -1) */
static void nt_beam(uint16_t *p, uint32_t road, int32_t x8, int32_t dir, uint32_t amp)
{
    int32_t k;
    for (k = 0; k < 6; k++) {
        const int32_t c = x8 + dir * (k * 256 + 128), jj = c >> 8;
        const int32_t d = c - x8 > 0 ? c - x8 : x8 - c;
        const uint32_t at = nt_wrap(jj, road), s = p[at] + ((amp * (uint32_t)(1536 - d) * 43u) >> 16);
        p[at] = (uint16_t)(s > 65535u ? 65535u : s);
    }
}

static uint32_t nt_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x), l, i;
    const uint32_t cars = x->u[1], lanes = x->u[2], cells = x->u[4], road = 2u * cells;
    (void)quota;
    switch (x->job[0]) {
    case 0:                                              /* (re)place the cars of a lane whose count changed, 40 a slice */
        for (l = 0u; l < 2u; l++) {
            if (h[10u + l] == cars) continue;
            {
                uint16_t *pos = nt_pos(x, l), *prev = nt_prev(x, l);
                uint8_t *v = nt_v(x, l), *fl = nt_fl(x, l);
                const uint32_t end = b5_min(cars, x->job[2] + 40u);
                for (i = x->job[2]; i < end; i++) {
                    pos[i] = prev[i] = (uint16_t)((i * cells) / cars);
                    v[i] = (uint8_t)(fxv_hash2(x->seed + l, i) & 1u);
                    fl[i] = 0u;
                }
                x->job[2] = end;
                if (end >= cars) { h[10u + l] = cars; x->job[2] = 0u; }
            }
            return 1u;
        }
        x->job[0] = 1u;
        x->job[2] = 0u;
        return 1u;
    case 1:                                              /* model steps: 24 cars a slice */
        if (!x->job[1]) {
            if (!b5_next(x)) { x->u[0] = b5_frac(x); x->job[0] = 2u; return 1u; }
            x->job[1] = 1u;
            x->job[2] = 0u;
        }
        {
            const uint32_t lane = x->job[1] - 1u, end = b5_min(cars, x->job[2] + 24u);
            nt_step(x, lane, x->job[2], end);
            x->job[2] = end;
            if (end < cars) return 1u;
            x->job[2] = 0u;
            x->job[1] = lane + 1u < 2u ? lane + 2u : 0u;     /* both lanes step (the second one waits) */
        }
        return 1u;
    case 2:                                              /* clear the light planes, 256 pixels a slice */
        {
            uint16_t *p = nt_plane(x, 0u);
            const uint32_t end = b5_min(2u * x->n, x->job[2] + 256u);
            for (i = x->job[2]; i < end; i++) p[i] = 0u;
            x->job[2] = end;
            if (end < 2u * x->n) return 1u;
            x->job[0] = 3u;
            x->job[1] = 0u;
            x->job[2] = 0u;
        }
        return 1u;
    case 3:                                              /* splat the cars at their glide positions, 12 a slice */
        if (x->n >= 2u) {
            const uint32_t lane = x->job[1], f = x->u[0], end = b5_min(cars, x->job[2] + 12u);
            const uint16_t *pos = nt_pos(x, lane), *prev = nt_prev(x, lane);
            const uint8_t *fl = nt_fl(x, lane);
            uint16_t *head = nt_plane(x, 0u), *tail = nt_plane(x, 1u);
            for (i = x->job[2]; i < end; i++) {
                const uint32_t d = pos[i] >= prev[i] ? pos[i] - prev[i] : pos[i] + cells - prev[i];
                const uint32_t brk = fxv_lerp16(fl[i] & 2u ? 65535u : 0u, fl[i] & 1u ? 65535u : 0u, f);
                uint32_t c8 = prev[i] * 512u + 256u + ((d * f) >> 7);
                const uint32_t tl = 24000u + fxv_scale16(41535u, brk);
                if (c8 >= road * 256u) c8 -= road * 256u;
                if (lanes == 1u) {
                    nt_splat(head, road, (int32_t)c8 + 160, 256, 60000u);
                    nt_beam(head, road, (int32_t)c8 + 288, 1, 9000u);
                    nt_splat(tail, road, (int32_t)c8 - 160, 256, tl);
                } else if (lane == 0u) {
                    nt_splat(head, road, (int32_t)c8 + 64, 320, 60000u);
                    nt_beam(head, road, (int32_t)c8 + 320, 1, 12000u);
                } else {
                    nt_splat(tail, road, (int32_t)(road * 256u - c8), 320, tl);
                }
            }
            x->job[2] = end;
            if (end < cars) return 1u;
            x->job[2] = 0u;
            if (++x->job[1] < lanes) return 1u;
        }
        x->job[0] = 4u;
        return 1u;
    default:
        return 0u;
    }
}

static void nt_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint16_t *head = nt_plane(x, 0u), *tail = nt_plane(x, 1u);
    uint32_t j;
    if (x->pal_own) {
        const fxv_px_t hc = fxv_rgb16(x->c[0]), tc = fxv_rgb16(x->c[1]);
        for (j = 0u; j < count; j++) {
            const uint32_t i = from + j, hd = head[i], tl = tail[i];
            fxv_px_t col = fxv_add_px(fxv_scale_px(tc, tl), fxv_scale_px(hc, hd));
            col.q = hd ? 255u : 0u;
            col.k = 160u;
            o[j] = col;
        }
    } else {
        /* palette: the place on the road picks the colour, so the cars run through the whole palette as they go round;
         * headlights take that position, tail lights the opposite one (b5_opp); the lights' brightness, braking and
         * the dark road stay */
        const uint32_t st = (65536u << 8) / (x->n ? x->n : 1u);
        const fxv_px_t road = { 0u, 0u, 0u, 0u, 0u };
        uint32_t a8 = from * st + (st >> 1);
        for (j = 0u; j < count; j++, a8 += st) {
            const uint32_t i = from + j, hd = head[i], tl = tail[i], ua = a8 >> 8;
            o[j] = (hd | tl) ? fxv_add_px(fxv_scale_px(fxv_pal(x, b5_opp(x, ua)), tl), fxv_scale_px(fxv_pal(x, ua), hd)) : road;
        }
    }
}

const fxv_desc_t fxv_d_night_traffic = { nt_bytes, b5_init, nt_advance, nt_slice, nt_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 84 Sand Pour
 * An hourglass on its side: sand (a mass 0..255 a pixel, each pixel with its layer's palette colour, tinted towards the
 * sand colour c1, on the dim glass c2) lies at the source end. Grains leave from the inner edge of that reservoir,
 * carrying their pixel's colour, fall along the strip (intensity: the crossing time, 10 .. 1.5 s), bounce now and then
 * and land on the face of the heap at the far end, grain by grain; a heap pixel spills onto its outer neighbour when it
 * stands more than the softness allows above it (custom2: firm .. soft slopes). Pour rate (speed): the reservoir
 * empties in 2 min .. 6 s. Full (reservoir empty, every grain landed): a hold, then the glass flips (custom1 > 0: the
 * heap is the new reservoir, its layers pour back in reverse) or, custom1 0, the heap melts away and the source fills
 * again. The sand is 40 % of the strip, one pixel a grain. At most 32 grains fly, gliding between steps (20 ms).
 * Arena after the header: 32 grains (x, previous x, v: Q16 pixels from the source end; colour, mass), then the mass
 * and layer planes (u8 a pixel, physical order) and the frame's grain light plane (u16) and colour plane (u8).
 * h[8] set up, h[9] direction (0: source at pixel 0), h[10] phase (0 pour, 1 full, 2 melt, 3 refill), h[11] the
 * phase's steps, h[12] reservoir front + 1 (from the source end, 0: empty), h[13] pour accumulator (Q16 grains),
 * h[14] grains to pour (G), h[15] live grains, h[16] the heap's face (lowest s landed on), h[17] the melt's
 * remainder, h[18] the first fill's position; u[1] grains a step (Q16), u[2] terminal speed (Q16 px a step),
 * u[3] softness threshold, u[4] hold steps. */
#define SP_G 32u
static uint32_t sp_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + SP_G * 16u + ((5u * n + 3u) & ~3u); }
static int32_t *sp_gx(const fxv_ctx_t *x) { return (int32_t *)(x->arena + B5_HDR); }
static int32_t *sp_gp(const fxv_ctx_t *x) { return sp_gx(x) + SP_G; }
static int32_t *sp_gv(const fxv_ctx_t *x) { return sp_gx(x) + 2u * SP_G; }
static uint8_t *sp_gc(const fxv_ctx_t *x) { return x->arena + B5_HDR + 12u * SP_G; }
static uint8_t *sp_gm(const fxv_ctx_t *x) { return sp_gc(x) + SP_G; }
static uint8_t *sp_m(const fxv_ctx_t *x) { return x->arena + B5_HDR + 16u * SP_G; }
static uint8_t *sp_lc(const fxv_ctx_t *x) { return sp_m(x) + x->n; }
static uint16_t *sp_gl(const fxv_ctx_t *x) { return (uint16_t *)(x->arena + B5_HDR + 16u * SP_G + ((2u * x->n + 1u) & ~1u)); }
static uint8_t *sp_gk(const fxv_ctx_t *x) { return (uint8_t *)(sp_gl(x) + x->n); }
static uint32_t sp_at(const fxv_ctx_t *x, uint32_t s) { return b5_h(x)[9] ? x->n - 1u - s : s; }   /* s -> pixel */

static void sp_advance(fxv_ctx_t *x)
{
    const uint32_t n = x->n ? x->n : 1u, g = b5_min(320u, (2u * n) / 5u ? (2u * n) / 5u : 1u);
    b5_h(x)[H_INC] = b5_inc(x, 50000u);                                  /* 50 steps a second */
    b5_h(x)[14] = g;
    x->u[1] = (g * 1310720u) / b5_log(6000u, 120000u, 255u - x->speed);
    x->u[2] = (n * 1310720u) / b5_log(1500u, 10000u, 255u - x->intensity);
    x->u[3] = 255u - (x->custom2 * 239u) / 255u;
    x->u[4] = (500u + x->custom1 * 16u) / 20u;
    x->job[0] = 0u;
    x->job[1] = 0u;
}

/* the reservoir at the source end, its pixels s in [from, to) (of g): amount more sand, the palette's layers in order */
static void sp_fill(fxv_ctx_t *x, uint32_t amount, uint32_t from, uint32_t to)
{
    uint8_t *m = sp_m(x), *lc = sp_lc(x);
    const uint32_t g = b5_h(x)[14], n = x->n, inv = 16777216u / g;     /* layer = s x 256 / g */
    uint32_t s;
    for (s = from; s < to && s < g && s < n; s++) {
        const uint32_t i = sp_at(x, s), v = m[i] + amount;
        m[i] = (uint8_t)(v > 255u ? 255u : v);
        lc[i] = (uint8_t)((s * inv) >> 16);
    }
}

static void sp_filled(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x);
    h[12] = b5_min(h[14], x->n);
    h[16] = x->n;
}

/* a model step's state: the hold and flip, the melt and refill (96 .. 200 pixels a slice), the launches. 0: call again */
static uint32_t sp_phase(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x), k;
    const uint32_t n = x->n;
    int32_t *gx = sp_gx(x), *gp = sp_gp(x), *gv = sp_gv(x);
    uint8_t *gc = sp_gc(x), *gm = sp_gm(x), *m = sp_m(x), *lc = sp_lc(x);
    if (h[10] == 1u) {                                   /* full: hold, then flip or melt */
        if (++h[11] < x->u[4]) return 1u;
        h[11] = 0u;
        if (x->custom1) {
            uint32_t s;
            h[9] ^= 1u;
            h[10] = 0u;
            h[12] = 0u;
            h[16] = n;
            for (s = n >> 1; s > 0u; s--) if (m[sp_at(x, s - 1u)]) { h[12] = s; break; }
        } else {
            h[10] = 2u;
        }
        return 1u;
    }
    if (h[10] == 2u) {                                   /* the heap melts away */
        const uint32_t end = b5_min(n, x->job[2] + 200u);
        uint32_t i;
        for (i = x->job[2]; i < end; i++) { m[i] = (uint8_t)(m[i] > 3u ? m[i] - 3u : 0u); h[17] |= m[i]; }
        x->job[2] = end;
        if (end < n) return 0u;
        if (!h[17]) { h[10] = 3u; h[11] = 0u; }
        h[17] = 0u;
        return 1u;
    }
    if (h[10] == 3u) {                                   /* the source fills again */
        const uint32_t end = x->job[2] + 96u;
        sp_fill(x, 5u, x->job[2], end);
        x->job[2] = end;
        if (end < b5_min(h[14], n)) return 0u;
        sp_filled(x);
        if (++h[11] >= 51u) { h[10] = 0u; h[11] = 0u; }
        return 1u;
    }
    /* pour: launch from the reservoir's inner edge */
    h[13] += x->u[1];
    while (h[13] >= 65536u) {
        uint32_t slot = SP_G, take = 0u, c = 0u;
        for (k = 0u; k < SP_G; k++) if (!gm[k]) { slot = k; break; }
        if (slot == SP_G || !h[12]) { h[13] = 65535u; break; }
        h[13] -= 65536u;
        while (h[12] && take < 255u) {
            const uint32_t i = sp_at(x, h[12] - 1u), t = b5_min(255u - take, m[i]);
            if (!take) c = lc[i];
            take += t;
            m[i] = (uint8_t)(m[i] - t);
            if (!m[i]) h[12]--;
        }
        if (!take) break;
        gx[slot] = gp[slot] = (int32_t)((h[12] << 16) + 32768u);
        gv[slot] = 0;
        gc[slot] = (uint8_t)c;
        gm[slot] = (uint8_t)take;
        h[15]++;
    }
    return 1u;
}

/* grains [from, to): speed up towards the terminal speed, bounce at random, land on the heap's face */
static void sp_fly(fxv_ctx_t *x, uint32_t from, uint32_t to)
{
    uint32_t *h = b5_h(x), k;
    const uint32_t n = x->n, gen = h[H_GEN];
    int32_t *gx = sp_gx(x), *gp = sp_gp(x), *gv = sp_gv(x);
    uint8_t *gc = sp_gc(x), *gm = sp_gm(x), *m = sp_m(x), *lc = sp_lc(x);
    for (k = from; k < to; k++) {
        int32_t v;
        uint32_t s, end;
        if (!gm[k]) continue;
        v = gv[k] + (int32_t)(x->u[2] / 15u);
        if (v > (int32_t)x->u[2]) v = (int32_t)x->u[2];
        if ((fxv_rand(x->seed, gen, 840u + k) & 15u) == 0u) v = (v * 23) >> 6;
        gv[k] = v;
        gp[k] = gx[k];
        s = (uint32_t)gx[k] >> 16;
        end = (uint32_t)(gx[k] + v) >> 16;
        for (; s < end && s + 1u < n && m[sp_at(x, s + 1u)] < 128u; s++) { }
        if (s + 1u >= n || m[sp_at(x, s + 1u)] >= 128u) {       /* landed at s */
            const uint32_t i = sp_at(x, s), sum = m[i] + gm[k];
            m[i] = (uint8_t)(sum > 255u ? 255u : sum);
            lc[i] = gc[k];
            if (sum > 255u && s) {
                const uint32_t o = sp_at(x, s - 1u), r = m[o] + sum - 255u;
                m[o] = (uint8_t)(r > 255u ? 255u : r);
                lc[o] = gc[k];
            }
            gm[k] = 0u;
            h[15]--;
            if (s < h[16]) h[16] = s;
        } else {
            gx[k] += v;
        }
    }
}

/* the heap's spill near its face (48 pixels either side, in the heap's half): a pixel more than the softness above its
 * outer neighbour gives it half the excess */
static void sp_spill(fxv_ctx_t *x)
{
    uint8_t *m = sp_m(x), *lc = sp_lc(x);
    uint32_t *h = b5_h(x);
    const uint32_t n = x->n, th = x->u[3], end = b5_min(n, h[16] + 48u);
    uint32_t s = h[16] > (n >> 1) + 48u ? h[16] - 48u : n >> 1;
    for (; s + 1u < end; s++) {
        const uint32_t a = sp_at(x, s), b = sp_at(x, s + 1u);
        if (m[b] > m[a] + th) {
            const uint32_t d = (m[b] - m[a] - th + 1u) >> 1;
            m[b] = (uint8_t)(m[b] - d);
            m[a] = (uint8_t)(m[a] + d);
            lc[a] = lc[b];
            if (s < h[16]) h[16] = s;
        }
    }
}

static void sp_dot(uint16_t *gl, uint8_t *gk, uint32_t n, int32_t x8, uint32_t amp, uint32_t c)
{
    int32_t jj;
    for (jj = (x8 - 256) >> 8; jj <= (x8 + 256) >> 8; jj++) {
        const int32_t cc = jj * 256 + 128, dd = cc > x8 ? cc - x8 : x8 - cc;
        if (jj >= 0 && jj < (int32_t)n && dd < 256) {
            const uint32_t s = gl[jj] + ((amp * (uint32_t)(256 - dd)) >> 8);
            gl[jj] = (uint16_t)(s > 65535u ? 65535u : s);
            gk[jj] = (uint8_t)c;
        }
    }
}

static uint32_t sp_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x), k;
    const uint32_t n = x->n;
    (void)quota;
    if (!h[8]) {                                         /* the first reservoir, 96 pixels a slice */
        const uint32_t end = h[18] + 96u;
        sp_fill(x, 255u, h[18], end);
        h[18] = end;
        if (end >= b5_min(h[14], n)) { sp_filled(x); h[8] = 1u; }
        return 1u;
    }
    if (x->job[0] == 0u) {                               /* model steps: the state, the grains in two halves, the spill */
        switch (x->job[1]) {
        case 1u:
            if (sp_phase(x)) x->job[1] = 2u;
            return 1u;
        case 2u: case 3u: case 4u:
            sp_fly(x, (x->job[1] - 2u) * 8u, (x->job[1] - 1u) * 8u);
            x->job[1]++;
            return 1u;
        case 5u:
            sp_fly(x, 24u, SP_G);
            if (h[10] == 0u && !h[12] && !h[15]) { h[10] = 1u; h[11] = 0u; }
            x->job[1] = 6u;
            return 1u;
        case 6u:
            if (h[10] == 0u) sp_spill(x);
            x->job[1] = 0u;
            return 1u;
        default:
            break;
        }
        if (b5_next(x)) { x->job[1] = 1u; x->job[2] = 0u; return 1u; }
        x->u[0] = b5_frac(x);
        x->job[0] = 1u;
        x->job[2] = 0u;
        return 1u;
    }
    if (x->job[0] == 1u) {                               /* clear the grain light and colour, 256 pixels a slice */
        uint16_t *gl = sp_gl(x);
        uint8_t *gk = sp_gk(x);
        const uint32_t end = b5_min(n, x->job[2] + 256u);
        for (k = x->job[2]; k < end; k++) { gl[k] = 0u; gk[k] = 0u; }
        x->job[2] = end;
        if (end < n) return 1u;
        x->job[0] = 2u;
        return 1u;
    }
    if (x->job[0] == 2u || x->job[0] == 3u) {            /* the grains at their glide positions (16 a slice), with a trail */
        const int32_t *gx = sp_gx(x), *gp = sp_gp(x);
        const uint8_t *gc = sp_gc(x), *gm = sp_gm(x);
        uint16_t *gl = sp_gl(x);
        uint8_t *gk = sp_gk(x);
        const uint32_t f = x->u[0], k0 = (x->job[0] - 2u) * 16u;
        for (k = k0; k < k0 + 16u; k++) {
            if (gm[k] && n) {
                const int32_t d = gx[k] - gp[k], s8 = (gp[k] + (int32_t)(((uint32_t)d >> 4) * (f >> 4) >> 8)) >> 8;
                const int32_t x8 = h[9] ? (int32_t)(n << 8) - s8 : s8, back = h[9] ? (d >> 9) : -(d >> 9);
                sp_dot(gl, gk, n, x8, 60000u, gc[k]);
                sp_dot(gl, gk, n, x8 + back, 22000u, gc[k]);
            }
        }
        x->job[0]++;
        return 1u;
    }
    return 0u;
}

static void sp_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint8_t *m = sp_m(x), *lc = sp_lc(x), *gk = sp_gk(x);
    const uint16_t *gl = sp_gl(x);
    const fxv_px_t sand = fxv_rgb16(x->c[0]), glass = fxv_rgb16(x->c[1]);
    uint32_t j;
    for (j = 0u; j < count; j++) {
        const uint32_t i = from + j;
        fxv_px_t col = glass;
        if (m[i]) col = fxv_blend16(glass, fxv_blend16(sand, fxv_pal(x, (uint32_t)lc[i] << 8), 40000u), (uint32_t)m[i] * 257u);
        if (gl[i]) col = fxv_add_px(col, fxv_scale_px(fxv_blend16(sand, fxv_pal(x, (uint32_t)gk[i] << 8), 20000u), gl[i]));
        o[j] = col;
    }
}

const fxv_desc_t fxv_d_sand_pour = { sp_bytes, b5_init, sp_advance, sp_slice, sp_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 89 Avalanche Loom
 * A sandpile on the strip: heights (15 bits) relax synchronously, a cell more than the topple threshold above a
 * neighbour (custom1: loose 2 % .. firm 27 % of the ridge height a pixel) gives that side a quarter of the excess, and a
 * cell over the ridge height (intensity) gives way: it sheds a quarter of its excess both ways each sweep until it is
 * below 60 % of the ridge height (bit 15: giving way), so the slump runs on through its neighbours as a cascade. The
 * ends are open and every height erodes slowly (~1 min). Feed spouts (2 + 1 a 60 pixels, at most 12) wander slowly and
 * drop small deposits near them (speed: the feed rate, 3 .. 120 deposits
 * a second a 100 pixels; speed 0 pauses, the layer's rule). Relaxation pace (custom2): 8 .. 80 sweeps a second; the
 * heights glide between sweeps. Colour: the palette by height (0.1 .. 0.8 of it) over the empty cells c2; a
 * toppling cell flares in c3 and fades. Planes: two u16 heights (ping-pong), a u8 glow: 5 bytes a pixel. h[8] deposit
 * accumulator (Q16); u[1] ridge height, u[2] slope threshold, u[3] deposits a sweep (Q16), u[4] spouts, u[5] glow
 * fall a sweep, u[6] 2^24 / the ridge height (palette scale), u[7] 2^8 x 65535 / the visible floor; h[40..51] the
 * spouts' cells (refreshed every 64 sweeps), h[52] their sweep epoch + 1. A sweep: spouts (when due), feed (at most 8
 * deposits), then 40 cells a slice. */
static uint32_t al_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + ((5u * n + 3u) & ~3u); }
static uint8_t *al_glow(const fxv_ctx_t *x) { return x->arena + B5_HDR + 4u * x->n; }

static void al_advance(fxv_ctx_t *x)
{
    const uint32_t pace = b5_log(8000u, 80000u, x->custom2), hmax = 4000u + x->intensity * 100u;
    const uint32_t feed = (b5_log(3000u, 120000u, x->speed) * x->n) / 100u;            /* mHz over the run */
    b5_h(x)[H_INC] = b5_inc(x, pace);
    x->u[1] = hmax;
    x->u[2] = (hmax * (5u + (x->custom1 * 64u) / 255u)) >> 8;
    x->u[3] = (feed * 256u) / (pace >> 8);
    x->u[4] = b5_min(12u, 2u + x->n / 60u);
    x->u[5] = 255u / (1u + pace / 2500u);
    x->u[6] = 16777216u / hmax;
    x->u[7] = (65535u << 8) / (hmax / 10u);
    x->job[0] = 0u;
}

/* the spouts' cells for this sweep's epoch of 64 sweeps: each wanders along a slow noise */
static void al_spouts(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x), s;
    const uint32_t e = h[H_GEN] >> 6, n = x->n;
    for (s = 0u; s < 12u; s++) {
        const int32_t nz = fxv_noise1_q15(x->seed + s * 977u, 0u, e * 3072u, 1u);
        h[40u + s] = (((uint32_t)(nz + 32768)) * n) >> 16;
    }
    h[52] = e + 1u;
}

/* the deposits of a new sweep (at most 8), into the current plane (before the sweep reads it) */
static void al_feed(fxv_ctx_t *x)
{
    uint32_t *h = b5_h(x), k = 0u;
    uint16_t *cur = b5_w(x, h[H_BANK]);
    const uint32_t n = x->n, gen = h[H_GEN], pk = x->u[1] / 12u;
    h[8] += x->u[3];
    while (h[8] >= 65536u && n) {
        const uint32_t r = fxv_rand(x->seed, gen, 890u + k), s = (r & 255u) % x->u[4];
        int32_t at = (int32_t)h[40u + s] + (int32_t)((r >> 8) % 7u) - 3;
        uint32_t v;
        h[8] -= 65536u;
        if (at < 0) at = 0;
        if (at >= (int32_t)n) at = (int32_t)n - 1;
        v = (cur[at] & 0x7FFFu) + pk;
        cur[at] = (uint16_t)((cur[at] & 0x8000u) | (v > 32767u ? 32767u : v));
        if (++k >= 8u) { h[8] &= 0xFFFFu; break; }
    }
}

static uint32_t al_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x);
    const uint32_t n = x->n;
    (void)quota;
    if (!x->job[0]) {
        if (!b5_next(x)) { x->u[0] = b5_frac(x); return 0u; }
        x->job[0] = h[52] != (h[H_GEN] >> 6) + 1u ? 1u : 2u;
        return 1u;
    }
    if (x->job[0] == 1u) { al_spouts(x); x->job[0] = 2u; return 1u; }
    if (x->job[0] == 2u) { al_feed(x); x->job[0] = 3u; x->job[1] = 0u; return 1u; }
    {
        const uint16_t *o = b5_w(x, h[H_BANK]);
        uint16_t *d = b5_w(x, 1u - h[H_BANK]);
        uint8_t *g = al_glow(x);
        const int32_t hm = (int32_t)x->u[1], th = (int32_t)x->u[2], lo = (hm * 154) >> 8;
        uint32_t i = x->job[1];
        const uint32_t end = b5_min(n, i + 40u);
        for (; i < end; i++) {
            /* what c gives each side, and what its neighbours give it (old plane only; outside the ends: 0) */
            const uint32_t oc = o[i], ol = i ? o[i - 1u] : 0u, orr = i + 1u < n ? o[i + 1u] : 0u;
            const int32_t c = (int32_t)(oc & 0x7FFFu), l = (int32_t)(ol & 0x7FFFu), r = (int32_t)(orr & 0x7FFFu);
            const int32_t gc = oc >> 15 ? lo : hm, gl = ol >> 15 ? lo : hm, gr = orr >> 15 ? lo : hm;
            const int32_t ex = c > gc ? (c - gc) >> 2 : 0;
            const int32_t outl = (c - l > th ? (c - l - th) >> 2 : 0) + ex, outr = (c - r > th ? (c - r - th) >> 2 : 0) + ex;
            const int32_t inl = i ? (l - c > th ? (l - c - th) >> 2 : 0) + (l > gl ? (l - gl) >> 2 : 0) : 0;
            const int32_t inr = i + 1u < n ? (r - c > th ? (r - c - th) >> 2 : 0) + (r > gr ? (r - gr) >> 2 : 0) : 0;
            int32_t v = c - outl - outr + inl + inr;
            if (v > 0) v -= (v >> 10) + 1;               /* erosion */
            if (v < 0) v = 0;
            if (v > 32767) v = 32767;
            d[i] = (uint16_t)((uint32_t)v | (ex > 0 && v > lo ? 0x8000u : 0u));
            g[i] = (uint8_t)(outl + outr > 0 ? 255u : g[i] > x->u[5] ? g[i] - x->u[5] : 0u);
        }
        x->job[1] = end;
        if (end < n) return 1u;
        h[H_BANK] ^= 1u;
        x->job[0] = 0u;
        return 1u;
    }
}

static void al_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b5_h(x), f = x->u[0], fall = x->u[5], ps = x->u[6], low = x->u[1] / 10u;
    const uint16_t *cur = b5_w(x, h[H_BANK]), *old = b5_w(x, 1u - h[H_BANK]);
    const uint8_t *g = al_glow(x);
    const fxv_px_t empty = fxv_rgb16(x->c[1]);
    fxv_px_t ridge = fxv_rgb16(x->c[2]);
    uint32_t j;
    ridge.q = 255u;                                      /* the flare: a warm natural white share for W / WW */
    ridge.k = 60u;
    for (j = 0u; j < count; j++) {
        const uint32_t i = from + j, hv = fxv_lerp16(old[i] & 0x7FFFu, cur[i] & 0x7FFFu, f);
        const uint32_t u = b5_min(65535u, (hv * ps) >> 8), fl = (fall * f) >> 16, gl = g[i] > fl ? g[i] - fl : 0u;
        fxv_px_t col = fxv_blend16(empty, fxv_pal(x, 6554u + ((u * 179u) >> 8)), hv >= low ? 65535u : (hv * x->u[7]) >> 8);
        if (gl) col = fxv_blend16(col, ridge, gl * 200u);
        o[j] = col;
    }
}

const fxv_desc_t fxv_d_avalanche_loom = { al_bytes, b5_init, al_advance, al_slice, al_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 71 Morphogen
 * Gray-Scott reaction-diffusion (fxv_rd_step, Q12, du 0.25, dv 0.125, no-flux ends) on cells of 2 pixels: feed
 * (intensity) 140 .. 200 and kill (custom1) 236 .. 252 in Q12 span the 1-D regimes: spots that divide and wander apart
 * to fill the strip, dense stripes, and spots that die out (then, after 2 s, two new seeds start again). Growth rate
 * (speed): three steps a tick, ~270 steps a second at the default speed. Reseed (custom2: 0 never, then every 30 .. 1 s
 * of growth at the default speed): a random patch is wiped (its neighbours divide into the gap) or, empty, gets a new
 * seed. A new seed (u = v = 0.5) fades in over ~0.35 s instead of popping. Colour:
 * the activator v along the palette (deep background to bright spots), pixels between cell centres interpolated; the
 * densest spots get a cool natural-white share for W / CW. Planes: u, v twice (ping-pong), u16 a cell. h[8] set up,
 * h[9] the set-up fill position, h[10] steps to the next reseed, h[11] the step's highest v, h[12] steps before
 * a die-out may reseed, h[13..14] the last two seeds' cells + 1, h[15..16] their ages (steps); u[1] cells, u[2] feed,
 * u[3] kill, u[4] steps between reseeds (0: none). */
static uint32_t mg_cells(uint32_t n) { return (n + 1u) >> 1; }
static uint32_t mg_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + 8u * mg_cells(n); }
static int16_t *mg_p(const fxv_ctx_t *x, uint32_t k) { return (int16_t *)(x->arena + B5_HDR) + k * mg_cells(x->n); }

/* B5-local candidate for B0: fxv_rd_step(.., FXV_EDGE_CLAMP) with du 1024 / dv 512 and the neighbours read inline:
 * the same planes (1024 lap >> 12 is lap >> 2, 512 lap >> 12 is lap >> 3), about a third of the cycles */
static void mg_rd(const int16_t *u, const int16_t *v, int16_t *un, int16_t *vn, uint32_t n, int32_t feed, int32_t fk,
                  uint32_t from, uint32_t to)
{
    uint32_t i;
    for (i = from; i < to; i++) {
        const int32_t uu = u[i], vv = v[i], uvv = (uu * ((vv * vv) >> 12)) >> 12;
        const int32_t ul = i ? u[i - 1u] : uu, ur = i + 1u < n ? u[i + 1u] : uu;
        const int32_t vl = i ? v[i - 1u] : vv, vr = i + 1u < n ? v[i + 1u] : vv;
        const int32_t a = uu + ((ul + ur - 2 * uu) >> 2) - uvv + ((feed * (4096 - uu)) >> 12);
        const int32_t b = vv + ((vl + vr - 2 * vv) >> 3) + uvv - ((fk * vv) >> 12);
        un[i] = (int16_t)(a < 0 ? 0 : a > 4096 ? 4096 : a);
        vn[i] = (int16_t)(b < 0 ? 0 : b > 4096 ? 4096 : b);
    }
}

static void mg_advance(fxv_ctx_t *x)
{
    b5_h(x)[H_INC] = x->speed ? 196608u : 0u;                           /* three steps a tick */
    x->u[1] = mg_cells(x->n);
    x->u[2] = 140u + (x->intensity * 60u) / 255u;
    x->u[3] = 236u + (x->custom1 * 16u) / 255u;
    x->u[4] = x->custom2 ? (b5_log(1000u, 30000u, 255u - x->custom2) * 27u) / 100u : 0u;
    x->job[0] = 0u;
}

/* a seed (u = v = 0.5 over 6 cells) centred at cell c of the current planes; it fades in (the newest two) */
static void mg_seed(fxv_ctx_t *x, uint32_t c)
{
    const uint32_t b = b5_h(x)[H_BANK] * 2u, cells = x->u[1];
    int16_t *u = mg_p(x, b), *v = mg_p(x, b + 1u);
    uint32_t *h = b5_h(x), i;
    for (i = c >= 3u ? c - 3u : 0u; i < c + 3u && i < cells; i++) { u[i] = 2048; v[i] = 2048; }
    h[14] = h[13];
    h[16] = h[15];
    h[13] = c + 1u;
    h[15] = 0u;
}

/* a reseed event before a step: wipe a patch with spots in it, or seed an empty one; or two seeds when all died */
static void mg_event(fxv_ctx_t *x, uint32_t dead)
{
    uint32_t *h = b5_h(x);
    const uint32_t cells = x->u[1], r = fxv_rand(x->seed, h[H_GEN], 710u), w = cells / 9u + 6u;
    const uint32_t c0 = cells > w ? ((r >> 8) % (cells - w + 1u)) : 0u, e = b5_min(cells, c0 + w);
    const uint32_t b = h[H_BANK] * 2u;
    int16_t *u = mg_p(x, b), *v = mg_p(x, b + 1u);
    uint32_t i, any = 0u;
    if (dead) {
        mg_seed(x, cells / 4u + ((r >> 4) & 7u));
        mg_seed(x, (cells * 3u) / 4u - ((r >> 12) & 7u));
        return;
    }
    for (i = c0; i < e; i++) any |= v[i] > 512;
    if (!any) { mg_seed(x, (c0 + e) >> 1); return; }
    for (i = c0; i < e; i++) { u[i] = 4096; v[i] = 0; }
}

static uint32_t mg_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x);
    const uint32_t cells = x->u[1];
    (void)quota;
    if (!h[8]) {                                         /* set up: u = 1 in both banks, then four seeds */
        int16_t *a = mg_p(x, 0u), *b = mg_p(x, 2u);
        const uint32_t end = b5_min(cells, h[9] + 128u);
        uint32_t i;
        for (i = h[9]; i < end; i++) { a[i] = 4096; b[i] = 4096; }
        h[9] = end;
        if (end < cells) return 1u;
        for (i = 0u; i < 4u; i++) mg_seed(x, (cells * (2u * i + 1u)) / 8u + ((fxv_hash2(x->seed, i) >> 16) % 5u));
        h[8] = 1u;
        h[10] = x->u[4];
        h[12] = 540u;
        return 1u;
    }
    if (!x->job[0]) {
        if (!b5_next(x)) { x->u[0] = b5_frac(x); return 0u; }
        if (x->u[4] && (!h[10] || h[10] > x->u[4])) { mg_event(x, 0u); h[10] = x->u[4]; }
        if (h[10]) h[10]--;
        if (h[15] < 96u) h[15]++;
        if (h[16] < 96u) h[16]++;
        x->job[0] = 1u;
        x->job[1] = 0u;
        h[11] = 0u;
        return 1u;                                       /* (the step's set-up a slice of its own: with a reseed's wipe
                                                            and a chunk of cells one slice reached 6,004 cycles) */
    }
    {
        const uint32_t b = h[H_BANK] * 2u, nb = 2u - b, from = x->job[1], end = b5_min(cells, from + 48u);   /* (56: 4,659) */
        const int16_t *u = mg_p(x, b), *v = mg_p(x, b + 1u);
        int16_t *un = mg_p(x, nb), *vn = mg_p(x, nb + 1u);
        uint32_t i;
        mg_rd(u, v, un, vn, cells, (int32_t)x->u[2], (int32_t)(x->u[2] + x->u[3]), from, end);
        for (i = from; i < end; i++) if ((uint32_t)vn[i] > h[11]) h[11] = (uint32_t)vn[i];
        x->job[1] = end;
        if (end < cells) return 1u;
        h[H_BANK] ^= 1u;
        x->job[0] = 0u;
        if (h[11] >= 64u) h[12] = 540u;                  /* everything died: start again after ~2 s */
        else if (h[12]) h[12]--;
        else if (cells) { mg_event(x, 1u); h[12] = 540u; }
        return 1u;
    }
}

static void mg_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint32_t *h = b5_h(x), cells = x->u[1];
    const int16_t *v = mg_p(x, h[H_BANK] * 2u + 1u);
    const uint32_t f0 = h[15] < 96u ? fxv_smooth16((h[15] * 65535u) / 96u) : 65535u;
    const uint32_t f1 = h[16] < 96u ? fxv_smooth16((h[16] * 65535u) / 96u) : 65535u;
    uint32_t j;
    for (j = 0u; j < count; j++) {
        const uint32_t i = from + j, c = i >> 1;
        uint32_t a, b, w, val;
        fxv_px_t col;
        if (i & 1u) { a = c; b = c + 1u < cells ? c + 1u : c; w = 16384u; }   /* a quarter cell past c */
        else { a = c ? c - 1u : 0u; b = c; w = 49152u; }                       /* a quarter cell before c */
        val = fxv_lerp16((uint32_t)v[a], (uint32_t)v[b], w);
        if (f0 < 65535u && c + 4u >= h[13] && c <= h[13] + 2u) val = fxv_scale16(val, f0);   /* a fresh seed */
        if (f1 < 65535u && c + 4u >= h[14] && c <= h[14] + 2u) val = fxv_scale16(val, f1);
        col = fxv_pal(x, b5_min(65535u, val * 46u));
        if (val > 1150u) { col.q = (uint8_t)b5_min(255u, val - 1150u); col.k = 230u; }
        o[j] = col;
    }
}

const fxv_desc_t fxv_d_morphogen = { mg_bytes, b5_init, mg_advance, mg_slice, mg_render, 0, 0, b5_digest };

/* ------------------------------------------------------------------------------- 64 Silt Settling
 * Fine grains (palette colours drifting slowly with time) appear in the water (c2, a slow shimmer) and drift towards the
 * nearest of 2 .. 12 resting banks (custom1), slowing as they settle (speed: the transit of the strip, 30 .. 2 s), and land
 * on the bank's face, a quarter pixel of sediment each, the newest layer on the outside. Supply (intensity): 0.3 .. 12
 * grains a second a 100 pixels. Each bank relaxes to the firmness slope (custom2: loose .. firm, the most sediment one
 * pixel may hold above the next) and, after a load of grains (20 .. 80 by the firmness), slumps: for 3 s its slope limit
 * drops to a third and the bank spreads into a new arrangement. Sediment dissolves slowly, so supply and loss balance
 * (about a third of the strip settled). The crests glow in c3 with a warm natural-white share. 25 model steps a second;
 * grains glide between them. Arena: 64 grains (x, previous x: Q16 px; bank, colour, mass), then the sediment and layer
 * planes (u8 a pixel), the frame's grain light (u16) and colour (u8). h[8] spawn accumulator (Q16), h[16..27] grains
 * landed on each bank, h[28..39] each bank's slump steps left; u[1] banks, u[2] grains a step (Q16), u[3] drift (Q16 px
 * a step), u[4] slope limit, u[5] banks a pixel (Q16), u[6] the load that slumps a bank. */
#define SS_G 64u
static uint32_t ss_bytes(uint32_t n, uint32_t tier) { (void)tier; return B5_HDR + SS_G * 12u + ((5u * n + 3u) & ~3u); }
static int32_t *ss_gx(const fxv_ctx_t *x) { return (int32_t *)(x->arena + B5_HDR); }
static int32_t *ss_gp(const fxv_ctx_t *x) { return ss_gx(x) + SS_G; }
static uint8_t *ss_gb(const fxv_ctx_t *x) { return x->arena + B5_HDR + 8u * SS_G; }
static uint8_t *ss_gc(const fxv_ctx_t *x) { return ss_gb(x) + SS_G; }
static uint8_t *ss_gm(const fxv_ctx_t *x) { return ss_gb(x) + 2u * SS_G; }
static uint8_t *ss_d(const fxv_ctx_t *x) { return x->arena + B5_HDR + 12u * SS_G; }
static uint8_t *ss_lc(const fxv_ctx_t *x) { return ss_d(x) + x->n; }
static uint16_t *ss_gl(const fxv_ctx_t *x) { return (uint16_t *)(x->arena + B5_HDR + 12u * SS_G + ((2u * x->n + 1u) & ~1u)); }
static uint8_t *ss_gk(const fxv_ctx_t *x) { return (uint8_t *)(ss_gl(x) + x->n); }

/* bank k's centre, Q16 px: evenly spaced, jittered by the seed */
static int32_t ss_centre(const fxv_ctx_t *x, uint32_t k)
{
    const uint32_t banks = x->u[1], sp = ((uint32_t)x->n << 16) / banks;
    const int32_t j = (int32_t)((fxv_hash2(x->seed, 640u + k) >> 18) % (sp >> 17 | 1u)) - (int32_t)(sp >> 18);
    return (int32_t)(sp * k + (sp >> 1)) + j * 65536;   /* +- a quarter spacing */
}

static void ss_advance(fxv_ctx_t *x)
{
    const uint32_t n = x->n ? x->n : 1u, banks = 2u + (x->custom1 * 10u) / 255u;
    b5_h(x)[H_INC] = b5_inc(x, 25000u);
    x->u[1] = banks;
    x->u[2] = (((b5_log(300u, 12000u, x->intensity) * n) / 100u) * 2621u) / 1000u;
    x->u[3] = (n * 2621440u) / b5_log(2000u, 30000u, 255u - x->speed);
    x->u[4] = 24u + (x->custom2 * 176u) / 255u;
    x->u[5] = (banks << 16) / n;
    x->u[6] = 20u + (x->custom2 * 60u) / 255u;
    x->job[0] = 0u;
}

/* one model step's grains, in four quarters (`half`: 0..3; 32 live grains in one slice cost ~6,200 cycles): spawns and
 * grains 0..15, then 16..31, 32..47, and 48..63 with the slump counters (the banks' relaxation runs as sliced sweeps
 * afterwards) */
static void ss_grains(fxv_ctx_t *x, uint32_t half)
{
    uint32_t *h = b5_h(x), k;
    const uint32_t n = x->n, gen = h[H_GEN], banks = x->u[1];
    int32_t *gx = ss_gx(x), *gp = ss_gp(x);
    uint8_t *gb = ss_gb(x), *gc = ss_gc(x), *gm = ss_gm(x), *d = ss_d(x), *lc = ss_lc(x);
    if (!half) h[8] += x->u[2];
    for (k = 0u; !half && k < SS_G && h[8] >= 65536u && n; k++) {
        if (!gm[k]) {
            const uint32_t r = fxv_rand(x->seed, gen, 6400u + k), px = ((r >> 16) * n) >> 16;
            gx[k] = gp[k] = (int32_t)((px << 16) + 32768u);
            gb[k] = (uint8_t)b5_min(banks - 1u, (px * x->u[5]) >> 16);
            gc[k] = (uint8_t)(64u + (gen >> 6) + ((r & 63u) >> 1));
            gm[k] = 64u;
            h[8] -= 65536u;
        }
    }
    if (h[8] >= 65536u) h[8] = 65535u;                   /* (every grain in the water: wait) */
    for (k = half * 16u; k < half * 16u + 16u; k++) {
        int32_t c, dist, ad, dir, v, mv, p, pe, landed = 0;
        if (!gm[k]) continue;
        c = ss_centre(x, gb[k]);
        dist = c - gx[k];
        ad = dist < 0 ? -dist : dist;
        dir = dist < 0 ? -1 : 1;
        v = (int32_t)x->u[3];
        if (ad < (16 << 16)) v = (int32_t)((((uint32_t)v >> 8) * ((uint32_t)(ad + 32768) >> 8)) >> 4);   /* settling */
        if (v < 2048) v = 2048;
        mv = v < ad ? v : ad;
        gp[k] = gx[k];
        p = gx[k] >> 16;
        pe = (gx[k] + dir * mv) >> 16;
        while (p != pe) {                                /* the bank's face (the next pixel in holds 200) stops it */
            const int32_t nx = p + dir;
            if (nx < 0 || nx >= (int32_t)n || d[nx] >= 200u) { landed = 1; break; }
            p = nx;
        }
        if (!landed) {
            gx[k] += dir * mv;
            landed = ad - mv <= 32768;
            p = gx[k] >> 16;
        }
        if (landed && p >= 0 && p < (int32_t)n) {
            const uint32_t at = (uint32_t)p, sum = d[at] + gm[k];
            const int32_t o = p - dir;
            if (d[at] < 128u) lc[at] = gc[k];
            d[at] = (uint8_t)(sum > 255u ? 255u : sum);
            if (sum > 255u && o >= 0 && o < (int32_t)n) {
                const uint32_t r2 = d[o] + sum - 255u;
                d[o] = (uint8_t)(r2 > 255u ? 255u : r2);
                lc[o] = gc[k];
            }
            if (h[16u + gb[k]] < 65535u) h[16u + gb[k]]++;
            gm[k] = 0u;
        }
    }
    for (k = 0u; half == 3u && k < banks; k++) {      /* slumps: a loaded bank spreads for 3 s */
        if (h[28u + k]) h[28u + k]--;
        else if (h[16u + k] >= x->u[6]) { h[28u + k] = 75u; h[16u + k] = 0u; }
    }
}

static uint32_t ss_slice(fxv_ctx_t *x, uint32_t quota)
{
    uint32_t *h = b5_h(x);
    const uint32_t n = x->n;
    (void)quota;
    if (x->job[0] == 0u) {                               /* a model step: grains (two halves), then the banks' sweep */
        if (x->job[1] == 1u) {
            uint8_t *d = ss_d(x), *lc = ss_lc(x);
            const uint32_t gen = h[H_GEN], lim = x->u[4], from = x->job[2], end = b5_min(n, from + 112u);
            uint32_t i;
            for (i = from; i < end; i++) {
                const uint32_t k = b5_min(x->u[1] - 1u, (i * x->u[5]) >> 16), th = h[28u + k] ? lim / 3u : lim;
                if (((gen + i) & 15u) == 0u && d[i]) d[i]--;                       /* dissolving */
                if (i + 1u < n) {
                    if (d[i] > d[i + 1u] + th) {
                        const uint32_t m = (d[i] - d[i + 1u] - th + 1u) >> 1;
                        d[i] = (uint8_t)(d[i] - m); d[i + 1u] = (uint8_t)(d[i + 1u] + m);
                        if (d[i + 1u] - m < 128u) lc[i + 1u] = lc[i];
                    } else if (d[i + 1u] > d[i] + th) {
                        const uint32_t m = (d[i + 1u] - d[i] - th + 1u) >> 1;
                        d[i + 1u] = (uint8_t)(d[i + 1u] - m); d[i] = (uint8_t)(d[i] + m);
                        if (d[i] - m < 128u) lc[i] = lc[i + 1u];
                    }
                }
            }
            x->job[2] = end;
            if (end >= n) x->job[1] = 0u;
            return 1u;
        }
        if (x->job[1] >= 2u) {                           /* quarters 1..3, then the sweep */
            ss_grains(x, x->job[1] - 1u);
            if (x->job[1] == 4u) { x->job[1] = 1u; x->job[2] = 0u; } else x->job[1]++;
            return 1u;
        }
        if (b5_next(x)) { ss_grains(x, 0u); x->job[1] = 2u; return 1u; }
        x->u[0] = b5_frac(x);
        x->job[0] = 1u;
        x->job[2] = 0u;
        return 1u;
    }
    if (x->job[0] == 1u) {                               /* clear the grain light and colour, 256 pixels a slice */
        uint16_t *gl = ss_gl(x);
        uint8_t *gk = ss_gk(x);
        const uint32_t end = b5_min(n, x->job[2] + 256u);
        uint32_t i;
        for (i = x->job[2]; i < end; i++) { gl[i] = 0u; gk[i] = 0u; }
        x->job[2] = end;
        if (end < n) return 1u;
        x->job[0] = 2u;
        x->job[2] = 0u;
        return 1u;
    }
    if (x->job[0] == 2u) {                               /* the grains at their glide positions, 32 a slice (64: ~7,850 cycles) */
        const int32_t *gx = ss_gx(x), *gp = ss_gp(x);
        const uint8_t *gc = ss_gc(x), *gm = ss_gm(x);
        const uint32_t f = x->u[0], k1 = x->job[2] + 32u;
        uint32_t k;
        for (k = x->job[2]; k < k1; k++)
            if (gm[k] && n) {
                const int32_t dd = gx[k] - gp[k], x8 = (gp[k] + (dd >> 8) * (int32_t)(f >> 8)) >> 8;
                sp_dot(ss_gl(x), ss_gk(x), n, x8, 40000u, gc[k]);
            }
        x->job[2] = k1;
        if (k1 < SS_G) return 1u;
        x->job[0] = 3u;
        return 1u;
    }
    return 0u;
}

static void ss_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *o)
{
    const uint8_t *d = ss_d(x), *lc = ss_lc(x), *gk = ss_gk(x);
    const uint16_t *gl = ss_gl(x);
    const fxv_px_t water = fxv_rgb16(x->c[1]);
    fxv_px_t glow = fxv_rgb16(x->c[2]);
    const uint32_t ts = x->et * 5u;
    uint32_t j;
    glow.q = 255u;
    glow.k = 50u;
    for (j = 0u; j < count; j++) {
        const uint32_t i = from + j, dv = d[i];
        fxv_px_t col = fxv_scale_px(water, 52000u + (((uint32_t)(fxv_sin16(i * 1500u + ts) + 32767)) >> 3));
        if (dv) {
            fxv_px_t layer = fxv_pal(x, (uint32_t)lc[i] << 8);
            if (dv > 200u) layer = fxv_blend16(layer, glow, (dv - 200u) * 600u);
            col = fxv_blend16(col, layer, fxv_smooth16(dv * 257u));
        }
        if (gl[i]) col = fxv_add_px(col, fxv_scale_px(fxv_blend16(fxv_pal(x, (uint32_t)gk[i] << 8), glow, 32768u), gl[i]));
        o[j] = col;
    }
}

const fxv_desc_t fxv_d_silt_settling = { ss_bytes, b5_init, ss_advance, ss_slice, ss_render, 0, 0, b5_digest };
