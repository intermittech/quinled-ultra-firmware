/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <string.h>
#include "math32.h"
#include "ultra.h"
#include "config.h"
#include "lto.h"
#include "../target.h"                   /* BOARD: IDENTIFY 7 on the receiver only */

#define RATE_NOMINAL (ULTRA_TICK_US << 16)
#define LIGHT_IDX(l) ((l) < ULTRA_LIGHTS ? (l) : ULTRA_LIGHTS)

/* ================================================================================================ math */
static uint32_t clz32(uint32_t x)
{
    uint32_t n = 0;
    if ((x & 0xFFFF0000u) == 0u) { n += 16u; x <<= 16; }
    if ((x & 0xFF000000u) == 0u) { n += 8u; x <<= 8; }
    if ((x & 0xF0000000u) == 0u) { n += 4u; x <<= 4; }
    if ((x & 0xC0000000u) == 0u) { n += 2u; x <<= 2; }
    if ((x & 0x80000000u) == 0u) { n += 1u; }
    return n;
}

NOINLINE uint16_t ultra_lerp16(uint32_t a, uint32_t b, uint32_t s)   /* (ultra.h) */
{
    int32_t d = (int32_t)b - (int32_t)a;
    return (uint16_t)((int32_t)a + (int32_t)m32_sar(d,s,16));
}

/* ramp_eval's own inline copy: its 11 lerps per render stay inline (through ultra_lerp16 the "5 lights fading" render
 * row was +400 cycles, +9 %); scene.c call ultra_lerp16 */
static ALWAYS_INLINE uint16_t lerp16(uint32_t a, uint32_t b, uint32_t s)
{
    int32_t d = (int32_t)b - (int32_t)a;
    return (uint16_t)((int32_t)a + (int32_t)m32_sar(d,s,16));
}

/* one Horner step of the polynomials in ultra_tables.c: acc = c + acc x t (t = [0, 1) as Q32, the product floored).
 * With coefficients >= 0 every step is monotonic in t. */
#define HORNER(acc, c, t) ((acc) = (c) + m32_hi((acc),(t)))

/* x^g for x = x_q32 / 2^32 = 2^(g log2 x): log2 of the mantissa and 2^ of the fraction as fixed-point polynomials
 * with non-negative coefficients (tools/gen_ultra_tables.py), so the curve is monotonic at every input and continuous
 * across powers of two. Within 1 count of the former 257-point tables at every 32-bit input, 0 and full scale
 * exact (test/test_gamma_tables.c). Result x^g x 65535 rounded (0..65535). */
uint32_t ultra_gamma(uint32_t x, uint32_t g_q16)
{
    const uint32_t *cl = ultra_log2_poly, *ce = ultra_exp2_poly;
    uint32_t n, f, q, fp, e, sh, y;
    uint32_t lo, hi, z, qh, ql;
    if (x == 0u) {
        return 0;
    }
    if (g_q16 == 65536u) {
        return m32_hi(x,65535u)+((x*65535u+0x80000000u)<x*65535u);
    }
    n = clz32(x);
    f = (x << n) << 1;                                   /* mantissa 1.f, f as Q32 */
    f = ~f;                                              /* u = 1 - .f: log2(1.f) = 1 - q(u) */
    q = cl[6];
    HORNER(q, cl[5], f);
    HORNER(q, cl[4], f);
    HORNER(q, cl[3], f);
    HORNER(q, cl[2], f);
    HORNER(q, cl[1], f);
    HORNER(q, cl[0], f);
    HORNER(q, 0u, f);                                    /* q(u), Q29, <= 1 (the coefficients sum to 1) */
    /* -log2(x)=n*2^31+4*q. Truncation of its negative product is
     * -floor((n*2^31+4*q)*g/2^16). Keep the magnitude in two words. */
    z=n*g_q16;lo=z<<15;hi=z>>17;
    qh=m32_hi(q,g_q16);ql=q*g_q16;
    z=(qh<<18)|(ql>>14);hi+=(qh>>14)+(lo+z<lo);lo+=z;
    sh=6u+(hi<<1)+(lo>>31)+((lo&0x7FFFFFFFu)!=0u);
    if(sh>=32u)return 0;
    fp=(0u-lo)<<1;
    e = ce[5];
    HORNER(e, ce[4], fp);
    HORNER(e, ce[3], fp);
    HORNER(e, ce[2], fp);
    HORNER(e, ce[1], fp);
    HORNER(e, 1u << 30, fp);                           /* 2^fp, Q30 (ce[0] = 1.0) */
    if (e > 0x80000000u) {
        e = 0x80000000u;
    }
    /* x^g = 2^fp * 2^k = e * 2^(k - 30), k <= 0; as Q24: e >> (6 - k), then x 65535 */
    y=m32_addshr(e>>sh,65535u,0x800000u,24);
    return y > 65535u ? 65535u : y;
}

/* product of two 16-bit fractions (0..65535 = 0..1) as a Q32 fraction, 65535 x 65535 -> ~1.0 */
static uint32_t q32_of(uint32_t a, uint32_t b)
{
    uint32_t x = a * b;
    return x + (x >> 15);
}

uint32_t ultra_smoothstep(uint32_t p)
{
    uint32_t x3,lo,hi,in,v,s;
    if(p>=65536u)return 65536u;
    x3=m32_shr(p*p,p,16);
    v=983040u-6u*p;lo=p*v;hi=10u-m32_hi(p,v)-(lo!=0u);
    in=(hi<<24)|((0u-lo)>>8);
    s=(m32_hi(x3,in)+128u)>>8;
    return s>65536u?65536u:s;

}

/* el / dur as Q16 (0..65536) without 64-bit division */
static uint32_t frac16(uint32_t el, uint32_t dur)
{
    if (el >= dur) {
        return 65536u;
    }
    while (dur >= 65536u) {
        el >>= 1;
        dur >>= 1;
    }
    return (el << 16) / dur;
}



static uint32_t sin16(uint32_t q)         /* sin(pi/2 * q / 65536) for q 0..65536, Q16 (quarter wave) */
{
    uint32_t idx = q >> 10, r = q & 0x3FFu;
    if (idx >= 64u) {
        return 65535u;
    }
    return ultra_sin_tab[idx] + (((uint32_t)(ultra_sin_tab[idx + 1] - ultra_sin_tab[idx]) * r) >> 10);
}

/* ultra_vals_t copy / compare: on GCC as its six words (4-byte aligned, ULTRA_ALIGN4). A struct copy at -Os is a
 * memcpy call (67 cycles against ~20 here). */
#if defined(__GNUC__)
typedef uint32_t __attribute__((may_alias)) vals_word_t;
typedef char vals_six_words[sizeof(ultra_vals_t) == 24u ? 1 : -1];
static NOINLINE void vals_cpy(ultra_vals_t *d, const ultra_vals_t *s)
{
    vals_word_t *dw = (vals_word_t *)d;
    const vals_word_t *sw = (const vals_word_t *)s;
    uint32_t a = sw[0], b = sw[1], c = sw[2];
    dw[0] = a;
    dw[1] = b;
    dw[2] = c;
    a = sw[3];
    b = sw[4];
    c = sw[5];
    dw[3] = a;
    dw[4] = b;
    dw[5] = c;
}

static int vals_eq(const ultra_vals_t *x, const ultra_vals_t *y)   /* the 22 bytes of values (little endian) */
{
    const vals_word_t *a = (const vals_word_t *)x, *b = (const vals_word_t *)y;
    return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | (a[2] ^ b[2]) | (a[3] ^ b[3]) | (a[4] ^ b[4]) |
            ((a[5] ^ b[5]) & 0xFFFFu)) == 0u;
}
#else
static void vals_cpy(ultra_vals_t *d, const ultra_vals_t *s)
{
    *d = *s;
}

static int vals_eq(const ultra_vals_t *x, const ultra_vals_t *y)
{
    uint32_t i;
    for (i = 0; i < 5u; i++) {
        if (x->a[i] != y->a[i] || x->b[i] != y->b[i]) {
            return 0;
        }
    }
    return x->m == y->m;
}
#endif

/* ================================================================================================ clock */
/* Before the first header the engine runs on its own clock (no controller): the same line
 * from ref (tick 0, time 0 at the init) at the nominal rate. ultra_clock_tend moves ref along, so the time stays
 * continuous across the tick's wrap (2.5 days) and a tick difference never leaves the int32 range. */
uint64_t ultra_host_us(const ultra_t *u, uint32_t tick)
{
    const ultra_clock_t *c = &u->clk;
    int32_t dt = (int32_t)(tick - c->ref_tick);
    if (dt >= 0 || !c->synced) {
        return c->ref_us + (((uint64_t)(uint32_t)dt * c->rate_q16) >> 16);
    }
    return c->ref_us - (((uint64_t)(uint32_t)(-dt) * c->rate_q16) >> 16);
}

void ultra_clock_tend(ultra_t *u, uint32_t tick)
{
    ultra_clock_t *c = &u->clk;
    if (tick - c->ref_tick >= ULTRA_TEND_TICKS) {
        c->ref_us = ultra_host_us(u, tick);
        c->ref_tick = tick;
        c->gap = 1;                                      /* the next header restarts the rate interval */
    }
}

/* host us / 1000 as 32 bits: one 32-bit division while the host time fits (its first 71.6 min), then the long
 * division by 1000 in three 16-bit digits (each partial dividend < 1000 x 2^16; the top digit's quotient is a multiple
 * of 2^32 and drops out), instead of the 64 / 32 division */
static uint32_t host_ms(const ultra_t *u, uint32_t tick)
{
    uint64_t us = ultra_host_us(u, tick);
    uint32_t hi = (uint32_t)(us >> 32), lo = (uint32_t)us, r, q;
    if (hi == 0u) {
        return lo / 1000u;
    }
    r = hi % 1000u;
    r = (r << 16) | (lo >> 16);
    q = r / 1000u;
    r = ((r - q * 1000u) << 16) | (lo & 0xFFFFu);
    return (q << 16) + r / 1000u;
}

/* the host clock jumped (first header, host reboot): keep every running transition where it is */
static void rebase(ultra_t *u, uint32_t delta_ms)
{
    uint32_t i, j;
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        ultra_light_t *L = &u->light[i];
        L->cur.t0 += delta_ms;
        L->prev.t0 += delta_ms;
    }
    for (i = 0; i <= ULTRA_LIGHTS; i++) {               /* a rebooted host restarts its sequence numbers */
        for (j = 0; j < ULTRA_LIGHT_TYPES; j++) {
            u->last_seq[i][j] = 0x100u;
        }
    }
}

MEASURED void ultra_on_header(ultra_t *u, uint32_t tick, uint32_t host_us)
{
    ultra_clock_t *c = &u->clk;
    uint64_t h, est;
    int64_t err;
    uint32_t em, tm;
    int32_t dm;
    /* unwrap TIME (modulo ULTRA_TIME_WRAP = 2^22 ms: wraps every 69.9 min, on a whole ms) around the board's own
     * estimate, which runs on without headers: after a gap of hours or days (the host gone) the header lands where the
     * estimate is, within +-34.95 min (not around the last header: a gap over 34.95 min would look like a host reboot:
     * times rebased, commands taken again). In ms and us: TIME's ms (< 2^22) minus the estimate's, sign-extended from
     * 22 bits, so h is the host's clock modulo whole wraps and frame_ms = floor(h / 1000) is on the host's ms grid,
     * whatever epoch this board first synchronized in: every board adds a START to the same host ms */
    est = ultra_host_us(u, tick);
    em = m32_div((uint32_t)(est>>32),(uint32_t)est,1000u);          /* the estimate's host ms (low 32 bits) */
    tm = host_us / 1000u;
    dm = c->synced ? (int32_t)((tm - em) << 10) >> 10 : (int32_t)(tm - em);
    u->frame_ms = em + (uint32_t)dm;
    err = (int32_t)((uint32_t)dm * 1000u + (host_us - tm * 1000u) - ((uint32_t)est - em * 1000u));   /* (exact when synced) */
    h = c->synced ? est + (uint64_t)err : host_us;
    if (!c->synced || err > 200000 || err < -200000) {   /* first header / host rebooted / long gap */
        rebase(u, (uint32_t)dm);
        c->synced = 1;
        c->ref_tick = c->last_tick = tick;
        c->ref_us = c->last_us = h;
        return;
    }
    if (c->gap) {                                        /* the interval spans a stop of the local tick */
        c->gap = 0;
        c->last_tick = tick;
        c->last_us = h;
    }
    if ((uint32_t)(tick - c->last_tick) >= MCU_TICKS_US(500000u)) {     /* >= 0.5 s: rate from the host clock */
        uint32_t dt = tick - c->last_tick;
        uint64_t dh = h - c->last_us;
        uint32_t meas = m32_div((uint32_t)(dh>>16),(uint32_t)dh<<16,dt);
        if (meas > RATE_NOMINAL + RATE_NOMINAL / 16u) meas = RATE_NOMINAL + RATE_NOMINAL / 16u;
        if (meas < RATE_NOMINAL - RATE_NOMINAL / 16u) meas = RATE_NOMINAL - RATE_NOMINAL / 16u;
        /* the first measurement as is (fast lock after boot), then averaged */
        c->rate_q16 = c->rated ? (uint32_t)((int32_t)c->rate_q16 + ((int32_t)meas - (int32_t)c->rate_q16) / 4) : meas;
        c->rated = 1;
        c->last_tick = tick;
        c->last_us = h;
        est = ultra_host_us(u, tick);
        err = (int64_t)(h - est);
    }
    /* phase: a quarter of the error per header, so transitions never jump */
    c->ref_us = (uint64_t)((int64_t)est + err / 4);
    c->ref_tick = tick;
}

void ultra_clock_gap(ultra_t *u)
{
    u->clk.gap = 1;
}

/* ================================================================================================ state */
MEASURED void ultra_init(ultra_t *u, uint32_t seed)
{
    uint32_t i, j;
    uint8_t *b = (uint8_t *)u;
    (void)seed;
    for (i = 0; i < sizeof(*u); i++) {
        b[i] = 0;
    }
    for (i = 0; i <= ULTRA_LIGHTS; i++) {
        for (j = 0; j < ULTRA_LIGHT_TYPES; j++) {
            u->last_seq[i][j] = 0x100u;
        }
    }
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        u->light[i].g_q16 = 183501u;                      /* 2.8 until LIGHT_DEF */
    }
    u->cfg[ULTRA_CFG_LOSS_TIMEOUT] = 30u;                 /* 3 s */
    u->clk.rate_q16 = RATE_NOMINAL;                       /* its own clock until the first header */
}

static void ramp_eval(const ultra_ramp_t *r, uint32_t now, ultra_vals_t *o)
{
    int32_t el = (int32_t)(now - r->t0);
    uint32_t p, s, i;
    if (el < 0) {
        vals_cpy(o, &r->from);
        return;
    }
    p = r->dur ? frac16((uint32_t)el, r->dur) : 65536u;
    if (p == 65536u) {                                   /* done (a settled light): exactly `to`, as ultra_lerp16 at 1 */
        vals_cpy(o, &r->to);
        return;
    }
    if (!r->via_off) {
        s = r->smooth ? ultra_smoothstep(p) : p;
        o->m = lerp16(r->from.m, r->to.m, s);
        for (i = 0; i < 5u; i++) {
            o->a[i] = lerp16(r->from.a[i], r->to.a[i], s);
            o->b[i] = lerp16(r->from.b[i], r->to.b[i], s);
        }
        return;
    }
    if (p < 32768u) {                                    /* first half: old values down to 0 */
        s = r->smooth ? ultra_smoothstep(p * 2u) : p * 2u;
        vals_cpy(o, &r->from);
        o->m = lerp16(r->from.m, 0, s);
    } else {                                             /* second half: new values up from 0 */
        s = r->smooth ? ultra_smoothstep(p * 2u - 65536u) : p * 2u - 65536u;
        vals_cpy(o, &r->to);
        o->m = lerp16(0, r->to.m, s);
    }
}

static void base_at(const ultra_light_t *L, uint32_t now, ultra_vals_t *o)
{
    ramp_eval((int32_t)(now - L->cur.t0) < 0 ? &L->prev : &L->cur, now, o);
}

/* a finished ramp becomes a constant at `now`, so its start time never ages past the int32 range. Once settled (every
 * render after that, until a writer of cur / prev clears `settled`), only the times move: the copies would not change
 * anything else. */
static void settle(ultra_light_t *L, uint32_t now)
{
    int32_t el = (int32_t)(now - L->cur.t0);
    if (el >= 0 && (uint32_t)el >= L->cur.dur) {
        if (!L->settled) {
            vals_cpy(&L->cur.from, &L->cur.to);
            L->cur.dur = 0;
            L->cur.via_off = 0;
            L->prev = L->cur;
            L->settled = 1;
        }
        L->cur.t0 = now;
        L->prev.t0 = now;
    }
}

static void start_ramp(ultra_light_t *L, const ultra_vals_t *to, uint32_t t0, uint32_t dur, uint8_t flags)
{
    ultra_vals_t from;
    uint32_t i;
    L->settled = 0;
    base_at(L, t0, &from);
    if ((int32_t)(t0 - L->cur.t0) >= 0) {
        L->prev = L->cur;
    }
    L->cur.to = *to;
    if (from.m == 0u) {                                  /* dark: colors jump (ESPHome off -> on) */
        for (i = 0; i < 5u; i++) {
            from.a[i] = to->a[i];
            from.b[i] = to->b[i];
        }
    } else if (to->m == 0u) {                            /* on -> off: only M falls, the colours stay (ESPHome) */
        for (i = 0; i < 5u; i++) {
            L->cur.to.a[i] = from.a[i];
            L->cur.to.b[i] = from.b[i];
        }
    }
    L->cur.from = from;
    L->cur.t0 = t0;
    L->cur.dur = dur;
    L->cur.smooth = (uint8_t)((flags & ULTRA_SET_SMOOTH) != 0u);
    L->cur.via_off = (uint8_t)((flags & ULTRA_SET_VIA_OFF) != 0u && from.m != 0u);
}

uint32_t ultra_rising(const ultra_t *u, uint32_t tick)
{
    uint32_t i, now = host_ms(u, tick);
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        const ultra_light_t *L = &u->light[i];
        int32_t el = (int32_t)(now - L->cur.t0);
        if (L->defined && (el < 0 || (uint32_t)el < L->cur.dur) && L->cur.to.m > L->last.m) {
            return 1;
        }
    }
    return 0;
}

void ultra_all_off(ultra_t *u, uint32_t tick, uint32_t dur_ms)
{
    uint32_t i, now = host_ms(u, tick);
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        ultra_light_t *L = &u->light[i];
        ultra_vals_t to;
        if (!L->defined) {
            continue;
        }
        base_at(L, now, &to);
        to.m = 0;
        start_ramp(L, &to, now, dur_ms, ULTRA_SET_SMOOTH);
        /* The unchanged host SET must apply again after signal loss.
         * Calibration, LIGHT_DEF and CONFIG remain deduplicated. */
        u->last_seq[i][ULTRA_SET] = 0x100u;
    }
}

/* ================================================================================================ slots */
/* LIGHT_OPT set points: the valid ones (M, mireds != 0), sorted by M, descending; the warm end at least the warmest */
static void opt_points(ultra_light_t *L, const uint8_t *p)
{
    uint32_t i, j;
    L->opt_n = 0;
    if (!(p[0] & ULTRA_OPT_POINTS)) {
        return;
    }
    for (i = 0; i < ULTRA_OPT_POINTS_MAX; i++) {
        uint16_t pm = (uint16_t)ultra_get16(p + 9 + 4 * i), pt = (uint16_t)ultra_get16(p + 11 + 4 * i);
        if (pm == 0u || pt == 0u) {
            continue;
        }
        for (j = L->opt_n; j > 0u && L->opt_pm[j - 1u] < pm; j--) {
            L->opt_pm[j] = L->opt_pm[j - 1u];
            L->opt_pt[j] = L->opt_pt[j - 1u];
        }
        L->opt_pm[j] = pm;
        L->opt_pt[j] = pt;
        L->opt_n++;
        if (pt > L->opt_warm) {
            L->opt_warm = pt;
        }
    }
}

uint32_t ultra_on_slot(ultra_t *u, const uint8_t *s, uint32_t tick)
{
    return ultra_slot_check(s) != 0 ? 0u : ultra_apply_slot(u, s, tick);
}

MEASURED uint32_t ultra_apply_slot(ultra_t *u, const uint8_t *s, uint32_t tick)
{
    uint8_t type = s[0] & ULTRA_TYPE_MASK, light = s[1], flags = s[0] & ULTRA_TYPE_FLAGS, seq = s[2];
    const uint8_t *p = s + ULTRA_PAYLOAD;
    uint16_t *last;
    ultra_light_t *L;
    uint32_t i;
    (void)tick;

    /* compatibility rules (ultra_protocol.h): what this firmware doesn't know is ignored, the state stays untouched */
    if ((s[0] & ULTRA_TYPE_BCAST) || type == ULTRA_NOP || type == ULTRA_POLL || type == 0x03u || type == 0x04u || type == 0x0Au || type >= ULTRA_LIGHT_TYPES) {
        /* POLL: reserved for the return channel, not implemented (frame_rx also skips RW / EXT / UPD frames);
         * 0x0A-0x7F later commands; 0x80-0xFF broadcast slots, never a board's own slot (a wrong board count) */
        return 0;
    }
    if ((type == ULTRA_IDENTIFY && light != ULTRA_TARGET_BOARD) ||
        (light >= ULTRA_LIGHTS && light != ULTRA_TARGET_BOARD)) {
        return 0;                                        /* LIGHT 5-14: reserved */
    }
    if ((type == ULTRA_IDENTIFY && ((p[0] > ULTRA_IDENT_RELEASE && p[0] != ULTRA_IDENT_VERSION &&
                                     (BOARD != BOARD_UR_ANALOG_M || p[0] != ULTRA_IDENT_POWER_OFF)) ||
                                    (p[0] == ULTRA_IDENT_CAL && p[1] != 0u &&
                                     (p[1] < ULTRA_IDENT_CAL_MIN || p[1] > ULTRA_IDENT_CAL_MAX)))) ||
        (type == ULTRA_SCENE && p[0] > 1u)) {
        return 0;                                        /* an IDENTIFY / SCENE of later firmware */
    }
    if (type == ULTRA_LIGHT_DEF) {
        for (i = 0; i < 5u; ++i) {
            uint32_t role = (p[i >> 1] >> ((i & 1u) ? 0 : 4)) & 15u;
            if (role && role <= ULTRA_ROLE_WARM_WHITE && (uint32_t)p[26] + i >= BOARD_CHANNELS)
                return 0;             /* reject atomically, including duplicate state */
        }
    }
    last = &u->last_seq[LIGHT_IDX(light)][type];
    if (*last == seq) {
        return 0;                                        /* repeat / refresh of a command already applied */
    }
    *last = seq;

    switch (type) {
    case ULTRA_CONFIG: {
        uint32_t ev = 0;
        for (i = 0; i < 9u; i++) {
            uint8_t id = p[3 * i];
            uint16_t v = (uint16_t)ultra_get16(p + 3 * i + 1);
            if (light == ULTRA_TARGET_BOARD) {
                if (id == ULTRA_CFG_DOMAIN_FIRST + 1u) id = ULTRA_CFG_SLOT_OUTPUT_MAX;
                else if (!(id >= 1u && id <= 10u) && !(id >= 21u && id <= 24u)) continue;
            } else if (light < BOARD_CHANNELS) {
                if (id == ULTRA_CFG_CH_CURRENT) id = ULTRA_CFG_SLOT_CURRENT + light;
                else if (id == ULTRA_CFG_CH_MAX) id = ULTRA_CFG_SLOT_MAX + light;
                else continue;
            } else continue;
            if (v == ULTRA_LIMIT_UNKNOWN && ((id >= ULTRA_CFG_SLOT_CURRENT && id <= ULTRA_CFG_BOARD_MAX) ||
                                             id == ULTRA_CFG_SLOT_OUTPUT_MAX)) continue;
            if (id == ULTRA_CFG_PWM_PERIOD) v = (uint16_t)mcu_pwm_units_10ns(v);
            /* Reuse the console/overlay bounds; CRC records accepted values only. */
            if (id <= ULTRA_CFG_FEEDBACK && id != ULTRA_CFG_PWM_PERIOD && id != ULTRA_CFG_LOSS_TIMEOUT) {
                uint32_t k;
                for (k = 1; k < SETD_N; ++k) {
                    const setdesc_t *t = &settings_desc[k];
                    if ((id == ULTRA_CFG_PWM_PRESET && k == 1u) || t->cfg == id) {
                        if (v < t->lo || v > t->hi) break;
                        goto config_valid;
                    }
                }
                continue;
            }
            if ((id == ULTRA_CFG_LOSS_TIMEOUT && (v < 5u || v > 600u)) ||
                (id == ULTRA_CFG_BOOT_PROTOCOL && v > PMODE_ULTRA) ||
                (id == ULTRA_CFG_SOFT_START && v > 1u)) continue;
config_valid:
            if (id >= ULTRA_CFG_SLOT_MAX && id < ULTRA_CFG_SLOT_MAX + BOARD_CHANNELS)
                v = !v || v > BOARD_CHANNEL_RATING_MA ? BOARD_CHANNEL_RATING_MA : v;
            if (id == ULTRA_CFG_BOARD_MAX) v = !v || v > ISENSE_BOARD_MA ? ISENSE_BOARD_MA : v;
            if (id == ULTRA_CFG_SLOT_OUTPUT_MAX) v = !v || v > BOARD_OUTPUT_RATING_MA ? BOARD_OUTPUT_RATING_MA : v;
            if (id == ULTRA_CFG_RETRY && !(v >> 8)) v |= 0x100u;
            if (!(u->cfg_valid & (1u << id)) || u->cfg[id] != v) {
                u->cfg[id] = v;
                u->cfg_changed |= 1u << id;
                ev = ULTRA_EV_CONFIG;
            }
            u->cfg_valid |= 1u << id;
        }
        return ev;
    }
    case ULTRA_IDENTIFY:
        u->identify = p[0];
        u->identify_arg = p[1];
        return ULTRA_EV_IDENTIFY;
    default:
        break;
    }
    if (light >= ULTRA_LIGHTS) {
        return 0;
    }
    L = &u->light[light];
    L->out_ok = 0;                                       /* (its head may change: the render cache) */
    switch (type) {
    case ULTRA_LIGHT_OPT:
        L->opt_flags = (uint8_t)(p[0] & ULTRA_OPT_FLAGS);   /* unknown bits: ignored */
        L->opt_cw = (uint16_t)ultra_get16(p + 1);
        L->opt_ww = (uint16_t)ultra_get16(p + 3);
        L->opt_warm = (uint16_t)ultra_get16(p + 5);
        L->opt_assist = (uint16_t)ultra_get16(p + 7);
        opt_points(L, p);
        return ULTRA_EV_LIGHT | ULTRA_EV_DEF;
    case ULTRA_SCENE: {
        uint32_t slot = p[0];
        L->scene_have |= (uint8_t)(1u << slot);
        L->scene_fade[slot] = p[1];
        L->scene_v[slot].m = (uint16_t)ultra_get16(p + 2);
        for (i = 0; i < 5u; ++i) {
            L->scene_v[slot].a[i] = (uint16_t)ultra_get16(p + 4 + 2 * i);
            L->scene_v[slot].b[i] = (uint16_t)ultra_get16(p + 14 + 2 * i);
        }
        return ULTRA_EV_SCENE;
    }
    case ULTRA_LIGHT_DEF:
        L->base_channel = p[26];
        L->defined = 1;
        L->mask = 0;
        for (i = 0; i < 5u; i++) {
            L->role[i] = (uint8_t)((p[i >> 1] >> ((i & 1u) ? 0 : 4)) & 0x0Fu);
            if (L->role[i] > ULTRA_ROLE_WARM_WHITE) {
                L->role[i] = ULTRA_ROLE_NONE;
            }
            if (L->role[i] != ULTRA_ROLE_NONE) {
                L->mask |= (uint8_t)(1u << i);
            }
            L->min_p[i] = (uint16_t)ultra_get16(p + 6 + 2 * i);
            L->max_p[i] = (uint16_t)ultra_get16(p + 16 + 2 * i);
        }
        L->g_q16 = (ultra_get16(p + 3) * 65536u + 500u) / 1000u;
        if (L->g_q16 == 0u) {
            L->g_q16 = 65536u;                           /* gamma 0: linear, as ESPHome's gamma_correct: 0 (x^0 would
                                                            show every level that is on at 100 %) */
        }
        L->flags = (uint8_t)(p[5] & ULTRA_DEF_FLAGS);
        return ULTRA_EV_LIGHT | ULTRA_EV_DEF;
    case ULTRA_SET: {
        ultra_vals_t to;
        to.m = (uint16_t)ultra_get16(p);
        for (i = 0; i < 5u; i++) {
            to.a[i] = (uint16_t)ultra_get16(p + 2 + 2 * i);
            to.b[i] = (uint16_t)ultra_get16(p + 12 + 2 * i);
        }
        start_ramp(L, &to, u->frame_ms + (uint32_t)(int32_t)(int16_t)ultra_get16(p + 25), ultra_get24(p + 22), flags);
        return ULTRA_EV_LIGHT | ULTRA_EV_STATE;
    }
    default:
        break;
    }
    return ULTRA_EV_LIGHT;
}

uint32_t ultra_breathe(uint32_t ph)
{
    uint32_t s = sin16(ph <= 32768u ? ph * 2u : (65536u - ph) * 2u);   /* sin(pi ph) */
    return (s * s) >> 16;
}

static int is_rgb(uint8_t r) { return r >= ULTRA_ROLE_RED && r <= ULTRA_ROLE_BLUE; }

static uint32_t g16(const ultra_light_t *L, uint32_t v16)
{
    return ultra_gamma(q32_of(v16, 65535u), L->g_q16);
}

/* M x A x B as a Q32 fraction (65535 x 65535 x 65535 -> ~1.0) */
static uint32_t q32_of3(uint32_t m, uint32_t a, uint32_t b)
{
    return m32_shr(q32_of(m,a),b+(b>>15),16);
}

/* Set points (sorted by M, descending): linear in mireds between them, the light's own temperature (own_q) at M = 1
 * unless a point is at 100 %, the lowest point's temperature below it. Mireds x 10, Q16, all below 2^32 (own_q <=
 * ww_q, a point's pt << 16). The interpolation pq + (hq - pq) (m - pm) / (hm - pm) in 32 bits (it
 * was the image's only signed 64-bit division): m <= hm, so n = m - pm <= q = hm - pm <= 65534 and the quotient is at most
 * |hq - pq|; with a = |hq - pq| / q, b = |hq - pq| mod q it is a n + (b n) / q exactly, a n <= |hq - pq| and
 * b n < q^2; the sign goes on the magnitude, as the 64-bit division truncated towards zero (test/test_r9_math.c). */
static uint32_t dtw_points(const ultra_light_t *L, uint32_t m, uint32_t own_q)
{
    uint32_t i, hm = 65535u, hq = own_q;
    for (i = 0; i < L->opt_n; i++) {
        uint32_t pm = L->opt_pm[i], pq = (uint32_t)L->opt_pt[i] << 16;
        if (m >= pm) {
            uint32_t q = hm - pm, n = m - pm, d, a, x;
            if (q == 0u) {
                return pq;
            }
            d = hq > pq ? hq - pq : pq - hq;
            a = d / q;
            x = a * n + ((d - a * q) * n) / q;
            return hq > pq ? pq + x : pq - x;
        }
        hm = pm;
        hq = pq;
    }
    return hq;
}

/* Dim to warm (LIGHT_OPT, see ultra_protocol.h): the CW / WW mix of y[] follows M, red assist below the WW LED.
 * gcw / gww: the light's own gamma'd CW / WW values; on[]: the channel is meant to be on. */
static void dim_to_warm(const ultra_light_t *L, uint32_t m, uint32_t cw_ch, uint32_t ww_ch, uint32_t gcw, uint32_t gww,
                        uint32_t *y, uint8_t *on)
{
    uint32_t sum = gcw + gww, mx = gcw > gww ? gcw : gww, cwm = L->opt_cw, wwm = L->opt_ww, warm = L->opt_warm;
    uint32_t t = 0, scale, fw, fc, ch, r_ch = 5, g_ch = 5, rgb_on = 0;
    uint32_t cw_q, ww_q, warm_q, top_q, tgt_q;            /* mireds x 10, Q16: all below 2^32 (16-bit mireds) */
    if (sum == 0u || wwm <= cwm) {
        return;                                          /* white off, or no colour temperatures */
    }
    cw_q = cwm << 16;
    ww_q = wwm << 16;
    warm_q = warm << 16;
    top_q = cw_q + (wwm - cwm) * ((gww << 16) / sum);    /* the light's own temperature: cw_q <= top_q <= ww_q
                                                            (gww <= sum: the share <= 65536) */
    tgt_q = top_q;
    if (L->opt_flags & ULTRA_OPT_POINTS) {
        uint32_t pq = dtw_points(L, m, top_q);
        if (pq > top_q) {
            tgt_q = pq;                                  /* never colder than the light's own temperature */
        }
    } else if (warm_q > top_q) {
        /* + (warm_q - top_q) x (65535 - m) / 65535, linear in mireds, M 1 -> 0. With d = a x 65535 + b the product
         * over 65535 is a x w + b x w / 65535 exactly, each in 32 bits (a <= 65536, b < 65535, w <= 65535): no
         * 64-bit division */
        uint32_t d = warm_q - top_q, w = 65535u - m, a = d / 65535u, b = d - a * 65535u;
        tgt_q += a * w + (b * w) / 65535u;
    }
    /* the dividends below: tgt_q - cw_q <= (wwm - cwm) << 16 and tgt_q - ww_q <= (warm - wwm) << 16
     * (cw_q <= top_q <= tgt_q <= warm_q), scale <= 65535 and fc, fw <= 65536 */
    if (tgt_q <= ww_q) {
        fw = (tgt_q - cw_q) / (wwm - cwm);               /* WW share, Q16 */
    } else {
        fw = 65536u;                                     /* warmer than the WW LED: all WW, red assist */
        t = (tgt_q - ww_q) / (warm - wwm);
    }
    fc = 65536u - fw;
    scale = (g16(L, m) * mx + 32767u) / 65535u;          /* g(M) x max(gcw, gww) */
    if (L->flags & ULTRA_DEF_CONSTANT_BRIGHTNESS) {
        y[cw_ch] = (scale * fc + 32768u) >> 16;          /* CW + WW = g(M) x max */
        y[ww_ch] = (scale * fw + 32768u) >> 16;
    } else {
        uint32_t fmax = fw > fc ? fw : fc;               /* the brighter channel stays at g(M) x max */
        y[cw_ch] = (scale * fc + fmax / 2u) / fmax;
        y[ww_ch] = (scale * fw + fmax / 2u) / fmax;
    }
    on[cw_ch] = (uint8_t)(scale != 0u && fc != 0u);
    on[ww_ch] = (uint8_t)(scale != 0u && fw != 0u);
    if (t == 0u || L->opt_assist == 0u) {
        return;
    }
    for (ch = 0; ch < 5u; ch++) {
        if (L->role[ch] == ULTRA_ROLE_RED) r_ch = ch;
        if (L->role[ch] == ULTRA_ROLE_GREEN) g_ch = ch;
        if (is_rgb(L->role[ch]) && on[ch]) rgb_on = 1;
    }
    if (r_ch < 5u && !rgb_on) {                          /* red assist: only while the light's RGB part is off */
        uint32_t r = (((y[ww_ch] * t) >> 16) * L->opt_assist + 32767u) / 65535u;   /* (y, t <= 65536: fits) */
        y[r_ch] = r;
        on[r_ch] = (uint8_t)(r != 0u);
        if (g_ch < 5u) {
            y[g_ch] = r / 4u;                            /* a touch of green keeps it on the warm-white line */
            on[g_ch] = (uint8_t)(r / 4u != 0u);
        }
    }
}

/* outputs of one light's channels (res[ch] for the channels with a role) for values v */
static void light_out(const ultra_light_t *L, const ultra_vals_t *vp, uint32_t *res)
{
    const ultra_vals_t *v = vp;
    const uint32_t cb = (L->flags & ULTRA_DEF_CONSTANT_BRIGHTNESS) != 0u, dtw = (L->opt_flags & ULTRA_OPT_DIM_TO_WARM) != 0u;
    uint32_t gcw = 0, gww = 0, cw_ch = 5, ww_ch = 5, ch, y[5];
    uint8_t on[5];
    if (cb || dtw) {
        for (ch = 0; ch < 5u; ch++) {
            if (L->role[ch] == ULTRA_ROLE_COLD_WHITE) { cw_ch = ch; gcw = ultra_gamma(q32_of(v->a[ch], v->b[ch]), L->g_q16); }
            if (L->role[ch] == ULTRA_ROLE_WARM_WHITE) { ww_ch = ch; gww = ultra_gamma(q32_of(v->a[ch], v->b[ch]), L->g_q16); }
        }
    }
    for (ch = 0; ch < 5u; ch++) {
        uint32_t x;
        y[ch] = 0;
        on[ch] = 0;
        if (!(L->mask & (1u << ch))) {
            continue;
        }
        if (cb && (ch == cw_ch || ch == ww_ch)) {
            /* ESPHome constant_brightness: g(M) x max(gcw, gww) x g(this) / (gcw + gww); every factor and partial
             * product is at most 65535 x 65535 (mine <= sum): 32 bits */
            uint32_t sum = gcw + gww, mxg = gcw > gww ? gcw : gww, mine = ch == cw_ch ? gcw : gww;
            y[ch] = sum ? (g16(L, v->m) * mxg / 65535u) * mine / sum : 0u;
            x = (uint32_t)v->m * mine;
        } else {
            x = L->role[ch] == ULTRA_ROLE_BRIGHTNESS ? q32_of(v->m, 65535u) : q32_of3(v->m, v->a[ch], v->b[ch]);
            y[ch] = ultra_gamma(x, L->g_q16);
        }
        on[ch] = (uint8_t)(x != 0u);
    }
    if (dtw && cw_ch < 5u && ww_ch < 5u) {
        dim_to_warm(L, v->m, cw_ch, ww_ch, gcw, gww, y, on);
    }
    for (ch = 0; ch < 5u; ch++) {
        uint32_t yc = y[ch];
        if (!(L->mask & (1u << ch))) {
            continue;
        }
        if (on[ch] && yc == 0u) {
            yc = 1;                                      /* on stays on (like ESPHome's gamma table) */
        }
        if (yc != 0u || !(L->flags & ULTRA_DEF_ZERO_MEANS_ZERO)) {
            /* min / max power: yc <= 65535 here (the gamma, the constant-brightness share, dim to warm's values and
             * the red assist all are), so yc x (mx - mn) + 32767 fits 32 bits: a division by a constant, no 64-bit one */
            uint32_t mn = L->min_p[ch], mx = L->max_p[ch];
            yc = mn + (yc * (mx >= mn ? mx - mn : 0u) + 32767u) / 65535u;
        }
        res[L->base_channel + ch] = yc > 65535u ? 65535u : yc;
    }
}

/* power limit per channel (CONFIG 16-20), from the LED current of each channel at 100 % (CONFIG 11-15). Returns 1
 * when a channel was capped. The board limit (CONFIG 21) is the current sense's (main.c, core/isense.h). */
static uint8_t power_limit(const ultra_t *u, uint32_t *res)
{
    uint32_t ch;
    uint64_t sum = 0;
    uint32_t ceiling = u->cfg[ULTRA_CFG_SLOT_OUTPUT_MAX];
    uint8_t f = 0;
    if (!ceiling) ceiling = BOARD_OUTPUT_RATING_MA;
    for (ch = 0; ch < BOARD_CHANNELS; ch++) {
        uint32_t ia = u->cfg[ULTRA_CFG_SLOT_CURRENT + ch], imax = u->cfg[ULTRA_CFG_SLOT_MAX + ch];
        if (ia == 0u) continue;
        if (!imax) imax = BOARD_CHANNEL_RATING_MA;
        if (res[ch] * ia > imax * 65535u) {
            res[ch] = imax * 65535u / ia;
            f = 1u;
        }
        sum += res[ch] * ia; /* each product fits u32; the sum needs u64 */
    }
    if (sum > (uint64_t)ceiling * 65535u) {
        for (ch = 0; ch < BOARD_CHANNELS; ++ch)
            res[ch] = m32_divwide(m32_hi(res[ch],ceiling*65535u),res[ch]*(ceiling*65535u),(uint32_t)(sum>>32),(uint32_t)sum);
        f = 1u;
    }
    return f;
}

MEASURED uint8_t ultra_render(ultra_t *u, uint32_t tick, uint16_t *out)
{
    uint32_t now = host_ms(u, tick), res[5] = { 0, 0, 0, 0, 0 }, l, ch, act = 0;
    uint8_t changed = 0;

    for (l = 0; l < ULTRA_LIGHTS; l++) {                 /* the lights to render (defined) */
        if (u->light[l].defined) {
            act |= 1u << l;
        }
    }
    for (l = 0; act != 0u; l++, act >>= 1) {
        ultra_light_t *L = &u->light[l];
        ultra_vals_t v;
        if (!(act & 1u)) {
            continue;
        }
        settle(L, now);
        base_at(L, now, &v);
        if (!L->out_ok || !vals_eq(&v, &L->last)) {
            vals_cpy(&L->last, &v);
            light_out(L, &v, res);
            for (ch = 0; ch < 5u; ch++) {
                L->out_y[ch] = (L->mask & (1u << ch)) ? (uint16_t)res[L->base_channel + ch] : 0u;        /* (the light's channels: <= 65535) */
            }
            L->out_ok = 1;
        } else {
            for (ch = 0; ch < 5u; ch++) {
                if (L->mask & (1u << ch)) {
                    res[L->base_channel + ch] = L->out_y[ch];              /* the same values as last time: the same outputs */
                }
            }
        }
    }
    u->ch_limited = power_limit(u, res);

    for (ch = 0; ch < 5u; ch++) {
        if (u->out[ch] != res[ch]) {
            u->out[ch] = (uint16_t)res[ch];
            changed = 1;
        }
        out[ch] = u->out[ch];
    }
    return changed;
}

static void scene_levels(const ultra_t *u, const scene_t *stored, scene_t *out, uint32_t slot)
{
    uint32_t res[5] = { 0, 0, 0, 0, 0 }, l, ch;
    uint8_t mask = 0, keep = 0, fade = 0, have_fade = 0;
    for (l = 0; l < ULTRA_LIGHTS; l++) {
        const ultra_light_t *L = &u->light[l];
        if (!L->defined) {
            continue;
        }
        if (!(L->scene_have & (1u << slot))) {                            /* its SCENE is still to come: keep what is stored */
            keep |= (uint8_t)(ultra_channel_mask(L) & stored->mask);
            continue;
        }
        if (L->scene_have & (1u << slot)) {
            light_out(L, &L->scene_v[slot], res);
            mask |= ultra_channel_mask(L);
            if (!have_fade || L->scene_fade[slot] > fade) {
                fade = L->scene_fade[slot];
            }
            have_fade = 1;
        }
    }
    keep = (uint8_t)(keep & ~mask);
    for (ch = 0; ch < 5u; ch++) {
        if (keep & (1u << ch)) {
            res[ch] = stored->v[ch];
        } else if (!(mask & (1u << ch))) {
            res[ch] = 0;
        }
    }
    (void)power_limit(u, res);
    out->mask = (uint8_t)(mask | keep);
    for (ch = 0; ch < 5u; ch++) {
        out->v[ch] = (out->mask & (1u << ch)) ? (uint16_t)res[ch] : 0u;
    }
    out->fade = have_fade ? fade : stored->fade;
}

void ultra_scene(const ultra_t *u, const scene_t *stored, scene_t *out)
{
    scene_t loss = *stored, result;
    scene_levels(u, stored, out, 0);
    memcpy(loss.v, stored->loss_v, sizeof(loss.v));
    loss.mask = stored->loss_mask; loss.fade = stored->loss_fade;
    scene_levels(u, &loss, &result, 1);
    memcpy(out->loss_v, result.v, sizeof(out->loss_v));
    out->loss_mask = result.mask; out->loss_fade = result.fade;
}
