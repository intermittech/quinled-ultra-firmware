/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "ultra.h"

#define RATE_NOMINAL (ULTRA_TICK_US << 16)
#define LIGHT_IDX(l) ((l) < ULTRA_LIGHTS ? (l) : ULTRA_LIGHTS)
#define FX_REBASE_MS 0x40000000u          /* move long-running effect clocks forward (int32 / uint32 wrap) */

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

/* one Horner step of the polynomials in ultra_tables.c: acc = c + acc x t (t = [0, 1) as Q32, the product floored).
 * With coefficients >= 0 every step is monotonic in t. */
#define HORNER(acc, c, t) ((acc) = (c) + (uint32_t)(((uint64_t)(acc) * (t)) >> 32))

/* x^g for x = x_q32 / 2^32 = 2^(g log2 x): log2 of the mantissa and 2^ of the fraction as fixed-point polynomials
 * with non-negative coefficients (tools/gen_ultra_tables.py), so the curve is monotonic at every input and continuous
 * across powers of two. Within 1 count of the former 257-point tables at every 32-bit input, 0 and full scale
 * exact (test/test_gamma_tables.c). Result x^g x 65535 rounded (0..65535). */
uint32_t ultra_gamma(uint32_t x, uint32_t g_q16)
{
    const uint32_t *cl = ultra_log2_poly, *ce = ultra_exp2_poly;
    uint32_t n, f, q, fp, e, sh, y;
    int64_t l2, L, k;
    if (x == 0u) {
        return 0;
    }
    if (g_q16 == 65536u) {
        return (uint32_t)(((uint64_t)x * 65535u + 0x80000000u) >> 32);
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
    l2 = ((int64_t)((1u << 29) - q) << 2) - ((int64_t)(n + 1u) << 31);   /* log2(x), Q31, < 0 */
    L = (l2 * (int64_t)g_q16) / 65536;                   /* g * log2(x), Q31, <= 0 */
    k = -((-L + 0x7FFFFFFF) >> 31);                      /* floor(L / 2^31) */
    fp = (uint32_t)(L - k * 0x80000000LL) << 1;          /* [0, 1) as Q32 */
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
    sh = (uint32_t)(6 - k);
    if (sh >= 32u) {
        return 0;
    }
    y = (uint32_t)(((uint64_t)(e >> sh) * 65535u + 0x800000u) >> 24);
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
    int64_t x = (int64_t)p;
    int64_t x3 = (x * x * x) >> 16;                                   /* Q32 */
    int64_t in = (x * (6 * x - 15 * 65536) + 10 * 4294967296LL) >> 8; /* Q24, 6x^2 - 15x + 10 */
    int64_t s = (x3 * in + (1LL << 39)) >> 40;                         /* Q16 */
    return s < 0 ? 0u : (s > 65536 ? 65536u : (uint32_t)s);
}

static uint16_t lerp16(uint32_t a, uint32_t b, uint32_t s)   /* s Q16, 0..65536 */
{
    int32_t d = (int32_t)b - (int32_t)a;
    return (uint16_t)((int32_t)a + (int32_t)(((int64_t)d * s) >> 16));
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

static uint32_t xorshift(uint32_t *s)
{
    uint32_t x = *s ? *s : 0x2545F491u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

static uint32_t sin16(uint32_t q)         /* sin(pi/2 * q / 65536) for q 0..65536, Q16 (quarter wave) */
{
    uint32_t idx = q >> 10, r = q & 0x3FFu;
    if (idx >= 64u) {
        return 65535u;
    }
    return ultra_sin_tab[idx] + (((uint32_t)(ultra_sin_tab[idx + 1] - ultra_sin_tab[idx]) * r) >> 10);
}

/* ================================================================================================ clock */
uint64_t ultra_host_us(const ultra_t *u, uint32_t tick)
{
    const ultra_clock_t *c = &u->clk;
    int32_t dt = (int32_t)(tick - c->ref_tick);
    if (!c->synced) {
        return (uint64_t)tick * ULTRA_TICK_US;
    }
    if (dt >= 0) {
        return c->ref_us + (((uint64_t)(uint32_t)dt * c->rate_q16) >> 16);
    }
    return c->ref_us - (((uint64_t)(uint32_t)(-dt) * c->rate_q16) >> 16);
}

static uint32_t host_ms(const ultra_t *u, uint32_t tick)
{
    return (uint32_t)(ultra_host_us(u, tick) / 1000u);
}

/* the host clock jumped (first header, host reboot): keep every running ramp / effect where it is */
static void rebase(ultra_t *u, uint32_t delta_ms)
{
    uint32_t i, j;
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        ultra_light_t *L = &u->light[i];
        L->cur.t0 += delta_ms;
        L->prev.t0 += delta_ms;
        L->fx_t0 += delta_ms;
        L->pend_t0 += delta_ms;
        L->fx_next += delta_ms;
        L->fx_ramp.t0 += delta_ms;
    }
    for (i = 0; i <= ULTRA_LIGHTS; i++) {               /* a rebooted host restarts its sequence numbers */
        for (j = 0; j < ULTRA_TYPES; j++) {
            u->last_seq[i][j] = 0x100u;
        }
    }
}

void ultra_on_header(ultra_t *u, uint32_t tick, uint32_t host_us)
{
    ultra_clock_t *c = &u->clk;
    uint64_t h, est;
    int64_t err;
    /* unwrap the 32-bit us clock (wraps every 71.6 min; headers come every few 100 ms) */
    h = c->synced ? c->ext_us + (uint64_t)(int64_t)(int32_t)(host_us - (uint32_t)c->ext_us) : host_us;
    c->ext_us = h;
    u->frame_ms = (uint32_t)(h / 1000u);
    est = ultra_host_us(u, tick);
    err = (int64_t)(h - est);
    if (!c->synced || err > 200000 || err < -200000) {   /* first header / host rebooted / long gap */
        rebase(u, u->frame_ms - (uint32_t)(est / 1000u));
        c->synced = 1;
        if (c->rate_q16 == 0u) {
            c->rate_q16 = RATE_NOMINAL;
        }
        c->ref_tick = c->last_tick = tick;
        c->ref_us = c->last_us = h;
        return;
    }
    if (c->gap) {                                        /* the interval spans a stop of the local tick */
        c->gap = 0;
        c->last_tick = tick;
        c->last_us = h;
    }
    if ((uint32_t)(tick - c->last_tick) >= 10000u) {     /* >= 0.5 s: rate from the host clock */
        uint32_t dt = tick - c->last_tick;
        uint64_t dh = h - c->last_us;
        uint32_t meas = (uint32_t)((dh << 16) / dt);
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
    /* phase: a quarter of the error per header, so effects never jump */
    c->ref_us = (uint64_t)((int64_t)est + err / 4);
    c->ref_tick = tick;
}

void ultra_clock_gap(ultra_t *u)
{
    u->clk.gap = 1;
}

/* ================================================================================================ state */
void ultra_init(ultra_t *u, uint32_t seed)
{
    uint32_t i, j;
    uint8_t *b = (uint8_t *)u;
    for (i = 0; i < sizeof(*u); i++) {
        b[i] = 0;
    }
    for (i = 0; i <= ULTRA_LIGHTS; i++) {
        for (j = 0; j < ULTRA_TYPES; j++) {
            u->last_seq[i][j] = 0x100u;
        }
    }
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        u->light[i].rng = seed ^ ((i + 1u) * 0x9E3779B9u);
        u->light[i].g_q16 = 183501u;                      /* 2.8 until LIGHT_DEF */
    }
    u->cfg[ULTRA_CFG_LOSS_TIMEOUT] = 30u;                 /* 3 s */
}

static void ramp_eval(const ultra_ramp_t *r, uint32_t now, ultra_vals_t *o)
{
    int32_t el = (int32_t)(now - r->t0);
    uint32_t p, s, i;
    if (el < 0) {
        *o = r->from;
        return;
    }
    p = r->dur ? frac16((uint32_t)el, r->dur) : 65536u;
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
        *o = r->from;
        o->m = lerp16(r->from.m, 0, s);
    } else {                                             /* second half: new values up from 0 */
        s = r->smooth ? ultra_smoothstep(p * 2u - 65536u) : p * 2u - 65536u;
        *o = r->to;
        o->m = lerp16(0, r->to.m, s);
    }
}

static void base_at(const ultra_light_t *L, uint32_t now, ultra_vals_t *o)
{
    ramp_eval((int32_t)(now - L->cur.t0) < 0 ? &L->prev : &L->cur, now, o);
}

/* a finished ramp becomes a constant at `now`, so its start time never ages past the int32 range */
static void settle(ultra_light_t *L, uint32_t now)
{
    int32_t el = (int32_t)(now - L->cur.t0);
    if (el >= 0 && (uint32_t)el >= L->cur.dur) {
        L->cur.from = L->cur.to;
        L->cur.dur = 0;
        L->cur.via_off = 0;
        L->cur.t0 = now;
        L->prev = L->cur;
    }
}

static void start_ramp(ultra_light_t *L, const ultra_vals_t *to, uint32_t t0, uint32_t dur, uint8_t flags)
{
    ultra_vals_t from;
    uint32_t i;
    base_at(L, t0, &from);
    if (from.m == 0u) {                                  /* dark: colors jump (ESPHome off -> on) */
        for (i = 0; i < 5u; i++) {
            from.a[i] = to->a[i];
            from.b[i] = to->b[i];
        }
    }
    if ((int32_t)(t0 - L->cur.t0) >= 0) {
        L->prev = L->cur;
    }
    L->cur.from = from;
    L->cur.to = *to;
    L->cur.t0 = t0;
    L->cur.dur = dur;
    L->cur.smooth = (uint8_t)((flags & ULTRA_SET_SMOOTH) != 0u);
    L->cur.via_off = (uint8_t)((flags & ULTRA_SET_VIA_OFF) != 0u && from.m != 0u);
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
        L->fx = ULTRA_FX_NONE;
        L->pend = 0;
        start_ramp(L, &to, now, dur_ms, ULTRA_SET_SMOOTH);
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

uint32_t ultra_apply_slot(ultra_t *u, const uint8_t *s, uint32_t tick)
{
    uint8_t type = s[0], light = (uint8_t)(s[1] >> 4), flags = (uint8_t)(s[1] & 0x0Fu), seq = s[2];
    const uint8_t *p = s + ULTRA_PAYLOAD;
    uint16_t *last;
    ultra_light_t *L;
    uint32_t i;
    (void)tick;

    /* compatibility rules (ultra_protocol.h): what this firmware doesn't know is ignored, the state stays untouched */
    if (type == ULTRA_NOP || type == ULTRA_POLL || type >= ULTRA_TYPES) {
        /* POLL: reserved for the return channel, not implemented (frame_rx also skips RW / EXT / UPD frames);
         * 0x0A-0x7F later commands; 0x80-0xFF broadcast slots, never a board's own slot (a wrong board count) */
        return 0;
    }
    if (light >= ULTRA_LIGHTS && light != ULTRA_LIGHT_BOARD) {
        return 0;                                        /* LIGHT 5-14: reserved */
    }
    if ((type == ULTRA_IDENTIFY && ((p[0] > ULTRA_IDENT_RELEASE && p[0] != ULTRA_IDENT_VERSION) ||
                                    (p[0] == ULTRA_IDENT_CAL && p[1] != 0u &&
                                     (p[1] < ULTRA_IDENT_CAL_MIN || p[1] > ULTRA_IDENT_CAL_MAX)))) ||
        (type == ULTRA_SCENE && p[0] > 1u)) {
        return 0;                                        /* an IDENTIFY / SCENE of later firmware */
    }
    if (type == ULTRA_STEP) {                            /* steps belong to an EFFECT seq, kept per index */
        uint8_t idx = (uint8_t)(p[0] >> 4), cnt = (uint8_t)(p[0] & 0x0Fu);
        if (light >= ULTRA_LIGHTS || idx >= cnt || cnt > ULTRA_MAX_STEPS) {
            return 0;
        }
        L = &u->light[light];
        if (L->step_seq != seq || L->step_count != cnt) {
            L->step_seq = seq;
            L->step_count = cnt;
            L->step_have = 0;
        }
        L->step[idx].dur = (uint16_t)ultra_get16(p + 1);
        L->step[idx].trans = (uint16_t)ultra_get16(p + 3);
        L->step[idx].v.m = (uint16_t)ultra_get16(p + 5);
        for (i = 0; i < 5u; i++) {
            L->step[idx].v.a[i] = (uint16_t)ultra_get16(p + 7 + 2 * i);
            L->step[idx].v.b[i] = (uint16_t)ultra_get16(p + 17 + 2 * i);
        }
        L->step_have |= (uint8_t)(1u << idx);
        return ULTRA_EV_LIGHT;
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
            if (id == ULTRA_CFG_NONE || id >= ULTRA_CFG_COUNT) {
                continue;
            }
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
    switch (type) {
    case ULTRA_LIGHT_OPT:
        L->opt_flags = (uint8_t)(p[0] & ULTRA_OPT_FLAGS);   /* unknown bits: ignored */
        L->opt_cw = (uint16_t)ultra_get16(p + 1);
        L->opt_ww = (uint16_t)ultra_get16(p + 3);
        L->opt_warm = (uint16_t)ultra_get16(p + 5);
        L->opt_assist = (uint16_t)ultra_get16(p + 7);
        opt_points(L, p);
        return ULTRA_EV_LIGHT | ULTRA_EV_DEF;
    case ULTRA_SCENE:
        L->scene_have = 1;
        L->scene_mode = p[0];
        L->scene_fade = p[1];
        L->scene_v.m = (uint16_t)ultra_get16(p + 2);
        for (i = 0; i < 5u; i++) {
            L->scene_v.a[i] = (uint16_t)ultra_get16(p + 4 + 2 * i);
            L->scene_v.b[i] = (uint16_t)ultra_get16(p + 14 + 2 * i);
        }
        return ULTRA_EV_SCENE;
    case ULTRA_SCENE_EFFECT:                             /* the power-on effect (main.c stores it) */
        L->scene_fx_have = 1;
        L->scene_fx = p[0] < ULTRA_FX_COUNT ? p[0] : ULTRA_FX_NONE;
        for (i = 0; i < ULTRA_SCENE_FX_PARAMS; i++) {
            L->scene_fxp[i] = p[1 + i];
        }
        return ULTRA_EV_SCENE;
    case ULTRA_LIGHT_DEF:
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
    case ULTRA_EFFECT:                                   /* switches at its start time (light_values) */
        L->scene_run = 0;                                /* the host's effect replaces the power-on one */
        L->pend = 1;
        L->pend_fx = p[0] < ULTRA_FX_COUNT ? p[0] : ULTRA_FX_NONE;
        L->pend_seq = seq;
        L->pend_t0 = u->frame_ms + ultra_get32(p + 1);
        for (i = 0; i < ULTRA_FX_PARAMS; i++) {
            L->pend_p[i] = p[5 + i];
        }
        return ULTRA_EV_LIGHT | ULTRA_EV_STATE;
    default:
        break;
    }
    return ULTRA_EV_LIGHT;
}

/* ================================================================================================ effects */
static uint16_t rnd16(ultra_light_t *L)
{
    return (uint16_t)(xorshift(&L->rng) >> 16);
}

/* ESPHome random_cubic_float() x intensity: (2r - 1)^3 x i, Q16 signed */
static int32_t rnd_cubic(ultra_light_t *L, uint32_t intensity)
{
    int64_t r = (int64_t)rnd16(L) * 2 - 65535;
    int64_t c = (((r * r) >> 16) * r) >> 16;
    return (int32_t)((c * (int64_t)intensity) >> 16);
}

static uint16_t clamp16(int32_t v)
{
    return (uint16_t)(v < 0 ? 0 : (v > 65535 ? 65535 : v));
}

static int is_rgb(uint8_t r) { return r >= ULTRA_ROLE_RED && r <= ULTRA_ROLE_BLUE; }
static int is_cww(uint8_t r) { return r == ULTRA_ROLE_COLD_WHITE || r == ULTRA_ROLE_WARM_WHITE; }

static int has_color(const ultra_light_t *L)
{
    uint32_t i;
    for (i = 0; i < 5u; i++) {
        if (is_rgb(L->role[i]) || is_cww(L->role[i])) {
            return 1;
        }
    }
    return 0;
}

static uint32_t strobe_cycle(const ultra_light_t *L)
{
    uint32_t c = L->step_count, cycle = 0, i;
    if (c == 0u || L->step_seq != L->fx_seq || L->step_have != (uint8_t)((1u << c) - 1u)) {
        return 0;                                        /* steps not complete (yet) */
    }
    for (i = 0; i < c; i++) {
        cycle += L->step[i].dur ? L->step[i].dur : 1u;
    }
    return cycle;
}

/* the effect's repeat period (ms), 0 = not periodic */
static uint32_t fx_period(const ultra_light_t *L)
{
    uint32_t v;
    switch (L->fx) {
    case ULTRA_FX_PULSE:
        v = ultra_get24(L->fxp + 6);
        return 2u * (v ? v : 1u);
    case ULTRA_FX_STROBE:
        return strobe_cycle(L);
    case ULTRA_FX_BREATHE:
        v = ultra_get24(L->fxp);
        return v ? v : 4000u;
    case ULTRA_FX_COLOR_LOOP:
        v = ultra_get24(L->fxp);
        return v ? v : 30000u;
    default:
        return 0;
    }
}

static uint32_t pulse_target(const ultra_light_t *L, uint32_t k)
{
    return (k & 1u) ? ultra_get16(L->fxp + 11) : ultra_get16(L->fxp + 9);   /* even: min, odd: max */
}

/* ESPHome pulse: every interval the brightness target flips min -> max -> min ..., with the off / on
 * transition length (smoothstep); a transition longer than the interval is cut short where it got to */
static void fx_pulse(const ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t on = ultra_get24(L->fxp), off = ultra_get24(L->fxp + 3), iv = ultra_get24(L->fxp + 6);
    uint32_t el = now - L->fx_t0, k, t, len, from, prev_len;
    if (iv == 0u) {
        iv = 1;
    }
    k = el / iv;
    t = el - k * iv;
    len = (k & 1u) ? on : off;
    if (k == 0u && L->fx_k == 0u) {
        from = b->m;
    } else {
        prev_len = (k & 1u) ? off : on;
        from = pulse_target(L, k + 1u);                  /* the previous target (other parity) */
        if (prev_len > iv) {
            uint32_t from2 = (k >= 2u || L->fx_k) ? pulse_target(L, k) : b->m;
            from = lerp16(from2, from, ultra_smoothstep(frac16(iv, prev_len)));
        }
    }
    *o = *b;
    o->m = lerp16(from, pulse_target(L, k), len ? ultra_smoothstep(frac16(t, len)) : 65536u);
}

static void fx_strobe(const ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t c = L->step_count, cycle = strobe_cycle(L), el, i, j, start = 0, s;
    const ultra_step_t *st, *pv;
    if (cycle == 0u) {
        *o = *b;
        return;
    }
    el = (now - L->fx_t0) % cycle;
    for (i = 0; i + 1u < c; i++) {
        uint32_t d = L->step[i].dur ? L->step[i].dur : 1u;
        if (el < start + d) {
            break;
        }
        start += d;
    }
    st = &L->step[i];
    pv = &L->step[(i + c - 1u) % c];
    s = st->trans ? ultra_smoothstep(frac16(el - start, st->trans)) : 65536u;
    o->m = lerp16(pv->v.m, st->v.m, s);
    for (j = 0; j < 5u; j++) {
        o->a[j] = lerp16(pv->v.a[j], st->v.a[j], s);
        o->b[j] = lerp16(pv->v.b[j], st->v.b[j], s);
    }
}

/* ESPHome flicker: out = remote x (1 - alpha) + current x alpha + random_cubic x intensity, per interval */
static void fx_flicker(ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t alpha = ultra_get16(L->fxp), inten = ultra_get16(L->fxp + 2), iv = ultra_get16(L->fxp + 4), n = 0, i;
    ultra_vals_t *c = &L->fx_ramp.to;
    uint32_t beta = 65535u - alpha;
    if (iv == 0u) {
        iv = 16u;
    }
    if ((int32_t)(now - L->fx_next) > (int32_t)(iv * 64u)) {
        L->fx_next = now;                                /* far behind (start / pause): no catch-up burst */
    }
    while ((int32_t)(now - L->fx_next) >= 0 && n++ < 64u) {
        c->m = clamp16((int32_t)((b->m * beta + c->m * alpha) / 65535u) + rnd_cubic(L, inten));
        for (i = 0; i < 5u; i++) {
            c->a[i] = clamp16((int32_t)((b->a[i] * beta + c->a[i] * alpha) / 65535u) + rnd_cubic(L, inten));
        }
        L->fx_next += iv;
    }
    *o = *c;
    for (i = 0; i < 5u; i++) {
        o->b[i] = b->b[i];
    }
}

/* ESPHome random: every interval a random color (RGB normalized, CW / WW random), brightness only for lights
 * without colors; transition with smoothstep; color brightness (B) stays the light's */
static void fx_random(ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t tr = ultra_get24(L->fxp), iv = ultra_get24(L->fxp + 3), i;
    int colored = has_color(L);
    if (iv == 0u) {
        iv = 1000u;
    }
    if ((int32_t)(now - L->fx_next) >= 0) {
        ultra_vals_t from, to = *b;
        uint32_t rgb_max = 0, r[5] = { 0, 0, 0, 0, 0 };
        ramp_eval(&L->fx_ramp, now, &from);
        for (i = 0; i < 5u; i++) {
            if (is_rgb(L->role[i])) {
                r[i] = rnd16(L);
                rgb_max = r[i] > rgb_max ? r[i] : rgb_max;
            } else if (is_cww(L->role[i])) {
                to.a[i] = rnd16(L);
            }
        }
        for (i = 0; i < 5u; i++) {
            if (is_rgb(L->role[i])) {                    /* normalized like ESPHome: brightest = 1 */
                to.a[i] = (uint16_t)(rgb_max ? r[i] * 65535u / rgb_max : 65535u);
            }
        }
        if (!colored) {
            to.m = rnd16(L);
        }
        L->fx_ramp.from = from;
        L->fx_ramp.to = to;
        L->fx_ramp.t0 = now;
        L->fx_ramp.dur = tr;
        L->fx_next += ((now - L->fx_next) / iv + 1u) * iv;
    }
    ramp_eval(&L->fx_ramp, now, o);
    if (colored) {
        o->m = b->m;                                     /* colors are random, brightness follows the light */
    }
    for (i = 0; i < 5u; i++) {
        o->b[i] = b->b[i];
    }
}

/* candle: mostly small dips, now and then a deeper one, smoothed (~16 ms) */
static void fx_candle(ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t inten = ultra_get16(L->fxp), speed = ultra_get16(L->fxp + 2), r;
    if (speed == 0u) {
        speed = 100u;
    }
    if ((int32_t)(now - L->fx_next) >= 0) {
        r = rnd16(L);
        r = (r < 6000u) ? r * 8u : (r * r) >> 18;
        L->candle_t = 65536u - ((r > 65535u ? 65535u : r) * inten >> 16);
        L->fx_next = now + (20u + (rnd16(L) & 31u)) * 100u / speed;
    }
    L->candle_f += (uint32_t)((int32_t)(L->candle_t - L->candle_f) / 16);
    *o = *b;
    o->m = (uint16_t)((b->m * (L->candle_f > 65536u ? 65536u : L->candle_f)) >> 16);
}

uint32_t ultra_breathe(uint32_t ph)
{
    uint32_t s = sin16(ph <= 32768u ? ph * 2u : (65536u - ph) * 2u);   /* sin(pi ph) */
    return (s * s) >> 16;
}

/* breathe: brightness min..max along (1 - cos) / 2, starts at min */
static void fx_breathe(const ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t per = fx_period(L), mn = ultra_get16(L->fxp + 3), mx = ultra_get16(L->fxp + 5);
    *o = *b;
    o->m = lerp16(mn, mx, ultra_breathe(frac16((now - L->fx_t0) % per, per)));
}

/* color loop: hue rotation at the given saturation on the RGB channels (W / CW / WW off) */
static void fx_color_loop(const ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o)
{
    uint32_t per = fx_period(L), sat = ultra_get16(L->fxp + 3), h6, f, pv, q, t, rgb[3], i, n = 0;
    h6 = frac16((now - L->fx_t0) % per, per) * 6u;
    f = h6 & 0xFFFFu;
    pv = 65535u - sat;
    q = 65535u - (uint32_t)(((uint64_t)sat * f) >> 16);
    t = 65535u - (uint32_t)(((uint64_t)sat * (65536u - f)) >> 16);
    switch ((h6 >> 16) % 6u) {
    case 0: rgb[0] = 65535u; rgb[1] = t; rgb[2] = pv; break;
    case 1: rgb[0] = q; rgb[1] = 65535u; rgb[2] = pv; break;
    case 2: rgb[0] = pv; rgb[1] = 65535u; rgb[2] = t; break;
    case 3: rgb[0] = pv; rgb[1] = q; rgb[2] = 65535u; break;
    case 4: rgb[0] = t; rgb[1] = pv; rgb[2] = 65535u; break;
    default: rgb[0] = 65535u; rgb[1] = pv; rgb[2] = q; break;
    }
    *o = *b;
    for (i = 0; i < 5u; i++) {
        if (is_rgb(L->role[i])) {
            o->a[i] = (uint16_t)rgb[L->role[i] - ULTRA_ROLE_RED];
            n++;
        }
    }
    for (i = 0; n && i < 5u; i++) {
        if (L->role[i] == ULTRA_ROLE_WHITE || is_cww(L->role[i])) {
            o->a[i] = 0;
        }
    }
}

/* sunrise: from a dark, deep orange dawn to the light's state; sunset: the reverse, ending off.
 * Returns 1 when done. */
static int fx_sun(const ultra_light_t *L, uint32_t now, const ultra_vals_t *b, ultra_vals_t *o, int rise)
{
    static const uint16_t dawn[3] = { 65535u, 11796u, 0u };   /* (1, 0.18, 0) */
    uint32_t d = ultra_get24(L->fxp), el = now - L->fx_t0, p, w, i;
    if (d == 0u) {
        d = 1u;
    }
    p = frac16(el, d);
    w = rise ? p : 65536u - p;                           /* weight of the light's own state */
    *o = *b;
    o->m = (uint16_t)((b->m * (uint32_t)(((uint64_t)w * w) >> 16)) >> 16);   /* eases in / out (quadratic) */
    for (i = 0; i < 5u; i++) {
        uint8_t r = L->role[i];
        if (is_rgb(r)) {
            o->a[i] = lerp16(dawn[r - ULTRA_ROLE_RED], b->a[i], w);
        } else if (r == ULTRA_ROLE_WARM_WHITE) {
            o->a[i] = lerp16(65535u, b->a[i], w);
        } else if (r == ULTRA_ROLE_COLD_WHITE || r == ULTRA_ROLE_WHITE) {
            o->a[i] = lerp16(0, b->a[i], w);
        }
    }
    return el >= d;
}

/* ================================================================================================ render */
static void fx_activate(ultra_light_t *L, uint32_t now)
{
    uint32_t i;
    L->pend = 0;
    L->fx = L->pend_fx;
    L->fx_seq = L->pend_seq;
    L->fx_t0 = L->pend_t0;
    L->fx_k = 0;
    for (i = 0; i < ULTRA_FX_PARAMS; i++) {
        L->fxp[i] = L->pend_p[i];
    }
    L->fx_next = now;
    base_at(L, now, &L->fx_ramp.to);                    /* random / flicker start from the light's state */
    L->fx_ramp.from = L->fx_ramp.to;
    L->fx_ramp.t0 = now;
    L->fx_ramp.dur = 0;
    L->fx_ramp.smooth = 1;
    L->fx_ramp.via_off = 0;
    L->candle_f = L->candle_t = 65536u;
}

static void light_values(ultra_light_t *L, uint32_t now, ultra_vals_t *v)
{
    ultra_vals_t b;
    uint32_t el, per;
    settle(L, now);
    base_at(L, now, &b);
    if (L->pend && (int32_t)(now - L->pend_t0) >= 0) {
        int32_t d = (int32_t)(L->cur.t0 - now);
        if (L->fx != ULTRA_FX_NONE && L->cur.dur != 0u && d >= -2 && d <= 2) {
            /* a transition that starts as the effect ends (turning off during an effect) starts from what the
             * effect showed, like ESPHome's; the host's two commands may land up to 1 ms apart */
            L->cur.from = L->last;
            L->prev.from = L->prev.to = L->last;
            L->prev.t0 = now;
            L->prev.dur = 0;
            L->prev.via_off = 0;
            base_at(L, now, &b);
        }
        fx_activate(L, now);
    }
    if (L->fx == ULTRA_FX_NONE) {
        *v = b;
        L->last = b;
        return;
    }
    el = now - L->fx_t0;
    if (el >= FX_REBASE_MS && (per = fx_period(L)) != 0u) {
        L->fx_t0 += el - el % per;
        L->fx_k = 1;
    }
    switch (L->fx) {
    case ULTRA_FX_PULSE: fx_pulse(L, now, &b, v); break;
    case ULTRA_FX_STROBE: fx_strobe(L, now, &b, v); break;
    case ULTRA_FX_FLICKER: fx_flicker(L, now, &b, v); break;
    case ULTRA_FX_RANDOM: fx_random(L, now, &b, v); break;
    case ULTRA_FX_CANDLE: fx_candle(L, now, &b, v); break;
    case ULTRA_FX_BREATHE: fx_breathe(L, now, &b, v); break;
    case ULTRA_FX_COLOR_LOOP: fx_color_loop(L, now, &b, v); break;
    case ULTRA_FX_SUNRISE:
        if (fx_sun(L, now, &b, v, 1)) {
            L->fx = ULTRA_FX_NONE;                       /* done: the light's state */
        }
        break;
    case ULTRA_FX_SUNSET:
        if (fx_sun(L, now, &b, v, 0)) {
            L->fx = ULTRA_FX_NONE;                       /* done: stays off until the next SET */
            L->cur.from = L->cur.to = b;
            L->cur.from.m = L->cur.to.m = 0;
            L->cur.t0 = now;
            L->cur.dur = 0;
            L->cur.via_off = 0;
            L->prev = L->cur;
        }
        break;
    default: *v = b; break;
    }
    if (b.m == 0u) {
        v->m = 0;                                        /* a light that is off stays off */
    }
    L->last = *v;
}

static uint32_t g16(const ultra_light_t *L, uint32_t v16)
{
    return ultra_gamma(q32_of(v16, 65535u), L->g_q16);
}

/* M x A x B as a Q32 fraction (65535 x 65535 x 65535 -> ~1.0) */
static uint32_t q32_of3(uint32_t m, uint32_t a, uint32_t b)
{
    return (uint32_t)(((uint64_t)q32_of(m, a) * (b + (b >> 15))) >> 16);
}

/* Set points (sorted by M, descending): linear in mireds between them, the light's own temperature (own_q) at M = 1
 * unless a point is at 100 %, the lowest point's temperature below it. Mireds x 10, Q16. */
static uint64_t dtw_points(const ultra_light_t *L, uint32_t m, uint64_t own_q)
{
    uint32_t i, hm = 65535u;
    int64_t hq = (int64_t)own_q;
    for (i = 0; i < L->opt_n; i++) {
        uint32_t pm = L->opt_pm[i];
        int64_t pq = (int64_t)((uint64_t)L->opt_pt[i] << 16);
        if (m >= pm) {
            return hm == pm ? (uint64_t)pq : (uint64_t)(pq + (hq - pq) * (int64_t)(m - pm) / (int64_t)(hm - pm));
        }
        hm = pm;
        hq = pq;
    }
    return (uint64_t)hq;
}

/* Dim to warm (LIGHT_OPT, see ultra_protocol.h): the CW / WW mix of y[] follows M, red assist below the WW LED.
 * gcw / gww: the light's own gamma'd CW / WW values; on[]: the channel is meant to be on. */
static void dim_to_warm(const ultra_light_t *L, uint32_t m, uint32_t cw_ch, uint32_t ww_ch, uint32_t gcw, uint32_t gww,
                        uint32_t *y, uint8_t *on)
{
    uint32_t sum = gcw + gww, mx = gcw > gww ? gcw : gww, cwm = L->opt_cw, wwm = L->opt_ww, warm = L->opt_warm;
    uint32_t t = 0, scale, fw, fc, ch, r_ch = 5, g_ch = 5, rgb_on = 0;
    uint64_t cw_q, ww_q, warm_q, top_q, tgt_q;            /* mireds x 10, Q16 */
    if (sum == 0u || wwm <= cwm) {
        return;                                          /* white off, or no colour temperatures */
    }
    cw_q = (uint64_t)cwm << 16;
    ww_q = (uint64_t)wwm << 16;
    warm_q = (uint64_t)warm << 16;
    top_q = cw_q + (uint64_t)(wwm - cwm) * (((uint64_t)gww << 16) / sum);   /* the light's own temperature */
    tgt_q = top_q;
    if (L->opt_flags & ULTRA_OPT_POINTS) {
        uint64_t pq = dtw_points(L, m, top_q);
        if (pq > top_q) {
            tgt_q = pq;                                  /* never colder than the light's own temperature */
        }
    } else if (warm_q > top_q) {
        tgt_q += (warm_q - top_q) * (65535u - m) / 65535u;                  /* linear in mireds, M 1 -> 0 */
    }
    if (tgt_q <= ww_q) {
        fw = (uint32_t)((tgt_q - cw_q) / (wwm - cwm));  /* WW share, Q16 */
    } else {
        fw = 65536u;                                     /* warmer than the WW LED: all WW, red assist */
        t = (uint32_t)((tgt_q - ww_q) / (warm - wwm));
    }
    fc = 65536u - fw;
    scale = (uint32_t)(((uint64_t)g16(L, m) * mx + 32767u) / 65535u);   /* g(M) x max(gcw, gww) */
    if (L->flags & ULTRA_DEF_CONSTANT_BRIGHTNESS) {
        y[cw_ch] = (uint32_t)(((uint64_t)scale * fc + 32768u) >> 16);   /* CW + WW = g(M) x max */
        y[ww_ch] = (uint32_t)(((uint64_t)scale * fw + 32768u) >> 16);
    } else {
        uint32_t fmax = fw > fc ? fw : fc;               /* the brighter channel stays at g(M) x max */
        y[cw_ch] = (uint32_t)(((uint64_t)scale * fc + fmax / 2u) / fmax);
        y[ww_ch] = (uint32_t)(((uint64_t)scale * fw + fmax / 2u) / fmax);
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
        uint32_t r = (uint32_t)(((((uint64_t)y[ww_ch] * t) >> 16) * L->opt_assist + 32767u) / 65535u);
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
    const ultra_vals_t v = *vp;
    const uint32_t cb = (L->flags & ULTRA_DEF_CONSTANT_BRIGHTNESS) != 0u, dtw = (L->opt_flags & ULTRA_OPT_DIM_TO_WARM) != 0u;
    uint32_t gcw = 0, gww = 0, cw_ch = 5, ww_ch = 5, ch, y[5];
    uint8_t on[5];
    if (cb || dtw) {
        for (ch = 0; ch < 5u; ch++) {
            if (L->role[ch] == ULTRA_ROLE_COLD_WHITE) { cw_ch = ch; gcw = ultra_gamma(q32_of(v.a[ch], v.b[ch]), L->g_q16); }
            if (L->role[ch] == ULTRA_ROLE_WARM_WHITE) { ww_ch = ch; gww = ultra_gamma(q32_of(v.a[ch], v.b[ch]), L->g_q16); }
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
            /* ESPHome constant_brightness: g(M) x max(gcw, gww) x g(this) / (gcw + gww) */
            uint32_t sum = gcw + gww, mxg = gcw > gww ? gcw : gww, mine = ch == cw_ch ? gcw : gww;
            y[ch] = sum ? (uint32_t)((uint64_t)g16(L, v.m) * mxg / 65535u * mine / sum) : 0u;
            x = (uint32_t)v.m * mine;
        } else {
            x = L->role[ch] == ULTRA_ROLE_BRIGHTNESS ? q32_of(v.m, 65535u) : q32_of3(v.m, v.a[ch], v.b[ch]);
            y[ch] = ultra_gamma(x, L->g_q16);
        }
        on[ch] = (uint8_t)(x != 0u);
    }
    if (dtw && cw_ch < 5u && ww_ch < 5u) {
        dim_to_warm(L, v.m, cw_ch, ww_ch, gcw, gww, y, on);
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
            uint32_t mn = L->min_p[ch], mx = L->max_p[ch];
            yc = mn + (uint32_t)(((uint64_t)yc * (mx >= mn ? mx - mn : 0u) + 32767u) / 65535u);
        }
        res[ch] = yc > 65535u ? 65535u : yc;
    }
}

/* power limit per channel (CONFIG 16-20), from the LED current of each channel at 100 % (CONFIG 11-15). Returns 1
 * when a channel was capped. The board limit (CONFIG 21) is the current sense's (main.c, core/isense.h). */
static uint8_t power_limit(const ultra_t *u, uint32_t *res)
{
    uint32_t ch;
    uint8_t f = 0;
    for (ch = 0; ch < 5u; ch++) {
        uint32_t ia = u->cfg[ULTRA_CFG_CH_CURRENT + ch], imax = u->cfg[ULTRA_CFG_CH_MAX + ch];
        if (ia == 0u) {
            continue;
        }
        if (imax != 0u && res[ch] * ia > imax * 65535u) {
            res[ch] = imax * 65535u / ia;
            f = 1u;
        }
    }
    return f;
}

uint8_t ultra_render(ultra_t *u, uint32_t tick, uint16_t *out)
{
    uint32_t now = host_ms(u, tick), res[5] = { 0, 0, 0, 0, 0 }, l, ch;
    uint8_t changed = 0;

    for (l = 0; l < ULTRA_LIGHTS; l++) {
        ultra_light_t *L = &u->light[l];
        ultra_vals_t v;
        if (!L->defined && !L->scene_run) {
            continue;
        }
        light_values(L, now, &v);
        light_out(L, &v, res);
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

int ultra_scene_fx_ok(const ultra_light_t *L)
{
    return L->defined && L->scene_mode == 1u && L->scene_fx != ULTRA_FX_NONE && L->scene_fx != ULTRA_FX_STROBE &&
           L->scene_fx < ULTRA_FX_COUNT;
}

void ultra_scene_fx_start(ultra_t *u, uint8_t mask, uint32_t tick)
{
    uint32_t l, i, now = host_ms(u, tick);
    for (l = 0; l < ULTRA_LIGHTS; l++) {
        ultra_light_t *L = &u->light[l];
        if (!(mask & (1u << l))) {
            continue;
        }
        L->scene_run = 1;
        start_ramp(L, &L->scene_v, now, 0, 0);           /* the state: the SCENE values */
        L->pend = 1;                                     /* the effect starts with the first render (its parameter */
        L->pend_fx = L->scene_fx;                        /* bytes past ULTRA_SCENE_FX_PARAMS: unused, 0 from init) */
        L->pend_t0 = now;
        for (i = 0; i < ULTRA_SCENE_FX_PARAMS; i++) {
            L->pend_p[i] = L->scene_fxp[i];
        }
    }
}

void ultra_scene_fx_stop(ultra_t *u)
{
    uint32_t l;
    for (l = 0; l < ULTRA_LIGHTS; l++) {
        ultra_light_t *L = &u->light[l];
        if (L->scene_run) {
            L->scene_run = 0;
            L->fx = ULTRA_FX_NONE;
            L->pend = 0;
        }
    }
}

void ultra_scene(const ultra_t *u, const scene_t *stored, scene_t *out)
{
    uint32_t res[5] = { 0, 0, 0, 0, 0 }, l, ch;
    uint8_t mask = 0, keep = 0, fade = 0, have_fade = 0;
    for (l = 0; l < ULTRA_LIGHTS; l++) {
        const ultra_light_t *L = &u->light[l];
        if (!L->defined) {
            continue;
        }
        if (!L->scene_have) {                            /* its SCENE is still to come: keep what is stored */
            keep |= (uint8_t)(L->mask & stored->mask);
            continue;
        }
        if (L->scene_mode) {
            light_out(L, &L->scene_v, res);
            mask |= L->mask;
            if (!have_fade || L->scene_fade > fade) {
                fade = L->scene_fade;
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
