/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "stagger.h"
#include "lto.h"
#include "scratch.h"

#define N        STAGGER_GRID
typedef char grid_fits_scratch[(4u * N <= CORE_SCRATCH_WORDS) ? 1 : -1];
#define FULL     65536u
#define DEADBAND 4                         /* counts: phase close enough (register read skew is ~2) */

enum { R = 0, G = 1, B = 2, W1 = 3, W2 = 4 };

/* pulse width of a 16-bit output value, Q16 (65535 = the whole period) */
static uint32_t width(uint16_t v)
{
    return (uint32_t)v + ((uint32_t)v >> 15);
}

/* overlap of the periodic pulses [a, a + x) and [b, b + y), Q16 */
static uint32_t ov(uint32_t a, uint32_t x, uint32_t b, uint32_t y)
{
    uint32_t s = (b - a) & 0xFFFFu;
    int32_t o1 = (int32_t)(x < s + y ? x : s + y) - (int32_t)s;
    int32_t o2 = (int32_t)(s + y) - (int32_t)FULL;
    if (o1 < 0) o1 = 0;
    if (o2 < 0) o2 = 0;
    if (o2 > (int32_t)x) o2 = (int32_t)x;
    return (uint32_t)(o1 + o2);
}

static uint32_t circ_dist(uint32_t a, uint32_t b)
{
    uint32_t d = (a - b) & 0xFFFFu;
    return d < FULL - d ? d : FULL - d;
}

typedef struct {
    uint32_t w[5];                        /* widths Q16 */
    uint32_t i[5];                        /* weights: peak current in 100 mA (cost tables) */
} pulses_t;

/* start of each pulse for offsets o2 / o3 */
static void starts(const pulses_t *p, uint32_t o2, uint32_t o3, uint32_t *s)
{
    s[R] = 0;
    s[G] = (FULL - p->w[G]) & 0xFFFFu;
    s[B] = o2 & 0xFFFFu;
    s[W1] = o3 & 0xFFFFu;
    s[W2] = (o3 - p->w[W2]) & 0xFFFFu;
}

/* weighted overlap of two channels, Q10 x (100 mA)^2 (fits 32 bits for the sums below) */
static uint32_t pair(const pulses_t *p, const uint32_t *s, int a, int b)
{
    return p->i[a] * p->i[b] * (ov(s[a], p->w[a], s[b], p->w[b]) >> 6);
}

/* sum over the channel pairs on different timers */
static uint32_t cross_cost(const pulses_t *p, uint32_t o2, uint32_t o3)
{
    uint32_t s[5];
    starts(p, o2, o3, s);
    return pair(p, s, R, B) + pair(p, s, G, B) + pair(p, s, R, W1) + pair(p, s, R, W2) + pair(p, s, G, W1) +
           pair(p, s, G, W2) + pair(p, s, B, W1) + pair(p, s, B, W2);
}

/* ---------------------------------------------------------------------------------------------- edge rule */
typedef struct {
    uint32_t on[2], off[2];
    uint8_t n;
} edges_t;

static ALWAYS_INLINE void add_edge(edges_t *e, uint16_t v, uint32_t on, uint32_t w)   /* (the search: lto.h) */
{
    if (v == 0u || v == 65535u) {
        return;                           /* always off / always on: no edges */
    }
    e->on[e->n] = on & 0xFFFFu;
    e->off[e->n] = (on + w) & 0xFFFFu;
    e->n++;
}

/* edges of each timer's channels, with the timer at offset 0 */
static void timer_edges(const uint16_t *duty, const pulses_t *p, edges_t *t1, edges_t *t2, edges_t *t3)
{
    t1->n = t2->n = t3->n = 0;
    add_edge(t1, duty[R], 0, p->w[R]);
    add_edge(t1, duty[G], FULL - p->w[G], p->w[G]);
    add_edge(t2, duty[B], 0, p->w[B]);
    add_edge(t3, duty[W1], 0, p->w[W1]);
    add_edge(t3, duty[W2], FULL - p->w[W2], p->w[W2]);
}

/* same-direction edges of timer a (at 0) and timer b (at offset) at least gap apart */
static int apart(const edges_t *a, const edges_t *b, uint32_t offset, uint32_t gap)
{
    uint32_t i, j;
    for (i = 0; i < a->n; i++) {
        for (j = 0; j < b->n; j++) {
            if (circ_dist(a->on[i], b->on[j] + offset) < gap || circ_dist(a->off[i], b->off[j] + offset) < gap) {
                return 0;
            }
        }
    }
    return 1;
}

static int valid3(const edges_t *t1, const edges_t *t2, const edges_t *t3, uint32_t o2, uint32_t o3, uint32_t gap)
{
    return apart(t1, t2, o2, gap) && apart(t1, t3, o3, gap) && apart(t2, t3, (o3 - o2) & 0xFFFFu, gap);
}

static void setup(const uint16_t *duty, const uint16_t *peak_ma, pulses_t *p)
{
    uint32_t k;
    for (k = 0; k < 5u; k++) {
        p->w[k] = width(duty[k]);
        p->i[k] = (peak_ma[k] + 50u) / 100u;
    }
}

int stagger_valid(const uint16_t *duty, uint16_t o2, uint16_t o3, uint16_t gap)
{
    pulses_t p;
    edges_t t1, t2, t3;
    static const uint16_t none[5] = { 0, 0, 0, 0, 0 };
    setup(duty, none, &p);
    timer_edges(duty, &p, &t1, &t2, &t3);
    return valid3(&t1, &t2, &t3, o2, o3, gap);
}

/* ---------------------------------------------------------------------------------------------- ripple */
static uint32_t isqrt64(uint64_t v)
{
    uint64_t r = 0, bit = (uint64_t)1 << 62;
    while (bit > v) {
        bit >>= 2;
    }
    while (bit != 0u) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)r;
}

uint32_t stagger_ripple_ma(const uint16_t *duty, const uint16_t *peak_ma, uint16_t o2, uint16_t o3)
{
    pulses_t p;
    uint32_t s[5], j, k;
    uint64_t s1 = 0, s2 = 0, m;
    setup(duty, peak_ma, &p);
    starts(&p, o2, o3, s);
    for (k = 0; k < 5u; k++) {
        s1 += (uint64_t)peak_ma[k] * p.w[k];                                 /* mA x Q16 */
        s2 += (uint64_t)peak_ma[k] * peak_ma[k] * p.w[k];                    /* mA^2 x Q16 */
        for (j = k + 1u; j < 5u; j++) {
            s2 += 2u * (uint64_t)peak_ma[k] * peak_ma[j] * ov(s[k], p.w[k], s[j], p.w[j]);
        }
    }
    m = s1 >> 8;                                                             /* mA x Q8 */
    m *= m;                                                                  /* mA^2 x Q16 */
    return s2 > m ? isqrt64((s2 - m) >> 16) : 0u;
}

uint16_t stagger_gap_q16(uint32_t steps)
{
    /* 0.5 us = 72 counts of 144 MHz */
    uint32_t g = (72u * FULL + steps / 2u) / steps;
    return (uint16_t)(g > 32767u ? 32767u : g);
}

/* ---------------------------------------------------------------------------------------------- search */
static uint32_t grid(uint32_t i)
{
    return ((i % N) * FULL + N / 2u) / N;
}

void stagger_choose(const uint16_t *duty, const uint16_t *peak_ma, uint16_t gap, uint16_t cur_o2, uint16_t cur_o3,
                    stagger_result_t *r)
{
    stagger_choose_max(duty, peak_ma, gap, cur_o2, cur_o3, 0, r);
}

MEASURED void stagger_choose_max(const uint16_t *duty, const uint16_t *peak_ma, uint16_t gap, uint16_t cur_o2,
                        uint16_t cur_o3, uint16_t o_max, stagger_result_t *r)
{
    uint32_t omax = o_max ? o_max : 0xFFFFu;
    /* cost per offset (0xFFFFFFFF = edge rule broken) and the grid offsets: in the main loop's shared scratch */
    uint32_t *c2 = core_scratch, *c3 = core_scratch + N, *c23 = core_scratch + 2u * N, *go = core_scratch + 3u * N;
    pulses_t p;
    edges_t t1, t2, t3;
    uint32_t i, j, best = 0xFFFFFFFFu, bi = 0, bj = 0, b2, b3, s[5];
    int32_t a, b;

    setup(duty, peak_ma, &p);
    timer_edges(duty, &p, &t1, &t2, &t3);

    /* one-dimensional tables: TIM1-TIM2 by o2, TIM1-TIM3 by o3, TIM2-TIM3 by o3 - o2 */
    for (i = 0; i < N; i++) {
        uint32_t o = grid(i);
        go[i] = o;
        starts(&p, o, o, s);
        c2[i] = (o <= omax && apart(&t1, &t2, o, gap)) ? pair(&p, s, R, B) + pair(&p, s, G, B) : 0xFFFFFFFFu;
        c3[i] = (o <= omax && apart(&t1, &t3, o, gap))
                    ? pair(&p, s, R, W1) + pair(&p, s, R, W2) + pair(&p, s, G, W1) + pair(&p, s, G, W2)
                    : 0xFFFFFFFFu;
        starts(&p, 0, o, s);
        c23[i] = apart(&t2, &t3, o, gap) ? pair(&p, s, B, W1) + pair(&p, s, B, W2) : 0xFFFFFFFFu;
    }
    for (i = 0; i < N; i++) {
        uint32_t k = N - i;                /* (j - i) mod N, counted along with j */
        if (c2[i] == 0xFFFFFFFFu) {
            continue;
        }
        for (j = 0; j < N; j++, k++) {
            uint32_t c;
            if (k == N) {
                k = 0;
            }
            if (c3[j] == 0xFFFFFFFFu || c23[k] == 0xFFFFFFFFu) {
                continue;
            }
            c = c2[i] + c3[j] + c23[k];
            /* c23 is tabled at grid(k), but grid(j) - grid(i) can be 1 count off (65536 / N is not whole): the
             * TIM2-TIM3 edge rule is checked at the exact difference before a point can win (c2, c3 are exact) */
            if (c < best && apart(&t2, &t3, (go[j] - go[i]) & 0xFFFFu, gap)) {
                best = c;
                bi = i;
                bj = j;
            }
        }
    }
    b2 = go[bi];
    b3 = go[bj];
    /* finer steps around the best grid point (T / (4 N)) */
    if (best != 0xFFFFFFFFu) {
        uint32_t c0 = b2, c1 = b3;
        for (a = -3; a <= 3; a++) {
            for (b = -3; b <= 3; b++) {
                uint32_t o2 = (c0 + (uint32_t)(a * (int32_t)(FULL / (4u * N)))) & 0xFFFFu;
                uint32_t o3 = (c1 + (uint32_t)(b * (int32_t)(FULL / (4u * N)))) & 0xFFFFu;
                uint32_t c;
                if ((a == 0 && b == 0) || o2 > omax || o3 > omax || !valid3(&t1, &t2, &t3, o2, o3, gap)) {
                    continue;
                }
                c = cross_cost(&p, o2, o3);
                if (c < best) {
                    best = c;
                    b2 = o2;
                    b3 = o3;
                }
            }
        }
    }
    /* the fixed stagger and the current offsets are candidates too */
    if (valid3(&t1, &t2, &t3, STAGGER_FIXED_O2, STAGGER_FIXED_O3, gap) &&
        cross_cost(&p, STAGGER_FIXED_O2, STAGGER_FIXED_O3) < best) {
        best = cross_cost(&p, STAGGER_FIXED_O2, STAGGER_FIXED_O3);
        b2 = STAGGER_FIXED_O2;
        b3 = STAGGER_FIXED_O3;
    }
    r->valid_now = (uint8_t)(cur_o2 <= omax && cur_o3 <= omax && valid3(&t1, &t2, &t3, cur_o2, cur_o3, gap));
    if (r->valid_now && cross_cost(&p, cur_o2, cur_o3) <= best) {
        best = cross_cost(&p, cur_o2, cur_o3);
        b2 = cur_o2;
        b3 = cur_o3;
    }
    r->o2 = (uint16_t)b2;
    r->o3 = (uint16_t)b3;
    r->ripple_now = stagger_ripple_ma(duty, peak_ma, cur_o2, cur_o3);
    r->ripple_best = best == 0xFFFFFFFFu ? r->ripple_now : stagger_ripple_ma(duty, peak_ma, (uint16_t)b2, (uint16_t)b3);
    r->move = 0;
    if (best == 0xFFFFFFFFu || (b2 == cur_o2 && b3 == cur_o3)) {
        return;
    }
    if (r->ripple_now > r->ripple_best + 300u && r->ripple_best * 100u < r->ripple_now * 85u) {
        r->move = 1;                      /* clearly better */
    } else if (!r->valid_now && r->ripple_best * 10u <= r->ripple_now * 11u) {
        r->move = 1;                      /* the current offsets break the edge rule */
    }
}

/* ---------------------------------------------------------------------------------------------- spread */
MEASURED void stagger_spread_arr(uint32_t steps, uint32_t len, uint16_t *arr)
{
    uint32_t i, x = 0x6D2B79F5u;
    int32_t span = (int32_t)((steps * STAGGER_SPREAD_Q16 + 32768u) >> 16), shift = 0;
    if (len < 2u) {
        if (len == 1u) arr[0] = (uint16_t)(steps - 1u);
        return;
    }
    if (steps + (uint32_t)span > 65536u) {
        shift = -span;                                   /* 2.197 kHz: only shorter, -12 .. 0 % */
    }
    /* even grid -span .. +span (symmetric: the deltas sum to 0) */
    for (i = 0; i < len; i++) {
        int32_t num = (2 * (int32_t)i - (int32_t)(len - 1u)) * span;
        int32_t d = num >= 0 ? (num + (int32_t)(len - 1u) / 2) / (int32_t)(len - 1u)
                             : -((-num + (int32_t)(len - 1u) / 2) / (int32_t)(len - 1u));
        arr[i] = (uint16_t)((int32_t)steps - 1 + d + shift);
    }
    /* shuffle (Fisher-Yates, xorshift) */
    for (i = len - 1u; i > 0u; i--) {
        uint32_t j;
        uint16_t t;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        j = x % (i + 1u);
        t = arr[i];
        arr[i] = arr[j];
        arr[j] = t;
    }
}

int32_t stagger_slew_straight(uint32_t cur, uint32_t tgt, int32_t pending, uint32_t max_step)
{
    int32_t rem = (int32_t)tgt - (int32_t)cur - pending;
    if (rem >= -DEADBAND && rem <= DEADBAND) {
        return 0;
    }
    if (rem > (int32_t)max_step) rem = (int32_t)max_step;
    if (rem < -(int32_t)max_step) rem = -(int32_t)max_step;
    return rem;
}

/* ---------------------------------------------------------------------------------------------- slew */
MEASURED int32_t stagger_slew(uint32_t cur, uint32_t tgt, int32_t pending, uint32_t steps, uint32_t max_step,
                     int allow_longer)
{
    int32_t s = (int32_t)steps, e, rem;
    e = (int32_t)(((tgt % steps) + steps - (cur % steps)) % steps);   /* 0..steps-1: wrap this much later */
    if (e <= DEADBAND || e >= s - DEADBAND) {
        e = 0;
    } else if (!allow_longer || e > s / 2) {
        e -= s;                                                       /* earlier instead */
    }
    rem = e - pending;
    if (rem >= -DEADBAND && rem <= DEADBAND) {
        return 0;
    }
    if (rem > (int32_t)max_step) rem = (int32_t)max_step;
    if (rem < -(int32_t)max_step) rem = -(int32_t)max_step;
    if (!allow_longer && rem > 0) rem = 0;
    return rem;
}
