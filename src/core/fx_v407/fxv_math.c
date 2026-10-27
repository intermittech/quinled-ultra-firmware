/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 effects layer's maths (fxv_api.h): seeded value noise and its walker, the 1-D field steps (diffusion, waves,
 * springs, cellular automata, reaction-diffusion). Integer only; JS: fx/v407/common.js gives the same values. */
#include "fxv_gen.h"

static int32_t lat(uint32_t seed, uint32_t xi, uint32_t ti)   /* a lattice value, -32768 .. 32767 */
{
    return (int32_t)(fx_hash(xi * 0x9E3779B1u + ti * 0x85EBCA6Bu + seed) >> 16) - 32768;
}

static int32_t lerp_s(int32_t a, int32_t b, uint32_t f16)       /* a -> b by f 0..65535 */
{
    return a + (((b - a) * (int32_t)(f16 >> 1)) >> 15);
}

static int32_t col(uint32_t seed, uint32_t xi, uint32_t ti, uint32_t tf)   /* the lattice column at xi, blended in t */
{
    return lerp_s(lat(seed, xi, ti), lat(seed, xi, ti + 1u), tf);
}

static int32_t mix_octaves(const int32_t *v, uint32_t octaves)
{
    const int32_t r = octaves == 1u ? v[0] : octaves == 2u ? ((2 * v[0] + v[1]) * 21845) >> 16
                                                          : ((4 * v[0] + 2 * v[1] + v[2]) * 9362) >> 16;
    return r < -32767 ? -32767 : r > 32767 ? 32767 : r;
}

/* octave o + 1: twice the frequency, elsewhere on the lattice */
#define NZ_X(x) (((x) << 1) + 0x3C6EF372u)
#define NZ_T(t) (((t) << 1) + 0x1B873593u)
#define NZ_S(s) ((s) + 0x9E3779B9u)

int32_t fxv_noise1_q15(uint32_t seed, uint32_t x, uint32_t t, uint32_t octaves)
{
    int32_t v[3] = { 0, 0, 0 };
    uint32_t o;
    if (octaves < 1u) octaves = 1u;
    if (octaves > 3u) octaves = 3u;
    for (o = 0; o < octaves; o++) {
        const uint32_t xi = x >> 16, ti = t >> 16, tf = fxv_smooth16(t & 0xFFFFu);
        v[o] = lerp_s(col(seed, xi, ti, tf), col(seed, xi + 1u, ti, tf), fxv_smooth16(x & 0xFFFFu));
        x = NZ_X(x);
        t = NZ_T(t);
        seed = NZ_S(seed);
    }
    return mix_octaves(v, octaves);
}

void fxv_nz_begin(fxv_nz_t *z, uint32_t seed, uint32_t t, uint32_t octaves)
{
    uint32_t o;
    if (octaves < 1u) octaves = 1u;
    if (octaves > 3u) octaves = 3u;
    z->oct = (uint8_t)octaves;
    for (o = 0; o < 3u; o++) {
        z->seed[o] = seed;
        z->t[o] = t;
        z->tf[o] = (uint16_t)fxv_smooth16(t & 0xFFFFu);
        z->ok[o] = 0;
        z->xi[o] = 0;
        z->a[o] = z->b[o] = 0;
        t = NZ_T(t);
        seed = NZ_S(seed);
    }
}

int32_t fxv_nz_at(fxv_nz_t *z, uint32_t x)
{
    int32_t v[3] = { 0, 0, 0 };
    uint32_t o;
    for (o = 0; o < z->oct; o++) {
        const uint32_t xi = x >> 16, ti = z->t[o] >> 16;
        if (!z->ok[o] || z->xi[o] != xi) {
            if (z->ok[o] && z->xi[o] + 1u == xi) z->a[o] = z->b[o];
            else z->a[o] = col(z->seed[o], xi, ti, z->tf[o]);
            z->b[o] = col(z->seed[o], xi + 1u, ti, z->tf[o]);
            z->xi[o] = xi;
            z->ok[o] = 1;
        }
        v[o] = lerp_s(z->a[o], z->b[o], fxv_smooth16(x & 0xFFFFu));
        x = NZ_X(x);
    }
    return mix_octaves(v, z->oct);
}

/* ------------------------------------------------------------------------------- 1-D fields */
static int32_t at16(const int16_t *s, uint32_t n, int32_t i, uint32_t edge)   /* the cell i reads (i = -1 or n) */
{
    if (i >= 0 && (uint32_t)i < n) return s[i];
    if (edge == FXV_EDGE_ZERO) return 0;
    if (edge == FXV_EDGE_WRAP) return s[i < 0 ? n - 1u : 0u];
    return s[i < 0 ? 0u : n - 1u];
}

static uint32_t at8(const uint8_t *s, uint32_t n, int32_t i, uint32_t edge)
{
    if (i >= 0 && (uint32_t)i < n) return s[i];
    if (edge == FXV_EDGE_ZERO) return 0;
    if (edge == FXV_EDGE_WRAP) return s[i < 0 ? n - 1u : 0u];
    return s[i < 0 ? 0u : n - 1u];
}

#define LAP(s, i) (at16((s), n, (int32_t)(i) - 1, edge) - 2 * (int32_t)(s)[i] + at16((s), n, (int32_t)(i) + 1, edge))

void fxv_diffuse(const int16_t *s, int16_t *d, uint32_t n, uint32_t coef, uint32_t from, uint32_t to, uint32_t edge)
{
    uint32_t i;
    if (coef > 1024u) coef = 1024u;
    if (to > n) to = n;
    for (i = from; i < to; i++) d[i] = (int16_t)fxv_sat16(s[i] + (((int32_t)coef * LAP(s, i)) >> 12));
}

void fxv_wave_step(const int16_t *prev, const int16_t *cur, int16_t *next, uint32_t n, uint32_t c2, uint32_t damp,
                   uint32_t from, uint32_t to, uint32_t edge)
{
    uint32_t i;
    if (c2 > 4000u) c2 = 4000u;                      /* (Courant 1 exactly: the alternating mode grows) */
    if (damp > 4096u) damp = 4096u;
    if (to > n) to = n;
    for (i = from; i < to; i++) {
        const int32_t vel = (int32_t)cur[i] - prev[i];
        next[i] = (int16_t)fxv_sat16(cur[i] + vel - ((vel * (int32_t)damp) >> 12) + (((int32_t)c2 * LAP(cur, i)) >> 12));
    }
}

void fxv_spring_step(int16_t *pos, int16_t *vel, uint32_t n, uint32_t k, uint32_t damp, uint32_t phase,
                     uint32_t from, uint32_t to, uint32_t edge)
{
    uint32_t i;
    if (k > 4096u) k = 4096u;
    if (damp > 4096u) damp = 4096u;
    if (to > n) to = n;
    for (i = from; i < to; i++) {
        if (phase == 0u) {
            vel[i] = (int16_t)fxv_sat16(vel[i] + (((int32_t)k * LAP(pos, i)) >> 12) - ((vel[i] * (int32_t)damp) >> 12));
        } else {
            pos[i] = (int16_t)fxv_sat16(pos[i] + vel[i]);
        }
    }
}

void fxv_ca_elem(const uint8_t *old, uint8_t *next, uint32_t n, uint32_t rule, uint32_t from, uint32_t to, uint32_t edge)
{
    uint32_t i;
    if (to > n) to = n;
    for (i = from; i < to; i++) {
        const uint32_t l = at8(old, n, (int32_t)i - 1, edge) != 0u, c = old[i] != 0u, r = at8(old, n, (int32_t)i + 1, edge) != 0u;
        next[i] = (uint8_t)((rule >> (l << 2 | c << 1 | r)) & 1u);
    }
}

void fxv_ca_cyclic(const uint8_t *old, uint8_t *next, uint32_t n, uint32_t states, uint32_t from, uint32_t to, uint32_t edge)
{
    uint32_t i;
    if (to > n) to = n;
    for (i = from; i < to; i++) {
        const uint32_t c = old[i], nx = c + 1u >= states ? 0u : c + 1u;
        next[i] = (uint8_t)(at8(old, n, (int32_t)i - 1, edge) == nx || at8(old, n, (int32_t)i + 1, edge) == nx ? nx : c);
    }
}

void fxv_ca_excitable(const uint8_t *old, uint8_t *next, uint32_t n, uint32_t states, uint32_t from, uint32_t to, uint32_t edge)
{
    uint32_t i;
    if (to > n) to = n;
    for (i = from; i < to; i++) {
        const uint32_t c = old[i];
        if (c == 0u) next[i] = (uint8_t)(at8(old, n, (int32_t)i - 1, edge) == 1u || at8(old, n, (int32_t)i + 1, edge) == 1u);
        else next[i] = (uint8_t)(c + 1u >= states ? 0u : c + 1u);
    }
}

static int16_t clamp12(int32_t v)
{
    return (int16_t)(v < 0 ? 0 : v > 4096 ? 4096 : v);
}

void fxv_rd_step(const int16_t *u, const int16_t *v, int16_t *un, int16_t *vn, uint32_t n, uint32_t du, uint32_t dv,
                 uint32_t feed, uint32_t kill, uint32_t from, uint32_t to, uint32_t edge)
{
    uint32_t i;
    if (du > 1024u) du = 1024u;
    if (dv > 1024u) dv = 1024u;
    if (feed > 4096u) feed = 4096u;
    if (kill > 4096u) kill = 4096u;
    if (to > n) to = n;
    for (i = from; i < to; i++) {
        const int32_t uu = u[i], vv = v[i], uvv = (uu * ((vv * vv) >> 12)) >> 12;
        un[i] = clamp12(uu + (((int32_t)du * LAP(u, i)) >> 12) - uvv + (((int32_t)feed * (4096 - uu)) >> 12));
        vn[i] = clamp12(vv + (((int32_t)dv * LAP(v, i)) >> 12) + uvv - ((((int32_t)feed + (int32_t)kill) * vv) >> 12));
    }
}
