/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 effects layer's particles and events (fxv_api.h): stepping, free records, bins (a counting sort by pixel),
 * kernel splats, a bounded index queue, event envelopes. Integer only; JS: fx/v407/common.js. */
#include "fxv_gen.h"
#include "../math32.h"

uint32_t fxv_particle_step(fxv_particle_t *p, uint32_t n, int32_t accel, uint32_t drag)
{
    uint32_t k, live = 0;
    for (k = 0; k < n; k++, p++) {
        if (p->life == 0u) continue;
        p->x = (int32_t)((uint32_t)p->x + (uint32_t)p->v);
        p->v = (int32_t)((uint32_t)p->v + (uint32_t)accel);
        if (drag != 0u) p->v -= (int32_t)m32_sar(p->v, drag, 16);
        if (p->age != 0xFFFFu) p->age++;
        if (p->age >= p->life) p->life = 0;
        else live++;
    }
    return live;
}

uint32_t fxv_particle_free(const fxv_particle_t *p, uint32_t n)
{
    uint32_t k;
    for (k = 0; k < n; k++) {
        if (p[k].life == 0u) return k;
    }
    return n;
}

uint32_t fxv_bins_count(uint32_t n, uint32_t shift)
{
    return (n >> shift) + 1u;
}

static uint32_t bin_of(int32_t x, uint32_t nb, uint32_t shift)
{
    const int32_t px = x >> 16;
    const uint32_t b = px < 0 ? 0u : (uint32_t)px >> shift;
    return b >= nb ? nb - 1u : b;
}

void fxv_bins_build(uint16_t *start, uint16_t *order, const fxv_particle_t *p, uint32_t pn, uint32_t n, uint32_t shift)
{
    const uint32_t nb = fxv_bins_count(n, shift);
    uint32_t k, b;
    for (b = 0; b <= nb; b++) start[b] = 0;
    for (k = 0; k < pn; k++) {
        if (p[k].life != 0u) start[bin_of(p[k].x, nb, shift) + 1u]++;
    }
    for (b = 1; b <= nb; b++) start[b] = (uint16_t)(start[b] + start[b - 1u]);
    for (k = 0; k < pn; k++) {                           /* start[b] runs as bin b's cursor: then it is bin b + 1's start */
        if (p[k].life != 0u) order[start[bin_of(p[k].x, nb, shift)]++] = (uint16_t)k;
    }
    for (b = nb; b > 0u; b--) start[b] = start[b - 1u];
    start[0] = 0;
}

void fxv_bins_span(const uint16_t *start, uint32_t n, uint32_t shift, int32_t lo, int32_t hi, uint32_t *a, uint32_t *b)
{
    const uint32_t nb = fxv_bins_count(n, shift);
    uint32_t b0 = lo < 0 ? 0u : (uint32_t)lo >> shift, b1 = hi <= 0 ? 0u : (uint32_t)(hi - 1) >> shift;
    if (b0 >= nb) b0 = nb - 1u;
    if (b1 >= nb) b1 = nb - 1u;
    if (hi <= lo) { *a = *b = start[b0]; return; }
    *a = start[b0];
    *b = start[b1 + 1u];
}

void fxv_splat(fxv_px_t *out, uint32_t from, uint32_t count, int32_t x, uint32_t r, fxv_px_t c)
{
    int32_t i, lo, hi;
    uint32_t kq;
    if (r < 32768u) r = 32768u;
    if (r > (64u << 16)) r = 64u << 16;
    if (x < ((int32_t)from - 65) * 65536 || x > ((int32_t)(from + count) + 65) * 65536) return;   /* (out of reach) */
    kq = (65535u << 8) / (r >> 8);                       /* one division a splat: w = ((r - d) >> 8) x kq >> 8 */
    lo = (x - (int32_t)r) >> 16;
    hi = ((x + (int32_t)r) >> 16) + 1;
    if (lo < (int32_t)from) lo = (int32_t)from;
    if (hi > (int32_t)(from + count)) hi = (int32_t)(from + count);
    for (i = lo; i < hi; i++) {
        const int32_t ci = i * 65536 + 32768, dd = ci - x;
        const uint32_t d = (uint32_t)(dd < 0 ? -dd : dd);
        uint32_t w;
        if (d >= r) continue;
        w = (((r - d) >> 8) * kq) >> 8;
        if (w > 65535u) w = 65535u;
        out[i - (int32_t)from] = fxv_add_px(out[i - (int32_t)from], fxv_scale_px(c, w));
    }
}

void fxv_queue_init(fxv_queue_t *q, uint16_t *buf, uint32_t cap)
{
    q->buf = buf;
    q->cap = (uint16_t)cap;
    q->head = 0;
    q->count = 0;
    q->spare = 0;
}

uint32_t fxv_queue_push(fxv_queue_t *q, uint32_t v)
{
    uint32_t i;
    if (q->count >= q->cap) return 0;
    i = (uint32_t)q->head + q->count;
    if (i >= q->cap) i -= q->cap;
    q->buf[i] = (uint16_t)v;
    q->count++;
    return 1;
}

uint32_t fxv_queue_pop(fxv_queue_t *q)
{
    uint32_t v;
    if (q->count == 0u) return 0xFFFFu;
    v = q->buf[q->head];
    q->head = (uint16_t)(q->head + 1u >= q->cap ? 0u : q->head + 1u);
    q->count--;
    return v;
}

uint32_t fxv_env(uint32_t t, uint32_t attack, uint32_t hold, uint32_t decay)
{
    if (t < attack) return fxv_smooth16(m32_muldiv(t, 65535u, attack));
    t -= attack;
    if (t < hold) return 65535u;
    t -= hold;
    if (t < decay) return fxv_smooth16(65535u - m32_muldiv(t, 65535u, decay));
    return 0;
}
