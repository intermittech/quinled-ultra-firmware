/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Supply dips: see vdip.h. */
#include "vdip.h"
#include "rc.h"                             /* rc_pack */
#include "lto.h"

void vdip_init(vdip_t *d)
{
    uint8_t *b = (uint8_t *)d;
    uint32_t i;
    for (i = 0; i < sizeof(*d); i++) {
        b[i] = 0;
    }
}

MEASURED void vdip_ms(vdip_t *d, uint32_t now, uint32_t now_s, uint32_t sum2, uint32_t n, int32_t i_ma)
{
    vdip_ev_t *c = &d->cur;
    uint32_t x, ref;
    if (n == 0u) {
        return;
    }
    x = n == 1u ? sum2 : sum2 / n;                       /* (one pair per ms: no division) */
    if (d->ref_q12 == 0u) {
        if (++d->run > VDIP_HOLDOFF_MS) {
            d->run = 0;
            d->ref_q12 = x << 12;                        /* the first reading after the hold-off */
        }
        return;
    }
    ref = d->ref_q12 >> 12;
    d->ref_q12 += x - ref;                               /* running mean, 1/4096 per ms */
    if (x * 100u < ref * (d->in_dip ? VDIP_LEAVE_PCT : VDIP_ENTER_PCT)) {
        d->zi1 = d->zi2 = 0xFFFFu;                       /* (the impedance: no settled window around a dip) */
        d->zok = 0;
        if (d->in_dip) {
            d->run = 0;                                  /* still down */
        } else {
            if (d->run == 0u) {                          /* a first low reading: a candidate */
                d->start_ms = now;
                c->t_s = now_s;
                c->ref_x = (uint16_t)ref;
                c->min_x = 0xFFFFu;
            }
            if (++d->run >= VDIP_MIN_MS) {
                d->in_dip = 1;
                d->run = 0;
                d->count++;
            }
        }
        if (x < c->min_x) {
            c->min_x = (uint16_t)x;
            c->i_ma = (uint16_t)(i_ma < 0 ? 0 : i_ma > 65535 ? 65535 : i_ma);
        }
        return;
    }
    if (!d->in_dip) {
        d->run = 0;                                      /* (a candidate that came back up is dropped) */
        return;
    }
    if (++d->run < VDIP_MIN_MS) {
        return;
    }
    /* back up for VDIP_MIN_MS: it ended at the first of those readings */
    d->in_dip = 0;
    d->run = 0;
    x = now + 1u - VDIP_MIN_MS - d->start_ms;
    c->dur_ms = (uint16_t)(x > 65535u ? 65535u : x);
    d->head = (uint8_t)((d->head + 1u) % VDIP_LOG);
    d->log[d->head] = *c;
    if (d->n_log < VDIP_LOG) {
        d->n_log++;
    }
    if (d->deep.ref_x == 0u || (uint32_t)c->min_x * d->deep.ref_x < (uint32_t)d->deep.min_x * c->ref_x) {
        d->deep = *c;                                    /* deeper: lower against its own reference */
    }
}

const vdip_ev_t *vdip_event(const vdip_t *d, uint32_t k)
{
    return k < d->n_log ? &d->log[(d->head + VDIP_LOG - k) % VDIP_LOG] : 0;
}

uint32_t vdip_pct(const vdip_ev_t *e)
{
    return e->ref_x ? ((uint32_t)e->min_x * 100u + e->ref_x / 2u) / e->ref_x : 0u;
}

MEASURED void vdip_z(vdip_t *d, const vsense_t *vs, uint32_t ma)
{
    int32_t i, tol, a, b;
    d->zma += ma;
    if (vs->ms != 0u) {
        return;                                          /* (not the end of a supply average) */
    }
    i = (int32_t)(d->zma / VSENSE_WIN_MS);
    d->zma = 0;
    if (d->ref_q12 == 0u) {
        return;                                          /* (the boot hold-off: PA4's RC still charging) */
    }
    tol = VDIP_Z_STEADY_MA + (i >> 8);
    a = i - d->zi1;
    b = (int32_t)d->zi1 - d->zi2;
    d->zi2 = d->zi1;
    d->zi1 = (uint16_t)i;
    if ((uint32_t)(a + tol) > 2u * (uint32_t)tol || (uint32_t)(b + tol) > 2u * (uint32_t)tol) {
        d->zok -= (uint8_t)(d->zok != 0u);                /* not settled */
        return;
    }
    a = i - d->zi_set;                                   /* the current's rise, the supply's fall (16-bit difference) */
    b = (int16_t)(uint16_t)(d->zv_set - vs->mv);
    if (a < 0) {
        a = -a;
        b = -b;
    }
    if (d->zok != 0u && a >= VDIP_Z_STEP_MA) {           /* a step */
        d->znum += b;
        d->zden += (uint32_t)a;
        if (d->zden >= VDIP_Z_DEN_MAX) {
            d->znum >>= 1;
            d->zden >>= 1;
        }
        d->zsteps += (uint16_t)(d->zsteps != 0xFFFFu);
    }
    d->zi_set = (uint16_t)i;
    d->zv_set = (uint16_t)vs->mv;
    d->zok = VDIP_Z_GAP;
}

uint32_t vdip_z_mohm(const vdip_t *d)
{
    /* mV / mA x 1000: x 125 / (den / 8) stays in 32 bits (num < 2^21) to within 0.4 % of 4 steps' den */
    int32_t r = d->zsteps >= VDIP_Z_MIN_STEPS ? d->znum * 125 / (int32_t)(d->zden >> 3) : 0;
    return r > 0 ? (uint32_t)r : 0u;
}

static uint32_t age_s(uint32_t now_s, uint32_t t_s)
{
    uint32_t s = now_s - t_s;
    return s > 0xFFFFFEu ? 0xFFFFFEu : s;
}

void vdip_page(const vdip_t *d, const vsense_t *vs, uint32_t now_s, uint32_t now_mv, uint8_t arg, uint8_t out[24])
{
    static const uint8_t len[13] = { 2, 2, 2, 2, 1, 3, 1, 3, 2, 1, 2, 2, 1 };   /* the ULTRA_SU_* fields, DIPS .. 23 */
    const vdip_ev_t *e = vdip_event(d, arg);
    uint32_t v[13] = { 0 };
    v[0] = d->count;
    v[1] = d->ref_q12 ? vsense_mv(vs, d->ref_q12 >> 8, 16u) : 0u;
    v[2] = now_mv > 65535u ? 65535u : now_mv;
    v[5] = 0xFFFFFFu;
    if (d->deep.ref_x) {
        v[3] = vsense_mv(vs, d->deep.min_x, 1u);
        v[4] = vdip_pct(&d->deep);
        v[5] = age_s(now_s, d->deep.t_s);
    }
    v[6] = d->n_log;
    v[7] = 0xFFFFFFu;
    if (e) {
        v[7] = age_s(now_s, e->t_s);
        v[8] = vsense_mv(vs, e->min_x, 1u);
        v[9] = vdip_pct(e);
        v[10] = e->dur_ms;
        v[11] = e->i_ma;
    }
    rc_pack(out, v, len, 13);                          /* (byte 23 reserved: 0) */
}
