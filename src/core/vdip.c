/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Supply dips: see vdip.h. */
#include "vdip.h"
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
    x = sum2 / n;
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

static uint32_t age_s(uint32_t now_s, uint32_t t_s)
{
    uint32_t s = now_s - t_s;
    return s > 0xFFFFFEu ? 0xFFFFFEu : s;
}

void vdip_page(const vdip_t *d, const vsense_t *vs, uint32_t now_s, uint32_t now_mv, uint8_t arg, uint8_t out[24])
{
    const vdip_ev_t *e = vdip_event(d, arg);
    uint32_t i;
    for (i = 0; i < 24u; i++) {
        out[i] = 0;
    }
    ultra_put16(out + ULTRA_SU_DIPS, d->count);
    ultra_put16(out + ULTRA_SU_REF_MV, d->ref_q12 ? vsense_mv(vs, d->ref_q12 >> 8, 16u) : 0u);
    ultra_put16(out + ULTRA_SU_NOW_MV, now_mv > 65535u ? 65535u : now_mv);
    if (d->deep.ref_x) {
        ultra_put16(out + ULTRA_SU_DEEP_MV, vsense_mv(vs, d->deep.min_x, 1u));
        out[ULTRA_SU_DEEP_PCT] = (uint8_t)vdip_pct(&d->deep);
    }
    ultra_put24(out + ULTRA_SU_DEEP_AGE, d->deep.ref_x ? age_s(now_s, d->deep.t_s) : 0xFFFFFFu);
    out[ULTRA_SU_LOGGED] = d->n_log;
    ultra_put24(out + ULTRA_SU_EV_AGE, e ? age_s(now_s, e->t_s) : 0xFFFFFFu);
    if (e) {
        ultra_put16(out + ULTRA_SU_EV_MV, vsense_mv(vs, e->min_x, 1u));
        out[ULTRA_SU_EV_PCT] = (uint8_t)vdip_pct(e);
        ultra_put16(out + ULTRA_SU_EV_MS, e->dur_ms);
        ultra_put16(out + ULTRA_SU_EV_MA, e->i_ma);
    }
}
