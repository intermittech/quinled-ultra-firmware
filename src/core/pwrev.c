/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Power events: see pwrev.h. */
#include "pwrev.h"
#include "climit.h"

void pwrev_init(pwrev_t *p, uint32_t now)
{
    uint8_t *b = (uint8_t *)p;
    uint32_t i;
    for (i = 0; i < sizeof(*p); i++) {
        b[i] = 0;
    }
    p->in.limit_ma = 0xFFFFu;
    p->in.factor = 65535u;
    p->limit_age_s = p->trip_age_s = PWREV_AGE_NONE;
    p->sec_ms = now;
}

void pwrev_update(pwrev_t *p, uint32_t now, const pwrev_in_t *in)
{
    /* the ages, in whole seconds (PWREV_AGE_NONE stays; 0xFFFFFE = 194 days or more) */
    while (now - p->sec_ms >= 1000u) {
        p->sec_ms += 1000u;
        if (p->limit_age_s < PWREV_AGE_NONE - 1u) {
            p->limit_age_s++;
        }
        if (p->trip_age_s < PWREV_AGE_NONE - 1u) {
            p->trip_age_s++;
        }
    }

    /* the board limit: a reduction after ULTRA_PW_HOLD_MS without one starts an event */
    if (in->limiting) {
        if (!p->limit_on) {
            p->limit_events++;
            p->limit_age_s = 0;
            p->event_ma = in->i_ma;
        }
        p->limit_on = 1;
        p->limit_seen_ms = now;
    } else if (p->limit_on && now - p->limit_seen_ms >= ULTRA_PW_HOLD_MS) {
        p->limit_on = 0;                   /* (cleared here, so the ms counter's wrap can't fake a hold) */
    }
    if (in->ch_limiting) {
        p->ch_on = 1;
        p->ch_seen_ms = now;
    } else if (p->ch_on && now - p->ch_seen_ms >= ULTRA_PW_HOLD_MS) {
        p->ch_on = 0;
    }

    /* trips */
    if (in->trip_cause != ULTRA_PW_CAUSE_NONE) {
        if (p->in.trip_cause == ULTRA_PW_CAUSE_NONE) {
            if (in->trip_cause == ULTRA_PW_CAUSE_SHORT) {
                p->short_trips++;
            } else {
                p->ovl_trips++;
            }
            p->last_cause = in->trip_cause;
            p->trip_age_s = 0;
            p->trip_ms = now;
            p->trip_ma = in->trip_ma;
        } else if (now - p->trip_ms <= 2u && in->trip_ma > p->trip_ma) {
            p->trip_ma = in->trip_ma;      /* a short's peak sample can land in the next ms' take */
        }
    }
    p->in = *in;
}

uint8_t pwrev_latched(const pwrev_t *p)
{
    return (uint8_t)((p->limit_events != p->ack_limit ? ULTRA_PW_EV_LIMITED : 0u) |
                     (p->ovl_trips != p->ack_ovl ? ULTRA_PW_EV_OVERLOAD : 0u) |
                     (p->short_trips != p->ack_short ? ULTRA_PW_EV_SHORT : 0u) |
                     (p->in.dips != p->ack_dips ? ULTRA_PW_EV_SUPPLY_DIP : 0u));
}

uint8_t pwrev_state(const pwrev_t *p)
{
    return (uint8_t)((p->limit_on ? ULTRA_ST_LIMITING : 0u) |
                     (p->in.trip_cause != ULTRA_PW_CAUSE_NONE ? ULTRA_ST_TRIPPED : 0u) |
                     (pwrev_latched(p) ? ULTRA_ST_POWER_EVENT : 0u) |
                     (p->ch_on ? ULTRA_ST_CH_LIMITING : 0u));
}

void pwrev_page(const pwrev_t *p, uint8_t out[PWREV_PAGE_BYTES])
{
    uint8_t tripped = (uint8_t)(p->in.trip_cause != ULTRA_PW_CAUSE_NONE);
    uint32_t r = 0;
    if (tripped) {
        r = (p->in.retry_ms + 999u) / 1000u;
        r = r < 1u ? 1u : r > 255u ? 255u : r;
    }
    out[ULTRA_PW_NOW] = (uint8_t)(pwrev_state(p) & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_CH_LIMITING));
    out[ULTRA_PW_LATCHED] = pwrev_latched(p);
    ultra_put16(out + ULTRA_PW_LIMIT_EVENTS, p->limit_events);
    ultra_put16(out + ULTRA_PW_OVL_TRIPS, p->ovl_trips);
    ultra_put16(out + ULTRA_PW_SHORT_TRIPS, p->short_trips);
    ultra_put24(out + ULTRA_PW_LIMIT_AGE, p->limit_age_s);
    ultra_put24(out + ULTRA_PW_TRIP_AGE, p->trip_age_s);
    ultra_put16(out + ULTRA_PW_LIMIT_MA, p->in.limit_ma);
    out[ULTRA_PW_SRC_CAUSE] = (uint8_t)(((p->in.limit_src & 0x0Fu) << 4) | (p->last_cause & 0x0Fu));
    ultra_put16(out + ULTRA_PW_EVENT_MA, p->event_ma);
    ultra_put16(out + ULTRA_PW_TRIP_MA, p->trip_ma);
    ultra_put16(out + ULTRA_PW_FACTOR, tripped ? 0u : p->in.factor);
    out[ULTRA_PW_RETRY_S] = (uint8_t)r;
}

void pwrev_ack(pwrev_t *p, const uint8_t arg[9])
{
    if (!(arg[0] & ULTRA_PW_ARG_ACK)) {
        return;
    }
    p->ack_limit = (uint16_t)ultra_get16(arg + 1);
    p->ack_ovl = (uint16_t)ultra_get16(arg + 3);
    p->ack_short = (uint16_t)ultra_get16(arg + 5);
    p->ack_dips = (uint16_t)ultra_get16(arg + 7);
}

uint8_t pwrev_src(uint8_t climit_src)
{
    return climit_src == CLIMIT_SRC_USER ? ULTRA_PW_SRC_USER
         : climit_src == CLIMIT_SRC_FUSE ? ULTRA_PW_SRC_FUSE : ULTRA_PW_SRC_BOARD;
}
