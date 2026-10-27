/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Power events: see pwrev.h. */
#include "pwrev.h"
#include "rc.h"                            /* rc_pack */
#include "../board/boards.h"

/* Non-PWM boards supply no channel-limit, trip or retry state. Those fields
 * retain their initialized wire values while limit and supply events stay live. */
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
    uint8_t ev = (uint8_t)(in->dips != p->in.dips ? ULTRA_PW_EV_SUPPLY_DIP : 0u);   /* counted now */
    /* the ages, in whole seconds (PWREV_AGE_NONE stays; 0xFFFFFE = 194 days or more) */
    while (now - p->sec_ms >= 1000u) {
        p->sec_ms += 1000u;
        if (p->limit_age_s < PWREV_AGE_NONE - 1u) {
            p->limit_age_s++;
        }
#if BOARD_PWM
        if (p->trip_age_s < PWREV_AGE_NONE - 1u) {
            p->trip_age_s++;
        }
#endif
    }

    /* the board limit: a reduction after ULTRA_PW_HOLD_MS without one starts an event; not within
     * ULTRA_PW_HOLD_MS after a trip: its retry starts scaled down (the current at the trip lands at the limit), a
     * reduction that is the trip's, counted already (else each retry would count a limit event, its current measured
     * while the outputs were off) */
    if (in->limiting) {
        if (!p->limit_on
#if BOARD_PWM
            && !p->after_trip
#endif
        ) {
            p->limit_events++;
            p->limit_age_s = 0;
            p->event_ma = in->i_ma;
            ev |= ULTRA_PW_EV_LIMITED;
        }
        p->limit_on = 1;
        p->limit_seen_ms = now;
    } else if (p->limit_on && now - p->limit_seen_ms >= ULTRA_PW_HOLD_MS) {
        p->limit_on = 0;                   /* (cleared here, so the ms counter's wrap can't fake a hold) */
    }
#if BOARD_PWM
    if (in->ch_limiting) {
        p->ch_on = 1;
        p->ch_seen_ms = now;
    } else if (p->ch_on && now - p->ch_seen_ms >= ULTRA_PW_HOLD_MS) {
        p->ch_on = 0;
    }

    /* trips */
    if (in->trip_cause != ULTRA_PW_CAUSE_NONE) {
        p->after_trip = ULTRA_PW_HOLD_MS;
        if (p->in.trip_cause == ULTRA_PW_CAUSE_NONE) {
            if (in->trip_cause == ULTRA_PW_CAUSE_SHORT) {
                p->short_trips++;
                ev |= ULTRA_PW_EV_SHORT;
            } else {
                p->ovl_trips++;
                ev |= ULTRA_PW_EV_OVERLOAD;
            }
            p->last_cause = in->trip_cause;
            p->trip_age_s = 0;
            p->trip_ms = now;
            p->trip_ma = in->trip_ma;
        } else if (now - p->trip_ms <= 2u && in->trip_ma > p->trip_ma) {
            p->trip_ma = in->trip_ma;      /* a short's peak sample can land in the next ms' take */
        }
    } else if (p->after_trip) {
        p->after_trip--;                   /* (a count, not a time: no wrap; a late ms only lengthens it) */
    }
#endif
    p->latched |= ev;
    p->ev = ev;                            /* (before the copy: it stays the tail call it was) */
    p->in = *in;
}

uint8_t pwrev_latched(const pwrev_t *p)
{
    return p->latched;
}

uint8_t pwrev_state(const pwrev_t *p)
{
    return (uint8_t)((p->limit_on ? ULTRA_ST_LIMITING : 0u) |
#if BOARD_PWM
                     (p->in.trip_cause != ULTRA_PW_CAUSE_NONE ? ULTRA_ST_TRIPPED : 0u) |
                     (p->ch_on ? ULTRA_ST_CH_LIMITING : 0u) |
#endif
                     (pwrev_latched(p) ? ULTRA_ST_POWER_EVENT : 0u));
}

void pwrev_page(const pwrev_t *p, uint8_t out[PWREV_PAGE_BYTES])
{
    uint32_t r = 0;
#if BOARD_PWM
    uint8_t tripped = (uint8_t)(p->in.trip_cause != ULTRA_PW_CAUSE_NONE);
    if (tripped) {
        r = (p->in.retry_ms + 999u) / 1000u;
        r = r < 1u ? 1u : r > 255u ? 255u : r;
    }
#else
    const uint8_t tripped = 0;
#endif
    {                                    /* the ULTRA_PW_* fields in order, NOW .. RETRY_S */
        const uint32_t len = RC_WIDTHS4(1, 1, 2, 2) | (RC_WIDTHS4(2, 3, 3, 2) << 8) | (RC_WIDTHS4(1, 2, 2, 2) << 16) | (RC_WIDTHS4(1, 1, 1, 1) << 24);
        uint32_t v[13];
        v[0] = pwrev_state(p) & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_CH_LIMITING);
        v[1] = pwrev_latched(p);
        v[2] = p->limit_events;
        v[3] = p->ovl_trips;
        v[4] = p->short_trips;
        v[5] = p->limit_age_s;
        v[6] = p->trip_age_s;
        v[7] = p->in.limit_ma;
        v[8] = ((p->in.limit_src & 0x0Fu) << 4) | (p->last_cause & 0x0Fu);
        v[9] = p->event_ma;
        v[10] = p->trip_ma;
        v[11] = tripped ? 0u : p->in.factor;
        v[12] = r;
        rc_pack(out, v, len, 13);
    }
}

void pwrev_ack(pwrev_t *p, const uint8_t arg[9])
{
    if (!(arg[0] & ULTRA_PW_ARG_ACK)) {
        return;
    }
    if (ultra_get16(arg + 1) == p->limit_events) {
        p->ack_limit = p->limit_events;
        p->latched &= (uint8_t)(0xFFu ^ ULTRA_PW_EV_LIMITED);
    }
#if BOARD_PWM
    if (ultra_get16(arg + 3) == p->ovl_trips) {
        p->ack_ovl = p->ovl_trips;
        p->latched &= (uint8_t)(0xFFu ^ ULTRA_PW_EV_OVERLOAD);
    }
    if (ultra_get16(arg + 5) == p->short_trips) {
        p->ack_short = p->short_trips;
        p->latched &= (uint8_t)(0xFFu ^ ULTRA_PW_EV_SHORT);
    }
#endif
    if (ultra_get16(arg + 7) == p->in.dips) {
        p->ack_dips = p->in.dips;
        p->latched &= (uint8_t)(0xFFu ^ ULTRA_PW_EV_SUPPLY_DIP);
    }
}
