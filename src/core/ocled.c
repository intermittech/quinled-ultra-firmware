/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Overcurrent signal on status LED: see ocled.h. */
#include "ocled.h"

void ocled_init(ocled_t *o)
{
    uint8_t *b = (uint8_t *)o;
    uint32_t i;
    for (i = 0; i < sizeof(*o); i++) {
        b[i] = 0;
    }
}

void ocled_update(ocled_t *o, uint32_t now, uint32_t now_s, uint8_t limiting, uint8_t trip_cause)
{
    uint8_t want;
    if (trip_cause != 0u) {
        o->last_cause = trip_cause;
        o->trip_last_ms = now;
        o->hold_trip = 1;
    } else if (o->hold_trip && now - o->trip_last_ms >= OCLED_TRIP_HOLD_MS) {
        o->hold_trip = 0;                  /* run out (and no false hold when the ms counter wraps much later) */
    }
    o->trip_cause = trip_cause;
    if (limiting) {
        o->limit_last_ms = now;
        o->hold_limit = 1;
    } else if (o->hold_limit && now - o->limit_last_ms >= OCLED_LIMIT_HOLD_MS) {
        o->hold_limit = 0;
    }
    o->limiting = limiting ? 1u : 0u;
    want = o->hold_trip ? OCLED_TRIP : o->hold_limit ? OCLED_LIMIT : OCLED_OFF;
    if (want != o->show) {
        o->show = want;
        o->phase_ms = now;                 /* a new pattern starts with a flash */
        o->since_s = now_s;
    }
}

uint8_t ocled_level(const ocled_t *o, uint32_t now)
{
    uint32_t t = now - o->phase_ms;
    if (o->show == OCLED_LIMIT) {
        return (uint8_t)(t % (OCLED_LIMIT_ON_MS + OCLED_LIMIT_OFF_MS) < OCLED_LIMIT_ON_MS);
    }
    if (o->show == OCLED_TRIP) {
        t %= OCLED_TRIP_PERIOD_MS;
        return (uint8_t)(t < OCLED_TRIP_ON_MS ||
                         (t >= OCLED_TRIP_ON_MS + OCLED_TRIP_GAP_MS && t < 2u * OCLED_TRIP_ON_MS + OCLED_TRIP_GAP_MS));
    }
    return 0;
}

uint8_t ocled_led(const ocled_t *o, uint32_t now, uint8_t ack_active, uint8_t ack_on)
{
    if (o->show == OCLED_TRIP) {
        return ocled_level(o, now);
    }
    if (ack_active) {
        return (uint8_t)(ack_on != 0u);
    }
    return ocled_level(o, now);
}

uint32_t ocled_hold_left(const ocled_t *o, uint32_t now)
{
    uint32_t last, hold;
    if (o->show == OCLED_TRIP) {
        if (o->trip_cause != 0u) {
            return 0;
        }
        last = o->trip_last_ms;
        hold = OCLED_TRIP_HOLD_MS;
    } else if (o->show == OCLED_LIMIT) {
        if (o->limiting) {
            return 0;
        }
        last = o->limit_last_ms;
        hold = OCLED_LIMIT_HOLD_MS;
    } else {
        return 0;
    }
    return now - last >= hold ? 0u : hold - (now - last);
}
