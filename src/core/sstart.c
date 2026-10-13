/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Soft start: see sstart.h. */
#include "sstart.h"

void ss_init(sstart_t *s, uint32_t uid)
{
    s->state = SS_ARMED;
    s->offset_ms = (uint8_t)(uid % (SS_OFFSET_MAX_MS + 1u));
    s->t0 = 0;
}

void ss_retry(sstart_t *s, uint32_t now)
{
    s->state = SS_RUN;
    s->t0 = now;
}

uint32_t ss_factor(sstart_t *s, uint32_t now, uint8_t lit, uint8_t on)
{
    uint32_t el;
    if (s->state == SS_ARMED) {
        if (!lit) {
            return 65536u;
        }
        s->state = SS_RUN;                               /* the first light */
        s->t0 = now;
    }
    if (s->state != SS_RUN || !on) {
        s->state = SS_DONE;                              /* (turned off: a later retry runs it again if on) */
        return 65536u;
    }
    el = now - s->t0;
    if (el < s->offset_ms) {
        return 0;
    }
    el -= s->offset_ms;
    if (el >= SS_RAMP_MS) {
        s->state = SS_DONE;
        return 65536u;
    }
    return SS_START_Q16 + (65536u - SS_START_Q16) * el / SS_RAMP_MS;
}
