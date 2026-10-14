/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Power events: the board limit and overcurrent state, the events with counters, ages and currents, and the
 * controller's acknowledge, in one struct (protocol 0.2.2: STATUS byte 8 ULTRA_ST_STATE, POWER page 0x08 ULTRA_PW_*;
 * quinled-ultra-protocol wiki "Return channel").
 *
 * Updated every ms from the limiter and trip state (main.c) and shown by the console's `status`. The dig2analog-ultra
 * can't report back, so there it stays local. A QuinLED Diff-Power ultra receiver answers its STATUS and POWER polls
 * from it (pwrev_state, pwrev_page, pwrev_ack), so the controller can show it, e.g. in Home Assistant.
 *
 * Rules (from the spec):
 *  - LIMITING / CH_LIMITING: a reduction now, held ULTRA_PW_HOLD_MS (2 s) after the last one;
 *  - a limit event starts when the limiter reduces after 2 s without reducing (one long overload = one event);
 *  - TRIPPED: the outputs are off after a trip, until the retry;
 *  - each event sets a latch independently of counter wrap; an ACK clears it only when its count matches the
 *    current count. Unmatched / delayed ACKs neither invent events nor acknowledge future ones;
 *  - everything is in RAM: a power cycle clears it.
 * Pure logic, host-tested (test/test_pwrev.c). */
#ifndef PWREV_H
#define PWREV_H

#include <stdint.h>
#include "ultra_protocol.h"
#include "climit.h"

#define PWREV_PAGE_BYTES 24u          /* POWER page DATA */
#define PWREV_AGE_NONE   0xFFFFFFu    /* u24 age: no event since power-up */

typedef struct {                      /* the inputs of one ms */
    uint8_t  limiting;                /* the board limit reduces the outputs now */
    uint8_t  ch_limiting;             /* a channel limit (CONFIG 16-20) reduces a channel now */
    uint8_t  trip_cause;              /* ULTRA_PW_CAUSE_*: a trip holds the outputs off now (0 = none) */
    uint16_t i_ma;                    /* board current now, mA (measured) */
    uint16_t trip_ma;                 /* the current to record for a trip, mA (short circuit: the peak sample) */
    uint16_t limit_ma;                /* the board limit in effect, mA (0xFFFF = none) */
    uint8_t  limit_src;               /* ULTRA_PW_SRC_* */
    uint16_t factor;                  /* the limiter's scale now, 0..65535 = 0..1 (65535 = no reduction) */
    uint32_t retry_ms;                /* ms until the retry after a trip (0 = not tripped) */
    uint16_t dips;                    /* supply dips since power-up (core/vdip.h; 0.2.3) */
} pwrev_in_t;

typedef struct {
    pwrev_in_t in;                    /* the last inputs */
    uint8_t  limit_on, ch_on;         /* LIMITING / CH_LIMITING: a reduction within the last ULTRA_PW_HOLD_MS */
    uint32_t limit_seen_ms, ch_seen_ms;   /* the last ms with a reduction */
    uint16_t limit_events, ovl_trips, short_trips;   /* since power-up, wrap */
    uint16_t ack_limit, ack_ovl, ack_short, ack_dips;   /* the counts the controller acknowledged */
    uint32_t limit_age_s, trip_age_s; /* s since the last limit event started / the last trip (PWREV_AGE_NONE) */
    uint16_t event_ma;                /* board current when the last limit event started, mA */
    uint16_t trip_ma;                 /* current at the last trip, mA */
    uint8_t  last_cause;              /* ULTRA_PW_CAUSE_* of the last trip */
    uint8_t  ev;                      /* (0.6.5, D2) the events the last update counted (ULTRA_PW_EV_*: a limit event,
                                         a trip, a new supply dip from in.dips): main.c's event log */
    uint8_t  latched;                 /* sticky events, including a full unacknowledged u16 counter cycle */
    uint32_t trip_ms;                 /* ms of the last trip (its peak can land in the next ms' samples) */
    uint32_t sec_ms;                  /* the ages count whole seconds from here */
} pwrev_t;

void    pwrev_init(pwrev_t *p, uint32_t now);
/* every ms */
void    pwrev_update(pwrev_t *p, uint32_t now, const pwrev_in_t *in);
/* ULTRA_PW_EV_* the controller hasn't acknowledged */
uint8_t pwrev_latched(const pwrev_t *p);
/* STATUS byte 8: ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_POWER_EVENT | ULTRA_ST_CH_LIMITING */
uint8_t pwrev_state(const pwrev_t *p);
/* POWER page DATA */
void    pwrev_page(const pwrev_t *p, uint8_t out[PWREV_PAGE_BYTES]);
/* POWER page POLL arguments, slot bytes 7..15: flags, then the four counts the controller read (u16 each) */
void    pwrev_ack(pwrev_t *p, const uint8_t arg[9]);
/* the board limit's source (core/climit.h CLIMIT_SRC_*) -> ULTRA_PW_SRC_* (inline: main.c asks every ms) */
static inline uint8_t pwrev_src(uint8_t climit_src)
{
    return climit_src == CLIMIT_SRC_USER ? ULTRA_PW_SRC_USER
         : climit_src == CLIMIT_SRC_FUSE ? ULTRA_PW_SRC_FUSE : ULTRA_PW_SRC_BOARD;
}

#endif
