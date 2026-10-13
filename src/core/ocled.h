/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Overcurrent signal on the status LED D9 (Quindor 2026-10-13: "next to the actions taken also make the MCU LED blink
 * fast to signal the user"). A local signal only: nothing of it goes over the wire.
 *
 *   limit  fast blink, 100 ms on / 100 ms off (5 Hz): the board current limit is reducing the outputs (the user's
 *          limit, Ultra CONFIG 21 / stored ilimit, or the 8 A board rating). Held OCLED_LIMIT_HOLD_MS after it ends.
 *   trip   double flash, 2 x (100 ms on, 100 ms off), then 400 ms off (800 ms period): an overcurrent trip holds the
 *          outputs off (short or overload, through the retry waits). Held OCLED_TRIP_HOLD_MS after the outputs are
 *          back.
 * Each pattern starts with a flash; the holds make a brief event visible.
 *
 * Who drives D9 (ocled_led), highest first:
 *   1. trip: a fault, and the outputs are off anyway (an acknowledgement on the feedback channel isn't visible);
 *   2. acknowledge / identify / version (ack.h): short and asked for, the user waits for them;
 *   3. limit: comes back after them, with its hold;
 *   4. off (normal operation).
 * The bootloader's update codes run without the app, so they never meet these.
 * Pure logic, host-tested (test/test_ocled.c). */
#ifndef OCLED_H
#define OCLED_H

#include <stdint.h>

#define OCLED_LIMIT_ON_MS    100u
#define OCLED_LIMIT_OFF_MS   100u
#define OCLED_TRIP_ON_MS     100u     /* each of the two flashes */
#define OCLED_TRIP_GAP_MS    100u     /* between them */
#define OCLED_TRIP_PERIOD_MS 800u
#define OCLED_LIMIT_HOLD_MS  3000u
#define OCLED_TRIP_HOLD_MS   10000u

enum { OCLED_OFF = 0, OCLED_LIMIT = 1, OCLED_TRIP = 2 };

typedef struct {
    uint8_t  show;                    /* OCLED_*: the pattern D9 shows (when it owns D9) */
    uint8_t  limiting;                /* the conditions as last seen */
    uint8_t  trip_cause;              /* 0 none, else the trip holding the outputs off (1 short, 2 overload) */
    uint8_t  last_cause;              /* the last trip's cause (its hold, the console) */
    uint8_t  hold_limit, hold_trip;   /* the condition was on and its hold hasn't run out */
    uint32_t limit_last_ms;           /* last ms the condition was on */
    uint32_t trip_last_ms;
    uint32_t since_ms;                /* show != OFF: when the pattern started */
    uint32_t phase_ms;                /* the pattern's origin (events and counters: pwrev.h) */
} ocled_t;

void ocled_init(ocled_t *o);
/* every ms: limiting = a board current limit reduces the outputs now; trip_cause = a trip holds them off now
 * (0 none, 1 short, 2 overload) */
void ocled_update(ocled_t *o, uint32_t now, uint8_t limiting, uint8_t trip_cause);
/* the pattern's level at now (0 when nothing is shown) */
uint8_t ocled_level(const ocled_t *o, uint32_t now);
/* D9: trip > ack_active (acknowledge / identify / version at level ack_on) > limit > off */
uint8_t ocled_led(const ocled_t *o, uint32_t now, uint8_t ack_active, uint8_t ack_on);
/* hold left in ms of the shown pattern: 0 while its condition is still on, or when nothing is shown */
uint32_t ocled_hold_left(const ocled_t *o, uint32_t now);

#endif
