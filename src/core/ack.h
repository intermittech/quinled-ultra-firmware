/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Acknowledge patterns on D9 and the feedback channel (wiki: WS modes and config frames).
 *   ACK          3 x (100 ms on, 100 ms off), then 800 ms pause and the value code
 *   value code   value + 1 blinks: long (700 ms) = 5, short (200 ms) = 1, 300 ms off between
 *   SAME         one 200 ms flash          NACK   one 1.5 s flash
 *   factory      ACK twice                 IDENTIFY 500 / 500 ms until stopped (30 s max)
 *   fault        D9 only (0.6.0, core/health.h): one 1.5 s flash, 800 ms off, then the code as short blinks
 *   version      3 fast, 800 ms, then major, minor, patch as value codes 1.2 s apart, then the pre-release
 *                (alpha 1 short, beta 2 short, rc 3 short; nothing for stable). Unknown (0, a development build):
 *                3 fast, then one 1.5 s flash (wiki: Console).
 * Every pattern ends with 1 s off, then normal output resumes.
 * D9 is shared with the overcurrent signal (ocled.h): a trip's double flash overrides these on D9, and these override
 * the limit blink. The feedback channel shows only these. */
#ifndef ACK_H
#define ACK_H

#include <stdint.h>

#define ACK_MAX_SEG 64u                 /* the longest: the version, 6 + 3 x 12 + 6 = 48 */

typedef struct {
    uint16_t ms[ACK_MAX_SEG];   /* segment lengths, levels alternate on / off starting with on */
    uint8_t  nseg;
    uint8_t  seg;
    uint16_t left;              /* ms left in the current segment */
    uint8_t  active;
    uint8_t  repeat;            /* identify: loop the 2 segments */
    uint8_t  led_only;          /* D9 only, not the feedback channel (the fault code) */
    uint16_t repeat_left_s;
} ack_t;

typedef enum { ACK_P_ACK, ACK_P_SAME, ACK_P_NACK, ACK_P_FACTORY, ACK_P_IDENTIFY, ACK_P_FAULT } ack_kind_t;

void    ack_start(ack_t *a, ack_kind_t kind, uint8_t value_code);
/* the firmware version (fwver.h word; each part shown up to 29) */
void    ack_start_version(ack_t *a, uint32_t version);
void    ack_stop(ack_t *a);
/* call every 1 ms; returns 1 while a pattern runs; *on = LED / feedback channel level */
uint8_t ack_tick(ack_t *a, uint8_t *on);

#endif
