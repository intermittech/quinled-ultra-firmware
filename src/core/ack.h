/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Acknowledge patterns on D9 and the feedback channel (DESIGN_NOTES "Acknowledge pattern").
 *   ACK          3 x (100 ms on, 100 ms off), then 800 ms pause and the value code
 *   value code   value + 1 blinks: long (700 ms) = 5, short (200 ms) = 1, 300 ms off between
 *   SAME         one 200 ms flash          NACK   one 1.5 s flash
 *   factory      ACK twice                 IDENTIFY 500 / 500 ms until stopped (30 s max)
 * Every pattern ends with 1 s off, then normal output resumes. */
#ifndef ACK_H
#define ACK_H

#include <stdint.h>

#define ACK_MAX_SEG 40u

typedef struct {
    uint16_t ms[ACK_MAX_SEG];   /* segment lengths, levels alternate on / off starting with on */
    uint8_t  nseg;
    uint8_t  seg;
    uint16_t left;              /* ms left in the current segment */
    uint8_t  active;
    uint8_t  repeat;            /* identify: loop the 2 segments */
    uint16_t repeat_left_s;
} ack_t;

typedef enum { ACK_P_ACK, ACK_P_SAME, ACK_P_NACK, ACK_P_FACTORY, ACK_P_IDENTIFY } ack_kind_t;

void    ack_start(ack_t *a, ack_kind_t kind, uint8_t value_code);
void    ack_stop(ack_t *a);
/* call every 1 ms; returns 1 while a pattern runs; *on = LED / feedback channel level */
uint8_t ack_tick(ack_t *a, uint8_t *on);

#endif
