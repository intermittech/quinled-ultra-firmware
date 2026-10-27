/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "ack.h"

/* one on / off pair, in ACK_UNIT_MS (100 ms) */
static void add(ack_t *a, uint8_t on, uint8_t off)
{
    if (a->nseg + 2u <= ACK_MAX_SEG) {
        a->ms[a->nseg++] = on;
        a->ms[a->nseg++] = off;
    }
}

static void add_ack(ack_t *a, uint8_t last_off)
{
    add(a, 1, 1);
    add(a, 1, 1);
    add(a, 1, last_off);
}

static void add_code(ack_t *a, uint32_t value, uint8_t gap);

void ack_start(ack_t *a, ack_kind_t kind, uint8_t value_code)
{
    a->nseg = 0;
    a->repeat = 0;
    a->led_only = (uint8_t)(kind == ACK_P_FAULT);
    switch (kind) {
    case ACK_P_ACK:
        if (value_code == 0u) {
            add_ack(a, 10);
        } else {
            uint8_t longs = (uint8_t)(value_code / 5u), shorts = (uint8_t)(value_code % 5u), i;
            add_ack(a, 9);                            /* 100 ms off + 800 ms pause */
            for (i = 0; i < longs; i++) {
                add(a, 7, 3);
            }
            for (i = 0; i < shorts; i++) {
                add(a, 2, 3);
            }
            a->ms[a->nseg - 1u] = 10;                 /* final pause before normal output */
        }
        break;
    case ACK_P_FACTORY:
        add_ack(a, 7);
        add_ack(a, 10);
        break;
    case ACK_P_SAME:
        add(a, 2, 10);
        break;
    case ACK_P_NACK:
        add(a, 15, 10);
        break;
    case ACK_P_IDENTIFY:
        add(a, 5, 5);
        a->repeat = 1;
        a->repeat_left_s = 30;
        break;
    case ACK_P_FAULT:
        add(a, 15, 8);
        add_code(a, value_code - 1u, 10);             /* value_code blinks (6 or less: short ones, 6 = long + short) */
        break;
    }
    a->seg = 0;
    a->left = (uint16_t)(a->ms[0] * ACK_UNIT_MS);
    a->active = 1;
}

/* one value code (value + 1 blinks: long = 5, short = 1, 300 ms off between), its last off-time `gap` (x 100 ms) */
static void add_code(ack_t *a, uint32_t value, uint8_t gap)
{
    uint32_t code = (value > 29u ? 29u : value) + 1u, i;
    for (i = 0; i < code / 5u; i++) {
        add(a, 7, 3);
    }
    for (i = 0; i < code % 5u; i++) {
        add(a, 2, 3);
    }
    a->ms[a->nseg - 1u] = gap;
}

void ack_start_version(ack_t *a, uint32_t version)
{
    uint32_t pre = version & 0xFFu;
    a->nseg = 0;
    a->repeat = 0;
    a->led_only = 0;
    if (version == 0u) {
        add_ack(a, 9);                                /* a development build: 3 fast, then one long flash */
        add(a, 15, 10);
    } else {
        add_ack(a, 9);
        add_code(a, version >> 24, 12);               /* major */
        add_code(a, (version >> 16) & 0xFFu, 12);     /* minor */
        add_code(a, (version >> 8) & 0xFFu, 12);      /* patch */
        if (pre != 0xFFu) {                           /* alpha 1 short, beta 2 short, rc 3 short */
            uint32_t i, n = pre >= 0x80u ? 3u : pre >= 0x40u ? 2u : 1u;
            for (i = 0; i < n; i++) {
                add(a, 2, 3);
            }
        }
        a->ms[a->nseg - 1u] = 10;                     /* final pause before normal output */
    }
    a->seg = 0;
    a->left = (uint16_t)(a->ms[0] * ACK_UNIT_MS);
    a->active = 1;
}

void ack_stop(ack_t *a)
{
    a->active = 0;
}

uint8_t ack_tick(ack_t *a, uint8_t *on)
{
    if (!a->active) {
        *on = 0;
        return 0;
    }
    *on = (uint8_t)((a->seg & 1u) == 0u);
    if (--a->left == 0u) {
        if (++a->seg >= a->nseg) {
            if (a->repeat && a->repeat_left_s > 0u) {
                a->repeat_left_s--;                   /* one 500/500 cycle = 1 s */
                a->seg = 0;
            } else {
                a->active = 0;
                return 1;
            }
        }
        a->left = (uint16_t)(a->ms[a->seg] * ACK_UNIT_MS);
    }
    return 1;
}
