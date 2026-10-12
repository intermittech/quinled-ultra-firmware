/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "ack.h"

static void add(ack_t *a, uint16_t on_ms, uint16_t off_ms)
{
    if (a->nseg + 2u <= ACK_MAX_SEG) {
        a->ms[a->nseg++] = on_ms;
        a->ms[a->nseg++] = off_ms;
    }
}

static void add_ack(ack_t *a, uint16_t last_off)
{
    add(a, 100, 100);
    add(a, 100, 100);
    add(a, 100, last_off);
}

void ack_start(ack_t *a, ack_kind_t kind, uint8_t value_code)
{
    a->nseg = 0;
    a->repeat = 0;
    switch (kind) {
    case ACK_P_ACK:
        if (value_code == 0u) {
            add_ack(a, 1000);
        } else {
            uint8_t longs = (uint8_t)(value_code / 5u), shorts = (uint8_t)(value_code % 5u), i;
            add_ack(a, 900);                          /* 100 ms off + 800 ms pause */
            for (i = 0; i < longs; i++) {
                add(a, 700, 300);
            }
            for (i = 0; i < shorts; i++) {
                add(a, 200, 300);
            }
            a->ms[a->nseg - 1u] = 1000;               /* final pause before normal output */
        }
        break;
    case ACK_P_FACTORY:
        add_ack(a, 700);
        add_ack(a, 1000);
        break;
    case ACK_P_SAME:
        add(a, 200, 1000);
        break;
    case ACK_P_NACK:
        add(a, 1500, 1000);
        break;
    case ACK_P_IDENTIFY:
        add(a, 500, 500);
        a->repeat = 1;
        a->repeat_left_s = 30;
        break;
    }
    a->seg = 0;
    a->left = a->ms[0];
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
        a->left = a->ms[a->seg];
    }
    return 1;
}
