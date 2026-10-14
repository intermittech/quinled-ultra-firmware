/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Board current limit in effect, and whether the connected load can reach it.
 *
 * The limit = the lowest of: the board rating, the user's limit (Ultra CONFIG 21 / the stored `ilimit`) and the fuse
 * rating. There is no limit per PWM preset: realistic content keeps C10 within its ripple rating at every preset
 * (C10's ripple model; none until the bench check confirms it).
 * "No limit needed": the connected load (each channel at 100 %, calibrated / learned by the current sense, or
 * configured) can never draw more than the limit, so the limiter never engages.
 * Pure logic, host-tested (test/test_cal.c). */
#ifndef CLIMIT_H
#define CLIMIT_H

#include <stdint.h>

enum { CLIMIT_SRC_BOARD = 0, CLIMIT_SRC_USER = 1, CLIMIT_SRC_FUSE = 2, CLIMIT_SRC_NONE = 3 };
enum { CLIMIT_NEED_UNKNOWN = 0, CLIMIT_NEED_NO = 1, CLIMIT_NEED_YES = 2 };

typedef struct {
    uint16_t limit_ma;                /* in effect (0xFFFF = none: nothing given) */
    uint8_t  src;                     /* CLIMIT_SRC_*: which one is the lowest */
    uint8_t  need;                    /* CLIMIT_NEED_*: can the connected load reach it */
    uint32_t connected_ma;            /* 0 = not known */
} climit_t;

/* board_ma / fuse_ma 0 = not part of it; user_ma 0 = none; connected_ma 0 = not known */
void climit_eval(climit_t *c, uint16_t user_ma, uint16_t board_ma, uint16_t fuse_ma, uint32_t connected_ma);
/* (the console names the source: core/cli.c, status) */

#endif
