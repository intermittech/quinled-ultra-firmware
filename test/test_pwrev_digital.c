/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compare Digital-M's reachable power-event states with the full PWM logic. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define BOARD 3
#include "../src/board/boards.h"
#include "../src/core/pwrev.c"

#undef BOARD_PWM
#define BOARD_PWM 1
#define pwrev_init reference_init
#define pwrev_update reference_update
#define pwrev_latched reference_latched
#define pwrev_state reference_state
#define pwrev_page reference_page
#define pwrev_ack reference_ack
#include "../src/core/pwrev.c"
#undef pwrev_init
#undef pwrev_update
#undef pwrev_latched
#undef pwrev_state
#undef pwrev_page
#undef pwrev_ack

int main(void)
{
    pwrev_t actual, reference;
    pwrev_in_t in;
    uint8_t a[24], b[24], arg[9];
    uint32_t i, now = 0xFFFFF000u;
    memset(&in, 0, sizeof in);
    in.factor = 65535u;
    in.limit_src = ULTRA_PW_SRC_BOARD;
    in.limit_ma = 6000u;
    pwrev_init(&actual, now);
    reference_init(&reference, now);
    for (i = 0; i < 200000u; i++) {
        now += 1u + (i % 103u == 0u ? 1000u : 0u);
        in.limiting = (uint8_t)(i % 12001u < 4000u);
        in.i_ma = (uint16_t)(i * 17u);
        in.trip_ma = in.i_ma;
        in.dips = (uint16_t)(i / 3u);
        pwrev_update(&actual, now, &in);
        reference_update(&reference, now, &in);
        memset(arg, 0, sizeof arg);
        arg[0] = i % 7u ? ULTRA_PW_ARG_ACK : 0;
        ultra_put16(arg + 1, (uint16_t)(actual.limit_events + (i & 1u)));
        ultra_put16(arg + 3, (uint16_t)(i & 1u));
        ultra_put16(arg + 5, (uint16_t)(i & 1u));
        ultra_put16(arg + 7, (uint16_t)(in.dips + (i & 1u)));
        pwrev_ack(&actual, arg);
        reference_ack(&reference, arg);
        pwrev_page(&actual, a);
        reference_page(&reference, b);
        assert(memcmp(a, b, sizeof a) == 0);
        assert(memcmp(&actual, &reference, sizeof actual) == 0);
        assert(pwrev_state(&actual) == reference_state(&reference));
    }
    puts("Digital-M power events: 200000 full-state/page/reference comparisons passed");
    return 0;
}
