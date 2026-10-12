/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* core/div64.c against the compiler's own 64-bit division (included by test_main.c): edge cases, then random
 * operands of every magnitude (the target's __udivdi3 / __divdi3 are these functions). */
#include "../src/core/div64.h"

static uint64_t d64_rs = 0x2545F4914F6CDD1Dull;
static uint64_t d64_rnd(void)
{
    d64_rs ^= d64_rs << 13;
    d64_rs ^= d64_rs >> 7;
    d64_rs ^= d64_rs << 17;
    return d64_rs;
}

/* a value with a random number of significant bits (0..64) */
static uint64_t d64_mag(void)
{
    uint32_t b = (uint32_t)(d64_rnd() % 65u);
    uint64_t v = d64_rnd();
    return b == 64u ? v : (b == 0u ? 0u : v & ((1ull << b) - 1u));
}

static void test_div64(void)
{
    static const uint64_t edge[] = { 0, 1, 2, 3, 0x7FFFFFFFull, 0x80000000ull, 0xFFFFFFFFull, 0x100000000ull,
                                     0x100000001ull, 65535, 65536, 0x7FFFFFFFFFFFFFFFull, 0x8000000000000000ull,
                                     0xFFFFFFFFFFFFFFFEull, 0xFFFFFFFFFFFFFFFFull, 4294836225ull + 32767u };
    uint32_t i, j, bad = 0;
    printf("64-bit division (core/div64.c, the target's __udivdi3 / __divdi3)\n");
    for (i = 0; i < sizeof edge / sizeof edge[0]; i++) {
        for (j = 0; j < sizeof edge / sizeof edge[0]; j++) {
            uint64_t n = edge[i], d = edge[j];
            if (d == 0u) {
                continue;
            }
            if (div64_u(n, d) != n / d) bad++;
            if (!((int64_t)n == INT64_MIN && (int64_t)d == -1) &&
                div64_s((int64_t)n, (int64_t)d) != (int64_t)n / (int64_t)d) bad++;
        }
    }
    CHECK(bad == 0u, "edge cases: %u wrong", (unsigned)bad);
    for (i = 0; i < 2000000u; i++) {
        uint64_t n = d64_mag(), d = d64_mag();
        if (d == 0u) {
            continue;
        }
        if (div64_u(n, d) != n / d) bad++;
        if (!((int64_t)n == INT64_MIN && (int64_t)d == -1) &&
            div64_s((int64_t)n, (int64_t)d) != (int64_t)n / (int64_t)d) bad++;
    }
    CHECK(bad == 0u, "random: %u wrong of 2 M", (unsigned)bad);
    CHECK(div64_u(5, 0) == 0xFFFFFFFFFFFFFFFFull && div64_u(0x100000000ull, 0) == 0xFFFFFFFFFFFFFFFFull,
          "division by zero: all ones (divu)");
}
