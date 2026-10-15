/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Standalone host equivalence test for the target memory routines. Include them
 * under private names so the reference is the host libc, not the replacements. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef _MSC_VER
#define restrict __restrict
#endif
#define memcpy small_memcpy
#define memset small_memset
#define memmove small_memmove
#include "../src/hal/mem.c"
#undef memcpy
#undef memset
#undef memmove

int main(void)
{
    unsigned char src[1024], want[1024], got[1024];
    size_t n, a, b, i, tests = 0;
    const int fills[] = { 0, 1, 127, 128, 255, 256, -1, -257 };
    for (i = 0; i < sizeof(src); i++) {
        src[i] = (unsigned char)(i * 157u + i / 7u);
    }
    for (n = 0; n <= 512; n++) {
        for (a = 0; a < 8; a++) {
            for (b = 0; b < 8; b++) {
                memset(want, 0xA5, sizeof(want));
                memcpy(got, want, sizeof(got));
                memcpy(want + 8 + a, src + b, n);
                assert(small_memcpy(got + 8 + a, src + b, n) == got + 8 + a);
                assert(memcmp(got, want, sizeof(got)) == 0);
                tests++;
            }
            for (b = 0; b < sizeof(fills) / sizeof(fills[0]); b++) {
                memcpy(want, src, sizeof(want));
                memcpy(got, src, sizeof(got));
                memset(want + 8 + a, fills[b], n);
                assert(small_memset(got + 8 + a, fills[b], n) == got + 8 + a);
                assert(memcmp(got, want, sizeof(got)) == 0);
                tests++;
            }
        }
        for (a = 0; a < 40; a++) {
            for (b = 0; b < 40; b++) {
                memcpy(want, src, sizeof(want));
                memcpy(got, src, sizeof(got));
                memmove(want + 8 + a, want + 8 + b, n);
                assert(small_memmove(got + 8 + a, got + 8 + b, n) == got + 8 + a);
                assert(memcmp(got, want, sizeof(got)) == 0);
                tests++;
            }
        }
    }
    printf("memory equivalence: %zu cases passed (alignment, length, overlap, fill, guards, return)\n", tests);
    return 0;
}
