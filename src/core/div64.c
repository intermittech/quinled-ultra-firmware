/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compact 64-bit division for the RV32IMC target (research/flash-size.md).
 *
 * libgcc's __udivdi3 / __divdi3 are 1.8 KB plus a 256-byte clz table. These are ~0.4 KB, about as fast:
 *  - dividend and divisor both 32 bits (most calls: products of two 16-bit values over a 16-bit divisor): divu;
 *  - a 32-bit divisor: the high word by divu, then the 64 / 32 step as a long division in two 16-bit digits
 *    (Knuth's algorithm D, as in Hacker's Delight "divlu"), each digit from divu and at most two corrections;
 *  - a divisor above 32 bits (the isense regression's norm): normalised to its top 32 bits, one divlu step on the
 *    halved dividend, then a correction of at most one (Hacker's Delight's 64 / 64 division).
 * No 64-bit '/' or '%' in here (that would call itself). Division by zero: all ones, as RISC-V's divu (C leaves it
 * undefined). Host-tested against the compiler's own division (test/test_div64.c). */
#include "div64.h"

#if defined(__GNUC__)
#define NOINLINE __attribute__((noinline))
#else
#define NOINLINE
#endif

static uint32_t clz32(uint32_t x)                   /* x != 0 */
{
    uint32_t n = 0;
    if (!(x & 0xFFFF0000u)) { n += 16u; x <<= 16; }
    if (!(x & 0xFF000000u)) { n += 8u; x <<= 8; }
    if (!(x & 0xF0000000u)) { n += 4u; x <<= 4; }
    if (!(x & 0xC0000000u)) { n += 2u; x <<= 2; }
    if (!(x & 0x80000000u)) { n += 1u; }
    return n;
}

/* (u1:u0) / v for u1 < v (the quotient fits 32 bits). Not inlined: the common divu path keeps a small prologue. */
static NOINLINE uint32_t divlu(uint32_t u1, uint32_t u0, uint32_t v)
{
    uint32_t s = clz32(v), vn1, vn0, un32, un10, un1, un0, q1, q0, rhat, un21;
    v <<= s;                                          /* normalise: the divisor's top bit set */
    vn1 = v >> 16;
    vn0 = v & 0xFFFFu;
    un32 = s ? (u1 << s) | (u0 >> (32u - s)) : u1;
    un10 = u0 << s;
    un1 = un10 >> 16;
    un0 = un10 & 0xFFFFu;
    q1 = un32 / vn1;                                  /* the high digit */
    rhat = un32 - q1 * vn1;
    while (q1 > 0xFFFFu || q1 * vn0 > ((rhat << 16) | un1)) {
        q1--;
        rhat += vn1;
        if (rhat > 0xFFFFu) {
            break;
        }
    }
    un21 = (un32 << 16) + un1 - q1 * v;               /* mod 2^32: the true remainder fits */
    q0 = un21 / vn1;                                  /* the low digit */
    rhat = un21 - q0 * vn1;
    while (q0 > 0xFFFFu || q0 * vn0 > ((rhat << 16) | un0)) {
        q0--;
        rhat += vn1;
        if (rhat > 0xFFFFu) {
            break;
        }
    }
    return (q1 << 16) | q0;
}

/* d >= 2^32 (Hacker's Delight's 64 / 64): 32-bit shifts only, no libgcc */
static NOINLINE uint64_t div64_big(uint64_t n, uint32_t d1, uint32_t d0)
{
    uint32_t s = clz32(d1), v1 = s ? (d1 << s) | (d0 >> (32u - s)) : d1;   /* d's top 32 bits, normalised */
    uint32_t q = divlu((uint32_t)(n >> 33), (uint32_t)(n >> 1), v1) >> (31u - s);   /* halved dividend */
    uint64_t d = ((uint64_t)d1 << 32) | d0;
    if (q != 0u) {
        q--;                                          /* now q or q + 1 */
    }
    if (n - (uint64_t)q * d >= d) {
        q++;
    }
    return q;
}

uint64_t div64_u(uint64_t n, uint64_t d)
{
    uint32_t n1 = (uint32_t)(n >> 32), n0 = (uint32_t)n, d1 = (uint32_t)(d >> 32), d0 = (uint32_t)d;
    if (d1 == 0u) {
        uint32_t q1;
        if (d0 == 0u) {
            return 0xFFFFFFFFFFFFFFFFull;
        }
        if (n1 == 0u) {
            return n0 / d0;
        }
        q1 = n1 / d0;
        return ((uint64_t)q1 << 32) | divlu(n1 - q1 * d0, n0, d0);
    }
    return div64_big(n, d1, d0);
}

int64_t div64_s(int64_t n, int64_t d)
{
    uint64_t un = n < 0 ? 0u - (uint64_t)n : (uint64_t)n, ud = d < 0 ? 0u - (uint64_t)d : (uint64_t)d;
    uint64_t q = div64_u(un, ud);
    return (int64_t)(((n < 0) != (d < 0)) ? 0u - q : q);     /* C truncates towards zero */
}

#if defined(__riscv)
/* the compiler's 64-bit division helpers: these replace libgcc's */
uint64_t __udivdi3(uint64_t n, uint64_t d);
int64_t __divdi3(int64_t n, int64_t d);
uint64_t __udivdi3(uint64_t n, uint64_t d) { return div64_u(n, d); }
int64_t __divdi3(int64_t n, int64_t d) { return div64_s(n, d); }
#endif
