/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Exact RV32 fixed-point arithmetic. No compiler 64-bit arithmetic helpers.
 * Products use mulhu on RV32; the portable path uses four 16-bit products.
 * Shift results and division results are the low 32 bits unless marked sat.
 * Callers retain wide clock/regression accumulators only where their range needs them. */
#ifndef MATH32_H
#define MATH32_H
#include <stdint.h>
static inline uint32_t m32_hi(uint32_t a, uint32_t b)
{
#if defined(__riscv)
    uint32_t h;
    __asm__("mulhu %0,%1,%2" : "=r"(h) : "r"(a), "r"(b));
    return h;
#else
    uint32_t a0=a&65535u,a1=a>>16,b0=b&65535u,b1=b>>16;
    uint32_t w0=a0*b0,t=a1*b0+(w0>>16),w1=t&65535u,w2=t>>16;
    w1+=a0*b1;
    return a1*b1+w2+(w1>>16);
#endif
}
/* s in 1..31. Unsigned shifts avoid signed-overflow and negative-shift UB. */
static inline uint32_t m32_shr(uint32_t a,uint32_t b,uint32_t s)
{ return (a*b>>s)|(m32_hi(a,b)<<(32u-s)); }
static inline uint32_t m32_sar(int32_t a,uint32_t b,uint32_t s)
{ uint32_t h=m32_hi((uint32_t)a,b)-(a<0?b:0u); return ((uint32_t)a*b>>s)|(h<<(32u-s)); }
static inline uint32_t m32_addshr(uint32_t a,uint32_t b,uint32_t c,uint32_t s)
{ uint32_t l=a*b,h=m32_hi(a,b);h+=(l+c<l);l+=c;return (l>>s)|(h<<(32u-s)); }
uint32_t m32_div(uint32_t hi,uint32_t lo,uint32_t d);
uint32_t m32_divwide(uint32_t hi,uint32_t lo,uint32_t dh,uint32_t dl);
static inline uint32_t m32_divsat(uint32_t hi,uint32_t lo,uint32_t d)
{ return hi>=d?0xFFFFFFFFu:m32_div(hi,lo,d); }
static inline uint32_t m32_muldiv(uint32_t a,uint32_t b,uint32_t d)
{ return m32_div(m32_hi(a,b),a*b,d); }
static inline uint32_t m32_maddiv(uint32_t a,uint32_t b,uint32_t c,uint32_t d)
{ uint32_t l=a*b,h=m32_hi(a,b);h+=(l+c<l);return m32_div(h,l+c,d); }
/* floor(floor(a*b/d)/2^s) = floor(floor(a*b/2^s)/d), including when the
 * unshifted quotient needs more than 32 bits. s in 1..31. */
static inline uint32_t m32_divshr(uint32_t a,uint32_t b,uint32_t d,uint32_t s)
{ uint32_t h=m32_hi(a,b),l=a*b;return m32_div(h>>s,(h<<(32u-s))|(l>>s),d); }
/* Exact through 75000 mV x 65535 mA. Result <= 4915125 mW. */
static inline uint32_t m32_power(uint32_t mv,uint32_t ma)
{ return (mv/1000u)*ma+(mv%1000u)*ma/1000u; }
#endif
