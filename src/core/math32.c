/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "math32.h"
#include "lto.h"
static uint32_t clz32(uint32_t x)
{
    uint32_t n=0;
    if(!(x&0xFFFF0000u)){n+=16;x<<=16;}
    if(!(x&0xFF000000u)){n+=8;x<<=8;}
    if(!(x&0xF0000000u)){n+=4;x<<=4;}
    if(!(x&0xC0000000u)){n+=2;x<<=2;}
    if(!(x&0x80000000u))n++;
    return n;
}
/* Two base-65536 quotient digits, each corrected at most twice (Knuth D).
 * u1<v; every multiply and remainder is modulo 2^32, the true remainder fits. */
static NOINLINE uint32_t divdigits(uint32_t u1,uint32_t u0,uint32_t v)
{
    uint32_t s=clz32(v),vn1,vn0,un32,un10,un1,un0,q1,q0,rhat,un21;
    v<<=s;vn1=v>>16;vn0=v&65535u;
    un32=s?(u1<<s)|(u0>>(32u-s)):u1;un10=u0<<s;un1=un10>>16;un0=un10&65535u;
    q1=un32/vn1;rhat=un32-q1*vn1;
    while(q1>65535u||q1*vn0>((rhat<<16)|un1)){q1--;rhat+=vn1;if(rhat>65535u)break;}
    un21=(un32<<16)+un1-q1*v;
    q0=un21/vn1;rhat=un21-q0*vn1;
    while(q0>65535u||q0*vn0>((rhat<<16)|un0)){q0--;rhat+=vn1;if(rhat>65535u)break;}
    return (q1<<16)|q0;
}
uint32_t m32_div(uint32_t hi,uint32_t lo,uint32_t d)
{
    if(!d)return 0xFFFFFFFFu;
    return hi?divdigits(hi%d,lo,d):lo/d;
}
/* Regression's sum of five squared Q16 duties needs 35 bits (39 after mu).
 * Normalize its top word, estimate on the halved numerator, correct once. */
uint32_t m32_divwide(uint32_t hi,uint32_t lo,uint32_t dh,uint32_t dl)
{
    uint32_t s,v,q,pl,ph,rl,rh;
    if(!dh)return m32_div(hi,lo,dl);
    s=clz32(dh);v=s?(dh<<s)|(dl>>(32u-s)):dh;
    q=divdigits(hi>>1,(lo>>1)|(hi<<31),v)>>(31u-s);
    if(q)q--;
    pl=q*dl;ph=m32_hi(q,dl)+q*dh;
    rl=lo-pl;rh=hi-ph-(lo<pl);
    if(rh>dh||(rh==dh&&rl>=dl))q++;
    return q;
}
