/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Integer oracle: old expressions use host 64-bit arithmetic only in tests.
 * Full word-range helper tests also cover carry, wrap and signed-floor behavior. */
#include "../src/core/math32.h"
static uint32_t m32_rng=0xC0FFEEu;
static uint32_t m32_rand(void){m32_rng^=m32_rng<<13;m32_rng^=m32_rng>>17;m32_rng^=m32_rng<<5;return m32_rng;}
static void test_math32(void)
{
    uint32_t i,j,bad=0;
    static const uint32_t edge[]={0,1,2,255,256,65535,65536,0x7FFFFFFF,0x80000000,0xFFFFFFFE,0xFFFFFFFF};
    for(i=0;i<2000000u;i++) {
        uint32_t a=m32_rand(),b=m32_rand(),d=m32_rand()|1u,s=1u+m32_rand()%31u,c=m32_rand();
        uint64_t p=(uint64_t)a*b,n=((uint64_t)a<<32)|b;
        uint64_t den=((uint64_t)(m32_rand()&255u)<<32)|d;
        if(m32_hi(a,b)!=(uint32_t)(p>>32)||m32_shr(a,b,s)!=(uint32_t)(p>>s)||
           m32_sar((int32_t)a,b,s)!=(uint32_t)(((int64_t)(int32_t)a*b)>>s)||
           m32_addshr(a,b,c,s)!=(uint32_t)((p+c)>>s)||m32_muldiv(a,b,d)!=(uint32_t)(p/d)||
           m32_maddiv(a,b,c,d)!=(uint32_t)((p+c)/d)||m32_div(a,b,d)!=(uint32_t)(n/d)||
           m32_divshr(a,b,d,s)!=(uint32_t)((p/d)>>s)||
           m32_divsat(a,b,d)!=(n/d>0xFFFFFFFFu?0xFFFFFFFFu:(uint32_t)(n/d))||
           m32_divwide(a,b,(uint32_t)(den>>32),(uint32_t)den)!=(uint32_t)(n/den))bad++;
        {
            /* Integral product used by fx_beat_cum: legal sine/delta ranges.
             * Limiter pre-saturation must preserve the final 0x7fffff clamp. */
            uint32_t k=a%2441218u,sine=b%65535u,delta=c&65535u;
            uint32_t lo=k*sine,hi=m32_hi(k,sine),cl=8u+c%249u;
            uint32_t got=(lo*delta>>16)|((m32_hi(lo,delta)+hi*delta)<<16);
            uint64_t q=n/d,scaled=q>0xFFFFFFFFu?0xFFFFFFFFu:q;
            uint32_t lim=m32_shr((uint32_t)scaled,cl,8);
            /* Avoid overflowing the oracle for irrelevant huge q: any q
             * >= 2^32, even at cl=8, is already above the final clamp. */
            uint64_t ref=q>=0x100000000ull?0x7FFFFFu:(q*cl>>8);
            if(lim>0x7FFFFFu)lim=0x7FFFFFu;
            if(ref>0x7FFFFFu)ref=0x7FFFFFu;
            if(got!=(uint32_t)(((uint64_t)k*sine*delta)>>16)||lim!=ref)bad++;
        }
    }
    for(i=0;i<sizeof edge/sizeof edge[0];i++)for(j=0;j<sizeof edge/sizeof edge[0];j++) {
        uint32_t k;uint64_t n=((uint64_t)edge[i]<<32)|edge[j];
        for(k=1;k<sizeof edge/sizeof edge[0];k++) {
            uint64_t d=((uint64_t)edge[k]<<32)|edge[i];
            if(m32_div(edge[i],edge[j],edge[k])!=(uint32_t)(n/edge[k])||
               m32_divwide(edge[i],edge[j],edge[k],edge[i])!=(uint32_t)(n/d))bad++;
        }
    }
    CHECK(!bad,"32-bit limbs: %u mismatches",bad);
    printf("math32: 2,000,000 full-range tuples + edge Cartesian products; %u mismatches\n",bad);
    bad=0;
    /* Every possible remainder and current; the quotient term is linear and
     * exact for all 0..75. Test the lowest and highest voltage quotient. */
    for(i=0;i<1000u;i++)for(j=0;j<=65535u;j++) {
        uint32_t v=74000u+i;
        if(m32_power(i,j)!=(uint64_t)i*j/1000u||m32_power(v,j)!=(uint64_t)v*j/1000u)bad++;
    }
    CHECK(m32_power(75000u,65535u)==4915125u&&!bad,"power range exact");
    {
        energy_t en; uint32_t t; uint64_t uj=0;
        memset(&en,0,sizeof en);
        for(t=0;t<10000u;t++) {
            uint32_t mv=t&1u?75000u:74999u,ma=t%3u?65535u:65000u;
            uj+=(uint64_t)mv*ma/1000u;
            energy_ms(&en,t,mv,(int32_t)ma,1);
            CHECK(en.boot_mwh==(uint32_t)(uj/3600000u)&&en.uj==(uint32_t)(uj%3600000u),"high-range energy accumulator");
        }
    }
    printf("math32 power: 131,072,000 endpoint/remainder cases through 75 V and 65535 mA; %u mismatches\n",bad);
}

#include "math32_reference.inc"
static void test_math32_curves(void)
{
    uint32_t p,g,i,bad=0;
    for(p=0;p<=65536u;p++) {
        if(ultra_smoothstep(p)!=ref_smoothstep(p))bad++;
        if(ultra_lerp16(0,65535,p)!=ref_old_lerp16(0,65535,p)||
           ultra_lerp16(65535,0,p)!=ref_old_lerp16(65535,0,p))bad++;
    }
    CHECK(!bad,"exhaustive smoothstep and signed lerp extremes: %u",bad);
    for(g=0;g<=65535u;g++) {
        uint32_t q=(g*65536u+500u)/1000u;
        static const uint32_t x[]={0,1,2,65535,65536,0x7FFFFFFF,0x80000000,0xFFFFFFFE,0xFFFFFFFF};
        for(i=0;i<sizeof x/sizeof x[0];i++)if(ultra_gamma(x[i],q)!=ref_gamma(x[i],q))bad++;
    }
    for(i=0;i<3000000u;i++) {
        uint32_t x=m32_rand(),a=m32_rand()&65535u,b=m32_rand()&65535u;
        p=m32_rand()%65537u;g=((m32_rand()&65535u)*65536u+500u)/1000u;
        if(ultra_gamma(x,g)!=ref_gamma(x,g)||ultra_lerp16(a,b,p)!=ref_old_lerp16(a,b,p))bad++;
    }
    CHECK(!bad,"gamma/lerp equivalence: %u",bad);
    printf("math32 curves: all 65537 smoothstep inputs; both lerp extremes; all 65536 gamma settings x 9 edges; 3000000 random gamma/lerp pairs; %u mismatches\n",bad);
}
