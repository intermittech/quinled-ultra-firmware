/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Acquisition, local time and coherent frozen statistics on the production core. */
#include <stdio.h>
#include <string.h>
#include "../src/core/ws_decode.h"
#include "../src/core/link_stats.h"
#include "../src/core/ultra_protocol.h"
static unsigned checks, fails;
#define CHECK(x) do { ++checks; if(!(x)) { ++fails; printf("FAIL line %u: %s\n",(unsigned)__LINE__,#x); } } while(0)
static const uint32_t Q = (1000u * 65536u + 18u) / 36u;
static ws_line_t capture;
static uint16_t runs[WS_LINE_WORDS];
static uint8_t slot[32], arg[22], a[24], b[24];

static void frame(uint32_t ms, unsigned pattern, int valid)
{
    unsigned i;
    memset(slot,0,sizeof slot);
    memset(slot+3,pattern,27);
    ultra_put16(slot+30,ultra_crc16(slot,30));
    for(i=0;i<256;i++) runs[i]=(slot[i>>3]&(0x80u>>(i&7u)))?34:11;
    ls_frame(&capture,runs,slot,valid?3:0,ms,0);
}
static void page(unsigned part, unsigned clear, uint32_t ms, uint32_t q, uint8_t *d)
{
    memset(arg,0,sizeof arg); arg[1]=(uint8_t)clear; arg[2]=(uint8_t)part;
    CHECK(ls_page(arg,d,ms,q)==ULTRA_PAGE_LINK_STATS);
}
static void fresh(unsigned profile)
{
    memset(&line_stats,0,sizeof line_stats);
    memset(&capture,0,sizeof capture);
    capture.profile=(uint8_t)profile; capture.bits=320;
    capture.period_min=44; capture.period_max=46;
    capture.low_min=10; capture.low_max=35;
    capture.high_min=11; capture.high_max=34;
}
int main(void)
{
    unsigned i;
    ws_lock_t lock;
    memset(&lock,0,sizeof lock);
    CHECK(!(ws_line_lock(&lock,3,1,0xfffffff0u)&1));
    CHECK(!(ws_line_lock(&lock,3,1,0x10u)&1));
    CHECK(ws_line_lock(&lock,3,1,0x30u)&1);
    CHECK(!ws_line_ready(&lock,0x30u));
    CHECK(ws_line_ready(&lock,0x400u));
    CHECK(!ws_line_ready(&lock,0x1000u));
    CHECK(ws_line_lock(&lock,3,1,0x1000u)&2);
    CHECK(!(ws_line_lock(&lock,2,1,0x1100u)&1));
    CHECK(!(ws_line_lock(&lock,2,1,0x1200u)&1));
    CHECK(ws_line_lock(&lock,2,1,0x1300u)&1);
    CHECK(!(ws_line_lock(&lock,2,0,0x1400u)&2));
    CHECK(ws_line_ready(&lock,0x1800u)); /* one bad CRC does not renegotiate */
    for(i=0;i<4u;i++) {
        memset(&lock,0,sizeof lock);
        CHECK(!(ws_line_lock(&lock,i|4u,1,100u)&5u));
        CHECK(!(ws_line_lock(&lock,i|4u,0,120u)&5u)); /* broken three-header run */
        CHECK(lock.good==0u);
        CHECK(!(ws_line_lock(&lock,i|4u,1,140u)&5u));
        CHECK(!(ws_line_lock(&lock,i|4u,1,160u)&5u));
        CHECK(ws_line_lock(&lock,i|4u,1,180u)==4u);
        CHECK(!ws_line_ready(&lock,1500u)); /* tentative never qualifies time */
        CHECK(ws_line_lock(&lock,i,1,1600u)==2u); /* full acquisition replaces it */
        CHECK(!(ws_line_lock(&lock,i,1,1620u)&1u));
        CHECK(ws_line_lock(&lock,i,1,1640u)==1u);
        CHECK(ws_line_ready(&lock,2600u));
    }

    fresh(0); page(1,0,0,Q,b);
    CHECK(b[2]==0 && b[1]==255);
    frame(100,0x55,1); frame(350,0xaa,1); frame(600,0,1);
    page(0,0,800,Q,a); CHECK(!(a[2]&ULTRA_LS_VALID));
    frame(850,255,1);
    page(0,0,1100,Q,a); CHECK((a[2]&3)==3);
    CHECK(ultra_get16(a+16)>=256 && ultra_get16(a+18)>=256);
    page(1,0,1100,Q,b);
    CHECK(b[1]==0 && b[2]==a[2] && b[3]==15 && ultra_get16(b+4)==ultra_get16(a+22));
    CHECK(ultra_get16(b+18)==0);
    frame(1150,0x55,1);
    page(1,0,1200,Q,slot); CHECK(memcmp(slot,b,24)==0);
    page(0,1,1250,Q,a); CHECK(line_stats.live.count[0]>=256 && line_stats.live.count[1]>=256);
    frame(1300,0xaa,1);
    page(1,0,1350,Q,b); CHECK(ultra_get16(b+4)==ultra_get16(a+22));
    CHECK(line_stats.live.frames==1);
    page(0,0,1350,0,a); CHECK(!(a[2]&3));
    page(1,0,4000,Q,b); CHECK(b[2]==0 && b[1]==255);

    fresh(0); frame(100,0x55,1); frame(350,0xaa,1);
    page(0,0,1100,Q,a); page(1,0,1100,Q,b);
    CHECK(!(a[2]&1) && ultra_get16(b+18)==2);
    frame(1200,0x55,1); frame(1450,0xaa,1); frame(1700,0,1); frame(1950,255,1);
    frame(2100,0x55,0); page(0,0,2200,Q,a); CHECK(!(a[2]&1));
    CHECK(line_stats.live.bad==1);

    fresh(0); frame(0xfffffc00u,0x55,1); frame(0xfffffd00u,0xaa,1);
    frame(0xfffffe00u,0,1); frame(0xffffff00u,255,1);
    page(0,0,0x10u,Q,a); CHECK(a[2]&1);
    line_stats.epoch=65535; page(0,0,0x20u,Q,a); page(1,0,0x20u,Q,b);
    CHECK(ultra_get16(a+22)==65535 && ultra_get16(b+4)==65535 && b[1]==0 && (b[2]&1));
    capture.profile=1; frame(0x30u,0x55,1); page(1,0,0x30u,Q,b);
    CHECK(b[1]==255 && b[2]==0);
    CHECK(line_stats.epoch==0); /* epoch advances at acquisition, including wrap */

    /* Isolated CRC errors and two bad windows never request a rate change. */
    fresh(2);
    line_stats.profile=2; line_stats.trained=1;
    line_stats.live.min[0]=17; line_stats.live.count[0]=256;
    for(i=0;i<3u;i++) {
        unsigned k;
        for(k=1;k<=3u;k++) ls_frame(0,0,0,0,1000u*i+100u*k,0);
        ls_frame(0,0,0,1,1000u*(i+1u),0);
    }
    CHECK(!(line_stats.flags&ULTRA_LS_RATE_REQUEST));
    CHECK(line_stats.profile==2 && line_stats.trained);
    for(i=3;i<6u;i++) {
        unsigned k;
        for(k=1;k<=4u;k++) ls_frame(0,0,0,0,1000u*i+100u*k,0);
        ls_frame(0,0,0,1,1000u*(i+1u),0);
        CHECK((line_stats.flags&ULTRA_LS_RATE_REQUEST)==(i==5u?ULTRA_LS_RATE_REQUEST:0));
    }
    CHECK(line_stats.profile==2); /* host owns the decision */
    CHECK(line_stats.live.min[0]==17 && line_stats.live.count[0]==256);
    page(0,0,6000,Q,a); page(1,0,6000,Q,b);
    CHECK((a[2]&ULTRA_LS_RATE_REQUEST) && a[2]==b[2]);
    capture.profile=3; frame(6100,0x55,1);
    CHECK(line_stats.profile==3 && !line_stats.trained && !(line_stats.flags&ULTRA_LS_RATE_REQUEST));

    /* Window boundaries do not drift with calls; silence breaks the streak.
     * Repeat across the unsigned millisecond counter's wrap. */
    for(i=0;i<2u;i++) {
        uint32_t base=i?0xfffff900u:100u, win, k;
        fresh(0); line_stats.error_ms=base;
        line_stats.trained=1;
        for(win=0;win<2;win++) {
            for(k=1;k<=4;k++) ls_frame(0,0,0,0,base+win*1000+100*k,0);
            ls_frame(0,0,0,1,base+(win+1)*1000+99,0);
        }
        CHECK(line_stats.bad_windows==2 && line_stats.error_ms==base+2000u);
        for(k=1;k<=4;k++) ls_frame(0,0,0,0,base+2000+100*k,0);
        /* The fourth window has no errors; delayed closure cannot join it
         * to the preceding bad window and latch a false request. */
        ls_frame(0,0,0,1,base+4000u,0);
        CHECK(!line_stats.bad_windows && !(line_stats.flags&ULTRA_LS_RATE_REQUEST));
        for(win=4;win<7;win++) {
            for(k=1;k<=4;k++) ls_frame(0,0,0,0,base+win*1000+100*k,0);
            ls_frame(0,0,0,1,base+(win+1)*1000,0);
        }
        CHECK(line_stats.flags&ULTRA_LS_RATE_REQUEST);
        ls_frame(0,0,0,1,base+20000u,0);
        CHECK(line_stats.flags&ULTRA_LS_RATE_REQUEST); /* latched until reacquisition */
        capture.profile=1; frame(base+20001u,0x55,1);
        CHECK(!(line_stats.flags&ULTRA_LS_RATE_REQUEST));
    }

    fresh(0); frame(100,0,1);
    line_stats.live.count[0]=65535; line_stats.live.sum[0]=123456;
    line_stats.live.min[0]=3; line_stats.live.max[0]=40;
    frame(200,0,1);
    CHECK(line_stats.live.count[0]==65535 && line_stats.live.sum[0]==123456);
    CHECK(line_stats.live.min[0]==3 && line_stats.live.max[0]==40 && (line_stats.flags&4));
    /* Saturating in the middle of a varied frame keeps only the accepted
     * widths; the batched path must match the per-bit reference exactly. */
    {
        uint32_t sums[2]={100,200}, counts[2]={65534,7};
        uint16_t mins[2]={9,29}, maxs[2]={14,38};
        fresh(0);
        for(i=0;i<32u;i++) slot[i]=(uint8_t)(i*73u+19u);
        for(i=0;i<2u;i++) {
            line_stats.live.sum[i]=sums[i];
            line_stats.live.count[i]=(uint16_t)counts[i];
            line_stats.live.min[i]=mins[i]; line_stats.live.max[i]=maxs[i];
        }
        for(i=0;i<256u;i++) {
            unsigned bit=(slot[i>>3]>>(7u-(i&7u)))&1u;
            unsigned h=(bit?30u:10u)+(i%7u);
            runs[i]=(uint16_t)h;
            if(counts[bit]<65535u) {
                counts[bit]++; sums[bit]+=h;
                if(h<mins[bit]) mins[bit]=(uint16_t)h;
                if(h>maxs[bit]) maxs[bit]=(uint16_t)h;
            }
        }
        ls_frame(&capture,runs,slot,3,100,0);
        for(i=0;i<2u;i++) {
            CHECK(line_stats.live.count[i]==counts[i]);
            CHECK(line_stats.live.sum[i]==sums[i]);
            CHECK(line_stats.live.min[i]==mins[i] && line_stats.live.max[i]==maxs[i]);
        }
        CHECK(line_stats.flags&4);
    }
    memset(arg,0,sizeof arg); arg[0]=1;
    CHECK(ls_page(arg,a,200,Q)==ULTRA_PAGE_UNSUPPORTED);
    for(i=3;i<22;i++) {
        memset(arg,0,sizeof arg);arg[i]=1;
        CHECK(ls_page(arg,a,200,Q)==ULTRA_PAGE_UNSUPPORTED);
    }
    printf("line acquisition/statistics: %u checks, %u failures\n",checks,fails);
    return fails!=0;
}
