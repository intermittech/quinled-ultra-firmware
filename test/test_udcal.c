/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Physical fake: configured bpp/order vs a separately specified strip's bpp,
 * powered address count, optional data break, nonlinear load and ADC noise.
 * Time includes a complete cap-length wire frame and reset before every latch. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ur_digital_m/udcal.h"
#include "../board/boards.h"       /* BOARD_CALIBRATION_MS = UC_DEADLINE_MS */
static unsigned checks, failures;
#define CHECK(x) do { checks++; if(!(x)) { failures++; printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
typedef struct { unsigned n,bpp,cut,noise; int drift,offset; unsigned nonlinear,over,cancel,timeout,missing,weak; } physical_t;
static udcal_t c;
static unsigned rng=7231, worst_ms, last_ms;
/* The board's own pixel limits (decided): V203 600 / 500 / 400, V407 800 / 600 / 500 (RGB / RGBW / five-channel): the
 * request's cap, the full-length runs and the stored record. */
#if FX_V407
static const unsigned board_max[3]={800,600,500};
#else
static const unsigned board_max[3]={600,500,400};
#endif
/* the model's wire bit: the slowest the boards send (V203 1.333 us, V407 1.25 us, each with a 2.2 % slow clock) */
#define TEST_BIT_NS 1400u
static unsigned run(physical_t p,unsigned configured,unsigned bpp)
{
    udcal_model_t cfg;
    unsigned ms,latched=0,send_at=0,held_duty=0,held_mask=0,held_k=0,held_pos=0,cap=board_max[bpp-3u];
    memset(&cfg,0,sizeof(cfg));
    cfg.configured=(uint16_t)configured; cfg.bpp=(uint8_t)bpp; cfg.vbus=24000; cfg.vled=12000; cfg.order=0x4688;
    udcal_request(&c,(uint8_t)(c.seq+1u),&cfg,(uint16_t)cap,2250,0,0);
    for(ms=1;ms<UC_DEADLINE_MS+1500u && udcal_active(&c);ms++) {
        unsigned s=0,i,j, lit=0;
        if(c.pending==1 && !send_at) {
            send_at=ms+(cap*bpp*8u*TEST_BIT_NS+999999u)/1000000u+1u;
            c.pending=2;
        }
        if(send_at && ms>=send_at) {
            held_duty=c.duty;held_mask=c.mask;held_k=c.k;held_pos=c.pos;
            send_at=0; latched++;
            udcal_latched(&c,ms);
        }
        /* Compute current from actual channel bytes; wrong bpp changes the boundary.
         * A broken data path limits observable addresses just like a short strip. */
        for(i=0;i<cap;i++) {
            unsigned selected=held_pos ? i+1u==held_pos : c.candidate.n && i<c.candidate.n &&
                ((i+1u)*held_k+c.candidate.n-1u)/c.candidate.n != (i*held_k+c.candidate.n-1u)/c.candidate.n;
            if(!selected || !held_k) continue;
            for(j=0;j<bpp;j++) if(held_mask & (1u<<j)) {
                unsigned address=(i*bpp+j)/p.bpp;
                if(address<p.n && (!p.cut || address<p.cut) && !(p.weak && j==2u)) lit++;
            }
        }
        for(i=0;i<20u;i++) {
            unsigned noise;
            int ma=100*256+p.offset*256+p.drift*(int)ms/10;
            unsigned gain=p.over && c.phase==ULTRA_UCS_CHANNELS ? 3u:1u;
            ma+=(int)(lit*20u*256u*held_duty/255u*gain);
            if(p.nonlinear && (held_mask&(held_mask-1u))) ma+=ma/3;
            rng=rng*1664525u+1013904223u; noise=(rng>>16)%7u;
            ma+=(int)(p.noise*noise*32u)-(int)(p.noise*96u);
            /* scale=1 mA/count, quantized with deterministic sub-count dither. */
            s+=(unsigned)(ma>0 ? (ma+(int)(rng&255u))/256:0);
        }
        udcal_feed(&c,s,p.missing && ms>300u?0u:20u,65536,ms,p.timeout && ms>100u,24000,12000);
        if(p.cancel && ms==300u) udcal_abort(&c,ULTRA_UCE_CANCELLED);
        udcal_step(&c,65536);
        if(c.phase==ULTRA_UCS_RESTORE && c.finish) {
            CHECK(!held_duty);udcal_restored(&c);
        }
    }
    CHECK(ms<UC_DEADLINE_MS+1500u); CHECK(latched>0); if(ms>worst_ms) worst_ms=ms;
    last_ms=ms;
    printf("  bpp %u count %u/%u: error %u stage %u at %u ms, detected %u\n",bpp,p.n,configured,c.error,c.stage,ms,c.candidate.detected);
    return c.error;
}
/* The stored record across a reboot: pack (the write), a cleared state (the reboot), udcal_valid (the page pair's
 * check) and udcal_load; one pixel above the board's limit is refused, as configured or as detected. */
static int stored(unsigned bpp,unsigned configured,unsigned detected)
{
    static const uint16_t order[3]={0x88,0x688,0x4688};
    static udcal_t w,b;
    uint8_t rec[UC_REC_BYTES];
    unsigned i;
    memset(&w,0,sizeof(w));memset(&b,0,sizeof(b));
    w.model.configured=(uint16_t)configured;w.model.detected=(uint16_t)detected;w.model.n=(uint16_t)configured;
    w.model.order=order[bpp-3u];w.model.vbus=24000;w.model.vled=12000;w.model.k=32;w.model.count=7;w.model.epoch=7;
    for(i=0;i<bpp;i++) w.model.ch[i]=(uint16_t)(2000u+100u*i);
    w.model.chip=(uint8_t)(bpp==3u?0u:bpp==4u?3u:5u);w.model.bpp=(uint8_t)bpp;
    w.model.flags=ULTRA_UCF_VALID|ULTRA_UCF_AGGREGATE;w.elapsed=1234;
    udcal_pack(&w,9,rec);
    if(!udcal_valid(rec)) return 0;
    udcal_load(&b,rec);
    CHECK(memcmp(&b.model,&w.model,39)==0 && b.elapsed==1234);
    return 1;
}
static void store_limits(void)
{
    unsigned bpp;
    for(bpp=3;bpp<=5;bpp++) {
        unsigned max=board_max[bpp-3u];
        CHECK(stored(bpp,max,max));
        CHECK(!stored(bpp,max+1u,max+1u));
        CHECK(!stored(bpp,max,max+1u));
        printf("  stored bpp %u: %u px valid after a reboot, %u refused\n",bpp,max,max+1u);
    }
}
int main(void)
{
    physical_t p={120,3,0,1,0,0,0,0,0,0,0,0};
    unsigned bpp;
    CHECK(sizeof(c)<=240u);
    store_limits();
    CHECK(run(p,120,3)==0); CHECK(c.model.detected==120); CHECK(c.model.n==120);
    CHECK(abs((int)c.model.ch[0]-2400)<50); CHECK(c.model.idle>=99 && c.model.idle<=101);
    p.offset=30;p.drift=10;
    CHECK(run(p,120,3)==0);CHECK(abs((int)c.model.ch[0]-2400)<50);
    p.offset=-30;p.drift=-10;
    CHECK(run(p,120,3)==0);CHECK(abs((int)c.model.ch[0]-2400)<50);
    p.offset=30;
    p.drift=0;p.n=57; CHECK(run(p,120,3)==0); CHECK(c.model.n==57); CHECK(c.model.flags&ULTRA_UCF_SHORT);
    p.n=120;p.cut=31; CHECK(run(p,120,3)==0); CHECK(c.model.detected==31);
    p.cut=0;p.bpp=4; CHECK(run(p,120,3)==0); CHECK(c.model.detected==160); /* optional byte remainder detection not claimed */
    p.bpp=3;p.n=120;p.over=1; CHECK(run(p,120,3)==ULTRA_UCE_PREDICTION); p.over=0;
    p.cancel=1;CHECK(run(p,120,3)==ULTRA_UCE_CANCELLED);p.cancel=0;
    p.timeout=1;CHECK(run(p,120,3)==ULTRA_UCE_TIMEOUT);p.timeout=0;
    p.missing=1;CHECK(run(p,120,3)==ULTRA_UCE_ADC);p.missing=0;
    p.weak=1;CHECK(run(p,120,3)==ULTRA_UCE_WEAK);p.weak=0;
    p.nonlinear=1;CHECK(run(p,120,3)==ULTRA_UCE_NONLINEAR);p.nonlinear=0;
    /* the board's full length (its own pixel limits, the slowest bit) completes inside its own deadline with the
     * decided 15 % margin (the descriptor's cal_ms) */
    for(bpp=3;bpp<=5;bpp++) {
        p.bpp=bpp;p.n=board_max[bpp-3u];
        CHECK(run(p,p.n,bpp)==0);CHECK(c.model.detected==p.n);CHECK(c.model.flags&ULTRA_UCF_AT_LEAST);
        CHECK(last_ms*115u<=UC_DEADLINE_MS*100u);
        printf("  full length bpp %u: %u px in %u ms, x 1.15 = %u, deadline %u ms\n",bpp,p.n,last_ms,
               (last_ms*115u+99u)/100u,(unsigned)UC_DEADLINE_MS);
    }
    p.bpp=3;p.n=600;
    CHECK(run(p,600,3)==0);
    { udcal_model_t good=c.model;
      p.cancel=1;CHECK(run(p,600,3)==ULTRA_UCE_CANCELLED);
      CHECK(memcmp(&good,&c.model,sizeof(good))==0);p.cancel=0; }
    p.noise=60;CHECK(run(p,600,3)!=0);p.noise=0;
    p.n=0;CHECK(run(p,600,3)==ULTRA_UCE_EMPTY);
    { unsigned phase=c.phase,count=c.model.count;udcal_model_t cfg=c.model;
      udcal_request(&c,c.seq,&cfg,600,2250,0,0); CHECK(c.phase==phase); CHECK(c.model.count==count);
      udcal_request(&c,(uint8_t)(c.seq+1u),&cfg,600,2250,0,ULTRA_UCE_BUSY); CHECK(!c.pending);CHECK(c.phase==ULTRA_UCS_FAILED); }
    { udcal_model_t cfg=c.model;uint8_t wire[32];unsigned i;
      c.have_seq=0;udcal_request(&c,255,&cfg,600,65535,0,0);CHECK(c.limit==2250);
      udcal_abort(&c,ULTRA_UCE_CANCELLED);udcal_restored(&c);
      udcal_request(&c,0,&cfg,600,500,0,0);CHECK(c.phase==ULTRA_UCS_QUEUED && c.limit==500);
      udcal_feed(&c,100*20,20,65536,1,0,24000,11000);CHECK(c.error==ULTRA_UCE_SUPPLY);
      udcal_restored(&c);udcal_request(&c,1,&cfg,600,100,0,0);
      udcal_feed(&c,100*20,20,65536,1,0,24000,12000);CHECK(c.error==ULTRA_UCE_LIMIT);
      for(i=2;i<=6;i+=2) {
        static const uint16_t crc_golden[3]={0x9966,0xE632,0xCCFE};
        ultra_slot_pixel_calibration(wire,0,42,(uint8_t)i);
        CHECK(wire[0]==6 && wire[1]==0 && wire[2]==42 && wire[3]==i);
        CHECK(ultra_slot_check(wire)==0);
        CHECK(ultra_get16(wire+30)==crc_golden[i/2u-1u]);
        { unsigned j;for(j=4;j<30;j++) CHECK(wire[j]==0); }
      }
    }
    printf("udcal: %u checks, %u failures; state %u bytes, worst %u ms\n",checks,failures,(unsigned)sizeof(c),worst_ms);
    return failures?1:0;
}
