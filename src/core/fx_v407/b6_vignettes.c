/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* B6 vignettes: bounded object caches, continuous envelopes, no render-order state.
 * Each node is 16 bytes; 64 nodes fit even S(0). No global particle allocation is needed:
 * these are compact deterministic event replays, not frame histories. Colours/controls
 * follow catalogue_v407.json. JS: fx/v407/b6_vignettes.js, bit-exact golden checked.
 * B6-local candidates for B0: compact light kernel, smooth seeded event noise and
 * fractional duration clock (the latter follows B1's tested continuity convention). */
#include "fxv_gen.h"
typedef struct { fxv_px_t color; uint16_t a,b,c,d; } b6_node_t;
typedef char b6_node_is_16[sizeof(b6_node_t)==16u?1:-1];
static b6_node_t *b6_nodes(const fxv_ctx_t *x) { return (b6_node_t *)x->arena; }
static uint32_t b6_bytes(uint32_t n,uint32_t tier) { (void)n;(void)tier;return 1024u; }
static void b6_init(fxv_ctx_t *x) { (void)x; }
static uint32_t b6_min(uint32_t a,uint32_t b) { return a<b?a:b; }
static uint32_t b6_dist(uint32_t a,uint32_t b) { return a>b?a-b:b-a; }
static uint32_t b6_mul(uint32_t a,uint32_t b) { return a*b; }
static uint32_t b6_u32(uint32_t a) { return a; }
static uint32_t b6_div(uint32_t a,uint32_t b) { return a/b; }
static uint32_t b6_wave(uint32_t p) { return (uint32_t)(fxv_sin16(p)+32767); }
static uint32_t b6_soft(uint32_t d,uint32_t sharp) { return fxv_smooth16(65535u-b6_min(65535u,d*sharp)); }
static fxv_px_t b6_white(fxv_px_t c,uint32_t q,uint32_t k) { c.q=(uint8_t)q;c.k=(uint8_t)k;return c; }
/* A palette position for object j of num along the strip: a cyclic palette spaced round its loop, a clamped one from
 * its first entry (j = 0) to its last (j = num - 1). */
static uint32_t b6_along(const fxv_ctx_t *x,uint32_t j,uint32_t num)
{
    if(x->pal_wrap)return b6_div(j*65536u,num);
    return num>1u?b6_div(j*65535u,num-1u):32768u;
}
/* Few objects at fixed places would leave palette entries unseen, so their places move on by drift (65536 = one
 * round): a cyclic palette round its loop, a clamped one back and forth between its ends (no jump either way). */
static uint32_t b6_moved(const fxv_ctx_t *x,uint32_t u,uint32_t drift)
{
    if(x->pal_wrap)return (u+drift)&65535u;
    u=(u+drift)&131071u;
    return u<65536u?u:131071u-u;
}
static uint32_t b6_noise(uint32_t seed,uint32_t p)
{
    const uint32_t cell=p>>16;
    return fxv_lerp16(fxv_hash32(seed^cell)>>16,fxv_hash32(seed^(cell+1u))>>16,fxv_smooth16(p&65535u));
}
/* Duration endpoints exact, logarithmic-like interpolation between powers of two. */
static uint32_t b6_period(uint32_t lo,uint32_t hi,uint32_t speed)
{
    uint32_t oct=0u,base=lo,q;
    while(base*2u<=hi) { base*=2u;oct+=256u; }
    oct+=b6_div((hi-base)*256u,base);
    q=b6_div((255u-speed)*oct,255u);base=lo<<(q>>8);
    return b6_min(hi,base+((base*(q&255u))>>8));
}
static void b6_clock(fxv_ctx_t *x,uint32_t period)
{
    uint32_t dt=x->t_last-x->job[0];uint64_t a;
    if(dt>0x7FFFFFFFu||!x->speed)dt=0;
    if(x->job[3]&&x->job[3]!=period)x->job[2]=(uint32_t)((uint64_t)x->job[2]*period/x->job[3]);
    a=(uint64_t)dt*65536u+x->job[2];x->job[0]=x->t_last;
    x->job[2]=(uint32_t)(a%period);a=a/period+x->job[1];
    x->u[7]+=(uint32_t)(a>>16);x->job[1]=(uint32_t)a&65535u;x->job[3]=period;x->u[6]=x->job[1];
}
/* BODY_BEGIN */
static void b6_advance(fxv_ctx_t *x)
{
    uint32_t n=1u,period=0u;
    switch(x->id) {
    case 58: n=2u+b6_div(x->vlen,5u);break;
    case 59: n=2u+(x->intensity>>3);break;
    case 60: n=2u+((255u-x->custom1)>>4);period=b6_period(1000u,12000u,x->speed);break;
    case 70: n=5u+b6_div(x->custom1*25u,255u);period=b6_period(180000u,2700000u,x->speed);break;
    case 83: n=3u+((255u-x->intensity)>>3);break;
    case 85: n=3u+(x->intensity>>2);break;
    case 93: n=1u+b6_div(x->intensity*4u+127u,255u);period=b6_period(5000u,45000u,x->speed);break;
    case 100: n=64u;break;
    case 104: n=3u+b6_div(x->custom1*9u,255u);period=b6_div(60000u,60u+b6_div(x->speed*90u,255u));break;
    case 108: n=1u+(x->intensity>>4);break;
    case 110: n=2u+((255u-x->custom1)>>4);period=b6_period(5000u,60000u,x->speed)+6000u+b6_div(x->custom2*27000u,255u);break;
    case 111: n=1u+b6_div(x->intensity*31u,255u);break;
    }
    /* Short strips retain one local story instead of synchronising many objects. */
    if(x->id!=93u&&x->id!=100u)n=b6_min(n,1u+b6_div(x->vlen,4u));
    x->u[1]=b6_min(64u,n);x->u[0]=b6_div(x->u[1]*65536u,x->vlen);
    x->u[2]=x->et;x->u[3]=b6_div(65536u,x->vlen);x->u[4]=b6_div((x->et%90000u)*32768u,45000u);x->u[5]=0u;
    if(period)b6_clock(x,period);
    if(x->id==93u)x->u[0]=b6_div(65536u,x->vlen);
    if(x->id==100u)x->u[0]=b6_div(63u*65536u,x->vlen);
}

/* One cached object per scheduler slice. Replays depend on time/seed, never cadence. */
static void b6_prepare(fxv_ctx_t *x,uint32_t j,b6_node_t *s)
{
    const uint32_t h=fxv_rand(x->seed,0u,j),t=x->u[2],num=x->u[1];
    const uint32_t noise=b6_noise(h,b6_u32(b6_mul(t,9u)+(h&65535u)));
    uint32_t a=0u,b=0u,c=0u,d=0u;
    fxv_px_t col=fxv_rgb16(x->c[0]);
    switch(x->id) {
    case 58: {
        /* Each window has a slow independent fade and stable warm/cool identity.
         * The one TV window has local smooth irregular flicker; street lights persist. */
        const uint32_t q=b6_noise(h,b6_u32(b6_mul(t,2u)+(h&65535u)));
        const uint32_t threshold=65535u-x->intensity*200u;
        const uint32_t bedtime=x->custom1?b6_min(65535u,b6_div(t,100u+(255u-x->custom1)*20u)):0u;
        a=q>threshold?fxv_smooth16(b6_min(65535u,(q-threshold)*6u)):0u;
        a=fxv_scale16(a,65535u-bedtime);
        if(j%7u==0u)a=b6_min(65535u,a+x->custom2*110u);
        /* With a palette: a window's place along the street picks its palette colour, the TV walks the palette with
         * its flicker; the timers, the warm / cool white identity, bedtime and the street lights stay. */
        if(x->pal_own)col=b6_white(fxv_rgb16(x->c[(h&3u)==0u?1u:0u]),210u,(h&3u)==0u?230u:35u);
        else col=b6_white(fxv_pal(x,b6_along(x,j,num)),210u,(h&3u)==0u?230u:35u);
        if(j==(num>>1)) { col=x->pal_own?fxv_rgb16(x->c[2]):fxv_pal(x,noise);a=fxv_scale16(a,24000u+(noise>>1)); }
        col=fxv_scale_px(col,a);a=32768u;b=9u;break;
    }
    case 59: {
        const uint32_t breeze=b6_wave(b6_u32(b6_mul(t,4u)+(h>>8)));
        const uint32_t sway=b6_div(t,18000u)%num==j?b6_soft(b6_dist(t%18000u,9000u),8u):0u;
        a=32768u-fxv_scale16(sway,4000u)+fxv_scale16(fxv_scale16(breeze,sway),8000u);
        b=3u+((255u-x->custom1)>>5);
        col=b6_white(fxv_pal(x,h&65535u),80u,40u);
        col=fxv_scale_px(col,65535u-fxv_scale16(noise,x->custom2*100u));break;
    }
    case 60: {
        /* Delayed neighbour turns, pauses between phrases and occasional reverse replies. */
        const uint32_t step=x->u[7]%(num+3u),phase=x->u[6];
        const uint32_t reply=(fxv_rand(x->seed,b6_div(x->u[7],num+3u),19u)&255u)<x->custom2;
        const uint32_t active=step==j?b6_soft(b6_dist(phase,32768u),2u):0u;
        const uint32_t back=reply&&step==num-1u-j?b6_soft(b6_dist(phase,49000u),4u):0u;
        const uint32_t depth=fxv_scale16(b6_min(65535u,active+back),x->intensity*257u);
        /* With a palette: a lamp's place in the row picks its palette colour, a reply glows in the colour of the lamp
         * it answers, the room is the lamp's colour very dim (render); turns, pauses and depth stay. The places
         * move on two palette colours a turn (smoothly), so two lamps also show every colour within about a minute. */
        if(x->pal_own)col=b6_white(fxv_blend16(fxv_rgb16(x->c[0]),fxv_rgb16(x->c[2]),back),125u,40u);
        else {
            const uint32_t drift=(x->u[7]<<13)+(phase>>3);
            col=b6_white(fxv_blend16(fxv_pal(x,b6_moved(x,b6_along(x,j,num),drift)),
                fxv_pal(x,b6_moved(x,b6_along(x,num-1u-j,num),drift)),back),125u,40u);
        }
        col=fxv_scale_px(col,31000u+(depth>>1));a=32768u;b=3u;break;
    }
    case 70: {
        /* Firing travels through the pieces; memory extends the hold/cooling tail.
         * Individual cooling rates and rose-grey intermediate glaze retain identity. */
        const uint32_t phase=(x->u[6]+65536u-b6_div(j*11000u,num))&65535u;
        const uint32_t hold=18000u+x->custom2*60u+(h&4095u);
        uint32_t heat=phase<15000u?fxv_smooth16(b6_div(phase*65535u,15000u)):
            phase<hold?65535u:65535u-fxv_smooth16(b6_min(65535u,b6_div((phase-hold)*65535u,65536u-hold)));
        heat=fxv_scale16(heat,54000u+(h>>19));
        /* With a palette: a piece's heat is its palette position (cold = the first colour, hottest = the last); the
         * firing order, holds, cooling rates, white temperature and the dim resting glow stay. */
        if(x->pal_own) {
            col=fxv_blend16(fxv_rgb16(x->c[1]),fxv_rgb16(x->c[0]),b6_min(65535u,heat*2u));
            col=fxv_blend16(col,fxv_rgb16(x->c[2]),heat>32767u?(heat-32767u)*2u:0u);
        } else col=fxv_pal(x,b6_min(65535u,heat+(heat>>3)));
        col=b6_white(col,180u,235u-fxv_scale16(heat,200u));
        col=fxv_scale_px(col,65535u-fxv_scale16(65535u-heat,x->intensity*180u));
        a=32768u;b=2u;break;
    }
    case 83: {
        /* Sequential ignition; faults affect exactly one tube at a time. */
        const uint32_t restart=x->custom2?60000u+(255u-x->custom2)*1000u:0u;
        const uint32_t age=restart?t%restart:t;
        const uint32_t strike=j*420u;
        const uint32_t dt=age>strike?age-strike:0u;
        a=fxv_smooth16(b6_min(65535u,dt*120u));
        if(dt<1400u)a=fxv_scale16(a,35000u+(b6_wave(dt*280u)>>2));
        if(b6_div(t,2600u)%num==j) {
            const uint32_t fault=b6_soft(b6_dist(t%2600u,1300u),70u);
            a=fxv_scale16(a,65535u-fxv_scale16(fault,b6_div(x->speed*40000u,255u)));
        }
        /* With a palette: each tube is one palette colour by its place in the sign (hard-edged tubes) and steps on
         * 1.75 palette colours each time it re-lights, at the middle of each of its stutters ((t + 1300) / 2600
         * stutters have passed, every num-th is this tube's): of the steps tried, the one that lets 2-6 tubes show
         * every colour soonest (within 10 stutters). Strikes, stutters and the halo stay. */
        if(x->pal_own)col=fxv_rgb16(x->c[j%3u]);
        else col=fxv_pal(x,b6_moved(x,b6_along(x,j,num),b6_div(b6_div(t+1300u,2600u)+num-1u-j,num)*7168u));
        col=fxv_scale_px(col,a);a=32768u;b=2u;break;
    }
    case 85: {
        /* Each tile topples, rests, then stands in the selected order. */
        const uint32_t phase=t%(num*600u+5000u),head=j*300u;
        const uint32_t reset=num*300u+2200u+(x->custom2<128u?num-1u-j:j)*300u;
        uint32_t fall=phase<head?0u:phase<head+360u?fxv_smooth16(b6_div((phase-head)*65535u,360u)):65535u;
        if(phase>=reset)fall=phase>=reset+500u?0u:65535u-fxv_smooth16(b6_div((phase-reset)*65535u,500u));
        col=fxv_blend16(fxv_pal(x,(j*9100u)&65535u),fxv_rgb16(x->c[1]),fall);
        c=phase>head&&phase<head+600u?fxv_scale16(b6_soft(b6_dist(phase-head,300u),180u),x->custom1*200u):0u;
        a=32768u+fxv_scale16(fall,10000u);b=9u-fxv_scale16(fall,6u);break;
    }
    case 93: {
        /* Nested compact gesture replays, alternating direction, bounded to five.
         * The smaller traces are delayed and dimmer, with no frame-buffer history. */
        uint32_t scale=65535u,k;
        for(k=0u;k<j;k++)scale=fxv_scale16(scale,26000u+x->custom2*110u);
        const uint32_t ph=(x->u[6]+65536u-j*6000u)&65535u;
        const uint32_t reversed=fxv_lerp16(ph,65535u-ph,x->custom1*257u);
        const uint32_t travel=j&1u?reversed:ph;
        a=32768u-(scale>>1)+fxv_scale16(travel,scale);
        b=b6_div(65535u,700u+(scale>>4));
        c=b6_min(65535u,b6_min(ph,65535u-ph)*8u);
        c=fxv_scale16(c,scale);
        col=fxv_blend16(fxv_pal(x,(j*13500u+ph)&65535u),fxv_rgb16(x->c[2]),22000u);
        col=b6_white(col,60u,135u);break;
    }
    case 100: {
        /* A 64-knot smooth field: paired outward filaments share a bright core.
         * Seeded wandering tips and thin branching ridges never flash globally. */
        const uint32_t pos=b6_div(j*65535u,63u),dist=b6_dist(pos,32768u)*2u;
        const uint32_t branches=1u+b6_div(x->intensity*5u,255u);
        uint32_t glow=0u,tips=0u,k;
        for(k=0u;k<branches;k++) {
            const uint32_t hh=fxv_rand(x->seed,0u,k+300u);
            const uint32_t tip=14000u+x->custom1*100u+(b6_wave(b6_u32(t*3u+(hh>>8)))>>2);
            const uint32_t ridge=b6_wave(b6_u32(pos*(5u+k*2u)+b6_mul(t,7u)+hh));
            const uint32_t body=dist<tip?b6_soft(b6_dist(ridge,32768u),9u):0u;
            glow=b6_min(65535u,glow+(body>>2));
            tips=b6_min(65535u,tips+(b6_soft(b6_dist(dist,tip),20u)>>1));
        }
        glow=fxv_scale16(glow,65535u-fxv_scale16(noise,x->custom2*100u));
        /* With a palette: the distance from the core is the palette position (core = the first colour, the furthest
         * reach = the last); filaments, tips and the whiter core and tips stay. */
        if(x->pal_own)col=fxv_add_px(fxv_scale_px(fxv_rgb16(x->c[0]),glow),fxv_scale_px(b6_white(fxv_rgb16(x->c[1]),170u,220u),b6_min(65535u,tips+b6_soft(dist,12u))));
        else {
            const fxv_px_t pc=fxv_pal(x,b6_min(65535u,b6_div(dist*1024u,(30383u+x->custom1*100u)>>6)));
            col=fxv_add_px(fxv_scale_px(pc,glow),fxv_scale_px(b6_white(pc,170u,220u),b6_min(65535u,tips+b6_soft(dist,12u))));
        }
        break;
    }
    case 104: {
        /* Offset clubs answer across a four-beat phrase. Pulses have >=120 ms
         * attacks; swing delays the offbeat, and an accent walks one club at a time. */
        const uint32_t beat=x->u[7],phase=x->u[6];
        const uint32_t down=(beat+j)%4u==0u?b6_soft(b6_dist(phase,25000u),2u):0u;
        const uint32_t off=(beat+j)%4u==2u?b6_soft(b6_dist(phase,33000u+x->custom2*40u),2u):0u;
        const uint32_t pulse=b6_min(65535u,down+off);
        /* With a palette: a club's place picks its downbeat colour, its offbeat is the colour half a palette away, the
         * room is the club's colour very dim (render); beats, swing and the white accent stay. The places move on one
         * palette colour a bar (smoothly, 1/64 of a round a beat). */
        if(x->pal_own)col=fxv_blend16(fxv_rgb16(x->c[0]),fxv_rgb16(x->c[2]),off);
        else {
            const uint32_t u=b6_moved(x,b6_along(x,j,num),(beat<<10)+(phase>>6));
            col=fxv_blend16(fxv_pal(x,u),fxv_pal(x,(u+32768u)&65535u),off);
        }
        col=fxv_scale_px(col,14000u+fxv_scale16(pulse,x->intensity*180u));
        c=beat%num==j?b6_soft(b6_dist(phase,32768u),2u)>>3:0u;
        a=32768u;b=2u;break;
    }
    case 108: {
        /* Fixed ice body; a bead swells, detaches, accelerates and splashes. */
        const uint32_t phase=(b6_u32(t*5u)+(h&65535u))&65535u;
        a=19000u+x->custom1*65u;
        const uint32_t fall=phase<30000u?0u:b6_min(65535u,(phase-30000u)*2u);
        b=a+fxv_scale16(fxv_scale16(fall,fall),58000u-a);
        b=b6_min(57000u,b);
        c=phase<30000u?b6_div(phase*65535u,30000u):phase<58000u?65535u:65535u-fxv_smooth16(b6_div((phase-58000u)*65535u,7536u));
        d=phase>58000u?b6_soft(b6_dist(phase,61000u),14u):0u;
        col=b6_white(fxv_rgb16(x->c[0]),230u,245u);break;
    }
    case 110: {
        /* Independent gifts: the centre ribbon slides apart to uncover jewels;
         * open share selects eligible gifts, rewrap delay controls their hold. */
        const uint32_t phase=(x->u[6]+(h&65535u))&65535u;
        const uint32_t hold=3000u+b6_div(x->custom2*27000u,255u);
        const uint32_t age=b6_div(phase*(x->job[3]>>1),32768u);
        uint32_t open=age<1500u?fxv_smooth16(b6_div(age*65535u,1500u)):
            age<1500u+hold?65535u:age<3000u+hold?65535u-fxv_smooth16(b6_div((age-1500u-hold)*65535u,1500u)):0u;
        if((h>>24)>=x->intensity)open=0u;
        a=open;b=32768u;c=0u;col=fxv_pal(x,(h>>8)&65535u);break;
    }
    case 111: {
        /* The cold draught travels through fixed candle pools. Ghosts and eyes
         * are separate slow, rare local events with soft entry/exit. */
        const uint32_t draught=(b6_div(t,2u)+j*3300u)&65535u;
        const uint32_t gutter=b6_soft(b6_dist(draught,32768u),4u);
        col=fxv_blend16(fxv_rgb16(x->c[0]),fxv_pal(x,(j*9000u)&65535u),9000u);
        col=fxv_blend16(col,fxv_kelvin16(245u),gutter>>3);
        col=fxv_scale_px(col,45000u+(noise>>2)-fxv_scale16(gutter,28000u));
        a=32768u;b=6u;
        const uint32_t eyephase=(b6_div(t,2u)+(h&65535u))&65535u;
        c=(h>>24)<x->custom2?b6_soft(b6_dist(eyephase,32000u),10u):0u;
        d=(fxv_rand(x->seed,b6_div(t,90000u),33u)&255u)<x->custom1?b6_soft(b6_dist(t%90000u,45000u),2u):0u;
        break;
    }
    }
    s->color=col;s->a=(uint16_t)a;s->b=(uint16_t)b;s->c=(uint16_t)c;s->d=(uint16_t)d;
}
static uint32_t b6_slice(fxv_ctx_t *x,uint32_t quota)
{
    (void)quota;
    if(x->u[5]<x->u[1]) { const uint32_t j=x->u[5]++;b6_prepare(x,j,&b6_nodes(x)[j]); }
    return x->u[5]<x->u[1];
}
static void b6_render(const fxv_ctx_t *x,uint32_t from,uint32_t count,fxv_px_t *o)
{
    uint32_t i;
    for(i=0u;i<count;i++) {
        const uint32_t coord=(x->vbase+from+i)*x->u[0]+(x->u[0]>>1);
        const uint32_t j=b6_min(x->u[1]-1u,coord>>16),f=coord&65535u;
        const b6_node_t *s=&b6_nodes(x)[j];
        fxv_px_t col=fxv_rgb16(x->c[1]);
        if(x->id==93u) {
            uint32_t k;
            for(k=0u;k<x->u[1];k++) {
                const b6_node_t *echo=&b6_nodes(x)[k];
                col=fxv_add_px(col,fxv_scale_px(echo->color,fxv_scale16(b6_soft(b6_dist(f,echo->a),echo->b),echo->c)));
            }
        } else if(x->id==100u) {
            col=fxv_blend16(s->color,b6_nodes(x)[b6_min(63u,j+1u)].color,f);
        } else if(x->id==108u) {
            const uint32_t ice=f<s->a?b6_min(50000u,(s->a-f)*3u):0u;
            const uint32_t bead=fxv_scale16(b6_soft(b6_dist(f,s->b),16u),s->c);
            const uint32_t splash=fxv_scale16(b6_soft(b6_dist(f,58000u),6u),s->d);
            const uint32_t glint=fxv_scale16(b6_soft(b6_dist(f,12000u),24u),fxv_scale16(x->custom2*19u,b6_wave(b6_u32(b6_mul(x->u[2],21u)+j*15000u))));
            /* With a palette: the depth below the icicle's root is the palette position (root = the first colour, the
             * splash = the last), so a falling drop walks the palette; the night is that colour at the night's level. */
            if(x->pal_own)col=fxv_add_px(col,fxv_scale_px(s->color,b6_min(65535u,ice+bead+splash+glint)));
            else col=fxv_scale_px(b6_white(fxv_pal(x,f),230u,245u),b6_min(65535u,1500u+ice+bead+splash+glint));
        } else if(x->id==110u) {
            const uint32_t interior=fxv_scale16(b6_soft(b6_dist(f,32768u),3u),s->a);
            const uint32_t ribbon=b6_soft(b6_dist(b6_dist(f,32768u),fxv_scale16(s->a,27000u)),20u);
            col=fxv_blend16(col,s->color,interior);
            col=fxv_blend16(col,b6_white(fxv_rgb16(x->c[2]),80u,40u),ribbon);
            col=fxv_scale_px(col,22000u+fxv_scale16(b6_soft(b6_dist(f,32768u),2u),43535u));
        } else {
            const uint32_t body=b6_soft(b6_dist(f,s->a),s->b);
            if((x->id==60u||x->id==104u)&&!x->pal_own) {
                /* with a palette the room is the lamp's / club's own colour very dim: one scale, no blend */
                const uint32_t room=x->id==60u?7000u:6000u;
                col=fxv_scale_px(s->color,room+fxv_scale16(65535u-room,body));
            } else {
                if(x->id==58u||x->id==83u||x->id==85u)col=fxv_scale_px(col,0u);
                if(x->id==70u)col=fxv_scale_px(s->color,18000u);
                if(x->id==111u)col=fxv_scale_px(fxv_rgb16(x->c[1]),6000u);
                col=fxv_blend16(col,s->color,body);
            }
            if(x->id==83u)col=fxv_add_px(col,fxv_scale_px(s->color,fxv_scale16(b6_soft(b6_dist(f,32768u),1u),x->custom1*100u)));
            if(x->id==85u||x->id==104u)col=fxv_add_px(col,fxv_scale_px(fxv_kelvin16(230u),fxv_scale16(body,s->c)));
            if(x->id==111u) {
                const uint32_t eyes=b6_min(65535u,b6_soft(b6_dist(f,23000u),40u)+b6_soft(b6_dist(f,42000u),40u));
                const uint32_t p=(x->vbase+from+i)*x->u[3];
                const uint32_t ghost=b6_soft(b6_dist(p,x->u[4]),7u);
                col=fxv_add_px(col,fxv_scale_px(fxv_rgb16(x->c[0]),fxv_scale16(eyes,s->c)>>2));
                col=fxv_add_px(col,fxv_scale_px(b6_white(fxv_rgb16(x->c[2]),210u,230u),fxv_scale16(ghost,s->d)>>2));
            }
        }
        o[i]=col;
    }
}
/* BODY_END */

const fxv_desc_t fxv_d_town_at_night={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 58 */
const fxv_desc_t fxv_d_lantern_row={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 59 */
const fxv_desc_t fxv_d_lamp_conversation={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 60 */
const fxv_desc_t fxv_d_porcelain_kiln={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 70 */
const fxv_desc_t fxv_d_neon_sign={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 83 */
const fxv_desc_t fxv_d_domino_run={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 85 */
const fxv_desc_t fxv_d_folded_time={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 93 */
const fxv_desc_t fxv_d_plasma_globe={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 100 */
const fxv_desc_t fxv_d_pocket_rave={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 104 */
const fxv_desc_t fxv_d_icicle_drip={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 108 */
const fxv_desc_t fxv_d_ribbon_unwrap={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 110 */
const fxv_desc_t fxv_d_haunted_hall={b6_bytes,b6_init,b6_advance,b6_slice,b6_render,0,0,0}; /* 111 */
