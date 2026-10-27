/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* B2 surfaces. Fixed 80 ms dye ticks, immutable diffusion planes, fractional
 * coordinates and pure rendering. The first 256 arena bytes hold event/clock state;
 * Ink Bloom uses exactly six int16 dye planes (12*n), Glass Memory four (8*n).
 * B2-local B0 candidates: compact smooth kernel and fractional duration clock.
 * Catalogue colours, controls and neutral-white shares are preserved. */
#include "fxv_gen.h"
static uint32_t *b2_h(const fxv_ctx_t *x) { return (uint32_t *)x->arena; }
static int16_t *b2_p(const fxv_ctx_t *x,uint32_t plane) { return (int16_t *)(x->arena+256u)+plane*x->n; }
static uint32_t b2_min(uint32_t a,uint32_t b) { return a<b?a:b; }
static uint32_t b2_max(uint32_t a,uint32_t b) { return a>b?a:b; }
static uint32_t b2_dist(uint32_t a,uint32_t b) { return a>b?a-b:b-a; }
static uint32_t b2_mul(uint32_t a,uint32_t b) { return a*b; }
static uint32_t b2_u32(uint32_t a) { return a; }
static uint32_t b2_div(uint32_t a,uint32_t b) { return a/b; }
static uint32_t b2_age(uint32_t phase,uint32_t period) { return (uint32_t)(((uint64_t)phase*period)>>16); }
static uint32_t b2_fraction(uint32_t a,uint32_t b) { return a>=b?65535u:(uint32_t)((uint64_t)a*65535u/b); }
static uint32_t b2_mix(uint32_t a,uint32_t b,uint32_t f) { return (uint32_t)((int32_t)a+(((int32_t)b-(int32_t)a)*(int32_t)f>>8)); }
static uint32_t b2_wave(uint32_t a) { return (uint32_t)(fxv_sin16(a)+32767); }
static uint32_t b2_soft(uint32_t d,uint32_t slope) { const uint32_t m=b2_min(d,65535u)*b2_min(slope,32768u);return m>=65535u?0u:fxv_smooth16(65535u-m); }
static fxv_px_t b2_white(fxv_px_t c,uint32_t q,uint32_t k) { c.q=(uint8_t)q;c.k=(uint8_t)k;return c; }
/* A palette colour half way to natural white at k (as Newton Cradle's chrome): foam and lightning with a palette. */
static fxv_px_t b2_pale(fxv_px_t c,uint32_t k) { return b2_white(fxv_blend16(c,fxv_kelvin16(k),32768u),128u,k); }
#define B2_SEA 36000u   /* Shoreline / Tide Line with a palette: the sea's level (the water slots' ~45 %) */
/* Palette mode: the position opposite u, half a turn on a cyclic palette, the mirror image on an open one (no jump). */
static uint32_t b2_opp(const fxv_ctx_t *x,uint32_t u) { return x->pal_wrap?(u+32768u)&65535u:65535u-u; }
/* Log-like duration interpolation, exact endpoints, no floating point. */
static uint32_t b2_period(uint32_t lo,uint32_t hi,uint32_t speed)
{
    uint32_t oct=0u,base=lo,q;
    while(base*2u<=hi) {base*=2u;oct+=256u;}
    oct+=b2_div((hi-base)*256u,base);
    q=b2_div((255u-speed)*oct,255u);base=lo<<(q>>8);
    return b2_min(hi,base+((base*(q&255u))>>8));
}
/* Real-time duration clock, Q16 phase plus remainder: no cadence truncation, no
 * 16-bit millisecond timers. Parameters rescale the remainder without a phase jump. */
static void b2_clock(fxv_ctx_t *x,uint32_t period)
{
    uint32_t *h=b2_h(x),dt=x->t_last-h[0];uint64_t a;
    if(dt>0x7FFFFFFFu||!x->speed)dt=0u;
    if(h[3]&&h[3]!=period)h[2]=(uint32_t)((uint64_t)h[2]*period/h[3]);
    a=(uint64_t)dt*65536u+h[2];h[0]=x->t_last;h[2]=(uint32_t)(a%period);
    a=a/period+h[1];h[4]+=(uint32_t)(a>>16);h[1]=(uint32_t)a&65535u;h[3]=period;
}
static uint32_t b2_bytes_0(uint32_t n,uint32_t tier) {(void)n;(void)tier;return 256u;}
static uint32_t b2_bytes_2(uint32_t n,uint32_t tier) {(void)tier;return 256u+2u*n;}
static uint32_t b2_bytes_8(uint32_t n,uint32_t tier) {(void)tier;return 256u+8u*n;}
static uint32_t b2_bytes_12(uint32_t n,uint32_t tier) {(void)tier;return 256u+12u*n;}
static void b2_init(fxv_ctx_t *x) {b2_h(x)[0]=x->t_last;}
static fxv_px_t b2_dye(const fxv_ctx_t *x,uint32_t k) {
    const uint32_t *h=b2_h(x);fxv_px_t c;
    c.r=(uint16_t)h[20u+k*3u];c.g=(uint16_t)h[21u+k*3u];c.b=(uint16_t)h[22u+k*3u];c.q=0u;c.k=0u;return c;
}
/* BODY_BEGIN */
static void b2_advance(fxv_ctx_t *x)
{
    uint32_t *h=b2_h(x),period=8000u;
    if(x->id==40u)period=b2_div(60000u,b2_max(1u,x->custom1));
    if(x->id==41u)period=b2_period(3500u,16000u,x->speed);
    if(x->id==42u)period=b2_period(60000u,1800000u,x->speed);
    if(x->id==65u)period=b2_period(90000u,1200000u,x->speed)+10000u+b2_div(x->custom2*170000u,255u);
    if(x->id==66u)period=b2_period(3000u,90000u,x->speed);
    if(x->id==82u)period=b2_period(5000u,22000u,x->speed);
    if(x->id==96u)period=b2_div(240000u,60u+b2_div(x->speed*120u,255u));
    b2_clock(x,period);
    x->u[0]=b2_div(65536u,x->vlen);x->u[1]=0u;x->u[2]=0u;
    x->u[7]=b2_age(h[1],h[3]);x->u[3]=x->et;x->u[4]=h[1];x->u[5]=b2_div(524288u,x->vlen);
    if(x->id==51u||x->id==66u)x->u[5]=b2_div(x->tick_acc*256u,80u);
    if(x->id==51u) {
        uint32_t k;
        for(k=0u;k<3u;k++) {
            const fxv_px_t c=fxv_pal(x,k*21845u);
            h[20u+k*3u]=c.r;h[21u+k*3u]=c.g;h[22u+k*3u]=c.b;
        }
    }
    if(x->id==40u) {
        h[25]=b2_max(12u,b2_div(x->vlen*2u,3u));h[24]=b2_div(65535u,h[25]);
    }
    if(x->id==65u) {
        const uint32_t hold=10000u+b2_div(x->custom2*170000u,255u),move=h[3]-hold;
        const uint32_t age=b2_age(h[1],h[3]);
        x->u[4]=age<(move>>1)?fxv_smooth16(b2_fraction(age,move>>1)):
            age<(move>>1)+hold?65535u:65535u-fxv_smooth16(b2_fraction(age-(move>>1)-hold,move>>1));
    }
    /* These are frame invariants, prepared once, never divided per pixel. */
    if(x->id==41u||x->id==42u) {
        const uint32_t swash=32767u-(uint32_t)fxv_sin16(h[1]+16384u);
        uint32_t reach=8000u+x->intensity*200u;
        if(x->id==41u)reach=fxv_scale16(reach,65535u-fxv_scale16(b2_wave(b2_div(x->et,8u)),x->custom2*70u));
        x->u[4]=5000u+fxv_scale16(swash,reach);
        if(x->id==42u)x->u[4]+=fxv_scale16(b2_wave(b2_mul(x->et,4u)),x->custom1*12u);
        x->u[6]=b2_fraction(b2_min(x->de,2000u),x->id==41u?9000u:15000u+x->custom2*1000u);
    }
    if(x->id==96u) {x->u[6]=b2_div(h[1]*65535u,39000u+(x->custom1>>1)*90u);}
    if(x->id==66u) {
        const uint32_t rate=8u+((x->speed*15u)>>3);
        h[8]=b2_max(1u,b2_div(period*rate,20480u));
        h[9]=b2_max(1u,b2_div((15000u+b2_div(x->custom2*465000u,255u))*rate,20480u));
    }
}
/* Stable fields: fxv_diffuse's coefficient never exceeds 0.25. Work is eight cells
 * of one immutable plane at a time; swap only after all planes have completed. */
static uint32_t b2_slice(fxv_ctx_t *x,uint32_t quota)
{
    uint32_t *h=b2_h(x),pos=x->u[1],end=b2_min(x->n,pos+8u),i;
    (void)quota;
    if(x->id!=51u&&x->id!=66u) {
        if(x->id==40u) {
            const uint32_t k=pos,age=x->u[7]+k*h[3];
            h[8u+k*2u]=b2_min(65535u,(fxv_rand(x->seed,h[4]-k,40u)>>16)+age*3u);
            h[9u+k*2u]=x->custom1&&age<1800u?fxv_env(age,180u,100u,1520u):0u;
            x->u[1]++;return x->u[1]<8u;
        }
        /* Analytic excitation, retained material response. Multiplicative drying
         * uses elapsed effect time; front/glow rendering remains continuous. */
        int16_t *s=b2_p(x,0u);
        const uint32_t dt=b2_min(2000u,x->de);
        if(x->id==65u)return 0u;
        for(i=pos;i<end;i++) {
            const uint32_t p=(x->vbase+i)*x->u[0]+(x->u[0]>>1);
            uint32_t v=s[i],excite=0u,decay=0u;
            if(x->id==41u||x->id==42u) {
                excite=p<x->u[4]?32767u:0u;
                decay=x->u[6];
            } else if(x->id==48u) {
                const uint32_t wave=b2_wave(p*7u-b2_mul(x->et,7u));
                const uint32_t life=b2_soft(b2_dist(wave,45000u),9u);
                const uint32_t patch=b2_wave(p*19u+b2_mul(x->et,3u)+(fxv_hash32(x->seed+i)>>16));
                excite=fxv_scale16(fxv_scale16(life,patch),x->intensity*128u);
                decay=b2_min(65535u,dt*(30u+((255u-x->custom1)>>1)));
            } else if(x->id==49u) {
                const uint32_t pool=65535u-(6000u+x->custom1*90u);
                excite=p>pool?fxv_scale16(b2_wave(p*23u+b2_mul(x->et,35u)),x->intensity*128u):0u;
                decay=b2_min(65535u,dt*180u);
            } else if(x->id==82u) {
                const uint32_t center=fxv_rand(x->seed,h[4],82u)>>16;
                const uint32_t region=b2_soft(b2_dist(p,center),19u-(x->custom1>>4));
                const uint32_t age=x->u[7];
                const uint32_t stroke=fxv_env(age,140u,20u,180u)+fxv_env(age>400u?age-400u:0u,140u,0u,250u);
                excite=fxv_scale16(region,b2_min(65535u,stroke))>>1;
                decay=b2_min(65535u,dt*(20u+((255u-x->custom2)>>1)));
            } else if(x->id==96u) {
                const uint32_t phase=h[1],release=39000u+(x->custom1>>1)*90u;
                const uint32_t wave=phase<release?0u:b2_min(65535u,(phase-release)*4u);
                const uint32_t d=b2_dist(p,32768u)*2u;
                excite=phase<release?0u:fxv_scale16(b2_soft(b2_dist(d,wave),16u),x->intensity*128u);
                decay=b2_min(65535u,dt*(25u+((255u-x->custom2)>>1)));
            }
            v=fxv_scale16(v,65535u-decay);v=b2_max(v,excite);s[i]=(int16_t)b2_min(32767u,v);
        }
        x->u[1]=end;return end<x->n;
    }
    /* Simulation starts once per fixed tick, plus one bounded initial seed job. */
    if(!x->u[2]) {
        if(h[63]) {if(!fxv_tick_next(x))return 0u;} else h[63]=1u;
        x->u[2]=1u;x->u[1]=0u;pos=0u;end=b2_min(x->n,8u);
        h[12]++;h[13]=0u;
        if(x->id==51u) {
            h[14]=b2_max(6u,100u-(x->intensity>>2));
            const uint32_t event=b2_div(x->tick,h[14]);
            h[15]=fxv_rand(x->seed,event,51u)>>16;
            h[16]=event%3u;
            h[17]=x->intensity?fxv_env((x->tick%h[14])*80u,240u,80u,400u)>>1:0u;
        } else {
            if(h[12]>=h[8]) {h[12]=0u;h[11]++;}
            h[15]=fxv_rand(x->seed,h[11],66u)>>16;
            h[17]=fxv_scale16(fxv_env(h[12]*80u,240u,100u,500u),x->intensity*48u);
        }
    }
    {
        const uint32_t channels=x->id==51u?3u:2u,channel=h[13],bank=h[10];
        const int16_t *old=b2_p(x,bank*channels+channel);
        int16_t *next=b2_p(x,(1u-bank)*channels+channel);
        fxv_diffuse(old,next,x->n,b2_div(x->custom1*1024u,255u),pos,end,FXV_EDGE_CLAMP);
        for(i=pos;i<end;i++) {
            const uint32_t p=(x->vbase+i)*x->u[0]+(x->u[0]>>1);
            uint32_t v=next[i];
            const uint32_t drop=b2_soft(b2_dist(p,h[15]),35u);
            if(x->id==51u) {
                /* Convex upwind drift: diffusion and advection remain bounded. */
                const uint32_t upstream=i?old[i-1u]:old[i];
                v=fxv_lerp16(v,upstream,x->custom2*96u);
                v=fxv_scale16(v,65395u);
                if(channel==h[16])v=b2_min(32767u,v+fxv_scale16(drop,h[17]));
            } else {
                const uint32_t fade=b2_max(1u,b2_div(65535u,h[9]));
                v=fxv_scale16(v,65535u-fade);
                if(channel==0u) {
                    v=fxv_scale16(v,64625u);
                    v=b2_min(32767u,v+fxv_scale16(drop,h[17]));
                } else {
                    const uint32_t fresh=b2_p(x,bank*2u)[i];
                    v=b2_min(32767u,v+fxv_scale16(fresh,910u));
                }
            }
            next[i]=(int16_t)v;
        }
        if(end<x->n) {x->u[1]=end;return 1u;}
        x->u[1]=0u;h[13]++;
        if(h[13]<channels)return 1u;
        h[10]=1u-bank;x->u[2]=0u;return x->tick_frame!=0u;
    }
}
static void b2_render(const fxv_ctx_t *x,uint32_t from,uint32_t count,fxv_px_t *o)
{
    const uint32_t *h=b2_h(x);uint32_t j;fxv_nz_t z;
    /* 41 / 42 with a palette: 2^27 / the waterline, so the sea's coordinate runs 0 offshore .. 32768 at the waterline */
    const uint32_t seak=!x->pal_own&&(x->id==41u||x->id==42u)?b2_div(134217728u,x->u[4]):0u;
    if(x->id==65u||x->id==82u)fxv_nz_begin(&z,x->seed,x->id==65u?0u:b2_mul(x->et,2u),1u);
    for(j=0u;j<count;j++) {
        const uint32_t i=from+j,vp=x->vbase+i,p=vp*x->u[0]+(x->u[0]>>1),t=x->et;
        fxv_px_t col=fxv_rgb16(x->c[0]);
        if(x->id==40u) {
            /* Independent, slow coal oscillators and dark stable fissures. */
            const uint32_t hash=fxv_hash32(x->seed^(vp>>1));
            const uint32_t heat=b2_wave(b2_mul(t,2u+(hash>>29))+(hash>>16));
            const uint32_t crack=(hash&255u)<x->custom2?15000u:65535u;
            const uint32_t hot=fxv_scale16(heat,30000u+x->intensity*125u);
            col=fxv_scale_px(fxv_pal(x,8000u+fxv_scale16(hot,56000u)),fxv_scale16(22000u+(heat>>1),crack));
            if(x->fmt==5u)col=b2_white(col,(hot>>11)>25u?45u:0u,0u);
            /* At most eight recent sparks; each is local with a soft launch. */
            if(x->custom1) {
                uint32_t k;
                for(k=0u;k<8u;k++) {
                    const uint32_t distance=b2_dist(p,h[8u+k*2u]),amp=h[9u+k*2u];
                    if(amp&&distance<h[24]) {
                        const uint32_t shape=b2_soft(distance,h[25]);
                        col=fxv_add_px(col,fxv_scale_px(fxv_rgb16(x->c[2]),fxv_scale16(shape,amp)));
                    }
                }
            }
        } else if(x->id==41u||x->id==42u) {
            const uint32_t wet=b2_p(x,0u)[i]*2u,front=x->u[4];
            const uint32_t sand=x->id==41u?1u:2u,foam=x->id==41u?2u:1u;
            const uint32_t water=p<front?65535u-b2_soft(front-p,12u):0u;
            const uint32_t fringe=b2_soft(b2_dist(p,front),55u);
            const uint32_t foamamt=x->id==41u?x->custom1*257u:48000u;
            if(x->pal_own) {
            col=fxv_scale_px(b2_white(fxv_rgb16(x->c[sand]),90u,45u),65535u-fxv_scale16(wet,34000u));
            col=fxv_blend16(col,fxv_scale_px(fxv_rgb16(x->c[0]),46000u+((b2_wave(p*15u-b2_mul(t,5u)))>>2)),water);
            col=fxv_blend16(col,b2_white(fxv_rgb16(x->c[foam]),245u,240u),fxv_scale16(fringe,foamamt));
            } else {
                /* Palette: the sea takes the palette's first half from offshore to the waterline (it stretches with the
                 * swash, so all of it shows at every reach), the sand its second half along the beach, the foam is the
                 * sea colour paled; swash, wetness, ripple and brightness stay. */
                const fxv_px_t sea=fxv_pal(x,b2_min(65535u,(p*seak)>>12));
                col=fxv_scale_px(fxv_pal(x,(p>>1)+32768u),fxv_scale16(48000u,65535u-fxv_scale16(wet,34000u)));
                const uint32_t fw=fxv_scale16(fringe,foamamt);
                if(water)col=fxv_blend16(col,fxv_scale_px(sea,fxv_scale16(46000u+((b2_wave(p*15u-b2_mul(t,5u)))>>2),B2_SEA)),water);
                if(fw)col=fxv_blend16(col,b2_pale(sea,240u),fw);
            }
        } else if(x->id==48u) {
            const uint32_t glow=b2_p(x,0u)[i]*2u;
            const uint32_t sparkle=fxv_scale16(b2_soft(b2_dist(b2_wave(b2_mul(t,9u)+(fxv_hash32(x->seed^vp)>>16)),60000u),20u),x->custom2*80u);
            if(x->pal_own) {
            col=fxv_blend16(fxv_rgb16(x->c[1]),fxv_rgb16(x->c[0]),glow);
            col=fxv_add_px(col,fxv_scale_px(fxv_rgb16(x->c[0]),fxv_scale16(sparkle,glow)));
            } else {
                /* Palette: position along the strip. The dark sea is that colour at ~3 %, the glow lights it to full,
                 * the sparkle adds on top; waves, glow and afterglow stay. */
                const fxv_px_t c=fxv_pal(x,p);
                col=fxv_blend16(fxv_scale_px(c,2056u),c,glow);
                col=fxv_add_px(col,fxv_scale_px(c,fxv_scale16(sparkle,glow)));
            }
            if(x->fmt==5u&&glow>54000u)col=b2_white(col,10u,255u);
        } else if(x->id==49u) {
            const uint32_t flow=b2_wave(p*30u-b2_mul(t,45u));
            const uint32_t streak=fxv_scale16(b2_soft(b2_dist(flow,60000u),5u),x->intensity*180u);
            const uint32_t pool=65535u-(6000u+x->custom1*90u),foam=b2_p(x,0u)[i]*2u;
            const uint32_t mist=p<pool?fxv_scale16(b2_soft(pool-p,7u),x->custom2*80u):0u;
            if(x->pal_own) {
            col=fxv_scale_px(col,26000u+(flow>>1));
            col=fxv_blend16(col,b2_white(fxv_rgb16(x->c[1]),245u,240u),b2_min(65535u,streak+foam+mist));
            } else {
                /* Palette: height down the fall (start = top), the whole palette top to pool; streaks, foam and
                 * mist pale that colour; flow, pool and mist stay. */
                const fxv_px_t c=fxv_pal(x,p);
                col=fxv_scale_px(c,fxv_scale16(26000u+(flow>>1),41000u));
                const uint32_t fw=b2_min(65535u,streak+foam+mist);
                if(fw)col=fxv_blend16(col,b2_pale(c,240u),fw);
            }
        } else if(x->id==51u) {
            const uint32_t bank=h[10]*3u,prior=(1u-h[10])*3u;
            const uint32_t a=b2_mix(b2_p(x,prior)[i],b2_p(x,bank)[i],x->u[5])*2u;
            const uint32_t b=b2_mix(b2_p(x,prior+1u)[i],b2_p(x,bank+1u)[i],x->u[5])*2u;
            const uint32_t c=b2_mix(b2_p(x,prior+2u)[i],b2_p(x,bank+2u)[i],x->u[5])*2u;
            const fxv_px_t ca=b2_dye(x,0u),cb=b2_dye(x,1u),cc=b2_dye(x,2u);
            const uint32_t coverage=b2_min(65535u,a+b+c);
            /* Multiply dye transmission: overlaps darken instead of bleaching. */
            col=fxv_scale_px(fxv_kelvin16(140u),14000u+fxv_scale16(coverage,51535u));
            col.r=(uint16_t)fxv_scale16(col.r,fxv_scale16(fxv_scale16(65535u-a+fxv_scale16(ca.r,a),65535u-b+fxv_scale16(cb.r,b)),65535u-c+fxv_scale16(cc.r,c)));
            col.g=(uint16_t)fxv_scale16(col.g,fxv_scale16(fxv_scale16(65535u-a+fxv_scale16(ca.g,a),65535u-b+fxv_scale16(cb.g,b)),65535u-c+fxv_scale16(cc.g,c)));
            col.b=(uint16_t)fxv_scale16(col.b,fxv_scale16(fxv_scale16(65535u-a+fxv_scale16(ca.b,a),65535u-b+fxv_scale16(cb.b,b)),65535u-c+fxv_scale16(cc.b,c)));
            col.q=(uint8_t)((65535u-coverage)>>8);
        } else if(x->id==65u) {
            const uint32_t texture=(uint32_t)(fxv_nz_at(&z,p*8u)+32767);
            const uint32_t wear=x->u[4];
            const uint32_t threshold=65535u-fxv_scale16(wear,x->intensity*230u);
            const uint32_t exposure=texture>threshold?b2_min(65535u,(texture-threshold)*(2u+((255u-x->custom1)>>4))):0u;
            const uint32_t edge=fxv_scale16(b2_soft(b2_dist(texture,threshold),12u),x->intensity*40u);
            if(x->pal_own) {
            col=fxv_blend16(col,fxv_rgb16(x->c[1]),fxv_smooth16(exposure));
            col=fxv_blend16(col,b2_white(fxv_rgb16(x->c[2]),80u,60u),edge);
            col=fxv_scale_px(col,48000u+(texture>>2));
            } else {
                /* Palette: position along the strip. The velvet runs through the whole palette, the underlayer and its
                 * edge sheen take the colour opposite (b2_opp); texture, wear, edges and the matte levels stay. */
                const fxv_px_t under=fxv_pal(x,b2_opp(x,p));
                col=fxv_scale_px(fxv_pal(x,p),16200u);
                if(exposure)col=fxv_blend16(col,fxv_scale_px(under,17200u),fxv_smooth16(exposure));
                if(edge)col=fxv_blend16(col,fxv_scale_px(under,29800u),edge);
                col=fxv_scale_px(col,48000u+(texture>>2));
            }
        } else if(x->id==66u) {
            const uint32_t bank=h[10]*2u,prior=(1u-h[10])*2u;
            const uint32_t fresh=b2_mix(b2_p(x,prior)[i],b2_p(x,bank)[i],x->u[5])*2u;
            const uint32_t old=b2_mix(b2_p(x,prior+1u)[i],b2_p(x,bank+1u)[i],x->u[5])*2u;
            if(x->pal_own) {
            col=fxv_rgb16(x->c[1]);
            col=fxv_blend16(col,b2_white(fxv_rgb16(x->c[2]),100u,113u),old);
            col=fxv_blend16(col,b2_white(fxv_rgb16(x->c[0]),160u,150u),fresh);
            } else {
                /* Palette: position along the strip. Old traces and the glass (at ~7 %) run through the whole palette,
                 * fresh touches take the colour opposite (b2_opp); touches, spread and memory stay. */
                const fxv_px_t lo=fxv_pal(x,p);
                col=fxv_scale_px(lo,4600u);
                if(old)col=fxv_blend16(col,fxv_scale_px(lo,28500u),old);
                if(fresh)col=fxv_blend16(col,fxv_scale_px(fxv_pal(x,b2_opp(x,p)),50100u),fresh);
            }
        } else if(x->id==82u) {
            const uint32_t cloud=(uint32_t)(fxv_nz_at(&z,p*6u)+32767);
            const uint32_t rain=fxv_scale16(b2_soft(b2_dist(b2_wave(p*48u-b2_mul(t,80u)),62000u),20u),x->intensity*180u);
            const uint32_t flash=b2_p(x,0u)[i]*2u;
            if(x->pal_own) {
            col=fxv_scale_px(col,22000u+(cloud>>1));
            col=fxv_add_px(col,fxv_scale_px(fxv_rgb16(x->c[1]),rain>>1));
            col=fxv_blend16(col,b2_white(fxv_rgb16(x->c[1]),250u,255u),flash);
            } else {
                /* Palette: position along the strip warped by the rolling cloud (a quarter of the palette). Clouds
                 * at ~15 %, rain in that colour, the lightning lights it paled; clouds, rain and strokes stay. */
                const fxv_px_t c=fxv_pal(x,fxv_scale16(p,49151u)+(cloud>>2));
                col=fxv_scale_px(c,fxv_scale16(22000u+(cloud>>1),10000u));
                if(rain>>1)col=fxv_add_px(col,fxv_scale_px(c,rain>>1));
                if(flash)col=fxv_blend16(col,b2_pale(c,255u),flash);
            }
        } else if(x->id==96u) {
            const uint32_t phase=h[1],release=39000u+(x->custom1>>1)*90u;
            const uint32_t build=phase<release?x->u[6]:0u;
            const uint32_t d=b2_dist(p,32768u)*2u;
            const uint32_t charge=fxv_scale16(b2_soft(b2_dist(d,65535u-build),12u),20000u+(build>>1));
            const uint32_t after=fxv_scale16(b2_p(x,0u)[i]*2u,56000u+(b2_wave(p*30u+b2_mul(t,8u))>>3));
            col=fxv_scale_px(fxv_pal(x,(p+phase)&65535u),charge);
            col=fxv_blend16(col,fxv_rgb16(x->c[0]),charge>>2);
            col=fxv_add_px(col,fxv_scale_px(b2_white(fxv_rgb16(x->c[1]),250u,240u),after));
        }
        o[j]=col;
    }
}
/* BODY_END */
static uint32_t b2_digest(const fxv_ctx_t *x) {
    uint32_t i,v=0u;for(i=0u;i<x->need;i++)v=fxv_hash32(v^x->arena[i]);return v;
}
const fxv_desc_t fxv_d_ember_bed={b2_bytes_0,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 40 */
const fxv_desc_t fxv_d_shoreline={b2_bytes_2,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 41 */
const fxv_desc_t fxv_d_tide_line={b2_bytes_2,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 42 */
const fxv_desc_t fxv_d_bioluminescence={b2_bytes_2,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 48 */
const fxv_desc_t fxv_d_waterfall={b2_bytes_2,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 49 */
const fxv_desc_t fxv_d_ink_bloom={b2_bytes_12,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 51 */
const fxv_desc_t fxv_d_velvet_erosion={b2_bytes_0,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 65 */
const fxv_desc_t fxv_d_glass_memory={b2_bytes_8,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 66 */
const fxv_desc_t fxv_d_thunderstorm={b2_bytes_2,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 82 */
const fxv_desc_t fxv_d_bass_drop={b2_bytes_2,b2_init,b2_advance,b2_slice,b2_render,0,0,b2_digest}; /* 96 */
