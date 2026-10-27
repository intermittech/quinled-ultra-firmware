/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "../math32.h"
#include <string.h>
#include <stddef.h>
#include "udcal.h"
#include "udfx.h"                       /* ULTRA_PX_MAX_*: the board's own pixel limits */
#include "../../board/boards.h"
#include "../lto.h"
#include "udpack.h"

enum { COUNT_FIRST, COUNT_SEARCH, COUNT_LAST, COUNT_AFTER, BOUNDS,
       CHANNEL, MIX_DARK, MIX_RGB, MIX_ALL, FINAL_DARK };
static const uint8_t duties[9] = {0,64,128,255,0,255,128,64,0};
static uint32_t umax(uint32_t a, uint32_t b) { return a > b ? a : b; }
static NOINLINE uint32_t diff(uint32_t a, uint32_t b) { return a > b ? a-b : b-a; }   /* (nine sites) */
/* Shared reduction guards and pattern setup keep repeated stages compact. */
static NOINLINE uint32_t tolerance(uint32_t v) { return umax(5u*256u, v/10u); }
static uint32_t counting(const udcal_t *c) { return c->phase==ULTRA_UCS_PROBE && c->stage<=COUNT_AFTER; }
uint32_t udcal_active(const udcal_t *c)
{
    return c->phase >= ULTRA_UCS_QUEUED && c->phase <= ULTRA_UCS_RESTORE;
}
void udcal_invalidate(udcal_t *c)
{
    if (c->model.flags & ULTRA_UCF_VALID) {
        c->model.flags &= 0xFEu;
        c->model.epoch++;
    }
}
static NOINLINE void pattern(udcal_t *c, uint32_t duty, uint32_t mask, uint32_t k, uint32_t pos)
{
    c->duty=(uint8_t)duty; c->mask=(uint8_t)mask;
    c->k=(uint16_t)k; c->pos=(uint16_t)pos;
    c->pending=1; c->sampling=0; c->finish=0;
}
static NOINLINE void dark_pattern(udcal_t *c) { pattern(c,0,0,0,0); }   /* (every stage's dark reference, an abort: one call) */
void udcal_abort(udcal_t *c, uint8_t error)
{
    if (!udcal_active(c) || c->phase == ULTRA_UCS_RESTORE) return;
    c->error=error;
    c->phase=ULTRA_UCS_RESTORE;
    dark_pattern(c);
}
void udcal_request(udcal_t *c, uint8_t seq, const udcal_model_t *cfg,
                   uint16_t cap, uint16_t limit, uint32_t ms, uint8_t error)
{
    if (c->have_seq && c->seq == seq) return;
    c->seq=seq; c->have_seq=1;
    if (udcal_active(c)) { udcal_abort(c,ULTRA_UCE_BUSY); return; }
    memset(&c->candidate,0,sizeof(*c)-offsetof(udcal_t,candidate));
    c->seq=seq; c->have_seq=1; c->candidate=*cfg;
    c->candidate.flags=ULTRA_UCF_AGGREGATE;
    c->candidate.epoch=c->model.epoch;
    c->start_ms=c->now_ms=c->last_ms=ms;
    c->cap=cap; c->limit=limit && limit<2250u ? limit : 2250u;
    if (!cfg->configured || cfg->configured>cap || cfg->bpp<3u || cfg->bpp>5u)
        error=ULTRA_UCE_CONFIG;
    c->error=error;
    c->phase=error ? ULTRA_UCS_FAILED : ULTRA_UCS_QUEUED;
    if (!error) dark_pattern(c);
}
static NOINLINE void restart(udcal_t *c, uint32_t ms)
{
    c->block_ms=ms; c->sum=0; c->samples=0;
    c->lo_adc=0xFFFFFFFFu; c->hi_adc=0; c->sampling=1;
}
void udcal_latched(udcal_t *c, uint32_t ms)
{
    c->pending=0;
    if (c->phase==ULTRA_UCS_RESTORE) { c->finish=1; return; }
    if (c->phase<=ULTRA_UCS_ACQUIRE) c->phase=ULTRA_UCS_DARK;
    restart(c,ms);
}
void udcal_restored(udcal_t *c)
{
    if (!c->error) {
        c->candidate.count=(uint16_t)(c->model.count+1u);
        c->candidate.epoch=(uint16_t)(c->model.epoch+1u);
        c->candidate.flags |= ULTRA_UCF_VALID|ULTRA_UCF_PENDING;
        c->model=c->candidate;
        c->model_ms=c->now_ms;
    }
    c->phase=c->error ? ULTRA_UCS_FAILED : ULTRA_UCS_DONE;
    c->elapsed=(uint16_t)(c->now_ms-c->start_ms);
}
static NOINLINE uint32_t supply_bad(uint32_t ref,uint32_t v,uint32_t pct)
{
    return !ref || !v || v>65535u || diff(ref,v)*100u>ref*pct;
}
void udcal_feed(udcal_t *c, uint32_t s, uint32_t n, uint32_t scale,
                uint32_t ms, uint32_t blocked, uint32_t vbus, uint32_t vled)
{
    uint32_t ma, raw;
    c->now_ms=ms;
    if (!udcal_active(c)) return;
    c->elapsed=(uint16_t)(ms-c->start_ms);
    if (c->phase==ULTRA_UCS_RESTORE) return;
    if (ms-c->start_ms>=UC_DEADLINE_MS) { udcal_abort(c,ULTRA_UCE_TIMEOUT); return; }
    if (!n || n>60u || ms-c->last_ms>2u) { udcal_abort(c,ULTRA_UCE_ADC); return; }
    c->last_ms=ms;
    if (supply_bad(c->candidate.vbus,vbus,2u) || supply_bad(c->candidate.vled,vled,2u)) {
        udcal_abort(c,ULTRA_UCE_SUPPLY); return;
    }
    ma=m32_divshr(s,scale,n,8);
    if (ma>c->limit*256u*4u/5u) { udcal_abort(c,ULTRA_UCE_LIMIT); return; }
    if (!c->sampling) return;
    if (blocked) { restart(c,ms); return; }
    if (c->sampling==1u && ms-c->block_ms<=(counting(c)?4u:16u)) return;
    c->sum+=s; c->samples=(uint16_t)(c->samples+n);
    raw=(s<<8)/n;
    if (raw<c->lo_adc) c->lo_adc=raw;
    if (raw>c->hi_adc) c->hi_adc=raw;
}
/* Conservative full-duty bound, inferred at one pixel/byte 64: 50% margin,
 * plus the observed noise. V_LED/V_bus changes are guarded separately at 2%.
 * The safety proof is empirical: this does not invent a rated strip current. */
static NOINLINE uint32_t bound(udcal_t *c,uint32_t delta)
{
    return (delta*(255u*3u)+(2u*UC_DUTY-1u))/(2u*UC_DUTY) + (uint32_t)c->candidate.noise*256u;
}
static NOINLINE uint32_t threshold(const udcal_t *c)
{
    uint32_t a=c->limit*7u/10u,b=c->limit>200u ? c->limit-200u : 0u;
    return (a<b?a:b)*256u;
}
static NOINLINE uint32_t probe_floor(const udcal_t *c)
{
    /* Two 6-ms means: threshold at least the observed ms peak range. */
    return umax(256u,(uint32_t)c->candidate.noise*256u/3u);
}
static uint32_t admit(udcal_t *c,uint32_t k,uint32_t channels)
{
    uint32_t predicted=c->upper*k*channels;
    if (c->dark[0]+umax(50u,c->candidate.noise)*256u+predicted>=threshold(c)) {
        udcal_abort(c,ULTRA_UCE_LIMIT); return 0;
    }
    c->prediction=predicted*c->duty/255u;
    return 1;
}
static void channels_begin(udcal_t *c)
{
    uint32_t baseline=c->dark[0]+umax(50u,c->candidate.noise)*256u,room,k;
    if(baseline>=threshold(c)) {udcal_abort(c,ULTRA_UCE_LIMIT);return;}
    room=threshold(c)-baseline;k=room/c->upper;
    if (k>32u) k=32u;
    if (k>c->candidate.n) k=c->candidate.n;
    if (!k) { udcal_abort(c,ULTRA_UCE_LIMIT); return; }
    c->candidate.k=(uint16_t)k;
    c->candidate.bound=(uint16_t)((c->upper+255u)/256u);
    c->phase=ULTRA_UCS_CHANNELS; c->stage=CHANNEL; c->index=0; c->channel=0;
    pattern(c,0,0,k,0);
}
static void count_done(udcal_t *c)
{
    c->candidate.detected=c->lo;
    c->candidate.n=c->lo<c->candidate.configured ? c->lo : c->candidate.configured;
    if (c->lo<c->candidate.configured) c->candidate.flags|=ULTRA_UCF_SHORT;
    if (c->lo==c->cap) c->candidate.flags|=ULTRA_UCF_AT_LEAST;
    if (c->model.detected!=c->lo) udcal_invalidate(c);
    c->channel=1; c->stage=BOUNDS;
    /* The first red probe already provided its bound. Other channels start alone. */
    c->prediction=0;
    pattern(c,UC_DUTY,2,1,1);
}
/* Fit each pass after its trailing dark; interpolate with actual block times.
 * All regression inputs are Q8 mA; exact product quotients use 32-bit limbs. */
static int32_t interpolate(uint32_t a,uint32_t b,uint32_t t,uint32_t span)
{
    uint32_t d=a>b?a-b:b-a,q=m32_muldiv(d,t,span?span:1u);
    return (int32_t)a+(b>=a?(int32_t)q:-(int32_t)q);
}
static void fit(udcal_t *c)
{
    uint32_t pass=c->index==4u ? 0u:1u, j, num=0;
    const uint32_t den=2u*(64u*64u+128u*128u+255u*255u);
    for(j=0;j<3u;j++) {
        uint32_t li=pass*3u+j, at=pass ? 5u+j:1u+j;
        uint32_t t0=c->stamp[pass*4u], dt=(uint16_t)(c->stamp[pass*4u+4u]-t0);
        int32_t base=interpolate(c->dark[pass],c->dark[pass+1u],(uint16_t)(c->stamp[at]-t0),dt);
        int32_t delta=(int32_t)c->light[li]-base;
        if (delta<(int32_t)umax(10u*256u,(uint32_t)c->candidate.noise*256u)) {
            udcal_abort(c,ULTRA_UCE_WEAK); return;
        }
        c->light[li]=(uint32_t)delta;
        if (pass) {
            uint32_t prev=c->light[2u-j];
            if (diff(prev,(uint32_t)delta)>tolerance(prev)) { udcal_abort(c,ULTRA_UCE_NOISE); return; }
        }
    }
    if (!pass) return;
    for(j=0;j<6u;j++) {
        uint32_t d=duties[j<3u ? j+1u:j+2u];
        num+=d*c->light[j];
    }
    for(j=0;j<6u;j++) {
        uint32_t d=duties[j<3u ? j+1u:j+2u], pred=m32_muldiv(num,d,den);
        uint32_t residual=diff(pred,c->light[j]);
        if (residual>tolerance(pred)) { udcal_abort(c,ULTRA_UCE_NONLINEAR); return; }
        c->candidate.residual=(uint16_t)umax(c->candidate.residual,residual*1000u/(pred?pred:1u));
    }
    {
        uint32_t coef=m32_maddiv(num,c->candidate.n*255u,den*c->k*128u,den*c->k*256u);
        if (coef>65535u) { udcal_abort(c,ULTRA_UCE_OVERFLOW); return; }
        c->candidate.ch[c->channel]=(uint16_t)coef;
    }
}
static void evaluate(udcal_t *c)
{
    uint32_t v=c->value, delta=v>c->dark[0] ? v-c->dark[0]:0u;
    if (c->phase==ULTRA_UCS_DARK) {
        c->dark[0]=v;
        c->stamp[0]=(uint16_t)(c->now_ms-c->start_ms);
        c->candidate.idle=(int16_t)((v+128u)/256u);
        if (v+umax(50u,c->candidate.noise)*256u>=threshold(c)) { udcal_abort(c,ULTRA_UCE_LIMIT); return; }
        c->phase=ULTRA_UCS_PROBE; c->stage=COUNT_FIRST;
        pattern(c,UC_DUTY,1,1,1); return;
    }
    if (c->stage==CHANNEL && c->duty && c->prediction && delta>c->prediction+tolerance(c->prediction)) {
        udcal_abort(c,ULTRA_UCE_PREDICTION); return;
    }
    if (c->stage<=COUNT_AFTER) {
        uint32_t lit, t=(uint16_t)(c->now_ms-c->start_ms), dt=(uint16_t)(t-c->stamp[0]);
        int32_t base;
        if(c->duty) {
            c->light[0]=v; c->stamp[1]=(uint16_t)t;
            pattern(c,0,0,0,c->pos); return;
        }
        base=interpolate(c->dark[0],v,(uint16_t)(c->stamp[1]-c->stamp[0]),dt);
        delta=c->light[0]>(uint32_t)base ? c->light[0]-(uint32_t)base:0u;
        c->dark[0]=v; c->stamp[0]=(uint16_t)t;
        lit=delta>=probe_floor(c);
        if(c->prediction && delta>c->prediction+tolerance(c->prediction)) {
            udcal_abort(c,ULTRA_UCE_PREDICTION); return;
        }
        if (c->stage==COUNT_FIRST) {
            if (!lit) { udcal_abort(c,ULTRA_UCE_EMPTY); return; }
            c->upper=bound(c,delta); c->lo=1; c->hi=(uint16_t)(c->cap+1u);
            c->stage=COUNT_SEARCH;
        } else if (c->stage==COUNT_SEARCH) {
            if(lit) c->lo=c->pos; else c->hi=c->pos;
        } else if (c->stage==COUNT_LAST) {
            if(!lit) { udcal_abort(c,ULTRA_UCE_NOISE); return; }
            if(c->lo==c->cap) { count_done(c); return; }
            c->stage=COUNT_AFTER; pattern(c,UC_DUTY,1,1,c->lo+1u); return;
        } else {
            if(lit) { udcal_abort(c,ULTRA_UCE_NOISE); return; }
            count_done(c); return;
        }
        if((uint32_t)(c->hi-c->lo)<=1u) { c->stage=COUNT_LAST; pattern(c,UC_DUTY,1,1,c->lo); }
        else pattern(c,UC_DUTY,1,1,(c->hi+c->lo)/2u);
        (void)admit(c,1,1); return;
    }
    if(c->stage==BOUNDS) {
        if(delta<probe_floor(c)) { udcal_abort(c,ULTRA_UCE_WEAK); return; }
        c->upper=umax(c->upper,bound(c,delta));
        if(++c->channel<c->candidate.bpp) pattern(c,UC_DUTY,1u<<c->channel,1,1);
        else channels_begin(c);
        return;
    }
    if(c->stage==CHANNEL) {
        uint32_t i=c->index;
        c->stamp[i]=(uint16_t)(c->now_ms-c->start_ms);
        if(i%4u==0u) c->dark[i/4u]=v;
        else c->light[i<4u ? i-1u:i-2u]=v;
        if(i==4u || i==8u) fit(c);
        if(c->phase==ULTRA_UCS_RESTORE) return;
        if(++c->index==9u) {
            c->index=0;
            if(++c->channel==c->candidate.bpp) {
                c->phase=ULTRA_UCS_MIXED; c->stage=MIX_DARK;
            }
        }
        pattern(c,duties[c->index],1u<<c->channel,c->candidate.k,0);
        if(c->duty) (void)admit(c,c->k,1);
        return;
    }
    if(c->stage==MIX_DARK || c->stage==MIX_RGB) {
        uint32_t channels, k;
        if(c->stage==MIX_DARK) c->dark[0]=v;
        else if(diff(delta,c->prediction)>tolerance(c->prediction)) { udcal_abort(c,ULTRA_UCE_NONLINEAR); return; }
        channels=c->stage==MIX_DARK ? 3u:c->candidate.bpp;
        k=c->candidate.k/channels;
        if(!k) { udcal_abort(c,ULTRA_UCE_LIMIT); return; }
        c->stage++;
        pattern(c,128u,(1u<<channels)-1u,k,0);
        if(admit(c,k,channels)) {
            uint32_t total=0,j;
            for(j=0;j<channels;j++) total+=c->candidate.ch[j];
            c->prediction=m32_muldiv(total,k*32768u,255u*c->candidate.n);
        }
        return;
    }
    if(c->stage==MIX_ALL) {
        if(diff(delta,c->prediction)>tolerance(c->prediction)) { udcal_abort(c,ULTRA_UCE_NONLINEAR); return; }
        c->stage=FINAL_DARK; c->phase=ULTRA_UCS_VALIDATE;
        dark_pattern(c); return;
    }
    /* Final dark confirms that probe residue has been extinguished. */
    c->candidate.idle=(int16_t)((v+128u)/256u);
    c->phase=ULTRA_UCS_RESTORE; dark_pattern(c);
}
void udcal_step(udcal_t *c,uint32_t scale)
{
    uint32_t v, noise;
    uint32_t wait=counting(c) ? (c->sampling==1u?10u:6u) : (c->sampling==1u?32u:16u);
    if(!c->sampling || c->now_ms-c->block_ms<wait) return;
    if(!c->samples) { udcal_abort(c,ULTRA_UCE_ADC); return; }
    v=m32_divshr(c->sum,scale,c->samples,8);
    if(!c->duty && c->hi_adc>=c->lo_adc) {
        noise=m32_shr(c->hi_adc-c->lo_adc,scale*3u,24);
        c->candidate.noise=(uint16_t)umax(c->candidate.noise,noise);
    }
    if(c->sampling==1u) {
        c->first=v; c->sum=0; c->samples=0; c->sampling=2;
        c->block_ms=c->now_ms; return;
    }
    c->sampling=0;
    if(diff(v,c->first)>tolerance(v>c->dark[0]?v-c->dark[0]:0u)) { udcal_abort(c,ULTRA_UCE_NOISE); return; }
    c->value=(v+c->first+1u)/2u;
    evaluate(c);
}
uint8_t udcal_pixel(const udcal_t *c,uint32_t pixel)
{
    if(!c->k) return 0;
    if(c->pos) return pixel+1u==c->pos ? c->duty:0;
    /* ceil((p+1)*K/N) != ceil(p*K/N) selects floor(j*N/K), j=0..K-1. */
    if(pixel>=c->candidate.n) return 0;
    return ((pixel+1u)*c->k+c->candidate.n-1u)/c->candidate.n !=
           (pixel*c->k+c->candidate.n-1u)/c->candidate.n ? c->duty:0;
}

uint8_t udcal_byte(const udcal_t *c,uint32_t pixel,uint32_t channel)
{
    return (c->mask & (1u<<channel)) ? udcal_pixel(c,pixel) : 0;
}

int udcal_page(const udcal_t *c,uint32_t part,uint8_t *d)
{
    const udcal_model_t *m=(c->model.flags&ULTRA_UCF_VALID) ? &c->model:&c->candidate;
    uint32_t i,total=0,valid=(c->model.flags&(ULTRA_UCF_VALID|ULTRA_UCF_STALE))==ULTRA_UCF_VALID;
    memset(d,0,24);
    if(part==0u) {
        d[0]=(uint8_t)(valid?3u:0u);d[1]=(uint8_t)((valid?2u:0u)|(c->error<<4));
        ud_put16(d+2,c->model.count);
        ultra_put24(d+4,c->model_ms ? (c->now_ms-c->model_ms)/1000u:0xFFFFFFu);
        if(valid) {
            ud_put16(d+7,(uint16_t)(int16_t)(m->idle/2));
            for(i=0;i<5u;i++) {ud_put16(d+9u+2u*i,m->ch[i]);total+=m->ch[i];}
        }
        ud_put16(d+19,(uint16_t)(total>65535u?65535u:total));
        ud_put16(d+21,c->limit?c->limit:2250u);
    } else if(part==1u) {
        static const ud_f16_t t[9]={{4,offsetof(udcal_t,model.count)},{20,offsetof(udcal_t,model.epoch)},{22,offsetof(udcal_t,elapsed)},
            {6|UD_F16_B,offsetof(udcal_model_t,configured)},{8|UD_F16_B,offsetof(udcal_model_t,detected)},{10|UD_F16_B,offsetof(udcal_model_t,n)},
            {14|UD_F16_B,offsetof(udcal_model_t,order)},{16|UD_F16_B,offsetof(udcal_model_t,vbus)},{18|UD_F16_B,offsetof(udcal_model_t,vled)}};
        d[0]=ULTRA_UC_FORMAT; d[1]=(uint8_t)(m->flags | (c->model.flags&(ULTRA_UCF_PENDING|ULTRA_UCF_CLEAR_PENDING)));
        d[2]=c->phase;d[3]=c->error;
        d[12]=m->chip;d[13]=m->bpp;
        ud_put16s(d,c,m,t,9);
        ud_put16(d+16,m->vbus/10u);ud_put16(d+18,m->vled/10u);
    } else if(part==2u) {
        static const ud_f16_t t[6]={{2,offsetof(udcal_t,model.count)},{4,offsetof(udcal_t,model.epoch)},
            {6|UD_F16_B,offsetof(udcal_model_t,k)},{8|UD_F16_B,offsetof(udcal_model_t,noise)},
            {10|UD_F16_B,offsetof(udcal_model_t,residual)},{12|UD_F16_B,offsetof(udcal_model_t,bound)}};
        d[0]=ULTRA_UC_FORMAT;d[1]=1;
        ud_put16s(d,c,m,t,6);
    } else return -1;
    return 0;
}

static void putle(uint8_t *p,uint32_t v) {p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static uint16_t getle(const uint8_t *p) {return (uint16_t)(p[0]|(uint16_t)p[1]<<8);}
/* UDC1, algorithm 1, 64 LE bytes: magic 0, seq 4, the 39-byte model at 8,
 * byte 47 erased padding, elapsed 48, algorithm 50, reserved 51..59, salt 60,
 * CRC-16 62 programmed last. Model u16 offsets are fixed by the assertion below;
 * both supported targets (CH32 and Windows host) are little endian. */
typedef char model_layout[(sizeof(udcal_model_t)==40u &&
    offsetof(udcal_model_t,configured)==0u && offsetof(udcal_model_t,flags)==38u) ? 1:-1];
void udcal_pack(const udcal_t *c,uint32_t seq,uint8_t r[UC_REC_BYTES])
{
    uint32_t crc,salt=0;
    memset(r,255,UC_REC_BYTES);
    putle(r,UC_REC_MAGIC);putle(r+2,UC_REC_MAGIC>>16);
    putle(r+4,seq);putle(r+6,seq>>16);
    memcpy(r+8,&c->model,39);
    r[46]&=ULTRA_UCF_VALID|ULTRA_UCF_AGGREGATE|ULTRA_UCF_SHORT|ULTRA_UCF_AT_LEAST;
    putle(r+48,c->elapsed);r[50]=3; r[51]=BOARD_OUTPUTS;
    do {putle(r+60,salt++);crc=ud_crc16(r,62);} while(ud_crc_erased(crc));
    putle(r+62,crc);
}
int udcal_valid(const uint8_t *r)
{
    uint32_t bpp=r[45],cap=bpp==3u?ULTRA_PX_MAX_RGB:bpp==4u?ULTRA_PX_MAX_RGBW:ULTRA_PX_MAX_RGBCCT,i,seen=0,order=getle(r+14),flags=r[46];
    if(memcmp(r,"UDC1",4) /* UC_REC_MAGIC */ || r[50]!=3u || r[51]!=BOARD_OUTPUTS || r[47]!=255u ||
       ud_crc_erased(getle(r+62)) || ud_crc16(r,62)!=getle(r+62) ||
       (flags&~(ULTRA_UCF_VALID|ULTRA_UCF_AGGREGATE|ULTRA_UCF_SHORT|ULTRA_UCF_AT_LEAST))) return 0;
    for(i=52;i<60;i++) if(r[i]!=255u) return 0;
    if(!(flags&ULTRA_UCF_VALID)) return 1;
    /* (x-1u >= n: x is 0 or above n; x-3000u > 62000u: x outside 3,000-65,000) */
    if(bpp-3u>2u || r[44]>=ULTRA_PXC_COUNT || getle(r+8)-1u>=cap ||
       getle(r+10)-1u>=cap || getle(r+12)-1u>=getle(r+8) || getle(r+12)>getle(r+10) ||
       getle(r+38)-1u>=32u || getle(r+38)>getle(r+12) ||
       getle(r+16)-3000u>62000u || getle(r+18)-3000u>62000u ||
       !(flags&ULTRA_UCF_AGGREGATE)) return 0;
    for(i=0;i<bpp;i++) {
        uint32_t ch=(order>>(3u*i))&7u;
        if(ch>=bpp || (seen&(1u<<ch)) || !getle(r+20u+2u*i)) return 0;
        seen|=1u<<ch;
    }
    for(;i<5u;i++) if(getle(r+20u+2u*i)) return 0;
    return 1;
}
void udcal_load(udcal_t *c,const uint8_t *r)
{
    memcpy(&c->model,r+8,39);
    c->elapsed=getle(r+48);
}
