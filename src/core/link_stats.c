/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "link_stats.h"
#include "ultra_protocol.h"
#include "math32.h"
#include <string.h>
#if defined(__GNUC__)
#define LS_COLD
#else
#define LS_COLD
#endif
link_stats_t line_stats;
typedef char ls_state_budget[sizeof(link_stats_t)<=136?1:-1];
static void clear(void) { memset(&line_stats.live,0,sizeof line_stats.live); }
static void inc(uint16_t *v) { if(*v!=65535u) ++*v; else line_stats.flags|=4u; }
static void bounds(uint16_t *lo,uint16_t *hi,uint32_t v)
{ if(!*lo||v<*lo) *lo=(uint16_t)v; if(v>*hi) *hi=(uint16_t)v; }
static void finish_window(uint32_t ms)
{
    link_stats_t *s=&line_stats;
    uint32_t bit;
    if(s->trained || !s->patterns || ms-s->train_ms<1000u) return;
    s->trained=s->patterns==15u;
    s->train_end_ms=s->train_ms+1000u;
    for(bit=1u;bit<16u;bit<<=1u) if(!(s->patterns&bit)) {
        inc(&s->live.missing);
        s->flags|=8u;
    }
    s->patterns=0;
}
void ls_frame(const ws_line_t *c,const uint16_t *runs,const uint8_t *slot,int crc_ok,uint32_t ms,uint32_t drops)
{
    link_stats_t *s=&line_stats;
    uint32_t i,pattern=0;
    finish_window(ms);
    if(c && (c->profile!=s->profile || ms-s->last_ms>=2000u)) {
        clear(); s->epoch++; s->profile=c->profile;
        s->snap_ms=ms-2000u; memset(s->frozen,0,sizeof s->frozen);
        s->patterns=s->trained=s->flags=0;
        s->error_ms=ms; s->error_count=s->bad_windows=0;
    }
    if(ms-s->error_ms>=1000u) {
        uint32_t elapsed=ms-s->error_ms;
        /* An unobserved whole second breaks a consecutive-window streak.
         * Keep the window origin fixed despite delayed calls and timer wrap. */
        s->bad_windows=elapsed<2000u && s->error_count>=4u?(uint8_t)(s->bad_windows+1u):0u;
        if(s->bad_windows>=3u) { s->flags|=ULTRA_LS_RATE_REQUEST; s->bad_windows=3u; }
        s->error_count=0; s->error_ms=ms-elapsed%1000u;
    }
    if(drops!=s->drops) {
        uint32_t n=(uint32_t)s->live.invalid+drops-s->drops;
        s->live.invalid=(uint16_t)(n>65535u?65535u:n);
        s->drops=drops; s->flags|=(uint8_t)(8u|(n>65535u?4u:0u));
        if(!s->trained) s->patterns=0;
    }
    /* A DARK preamble has an all-zero unsealed slot. Other bad slot CRCs
     * are errors, even when their trailer still qualifies rate acquisition. */
    if(slot && !(crc_ok&2u)) {
        uint32_t raw=0;
        for(i=0;i<32u;i++) raw|=slot[i];
        if(raw) crc_ok=0;
        slot=0;
    }
    if(!(crc_ok&1u) || (c && (c->profile>=4u || c->bad))) {
        inc(&s->live.bad); if(c && c->bad) inc(&s->live.invalid);
        if(s->error_count<255u) s->error_count++;
        s->flags|=8u; if(!s->trained) s->patterns=0;
        return;
    }
    s->last_ms=ms; inc(&s->live.frames);
    if(!slot || !c || s->trained) return; /* widths are an acquisition snapshot */
    /* Accumulate each symbol class locally. Publishing once per class keeps
     * acquisition from holding the frame mailbox through the next reset. */
    for(uint32_t b=0;b<2u;b++) {
        uint32_t n=s->live.count[b], sum=s->live.sum[b];
        uint32_t lo=s->live.min[b], hi=s->live.max[b];
        if(!lo) lo=65535u;
        for(i=0;i<32u;i++) {
            uint32_t v=b ? slot[i] : (uint8_t)~slot[i];
            const uint16_t *r=runs+(i<<3);
            while(v) {
                if(v&128u) {
                    uint32_t h=*r;
                    if(n==65535u) { s->flags|=4u; break; }
                    n++; sum+=h;
                    if(h<lo) lo=h;
                    if(h>hi) hi=h;
                }
                r++; v=(v<<1)&255u;
            }
        }
        s->live.count[b]=(uint16_t)n; s->live.sum[b]=sum;
        if(n) { s->live.min[b]=(uint16_t)lo; s->live.max[b]=(uint16_t)hi; }
    }
    bounds(&s->live.period_min,&s->live.period_max,c->period_min);
    bounds(&s->live.period_min,&s->live.period_max,c->period_max);
    bounds(&s->live.low_min,&s->live.low_max,c->low_min);
    bounds(&s->live.low_min,&s->live.low_max,c->low_max);
    if(!slot[0]&&!slot[1]&&!slot[2]) {
        static const uint8_t patterns[4]={0x55,0xaa,0,255};
        for(i=0;i<4;i++) if(slot[3]==patterns[i]) pattern=1u<<i;
        for(i=4;i<30;i++) if(slot[i]!=slot[3]) pattern=0;
    }
    if(pattern) {
        if(!s->patterns) s->train_ms=ms;
        s->patterns|=(uint8_t)pattern;
    }
}
static uint32_t ns(uint32_t samples,uint32_t q,int ceil)
{ return m32_addshr(samples,q,ceil?65535u:0u,16); }
LS_COLD uint8_t ls_page(const uint8_t *a,uint8_t *d,uint32_t ms,uint32_t q)
{
    link_stats_t *s=&line_stats;ls_counts_t *c=&s->live;
    uint32_t i,f;
    /* Only ingress 0 is sampled; all remaining arguments are reserved zero. */
    if(a[0] || a[1]>1u || a[2]>1u || (a[2]&&a[1])) return ULTRA_PAGE_UNSUPPORTED;
    for(i=3;i<22u;i++) if(a[i]) return ULTRA_PAGE_UNSUPPORTED;
    finish_window(ms);
    f=s->flags;
    if(!a[2]) {
        uint8_t *p=s->frozen,*x=p+24;
        memset(p,0,48);s->snap_ms=ms;
        if(ms-s->last_ms>=ULTRA_RATE_LOCK_LOSS_MS) q=0;
        if(q) f|=2u;
        if(q && s->trained && c->count[0]>=256u && c->count[1]>=256u && s->profile<4u && ms-s->last_ms<ULTRA_RATE_LOCK_LOSS_MS) f|=1u;
        p[1]=1; p[2]=x[2]=(uint8_t)f; p[3]=(uint8_t)ns(1,q,1);
        x[1]=s->profile<4u?s->profile:255; x[3]=15;
        for(i=0;i<2u;i++) {
            uint32_t off=4u+6u*i, n=c->count[i];
            ultra_put16(p+off,(uint16_t)ns(c->min[i],q,0));
            ultra_put16(p+off+2,(uint16_t)ns(c->max[i],q,1));
            ultra_put16(p+off+4,(uint16_t)(n?(m32_muldiv(c->sum[i],q,n)+32768u)>>16:0u));
            ultra_put16(p+16+2*i,(uint16_t)n);
        }
        ultra_put16(p+20,c->invalid);ultra_put16(p+22,s->epoch);ultra_put16(x+4,s->epoch);
        ultra_put16(x+6,(uint16_t)ns(c->period_min,q,0));ultra_put16(x+8,(uint16_t)ns(c->period_max,q,1));
        ultra_put16(x+10,(uint16_t)ns(c->low_min,q,0));ultra_put16(x+12,(uint16_t)ns(c->low_max,q,1));
        ultra_put16(x+14,c->frames);ultra_put16(x+16,c->bad);ultra_put16(x+18,c->missing);
        ultra_put16(x+20,q?200u:65535u);
        if(a[1]) { c->frames=c->bad=c->invalid=c->missing=0; }
    }
    memcpy(d,s->frozen+24u*a[2],24);
    if(ms-s->snap_ms>=2000u || s->frozen[27]!=15u) { d[2]=0; if(a[2]) d[1]=255; }
    return ULTRA_PAGE_LINK_STATS;
}
