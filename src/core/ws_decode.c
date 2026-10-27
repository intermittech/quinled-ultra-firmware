/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "ws_decode.h"
#include "lto.h"
#if defined(__GNUC__)
#define ACQUISITION __attribute__((optimize("Os")))
#else
#define ACQUISITION
#endif

#if defined(BOARD) && BOARD == 3
/* Exact nibble lookup; digital builds trade a few decoder instructions for flash.
 * Other boards retain their original byte table and generated code. */
static const uint8_t clz4[16] = {4,3,2,2,1,1,1,1,0,0,0,0,0,0,0,0};
static ALWAYS_INLINE uint32_t ws_clz8(uint32_t x)
{
    return (x & 0xF0u) ? clz4[x>>4] : 4u+clz4[x];
}
#define CLZ8(x) ws_clz8(x)
#else
/* leading zeros of a byte (8 for 0) */
static const uint8_t clz8[256] = {
    8, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

#define CLZ8(x) clz8[x]
#endif

/* Walks the level transitions word by word: t has a bit set wherever a sample differs from the previous
 * one, so each word costs a few instructions plus ~10 per edge (two edges per WS bit). */
MEASURED int ws_decode_frame(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, const ws_timing_t *t, uint8_t *out)
{
    uint32_t nwords = nsamples >> 4, k, b = 0, prev = 0, rise = 0, fall = 0, seen_rise = 0;
    const uint32_t thr = t->thr_one, hmax = t->high_max, gmin = t->gap_min;

    if (nbits == 0u) {
        return 0;
    }
    for (k = 0; k < (uint32_t)(nbits + 7u) / 8u; k++) {
        out[k] = 0;
    }
    if (nwords == 0u || (buf[0] & 0x8000u) != 0u) {
        return WS_ERR_NO_GAP;                         /* must start inside the reset gap */
    }
    for (k = 0; k < nwords; k++) {
        uint32_t w = buf[k];
        uint32_t tr = (w ^ ((w >> 1) | (prev << 15))) & 0xFFFFu;
        prev = w & 1u;
        while (tr != 0u) {
            uint32_t j = (tr & 0xFF00u) ? CLZ8(tr >> 8) : 8u + CLZ8(tr & 0xFFu);
            uint32_t pos = (k << 4) + j;
            tr ^= 0x8000u >> j;                  /* j names a set bit: toggle it off */
            if ((w << j) & 0x8000u) {                 /* rising edge */
                if (seen_rise && pos - fall >= gmin) {
                    return WS_ERR_SHORT;
                }
                rise = pos;
                seen_rise = 1;
            } else if (seen_rise) {                   /* falling edge: one bit done */
                uint32_t high = pos - rise;
                if (high > hmax) {
                    return WS_ERR_PULSE;
                }
                if (high >= thr) {
                    out[b >> 3] |= (uint8_t)(0x80u >> (b & 7u));
                }
                if (++b == nbits) {
                    return (int)nbits;
                }
                fall = pos;
            }
        }
    }
    return seen_rise ? WS_ERR_TRUNCATED : WS_ERR_NO_EDGE;
}

/* ws_decode_frame's walk to the end of the buffer: the first nbits into out, then every further bit only counted */
MEASURED int ws_decode_count(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, const ws_timing_t *t, uint8_t *out)
{
    uint32_t nwords = nsamples >> 4, k, b = 0, prev = 0, rise = 0, fall = 0, seen_rise = 0;
    const uint32_t thr = t->thr_one, hmax = t->high_max, gmin = t->gap_min;

    for (k = 0; k < (uint32_t)(nbits + 7u) / 8u; k++) {
        out[k] = 0;
    }
    if (nwords == 0u || (buf[0] & 0x8000u) != 0u) {
        return WS_ERR_NO_GAP;                         /* must start inside the reset gap */
    }
    for (k = 0; k < nwords; k++) {
        uint32_t w = buf[k];
        uint32_t tr = (w ^ ((w >> 1) | (prev << 15))) & 0xFFFFu;
        prev = w & 1u;
        while (tr != 0u) {
            uint32_t j = (tr & 0xFF00u) ? CLZ8(tr >> 8) : 8u + CLZ8(tr & 0xFFu);
            uint32_t pos = (k << 4) + j;
            tr ^= 0x8000u >> j;
            if ((w << j) & 0x8000u) {                 /* rising edge */
                if (seen_rise && pos - fall >= gmin) {
                    return WS_ERR_SHORT;               /* a reset gap inside: two frames */
                }
                rise = pos;
                seen_rise = 1;
            } else if (seen_rise) {                   /* falling edge: one bit done */
                uint32_t high = pos - rise;
                if (high > hmax) {
                    return WS_ERR_PULSE;
                }
                if (b < nbits && high >= thr) {
                    out[b >> 3] |= (uint8_t)(0x80u >> (b & 7u));
                }
                b++;
                fall = pos;
            }
        }
    }
    return seen_rise ? (int)b : WS_ERR_NO_EDGE;
}

/* the rising edges of a word (the samples that are HIGH and differ from the one before): the word, the one before it's last sample */
#define WS_RISES(w, prev) (((w) ^ (((w) >> 1) | ((prev) << 15))) & (w) & 0xFFFFu)

static NOINLINE int ws_rise_walk(ws_rise_t *st, const uint16_t *buf, uint32_t mask, uint32_t start,
                                uint32_t nwords, uint16_t nbits, uint32_t threshold, uint8_t *out)
{
    uint32_t c = st->k, b = st->b, prev = st->prev, w;
    uint32_t ahead = (threshold + 14u) >> 4;
    uint32_t hi=0, lo=0, k;

    if (nbits && c == 0u && nwords != 0u && (buf[start & mask] & 0x8000u) != 0u) {
        return WS_ERR_NO_GAP;                         /* must start inside the reset gap */
    }
    if (c + ahead >= nwords) {
        return 0;
    }
    do {
        w = buf[(start + c) & mask];
        uint32_t r = WS_RISES(w, prev);
        if(!nbits && !c) r&=0x7fffu;
        prev = w & 1u;
        while (r != 0u) {
            uint32_t j = (r & 0xFF00u) ? CLZ8(r >> 8) : 8u + CLZ8(r & 0xFFu);
            r ^= 0x8000u >> j;
            uint32_t sample = j + threshold - 1u;
            uint32_t bit=(buf[(start + c + (sample >> 4)) & mask] >> (15u-(sample&15u)))&1u;
            if(!nbits) {
                hi=(hi<<1)|(lo>>31); lo=(lo<<1)|bit;
            } else if(bit) {
                out[b >> 3] |= (uint8_t)(0x80u >> (b & 7u));
            }
            if (++b == nbits) {
                st->b = (uint16_t)b;
                return (int)nbits;
            }
        }
        c++;
    } while (c + ahead < nwords);
    st->k = (uint16_t)c;
    st->b = (uint16_t)b;
    st->prev = (uint8_t)prev;
    if(!nbits) {
        for(k=0;k<4u;k++) { out[k]=(uint8_t)(hi>>(24u-8u*k)); out[k+4u]=(uint8_t)(lo>>(24u-8u*k)); }
        return (int)b;
    }
    return 0;
}

MEASURED int ws_rise_ring(ws_rise_t *st, const uint16_t *buf, uint32_t mask, uint32_t start,
                         uint32_t nwords, uint16_t nbits, uint32_t threshold, uint8_t *out)
{
    return ws_rise_walk(st,buf,mask,start,nwords,nbits,threshold,out);
}

MEASURED int ws_rise_step(ws_rise_t *st, const uint16_t *buf, uint32_t nwords, uint16_t nbits, uint8_t *out)
{
    return ws_rise_ring(st,buf,0xffffffffu,0,nwords,nbits,MCU_WS_ZERO_THRESHOLD,out);
}

MEASURED int ws_rise_frame(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, uint8_t *out)
{
    ws_rise_t st = { 0, 0, 0 };
    uint32_t k;
    int n;

    for (k = 0; k < (uint32_t)(nbits + 7u) / 8u; k++) {
        out[k] = 0;
    }
    n = ws_rise_step(&st, buf, nsamples >> 4, nbits, out);
    return n != 0 ? n : st.b != 0u ? WS_ERR_TRUNCATED : WS_ERR_NO_EDGE;
}

MEASURED int ws_rise_tail(const uint16_t *ring, uint32_t mask, uint32_t start, uint32_t nwords,
                         uint32_t threshold, uint8_t *out)
{
    ws_rise_t st={0,0,0};
    return ws_rise_walk(&st,ring,mask,start,nwords,0,threshold,out);
}

MEASURED int ws_rise_last(const uint16_t *ring, uint32_t mask, uint32_t start, uint32_t nwords, uint8_t *out)
{
    return ws_rise_tail(ring,mask,start,nwords,MCU_WS_ZERO_THRESHOLD,out);
}

MEASURED int ws_decode_last(const uint16_t *buf, uint32_t nsamples, const ws_timing_t *t, uint8_t *out)
{
    uint32_t nwords = nsamples >> 4, k, n = 0, prev, rise = 0, fall = 0, seen_rise = 0, hi = 0, lo = 0;
    const uint32_t thr = t->thr_one, hmax = t->high_max, gmin = t->gap_min;

    if (nwords == 0u) {
        return WS_ERR_NO_EDGE;
    }
    prev = (buf[0] & 0x8000u) ? 1u : 0u;              /* started inside a high pulse: its fall is not a bit */
    for (k = 0; k < nwords; k++) {
        uint32_t w = buf[k];
        uint32_t tr = (w ^ ((w >> 1) | (prev << 15))) & 0xFFFFu;
        if (k == 0u) {
            tr &= 0x7FFFu;                               /* no transition before the first sample */
        }
        prev = w & 1u;
        while (tr != 0u) {
            uint32_t j = (tr & 0xFF00u) ? CLZ8(tr >> 8) : 8u + CLZ8(tr & 0xFFu);
            uint32_t pos = (k << 4) + j;
            tr ^= 0x8000u >> j;                  /* j names a set bit: toggle it off */
            if ((w << j) & 0x8000u) {                 /* rising edge */
                if (seen_rise && pos - fall >= gmin) {
                    return WS_ERR_SHORT;               /* a reset gap inside the window */
                }
                rise = pos;
                seen_rise = 1;
            } else if (seen_rise) {                   /* falling edge: one bit done */
                uint32_t high = pos - rise;
                if (high > hmax) {
                    return WS_ERR_PULSE;
                }
                hi = (hi << 1) | (lo >> 31);
                lo = (lo << 1) | (high >= thr ? 1u : 0u);
                n++;
                fall = pos;
            }
        }
    }
    if (n == 0u) {
        return WS_ERR_NO_EDGE;
    }
    for (k = 0; k < 4u; k++) {
        out[k] = (uint8_t)(hi >> (24u - 8u * k));
        out[4u + k] = (uint8_t)(lo >> (24u - 8u * k));
    }
    return (int)n;
}

/* The profile is discovered from 256 rising intervals before any HIGH is
 * interpreted. Capturing runs instead of raw samples fits 200 kHz without
 * enlarging either board's sample mailbox. A partial frame never qualifies. */
static const uint16_t line_period[4] = MCU_LINE_PERIODS;
static const uint8_t line_threshold[4] = MCU_LINE_THRESHOLDS;
static const uint16_t line_limits[4][4] = MCU_LINE_LIMITS;
uint32_t ws_line_threshold(uint32_t profile) { return line_threshold[profile & 3u]; }
ACQUISITION void ws_line_init(ws_line_t *s)
{
    s->pos=s->rise=s->fall=s->first=s->bits=0;
    s->intervals=s->period_max=s->low_max=0;
    s->period_min=s->low_min=s->high_min=65535;
    s->high_max=0;
    s->prev=s->have=s->bad=s->ended=0;
    s->profile=255;
}
ACQUISITION void ws_line_feed(ws_line_t *s, uint16_t *runs, const uint16_t *samples, uint32_t words)
{
    uint32_t k, prev=s->prev, base=s->pos;
    uint32_t rise=s->rise, fall=s->fall, bits=s->bits;
#if defined(BOARD) && BOARD == 3
    uint32_t idle=(0u-prev)>>16;                  /* the word of no edge: every sample = prev */
#endif
    if(s->ended) return;
    for(k=0;k<words;k++) {
        uint32_t w=samples[k], tr;
#if defined(BOARD) && BOARD == 3
        /* main feeds a frame as it arrives: skip the words without an edge (most of them on a slow line) */
        if(w==idle) continue;
#endif
        tr=(w^((w>>1)|(prev<<15)))&0xFFFFu;
        prev=w&1u;
#if defined(BOARD) && BOARD == 3
        idle=(0u-prev)>>16;
#endif
        while(tr) {
            uint32_t j=(tr&0xFF00u)?CLZ8(tr>>8):8u+CLZ8(tr&255u);
            uint32_t p=base+(k<<4)+j;
            tr^=0x8000u>>j;
            if((w<<j)&0x8000u) {
                if(s->have) {
                    uint32_t low=p-fall;
                    if(low>=MCU_US2S(10u)) { s->ended=1; goto done; }
                    {
                        uint32_t period=p-rise;
                        if(period<s->period_min) s->period_min=(uint16_t)period;
                        if(period>s->period_max) s->period_max=(uint16_t)period;
                        if(low<s->low_min) s->low_min=(uint16_t)low;
                        if(low>s->low_max) s->low_max=(uint16_t)low;
                    }
                    if(s->intervals<256u && ++s->intervals==256u) {
                        uint32_t span=p-s->first, q;
                        for(q=0;q<4u;q++) {
                            uint32_t nominal=(uint32_t)line_period[q]*256u;
                            if(span*100u>=nominal*92u && span*100u<=nominal*108u) s->profile=(uint8_t)q;
                        }
                        if(s->profile==255) s->bad=1;
                    }
                } else { s->first=p; s->have=1; }
                rise=p;
            } else if(s->have) {
                uint32_t high=p-rise;
                if(high<s->high_min) s->high_min=(uint16_t)high;
                if(high>s->high_max) s->high_max=(uint16_t)high;
                if(!high || high>=MCU_US2S(10u)) s->bad=1;
                if(bits<WS_LINE_HEAD) runs[bits]=(uint16_t)high;
                runs[WS_LINE_HEAD+(bits&63u)]=(uint16_t)high;
                bits++;
                fall=p;
            }
        }
    }
done:
    s->rise=rise; s->fall=fall; s->bits=bits;
    s->prev=(uint8_t)prev;
    s->pos=base+(words<<4);
}
ACQUISITION int ws_line_decode(ws_line_t *s, const uint16_t *runs, uint32_t bits, uint8_t *out, int tail)
{
    uint32_t i, profile=s->profile;
    /* A receiver beyond the last slot gets only the header. Its 63 periods
     * identify a tentative profile; callers keep it separate from training. */
    if(profile>=4u && s->bits==64u && s->intervals==63u) {
        uint32_t span=s->rise-s->first;
        for(i=0;i<4u;i++) {
            uint32_t nominal=(uint32_t)line_period[i]*63u;
            if(span*100u>=nominal*92u && span*100u<=nominal*108u) s->profile=(uint8_t)i;
        }
        profile=s->profile;
    }
    if(s->bad || profile>=4u || bits>s->bits || (!tail && bits>WS_LINE_HEAD)) return WS_ERR_PULSE;
    if(s->high_min<line_limits[profile][0] || s->high_max>line_limits[profile][1] ||
       s->low_min<line_limits[profile][2] || s->low_max>line_limits[profile][3]) { s->bad=1; return WS_ERR_PULSE; }
    for(i=0;i<(bits+7u)/8u;i++) out[i]=0;
    for(i=0;i<bits;i++) {
        uint32_t h=runs[tail ? WS_LINE_HEAD+((s->bits-bits+i)&63u) : i];
        if(!h || h>line_limits[profile][1]) return WS_ERR_PULSE;
        if(h>=line_threshold[profile]) out[i>>3]|=(uint8_t)(0x80u>>(i&7u));
    }
    return (int)bits;
}

uint32_t ws_line_lock(ws_lock_t *s, uint32_t profile, int crc_ok, uint32_t now_ms)
{
    uint32_t reset=profile!=s->profile || now_ms-s->last_ms>=2000u;
    if(reset || (!crc_ok && s->good<3u) || profile>=8u) s->good=0;
    s->profile=(uint8_t)profile;
    if(crc_ok && profile<8u) {
        if(!s->good) s->start_ms=now_ms;
        s->last_ms=now_ms;
        if(s->good<3u) s->good++;
    }
    return (reset?2u:0u) | (s->good>=3u ? (profile<4u?1u:4u) : 0u);
}
