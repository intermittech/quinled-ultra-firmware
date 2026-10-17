/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "dline.h"
#include "lto.h"

/* the PASS scan reads the ring two words at a time (the ring is 4-byte aligned: hw_dline.c / the test); may_alias: the
 * same memory is read and zeroed as 16-bit words */
#if defined(__GNUC__)
typedef uint32_t __attribute__((__may_alias__)) dl_pair_t;
#define DL_BARRIER() __asm volatile("" ::: "memory")
#else
typedef uint32_t dl_pair_t;
#define DL_BARRIER() ((void)0)
#endif

#define DL_M          DLINE_RING_MASK
/* a 4-word block may start at word j. DLINE_WORDS_ONLY (the host test's reference build): never, every word by the
 * single-word rules, which the block paths must match exactly (test/build_dline.cmd compares the two) */
#ifdef DLINE_WORDS_ONLY
#define DL_BLOCK(j)   0
#else
#define DL_BLOCK(j)   (((j) & 3u) == 0u)
#endif
#define DL_ONES       0xFFFFu
#define DL_SYNC_BASE  1u                 /* a window's end: re-base TIM4's count */
#define DL_SYNC_FRAME 2u                 /* a latch after activity: a frame's bits, then re-base */
#define DL_ARM_YES    1u
#define DL_ARM_NO     2u

/* RX's free-running word count from its 9-bit position (a tick at least every 227 us) */
static ALWAYS_INLINE uint32_t dl_sync(dline_t *d, uint32_t rx_pos)
{
    d->wpos += (rx_pos - d->wpos) & DL_M;
    return d->wpos;
}

/* a word with its leading HIGH run cleared: the samples up to the first 1 -> 0 transition (a run that started in the
 * words before); what follows the transition stays */
static ALWAYS_INLINE uint32_t dl_clr_lead(uint32_t v)
{
    uint32_t m = 0x8000u;
    while (v & m) {
        v &= ~m;
        m >>= 1;
    }
    return v;
}

static ALWAYS_INLINE void dl_sat8(uint8_t *c)
{
    if (*c != 0xFFu) {
        (*c)++;
    }
}

static ALWAYS_INLINE void dl_sat16(uint16_t *c)
{
    if (*c != 0xFFFFu) {
        (*c)++;
    }
}

/* back to PASS at word j (the scan goes on there); zs = where the LOW run in progress started (j: none). That LOW run
 * counts no frame at its latch (lzs), and TIM4 re-bases at this tick's end if it is still LOW then. The window's
 * BLANK_N goes with it (dl_blank, at the tick's end) */
DLINE_RAMCODE static uint32_t dl_pass(dline_t *d, uint32_t j, uint32_t zs)
{
    d->state = DLINE_PASS;
    d->zs = zs;
    d->lzs = zs;
    d->os = j;
    d->hi = 0;
    d->blank = 0;
    if (d->sync != 0u) {
        d->base_ok = 0;                  /* (an earlier sync in this tick: not exact) */
    }
    d->sync = DL_SYNC_BASE;
    d->sync_zs = zs;
    return DLINE_A_WINDOW_END;
}

/* BLANK_N = the window's or the recovery's: the action when it changes */
static ALWAYS_INLINE uint32_t dl_blank(dline_t *d)
{
    uint8_t want = (uint8_t)((d->blank & (uint8_t)!d->local) | d->rec);   /* (local: the window never cuts the effect) */
    if (want == d->bl) {
        return 0;
    }
    d->bl = want;
    return want ? DLINE_A_BLANK_ON : DLINE_A_BLANK_OFF;
}

/* a fault: the window dropped, BLANK_N LOW, the ring restarted by the glue (the mux stays: delayed replays the zeroed
 * ring), then BOOT's recovery. From the SRAM tick (ram): no RESTART now (it needs the flash): the next 20 kHz tick asks */
DLINE_RAMCODE static uint32_t dl_fault(dline_t *d, uint32_t ram)
{
    uint32_t a = ram ? 0u : DLINE_A_RESTART;
    if (d->state != DLINE_PASS) {
        a |= DLINE_A_WINDOW_END;         /* (a capture in progress is dropped: no FRAME_READY) */
    }
    d->state = DLINE_PASS;
    d->blank = 0;
    d->hi = 0;
    d->rec = 1;
    d->restart = (uint8_t)(ram ? 1u : 2u);
    d->sync = 0;
    d->pend_k = 0;
    d->base_ok = 0;
    d->ch_st = DLINE_CH_IDLE;            /* (CHAIN: the frame in progress is lost, an armed answer is dropped) */
    d->ch_lz = 0;
    d->ch_arm = 0;
    dl_sat8(&d->st.faults);
    return a | dl_blank(d);
}

/* PASS: a long HIGH, r = its rising-edge word (the word before its first all-HIGH word), j = its newest all-HIGH word:
 * zero it from the edge (the rising-edge word only from the edge on: what came before it is the line's, e.g. a pixel
 * frame's last bit when a released line follows it at once) */
DLINE_RAMCODE static void dl_mark(dline_t *d, uint16_t *ring, uint32_t r, uint32_t j)
{
    uint32_t v = ring[r & DL_M], k;
    if (!d->local) {                     /* (local pixel effects: the words behind the scan, up to 9 of them, are the encoder's
                                            output already; it replaces every word it writes, so nothing is zeroed here) */
        ring[r & DL_M] = (uint16_t)(v & (v + 1u));     /* (its trailing HIGH run cleared) */
        for (k = r + 1u; k != j + 1u; k++) {
            ring[k & DL_M] = 0;
        }
    }
    d->state = DLINE_MARK;
    d->mark_w = r;
    d->run = j - r;
}

/* PASS: a latch (the LOW run from zs is DLINE_W_LATCH words long). A frame if the line was active since the last one */
DLINE_RAMCODE static void dl_latch(dline_t *d, uint32_t zs)
{
    uint8_t k = DL_SYNC_BASE;
    if (zs != d->lzs) {
        d->st.frames++;
        k = DL_SYNC_FRAME;
    }
    d->lzs = zs;
    if (d->sync != 0u) {
        d->base_ok = 0;                  /* an earlier latch in this tick, the line active after it: not exact */
    }
    d->sync = k;
    d->sync_zs = zs;
}

/* the marker frame ended (END or REPLY): hand the capture to the main loop if it is whole */
DLINE_RAMCODE static uint32_t dl_frame_end(dline_t *d)
{
    if (d->cap_ok) {
        if (++d->seq == 0u) {
            d->seq = 1;
        }
        d->cur_seq = d->seq;
        d->cap_ready = 1;
        return DLINE_A_FRAME_READY;
    }
    d->cur_seq = 0;
    dl_sat8(&d->st.cap_drops);
    return 0;
}

/* the chain scanner's FRAME path takes two words at a time (j even; DLINE_WORDS_ONLY: never, the word path is the reference) */
#ifdef DLINE_WORDS_ONLY
#define DL_PAIR(j)    0
#else
#define DL_PAIR(j)    (((j) & 1u) == 0u)
#endif

/* the ring's 4-word block at word j (a multiple of 4) */
#define DL_Q(j)       (pr + (((j) & DL_M) >> 1))

/* aligned 4-word blocks from j (a multiple of 4) before w or the ring's end */
static ALWAYS_INLINE uint32_t dl_blocks(uint32_t j, uint32_t w)
{
    uint32_t n = w - j, r = DLINE_RING_WORDS - (j & DL_M);
    return (n < r ? n : r) >> 2;
}

/* blocks all HIGH from q (before qe) zeroed: their words */
static ALWAYS_INLINE uint32_t dl_zero_high(dl_pair_t *q, const dl_pair_t *qe)
{
    dl_pair_t *q0 = q;
    while (q != qe && (q[0] & q[1]) == 0xFFFFFFFFu) {
        q[0] = 0;
        q[1] = 0;
        q += 2;
    }
    return (uint32_t)(q - q0) * 2u;
}

/* blocks all LOW from q (before qe): their words */
static ALWAYS_INLINE uint32_t dl_skip_low(const dl_pair_t *q, const dl_pair_t *qe)
{
    const dl_pair_t *q0 = q;
    while (q != qe && (q[0] | q[1]) == 0u) {
        q += 2;
    }
    return (uint32_t)(q - q0) * 2u;
}

/* trailing all-LOW (lo) / all-HIGH words of a block with edges, x = words 0-1, y = words 2-3 (0..3) */
static ALWAYS_INLINE uint32_t dl_trail(uint32_t x, uint32_t y, uint32_t u)
{
    return (y & 0xFFFFu) != u ? 1u : (x >> 16) != u ? 2u : 3u;
}

/* FRAME after the block path: the last two words with an edge (m1, m0) and m1's value from the capture (cap[i] is word
 * cap_w0 + i; none: the marker's falling-edge word, as the word path starts) */
DLINE_RAMCODE static void dl_mres(dline_t *d, uint32_t n)
{
    uint32_t i = n, f = 0;
    d->m0 = d->m1 = d->fall_w;
    d->v1 = 0;
    while (i-- != 0u) {
        uint32_t v = d->cap[i];
        if (v != 0u && v != DL_ONES) {
            if (f) {
                d->m0 = d->cap_w0 + i;
                return;
            }
            f = 1;
            d->m1 = d->cap_w0 + i;
            d->v1 = (uint16_t)v;
        }
    }
}

/* The scan: the words [zpos, w), each once, by state (the header). Each state first takes aligned 4-word blocks that
 * can't hold one of its events (two 32-bit reads: all LOW, all HIGH, or edges), then single words: the exact rules,
 * which also run every block that might hold an event (a run reaching its limit, a long HIGH, a falling edge that ends
 * a zeroing). cap_on: the capture goes to the main loop (the 20 kHz tick; the SRAM tick copies too, for t_end, but
 * can't hand it over). edges: TIM4, for this tick's latch / window end. */
DLINE_RAMCODE static uint32_t dl_scan(dline_t *d, uint16_t *ring, uint32_t w, uint32_t cap_on, uint32_t edges)
{
    dl_pair_t *const pr = (dl_pair_t *)(void *)ring;
    uint32_t j = d->zpos, a = 0, v, stop = 0;

    if (d->pend_k != 0u && w != d->pend_w) { /* the last latch / window end: the word RX was writing then all LOW? */
        if (ring[d->pend_w & DL_M] == 0u) {
            if (d->pend_k == DL_SYNC_FRAME && d->base_ok) {
                uint32_t b = (uint32_t)(d->pend_e - d->base) & DLINE_EDGE_MASK;
                d->st.bits_last = b;
                d->st.bits_n++;
                if (b < d->st.bits_min) {
                    d->st.bits_min = b;
                }
                if (b > d->st.bits_max) {
                    d->st.bits_max = b;
                }
            }
            d->base = d->pend_e;
            d->base_ok = 1;
        } else {
            d->base_ok = 0;
        }
        d->pend_k = 0;
    }

    if (w - j > DLINE_W_LATE) {          /* the ticks fell behind: those words are out (or going out) already */
        dl_sat8(&d->st.late);
        if (d->state != DLINE_PASS) {
            a |= dl_pass(d, w, w);
        }
        j = w;
        d->zs = d->os = d->lzs = w;
        d->base_ok = 0;
        d->sync = 0;
        d->pend_k = 0;
    }
    while (j != w && !stop) {
        uint32_t st = d->state;          /* (no switch: its jump table would be read from the flash: SRAM tick) */
        if (st == DLINE_PASS) {
            /* blocks: edges (the runs restart at the block's end), LOW (the run grows, not into the 8 words before its
               latch); an all-HIGH block, a run near an event: words */
            uint32_t zs = d->zs, os = d->os;
            for (;;) {
                if (DL_BLOCK(j) && j - os < 5u && j - zs - (DLINE_W_LATCH - 8u) >= 8u) {
                    dl_pair_t *q = DL_Q(j), *qe = q + 2u * dl_blocks(j, w);
                    for (;;) {
                        dl_pair_t *q0 = q;
                        uint32_t x, y;
                        while (q != qe && (q[1] >> 16) - 1u < 0xFFFEu) {
                            q += 2;              /* pixel data: the block's last word has an edge */
                        }
                        if (q != q0) {
                            j += (uint32_t)(q - q0) * 2u;
                            zs = os = j;
                        }
                        if (q == qe) {
                            break;
                        }
                        x = q[0];
                        y = q[1];
                        if ((x | y) == 0u) {     /* LOW: up to 8 words before the latch (then words) */
                            const dl_pair_t *ql = qe;
                            uint32_t zl = j - zs, k;
                            if (zl < DLINE_W_LATCH) {
                                k = (DLINE_W_LATCH - 9u - zl) >> 2;
                                if (k < (uint32_t)(qe - q) >> 1) {
                                    ql = q + 2u * k;
                                }
                            }
                            k = dl_skip_low(q, ql);
                            if (k == 0u) {
                                break;
                            }
                            q += k >> 1;
                            j += k;
                            os = j;
                            continue;
                        }
                        if ((x & y) == 0xFFFFFFFFu) {
                            break;               /* a long HIGH may start here */
                        }
                        q += 2;                  /* edges; its last word all LOW / HIGH: that run goes on */
                        j += 4u;
                        zs = os = j;
                        if ((y >> 16) == 0u) {
                            zs = j - dl_trail(x, y, 0u);
                        } else {
                            os = j - dl_trail(x, y, DL_ONES);
                        }
                    }
                }
                if (j == w) {
                    break;
                }
                v = ring[j & DL_M];              /* one word */
                j++;
                if (v == 0u) {
                    os = j;
                    if (j - zs == DLINE_W_LATCH) {
                        dl_latch(d, zs);
                    }
                } else if (v == DL_ONES) {
                    zs = j;
                    if (j - os >= DLINE_W_LONG) {
                        dl_mark(d, ring, os - 1u, j - 1u);
                        break;
                    }
                } else {
                    zs = os = j;
                }
            }
            d->zs = zs;
            d->os = os;
        } else if (st == DLINE_MARK) {
            uint32_t run = d->run;
            for (;;) {
                if (DL_BLOCK(j) && run + 4u <= DLINE_W_MARK_MAX) {   /* blocks: the marker's HIGH, < 60 us */
                    dl_pair_t *q = DL_Q(j);
                    uint32_t nb = dl_blocks(j, w), lim = (DLINE_W_MARK_MAX - run) >> 2, k;
                    k = dl_zero_high(q, q + 2u * (nb < lim ? nb : lim));
                    j += k;
                    run += k;
                }
                if (j == w) {
                    break;
                }
                v = ring[j & DL_M];
                ring[j & DL_M] = 0;              /* the marker's HIGH; its falling-edge word whole (the gap after it
                                                    is >= 5 us LOW) */
                j++;
                if (v != DL_ONES) {
                    /* the frame: captured from the word after this one (ws_decode needs a LOW start), the capture
                       starting at an even word for 32-bit copies (then this word, zeroed, comes first) */
                    d->state = DLINE_FRAME;
                    d->fall_w = j - 1u;
                    d->cap_own = (uint8_t)(!d->cap_ready && !d->ch_ready && d->ch_st != DLINE_CH_FRAME);
                    d->cap_ok = d->cap_own;
                    if (d->cap_own) {
                        d->cap_w0 = j & ~1u;
                        d->cap_n = 0;
                        if (j & 1u) {
                            d->cap[0] = 0;
                            d->cap_n = 1;
                        }
                    }
                    run = 0;
                    d->orun = 0;
                    d->m0 = d->m1 = j - 1u;
                    d->v1 = 0;
                    dl_sat16(&d->st.marks);
                    break;
                }
                if (++run > DLINE_W_MARK_MAX) {
                    d->state = DLINE_HOLD;       /* not a marker: the line released / undriven */
                    dl_sat16(&d->st.long_high);
                    break;
                }
            }
            d->run = run;
        } else if (st == DLINE_FRAME) {
            uint32_t z = d->run, o = d->orun, n = d->cap_n;
            uint16_t *cap = d->cap;
            if (!cap_on) {
                d->cap_ok = 0;
            }
            for (;;) {
                if (DL_BLOCK(j) && d->cap_own && o < 5u && z < DLINE_W_LATCH - 4u) {
                    /* blocks, copied (32-bit: the capture's word n is ring word j, both even) and zeroed: the frame's
                       bits (its two halves differ: not all LOW / HIGH; a run can't reach an event inside), LOW (not
                       reaching the frame's end); all HIGH (the release): words. The runs at the end of the last block
                       of bits from the capture, t_end's words (m0 / m1) afterwards */
                    dl_pair_t *q = DL_Q(j), *q0 = q;
                    dl_pair_t *c = (dl_pair_t *)(void *)(cap + n);
                    uint32_t nb = dl_blocks(j, w), room = (DLINE_CAP_WORDS - n) >> 2, lazy = 0;
                    dl_pair_t *qe = q + 2u * (nb < room ? nb : room);
                    for (;;) {
                        dl_pair_t *qm = q;
                        uint32_t x, y;
                        while (q != qe) {
                            x = q[0];
                            y = q[1];
                            if (x == y) {
                                break;
                            }
                            q[0] = 0;
                            q[1] = 0;
                            c[0] = x;
                            c[1] = y;
                            q += 2;
                            c += 2;
                        }
                        if (q != qm) {
                            x = c[-2];
                            y = c[-1];
                            lazy = 1;
                            z = o = 0;
                            if ((y >> 16) == 0u) {
                                z = dl_trail(x, y, 0u);
                            } else if ((y >> 16) == DL_ONES) {
                                o = dl_trail(x, y, DL_ONES);
                            }
                        }
                        if (q == qe) {
                            break;
                        }
                        x = q[0];
                        y = q[1];
                        if (x == 0u) {           /* LOW */
                            if (z >= DLINE_W_LATCH - 8u) {
                                break;
                            }
                            c[0] = 0;
                            c[1] = 0;
                            z += 4u;
                            o = 0;
                        } else if (x == 0xFFFFFFFFu) {
                            break;
                        } else {                 /* bits, both halves alike */
                            q[0] = 0;
                            q[1] = 0;
                            c[0] = x;
                            c[1] = y;
                            lazy = 1;
                            z = o = 0;
                            if ((y >> 16) == 0u) {
                                z = dl_trail(x, y, 0u);
                            } else if ((y >> 16) == DL_ONES) {
                                o = dl_trail(x, y, DL_ONES);
                            }
                        }
                        q += 2;
                        c += 2;
                    }
                    if (q != q0) {
                        uint32_t k = (uint32_t)(q - q0) * 2u;
                        j += k;
                        n += k;
                        if (lazy) {
                            dl_mres(d, n);
                        }
                    }
                }
                if (j == w) {
                    break;
                }
                v = ring[j & DL_M];
                ring[j & DL_M] = 0;
                if (d->cap_own && n < DLINE_CAP_WORDS) {
                    cap[n++] = (uint16_t)v;
                }
                j++;
                if (v == 0u) {
                    o = 0;
                    if (++z >= DLINE_W_LATCH) {  /* LOW 46 us: the frame's end */
                        d->state = DLINE_END;
                        d->end_w = d->m1;
                        d->zs = j - z;
                        break;
                    }
                } else if (v == DL_ONES) {
                    z = 0;
                    if (++o >= DLINE_W_LONG) {   /* a long HIGH: the hub released the line already */
                        uint32_t r = j - 1u - o;
                        d->state = DLINE_REPLY;
                        d->end_w = (d->m1 == r && (d->v1 & (d->v1 + 1u)) == 0u) ? d->m0 : d->m1;
                        d->rel_w = r;
                        d->run = o;
                        break;
                    }
                } else {
                    z = o = 0;
                    d->m0 = d->m1;
                    d->m1 = j - 1u;
                    d->v1 = (uint16_t)v;
                }
            }
            if (d->cap_own) {
                d->cap_n = (uint16_t)n;
            }
            if (d->state == DLINE_FRAME) {
                d->run = z;
                d->orun = o;
            } else {
                a |= dl_frame_end(d);
            }
        } else if (st == DLINE_END) {
            /* LOW until t_end + 250 us (then PASS, whatever comes), unless the hub releases the line: a run of
               DLINE_W_LONG all-HIGH words from a rising edge (zeroed -> REPLY). A shorter HIGH is pixels (the hub broke
               its >= 300 us): PASS at its word, not zeroed */
            for (;;) {
                if (DL_BLOCK(j)) {
                    dl_pair_t *q = DL_Q(j);
                    uint32_t nb = dl_blocks(j, w), lim = (DLINE_W_END - 3u - (j - d->end_w)) >> 2;
                    if (j - d->end_w < DLINE_W_END - 3u) {
                        j += dl_skip_low(q, q + 2u * (nb < lim ? nb : lim));
                    }
                }
                if (j == w) {
                    break;
                }
                if (j - d->end_w >= DLINE_W_END) {
                    a |= dl_pass(d, j, d->zs);   /* not released by t_end + 250 us */
                    break;
                }
                v = ring[j & DL_M];
                if (v == 0u) {
                    j++;
                    continue;
                }
                {
                    uint32_t s0 = v == DL_ONES ? j : j + 1u, k;   /* the HIGH run's first all-HIGH word */
                    if ((v & 1u) == 0u) {
                        a |= dl_pass(d, j, d->zs);   /* a pulse inside this word: pixels */
                        break;
                    }
                    if (w - s0 < DLINE_W_LONG) {
                        stop = 1;                /* (not known yet: the next tick) */
                        break;
                    }
                    for (k = s0; k != s0 + DLINE_W_LONG && ring[k & DL_M] == DL_ONES; k++) {
                    }
                    if (k != s0 + DLINE_W_LONG) {
                        a |= dl_pass(d, j, d->zs);   /* short: pixels */
                        break;
                    }
                    d->rel_w = s0 - 1u;          /* the release: zeroed from its rising-edge word */
                    for (k = j; k != s0 + DLINE_W_LONG; k++) {
                        ring[k & DL_M] = 0;
                    }
                    j = s0 + DLINE_W_LONG;
                    d->state = DLINE_REPLY;
                    d->run = DLINE_W_LONG;
                    break;
                }
            }
        } else if (st == DLINE_REPLY || st == DLINE_HOLD || ((st == DLINE_BURST || st == DLINE_RETAKE) && d->hi)) {
            /* a HIGH being zeroed: the release (REPLY: counted; BURST: until our DE rise), the fail-safe HIGH after DE
               low (RETAKE), a line released / undriven (HOLD); its first 1 -> 0 transition ends it */
            uint32_t run = d->run;
            for (;;) {
                if (DL_BLOCK(j)) {
                    dl_pair_t *q = DL_Q(j);
                    uint32_t k = dl_zero_high(q, q + 2u * dl_blocks(j, w));
                    j += k;
                    run += k;
                }
                if (j == w) {
                    break;
                }
                v = ring[j & DL_M];
                if (v == DL_ONES) {
                    ring[j & DL_M] = 0;
                    j++;
                    run++;
                    continue;
                }
                ring[j & DL_M] = (uint16_t)dl_clr_lead(v);
                j++;
                if (st == DLINE_BURST) {
                    d->hi = 0;                   /* our DE rise: the burst's LOW follows */
                } else {
                    a |= dl_pass(d, j, j);       /* the hub drives again (REPLY: no burst of ours) / its retake */
                }
                break;
            }
            if (st == DLINE_REPLY) {
                d->run = run;
            }
            if ((st == DLINE_REPLY || st == DLINE_RETAKE) && d->state == st && w - d->end_w > DLINE_W_HOLD) {
                d->state = DLINE_HOLD;
                dl_sat16(&d->st.long_high);
            }
        } else if (st == DLINE_BURST) {
            for (;;) {
                if (DL_BLOCK(j)) {            /* blocks: LOW (DE high), not reaching DE low's word */
                    dl_pair_t *q = DL_Q(j);
                    uint32_t nb = dl_blocks(j, w);
                    if (d->txdone) {
                        int32_t l = (int32_t)(d->txend_w - j) >> 2;
                        nb = l <= 0 ? 0u : (uint32_t)l < nb ? (uint32_t)l : nb;
                    }
                    j += dl_skip_low(q, q + 2u * nb);
                }
                if (j == w) {
                    break;
                }
                v = ring[j & DL_M];
                if (d->txdone && (int32_t)(j - d->txend_w) >= 0) {
                    d->state = DLINE_RETAKE;
                    break;
                }
                if (v != 0u) {                   /* HIGH: DE is low already (the burst-done not in yet) */
                    if (!d->txdone) {
                        d->txdone = 1;
                        d->txend_w = j;
                    }
                    d->state = DLINE_RETAKE;
                    break;
                }
                j++;
            }
        } else {                                 /* DLINE_RETAKE, before a HIGH after DE low */
            v = ring[j & DL_M];
            if (v == 0u) {
                j++;
                if ((int32_t)(j - d->txend_w) >= (int32_t)DLINE_W_RO_EN) {
                    a |= dl_pass(d, j, j);       /* LOW at DE low: the hub had the line already */
                }
            } else {
                ring[j & DL_M] = 0;              /* the fail-safe HIGH after DE low */
                j++;
                if ((v & 1u) == 0u) {
                    a |= dl_pass(d, j, j);       /* ... and the hub's retake in the same word */
                } else {
                    d->hi = 1;
                }
            }
        }
    }
    d->zpos = j;

    /* this tick's latch / window end: TIM4 is exact if the line stayed LOW from it to the newest word, and through the
       word RX was writing (TIM4 was read before the RX position: an edge there may be counted): kept until that word
       is in (the next tick) */
    if (d->sync != 0u) {
        if (d->state == DLINE_PASS && d->zs == d->sync_zs && d->zs != w) {
            d->pend_k = d->sync;
            d->pend_e = (uint16_t)edges;
            d->pend_w = w;
        } else {
            d->base_ok = 0;
        }
        d->sync = 0;
    }
    if (d->state == DLINE_PASS) {
        if (d->zs != w) {                        /* LOW at the newest word */
            uint32_t run;
            if (w - d->zs > 0x40000000u) {       /* (a LOW line for 8 minutes: keep the run long, not wrapping) */
                if (d->lzs == d->zs) {
                    d->lzs = w - 0x40000000u;
                }
                d->zs = w - 0x40000000u;
            }
            d->edge_w = d->zs - 1u;
            run = w - d->zs;
            if (run >= DLINE_W_QUIET) {
                if (d->rec) {                    /* BOOT / after a fault: delayed and BLANK_N off together (the
                                                    ring's last 512 words and the line are LOW); AUTO still deciding
                                                    (ch_hold): neither */
                    if (!d->ch_hold) {
                        if (d->direct) {
                            a |= DLINE_A_MUX_DELAYED;
                        }
                        d->direct = 0;
                        d->rec = 0;
                    }
                } else if (d->direct) {          /* DIRECT-PASS: the direct path is visible: the live check */
                    d->undo = (uint8_t)(1u | d->bl << 2);
                    d->direct = 0;
                    a |= DLINE_A_MUX_DELAYED | DLINE_A_LIVE_CHECK;
                }
            } else if (d->rec && !d->ch_hold && d->direct && run >= DLINE_W_GAP && w - d->rec_w >= DLINE_W_FALLBACK) {
                d->undo = (uint8_t)(3u | d->bl << 2);   /* no quiet run for 50 ms while direct (BOOT; never after
                                                           delayed): BLANK_N off in this latch gap */
                d->rec = 0;
                a |= DLINE_A_LIVE_CHECK;
            }
        } else {
            d->edge_w = d->os != w ? d->os - 1u : w - 1u;
        }
    } else if (d->state == DLINE_HOLD || d->state == DLINE_MARK) {
        d->edge_w = d->mark_w;                   /* (a released line: since its rising edge) */
        if (w - d->edge_w > 0x40000000u) {
            d->edge_w = d->mark_w = w - 0x40000000u;
        }
    } else {
        d->edge_w = w - 1u;
    }
    return a;
}

/* both ticks: the ring's health, the scan, BLANK_N */
DLINE_RAMCODE static uint32_t dl_tick(dline_t *d, uint16_t *ring, uint32_t rx_pos, uint32_t tx_pos, uint32_t edges,
                                      uint32_t cap_on)
{
    uint32_t a;
    if (d->restart != 0u) {                      /* a fault: no scan until dline_restart */
        if (d->restart == 1u && cap_on) {
            d->restart = 2;                      /* (found by the SRAM tick: the restart now) */
            return DLINE_A_RESTART;
        }
        return 0;
    }
    d->stall = (uint8_t)(rx_pos == d->last_rx ? d->stall + 1u : 0u);
    d->last_rx = rx_pos;
    if (!dline_dist_ok(rx_pos, tx_pos) || d->stall >= 2u) {
        return dl_fault(d, !cap_on);             /* RX stalled (OVR) or both stopped: the ring is stale */
    }
    a = dl_scan(d, ring, dl_sync(d, rx_pos), cap_on, edges);
    if (d->local && d->rec && !d->direct) {
        d->rec = 0;                              /* a fault's restart in local mode: the ring is zeroed and the encoder
                                                    starts from LOW, nothing to wait for */
    }
    return a | dl_blank(d);
}

void dline_init(dline_t *d, uint32_t rx_pos)
{
    uint8_t *p = (uint8_t *)d;
    uint32_t i;
    for (i = 0; i < (uint32_t)sizeof *d; i++) {
        p[i] = 0;
    }
    d->wpos = d->zpos = d->zs = d->os = d->lzs = d->edge_w = d->rec_w = rx_pos & DL_M;
    d->last_rx = 0xFFFFFFFFu;
    d->direct = 1;
    d->rec = 1;
    d->bl = 1;
    d->st.bits_min = 0xFFFFFFFFu;
}

/* A long HIGH inside a captured frame: the host's release after an RW frame (see DLINE_CH_REL_GAP). Looks back in the snapshot from the HIGH run (o all-ones words,
 * the last at j - 1): an optional partial word (the rise), the LOW words, the last pulse's word m1b. If it is a frame end the frame is handed over like a LOW run's:
 * t0 = m1b, the window of the header ends 2 LOW words after it (the release's rise must not be in it). Returns DLINE_A_CH_READY or 0 */
static uint32_t dl_ch_rel_end(dline_t *d, const uint16_t *snap, uint32_t hn, uint32_t j, uint32_t o, uint32_t edges)
{
    uint32_t p = j - o - 1u, z = 0, m1b, we;
    if (snap[p & DL_M] != 0u) {
        p--;                                     /* (the word of the rise) */
    }
    while (z < DLINE_W_LATCH && snap[(p - z) & DL_M] == 0u) {
        z++;
    }
    m1b = p - z;
    if (z < DLINE_CH_REL_GAP || z >= DLINE_W_LATCH || (int32_t)(m1b - d->ch_js) < (int32_t)DLINE_CH_REL_MIN) {
        return 0;                                /* (the release HIGH itself starting a "frame": the LOW before it is the last frame's) */
    }
    we = m1b + 3u;
    d->ch_end = m1b;
    d->ch_bits = (uint16_t)edges;
    d->ch_hn = (uint16_t)hn;
    d->ch_tn = (uint16_t)(we - d->ch_js > 0xFFFFu ? 0xFFFFu : we - d->ch_js);
    d->ch_tw = we;
    d->ch_ready = 1;
    return DLINE_A_CH_READY;
}

/* CHAIN: the frame scanner (header of dline.h). The words [cpos, w) once, raw: before the encoder replaces them and before the
 * stream scanner reads them. IDLE: LOW words are counted (a latch is DLINE_W_LATCH of them: the gap before a frame); the first HIGH
 * sample after a latch starts a frame, a HIGH sample before one is garbage. FRAME: every word is copied to the head (the first
 * DLINE_CH_HEAD) and to the snapshot (its own ring position), two at a time where it can (the exact rules of the word path, which
 * is the reference: DLINE_WORDS_ONLY); a LOW run of DLINE_W_LATCH ends it (the frame bits = TIM4, zeroed in the gap before; t0 =
 * the last word with a HIGH sample; the snapshot ends with the word that ended it). DLINE_W_LONG all-HIGH words in a row are no
 * Ultra frame (a marker HIGH, a released line): HIGH waits for the end of the run; a run that ends within DLINE_W_MARK_MAX words
 * after the window is a marker. DLINE_A_EDGES_ZERO each time the gap becomes a latch. */
static uint32_t dl_ch_scan(dline_t *d, uint16_t *ring, uint32_t w, uint32_t edges)
{
    dl_pair_t *const pr = (dl_pair_t *)(void *)ring;
    uint16_t *const head = d->cap, *const snap = d->cap + DLINE_CH_HEAD;
    uint32_t j = d->cpos, a = 0, v;
    if (d->ch_skip || w - j > DLINE_W_LATE) {   /* ticks fell behind (an interrupt storm) or the SRAM tick ran (a flash operation, however
                                                   short: it has no chain scan, the encoder went on replacing the words): they are the encoder's */
        d->ch_skip = 0;
        d->ch_st = DLINE_CH_IDLE;
        d->ch_lz = 0;
        j = w;
    }
    while (j != w) {
        uint32_t lz = d->ch_lz;
        if (d->ch_st == DLINE_CH_IDLE) {
            const uint32_t lz0 = lz;
            for (;;) {
                if (DL_BLOCK(j)) {
                    dl_pair_t *q = DL_Q(j);
                    const uint32_t k = dl_skip_low(q, q + 2u * dl_blocks(j, w));
                    j += k;
                    lz += k;
                }
                if (j == w || ring[j & DL_M] != 0u) {
                    break;
                }
                j++;
                lz++;
            }
            if (lz > 0xFFFFu) {
                lz = 0xFFFFu;
            }
            if (lz0 < DLINE_W_LATCH && lz >= DLINE_W_LATCH) {
                a |= DLINE_A_EDGES_ZERO;     /* the gap is a latch: TIM4 starts the next frame at 0 */
            }
            d->ch_lz = (uint16_t)lz;
            if (j == w) {
                break;
            }
            if (lz < DLINE_W_LATCH) {        /* a HIGH sample before a latch: the middle of something else */
                d->ch_lz = 0;
                j++;
                continue;
            }
            d->ch_st = DLINE_CH_FRAME;       /* a frame starts at word j */
            d->ch_cap = (uint8_t)(!d->ch_ready && !d->cap_ready && (d->state == DLINE_PASS || d->state == DLINE_HOLD));
            if (d->ch_cap) {                 /* (no capture: the last frame is still being decoded, its head and snapshot stay) */
                d->ch_hn = 1;
                head[0] = 0;                 /* (ws_decode needs a LOW start) */
                d->ch_cjs = j;
            }
            d->ch_js = j;
            d->ch_m1 = j;
            d->ch_o = 0;
            d->ch_lz = 0;                    /* (an armed answer stays: the release HIGH after an RW frame starts a "frame" too; it is judged by its word) */
            lz = 0;
        }
        if (d->ch_st == DLINE_CH_FRAME) {
            uint32_t hn = d->ch_hn, m1 = d->ch_m1, o = d->ch_o;
            const uint32_t cap = d->ch_cap;     /* (hn is written back only with it) */
            while (j != w) {
                if (DL_PAIR(j) && w - j >= 2u && o <= 5u && lz + 2u < DLINE_W_LATCH) {
                    /* two words at a time: a pair that can neither end the frame nor start a long HIGH is copied whole and classified as its
                       two words would be one after the other */
                    const uint32_t x = pr[(j & DL_M) >> 1];
                    if (cap) {
                        ((dl_pair_t *)(void *)snap)[(j & DL_M) >> 1] = x;
                        if (hn + 2u <= DLINE_CH_HEAD) {
                            head[hn] = (uint16_t)x;
                            head[hn + 1u] = (uint16_t)(x >> 16);
                            hn += 2u;
                        } else if (hn < DLINE_CH_HEAD) {
                            head[hn++] = (uint16_t)x;
                        }
                    }
                    if (x == 0u) {
                        lz += 2u;
                        o = 0;
                    } else if (x == 0xFFFFFFFFu) {
                        lz = 0;
                        o += 2u;
                        m1 = j + 1u;
                    } else {
                        const uint32_t hi = x >> 16;
                        lz = hi == 0u ? 1u : 0u;
                        o = hi == DL_ONES ? 1u : 0u;
                        m1 = hi != 0u ? j + 1u : j;
                    }
                    j += 2u;
                    continue;
                }
                v = ring[j & DL_M];
                if (cap) {
                    snap[j & DL_M] = (uint16_t)v;
                    if (hn < DLINE_CH_HEAD) {
                        head[hn++] = (uint16_t)v;
                    }
                }
                j++;
                if (v == 0u) {
                    o = 0;
                    if (++lz >= DLINE_W_LATCH) {         /* LOW 46 us: the frame end */
                        d->ch_st = DLINE_CH_IDLE;
                        a |= DLINE_A_EDGES_ZERO;
                        if (cap) {
                            const uint32_t n = j - DLINE_CH_TAIL_CUT - d->ch_js;   /* (the header's window ends 8 LOW words after the last pulse) */
                            d->ch_end = m1;
                            d->ch_bits = (uint16_t)edges;
                            d->ch_hn = (uint16_t)hn;
                            d->ch_tn = (uint16_t)(n > 0xFFFFu ? 0xFFFFu : n);
                            d->ch_tw = j - DLINE_CH_TAIL_CUT;
                            d->ch_ready = 1;
                            a |= DLINE_A_CH_READY;
                        } else {
                            dl_sat8(&d->st.cap_drops);
                        }
                        break;
                    }
                } else {
                    lz = 0;
                    m1 = j - 1u;
                    if (v == DL_ONES) {
                        if (++o >= DLINE_W_LONG) {       /* a long HIGH: no Ultra frame, or the release after an RW frame whose last pulse it follows */
                            d->ch_st = DLINE_CH_HIGH;
                            lz = o;
                            if (cap) {
                                a |= dl_ch_rel_end(d, snap, hn, j, o, edges);
                            }
                            break;
                        }
                    } else {
                        o = 0;
                    }
                }
            }
            if (d->ch_st == DLINE_CH_FRAME) {
                if (cap) {
                    d->ch_hn = (uint16_t)hn;
                }
                d->ch_m1 = m1;
                d->ch_o = (uint8_t)o;
            }
            d->ch_lz = (uint16_t)lz;
        } else if (d->ch_st == DLINE_CH_HIGH) {
            while (j != w) {
                v = ring[j & DL_M];
                j++;
                if (v == DL_ONES) {
                    if (lz < 0xFFFFu) {
                        lz++;
                    }
                    continue;
                }
                if (lz <= DLINE_W_MARK_MAX && (d->ch_win == 0u || (int32_t)(j - d->ch_win) >= 0)) {
                    a |= DLINE_A_CH_MARKER;      /* HIGH 3.6 .. 60 us that ends: a marker (a pixel-line hub) */
                    d->ch_win = 0;
                }
                d->ch_st = DLINE_CH_IDLE;
                lz = v == 0u ? 1u : 0u;
                break;
            }
            d->ch_lz = (uint16_t)lz;
        }
    }
    d->cpos = j;
    return a;
}

uint32_t dline_tick(dline_t *d, uint16_t *ring, uint32_t rx_pos, uint32_t tx_pos, uint32_t edges)
{
    uint32_t w = d->wpos + ((rx_pos - d->wpos) & DL_M), a = 0;

    if (d->ch_req != 0u) {                       /* the main loop CHAIN request */
        const uint32_t r = d->ch_req;
        d->ch_req = 0;
        if (r & 2u) {
            d->ch_on = 0;
            d->ch_st = DLINE_CH_IDLE;
            d->ch_arm = 0;
            d->ch_enter = 0;
        } else {
            if (!d->ch_on) {
                d->ch_on = 1;
                d->cpos = w;
                d->ch_st = DLINE_CH_IDLE;
                d->ch_lz = 0;
                d->ch_arm = 0;
            }
            if ((r & 4u) && d->rec) {
                d->ch_enter = 1;
            }
        }
    }
    if (d->ch_on && d->restart == 0u) {
        a = dl_ch_scan(d, ring, w, edges);       /* (before the stream scanner: it zeroes in some states) */
    }

    if (d->tx_flag) {                            /* DE low: its word (a burst-done after this tick's RX read waits) */
        uint32_t back = (w - d->tx_raw) & DL_M;
        if (back < DLINE_RING_WORDS / 2u) {
            d->tx_flag = 0;
            if (d->state == DLINE_BURST && !d->txdone) {
                d->txdone = 1;
                d->txend_w = w - back;
            }
        }
    }
    if (d->state == DLINE_BURST && !d->txdone && w - d->send_w > DLINE_W_TX_MAX) {
        d->txdone = 1;                           /* (no burst-done: DE is low by now) */
        d->txend_w = w;
    }
    a |= dl_tick(d, ring, rx_pos, tx_pos, edges, 1u);
    /* CHAIN: the ring zeroed and the mux switched while BLANK_N is held (BOOT, or AUTO decided): udfx_tick takes the ring */
    if (d->ch_enter == 1u && d->restart == 0u) {
        uint32_t i;
        for (i = 0; i < DLINE_RING_WORDS; i++) {
            ring[i] = 0;
        }
        if (d->direct) {
            a |= DLINE_A_MUX_DELAYED;
            d->direct = 0;
        }
        d->ch_hold = 0;
        d->ch_enter = 2;
    }
    /* CHAIN: this board answer starts at its word (t0 + the slot time), not at a release: the first tick at or after it, no
     * later than DLINE_W_CH_LATE words; the stream scanner takes the burst from there as in a window (BURST, RETAKE) */
    if (d->ch_on && d->ch_arm && (int32_t)(w - d->ch_tx) >= 0) {
        d->ch_arm = 0;
        if (w - d->ch_tx <= DLINE_W_CH_LATE && d->restart == 0u && (d->state == DLINE_PASS || d->state == DLINE_HOLD)) {
            d->state = DLINE_BURST;
            d->hi = 1;
            d->txdone = 0;
            d->send_w = w;
            d->blank = 1;
            d->st.answered++;
            a |= DLINE_A_SEND | dl_blank(d);
        } else {
            a |= DLINE_A_CH_MISSED;
        }
    }
    /* the answer: armed for this window, the line released >= 20 us (and still: the scan ends in the release's HIGH),
     * t_end + 900 us not passed, the mux delayed (direct, BLANK_N alone can't hide the fail-safe HIGH after it) */
    if (d->state == DLINE_REPLY && d->cur_seq != 0u && !d->direct && d->run >= DLINE_W_RELEASED &&
        w - d->end_w <= DLINE_W_REPLY_MAX && d->arm == (((uint32_t)d->cur_seq << 2) | DL_ARM_YES)) {
        d->state = DLINE_BURST;
        d->hi = 1;
        d->txdone = 0;
        d->send_w = w;
        d->blank = 1;
        d->st.answered++;
        a |= DLINE_A_SEND | dl_blank(d);
    }
    return a;
}

DLINE_RAMCODE uint32_t dline_ram_tick(dline_t *d, uint16_t *ring, uint32_t rx_pos, uint32_t tx_pos, uint32_t edges)
{
    d->ch_skip = d->ch_on;                       /* (the chain scanner does not run here: it starts again at the newest word) */
    return dl_tick(d, ring, rx_pos, tx_pos, edges, 0u);
}

void dline_arm(dline_t *d, int ok)
{
    d->arm = ((uint32_t)d->seq << 2) | (ok ? DL_ARM_YES : DL_ARM_NO);
    DL_BARRIER();                                /* the capture's reads done before it is released */
    d->cap_ready = 0;
}

void dline_burst_done(dline_t *d, uint32_t rx_pos)
{
    d->tx_raw = (uint16_t)(rx_pos & DL_M);
    DL_BARRIER();
    d->tx_flag = 1;
}

uint32_t dline_fault(dline_t *d)
{
    if (d->restart != 0u) {                      /* (already: a RESTART asked for, or now when the SRAM tick found it) */
        if (d->restart == 1u) {
            d->restart = 2;
            return DLINE_A_RESTART;
        }
        return 0;
    }
    return dl_fault(d, 0u);
}

void dline_restart(dline_t *d, uint32_t rx_pos)
{
    uint32_t w = dl_sync(d, rx_pos);             /* (the restart moved RX: positions from here on) */
    d->zpos = d->zs = d->os = d->lzs = d->edge_w = d->rec_w = w;
    d->cpos = w;
    d->ch_st = DLINE_CH_IDLE;
    d->ch_lz = 0;
    d->last_rx = 0xFFFFFFFFu;
    d->stall = 0;
    d->tx_flag = 0;
    d->restart = 0;
}

void dline_minmax_restart(dline_t *d)
{
    d->st.bits_min = 0xFFFFFFFFu;
    d->st.bits_max = 0;
}

void dline_ch_start(dline_t *d, int enter)
{
    d->ch_req = (uint8_t)(enter ? 5u : 1u);
}

void dline_ch_stop(dline_t *d)
{
    d->ch_req = 2;
}

void dline_ch_done(dline_t *d, int answer, uint32_t tx_w, uint32_t win_w)
{
    d->ch_win = win_w;
    d->ch_tx = tx_w;
    DL_BARRIER();
    d->ch_arm = (uint8_t)(answer != 0);
}

void dline_ch_free(dline_t *d)
{
    DL_BARRIER();                                /* the capture's reads done before it is released */
    d->ch_ready = 0;
}
