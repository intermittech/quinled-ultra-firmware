/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The QuinLED Diff-Power Ultra-Digital receiver's delay line + poll window (core/dline.c, BOARD 3) on a sample-exact
 * model of its hardware (test/build_dline.cmd: also against dline.c's word-by-word reference, DLINE_WORDS_ONLY, whose
 * output must be the same to the digest). `dline_tests.exe <scenario> [<trace scenario> [<seed>]]`: the scenarios whose
 * name starts so (`*`: all), a trace of the glue's actions, another random seed.
 *
 * The model, at the receiver's 36 MHz sample clock (the hub's clock 0-1 % off):
 *  - a hub drives the pair: pixel frames of six chip types with their sources' timings, polls behind markers (design
 *    note section 4: >= 350 us LOW, marker, gap, an Ultra frame N = 1, the release, the retake keyed to the receiver's
 *    burst or the hub's window W_D = 900 + the slot pitch, pixels >= 20 us later), IDENTIFY / UPD frames, undriven HIGHs;
 *  - RO -> PA6 (LOW while the receiver's DE is high: RO high-Z on the 22k, and 0.3 us after it: RO's enable);
 *  - SPI1: RX writes word k into ring[k % 512] at the word's end, TX reads slot (k % 512) for output word k + 512 two
 *    words before it goes out; a fault freezes RX; the glue's restart zeroes the ring and starts both again;
 *  - TIM4 counts PA6's falling edges, 3 samples late (the input filter);
 *  - the mux, BLANK_N and the gate: out = (mux ? TX : PA6) AND NOT BLANK;
 *  - the CPU: the 20 kHz tick (lateness 0-3 us, or 0-50 us), the glue applying the actions 0-2 us after the tick's
 *    reads (the live check then), the burst's TC, the main loop decoding 20-300 us after FRAME_READY, the SRAM tick
 *    alone during a 16 ms flash operation.
 * Checks, every sample: the output is the input delayed by D with every non-pixel HIGH (marker, frame, release,
 * fail-safe, undriven) removed, AND NOT BLANK; no non-pixel HIGH on the output in any mode; the window's BLANK_N never
 * cuts a pixel. At each mux switch / BOOT or fault release: the line LOW >= D before it. The strip (a WS decoder on the
 * output): every frame is a sent one, whole, in order (whole frames dropped only behind BOOT's / a fault's BLANK_N; one
 * cut frame at a fault). The polls: the capture decodes to the sent frame; SEND only into the released line (>= 20 us),
 * <= t_end + 900 us, with BLANK_N and the mux delayed; expected answers sent, late releases not. The statistics: frames,
 * bits per frame. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/dline.h"
#include "../src/core/ultra_protocol.h"
#include "../src/core/ultra_update.h"
#include "../src/core/ws_decode.h"

#define D_SMP     ((int64_t)DLINE_RING_WORDS * 16)  /* 8,192 samples */
#define HMASK     32767
#define US(x)     ((int64_t)((x) * 36.0 + 0.5))     /* receiver us -> samples */
#define TICK_SMP  1800                              /* 50 us */
#define RO_EN     11                                /* RO's enable after DE low, samples (0.3 us) */

static int g_fail, g_sfail;
static long g_checks;
static const char *sc_name = "";
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; if (g_sfail++ < 12) { printf("  FAIL %s:%d [%s] ", __FILE__, \
    __LINE__, sc_name); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static unsigned long long rs = 0x2545F4914F6CDD1DULL;
static unsigned long long rnd(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs; }
static double urand(double a, double b) { return a + (b - a) * (double)(rnd() >> 11) / 9007199254740992.0; }

/* ------------------------------------------------------------------------------------------------ chips */
typedef struct { const char *name; double t0h, t1h, per0, per1; int bpp, hdr; double reset_us; } chip_t;
static const chip_t chips[] = {
    { "WS2812B", 400, 800, 1250, 1250, 24, 0, 300 },        /* NeoPixelBus Ws2812x */
    { "SK6812-RGBW", 300, 600, 1250, 1250, 32, 0, 80 },     /* NeoPixelBus Sk6812: 80 us reset */
    { "WS2811-400k", 500, 1200, 2500, 2500, 24, 0, 50 },    /* datasheet-centred, NeoPixelBus 400 kbps reset 50 us */
    { "WS2811-400k-T1Hmax", 500, 1350, 2500, 2500, 24, 0, 300 },   /* T1H 1.35 us: up to 3 whole HIGH words per bit */
    { "WS2811-800k", 300, 900, 1250, 1250, 24, 0, 300 },    /* NeoPixelBus Ws2811 */
    { "WS2805", 300, 790, 1090, 1090, 40, 0, 300 },         /* WLED / NeoPixelBus Ws2805 */
    { "WS2915", 340, 700, 1300, 1350, 80, 32, 300 },        /* esphome-ws2915: 340/960, 700/650; 32-bit header */
};
enum { C_WS2812, C_SK6812, C_WS2811S, C_WS2811M, C_WS2811, C_WS2805, C_WS2915, C_ULTRA };
static const chip_t ultra_t = { "Ultra", 340, 700, 1300, 1350, 0, 0, 0 };   /* the hub's Ultra frames: WS2915 timing */

/* ------------------------------------------------------------------------------------------------ scenarios */
enum { I_LOW, I_PIX, I_POLL, I_IDENT, I_UPD, I_HIGH };
typedef struct {
    int type;
    double a;       /* LOW / HIGH: us; PIX: pixels per frame; POLL: release after t_end, us (< 0: never) */
    double b;       /* PIX: LOW after each frame's last edge, us (0: the chip's reset); POLL: the hub's retake after the
                       burst's end, us (may be < 0: before our DE low); HIGH / IDENT / UPD: LOW after, us */
    double c;       /* POLL: LOW after the retake, us */
    int n;          /* PIX: frames; POLL: rate */
    int expect;     /* POLL: 1 an answer, 0 none, -1 either */
    double wd;      /* POLL: the hub's retake this long after t_end, us, whatever happens (0: W_D / after the burst) */
} item_t;

typedef struct {
    const char *name;
    int chip;
    double jit_us;                      /* the tick's lateness 0 .. jit_us */
    double ml_us;                       /* the main loop decodes ml_min_us .. ml_us after FRAME_READY */
    double ml_min_us;
    double flash_ms, flash_len_ms;      /* a flash operation (len 0: none) */
    double freeze_ms;                   /* RX freezes (SPI OVR) (0: never) */
    int ovr_report;                     /* ... the glue reports it (dline_fault), else the distance check finds it */
    int skip;                           /* whole frames may drop behind a fault's BLANK_N */
    int trunc;                          /* cut frames allowed (a fault mid-frame) */
    double hsi;                         /* the receiver's sample period x (1 + hsi) in hub time */
    int tick_phase;                     /* the tick grid's phase in samples (-1: random) */
    int want_frames, want_bits, want_hold, want_marks, want_direct;   /* -1: not checked */
    int bits_maybe;                     /* resets < 100 us: a tick rarely lands in the latch's exact part (none: ok) */
    int race;                           /* a tick just after each frame's first bit (a '0') falls, none since 40 us
                                           before the frame: its latch's count must not take that bit (in the word RX
                                           is still writing) */
    double marker_us;                   /* the markers' HIGH (0: 15-40 us at random) */
    double probe_ms, probe_want_ms;     /* at probe_ms: ms since the line's last edge (wpos - edge_w) (0: none) */
    int nit;
    item_t it[40];
} scen_t;

/* ------------------------------------------------------------------------------------------------ the line */
enum { K_LOW = 0, K_PIX = 1, K_WIN = 2 };
#define EB_N (1 << 17)
typedef struct { long long s; unsigned char lvl, kind; } edge_t;
static edge_t eb[EB_N];
static long long eh, ec;                /* edges written; the one in effect */
static double ts_ns, ph_ns, hub_ns;     /* sample period / phase in hub ns; the hub's time */
static const scen_t *SC;
static int itn;                         /* the next item */
static int hub_wait;                    /* released after an RW frame, waiting for the burst / W_D */
static long long retake_s, wd_s;
static double post_us, ret_lat_us;
static int poll_rate;
static long long hi_rise_s;             /* the last undriven HIGH's rising edge */
static int hub_dead;                    /* the hub doesn't retake after the answer (only at wd_s) */

static long long s_of(double t) { return (long long)ceil((t - ph_ns) / ts_ns); }
static double ns_of(long long s) { return (double)s * ts_ns + ph_ns; }

static void put_s(long long s, int lvl, int kind)
{
    edge_t *l = &eb[(eh - 1) & (EB_N - 1)];
    if (eh > 0 && s <= l->s) {
        if (s < l->s) {
            printf("  model: edges out of order\n");
            exit(3);
        }
        l->lvl = (unsigned char)lvl;
        l->kind = (unsigned char)kind;
        return;
    }
    eb[eh & (EB_N - 1)].s = s;
    eb[eh & (EB_N - 1)].lvl = (unsigned char)lvl;
    eb[eh & (EB_N - 1)].kind = (unsigned char)kind;
    eh++;
}
static void put(double t, int lvl, int kind) { put_s(s_of(t), lvl, kind); }

/* frames the strip should see, and the hub's marker frames */
typedef struct { int nbits; unsigned char *bits; long long s0, s1, ff; } xframe_t;   /* ff: the first bit's fall */
#define XF_N 2048
static xframe_t xf[XF_N];
static int nxf;
typedef struct {
    int rw, nbits, expect, sent, flash;
    long long t_end, t_rel, send_s;
    unsigned char bytes[UPD_FRAME_BYTES];
} mframe_t;
#define MF_N 64
static mframe_t mf[MF_N];
static int nmf;

/* bits MSB first at the hub's time; returns the last falling edge's time */
static double first_fall_ns;
static double gen_bits(const unsigned char *bits, int nb, const chip_t *c, int kind)
{
    int i;
    double last = hub_ns;
    first_fall_ns = hub_ns;
    for (i = 0; i < nb; i++) {
        int b = (bits[i >> 3] >> (7 - (i & 7))) & 1;
        double h = (b ? c->t1h : c->t0h) + urand(-10.0, 10.0);
        put(hub_ns, 1, kind);
        put(hub_ns + h, 0, K_LOW);
        last = hub_ns + h;
        if (i == 0) {
            first_fall_ns = last;
        }
        hub_ns += b ? c->per1 : c->per0;
    }
    return last;
}

static void gen_item(void)
{
    const item_t *it = &SC->it[itn++];
    unsigned char f[UPD_FRAME_BYTES], arg[4] = { 0x12, 0x34, 0x56, 0x78 };
    static unsigned char seq;
    mframe_t *m;
    int k;
    switch (it->type) {
    case I_LOW:
        hub_ns += it->a * 1000.0;
        break;
    case I_HIGH:                        /* undriven: the fail-safe HIGH */
        put(hub_ns, 1, K_WIN);
        hi_rise_s = s_of(hub_ns);
        hub_ns += it->a * 1000.0;
        put(hub_ns, 0, K_LOW);
        hub_ns += it->b * 1000.0;
        break;
    case I_PIX: {
        const chip_t *c = &chips[SC->chip];
        for (k = 0; k < it->n; k++) {
            int nb = (int)it->a * c->bpp + c->hdr, i;
            xframe_t *x = &xf[nxf < XF_N ? nxf++ : XF_N - 1];
            double last;
            x->nbits = nb;
            x->bits = (unsigned char *)realloc(x->bits, (size_t)(nb + 7) / 8);
            for (i = 0; i < (nb + 7) / 8; i++) {
                x->bits[i] = (unsigned char)rnd();
            }
            if (SC->race) {
                x->bits[0] &= 0x7Fu;
            }
            x->s0 = s_of(hub_ns);
            last = gen_bits(x->bits, nb, c, K_PIX);
            x->s1 = s_of(last);
            x->ff = s_of(first_fall_ns);
            hub_ns = last + (it->b > 0 ? it->b : c->reset_us) * 1000.0;
        }
        break;
    }
    default: {                          /* a marker frame */
        int nb = it->type == I_UPD ? (int)UPD_FRAME_BITS : (int)(ULTRA_SLOT_BITS + ULTRA_HDR_BITS);
        double t_end;
        m = &mf[nmf < MF_N ? nmf++ : MF_N - 1];
        memset(m, 0, sizeof *m);
        if (it->type == I_POLL) {
            ultra_slot_poll(f, ++seq, 0, 1, ULTRA_PAGE_STATUS, (uint8_t)it->n, arg, 4);
            ultra_build_header_flags(f + ULTRA_SLOT_BYTES, (uint32_t)rnd(), ULTRA_HDR_RW);
        } else if (it->type == I_IDENT) {
            ultra_slot_power_off(f, ++seq);
            ultra_build_header_flags(f + ULTRA_SLOT_BYTES, (uint32_t)rnd(), 0);
        } else {
            for (k = 0; k < (int)UPD_BLOCK_BYTES - 2; k++) {
                f[k] = (unsigned char)rnd();
            }
            ultra_put16(f + UPD_BLOCK_BYTES - 2, ultra_crc16(f, UPD_BLOCK_BYTES - 2));
            ultra_build_header_flags(f + UPD_BLOCK_BYTES, (uint32_t)rnd(), ULTRA_HDR_UPD);
        }
        memcpy(m->bytes, f, (size_t)nb / 8);
        m->nbits = nb;
        m->rw = it->type == I_POLL;
        m->expect = it->type == I_POLL ? it->expect : 0;
        put(hub_ns, 1, K_WIN);                          /* the marker: HIGH 15-40 us, LOW 5-20 us */
        hub_ns += SC->marker_us > 0 ? SC->marker_us * 1000.0 : urand(15e3, 40e3);
        put(hub_ns, 0, K_LOW);
        hub_ns += SC->marker_us > 0 ? 10e3 : urand(5e3, 20e3);
        t_end = gen_bits(f, nb, &ultra_t, K_WIN);
        m->t_end = s_of(t_end);
        m->t_rel = -1;
        if (it->type == I_POLL && it->a >= 0.0) {
            put(t_end + it->a * 1000.0, 1, K_WIN);      /* the release: fail-safe HIGH */
            m->t_rel = s_of(t_end + it->a * 1000.0);
            hub_wait = 1;
            wd_s = s_of(t_end + (900.0 + (double)ultra_rw_slot_us((uint8_t)it->n)) * 1000.0);
            retake_s = wd_s;
            hub_dead = it->wd > 0.0;
            if (hub_dead) {
                retake_s = wd_s = s_of(t_end + it->wd * 1000.0);
            }
            post_us = it->c;
            ret_lat_us = it->b;
            poll_rate = it->n;
        } else {
            hub_ns = t_end + (it->type == I_POLL ? it->c : it->b) * 1000.0;
        }
        break;
    }
    }
}

/* ------------------------------------------------------------------------------------------------ the receiver */
static dline_t D;
static uint32_t ringst[DLINE_RING_WORDS / 2];
static uint16_t *const ring = (uint16_t *)ringst;
static long long k0;                    /* the word that went into slot 0 at the last (re)start */
static int frozen;
static long long kf;
static uint16_t txq[4];
static unsigned int inw;
static int mux, blank, de;
static long long de_rise, de_fall, tc_s;
static unsigned int falls, cum[4];
static unsigned char hist[HMASK + 1];
static long long last_high;             /* the last sample PA6 was HIGH */
static int pa6, kind_now, flashing, ovr_flag;
static long long pend_s, dec_s, next_tick, next_ram, f0, f1, freeze_s, tick_n, tick_ph;
static int race_i;
static uint32_t pend_a, pend_edges;
static int pend_on, pend_rec, pend_ram, dec_on, dec_mf, tick_pending;
static long long pend_lasthigh, tick_s;
/* the strip */
static struct { int prev, nb; long long rise, fall, s0; unsigned char bits[4096]; int idx, matched, skipped, cut; } sd;
static int64_t n_mismatch, n_leak, n_cut;
static int n_send, n_switch, n_fault_rec, n_refused;

static uint32_t rx_pos(long long s)
{
    long long k = frozen ? kf : s >> 4;
    return k < k0 ? 0u : (uint32_t)((k - k0) & DLINE_RING_MASK);   /* (0 until the restarted DMA's first word) */
}
static uint32_t tx_pos(long long s)
{
    return (uint32_t)(((s >> 4) + 3 - k0) & DLINE_RING_MASK);
}

static void strip_frame(void)
{
    int i, m = -1, lim = sd.idx + 60 > nxf ? nxf : sd.idx + 60;
    for (i = sd.idx; i < lim && m < 0; i++) {
        if (xf[i].nbits == sd.nb && memcmp(xf[i].bits, sd.bits, (size_t)sd.nb / 8) == 0 &&
            (sd.nb % 8 == 0 || ((xf[i].bits[sd.nb / 8] ^ sd.bits[sd.nb / 8]) & (0xFF00 >> (sd.nb % 8))) == 0)) {
            m = i;
        }
    }
    if (m >= 0) {
        if (m != sd.idx && sd.matched != 0) {           /* frames dropped after the first one shown */
            CHECK(SC->skip, "the strip lost %d frame(s) before frame %d", m - sd.idx, m);
        }
        sd.skipped += m - sd.idx;
        sd.idx = m + 1;
        sd.matched++;
    } else {                                            /* a frame not sent: a cut one (a prefix) at a fault, or junk */
        int pre = 0;
        for (i = sd.idx; i < lim && !pre; i++) {
            int b;
            pre = sd.nb < xf[i].nbits;
            for (b = 0; b + 1 < sd.nb && pre; b++) {   /* (the last one may be cut short by BLANK_N) */
                pre = ((xf[i].bits[b >> 3] ^ sd.bits[b >> 3]) & (0x80 >> (b & 7))) == 0;
            }
        }
        CHECK(pre && sd.cut < SC->trunc, "the strip got a frame of %d bits that was never sent (%s) at %.1f us", sd.nb,
              pre ? "a cut frame" : "junk", (double)sd.s0 / 36.0);
        sd.cut++;
    }
    sd.nb = 0;
}

static int dbg;
static const char *dbg_name;
/* the DUT's whole state after every tick and every capture handed over, hashed (FNV-1a): the block paths must give
 * the same digest as the word-by-word reference (DLINE_WORDS_ONLY; test/build_dline.cmd compares the outputs) */
static uint32_t dig;
static void dig32(uint32_t v)
{
    int i;
    for (i = 0; i < 4; i++) {
        dig = (dig ^ ((v >> (8 * i)) & 0xFFu)) * 16777619u;
    }
}
static void dig_state(uint32_t a)
{
    dig32(a);
    dig32(D.wpos); dig32(D.zpos); dig32(D.zs); dig32(D.os); dig32(D.lzs); dig32(D.sync_zs); dig32(D.run);
    dig32(D.orun); dig32(D.m0); dig32(D.m1); dig32(D.mark_w); dig32(D.end_w); dig32(D.rel_w); dig32(D.send_w);
    dig32(D.txend_w); dig32(D.edge_w); dig32(D.rec_w); dig32(D.pend_w); dig32(D.v1); dig32(D.base); dig32(D.pend_e);
    dig32(D.cap_n); dig32(D.state); dig32(D.direct); dig32(D.rec); dig32(D.restart); dig32(D.bl); dig32(D.blank);
    dig32(D.hi); dig32(D.txdone); dig32(D.cap_ok); dig32(D.cap_own); dig32(D.base_ok); dig32(D.sync); dig32(D.pend_k);
    dig32(D.seq); dig32(D.cur_seq); dig32(D.cap_ready); dig32(D.st.frames); dig32(D.st.bits_last);
    dig32(D.st.bits_min); dig32(D.st.bits_max); dig32(D.st.bits_n); dig32(D.st.long_high); dig32(D.st.marks);
    dig32(D.st.answered); dig32(D.st.cap_drops); dig32(D.st.late); dig32(D.st.faults);
}
static void apply(long long s)
{
    uint32_t a = pend_a;
    int ok = 1;
    pend_on = 0;
    if (dbg && (a & ~(uint32_t)DLINE_A_WINDOW_END) != 0u) {
        printf("    %9.2f us: actions %03x (tick %.2f) state %u rec %u direct %u, mux %d blank %d\n", (double)s / 36.0,
               (unsigned)a, (double)tick_s / 36.0, D.state, D.rec, D.direct, mux, blank);
    }
    if (a & DLINE_A_BLANK_ON) {
        blank = 1;
    }
    if (a & DLINE_A_RESTART) {                          /* the glue: zero the ring, restart RX / TX, the mux as it is */
        CHECK(!pend_ram, "RESTART from the SRAM tick");
        memset(ringst, 0, sizeof ringst);
        k0 = (s >> 4) + 1;
        frozen = 0;
        memset(txq, 0, sizeof txq);
        dline_restart(&D, 0);
        n_fault_rec++;
    }
    if (a & DLINE_A_LIVE_CHECK) {
        ok = !pa6 && (falls & 0xFFFFu) == pend_edges;  /* (TIM4 re-read after its input filter's delay) */
        if (!ok) {
            dline_refused(&D);
            n_refused++;
        }
    }
    if (ok) {
        long long lh = (a & DLINE_A_LIVE_CHECK) ? last_high : pend_lasthigh;
        if (a & DLINE_A_MUX_DELAYED) {
            CHECK(s - lh > D_SMP, "mux -> delayed with the line HIGH %.1f us ago (< D)", (double)(s - lh) / 36.0);
            mux = 1;
            n_switch++;
        }
        if (a & DLINE_A_MUX_DIRECT) {
            CHECK(0, "MUX_DIRECT (never back to direct)");
            mux = 0;
        }
        if (a & DLINE_A_BLANK_OFF) {
            if (pend_rec && mux && !(a & DLINE_A_MUX_DELAYED)) {
                CHECK(s - lh > D_SMP, "BOOT / fault release with the line HIGH %.1f us ago", (double)(s - lh) / 36.0);
            }
            blank = 0;
        }
    }
    if (a & DLINE_A_SEND) {
        mframe_t *m = nmf > 0 ? &mf[nmf - 1] : NULL;
        CHECK(m != NULL && m->rw && hub_wait, "SEND without an RW poll waiting");
        if (m != NULL) {
            CHECK(blank && mux, "SEND without BLANK_N (%d) / the mux delayed (%d)", blank, mux);
            CHECK(eb[ec & (EB_N - 1)].lvl == 1 && m->t_rel >= 0 && s - m->t_rel >= US(20),
                  "SEND into a line not released 20 us (release %lld us before)", (s - m->t_rel) / 36);
            CHECK(s - m->t_end <= US(900) + 72, "SEND %.1f us after t_end (> 900)", (double)(s - m->t_end) / 36.0);
            CHECK(!m->sent, "two SENDs in one window");
            m->sent = 1;
            m->send_s = s;
        }
        n_send++;
        de = 1;
        de_rise = s;
        de_fall = s + US(3.0 + 330.0 * (double)(1 << poll_rate));
        tc_s = de_fall;
        if (hub_wait && de_fall <= wd_s && !hub_dead) { /* the hub has the answer at the burst's end */
            long long r = s_of(ns_of(de_fall) + ret_lat_us * 1000.0);
            if (r < retake_s) {
                retake_s = r;
            }
        }
    }
    if (a & DLINE_A_FRAME_READY) {
        dec_on = 1;
        dec_s = s + US(urand(SC->ml_min_us, SC->ml_us));
        for (dec_mf = nmf - 1; dec_mf > 0 && mf[dec_mf].t_end >= s; dec_mf--) {
        }
    }
}

static void run_tick(long long s, int ram)
{
    uint32_t edges = cum[(s - 3) & 3] & 0xFFFFu;   /* (TIM4: 3 samples late) */
    uint8_t late0 = D.st.late;
    pend_rec = D.rec;
    pend_lasthigh = last_high;
    if (!ram && ovr_flag) {
        ovr_flag = 0;
        pend_a = dline_fault(&D);
    } else if (ram) {
        pend_a = dline_ram_tick(&D, ring, rx_pos(s), tx_pos(s), edges);
    } else {
        pend_a = dline_tick(&D, ring, rx_pos(s), tx_pos(s), edges);
    }
    if (dbg && D.st.late != late0) {
        printf("    %9.2f us: %s tick late (rx %u tx %u, zpos %u wpos %u, state %u restart %u)\n", (double)s / 36.0,
               ram ? "SRAM" : "20 kHz", rx_pos(s), tx_pos(s), D.zpos, D.wpos, D.state, D.restart);
    }
    dig_state(pend_a);
    if (pend_a & DLINE_A_FRAME_READY) {
        uint32_t i;
        dig32(D.cap_w0);
        for (i = 0; i < D.cap_n; i++) {
            dig32(D.cap[i]);
        }
    }
    pend_edges = edges;
    pend_ram = ram;
    pend_on = 1;
    pend_s = s + (ram ? 0 : (long long)(rnd() % 73));
    tick_s = s;
}

static void main_decode(void)
{
    static const ws_timing_t tm = WS_TIMING_DEFAULT;
    unsigned char fb[UPD_FRAME_BYTES];
    int nb = 0, ok = 0, n;
    mframe_t *m = dec_mf >= 0 ? &mf[dec_mf] : NULL;
    /* as main_ud.c's ud_decode: the capture's bits counted to its end, exactly 320 or 384 */
    n = ws_decode_count(D.cap, (uint32_t)D.cap_n * 16u, UPD_FRAME_BITS, &tm, fb);
    if (n == (int)(ULTRA_SLOT_BITS + ULTRA_HDR_BITS) || n == (int)UPD_FRAME_BITS) {
        nb = n;
    }
    CHECK(m != NULL && nb == m->nbits && memcmp(fb, m->bytes, (size_t)nb / 8) == 0,
          "the capture (%u words) decodes to %d bits, not the frame sent (%d bits)", D.cap_n, nb, m ? m->nbits : 0);
    if (nb == (int)(ULTRA_SLOT_BITS + ULTRA_HDR_BITS)) {
        ok = (ultra_header_flags(fb + ULTRA_SLOT_BYTES) & (ULTRA_HDR_RW | ULTRA_HDR_EXT)) == ULTRA_HDR_RW &&
             ultra_slot_check(fb) == 0 && ultra_poll_valid(fb) == 0 && fb[ULTRA_POLL_INDEX] == 0;
    }
    dline_arm(&D, ok);
}

static void run(const scen_t *sc)
{
    long long s, end_s = -1;
    int prev_out = 0;
    SC = sc;
    sc_name = sc->name;
    g_sfail = 0;
    dbg = dbg_name != NULL && strncmp(sc->name, dbg_name, strlen(dbg_name)) == 0;
    /* the line */
    eh = ec = 0;
    itn = 0;
    hub_wait = 0;
    nxf = nmf = 0;
    ts_ns = 1e9 / 36e6 * (1.0 + sc->hsi);
    ph_ns = urand(0.0, ts_ns);
    hub_ns = 0;
    put_s(0, 0, K_LOW);
    /* the receiver */
    memset(ringst, 0, sizeof ringst);
    k0 = 0;
    frozen = 0;
    memset(txq, 0, sizeof txq);
    inw = 0;
    mux = 0;                            /* BOOT: direct, BLANK_N LOW */
    blank = 1;
    de = 0;
    de_rise = de_fall = tc_s = -1;
    falls = 0;
    memset(cum, 0, sizeof cum);
    memset(hist, 0, sizeof hist);
    last_high = -(1LL << 40);
    pa6 = 0;
    pend_on = dec_on = 0;
    ovr_flag = 0;
    tick_pending = 0;
    tick_ph = (sc->tick_phase >= 0 ? sc->tick_phase : (long long)(rnd() % TICK_SMP)) + 4;
    tick_n = 0;
    next_tick = tick_ph;
    race_i = 0;
    f0 = sc->flash_len_ms > 0 ? US(sc->flash_ms * 1000.0) : -1;
    f1 = sc->flash_len_ms > 0 ? f0 + US(sc->flash_len_ms * 1000.0) : -1;
    next_ram = f0;
    flashing = 0;
    freeze_s = sc->freeze_ms > 0 ? US(sc->freeze_ms * 1000.0) : -1;
    memset(&sd, 0, sizeof sd);
    n_mismatch = n_leak = n_cut = 0;
    n_send = n_switch = n_fault_rec = n_refused = 0;
    dline_init(&D, 0);
    dig = 2166136261u;

    for (s = 0;; s++) {
        long long k = s >> 4;
        int out, src, srck, ref;
        /* the hub: items ahead of the line, the retake */
        while (!hub_wait && itn < sc->nit && eb[(eh - 1) & (EB_N - 1)].s < s + 40000) {
            gen_item();
        }
        if (hub_wait && s >= retake_s) {
            put_s(retake_s, 0, K_LOW);
            hub_ns = ns_of(retake_s) + post_us * 1000.0;
            hub_wait = 0;
        }
        if (end_s < 0 && !hub_wait && itn >= sc->nit) {
            end_s = (eb[(eh - 1) & (EB_N - 1)].s > s ? eb[(eh - 1) & (EB_N - 1)].s : s) + US(1200);
            if (hub_ns > ns_of(end_s - US(1000))) {
                end_s = s_of(hub_ns) + US(1000);
            }
        }
        if (end_s >= 0 && s >= end_s) {
            break;
        }
        /* the line at s */
        while (ec + 1 < eh && eb[(ec + 1) & (EB_N - 1)].s <= s) {
            ec++;
        }
        {
            const edge_t *e = &eb[ec & (EB_N - 1)];
            int ro = de_rise >= 0 && s >= de_rise && s < de_fall + RO_EN;
            int p = e->lvl && !ro;
            if (pa6 && !p) {
                falls++;
            }
            pa6 = p;
            kind_now = p ? e->kind : K_LOW;
        }
        cum[s & 3] = falls;
        if (pa6) {
            last_high = s;
        }
        hist[s & HMASK] = (unsigned char)(pa6 | kind_now << 1);
        /* TX prefetch: output word k + 2's slot, at the start of word k */
        if ((s & 15) == 0) {
            long long kk = k + 2;
            txq[kk & 3] = kk >= k0 ? ring[(kk - k0) & DLINE_RING_MASK] : 0;
        }
        /* the CPU */
        if (s == freeze_s) {
            frozen = 1;
            kf = k;
            if (sc->ovr_report) {
                ovr_flag = 1;
            }
        }
        if (s == tc_s) {                                /* the burst's TC: DE low */
            de = 0;
            dline_burst_done(&D, rx_pos(s));
            tc_s = -1;
        }
        if (f0 >= 0) {
            if (s == f0) {
                flashing = 1;
            }
            if (s == f1) {
                flashing = 0;
                if (tick_pending) {
                    tick_pending = 0;
                    if (pend_on) {
                        apply(s);
                    }
                    run_tick(s, 0);                     /* the tick held off by the flash operation */
                }
            }
        }
        if (pend_on && s >= pend_s) {
            apply(s);
        }
        if (sc->race) {
            while (race_i < nxf && xf[race_i].ff + 3 < s) {
                race_i++;
            }
            if (race_i < nxf && s == xf[race_i].ff + 3) {
                if (pend_on) {
                    apply(s);
                }
                run_tick(s, 0);                         /* the tick right after the first bit's fall */
                race_i++;
            }
        }
        if (s == next_tick && sc->race && race_i < nxf && s >= xf[race_i].s0 - US(40)) {
            tick_n++;                                   /* (none in the 40 us before a frame) */
            next_tick = tick_n * TICK_SMP + tick_ph + (long long)urand(0.0, sc->jit_us * 36.0);
        }
        if (s == next_tick) {
            if (flashing) {
                tick_pending = 1;
            } else {
                if (pend_on) {
                    apply(s);                           /* (the glue finishes before the next tick) */
                }
                run_tick(s, 0);
            }
            tick_n++;
            next_tick = tick_n * TICK_SMP + tick_ph + (long long)urand(0.0, sc->jit_us * 36.0);
            if (next_tick <= s) {
                next_tick = s + 1;
            }
        }
        if (flashing && s == next_ram) {
            if (pend_on) {
                apply(s);
            }
            run_tick(s, 1);
            next_ram += TICK_SMP + (long long)(rnd() % 36);
        }
        if (pend_on && s >= pend_s) {
            apply(s);
        }
        if (sc->probe_ms > 0 && s == US(sc->probe_ms * 1000.0)) {
            double ms = (double)(D.wpos - D.edge_w) / DLINE_W_PER_MS;
            double want = sc->probe_want_ms >= 0.0 ? sc->probe_want_ms : (double)(s - hi_rise_s) / 36000.0;
            CHECK(fabs(ms - want) < 0.2, "%.2f ms since the last edge, want %.2f", ms, want);
        }
        if (dec_on && s >= dec_s && !flashing) {
            dec_on = 0;
            main_decode();
        }
        /* the output and its reference */
        out = ((mux ? (txq[k & 3] >> (15 - (s & 15))) & 1u : (unsigned)pa6) != 0) && !blank;
        if (mux) {
            if (k - DLINE_RING_WORDS >= k0) {
                int h = hist[(s - D_SMP) & HMASK];
                src = h & 1;
                srck = h >> 1;
            } else {
                src = 0;
                srck = K_LOW;
            }
        } else {
            src = pa6;
            srck = kind_now;
        }
        ref = src && srck == K_PIX;
        if (out != (ref && !blank)) {
            if (n_mismatch++ < 3) {
                CHECK(0, "output %d, expected %d at %.2f us (mux %d blank %d, state %d, source kind %d)", out,
                      ref && !blank, (double)s / 36.0, mux, blank, D.state, srck);
            }
        }
        if (out && srck != K_PIX) {
            n_leak++;
        }
        if (blank && ref && !D.rec && !pend_on) {
            if (n_cut++ < 3) {
                CHECK(0, "the window's BLANK_N cut a pixel at %.2f us", (double)s / 36.0);
            }
        }
        /* the strip */
        if (out && !prev_out) {
            sd.rise = s;
            if (sd.nb == 0) {
                sd.s0 = s;
            }
        } else if (!out && prev_out) {
            double w = (double)(s - sd.rise) * 1e9 / 36e6;
            const chip_t *c = &chips[sc->chip];
            if (sd.nb < 4096 * 8) {
                if (w >= (c->t0h + c->t1h) / 2.0) {
                    sd.bits[sd.nb >> 3] |= (unsigned char)(0x80 >> (sd.nb & 7));
                } else {
                    sd.bits[sd.nb >> 3] &= (unsigned char)~(0x80 >> (sd.nb & 7));
                }
                sd.nb++;
            }
            sd.fall = s;
        }
        if (!out && sd.nb > 0 && s - sd.fall >= US(40)) {
            strip_frame();
        }
        prev_out = out;
        /* RX: the word's end */
        inw = (inw << 1) | (unsigned)pa6;
        if ((s & 15) == 15) {
            if (!frozen && k >= k0) {
                ring[(k - k0) & DLINE_RING_MASK] = (uint16_t)inw;
            }
            inw = 0;
        }
    }
    /* the strip saw every frame that ended a delay + latch before the end (after the first one shown) */
    {
        int i, lost = 0;
        for (i = sd.idx; i < nxf; i++) {
            if (xf[i].s1 + D_SMP + US(200) < end_s) {
                lost++;
            }
        }
        CHECK(lost == 0 || (sc->skip && sd.matched > 0), "%d frame(s) never reached the strip (%d shown, %d skipped)",
              lost, sd.matched, sd.skipped);
        CHECK(sd.matched > 0 || nxf == 0, "no pixel frame reached the strip");
    }
    CHECK(n_mismatch == 0, "%lld output samples differ from the delayed pixel stream", (long long)n_mismatch);
    CHECK(n_leak == 0, "%lld non-pixel HIGH samples reached the strip", (long long)n_leak);
    {
        int i;
        for (i = 0; i < nmf; i++) {
            if (mf[i].expect >= 0) {
                CHECK(mf[i].sent == mf[i].expect, "poll %d (release %.0f us after t_end): %s", i,
                      mf[i].t_rel >= 0 ? (double)(mf[i].t_rel - mf[i].t_end) / 36.0 : -1.0,
                      mf[i].sent ? "answered (not expected)" : "not answered");
            }
        }
    }
    if (sc->want_frames >= 0) {
        CHECK(D.st.frames == (uint32_t)sc->want_frames, "frames %u, want %d", D.st.frames, sc->want_frames);
    }
    if (sc->want_bits >= 0) {
        CHECK(D.st.bits_n == 0 ? sc->bits_maybe : D.st.bits_last == (uint32_t)sc->want_bits && D.st.bits_min ==
              D.st.bits_last && D.st.bits_max == D.st.bits_last, "bits last %u min %u max %u (%u measured), want %d",
              D.st.bits_last, D.st.bits_min, D.st.bits_max, D.st.bits_n, sc->want_bits);
    }
    if (sc->want_hold >= 0) {
        CHECK(D.st.long_high == (uint16_t)sc->want_hold, "long-HIGH events %u, want %d", D.st.long_high, sc->want_hold);
    }
    if (sc->want_marks >= 0) {
        CHECK(D.st.marks == (uint16_t)sc->want_marks, "markers %u, want %d", D.st.marks, sc->want_marks);
    }
    if (sc->want_direct >= 0) {
        CHECK(mux == !sc->want_direct, "the mux is %s at the end", mux ? "delayed" : "direct");
    }
    CHECK(D.st.late == 0, "%u ticks too late for the ring", D.st.late);
    printf("  %-34s %6.1f ms  frames %3d shown %3d (skipped %d, cut %d)  markers %u  SENDs %d  holds %u  "
           "bits %u/%u  faults %u  refused %d  digest %08x%s\n", sc->name, (double)end_s / 36000.0, nxf, sd.matched, sd.skipped, sd.cut,
           D.st.marks, n_send, D.st.long_high, D.st.bits_last, D.st.bits_n, D.st.faults, n_refused, (unsigned)dig, g_sfail ? "  FAILED" : "");
}

/* ------------------------------------------------------------------------------------------------ the scenarios */
static scen_t S;
static void sc_begin(const char *name, int chip)
{
    memset(&S, 0, sizeof S);
    S.name = name;
    S.chip = chip;
    S.jit_us = 3.0;
    S.ml_us = 300.0;
    S.ml_min_us = 20.0;
    S.tick_phase = -1;
    S.hsi = urand(-0.01, 0.01);
    S.want_frames = S.want_bits = S.want_hold = S.want_marks = S.want_direct = -1;
}
static void add(int type, double a, double b, double c, int n, int expect)
{
    item_t *it = &S.it[S.nit++];
    it->type = type;
    it->a = a;
    it->b = b;
    it->c = c;
    it->n = n;
    it->expect = expect;
}
#define LOW(us)                add(I_LOW, us, 0, 0, 0, -1)
#define PIX(px, frames, gap)   add(I_PIX, px, gap, 0, frames, -1)
#define POLL(rel, lat, after, rate, exp) add(I_POLL, rel, lat, after, rate, exp)
#define POLLWD(rel, wd_us, after, rate, exp) (add(I_POLL, rel, 0, after, rate, exp), S.it[S.nit - 1].wd = (wd_us))
#define IDENT(after)           add(I_IDENT, 0, after, 0, 0, -1)
#define UPD(after)             add(I_UPD, 0, after, 0, 0, -1)
#define HIGH(us, after)        add(I_HIGH, us, after, 0, 0, -1)

static const char *only;
static void go(void)
{
    if (only == NULL || strncmp(S.name, only, strlen(only)) == 0) {
        run(&S);
    }
}

int main(int argc, char **argv)
{
    int c, i, g;
    only = argc > 1 && strcmp(argv[1], "*") != 0 ? argv[1] : NULL;   /* (`*`: all, with a trace: argv[2]) */
    dbg_name = argc > 2 && strcmp(argv[2], "-") != 0 ? argv[2] : NULL;
    if (argc > 3) {
        rs = strtoull(argv[3], NULL, 0) | 1u;          /* another seed: `dline_tests.exe * - <seed>` */
    }
    printf("dline: the Ultra-Digital receiver's delay line on a sample-exact model\n");

    /* (6) every chip: frames and bits per frame; the short resets (80 / 50 us) never give a quiet run: BLANK_N is
     * released in direct mode after 50 ms (the fallback), in a latch gap */
    for (c = 0; c <= C_WS2915; c++) {
        char *nm = (char *)malloc(64);
        int px = c == C_WS2915 ? 12 : 40, nf = chips[c].reset_us < 200 ? 40 : 14;
        sprintf(nm, "chip %s", chips[c].name);
        sc_begin(nm, c);
        LOW(400);
        PIX(px, nf, 0);
        LOW(400);
        S.want_frames = nf;
        S.want_bits = px * chips[c].bpp + chips[c].hdr;
        S.bits_maybe = chips[c].reset_us < 100;
        S.want_hold = 0;
        S.want_direct = 0;                              /* (the last 400 us LOW: quiet) */
        go();
    }

    /* (3) polls on a WS2812B stream: releases 5 / 80 / 150 / 850 / 950 us, rates 0 / 1 / 2, the hub's retake 3 us
     * after the burst / 1 us before our DE low (the line LOW at DE low) / 40 us after; pixels 20 us after the retake.
     * 850 / 950: END went to PASS at t_end + 250 us (rule 4): the late release is a long HIGH -> MARK -> HOLD, zeroed,
     * no answer (2 long-HIGH events) */
    sc_begin("polls releases + retakes", C_WS2812);
    LOW(400);
    PIX(30, 2, 0);
    POLL(5, 3, 20, 0, 1);
    PIX(30, 2, 0);
    POLL(80, -1, 20, 0, 1);
    PIX(30, 2, 0);
    POLL(150, 40, 20, 0, 1);
    PIX(30, 2, 0);
    POLL(850, 3, 20, 0, 0);
    PIX(30, 2, 0);
    POLL(950, 3, 20, 0, 0);
    PIX(30, 2, 0);
    POLL(5, 3, 20, 1, 1);
    PIX(30, 2, 0);
    POLL(100, 5, 20, 2, 1);
    PIX(30, 2, 0);
    POLL(-1, 0, 300, 0, 0);                             /* never released (a hub too late): END -> PASS at 250 us */
    PIX(30, 2, 0);
    S.want_marks = 8;
    S.want_hold = 2;
    S.want_direct = 0;
    go();
    /* the release limit: 230 us answered at every tick phase (END keeps waiting until t_end + 250 us) */
    sc_begin("polls release 230 us", C_WS2812);
    LOW(400);
    for (i = 0; i < 6; i++) {
        PIX(30, 1, 0);
        POLL(230, 3, 20, i % 3, 1);
    }
    PIX(30, 1, 0);
    S.want_hold = 0;
    go();

    /* (3) non-RW frames: IDENTIFY 7 and an UPD frame (384 bits), pixels 300 us after */
    sc_begin("polls non-RW IDENTIFY + UPD", C_WS2805);
    LOW(400);
    PIX(30, 2, 0);
    IDENT(300);
    PIX(30, 2, 0);
    UPD(300);
    PIX(30, 2, 0);
    POLL(30, 3, 20, 0, 1);
    PIX(30, 2, 0);
    S.want_marks = 3;
    S.want_hold = 0;
    go();
    /* (review #2) the main loop still decodes a non-RW frame when the next marker frame comes 300 us later: that one
     * isn't captured (no answer) and the first capture stays as it was (main_decode checks it) */
    sc_begin("capture held by the main loop", C_WS2812);
    LOW(400);
    PIX(30, 2, 0);
    IDENT(300);
    POLL(20, 3, 20, 0, 0);
    PIX(30, 2, 0);
    POLL(20, 3, 20, 0, 1);
    PIX(30, 2, 0);
    S.ml_min_us = 600.0;
    S.ml_us = 700.0;
    S.want_marks = 3;
    go();
    /* (review #5) a hub 60 us early: pixels 240 us after a non-RW frame end the window at their first edge, whole */
    sc_begin("pixels 240 us after IDENTIFY", C_WS2805);
    LOW(400);
    PIX(30, 2, 0);
    IDENT(240);
    PIX(30, 2, 0);
    UPD(200);
    PIX(30, 2, 0);
    S.want_hold = 0;
    go();
    /* a dead hub: released 20 ms (REPLY -> HOLD at t_end + 5 ms: a POLL of rate 3, not valid, no answer; RETAKE ->
     * HOLD: answered, then nobody drives) */
    sc_begin("dead hub after a poll", C_WS2812);
    LOW(400);
    PIX(30, 2, 0);
    POLLWD(20, 20000, 300, 3, 0);
    PIX(30, 2, 0);
    POLLWD(20, 20000, 300, 0, 1);
    PIX(30, 2, 0);
    S.want_hold = 2;
    go();

    /* back-to-back polls, every chip type's stream */
    for (c = 0; c <= C_WS2915; c++) {
        char *nm = (char *)malloc(64);
        sprintf(nm, "polls on %s", chips[c].name);
        sc_begin(nm, c);
        LOW(400);
        for (i = 0; i < 6; i++) {
            PIX(c == C_WS2915 ? 8 : 25, 3, 0);
            if (chips[c].reset_us < 200) {
                LOW(350);                               /* the hub's >= 350 us before the marker (Quindor, rule 2) */
            }
            POLL(urand(5, 150), urand(-1, 40), 20, i % 3, 1);
        }
        PIX(c == C_WS2915 ? 8 : 25, 3, 0);
        S.want_marks = 6;
        S.want_direct = 0;
        go();
    }

    /* (5) HOLD: an undriven line for 1.2 s, pixels 1 us after it; a line released 0.6 us after a frame's last bit
     * (signal loss mid-stream) for 10 ms */
    sc_begin("hold 1.2 s + pixels 1 us after", C_WS2812);
    LOW(400);
    PIX(30, 2, 0);
    HIGH(1.2e6, 1);
    PIX(30, 3, 0);
    S.want_hold = 1;
    S.probe_ms = 1001.0;                                /* inside the HIGH that began at ~2.7 ms */
    S.probe_want_ms = -1.0;
    go();
    sc_begin("hold + pixels 0.2 us after", C_WS2805);   /* (the edge in the HOLD's last word: 8 phases) */
    LOW(400);
    PIX(30, 2, 0);
    for (i = 0; i < 8; i++) {
        HIGH(200 + 7.3 * i, 0.2);
        PIX(30, 1, 0);
    }
    S.want_hold = 8;
    go();
    /* boot into an undriven line (HIGH from power-up), then the hub drives */
    sc_begin("boot into an undriven line", C_WS2812);
    HIGH(5000, 400);
    PIX(30, 4, 0);
    S.want_hold = 1;
    go();
    sc_begin("hold right after a frame", C_WS2805);
    LOW(400);
    PIX(30, 1, 0);
    PIX(30, 1, 0.6);
    HIGH(10000, 300);
    PIX(30, 2, 0);
    POLL(20, 3, 20, 0, 1);
    PIX(30, 2, 0);
    S.want_hold = 1;
    go();

    /* faults: RX frozen mid-frame (the distance check; the glue's OVR report): the output held LOW (one cut frame),
     * frames dropped until the next quiet run, then intact */
    for (i = 0; i < 2; i++) {
        sc_begin(i ? "fault OVR reported" : "fault RX frozen", C_WS2812);
        LOW(400);
        PIX(60, 6, 0);
        POLL(20, 3, 20, 0, -1);
        PIX(60, 6, 0);
        S.freeze_ms = 3.0;
        S.ovr_report = i;
        S.skip = 1;
        S.trunc = 1;
        S.want_direct = 0;
        go();
    }

    /* bits per frame: a tick right after the next frame's first bit, with the latch not seen before: the bit's whole
     * pulse may sit in the word RX is still writing (TIM4 counted it, the ring doesn't show it yet) */
    sc_begin("bits: the in-flight word", C_SK6812);
    LOW(400);
    PIX(20, 60, 0);
    LOW(400);
    S.race = 1;
    S.want_frames = 60;
    S.want_bits = 20 * 32;
    S.bits_maybe = 1;
    go();
    sc_begin("fault on an 80 us-reset stream", C_SK6812);
    LOW(400);
    PIX(30, 50, 0);                                     /* > 50 ms of frames without a quiet run after the fault */
    LOW(400);
    PIX(30, 4, 0);
    S.freeze_ms = 4.0;
    S.skip = 1;
    S.trunc = 1;
    S.want_direct = 0;
    go();

    /* (zeroing) a 16 ms flash operation (the SRAM tick alone): a 20 us marker and its RW frame inside it, no
     * capture, no answer, zeroed; a poll right after it answered */
    sc_begin("flash op with a poll inside", C_WS2812);
    LOW(400);
    PIX(30, 3, 0);
    POLL(20, 3, 20, 0, 0);
    PIX(30, 3, 0);
    POLL(10, 3, 20, 0, -1);
    PIX(30, 14, 0);
    POLL(10, 3, 20, 0, 1);
    PIX(30, 2, 0);
    S.flash_ms = 3.0;
    S.flash_len_ms = 16.0;
    S.marker_us = 20.0;
    go();
    /* ... an undriven line and a fault inside it: the SRAM tick zeroes the HIGH, blanks at the fault (no RESTART from
     * it); the tick after the flash operation restarts the ring */
    sc_begin("flash op with a hold + a fault", C_WS2805);
    LOW(400);
    PIX(30, 3, 0);
    HIGH(2000, 300);
    PIX(30, 10, 0);
    POLL(20, 3, 20, 0, 1);
    PIX(30, 2, 0);
    S.flash_ms = 3.0;
    S.flash_len_ms = 16.0;
    S.freeze_ms = 10.0;
    S.skip = 1;
    S.trunc = 1;
    S.want_hold = 1;
    go();
    /* ... from boot (BLANK_N held, direct) on a short-reset stream: the poll's gap switches the mux from the SRAM
     * tick */
    sc_begin("flash op at boot, quiet inside", C_SK6812);
    LOW(100);
    PIX(30, 3, 0);
    LOW(300);
    POLL(20, 3, 20, 0, 0);
    PIX(30, 30, 0);
    S.flash_ms = 0.05;
    S.flash_len_ms = 16.0;
    go();

    /* the tick up to 50 us late: zeroing holds (some answers may miss the tick grid) */
    sc_begin("tick 0-50 us late", C_WS2812);
    LOW(400);
    for (i = 0; i < 8; i++) {
        PIX(30, 2, 0);
        POLL(urand(5, 150), urand(-1, 40), 20, i % 3, -1);
    }
    PIX(30, 2, 0);
    S.jit_us = 50.0;
    go();

    /* a slow main loop (decode 600-1,000 us after the frame): no answer after t_end + 900, zeroing holds */
    sc_begin("slow main loop", C_WS2812);
    LOW(400);
    for (i = 0; i < 4; i++) {
        PIX(30, 2, 0);
        POLL(urand(5, 150), 3, 20, 0, -1);
    }
    PIX(30, 2, 0);
    S.ml_us = 1000.0;
    go();

    /* Quindor's rule: a poll whose marker comes exactly at the hub's minimum gap after the last pixel edge, right after
     * boot (BLANK_N held) and in DIRECT-PASS (after the 50 ms fallback on an 80 us-reset stream), every tick phase:
     * nothing of it on the strip. 350 us (rule 2, 2026-10-16) with ticks up to 45 us late; 300 us (the first rule) with
     * up to 15 us: the margin */
    for (g = 300; g <= 350; g += 50) {
        for (i = 0; i < 18; i++) {
            char *nm = (char *)malloc(64);
            double jit = g == 350 ? (i & 1 ? 15.0 : 45.0) : (i & 1 ? 3.0 : 15.0);
            sprintf(nm, "boot marker at %d us, phase %d", g, i * 100);
            sc_begin(nm, C_WS2812);
            PIX(30, 1, g);
            POLL(20, 3, 20, 0, -1);
            PIX(30, 3, 0);
            S.tick_phase = i * 100;
            S.jit_us = jit;
            go();
            sprintf(nm = (char *)malloc(64), "direct marker at %d us, phase %d", g, i * 100);
            sc_begin(nm, C_SK6812);
            LOW(100);
            PIX(30, 45, 0);                             /* 80 us resets for > 50 ms: the fallback */
            LOW(g - 80);                                /* (+ the frame's 80 us: g us from its last edge) */
            POLL(20, 3, 20, 0, -1);
            PIX(30, 5, 0);
            S.tick_phase = i * 100;
            S.jit_us = jit;
            go();
        }
    }

    printf("dline: %ld checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
