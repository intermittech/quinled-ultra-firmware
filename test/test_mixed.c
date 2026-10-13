/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Diff-Power mixed chains at the wire level (old and ultra receivers in one chain). Standalone runner:
 * test\build_mixed.cmd (not part of test_main.c; test_wire.c's chain model, extended).
 *
 * Devices:
 *  - MCU: a dig2analog-ultra board or a Diff-Power ultra receiver (the same firmware). The real frame tracker
 *    (frame_track.h) and per-frame decisions (frame_rx.c), gate plans one frame later, a main-loop latency model.
 *    Its decoder is modelled by its rule: a high run of >= 17 samples at 36 MHz is a '1' (HSI +-2.2 % per device,
 *    a random sampling phase per bit); a capture with a reset gap (>= 50 us) or a high pulse > 2.2 us fails.
 *    Its output is DIN AND FWD_EN: no re-timing, the pulse widths go on as they came, plus the gate's own skew.
 *  - WS2805 / WS2915 chip (a chip-based dig2analog board): takes its 40 bits (WS2915: keeps the 32-bit header and
 *    takes 80 bits) and re-times the rest (fresh pulses, 0.3 us later). A WS2915 sends the header again in front of
 *    the forwarded data, so its output is one contiguous frame (datasheet: every chip shortens the stream by 80 bits).
 *    It latches on a low >= its reset time (datasheet: RES >= 280 us); '1' = high >= 450 ns.
 *  - RS-485 hop (Diff-Power sender -> receiver, receiver -> receiver): GM3088 driver + receiver, a delay and a pulse
 *    width change (skew, up to +-10 ns each), plus optional random jitter per bit.
 * Every bit keeps its own rise and high time, so hops and gates act on each pulse.
 * Positions: every device takes the next slot of what it sees. A receiver's output feeds the next receiver (RS-485)
 * and its local chain (single-ended J101) alike, so the local chain mirrors the next receiver's positions.
 * Long chains: up to 64 dig2analog-ultra boards board to board on one data pin, every hop
 * with a pulse-width change (DIN divider + AND gate + output into the cable), at the hub's default and centred
 * timing. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/frame_rx.h"
#include "../src/core/frame_track.h"

static int failures;
#define CHECK(c, ...) do { if (!(c)) { failures++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define MAXD    64
#define MAXW    96
#define MF      216                     /* host frames, at most (a 50-board preamble: 202) */
#define MFN     72                      /* host frames per run in the short-chain scenarios */
#define MAXPOS  64                      /* slots per host frame */
#define TICK    50.0
#define TAIL_US (576.0 * 16.0 / 36.0)
#define GAP_US  50.0

typedef struct { double rise; float hi; int16_t frame; uint16_t idx; } mbit_t;   /* frame -1: not a host frame */
typedef struct { mbit_t *b; int n, cap; } mwire_t;
static mwire_t mw[MAXW];
static int nw;

static unsigned long long rs = 0x9E3779B97F4A7C15ULL, ds = 0x2545F4914F6CDD1DULL;
static double ur(unsigned long long *s, double a, double b)
{
    *s ^= *s << 13; *s ^= *s >> 7; *s ^= *s << 17;
    return a + (b - a) * (double)(*s >> 11) / 9007199254740992.0;
}

static int mw_new(void)
{
    mwire_t *w = &mw[nw];
    w->n = 0;
    if (!w->b) { w->cap = 1 << 18; w->b = (mbit_t *)malloc(sizeof(mbit_t) * (size_t)w->cap); }
    return nw++;
}
static void mw_push(int w, const mbit_t *b)
{
    if (mw[w].n == mw[w].cap) {                      /* long chains with their preamble: grow */
        mbit_t *nb = (mbit_t *)realloc(mw[w].b, sizeof(mbit_t) * (size_t)mw[w].cap * 2u);
        if (!nb) return;
        mw[w].b = nb;
        mw[w].cap *= 2;
    }
    mw[w].b[mw[w].n++] = *b;
}
static double mfall(const mbit_t *b) { return b->rise + b->hi; }

/* ---------------------------------------------------------------------------------------------- host */
enum { HK_ULTRA = 0, HK_WS2805 = 1, HK_WS2915 = 2 };
static int hkind, hslots, hframes, hpre;
static float hub_t0h = 0.34f;                   /* the hub's '0' high time (us) */
static float hub_t1h = 0.70f;                   /* ... and '1' */
static uint8_t hbytes[MF][32 * MAXPOS + 8];
static int hbits[MF], hrw[MF];
static double hend[MF], hwin[MF];               /* hwin: end of the response window after an RW frame (0 = none) */

static void ws2805_bytes(int k, int f, uint8_t *o)
{
    o[0] = (uint8_t)(0x10 + k); o[1] = (uint8_t)(f & 0x7F); o[2] = 0x55; o[3] = (uint8_t)(0xAA ^ k); o[4] = (uint8_t)((f >> 7) & 0x7F);
}
static void ws2915_bytes(int k, int f, uint8_t *o)
{
    static const uint8_t tail[5] = { 0x10, 0x20, 0x30, 0x40, 0x50 };
    int i;
    o[0] = (uint8_t)(0x20 + k); o[1] = (uint8_t)(f & 0x7F); o[2] = 0x33; o[3] = (uint8_t)(0x44 ^ k); o[4] = (uint8_t)((f >> 7) & 0x7F);
    for (i = 0; i < 5; i++) o[5 + i] = (uint8_t)(tail[i] ^ (i == 4 ? k : 0));
}

/* host frames on wire 0: Ultra (DARK preamble below pre, RW frames with POLL slots every rw_every), or WS frames;
 * period 0: back to back (300 us), else frame starts every period. RW frames are followed by their window. */
static void host_build(int kind, int slots, int nframes, double start, double period, int pre, int rw_every)
{
    static const uint16_t one[5] = { 65535, 65535, 65535, 65535, 65535 };
    int f, i, k, w;
    double t = start;
    nw = 0;
    w = mw_new();                                    /* wire 0: the host's */
    hkind = kind; hslots = slots; hframes = nframes; hpre = pre;
    for (f = 0; f < nframes; f++) {
        uint8_t *fr = hbytes[f];
        int n = 0, rw = kind == HK_ULTRA && rw_every && f >= pre && (f - pre) % rw_every == rw_every - 1;
        double t0 = period > 0 ? start + f * period : t, tt;
        hrw[f] = rw;
        hwin[f] = 0;
        if (kind == HK_WS2805) {
            for (k = 0; k < slots; k++, n += 5) ws2805_bytes(k, f, fr + n);
        } else if (kind == HK_WS2915) {
            fr[0] = 0xFF; fr[1] = 0xFF; fr[2] = 0xFF; fr[3] = 0x80; n = 4;
            for (k = 0; k < slots; k++, n += 10) ws2915_bytes(k, f, fr + n);
        } else {
            for (k = 0; k < slots; k++) {
                uint8_t *s = fr + 32 * k;
                uint16_t a[5] = { 0, 0, 0, 0, 0 };
                if (f < pre) {
                    ultra_slot_dark(s);
                } else if (rw) {
                    ultra_slot_begin(s, ULTRA_POLL, ULTRA_LIGHT_BOARD, 0, (uint8_t)f);
                    s[ULTRA_PAYLOAD] = (uint8_t)k;
                    s[ULTRA_PAYLOAD + 1] = (uint8_t)slots;
                    ultra_slot_seal(s);
                } else {
                    a[0] = (uint16_t)f;
                    a[1] = (uint16_t)k;
                    ultra_slot_set(s, 0, (uint8_t)f, 0, (uint16_t)(1000 + k), a, one, 0, 0);
                }
            }
            n = 32 * slots + 8;
        }
        hbits[f] = n * 8;
        for (i = 0; i < 2; i++) {                    /* Ultra: twice, the second time with TIME at the frame end */
            int j, first = mw[w].n;
            tt = t0;
            for (j = 0; j < n * 8; j++) {
                mbit_t b;
                int v = (fr[j >> 3] >> (7 - (j & 7))) & 1;
                b.rise = tt; b.hi = v ? hub_t1h : hub_t0h; b.frame = (int16_t)f; b.idx = (uint16_t)j;
                mw_push(w, &b);
                tt += kind == HK_ULTRA ? (v ? 1.35 : 1.30) : 1.25;
            }
            hend[f] = mfall(&mw[w].b[mw[w].n - 1]);
            if (kind == HK_ULTRA && i == 0) {
                ultra_build_header_flags(fr + 32 * slots, (uint32_t)hend[f], rw ? ULTRA_HDR_RW : 0);
                mw[w].n = first;
                continue;
            }
            break;
        }
        t = hend[f] + 300.0;
        if (rw) {                                    /* the response window: 1000 + 450 R us, + 200 us hold-off */
            hwin[f] = hend[f] + ULTRA_RW_FIRST_US + ULTRA_RW_SLOT_US * slots;
            t = hwin[f] + ULTRA_RW_HOLDOFF_US;
        }
    }
}

/* the answers in the response windows as they appear on a receiver's DIN: per slot, the line goes HIGH, 32 bytes of
 * UART (1 Mbaud, 8N1, LSB first, HIGH = idle / 1), HIGH until ~400 us, then LOW. Merged into wire w, sorted. */
static void inject_answers(int w, int answers)
{
    mwire_t *x = &mw[w];
    int f, a, j;
    for (f = 0; f < hframes; f++) {
        if (!hwin[f]) continue;
        for (a = 0; a < answers; a++) {
            double s = hend[f] + ULTRA_RW_FIRST_US + ULTRA_RW_SLOT_US * a, t = s, hs = s;
            int high = 1;
            uint8_t cells[32 * 10];
            int nc = 0;
            for (j = 0; j < 32; j++) {
                int bit, byte = (int)ur(&rs, 0, 255.99);
                cells[nc++] = 0;                     /* start */
                for (bit = 0; bit < 8; bit++) cells[nc++] = (uint8_t)((byte >> bit) & 1);
                cells[nc++] = 1;                     /* stop */
            }
            t = s + 5.0;                             /* HIGH from s, first start bit 5 us later */
            for (j = 0; j < nc; j++, t += 1.0) {
                if (cells[j] != high) {
                    if (high) {                      /* HIGH segment ends */
                        mbit_t b;
                        b.rise = hs; b.hi = (float)(t - hs); b.frame = -1; b.idx = 0;
                        mw_push(w, &b);
                    } else {
                        hs = t;
                    }
                    high = cells[j];
                }
            }
            if (high) {                              /* stays HIGH until the release */
                mbit_t b;
                b.rise = hs; b.hi = (float)(s + 400.0 - hs); b.frame = -1; b.idx = 0;
                mw_push(w, &b);
            }
        }
    }
    /* sort by rise (insertion; the injected bits sit in windows where the wire was quiet) */
    for (j = 1; j < x->n; j++) {
        mbit_t v = x->b[j];
        int k = j - 1;
        while (k >= 0 && x->b[k].rise > v.rise) { x->b[k + 1] = x->b[k]; k--; }
        x->b[k + 1] = v;
    }
}

/* an RS-485 hop: delay, a pulse-width change (skew, ns) and random jitter (ns) per bit */
static int hop(int src, double skew_ns, double jitter_ns, double cable_m)
{
    int w = mw_new(), i;
    for (i = 0; i < mw[src].n; i++) {
        mbit_t b = mw[src].b[i];
        b.rise += 0.10 + 0.005 * cable_m;
        b.hi = (float)(b.hi + (skew_ns + ur(&rs, -jitter_ns, jitter_ns)) / 1000.0);
        if (b.hi < 0.03f) b.hi = 0.03f;
        mw_push(w, &b);
    }
    return w;
}

/* ---------------------------------------------------------------------------------------------- devices */
enum { D_MCU = 0, D_2805 = 1, D_2915 = 2 };

typedef struct {
    int kind;
    double ready;
    uint32_t frame, total, nbits, own, skip;
    proto_t proto;
    int valid, tail_ok;
    uint8_t bytes[PROTO_BITS_BUF], tail[8];
    double fe_tick;
} mjob_t;

typedef struct {
    int kind, in, out, pos, receiver;
    proto_mode_t mode;
    proto_t sticky;
    int conf;
    double t_samp, gate_ns, reset_us;          /* MCU: ns per sample; gate skew; chips: reset time */
    double jit_ns;                             /* MCU: random pulse-width change per forwarded bit, +- */
    double dec_ns;                             /* MCU: its own input's pulse-width offset (decoder only) */
    int win_closed;                            /* receiver: holds its gate closed in a response window */
    double boot_at;                            /* > 0: power-up time instead of a random one */
    /* MCU state */
    frx_t frx;
    ftrack_t ft;
    gate_plan_t plan, buf[2];
    int idx, rearmed, cap_skip, cnt_from, cnt_ptr, fwd_ptr, cap_first, njob;
    double closed_t, boot, phase, main_free, edge_t;
    uint32_t frames, cap_frame;
    proto_t cap_proto;
    uint16_t cap_own;
    mjob_t job[8];
    /* results */
    int first_f, shown, garbage, dark, fwd_junk;
    double sync_err;
} mdev_t;

static mdev_t md[MAXD];
static int nd;

static int decode_bit(const mdev_t *d, double hi_us)
{
    double ph = ur(&ds, 0, d->t_samp);
    return floor((hi_us * 1000.0 + d->dec_ns + ph) / d->t_samp) >= 17.0;
}

/* decode n bits from `from` as the MCU's decoder would; -1: a pulse too long (not a WS bit) */
static int mpack(const mdev_t *d, const mbit_t *in, int from, int n, uint8_t *out)
{
    int i;
    memset(out, 0, (size_t)(n + 7) / 8);
    for (i = 0; i < n; i++) {
        if (in[from + i].hi > 2.2f) return -1;
        if (decode_bit(d, in[from + i].hi)) out[i >> 3] |= (uint8_t)(0x80u >> (i & 7));
    }
    return 0;
}

static int has_gap(const mbit_t *in, int from, int to)
{
    int i;
    for (i = from + 1; i < to; i++) if (in[i].rise - mfall(&in[i - 1]) >= GAP_US) return 1;
    return 0;
}

static void show(mdev_t *d, int ok, int f)
{
    if (!ok) { d->garbage++; return; }
    if (d->first_f < 0) d->first_f = f;
    d->shown++;
}

static void show_ws(mdev_t *d, const slot_t *s, proto_t p)
{
    uint8_t e[10];
    int f = (s->raw[1] & 0x7F) | ((s->raw[4] & 0x7F) << 7), n = p == PROTO_WS2805 ? 5 : 10, i, zero = 1;
    for (i = 0; i < n; i++) zero &= s->raw[i] == 0;
    if (hkind == HK_ULTRA && zero) { d->dark++; return; }
    if (p == PROTO_WS2805) ws2805_bytes(d->pos, f, e); else ws2915_bytes(d->pos, f, e);
    show(d, f < hframes && s->raw_len == n && memcmp(s->raw, e, (size_t)n) == 0 && hkind != HK_ULTRA, f);
}

static void show_ultra(mdev_t *d, const uint8_t *s)
{
    if (s[0] == ULTRA_NOP) return;
    show(d, s[0] == ULTRA_SET && ultra_get16(s + ULTRA_PAYLOAD) == 1000u + (uint32_t)d->pos, (int)ultra_get16(s + ULTRA_PAYLOAD + 2));
}

static int gate_open(const gate_plan_t *g, int n)
{
    int open = g->start_open;
    if (g->t1 && n >= g->t1) open = !open;
    if (g->t2 && n >= g->t2) open = !open;
    return open;
}

static void mcu_job(mdev_t *d, mjob_t *j)
{
    if (j->kind == 0) {
        slot_t ws;
        if (j->valid && (frx_capture(&d->frx, j->frame, j->proto, j->bytes, j->nbits, j->own, j->skip, &ws) & FRX_SHOW_WS))
            show_ws(d, &ws, j->proto);
        return;
    }
    {
        frx_end_t e;
        frx_frame_end(&d->frx, j->frame, j->total, j->tail_ok ? j->tail : NULL, &e);
        if (e.show_ws) show_ws(d, &d->frx.ws, (proto_t)d->frx.ws_proto);
        if (e.close_gate) d->closed_t = j->ready;
        if (e.ultra_sync) {
            double err = fabs(j->fe_tick - (double)e.host_us);
            if (err > d->sync_err) d->sync_err = err;
            if (e.ultra_slot) show_ultra(d, d->frx.slot);
        }
        d->idx ^= 1;
        d->buf[d->idx] = e.plan;
    }
}

static void mcu_loop(mdev_t *d, double t)
{
    while (d->njob) {
        mjob_t *j = &d->job[0];
        double start = j->ready > d->main_free ? j->ready : d->main_free;
        double done = start + (j->kind == 0 ? 20.0 + 0.74 * j->nbits : 10.0 + (j->tail_ok ? 120.0 : 0.0));
        if (done > t) return;
        d->main_free = done;
        mcu_job(d, j);
        memmove(&d->job[0], &d->job[1], sizeof(mjob_t) * (size_t)(--d->njob));
    }
}

static int in_window(double t)
{
    int f;
    for (f = 0; f < hframes; f++) if (hwin[f] && t > hend[f] && t < hwin[f]) return 1;
    return 0;
}

static void sim_mcu(mdev_t *d)
{
    const mbit_t *in = mw[d->in].b;
    int n = mw[d->in].n;
    double t, end = (n ? mfall(&in[n - 1]) : 0) + 2000.0;
    d->boot = d->boot_at > 0 ? d->boot_at : 24000.0 + ur(&rs, 0, 6000.0);
    d->phase = ur(&rs, 0, TICK);
    frx_init(&d->frx, d->mode, d->sticky);
    if (d->conf) d->frx.ps.provisional = 0;
    d->plan = d->buf[0] = d->buf[1] = proto_gate_plan(proto_active(&d->frx.ps));
    d->closed_t = -1;
    d->first_f = -1;
    while (d->cnt_ptr < n && mfall(&in[d->cnt_ptr]) <= d->boot) d->cnt_ptr++;
    d->cnt_from = d->cnt_ptr;
    while (d->fwd_ptr < n && in[d->fwd_ptr].rise <= d->boot) d->fwd_ptr++;
    for (t = d->boot + d->phase; t < end; t += TICK) {
        uint32_t ev;
        uint16_t c, own;
        mcu_loop(d, t);
        for (; d->fwd_ptr < n && in[d->fwd_ptr].rise <= t; d->fwd_ptr++) {
            const mbit_t *w = &in[d->fwd_ptr];
            int cnt = d->fwd_ptr - d->cnt_from;
            if (d->rearmed && cnt >= 0 && !(d->closed_t >= 0 && w->rise > d->closed_t) && gate_open(&d->plan, cnt) &&
                !(d->receiver && d->win_closed && in_window(w->rise))) {
                mbit_t o = *w;
                o.rise += 0.01;
                o.hi = (float)(o.hi + (d->gate_ns + (d->jit_ns != 0 ? ur(&ds, -d->jit_ns, d->jit_ns) : 0)) / 1000.0);
                if (o.hi < 0.02f) o.hi = 0.02f;
                if (o.frame < 0) d->fwd_junk++;
                mw_push(d->out, &o);
            }
        }
        while (d->cnt_ptr < n && mfall(&in[d->cnt_ptr]) <= t) d->cnt_ptr++;
        c = (uint16_t)(d->cnt_ptr - d->cnt_from);
        own = d->plan.own_bits;
        if (d->frx.ps.provisional && own < PROTO_DETECT_BITS) own = PROTO_DETECT_BITS;
        ev = ftrack_tick(&d->ft, c, own, d->plan.hdr_bits);
        if (ev & FT_EDGE) d->edge_t = t;
        if (ev & FT_CAP_START) {
            if (ev & FT_CAP_DATA) {
                d->cap_skip = d->plan.hdr_bits;
            } else {
                d->cap_skip = 0;
                d->cap_frame = d->frames;
                d->cap_proto = proto_active(&d->frx.ps);
                d->cap_own = d->plan.own_bits;
            }
            d->cap_first = d->cnt_from + d->cap_skip;
        }
        if ((ev & (FT_CAP_COPY | FT_CAP_SHORT)) && d->njob < 8) {
            mjob_t *j = &d->job[d->njob++];
            int want = d->ft.cap_bits - d->cap_skip;
            j->kind = 0; j->ready = t; j->frame = d->cap_frame; j->proto = d->cap_proto; j->own = d->cap_own;
            j->skip = (uint32_t)d->cap_skip;
            j->nbits = (uint32_t)(want > 0 ? want : 0);
            j->valid = want > 0 && d->cap_first + want <= d->cnt_ptr && !has_gap(in, d->cap_first, d->cap_first + want) &&
                       (d->cap_first == 0 || in[d->cap_first].rise - mfall(&in[d->cap_first - 1]) >= 60.0 ||
                        d->cap_first == d->cnt_from);
            if (j->valid && mpack(d, in, d->cap_first, want, j->bytes) != 0) j->valid = 0;
        }
        if (ev & (FT_FRAME_END | FT_BOOT_ARM)) {
            if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END && d->njob < 8) {
                int from = d->cnt_from, to = d->cnt_ptr, w0 = to, nb = to - from;
                mjob_t *j = &d->job[d->njob++];
                j->kind = 1; j->ready = t; j->frame = d->frames; j->total = (uint32_t)nb; j->fe_tick = d->edge_t;
                j->tail_ok = 0;
                if (frx_tail_wanted(&d->frx, (uint32_t)nb)) {
                    double ws = d->edge_t + 1.8 - TAIL_US;
                    while (w0 > from && in[w0 - 1].rise >= ws) w0--;
                    if (to - w0 >= 64 && !has_gap(in, w0, to) && mpack(d, in, to - 64, 64, j->tail) == 0) j->tail_ok = 1;
                }
            }
            d->plan = d->buf[d->idx];
            d->rearmed = 1;
            d->closed_t = -1;
            d->cnt_from = d->cnt_ptr;
            if (ev & FT_FRAME_END) d->frames++;
        }
    }
}

/* a chip latches on a reset: its slot, compared with the host's for its position */
static void chip_latch(mdev_t *d, const uint8_t *data)
{
    uint8_t e[10];
    int f;
    if (d->kind == D_2805) {
        f = (data[1] & 0x7F) | ((data[4] & 0x7F) << 7);
        ws2805_bytes(d->pos, f, e);
        show(d, f < hframes && memcmp(data, e, 5) == 0, f);
    } else {
        f = (data[5] & 0x7F) | ((data[8] & 0x7F) << 7);
        ws2915_bytes(d->pos, f, e);
        show(d, f < hframes && data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0x80 &&
                memcmp(data + 4, e, 10) == 0, f);
    }
}

static void sim_chip(mdev_t *d)
{
    const mwire_t *in = &mw[d->in];
    int i, nb = 0, hlen = d->kind == D_2915 ? 32 : 0, need = hlen + (d->kind == D_2915 ? 80 : 40);
    uint8_t data[16];
    double last_fall = -1e9;
    d->first_f = -1;
    memset(data, 0, sizeof data);
    for (i = 0; i <= in->n; i++) {
        const mbit_t *b = i < in->n ? &in->b[i] : NULL;
        int v;
        if (!b || b->rise - last_fall >= d->reset_us) {
            if (nb >= need) chip_latch(d, data);
            nb = 0;
            memset(data, 0, sizeof data);
            if (!b) break;
        }
        v = b->hi >= 0.45f;
        if (nb < need) {
            if (v) data[nb >> 3] |= (uint8_t)(0x80u >> (nb & 7));
            if (d->kind == D_2915 && nb >= need - hlen) {   /* the header again, right before the forwarded data */
                int h = nb - (need - hlen);
                mbit_t o = *b;
                o.rise += 0.3;
                o.hi = (data[h >> 3] & (0x80u >> (h & 7))) ? 0.60f : 0.30f;
                o.idx = (uint16_t)h;
                mw_push(d->out, &o);
            }
        } else {
            mbit_t o = *b;
            o.rise += 0.3;
            o.hi = v ? 0.60f : 0.30f;                       /* re-timed */
            mw_push(d->out, &o);
        }
        nb++;
        last_fall = mfall(b);
    }
}

/* ---------------------------------------------------------------------------------------------- networks */
typedef struct {
    int rx;                       /* receivers in the RS-485 chain */
    int local[16];                /* devices behind each receiver's J101 (a branch: they mirror the next receiver) */
    int local_kind;               /* D_MCU / D_2805 / D_2915 */
    double skew_ns, jitter_ns, gate_ns, cable_m, chip_reset_us;
    int skew_mode;                /* 0 every hop +skew, 1 every hop -skew, 2 random in +-skew */
    proto_mode_t mode;            /* every MCU */
    proto_t sticky;
    int conf;
    double hsi;                   /* MCU sample clock error (0: random per device in +-2.2 %) */
    int win_closed, answers;
} mnet_t;

typedef struct { int devices, missing, garbage, junk_fwd, late; double sync; int worst_first; } mres_t;

static mdev_t *dev_new(int kind, int in, int pos, const mnet_t *n)
{
    mdev_t *d = &md[nd++];
    memset(d, 0, sizeof *d);
    d->kind = kind; d->in = in; d->out = mw_new(); d->pos = pos;
    d->mode = n->mode; d->sticky = n->sticky; d->conf = n->conf;
    d->t_samp = 1e3 / 36.0 * (1.0 + (n->hsi != 0 ? n->hsi : ur(&rs, -0.022, 0.022)));
    d->gate_ns = n->gate_ns;
    d->reset_us = n->chip_reset_us;
    d->win_closed = n->win_closed;
    return d;
}

static void run_dev(mdev_t *d)
{
    if (d->kind == D_MCU) sim_mcu(d); else sim_chip(d);
}

/* hub -> hop -> R0 -> hop -> R1 ...; Rk's J101 chain has local[k] devices (positions k + 1, k + 2, ...) */
static mres_t run_net(const mnet_t *n)
{
    mres_t r;
    int k, j, src = 0, prev_out = -1;
    memset(&r, 0, sizeof r);
    r.worst_first = -1;
    nd = 0;
    nw = 1;                                   /* wire 0: the host's */
    for (k = 0; k < n->rx; k++) {
        double s = n->skew_mode == 2 ? ur(&rs, -n->skew_ns, n->skew_ns) : n->skew_mode == 1 ? -n->skew_ns : n->skew_ns;
        int in = hop(k == 0 ? src : prev_out, s, n->jitter_ns, n->cable_m);
        mdev_t *rx;
        if (n->answers) inject_answers(in, n->answers);
        rx = dev_new(D_MCU, in, k, n);
        rx->receiver = 1;
        run_dev(rx);
        prev_out = rx->out;
        for (j = 0, src = rx->out; j < n->local[k]; j++) {
            mdev_t *d = dev_new(n->local_kind, src, k + 1 + j, n);
            run_dev(d);
            src = d->out;
        }
        src = 0;
    }
    for (k = 0; k < nd; k++) {
        mdev_t *d = &md[k];
        r.devices++;
        r.garbage += d->garbage;
        r.junk_fwd += d->fwd_junk;
        if (d->first_f < 0) r.missing++;
        else if (d->first_f > r.worst_first) r.worst_first = d->first_f;
        if (d->sync_err > r.sync) r.sync = d->sync_err;
    }
    return r;
}

static int max_pos(const mnet_t *n)
{
    int k, m = n->rx - 1;
    for (k = 0; k < n->rx; k++) if (k + n->local[k] > m) m = k + n->local[k];
    return m + 1;
}

static mnet_t net_default(void)
{
    mnet_t n;
    memset(&n, 0, sizeof n);
    n.gate_ns = 3.0;
    n.cable_m = 20.0;
    n.chip_reset_us = 280.0;
    n.mode = PMODE_AUTO;
    return n;
}

/* ---------------------------------------------------------------------------------------------- long chains */
/* N dig2analog-ultra boards on one data pin, board to board (single-ended, no RS-485): the hub's output (one hop),
 * then board 0, board 1, ... Every hop changes the pulse width by sys_ns (the same on every hop: one batch of parts,
 * one cable type) + a fixed part per board in +-rnd_ns (board to board) + per bit +-jit_ns; each board's decoder
 * reads its own input with a fixed +-dec_ns offset. Boards: mode / stored protocol as given; fresh_at >= 0: these
 * positions have nothing stored (fresh, auto). Two wires alternate (a board's input and output). Returns the boards
 * from the hub that showed every command frame, i.e. the index of the first that missed one (or N). */
typedef struct {
    double sys_ns, rnd_ns, jit_ns, dec_ns;
    proto_mode_t mode;
    proto_t sticky;
    int conf;
    int fresh_at[4];              /* -1 = none */
} mchain_t;

static mchain_t chain_default(void)
{
    mchain_t c;
    memset(&c, 0, sizeof c);
    c.mode = PMODE_ULTRA;
    c.fresh_at[0] = c.fresh_at[1] = c.fresh_at[2] = c.fresh_at[3] = -1;
    return c;
}

static int run_chain(int boards, const mchain_t *c, int frames, int pre, double start_us, int *worst_first)
{
    int k, a = 0, b, i;
    double hub = c->sys_ns + ur(&rs, -c->rnd_ns, c->rnd_ns);
    host_build(HK_ULTRA, boards, frames, start_us, 0, pre, 0);
    for (i = 0; i < mw[0].n; i++) {                  /* the hub's own output stage and the first cable, in place */
        mbit_t *x = &mw[0].b[i];
        x->rise += 0.02;
        x->hi = (float)(x->hi + (hub + (c->jit_ns != 0 ? ur(&ds, -c->jit_ns, c->jit_ns) : 0)) / 1000.0);
    }
    b = mw_new();                                    /* wires 1 and 2 alternate */
    (void)mw_new();
    if (worst_first) *worst_first = -1;
    for (k = 0; k < boards; k++) {
        mdev_t *d = &md[0];
        int fresh = 0, t;
        for (t = 0; t < 4; t++) fresh |= c->fresh_at[t] == k;
        memset(d, 0, sizeof *d);
        d->kind = D_MCU; d->in = a; d->out = b; d->pos = k;
        d->mode = fresh ? PMODE_AUTO : c->mode;
        d->sticky = fresh ? PROTO_NONE : c->sticky;
        d->conf = fresh ? 0 : c->conf;
        d->t_samp = 1e3 / 36.0 * (1.0 + ur(&rs, -0.022, 0.022));
        d->gate_ns = c->sys_ns + ur(&rs, -c->rnd_ns, c->rnd_ns);
        d->jit_ns = c->jit_ns;
        d->dec_ns = ur(&rs, -c->dec_ns, c->dec_ns);
        mw[b].n = 0;
        sim_mcu(d);
        if (d->garbage || d->shown != frames - pre) return k;
        if (worst_first && d->first_f > *worst_first) *worst_first = d->first_f;
        a = b;
        b = a == 1 ? 2 : 1;
    }
    return boards;
}

/* ---------------------------------------------------------------------------------------------- scenarios */
int main(int argc, char **argv)
{
    mnet_t n;
    mres_t r;
    int rx, l, sm, bad, k;
    static const char *const smn[3] = { "+", "-", "+-random" };
    (void)argc; (void)argv;
    printf("Diff-Power mixed chains (MCU receivers, RS-485 hops, ultra / chip boards behind)\n\n");

    /* 1a. Ultra, receiver -> 1-10 boards, and 1-10 chained receivers with 0-3 boards each; fresh auto, preamble */
    printf("1a. Ultra line, fresh auto + DARK preamble, 5 ns per hop (+, -, random; typical GM3088), gate 3 ns\n");
    bad = 0;
    for (sm = 0; sm < 3; sm++) {
        for (l = 1; l <= 10; l++) {             /* one receiver, l boards */
            n = net_default();
            n.rx = 1; n.local[0] = l; n.local_kind = D_MCU; n.skew_ns = 5; n.skew_mode = sm;
            host_build(HK_ULTRA, max_pos(&n), MFN, 200000.0, 0, 4 * max_pos(&n) + 2, 0);
            r = run_net(&n);
            if (r.garbage || r.missing || r.worst_first != 4 * max_pos(&n) + 2 || r.sync > 60) {
                bad++;
                printf("  FAIL detail: 1 receiver + %d boards, skew %s: garbage %d missing %d first %d sync %.0f\n", l,
                       smn[sm], r.garbage, r.missing, r.worst_first, r.sync);
            }
        }
        for (rx = 2; rx <= 10; rx++) {
            for (l = 0; l <= 3; l++) {
                n = net_default();
                n.rx = rx; n.local_kind = D_MCU; n.skew_ns = 5; n.skew_mode = sm;
                for (k = 0; k < rx; k++) n.local[k] = l;
                host_build(HK_ULTRA, max_pos(&n), MFN, 200000.0, 0, 4 * max_pos(&n) + 2, 0);
                r = run_net(&n);
                if (r.garbage || r.missing || r.worst_first != 4 * max_pos(&n) + 2 || r.sync > 60) {
                    bad++;
                    printf("  FAIL detail: %d receivers x %d boards, skew %s: garbage %d missing %d first %d sync %.0f\n",
                           rx, l, smn[sm], r.garbage, r.missing, r.worst_first, r.sync);
                }
            }
        }
    }
    CHECK(bad == 0, "1a: every device its slot from the first command, clock sync (%d)", bad);
    printf("  receiver + 1-10 boards and 2-10 receivers x 0-3 boards, 3 skew patterns: %s\n\n", bad ? "FAILURES above" : "all ok");

    /* stale stored protocols and the hub already running (no preamble) */
    printf("1a. Ultra line running (no preamble), every MCU with a stale stored WS2805 / WS2915 protocol\n");
    {
        static const proto_t st[2] = { PROTO_WS2805, PROTO_WS2915_32 };
        int s2;
        bad = 0;
        for (s2 = 0; s2 < 2; s2++) {
            for (rx = 1; rx <= 6; rx++) {
                n = net_default();
                n.rx = rx; n.local_kind = D_MCU; n.skew_ns = 10; n.skew_mode = 2; n.sticky = st[s2];
                for (k = 0; k < rx; k++) n.local[k] = 2;
                host_build(HK_ULTRA, max_pos(&n), MFN, ur(&rs, 0, 30000.0), 0, 0, 0);
                r = run_net(&n);
                if (r.garbage || r.missing) {
                    bad++;
                    printf("  FAIL detail: %d receivers x 2 boards, stale %d: garbage %d missing %d\n", rx, st[s2], r.garbage,
                           r.missing);
                }
            }
        }
        CHECK(bad == 0, "1a stale: nothing shown, all in Ultra (%d)", bad);
        printf("  1-6 receivers x 2 boards, stale WS2805 / WS2915: %s\n\n", bad ? "FAILURES above" : "nothing shown, every device in Ultra");
    }

    /* the skew limit: receivers only, forced Ultra, the same skew on every hop; random HSI and sampling phase */
    {
    int t0;
    for (t0 = 0; t0 < 2; t0++) {
    hub_t0h = t0 ? 0.25f : 0.34f;
    printf("Skew limit, Ultra line (hub '0' %.0f ns, '1' 700 ns): receivers in a chain until a device misdecodes\n",
           hub_t0h * 1000.0);
    printf("  skew per hop (ns)  |  all +  |  all -  | +-random (worst of 10 runs)\n");
    {
        static const double sk[] = { 2, 5, 10, 15, 20, 25, 30 };
        int si;
        for (si = 0; si < 7; si++) {
            int lim[3];
            for (sm = 0; sm < 3; sm++) {
                int runs = sm == 2 ? 10 : 3, run, worst = 99;
                for (run = 0; run < runs; run++) {
                    int ok_up_to = 0;
                    for (rx = 1; rx <= 16; rx++) {
                        n = net_default();
                        n.rx = rx; n.skew_ns = sk[si]; n.skew_mode = sm; n.mode = PMODE_ULTRA;
                        host_build(HK_ULTRA, rx, 16, 40000.0, 0, 0, 0);
                        r = run_net(&n);
                        if (r.garbage || r.missing) break;
                        ok_up_to = rx;
                    }
                    if (ok_up_to < worst) worst = ok_up_to;
                }
                lim[sm] = worst;
            }
            printf("  %5.0f              |  %3d%s  |  %3d%s  |  %3d%s\n", sk[si], lim[0], lim[0] >= 16 ? "+" : " ",
                   lim[1], lim[1] >= 16 ? "+" : " ", lim[2], lim[2] >= 16 ? "+" : " ");
        }
        printf("  (receivers that still decode every bit; '16+' = all 16 tried; gate +3 ns per receiver)\n\n");
    }
    }
    hub_t0h = 0.34f;
    }

    /* long chains of dig2analog-ultra boards on one data pin */
    printf("Long chains, board to board on one data pin (single-ended), 36 MHz decoder with HSI +-2.2 %%\n");
    {
        mchain_t c;
        int got, wf;
        /* typical parts: +1 ns per hop (AHCT threshold a little under the divided swing's middle), +-1.5 ns per
         * board, +-1 ns per bit, +-2 ns at each decoder */
        c = chain_default();
        c.sys_ns = 1.0; c.rnd_ns = 1.5; c.jit_ns = 1.0; c.dec_ns = 2.0;
        got = run_chain(50, &c, 10, 0, 40000.0, NULL);
        CHECK(got == 50, "50 boards, forced Ultra, typical distortion: %d boards decode every frame", got);
        printf("  50 boards, forced Ultra, typical distortion (+1 ns per hop): %d of 50 boards decode every frame\n", got);
        c.mode = PMODE_AUTO; c.sticky = PROTO_NONE;   /* fresh boards (nothing stored), the full preamble */
        got = run_chain(50, &c, 4 * 50 + 2 + 6, 4 * 50 + 2, 200000.0, &wf);
        CHECK(got == 50 && wf == 4 * 50 + 2, "50 fresh boards, preamble 202: %d boards, first command frame %d", got, wf);
        printf("  50 fresh boards (auto, nothing stored), preamble 4N + 2 = 202 DARK frames: %d of 50 from the first "
               "command (frame %d)\n", got, wf);
        c.sticky = PROTO_ULTRA;                       /* a power-up: every board stored Ultra (provisional) */
        got = run_chain(50, &c, 10 + 6, 10, 40000.0, &wf);
        CHECK(got == 50 && wf == 10, "power-up, stored Ultra, short preamble 10: %d boards, first %d", got, wf);
        printf("  power-up, every board stored Ultra, short preamble of 10 frames: %d of 50 from the first command\n", got);
        c.fresh_at[0] = 12; c.fresh_at[1] = 37;     /* two new boards in the chain since the last full preamble */
        got = run_chain(50, &c, 10 + 6, 10, 40000.0, &wf);
        CHECK(got == 50 && wf == 10, "power-up with 2 fresh boards, short preamble 10: %d boards, first %d", got, wf);
        printf("  the same with 2 fresh boards (positions 12, 37): %d of 50 from the first command\n", got);
        c.fresh_at[2] = 3; c.fresh_at[3] = 25;
        got = run_chain(50, &c, 10 + 30, 10, 40000.0, &wf);
        printf("  ... with 4 fresh boards: %d of 50 show every command frame (the first commands miss the boards behind "
               "the 3rd fresh one; the refresh brings them)\n", got);
    }
    {
        static const double sys[] = { -8, -6, -4, -3, -2, -1, 0, 1, 2, 3, 4, 6, 8 };
        static const float t0[3] = { 0.34f, 0.25f, 0.25f }, t1[3] = { 0.70f, 0.70f, 0.75f };
        int si, ti;
        printf("  pulse-width change per hop (the same on every hop, +-1.5 ns per board, +-1 ns per bit) vs the boards "
               "that decode every frame (worst of 3 chains, up to 64):\n");
        printf("  per hop (ns) |  hub 340 / 700 ns  |  hub 250 / 700 ns  |  hub 250 / 750 ns\n");
        for (si = 0; si < 13; si++) {
            int lim[3];
            for (ti = 0; ti < 3; ti++) {
                int run, worst = 99;
                hub_t0h = t0[ti];
                hub_t1h = t1[ti];
                for (run = 0; run < 3; run++) {
                    mchain_t c = chain_default();
                    int got;
                    c.sys_ns = sys[si]; c.rnd_ns = 1.5; c.jit_ns = 1.0; c.dec_ns = 2.0;
                    got = run_chain(64, &c, 6, 0, 40000.0, NULL);
                    if (got < worst) worst = got;
                }
                lim[ti] = worst;
            }
            printf("  %+5.0f        |  %3d%s              |  %3d%s              |  %3d%s\n", sys[si], lim[0],
                   lim[0] >= 64 ? "+" : " ", lim[1], lim[1] >= 64 ? "+" : " ", lim[2], lim[2] >= 64 ? "+" : " ");
            if (sys[si] >= -3 && sys[si] <= 3) {
                CHECK(lim[1] >= 50, "hub 250 / 700 ns, %+.0f ns per hop: %d boards (want 50)", sys[si], lim[1]);
            }
        }
        hub_t0h = 0.34f;
        hub_t1h = 0.70f;
        printf("  ('64+' = all 64 tried)\n\n");
    }

    /* the response window (reserved): RW frames with POLL slots, answers on every receiver's DIN */
    printf("Response window (reserved): RW frames with POLL slots every 4th frame, 3 receivers x 2 boards, answers on DIN\n");
    for (k = 0; k < 2; k++) {
        n = net_default();
        n.rx = 3; n.local_kind = D_MCU; n.skew_ns = 10; n.skew_mode = 2; n.answers = 3; n.win_closed = k;
        n.local[0] = n.local[1] = n.local[2] = 2;
        host_build(HK_ULTRA, max_pos(&n), MFN, 200000.0, 0, 4 * max_pos(&n) + 2, 4);
        r = run_net(&n);
        CHECK(r.garbage == 0 && r.missing == 0, "window, gate %s: no garbage (%d), all served (%d)", k ? "held closed" : "as today",
              r.garbage, r.missing);
        printf("  gate %-12s: garbage %d, missing %d, answer pulses forwarded to the boards behind: %d, sync within %.0f us\n",
               k ? "held closed" : "as today", r.garbage, r.missing, r.junk_fwd, r.sync);
    }
    printf("\n");

    /* 1b. WS line: receiver (WS2805 / WS2915 mode) -> chip boards, and chained receivers with chips behind */
    printf("1b. WS line at 50 fps, MCU receivers in WS mode, chip boards behind (re-timing)\n");
    {
        int kind, stored;
        for (kind = 0; kind < 2; kind++) {
            int hk = kind ? HK_WS2915 : HK_WS2805, dk = kind ? D_2915 : D_2805;
            proto_mode_t pm = kind ? PMODE_WS2915 : PMODE_WS2805;
            bad = 0;
            for (l = 1; l <= 10; l++) {
                n = net_default();
                n.rx = 1; n.local[0] = l; n.local_kind = dk; n.skew_ns = 10; n.skew_mode = 2; n.mode = pm;
                host_build(hk, max_pos(&n), MFN, 3000.0, 20000.0, 0, 0);
                r = run_net(&n);
                if (r.garbage || r.missing) {
                    bad++;
                    printf("  FAIL detail: receiver + %d %s chips: garbage %d missing %d\n", l, kind ? "WS2915" : "WS2805",
                           r.garbage, r.missing);
                }
            }
            CHECK(bad == 0, "1b: receiver (%s) + 1-10 chips (%d)", kind ? "WS2915" : "WS2805", bad);
            printf("  receiver (forced %s) + 1-10 %s chips: %s\n", kind ? "WS2915" : "WS2805", kind ? "WS2915" : "WS2805",
                   bad ? "FAILURES above" : "every chip latches its own data every frame");
            /* chained receivers, 2 chips behind each: the WS2915 gap after the header grows by ~100 us per receiver */
            for (k = 0; k < 2; k++) {
                double rst = k ? 150.0 : 280.0;
                int last_ok = 0;
                for (rx = 1; rx <= 8; rx++) {
                    n = net_default();
                    n.rx = rx; n.local_kind = dk; n.skew_ns = 10; n.skew_mode = 2; n.mode = pm; n.chip_reset_us = rst;
                    for (stored = 0; stored < rx; stored++) n.local[stored] = 2;
                    host_build(hk, max_pos(&n), MFN, 3000.0, 20000.0, 0, 0);
                    r = run_net(&n);
                    if (r.garbage || r.missing) break;
                    last_ok = rx;
                }
                printf("  %d-8 receivers x 2 %s chips (chip reset %3.0f us): ok up to %d receivers\n", 1,
                       kind ? "WS2915" : "WS2805", rst, last_ok);
            }
        }
        /* MCU boards (dig2analog-ultra in WS2915 mode) behind chained receivers: they hold the frame across the gap */
        {
            int last_ok = 0;
            for (rx = 1; rx <= 8; rx++) {
                n = net_default();
                n.rx = rx; n.local_kind = D_MCU; n.skew_ns = 5; n.skew_mode = 2; n.mode = PMODE_WS2915;
                for (k = 0; k < rx; k++) n.local[k] = 2;
                host_build(HK_WS2915, max_pos(&n), MFN, 3000.0, 20000.0, 0, 0);
                r = run_net(&n);
                if (r.garbage || r.missing) break;
                last_ok = rx;
            }
            CHECK(last_ok == 8, "WS2915, MCU boards behind receivers: ok up to %d receivers", last_ok);
            printf("  1-8 receivers x 2 MCU boards (WS2915 mode): ok up to %d receivers (FT_HOLD bridges the gap)\n", last_ok);
        }
        /* start-up: receivers power up while the WS controller runs (chips keep their outputs); stored protocols */
        printf("  start-up, WS2805 line running, receiver + 4 (8) WS2805 chips, receiver's stored protocol:\n");
        {
            static const struct { proto_mode_t m; proto_t s; int conf; const char *name; } cs[] = {
                { PMODE_WS2805, PROTO_NONE, 0, "forced WS2805" }, { PMODE_AUTO, PROTO_WS2805, 0, "auto, stored WS2805" },
                { PMODE_AUTO, PROTO_NONE, 0, "auto, nothing stored" }, { PMODE_AUTO, PROTO_ULTRA, 0, "auto, stored Ultra (stale)" },
                { PMODE_AUTO, PROTO_WS2915_32, 0, "auto, stored WS2915 (stale)" },
                { PMODE_AUTO, PROTO_ULTRA, 0, "auto, stored Ultra, 8 chips" },
            };
            int c, run;
            for (c = 0; c < 6; c++) {
                int garb = 0, chip_garb = 0, late = 0;
                for (run = 0; run < 20; run++) {
                    n = net_default();
                    n.rx = 1; n.local[0] = c == 5 ? 8 : 4; n.local_kind = D_2805; n.skew_ns = 5; n.skew_mode = 2;
                    n.mode = cs[c].m; n.sticky = cs[c].s; n.conf = cs[c].conf;
                    host_build(HK_WS2805, max_pos(&n), MFN, ur(&rs, 0, 20000.0), 20000.0, 0, 0);
                    r = run_net(&n);
                    garb += md[0].garbage;
                    for (k = 1; k < nd; k++) chip_garb += md[k].garbage;
                    if (r.worst_first > 6) late++;
                }
                printf("    %-28s: receiver garbage %d, chip garbage %d (frames, 20 runs), late %d\n", cs[c].name, garb,
                       chip_garb, late);
            }
        }
    }
    printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL MIXED-CHAIN CHECKS PASSED", failures);
    return failures ? 1 : 0;
}
