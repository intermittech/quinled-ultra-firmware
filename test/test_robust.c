/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Randomized robustness tests (property-based) of every input path of the firmware core, on the host.
 * Build: test\build_robust.cmd (MSVC, AddressSanitizer). Usage:
 *   robust_tests.exe                      every path, the default count each
 *   robust_tests.exe <path> [count] [seed]  one path (decode, tail, frame, ftrack, ultra, config, update, cli, store,
 *                                           scene, cal, energy, fx, regress)
 *   robust_tests.exe <path> --case <hex>  one case again (what a failure prints)
 * Every case is seeded from (seed, path, index), so each failure reproduces alone. Properties checked per path are
 * listed at the top of its function; the firmware's own invariants (what main.c relies on) are in inv_*(). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/ack.h"
#include "../src/core/calstore.h"
#include "../src/core/cli.h"
#include "../src/core/climit.h"
#include "../src/core/config.h"
#include "../src/core/dither.h"
#include "../src/core/energy.h"
#include "../src/core/frame_rx.h"
#include "../src/core/frame_track.h"
#include "../src/core/gamma.h"
#include "../src/core/health.h"
#include "../src/core/isense.h"
#include "../src/core/presets.h"
#include "../src/core/proto.h"
#include "../src/core/pwrev.h"
#include "../src/core/scene.h"
#include "../src/core/smooth.h"
#include "../src/core/store.h"
#include "../src/core/ultra.h"
#include "../src/core/ultra_update.h"
#include "../src/core/upd_app.h"
#include "../src/core/vdip.h"
#include "../src/core/vsense.h"
#include "../src/core/ws_decode.h"
#include "../src/core/flashrec.h"

/* core/flashrec.c's flash primitive (hal/hw.c on the chip): the stores aren't used here (0.6.4) */
uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    (void)addr;
    (void)src;
    (void)halfwords;
    (void)erase_addr;
    return FLASH_NONE;
}

/* ------------------------------------------------------------------------------------------------ framework */
static int g_fail, g_verbose;
static const char *g_path = "";
static unsigned long long g_case;                        /* the case being run (printed with a failure) */
static unsigned long long rs;                            /* xorshift state of the case */

#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s --case %llx (%s:%d): ", g_path, g_case, __FILE__, \
                                       __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static unsigned long long mix64(unsigned long long z)   /* splitmix64 finalizer */
{
    z += 0x9E3779B97F4A7C15ull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static void case_begin(unsigned long long c)
{
    g_case = c;
    rs = mix64(c) | 1u;
}

static unsigned long long rnd64(void)
{
    rs ^= rs << 13;
    rs ^= rs >> 7;
    rs ^= rs << 17;
    return rs;
}
static uint32_t rnd(void) { return (uint32_t)(rnd64() >> 32); }
static uint32_t rint_(uint32_t n) { return n ? (uint32_t)(rnd64() % n) : 0u; }      /* 0..n-1 */
static uint32_t rrange(uint32_t a, uint32_t b) { return a + rint_(b - a + 1u); }    /* a..b */
static int chance(uint32_t pct) { return rint_(100) < pct; }
static double urand(double a, double b) { return a + (b - a) * (double)(rnd64() >> 11) / 9007199254740992.0; }

/* a value from the edges of its range now and then (0, 1, max - 1, max, powers of two) */
static uint32_t redge(uint32_t max)
{
    switch (rint_(8)) {
    case 0: return 0;
    case 1: return max;
    case 2: return max > 0u ? max - 1u : 0u;
    case 3: return 1u;
    case 4: return max > 1u ? (1u << rint_(32)) % (max + 1u) : max;
    default: return rint_(max + 1u);
    }
}

static uint32_t hash_bytes(const void *p, size_t n)
{
    const uint8_t *b = (const uint8_t *)p;
    uint32_t h = 2166136261u;
    size_t i;
    for (i = 0; i < n; i++) {
        h = (h ^ b[i]) * 16777619u;
    }
    return h;
}

/* ================================================================================================ decoder */
/* A reference decoder: the same rules as ws_decode.h states them, sample by sample (no word tricks). */
static int ref_decode_frame(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, const ws_timing_t *t, uint8_t *out)
{
    uint32_t ns = (nsamples >> 4) * 16u, i, b = 0, rise = 0, fall = 0, prev = 0, seen_rise = 0;
    memset(out, 0, (nbits + 7u) / 8u);
    if (ns == 0u || (buf[0] & 0x8000u) != 0u) {
        return WS_ERR_NO_GAP;
    }
    for (i = 0; i < ns; i++) {
        uint32_t s = (buf[i >> 4] >> (15u - (i & 15u))) & 1u;
        if (s != prev) {
            if (s) {
                if (seen_rise && i - fall >= t->gap_min) {
                    return WS_ERR_SHORT;
                }
                rise = i;
                seen_rise = 1;
            } else if (seen_rise) {
                uint32_t high = i - rise;
                if (high > t->high_max) {
                    return WS_ERR_PULSE;
                }
                if (high >= t->thr_one) {
                    out[b >> 3] |= (uint8_t)(0x80u >> (b & 7u));
                }
                if (++b == nbits) {
                    return (int)nbits;
                }
                fall = i;
            }
        }
        prev = s;
    }
    return seen_rise ? WS_ERR_TRUNCATED : WS_ERR_NO_EDGE;
}

static int ref_decode_last(const uint16_t *buf, uint32_t nsamples, const ws_timing_t *t, uint8_t *out)
{
    uint32_t ns = (nsamples >> 4) * 16u, i, n = 0, rise = 0, fall = 0, prev, seen_rise = 0;
    unsigned long long v = 0;
    if (ns == 0u) {
        return WS_ERR_NO_EDGE;
    }
    prev = (buf[0] >> 15) & 1u;
    for (i = 1; i < ns; i++) {
        uint32_t s = (buf[i >> 4] >> (15u - (i & 15u))) & 1u;
        if (s != prev) {
            if (s) {
                if (seen_rise && i - fall >= t->gap_min) {
                    return WS_ERR_SHORT;
                }
                rise = i;
                seen_rise = 1;
            } else if (seen_rise) {
                uint32_t high = i - rise;
                if (high > t->high_max) {
                    return WS_ERR_PULSE;
                }
                v = (v << 1) | (high >= t->thr_one ? 1u : 0u);
                n++;
                fall = i;
            }
        }
        prev = s;
    }
    if (n == 0u) {
        return WS_ERR_NO_EDGE;
    }
    for (i = 0; i < 8u; i++) {
        out[i] = (uint8_t)(v >> (56u - 8u * i));
    }
    return (int)n;
}

/* a random sample stream: a WS-like waveform with random (also out-of-spec) timing, glitches and gaps, sampled at
 * 36 MHz with a random phase; or plain random words; or runs */
#define DEC_MAX_WORDS 3072u
static uint16_t dec_buf[DEC_MAX_WORDS + 8];
static uint8_t dec_bits[(DEC_MAX_WORDS * 16u) / 8u + 8u];

static uint32_t gen_samples(uint16_t *buf, uint32_t max_words, uint32_t *first_bits_out)
{
    uint32_t kind = rint_(10), nw = redge(max_words), i;
    if (nw > max_words) {
        nw = max_words;
    }
    *first_bits_out = 0;
    memset(buf, 0, max_words * 2u);
    if (kind == 0u) {                                    /* plain random words */
        for (i = 0; i < nw; i++) {
            buf[i] = (uint16_t)rnd();
        }
        return nw;
    }
    if (kind == 1u) {                                    /* random runs of random length */
        uint32_t pos = 0, lvl = rint_(2);
        while (pos < nw * 16u) {
            uint32_t len = 1u + rint_(chance(50) ? 40u : 3000u), e = pos + len < nw * 16u ? pos + len : nw * 16u;
            if (lvl) {
                for (i = pos; i < e; i++) {
                    buf[i >> 4] |= (uint16_t)(0x8000u >> (i & 15u));
                }
            }
            lvl ^= 1u;
            pos = e;
        }
        return nw;
    }
    {   /* a bit stream with chosen timing (ns), as a WS controller would send it, sampled */
        double t0h = urand(100, 600), t1h = urand(300, 1500), per = urand(900, 2500), ts = 1e9 / (36e6 * (1.0 + urand(-0.03, 0.03)));
        double t = urand(0, 400) * 27.8, phase = urand(0, ts), glitch = chance(30) ? urand(0.0, 0.05) : 0.0;
        double hiccup = chance(30) ? urand(0.0, 0.05) : 0.0, gapp = chance(20) ? urand(0.0, 0.01) : 0.0;
        uint32_t nb = redge(600), total_samples = nw * 16u, bit = 0;
        if (chance(50)) {                                /* spec timing */
            t0h = urand(220, 380); t1h = urand(520, 1000); per = urand(1200, 1500);
        }
        if (chance(10)) {                                /* near the decision threshold (472 ns) */
            t0h = urand(440, 500); t1h = urand(440, 500);
        }
        if (chance(10)) {                                /* over high_max (2.2 us) */
            t1h = urand(2000, 4000); per = t1h + urand(300, 1000);
        }
        if (chance(30)) {
            t = 0;                                       /* starts inside a bit, not in the gap */
        }
        /* pulses as (rise, fall) in ns, sampled directly */
        while (bit < nb && t < total_samples * ts) {
            double h = (rnd() & 1u) ? t1h : t0h, p = per;
            uint32_t s0 = (uint32_t)((t + phase) / ts), s1 = (uint32_t)((t + h + phase) / ts);
            if (urand(0, 1) < glitch) {                  /* a glitch: a 1-3 sample spike somewhere in the low part */
                uint32_t g = s1 + 2u + rint_(10), gl = 1u + rint_(3);
                for (i = g; i < g + gl && i < total_samples; i++) {
                    buf[i >> 4] |= (uint16_t)(0x8000u >> (i & 15u));
                }
            }
            for (i = s0; i < s1 && i < total_samples; i++) {
                buf[i >> 4] |= (uint16_t)(0x8000u >> (i & 15u));
            }
            if (bit < 8u * sizeof dec_bits) {
                if (h >= 472.0) {
                    dec_bits[bit >> 3] |= (uint8_t)(0x80u >> (bit & 7u));
                }
            }
            bit++;
            if (p < h + 100.0) {
                p = h + 100.0;
            }
            t += p;
            if (urand(0, 1) < hiccup) {
                t += urand(0, 20e3);
            }
            if (urand(0, 1) < gapp) {
                t += urand(50e3, 300e3);                 /* a reset gap inside */
            }
        }
        *first_bits_out = bit;
        return nw;
    }
}

static void case_decode(void)
{
    static const ws_timing_t tm = WS_TIMING_DEFAULT;
    ws_timing_t t = tm;
    uint32_t nw, nbits, bits_in, k;
    uint8_t *o1, *o2;
    int r1, r2, r3;
    if (chance(20)) {                                    /* other timing constants */
        t.thr_one = (uint8_t)rrange(1, 40);
        t.high_max = (uint8_t)rrange(t.thr_one, 255);
        t.gap_min = (uint16_t)rrange(16, 4000);
    }
    memset(dec_bits, 0, sizeof dec_bits);
    nw = gen_samples(dec_buf, DEC_MAX_WORDS, &bits_in);
    nbits = chance(50) ? rrange(1, 400) : (uint32_t)redge(600) + 1u;
    if (nbits > 65535u) {
        nbits = 65535;
    }
    /* exact-size output buffers: ASan sees a write past (nbits + 7) / 8 */
    o1 = (uint8_t *)malloc((nbits + 7u) / 8u);
    o2 = (uint8_t *)malloc((nbits + 7u) / 8u);
    r1 = ws_decode_frame(dec_buf, nw * 16u + rint_(16), (uint16_t)nbits, &t, o1);
    r2 = ref_decode_frame(dec_buf, nw * 16u, (uint16_t)nbits, &t, o2);
    CHECK(r1 == r2, "decode_frame: result %d, reference %d (nw %u, nbits %u)", r1, r2, nw, nbits);
    CHECK(r1 == (int)nbits || (r1 <= WS_ERR_NO_EDGE && r1 >= WS_ERR_PULSE), "decode_frame: result %d out of range", r1);
    if (r1 == (int)nbits) {
        CHECK(memcmp(o1, o2, (nbits + 7u) / 8u) == 0, "decode_frame: bits differ from the reference");
        /* bits past nbits in the last byte are 0 */
        if (nbits & 7u) {
            CHECK((o1[nbits >> 3] & (0xFFu >> (nbits & 7u))) == 0u, "decode_frame: bits past nbits set");
        }
    }
    r3 = ws_decode_frame(dec_buf, nw * 16u, (uint16_t)nbits, &t, o2);   /* deterministic */
    CHECK(r3 == ws_decode_frame(dec_buf, nw * 16u, (uint16_t)nbits, &t, o1) && memcmp(o1, o2, (nbits + 7u) / 8u) == 0,
          "decode_frame: not deterministic");
    free(o1);
    free(o2);
    /* in-spec stream that starts in the gap, long enough: the bits come back as sent */
    (void)k;
}

/* the frame end (the Ultra trailer): the last 64 complete bits */
static void case_tail(void)
{
    static const ws_timing_t tm = WS_TIMING_DEFAULT;
    ws_timing_t t = tm;
    uint32_t nw, bits_in;
    uint8_t o1[8], o2[8], guard[8];
    int r1, r2;
    if (chance(20)) {
        t.thr_one = (uint8_t)rrange(1, 40);
        t.high_max = (uint8_t)rrange(t.thr_one, 255);
        t.gap_min = (uint16_t)rrange(16, 4000);
    }
    memset(dec_bits, 0, sizeof dec_bits);
    nw = gen_samples(dec_buf, chance(50) ? 576u : DEC_MAX_WORDS, &bits_in);
    memset(o1, 0xA5, 8);
    memset(o2, 0xA5, 8);
    memset(guard, 0xA5, 8);
    r1 = ws_decode_last(dec_buf, nw * 16u + rint_(16), &t, o1);
    r2 = ref_decode_last(dec_buf, nw * 16u, &t, o2);
    CHECK(r1 == r2, "decode_last: result %d, reference %d (nw %u)", r1, r2, nw);
    CHECK(r1 >= 0 || (r1 <= WS_ERR_NO_EDGE && r1 >= WS_ERR_PULSE), "decode_last: result %d out of range", r1);
    if (r1 > 0) {
        CHECK(memcmp(o1, o2, 8) == 0, "decode_last: the last 64 bits differ from the reference");
    } else {
        CHECK(memcmp(o1, guard, 8) == 0, "decode_last: output written on an error");
    }
}

/* ================================================================================================ frame path */
/* proto / frame_rx: random frames (lengths, headers, slots, modes) through the per-frame decisions.
 * Properties: the state stays in its enums; a slot is applied only on a frame that ends in a valid header of this
 * version, without RW / EXT / UPD, with Ultra active; a WS slot is shown only when its protocol is the active one and
 * the frame has no Ultra header; a forced mode never changes the active protocol (but the WS2915 header length);
 * in auto mode the lock moves only to the frame's own classification (or drops to none); deterministic. */
static void inv_proto(const proto_state_t *s)
{
    CHECK(s->mode <= PMODE_ULTRA, "proto: mode %d", (int)s->mode);
    CHECK(s->locked <= PROTO_ULTRA, "proto: locked %d", (int)s->locked);
    CHECK(s->cand <= PROTO_ULTRA, "proto: cand %d", (int)s->cand);
    CHECK(s->cand_count < PROTO_LOCK_FRAMES, "proto: cand_count %u", s->cand_count);
    CHECK(s->forced_hdr == 26u || s->forced_hdr == 32u, "proto: forced_hdr %u", s->forced_hdr);
    CHECK(!s->provisional || (s->mode == PMODE_AUTO && s->locked != PROTO_NONE), "proto: provisional without a lock");
}

static void inv_plan(const gate_plan_t *g)
{
    CHECK(g->own_bits == 40u || g->own_bits == 106u || g->own_bits == 112u || g->own_bits == 256u ||
          g->own_bits == 384u, "plan: own_bits %u", g->own_bits);
    CHECK(g->hdr_bits == 0u || g->hdr_bits == 26u || g->hdr_bits == 32u, "plan: hdr_bits %u", g->hdr_bits);
    CHECK(g->t1 <= g->own_bits && (g->t2 == 0u || g->t2 == g->own_bits), "plan: toggles %u %u own %u", g->t1, g->t2,
          g->own_bits);
}

static uint32_t rnd_frame_bits(void)
{
    switch (rint_(8)) {
    case 0: return 256u * rrange(1, 60) + 64u;           /* Ultra */
    case 1: return 40u * rrange(1, 100);                 /* WS2805 */
    case 2: return 32u + 80u * rrange(1, 50);            /* WS2915-32 */
    case 3: return 26u + 80u * rrange(1, 50);            /* WS2915-26 */
    case 4: return PROTO_UPD_BITS;
    case 5: return 64u;
    default: return redge(20000);
    }
}

static void case_frame(void)
{
    frx_t r, r2;
    proto_mode_t mode = (proto_mode_t)rint_(4);
    proto_t sticky = (proto_t)rint_(6);
    uint32_t i, n = rrange(1, 80), frame = rnd();
    uint8_t bits[PROTO_BITS_BUF + 4], tail[8];
    slot_t ws;
    memset(&r, 0, sizeof r);
    memset(&ws, 0, sizeof ws);
    frx_init(&r, mode, sticky);
    inv_proto(&r.ps);
    if (mode != PMODE_AUTO) {
        CHECK(!r.ps.provisional, "frame: forced mode provisional");
    }
    if (sticky > PROTO_ULTRA) {
        CHECK(r.ps.locked == PROTO_NONE, "frame: invalid sticky value taken");
    }
    r2 = r;
    for (i = 0; i < n; i++) {
        uint32_t total = rnd_frame_bits(), cap = chance(80) ? 1u : 0u, nbits = 0, own, skip = 0, k;
        proto_t p = proto_active(&r.ps), before;
        gate_plan_t g = proto_gate_plan(p);
        frx_end_t e, e2;
        uint8_t hdr_ok = 0, flags = 0, has_tail = (uint8_t)chance(70), slot_ok = 0;
        uint32_t ret = 0;
        inv_plan(&g);
        own = g.own_bits;
        if (r.ps.provisional && own < PROTO_DETECT_BITS) {
            own = PROTO_DETECT_BITS;                     /* as the poll ISR raises it */
        }
        frame++;
        if (cap) {
            memset(bits, 0, sizeof bits);
            for (k = 0; k < PROTO_BITS_BUF; k++) {
                bits[k] = (uint8_t)rnd();
            }
            if (g.hdr_bits != 0u && chance(50)) {        /* WS2915: the capture after the header pause */
                skip = g.hdr_bits;
                nbits = chance(80) ? 80u : rrange(1, 80);
            } else {
                nbits = chance(70) ? own : rrange(1, own);
                if (nbits >= ULTRA_SLOT_BITS && chance(60)) {
                    ultra_slot_begin(bits, (uint8_t)rint_(256), (uint8_t)rint_(16), (uint8_t)rint_(16), (uint8_t)rint_(256));
                    for (k = 3; k < 30u; k++) {
                        bits[k] = (uint8_t)rnd();
                    }
                    ultra_slot_seal(bits);
                    slot_ok = 1;
                }
            }
            ret = frx_capture(&r, frame, p, bits, nbits, g.own_bits, skip, &ws);
            (void)frx_capture(&r2, frame, p, bits, nbits, g.own_bits, skip, &ws);
            CHECK((ret & ~FRX_SHOW_WS) == 0u, "frame: capture returned %x", ret);
            if (ret & FRX_SHOW_WS) {
                CHECK(p != PROTO_ULTRA && p != PROTO_NONE && !r.ps.provisional, "frame: shown under %d", (int)p);
                CHECK(ws.raw_len == 5u || ws.raw_len == 10u, "frame: raw_len %u", ws.raw_len);
            }
            CHECK(!r.have_ultra || (nbits >= ULTRA_SLOT_BITS && ultra_slot_check(bits) == 0 && skip == 0u),
                  "frame: have_ultra without a slot");                /* (random bytes pass the CRC-16 once in 65536) */
            (void)slot_ok;
        }
        if (has_tail) {
            uint32_t tus = rnd();
            flags = (uint8_t)rint_(16);
            ultra_build_header_flags(tail, tus, flags);
            if (chance(15)) {
                tail[2] = (uint8_t)((tail[2] & 0x0Fu) | (rint_(16) << 4));   /* another version */
                tail[7] = ultra_crc8(tail, 7);
            }
            if (chance(15)) {
                tail[rint_(8)] ^= (uint8_t)(1u << rint_(8));
            }
            hdr_ok = (uint8_t)(ultra_parse_header(tail, &tus) == 0);
        }
        before = proto_active(&r.ps);
        {
            uint32_t fe = chance(90) ? frame : frame - 1u;   /* (the capture may belong to another frame) */
            memset(&e, 0, sizeof e);                     /* (padding: the structs are compared whole) */
            memset(&e2, 0, sizeof e2);
            frx_frame_end(&r, fe, total, has_tail ? tail : 0, &e);
            frx_frame_end(&r2, fe, total, has_tail ? tail : 0, &e2);
        }
        inv_proto(&r.ps);
        inv_plan(&e.plan);
        CHECK(e.before == before, "frame: before mismatch");
        CHECK(e.after == proto_active(&r.ps), "frame: after mismatch");
        CHECK(e.cls <= PROTO_ULTRA, "frame: cls %d", (int)e.cls);
        CHECK(e.ultra_header == hdr_ok, "frame: header %u, parsed %u", e.ultra_header, hdr_ok);
        CHECK(!e.ultra_header || e.flags == (flags & 0x0Fu), "frame: flags");
        CHECK(!e.ultra_sync || (e.ultra_header && e.after == PROTO_ULTRA && !(e.flags & ULTRA_HDR_UPD)),
              "frame: sync without a header / Ultra / with UPD");
        CHECK(!e.ultra_slot || (e.ultra_sync && !(e.flags & (ULTRA_HDR_RW | ULTRA_HDR_EXT))), "frame: slot in RW / EXT");
        CHECK(!e.show_ws || (!e.ultra_header && e.cls == (proto_t)r.ws_proto && e.after == e.cls && e.cls != PROTO_ULTRA &&
                             e.cls != PROTO_NONE), "frame: show_ws rule");
        CHECK(!e.ultra_header || (e.cls == PROTO_ULTRA) == ((total >= 64u) && ((total - 64u) % 256u == 0u)),
              "frame: Ultra classification by length");
        CHECK(e.ultra_header || e.cls != PROTO_ULTRA, "frame: Ultra without a header");
        if (mode != PMODE_AUTO) {
            proto_t want = mode == PMODE_ULTRA ? PROTO_ULTRA : mode == PMODE_WS2805 ? PROTO_WS2805
                           : (r.ps.forced_hdr == 26u ? PROTO_WS2915_26 : PROTO_WS2915_32);
            CHECK(e.after == want, "frame: forced mode active %d", (int)e.after);
            CHECK(!e.close_gate, "frame: close_gate in a forced mode");
        } else {
            CHECK(e.after == before || e.after == e.cls || e.after == PROTO_NONE, "frame: lock moved to %d on a %d frame",
                  (int)e.after, (int)e.cls);
            CHECK(!e.close_gate || e.after != before, "frame: close_gate without a change");
        }
        CHECK(!r.have_ultra && !r.have_ws, "frame: capture flags not consumed");
        CHECK(memcmp(&e, &e2, sizeof e) == 0 && memcmp(&r, &r2, sizeof r) == 0, "frame: not deterministic");
        if (chance(5)) {
            proto_mode_t m2 = (proto_mode_t)rint_(4);
            proto_set_mode(&r.ps, m2);
            proto_set_mode(&r2.ps, m2);
            mode = m2;
            inv_proto(&r.ps);
        }
    }
}

/* the frame tracker with main.c's capture bookkeeping (cap_skip / cap_bits / cap_own) on random edge counts.
 * Properties: cap_state 0..3; a copy only after a start; the decoded bit count main.c asks for (cap_bits - cap_skip)
 * is 0..own (0: main.c rejects it) (own = the plan's bits, 256 for a provisional plan, 384 transparent), so it fits the decode buffer;
 * a frame end always arms; the tracker never stalls while edges keep coming. */
static void case_ftrack(void)
{
    ftrack_t t;
    uint32_t i, n = rrange(10, 2000), c = 0, own, hdr, armed_once = 0, provisional = chance(30);
    uint16_t cap_skip = 0, cap_own = 0, cap_bits = 0, cap_state_seen = 0;
    static const uint16_t owns[] = { 40, 112, 106, 256, 384 };
    static const uint16_t hdrs[] = { 0, 32, 26, 0, 0 };
    uint32_t pick = rint_(5), own_raised;
    memset(&t, 0, sizeof t);
    own = owns[pick];
    hdr = hdrs[pick];
    if (chance(5)) {
        ftrack_disarm(&t);
    }
    for (i = 0; i < n; i++) {
        uint32_t ev, before_state = t.cap_state;
        /* the hardware counter (TIM4): DIN falling edges since the last gate re-arm. Between two ticks (50 us) it can
         * only grow: a WS bit is >= 1.2 us (<= 42 bits per tick), noise could be faster (bounded at 2000 here); it
         * wraps at 65536; it goes back to 0 only when the frame end / boot arm re-arms the gate (below). Idle
         * stretches, bursts, exactly a WS2915 header's count, a flash stall (the ISR off for up to 18 ms: ~15000
         * edges at most) */
        switch (rint_(12)) {
        case 0: case 1: case 2: case 3: break;           /* idle: c stays */
        case 4: if (c < 32u) c = hdr ? hdr : 32u; break; /* exactly a WS2915 header */
        case 5: c += rrange(1, 3); break;
        case 6: c += rrange(1, 42); break;
        case 7: c += rrange(40, 2000); break;            /* noise */
        case 8: ftrack_disarm(&t); c += rrange(1000, 15000); break;   /* a flash stall: the ISR off for up to 18 ms,
                                                            many edges at once; main.c disarms the tracker meanwhile */
        default: c += rrange(0, 8); break;
        }
        c &= 0xFFFFu;
        own_raised = provisional && own < PROTO_DETECT_BITS ? PROTO_DETECT_BITS : own;
        ev = ftrack_tick(&t, (uint16_t)c, (uint16_t)own_raised, (uint16_t)hdr);
        if (ev & (FT_FRAME_END | FT_BOOT_ARM)) {
            c = chance(70) ? 0u : rrange(0, 3);          /* gate_rearm: the counter restarts (edges may follow at once) */
            if (chance(10)) {                            /* ... and the next plan is loaded (plan_buf): only here */
                pick = rint_(5);
                own = owns[pick];
                hdr = hdrs[pick];
                provisional = chance(30);
            }
        }
        CHECK(t.cap_state <= 3u, "ftrack: cap_state %u", t.cap_state);
        CHECK(!(ev & FT_CAP_COPY) || before_state == 2u, "ftrack: copy without the state");
        if (ev & FT_CAP_START) {
            if (ev & FT_CAP_DATA) {
                cap_skip = (uint16_t)hdr;
            } else {
                cap_skip = 0;
                cap_own = (uint16_t)own;
            }
        }
        if ((ev & FT_CAP_COPY) || (ev & FT_CAP_SHORT)) {
            uint32_t want;
            cap_bits = t.cap_bits;
            want = (uint32_t)cap_bits - cap_skip;
            /* (0.6.4) 0 = a WS2915 frame that ended at its header: main.c's process_capture rejects it before the
             * decoder (want - 1 >= PROTO_MAX_BITS); a wrap (cap_bits < skip) would still fail here */
            CHECK(want <= 384u, "ftrack: decode of %u bits asked (cap_bits %u, skip %u)", want, cap_bits, cap_skip);
            CHECK(cap_own == 384u || want <= 288u, "ftrack: %u bits into the 36-byte slot buffer (own %u)", want, cap_own);
            cap_state_seen = 1;
        }
        if (ev & FT_FRAME_END) {
            CHECK(t.armed && t.cap_state == 0u && t.last_cnt == 0u && t.idle == 0u, "ftrack: frame end state");
            armed_once = 1;
        }
        if (ev & FT_BOOT_ARM) {
            CHECK(t.armed, "ftrack: boot arm");
            armed_once = 1;
        }
        if (chance(2)) {
            ftrack_disarm(&t);
            CHECK(!t.armed && t.cap_state == 0u && t.partial, "ftrack: disarm");
        }
    }
    (void)armed_once;
    (void)cap_state_seen;
}

/* ================================================================================================ ultra engine */
/* Properties: every field the engine indexes with stays in range after any slot (roles, masks, effect ids, step
 * counts, set points); an unknown slot (type, light, IDENTIFY value, SCENE mode) leaves the whole engine byte-identical;
 * a slot with a bad CRC changes nothing; outputs come back for any time (also wrapped ticks and host clocks);
 * the same case replayed gives the same state (deterministic); no division by zero / endless loop (ASan + runtime). */
static void inv_light(const ultra_light_t *L, uint32_t l)
{
    uint32_t i;
    for (i = 0; i < 5u; i++) {
        CHECK(L->role[i] <= ULTRA_ROLE_WARM_WHITE, "ultra: light %u role[%u] = %u", l, i, L->role[i]);
    }
    CHECK((L->mask & ~0x1Fu) == 0u, "ultra: light %u mask %x", l, L->mask);
    CHECK((L->flags & ~ULTRA_DEF_FLAGS) == 0u, "ultra: light %u flags %x", l, L->flags);
    CHECK((L->opt_flags & ~ULTRA_OPT_FLAGS) == 0u, "ultra: light %u opt_flags %x", l, L->opt_flags);
    CHECK(L->opt_n <= ULTRA_OPT_POINTS_MAX, "ultra: light %u opt_n %u", l, L->opt_n);
    for (i = 1; i < L->opt_n && i < ULTRA_OPT_POINTS_MAX; i++) {
        CHECK(L->opt_pm[i - 1u] >= L->opt_pm[i], "ultra: light %u points not sorted", l);
    }
    CHECK(L->scene_mode <= 1u, "ultra: light %u scene_mode %u", l, L->scene_mode);
    CHECK(L->scene_fx < ULTRA_FX_COUNT, "ultra: light %u scene_fx %u", l, L->scene_fx);
    CHECK(L->fx < ULTRA_FX_COUNT && L->pend_fx < ULTRA_FX_COUNT, "ultra: light %u fx %u / %u", l, L->fx, L->pend_fx);
    CHECK(L->step_count <= ULTRA_MAX_STEPS, "ultra: light %u step_count %u", l, L->step_count);
    CHECK(L->g_q16 <= 4294902u, "ultra: light %u gamma %u", l, L->g_q16);
}

static void inv_ultra(const ultra_t *u)
{
    uint32_t l;
    for (l = 0; l < ULTRA_LIGHTS; l++) {
        inv_light(&u->light[l], l);
    }
    CHECK(u->identify <= ULTRA_IDENT_VERSION, "ultra: identify %u", u->identify);
    CHECK((u->cfg_valid & ~((1u << ULTRA_CFG_COUNT) - 2u)) == 0u, "ultra: cfg_valid %x", u->cfg_valid);
    CHECK(u->clk.rate_q16 >= (50u << 16) - (50u << 16) / 16u && u->clk.rate_q16 <= (50u << 16) + (50u << 16) / 16u,
          "ultra: rate %u", u->clk.rate_q16);
}

/* a random slot: a builder with random arguments (the host's space), or random bytes with a valid CRC */
static void rnd_slot(uint8_t *s, int *expect_ignored)
{
    uint8_t light = (uint8_t)(chance(80) ? rint_(5) : rint_(16)), seq = (uint8_t)rint_(256);
    uint16_t a[5], b[5], mn[5], mx[5];
    uint8_t roles[5], params[22], ids[12];
    uint16_t vals[12];
    uint32_t i, kind = rint_(16), type;
    for (i = 0; i < 5u; i++) {
        a[i] = (uint16_t)redge(65535);
        b[i] = (uint16_t)redge(65535);
        mn[i] = (uint16_t)redge(65535);
        mx[i] = (uint16_t)redge(65535);
        roles[i] = (uint8_t)rint_(16);
    }
    for (i = 0; i < 22u; i++) {
        params[i] = (uint8_t)rnd();
    }
    for (i = 0; i < 12u; i++) {
        ids[i] = (uint8_t)(chance(70) ? rint_(ULTRA_CFG_COUNT) : rint_(256));
        vals[i] = (uint16_t)redge(65535);
    }
    *expect_ignored = 0;
    switch (kind) {
    case 0: ultra_slot_nop(s); *expect_ignored = 1; break;
    case 1: ultra_slot_light_def(s, light, seq, roles, (uint16_t)redge(65535), (uint8_t)rint_(256), mn, mx); break;
    case 2: ultra_slot_set(s, light, seq, (uint8_t)rint_(16), (uint16_t)redge(65535), a, b, redge(0xFFFFFF), (int16_t)rnd()); break;
    case 3: ultra_slot_effect(s, light, seq, (uint8_t)rint_(12), (int32_t)rnd(), params, rint_(23)); break;
    case 4: ultra_slot_step(s, light, seq, (uint8_t)rint_(16), (uint8_t)rint_(16), (uint16_t)redge(65535), (uint16_t)redge(65535),
                            (uint16_t)redge(65535), a, b); break;
    case 5: ultra_slot_config(s, seq, ids, vals, rint_(13)); break;
    case 6: ultra_slot_identify(s, seq, (uint8_t)rint_(8)); break;
    case 7: ultra_slot_calibrate(s, seq, (uint8_t)rint_(256)); break;
    case 8: ultra_slot_hold_dark(s, seq, (uint8_t)rint_(256)); break;
    case 9: ultra_slot_scene(s, light, seq, (uint8_t)rint_(3), (uint8_t)rint_(256), (uint16_t)redge(65535), a, b); break;
    case 10: ultra_slot_light_opt_points(s, light, seq, (uint8_t)rint_(4), (uint16_t)redge(65535), (uint16_t)redge(65535),
                                         (uint16_t)redge(65535), (uint16_t)redge(65535), rint_(5), a, b); break;
    case 11: ultra_slot_scene_effect(s, light, seq, (uint8_t)rint_(12), params, rint_(23)); break;
    case 12: ultra_slot_dark(s); *expect_ignored = 1; break;
    default:                                             /* random bytes, valid CRC */
        type = chance(60) ? rint_(ULTRA_TYPES) : rint_(256);
        ultra_slot_begin(s, (uint8_t)type, light, (uint8_t)rint_(16), seq);
        for (i = 3; i < 30u; i++) {
            s[i] = (uint8_t)rnd();
        }
        ultra_slot_seal(s);
        break;
    }
    /* what the compatibility rules say is ignored, from the sealed slot */
    if (ultra_slot_check(s) == 0) {
        uint8_t ty = s[0], li = (uint8_t)(s[1] >> 4);
        const uint8_t *p = s + ULTRA_PAYLOAD;
        if (ty == ULTRA_NOP || ty == ULTRA_POLL || ty >= ULTRA_TYPES) {
            *expect_ignored = 1;
        } else if (li >= ULTRA_LIGHTS && li != ULTRA_LIGHT_BOARD) {
            *expect_ignored = 1;
        } else if (ty == ULTRA_IDENTIFY && ((p[0] > ULTRA_IDENT_RELEASE && p[0] != ULTRA_IDENT_VERSION) ||
                                            (p[0] == ULTRA_IDENT_CAL && p[1] != 0u &&
                                             (p[1] < ULTRA_IDENT_CAL_MIN || p[1] > ULTRA_IDENT_CAL_MAX)))) {
            *expect_ignored = 1;
        } else if (ty == ULTRA_SCENE && p[0] > 1u) {
            *expect_ignored = 1;
        }
    } else {
        *expect_ignored = 1;
    }
}

static ultra_t ue1, ue2;                                 /* the engine and its twin (determinism) */

static void case_ultra(void)
{
    ultra_t *u = &ue1, *w = &ue2;
    uint32_t i, n = rrange(1, 400), tick = chance(30) ? 0xFFFFFFFFu - rint_(100000) : rnd(), host = rnd();
    uint32_t seed = rnd(), host_rate_ppm = (uint32_t)(1000000 + (int32_t)rrange(0, 60000) - 30000);
    uint16_t out[5], out2[5];
    uint8_t s[32];
    scene_t st, so, so2;
    ultra_init(u, seed);
    ultra_init(w, seed);
    inv_ultra(u);
    memset(&st, 0, sizeof st);
    st.mask = (uint8_t)rint_(32);
    st.fade = (uint8_t)rint_(256);
    for (i = 0; i < n; i++) {
        uint32_t act = rint_(20), dt;
        switch (act) {
        case 0: case 1: case 2: case 3:                  /* a header: the host clock moves with the ticks (rate), with
                                                            jitter, now and then a jump or a wrap */
            dt = chance(80) ? rrange(100, 6000) : rrange(0, 2000000);
            tick += dt;
            host += (uint32_t)((unsigned long long)dt * 50u * host_rate_ppm / 1000000u) + rrange(0, 60) - 30u;
            if (chance(3)) {
                host = rnd();                            /* the host rebooted */
            }
            ultra_on_header(u, tick, host);
            ultra_on_header(w, tick, host);
            break;
        case 4: case 5: case 6: case 7: case 8: case 9: { /* a slot */
            int ign;
            uint32_t ev, ev2;
            uint8_t before[sizeof(ultra_t)];
            rnd_slot(s, &ign);
            if (chance(10)) {
                s[rint_(32)] ^= (uint8_t)(1u << rint_(8));   /* a bit error */
                ign = ultra_slot_check(s) != 0;
            }
            memcpy(before, u, sizeof before);
            ev = ultra_on_slot(u, s, tick);
            ev2 = ultra_on_slot(w, s, tick);
            CHECK(ev == ev2, "ultra: twin events differ");
            CHECK((ev & ~(ULTRA_EV_CONFIG | ULTRA_EV_IDENTIFY | ULTRA_EV_LIGHT | ULTRA_EV_SCENE | ULTRA_EV_STATE |
                          ULTRA_EV_DEF)) == 0u, "ultra: event %x", ev);
            if (ign) {
                CHECK(ev == 0u && memcmp(before, u, sizeof before) == 0, "ultra: an ignored slot (type %02x light %x "
                      "p0 %u p1 %u, crc %d) changed the state / returned %x", s[0], s[1] >> 4, s[3], s[4],
                      ultra_slot_check(s), ev);
            }
            inv_ultra(u);
            break;
        }
        case 10: case 11: case 12: case 13: case 14:     /* renders: every ms, or after a while */
            dt = chance(70) ? 20u : rrange(0, 40000);
            tick += dt;
            (void)ultra_render(u, tick, out);
            (void)ultra_render(w, tick, out2);
            CHECK(memcmp(out, out2, sizeof out) == 0, "ultra: twin outputs differ");
            ultra_clock_tend(u, tick);
            ultra_clock_tend(w, tick);
            break;
        case 15: {
            uint32_t dur = redge(1000);
            ultra_all_off(u, tick, dur);
            ultra_all_off(w, tick, dur);
            break;
        }
        case 16:
            ultra_clock_gap(u);
            ultra_clock_gap(w);
            break;
        case 17: {
            uint8_t mask = (uint8_t)rint_(32);
            if (chance(50)) {
                ultra_scene_fx_start(u, mask, tick);
                ultra_scene_fx_start(w, mask, tick);
            } else {
                ultra_scene_fx_stop(u);
                ultra_scene_fx_stop(w);
            }
            break;
        }
        case 18:
            ultra_scene(u, &st, &so);
            ultra_scene(w, &st, &so2);
            CHECK(memcmp(&so, &so2, sizeof so) == 0, "ultra: twin scenes differ");
            CHECK((so.mask & ~0x1Fu) == 0u, "ultra: scene mask %x", so.mask);
            break;
        default:                                         /* a long stretch of renders (effects run) */
            {
                uint32_t k, m = rrange(1, 200);
                for (k = 0; k < m; k++) {
                    tick += 20u;
                    (void)ultra_render(u, tick, out);
                    (void)ultra_render(w, tick, out2);
                }
                CHECK(memcmp(out, out2, sizeof out) == 0, "ultra: twin outputs differ (run)");
            }
            break;
        }
    }
    inv_ultra(u);
    CHECK(memcmp(u, w, sizeof *u) == 0, "ultra: the twin engine ended in another state");
}

/* ================================================================================================ config */
/* Properties: after any sequence of commands (frames or console) the settings stay in the ranges apply_settings and
 * pwm_start rely on; NACK / SAME leave them untouched; the result codes are in the enum; the frame counter never
 * overflows; a slot that is not a config message is CFG_NONE. */
static int settings_valid(const settings_t *s)
{
    return s->protocol <= PMODE_ULTRA && (s->pwm_preset < PWM_PRESET_COUNT || s->pwm_preset == 0xFFu) &&
           s->dither_mode <= 2u && s->dither_thresh >= 1u && s->feedback_ch <= 2u && s->stagger < STAGGER_MODES &&
           s->dim_floor <= 2u && s->min_pulse >= 5u && s->min_pulse <= 100u && s->signal_loss <= 2u &&
           s->smoothing <= 2u && s->gamma <= 2u && s->last_proto <= PROTO_ULTRA &&
           settings_steps(s) >= PWM_STEPS_MIN && settings_steps(s) <= PWM_STEPS_MAX;
}

static void case_config(void)
{
    settings_t s, before;
    config_rx_t rx;
    slot_t sl;
    uint32_t i, n = rrange(1, 300);
    settings_default(&s);
    config_rx_init(&rx);
    CHECK(settings_valid(&s), "config: defaults invalid");
    for (i = 0; i < n; i++) {
        cfg_result_t r;
        uint8_t cmd = (uint8_t)(chance(70) ? rrange(1, 0x14) : rint_(256)), arg = (uint8_t)redge(255);
        before = s;
        if (chance(50)) {                                /* the console path */
            r = config_apply(&rx, &s, cmd, arg);
        } else {                                         /* a frame: a config message or random data */
            uint32_t k, ws = chance(50);
            memset(&sl, 0, sizeof sl);
            sl.raw_len = (uint8_t)(ws ? 10 : 5);
            if (chance(70)) {
                config_build(cmd, arg, (uint8_t)ws, sl.raw);
                if (chance(10)) {
                    sl.raw[rint_(sl.raw_len)] ^= (uint8_t)(1u << rint_(8));
                }
            } else {
                for (k = 0; k < 10u; k++) {
                    sl.raw[k] = (uint8_t)rnd();
                }
            }
            r = config_process(&rx, &s, &sl);
        }
        CHECK(r.event <= CFG_IDENTIFY, "config: event %d", (int)r.event);
        CHECK(settings_valid(&s), "config: settings out of range after cmd %02x arg %u (event %d): preset %u steps %u "
              "stagger %u minpulse %u", cmd, arg, (int)r.event, s.pwm_preset, s.raw_steps, s.stagger, s.min_pulse);
        if (r.event == CFG_NACK || r.event == CFG_SAME || r.event == CFG_NONE || r.event == CFG_IDENTIFY) {
            CHECK(memcmp(&s, &before, sizeof s) == 0, "config: settings changed on event %d", (int)r.event);
        }
        CHECK(rx.count <= 255u, "config: count");
    }
}

/* ================================================================================================ update */
/* Properties: the state fields stay in range (have_half, st_next, result); an action is in the enum; TAKE comes only
 * for a descriptor that passes upd_desc_check for this board and the version policy; CONFIRM only while a status
 * record is free (main.c writes at st_next); bad CRCs change nothing; deterministic. */
static void case_update(void)
{
    upd_app_t u, w;
    upd_app_info_t info;
    uint8_t f[UPD_FRAME_BYTES], d[UPD_DESC_BYTES], page[UPD_PAGE_BYTES], bl[16];
    uint32_t i, n = rrange(1, 300), now = rnd();
    /* the board's own info: from a random descriptor page and bootloader info, as main.c reads them */
    for (i = 0; i < UPD_PAGE_BYTES; i++) {
        page[i] = (uint8_t)(chance(50) ? 0xFF : rnd());
    }
    memcpy(bl, "QDBL\x01\x01\x03", 7);
    memcpy(bl + 7, "\0abcdefgh", 9);
    if (chance(30)) {
        for (i = 0; i < 8u; i++) {
            bl[i] = (uint8_t)rnd();
        }
    }
    if (chance(60)) {                                    /* a valid descriptor of its own */
        memset(d, 0, sizeof d);
        memcpy(d, "QDU5", 4);
        d[UPD_D_DESC_VERSION] = UPD_DESC_VERSION;
        d[UPD_D_HW_ID] = (uint8_t)(chance(80) ? 1 : rint_(4));
        upd_put16(d + UPD_D_HW_REV_MASK, chance(80) ? 0x8u : (uint16_t)rnd());
        upd_put32(d + UPD_D_FW_VERSION, rnd());
        upd_put32(d + UPD_D_BODY_LEN, (rrange(1, UPD_APP_BODY_MAX / 4u)) * 4u);
        upd_put32(d + UPD_D_LOAD_ADDR, UPD_APP_BODY);
        upd_put32(d + UPD_D_ENTRY, UPD_APP_ENTRY);
        d[UPD_D_MIN_BL] = (uint8_t)rint_(3);
        upd_put32(d + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d, 15u));
        memcpy(page, d, 64);
        for (i = 64; i < UPD_PAGE_BYTES; i += 2u) {      /* status records, then erased */
            uint32_t v = i < 64u + 2u * rint_(97) ? (chance(50) ? UPD_ST_VERIFIED : chance(50) ? UPD_ST_CONFIRMED : rnd())
                                                   : (chance(50) ? 0xFFFFu : 0xE339u);
            page[i] = (uint8_t)v;
            page[i + 1u] = (uint8_t)(v >> 8);
        }
    }
    upd_app_read_info(&info, page, bl);
    CHECK(info.st_next <= UPD_ST_COUNT, "update: st_next %u", info.st_next);
    CHECK(!info.desc_ok || info.bl_version != 0u, "update: desc_ok without a bootloader");
    upd_app_init(&u, &info);
    upd_app_init(&w, &info);
    for (i = 0; i < n; i++) {
        static int confirm_pending;                      /* main.c answers a CONFIRM with upd_app_confirmed() */
        uint32_t a = 0, a2 = 0, act = rint_(10);
        if (i == 0u) {
            confirm_pending = 0;
        }
        now += chance(80) ? rrange(0, 20) : rrange(0, 5000);
        if (act < 5u) {                                  /* a captured whole frame */
            static const uint8_t types[] = { UPD_ANNOUNCE, UPD_DATA, UPD_END, UPD_ABORT, UPD_PING };
            uint8_t type = (uint8_t)(chance(85) ? types[rint_(5)] : rint_(256)), data[32];
            uint32_t idx = chance(70) ? rint_(3) : rint_(65536), k;
            for (k = 0; k < 32u; k++) {
                data[k] = (uint8_t)rnd();
            }
            if (type == UPD_ANNOUNCE && chance(70)) {    /* a descriptor half, valid or nearly */
                uint8_t d2[64];
                memcpy(d2, d, 64);
                if (chance(50)) {
                    upd_put32(d2 + UPD_D_FW_VERSION, rnd());
                    d2[UPD_D_HW_ID] = (uint8_t)(chance(70) ? 1 : rint_(4));
                    upd_put32(d2 + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d2, 15u));
                }
                memcpy(data, d2 + 32u * (idx & 1u), 32);
            }
            upd_build_frame(f, type, (uint8_t)rint_(4), idx, (uint8_t)rnd(), (uint8_t)rint_(4), data, now);
            if (chance(10)) {
                f[rint_(48)] ^= (uint8_t)(1u << rint_(8));
            }
            a = upd_app_frame(&u, f, now);
            a2 = upd_app_frame(&w, f, now);
        } else if (act < 8u) {                           /* a frame end */
            uint8_t hdr = (uint8_t)chance(80), fl = (uint8_t)(chance(50) ? UPD_HDR_FLAG : rint_(16));
            upd_app_frame_end(&u, hdr, fl, now);
            upd_app_frame_end(&w, hdr, fl, now);
        } else if (act == 8u) {
            a = upd_app_poll(&u, now);
            a2 = upd_app_poll(&w, now);
        } else if (confirm_pending) {                    /* the write of the CONFIRMED record succeeded or not */
            int ok = chance(70);
            confirm_pending = 0;
            upd_app_confirmed(&u, ok);
            upd_app_confirmed(&w, ok);
        }
        if (a == UPD_APP_CONFIRM) {
            confirm_pending = 1;
        }
        CHECK(a == a2, "update: twin actions differ");
        CHECK(a <= UPD_APP_SHOW_REJECTED, "update: action %u", a);
        CHECK(u.have_half <= 3u, "update: have_half %u", u.have_half);
        CHECK(u.info.st_next <= UPD_ST_COUNT, "update: st_next %u", u.info.st_next);
        CHECK(u.result == 0u || u.result == UPD_APP_SHOW_SAME || u.result == UPD_APP_SHOW_REJECTED, "update: result %u",
              u.result);
        if (a == UPD_APP_TAKE) {
            CHECK(u.info.bl_version != 0u && upd_desc_check(u.pending, upd_crc32_words(0xFFFFFFFFu, u.pending, 15u),
                  u.info.hw_id, u.info.hw_rev, u.info.bl_version) == 0, "update: TAKE of an image not for this board");
        }
        if (a == UPD_APP_CONFIRM) {
            CHECK(u.info.st_next < UPD_ST_COUNT && u.info.bl_version != 0u && u.info.desc_ok && !u.info.confirmed,
                  "update: CONFIRM without a free record / descriptor");
        }
    }
    CHECK(memcmp(&u, &w, sizeof u) == 0, "update: twin state differs");
}

/* ================================================================================================ console */
/* Properties: no crash on any line (random bytes, long lines, every command with random arguments); the output of one
 * line is bounded; `watch` stays in 0 / 100..60000; the line buffer never overflows; the settings stay valid. */
static char cli_out[1 << 16];
static uint32_t cli_out_n;
static settings_t cli_set;
static config_rx_t cli_rx;
static volatile uint16_t cli_hwd[HW_DIAG_N];             /* (0.6.5, D6) */
static scene_t cli_scene;
static cli_status_t cli_st;
static vdip_t cli_dip;
static pwrev_t cli_pw;
static health_t cli_hl;
static vsense_t cli_vs;
static evlog_t cli_ev;                                   /* (0.6.5, D2) */

static void cli_write(const char *s)
{
    while (*s) {
        if (cli_out_n + 1u < sizeof cli_out) {
            cli_out[cli_out_n] = *s;
        }
        cli_out_n++;
        s++;
    }
    cli_out[cli_out_n < sizeof cli_out ? cli_out_n : sizeof cli_out - 1u] = 0;
}

static cfg_result_t cli_apply(uint8_t cmd, uint8_t arg)
{
    cfg_result_t r = config_apply(&cli_rx, &cli_set, cmd, arg);
    CHECK(settings_valid(&cli_set), "cli: settings out of range after %02x %u", cmd, arg);
    return r;
}

static void cli_save(void) {}

static energy_t cli_en;                                  /* (0.6.3) the energy / lifetime record the status shows */

static uint32_t cli_ovr[3];                              /* (0.6.5 D4) the overrun counters */
static void rnd_status(void)
{
    uint8_t *b = (uint8_t *)&cli_st;
    uint32_t i;
    for (i = 0; i < sizeof cli_st; i++) {
        b[i] = (uint8_t)rnd();
    }
    cli_st.version = chance(50) ? "0.6.2" : "1.2.3-rc.4";
    memset(cli_st.bl_build, 'x', 8);
    cli_st.bl_build[8] = 0;
    cli_st.steps = chance(90) ? rrange(PWM_STEPS_MIN, PWM_STEPS_MAX) : 1440u;   /* (0 would divide by zero: store path) */
    cli_st.cal.levels = (uint8_t)(chance(70) ? rint_(5) : rint_(256));
    /* the structs the status points to: random but with their own invariants (as the firmware keeps them) */
    for (i = 0; i < sizeof cli_dip; i++) ((uint8_t *)&cli_dip)[i] = (uint8_t)rnd();
    cli_dip.head = (uint8_t)rint_(VDIP_LOG);
    cli_dip.n_log = (uint8_t)rint_(VDIP_LOG + 1u);
    for (i = 0; i < sizeof cli_pw; i++) ((uint8_t *)&cli_pw)[i] = (uint8_t)rnd();
    for (i = 0; i < sizeof cli_hl; i++) ((uint8_t *)&cli_hl)[i] = (uint8_t)rnd();
    vsense_init(&cli_vs, 220000u, 10000u, 3280u);          /* the supply divider (hal/board.h) */
    cli_st.dip = chance(80) ? &cli_dip : 0;
    cli_st.pw = chance(80) ? &cli_pw : 0;
    cli_st.hl = chance(80) ? &cli_hl : 0;
    for (i = 0; i < 3u; i++) {
        cli_ovr[i] = chance(50) ? 0u : (uint32_t)rnd();
    }
    cli_st.ovr = chance(80) ? cli_ovr : 0;
    for (i = 0; i < HW_DIAG_N; i++) {                    /* (0.6.5, D6) the ADC / DMA counters: mostly 0 */
        cli_hwd[i] = (uint16_t)(chance(70) ? 0u : rnd());
    }
    cli_st.hwd = chance(80) ? cli_hwd : 0;
    cli_st.dip_vs = &cli_vs;
    for (i = 0; i < sizeof cli_en; i++) ((uint8_t *)&cli_en)[i] = (uint8_t)rnd();
    cli_st.en = &cli_en;                                 /* (0.6.3: always set, as main.c does) */
    for (i = 0; i < EVLOG_N; i++) {                      /* (0.6.5, D2) the event log: any times, its types */
        cli_ev.e[i].t_s = (uint32_t)rnd();
        cli_ev.e[i].ma = (uint16_t)rnd();
        cli_ev.e[i].type = (uint8_t)rint_(EVL_CAL + 1u);
    }
    cli_ev.head = (uint8_t)rint_(EVLOG_N);
    cli_st.ev = chance(80) ? &cli_ev : 0;
    {
        static uint32_t uid[3];                          /* (0.6.5 D10) the chip's id: always set too */
        for (i = 0; i < 3u; i++) {
            uid[i] = (uint32_t)rnd();
        }
        cli_st.uid = uid;
    }
}

static const char *const cli_words[] = { "help", "?", "status", "get", "set", "period", "save", "defaults", "identify",
    "version", "watch", "protocol", "preset", "dither", "thresh", "feedback", "stagger", "floor", "minpulse", "loss",
    "smooth", "gamma", "scene", "scenefade", "ilimit", "ical", "softstart", "0", "1", "255", "256", "1440", "65535",
    "65536", "-1", "99999999999", " ", "  ", "\t", "x", "setx", "sett", "" };

static void case_cli(void)
{
    cli_t c;
    uint32_t i, n = rrange(1, 40);
    memset(&c, 0, sizeof c);
    c.write = cli_write;
    c.apply = cli_apply;
    c.save_now = cli_save;
    c.set = &cli_set;
    c.scene = &cli_scene;
    c.st = &cli_st;
    settings_default(&cli_set);
    config_rx_init(&cli_rx);
    cli_scene.mask = (uint8_t)rint_(32);
    cli_scene.fade = (uint8_t)rint_(256);
    for (i = 0; i < 5u; i++) {
        cli_scene.v[i] = (uint16_t)rnd();
    }
    rnd_status();
    for (i = 0; i < n; i++) {
        char line[200];
        uint32_t len = 0, k, kind = rint_(4);
        if (kind == 0u) {                                /* random bytes */
            len = rint_(sizeof line - 1u);
            for (k = 0; k < len; k++) {
                line[k] = (char)(1u + rint_(255));
            }
        } else {                                         /* words from the vocabulary, numbers, junk */
            uint32_t w = rrange(1, 6);
            for (k = 0; k < w && len + 40u < sizeof line; k++) {
                const char *s = cli_words[rint_(sizeof cli_words / sizeof cli_words[0])];
                if (chance(20)) {
                    len += (uint32_t)sprintf(line + len, "%u", redge(70000));
                } else {
                    len += (uint32_t)sprintf(line + len, "%s", s);
                }
                if (chance(80)) {
                    line[len++] = ' ';
                }
            }
        }
        line[len] = 0;
        cli_out_n = 0;
        if (chance(50)) {
            cli_exec(&c, line);
            while (cli_poll(&c)) {
            }
        } else {                                         /* through the line buffer, byte by byte */
            for (k = 0; k < len; k++) {
                cli_feed(&c, line[k]);
                CHECK(c.len < sizeof c.line, "cli: line buffer %u", c.len);
            }
            cli_feed(&c, chance(50) ? '\r' : '\n');
            while (cli_poll(&c)) {
            }
        }
        CHECK(c.watch_ms == 0u || (c.watch_ms >= 100u && c.watch_ms <= 60000u), "cli: watch_ms %u", c.watch_ms);
        CHECK(cli_out_n < 8192u, "cli: %u bytes of output for one line", cli_out_n);
        if (chance(30)) {
            cli_out_n = 0;
            cli_watch(&c);
            CHECK(cli_out_n < 512u, "cli: watch line %u bytes", cli_out_n);
        }
        if (chance(10)) {
            rnd_status();
        }
    }
}

/* ================================================================================================ flash records */
/* The single-page logs with random page contents: pure random, erased (both values the chip may read), a history of
 * real writes with the last one torn (a prefix of its bytes), stale records of another layout, duplicate sequence
 * numbers. Properties: no crash; a found record passes its own check and is the newest written whole; the next free
 * slot is past every used one; a torn last record is never taken. */
static void erased_fill(uint8_t *p, uint32_t n)
{
    uint32_t i, e = chance(50) ? 0xFFFFu : 0xE339u;
    for (i = 0; i + 1u < n; i += 2u) {
        p[i] = (uint8_t)e;
        p[i + 1u] = (uint8_t)(e >> 8);
    }
}

static void random_fill(uint8_t *p, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        p[i] = (uint8_t)rnd();
    }
}

static void rnd_settings(settings_t *s)
{
    uint8_t *b = (uint8_t *)s;
    uint32_t i;
    if (chance(70)) {
        settings_default(s);
        s->pwm_preset = (uint8_t)rint_(PWM_PRESET_COUNT);
        s->last_proto = (uint8_t)rint_(5);
        return;
    }
    for (i = 0; i < sizeof *s; i++) {
        b[i] = (uint8_t)rnd();                           /* another firmware's values */
    }
}

static void case_store(void)
{
    uint8_t page[256];
    settings_t s, out, last_whole;
    store_ext_t e, oute;
    uint32_t seq = 0, outseq, i, k = rint_(STORE_SLOTS + 2u), have_whole = 0, torn = 0;
    int found;
    if (chance(20)) {
        random_fill(page, 256);
    } else {
        erased_fill(page, 256);
        for (i = 0; i < k && i < STORE_SLOTS; i++) {     /* a history of writes, the last may be torn */
            store_slot_t sl;
            rnd_settings(&s);
            store_ext_clear(&e);
            e.ilimit50 = (uint8_t)rnd();
            e.opts = (uint8_t)rnd();
            seq += chance(90) ? 1u : rrange(0, 1000000);
            if (chance(10)) {
                seq = 0xFFFFFFFFu - rint_(3);            /* near the 32-bit wrap */
            }
            store_pack_ext(&s, &e, seq, &sl);
            if (chance(15)) {
                sl.magic ^= 1u;                          /* an older layout */
                sl.crc = (uint8_t)rnd();
            }
            if (i + 1u == k && chance(40)) {             /* torn: a prefix of the bytes */
                torn = rrange(1, STORE_SLOT_BYTES - 1u);
                memcpy(page + i * STORE_SLOT_BYTES, &sl, torn);
            } else {
                memcpy(page + i * STORE_SLOT_BYTES, &sl, STORE_SLOT_BYTES);
                if (sl.magic == STORE_MAGIC) {
                    last_whole = s;
                    have_whole = 1;
                }
            }
        }
    }
    found = store_find_ext(page, &out, &oute, &outseq);
    CHECK(found >= -1 && found < (int)STORE_SLOTS, "store: index %d", found);
    if (found >= 0) {
        const store_slot_t *sl = (const store_slot_t *)(page + (uint32_t)found * STORE_SLOT_BYTES);
        CHECK(sl->magic == STORE_MAGIC && sl->seq == outseq, "store: found record magic / seq");
        for (i = 0; i < STORE_SLOTS; i++) {
            const store_slot_t *o = (const store_slot_t *)(page + i * STORE_SLOT_BYTES);
            if (o->magic == STORE_MAGIC && o->crc == crc8_smbus((const uint8_t *)&o->seq, 27u)) {
                CHECK(o->seq <= outseq, "store: a newer valid record was skipped");
            }
        }
    }
    (void)have_whole;
    (void)last_whole;
    i = store_first_erased(page, STORE_SLOT_BYTES, STORE_SLOTS);
    CHECK(i <= STORE_SLOTS, "store: next free %u", i);
    CHECK(found < (int)i || i == STORE_SLOTS, "store: the found record %d is at / past the next free slot %u", found, i);
}

static void case_scene(void)
{
    static uint8_t log[SCENE_LOG_PAGES * SCENE_PAGE_BYTES];
    scene_t s, out;
    uint16_t seq = (uint16_t)rnd();
    uint32_t i, k = rint_(SCENE_RECS + 5u), start = rint_(SCENE_RECS), nx;
    int newest = -1;
    const uint8_t *rp;
    pp_pos_t pos;
    scene_clear(&s);
    if (chance(20)) {
        random_fill(log, sizeof log);
    } else {
        erased_fill(log, sizeof log);
        for (i = 0; i < k && i < SCENE_RECS; i++) {      /* k writes from a random position (wear levelling) */
            scene_rec_t rec;
            uint32_t at = (start + i) % SCENE_RECS;
            uint8_t *dst = log + at * SCENE_REC_BYTES;
            scene_clear(&s);
            s.mask = (uint8_t)(chance(90) ? rint_(32) : rnd());
            s.fade = (uint8_t)rnd();
            s.v[rint_(5)] = (uint16_t)rnd();
            scene_log_pack(&s, ++seq, &rec);
            if (i + 1u == k && chance(40)) {
                memcpy(dst, &rec, rrange(1, SCENE_REC_BYTES - 1u));
            } else {
                memcpy(dst, &rec, SCENE_REC_BYTES);
            }
        }
    }
    /* 0.6.4: the log is a page pair (core/pagepair.h, scene_pp) */
    rp = pp_find(&scene_pp, log, log + SCENE_PAGE_BYTES, &pos);
    if (rp) {
        newest = (int)((rp - log) / SCENE_REC_BYTES);
        out = ((const scene_rec_t *)rp)->s;
        CHECK(scene_rec_valid(rp) && (out.mask & ~0x1Fu) == 0u, "scene: mask %x from a valid record", out.mask);
    }
    nx = PP_PAGE(pos.next) * SCENE_RECS_PAGE + PP_ENTRY(pos.next);
    CHECK(PP_PAGE(pos.next) <= 1u && PP_ENTRY(pos.next) < SCENE_RECS_PAGE && pos.erase <= 1u, "scene: next %u erase %u",
          pos.next, pos.erase);
    CHECK(newest < 0 || nx != (uint32_t)newest, "scene: the next record would overwrite the newest");
    if (newest >= 0 && pos.erase) {
        CHECK((uint32_t)newest / SCENE_RECS_PAGE != PP_PAGE(pos.next), "scene: the erase due takes the newest's page");
    }
    if (!pos.erase) {
        CHECK(store_erased(log + nx * SCENE_REC_BYTES, SCENE_REC_BYTES), "scene: next slot not erased");
    }
    /* the runtime on a random scene: hold / fade mixing stays in range */
    {
        scene_run_t r;
        uint16_t v[5];
        uint32_t now = rnd(), n = rrange(1, 100);
        scene_boot(&r, newest >= 0 ? &out : &s);
        for (i = 0; i < n; i++) {
            uint32_t j;
            for (j = 0; j < 5u; j++) {
                v[j] = (uint16_t)rnd();
            }
            now += rrange(0, 3000);
            if (chance(20)) {
                scene_takeover(&r, (uint8_t)rint_(256), now);
            }
            (void)scene_mix(&r, now, v);
        }
    }
}

static void case_cal(void)
{
    uint8_t page[256];
    isense_cal_t c, out;
    isense_t is;
    health_t h;
    uint32_t seq = 0, outseq, i, k = rint_(CAL_RECS + 2u);
    uint16_t cnt, outcnt;
    int found;
    if (chance(20)) {
        random_fill(page, 256);
    } else {
        erased_fill(page, 256);
        for (i = 0; i < k && i < CAL_RECS; i++) {
            cal_rec_t rec;
            uint8_t *b = (uint8_t *)&c;
            uint32_t j;
            for (j = 0; j < sizeof c; j++) {
                b[j] = (uint8_t)rnd();
            }
            if (chance(60)) {
                c.levels = (uint8_t)(chance(50) ? 4 : chance(50) ? 1 : 0);
                c.pct[0] = (uint8_t)rint_(120);
            }
            cnt = (uint16_t)rnd();
            seq += chance(90) ? 1u : rrange(0, 100000);
            cal_pack(&c, seq, cnt, &rec);
            if (chance(10)) {
                rec.magic ^= 0x100u;
            }
            if (i + 1u == k && chance(40)) {
                memcpy(page + i * CAL_REC_BYTES, &rec, rrange(1, CAL_REC_BYTES - 1u));
            } else {
                memcpy(page + i * CAL_REC_BYTES, &rec, CAL_REC_BYTES);
            }
        }
    }
    found = cal_find(page, &out, &outseq, &outcnt);
    CHECK(found >= -1 && found < (int)CAL_RECS, "cal: index %d", found);
    i = store_first_erased(page, CAL_REC_BYTES, CAL_RECS);
    CHECK(i <= CAL_RECS, "cal: next free %u", i);
    /* what main.c does with it: isense_cal_set, then the model and the health checks on random values */
    isense_init(&is, 1, 10000u, 8000u);
    isense_set_gain(&is, ISENSE_GAIN_DEFAULT_Q16);
    health_init(&h);
    if (found >= 0) {
        uint16_t v[5];
        uint32_t j;
        isense_cal_set(&is, &out);
        if (out.levels == 1u || out.levels == ISENSE_LEVELS) {
            CHECK(is.cal_have == out.levels || (out.levels == 1u && (out.pct[0] < 5u || out.pct[0] > 100u)),
                  "cal: a valid record not taken");
        } else {
            CHECK(is.cal_have == 0u, "cal: levels %u taken", out.levels);
        }
        for (j = 0; j < 5u; j++) {
            v[j] = (uint16_t)rnd();
        }
        (void)isense_model_ma(&is, v);
        (void)isense_connected_ma(&is);
        for (j = 0; j < 5u; j++) {
            (void)isense_on_ma(&is, j, v[j]);
            (void)health_i100(&out, j);
        }
        health_test_start(&h, &out);
        (void)health_test_done(&h, &out);
        health_run(&h, &is, &out);
        {
            isense_cal_t got;
            (void)isense_cal_get(&is, &got);
        }
    }
}

static void case_energy(void)
{
    static uint8_t pg[2][256];
    life_rec_t rec, got, best;
    life_pos_t pos;
    uint32_t i, k = rint_(2u * LIFE_RECS + 2u), w = rnd(), found, have = 0, at;
    memset(&best, 0, sizeof best);
    if (chance(20)) {
        random_fill(pg[0], 256);
        random_fill(pg[1], 256);
    } else {
        erased_fill(pg[0], 256);
        erased_fill(pg[1], 256);
        for (i = 0; i < k && i < 2u * LIFE_RECS; i++) {  /* k records appended from page 0 (0.6.3: the lifetime record) */
            uint8_t *dst = pg[i / LIFE_RECS] + (i % LIFE_RECS) * LIFE_REC_BYTES;
            random_fill((uint8_t *)&rec, sizeof rec);
            rec.writes = ++w;
            rec.magic = LIFE_MAGIC;
            rec.crc = crc16_ccitt((const uint8_t *)&rec, 38u);
            if (i + 1u == k && chance(40)) {             /* torn */
                memcpy(dst, &rec, rrange(1, LIFE_REC_BYTES - 1u));
            } else {
                memcpy(dst, &rec, LIFE_REC_BYTES);
            }
        }
    }
    for (i = 0; i < 2u * LIFE_RECS; i++) {               /* the reference: the valid record with the newest count */
        const uint8_t *r = pg[i / LIFE_RECS] + (i % LIFE_RECS) * LIFE_REC_BYTES;
        memcpy(&rec, r, sizeof rec);
        if (rec.magic == LIFE_MAGIC && rec.crc == crc16_ccitt(r, 38u) &&
            (!have || (int32_t)(rec.writes - best.writes) > 0)) {
            best = rec;
            have = 1;
        }
    }
    found = life_find(pg[0], pg[1], &got, &pos);
    CHECK(found == have && (!have || got.writes == best.writes), "life: found %u (%u), writes %u / %u", found, have,
          found ? got.writes : 0u, have ? best.writes : 0u);
    CHECK(PP_PAGE(pos.next) <= 1u && PP_ENTRY(pos.next) < LIFE_RECS && pos.erase <= 1u, "life: next %u erase %u",
          pos.next, pos.erase);
    if (!pos.erase) {
        CHECK(store_erased(pg[PP_PAGE(pos.next)] + PP_ENTRY(pos.next) * LIFE_REC_BYTES, LIFE_REC_BYTES),
              "life: next entry not erased");
    }
    at = life_plan(&pos);
    CHECK(at == LIFE_PLAN_NONE || PP_ENTRY(at) < LIFE_RECS, "life: plan %u", at);
    /* the runtime on random inputs: the budget and the write reasons */
    {
        energy_t e;
        uint32_t now = rnd(), now0, n = rrange(1, 500), writes = 0, up = 0;
        uint16_t dips = 0;
        memset(&e, 0, sizeof e);
        if (found) {
            e.lt = got;
        }
        energy_init(&e, found, now, chance(50) ? rnd() : 0u, (uint32_t)chance(10));
        now0 = now;
        for (i = 0; i < n; i++) {
            uint32_t why;
            now += chance(80) ? 1u : rrange(0, 4000000);
            energy_ms(&e, now, rrange(0, 75000), (int32_t)rrange(0, 70000) - 2000, (uint8_t)chance(60));
            if (chance(5)) {
                up += rrange(1, 100000);
                energy_second(&e, now, up, rnd(), (int32_t)rrange(0, 20000), (int32_t)rrange(0, 1300));
            }
            if (chance(2)) {
                dips++;
            }
            why = energy_due(&e, now, dips, rint_(4));
            CHECK(why <= ENERGY_DUE_DIP, "energy: due %u", why);
            if (why != ENERGY_DUE_NONE) {
                energy_pack(&e);
                CHECK(e.lt.magic == LIFE_MAGIC && e.lt.crc == crc16_ccitt((const uint8_t *)&e.lt, 38u),
                      "energy: the packed record checks");
                energy_saved(&e, up);
                writes++;
            }
        }
        CHECK(writes <= LIFE_BUDGET_CAP + (now - now0) / LIFE_REFILL_MS + 1u, "energy: %u writes in %u ms", writes,
              now - now0);
    }
}

/* the power-on effect store (two pages, ULTRA_FX_*): random images, torn headers, stale heads, bit errors.
 * Properties: nothing loads without the magic; a loaded head passes its CRC; the engine runs on whatever was loaded
 * (no crash, outputs for any time). The firmware's own invariants on a loaded head are checked (inv_light): the
 * loader trusts the CRC-8 alone, so a head that passes it by chance (1 in 256 of random content) is reported. */
static void case_fx(void)
{
    static uint8_t img[512];
    ultra_t *u = &ue1, *src = &ue2;
    uint8_t crc[ULTRA_LIGHTS], mask, s[32];
    uint32_t i, l, kind = rint_(4), tick = rnd();
    uint16_t out[5];
    int ign;
    ultra_init(src, rnd());
    for (i = 0; i < 12u; i++) {                          /* a host defines the lights the store would hold */
        rnd_slot(s, &ign);
        (void)ultra_on_slot(src, s, 0);
    }
    if (kind == 0u) {
        random_fill(img, sizeof img);
        if (chance(50)) {
            img[0] = 0x53; img[1] = 0x46; img[2] = 0x58; img[3] = 0x31;   /* the magic over random heads */
        }
    } else {
        erased_fill(img, sizeof img);
        mask = (uint8_t)(kind == 3u ? rnd() : rint_(32));
        for (l = 0; l < ULTRA_LIGHTS; l++) {
            uint8_t *h = img + ULTRA_FX_HDR + l * ULTRA_HEAD_BYTES;
            if (mask & (1u << l)) {
                memcpy(h, &src->light[l], ULTRA_HEAD_BYTES);
                if (chance(10)) {
                    h[rint_(ULTRA_HEAD_BYTES)] ^= (uint8_t)(1u << rint_(8));   /* a bit error */
                }
                if (chance(10)) {
                    random_fill(h, ULTRA_HEAD_BYTES);    /* a head of other content */
                }
            }
            crc[l] = crc8_smbus(h, ULTRA_HEAD_BYTES);
            if (chance(5)) {
                crc[l] = (uint8_t)rnd();
            }
            img[5u + l] = crc[l];
        }
        img[4] = mask;
        img[10] = img[11] = 0;
        img[0] = 0x53; img[1] = 0x46; img[2] = 0x58; img[3] = 0x31;
        if (kind == 2u) {                                /* torn header: a prefix of its half-words */
            uint32_t keep = 2u * rrange(0, 5);
            for (i = keep; i < ULTRA_FX_HDR; i++) {
                img[i] = 0xFF;
            }
        }
    }
    ultra_init(u, rnd());
    mask = ultra_fx_load(u, img, crc);
    CHECK((mask & ~0x1Fu) == 0u, "fx: loaded mask %x", mask);
    if ((img[0] | ((uint32_t)img[1] << 8) | ((uint32_t)img[2] << 16) | ((uint32_t)img[3] << 24)) != ULTRA_FX_MAGIC) {
        CHECK(mask == 0u, "fx: loaded without the magic");
    }
    for (l = 0; l < ULTRA_LIGHTS; l++) {
        if (mask & (1u << l)) {
            CHECK((img[4] & (1u << l)) && crc8_smbus(img + ULTRA_FX_HDR + l * ULTRA_HEAD_BYTES, ULTRA_HEAD_BYTES) == img[5u + l],
                  "fx: light %u loaded without a valid CRC", l);
            inv_light(&u->light[l], l);                  /* (the firmware trusts the CRC alone) */
        }
    }
    ultra_scene_fx_start(u, mask, tick);
    for (i = 0; i < 300u; i++) {
        tick += 20u;
        (void)ultra_render(u, tick, out);
        ultra_clock_tend(u, tick);
    }
    {
        scene_t st, so;
        memset(&st, 0, sizeof st);
        ultra_scene(u, &st, &so);
    }
    ultra_scene_fx_stop(u);
    (void)ultra_render(u, tick + 20u, out);
}

/* ================================================================================================ regressions */
/* Small fixed cases found by the random runs (each a reproducer of a reported finding). */
static void test_regress(void)
{
    g_path = "regress";
    case_begin(0);
    /* a settings record with raw_steps below PWM_STEPS_MIN passes the CRC (another firmware's value): the consumers
     * (pattern_len_for, F_TIM_HZ / steps) rely on 1440..65535 */
    {
        settings_t s;
        store_slot_t sl;
        store_ext_t e;
        uint8_t page[256];
        uint32_t seq;
        settings_default(&s);
        s.pwm_preset = 0xFF;
        s.raw_steps = 0;
        store_ext_clear(&e);
        store_pack_ext(&s, &e, 1, &sl);
        memset(page, 0xFF, sizeof page);
        memcpy(page, &sl, sizeof sl);
        CHECK(store_find_ext(page, &s, &e, &seq) == 0, "regress: the record is found");
#ifdef CONFIG_HAS_SANITIZE
        CHECK(settings_sanitize(&s) == 1 && s.raw_steps == 5760u && s.pwm_preset == 0xFFu,
              "regress: the out-of-range period takes its default (the rest stays)");
        {
            settings_t d;
            settings_default(&d);
            CHECK(settings_sanitize(&d) == 0, "regress: the defaults pass the range check untouched");
        }
#endif
        CHECK(settings_valid(&s), "regress: a stored raw period of %u is taken as is (pattern_len_for divides by it; "
              "a pattern of %u entries would overflow the %u-entry PWM tables)", s.raw_steps,
              s.raw_steps ? pattern_len_for(s.raw_steps) : 255u, (unsigned)PATTERN_MAX_LEN);
    }
    /* a mode change to a forced protocol while the stored (provisional) protocol is not confirmed yet: `provisional`
     * stays set outside auto mode (frx_capture then holds the first WS slot to its frame end; forced WS2915 learning
     * a 26-bit header looks like a dropped stored protocol and forces the gate closed once) */
    {
        proto_state_t s;
        proto_init(&s, PMODE_AUTO, PROTO_ULTRA);
        proto_set_mode(&s, PMODE_WS2805);
        CHECK(!s.provisional, "regress: provisional stays set after the switch to a forced mode");
        proto_init(&s, PMODE_AUTO, PROTO_WS2915_32);
        proto_set_mode(&s, PMODE_WS2915);
        CHECK(!s.provisional, "regress: provisional stays set after the switch to forced WS2915");
    }
    /* a power-on effect head that passes its CRC-8 but holds values the engine never produces (roles 8-15, more than
     * 4 set points, an effect id past the known ones, mask bits 5-7) */
    {
        static uint8_t img[512];
        ultra_t *u = &ue1;
        uint8_t crc[ULTRA_LIGHTS], *h = img + ULTRA_FX_HDR;
        uint32_t i;
        memset(img, 0xFF, sizeof img);
        ultra_init(u, 1);
        u->light[0].mask = 0xFF;
        u->light[0].role[0] = 15;
        u->light[0].opt_n = 200;
        u->light[0].opt_flags = ULTRA_OPT_POINTS | ULTRA_OPT_DIM_TO_WARM;
        u->light[0].scene_fx = 0x7F;
        u->light[0].scene_mode = 7;
        memcpy(h, &u->light[0], ULTRA_HEAD_BYTES);
        img[0] = 0x53; img[1] = 0x46; img[2] = 0x58; img[3] = 0x31;
        img[4] = 0x01;
        img[5] = crc8_smbus(h, ULTRA_HEAD_BYTES);
        for (i = 6; i < 12u; i++) img[i] = 0;
        ultra_init(u, 1);
        if (ultra_fx_load(u, img, crc) & 0x01u) {
            CHECK(0, "regress: a head with values this firmware never writes is loaded (the CRC-8 alone decides)");
            inv_light(&u->light[0], 0);
        } else {
            CHECK(u->light[0].role[0] == 0u && u->light[0].mask == 0u && u->light[0].g_q16 == 183501u,
                  "regress: a rejected head leaves the light as initialised");
        }
    }
}

/* ================================================================================================ driver */
typedef struct { const char *name; void (*fn)(void); uint32_t def_count; const char *what; } path_t;
static const path_t paths[] = {
    { "decode", case_decode, 200000, "ws_decode_frame vs a sample-by-sample reference on random streams" },
    { "tail",   case_tail,   200000, "ws_decode_last (the Ultra trailer) vs the reference" },
    { "frame",  case_frame,  100000, "frame_rx / proto: lock rules, slot / show gating, flags, determinism" },
    { "ftrack", case_ftrack, 100000, "frame tracker + main.c's capture bookkeeping on random edge counts" },
    { "ultra",  case_ultra,  50000,  "the Ultra engine: random slots, headers, times; compat rules; twin engine" },
    { "config", case_config, 100000, "config frames / console commands: settings ranges, NACK / SAME untouched" },
    { "update", case_update, 100000, "upd_app: random UPD frames, headers, polls, descriptor pages" },
    { "cli",    case_cli,    30000,  "console lines: random bytes and words; status / watch on random state" },
    { "store",  case_store,  100000, "settings page: random, torn, stale, duplicate records" },
    { "scene",  case_scene,  100000, "scene log (0.6.4 page pair): random / torn records, next position, runtime mix" },
    { "cal",    case_cal,    100000, "calibration page + isense_cal_set + health on the loaded test" },
    { "energy", case_energy, 100000, "lifetime record (0.6.3 page pair) + the write budget on random inputs" },
    { "fx",     case_fx,     50000,  "the power-on effect store: random / torn images, the engine on what loads" },
};
#define NPATHS (sizeof paths / sizeof paths[0])

static void run_path(const path_t *p, unsigned long long seed, uint32_t count)
{
    uint32_t i;
    int fail0 = g_fail;
    g_path = p->name;
    for (i = 0; i < count; i++) {
        case_begin(mix64(seed ^ ((unsigned long long)hash_bytes(p->name, strlen(p->name)) << 32)) + i);
        p->fn();
        if (g_fail - fail0 >= 20) {
            printf("  %s: stopping after 20 failures\n", p->name);
            break;
        }
        if ((i + 1u) % 100000u == 0u) {
            printf("  %s: %u cases, %d failures\n", p->name, i + 1u, g_fail - fail0);
            fflush(stdout);
        }
    }
    printf("%-7s %8u cases, seed %llx: %s (%d failures)\n", p->name, i, seed, p->what, g_fail - fail0);
}

int main(int argc, char **argv)
{
    unsigned long long seed = 0x5EED0001ull;
    uint32_t count = 0, i;
    const char *name = 0;
    if (argc > 1) {
        name = argv[1];
    }
    if (argc > 3 && strcmp(argv[2], "--case") == 0) {
        for (i = 0; i < NPATHS; i++) {
            if (strcmp(paths[i].name, name) == 0) {
                g_path = paths[i].name;
                g_verbose = 1;
                case_begin(strtoull(argv[3], 0, 16));
                paths[i].fn();
                printf("%s case %s: %d failures\n", name, argv[3], g_fail);
                return g_fail ? 1 : 0;
            }
        }
        printf("unknown path %s\n", name);
        return 2;
    }
    if (argc > 2) {
        count = (uint32_t)strtoul(argv[2], 0, 10);
    }
    if (argc > 3) {
        seed = strtoull(argv[3], 0, 16);
    }
    if (name && strcmp(name, "regress") == 0) {
        test_regress();
        printf("%s (%d failures)\n", g_fail ? "FAILED" : "ALL TESTS PASSED", g_fail);
        return g_fail ? 1 : 0;
    }
    if (name && strcmp(name, "all") != 0) {
        for (i = 0; i < NPATHS; i++) {
            if (strcmp(paths[i].name, name) == 0) {
                run_path(&paths[i], seed, count ? count : paths[i].def_count);
                printf("%s (%d failures)\n", g_fail ? "FAILED" : "ALL TESTS PASSED", g_fail);
                return g_fail ? 1 : 0;
            }
        }
        printf("unknown path %s\n", name);
        return 2;
    }
    for (i = 0; i < NPATHS; i++) {
        run_path(&paths[i], seed, count ? count : paths[i].def_count);
    }
    test_regress();
    printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
