/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Diff-Power-UR-Digital-M's CHAIN line mode (core/dline.c's chain scanner, core/ur_digital_m/udchain.c, udfx.c and the encoder) on a word-exact model of the delay
 * ring: RX DMA writes word k at time k, TX reads it 510 words later; the 20 kHz tick runs dline_tick and udfx_tick as main_ur_digital_m.c's dl_tick does, the main loop
 * decodes a captured frame 70 .. 200 us later (model_chain_decode is main_ur_digital_m.c's ud_chain_decode, step for step; the slot is decoded as the head's words
 * arrive, udc_stream, from the main loop's passes) and the board answers. The hub is a model
 * of the sender's wire: real Ultra frames (slots + the header trailer, WS2915 bit timing, >= 280 us resets), RW frames with their window (the pair released
 * 150 us after the frame, the board's burst reads LOW at RO, the hub's retake), the preamble, pixel frames and the markers of a pixel-line hub. The strip is
 * decoded sample by sample: only whole encoder frames (or, in stream mode, the hub's own whole frames) may ever appear on it.
 *
 *   test\build_udchain.cmd [outdir] [scenario-prefix]     (MSVC; none: all)
 *
 * What it proves: the raw capture (the head = the slot, the snapshot = the header) while the encoder replaces every ring word (R5); the frame's bits from
 * TIM4, zeroed in the gap by dline only (R4; a negative control without the zero); AUTO's decision and the forced modes; the answer's start inside the Ultra
 * window slot at every HSI and tick lateness; no marker, frame or answer ever on the strip; STREAM_WINS off; faults, a flash operation and a re-configuration
 * without a leak; frames of more than one slot, UPD frames, pixel streams and markers in chain mode. The board at any position of
 * its line: the relay window of every valid POLL (the turnaround at t0 + 500 us: port 2 listens, the line above checked and driven from the relay node, RO
 * high-Z; forward again at W + 6 us) and the gate's slot plan, re-armed after ULTRA_RW_REARM_US of quiet LOW (main_ur_digital_m.c dl_tick): never in a
 * window, a frame or a released HIGH, in every gap before the next frame; a line above still driven at the turnaround: no relay, no answer, counted. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/dline.h"
#include "../src/core/ws_decode.h"
#include "../src/core/ur_digital_m/udfx.h"
#include "../src/core/ur_digital_m/udchain.h"
#include "../src/core/ultra_protocol.h"
#include "../src/core/ultra_update.h"

#define M DLINE_RING_MASK

static int g_fail;
static long g_checks;
static const char *g_name = "";
static int g_printed;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; if (g_printed++ < 24) { printf("  FAIL %s:%d [%s] ", __FILE__, \
    __LINE__, g_name); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static uint32_t rng = 0xC0FFEEu;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static double urnd(double a, double b) { return a + (b - a) * ((double)(rnd() >> 8) / 16777216.0); }

/* ------------------------------------------------------------------------------------------------ the world */
#if defined(_MSC_VER)
__declspec(align(4)) static uint16_t ring[UX_RING_WORDS];
#else
static uint16_t ring[UX_RING_WORDS] __attribute__((aligned(4)));
#endif
static uint16_t *line;                  /* RO as the hub drives it (HIGH while the pair is released) */
static uint16_t *out;                   /* the strip's pin by time: word T */
static uint32_t *fallp;                 /* falling edges of RX's samples in words [0, k] (TIM4 without its zero) */
static uint32_t cap_w;

static dline_t D;
static uint16_t CAP[DLINE_CAP_WORDS];              /* dline's capture buffer (dline_init) */
static udfx_t U;
static udc_t C;
static udc_pre_t P;

static double S = 36.0;                 /* samples per hub us: 36 x (1 + hsi) */
static uint32_t g_phase;                /* the sample of hub time 0 */

static uint32_t t2s(double us) { return g_phase + (uint32_t)floor(us * S + 0.5); }

static void world_reserve(uint32_t words)
{
    free(line); free(out); free(fallp);
    line = (uint16_t *)calloc(words + 64u, sizeof *line);
    out = (uint16_t *)calloc(words + 64u, sizeof *out);
    fallp = (uint32_t *)calloc(words + 64u, sizeof *fallp);
    if (!line || !out || !fallp) {
        printf("out of memory\n");
        exit(3);
    }
    cap_w = words;
}

static void set_samples(uint32_t a, uint32_t b)
{
    uint32_t x;
    for (x = a; x < b; x++) {
        line[x >> 4] |= (uint16_t)(0x8000u >> (x & 15u));
    }
}

/* ------------------------------------------------------------------------------------------------ the hub */
#define HUB_T0H 0.34
#define HUB_T1H 0.70
#define HUB_BIT 1.25

typedef struct {
    double   t0, t1;                    /* the first bit's rise, the last falling edge (hub us) */
    uint32_t bits, slots, rw, upd, marker, pixels;
    uint8_t  slot0[ULTRA_SLOT_BYTES];
    uint32_t end_w;                     /* the ring word of the last falling edge (the model's truth) */
    uint32_t index, rate, rcount;       /* RW: the POLL's */
    uint32_t host;                      /* the header's TIME */
} hf_t;
#define MAXH 4096
static hf_t hf[MAXH];
static uint32_t nhf;
static double hub_t;                    /* the hub's clock: the next frame may start here */

static uint32_t g_mixed;                                   /* the board sits behind a BOARD 2 stand-in: its gate removes the first slot */
static double g_rel = 150.0;                               /* us after an RW frame's last edge: the host stops driving, the pair floats HIGH (the protocol: 60 .. 150) */
static double g_mx_dly = 0.20;                             /* us: two hops and the gate */
static uint32_t g_hold500;                                 /* the stand-in holds the board's input LOW until its turnaround (+500 us): a host releasing
                                                              late or a board in front re-armed early; else it forwards the host's release (its slot
                                                              plan re-arms only after 200 us of quiet LOW, ULTRA_RW_REARM_US) */
/* us after an RW frame's last edge: the line at the board goes HIGH (the BOARD 2 stand-in in front: its release or its turnaround) */
static double stand_in_release(void)
{
    return g_hold500 ? 500.0 : g_rel;
}
static double g_mx_skew = 0.025;                           /* us: the interface drivers / receivers and the gate's tPLH / tPHL difference, per edge */

static double hub_bits(double t, const uint8_t *b, uint32_t nbits, double t0h, double t1h)
{
    uint32_t i;
    double last = t;
    for (i = 0; i < nbits; i++) {
        const double hi = ((b[i >> 3] >> (7u - (i & 7u))) & 1u) ? t1h : t0h, r = t + i * HUB_BIT;
        if (g_mixed && nbits > 320u && i < 256u) {         /* the stand-in's own slot: gone (its gate holds the output LOW) */
            continue;
        }
        if (g_mixed) {
            const double rr = r + g_mx_dly + urnd(-g_mx_skew, g_mx_skew), ff = r + hi + g_mx_dly + urnd(-g_mx_skew, g_mx_skew);
            set_samples(t2s(rr), t2s(ff > rr + 0.1 ? ff : rr + 0.1));
            last = ff;
            continue;
        }
        set_samples(t2s(r), t2s(r + hi));
        last = r + hi;
    }
    return last;
}

/* an Ultra frame of `payload` bytes (a multiple of 8) and the header (TIME = its end, flags) */
static hf_t *hub_frame(const uint8_t *payload, uint32_t nbytes, uint8_t flags, double gap_us)
{
    static uint8_t buf[ULTRA_SLOT_BYTES * 80u + ULTRA_HDR_BYTES];
    const uint32_t nbits = 8u * nbytes + 64u;
    hf_t *f = &hf[nhf++];
    const double end_guess = hub_t + nbits * HUB_BIT - 0.4;
    memset(f, 0, sizeof *f);
    memcpy(buf, payload, nbytes);
    f->host = (uint32_t)floor(end_guess + 0.5);
    ultra_build_header_flags(buf + nbytes, f->host, flags);
    f->t0 = hub_t;
    f->t1 = hub_bits(hub_t, buf, nbits, HUB_T0H, HUB_T1H);
    f->bits = nbits - (g_mixed && nbytes > 32u ? 256u : 0u);
    f->slots = nbytes / 32u - (g_mixed && nbytes > 32u ? 1u : 0u);
    f->rw = (flags & ULTRA_HDR_RW) != 0u;
    f->upd = (flags & ULTRA_HDR_UPD) != 0u;
    memcpy(f->slot0, payload + (g_mixed && nbytes > 32u ? 32u : 0u), 32u);
    f->end_w = t2s(f->t1) / 16u;
    hub_t = f->t1 + gap_us;
    return f;
}
static hf_t *hub_ultra(const uint8_t *slots, uint32_t n, uint8_t flags, double gap_us) { return hub_frame(slots, 32u * n, flags, gap_us); }
/* a frame for the board's slot: on a mixed line the BOARD 2 stand-in's slot (a NOP) goes in front of it */
static hf_t *hub_ultra1(const uint8_t *s, uint8_t flags, double gap_us)
{
    uint8_t b[ULTRA_SLOT_BYTES * 2u];
    if (!g_mixed) {
        return hub_ultra(s, 1, flags, gap_us);
    }
    ultra_slot_nop(b);
    memcpy(b + 32, s, 32);
    return hub_ultra(b, 2, flags, gap_us);
}
static double gap(void) { return urnd(285.0, 420.0); }

/* an RW frame with the board's POLL after `nbefore` slots of upstream boards (the frame at the board has only its own: the gates removed theirs); the pair
 * is released 150 us after the frame until the retake at the window's end. index: the answer index, r: slots in the window */
static hf_t *hub_poll(uint32_t index, uint32_t r, uint32_t rate, uint32_t page)
{
    uint8_t s[ULTRA_SLOT_BYTES * 2u];
    hf_t *f;
    const double pitch = (double)ultra_rw_slot_us((uint8_t)rate);
    double rel = g_rel, ret = 120.0;
    if (g_mixed) {                                         /* [the stand-in: answer index 0] [this board: the given index] */
        ultra_slot_poll(s, (uint8_t)(rnd() & 0xFFu), 0, (uint8_t)r, (uint8_t)page, (uint8_t)rate, 0, 0);
        ultra_slot_poll(s + 32, (uint8_t)(rnd() & 0xFFu), (uint8_t)index, (uint8_t)r, (uint8_t)page, (uint8_t)rate, 0, 0);
        f = hub_ultra(s, 2, ULTRA_HDR_RW, 0.0);
        rel = stand_in_release();                          /* the stand-in turns around: its output transceiver stops driving the pair (until then, with
                                                              the old 100-150 us re-arm, it held this board's input LOW; with the 200 us re-arm it forwards
                                                              the host's release) */
        ret = 40.0;                                        /* ... and is in forward mode again W .. W + 80 us (LOW) */
    } else {
        ultra_slot_poll(s, (uint8_t)(rnd() & 0xFFu), (uint8_t)index, (uint8_t)r, (uint8_t)page, (uint8_t)rate, 0, 0);
        f = hub_ultra(s, 1, ULTRA_HDR_RW, 0.0);
    }
    f->index = index;
    f->rate = rate;
    f->rcount = r;
    set_samples(t2s(f->t1 + rel), t2s(f->t1 + 1000.0 + pitch * r + ret));      /* RO: the pair idles HIGH until the retake */
    hub_t = f->t1 + 1000.0 + pitch * r + 120.0 + 220.0 + urnd(0, 80);            /* the host holds off >= W + 200 us */
    return f;
}

static void hub_dark(uint32_t n)
{
    uint32_t i;
    uint8_t s[ULTRA_SLOT_BYTES];
    ultra_slot_dark(s);
    for (i = 0; i < n; i++) {
        (void)hub_ultra1(s, 0, gap());
    }
}

static void hub_nop(uint32_t n, double every_us)
{
    uint32_t i;
    uint8_t s[ULTRA_SLOT_BYTES];
    for (i = 0; i < n; i++) {
        ultra_slot_nop(s);
        (void)hub_ultra1(s, 0, gap());
        hub_t += every_us;
    }
}

static void hub_slot1(const uint8_t *s) { (void)hub_ultra1(s, 0, gap()); }

/* a pixel frame of a controller: npx GRB pixels at the WS2812 timing, a 300 us reset */
#define MAXPXF 256
static uint8_t pxf[MAXPXF][3000];
static uint32_t pxf_n[MAXPXF], npxf;
static void hub_pixels(uint32_t npx, double reset_us)
{
    uint32_t i;
    uint8_t *b = pxf[npxf % MAXPXF];
    for (i = 0; i < npx * 3u; i++) {
        b[i] = (uint8_t)(npxf * 31u + i * 7u + 1u);
    }
    pxf_n[npxf % MAXPXF] = npx * 3u;
    npxf++;
    hub_t = hub_bits(hub_t, b, npx * 24u, 0.40, 0.80) + reset_us;
}

/* the marker of a pixel-line hub (a HIGH of 20 us, a LOW of 10 us) and a NOP frame behind it */
static void hub_marker_frame(void)
{
    uint8_t s[ULTRA_SLOT_BYTES];
    hf_t *f;
    ultra_slot_nop(s);
    hub_t += 350.0;
    set_samples(t2s(hub_t), t2s(hub_t + 20.0));
    hub_t += 30.0;
    f = hub_ultra(s, 1, 0, 400.0);                            /* (a pixel-line hub waits 300 us; here a chain frame follows: room for the stream capture's decode) */
    f->marker = 1;
}

/* ------------------------------------------------------------------------------------------------ the receiver */
typedef struct {
    uint32_t T;
    uint32_t gap_max;                   /* the tick lateness, words (113 + 112 = up to 60 us) */
    uint32_t flash_from, flash_to;      /* a flash operation: the SRAM tick only, the main loop stalled */
    uint32_t flash_every, flash_len;    /* ... from flash_from on, one of flash_len words every flash_every words */
    int      steps;
    uint32_t cfg;                       /* CONFIG 75 at boot (the record's) */
    uint32_t no_zero;                   /* the negative control: DLINE_A_EDGES_ZERO ignored */
    uint32_t fault_at;                  /* an SPI OVR at this word */
    void   (*cmd)(uint32_t ms);         /* the main loop's scripted work, once a ms */
} sim_t;
static const sim_t *SIM;

static uint32_t g_blank, g_mux, g_de, de_end_w, de_start_w, g_burst_rate;
static uint32_t ready_pending, arm_at, sready_pending, sarm_at;
static uint32_t tim4_base;
static uint32_t ev_marker, ev_missed, n_ready, n_sends, n_zero;
static uint32_t g_fault_pend;
#define GATE_REARM_W MCU_US2W(ULTRA_RW_REARM_US)
static uint32_t g_gate_slot;            /* main: CHAIN decided (ud_line_apply) */
static uint32_t g_armed, g_armed_f;     /* the slot plan armed, at this falling-edge count (TIM2 counts on from it) */
static uint32_t g_relay;                /* the relay window turned: RO high-Z (the ring reads LOW) until FWD */
static uint32_t turn_k[64], fwd_k[64], nturn, nfwd, nbusy;
static uint32_t rearm_k[1024], nrearm;
static uint32_t blank_off_k, mux_k;
static uint32_t sends_w[256], sends_n;
typedef struct { uint32_t cls, bits, slots, end_w, flags, host; uint8_t slot[UPD_BLOCK_BYTES]; } rxrec_t;
#define MAXR 4096
static rxrec_t rxr[MAXR];
static uint32_t nrx;
static uint32_t answers_sched, cfgs_seen, fx_seen, g_paused_seen;
static uint32_t n_exp;
#define MAXFR 2048
#define MAXB (1024 * 5)
static uint8_t exp_fb[MAXFR][MAXB];
static uint32_t exp_bytes[MAXFR];

static uint32_t tim4_at(uint32_t k) { return (fallp[k] - tim4_base) & 0xFFFFu; }

/* main_ur_digital_m.c ud_line_apply */
static void line_apply(uint32_t act)
{
    if (act == UDC_A_CHAIN) {
        g_gate_slot = 1;
        dline_ch_start(&D, 1);
        udfx_chain(&U, 1);
    } else if (act == UDC_A_STREAM) {
        g_gate_slot = 0;
        g_armed = 0;
        dline_ch_stop(&D);
        dline_ch_hold(&D, 0);
        udfx_chain(&U, 0);
    } else if (act == UDC_A_QUIET) {
        dline_ch_hold(&D, 0);
    }
}

/* main_ur_digital_m.c ud_slot: the slot types of this board that the model acts on */
static void model_slot(const uint8_t *f, uint32_t host)
{
    if (f[0] == ULTRA_PIXEL_FX) {
        udfx_pixel_fx(&U, f, host);
        fx_seen++;
    } else if (f[0] == ULTRA_CONFIG && f[1] == 0u) {
        uint32_t j;
        for (j = 0; j < 9u; j++) {
            (void)udfx_config(&U, f[ULTRA_PAYLOAD + 3u * j], ultra_get16(f + ULTRA_PAYLOAD + 3u * j + 1u));
        }
        cfgs_seen++;
    }
}

/* Digest the owned tail and decoded bytes; ch_hn is a progress counter,
 * not a linear buffer length. The word-by-word build must print the same. */
static uint32_t g_digest = 2166136261u;
static void dig(uint32_t v) { g_digest = (g_digest ^ v) * 16777619u; }

/* main_ur_digital_m.c ud_chain_decode, step for step */
static void model_chain_decode(uint32_t k)
{
    udc_rx_t rx;
    uint32_t answer = 0, tx_w = 0, win_w = 0, act;
    uint32_t relay = 0, dir_w = 0, fwd_w = 0;
    const uint32_t end_w = D.ch_end, ms = k / 2250u;
    rxrec_t *r = nrx < MAXR ? &rxr[nrx++] : &rxr[MAXR - 1u];
    {
        uint32_t i, ts, tn;
        const uint16_t *sn = dline_ch_tail(&D, &ts, &tn);
        for (i = 0; i < tn; i++) dig(sn[(ts + i) & (DLINE_CH_TAIL - 1u)]);
        dig(D.ch_hn); dig(tn); dig(D.ch_bits); dig(D.ch_end); dig(D.ch_tx);
    }
    (void)udc_classify(&D, &rx, &P, ms);                          /* (frees the capture as soon as the slot is out) */
    {
        uint32_t i;
        for (i = 0; i < sizeof rx.slot; i++) dig(rx.slot[i]);
        dig(rx.cls); dig(rx.flags); dig(rx.host);
    }
    act = udc_frame(&C, &rx, ms);
    r->cls = rx.cls; r->bits = rx.bits; r->slots = rx.slots; r->end_w = end_w; r->flags = rx.flags; r->host = rx.host;
    memcpy(r->slot, rx.slot, sizeof r->slot);
    if (rx.cls != UDC_C_NONULTRA) {
        if (!(rx.flags & ULTRA_HDR_UPD)) {
            udfx_anchor(&U, rx.host, end_w);
            if (rx.cls == UDC_C_SLOT && !(rx.flags & ULTRA_HDR_EXT)) {
                if (!(rx.flags & ULTRA_HDR_RW)) {
                    model_slot(rx.slot, rx.host);
                } else if (ultra_poll_valid(rx.slot) == 0) {
                    C.windows++;
                    udc_window(rx.slot, end_w, U.wpu24, &tx_w, &win_w, &dir_w, &fwd_w);
                    if (U.r_have) {                           /* (main_ur_digital_m.c: the relay and the answer only with a measured rate) */
                        relay = 1;
                        if (rx.slot[ULTRA_POLL_INDEX] != ULTRA_POLL_RELAY_ONLY) {
                            answer = 1;
                            answers_sched++;
                            g_burst_rate = rx.slot[ULTRA_POLL_RATE];
                        }
                    } else {
                        C.answers_missed++;                   /* an unmeasured HSI would put the start outside the window */
                    }
                }
            }
        }
    }
    if (win_w == 0u && (rx.flags & ULTRA_HDR_RW)) {
        win_w = end_w + DLINE_CH_WIN_W;
    }
    dline_ch_done(&D, (int)answer, tx_w, win_w, relay, dir_w, fwd_w);
    line_apply(act);
}

static void apply(uint32_t a, uint32_t k)
{
    if (a & DLINE_A_BLANK_ON) {
        g_blank = 1;
    }
    if (a & DLINE_A_MUX_DELAYED) {
        g_mux = 1;
        mux_k = k;
    }
    if ((a & DLINE_A_BLANK_OFF) && !D.restart) {
        g_blank = 0;
        blank_off_k = k;
    }
    if (a & DLINE_A_EDGES_ZERO) {
        n_zero++;
        if (!SIM->no_zero) {
            tim4_base = fallp[k];
        }
    }
    if (a & DLINE_A_FRAME_READY) {                        /* a marker frame of the stream scanner: nobody here answers */
        sarm_at = k + 150u + (rnd() % 300u);
        sready_pending = 1;
    }
    if (a & DLINE_A_CH_READY) {
        arm_at = k + 60u + (rnd() % 120u);                /* the main loop classifies the frame 27 .. 80 us after this tick (the ready tick is 46 .. 106 us after the frame's last edge: the
                                                             compiled image frees the capture 126 .. 189 us after it, test_ur_digital_m_chain_budget.py) */
        ready_pending = 1;
        n_ready++;
    }
    if (a & DLINE_A_CH_MARKER) {
        ev_marker++;
        line_apply(udc_marker(&C));
    }
    if (a & DLINE_A_CH_MISSED) {
        ev_missed++;
        C.answers_missed++;
    }
    if (a & DLINE_A_CH_TURN) {                            /* main: the gate closed, rc_hw_relay(1): RO (DE still low) LOW = the line above still driven */
        g_armed = 0;
        if (nturn < 64u) {
            turn_k[nturn] = k;
        }
        nturn++;
        if (line[k] == 0u) {
            D.ch_arm = 0;
            C.relay_busy++;
            nbusy++;
        } else {
            g_relay = 1;
        }
    }
    if (a & DLINE_A_CH_FWD) {                             /* rc_hw_relay(0) */
        g_relay = 0;
        if (nfwd < 64u) {
            fwd_k[nfwd] = k;
        }
        nfwd++;
    }
    if (a & DLINE_A_SEND) {
        g_de = 1;
        de_start_w = k;
        de_end_w = k + 1u + (uint32_t)(((double)(3u + (330u << g_burst_rate)) * 2.25) + 0.5);   /* the lead byte + 32 bytes at the POLL's rate */
        if (sends_n < 256u) {
            sends_w[sends_n] = k;
        }
        sends_n++;
        n_sends++;
    }
}

static void restart_ring(uint32_t k)
{
    memset(ring, 0, sizeof ring);
    dline_restart(&D, (k + 1u) & M);
    udfx_restarted(&U, &D);
}

static void tick(uint32_t k, uint32_t ram)
{
    const uint32_t rx = (k + 1u) & M;
    const uint32_t e = tim4_at(k);
    uint32_t a;
    if (g_fault_pend && !ram) {                           /* main_ur_digital_m.c dl_tick: the OVR path restarts the ring and returns */
        g_fault_pend = 0;
        g_blank = 1;
        a = dline_fault(&D);
        apply(a, k);
        if (a & DLINE_A_RESTART) {
            restart_ring(k);
        }
        return;
    }
    if (ram) {
        a = dline_ram_tick(&D, ring, rx, (rx + 2u) & M, e);
        apply(a, k);
        if (!D.restart) {
            udfx_ram_tick(&U, ring, D.wpos);
        }
    } else {
        a = dline_tick(&D, ring, rx, (rx + 2u) & M, e);
        apply(a, k);
        if (a & DLINE_A_RESTART) {
            restart_ring(k);
        }
        /* main_ur_digital_m.c dl_tick: the slot plan re-armed after 200 us of quiet LOW (the chain scanner's run), never in a window, while it is armed
         * and no edge came since (TIM2 0, toggle mode), and only while the line is LOW now with no edge since the tick read TIM4 (the live check) */
        if (g_gate_slot && !D.ch_rel && D.ch_st == DLINE_CH_IDLE && D.ch_lz >= GATE_REARM_W && !(g_armed && fallp[k] == g_armed_f) &&
            !((g_de || g_relay) ? 0u : line[k])) {
            g_armed = 1;
            g_armed_f = fallp[k];
            if (nrearm < 1024u) {
                rearm_k[nrearm] = k;
            }
            nrearm++;
        }
        udfx_tick(&U, &D, ring);
    }
    if (ram) {
        g_armed = 0;                                      /* hw_cold.c flash_op: a toggling slot plan closes during flash work */
    }
}

static void ud_line_service(uint32_t ms)
{
    const uint32_t active = D.st.frames + D.st.marks + D.st.long_high;
    line_apply(udc_service(&C, udfx_line_cfg(&U), ms, active));
}

/* the main loop's decodes run whenever the frame is ready (the real loop spins; the render steps below are per tick) */
static void main_decodes(uint32_t k)
{
    if (sready_pending && k >= sarm_at) {                 /* (a marker frame: decoded, nothing to answer here) */
        dline_arm(&D, 0);
        sready_pending = 0;
    }
    if (ready_pending && k >= arm_at) {
        model_chain_decode(k);
        ready_pending = 0;
    }
}

static void main_loop(uint32_t k, uint32_t ms_tick)
{
    uint32_t i;
    if (U.paused) {
        g_paused_seen++;
    }
    if (D.ch_st == DLINE_CH_FRAME && D.ch_cap) {              /* main_ur_digital_m.c dl_main: the frame coming in, its slot as the words arrive */
        udc_stream(&D, &P);
    }
    for (i = 0; i < (uint32_t)SIM->steps; i++) {
        const uint32_t was_ready = U.e.ready;
        (void)udfx_render(&U);
        if (!was_ready && U.e.ready && n_exp < MAXFR) {
            exp_bytes[n_exp] = U.npx * U.chip.bpp;
            memcpy(exp_fb[n_exp], ux_pool, exp_bytes[n_exp]);
            n_exp++;
        }
    }
    if (ms_tick) {
        udfx_service(&U, &D, k / 2250u, 0, 256u);
        ud_line_service(k / 2250u);
        if (SIM->cmd) {
            SIM->cmd(k / 2250u);
        }
    }
}

/* the power-up of main.c: the record's CONFIG 75, then what the mode needs */
static void boot_modes(const sim_t *sc)
{
    udc_init(&C, udfx_line_cfg(&U), 0);
    memset(&P, 0, sizeof P);
    (void)sc;
    if (C.mode == UDC_CHAIN) {
        g_gate_slot = 1;
        dline_ch_start(&D, 1);
        udfx_chain(&U, 1);
    } else if (C.mode == UDC_UNDECIDED) {
        dline_ch_hold(&D, 1);
        dline_ch_start(&D, 0);
        udfx_chain(&U, 2);
    } else {
        udfx_chain(&U, 0);
    }
}

static uint32_t strip_high_blanked;                       /* samples where RO was HIGH while BLANK_N held (not a leak: the pin is LOW) */

static void simulate(const sim_t *sc)
{
    uint32_t k, next_tick = 100u, ntk = 0, next_ms = 0, prev = 0;
    if (sc->T > cap_w) {
        printf("scenario of %u words, the world holds %u\n", sc->T, cap_w);
        exit(3);
    }
    SIM = sc;
    memset(ring, 0, sizeof ring);
    g_blank = 1;
    g_mux = 0;
    g_de = 0;
    g_burst_rate = 0;
    ready_pending = sready_pending = 0;
    ev_marker = ev_missed = n_ready = n_sends = n_zero = 0;
    n_exp = 0;
    nrx = 0;
    tim4_base = 0;
    blank_off_k = mux_k = 0;
    sends_n = 0;
    answers_sched = cfgs_seen = fx_seen = g_paused_seen = 0;
    g_fault_pend = 0;
    strip_high_blanked = 0;
    g_gate_slot = g_armed = g_armed_f = g_relay = 0;
    nturn = nfwd = nbusy = nrearm = 0;
    dline_init(&D, 0, CAP);
    boot_modes(sc);
    for (k = 0; k < sc->T; k++) {
        const uint32_t in_flash = (sc->flash_to != 0u && k >= sc->flash_from && k < sc->flash_to) ||
                                  (sc->flash_len != 0u && k >= sc->flash_from && (k - sc->flash_from) % sc->flash_every < sc->flash_len);
        const uint32_t w = g_de || g_relay ? 0u : line[k];   /* RO as RX sees it: our own burst and the relay window (DE high: RO high-Z) read LOW */
        uint32_t x, f = fallp[k ? k - 1u : 0u], lv = prev;
        for (x = 0; x < 16u; x++) {                       /* TIM4: the falling edges, counted as they come */
            const uint32_t b = (w >> (15u - x)) & 1u;
            f += lv && !b;
            lv = b;
        }
        prev = lv;
        fallp[k] = f;
        ring[k & M] = (uint16_t)w;                        /* RX DMA */
        if (g_de && k >= de_end_w) {
            g_de = 0;
            dline_burst_done(&D, (k + 1u) & M);
        }
        out[k] = (uint16_t)(g_blank ? 0u : (g_mux ? ring[(k + 2u) & M] : w));   /* the strip's pin at time k */
        strip_high_blanked += g_blank && w;
        if (!in_flash && (ready_pending || sready_pending)) {
            main_decodes(k);
        }
        if (sc->fault_at != 0u && k == sc->fault_at) {
            g_fault_pend = 1;
        }
        if (k + 1u >= next_tick) {                        /* a tick: RX has written words < k + 1 */
            uint32_t jit;
            tick(k, in_flash);
            ntk++;
            jit = sc->gap_max > 113u ? rnd() % (sc->gap_max - 112u) : 0u;
            next_tick = 100u + (ntk * 225u) / 2u + jit;
            if (!in_flash) {
                main_loop(k, k >= next_ms);
                if (k >= next_ms) {
                    next_ms = k + 2250u;
                }
            }
        }
    }
}

/* ------------------------------------------------------------------------------------------------ the strip */
typedef struct { uint32_t start_w, nbits, low_before, exact, closed; uint8_t b[MAXB]; } dfr_t;
static dfr_t fr[MAXFR];
typedef struct { uint32_t s, t0h, t1h; } tim_t;
static tim_t g_tim[2];
static uint32_t g_ntim;
static uint32_t g_latch = 104u * 16u;

static uint32_t decode_strip(uint32_t w0, uint32_t w1)
{
    uint32_t n = 0, x, lvl = 0, len = 0, low = g_latch, hi = 0, inframe = 0, i;
    dfr_t *f = 0;
    for (x = w0 * 16u; x <= w1 * 16u; x++) {
        const uint32_t v = x < w1 * 16u ? (out[x >> 4] >> (15u - (x & 15u))) & 1u : 2u;
        if (v == lvl) {
            len++;
            continue;
        }
        if (lvl == 1u) {
            hi = len;
            if (!inframe) {
                if (n == MAXFR) {
                    break;
                }
                f = &fr[n++];
                memset(f, 0, sizeof *f);
                f->start_w = (x - len) >> 4;
                f->low_before = low;
                f->exact = (1u << g_ntim) - 1u;
                inframe = 1;
            }
            if (f->nbits < MAXB * 8u && hi > 19u) {
                f->b[f->nbits >> 3] |= (uint8_t)(0x80u >> (f->nbits & 7u));
            }
            for (i = 0; i < g_ntim; i++) {
                if (!(hi == g_tim[i].t0h || hi == g_tim[i].t1h)) {
                    f->exact &= ~(1u << i);
                }
            }
            f->nbits++;
        } else if (lvl == 0u && inframe) {
            if (len >= g_latch || v == 2u) {
                inframe = 0;
                f->closed = len >= g_latch;
            } else {
                for (i = 0; i < g_ntim; i++) {
                    if (len != g_tim[i].s - hi) {
                        f->exact &= ~(1u << i);
                    }
                }
            }
        }
        if (lvl == 0u) {
            low = len;
        }
        lvl = v;
        len = 1;
    }
    while (n > 0u && !fr[n - 1u].closed) {
        n--;
    }
    return n;
}

static void tim_one(uint32_t s, uint32_t t0h, uint32_t t1h) { g_ntim = 1; g_tim[0].s = s; g_tim[0].t0h = t0h; g_tim[0].t1h = t1h; }

static void make_rx(uint32_t chip, uint32_t npx, uint32_t cfg)
{
    udfx_init(&U);
    (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, chip);
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, npx);
    (void)udfx_config(&U, ULTRA_CFG_PX_FPS, 0);
    (void)udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, cfg);
}

static uint32_t g_allow_bits;                             /* strip frames of exactly this many bits are tolerated (Ultra frames on a stream board) */
static uint32_t g_allowed_n;

/* the next hub pixel frame (from jh on: the ones held before the stream passes never reach the strip) that equals b */
static int hub_match(const uint8_t *b, uint32_t nb, uint32_t *jh)
{
    uint32_t q;
    for (q = *jh; q < npxf && q < *jh + 64u; q++) {
        if (nb == pxf_n[q % MAXPXF] && memcmp(b, pxf[q % MAXPXF], nb) == 0) {
            *jh = q + 1u;
            return 1;
        }
    }
    return 0;
}

/* every strip frame must be a whole encoder frame (the rendered bytes, in order, exact pulses) or, when hubpx is set, one of the hub's pixel frames;
 * nothing else. Returns the number of local frames. */
static uint32_t check_strip(const char *what, uint32_t w0, uint32_t w1, uint32_t bytes0, uint32_t hubpx, uint32_t *n_hub)
{
    const uint32_t n = decode_strip(w0, w1);
    uint32_t k, jl = 0, jh = 0, nl = 0, nh = 0, bad = 0;
    for (k = 0; k < n; k++) {
        const dfr_t *f = &fr[k];
        const uint32_t nb = f->nbits / 8u;
        if (f->nbits % 8u == 0u && jl < n_exp && nb == exp_bytes[jl] && f->exact && memcmp(f->b, exp_fb[jl], nb) == 0) {
            jl++; nl++;
        } else if (hubpx && hub_match(f->b, nb, &jh)) {
            nh++;
        } else if (g_allow_bits && f->nbits == g_allow_bits) {
            g_allowed_n++;
        } else {
            bad++;
            if (bad < 4u) {
                printf("    strip frame %u at word %u: %u bits, exact %u, low_before %u: not the next local frame (%u of %u) nor a hub frame\n", k,
                       f->start_w, f->nbits, f->exact, f->low_before, jl, n_exp);
            }
        }
    }
    (void)bytes0;
    CHECK(bad == 0u, "%s: %u of %u strip frames are not whole encoder / hub frames (a leak or a cut)", what, bad, n);
    if (n_hub) {
        *n_hub = nh;
    }
    return nl;
}

/* nothing on the strip before the first frame: from the BLANK_N release to the first frame, every sample LOW */
static void check_quiet_before(const char *what, uint32_t from_w, uint32_t to_w)
{
    uint32_t w, high = 0;
    for (w = from_w; w < to_w; w++) {
        high += out[w] != 0u;
    }
    CHECK(high == 0u, "%s: %u HIGH words on the strip between words %u and %u", what, high, from_w, to_w);
}

/* ------------------------------------------------------------------------------------------------ helpers for scenarios */
static const uint8_t COL[9] = { 255, 80, 0, 0, 0, 40, 0, 200, 90 };
static uint8_t g_seq = 1;

static void hub_fx(uint8_t effect, uint8_t seg)
{
    ultra_pixel_fx_t f;
    uint8_t s[32];
    uint32_t i;
    memset(&f, 0, sizeof f);
    f.effect = effect;
    f.speed = 150;
    f.intensity = 128;
    f.palette = ULTRA_PX_PAL_DEFAULT;
    for (i = 0; i < 9u; i++) {
        f.c[i / 3u][i % 3u] = COL[i];
    }
    f.brightness = 255;
    ultra_slot_pixel_fx(s,0, seg, g_seq++, &f);
    hub_slot1(s);
}

static void hub_config(uint32_t id, uint32_t value)
{
    uint8_t s[32], ids[1], k;
    uint16_t v[1];
    ids[0] = (uint8_t)id;
    v[0] = (uint16_t)value;
    k = g_seq++;
    ultra_slot_config(s,0, k, ids, v, 1);
    hub_slot1(s);
}

static void world_begin(uint32_t T_words, double hsi)
{
    world_reserve(T_words + 4096u);
    S = 36.0 * (1.0 + hsi);
    g_phase = 16u * (rnd() % 100u);
    nhf = 0;
    npxf = 0;
    hub_t = 2000.0;
    g_seq = 1;
    g_ntim = 0;
}

static sim_t simdef(uint32_t T, uint32_t cfg)
{
    sim_t sc;
    memset(&sc, 0, sizeof sc);
    sc.T = T;
    sc.gap_max = 135u + 112u;
    sc.steps = 2;
    sc.cfg = cfg;
    return sc;
}

/* ================================================================================================ the scenarios */

/* [1] every hub frame is captured raw while the encoder runs, whatever it is: slots, header, bits, t0 */
static uint32_t scenario_capture(double hsi, uint32_t gapmax, uint32_t no_zero, uint32_t verbose)
{
    const uint32_t T = 2250u * 330u;
    sim_t sc = simdef(T, ULTRA_PXLM_CHAIN);
    uint32_t i, ok = 0, total, bad_end = 0;
    world_begin(T, hsi);
    sc.gap_max = gapmax;
    sc.no_zero = no_zero;
    make_rx(0, 60, ULTRA_PXLM_CHAIN);
    tim_one(48, 13, 25);
    hub_t = 4000.0;
    hub_fx(FX_RAINBOW, 0);
    for (i = 0; i < 60u; i++) {                           /* frames of 1 .. 6 slots, random bytes with right CRCs, and DARK / NOP frames */
        uint8_t s[ULTRA_SLOT_BYTES * 8u];
        uint32_t n = 1u + rnd() % 6u + (g_mixed ? 1u : 0u), j;
        for (j = 0; j < n; j++) {
            uint32_t b;
            ultra_slot_begin(s + 32u * j, ULTRA_NOP, 0, 0, (uint8_t)rnd());
            for (b = 3; b < 30u; b++) {
                s[32u * j + b] = (uint8_t)rnd();
            }
            ultra_slot_seal(s + 32u * j);
        }
        if (i % 7u == 3u) {
            ultra_slot_dark(s);
        }
        (void)hub_ultra(s, n, 0, gap());
    }
    simulate(&sc);
    total = nhf;
    for (i = 0; i < total; i++) {
        const hf_t *f = &hf[i];
        const rxrec_t *r = i < nrx ? &rxr[i] : 0;
        if (!r) {
            continue;
        }
        if (verbose) {
            CHECK(r->cls != UDC_C_NONULTRA, "frame %u of %u slots: class %u (bits %u, truth %u)", i, f->slots, r->cls, r->bits, f->bits);
            CHECK(r->bits == f->bits, "frame %u: %u bits, not %u", i, r->bits, f->bits);
            CHECK(r->slots == f->slots, "frame %u: %u slots, not %u", i, r->slots, f->slots);
            CHECK(r->host == f->host, "frame %u: TIME %u, not %u", i, r->host, f->host);
            if (r->cls == UDC_C_SLOT) {
                CHECK(memcmp(r->slot, f->slot0, 32) == 0, "frame %u: the slot differs", i);
            }
            bad_end += (int32_t)(r->end_w - f->end_w) > 2 || (int32_t)(f->end_w - r->end_w) > 2;
        }
        ok += r->cls != UDC_C_NONULTRA && r->bits == f->bits && r->slots == f->slots && r->host == f->host;
    }
    if (verbose) {
        CHECK(nrx == total, "%u frames captured, the hub sent %u", nrx, total);
        CHECK(bad_end == 0u, "%u frames with t0 more than 2 words off the truth", bad_end);
        CHECK(C.mode == UDC_CHAIN && D.ch_on && U.chain, "the mode");
        printf("  HSI %+.1f %%, ticks up to %u us late: %u frames, %u classified exactly (bits, slots, TIME, slot bytes), t0 within 2 words\n", hsi * 100.0,
               (gapmax - 112u) * 16u / 36u, total, ok);
    }
    return total - ok;
}

static void test_capture(void)
{
    static const double hs[3] = { -0.022, 0.0, 0.022 };
    uint32_t h, g, bad;
    printf("[1] the raw capture while the encoder runs: 60 frames of 1-6 slots, three HSI errors, ticks late\n");
    for (h = 0; h < 3u; h++) {
        for (g = 0; g < 2u; g++) {
            g_name = "capture";
            (void)scenario_capture(hs[h], g ? 135u + 112u : 113u, 0, 1);
        }
    }
    g_name = "capture / no TIM4 zero";
    bad = scenario_capture(0.0, 135u + 112u, 1, 0);
    CHECK(bad > 0u, "the negative control: without the TIM4 zero the frames still all classified");
    printf("  negative control: without dline's TIM4 zero %u of 60 frames fail the length check (TIM4 counts on across the gaps)\n", bad);
}

/* [2] the answers: a POLL in an RW frame is answered inside its Ultra window slot, at any HSI, whatever came before; nothing of the window shows on
 * the strip or in the frames after it; a RELAY_ONLY POLL opens a window and gets no answer */
/* the first hub frame the board did not capture (its last edge matched within 4 words): its index, else nhf */
static uint32_t first_missing(void)
{
    uint32_t i, q;
    for (i = 0; i < nhf; i++) {
        int found = 0;
        for (q = 0; q < nrx && !found; q++) {
            found = (int32_t)(rxr[q].end_w - hf[i].end_w) <= 4 && (int32_t)(hf[i].end_w - rxr[q].end_w) <= 4;
        }
        if (!found) {
            return i;
        }
    }
    return nhf;
}

static void answer_run(double hsi, uint32_t rate, uint32_t gapmax, uint32_t lead_ms, const char *label, uint32_t strict)
{
    const uint32_t T = 2250u * (lead_ms + 140u);
    sim_t sc = simdef(T, ULTRA_PXLM_CHAIN);
    uint32_t i, window_polls = 0, n_idx[16], nidx = 0, ok_time = 0, bad_time = 0;
    const uint32_t known = lead_ms >= 1200u;                                      /* the clock's rate needs two anchors a second apart */
    uint32_t first_nop;
    double dt_min = 1e9, dt_max = -1e9;
    world_begin(T, hsi);
    sc.gap_max = gapmax;
    make_rx(0, 60, ULTRA_PXLM_CHAIN);
    tim_one(48, 13, 25);
    hub_t = 4000.0;
    hub_fx(FX_RAINBOW, 0);
    first_nop = nhf;
    (void)first_nop;
    while (hub_t < 1000.0 * (double)lead_ms) {                                     /* headers: the rate gets measured */
        hub_nop(1, 20000.0);
    }
    for (i = 0; i < 4u; i++) {                                                     /* a POLL for each answer index, R = 4 */
        hf_t *f = hub_poll(i, 4, rate, ULTRA_PAGE_STATUS);
        n_idx[nidx++] = (uint32_t)(f - hf);
        window_polls++;
        hub_nop(2, 700.0);
    }
    {
        uint8_t s[ULTRA_SLOT_BYTES];                                               /* RELAY_ONLY: a window, no answer */
        hf_t *f;
        ultra_slot_poll(s, 99, ULTRA_POLL_RELAY_ONLY, 3, ULTRA_PAGE_STATUS, (uint8_t)rate, 0, 0);
        f = hub_ultra1(s, ULTRA_HDR_RW, 0.0);
        f->index = ULTRA_POLL_RELAY_ONLY;
        f->rate = rate;
        f->rcount = 3;
        set_samples(t2s(f->t1 + (g_mixed ? stand_in_release() : 150.0)), t2s(f->t1 + 1000.0 + (double)ultra_rw_slot_us((uint8_t)rate) * 3.0 + (g_mixed ? 40.0 : 120.0)));
        hub_t = f->t1 + 1000.0 + (double)ultra_rw_slot_us((uint8_t)rate) * 3.0 + 120.0 + 260.0;
        window_polls++;
        hub_nop(3, 700.0);
    }
    {                                                                              /* a POLL for index 0 of 1 */
        hf_t *f = hub_poll(0, 1, rate, ULTRA_PAGE_ENERGY);
        n_idx[nidx++] = (uint32_t)(f - hf);
        window_polls++;
        hub_nop(3, 700.0);
    }
    simulate(&sc);
    CHECK(C.mode == UDC_CHAIN && C.nonultra == 0u, "%s: the mode %u, non-Ultra frames %u", label, C.mode, C.nonultra);
    CHECK(fx_seen > 0u && U.local && U.seg[0].run && n_exp > 8u, "%s: fx_seen %u, local %u, run %u, exp %u, npx %u, chain %u, hold %u, rec %u, direct %u, blank %u, mux %u", label,
          fx_seen, U.local, U.seg[0].run, n_exp, U.npx, U.chain, D.ch_hold, D.rec, D.direct, g_blank, g_mux);
    {
        const uint32_t m = first_missing();
        CHECK(nrx == nhf, "%s: %u frames captured, the hub sent %u (a window lost one; the first missing: %u, rw %u, %.1f us after the frame before)",
              label, nrx, nhf, m, m < nhf ? hf[m].rw : 0u, m > 0u && m < nhf ? hf[m].t0 - hf[m - 1u].t1 : 0.0);
    }
    CHECK(ev_marker == 0u, "%s: %u markers counted around the windows", label, ev_marker);
    CHECK(C.windows == window_polls, "%s: %u windows, %u POLLs", label, C.windows, window_polls);
    CHECK(answers_sched == (known ? 5u : 0u), "%s: %u answers scheduled, %u expected", label, answers_sched, known ? 5u : 0u);
    if (!known) {
        CHECK(sends_n == 0u && ev_missed == 0u && C.answers_missed == window_polls && !U.r_have, "%s: %u sent, %u late, %u windows missed for "
              "the unmeasured rate (%u expected: no answer, no relay), r_have %u", label, sends_n, ev_missed, C.answers_missed, window_polls, U.r_have);
    } else if (strict) {
        CHECK(sends_n == 5u && ev_missed == 0u, "%s: %u answers sent, %u missed; 5 / 0 expected", label, sends_n, ev_missed);
    } else {
        CHECK(sends_n + ev_missed == answers_sched, "%s: %u sent + %u missed, %u scheduled", label, sends_n, ev_missed, answers_sched);
    }
    CHECK(D.st.answered == sends_n, "%s: answered %u, sent %u", label, D.st.answered, sends_n);
    CHECK(D.state == DLINE_PASS || D.state == DLINE_HOLD, "%s: the window logic is in state %u", label, D.state);
    for (i = 0; i < sends_n && i < nidx && sends_n <= 5u; i++) {
        uint32_t j = 0, q;
        const hf_t *f;
        for (q = 0; q < nidx; q++) {                                                 /* the POLL this answer follows: the last one that had ended */
            if ((int32_t)(sends_w[i] - hf[n_idx[q]].end_w) > 0) {
                j = q;
            }
        }
        f = &hf[n_idx[j]];
        {
        const double pitch = (double)ultra_rw_slot_us((uint8_t)f->rate), si = 1000.0 + pitch * (double)f->index;
        const double start_us = ((double)(sends_w[i] + 1u) * 16.0 - (double)g_phase) / S - f->t1;   /* (the tick's end: DE rises) */
        const double early = si - start_us, late = start_us - si;
        (void)early;
        if (late < dt_min) dt_min = late;
        if (late > dt_max) dt_max = late;
        if (start_us >= si - (f->index == 0u ? ULTRA_RW_EARLY0_US : ULTRA_RW_EARLY_US) && start_us <= si + ULTRA_RW_LATE_US) {
            ok_time++;
        } else {
            bad_time++;
            CHECK(0, "%s: answer %u (index %u, rate %u) starts %.1f us after t0, its slot is %.1f (-%u / +%u)", label, i, f->index, f->rate, start_us, si,
                  f->index == 0u ? ULTRA_RW_EARLY0_US : ULTRA_RW_EARLY_US, ULTRA_RW_LATE_US);
        }
        }
    }
    {
        uint32_t nl = 0, nfr, w1 = sc.T - 3000u;
        nfr = decode_strip(0, w1);
        (void)nfr;
        nl = check_strip(label, 0, w1, 0, 0, 0);
        CHECK(nl > 8u, "%s: only %u local frames", label, nl);
        if (nfr > 0u) {
            check_quiet_before(label, blank_off_k + 4u, fr[0].start_w);
        }
    }
    if (!known) {
        printf("  %-34s no answer (%u POLLs counted missed), %u frames all captured, nothing on the strip\n", label, C.answers_missed, nrx);
    } else if (strict) {
        printf("  %-34s %u answers, starts %+5.1f .. %+5.1f us from their slots (allowed -5/-20 .. +80), %u frames all captured, nothing on the strip\n", label,
               sends_n, dt_min, dt_max, nrx);
    } else {
        printf("  %-34s %u sent, %u missed (late ticks), %u frames all captured, nothing on the strip\n", label, sends_n, ev_missed, nrx);
    }
    (void)ok_time; (void)bad_time;
}

static void test_answers(void)
{
    static const double hs[3] = { -0.022, 0.0, 0.022 };
    uint32_t h, r;
    char lab[64];
    printf("[2] the Ultra window: answers inside their slots at every rate and HSI error, no marker, nothing on the strip\n");
    for (h = 0; h < 3u; h++) {
        for (r = 0; r < 3u; r++) {
            g_name = "answers";
            sprintf(lab, "HSI %+.1f %%, rate %u, ticks <= 7 us", hs[h] * 100.0, r);
            answer_run(hs[h], r, 113u + 16u, 1300u, lab, 1);
        }
    }
    g_name = "answers / late ticks";
    answer_run(0.0, 1, 135u + 112u, 1300u, "ticks up to 60 us late", 0);
    g_name = "answers / rate unknown";
    answer_run(0.0, 0, 113u + 16u, 100u, "no rate measured yet (100 ms): no answer", 0);
}

/* [2b] Current release range: every valid POLL is answered in its slot.
 * A HIGH before the frame-end threshold cannot publish a chain capture. */
static void test_release(void)
{
    static const double rels[] = { 60.0, 100.0, 150.0 };
    static const double invalid[] = { 2.0, 5.0, 20.0, 40.0 };
    uint32_t i, r;
    char lab[64];
    printf("[2b] host release +60 .. +150 us: every POLL answered; sub-46 us releases fail closed\n");
    for (i = 0; i < sizeof rels / sizeof rels[0]; i++) {
        for (r = 0; r < 2u; r++) {
            g_name = "release";
            g_rel = rels[i];
            sprintf(lab, "release at +%.0f us, rate %u", rels[i], r * 2u);
            answer_run(0.0, r * 2u, 113u + 16u, 1300u, lab, 1);
        }
    }
    printf("     and behind a BOARD 2 stand-in (its output holds the board's input LOW until it turns around at +500 us, whenever the host let go)\n");
    g_mixed = 1;
    for (i = 0; i < sizeof rels / sizeof rels[0]; i += 2u) {
        g_name = "release / mixed";
        g_rel = rels[i];
        sprintf(lab, "mixed, host release at +%.0f us", rels[i]);
        answer_run(0.0, 0u, 113u + 16u, 1300u, lab, 1);
    }
    g_mixed = 0;
    for (i = 0; i < sizeof invalid / sizeof invalid[0]; i++) {
        sim_t sc = simdef(2250u * 1400u, ULTRA_PXLM_CHAIN);
        g_name = "invalid early release";
        world_begin(sc.T, 0.0);
        make_rx(0, 60, ULTRA_PXLM_CHAIN);
        tim_one(48, 13, 25);
        hub_t = 4000.0;
        hub_fx(FX_RAINBOW, 0);
        while (hub_t < 1300000.0) hub_nop(1, 20000.0);
        g_rel = invalid[i];
        hub_poll(0, 1, 0, ULTRA_PAGE_STATUS);
        simulate(&sc);
        CHECK(!sends_n && !C.windows && !answers_sched,
              "release +%.0f us: no answer/window (%u/%u/%u)",
              g_rel, sends_n, C.windows, answers_sched);
    }
    g_rel = 150.0;
}

/* [3] AUTO and the forced modes */
static uint32_t g_cmd_t[8], g_cmd_v[8], g_cmd_n, g_cmd_i;
static uint32_t g_mode_ms[4];                                 /* ms at which the mode first became UNDECIDED / STREAM / CHAIN (+1) */
static uint32_t g_done;                                       /* cmd_script: which of the scripted commands were given */
static uint32_t g_ws_undecided;                              /* ms at which the strip was asked back for the stream while the mode was undecided */
static void cmd_watch(uint32_t ms)
{
    if (C.mode < 3u && g_mode_ms[C.mode] == 0u) {
        g_mode_ms[C.mode] = ms + 1u;
    }
    g_ws_undecided += U.want_stream && C.mode == UDC_UNDECIDED;
    while (g_cmd_i < g_cmd_n && ms >= g_cmd_t[g_cmd_i]) {     /* scripted CONFIG 75 writes */
        (void)udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, g_cmd_v[g_cmd_i]);
        g_cmd_i++;
    }
}

static void auto_begin(uint32_t ms_total, uint32_t cfg, const char *name)
{
    memset(g_mode_ms, 0, sizeof g_mode_ms);
    g_cmd_n = g_cmd_i = 0;
    g_done = 0;
    g_name = name;
    world_begin(2250u * ms_total, 0.0);
    make_rx(0, 60, cfg);
    tim_one(48, 13, 25);
    hub_t = 3000.0;
}

static void auto_chain_case(void)
{
    sim_t sc = simdef(2250u * 500u, ULTRA_PXLM_AUTO);
    uint32_t nl, nfr;
    auto_begin(500, ULTRA_PXLM_AUTO, "auto / preamble");
    sc.cmd = cmd_watch;
    hub_dark(6);                                              /* the start-up preamble of a one-board line */
    hub_fx(FX_RAINBOW, 0);
    hub_nop(20, 5000.0);
    simulate(&sc);
    CHECK(C.mode == UDC_CHAIN && D.ch_on && U.chain, "mode %u", C.mode);
    CHECK(g_mode_ms[UDC_UNDECIDED] != 0u && g_mode_ms[UDC_CHAIN] != 0u, "undecided at ms %u, chain at ms %u", g_mode_ms[0], g_mode_ms[2]);
    CHECK(C.frames >= 20u && C.nonultra == 0u, "frames %u, non-Ultra %u", C.frames, C.nonultra);
    /* BLANK_N waited for the second frame and the strip stayed LOW until the first encoder frame */
    CHECK(blank_off_k > hf[1].end_w && blank_off_k < hf[1].end_w + 2250u * 3u, "BLANK_N released at word %u, the second frame ended at %u", blank_off_k, hf[1].end_w);
    nfr = decode_strip(0, sc.T - 3000u);
    nl = check_strip("auto / preamble", 0, sc.T - 3000u, 0, 0, 0);
    CHECK(nl > 30u && nfr == nl, "local frames %u of %u", nl, nfr);
    if (nfr) {
        check_quiet_before("auto / preamble", 0, fr[0].start_w);
    }
    printf("  AUTO, the preamble: CHAIN after the 2nd Ultra frame (BLANK_N held until then), %u local frames, strip LOW until the first\n", nl);
}

static void auto_stream_case(const char *name, uint32_t between, uint32_t saved)
{
    sim_t sc = simdef(2250u * 2200u, ULTRA_PXLM_AUTO);
    uint32_t nl, nh = 0, i, nfr;
    auto_begin(2200, ULTRA_PXLM_AUTO, name);
    if (!saved) {
        udfx_init(&U);                                  /* erased settings, including zero geometry */
    } else if (saved == 2u) {
        uint8_t rec[UX_REC_BYTES];
        U.stored_have = 1;
        U.stored[0].effect = FX_RAINBOW;
        U.stored[0].brightness = 255;
        (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, ULTRA_PXP_POWER_ON | ULTRA_PXP_ON_LOSS);
        (void)udfx_config(&U, ULTRA_CFG_PX_LOSS_TIMEOUT, 5);
        udfx_rec_pack(&U, 1, rec);
        udfx_init(&U);
        udfx_load(&U, rec);                             /* a reboot keeps geometry/source, re-detects AUTO */
        CHECK(!U.stream && !U.chain && U.cfg[ULTRA_CFG_PX_PIXELS - ULTRA_CFG_PX_FIRST] == 60u,
              "saved AUTO decision survived reboot or geometry was lost");
    }
    g_allow_bits = between ? 320u : 0u;                       /* (the Ultra frames among the pixels reach the strip of a stream board: by design) */
    sc.cmd = cmd_watch;
    for (i = 0; i < 40u; i++) {                               /* a controller: pixel frames every 30 ms (in the other case, Ultra NOPs between them) */
        if (between && (i & 1u)) {
            hub_nop(1, 100.0);
        }
        hub_pixels(100, 300.0);
        hub_t += 28000.0;
    }
    simulate(&sc);
    CHECK(C.mode == UDC_STREAM && !D.ch_on && !U.chain, "mode %u, ch_on %u", C.mode, D.ch_on);
    CHECK(g_mode_ms[UDC_STREAM] >= 500u && g_mode_ms[UDC_STREAM] < 1100u, "STREAM decided at ms %u (0.5 s of activity, 4 frames that are no Ultra frame)", g_mode_ms[UDC_STREAM]);
    nl = check_strip(name, 0, sc.T - 3000u, 0, 1, &nh);
    CHECK(nh >= 15u && !U.local && !U.want_local, "local %u, hub frames %u on the strip", nl, nh);
    CHECK(U.cfg[ULTRA_CFG_PX_SOURCE - ULTRA_CFG_PX_FIRST] == 0u, "AUTO rewrote saved SOURCE");
    nfr = decode_strip(0, sc.T - 3000u);
    for (i = 0; i < nfr; i++) {
        if (fr[i].start_w > 2250u * (g_mode_ms[UDC_STREAM] + 20u))
            CHECK(fr[i].nbits == 100u * 24u || (between && fr[i].nbits == 320u), "local frame after AUTO STREAM");
    }
    CHECK(blank_off_k > 2250u * 480u, "BLANK_N released at word %u, before the decision", blank_off_k);
    g_allow_bits = 0;
    printf("  %-30s STREAM at ms %u; %u hub frames on the strip whole, none before the decision\n", name, g_mode_ms[UDC_STREAM] - 1u, nh);
}

static void auto_marker_case(void)
{
    sim_t sc = simdef(2250u * 300u, ULTRA_PXLM_AUTO);
    auto_begin(300, ULTRA_PXLM_AUTO, "auto / marker");
    sc.cmd = cmd_watch;
    hub_marker_frame();
    hub_nop(10, 3000.0);
    simulate(&sc);
    CHECK(C.mode == UDC_STREAM && !D.ch_on, "mode %u", C.mode);
    CHECK(g_mode_ms[UDC_STREAM] != 0u && g_mode_ms[UDC_STREAM] < 8u, "STREAM at ms %u", g_mode_ms[UDC_STREAM]);
    printf("  AUTO, a marker frame: STREAM at ms %u\n", g_mode_ms[UDC_STREAM] - 1u);
}

/* silence is no evidence: after 2 s the strip is let go (the power-on effect may run), the mode stays open and the chain scanner goes on */
static void auto_quiet_case(void)
{
    sim_t sc = simdef(2250u * 2600u, ULTRA_PXLM_AUTO);
    auto_begin(2600, ULTRA_PXLM_AUTO, "auto / silence");
    sc.cmd = cmd_watch;
    simulate(&sc);
    CHECK(C.mode == UDC_UNDECIDED && C.quiet && D.ch_on && !D.ch_hold && !U.chain && U.undec, "mode %u quiet %u ch_on %u hold %u chain %u undec %u", C.mode, C.quiet, D.ch_on,
          D.ch_hold, U.chain, U.undec);
    CHECK(g_mode_ms[UDC_STREAM] == 0u && g_mode_ms[UDC_CHAIN] == 0u, "silence decided: STREAM at ms %u, CHAIN at ms %u", g_mode_ms[UDC_STREAM], g_mode_ms[UDC_CHAIN]);
    CHECK(!g_blank && g_mux && blank_off_k > 2250u * UDC_AUTO_QUIET_MS && blank_off_k < 2250u * (UDC_AUTO_QUIET_MS + 3u), "BLANK_N released at word %u (2 s = %u)", blank_off_k,
          2250u * UDC_AUTO_QUIET_MS);
    printf("  AUTO, silence: after 2 s the strip is let go (BLANK_N at word %u), the mode is still open, the chain scanner runs on\n", blank_off_k);
}

/* the line starts late (after the 2 s of silence): two Ultra frames still decide CHAIN. power_on: a stored effect has been running since 2 s (the encoder owns
 * the ring: not one line frame reaches the strip; STREAM_WINS is set and must not take the strip meanwhile); else the strip was dark and the first frames
 * (at most the two that decide) pass as pixels before the encoder takes the ring */
static void auto_late_case(uint32_t power_on, uint32_t start_ms, uint32_t extra_us, const char *name)
{
    sim_t sc = simdef(2250u * (start_ms + 700u), ULTRA_PXLM_AUTO);
    uint32_t nl, nfr, nh = 0, allowed;
    auto_begin(start_ms + 700u, ULTRA_PXLM_AUTO, name);
    sc.cmd = cmd_watch;
    (void)udfx_config(&U, ULTRA_CFG_PX_FPS, 60);
    if (power_on) {
        ultra_pixel_fx_t f;
        uint32_t i;
        memset(&f, 0, sizeof f);
        f.effect = FX_RAINBOW; f.speed = 150; f.intensity = 128; f.palette = ULTRA_PX_PAL_DEFAULT; f.brightness = 255;
        for (i = 0; i < 9u; i++) f.c[i / 3u][i % 3u] = COL[i];
        U.stored[0] = f;
        U.stored_have = 1;
        (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, ULTRA_PXP_POWER_ON | ULTRA_PXP_STREAM_WINS);
    }
    hub_t = 1000.0 * (double)start_ms + (double)extra_us;     /* (extra_us: where the line's frames fall against the 1 ms service) */
    if (!power_on) {
        hub_fx(FX_RAINBOW, 0);
    }
    hub_nop(40, 5000.0);
    g_allow_bits = power_on ? 0u : 320u;                      /* (the frames of the line before the board knows) */
    g_allowed_n = 0;
    g_ws_undecided = 0;
    simulate(&sc);
    CHECK(g_ws_undecided == 0u, "STREAM_WINS asked for the strip back %u times while the mode was undecided (two Ultra frames look like a stream taking it)", g_ws_undecided);
    CHECK(C.mode == UDC_CHAIN && D.ch_on && U.chain && !U.undec && U.local, "mode %u ch_on %u chain %u undec %u local %u", C.mode, D.ch_on, U.chain, U.undec, U.local);
    CHECK(g_mode_ms[UDC_CHAIN] >= start_ms && g_mode_ms[UDC_CHAIN] <= start_ms + 12u && g_mode_ms[UDC_STREAM] == 0u, "CHAIN at ms %u (the line starts at %u), STREAM at ms %u",
          g_mode_ms[UDC_CHAIN], start_ms, g_mode_ms[UDC_STREAM]);
    CHECK(C.nonultra == 0u && C.frames >= 38u, "frames %u, non-Ultra %u", C.frames, C.nonultra);
    nfr = decode_strip(0, sc.T - 3000u);
    nl = check_strip(name, 0, sc.T - 3000u, 0, 0, &nh);
    allowed = g_allowed_n;
    CHECK(power_on ? allowed == 0u : allowed <= 3u, "%u line frames reached the strip before the decision", allowed);
    CHECK(nl > 10u && nl + allowed == nfr, "local frames %u + line frames %u of %u", nl, allowed, nfr);
    CHECK(U.seg[0].run, "no effect runs");
    g_allow_bits = 0;
    if (!extra_us) {
        printf("  %-30s CHAIN at ms %u, %u local frames, %u line frame(s) on the strip before the decision\n", name, g_mode_ms[UDC_CHAIN] - 1u, nl, allowed);
    }
}

/* an input that is HIGH for a while at power-up (a cable not plugged in, a hub not powered: the idle line reads HIGH) and then LOW and silent: the
 * 2 s of silence count from the end of the HIGH, the strip is let go and the stored power-on effect runs (VERIFY-CHAIN N2) */
static void auto_quiet_high_case(void)
{
    sim_t sc = simdef(2250u * 3200u, ULTRA_PXLM_AUTO);
    ultra_pixel_fx_t f;
    uint32_t i, nl, nfr;
    auto_begin(3200, ULTRA_PXLM_AUTO, "auto / silence after a long HIGH");
    sc.cmd = cmd_watch;
    (void)udfx_config(&U, ULTRA_CFG_PX_FPS, 60);
    memset(&f, 0, sizeof f);
    f.effect = FX_RAINBOW; f.speed = 150; f.intensity = 128; f.palette = ULTRA_PX_PAL_DEFAULT; f.brightness = 255;
    for (i = 0; i < 9u; i++) f.c[i / 3u][i % 3u] = COL[i];
    U.stored[0] = f;
    U.stored_have = 1;
    (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, ULTRA_PXP_POWER_ON);
    set_samples(t2s(500.0), t2s(300000.0));                   /* HIGH from 0.5 ms to 300 ms, LOW and silent after it */
    simulate(&sc);
    CHECK(C.mode == UDC_UNDECIDED && C.quiet && D.ch_on && !D.ch_hold, "mode %u quiet %u ch_on %u hold %u", C.mode, C.quiet, D.ch_on, D.ch_hold);
    CHECK(D.st.long_high >= 1u && D.st.frames == 0u, "long HIGHs %u, frames %u", D.st.long_high, D.st.frames);
    CHECK(!g_blank && blank_off_k > 2250u * 2000u && blank_off_k < 2250u * 2500u, "BLANK_N released at word %u (2 s after the line's last activity, at ms 2000 .. 2500)", blank_off_k);
    CHECK(U.local && U.seg[0].run, "the stored effect does not run: local %u run %u", U.local, U.seg[0].run);
    nfr = decode_strip(0, sc.T - 3000u);
    nl = check_strip("auto / silence after a long HIGH", 0, sc.T - 3000u, 0, 0, 0);
    CHECK(nl > 20u && nfr == nl, "local frames %u of %u", nl, nfr);
    printf("  AUTO, the input HIGH for 300 ms then silent: the strip is let go at ms %u (2 s after the line's last activity), the stored effect runs (%u whole frames)\n", blank_off_k / 2250u, nl);
}

static void forced_case(uint32_t cfg, const char *name)
{
    sim_t sc = simdef(2250u * 300u, cfg);
    uint32_t nl, nfr;
    auto_begin(300, cfg, name);
    sc.cmd = cmd_watch;
    hub_fx(FX_RAINBOW, 0);
    hub_nop(30, 4000.0);
    simulate(&sc);
    if (cfg == ULTRA_PXLM_CHAIN) {
        CHECK(C.mode == UDC_CHAIN && D.ch_on && U.chain && U.local, "mode %u", C.mode);
        CHECK(g_mode_ms[UDC_UNDECIDED] == 0u, "a forced CHAIN was undecided");
        CHECK(blank_off_k < 2250u * 2u, "BLANK_N released at word %u: a forced CHAIN waits for no frame", blank_off_k);
        nfr = decode_strip(0, sc.T - 3000u);
        nl = check_strip(name, 0, sc.T - 3000u, 0, 0, 0);
        CHECK(nl > 20u && nfr == nl, "local frames %u of %u", nl, nfr);
        if (nfr) {
            check_quiet_before(name, 0, fr[0].start_w);
        }
        printf("  %-30s CHAIN from the first tick, BLANK_N at word %u, %u local frames\n", name, blank_off_k, nl);
    } else {
        CHECK(C.mode == UDC_STREAM && !D.ch_on && !U.chain && U.local && n_zero == 0u && n_ready == 0u,
              "mode %u ch_on %u local %u zero %u ready %u", C.mode, D.ch_on, U.local, n_zero, n_ready);
        printf("  %-30s STREAM, the chain scanner never ran (TIM4 untouched), nothing captured\n", name);
    }
}

static void test_auto(void)
{
    udc_t reboot;
    udc_rx_t rx;
    uint32_t i;
    printf("[3] the line mode: AUTO (the factory default) and the forced values\n");
    memset(&rx, 0, sizeof rx);
    udc_init(&reboot, ULTRA_PXLM_AUTO, 0);
    rx.cls = UDC_C_NONULTRA;
    (void)udc_service(&reboot, ULTRA_PXLM_AUTO, 1, 1);
    for (i = 0; i < 4u; i++) (void)udc_frame(&reboot, &rx, 2u + i);
    CHECK(udc_service(&reboot, ULTRA_PXLM_AUTO, 501, 4) == UDC_A_STREAM, "AUTO did not select STREAM");
    CHECK(udc_service(&reboot, ULTRA_PXLM_STREAM, 502, 4) == UDC_A_NONE &&
          udc_service(&reboot, ULTRA_PXLM_AUTO, 503, 4) == UDC_A_NONE && reboot.mode == UDC_STREAM,
          "forced STREAM -> AUTO must keep the decided mode");
    rx.cls = UDC_C_SLOT; rx.bits = ULTRA_SLOT_BITS + ULTRA_HDR_BITS;
    for (i = 0; i < 4u; i++) (void)udc_frame(&reboot, &rx, 504u + i);
    CHECK(reboot.mode == UDC_STREAM, "AUTO STREAM changed without reboot/force");
    udc_init(&reboot, ULTRA_PXLM_AUTO, 0);
    CHECK(reboot.mode == UDC_UNDECIDED, "reboot retained the AUTO decision");
    (void)udc_frame(&reboot, &rx, 1);
    CHECK(udc_frame(&reboot, &rx, 2) == UDC_A_CHAIN && reboot.mode == UDC_CHAIN, "reboot could not detect CHAIN");
    auto_chain_case();
    auto_stream_case("auto / fresh pixel stream", 0, 0);
    auto_stream_case("auto / saved LOCAL geometry", 0, 1);
    auto_stream_case("auto / reboot, stored loss FX", 0, 2);
    auto_stream_case("auto / Ultra between pixels", 1, 1);
    auto_marker_case();
    auto_quiet_case();
    auto_quiet_high_case();
    auto_late_case(0, 2300, 0, "auto / late start, dark");
    auto_late_case(1, 2300, 0, "auto / late start, power-on");
    auto_late_case(0, 2000, 0, "auto / start at 2.0 s");
    {                                                         /* the power-on effect with STREAM_WINS: the line's start at twelve places around the 1 ms service period */
        uint32_t q;
        for (q = 0; q < 12u; q++) {
            auto_late_case(1, 2300, 601u + q * 25u, "auto / late start, power-on, phases");
        }
        printf("  auto / late start, power-on   twelve more starts, 25 us apart around the 1 ms service: no line frame on the strip, CHAIN every time\n");
    }
    forced_case(ULTRA_PXLM_CHAIN, "forced CHAIN");
    forced_case(ULTRA_PXLM_STREAM, "forced STREAM");
}

/* [3b] the hub's queue: frames back to back at the default 300 us reset; the capture is free before the next one starts, whatever the tick (S2-2 of
 * VERIFY-CHAIN: the decode held it past the next frame's start and every other frame was lost). The cycle-accurate figure: test_ur_digital_m_chain_budget.py */
static void test_b2b(void)
{
    static const double hs[3] = { -0.022, 0.0, 0.022 };
    uint32_t h;
    printf("[3b] a burst of frames at the 300 us reset: every one captured and classified\n");
    for (h = 0; h < 3u; h++) {
        sim_t sc = simdef(2250u * 300u, ULTRA_PXLM_CHAIN);
        uint32_t i, nl, nfr;
        g_name = "b2b";
        memset(g_mode_ms, 0, sizeof g_mode_ms);
        g_cmd_n = g_cmd_i = 0;
        world_begin(2250u * 300u, hs[h]);
        make_rx(0, 60, ULTRA_PXLM_CHAIN);
        tim_one(48, 13, 25);
        sc.gap_max = 113u + 16u;
        hub_t = 4000.0;
        hub_fx(FX_RAINBOW, 0);
        hub_t += 20000.0;
        for (i = 0; i < 60u; i++) {
            uint8_t s[ULTRA_SLOT_BYTES];
            ultra_slot_nop(s);
            (void)hub_ultra1(s, 0, 300.0);
        }
        simulate(&sc);
        CHECK(nrx == nhf, "HSI %+.1f %%: %u of %u frames captured", hs[h] * 100.0, nrx, nhf);
        CHECK(C.frames == nhf && C.nonultra == 0u && D.st.cap_drops == 0u, "HSI %+.1f %%: classified %u of %u, %u non-Ultra, %u capture drops", hs[h] * 100.0, C.frames, nhf,
              C.nonultra, D.st.cap_drops);
        nfr = decode_strip(0, sc.T - 3000u);
        nl = check_strip("b2b", 0, sc.T - 3000u, 0, 0, 0);
        CHECK(nl > 10u && nfr == nl, "local frames %u of %u", nl, nfr);
        printf("  HSI %+.1f %%: %u of %u frames at a 300 us reset captured and classified, no capture drop\n", hs[h] * 100.0, nrx, nhf);
    }
}

/* [4] runtime changes, failures and foreign traffic in chain mode */
static uint32_t host_now(void) { return (uint32_t)floor(((double)D.wpos * 16.0 - (double)g_phase) / S + 0.5); }

static void send_fx_now(uint8_t effect, uint8_t seq)
{
    ultra_pixel_fx_t f;
    uint8_t s[32];
    uint32_t i;
    memset(&f, 0, sizeof f);
    f.effect = effect;
    f.speed = 150;
    f.intensity = 128;
    f.palette = ULTRA_PX_PAL_DEFAULT;
    for (i = 0; i < 9u; i++) {
        f.c[i / 3u][i % 3u] = COL[i];
    }
    f.brightness = 255;
    ultra_slot_pixel_fx(s,0, 0, seq, &f);
    udfx_pixel_fx(&U, s, host_now());
}

static uint32_t g_ms_a, g_ms_b, g_ms_c;                        /* the scripted commands: given once, at the first ms tick at or after (the ticks' jitter can skip an ms) */
static int g_trace;
static void cmd_script(uint32_t ms)
{
    cmd_watch(ms);
    if (g_trace && (ms % 10u == 0u || U.pause_req || U.paused)) {
        printf("    t%u ms: reconf %u pause_req %u paused %u npx %u bpp %u local %u want_stream %u e.state %u low_w %u e.w-wpos %d  lowrun %u dstate %u\n", ms, U.reconf,
               U.pause_req, U.paused, U.npx, U.chip.bpp, U.local, U.want_stream, U.e.state, U.e.low_w, (int)(U.e.w - D.wpos), dline_low_run(&D), D.state);
    }
    if (ms >= g_ms_a && !(g_done & 1u)) {
        g_done |= 1u;
        send_fx_now(FX_RAINBOW, 77);
    }
    if (ms >= g_ms_b && !(g_done & 2u)) {
        g_done |= 2u;
        (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, 90);
    }
    if (ms >= g_ms_c && !(g_done & 4u)) {
        g_done |= 4u;
        (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, ULTRA_PXC_SK6812_RGBW);
    }
}

static void test_modes(void)
{
    sim_t sc;
    uint32_t nl, nh, i;
    printf("[4] CONFIG 75 at run time, a re-configuration, a fault, a flash operation, foreign traffic in chain mode\n");

    /* STREAM -> CHAIN with the controller's pixel stream still running: whole hub frames, then whole local ones */
    sc = simdef(2250u * 700u, ULTRA_PXLM_STREAM);
    auto_begin(700, ULTRA_PXLM_STREAM, "stream -> chain");
    sc.cmd = cmd_script;
    g_allow_bits = 0;
    g_ms_a = 160; g_ms_b = g_ms_c = 0xFFFFFFFFu;
    g_cmd_t[0] = 150; g_cmd_v[0] = ULTRA_PXLM_CHAIN; g_cmd_n = 1;
    for (i = 0; i < 20u; i++) {
        hub_pixels(100, 300.0);
        hub_t += 28000.0;
    }
    simulate(&sc);
    CHECK(C.mode == UDC_CHAIN && D.ch_on && U.chain && U.local, "mode %u ch_on %u chain %u local %u", C.mode, D.ch_on, U.chain, U.local);
    CHECK(g_mode_ms[UDC_CHAIN] >= 151u && g_mode_ms[UDC_CHAIN] <= 155u, "CHAIN at ms %u", g_mode_ms[UDC_CHAIN]);
    nl = check_strip("stream -> chain", 0, sc.T - 3000u, 0, 1, &nh);
    CHECK(nl > 20u && nh == 0u, "local %u, hub frames %u", nl, nh);
    CHECK(C.nonultra > 0u && (C.flags & ULTRA_PXL_F_NONULTRA), "the pixel stream in chain mode is counted: %u frames, flags %02x", C.nonultra, C.flags);
    printf("  STREAM -> CHAIN at ms 150: %u whole hub frames, then %u whole local ones; %u non-Ultra frames counted\n", nh, nl, C.nonultra);

    /* CHAIN -> STREAM: the encoder finishes its frame, the hub's pixel frames pass */
    sc = simdef(2250u * 700u, ULTRA_PXLM_CHAIN);
    auto_begin(700, ULTRA_PXLM_CHAIN, "chain -> stream");
    sc.cmd = cmd_script;
    g_allow_bits = 320;                                       /* (the Ultra frames the hub sends until it follows the board: garbage on a stream board, by design) */
    g_ms_a = g_ms_b = g_ms_c = 0xFFFFFFFFu;
    g_cmd_t[0] = 250; g_cmd_v[0] = ULTRA_PXLM_STREAM; g_cmd_n = 1;
    hub_fx(FX_RAINBOW, 0);
    while (hub_t < 252000.0) {
        hub_nop(1, 3000.0);
    }
    for (i = 0; i < 12u; i++) {
        hub_pixels(100, 300.0);
        hub_t += 28000.0;
    }
    simulate(&sc);
    CHECK(C.mode == UDC_STREAM && !D.ch_on && !U.chain && U.local, "mode %u ch_on %u chain %u local %u", C.mode, D.ch_on, U.chain, U.local);
    nl = check_strip("chain -> stream", 0, sc.T - 3000u, 0, 1, &nh);
    CHECK(nl > 20u && nh == 0u, "local %u, hub frames %u", nl, nh);
    g_allow_bits = 0;
    printf("  CHAIN -> STREAM at ms 250: %u whole local frames, then %u whole hub frames\n", nl, nh);
}

static void test_reconf(void)
{
    sim_t sc = simdef(2250u * 600u, ULTRA_PXLM_CHAIN);
    uint32_t nl, nfr;
    g_name = "reconfigure";
    auto_begin(600, ULTRA_PXLM_CHAIN, "reconfigure");
    sc.cmd = cmd_script;
    g_ms_a = 20; g_ms_b = 200; g_ms_c = 350;
    hub_nop(60, 7000.0);
    simulate(&sc);
    g_ntim = 2;
    g_tim[0].s = 48; g_tim[0].t0h = 13; g_tim[0].t1h = 25;
    g_tim[1].s = 48; g_tim[1].t0h = 12; g_tim[1].t1h = 26;
    nfr = decode_strip(0, sc.T - 3000u);
    nl = check_strip("reconfigure", 0, sc.T - 3000u, 0, 0, 0);
    if (g_trace) {
        uint32_t x, y;
        for (y = 0; y < nfr && y < 400u; y++) {
            if (fr[y].nbits < 100u) {
                printf("    short strip frame %u at word %u: %u bits, low_before %u; the pulses (samples HIGH/LOW):", y, fr[y].start_w, fr[y].nbits, fr[y].low_before);
                for (x = fr[y].start_w * 16u; x < fr[y].start_w * 16u + 260u; x++) {
                    printf("%c", ((out[x >> 4] >> (15u - (x & 15u))) & 1u) ? '#' : '.');
                }
                printf("\n");
            }
        }
    }
    CHECK(!U.reconf && U.npx == 90u && U.chip.bpp == 4u && U.local && U.chain, "reconf %u npx %u bpp %u local %u", U.reconf, U.npx, U.chip.bpp, U.local);
    CHECK(g_paused_seen >= 2u, "the encoder paused %u times (npx at boot, the pixel count, the chip)", g_paused_seen);
    CHECK(nl > 40u && nfr == nl, "local frames %u of %u", nl, nfr);
    printf("  pixels 0 -> 60 (boot) -> 90 and WS2812B -> SK6812 RGBW while an effect runs: %u whole frames, the encoder paused %u ticks, the ring never the line's\n", nl,
           g_paused_seen);
}

static void test_fault(void)
{
    sim_t sc = simdef(2250u * 400u, ULTRA_PXLM_CHAIN);
    uint32_t nl, w_fault = 2250u * 150u, nfr, bad = 0, k;
    g_name = "fault";
    auto_begin(400, ULTRA_PXLM_CHAIN, "fault");
    sc.cmd = cmd_script;
    sc.fault_at = w_fault;
    g_ms_a = 20; g_ms_b = g_ms_c = 0xFFFFFFFFu;
    hub_nop(60, 3000.0);
    simulate(&sc);
    CHECK(D.st.faults >= 1u, "the fault was not seen: %u", D.st.faults);
    nfr = decode_strip(0, sc.T - 3000u);
    for (k = 0; k < nfr; k++) {                           /* the frame being sent when BLANK_N fell is cut: at most that one */
        const dfr_t *f = &fr[k];
        const uint32_t nb = f->nbits / 8u;
        bad += !(nb == exp_bytes[0] && f->exact);
    }
    CHECK(bad <= 1u, "%u strip frames with a wrong length or pulse after one fault", bad);
    nl = nfr;
    CHECK(nl > 40u, "local frames %u", nl);
    CHECK(C.mode == UDC_CHAIN && nrx > 40u, "after the restart: mode %u, %u frames captured", C.mode, nrx);
    printf("  an SPI overrun at 150 ms: BLANK_N LOW at once, the ring restarted, %u strip frames (at most one cut), %u hub frames captured, the encoder runs again\n", nl, nrx);
}

static void test_flash(void)
{
    sim_t sc = simdef(2250u * 400u, ULTRA_PXLM_CHAIN);
    uint32_t nl, nfr, late = 0;
    g_name = "flash";
    auto_begin(400, ULTRA_PXLM_CHAIN, "flash");
    U.r_have = 1;                                         /* (the clock's rate is measured: an unmeasured one answers no POLL) */
    sc.cmd = cmd_script;
    sc.flash_from = 2250u * 150u;
    sc.flash_to = 2250u * 166u;                           /* a 16 ms page erase */
    g_ms_a = 20; g_ms_b = g_ms_c = 0xFFFFFFFFu;
    hub_nop(15, 3000.0);
    {                                                     /* a POLL whose answer slot falls inside the operation */
        while (hub_t < 148000.0) {
            hub_nop(1, 500.0);
        }
        if (hub_t < 148900.0) {
            hub_t = 148900.0;
        }
        (void)hub_poll(0, 1, 0, ULTRA_PAGE_STATUS);   /* t0 ~149.4 ms: the frame is decoded before the operation (150 ms), its answer slot is inside it */
        hub_nop(30, 3000.0);
    }
    simulate(&sc);
    nfr = decode_strip(0, sc.T - 3000u);
    nl = check_strip("flash", 0, sc.T - 3000u, 0, 0, 0);
    late = ev_missed;
    if (g_trace) {
        uint32_t q;
        printf("    answers_sched %u windows %u arm %u tx %u ch_win %u wpos %u\n", answers_sched, C.windows, D.ch_arm, D.ch_tx, D.ch_win, D.wpos);
        for (q = 0; q < nhf; q++) {
            if (hf[q].rw) {
                printf("    RW frame %u: t1 %.1f us end word %u (flash %u..%u)\n", q, hf[q].t1, hf[q].end_w, sc.flash_from, sc.flash_to);
            }
        }
        for (q = 0; q < nrx; q++) {
            if (rxr[q].flags & ULTRA_HDR_RW) {
                printf("    decoded RW frame: class %u end word %u at k %u\n", rxr[q].cls, rxr[q].end_w, 0u);
            }
        }
    }
    CHECK(nl + 3u >= nfr && nfr > 40u, "strip frames %u, whole local frames %u", nfr, nl);
    CHECK(sends_n == 0u && ev_missed == 1u && C.answers_missed == 1u, "the answer inside the flash operation: sent %u, missed %u", sends_n, ev_missed);
    CHECK(nrx + 8u >= nhf, "captured %u of %u frames (those inside the 16 ms are lost)", nrx, nhf);
    CHECK(C.mode == UDC_CHAIN, "mode %u", C.mode);
    {
        const hf_t *last = &hf[nhf - 1u];
        const int32_t d = nrx ? (int32_t)(rxr[nrx - 1u].end_w - last->end_w) : 99;
        CHECK(d >= -2 && d <= 2, "the last hub frame is the last one captured (off by %d words)", d);
    }
    printf("  a 16 ms flash operation (SRAM tick only): %u whole strip frames, %u of %u hub frames captured, the answer inside it not sent (missed %u)\n", nl, nrx, nhf, late);
}

/* short flash operations (< 222 us) while a strip is lit and the line is quiet: the SRAM tick has no chain scan but the encoder goes on replacing the
 * words behind RX; the scanner must start again at the newest word, not read the encoder's symbols as a frame (S3-1 of VERIFY-CHAIN: 1 .. 8 phantom
 * frames per 40 operations, the non-Ultra counter, the fault flag and the LED's blinks, with no pixel stream on the line) */
static void test_flash_short(void)
{
    static const uint32_t lens[6] = { 60, 150, 300, 450, 600, 1200 };
    uint32_t q;
    printf("[4b] short flash operations while lit, the line quiet: no phantom frame\n");
    for (q = 0; q < 6u; q++) {
        sim_t sc = simdef(2250u * 160u, ULTRA_PXLM_CHAIN);
        g_name = "flash short";
        auto_begin(160, ULTRA_PXLM_CHAIN, "flash short");
        sc.cmd = cmd_script;
        sc.flash_from = 2250u * 40u;
        sc.flash_every = 2250u * 3u + 37u * q;
        sc.flash_len = lens[q];
        g_ms_a = 20; g_ms_b = g_ms_c = 0xFFFFFFFFu;
        hub_fx(FX_RAINBOW, 0);
        hub_nop(5, 3000.0);
        simulate(&sc);
        CHECK(C.nonultra == 0u && !(C.flags & ULTRA_PXL_F_NONULTRA), "%u-word operations: %u phantom frames, flags %02x", lens[q], C.nonultra, C.flags);
        CHECK(C.frames == nhf && nrx == nhf, "%u-word operations: %u of %u frames classified (%u captured)", lens[q], C.frames, nhf, nrx);
        CHECK(C.mode == UDC_CHAIN && U.local && U.seg[0].run, "mode %u local %u run %u", C.mode, U.local, U.seg[0].run);
        printf("  %4u-word operations (%3u us) every %u words: %u frames classified, %u phantom\n", lens[q], lens[q] * 4u / 9u, sc.flash_every, C.frames, C.nonultra);
    }
}

static void test_foreign(void)
{
    sim_t sc;
    uint32_t i, nl, nfr, markers;
    g_name = "foreign";
    /* a pixel stream and STREAM_WINS in chain mode: counted, never shown, the effect stays */
    sc = simdef(2250u * 500u, ULTRA_PXLM_CHAIN);
    auto_begin(500, ULTRA_PXLM_CHAIN, "foreign / pixel stream");
    sc.cmd = cmd_script;
    g_ms_a = 20; g_ms_b = g_ms_c = 0xFFFFFFFFu;
    for (i = 0; i < 20u; i++) {
        hub_nop(1, 5000.0);
        hub_pixels(100, 300.0);
        hub_t += 12000.0;
    }
    simulate(&sc);
    CHECK(C.mode == UDC_CHAIN && U.local && U.chain && U.seg[0].run, "mode %u local %u run %u", C.mode, U.local, U.seg[0].run);
    CHECK(C.nonultra >= 15u && (C.flags & ULTRA_PXL_F_NONULTRA), "non-Ultra frames %u flags %02x", C.nonultra, C.flags);
    nl = check_strip("foreign / pixel stream", 0, sc.T - 3000u, 0, 0, 0);
    nfr = decode_strip(0, sc.T - 3000u);
    CHECK(nl > 30u && nfr == nl, "local frames %u of %u: a hub frame reached the strip", nl, nfr);
    printf("  pixel frames in chain mode: %u counted as no Ultra frame, none on the strip (%u local frames), the effect runs on\n", C.nonultra, nl);

    /* the same with STREAM_WINS set from the start */
    sc = simdef(2250u * 500u, ULTRA_PXLM_CHAIN);
    auto_begin(500, ULTRA_PXLM_CHAIN, "foreign / stream wins");
    sc.cmd = cmd_script;
    g_ms_a = 20; g_ms_b = g_ms_c = 0xFFFFFFFFu;
    (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, ULTRA_PXP_STREAM_WINS);
    for (i = 0; i < 20u; i++) {
        hub_nop(1, 5000.0);
        hub_pixels(100, 300.0);
        hub_t += 12000.0;
    }
    simulate(&sc);
    CHECK(U.local && U.seg[0].run && !U.want_stream, "STREAM_WINS took the strip in chain mode: local %u run %u", U.local, U.seg[0].run);
    nl = check_strip("foreign / stream wins", 0, sc.T - 3000u, 0, 0, 0);
    nfr = decode_strip(0, sc.T - 3000u);
    CHECK(nl > 30u && nfr == nl, "local frames %u of %u", nl, nfr);
    printf("  STREAM_WINS set: the controller's frames do not take the strip in chain mode\n");

    /* the markers of a pixel-line hub: counted, the chain frames around them captured */
    sc = simdef(2250u * 400u, ULTRA_PXLM_CHAIN);
    auto_begin(400, ULTRA_PXLM_CHAIN, "foreign / markers");
    sc.cmd = cmd_script;
    g_ms_a = 20; g_ms_b = g_ms_c = 0xFFFFFFFFu;
    for (i = 0; i < 5u; i++) {
        hub_nop(6, 4000.0);
        hub_marker_frame();
    }
    hub_nop(10, 4000.0);
    simulate(&sc);
    markers = 0;
    for (i = 0; i < nhf; i++) {
        markers += hf[i].marker;
    }
    CHECK(markers == 5u && ev_marker == 5u && C.markers == 5u, "markers sent %u, seen %u, counted %u", markers, ev_marker, C.markers);
    CHECK(C.mode == UDC_CHAIN && (C.flags & ULTRA_PXL_F_MARKERS), "mode %u flags %02x", C.mode, C.flags);
    CHECK(nrx + markers == nhf, "%u chain frames captured, %u sent without the %u marker frames", nrx, nhf - markers, markers);
    if (g_trace) {
        uint32_t a = 0;
        for (i = 0; i < nhf; i++) {
            if (hf[i].marker) { continue; }
            if (a < nrx && (int32_t)(rxr[a].end_w - hf[i].end_w) >= -2 && (int32_t)(rxr[a].end_w - hf[i].end_w) <= 2) { a++; } else { printf("    hub frame %u (end word %u) was not captured\n", i, hf[i].end_w); }
        }
    }
    nl = check_strip("foreign / markers", 0, sc.T - 3000u, 0, 0, 0);
    nfr = decode_strip(0, sc.T - 3000u);
    CHECK(nl > 20u && nfr == nl, "local frames %u of %u", nl, nfr);
    printf("  5 marker frames of a pixel-line hub: all counted, the chain frames between them captured, nothing on the strip\n");
}

static void test_long(void)
{
    sim_t sc = simdef(2250u * 300u, ULTRA_PXLM_CHAIN);
    uint8_t s[ULTRA_SLOT_BYTES * 3u], payload[40];
    ultra_pixel_fx_t f;
    uint32_t i, nl, nfr;
    g_name = "long";
    auto_begin(300, ULTRA_PXLM_CHAIN, "long");
    sc.cmd = cmd_script;
    g_ms_a = g_ms_b = g_ms_c = 0xFFFFFFFFu;
    memset(&f, 0, sizeof f);
    f.effect = FX_RAINBOW; f.speed = 150; f.intensity = 128; f.palette = ULTRA_PX_PAL_DEFAULT; f.brightness = 255;
    for (i = 0; i < 9u; i++) f.c[i / 3u][i % 3u] = COL[i];
    ultra_slot_pixel_fx(s,0, 0, 5, &f);                     /* slot 0 of a 3-slot frame: the board is not the last (the phase-1 limit) */
    ultra_slot_nop(s + 32);
    ultra_slot_nop(s + 64);
    (void)hub_ultra(s, 3, 0, gap());
    ultra_slot_nop(s);
    (void)hub_ultra(s, 2, 0, gap());
    for (i = 0; i < 40u; i++) {
        payload[i] = (uint8_t)(i * 5u + 1u);
    }
    (void)hub_frame(payload, 40, ULTRA_HDR_UPD, gap());   /* an UPD frame: 384 bits */
    ultra_slot_dark(s);
    (void)hub_ultra(s, 1, 0, gap());
    ultra_slot_nop(s);
    hub_t += 50000.0;
    (void)hub_ultra(s, 1, 0, gap());
    hub_nop(20, 4000.0);
    simulate(&sc);
    CHECK(nrx == nhf, "%u of %u frames captured", nrx, nhf);
    CHECK(rxr[0].cls == UDC_C_SLOT && rxr[0].slots == 3u && rxr[1].slots == 2u && C.longs == 2u && (C.flags & ULTRA_PXL_F_LONG), "long frames: %u, %u, counted %u, flags %02x",
          rxr[0].slots, rxr[1].slots, C.longs, C.flags);
    CHECK(fx_seen == 1u && U.seg[0].run, "slot 0 of the 3-slot frame is this board's: fx_seen %u run %u", fx_seen, U.seg[0].run);
    CHECK(rxr[2].cls == UDC_C_UPDATE && (rxr[2].flags & ULTRA_HDR_UPD) && rxr[2].bits == 384u, "the UPD frame: class %u flags %02x bits %u", rxr[2].cls, rxr[2].flags, rxr[2].bits);
    CHECK(memcmp(rxr[2].slot, payload, UPD_BLOCK_BYTES) == 0, "the complete 40-byte update payload reaches the app validator");
    CHECK(rxr[3].cls == UDC_C_HDR, "the DARK slot's frame: class %u (a header and a length, no slot)", rxr[3].cls);
    CHECK(C.nonultra == 0u, "non-Ultra frames %u", C.nonultra);
    nl = check_strip("long", 0, sc.T - 3000u, 0, 0, 0);
    nfr = decode_strip(0, sc.T - 3000u);
    CHECK(nl > 20u && nfr == nl, "local frames %u of %u", nl, nfr);
    printf("  3- and 2-slot frames (counted: not the last board), a complete UPD frame (384 bits), a DARK slot: all classified, slot 0 acted on\n");
}

/* [5] the board behind a BOARD 2 stand-in on one Ultra line: the stand-in's gate removes the first slot (256 bits LOW) and forwards the rest through
 * two RS-485 hops and its AND gate (+-25 ns of skew per edge, 0.2 us late), turns around 500 us after t0 (the pair floats), is in forward mode
 * again at the window's end. The board's slot is the second of a two-slot frame; its POLL has answer index 1 of 2 (the stand-in is index 0). */
static void test_mixed(void)
{
    static const double hs[3] = { -0.022, 0.0, 0.022 };
    uint32_t h, r;
    char lab[64];
    printf("[5] a BOARD 3 node behind a BOARD 2 stand-in (two-slot frames, the gate and the hops distort every pulse)\n");
    g_mixed = 1;
    for (h = 0; h < 3u; h++) {
        for (r = 0; r < 3u; r++) {
            g_name = "mixed";
            sprintf(lab, "mixed, HSI %+.1f %%, rate %u", hs[h] * 100.0, r);
            answer_run(hs[h], r, 113u + 16u, 1300u, lab, 1);
        }
    }
    g_name = "mixed / capture";
    (void)scenario_capture(0.0, 135u + 112u, 0, 1);
    g_mixed = 0;
}

/* [0] the rising-edge decoders (ws_rise_frame / _step / _last) against the edge-pair decoders (ws_decode_frame / _last) on random frames: pulses 11 .. 15
 * samples for a '0', 23 .. 27 for a '1' (T0H 0.34 us / T1H 0.70 us at 36 MHz with the HSI's +-2.2 % and +-1.5 samples of edge distortion), periods 43 .. 47
 * samples, a LOW gap first, random cuts of the window for the last-64-bits decode, the step decoder fed in random chunks */
static void test_wsrise(void)
{
    static uint16_t words[1400];
    static uint8_t smp[20000];
    uint32_t it, bad_frame = 0, bad_step = 0, bad_last = 0, bad_cut = 0, total = 0;
    g_name = "ws rise";
    printf("[0] the rising-edge decoders against ws_decode_frame / ws_decode_last on random frames\n");
    for (it = 0; it < 3000u; it++) {
        const ws_timing_t tm = { 17u, 80u, 1800u };
        const uint32_t nbits = 64u + 8u * (rnd() % 40u), gap_s = 60u + rnd() % 300u, lead = rnd() % 16u;
        uint8_t bits[400], a[64], b[64];
        uint32_t n = 0, i, k, nw, v, cut;
        int ra, rb;
        for (i = 0; i < nbits; i++) {
            bits[i] = (uint8_t)(rnd() & 1u);
        }
        memset(smp, 0, sizeof smp);
        n = gap_s + lead;
        for (i = 0; i < nbits; i++) {
            const uint32_t hi = bits[i] ? 23u + rnd() % 5u : 11u + rnd() % 5u, per = 43u + rnd() % 5u;
            for (k = 0; k < hi; k++) smp[n + k] = 1;
            n += per;
        }
        n += 300;
        nw = n / 16u;
        for (i = 0; i < nw; i++) {
            v = 0;
            for (k = 0; k < 16u; k++) v = (v << 1) | smp[i * 16u + k];
            words[i] = (uint16_t)v;
        }
        /* the first nbits bits */
        memset(a, 0xAA, sizeof a); memset(b, 0x55, sizeof b);
        ra = ws_decode_frame(words, nw * 16u, (uint16_t)nbits, &tm, a);
        rb = ws_rise_frame(words, nw * 16u, (uint16_t)nbits, b);
        total++;
        bad_frame += ra != rb || ra != (int)nbits || memcmp(a, b, (nbits + 7u) / 8u) != 0;
        {                                                    /* the same in random chunks (the main loop's passes) */
            ws_rise_t st = { 0, 0, 0 };
            uint8_t c[64];
            uint32_t have = 0;
            int rc = 0;
            memset(c, 0, sizeof c);
            while (rc == 0 && have < nw) {
                have += 1u + rnd() % 130u;
                if (have > nw) have = nw;
                rc = ws_rise_step(&st, words, have, (uint16_t)nbits, c);
                if (have == nw) break;
            }
            bad_step += rc != (int)nbits || memcmp(c, b, (nbits + 7u) / 8u) != 0;
        }
        /* the last 64 bits of a window that starts anywhere (a bit may be cut) and ends in the LOW after the last bit */
        cut = rnd() % (nw > 300u ? nw - 260u : 1u);
        memset(a, 0xAA, sizeof a); memset(b, 0x55, sizeof b);
        ra = ws_decode_last(words + cut, (nw - cut) * 16u, &tm, a);
        rb = ws_rise_last(words, 0xFFFFu, cut, nw - cut, b);
        total++;
        if (ra >= 64) {
            bad_last += ra != rb || memcmp(a, b, 8) != 0;
        } else {
            bad_cut += ra != rb;
        }
        {                                                    /* the same window as a ring (indices wrap) */
            static uint16_t ringb[512];
            const uint32_t m = nw - cut < 512u ? nw - cut : 512u, st0 = rnd() % 512u;
            uint8_t d[64];
            for (i = 0; i < m; i++) ringb[(st0 + i) & 511u] = words[nw - m + i];
            memset(d, 0x33, sizeof d);
            rb = ws_rise_last(ringb, 511u, st0, m, d);
            ra = ws_decode_last(words + nw - m, m * 16u, &tm, a);
            if (ra >= 64) bad_last += ra != rb || memcmp(a, d, 8) != 0;
        }
    }
    CHECK(bad_frame == 0u, "%u of %u frames decode differently (ws_rise_frame)", bad_frame, total / 2u);
    CHECK(bad_step == 0u, "%u frames decode differently in chunks (ws_rise_step)", bad_step);
    CHECK(bad_last == 0u && bad_cut == 0u, "%u windows differ in the last 64 bits, %u in the count of a short window (ws_rise_last)", bad_last, bad_cut);
    printf("  3000 random frames (64 .. 384 bits, pulse and period jitter), 6000 windows: first-bits decode, the same in chunks, and the last 64 bits agree with the edge-pair decoders\n");
}

/* ================================================================================================ main */
typedef struct { const char *name; void (*fn)(void); } scen_t;
/* [6] the board at any position: the relay window of every valid POLL and the slot plan's re-arm. POLLs for index 0..3 of R = 4, RELAY_ONLY of R = 3, index 0
 * of R = 1; the host's next frame >= W + 580 us (timing.md: W + max(200, 80 + 500)); hub = the host releasing at `rel`, or a BOARD 2 stand-in in front (it
 * turns around at +500 us); rel 0: nothing above ever releases (the turnaround finds the line driven: no relay, no answer) */
static uint32_t win_of(uint32_t k, const uint32_t *wins, uint32_t nw)
{
    uint32_t q, j = nw;
    for (q = 0; q < nw; q++) {
        if ((int32_t)(k - hf[wins[q]].end_w) > 0) {
            j = q;
        }
    }
    return j;
}

static void relay_run(double rel, uint32_t mixed, uint32_t rate, uint32_t gapmax, const char *label)
{
    const uint32_t lead_ms = 1300u, T = 2250u * (lead_ms + 160u);
    sim_t sc = simdef(T, ULTRA_PXLM_CHAIN);
    uint32_t i, nw = 0, wins[8], bad_turn = 0, bad_fwd = 0, bad_rearm = 0, bad_gap = 0, armed_frames = 0, chain_frames = 0;
    const double pitch = (double)ultra_rw_slot_us((uint8_t)rate);
    double turn_min = 1e9, turn_max = -1e9, fwd_min = 1e9, fwd_max = -1e9, rearm_min = 1e9;
    world_begin(T, 0.0);
    sc.gap_max = gapmax;
    make_rx(0, 60, ULTRA_PXLM_CHAIN);
    tim_one(48, 13, 25);
    hub_t = 4000.0;
    hub_fx(FX_RAINBOW, 0);
    while (hub_t < 1000.0 * (double)lead_ms) {
        hub_nop(1, 20000.0);
    }
    g_mixed = mixed;
    g_rel = rel > 0.0 ? rel : 1e9;
    for (i = 0; i < 6u; i++) {
        hf_t *f;
        if (i == 4u) {                                                             /* RELAY_ONLY: a window, no answer */
            uint8_t s[ULTRA_SLOT_BYTES];
            ultra_slot_poll(s, 99, ULTRA_POLL_RELAY_ONLY, 3, ULTRA_PAGE_STATUS, (uint8_t)rate, 0, 0);
            f = hub_ultra1(s, ULTRA_HDR_RW, 0.0);
            f->index = ULTRA_POLL_RELAY_ONLY;
            f->rate = rate;
            f->rcount = 3;
            if (rel > 0.0 || mixed) {
                set_samples(t2s(f->t1 + (mixed ? stand_in_release() : rel)), t2s(f->t1 + 1000.0 + pitch * 3.0 + (mixed ? 40.0 : 120.0)));
            }
        } else {
            f = hub_poll(i < 4u ? i : 0u, i < 4u ? 4u : 1u, rate, ULTRA_PAGE_STATUS);
        }
        wins[nw++] = (uint32_t)(f - hf);
        {
            const double W = f->t1 + 1000.0 + pitch * (double)f->rcount;
            hub_t = W + 580.0 + urnd(0.0, 80.0);
        }
        hub_nop(2, 700.0);
    }
    simulate(&sc);
    g_mixed = 0;
    g_rel = 150.0;
    CHECK(C.mode == UDC_CHAIN && C.windows == nw && nrx == nhf, "%s: mode %u, %u windows of %u, %u frames of %u", label, C.mode, C.windows, nw, nrx, nhf);
    CHECK(nturn == nw && nfwd == nw, "%s: %u turnarounds, %u forward-agains, %u windows", label, nturn, nfwd, nw);
    for (i = 0; i < nturn && i < 64u; i++) {                                       /* the turnaround at t0 + 500 us, by the next tick (<= 60 us late) */
        const uint32_t j = win_of(turn_k[i], wins, nw);
        const double us = ((double)(turn_k[i] + 1u) * 16.0 - (double)g_phase) / S - (j < nw ? hf[wins[j]].t1 : 0.0);
        turn_min = us < turn_min ? us : turn_min;
        turn_max = us > turn_max ? us : turn_max;
        bad_turn += j >= nw || us < 500.0 - 1.0 || us > 500.0 + (gapmax > 113u + 16u ? 6.0 + 50.0 + 60.0 : 62.0);
    }
    for (i = 0; i < nfwd && i < 64u; i++) {                                        /* forward mode again W .. W + 80 us (ULTRA_RW_END_US) */
        const uint32_t j = win_of(fwd_k[i], wins, nw);
        const hf_t *f = j < nw ? &hf[wins[j]] : 0;
        const double W = f ? f->t1 + 1000.0 + pitch * (double)f->rcount : 0.0;
        const double us = ((double)(fwd_k[i] + 1u) * 16.0 - (double)g_phase) / S - W;
        fwd_min = us < fwd_min ? us : fwd_min;
        fwd_max = us > fwd_max ? us : fwd_max;
        bad_fwd += !f || us < 0.0 || us > (gapmax > 113u + 16u ? 6.0 + 50.0 + 60.0 : (double)ULTRA_RW_END_US);   /* (ticks up to 60 us
                                                                                         late: beyond the budget's 10 us, by design later) */
    }
    for (i = 0; i < nrearm && i < 1024u; i++) {                                    /* every re-arm: >= 200 us of LOW at RO, not in a frame, not in a released HIGH */
        uint32_t k = rearm_k[i], j = k, q;
        while (j > 0u && fallp[j] == fallp[k]) {
            j--;
        }
        {
            const double low_us = (double)(k - j) * 16.0 / S;
            int inside = line[k] != 0u;
            for (q = 0; q < nhf && !inside; q++) {
                inside = k + 1u >= t2s(hf[q].t0) / 16u && k <= hf[q].end_w;
            }
            if (k > 2250u * 1000u) {
                rearm_min = low_us < rearm_min ? low_us : rearm_min;
            }
            bad_rearm += inside || low_us + 0.5 < (double)ULTRA_RW_REARM_US;
        }
    }
    for (i = 1; i < nhf; i++) {                                                    /* every chain frame after the first ones: the slot plan armed in its gap */
        uint32_t q, ok = 0;
        const uint32_t start = t2s(hf[i].t0) / 16u;
        if (start < 2250u * 1000u) {
            continue;
        }
        chain_frames++;
        for (q = 0; q < nrearm && q < 1024u; q++) {
            ok |= rearm_k[q] > hf[i - 1u].end_w && rearm_k[q] < start;
        }
        armed_frames += ok;
        bad_gap += !ok;
    }
    if (g_hold500) {                                                             /* a race with the board in front: every answer sent or given up busy */
        CHECK(sends_n + nbusy >= 5u && sends_n + nbusy <= nw && ev_missed == 0u, "%s: %u answers sent, %u busy turnarounds, %u missed (5 answers asked)",
              label, sends_n, nbusy, ev_missed);
    } else if (rel > 0.0 || mixed) {
        CHECK(nbusy == 0u && sends_n == 5u && ev_missed == 0u, "%s: %u busy turnarounds, %u answers sent (5), %u missed", label, nbusy, sends_n, ev_missed);
    } else {
        CHECK(nbusy == nw && sends_n == 0u && C.relay_busy == nw, "%s: the line above driven: %u busy of %u, %u answers sent (none), counter %u", label, nbusy, nw, sends_n,
              C.relay_busy);
    }
    CHECK(!bad_turn && !bad_fwd, "%s: turnarounds %.1f .. %.1f us after t0 (%u bad), forward again W + %.1f .. %.1f us (%u bad)", label, turn_min, turn_max, bad_turn,
          fwd_min, fwd_max, bad_fwd);
    CHECK(!bad_rearm && nrearm > 0u, "%s: %u re-arms, %u inside a frame / a released HIGH or under 200 us of LOW (min %.1f us)", label, nrearm, bad_rearm, rearm_min);
    CHECK(!bad_gap, "%s: %u of %u frames without the slot plan armed in their gap", label, bad_gap, chain_frames);
    CHECK(D.state == DLINE_PASS || D.state == DLINE_HOLD, "%s: the window logic in state %u", label, D.state);
    printf("  %s: turn +%.0f..%.0f us, forward W+%.0f..%.0f us, %u re-arms (>= %.0f us of LOW), %u/%u frames armed, %u answers, %u busy\n", label, turn_min, turn_max,
           fwd_min, fwd_max, nrearm, rearm_min, armed_frames, chain_frames, sends_n, nbusy);
}

static void test_relay(void)
{
    printf("[6] the relay window and the slot plan's re-arm (the board at any position of its line)\n");
    g_name = "relay";
    relay_run(60.0, 0, 0, 113u + 16u, "host release +60 us, rate 0");
    relay_run(150.0, 0, 2, 113u + 16u, "host release +150 us, rate 2");
    relay_run(150.0, 0, 1, 135u + 112u, "host release +150 us, rate 1, ticks up to 60 us late");
    relay_run(60.0, 1, 0, 113u + 16u, "behind a BOARD 2 stand-in (it forwards the host's release at +60 us)");
    relay_run(150.0, 1, 2, 113u + 16u, "behind a BOARD 2 stand-in (it forwards the host's release at +150 us), rate 2");
    g_hold500 = 1;
    relay_run(150.0, 1, 0, 113u + 16u, "behind a stand-in holding the line until its turnaround (+500 us): answered or busy");
    g_hold500 = 0;
    g_name = "relay / busy";
    relay_run(0.0, 0, 0, 113u + 16u, "the line above never released");
}

static const scen_t SCEN[] = {
    { "wsrise", test_wsrise },
    { "capture", test_capture },
    { "answers", test_answers },
    { "release", test_release },
    { "auto", test_auto },
    { "b2b", test_b2b },
    { "modes", test_modes },
    { "reconf", test_reconf },
    { "fault", test_fault },
    { "flash", test_flash },
    { "flash_short", test_flash_short },
    { "foreign", test_foreign },
    { "long", test_long },
    { "mixed", test_mixed },
    { "relay", test_relay },
};

int main(int argc, char **argv)
{
    g_trace = getenv("UDCHAIN_TRACE") != 0;
    if (getenv("UDCHAIN_SEED")) {                         /* another random sequence (tick jitter, gaps, phases): the scenarios must not depend on this one */
        rng = (uint32_t)strtoul(getenv("UDCHAIN_SEED"), 0, 0) | 1u;
    }
    setvbuf(stdout, NULL, _IONBF, 0);
    const char *pre = argc > 1 && strcmp(argv[1], "*") != 0 ? argv[1] : "";
    uint32_t i;
    for (i = 0; i < sizeof SCEN / sizeof SCEN[0]; i++) {
        if (strncmp(SCEN[i].name, pre, strlen(pre)) == 0) {
            SCEN[i].fn();
        }
    }
    printf("udchain: %ld checks, %d failed, capture digest %08x\n", g_checks, g_fail, g_digest);
    return g_fail != 0;
}
