/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The return channel (core/rc.c, the Diff-Power ultra receiver; included by test_main.c).
 *
 * Units: POLL parsing, the slot pitch, the last edge in a DIN copy, the schedule (rounding, wrap, the distance back),
 * the BRR correction, the answer framing.
 *
 * The chain: a hub and up to 8 receivers on one Diff-Power port, simulated on every RS-485 segment in 50 ns steps.
 *  - Each receiver has its own HSI error (up to +-2.2 %, its tick and its UART), the shared clock's rate as it measured
 *    it (+-200 ppm), its own tick phase, and the hop delays of its transceivers and cable (0.15-0.95 us).
 *  - It measures t0 as main.c does: the last-edge tick and the ring position read in it, the last edge's sample in the
 *    copy, rc_schedule. It then turns around, does the turnaround check, sends its burst (line HIGH, 3 us, the bytes
 *    at its own baud) and goes back to forward mode, on its own ticks.
 *  - Segment s (between the hub or receiver s-1 and receiver s) is driven by the one above it (the hub until its
 *    release; a receiver in forward mode: its output transceiver, LOW, the gate closed) and by receiver s's input
 *    transceiver while it answers (the relay node: its own TX while it sends, else what its output transceiver
 *    receives from segment s+1). Two drivers at different levels: a fight. No driver: bias LOW.
 *  - The hub decodes segment 0 as a UART at its nominal rate (line HIGH, start bit, 8 bits LSB first, stop bit),
 *    32 bytes per answer, checks the CRC and the content.
 * Checked over random chains: no fight, each receiver's timing inside the spec's windows (turn around 250-900 us, line
 * HIGH at s_i -20 / +80 us, its start bit 2-10 us later, LOW again before the next slot, forward again W..W+64 us,
 * before the hub's next frame at W + 200 us), every answer decoded by the hub, relay-only receivers silent.
 * Failures: a receiver that doesn't take part (its POLL lost, the clock not rated, a late main loop): the one behind it
 * stops after the turnaround check (the HIGH probe can fight; without the check, the relayed bursts further down fight
 * its forward driver), the hub gets the answers before it and retries the rest; an answer the hub receives damaged
 * (CRC): retried; 500 / 250 kbaud. Power events (core/pwrev.c): a hub reading POWER from 8 receivers with trips,
 * limit events and dips between its polls and lost answers / acknowledges never misses an event, and a latch clears
 * only with its counts. */
#include "../src/core/rc.h"
#ifndef RC_LEAD_BYTES
#define RC_LEAD_BYTES 0u                   /* (firmware before the lead byte: the burst is the 32-byte answer alone) */
#endif

/* ------------------------------------------------------------------------------------------------ units */
static void rc_poll_slot(uint8_t *s, uint8_t seq, uint8_t index, uint8_t slots, uint8_t page, uint8_t rate,
                         const uint8_t *arg, uint32_t nargs)
{
    uint32_t i;
    memset(s, 0, 32);
    s[0] = ULTRA_POLL;
    s[1] = (uint8_t)(ULTRA_LIGHT_BOARD << 4);
    s[2] = seq;
    s[RC_P_INDEX] = index;
    s[RC_P_SLOTS] = slots;
    s[RC_P_PAGE] = page;
    s[RC_P_RATE] = rate;
    for (i = 0; i < nargs && i < 23u; i++) {
        s[RC_P_ARG + i] = arg[i];
    }
    ultra_put16(s + 30, ultra_crc16(s, 30));
}

/* forward mode again W .. W + this (protocol 0.2.4: ULTRA_RW_END_US 80; review r6: the proposal's 60 can't hold a
 * 50 us tick with a 200 ppm rate error over the longest window) */
#define RCS_END_US ((double)ULTRA_RW_END_US)

static void test_rc_units(void)
{
    uint8_t s[32], a[32];
    uint16_t w[40];
    rc_sched_t sc;
    uint32_t i, nominal = 50u << 16;

    printf("return channel: POLL, schedule, last edge, BRR, answer\n");
    rc_poll_slot(s, 9, 2, 5, ULTRA_PAGE_STATUS, 0, 0, 0);
    CHECK(rc_poll_parse(s) == 0, "rc: a valid POLL");
    s[1] |= 0x03u;
    CHECK(rc_poll_parse(s) == 0, "rc: slot FLAGS bits are ignored");
    rc_poll_slot(s, 9, ULTRA_POLL_RELAY_ONLY, 5, 0x77, 2, 0, 0);
    CHECK(rc_poll_parse(s) == 0, "rc: relay only, any page, rate 2");
    rc_poll_slot(s, 9, 5, 5, 0, 0, 0, 0);
    CHECK(rc_poll_parse(s) != 0, "rc: index == R");
    rc_poll_slot(s, 9, 0, 0, 0, 0, 0, 0);
    CHECK(rc_poll_parse(s) != 0, "rc: R 0");
    rc_poll_slot(s, 9, 0, 17, 0, 0, 0, 0);
    CHECK(rc_poll_parse(s) != 0, "rc: R 17");
    rc_poll_slot(s, 9, 15, 16, 0, 0, 0, 0);
    CHECK(rc_poll_parse(s) == 0, "rc: index 15 of 16");
    rc_poll_slot(s, 9, 0, 1, 0, 3, 0, 0);
    CHECK(rc_poll_parse(s) != 0, "rc: rate 3 (not defined)");
    rc_poll_slot(s, 9, 0, 1, 0, 0, 0, 0);
    s[1] = 0x0Fu << 4 ^ 0x10u;
    CHECK(rc_poll_parse(s) != 0, "rc: LIGHT 0xE");
    s[1] = ULTRA_LIGHT_BOARD << 4;
    s[0] = ULTRA_SET;
    CHECK(rc_poll_parse(s) != 0, "rc: not a POLL");
    CHECK(rc_slot_us(0) == 460u && rc_slot_us(1) == 790u && rc_slot_us(2) == 1450u && rc_slot_us(1) ==
          ultra_rw_slot_us(1), "rc: pitches 460 / 790 / 1450 (0.2.4: with the lead byte), as ultra_rw_slot_us");

    memset(w, 0, sizeof w);
    CHECK(rc_last_edge(w, 40) == -1, "rc: no edge");
    w[10] = 0xFFC0u;                                     /* samples 160..169 high: the last high sample is 169 */
    CHECK(rc_last_edge(w, 40) == (int32_t)(40u * 16u - 170u), "rc: last edge mid-word (%d)", (int)rc_last_edge(w, 40));
    w[39] = 0x8000u;                                     /* sample 624 high */
    CHECK(rc_last_edge(w, 40) == 15, "rc: last edge in the last word (%d)", (int)rc_last_edge(w, 40));
    w[39] = 0x0001u;
    CHECK(rc_last_edge(w, 40) == 0, "rc: the copy ends on the edge");

    rc_poll_slot(s, 9, 3, 5, 0, 0, 0, 0);
    rc_schedule(&sc, 1000u, 0, nominal, s);
    CHECK(sc.dir_tick == 1010u && sc.check_tick == 1019u && sc.late_tick == 1018u &&
          sc.tx_tick == 1000u + (1000u + 3u * rc_slot_us(0) + 49u) / 50u &&
          sc.end_tick == 1000u + (1000u + 5u * rc_slot_us(0) + RC_END_US + 49u) / 50u,
          "rc: schedule at the nominal rate (%u %u %u %u %u)", sc.dir_tick - 1000u, sc.check_tick - 1000u,
          sc.late_tick - 1000u, sc.tx_tick - 1000u, sc.end_tick - 1000u);
    rc_schedule(&sc, 1000u, 1800, nominal, s);           /* t0 one tick before the read */
    CHECK(sc.dir_tick == 1009u && sc.end_tick == 1000u + (1000u + 5u * rc_slot_us(0) + RC_END_US + 49u) / 50u - 1u,
          "rc: t0 a tick back (%u %u)", sc.dir_tick - 1000u, sc.end_tick - 1000u);
    rc_schedule(&sc, 1000u, -40, nominal, s);            /* the edge 1.1 us after the read: 500 us is past tick 10 */
    CHECK(sc.dir_tick == 1011u, "rc: t0 just after the read (%u)", sc.dir_tick - 1000u);
    rc_schedule(&sc, 0xFFFFFFF0u, 0, nominal, s);
    CHECK(sc.dir_tick == 0xFFFFFFF0u + 10u &&
          sc.end_tick == 0xFFFFFFF0u + (1000u + 5u * rc_slot_us(0) + RC_END_US + 49u) / 50u, "rc: across the tick's wrap");
    rc_schedule(&sc, 0, 0, (uint32_t)(50.0 / 1.02 * 65536.0), s);   /* a 2 % fast HSI: more ticks per host us */
    CHECK(sc.end_tick == (uint32_t)ceil((1000.0 + 5.0 * rc_slot_us(0) + RC_END_US) * 1.02 / 50.0),
          "rc: a fast HSI's ticks (%u)", sc.end_tick);
    /* (review r6) forward again at W .. W + RCS_END_US for every t0 phase in the tick and a +-200 ppm error of the
     * measured rate, at the longest window (16 slots at 250 kbaud: W = 24,200 us); 3 us for the ISR's latency and a
     * 2.2 % slow HSI's longer tick */
    {
        double lo = 1e9, hi = -1e9, e;
        int32_t bk;
        int sgn;
        rc_poll_slot(s, 9, 0, 16, 0, 2, 0, 0);
        for (sgn = -1; sgn <= 1; sgn += 2) {
            for (bk = 0; bk < 1800; bk += 3) {
                rc_schedule(&sc, 1000u, bk, (uint32_t)(nominal * (1.0 + sgn * 200e-6)), s);
                e = (double)(sc.end_tick - 1000u) * 50.0 + bk / 36.0 - (1000.0 + 16.0 * rc_slot_us(2));
                lo = e < lo ? e : lo;
                hi = e > hi ? e : hi;
            }
        }
        CHECK(lo >= 0.0 && hi + 3.0 <= RCS_END_US, "rc: forward again W %+.1f .. %+.1f us (+-200 ppm, 24.2 ms)",
              lo, hi);
    }
    rc_poll_slot(s, 9, ULTRA_POLL_RELAY_ONLY, 16, 0, 2, 0, 0);
    rc_schedule(&sc, 0, 0, nominal, s);
    CHECK(sc.end_tick == (1000u + 16u * 1450u + RC_END_US + 49u) / 50u, "rc: relay only, 16 slots at 250 kbaud (%u)",
          sc.end_tick);

    CHECK(rc_brr(nominal, 0) == 144u && rc_brr(nominal, 1) == 288u && rc_brr(nominal, 2) == 576u, "rc: BRR nominal");
    CHECK(rc_brr((uint32_t)(50.0 / 1.02 * 65536.0), 0) == 147u, "rc: BRR for a 2 %% fast HSI (%u)",
          rc_brr((uint32_t)(50.0 / 1.02 * 65536.0), 0));
    CHECK(rc_brr((uint32_t)(50.0 / 0.979 * 65536.0), 0) == 141u, "rc: BRR for a 2.1 %% slow HSI (%u)",
          rc_brr((uint32_t)(50.0 / 0.979 * 65536.0), 0));
    CHECK(rc_brr(nominal / 2u, 0) == 144u && rc_brr(0xFFFFFFFFu, 1) == 288u, "rc: BRR for an implausible rate");
    /* (review r6) 500 / 250 kbaud at their own resolution: 576 x 1.015 = 584.6 (0.6.6: 146 x 4 = 584, 0.35 % off) */
    CHECK(rc_brr((uint32_t)(50.0 / 1.015 * 65536.0), 2) == 585u && rc_brr((uint32_t)(50.0 / 1.015 * 65536.0), 1) ==
          292u && rc_brr((uint32_t)(50.0 / 1.07 * 65536.0), 2) == 576u, "rc: BRR at 250 / 500 kbaud for a 1.5 %% fast "
          "HSI (%u %u), 7 %% implausible", rc_brr((uint32_t)(50.0 / 1.015 * 65536.0), 2),
          rc_brr((uint32_t)(50.0 / 1.015 * 65536.0), 1));

    for (i = 0; i < 32u; i++) {
        a[i] = (uint8_t)(i * 7u);
    }
    {
        static const uint8_t len[4] = { 3, 1, 2, 4 };
        const uint32_t v[4] = { 0x123456u, 0x1FFu, 0xABCDu, 0x89ABCDEFu };
        static const uint8_t want[10] = { 0x12, 0x34, 0x56, 0xFF, 0xAB, 0xCD, 0x89, 0xAB, 0xCD, 0xEF };
        uint8_t pk[12];
        memset(pk, 0x77, sizeof pk);
        rc_pack(pk, v, len, 4);
        CHECK(memcmp(pk, want, 10) == 0 && pk[10] == 0x77u, "rc: rc_pack (big-endian fields, the low bytes)");
    }
    rc_poll_slot(s, 0xC3, 4, 5, 0, 0, 0, 0);
    rc_answer(a, s, ULTRA_PAGE_POWER, ULTRA_RSP_EVENT | ULTRA_RSP_CONTENTION | 0xF0u, 0xBEEFu);
    CHECK(a[0] == 0xA5u && a[1] == ULTRA_PAGE_POWER && a[2] == 0x45u && a[3] == 0xC3u && a[4] == 0xBEu &&
          a[5] == 0xEFu && a[6] == 42u && ultra_get16(a + 30) == ultra_crc16(a, 30), "rc: answer framing");
}

/* ------------------------------------------------------------------------------------------------ the chain */
#define RCS_MAX     8
#define RCS_STEP    0.05                   /* us: 20 steps per bit at 1 Mbaud */
#define RCS_HOST_US 150.0                  /* the hub's sender releases this long after the frame end */
enum { RCS_NONE = 0, RCS_LOST, RCS_UNRATED, RCS_LATE };   /* why a receiver doesn't take part */

typedef struct {
    /* the receiver */
    double hsi;                            /* its clock: 1 + error */
    double phase;                          /* its first tick, us of real time */
    double hop;                            /* its input transceiver + cable, us (both ways) */
    double latency;                        /* the ISR's entry to the ring read / the GPIO write, us */
    uint32_t rate_q16;                     /* the shared clock's rate as measured */
    int fail;                              /* RCS_* */
    uint8_t poll[32];
    uint16_t uid16;
    /* what it did (us of real time; 0 = never) */
    double t0;                             /* the frame's last edge as it arrived here */
    double dir_on, check_at, check_end, tx_high, tx_start, tx_end, fwd_at, quiet_at;
    int took_part, quiet, sent, aborted;
    uint8_t ans[32];
} rcs_board_t;

typedef struct {
    int n, slots, rate, check;             /* check 0: no turnaround check (to show what it prevents) */
    rcs_board_t b[RCS_MAX];
    double t0;                             /* the hub's frame end (its last edge leaves) */
    uint32_t steps;
    uint8_t *node[RCS_MAX];                /* each receiver's relay node (its input DI), per step */
    uint32_t fights[RCS_MAX + 1], fight_steps, late_fights;
    uint8_t got[16][32];                   /* the hub's decoded answers by index */
    int got_n[16], hub_bytes_bad;
} rcs_t;

static double rcs_tick_time(const rcs_board_t *b, uint32_t k) { return b->phase + (double)k * 50.0 / b->hsi; }

/* a receiver's view: measure t0, schedule, act on its ticks (us of real time) */
static void rcs_board_run(rcs_t *c, int k)
{
    rcs_board_t *b = &c->b[k];
    rc_sched_t s;
    uint32_t edge;
    double hop_total = 0.0, read, back, tick_us = 50.0 / b->hsi, bit;
    int i;
    for (i = 0; i <= k; i++) {
        hop_total += c->b[i].hop;
    }
    b->t0 = c->t0 + hop_total;                           /* the frame's last edge, here */
    b->took_part = b->quiet = b->sent = b->aborted = 0;
    b->dir_on = b->check_at = b->tx_high = b->tx_start = b->tx_end = b->quiet_at = 0.0;
    if (b->fail != RCS_NONE) {
        return;                                          /* forward mode throughout */
    }
    /* the last-edge tick: the first tick after the edge (the edge counter has it), its ring read latency later */
    edge = (uint32_t)ceil((b->t0 + 0.05 - b->phase) / tick_us);
    read = rcs_tick_time(b, edge) + b->latency;
    back = (read - b->t0) * 36.0 * b->hsi;               /* samples (at the HSI's 36 MHz) from the edge to the read */
    rc_schedule(&s, edge, (int32_t)floor(back), b->rate_q16, b->poll);
    b->took_part = 1;
    b->dir_on = rcs_tick_time(b, s.dir_tick) + b->latency;
    b->check_at = rcs_tick_time(b, s.check_tick) + b->latency;
    b->check_end = b->check_at + (double)RC_PREAMBLE_CYC / (144.0 * b->hsi) + 0.1;   /* HIGH, read back, released */
    b->fwd_at = rcs_tick_time(b, s.end_tick) + b->latency;
    if (b->poll[RC_P_INDEX] != ULTRA_POLL_RELAY_ONLY) {
        b->sent = 1;
        b->tx_high = rcs_tick_time(b, s.tx_tick) + b->latency;
        b->tx_start = b->tx_high + (double)RC_PREAMBLE_CYC / (144.0 * b->hsi) + 0.1;
        bit = (double)rc_brr(b->rate_q16, b->poll[RC_P_RATE]) / (144.0 * b->hsi);
        b->tx_end = b->tx_start + (32.0 + RC_LEAD_BYTES) * 10.0 * bit;
    }
}

/* is the segment above receiver k driven by someone else at real time t (the hub, or receiver k-1 forward) */
static int rcs_upper_drives(const rcs_t *c, int k, double t)
{
    const rcs_board_t *u;
    if (k == 0) {
        return t < c->t0 + RCS_HOST_US;
    }
    u = &c->b[k - 1];
    return !u->took_part || t < u->dir_on || t >= u->fwd_at;
}

/* Assumption, NOT an electrical guarantee: an opposing LOW driver wins the read-back.
 * Model the HIGH probe and the time before the result shuts DE off; never apply future quiet retroactively. */
static void rcs_checks(rcs_t *c)
{
    int k;
    for (k = 0; k < c->n; k++) {
        rcs_board_t *b = &c->b[k];
        if (!b->took_part) {
            continue;
        }
        if (!c->check) {
            b->check_at = 0.0;
        }
        if (c->check && rcs_upper_drives(c, k, b->check_at + (double)RC_PREAMBLE_CYC / (144.0 * b->hsi))) {
            b->quiet = 1;                                /* stops driving upstream until the window's end */
            b->quiet_at = b->check_at + (double)RC_PREAMBLE_CYC / (144.0 * b->hsi);
            b->sent = 0;
            b->tx_high = b->tx_start = 0.0;
            b->aborted = 1;
        } else if (b->sent && rcs_upper_drives(c, k, b->tx_high + 2.9)) {
            b->quiet = 1;
            b->quiet_at = b->tx_start;
            b->sent = 0;
            b->aborted = 1;
        }
    }
}

static int rcs_tx_level(const rcs_t *c, const rcs_board_t *b, double t, double bit)
{
    int i, byte, pos;
    (void)c;
    if ((b->check_at > 0.0 && t >= b->check_at && t < b->check_end) ||
        (b->tx_high > 0.0 && t >= b->tx_high && t < b->tx_start)) {
        return 1;                                        /* the turnaround check's HIGH, an aborted burst's preamble:
                                                            driven even without an answer (gpt-protocol: always;
                                                            they reach the hub: claude-telemetry's UART test) */
    }
    if (!b->sent || t < b->tx_high || t >= b->tx_end) {
        return -1;                                       /* PB10 not driven */
    }
    if (t < b->tx_start) {
        return 1;
    }
    i = (int)((t - b->tx_start) / bit);
    byte = i / 10;
    pos = i % 10;
    if (pos == 0) {
        return 0;
    }
    if (pos == 9) {
        return 1;
    }
    return ((byte < (int)RC_LEAD_BYTES ? 0xFF : b->ans[byte - (int)RC_LEAD_BYTES]) >> (pos - 1)) & 1;
}

/* drives segment s (s = 1..n: below receiver s-1) from below: receiver s answering (its DE in) */
static int rcs_lower_drives(const rcs_t *c, int s, double t)
{
    const rcs_board_t *b = &c->b[s];
    return s < c->n && b->took_part && (!b->quiet || t < b->quiet_at) && t >= b->dir_on && t < b->fwd_at;
}

static void rcs_simulate(rcs_t *c, double t_end)
{
    int k;
    uint32_t i;
    c->steps = (uint32_t)((t_end - c->t0) / RCS_STEP) + 1u;
    c->late_fights = 0;
    for (k = 0; k <= RCS_MAX; k++) {
        c->fights[k] = 0;
    }
    for (k = c->n - 1; k >= 0; k--) {                    /* from the far end: each node needs the one below */
        rcs_board_t *b = &c->b[k];
        double bit = b->sent ? (double)rc_brr(b->rate_q16, b->poll[RC_P_RATE]) / (144.0 * b->hsi) : 1.0;
        int dly = (int)(c->b[k + 1 < c->n ? k + 1 : k].hop / RCS_STEP);
        c->node[k] = (uint8_t *)malloc(c->steps);
        for (i = 0; i < c->steps; i++) {
            double t = c->t0 + i * RCS_STEP;
            int tx = rcs_tx_level(c, b, t, bit), lvl, up, low;
            if (tx >= 0) {
                lvl = tx;
            } else {                                     /* the output transceiver's RO: segment k+1 */
                int s = k + 1;
                up = !b->took_part || t < b->dir_on || t >= b->fwd_at;      /* this receiver forward: drives LOW */
                low = rcs_lower_drives(c, s, t);
                if (up && low) {
                    int v = (i >= (uint32_t)dly) ? c->node[s][i - dly] : 0;
                    if (v != 0) {
                        c->fights[s]++;                  /* receiver s relays a HIGH into this one's LOW */
                        c->late_fights += t >= c->t0 + ULTRA_RW_FIRST_US;
                    }
                    lvl = 0;
                } else if (low) {
                    lvl = (i >= (uint32_t)dly) ? c->node[s][i - dly] : 0;
                } else {
                    lvl = 0;                             /* forward (driving LOW) or nobody: bias LOW */
                }
            }
            c->node[k][i] = (uint8_t)lvl;
        }
    }
}

/* the hub: segment 0 as a UART at its nominal rate */
static void rcs_hub_decode(rcs_t *c)
{
    double bit = 1.0 * (1u << c->b[0].poll[RC_P_RATE]), t;
    int dly = (int)(c->b[0].hop / RCS_STEP), idx, nb = 0, k;
    uint8_t buf[32 + RC_LEAD_BYTES];
    uint32_t i = 0;
    memset(c->got_n, 0, sizeof c->got_n);
    c->hub_bytes_bad = 0;
#define SEG0(j) ((c->n > 0 && rcs_lower_drives(c, 0, c->t0 + (j) * RCS_STEP) && (j) >= (uint32_t)dly) ? \
                 c->node[0][(j) - dly] : 0)
    while (i < c->steps) {
        /* wait for HIGH (a preamble), then for a start bit */
        while (i < c->steps && !SEG0(i)) i++;
        while (i < c->steps && SEG0(i)) i++;
        if (i >= c->steps) {
            break;
        }
        nb = 0;
        for (;;) {                                       /* bytes while start bits follow */
            uint8_t v = 0;
            uint32_t s0 = i, j;
            for (k = 0; k < 8; k++) {
                t = (1.5 + k) * bit / RCS_STEP;
                j = s0 + (uint32_t)t;
                if (j < c->steps && SEG0(j)) {
                    v |= (uint8_t)(1u << k);
                }
            }
            j = s0 + (uint32_t)(9.5 * bit / RCS_STEP);
            if (j >= c->steps || !SEG0(j)) {
                break;                                   /* no stop bit: the burst ended (line LOW) */
            }
            if (nb < (int)sizeof buf) {
                buf[nb] = v;
            }
            nb++;
            i = j;
            while (i < c->steps && SEG0(i) && i < j + (uint32_t)(2.0 * bit / RCS_STEP)) i++;
            if (i >= c->steps || SEG0(i)) {
                break;                                   /* idle HIGH: no more start bits */
            }
        }
        if (nb == (int)sizeof buf && buf[RC_LEAD_BYTES] == ULTRA_RSP_SYNC &&
            ultra_get16(buf + RC_LEAD_BYTES + 30) == ultra_crc16(buf + RC_LEAD_BYTES, 30)) {
            idx = buf[RC_LEAD_BYTES + 2] >> 4;
            memcpy(c->got[idx], buf + RC_LEAD_BYTES, 32);
            c->got_n[idx]++;
        } else if (nb >= 4) {
            c->hub_bytes_bad++;
        }
        while (i < c->steps && SEG0(i)) i++;
    }
#undef SEG0
}

/* The hub as a real UART on segment 0 (review r6): a start bit is a falling edge after >= 3/16 bit HIGH (an STM32 /
 * ESP32 style receiver at 16x), sampled mid-bit, the stop bit checked (LOW: a framing error, the byte kept with its
 * flag); the next start bit is searched from the stop bit's sample on. The hub's software: bytes before SYNC are
 * discarded, then 32 bytes without a framing error and a valid CRC are an answer (ULTRA_PROTOCOL 0.2.4: "discard to
 * SYNC, take 32 bytes, check them"). Unlike rcs_hub_decode it does not resynchronise on every HIGH: a frame started by
 * a turnaround check's pulse runs its 10 bits whatever comes. */
static void rcs_hub_uart(rcs_t *c)
{
    static uint8_t by[2048], fe[2048];
    double bit = 1.0 * (1u << c->b[0].poll[RC_P_RATE]);
    int dly = (int)(c->b[0].hop / RCS_STEP), k;
    uint32_t i = 0, hi = 0, need = (uint32_t)(3.0 / 16.0 * bit / RCS_STEP), n = 0, m;
    memset(c->got_n, 0, sizeof c->got_n);
    c->hub_bytes_bad = 0;
#define SEG0(j) ((c->n > 0 && (j) < c->steps && rcs_lower_drives(c, 0, c->t0 + (j) * RCS_STEP) && \
                  (j) >= (uint32_t)dly) ? c->node[0][(j) - dly] : 0)
    while (i < c->steps) {
        uint32_t s0, j;
        uint8_t v = 0;
        if (SEG0(i)) {
            hi++;
            i++;
            continue;
        }
        if (hi < need) {                                 /* LOW, no idle before it: no start bit */
            hi = 0;
            i++;
            continue;
        }
        s0 = i;
        if (SEG0(s0 + (uint32_t)(0.5 * bit / RCS_STEP))) {
            hi = 1;                                      /* a glitch, not a start bit */
            i = s0 + (uint32_t)(0.5 * bit / RCS_STEP) + 1u;
            continue;
        }
        for (k = 0; k < 8; k++) {
            if (SEG0(s0 + (uint32_t)((1.5 + k) * bit / RCS_STEP))) {
                v |= (uint8_t)(1u << k);
            }
        }
        j = s0 + (uint32_t)(9.5 * bit / RCS_STEP);
        if (j >= c->steps) {
            break;
        }
        if (n < sizeof by) {
            by[n] = v;
            fe[n] = (uint8_t)!SEG0(j);
            n++;
        }
        hi = SEG0(j) ? 1u : 0u;
        i = j + 1u;
    }
#undef SEG0
    for (m = 0; m + 32u <= n; m++) {                     /* the hub's software: to SYNC, 32 bytes, CRC */
        uint32_t q, ok = by[m] == ULTRA_RSP_SYNC && !fe[m];
        for (q = 1; ok && q < 32u; q++) {
            ok = !fe[m + q];
        }
        if (ok) {
            uint8_t a[32];
            for (q = 0; q < 32u; q++) {
                a[q] = by[m + q];
            }
            if (ultra_get16(a + 30) == ultra_crc16(a, 30)) {
                memcpy(c->got[a[2] >> 4], a, 32);
                c->got_n[a[2] >> 4]++;
                m += 31u;
            }
        }
    }
}

static void rcs_free(rcs_t *c)
{
    int k;
    for (k = 0; k < c->n; k++) {
        free(c->node[k]);
        c->node[k] = 0;
    }
}

/* one RW frame + window. idx[k]: receiver k's answer index (0xFF relay only); returns the window W (us) */
static double rcs_window(rcs_t *c, const uint8_t *idx, uint8_t page, uint8_t seq)
{
    int k;
    double w = ULTRA_RW_FIRST_US + (double)rc_slot_us((uint8_t)c->rate) * c->slots, t_end;
    c->t0 = 100.0;
    for (k = 0; k < c->n; k++) {
        rcs_board_t *b = &c->b[k];
        uint32_t i;
        rc_poll_slot(b->poll, seq, idx[k], (uint8_t)c->slots, page, (uint8_t)c->rate, 0, 0);
        memset(b->ans, 0, 32);
        for (i = 0; i < 24u; i++) {
            b->ans[ULTRA_RSP_DATA + i] = (uint8_t)(k * 31 + i * 7 + seq);
        }
        rc_answer(b->ans, b->poll, page, 0, b->uid16);
        rcs_board_run(c, k);
    }
    rcs_checks(c);
    t_end = c->t0 + w + 400.0;
    rcs_simulate(c, t_end);
    rcs_hub_decode(c);
    return w;
}

static void rcs_random_chain(rcs_t *c, int n, int slots, int rate)
{
    int k;
    memset(c, 0, sizeof *c);
    c->n = n;
    c->slots = slots;
    c->rate = rate;
    c->check = 1;
    for (k = 0; k < n; k++) {
        rcs_board_t *b = &c->b[k];
        b->hsi = 1.0 + urand(-0.022, 0.022);
        b->phase = urand(0.0, 50.0);
        b->hop = urand(0.15, 0.95);
        b->latency = urand(0.4, 1.6);
        b->rate_q16 = (uint32_t)(50.0 / b->hsi * (1.0 + urand(-200e-6, 200e-6)) * 65536.0);
        b->uid16 = (uint16_t)(0x1000u + (uint32_t)k * 0x111u);
    }
}

/* the spec's windows for each receiver (relative to its own t0), and the hub's next frame */
static int rcs_timing_ok(const rcs_t *c, double w, int k, char *why, size_t n)
{
    const rcs_board_t *b = &c->b[k];
    double pitch = rc_slot_us((uint8_t)c->rate), s_i, d;
    if (!b->took_part) {
        return 1;
    }
    d = b->dir_on - b->t0;
    if (d < ULTRA_RW_DIR_MIN_US || d > ULTRA_RW_DIR_MAX_US) {
        snprintf(why, n, "turn around at %.1f us", d);
        return 0;
    }
    d = b->check_at - b->t0;
    if (d < ULTRA_RW_DIR_MAX_US || d + 3.5 > ULTRA_RW_FIRST_US - 20.0) {
        snprintf(why, n, "turnaround check at %.1f us", d);
        return 0;
    }
    d = b->fwd_at - b->t0;
    if (d < w || d > w + RCS_END_US || b->fwd_at > c->t0 + w + ULTRA_RW_HOLDOFF_US) {
        snprintf(why, n, "forward again at %.1f us (W %.0f)", d, w);
        return 0;
    }
    if (b->sent) {
        s_i = ULTRA_RW_FIRST_US + pitch * b->poll[RC_P_INDEX];
        d = b->tx_high - b->t0 - s_i;
        if (d < -20.0 || d > 80.0) {
            snprintf(why, n, "line HIGH at s_i %+.1f us", d);
            return 0;
        }
        if (b->tx_start - b->tx_high < 2.0 || b->tx_start - b->tx_high > 10.0) {
            snprintf(why, n, "start bit %.1f us after HIGH", b->tx_start - b->tx_high);
            return 0;
        }
        if (b->tx_end - b->t0 > s_i + pitch - 30.0) {
            snprintf(why, n, "burst ends at s_i + %.1f us", b->tx_end - b->t0 - s_i);
            return 0;
        }
    }
    return 1;
}

static void test_rc_chain(void)
{
    static rcs_t c;
    uint8_t idx[RCS_MAX];
    char why[80];
    int trial, k, bad_timing = 0, fights = 0, missing = 0, extra = 0, wrong = 0, relay_sent = 0;
    double worst_hi = -1e9, worst_lo = 1e9, w;

    printf("return channel: chains of 1-8 receivers, 50 ns steps, HSI +-2.2 %%, the hub's UART\n");
    for (trial = 0; trial < 300; trial++) {
        int n = 1 + (int)rint_(RCS_MAX), rate = trial % 10 == 9 ? 1 + (int)rint_(2) : 0;
        int slots = n + (int)rint_((unsigned)(17 - n) < 3u ? (unsigned)(17 - n) : 3u);
        uint32_t perm = (uint32_t)rnd();
        rcs_random_chain(&c, n, slots, rate);
        for (k = 0; k < n; k++) {                        /* indices: chain positions, some relay only */
            idx[k] = (perm >> k) % 7u == 0u ? (uint8_t)ULTRA_POLL_RELAY_ONLY : (uint8_t)k;
        }
        w = rcs_window(&c, idx, (uint8_t)(trial % 12), (uint8_t)trial);
        for (k = 0; k <= n; k++) {
            fights += c.fights[k] != 0u;
        }
        for (k = 0; k < n; k++) {
            const rcs_board_t *b = &c.b[k];
            if (!rcs_timing_ok(&c, w, k, why, sizeof why)) {
                if (bad_timing++ < 5) {
                    printf("  receiver %d of %d (rate %d): %s\n", k, n, rate, why);
                }
            }
            if (b->sent) {
                double d = b->tx_high - b->t0 - (ULTRA_RW_FIRST_US + rc_slot_us((uint8_t)rate) * idx[k]);
                worst_hi = d > worst_hi ? d : worst_hi;
                worst_lo = d < worst_lo ? d : worst_lo;
            }
            if (idx[k] == ULTRA_POLL_RELAY_ONLY) {
                relay_sent += b->sent;
                continue;
            }
            if (c.got_n[idx[k]] != 1) {
                missing++;
            } else if (memcmp(c.got[idx[k]], b->ans, 32) != 0) {
                wrong++;
            }
        }
        for (k = n; k < 16; k++) {
            extra += c.got_n[k];
        }
        rcs_free(&c);
    }
    printf("  300 windows: line HIGH at s_i %+.1f .. %+.1f us\n", worst_lo, worst_hi);
    CHECK(bad_timing == 0 && fights == 0 && missing == 0 && wrong == 0 && extra == 0 && relay_sent == 0,
          "rc chain: timing %d, fights %d, missing %d, wrong %d, extra %d, relay-only sent %d", bad_timing, fights,
          missing, wrong, extra, relay_sent);

    /* a receiver that doesn't take part (its POLL lost, the clock not rated, a late main loop) */
    {
        static const int why_fail[3] = { RCS_LOST, RCS_UNRATED, RCS_LATE };
        int f, mode, got_before, got_after, quiet_ok, fights_check = 0, fights_nocheck = 0, retry_ok = 1;
        for (mode = 0; mode < 3; mode++) {
            for (f = 0; f < RCS_MAX; f++) {
                rcs_random_chain(&c, RCS_MAX, RCS_MAX, 0);
                for (k = 0; k < RCS_MAX; k++) {
                    idx[k] = (uint8_t)k;
                }
                c.b[f].fail = why_fail[mode];
                w = rcs_window(&c, idx, ULTRA_PAGE_STATUS, (uint8_t)(f + 1));
                got_before = got_after = 0;
                for (k = 0; k < RCS_MAX; k++) {
                    if (k < f) {
                        got_before += c.got_n[k] == 1 && memcmp(c.got[k], c.b[k].ans, 32) == 0;
                    } else {
                        got_after += c.got_n[k];
                    }
                }
                quiet_ok = f + 1 >= RCS_MAX || (c.b[f + 1].quiet && c.b[f + 1].aborted);
                for (k = 0; k <= RCS_MAX; k++) {
                    fights_check += (int)c.fights[k];
                }
                CHECK(c.late_fights == 0, "rc: no payload-slot fights after the probes (LOW-wins model)");
                CHECK(got_before == f && got_after == 0 && quiet_ok && c.hub_bytes_bad == 0,
                      "rc: receiver %d doesn't take part (%d): %d of %d before it, %d after, the next one quiet %d, "
                      "bad bursts %d", f, mode, got_before, f, got_after, quiet_ok, c.hub_bytes_bad);
                rcs_free(&c);
                /* the same without the turnaround check: the bursts from further down fight its forward driver */
                rcs_random_chain(&c, RCS_MAX, RCS_MAX, 0);
                c.check = 0;
                c.b[f].fail = why_fail[mode];
                (void)rcs_window(&c, idx, ULTRA_PAGE_STATUS, (uint8_t)(f + 1));
                for (k = 0; k <= RCS_MAX; k++) {
                    fights_nocheck += (int)c.fights[k];
                }
                rcs_free(&c);
                /* the hub's retry: the receivers from f on, indices 0.., receiver f fine again */
                rcs_random_chain(&c, RCS_MAX, RCS_MAX - f, 0);
                for (k = 0; k < RCS_MAX; k++) {
                    idx[k] = k < f ? (uint8_t)ULTRA_POLL_RELAY_ONLY : (uint8_t)(k - f);
                }
                (void)rcs_window(&c, idx, ULTRA_PAGE_STATUS, (uint8_t)(f + 100));
                for (k = f; k < RCS_MAX; k++) {
                    retry_ok &= c.got_n[k - f] == 1 && memcmp(c.got[k - f], c.b[k].ans, 32) == 0;
                }
                rcs_free(&c);
                for (k = 0; k < RCS_MAX; k++) {
                    idx[k] = (uint8_t)k;
                }
            }
        }
        printf("  one receiver out (lost POLL / unrated / late, each position): fight steps with the check %d, "
               "without %d\n", fights_check, fights_nocheck);
        CHECK(fights_check > 0 && fights_check < 24 * RCS_MAX * 1200 && fights_nocheck > 0 && retry_ok,
              "rc: bounded probe fights with the check (%d; "
              "without it %d), the retry gets the rest (%d)", fights_check, fights_nocheck, retry_ok);
    }

    /* an answer damaged on the way (a bit flipped at the hub): its CRC drops it, the retry has it */
    {
        int dropped, ok;
        rcs_random_chain(&c, 5, 5, 0);
        for (k = 0; k < 5; k++) {
            idx[k] = (uint8_t)k;
        }
        (void)rcs_window(&c, idx, ULTRA_PAGE_ENERGY, 7);
        rcs_free(&c);
        c.b[2].ans[ULTRA_RSP_DATA + 3] ^= 0x10u;         /* sent damaged: as a bit flipped on the line */
        rcs_simulate(&c, c.t0 + ULTRA_RW_FIRST_US + rc_slot_us(0) * 5.0 + 400.0);
        rcs_hub_decode(&c);
        dropped = c.got_n[2] == 0 && c.hub_bytes_bad == 1;
        rcs_free(&c);
        (void)rcs_window(&c, idx, ULTRA_PAGE_ENERGY, 8);
        ok = c.got_n[2] == 1 && memcmp(c.got[2], c.b[2].ans, 32) == 0;
        rcs_free(&c);
        CHECK(dropped && ok, "rc: a damaged answer is dropped (%d) and the retry has it (%d)", dropped, ok);
    }
}

/* ------------------------------------------------------------------------------------------------ power events */
/* A hub polls POWER from 8 receivers (pwrev.c each) every 1-5 s, with trips, limit events and dips at random moments,
 * answers lost both ways. Its rule: acknowledge the counts it read in its next POWER poll; a latch it saw is an event
 * to show. Checked: every event the receiver counted reaches the hub (its count deltas sum to the receiver's), the
 * latch clears only when the acknowledged counts match, an event after the read stays latched, a lost or repeated
 * acknowledge does no harm, EVENT is set exactly while a latch is. */
static void test_rc_power(void)
{
    pwrev_t p[RCS_MAX];
    uint16_t seen[RCS_MAX][4], ack[RCS_MAX][4];
    uint32_t events[RCS_MAX][4], hub_events[RCS_MAX][4];
    int have_ack[RCS_MAX], k, j, bad = 0, polls = 0, lost = 0, acks = 0, latch_bad = 0;
    uint32_t ms;
    memset(seen, 0, sizeof seen);
    memset(ack, 0, sizeof ack);
    memset(events, 0, sizeof events);
    memset(hub_events, 0, sizeof hub_events);
    memset(have_ack, 0, sizeof have_ack);
    printf("return channel: POWER latches and acknowledges, 8 receivers, 2 h of events and polls\n");
    for (k = 0; k < RCS_MAX; k++) {
        pwrev_init(&p[k], 0xFFFF0000u);
    }
    for (ms = 0xFFFF0000u; ms != 0xFFFF0000u + 7200000u; ms += 1u) {
        for (k = 0; k < RCS_MAX; k++) {
            pwrev_in_t in;
            uint32_t r = (uint32_t)(rnd() % 200000u);
            memset(&in, 0, sizeof in);
            in.limit_ma = 6000;
            in.factor = 65535u;
            in.i_ma = 3000;
            in.dips = p[k].in.dips;
            if (p[k].in.trip_cause != 0u && (uint32_t)(rnd() % 300u) != 0u) {
                in.trip_cause = p[k].in.trip_cause;      /* a trip holds ~0.3 s */
                in.retry_ms = 300u;
            } else if (r < 3u) {
                in.trip_cause = (uint8_t)(1u + r % 2u);
                in.trip_ma = 9000;
                in.retry_ms = 300u;
            } else if (r < 9u || (p[k].in.limiting && (uint32_t)(rnd() % 50u) != 0u)) {
                in.limiting = 1;                         /* a reduction for ~50 ms */
                in.factor = 50000u;
            }
            if (r == 9u) {
                in.dips = (uint16_t)(in.dips + 1u);      /* a supply dip */
            }
            pwrev_update(&p[k], ms, &in);
        }
        if (ms % 1000u != 0u) {
            continue;
        }
        for (k = 0; k < RCS_MAX; k++) {
            uint8_t arg[9], page[PWREV_PAGE_BYTES], latched, ev;
            uint16_t now[4];
            if ((uint32_t)(rnd() % 3u) != 0u) {
                continue;                                /* this receiver's POWER poll: every 1-5 s on average */
            }
            polls++;
            memset(arg, 0, sizeof arg);
            if (have_ack[k]) {
                arg[0] = ULTRA_PW_ARG_ACK;
                for (j = 0; j < 4; j++) {
                    ultra_put16(arg + 1 + 2 * j, ack[k][j]);
                }
            }
            if ((uint32_t)(rnd() % 10u) == 0u) {
                lost++;                                  /* the POLL never arrived: no ack, no answer */
                continue;
            }
            pwrev_ack(&p[k], arg);                       /* (main.c rc_arm: before the page) */
            acks += arg[0] != 0u;
            pwrev_page(&p[k], page);
            latched = pwrev_latched(&p[k]);
            ev = latched != 0u;
            latch_bad += page[ULTRA_PW_LATCHED] != latched || ((pwrev_state(&p[k]) & ULTRA_ST_POWER_EVENT) != 0u) != ev;
            if ((uint32_t)(rnd() % 10u) == 0u) {
                lost++;                                  /* the answer was lost: the hub acknowledges what it had */
                continue;
            }
            now[0] = (uint16_t)ultra_get16(page + ULTRA_PW_LIMIT_EVENTS);
            now[1] = (uint16_t)ultra_get16(page + ULTRA_PW_OVL_TRIPS);
            now[2] = (uint16_t)ultra_get16(page + ULTRA_PW_SHORT_TRIPS);
            now[3] = p[k].in.dips;                       /* (the SUPPLY page's count) */
            for (j = 0; j < 4; j++) {
                uint16_t d = (uint16_t)(now[j] - seen[k][j]);
                hub_events[k][j] += d;                   /* new events since the last read: shown */
                seen[k][j] = now[j];
                /* a latch the hub sees must be an event it hasn't acknowledged */
                if ((latched >> j) & 1u) {
                    latch_bad += now[j] == ack[k][j] && have_ack[k];
                }
                ack[k][j] = now[j];
            }
            have_ack[k] = 1;
        }
    }
    for (k = 0; k < RCS_MAX; k++) {
        uint8_t arg[9], page[PWREV_PAGE_BYTES];
        events[k][0] = p[k].limit_events;
        events[k][1] = p[k].ovl_trips;
        events[k][2] = p[k].short_trips;
        events[k][3] = p[k].in.dips;
        for (j = 0; j < 4; j++) {
            /* what the hub hasn't read yet is still latched */
            bad += seen[k][j] != (uint16_t)events[k][j] && !((pwrev_latched(&p[k]) >> j) & 1u);
        }
        /* a last read and its acknowledge (both arrive): every event counted by the hub, nothing latched */
        memset(arg, 0, sizeof arg);
        pwrev_page(&p[k], page);
        arg[0] = ULTRA_PW_ARG_ACK;
        ultra_put16(arg + 1, ultra_get16(page + ULTRA_PW_LIMIT_EVENTS));
        ultra_put16(arg + 3, ultra_get16(page + ULTRA_PW_OVL_TRIPS));
        ultra_put16(arg + 5, ultra_get16(page + ULTRA_PW_SHORT_TRIPS));
        ultra_put16(arg + 7, p[k].in.dips);
        for (j = 0; j < 3; j++) {
            hub_events[k][j] += (uint16_t)(ultra_get16(page + ULTRA_PW_LIMIT_EVENTS + 2 * j) - seen[k][j]);
        }
        hub_events[k][3] += (uint16_t)(p[k].in.dips - seen[k][3]);
        pwrev_ack(&p[k], arg);
        for (j = 0; j < 4; j++) {
            bad += (uint16_t)hub_events[k][j] != (uint16_t)events[k][j];
        }
        bad += pwrev_latched(&p[k]) != 0u || (pwrev_state(&p[k]) & ULTRA_ST_POWER_EVENT) != 0u;
    }
    printf("  %d POWER polls, %d lost (either way), %d acknowledges\n", polls, lost, acks);
    CHECK(bad == 0 && latch_bad == 0, "rc power: hub counts %d off, latch / EVENT %d wrong", bad, latch_bad);
}

/* (review r6) the turnaround check's pulses reach the hub: each is a HIGH of 3 us, then the line LOW, which a UART takes
 * as a start bit and runs as a 10-bit frame (a break). A check late in its tick (920-970 us) on one receiver and slot 0
 * early in its tick on another: at 500 / 250 kbaud that frame's stop bit falls into slot 0's first byte, so the hub's
 * UART is misframed and answer 0 is lost. Chains of 2-8 receivers, every one answering, the check pulses on the line,
 * the hub a real UART. */
#ifndef RCS_UART_TRIALS
#define RCS_UART_TRIALS 500                /* windows per rate (0.6.6: ~15 answers lost at 250 kbaud) */
#endif
static void test_rc_hub_uart(void)
{
    static rcs_t c;
    uint8_t idx[RCS_MAX];
    int rate, trial, k, lost[3] = { 0, 0, 0 }, sent[3] = { 0, 0, 0 }, fights = 0;
    double end_worst = -1e9;                   /* the burst's end against its slot's guard (s_i + pitch - 30 us) */
    printf("return channel: the hub as a real UART, the turnaround check's pulses on the line (lead bytes %u)\n",
           (unsigned)RC_LEAD_BYTES);
    for (rate = 0; rate < 3; rate++) {
        for (trial = 0; trial < RCS_UART_TRIALS; trial++) {
            int n = 2 + (int)rint_(RCS_MAX - 1);
            double w;
            rcs_random_chain(&c, n, n, rate);
            for (k = 0; k < n; k++) {
                idx[k] = (uint8_t)k;
            }
            w = ULTRA_RW_FIRST_US + (double)rc_slot_us((uint8_t)rate) * n;
            c.t0 = 100.0;
            for (k = 0; k < n; k++) {
                rcs_board_t *b = &c.b[k];
                uint32_t i;
                rc_poll_slot(b->poll, (uint8_t)trial, idx[k], (uint8_t)n, ULTRA_PAGE_STATUS, (uint8_t)rate, 0, 0);
                for (i = 0; i < 24u; i++) {
                    b->ans[ULTRA_RSP_DATA + i] = (uint8_t)(k * 31 + i * 7 + trial);
                }
                rc_answer(b->ans, b->poll, ULTRA_PAGE_STATUS, 0, b->uid16);
                rcs_board_run(&c, k);
            }
            rcs_checks(&c);
            rcs_simulate(&c, c.t0 + w + 400.0);
            rcs_hub_uart(&c);
            for (k = 0; k <= n; k++) {
                fights += c.fights[k] != 0u;
            }
            for (k = 0; k < n; k++) {
                sent[rate]++;
                lost[rate] += c.got_n[k] != 1 || memcmp(c.got[k], c.b[k].ans, 32) != 0;
                if (c.b[k].sent) {
                    double e = c.b[k].tx_end - c.b[k].t0 - (ULTRA_RW_FIRST_US + rc_slot_us((uint8_t)rate) * (k + 1u)
                                                             - 30.0);
                    end_worst = e > end_worst ? e : end_worst;
                }
            }
            rcs_free(&c);
        }
        printf("  rate %d: %d of %d answers lost at the hub's UART\n", rate, lost[rate], sent[rate]);
    }
    printf("  the bursts end by their slot's guard %+.1f us (s_i + pitch - 30 us)\n", end_worst);
    CHECK(lost[0] == 0 && lost[1] == 0 && lost[2] == 0 && fights == 0 && end_worst <= 0.0,
          "rc hub UART: answers lost %d / %d / %d (1 M / 500 k / 250 kbaud), fights %d, burst end past the guard %+.1f "
          "us", lost[0], lost[1], lost[2], fights, end_worst);
}

static void test_rc(void)
{
    test_rc_units();
    test_rc_chain();
    test_rc_hub_uart();
    test_rc_power();
}
