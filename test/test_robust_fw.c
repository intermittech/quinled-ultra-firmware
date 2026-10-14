/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Whole-firmware robustness fuzz: src/app/main.c itself (UPD_APP = 1: the build behind the bootloader) on a model of
 * the hardware (as test_soak.c), fed random input on every path a board has:
 *  - the DIN wire: Ultra frames (this board's slot first, random chains), WS2805 / WS2915 frames (this board at a
 *    random position: the header, the gap the boards before leave, the data), firmware-update frames (ANNOUNCE /
 *    DATA / END / PING, descriptors for this and other boards), garbage bits, truncated frames, header-only frames,
 *    glitch bursts; spec, typical and out-of-spec timing; gaps from 0 to seconds; repeats (config frames, SEQ);
 *  - the console: random bytes and random command lines;
 *  - the supply (dips, 12 / 24 / 48 V), the current (loads, shorts), the TP9 pad (test jig, the reset gesture);
 *  - the flash at boot: erased, random, valid, torn and foreign-but-CRC-valid records in every store, the descriptor
 *    page and the bootloader info, the hand-over block.
 * Checks after every main-loop pass: the state main.c relies on (PWM period / table length, the protocol state, the
 * acknowledge pattern, the engine's fields, the capture bookkeeping), the flash writes within their budgets, an
 * isolated clean frame applied as sent (Ultra SEQ, WS targets), the UPD confirm written only into an erased status
 * record, no crash (a hardware exception on the firmware's fiber is reported, AddressSanitizer reports memory errors).
 * Usage: robust_fw.exe [seed-hex] [sim-seconds]   (one random boot and run per process; test\run_robust_fw.cmd runs many)
 *        robust_fw.exe regress                     the fixed scenario below (the power-on effect store's change check) */
#include <excpt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UPD_APP 1
static void NVIC_SystemReset(void);
#define main fw_main
#include "../src/app/main.c"
#undef main
#include "../src/core/scratch.h"

__declspec(dllimport) void *__stdcall VirtualAlloc(void *addr, size_t size, unsigned long type, unsigned long prot);
__declspec(dllimport) void *__stdcall ConvertThreadToFiber(void *param);
__declspec(dllimport) void *__stdcall CreateFiber(size_t stack, void(__stdcall *start)(void *), void *param);
__declspec(dllimport) void __stdcall SwitchToFiber(void *fiber);

static int g_fail;
static unsigned long long g_seed;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; if (g_fail <= 30) { printf("  FAIL seed %llx t=%.3fs (%s:%d): ", g_seed, \
                                       (double)sim_ns / 1e9, __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

/* ================================================================================================ rng */
static unsigned long long rs;
static unsigned long long rnd64(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs; }
static uint32_t rnd(void) { return (uint32_t)(rnd64() >> 32); }
static uint32_t rint_(uint32_t n) { return n ? (uint32_t)(rnd64() % n) : 0u; }
static uint32_t rrange(uint32_t a, uint32_t b) { return a + rint_(b - a + 1u); }
static int chance(uint32_t pct) { return rint_(100) < pct; }
static double urand(double a, double b) { return a + (b - a) * (double)(rnd64() >> 11) / 9007199254740992.0; }

/* ================================================================================================ hardware model */
const uint32_t soak_uid[3] = { 1u, 2u, 3u };
soak_tim_t soak_tim4;
soak_systick_t soak_systick;
soak_adc_t soak_adc1;
soak_gpio_t soak_gpiob = { 0xFFFFFFFFu };
void soak_sleep(void) {}                                 /* (soak/ch32v20x.h CPU_SLEEP: the WFI) */

/* the PWM tables exactly as hal/hw.c sizes them (ASan sees a table built past them) */
uint16_t pwm_tab1[PATTERN_MAX_LEN * 4], pwm_tab2[PATTERN_MAX_LEN * 6], pwm_tab3[PATTERN_MAX_LEN * 6];
uint16_t din_ring[DIN_RING_WORDS], din_copy[DIN_COPY_WORDS];
static uint32_t cur_steps;
static uint8_t cur_stagger, cur_len, cur_spread;
static uint16_t spread_arr[PATTERN_MAX_LEN];

static void *fib_main, *fib_fw;
static uint64_t sim_ns, sim_ticks;
static uint64_t ring_words;
static uint32_t edges_total, edges_rearm;
static uint32_t vs_pending, supply_mv = 24000u;
static uint8_t sim_forced, sim_reset, sim_pad_low;
static uint32_t sim_crash;                               /* an exception code from the firmware's fiber */
static uint32_t short_ticks, sim_stalls, sim_passes;
static double fs_scale = 1.0;                            /* the board's HSI error: sample period x fs_scale */
static uint32_t ld_ma[5];                                /* the strips: mA at 100 % */

/* ---- the wire: a queue of pulses (board ns), sorted, non-overlapping */
#define PQ_N (1u << 17)
static uint64_t pq_rise[PQ_N], pq_fall[PQ_N];
static uint32_t pq_head, pq_tail, pq_edge;               /* consumed by the sampler / the edge counter */
static uint64_t wire_t;                                  /* where the next pulse goes (board ns) */

static int pq_full(void) { return pq_tail - pq_head >= PQ_N - 2u; }

static void pq_push(uint64_t rise, uint64_t fall)
{
    if (pq_full()) {
        return;                                          /* (the generator fills ahead: never reached) */
    }
    pq_rise[pq_tail % PQ_N] = rise;
    pq_fall[pq_tail % PQ_N] = fall;
    pq_tail++;
}

static int wire_level(uint64_t t)
{
    while (pq_head != pq_tail && pq_fall[pq_head % PQ_N] <= t) {
        pq_head++;
    }
    return pq_head != pq_tail && pq_rise[pq_head % PQ_N] <= t;
}

static double sample_ns(void) { return 250.0 / 9.0 * fs_scale; }

/* the DMA ring and the edge counter up to real time t */
static void wave_to(uint64_t t)
{
    uint64_t target = (uint64_t)((double)t / (16.0 * sample_ns()));
    while (ring_words < target) {
        uint64_t tw0 = (uint64_t)((double)(ring_words * 16u) * sample_ns());
        uint64_t tw1 = (uint64_t)((double)(ring_words * 16u + 16u) * sample_ns());
        uint16_t w = 0;
        (void)wire_level(tw0);
        if (pq_head == pq_tail || pq_rise[pq_head % PQ_N] >= tw1) {   /* nothing in this word: skip ahead in bulk */
            uint64_t next = pq_head == pq_tail ? target : (uint64_t)((double)pq_rise[pq_head % PQ_N] / (16.0 * sample_ns()));
            uint64_t n;
            if (next > target) {
                next = target;
            }
            if (next <= ring_words) {
                next = ring_words + 1u;
            }
            n = next - ring_words;
            if (n >= DIN_RING_WORDS) {
                memset(din_ring, 0, sizeof din_ring);
                ring_words = next;
            } else {
                while (ring_words < next) {
                    din_ring[ring_words % DIN_RING_WORDS] = 0;
                    ring_words++;
                }
            }
            continue;
        }
        {
            uint32_t j;
            for (j = 0; j < 16u; j++) {
                if (wire_level((uint64_t)((double)(ring_words * 16u + j) * sample_ns()))) {
                    w |= (uint16_t)(0x8000u >> j);
                }
            }
        }
        din_ring[ring_words % DIN_RING_WORDS] = w;
        ring_words++;
    }
    while (pq_edge != pq_tail && pq_fall[pq_edge % PQ_N] <= t) {
        edges_total++;
        pq_edge++;
    }
}

/* ---- frames onto the wire */
typedef struct {
    double t0h, t1h, per;                                /* ns */
    double glitch;                                       /* probability of a 1-3 sample spike per bit */
} wtiming_t;

static wtiming_t rnd_timing(void)
{
    wtiming_t w;
    uint32_t k = rint_(10);
    w.glitch = chance(10) ? urand(0, 0.02) : 0.0;
    if (k < 7u) {                                        /* spec */
        w.t0h = urand(220, 380); w.t1h = urand(520, 1000); w.per = urand(1200, 1500);
    } else if (k < 9u) {                                 /* typical controller */
        w.t0h = urand(280, 320); w.t1h = urand(680, 820); w.per = urand(1240, 1260);
    } else {                                             /* out of spec */
        w.t0h = urand(100, 700); w.t1h = urand(300, 3000); w.per = urand(600, 4000);
    }
    return w;
}

/* nbits from bits (MSB first) at wire_t; low_bits_after: bits of LOW appended (the boards before a WS2915 board) */
static void wire_bits(const uint8_t *bits, uint32_t nbits, const wtiming_t *w)
{
    uint32_t i;
    for (i = 0; i < nbits; i++) {
        int b = (bits[i >> 3] >> (7u - (i & 7u))) & 1;
        double h = (b ? w->t1h : w->t0h) + urand(-10, 10), p = w->per + urand(-20, 20);
        if (p < h + 200.0) {
            p = h + 200.0;
        }
        pq_push(wire_t, wire_t + (uint64_t)h);
        if (urand(0, 1) < w->glitch) {
            uint64_t g = wire_t + (uint64_t)h + (uint64_t)urand(50, p - h - 100);
            pq_push(g, g + (uint64_t)urand(10, 90));
        }
        wire_t += (uint64_t)p;
        if (chance(1)) {
            wire_t += (uint64_t)urand(0, 20000);         /* a controller hiccup */
        }
    }
}

static void wire_gap(double ns) { wire_t += (uint64_t)ns; }

static double rnd_gap(void)
{
    switch (rint_(20)) {
    case 0: return urand(100e3, 280e3);                  /* ambiguous */
    case 1: case 2: return urand(0, 100e3);              /* frames run into each other */
    case 3: return urand(50e6, 1000e6);                  /* signal loss */
    case 4: case 5: return urand(5e6, 25e6);             /* slow refresh */
    default: return urand(300e3, 8e6);
    }
}

/* the chain this board sits in */
static uint32_t chain_n = 1, chain_pos;                  /* boards, this board's position (WS2915: the gap before it) */

/* the last clean frame sent (for the end-to-end check): kind, what it carried, when it ended */
enum { CF_NONE = 0, CF_ULTRA, CF_WS2805, CF_WS2915 };
static struct {
    int kind;
    uint8_t slot[32];
    uint8_t raw[10];
    uint16_t ch[5];
    uint64_t t_end;
    proto_t locked_at_send;
    uint8_t cfg_msg;                                     /* WS: the slot is a config message */
    int checked;
} cf;

static uint8_t last_frame[1700];
static uint32_t last_nbits;
static int have_last;
static uint8_t useq = 1;
static uint8_t desc_this[64], desc_other[64], desc_newer[64];

static void slot_random(uint8_t *s)                      /* a valid slot, a builder with random arguments or random bytes */
{
    uint8_t light = (uint8_t)(chance(85) ? rint_(5) : rint_(16)), seq = useq++;
    uint16_t a[5], b[5], mn[5], mx[5];
    uint8_t roles[5], params[22], ids[12];
    uint16_t vals[12];
    uint32_t i, kind = rint_(14);
    for (i = 0; i < 5u; i++) {
        a[i] = (uint16_t)(chance(50) ? 65535u : rnd());
        b[i] = (uint16_t)(chance(50) ? 65535u : rnd());
        mn[i] = (uint16_t)(chance(70) ? 0u : rnd());
        mx[i] = (uint16_t)(chance(70) ? 65535u : rnd());
        roles[i] = (uint8_t)(chance(80) ? rint_(8) : rint_(16));
    }
    for (i = 0; i < 22u; i++) params[i] = (uint8_t)rnd();
    for (i = 0; i < 12u; i++) { ids[i] = (uint8_t)(chance(70) ? rint_(ULTRA_CFG_COUNT) : rint_(256)); vals[i] = (uint16_t)rnd(); }
    switch (kind) {
    case 0: ultra_slot_nop(s); break;
    case 1: ultra_slot_light_def(s, light, seq, roles, (uint16_t)(chance(50) ? 2800u : rnd()), (uint8_t)rint_(4), mn, mx); break;
    case 2: case 3: ultra_slot_set(s, light, seq, (uint8_t)rint_(4), (uint16_t)rnd(), a, b, chance(50) ? rint_(5000) : rnd() & 0xFFFFFFu,
                                   (int16_t)(chance(70) ? 30 : rnd())); break;
    case 4: ultra_slot_effect(s, light, seq, (uint8_t)rint_(11), (int32_t)(chance(70) ? 30 : rnd()), params, rint_(23)); break;
    case 5: ultra_slot_step(s, light, seq, (uint8_t)rint_(9), (uint8_t)rint_(9), (uint16_t)rnd(), (uint16_t)rnd(), (uint16_t)rnd(), a, b); break;
    case 6: ultra_slot_config(s, seq, ids, vals, rint_(10)); break;
    case 7: ultra_slot_identify(s, seq, (uint8_t)rint_(7)); break;
    case 8: ultra_slot_calibrate(s, seq, (uint8_t)rint_(120)); break;
    case 9: ultra_slot_hold_dark(s, seq, (uint8_t)rint_(5)); break;
    case 10: ultra_slot_scene(s, light, seq, (uint8_t)rint_(3), (uint8_t)rint_(30), (uint16_t)rnd(), a, b); break;
    case 11: ultra_slot_light_opt_points(s, light, seq, (uint8_t)rint_(4), (uint16_t)rnd(), (uint16_t)rnd(), (uint16_t)rnd(),
                                         (uint16_t)rnd(), rint_(5), a, b); break;
    case 12: ultra_slot_scene_effect(s, light, seq, (uint8_t)rint_(11), params, rint_(23)); break;
    default:
        ultra_slot_begin(s, (uint8_t)(chance(60) ? rint_(ULTRA_TYPES) : rint_(256)), light, (uint8_t)rint_(16), seq);
        for (i = 3; i < 30u; i++) s[i] = (uint8_t)rnd();
        ultra_slot_seal(s);
        break;
    }
}

static void mk_desc(uint8_t *d, uint32_t version, uint8_t hw_id, uint16_t revmask, uint8_t min_bl)
{
    memset(d, 0, 64);
    memcpy(d, "QDU5", 4);
    d[UPD_D_DESC_VERSION] = UPD_DESC_VERSION;
    d[UPD_D_HW_ID] = hw_id;
    upd_put16(d + UPD_D_HW_REV_MASK, revmask);
    upd_put32(d + UPD_D_FW_VERSION, version);
    upd_put32(d + UPD_D_BODY_LEN, 30548u);
    upd_put32(d + UPD_D_BODY_CRC, 0x12345678u);
    upd_put32(d + UPD_D_LOAD_ADDR, UPD_APP_BODY);
    upd_put32(d + UPD_D_ENTRY, UPD_APP_ENTRY);
    d[UPD_D_MIN_BL] = min_bl;
    upd_put32(d + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d, 15u));
}

/* one frame onto the wire (after a gap), as this board sees it */
static void send_frame(void)
{
    static uint8_t f[1700];
    uint32_t nbits = 0, i, kind = rint_(100);
    wtiming_t w = rnd_timing();
    uint8_t flags = 0;
    int clean;
    double gap = rnd_gap();
    cf.kind = CF_NONE;
    if (have_last && chance(30)) {                       /* the same frame again (config frames, SEQ repeats) */
        memcpy(f, last_frame, (last_nbits + 7u) / 8u);
        nbits = last_nbits;
        kind = 200;
    } else if (kind < 35u) {                             /* Ultra */
        uint32_t n = chain_n, k;
        uint8_t *h;
        for (k = 0; k < n; k++) {
            uint8_t *s = f + 32u * k;
            if (chance(15)) {
                ultra_slot_dark(s);                      /* the preamble */
            } else {
                slot_random(s);
                if (chance(5)) {
                    s[rint_(32)] ^= (uint8_t)(1u << rint_(8));
                }
            }
        }
        h = f + 32u * n;
        flags = (uint8_t)(chance(85) ? 0u : rint_(16));
        ultra_build_header_flags(h, (uint32_t)(wire_t / 1000u), flags);
        if (chance(5)) {
            h[rint_(8)] ^= (uint8_t)(1u << rint_(8));
        }
        nbits = 256u * n + 64u;
        if (chance(8)) {
            nbits = rrange(1, nbits);                    /* truncated */
        }
        if (nbits == 256u * n + 64u && ultra_slot_check(f) == 0 && flags == 0u) {
            memcpy(cf.slot, f, 32);
            cf.kind = CF_ULTRA;
        }
    } else if (kind < 50u) {                             /* WS2805: this board's slot first (the ones before are blocked) */
        uint32_t n = chain_n, k;
        for (k = 0; k < n * 5u; k++) {
            f[k] = (uint8_t)rnd();
        }
        if (chance(15)) {
            config_build((uint8_t)(chance(80) ? rrange(1, 0x14) : rint_(256)), (uint8_t)rint_(256), 0, f);
        }
        nbits = 40u * n;
        if (chance(5)) {
            nbits = rrange(1, nbits);
        }
        if (nbits == 40u * n) {
            memcpy(cf.raw, f, 5);
            cf.kind = CF_WS2805;
        }
    } else if (kind < 65u) {                             /* WS2915: header, the gap of the boards before, the data */
        uint32_t hb = chance(50) ? 32u : 26u, n = chain_n, k, pos = chain_pos % n, bit = 0;
        uint8_t hdr[4] = { 0xFF, 0xFF, 0xFF, 0x80 };
        memset(f, 0, sizeof f);
        if (hb == 26u) {
            hdr[0] = 0xFF; hdr[1] = 0xFF; hdr[2] = 0xFF; hdr[3] = 0xC0;   /* ... | 1 */
        }
        if (chance(20)) {
            for (k = 0; k < 4u; k++) hdr[k] = (uint8_t)rnd();
        }
        wire_gap(gap);
        gap = 0;
        wire_bits(hdr, hb, &w);
        wire_gap(pos * 80.0 * w.per);                    /* the boards before block their 80 bits */
        for (k = 0; k < (n - pos) * 10u; k++) {
            f[k] = (uint8_t)rnd();
        }
        if (chance(15)) {
            config_build((uint8_t)(chance(80) ? rrange(1, 0x14) : rint_(256)), (uint8_t)rint_(256), 1, f);
        }
        nbits = 80u * (n - pos);
        if (chance(5)) {
            nbits = rrange(1, nbits);
        }
        (void)bit;
        if (nbits == 80u * (n - pos) && pos != 0u) {
            memcpy(cf.raw, f, 10);
            for (k = 0; k < 5u; k++) cf.ch[k] = (uint16_t)((f[2u * k] << 8) | f[2u * k + 1u]);
            cf.kind = CF_WS2915;
        } else if (nbits == 80u * (n - pos)) {           /* position 0: header + data contiguous; the whole slot */
            uint8_t g[14];
            memcpy(g, hdr, hb / 8u);                     /* (26-bit header: not byte-aligned, skip the check) */
            (void)g;
        }
    } else if (kind < 78u) {                             /* firmware update frames */
        static const uint8_t types[] = { UPD_ANNOUNCE, UPD_ANNOUNCE, UPD_DATA, UPD_END, UPD_PING, UPD_ABORT };
        static uint8_t session = 0x21, half;
        uint8_t type = types[rint_(6)], data[32];
        uint32_t idx = type == UPD_ANNOUNCE ? (half ^= 1u) : rint_(1800);
        const uint8_t *d = chance(60) ? desc_this : chance(50) ? desc_newer : desc_other;
        if (chance(5)) {
            session++;
        }
        for (i = 0; i < 32u; i++) data[i] = (uint8_t)rnd();
        if (type == UPD_ANNOUNCE) {
            memcpy(data, d + 32u * idx, 32);
        }
        upd_build_frame(f, type, session, idx, 0x13, (uint8_t)(chance(20) ? UPD_F_FORCE : 0u), data, (uint32_t)(wire_t / 1000u));
        if (chance(5)) {
            f[rint_(48)] ^= (uint8_t)(1u << rint_(8));
        }
        nbits = UPD_FRAME_BITS;
    } else if (kind < 88u) {                             /* garbage */
        nbits = rrange(1, 2000);
        for (i = 0; i < (nbits + 7u) / 8u; i++) {
            f[i] = (uint8_t)rnd();
        }
    } else if (kind < 93u) {                             /* a header alone */
        ultra_build_header_flags(f, (uint32_t)(wire_t / 1000u), (uint8_t)(chance(80) ? 0u : rint_(16)));
        nbits = 64;
    } else if (kind < 97u) {                             /* a burst of 26 / 32 bits (looks like a WS2915 header) */
        for (i = 0; i < 4u; i++) f[i] = (uint8_t)rnd();
        nbits = chance(50) ? 26u : 32u;
    } else {                                             /* spikes */
        uint32_t k, n = rrange(1, 50);
        wire_gap(gap);
        for (k = 0; k < n; k++) {
            uint64_t t = wire_t + (uint64_t)urand(0, 50000);
            pq_push(t, t + (uint64_t)urand(10, 120));
        }
        wire_t += 60000u;
        have_last = 0;
        return;
    }
    if (gap > 0) {
        wire_gap(gap);
    }
    clean = cf.kind != CF_NONE && gap >= 1e6 && w.glitch == 0.0 && w.t0h >= 220 && w.t0h <= 380 && w.t1h >= 520 &&
            w.t1h <= 1000 && w.per >= 1200 && w.per <= 1500 && kind != 200u;
    if (clean) {
        cf.locked_at_send = proto_active(&frx.ps);
        cf.checked = 0;
        {
            slot_t sl;
            memset(&sl, 0, sizeof sl);
            sl.raw_len = (uint8_t)(cf.kind == CF_WS2805 ? 5 : 10);
            memcpy(sl.raw, cf.raw, sl.raw_len);
            cf.cfg_msg = (uint8_t)(cf.kind != CF_ULTRA && sl.raw[0] == 0xD2u && sl.raw[1] == 0xA5u &&
                                   crc8_smbus(sl.raw, 4) == sl.raw[4]);
        }
    } else {
        cf.kind = CF_NONE;
    }
    wire_bits(f, nbits, &w);
    cf.t_end = wire_t;
    wire_gap(300e3);                                     /* at least a reset after a clean frame's check window */
    if (!clean) {
        wire_t -= 300000u;
    }
    memcpy(last_frame, f, (nbits + 7u) / 8u);
    last_nbits = nbits;
    have_last = 1;
}

/* keep the wire filled ahead of real time */
static uint8_t wire_quiet;                               /* no traffic (regress: the engine is driven directly) */

static void wire_fill(uint64_t t)
{
    while (!wire_quiet && wire_t < t + 50000000u && !pq_full()) {
        send_frame();
    }
}

/* ---- hal/hw.h */
uint16_t *pwm_table(uint8_t ch, uint8_t *stride)
{
    if (cur_spread) {
        switch (ch) {
        case CH_G: *stride = 4; return &pwm_tab1[2];
        case CH_R: *stride = 4; return &pwm_tab1[3];
        case CH_B: *stride = 6; return &pwm_tab2[5];
        case CH_W2: *stride = 6; return &pwm_tab3[4];
        default: *stride = 6; return &pwm_tab3[5];
        }
    }
    switch (ch) {
    case CH_G: *stride = 2; return &pwm_tab1[0];
    case CH_R: *stride = 2; return &pwm_tab1[1];
    case CH_B: *stride = 1; return &pwm_tab2[0];
    case CH_W2: *stride = 2; return &pwm_tab3[0];
    default: *stride = 2; return &pwm_tab3[1];
    }
}
const uint16_t *pwm_spread_arr(void) { return cur_spread ? spread_arr : 0; }
void hw_init(void) {}
void stack_paint(void) {}
uint32_t stack_used(void) { return 0x2A0u; }
static uint32_t uid_hash;
uint32_t hw_uid_hash(void) { return uid_hash; }
uint32_t hw_reset_flags(void) { return 0x0C000000u; }
void pwm_start(uint32_t s, uint8_t l, uint8_t st)
{
    CHECK(s >= PWM_STEPS_MIN && s <= PWM_STEPS_MAX, "pwm_start: %u steps", s);
    CHECK(l >= 1u && l <= PATTERN_MAX_LEN, "pwm_start: table length %u", l);
    cur_steps = s;
    cur_len = l;
    cur_stagger = st;
    cur_spread = (uint8_t)(stagger_is_spread(st) && l >= 2u);
    if (cur_spread) {
        stagger_spread_arr(s, l, spread_arr);
    }
}
uint8_t pwm_right_aligned(uint8_t ch) { return (uint8_t)(cur_stagger && cur_steps <= 65535u && (ch == CH_G || ch == CH_W2)); }
void pwm_set_phase(uint16_t o2, uint16_t o3) { (void)o2; (void)o3; }
static uint32_t sim_rearms;
void gate_rearm(const gate_plan_t *g)
{
    CHECK(g->own_bits == 40u || g->own_bits == 106u || g->own_bits == 112u || g->own_bits == 256u || g->own_bits == 384u,
          "gate plan own_bits %u", g->own_bits);
    edges_rearm = edges_total;
    sim_rearms++;
}
void gate_force_closed(void) {}
uint32_t din_ring_pos(void) { return (uint32_t)(ring_words % DIN_RING_WORDS); }
void hw_adc_init(void) {}
uint32_t hw_adc_ms(void) { vs_pending++; return 0; }   /* (0.6.4: 1 = the temperature's sequence) */
static uint32_t pair_of(uint32_t mv)
{
    uint32_t p = (uint32_t)((double)mv * 10.0 / 230.0 / 3280.0 * 4095.0 + 0.5);
    return 2u * (p > 4095u ? 4095u : p);
}
void hw_vsense_take(uint32_t sum2[2], uint32_t *n)
{
    *n = vs_pending;
    sum2[0] = vs_pending * pair_of(supply_mv);
    sum2[1] = vs_pending * 2u * rint_(4096);
    vs_pending = 0;
}
void hw_temp_start(void) {}
volatile uint16_t hw_diag[HW_DIAG_N];                    /* (0.6.5, D6: hal/hw.c's counters, none here) */
void hw_dma_check(void) {}
uint8_t hw_temp_read(uint32_t *ts2, uint32_t *vref2)
{
    *ts2 = chance(90) ? 2u * 1710u + rint_(40) : rnd() & 0x1FFFu;
    *vref2 = chance(90) ? 2u * 1489u + rint_(20) : rnd() & 0x1FFFu;
    return (uint8_t)chance(95);
}
void pwm_force_off(uint8_t off) { sim_forced = off; }
void led_set(uint8_t on) { (void)on; }
void watchdog_kick(void) { sim_passes++; SwitchToFiber(fib_main); }
static void NVIC_SystemReset(void)
{
    sim_reset = 1;
    for (;;) {
        SwitchToFiber(fib_main);
    }
}

/* ---- console: random input, the output counted */
static char con_in[4096];
static uint32_t con_in_r, con_in_w, con_out_n;
static char con_out[1u << 15];
int uart_getc(void) { return con_in_r < con_in_w ? (int)(unsigned char)con_in[con_in_r++] : -1; }
static FILE *con_dump;                                   /* ROBUST_CON_DUMP=<file>: every byte the console wrote */
void uart_write(const char *s)
{
    static int con_dump_init;
    if (!con_dump_init) {
        const char *f = getenv("ROBUST_CON_DUMP");
        con_dump_init = 1;
        con_dump = f ? fopen(f, "wb") : NULL;
    }
    if (con_dump) {
        fputs(s, con_dump);
        fflush(con_dump);
    }
    while (*s) {
        if (con_out_n + 1u < sizeof con_out) {
            con_out[con_out_n] = *s;
        }
        con_out_n++;
        s++;
    }
    con_out[con_out_n < sizeof con_out ? con_out_n : sizeof con_out - 1u] = 0;
}
void uart_kick(void) {}
uint32_t uart_room(void) { return 1024u; }

/* ---- flash (the real addresses), writes / erases per page. 0.6.4: the stores are core/flashrec.c's themselves;
 * this is hal/hw.c's flash_op: the main loop waits (an erase 16 ms, a half-word 100 us: test durations), the tick runs
 * on as the SRAM path does (flash_poll_isr: the time bases, the fast trip) */
#define FL_BASE 0x08000000u
#define FL_SIZE (RECORDS_END - FL_BASE)       /* (0.6.5) the 64 KB and the record sectors above */
static uint8_t *fl;
static uint32_t fl_writes[FL_SIZE / 256u], fl_erases[FL_SIZE / 256u];
static uint32_t load_raw(void);

uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i, us = 100u * halfwords;
    CHECK(addr >= FL_BASE && addr + 2u * halfwords <= FL_BASE + FL_SIZE, "flash write outside the chip: %08x", addr);
    if (addr >= UPD_APP_DESC && addr < UPD_APP_DESC + UPD_PAGE_BYTES && halfwords != 0u) {
        CHECK(halfwords == 1u && addr >= UPD_APP_DESC + UPD_DESC_BYTES && (addr & 1u) == 0u,
              "the confirm record at %08x: outside the status records", addr);
    }
    if (erase_addr != 0u) {
        CHECK(erase_addr >= FL_BASE + 0x2000u && erase_addr < FL_BASE + FL_SIZE && (erase_addr & 0xFFu) == 0u,
              "flash erase at %08x", erase_addr);
        memset(fl + (erase_addr - FL_BASE), 0xFF, 256u);
        fl_erases[(erase_addr - FL_BASE) / 256u]++;
        us += 16000u;
    }
    for (i = 0; i < halfwords; i++) {
        uint16_t *p = (uint16_t *)(uintptr_t)(addr + 2u * i);
        CHECK(*p == 0xFFFFu || *p == 0xE339u, "flash: programming a half-word that is not erased at %08x",
              addr + 2u * i);
        *p = src[i];
    }
    if (halfwords) {
        fl_writes[(addr - FL_BASE) / 256u]++;
    }
    sim_stalls++;
    for (; us >= 50u; us -= 50u) {                       /* the main loop waits, the tick runs on (from SRAM) */
        sim_ns += 50000u;
        sim_ticks++;
        wave_to(sim_ns);
        soak_adc1.RDATAR = load_raw();
        flash_poll_isr();
    }
    return FLASH_OK;
}
uint32_t uart_busy(void) { return 0; }
void hw_flash_trip(void) { pwm_force_off(1); }
static uint32_t sim_window;                              /* (0.6.5) the boot window for the record reads: opened */
void hw_records_window(uint32_t open) { sim_window += open; }

static uint32_t page_of(uint32_t addr) { return (addr - FL_BASE) / 256u; }
static uint32_t writes_in(uint32_t addr, uint32_t pages)
{
    uint32_t n = 0, i;
    for (i = 0; i < pages; i++) {
        n += fl_writes[page_of(addr) + i];
    }
#define W_FX() (writes_in(SCENE_FX_ADDR, 2u) + (SCENE_FX_B_ADDR != SCENE_FX_ADDR ? writes_in(SCENE_FX_B_ADDR, 2u) : 0u))
    return n;
}

/* ================================================================================================ flash images */
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

static void flash_map(void)
{
    fl = (uint8_t *)VirtualAlloc((void *)(uintptr_t)FL_BASE, FL_SIZE, 0x3000u, 0x04u);
    if (fl != (uint8_t *)(uintptr_t)FL_BASE) {
        printf("robust_fw: cannot map the flash model at 0x%08X\n", FL_BASE);
        exit(2);
    }
    memset(fl, 0xFF, FL_SIZE);
}

static void flash_init(void)
{
    uint32_t i, k;
    flash_map();
    erased_fill(fl, FL_SIZE);
    /* the bootloader info and the descriptor page (UPD_APP = 1) */
    if (chance(85)) {
        uint8_t *bl = fl + 0x10u, *dp = fl + 0x2000u;
        memcpy(bl, "QDBL\x01\x01\x03", 7);
        bl[7] = 0;
        memcpy(bl + 8, "20261013", 8);
        if (chance(80)) {
            mk_desc(dp, 0x00060200u | 0xFFu, 1, 0x8u, 1);   /* 0.6.2 */
            for (k = 0; k < rint_(5); k++) {
                uint32_t v = k == 0u ? UPD_ST_VERIFIED : chance(50) ? UPD_ST_CONFIRMED : UPD_ST_TRIAL_SEEN;
                dp[64u + 2u * k] = (uint8_t)v;
                dp[65u + 2u * k] = (uint8_t)(v >> 8);
            }
        } else if (chance(50)) {
            random_fill(dp, 256);
        }
    } else {
        random_fill(fl + 0x10u, 16);
    }
    /* the settings page */
    {
        uint8_t *p = fl + (SETTINGS_ADDR - FL_BASE);
        uint32_t kind = rint_(10);
        if (kind < 5u) {
            uint32_t n = rrange(1, 8);
            for (k = 0; k < n; k++) {
                settings_t s;
                store_ext_t e;
                store_slot_t sl;
                settings_default(&s);
                s.protocol = (uint8_t)(chance(70) ? 0u : rint_(4));
                s.last_proto = (uint8_t)rint_(5);
                s.pwm_preset = (uint8_t)(chance(90) ? rint_(15) : 0xFFu);
                s.raw_steps = rrange(1440, 65535);
                s.stagger = (uint8_t)rint_(5);
                s.smoothing = (uint8_t)rint_(3);
                s.gamma = (uint8_t)rint_(3);
                s.signal_loss = (uint8_t)rint_(3);
                store_ext_clear(&e);
                e.ilimit50 = (uint8_t)(chance(50) ? 0xFFu : rint_(200));
                e.opts = (uint8_t)(chance(50) ? 0xFFu : rint_(2));
                if (kind == 4u) {                        /* another firmware's values */
                    random_fill((uint8_t *)&s, sizeof s);
                }
                store_pack_ext(&s, &e, k + 1u, &sl);
                memcpy(p + 32u * k, &sl, k + 1u == n && chance(20) ? rrange(1, 31) : 32u);
            }
        } else if (kind < 7u) {
            random_fill(p, 256);
        }
    }
    /* the scene log */
    {
        uint8_t *p = fl + (SCENE_LOG_ADDR - FL_BASE);
        uint32_t kind = rint_(10);
        if (kind < 5u) {
            uint32_t n = rint_(SCENE_RECS), start = rint_(SCENE_RECS);
            for (k = 0; k < n; k++) {
                scene_rec_t rec;
                scene_t s;
                scene_clear(&s);
                s.mask = (uint8_t)(chance(90) ? rint_(32) : rnd());
                s.fade = (uint8_t)rint_(40);
                for (i = 0; i < 5u; i++) s.v[i] = (uint16_t)rnd();
                scene_log_pack(&s, (uint16_t)(0xFFF0u + k), &rec);
                i = (start + k) % SCENE_RECS;            /* (0.6.5: the B page in the other sector) */
                memcpy(fl + ((i < SCENE_RECS_PAGE ? SCENE_LOG_ADDR : SCENE_B_ADDR) - FL_BASE) + (i % SCENE_RECS_PAGE) * 16u,
                       &rec, k + 1u == n && chance(20) ? rrange(1, 15) : 16u);
            }
        } else if (kind < 7u) {
            random_fill(p, 256u);
            random_fill(fl + (SCENE_B_ADDR - FL_BASE), 256u);
        }
    }
    /* the calibration page */
    {
        uint8_t *p = fl + (CAL_PAGE_ADDR - FL_BASE);
        uint32_t kind = rint_(10);
        if (kind < 5u) {
            uint32_t n = rrange(1, 4);
            for (k = 0; k < n; k++) {
                isense_cal_t c;
                cal_rec_t rec;
                memset(&c, 0, sizeof c);
                c.levels = (uint8_t)(chance(60) ? 4u : chance(50) ? 1u : rint_(256));
                c.pct[0] = (uint8_t)(c.levels == 1u ? rrange(0, 120) : 25u);
                c.pct[1] = 50; c.pct[2] = 75; c.pct[3] = 100;
                c.zero_ma = (int16_t)(chance(80) ? 45 : (int16_t)rnd());
                for (i = 0; i < 5u; i++) {
                    uint32_t l, full = chance(80) ? rrange(0, 5000) : rnd() & 0xFFFFu;
                    for (l = 0; l < 4u; l++) c.ma[i][l] = (uint16_t)(chance(80) ? full * (l + 1u) / 4u : rnd());
                }
                if (kind == 4u) {
                    random_fill((uint8_t *)&c, sizeof c);
                }
                cal_pack(&c, k + 1u, (uint16_t)k, &rec);
                memcpy(p + 64u * k, &rec, k + 1u == n && chance(20) ? rrange(1, 63) : 64u);
            }
        } else if (kind < 7u) {
            random_fill(p, 256);
        }
    }
    /* the settings' and calibration's B pages (0.6.4: A/B pairs): erased, random, or a copy of the A page */
    {
        static const uint32_t ab[2][2] = { { SETTINGS_ADDR, SETTINGS_B_ADDR }, { CAL_PAGE_ADDR, CAL_B_ADDR } };
        for (k = 0; k < 2u; k++) {
            uint32_t kind = rint_(10);
            if (kind < 2u) {
                random_fill(fl + (ab[k][1] - FL_BASE), 256);
            } else if (kind < 3u) {
                memcpy(fl + (ab[k][1] - FL_BASE), fl + (ab[k][0] - FL_BASE), 256);
            }
        }
    }
    /* the lifetime record (0.6.3): an A/B pair of 6 records each */
    {
        uint32_t kind = rint_(10);
        uint8_t *pg[2];
        pg[0] = fl + (LIFE_PAGE0_ADDR - FL_BASE);
        pg[1] = fl + (LIFE_PAGE1_ADDR - FL_BASE);
        if (kind < 5u) {
            uint32_t n = rint_(2u * LIFE_RECS + 1u), w = rnd();
            for (k = 0; k < n; k++) {
                life_rec_t r;
                random_fill((uint8_t *)&r, sizeof r);
                r.writes = ++w;
                r.magic = LIFE_MAGIC;
                r.crc = crc16_ccitt((const uint8_t *)&r, 38u);
                memcpy(pg[k / LIFE_RECS] + (k % LIFE_RECS) * LIFE_REC_BYTES, &r,
                       k + 1u == n && chance(20) ? rrange(1, LIFE_REC_BYTES - 1u) : LIFE_REC_BYTES);
            }
        } else if (kind < 7u) {
            random_fill(pg[0], 256);
            random_fill(pg[1], 256);
        }
    }
    /* the power-on effect store */
    {
        uint8_t *p = fl + (SCENE_FX_ADDR - FL_BASE);
        uint32_t kind = rint_(10);
        if (kind < 5u) {
            ultra_t *u = (ultra_t *)malloc(sizeof(ultra_t));
            uint8_t s[32], mask = (uint8_t)(kind == 4u ? rnd() : rint_(32));
            ultra_init(u, rnd());
            for (k = 0; k < 20u; k++) {
                slot_random(s);
                (void)ultra_on_slot(u, s, 0);
            }
            for (k = 0; k < ULTRA_LIGHTS; k++) {
                uint8_t *h = p + ULTRA_FX_HDR + k * ULTRA_HEAD_BYTES;
                if (mask & (1u << k)) {
                    memcpy(h, &u->light[k], ULTRA_HEAD_BYTES);
                    if (kind == 4u && chance(50)) {
                        random_fill(h, ULTRA_HEAD_BYTES);   /* foreign content, its CRC fits below */
                    }
                }
                p[5u + k] = crc8_smbus(h, ULTRA_HEAD_BYTES);
            }
            p[4] = mask;
            p[10] = p[11] = 0;
            p[0] = 0x53; p[1] = 0x46; p[2] = 0x58; p[3] = 0x31;
            if (chance(10)) {
                for (i = 2u * rint_(6); i < 12u; i++) p[i] = 0xFF;   /* a torn header */
            }
            free(u);
        } else if (kind < 7u) {
            random_fill(p, 512);
        }
    }
}

/* ================================================================================================ the run */
static void __stdcall fw_entry(void *p)
{
    (void)p;
    __try {
        (void)fw_main();
    } __except (sim_crash = (uint32_t)_exception_code(), EXCEPTION_EXECUTE_HANDLER) {
    }
    for (;;) {
        SwitchToFiber(fib_main);
    }
}

static uint32_t load_raw(void)
{
    uint32_t ma = 45u, i, g = is.ma_per_lsb_q16 ? is.ma_per_lsb_q16 : 262410u, raw;
    if (short_ticks) {
        short_ticks--;
        return 4095u;
    }
    for (i = 0; i < 5u && !sim_forced; i++) {
        ma += (uint32_t)out_now[i] * ld_ma[i] / 65535u;
    }
    ma = (uint32_t)((double)ma * urand(0.98, 1.02));
    raw = (uint32_t)(((uint64_t)ma << 16) / g);
    return raw > 4095u ? 4095u : raw;
}

/* the invariants main.c relies on, after every pass */
static void inv_fw(void)
{
    uint32_t l, i;
    CHECK(steps >= PWM_STEPS_MIN && steps <= PWM_STEPS_MAX, "steps %u", steps);
    CHECK(len >= 1u && len <= PATTERN_MAX_LEN, "len %u", len);
    CHECK(frx.ps.mode <= PMODE_ULTRA && frx.ps.locked <= PROTO_ULTRA && frx.ps.cand <= PROTO_ULTRA &&
          frx.ps.cand_count < PROTO_LOCK_FRAMES && (frx.ps.forced_hdr == 26u || frx.ps.forced_hdr == 32u),
          "proto state %d %d %d %u %u", (int)frx.ps.mode, (int)frx.ps.locked, (int)frx.ps.cand, frx.ps.cand_count,
          frx.ps.forced_hdr);
    CHECK(!frx.ps.provisional || (frx.ps.mode == PMODE_AUTO && frx.ps.locked != PROTO_NONE), "provisional without a lock");
    CHECK(ack.nseg <= ACK_MAX_SEG && (!ack.active || ack.seg < ack.nseg), "ack segments %u / %u", ack.seg, ack.nseg);
    CHECK(is.cal_step <= ISENSE_CAL_STEPS_MAX && is.k <= 65536u && is.k_ss <= 65536u, "isense %u %u %u", is.cal_step, is.k, is.k_ss);
    CHECK(ul.identify <= ULTRA_IDENT_VERSION, "identify %u", ul.identify);
    for (l = 0; l < ULTRA_LIGHTS; l++) {
        const ultra_light_t *L = &ul.light[l];
        for (i = 0; i < 5u; i++) {
            CHECK(L->role[i] <= ULTRA_ROLE_WARM_WHITE, "light %u role %u", l, L->role[i]);
        }
        CHECK((L->mask & ~0x1Fu) == 0u && L->opt_n <= ULTRA_OPT_POINTS_MAX && L->scene_fx < ULTRA_FX_COUNT &&
              L->fx < ULTRA_FX_COUNT && L->pend_fx < ULTRA_FX_COUNT && L->step_count <= ULTRA_MAX_STEPS &&
              L->scene_mode <= 1u, "light %u fields", l);
    }
    if (cap_ready) {
        uint32_t want = (uint32_t)cap_bits - cap_skip;
        CHECK(want >= 1u && want <= 384u && (cap_own == PROTO_UPD_BITS || want <= 288u), "capture of %u bits (own %u skip %u)",
              want, cap_own, cap_skip);
    }
    CHECK((srun.hold & ~0x1Fu) == 0u && (fx_ch & ~0x1Fu) == 0u, "scene hold %x fx_ch %x", srun.hold, fx_ch);
    CHECK(tj.rst <= TJ_RST_CONFIRM, "tj rst %u", tj.rst);
}

/* the end-to-end check of the last clean frame, once the board has had 2 ms after it */
static void check_clean(void)
{
    uint32_t i;
    if (cf.kind == CF_NONE || cf.checked || sim_ns < cf.t_end + 2500000u) {
        return;
    }
    cf.checked = 1;
    if (cf.kind == CF_ULTRA && cf.locked_at_send == PROTO_ULTRA && proto_active(&frx.ps) == PROTO_ULTRA) {
        uint8_t type = cf.slot[0], light = (uint8_t)(cf.slot[1] >> 4), seq = cf.slot[2];
        if ((type == ULTRA_SET || type == ULTRA_LIGHT_DEF || type == ULTRA_SCENE || type == ULTRA_LIGHT_OPT) && light < 5u &&
            !tj.active) {
            CHECK(ul.last_seq[light][type] == seq, "a clean Ultra frame (type %u light %u seq %u) was not applied (last %u)",
                  type, light, seq, ul.last_seq[light][type]);
        }
    } else if (cf.kind == CF_WS2805 && cf.locked_at_send == PROTO_WS2805 && proto_active(&frx.ps) == PROTO_WS2805 && !cf.cfg_msg &&
               !tj.active) {
        for (i = 0; i < 5u; i++) {
            CHECK(sm.to[i] == gamma_apply(set.gamma, cf.raw[i]), "a clean WS2805 frame: channel %u shows %u, sent %u", i,
                  sm.to[i], gamma_apply(set.gamma, cf.raw[i]));
        }
    } else if (cf.kind == CF_WS2915 && (cf.locked_at_send == PROTO_WS2915_32 || cf.locked_at_send == PROTO_WS2915_26) &&
               proto_active(&frx.ps) == cf.locked_at_send && !cf.cfg_msg && !tj.active) {
        for (i = 0; i < 5u; i++) {
            CHECK(sm.to[i] == cf.ch[i], "a clean WS2915 frame: channel %u shows %u, sent %u", i, sm.to[i], cf.ch[i]);
        }
    }
}

static void sim_tick(void)
{
    sim_ns += 50000u;
    sim_ticks++;
    wire_fill(sim_ns);
    wave_to(sim_ns);
    soak_tim4.CNT = (uint16_t)(edges_total - edges_rearm);
    soak_adc1.RDATAR = load_raw();
    soak_gpiob.INDR = sim_pad_low ? 0xFFFFFBFFu : 0xFFFFFFFFu;
    poll_isr();
}

static void pass(void)
{
    SwitchToFiber(fib_fw);
    if (sim_crash || sim_reset) {
        return;
    }
    inv_fw();
    check_clean();
}

static const char *const con_words[] = { "help", "status", "get", "set", "period", "save", "defaults", "identify", "version",
    "watch", "protocol", "preset", "dither", "thresh", "feedback", "stagger", "floor", "minpulse", "loss", "smooth", "gamma",
    "scene", "scenefade", "ilimit", "ical", "softstart", "0", "1", "2", "3", "14", "100", "255", "1440", "65535", "5000", "x" };

static void console_random(void)
{
    uint32_t k, n;
    con_in_r = con_in_w = 0;
    con_out_n = 0;
    if (chance(30)) {                                    /* random bytes */
        n = rrange(1, 200);
        for (k = 0; k < n; k++) {
            con_in[con_in_w++] = (char)(1u + rint_(255));
        }
        con_in[con_in_w++] = '\r';
        return;
    }
    n = rrange(1, 5);
    for (k = 0; k < n && con_in_w + 20u < sizeof con_in; k++) {
        const char *s = con_words[rint_(sizeof con_words / sizeof con_words[0])];
        con_in_w += (uint32_t)sprintf(con_in + con_in_w, "%s ", s);
    }
    con_in[con_in_w++] = chance(50) ? '\r' : '\n';
}

static void *map_handover(void)
{
    void *ho = VirtualAlloc((void *)(uintptr_t)UPD_HANDOVER_ADDR, 0x1000u, 0x3000u, 0x04u);
    if (ho != (void *)(uintptr_t)UPD_HANDOVER_ADDR) {
        printf("robust_fw: cannot map the hand-over block at 0x%08X\n", UPD_HANDOVER_ADDR);
        exit(2);
    }
    memset(ho, 0, 0x1000u);
    return ho;
}

/* Regression (main.c fx_want, 0.6.2): the five power-on effect heads' CRC-8s were summed into one value with
 * overlapping shifts (crc[i] << (8 + 4 i)), so two lights whose SCENE_EFFECT content changed in the same quiet period
 * with CRC deltas d0 = -16 d1 looked unchanged and were never stored. Here: an erased flash, two lights with a
 * power-on candle each, stored once; then both effects change to such content; the store must be written again. */
static int regress_fx(void)
{
    uint8_t s[32], roles0[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 }, roles1[5] = { 0, ULTRA_ROLE_BRIGHTNESS, 0, 0, 0 };
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, mn[5] = { 0, 0, 0, 0, 0 };
    uint8_t p0[4] = { 0x9C, 0x40, 0, 100 }, p1[4] = { 0x80, 0x00, 0, 80 }, q[4];
    uint32_t w0, w1, t, tries, c0, c1, want0, want1;
    uint8_t seq = 1;
    g_seed = 0xF0F0ull;
    rs = 0x1234567ull;
    flash_map();
    memcpy(fl + 0x10u, "QDBL\x01\x01\x03", 7);
    fl[0x17] = 0;
    mk_desc(fl + 0x2000u, 0x000602FFu, 1, 0x8u, 1);
    mk_desc(desc_this, 0x000602FFu, 1, 0x8u, 1);
    map_handover();
    uid_hash = 1;
    chain_n = 1;
    wire_quiet = 1;                                      /* (0.6.4: the random traffic's SETs ramp the lights up, and the
                                                            effect store waits for a low moment: nothing rising) */
    fib_main = ConvertThreadToFiber(0);
    fib_fw = CreateFiber(1u << 20, fw_entry, 0);
    SwitchToFiber(fib_fw);
    frx.ps.mode = PMODE_ULTRA;                           /* forced Ultra: the main loop switches the engine on */
    for (t = 0; t < 40u; t++) {
        sim_tick();
        pass();
    }
    CHECK(ultra_on, "regress fx: Ultra mode on");
    /* the host defines two lights with a scene and a power-on candle each (the slots straight into the engine) */
    ultra_slot_light_def(s, 0, seq++, roles0, 1000, 0, mn, a); (void)ultra_on_slot(&ul, s, tick_now);
    ultra_slot_light_def(s, 1, seq++, roles1, 1000, 0, mn, a); (void)ultra_on_slot(&ul, s, tick_now);
    ultra_slot_scene(s, 0, seq++, 1, 10, 65535, a, a); (void)ultra_on_slot(&ul, s, tick_now);
    ultra_slot_scene(s, 1, seq++, 1, 10, 65535, a, a); (void)ultra_on_slot(&ul, s, tick_now);
    ultra_slot_scene_effect(s, 0, seq++, ULTRA_FX_CANDLE, p0, 4); (void)ultra_on_slot(&ul, s, tick_now);
    ultra_slot_scene_effect(s, 1, seq++, ULTRA_FX_CANDLE, p1, 4); (void)ultra_on_slot(&ul, s, tick_now);
    scene_check = 1;
    scene_check_ms = ms_now - SCENE_SAVE_MS - 1000u;      /* the quiet time is over: compare and store */
    for (t = 0; t < 30000u; t++) {                       /* (0.6.4: in a low moment, every output below 15 % for
                                                            500 ms: the lights are dark here) */
        sim_tick();
        pass();
    }
    w0 = W_FX();
    CHECK(w0 >= 3u && fx_mask == 0x03u, "regress fx: the two effects stored (%u writes, mask %x)", w0, fx_mask);
    /* new content for both whose head CRCs move by -16 and +1: the summed checksum stays the same */
    c0 = crc8_smbus((const uint8_t *)&ul.light[0], ULTRA_HEAD_BYTES);
    c1 = crc8_smbus((const uint8_t *)&ul.light[1], ULTRA_HEAD_BYTES);
    want0 = (c0 - 16u) & 0xFFu;
    want1 = (c1 + 1u) & 0xFFu;
    for (tries = 0; tries < 100000u; tries++) {
        q[0] = (uint8_t)rnd(); q[1] = (uint8_t)rnd(); q[2] = 0; q[3] = (uint8_t)rrange(50, 150);
        ultra_slot_scene_effect(s, 0, seq++, ULTRA_FX_CANDLE, q, 4);
        (void)ultra_on_slot(&ul, s, tick_now);
        if (crc8_smbus((const uint8_t *)&ul.light[0], ULTRA_HEAD_BYTES) == want0) {
            break;
        }
    }
    for (tries = 0; tries < 100000u; tries++) {
        q[0] = (uint8_t)rnd(); q[1] = (uint8_t)rnd(); q[2] = 0; q[3] = (uint8_t)rrange(50, 150);
        ultra_slot_scene_effect(s, 1, seq++, ULTRA_FX_CANDLE, q, 4);
        (void)ultra_on_slot(&ul, s, tick_now);
        if (crc8_smbus((const uint8_t *)&ul.light[1], ULTRA_HEAD_BYTES) == want1) {
            break;
        }
    }
    CHECK(crc8_smbus((const uint8_t *)&ul.light[0], ULTRA_HEAD_BYTES) == want0 &&
          crc8_smbus((const uint8_t *)&ul.light[1], ULTRA_HEAD_BYTES) == want1, "regress fx: colliding content found");
    scene_check = 1;
    scene_check_ms = ms_now - SCENE_SAVE_MS - 1000u;
    for (t = 0; t < 30000u; t++) {
        sim_tick();
        pass();
    }
    w1 = W_FX();
    CHECK(w1 > w0, "regress fx: both power-on effects changed, nothing was written (%u writes before, %u after): the "
          "summed checksum of the head CRCs did not change", w0, w1);
    printf("regress fx: %s (%d failures)\n", g_fail ? "FAILED" : "passed", g_fail);
    return g_fail ? 1 : 0;
}

int main(int argc, char **argv)
{
    uint32_t sim_s = 4, i, settings_w, scene_w, cal_w, energy_w, fx_w, ms0, tick0;
    uint64_t end_ns, dense_ticks = 0;
    void *ho;
    if (argc > 1 && strcmp(argv[1], "regress") == 0) {
        return regress_fx();
    }
    g_seed = argc > 1 ? strtoull(argv[1], 0, 16) : 0x600D5EEDull;
    if (argc > 2) {
        sim_s = (uint32_t)strtoul(argv[2], 0, 10);
    }
    rs = (g_seed * 0x9E3779B97F4A7C15ull) | 1u;
    for (i = 0; i < 8u; i++) {
        (void)rnd64();
    }
    flash_init();
    ho = map_handover();
    if (chance(40)) {                                    /* the bootloader left a result */
        upd_handover_t *h = (upd_handover_t *)ho;
        h->result = rint_(6);
        h->result_version = rnd();
        h->wd_resets = rint_(4);
        h->reset_flags = rnd();
        if (chance(80)) {
            upd_ho_seal(h);
        } else {
            h->magic = UPD_HO_MAGIC;
            h->check = rnd();
        }
    }
    /* the images the hub may announce: this one (0.6.2: "already active"), a newer one (taken: the run ends in the
     * reset into the bootloader), one for other hardware (NACK) */
    mk_desc(desc_this, 0x000602FFu, 1, 0x8u, 1);
    mk_desc(desc_newer, 0x000700FFu, 1, 0x8u, 1);
    mk_desc(desc_other, 0x000700FFu, 2, 0x8u, 1);
    uid_hash = rnd();
    chain_n = chance(60) ? rrange(1, 4) : rrange(1, 50);
    chain_pos = rint_(chain_n);
    fs_scale = urand(0.978, 1.022);
    for (i = 0; i < 5u; i++) {
        ld_ma[i] = chance(80) ? rrange(0, 3000) : rrange(0, 12000);
    }
    supply_mv = chance(70) ? 24000u : chance(50) ? 12000u : 48000u;
    /* boot at a random time base, as the soak does */
    ms0 = chance(50) ? rnd() : 0u;
    tick0 = chance(50) ? rnd() : 0u;
    ms_now = ms0;
    tick_now = tick0;
    last_data_ms = changed_ms = st_ms = scene_changed_ms = scene_check_ms = ultra_first_ms = temp_ms = lim_ms = ms0;
    wire_t = (uint64_t)urand(0, 30e6);                   /* the controller may already be sending at power-up */
    fib_main = ConvertThreadToFiber(0);
    fib_fw = CreateFiber(1u << 20, fw_entry, 0);
    SwitchToFiber(fib_fw);                               /* the boot, up to the main loop's first kick */
    if (sim_crash) {
        printf("  FAIL seed %llx: the boot raised exception %08x (flash: settings preset %u steps %u)\n", g_seed, sim_crash,
               set.pwm_preset, set.raw_steps);
        g_fail++;
    }
    end_ns = (uint64_t)sim_s * 20000u;                   /* dense ticks (the sparse jumps and stalls come on top) */
    while (dense_ticks < end_ns && !sim_crash && !sim_reset) {
        dense_ticks++;
        sim_tick();
        pass();
        if (sim_crash || sim_reset) {
            break;
        }
        /* the other inputs, now and then */
        if (rint_(2000) == 0u) {
            console_random();
        }
        if (rint_(20000) == 0u) {
            supply_mv = chance(50) ? (uint32_t)(supply_mv * urand(0.5, 0.95)) : chance(50) ? 24000u : chance(50) ? 12000u : 48000u;
        }
        if (rint_(40000) == 0u) {
            short_ticks = rrange(1, 10);
        }
        if (rint_(20000) == 0u) {
            sim_pad_low ^= 1u;
        }
        if (rint_(150000) == 0u) {                       /* a sparse stretch: the time bases jump (as a stall) */
            uint32_t ms = rrange(1, 60000);
            sim_ns += (uint64_t)ms * 1000000u;
            sim_ticks += 20u * (uint64_t)ms;
            tick_now += 20u * ms;
            ms_now += ms;
            vs_pending += ms;
            wire_t = sim_ns + 1000000u;                  /* the wire restarts after the jump */
            pq_head = pq_tail = pq_edge = 0;
            wave_to(sim_ns);
        }
    }
    if (sim_crash) {
        printf("  FAIL seed %llx t=%.3fs: the firmware raised exception %08x\n", g_seed, (double)sim_ns / 1e9, sim_crash);
        g_fail++;
    }
    /* the write budgets over the simulated time (plus the boot's sticky protocol / the hand-over) */
    settings_w = writes_in(SETTINGS_ADDR, 1u) + writes_in(SETTINGS_B_ADDR, 1u);
    scene_w = writes_in(SCENE_LOG_ADDR, 1u) + writes_in(SCENE_B_ADDR, 1u);
    cal_w = writes_in(CAL_PAGE_ADDR, 1u) + writes_in(CAL_B_ADDR, 1u);
    energy_w = writes_in(LIFE_PAGE0_ADDR, 1u) + writes_in(LIFE_PAGE1_ADDR, 1u);
    fx_w = W_FX();
    {
        uint32_t s = (uint32_t)(sim_ns / 1000000000u) + 1u;
        CHECK(settings_w <= STORE_BUDGET_CAP + s / (STORE_REFILL_MS / 1000u) + 2u, "settings writes %u in %u s",
              settings_w, s);
        CHECK(scene_w <= SCENE_BUDGET_CAP + s / (SCENE_REFILL_MS / 1000u) + 2u, "scene writes %u in %u s", scene_w, s);
        CHECK(cal_w <= CAL_BUDGET_CAP + s / (CAL_REFILL_MS / 1000u) + 2u, "calibration writes %u in %u s", cal_w, s);
        CHECK(energy_w <= LIFE_BUDGET_CAP + s / (LIFE_REFILL_MS / 1000u) + 2u, "lifetime record writes %u in %u s",
              energy_w, s);
        CHECK(fx_w <= 7u * (FX_BUDGET_CAP + s / 21600u + 2u), "effect store writes %u in %u s", fx_w, s);
    }
    if (!sim_crash && !sim_reset) {                      /* the console still answers */
        uint32_t t;
        con_in_r = con_in_w = 0;
        con_out_n = 0;
        memcpy(con_in, "\rstatus\r", 8);                 /* (the first key ends a `watch` the random input may have started) */
        con_in_w = 8;
        for (t = 0; t < 2000u; t++) {
            sim_tick();
            pass();
        }
        CHECK(strstr(con_out, "uptime=") != 0, "the console's status after the run");
    }
    printf("seed %llx: %u s, %u passes, chain %u pos %u, proto %d, frames %u decoded %u headers %u, flash writes s%u c%u "
           "k%u e%u f%u%s%s: %s (%d failures)\n", g_seed, (uint32_t)(sim_ns / 1000000000u), sim_passes, chain_n, chain_pos,
           (int)proto_active(&frx.ps), frames, stat.decoded, stat.ultra_headers, settings_w, scene_w, cal_w, energy_w, fx_w,
           sim_reset ? ", reset into the bootloader (update taken)" : "", sim_crash ? ", CRASHED" : "",
           g_fail ? "FAILED" : "passed", g_fail);
    return g_fail ? 1 : 0;
}
