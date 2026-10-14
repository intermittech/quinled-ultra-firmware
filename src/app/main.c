/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED dig2analog-ultra-5ch firmware.
 *
 * Data path (no CPU in the forwarding path):
 *   DIN --> edge counter (TIM3 CH1 capture -> TIM4) --> FWD_EN at the planned edge counts (DMA write)
 *       |   --> U2 (AND) --> data out                                                   (pin map: hal/board.h)
 *       \-> SPI1 RX-only 36 MHz --> DMA ring --> (poll ISR copies the frame) --> decoder in the main loop
 * Output path: WS frames -> gamma (WS2805) -> smoothing / fades,  or  Ultra commands -> light engine (ultra.c)
 *              -> dither + dim floor tables --> DMA bursts them into TIM1 / TIM3 CCRs, one entry per PWM period.
 * The 20 kHz poll tick (SysTick) reads the edge count: frame start / end (DIN idle 100-150 us), gate re-arm, ring copy
 * of the frame start (own bits) and of its end (the Ultra header trailer), 50 us tick / 1 ms time base
 * (core/frame_track.h). The main loop decodes both, decides per frame (core/frame_rx.c: show, classify, lock) and
 * hands the gate plan for the following frames back to the ISR.
 * Boot: the tracker arms on an idle line, so the first complete frame after power-up is used; in a forced protocol
 * mode, or in auto mode with the protocol stored at the last lock ("sticky auto"), that frame is gated and decoded.
 * Firmware update (core/upd_app.h; wiki: Updating over the data line): UPD frames make the board transparent (forward and
 * capture whole 384-bit frames); an ANNOUNCE it takes hands over to the bootloader by a software reset. UPD_APP = 1
 * (the build behind the bootloader, tools/build_release.sh) uses the hand-over block and the descriptor page. */
#include "ch32v20x.h"
#include "../hal/hw.h"
#include "../core/ack.h"
#include "../core/cli.h"
#include "../core/climit.h"
#include "../core/config.h"
#include "../core/dark.h"
#include "../core/dither.h"
#include "../core/frame_rx.h"
#include "../core/frame_track.h"
#include "../core/fwver.h"
#include "../core/gamma.h"
#include "../core/health.h"
#include "../core/isense.h"
#include "../core/lto.h"
#include "../core/mcutemp.h"
#include "../core/ocled.h"
#include "../core/pwrev.h"
#include "../core/vsense.h"
#include "../core/proto.h"
#include "../core/scene.h"
#include "../core/smooth.h"
#include "../core/sstart.h"
#include "../core/stagger.h"
#include "../core/testjig.h"
#include "../core/ultra.h"
#include "../core/upd_app.h"
#include "../core/vdip.h"
#include "../core/ws_decode.h"

#ifndef FW_VERSION
#define FW_VERSION       "0.6.4"   /* release builds: the release version (tools/build_release.sh) */
#endif
#ifndef UPD_APP
#define UPD_APP 0                  /* 1: linked behind the bootloader (vendor/wch/Link_app.ld, build_release.sh) */
#endif
#define UPD_WD_CLEAR_S   10u       /* the bootloader's watchdog-loop count is cleared after this long running */
#define LOSS_TIMEOUT_MS  2000u     /* WS modes: signal loss = no valid frame for 2 s */
#define LOSS_FADE_MS     1000u     /* fade-off duration */
#define SAVE_DEBOUNCE_MS 2000u     /* settings saved 2 s after the last change */
static const ws_timing_t ws_tm = WS_TIMING_DEFAULT;   /* the decoder's bit timing (one copy for its three callers) */

/* ------------------------------------------------------------------------------- shared with the poll ISR
 * The ISR writes volatile variables, ft (the main loop only with interrupts off) and the copies din_copy / tail_copy
 * (read after cap_ready / fe_ready, behind a BARRIER). frx and plan_buf are the main loop's; the ISR only reads them,
 * published by the BARRIER before plan_idx. */
static frx_t frx;                          /* protocol state and per-frame decisions (core/frame_rx.h) */
static gate_plan_t plan;                   /* plan of the frame in progress (the ISR's; the main loop writes it only
                                              with interrupts off) */
static gate_plan_t plan_buf[2];            /* next plan, written by the main loop (double buffer) */
static volatile uint8_t plan_idx;
static ftrack_t ft;                        /* frame tracking (ISR; the main loop only with interrupts off) */
static volatile uint8_t rx_ready;          /* main has set up the plans: the ISR starts tracking frames */
static volatile uint16_t cap_start;        /* ring word 60 us before the frame's first edge (inside the gap) */
static volatile uint16_t cap_words;
static volatile uint8_t cap_ready;
static uint8_t cap_accept;                 /* ISR: this frame owns the empty capture mailbox */
static volatile uint16_t cap_bits;         /* bits to decode */
static volatile uint16_t cap_own;          /* this board's bits under the plan of that frame */
static volatile uint16_t cap_skip;         /* WS2915: the capture starts after the header pause (hdr bits) */
static volatile uint8_t cap_proto;
static volatile uint32_t cap_frame;
static volatile uint8_t fe_ready;          /* frame end: total bits for the classifier */
static volatile uint32_t fe_bits;
static volatile uint32_t fe_frame;
static volatile uint32_t fe_tick;          /* tick of the frame's last edge (0-50 us after it): Ultra clock */
static volatile uint16_t tail_words;       /* tail_copy holds the frame's end (Ultra lengths only), 0 = none */
static uint32_t edge_tick;                 /* ISR: tick and ring position of the latest new edges */
static uint16_t edge_pos;
static volatile uint32_t ms_now, tick_now; /* 1 ms and 50 us time bases */
static uint32_t poll_sub;                  /* ticks into the ms (poll_isr, flash_poll_isr; 32 bits: no zero-extension
                                              every tick) */
static uint8_t adc_hold;                   /* the next tick's PA7 sample is held (the temperature sequence) */
static volatile uint32_t frames;
/* board current sense (core/isense.h): the tick sums the ADC samples and trips; the main loop limits (isense_apply
 * in outputs_update), calibrates and retries */
static isense_t is;

#define HINT_WORDS 135u                    /* 60 us at 36 MHz, 16 samples per word */
#define TAIL_WORDS 576u                    /* 256 us up to the frame's last edge: 64 bits at 1.5 us + hiccups + tick */
#define TAIL_PAST  4u                      /* words past the ring position of the last-edge tick */
/* the end of the frame, for the Ultra header (ws_decode_last); 4-byte aligned like din_copy: copy_run writes it in
 * 32-bit words (an array of uint16_t is only 2-byte aligned: today it lands on a word by the sizes before it) */
static uint16_t tail_copy[TAIL_WORDS] __attribute__((aligned(4)));

/* k words s -> d, both 4-byte aligned: 32-bit copies, four per step (the poll ISR's longest path); returns the end */
static uint16_t *copy_run(uint16_t *d, const uint16_t *s, uint32_t k)
{
    uint32_t *dw = (uint32_t *)d;
    const uint32_t *sw = (const uint32_t *)s, *se = sw + (k >> 1);
    while (se - sw >= 4) {
        uint32_t a = sw[0], b = sw[1], c = sw[2], e = sw[3];
        dw[0] = a;
        dw[1] = b;
        dw[2] = c;
        dw[3] = e;
        dw += 4;
        sw += 4;
    }
    while (sw < se) {
        *dw++ = *sw++;
    }
    d = (uint16_t *)dw;
    if (k & 1u) {
        *d++ = *(const uint16_t *)sw;
    }
    return d;
}

/* ring words [start, end) -> dst (at most max words), two straight runs; returns the words copied */
static uint32_t ring_copy(uint16_t *dst, uint32_t start, uint32_t end, uint32_t max)
{
    uint32_t n = (end - start) & (DIN_RING_WORDS - 1u), first;
    if (n > max) {
        n = max;
    }
    first = DIN_RING_WORDS - start;
    if (first > n) {
        first = n;
    }
    dst = copy_run(dst, &din_ring[start], first);              /* start is even: aligned */
    (void)copy_run(dst, din_ring, n - first);                  /* when it wraps, first is even too (ring size even) */
    return n;
}

/* the tracker's events of a tick but FT_EDGE (a few ticks per frame): out of line, so that the every-tick path of
 * poll_isr keeps no callee-saved registers (0.6.4: claude-cpu; fable-algo's idle-line test before the whole tracking
 * gave the idle tick the same and cost the busy ticks 11 cycles) */
static NOINLINE void poll_events(uint32_t ev, uint16_t c)
{
    if ((ev & FT_CAP_COPY) && cap_accept) {                /* one tick after our last bit: the DMA has surely written it */
        cap_words = (uint16_t)ring_copy(din_copy, cap_start, din_ring_pos(), DIN_COPY_WORDS);
        cap_bits = ft.cap_bits;
        cap_ready = 1;
    }
    if ((ev & (FT_CAP_START | FT_CAP_DATA)) == FT_CAP_START) {
        cap_accept = (uint8_t)!cap_ready;  /* drop a new capture while the old one is being consumed */
    }
    if ((ev & FT_CAP_START) && cap_accept) {               /* first edges (0-50 us ago): a ring position inside the reset gap */
        cap_start = (uint16_t)((din_ring_pos() - HINT_WORDS) & (DIN_RING_WORDS - 2u));   /* even */
        if (ev & FT_CAP_DATA) {
            cap_skip = plan.hdr_bits;      /* WS2915 data after the header pause (the gap is >= 100 us) */
        } else {
            cap_skip = 0;
            cap_own = plan.own_bits;
            cap_proto = (uint8_t)proto_active(&frx.ps);
            cap_frame = frames;
        }
    }
    if (ev & (FT_FRAME_END | FT_BOOT_ARM)) {
        const gate_plan_t *g = &plan_buf[plan_idx];   /* (the main loop writes only the other one) */
        if ((ev & FT_CAP_SHORT) && cap_accept) {           /* frame shorter than the capture: decode what came */
            cap_words = (uint16_t)ring_copy(din_copy, cap_start, din_ring_pos(), DIN_COPY_WORDS);
            cap_bits = ft.cap_bits;
            cap_ready = 1;
        }
        if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END && !fe_ready) {
            uint32_t total = c + ((uint32_t)ft.end_wraps << 16);
            fe_bits = total;                   /* the main loop classifies the frame and plans the next ones */
            fe_frame = frames;
            fe_tick = edge_tick;
            /* the end of a frame of an Ultra length: its last 64 bits are the header (read in the main loop) */
            tail_words = frx_tail_wanted(&frx, total)
                ? (uint16_t)ring_copy(tail_copy, (edge_pos + TAIL_PAST - TAIL_WORDS) & (DIN_RING_WORDS - 2u),
                                      edge_pos + TAIL_PAST, TAIL_WORDS)
                : 0u;
            fe_ready = 1;
        }
        gate_rearm(g);                     /* the next frame starts >= 280 us after the last edge */
        plan.start_open = g->start_open;   /* (field by field: a struct copy is a memcpy call) */
        plan.t1 = g->t1;
        plan.t2 = g->t2;
        plan.own_bits = g->own_bits;
        plan.hdr_bits = g->hdr_bits;
        if (ev & FT_FRAME_END) {
            frames++;
        }
    }
}

void poll_isr(void);                       /* 20 kHz tick, entered via isr.S (HPE + mret) */
void poll_isr(void)
{
    uint32_t c = DIN_EDGES(), own, ev;

    POLL_ACK();
    tick_now++;
    if (is.enabled) {                      /* (not before isense_setup at boot) */
        if (adc_hold) {
            /* (0.6.4, gpt-hw H7) the MCU temperature's injected sequence (84 us, started at the last tick) holds
             * PA7's conversion: this tick would read the value the last one counted, and one high sample would
             * trip as two. Not counted; the next tick has a new one (84 + 7 us < 100 us). */
            adc_hold = 0;
        } else if (isense_tick(&is, ISENSE_RAW())) {
            PWM_TRIP();                    /* a short: all outputs off now (pwm_force_off(1) inline), no table rebuild */
        }
    }
    if (++poll_sub >= POLL_HZ / 1000u) {
        poll_sub = 0;
        ms_now++;
        adc_hold = (uint8_t)hw_adc_ms();   /* the ADC's injected group: voltage channels / MCU temperature */
    }
    /* (fable-algo) an idle line: ftrack_tick would return no event and change nothing (no edges, the count at 0
     * without wraps, no copy due, armed) */
    if (!rx_ready || (c == 0u && ft.last_cnt == 0u && ft.wraps == 0u && ft.cap_state != 2u && ft.armed)) {
        return;
    }
    /* a provisional (stored) protocol captures an Ultra slot too: a different protocol locks at once (an update
     * session's transparent plan: still its whole 384-bit frame) */
    own = plan.own_bits;
    if (frx.ps.provisional && own < PROTO_DETECT_BITS) {
        own = PROTO_DETECT_BITS;
    }
    ev = ftrack_tick(&ft, (uint16_t)c, (uint16_t)own, plan.hdr_bits);
    if (ev & FT_EDGE) {                    /* the last edges so far: where the frame ends (in time and ring) */
        edge_tick = tick_now;
        edge_pos = (uint16_t)din_ring_pos();
    }
    if (ev & ~FT_EDGE) {
        poll_events(ev, (uint16_t)c);
    }
}

/* The tick during a flash operation (0.6.4, hw.c flash_op: through VTF, from SRAM, the HCLK halved and SysTick's
 * compare with it): the time bases and the fast trip, as poll_isr (the same state: a high sample before the operation
 * still counts toward the two), nothing else (no frames: the gate is closed; no injected ADC sequences). */
RAMCODE void flash_poll_isr(void)
{
    POLL_ACK();
    tick_now++;
    if (++poll_sub >= POLL_HZ / 1000u) {
        poll_sub = 0;
        ms_now++;
    }
    if (is.enabled) {
        if (adc_hold) {
            adc_hold = 0;
        } else if (isense_tick(&is, ISENSE_RAW())) {
            hw_flash_trip();               /* all outputs off (SRAM; pwm_forced as pwm_force_off(1)) */
        }
    }
}

/* ------------------------------------------------------------------------------- main loop state */
static settings_t set;                     /* stored settings */
static store_ext_t sext;                   /* stored with them: current limit and calibration (store.h) */
static settings_t eff;                     /* in effect: set, in Ultra mode overlaid by the host's CONFIG */
static config_rx_t rx;
static ack_t ack;
static smooth_t sm;
static ultra_t ul;
static uint16_t uout[5];
static uint8_t ultra_on;
static dither_opts_t dopt;
static cli_t cli;
static cli_status_t stat;
static uint8_t outputs_dirty = 1, settings_dirty, signal_lost;
static uint32_t steps;
static uint8_t len, run_stagger = 0xFF;
static uint32_t last_data_ms, changed_ms;
static scene_budget_t set_budget;          /* write budget of the settings page (store.h STORE_BUDGET_*) */
static uint32_t up_s;                      /* uptime in s: `uptime` and the long ages (ms_now wraps after 49.7 days) */
static uint16_t out_now[5];                /* output values in use, = the value each channel's table holds (0.6.4,
                                              gpt-opt2: one vector; main loop only, no ISR or build() reads it) */
static uint8_t built_ok;
static uint16_t st_o2 = STAGGER_FIXED_O2, st_o3 = STAGGER_FIXED_O3;
static uint8_t st_force;
static uint16_t st_seen[5];
static uint32_t st_ms;
/* power-on scene (core/scene.h): shown from power-up until data comes, kept in its own flash log */
#define SCENE_TAKEOVER_MS 3000u            /* Ultra: hold it at most this long after the first header */
static scene_run_t srun;
static scene_t scene_stored;               /* in flash */
static scene_t scene_want;                 /* the latest scene; written when it differs from scene_stored */
static scene_budget_t sbudget;             /* write budget of the scene log */
static uint8_t scene_major_pend, scene_check, ultra_seen;
/* firmware update, app side (core/upd_app.h) */
static upd_app_t upd;
/* the firmware version (core/fwver.h): the descriptor's in release builds, else FW_VERSION; 0 = development build */
static uint32_t fw_version;
static char fw_version_text[FWVER_TEXT_MAX];
#if UPD_APP
#define HO ((upd_handover_t *)UPD_HANDOVER_ADDR)  /* outside every linker section (Link_app.ld) */
static uint8_t upd_wd_cleared;
#endif
static uint32_t scene_changed_ms, scene_check_ms, ultra_first_ms;
/* board limit in effect (core/climit.h) and the stored current calibration (core/calstore.h) */
static climit_t clim;
static uint32_t lim_ms;
static uint16_t cal_cnt;
static uint8_t cal_present, cal_save_pend;
static scene_budget_t cal_budget;          /* write budget of the calibration page (calstore.h CAL_BUDGET_*) */
static dark_t dk;                          /* dark hold of the pin's calibration (Ultra IDENTIFY 3 / 4, core/dark.h) */
/* the overcurrent signal on D9 (core/ocled.h) and D9's level as set; the power events (core/pwrev.h: the state a
 * Diff-Power ultra receiver reports on its STATUS / POWER pages; local here) */
static ocled_t ocl;
static uint8_t led_now;
static pwrev_t pw;
/* MCU temperature (core/mcutemp.h): mt.c10 / mt.valid / mt.vdda_mv for the console and the telemetry (STATUS) */
static mcutemp_t mt;
static uint32_t temp_ms;
static uint8_t temp_started;
static int16_t temp_peak = MCUTEMP_NA;     /* the highest valid MCU temperature since boot, and when (uptime, s) */
static uint32_t temp_peak_s;
/* voltage channels (core/vsense.h): PA4 the supply, PA0 the spare input on TP8 */
static vsense_t vs_sup, vs_aux;
static vdip_t vdip;                        /* supply dips on PA4 (core/vdip.h) */
static energy_t en;                        /* input energy (core/energy.h) */
static health_t hl;                        /* channel health (core/health.h) */
static sstart_t ss;                        /* soft start (core/sstart.h) */
/* the power-on effect (SCENE_EFFECT): what is stored (lights, their CRC-8), the channels still running it, the
 * engine untouched by a host since boot (it holds the power-on effect), a write pending, its budget */
static uint8_t fx_mask, fx_crc[ULTRA_LIGHTS], fx_ch, ul_used, fx_pend;
static scene_budget_t fx_budget;
#define FX_BUDGET_CAP 4u
#define FX_REFILL_MS  21600000u            /* one write per 6 h sustained: >= 6 years at 10k cycles (2 erases each) */
static testjig_t tj;                       /* test jig mode (core/testjig.h): TP9 grounded */

static void flash_stall_begin(uint32_t may);
static uint32_t flash_stall_end(uint32_t r);

/* ------------------------------------------------------------------------------- the flash: when (0.6.4)
 * Quindor's policy (2026-10-14). An erase / program halves HCLK (RM 32.1): the PWM runs at half frequency, the duty
 * unchanged, and the main loop waits (renders, fades) for as long as it takes; the current trip keeps running (hw.c:
 * from SRAM). So:
 *  - low: every output below 15 % of full scale (dark, or really dimmed down: the halved frequency is least visible
 *    there) for FLASH_LOW_MS, no calibration / test jig pattern / soft start, nothing fading up (a WS ramp's target, an
 *    Ultra ramp that ends brighter than it shows); and at power-up before the main loop: anything, erases too;
 *  - brighter: a program into erased flash only (no erase), at most one per FLASH_BRIGHT_MS (pending changes wait:
 *    the newest values are written); erases wait for a low moment, which also erases each store's next page ahead;
 *  - at a supply dip: programs into erased flash (the lifetime record's dip write, any store's pending write);
 *  - the console's `save`: a program at once (a user's command, not housekeeping).
 * "Below 15 %" is the outputs as the tables hold them (out_now: transitions and effects as they show), which is what
 * the PWM runs while the main loop waits: nothing can cross the level during an operation. Every regular write also
 * waits until the console sends nothing (hw.c flash_op: the baud halves with HCLK). */
#define FLASH_LOW_LEVEL  9830u             /* 15 % of 65535 */
#define FLASH_LOW_MS     500u              /* every output below it this long: an effect's peaks reset it */
#define FLASH_BRIGHT_MS  3600000u          /* brighter: one write per hour */
#define FM_LOW    1u                       /* flash_may: low (or boot): erases too */
#define FM_PROG   2u                       /* a program into erased flash (a dip, the console's save) */
#define FM_BRIGHT 3u                       /* ... brighter: with the hour's token */
static uint32_t flash_hi_ms;               /* the last ms the outputs were not low */
static scene_budget_t flash_bright;        /* the writes while brighter */
static uint8_t flash_boot = 1;             /* before the main loop: the outputs are dark */
static uint8_t fx_clear_pend;              /* a factory reset: the power-on effects' erase is due */

/* may a store write now? need_erase: its next record needs an erase first (only while low); prog: FM_PROG when a
 * program may run whatever the outputs show (a dip, the console's save). Returns FM_LOW / FM_PROG / FM_BRIGHT
 * (flash_stall_begin takes the brighter state's token), 0 = not now. */
static uint32_t flash_may(uint32_t need_erase, uint32_t prog)
{
    if (prog != FM_PROG && uart_busy()) {
        return 0;                                        /* (the console first: hw.c flash_op) */
    }
    if (flash_boot || (ms_now - flash_hi_ms >= FLASH_LOW_MS && !vdip.in_dip)) {
        return FM_LOW;
    }
    if (need_erase) {
        return 0;
    }
    if (prog == FM_PROG || vdip.in_dip) {
        return FM_PROG;
    }
    return budget_avail(&flash_bright, ms_now) ? FM_BRIGHT : 0u;
}

/* every ms while low (and once at power-up): one erase that is due, so a later write needs none. Not before
 * LIFE_PREP_S after power-up (a steady supply) except at power-up itself (before the outputs start: 0.6.3's
 * lifetime erase there keeps wall-switched boards going). */
static uint32_t (*const erase_due[4][2])(void) = {
    { life_pending, life_prepare }, { settings_pending, settings_prepare }, { cal_pending, cal_prepare },
    { scene_pending, scene_prepare } };

static void flash_erase_ahead(void)
{
    uint32_t i;
    if ((!flash_boot && up_s < LIFE_PREP_S) || flash_may(1u, 0u) != FM_LOW) {
        return;
    }
    if (fx_clear_pend) {
        flash_stall_begin(FM_LOW);                       /* (a factory reset: the power-on effects go) */
        fx_clear_pend = (uint8_t)!flash_stall_end(scene_fx_store(&ul, 0, fx_crc));
        return;
    }
    for (i = 0; i < 4u; i++) {
        if (erase_due[i][0]()) {
            flash_stall_begin(FM_LOW);
            (void)flash_stall_end(erase_due[i][1]());
            return;
        }
    }
}

/* the lifetime record's writes (core/energy.h ENERGY_DUE_*): a regular one as any store's, a dip's always (into an
 * erased entry; none while its next page waits for an erase) */
static uint32_t life_admit(void)
{
    uint32_t m = flash_may(0u, 0u);
    if (m == FM_BRIGHT && life_room() < 2u) {
        m = 0;                                           /* brighter: the last erased entry stays for a dip write */
    }
    return life_pending() == 2u ? 0u : ENERGY_DUE_DIP | (m != 0u ? ENERGY_DUE_SAVE : 0u);
}

/* every ms: the injected group's voltage pairs of that ms; the supply dips; the energy and the lifetime record */
static void vsense_service(void)
{
    uint32_t sum2[2], n, i, lit = 0, hi = tj.active || is.cal_step != 0u || ss.state == SS_RUN;
    hw_vsense_take(sum2, &n);
    vsense_add(&vs_sup, sum2[0], n);
    vsense_add(&vs_aux, sum2[1], n);
    vdip_ms(&vdip, ms_now, up_s, sum2[0], n, is.i_ms);
    for (i = 0; i < 5u; i++) {
        lit |= out_now[i];
        hi |= out_now[i] >= FLASH_LOW_LEVEL || (!ultra_on && sm.dur != 0u && sm.to[i] >= FLASH_LOW_LEVEL);
    }
    if (hi || ((ultra_on || fx_ch) && ultra_rising(&ul, tick_now))) {
        flash_hi_ms = ms_now;                            /* the flash: the outputs are not low (see above) */
    }
    energy_ms(&en, ms_now, vs_sup.mv, is.i_ms, (uint8_t)(lit != 0u));
    n = energy_due(&en, ms_now, vdip.count, life_admit());
    if (n != ENERGY_DUE_NONE) {
        flash_stall_begin(n == ENERGY_DUE_DIP ? FM_PROG : flash_may(0u, 0u));
        if (flash_stall_end(life_store(&en))) {
            energy_saved(&en, up_s);
        }
    }
}

/* once a second, in the main loop: read the conversion started a second ago, start the next one. Not while a current
 * calibration runs (its 30 ms windows read PA7 alone; the injected group would hold the PA7 results for 84 us). */
static void temp_service(void)
{
    uint32_t ts2 = 0, vref2 = 0;
    if (ms_now - temp_ms < 1000u) {
        return;
    }
    temp_ms = ms_now;
    if (temp_started && !hw_temp_read(&ts2, &vref2)) {
        ts2 = vref2 = 0;                   /* not done: counts as a bad sample */
    }
    {
        isense_cal_t c;
        (void)isense_cal_get(&is, &c);
        health_run(&hl, &is, &c);          /* the running model and the dark current against the stored test */
    }
    if (temp_started) {
        mcutemp_sample(&mt, ts2, vref2);
        if (mt.valid && mt.c10 > temp_peak) {
            temp_peak = mt.c10;
            temp_peak_s = up_s;
        }
    }
    temp_started = (uint8_t)(is.cal_step == 0u);
    if (temp_started) {
        hw_temp_start();
    }
}

/* Ultra CONFIG values (host-managed, RAM only) over the stored settings */
static void ultra_overlay(settings_t *e)
{
    const uint16_t *c = ul.cfg;
    uint32_t v = ul.cfg_valid;
#define HAS(id) ((v & (1u << (id))) != 0u)
    if (HAS(ULTRA_CFG_PWM_PERIOD) && c[ULTRA_CFG_PWM_PERIOD] >= PWM_STEPS_MIN) {
        e->pwm_preset = 0xFF;
        e->raw_steps = c[ULTRA_CFG_PWM_PERIOD];
    } else if (HAS(ULTRA_CFG_PWM_PRESET) && c[ULTRA_CFG_PWM_PRESET] < PWM_PRESET_COUNT) {
        e->pwm_preset = (uint8_t)c[ULTRA_CFG_PWM_PRESET];
    }
    if (HAS(ULTRA_CFG_DITHER) && c[ULTRA_CFG_DITHER] <= 2u) e->dither_mode = (uint8_t)c[ULTRA_CFG_DITHER];
    if (HAS(ULTRA_CFG_DITHER_THRESH) && c[ULTRA_CFG_DITHER_THRESH] >= 1u && c[ULTRA_CFG_DITHER_THRESH] <= 255u) {
        e->dither_thresh = (uint8_t)c[ULTRA_CFG_DITHER_THRESH];
    }
    if (HAS(ULTRA_CFG_DIM_FLOOR) && c[ULTRA_CFG_DIM_FLOOR] <= 2u) e->dim_floor = (uint8_t)c[ULTRA_CFG_DIM_FLOOR];
    if (HAS(ULTRA_CFG_MIN_PULSE) && c[ULTRA_CFG_MIN_PULSE] >= 5u && c[ULTRA_CFG_MIN_PULSE] <= 100u) {
        e->min_pulse = (uint8_t)c[ULTRA_CFG_MIN_PULSE];
    }
    if (HAS(ULTRA_CFG_STAGGER) && c[ULTRA_CFG_STAGGER] < STAGGER_MODES) e->stagger = (uint8_t)c[ULTRA_CFG_STAGGER];
    if (HAS(ULTRA_CFG_LOSS) && c[ULTRA_CFG_LOSS] <= 2u) e->signal_loss = (uint8_t)c[ULTRA_CFG_LOSS];
    if (HAS(ULTRA_CFG_FEEDBACK) && c[ULTRA_CFG_FEEDBACK] <= 2u) e->feedback_ch = (uint8_t)c[ULTRA_CFG_FEEDBACK];
#undef HAS
}

/* ------------------------------------------------------------------------------- board current sense */
/* the limit in effect: the stored one (config frame / console) and, in Ultra mode, the host's CONFIG 21; the
 * lower of the two that are set (0 = the board rating, isense clamps to it) */
static uint16_t isense_limit_ma(void)
{
    uint32_t l = (sext.ilimit50 == 0xFFu) ? 0u : (uint32_t)sext.ilimit50 * 50u;
    if (ultra_on && (ul.cfg_valid & (1u << ULTRA_CFG_BOARD_MAX)) && ul.cfg[ULTRA_CFG_BOARD_MAX] != 0u) {
        uint32_t u = ul.cfg[ULTRA_CFG_BOARD_MAX];
        l = (l == 0u || u < l) ? u : l;
    }
    return (uint16_t)(l > 65535u ? 65535u : l);
}

/* a channel's LED current at 100 % (stagger weights without the current sense): Ultra CONFIG 11-15 when set, else
 * the default */
static uint16_t cfg_load_ma(uint8_t ch)
{
    static const uint16_t def_ma[5] = { 3000, 3000, 3000, 5000, 5000 };   /* CH_R, CH_G, CH_B, CH_W1, CH_W2 */
    uint8_t id = (uint8_t)(ULTRA_CFG_CH_CURRENT + ch);
    return (ultra_on && (ul.cfg_valid & (1u << id)) && ul.cfg[id] != 0u) ? ul.cfg[id] : def_ma[ch];
}

/* The board limit in effect (core/climit.h): the lowest of the board rating, the user's limit (stored ilimit / Ultra
 * CONFIG 21) and the fuse. The current sense's soft limiter holds it (measured); the connected load is the calibrated /
 * learned one. Every apply_settings, limit command, finished calibration and once a second (the learned model
 * drifts). */
static void limit_update(void)
{
    lim_ms = ms_now;
    climit_eval(&clim, isense_limit_ma(), ISENSE_BOARD_MA, ISENSE_FUSE_MA, isense_connected_ma(&is));
    isense_set_limit(&is, clim.limit_ma);
}

/* calibration (config frame, console, Ultra IDENTIFY 2): duty_pct 0 = the standard test (each channel alone at
 * 25 / 50 / 75 / 100 %), 5..100 = each channel alone at that duty only */
/* the current sense from scratch (boot, factory reset): the tick is running, so no half-initialised trip level */
static void isense_setup(void)
{
    uint8_t trip;
    __disable_irq();
    trip = is.tripped;                    /* factory reset: keep a hardware force-off paired with a retry */
    isense_init(&is, 1, ISENSE_FUSE_MA, ISENSE_BOARD_MA);
    is.tripped = trip;
    isense_set_gain(&is, ISENSE_GAIN_DEFAULT_Q16 * 100u / ISENSE_AMP_GAIN);   /* board.h: amplifier gain */
    __enable_irq();
}

static uint8_t isense_calibrate(uint8_t duty_pct)
{
    isense_cal_t c;
    if (is.tripped != ISENSE_TRIP_NONE || tj.active) {
        return 0;                          /* (the test jig's pattern keeps the outputs) */
    }
    (void)isense_cal_get(&is, &c);
    health_test_start(&hl, &c);            /* the test before this one (levels 0: none) */
    isense_cal_start(&is, duty_pct == 0u ? 0u : (uint16_t)((uint32_t)duty_pct * 65535u / 100u));
    ack_stop(&ack);                        /* the calibration pattern is the feedback */
    outputs_dirty = 1;
    return 1;
}

/* every ms, before the outputs are rendered: the tick's samples of that ms -> limit, trips, calibration */
static void isense_service(void)
{
    uint32_t sum, n, peak, ev;
    __disable_irq();
    isense_take(&is, &sum, &n, &peak);
    __enable_irq();
    ev = isense_ms(&is, sum, n, peak);
    if (ev & ISENSE_EV_TRIP) {
        pwm_force_off(1);                  /* overload the limit could not hold */
    }
    if (ev & ISENSE_EV_RELEASE) {
        __disable_irq();
        if (is.tripped == ISENSE_TRIP_NONE) {
            pwm_force_off(0);              /* retry (not if the tick tripped again meanwhile) */
        }
        __enable_irq();
        ss_retry(&ss, ms_now);             /* with the soft start (offset, ramp) */
    }
    if (ev & ISENSE_EV_REAPPLY) {
        outputs_dirty = 1;
    }
    if (ev & ISENSE_EV_CAL_DONE) {
        isense_cal_t c;
        uint8_t code;
        cal_save_pend = 1;                 /* the test with its curves: its own page, once the content is back */
        limit_update();                    /* the connected load is known now */
        (void)isense_cal_get(&is, &c);
        code = health_test_done(&hl, &c);
        if (code != 0u) {
            ack_start(&ack, ACK_P_FAULT, code);   /* D9: the first abnormal channel, or 6 = current while dark */
        }
    }
}

static void apply_settings(void)
{
    uint32_t s;
    eff = set;
    if (ultra_on) {
        ultra_overlay(&eff);
    }
    s = settings_steps(&eff);
    dopt.mode = (dither_mode_t)eff.dither_mode;
    dopt.threshold16 = dither_threshold16(eff.dither_thresh);
    dopt.floor_policy = (floor_policy_t)eff.dim_floor;
    dopt.floor = dither_floor_counts(eff.min_pulse);
    if (s != steps || eff.stagger != run_stagger) {
        steps = s;
        run_stagger = eff.stagger;
        len = pattern_len_for(steps);
        if (stagger_is_spread(eff.stagger) && len < STAGGER_SPREAD_LEN) {
            len = STAGGER_SPREAD_LEN;            /* spread spectrum: a longer period sequence */
        }
        pwm_start(steps, len, eff.stagger);
        st_o2 = STAGGER_FIXED_O2;            /* pwm_start puts TIM2 / TIM3 at the fixed offsets */
        st_o3 = STAGGER_FIXED_O3;
        st_force = 1;
    }
    limit_update();                          /* CONFIG 21 / 11-15 */
    built_ok = 0;                            /* new period / options: every table again */
    outputs_dirty = 1;
}

static void build(uint8_t ch, uint16_t v, uint8_t phase)
{
    uint8_t i, stride;
    uint16_t *dst = pwm_table(ch, &stride);
    const uint16_t *arr = pwm_spread_arr();
    if (arr != 0) {                        /* spread spectrum: every period its own length */
        dither_build_var(v, steps, arr, len, pattern_len_for(steps), &dopt, phase, dst, stride, pwm_right_aligned(ch));
        return;
    }
    dither_build(v, steps, len, &dopt, phase, dst, stride);
    if (pwm_right_aligned(ch)) {           /* PWM mode 2: active while CNT >= CCR */
        for (i = 0; i < len; i++) {
            dst[(uint32_t)i * stride] = (uint16_t)(steps - dst[(uint32_t)i * stride]);
        }
    }
}

/* the channels' tables for the values v (all, or only the ones whose value changed): phases spread the extra dither
 * steps / skipped pulses of the five channels over the pattern */
static void build_all(const uint16_t *v, uint8_t changed_only)
{
    static const uint8_t order[5] = { CH_G, CH_R, CH_B, CH_W2, CH_W1 };
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        uint8_t ch = order[i];
        if (!changed_only || out_now[ch] != v[ch]) {
            build(ch, v[ch], (uint8_t)(i * len / 5u));
        }
        out_now[ch] = v[ch];
    }
}

static void outputs_update(uint8_t ack_on)
{
    uint16_t v[5];
    uint8_t i;
    if (tj.active) {
        tj_outputs(&tj, ms_now, v);        /* test jig: the fade over the five channels; data doesn't change it */
    } else if (ack.active && !ack.led_only) {
        uint16_t lvl = ack_on ? 16384u : 0u;          /* 25 % */
        for (i = 0; i < 5u; i++) {
            v[i] = (eff.feedback_ch == 1u || (eff.feedback_ch == 0u && i == CH_R)) ? lvl : 0u;
        }
    } else {
        for (i = 0; i < 5u; i++) {
            v[i] = ultra_on ? uout[i] : sm.cur[i];
        }
        scene_mix(&srun, ms_now, v);       /* the power-on scene until data comes, then the cross-fade */
        dark_apply(&dk, ms_now, v);        /* the pin's calibration: dark (a test still overrides it below) */
    }
    is.k_ss = tj.active ? 65536u                                    /* soft start (not the test jig's pattern) */
                        : ss_factor(&ss, ms_now, (uint8_t)((v[0] | v[1] | v[2] | v[3] | v[4]) != 0u),
                                    (uint8_t)(sext.opts & STORE_OPT_SOFT_START));
    isense_apply(&is, v, v);               /* board limit (all channels scaled together), or the calibration pattern */
    build_all(v, built_ok);
    built_ok = 1;
}

/* pwm_start (hw.h): the tables for the new period / mode, from the values they held, before the timers start */
void pwm_fill(void)
{
    build_all(out_now, 0);
}

/* what the outputs show now, without an acknowledge pattern */
static void shown_outputs(uint16_t *v)
{
    scene_run_t r = srun;
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        v[i] = ultra_on ? uout[i] : sm.cur[i];
    }
    scene_mix(&r, ms_now, v);
}

/* config frame / console: power-on scene commands (and factory reset). Written after the normal debounce. */
static cfg_event_t scene_command(uint8_t cmd, uint8_t arg)
{
    scene_t s = scene_want;
    if (cmd == CMD_SCENE_FADE) {
        s.fade = arg;
    } else if (cmd == CMD_SCENE && arg == 1u) {
        shown_outputs(s.v);
        s.mask = 0x1Fu;
    } else {
        s.mask = 0;                        /* none: dark at power-up */
        if (cmd == CMD_FACTORY_RESET) {
            s.fade = SCENE_FADE_DEFAULT;
        }
    }
    if (scene_equal(&s, &scene_want)) {
        return CFG_SAME;
    }
    scene_want = s;
    scene_major_pend = 1;
    scene_changed_ms = ms_now - (SCENE_SAVE_MS - SAVE_DEBOUNCE_MS);
    return CFG_ACK;
}

/* soft start on / off (console, config frame, Ultra CONFIG 23): stored with the settings; 1 = changed */
static uint8_t soft_start_set(uint32_t on)
{
    uint8_t o = (uint8_t)(on ? sext.opts | STORE_OPT_SOFT_START : sext.opts & ~STORE_OPT_SOFT_START);
    if (o == sext.opts) {
        return 0;
    }
    sext.opts = o;
    settings_dirty = 1;
    changed_ms = ms_now;
    return 1;
}

/* follow-up of an applied command: blink feedback, new settings live, save debounced */
static cfg_result_t handle_result(cfg_result_t r)
{
    if (r.event == CFG_ACK && (r.cmd == CMD_ISENSE_LIMIT || r.cmd == CMD_ISENSE_CAL)) {
        if (r.cmd == CMD_ISENSE_CAL) {
            if (!isense_calibrate(r.arg)) {
                r.event = CFG_NACK;        /* tripped: not now */
                ack_start(&ack, ACK_P_NACK, 0);
            } else {
                stat.config_frames++;
            }
            return r;
        }
        if (sext.ilimit50 == r.arg || (r.arg == 0u && sext.ilimit50 == 0xFFu)) {
            r.event = CFG_SAME;
            ack_start(&ack, ACK_P_SAME, 0);
            return r;
        }
        sext.ilimit50 = r.arg;             /* stored with the settings */
        limit_update();
        ack_start(&ack, ACK_P_ACK, 0);
        settings_dirty = 1;
        changed_ms = ms_now;
        stat.config_frames++;
        return r;
    }
    if (r.event == CFG_ACK && r.cmd == CMD_SOFT_START) {
        r.event = soft_start_set(r.arg) ? CFG_ACK : CFG_SAME;
        ack_start(&ack, r.event == CFG_ACK ? ACK_P_ACK : ACK_P_SAME, r.event == CFG_ACK ? r.value_code : 0u);
        stat.config_frames += r.event == CFG_ACK;
        return r;
    }
    if (r.event == CFG_ACK && (r.cmd == CMD_SCENE || r.cmd == CMD_SCENE_FADE)) {
        r.event = scene_command(r.cmd, r.arg);         /* not a setting: no settings write */
        if (r.event == CFG_ACK) {
            ack_start(&ack, ACK_P_ACK, r.value_code);
            stat.config_frames++;
        } else {
            ack_start(&ack, ACK_P_SAME, 0);
        }
        return r;
    }
    switch (r.event) {
    case CFG_ACK:
        ack_start(&ack, r.value_code == 0xFFu ? ACK_P_FACTORY : ACK_P_ACK, r.value_code == 0xFFu ? 0u : r.value_code);
        if (r.cmd == CMD_PROTOCOL || r.cmd == CMD_FACTORY_RESET) {
            proto_set_mode(&frx.ps, (proto_mode_t)set.protocol);   /* a config frame / the console: at once */
        }
        if (r.cmd == CMD_FACTORY_RESET) {
            (void)scene_command(CMD_FACTORY_RESET, 0);
            store_ext_clear(&sext);        /* current limit and calibration too (the model relearns) */
            /* and the power-on effects and the stored calibration (never the lifetime record): in the flash's next
             * low moment (0.6.4: an erase; 0.6.3 at once, the outputs as they were). The acknowledge pattern's final
             * dark second is usually that moment. */
            fx_clear_pend = 1;
            fx_mask = 0;
            if (cal_present) {
                cal_save_pend = 2;         /* a cleared record (levels 0) */
                cal_present = 0;
            }
            isense_setup();
        }
        apply_settings();
        settings_dirty = 1;
        changed_ms = ms_now;
        stat.config_frames++;
        break;
    case CFG_SAME:
        ack_start(&ack, ACK_P_SAME, 0);
        break;
    case CFG_NACK:
        ack_start(&ack, ACK_P_NACK, 0);
        break;
    case CFG_IDENTIFY:
        if (r.arg == CFG_IDENTIFY_VERSION) {
            ack_start_version(&ack, fw_version);   /* config frame / console `version` */
        } else {
            ack_start(&ack, ACK_P_IDENTIFY, 0);
        }
        break;
    default:
        break;
    }
    return r;
}

static cfg_result_t cli_apply(uint8_t cmd, uint8_t arg)
{
    return handle_result(config_apply(&rx, &set, cmd, arg));
}

/* A flash operation (flash_may's answer `may`): the frame tracking stops meanwhile (the tick runs from SRAM: no frames,
 * the gate closed; downstream boards miss a frame or two, the next idle gap or frame end re-arms it), so a capture in
 * progress is dropped. The time bases run on (0.6.4; 0.6.3 added an estimate). */
static void flash_stall_begin(uint32_t may)
{
    if (may == FM_BRIGHT) {
        (void)budget_take(&flash_bright, ms_now, 1u);    /* the hour's write while brighter */
    }
    __disable_irq();
    ftrack_disarm(&ft);
    cap_ready = 0;
    __enable_irq();
}

/* ... done with result r (core/flashrec.h): 1 = written */
static uint32_t flash_stall_end(uint32_t r)
{
    __disable_irq();
    ftrack_disarm(&ft);
    cap_ready = 0;
    __enable_irq();
    ultra_clock_gap(&ul);
    if (r == FLASH_OK) {
        stat.saves++;
    } else if (r != FLASH_NONE) {
        uart_write("flash: error\r\n");                 /* (the record stays pending; its budget asks again) */
    }
    return r == FLASH_OK;
}

/* the settings record (may: flash_may's answer for it; the store erases the next page first when that is due):
 * 1 = written */
static uint32_t settings_write(uint32_t may)
{
    flash_stall_begin(may);
    if (flash_stall_end(settings_store(&set, &sext))) {
        settings_dirty = 0;
        return 1;
    }
    return 0;
}

/* the console's `save`: at once (a program: a user's command), unless the next page needs its erase while the outputs
 * are bright: then it is written in the next low moment */
static void save_now(void)
{
    uint32_t t0 = ms_now, may;
    settings_dirty = 1;
    changed_ms = ms_now - SAVE_DEBOUNCE_MS;
    while (uart_busy() && ms_now - t0 < 100u) {          /* (the console's line first: hw.c flash_op) */
        uart_kick();
        watchdog_kick();
    }
    may = flash_may(settings_pending() == 2u, FM_PROG);
    uart_write(may == 0u ? "pending\r\n" : settings_write(may) ? "ok\r\n" : "failed\r\n");
}

static void scene_save_now(uint32_t may)
{
    flash_stall_begin(may);
    if (flash_stall_end(scene_store(&scene_want))) {
        scene_stored = scene_want;
    }
}

/* the power-on effects to store (of the lights a SCENE_EFFECT reached since the engine's init): their mask, their
 * CRC-8s into crc; *any = a host sent SCENE_EFFECT at all */
static uint8_t fx_want(uint8_t *crc, uint8_t *any)
{
    uint8_t mask = 0;
    uint32_t i;
    *any = 0;
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        const ultra_light_t *L = &ul.light[i];
        if (L->scene_fx_have) {
            *any = 1;
        }
        if (L->scene_fx_have && ultra_scene_fx_ok(L)) {
            crc[i] = crc8_smbus((const uint8_t *)L, ULTRA_HEAD_BYTES);
            mask |= (uint8_t)(1u << i);
        } else {
            crc[i] = 0;
        }
    }
    return mask;
}

/* what the engine holds is what the store holds (fx_mask, fx_crc)? Compared light by light: a sum of the CRCs
 * (0.6.2) let two lights' changes cancel each other out (CRC deltas d0 = -16 d1 ...), so that the new effects
 * were never written */
static int fx_stored(uint8_t mask, const uint8_t *crc)
{
    uint32_t i;
    if (mask != fx_mask) {
        return 0;
    }
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        if ((mask & (1u << i)) && crc[i] != fx_crc[i]) {
            return 0;
        }
    }
    return 1;
}

/* store the power-on effects (budget permitting): only once a host sent SCENE_EFFECT (a host without it never
 * changes them), and only when they differ from what is stored. fx_pend 1: compare (after a quiet change), 2: differs,
 * waiting for a token (only the budget is checked meanwhile) */
static void fx_service(void)
{
    uint8_t crc[ULTRA_LIGHTS], mask, any;
    uint32_t i;
    if (fx_pend == 0u ||
        (fx_pend == 2u && (flash_may(1u, 0u) != FM_LOW || !budget_take(&fx_budget, ms_now, 1u)))) {
        return;                            /* (two erases: only in a low moment, 0.6.4) */
    }
    mask = fx_want(crc, &any);
    if (!any || fx_stored(mask, crc)) {
        fx_pend = 0;                       /* nothing to write */
        return;
    }
    if (fx_pend == 1u) {
        fx_pend = 2;                       /* differs: written with the next token (a token per 6 h) */
        return;
    }
    fx_pend = 0;
    flash_stall_begin(FM_LOW);
    if (flash_stall_end(scene_fx_store(&ul, mask, crc))) {
        fx_mask = mask;
        for (i = 0; i < ULTRA_LIGHTS; i++) {
            fx_crc[i] = crc[i];
        }
        fx_clear_pend = 0;                 /* (it erased the store anyway) */
    } else {
        fx_mask = 0;                       /* failed: nothing valid stored; again with the next token */
        fx_pend = 2;
    }
}

/* every ms while the power-on effect runs: its channels follow the engine until the take-over (then the scene's
 * cross-fade starts from where the effect was); when none is left, the engine's power-on effect stops */
static void fx_follow(void)
{
    uint8_t i;
    fx_ch &= srun.hold;
    for (i = 0; i < 5u; i++) {
        if (fx_ch & (1u << i)) {
            srun.sc.v[i] = uout[i];
        }
    }
    if (fx_ch == 0u) {
        ultra_scene_fx_stop(&ul);
    }
}

/* every ms: Ultra take-over deadline, recompute the scene from the lights' SCENE values once things are quiet,
 * write it when due (SCENE_SAVE_MS after the last change, within the write budget: core/scene.h), not while a
 * pattern blinks (except its final dark pause) */
static void scene_service(void)
{
    uint32_t may;
    if (ultra_on && srun.hold && ultra_seen && ms_now - ultra_first_ms >= SCENE_TAKEOVER_MS) {
        scene_takeover(&srun, 0x1Fu, ms_now);     /* the host is there but sends nothing for these channels */
        outputs_dirty = 1;
    }
    if (scene_check && ultra_on && ms_now - scene_check_ms >= SCENE_SAVE_MS) {
        scene_t sc;
        scene_check = 0;
        fx_pend = 1;                       /* and the power-on effects (fx_service) */
        ultra_scene(&ul, &scene_stored, &sc);
        scene_want = sc;
        scene_major_pend = (uint8_t)scene_major(&scene_stored, &sc);
        scene_changed_ms = scene_check_ms;
    }
    if (!scene_equal(&scene_want, &scene_stored) && (!ack.active || ack.seg + 1u == ack.nseg) &&
        (may = flash_may(scene_pending() == 2u, 0u)) != 0u && scene_save_due(&sbudget, ms_now, scene_changed_ms, scene_major_pend)) {
        scene_save_now(may);
    }
}

static void handle_slot(const slot_t *slot, proto_t p)
{
    cfg_result_t r = config_process(&rx, &set, slot);
    uint16_t tgt[5];
    uint8_t i, ramp;

    last_data_ms = ms_now;
    signal_lost = 0;
    if (r.event != CFG_NONE) {
        handle_result(r);                  /* config frame: outputs hold */
        return;
    }
    if (p == PROTO_WS2805) {
        for (i = 0; i < 5u; i++) {
            tgt[i] = gamma_apply(set.gamma, slot->raw[i]);
        }
        ramp = (uint8_t)(set.smoothing != 0u);
    } else {
        for (i = 0; i < 5u; i++) {
            tgt[i] = slot->ch[i];          /* WS2915 gain header ignored (wiki: WS modes) */
        }
        ramp = (uint8_t)(set.smoothing == 2u);
    }
    smooth_frame(&sm, tgt, tick_now, ramp);
    scene_takeover(&srun, 0x1Fu, ms_now);  /* valid data: the power-on scene fades over to it */
    if (ack.repeat) {
        ack_stop(&ack);                    /* identify ends with the next normal frame */
    }
    outputs_dirty = 1;
}

/* Ultra: the header at the end of a frame (host time at the frame end, local tick of its last edge) and this
 * board's slot from the start of the same frame (NULL: none, e.g. a board past the last slot) */
static void handle_ultra(uint32_t host_us, uint32_t hdr_tick, const uint8_t *slot)
{
    uint32_t ev;
    ultra_on_header(&ul, hdr_tick, host_us);
    last_data_ms = ms_now;
    signal_lost = 0;
    ul_used = 1;                           /* a host drives the engine now */
    if (!ultra_seen) {
        ultra_seen = 1;                    /* the power-on scene holds at most SCENE_TAKEOVER_MS from here */
        ultra_first_ms = ms_now;
    }
    if (slot == 0) {
        return;
    }
    ev = ultra_apply_slot(&ul, slot, hdr_tick);       /* frx_capture checked its CRC */
    if (ev & ULTRA_EV_STATE) {
        scene_takeover(&srun, 0x1Fu, ms_now);   /* the host sends states: the power-on scene fades over */
        outputs_dirty = 1;
    }
    if (ev & (ULTRA_EV_SCENE | ULTRA_EV_DEF | ULTRA_EV_CONFIG)) {
        scene_check = 1;                   /* the scene's outputs may change: recompute when quiet */
        scene_check_ms = ms_now;
    }
    if (ev & ULTRA_EV_CONFIG) {
        if (ul.cfg_changed & (1u << ULTRA_CFG_BOOT_PROTOCOL)) {
            uint16_t v = ul.cfg[ULTRA_CFG_BOOT_PROTOCOL];
            if (v <= PMODE_ULTRA && v != set.protocol) {
                set.protocol = (uint8_t)v;     /* persisted: saved after the debounce, only when it differs */
                settings_dirty = 1;
                changed_ms = ms_now;
                if (v == PMODE_AUTO || v == PMODE_ULTRA) {
                    proto_set_mode(&frx.ps, (proto_mode_t)v);   /* Ultra keeps running; WS modes from the next power-up */
                }
            }
        }
        if ((ul.cfg_changed & (1u << ULTRA_CFG_SOFT_START)) && ul.cfg[ULTRA_CFG_SOFT_START] <= 1u) {
            (void)soft_start_set(ul.cfg[ULTRA_CFG_SOFT_START]);   /* persisted, like 22 */
        }
        ul.cfg_changed = 0;
        apply_settings();                  /* host-managed: live at once, no blink, not saved (except 22, 23) */
        stat.config_frames++;
    }
    if (ev & ULTRA_EV_IDENTIFY) {
        if (ul.identify == ULTRA_IDENT_CAL) {
            (void)isense_calibrate(ul.identify_arg);   /* refused while tripped or in test mode */
        } else if (ul.identify == ULTRA_IDENT_DARK) {
            dark_hold(&dk, ms_now, (ul.identify_arg ? ul.identify_arg : 60u) * 1000u);
        } else if (ul.identify == ULTRA_IDENT_RELEASE) {
            dark_release(&dk, ms_now);
        } else if (ul.identify == ULTRA_IDENT_VERSION) {
            ack_start_version(&ack, fw_version);
        } else if (ul.identify == ULTRA_IDENT_ON) {      /* (ultra_apply_slot drops unknown values) */
            ack_start(&ack, ACK_P_IDENTIFY, 0);
        } else {
            ack_stop(&ack);
        }
        outputs_dirty = 1;
    }
}

static void protocol_changed(proto_t from, proto_t to)
{
    if (to == PROTO_ULTRA) {
        if (ul_used) {
            ultra_init(&ul, tick_now ^ 0xA5C3u);   /* (not before a host used it: it may run the power-on effect) */
        }
        ultra_on = 1;
    } else if (from == PROTO_ULTRA) {
        ultra_on = 0;                      /* the host's CONFIG ends with Ultra: stored settings again */
    } else {
        return;
    }
    apply_settings();
}

/* A stored (provisional) protocol turned out wrong: its gate plan may cut the frame at the wrong bits, so
 * forward nothing more until the next re-arm (the new plan). The gate ISR must not re-open it meanwhile. */
static void gate_close_now(void)
{
    __disable_irq();
    gate_force_closed();
    __enable_irq();
}

/* ------------------------------------------------------------------------------- firmware update (app side) */
static void upd_action(uint32_t a)
{
    switch (a) {
    case UPD_APP_TAKE:
#if UPD_APP
    {
        uint32_t t0 = ms_now;
        HO->request = UPD_REQ_ENTER;       /* the bootloader checks the ANNOUNCE again and installs it */
        upd_ho_seal(HO);
        uart_write("firmware update: into the bootloader\r\n");
        while (ms_now - t0 < 10u) {        /* the host's lead-in is >= 500 ms: time to send the line */
            uart_kick();
            watchdog_kick();
        }
        gate_close_now();
        NVIC_SystemReset();
    }
#endif
        break;
    case UPD_APP_CONFIRM:
#if UPD_APP
    {
        /* one half-word, as any regular write (0.6.4: the program's status decides, not a read back, which may come
         * from the copy made at reset; not admitted now: at the next PING) */
        uint32_t a = UPD_APP_DESC + 2u * (UPD_ST_FIRST + upd.info.st_next), ok = 0, may = flash_may(0u, 0u);
        if (may != 0u) {
            flash_stall_begin(may);
            ok = flash_stall_end(hw_flash_half(a, UPD_ST_CONFIRMED));
        }
        upd_app_confirmed(&upd, (int)ok);
    }
#endif
        break;
    case UPD_APP_SHOW_SAME:
        ack_start(&ack, ACK_P_SAME, 0);    /* the session's image is already installed */
        break;
    case UPD_APP_SHOW_REJECTED:
        ack_start(&ack, ACK_P_NACK, 0);    /* not for this board, or older without FORCE */
        break;
    default:
        break;
    }
}

/* the decoded bits of a capture: this board's slot (process_capture) or a whole update frame (upd_capture), never
 * both at once (each is used up within its call). 16-byte aligned (0.6.3): ws_decode_frame clears it with memset,
 * whose 16-byte path otherwise came and went with the RAM layout (+27 instructions per slot) */
static union {
    uint8_t slot[PROTO_BITS_BUF];
    uint8_t upd[UPD_FRAME_BYTES + 4];
} dec __attribute__((aligned(16)));

static void upd_capture(void)
{
    uint8_t *f = dec.upd;
    if (cap_bits < UPD_FRAME_BITS ||
        ws_decode_frame(din_copy, (uint32_t)cap_words * 16u, (uint16_t)UPD_FRAME_BITS, &ws_tm, f) != (int)UPD_FRAME_BITS) {
        return;                            /* the session repeats every block (passes) */
    }
    upd_action(upd_app_frame(&upd, f, ms_now));
}

/* boot: what the bootloader left in the hand-over block (UPD_APP), the app's descriptor and status records */
static void upd_boot(void)
{
    upd_app_info_t info;
#if UPD_APP
    upd_app_read_info(&info, (const uint8_t *)UPD_APP_DESC, (const uint8_t *)UPD_BL_INFO);
    if (upd_ho_valid(HO)) {
        stat.reset_flags = HO->reset_flags;   /* the bootloader cleared the RCC flags (its own resets pile up) */
        HO->reset_flags = 0;
        switch (HO->result) {
        case UPD_RES_UPDATED: {
            uint32_t minor = (HO->result_version >> 16) & 0xFFu;
            ack_start(&ack, ACK_P_ACK, (uint8_t)((minor > 29u ? 29u : minor) + 1u));   /* ACK + the minor version */
            break;
        }
        case UPD_RES_SAME:
            ack_start(&ack, ACK_P_SAME, 0);
            break;
        case UPD_RES_REJECTED:
            ack_start(&ack, ACK_P_NACK, 0);
            break;
        default:
            break;
        }
        HO->result = UPD_RES_NONE;
        HO->request = UPD_REQ_NONE;
        upd_ho_seal(HO);
    }
#else
    info.hw_id = info.hw_rev = info.bl_version = 0;   /* standalone: forward update sessions only */
    info.desc_ok = info.confirmed = 0;
    info.st_next = (uint8_t)UPD_ST_COUNT;
    info.version = 0;
#endif
    upd_app_init(&upd, &info);
}

/* every ms: transparency timeout / pending results / a delayed confirm; the watchdog-loop count */
static void upd_service(void)
{
    uint8_t was_transparent = (uint8_t)upd_app_transparent(&upd);
    upd_action(upd_app_poll(&upd, ms_now));
    if (was_transparent && !upd_app_transparent(&upd)) {
        gate_plan_t g = proto_gate_plan(proto_active(&frx.ps));
        uint8_t i = (uint8_t)(plan_idx ^ 1u);
        plan_buf[i] = g;
        BARRIER();
        plan_idx = i;
        /* With no more frames there is no frame-end ISR to install the normal plan.
         * Re-arm only on an armed idle input; an active/partial frame finishes normally. */
        __disable_irq();
        if (ft.armed && ft.cap_state == 0u && DIN_EDGES() == 0u) {
            gate_rearm(&g);
            plan = g;
        }
        __enable_irq();
    }
#if UPD_APP
    if (!upd_wd_cleared && up_s >= UPD_WD_CLEAR_S && upd_ho_valid(HO)) {
        HO->wd_resets = 0;                 /* running fine: the bootloader starts counting again */
        upd_ho_seal(HO);
        upd_wd_cleared = 1;
    }
#endif
}

static void process_capture(void)
{
    uint8_t *bits = dec.slot;
    uint32_t skip = cap_skip, want = cap_bits - skip;
    proto_t p = (proto_t)cap_proto;
    slot_t slot;
    int n;

    BARRIER();                             /* din_copy: read after cap_ready (the ISR's copy is complete) */
    if (cap_own == PROTO_UPD_BITS && skip == 0u) {
        upd_capture();                     /* a whole update frame (the transparent plan): not a slot */
        BARRIER();
        cap_ready = 0;
        return;
    }
    if (want - 1u >= PROTO_MAX_BITS) {       /* 0 or too many (also cap_bits < cap_skip); 0.6.4: 0 = a WS2915 frame
                                              that ended at its header (fable-robust: no 0-bit decode, not counted) */
        BARRIER();
        cap_ready = 0;
        return;
    }
    n = ws_decode_frame(din_copy, (uint32_t)cap_words * 16u, (uint16_t)want, &ws_tm, bits);
    if (n != (int)want) {
        BARRIER();
        cap_ready = 0;
        if (n < 0 && -n < 6) {
            stat.dec_err[-n]++;
        }
        return;
    }
    stat.decoded++;
    n = (int)frx_capture(&frx, cap_frame, p, bits, want, cap_own, skip, &slot);
    BARRIER();                             /* release only after every sample and metadata read */
    cap_ready = 0;
    if ((uint32_t)n & FRX_SHOW_WS) {
        handle_slot(&slot, p);
    }
}

static void process_frame_end(void)
{
    uint8_t tail[8], i;
    const uint8_t *t8 = 0;
    frx_end_t e;
    uint32_t end_tick = fe_tick, total = fe_bits;
    BARRIER();                             /* tail_copy: read after fe_ready */
    if (tail_words != 0u && ws_decode_last(tail_copy, (uint32_t)tail_words * 16u, &ws_tm, tail) >= (int)ULTRA_HDR_BITS) {
        t8 = tail;                         /* the frame's last 64 bits: an Ultra header? */
    }
    frx_frame_end(&frx, fe_frame, total, t8, &e);
    BARRIER();
    fe_ready = 0;
    upd_app_frame_end(&upd, e.ultra_header, e.flags, ms_now);
    if (e.ultra_header && (e.flags & ULTRA_HDR_UPD)) {
        last_data_ms = ms_now;             /* an update session is signal too (lights hold / effects run) */
        signal_lost = 0;
    }
    if (e.after == PROTO_ULTRA && proto_tail_wanted(total)) {
        if (e.ultra_header) {
            stat.ultra_headers++;
        } else {
            stat.ultra_tail_err++;         /* bench: the tail window (TAIL_WORDS) or the hub's timing */
        }
    }
    if (e.show_ws) {
        handle_slot(&frx.ws, (proto_t)frx.ws_proto);   /* the stored protocol is confirmed by this frame */
    }
    if (e.close_gate) {
        gate_close_now();                  /* the stored protocol was wrong: forward nothing more of it */
    }
    if (e.after != e.before) {
        protocol_changed(e.before, e.after);
    }
    if (e.ultra_sync) {
        handle_ultra(e.host_us, end_tick, e.ultra_slot ? frx.slot : 0);   /* also on the frame that locked Ultra */
    }
    if (frx.ps.mode == PMODE_AUTO && frx.ps.locked != PROTO_NONE && !frx.ps.provisional &&
        frx.ps.locked != set.last_proto) {
        set.last_proto = (uint8_t)frx.ps.locked;   /* sticky auto: written only when the locked protocol changes */
        settings_dirty = 1;
        changed_ms = ms_now;
    }
    i = (uint8_t)(plan_idx ^ 1u);          /* the ISR reads plan_buf[plan_idx] at the next frame end */
    plan_buf[i] = upd_app_transparent(&upd) ? proto_gate_plan_transparent() : e.plan;
    BARRIER();                             /* the plan and frx.ps are written before the ISR can take them */
    plan_idx = i;
}

/* Adaptive stagger (STAGGER_ADAPTIVE, _ADAPTIVE_SPREAD): at most every 50 ms, when an output moved by more than ~0.4 %, pick the TIM2 /
 * TIM3 offsets with the least LED_+ ripple current for the current outputs (stagger.c, ~0.3 ms) and slew there.
 * Weights: the LED current per channel at 100 % measured by the current sense (once calibrated / learned), else
 * from Ultra's CONFIG, else 3 A (R/G/B) and 5 A (W1/W2). */
static void stagger_adapt(void)
{
    uint16_t peak[5];
    stagger_result_t r;
    uint8_t i, changed = st_force;
    if (!stagger_is_adaptive(eff.stagger) || ms_now - st_ms < 50u) {
        return;
    }
    for (i = 0; i < 5u; i++) {
        int32_t dv = (int32_t)out_now[i] - (int32_t)st_seen[i];
        if (dv > 256 || dv < -256) {
            changed = 1;
        }
    }
    if (!changed) {
        return;
    }
    st_ms = ms_now;
    st_force = 0;
    for (i = 0; i < 5u; i++) {
        st_seen[i] = out_now[i];
        peak[i] = cfg_load_ma(i);
        if (isense_model_ok(&is)) {        /* measured (calibrated / learned): the real strips, in every mode; */
            uint32_t on = isense_on_ma(&is, i, out_now[i]);   /* the current while on, from the channel's curve */
            peak[i] = (uint16_t)(on > 65535u ? 65535u : on);
        }
    }
    stagger_choose_max(out_now, peak, stagger_gap_q16(steps), st_o2, st_o3,
                       (uint16_t)(stagger_is_spread(eff.stagger) ? STAGGER_SPREAD_OMAX : 0u), &r);
    stat.ripple_ma = r.move ? r.ripple_best : r.ripple_now;
    if (r.move) {
        st_o2 = r.o2;
        st_o3 = r.o3;
        pwm_set_phase(st_o2, st_o3);
        stat.stagger_moves++;
    }
    stat.stagger_o2 = st_o2;
    stat.stagger_o3 = st_o3;
}

static void signal_loss_check(void)
{
    static const uint16_t off[5] = { 0, 0, 0, 0, 0 };
    uint32_t timeout = LOSS_TIMEOUT_MS;
    if (ultra_on) {
        timeout = (uint32_t)ul.cfg[ULTRA_CFG_LOSS_TIMEOUT] * 100u;
        if (timeout < 500u) {
            timeout = 500u;
        }
    }
    if (signal_lost || eff.signal_loss == 0u || ms_now - last_data_ms < timeout) {
        return;                            /* mode 0 = hold (Ultra: effects keep running), like a WS2805 */
    }
    signal_lost = 1;
    if (ultra_on) {
        ultra_all_off(&ul, tick_now, eff.signal_loss == 2u ? LOSS_FADE_MS : 0u);
    } else {
        smooth_fade(&sm, off, tick_now, eff.signal_loss == 2u ? LOSS_FADE_MS * SMOOTH_TICKS_PER_MS : 0u);
    }
    outputs_dirty = 1;
}

static uint16_t sat16(int32_t v)
{
    return (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v);
}

/* every ms: the power events (core/pwrev.h) and D9. The state comes from the current sense's limiter and trips, and
 * the Ultra channel limits (CONFIG 16-20). D9 by priority: trip > acknowledge / identify / version / the health code >
 * the TP9 reset gesture / test mode > limit > off (core/ocled.h; the feedback channel shows the acknowledge patterns,
 * outputs_update) */
static void power_service(uint8_t ack_on)
{
    pwrev_in_t in;
    uint8_t lv;
    in.limit_ma = clim.limit_ma;
    in.limit_src = pwrev_src(clim.src);
    in.ch_limiting = (uint8_t)(ultra_on && ul.ch_limited);
    in.trip_cause = is.tripped;            /* ISENSE_TRIP_FAST / _OVL = ULTRA_PW_CAUSE_SHORT / _OVERLOAD */
    in.limiting = (uint8_t)(is.limiting && in.trip_cause == ISENSE_TRIP_NONE);
    in.i_ma = sat16(is.i_ms);
    in.trip_ma = in.trip_cause == ISENSE_TRIP_FAST ? is.peak_ma : in.i_ma;
    in.factor = (uint16_t)(is.k >= 65535u ? 65535u : is.k);
    in.retry_ms = isense_retry_left_ms(&is);
    in.dips = vdip.count;
    pwrev_update(&pw, ms_now, &in);
    ocled_update(&ocl, ms_now, up_s, in.limiting, in.trip_cause);
    lv = ocled_led(&ocl, ms_now, ack.active, ack_on);
    if ((tj.active || tj.rst >= TJ_RST_ARMED) && ocl.show == OCLED_OFF && !ack.active) {
        lv = tj_led(&tj, ms_now);          /* test jig: on, off at each cycle start; the reset gesture armed: fast
                                              (overcurrent / acknowledge win) */
    }
    if (lv != led_now) {
        led_now = lv;
        led_set(lv);
    }
}

/* every ms: the test jig's pad (debounced), the TP9 reset gesture, and in test mode the pattern (every ms) and its
 * current report */
static void testjig_service(void)
{
    uint8_t ev = tj_pad(&tj, ms_now, TESTJIG_PAD_LOW());
    if (ev & TJ_EV_MODE) {
        uart_write(tj.active ? "test mode: on (TP9 grounded)\r\n" : "test mode: off\r\n");
        outputs_dirty = 1;
    }
    if (ev & TJ_EV_RESET) {
        uart_write("factory reset (TP9)\r\n");
        (void)cli_apply(CMD_FACTORY_RESET, 0xA5u);     /* as the console's `defaults` */
    }
    if (!tj.active) {
        return;
    }
    outputs_dirty = 1;
    if (tj_measure(&tj, ms_now, is.i_ms, is.zero_ma)) {
        cli_test_report(&cli, tj.ma, TJ_PEAK_PCT, tj.cycles, vs_sup.valid ? vs_sup.mv : 0u);
    }
}

static inline __attribute__((always_inline)) void status_snapshot(void)
{
    uint8_t i;
    stat.uptime_s = up_s;
    stat.stack_max = stack_used();
    stat.proto_mode = (uint8_t)frx.ps.mode;
    stat.proto_locked = (uint8_t)proto_active(&frx.ps);
    stat.proto_provisional = frx.ps.provisional;
    stat.frames = frames;
    stat.steps = steps;
    for (i = 0; i < 5u; i++) {
        stat.out[i] = ultra_on ? uout[i] : sm.cur[i];
    }
    stat.ilimit50 = sext.ilimit50 == 0xFFu ? 0u : sext.ilimit50;
    stat.softstart = (uint8_t)(sext.opts & STORE_OPT_SOFT_START);
    {
        stat.i_ma = is.i_avg;
        stat.i_peak_ma = is.peak_ma;
        stat.i_zero_ma = is.zero_valid ? is.zero_ma : 0;
        stat.i_limit_ma = is.limit_ma;
        stat.i_k_permille = (is.k * 1000u) >> 16;
        stat.i_trips = is.trips;
        stat.i_tripped = (uint8_t)(is.tripped != ISENSE_TRIP_NONE);
        stat.i_trip_cause = is.trip_cause != ISENSE_TRIP_NONE ? is.trip_cause : stat.i_trip_cause;
        stat.i_cal = (uint8_t)(is.cal_step != 0u);
        stat.i_model = (uint8_t)(is.calibrated ? 2u : (isense_model_ok(&is) ? 1u : 0u));
        for (i = 0; i < 5u; i++) {
            stat.i_ch_ma[i] = is.ch_ma[i];
        }
        (void)isense_cal_get(&is, &stat.cal);   /* (levels 0: none) */
        stat.cal_count = cal_cnt;
    }
    stat.lim_ma = clim.limit_ma;
    stat.lim_src = clim.src;
    stat.lim_need = clim.need;
    stat.lim_conn_ma = clim.connected_ma;
    stat.mcu_temp_c10 = mt.c10;
    stat.vdda_mv = mt.vdda_mv;
    stat.mcu_peak_c10 = temp_peak;
    stat.mcu_peak_s = temp_peak_s;
    for (i = 0; i < 5u; i++) {
        stat.i_now_ch_ma[i] = (int32_t)isense_ch_ma(&is, i, is.applied[i]);
    }
    stat.test_mode = tj.active;
    stat.vs_supply_mv = vs_sup.valid ? vs_sup.mv : 0u;
    /* Extrema are consumed only by status_begin, never by watch or keystrokes. */
    stat.vs_aux_mv = vs_aux.valid ? vs_aux.mv : 0u;
    stat.dip_ref_mv = vsense_mv(&vs_sup, vdip.ref_q12 >> 8, 16u);   /* Q12 as 16 pairs */
    stat.oc_show = ocl.show;
    stat.oc_cause = ocl.last_cause;
    stat.oc_limiting = ocl.limiting;
    stat.oc_tripped = (uint8_t)(ocl.trip_cause != 0u);
    stat.oc_since_s = ocl.since_s;
    stat.oc_hold_ms = ocled_hold_left(&ocl, ms_now);

}

static void status_begin(void)
{
    vsense_report(&vs_sup, &stat.vs_supply_min_mv, &stat.vs_supply_max_mv);
}

int main(void)
{
    uint32_t last_ms = 0, fps_ms = ms_now, fps_frames = 0, watch_t = 0, may;
    uint8_t ack_on = 0, was_active = 0;
    int ch;

    stack_paint();                         /* the high-water mark in `status` */
    if (settings_load(&set, &sext) != 0) {
        settings_default(&set);            /* (the extension is cleared too) */
    } else {
        (void)settings_sanitize(&set);     /* a record of another build: out-of-range fields take their defaults */
    }
    if (scene_load(&scene_stored) != 0) {
        scene_clear(&scene_stored);        /* none stored: dark until data comes (as before) */
    }
    scene_want = scene_stored;
    scene_boot(&srun, &scene_stored);
    ultra_init(&ul, hw_uid_hash());        /* (a seed per board: the power-on effects' randomness) */
    fx_mask = ultra_fx_load(&ul, (const uint8_t *)SCENE_FX_ADDR, fx_crc);   /* the power-on effects: from here on */
    ultra_scene_fx_start(&ul, fx_mask, tick_now);
    for (ch = 0; ch < (int)ULTRA_LIGHTS; ch++) {
        if (fx_mask & (1u << ch)) {
            fx_ch |= ul.light[ch].mask;
        }
    }
    srun.sc.mask |= fx_ch;
    srun.hold |= fx_ch;
    budget_init(&fx_budget, ms_now, FX_BUDGET_CAP, FX_REFILL_MS);   /* (the time bases: any start value, any wrap) */
    budget_init(&set_budget, ms_now, STORE_BUDGET_CAP, STORE_REFILL_MS);
    budget_init(&cal_budget, ms_now, CAL_BUDGET_CAP, CAL_REFILL_MS);
    dark_init(&dk);
    ocled_init(&ocl);
    tj_init(&tj, TJ_CYCLE_MS);
    pwrev_init(&pw, ms_now);
    scene_budget_init(&sbudget, ms_now);
    config_rx_init(&rx);
    frx_init(&frx, (proto_mode_t)set.protocol, (proto_t)set.last_proto);
    smooth_init(&sm);
    hw_init();
    mcutemp_init(&mt);
    health_init(&hl);
    ss_init(&ss, hw_uid_hash());
    vsense_init(&vs_sup, VSENSE_SUPPLY_RT, VSENSE_SUPPLY_RB, VSENSE_VREF_MV);
    vsense_init(&vs_aux, VSENSE_AUX_RT, VSENSE_AUX_RB, VSENSE_VREF_MV);
    hw_adc_init();                         /* PA7 continuously, the injected group from the poll tick */
    isense_setup();
    {
        isense_cal_t c;
        if (cal_load(&c, &cal_cnt) == 0) {
            isense_cal_set(&is, &c);       /* the last test with its curves wins */
            cal_present = 1;
        }
    }
    stat.reset_flags = hw_reset_flags();
    upd_boot();                            /* UPD_APP: the reset flags from the hand-over, the bootloader's result */
#if UPD_APP
    fw_version = upd.info.desc_ok ? upd.info.version : 0u;   /* the descriptor's: what the boards compare in an update */
#else
    fw_version = fwver_parse(FW_VERSION);
#endif
    fwver_format(fw_version, fw_version_text);
    energy_init(&en, life_load(&en.lt), ms_now, fw_version, stat.reset_flags & RCC_IWDGRSTF);
    stat.version = fw_version != 0u ? fw_version_text : FW_VERSION " (development build)";
    stat.bl_version = upd.info.bl_version;
    stat.hw_id = upd.info.hw_id;
    stat.hw_rev = upd.info.hw_rev;
    stat.app_state = upd.info.bl_version == 0u ? 0u : upd.info.confirmed ? 2u : 1u;
#if UPD_APP
    if (upd.info.bl_version != 0u) {
        const upd_bl_info_t *bl = (const upd_bl_info_t *)UPD_BL_INFO;
        uint32_t i;
        for (i = 0; i < 8u && bl->build_id[i] >= ' ' && bl->build_id[i] <= '~'; i++) {
            stat.bl_build[i] = (char)bl->build_id[i];
        }
        stat.bl_build[i] = 0;
    }
#endif
    cli.write = uart_write;
    cli.apply = cli_apply;
    cli.save_now = save_now;
    cli.status_begin = status_begin;
    cli.set = &set;
    cli.scene = &scene_want;
    cli.st = &stat;
    stat.dip = &vdip;
    stat.pw = &pw;
    stat.hl = &hl;
    stat.dip_vs = &vs_sup;
    stat.en = &en;
    ultra_on = (uint8_t)(proto_active(&frx.ps) == PROTO_ULTRA);
    apply_settings();
    {
        gate_plan_t g = proto_gate_plan(proto_active(&frx.ps));
        plan_buf[0] = plan_buf[1] = g;
        plan = g;
        BARRIER();                         /* (as at the frame end) */
        rx_ready = 1;                      /* gate closed until the tracker arms (idle line) or a frame ends */
    }
    for (ch = 0; ch < 4; ch++) {
        flash_erase_ahead();               /* the stores' next pages: erased now, the outputs still dark (0.6.3: the
                                              lifetime record's) */
    }
    flash_boot = 0;
    budget_init(&flash_bright, ms_now, 1u, FLASH_BRIGHT_MS);
    cli_banner(&cli);
    uart_write("type help\r\n");

    for (;;) {
        watchdog_kick();
        if (cap_ready) {
            process_capture();
        }
        if (fe_ready) {
            if (cap_ready) {
                process_capture();         /* a short frame: capture and frame end came together */
            }
            process_frame_end();
        }
        if (ultra_on != (uint8_t)(proto_active(&frx.ps) == PROTO_ULTRA)) {
            protocol_changed(ultra_on ? PROTO_ULTRA : PROTO_NONE, proto_active(&frx.ps));   /* console mode change */
        }
        ch = uart_getc();
        if (ch == -2) {
            cli.len = 0;                   /* a flash operation dropped received bytes: the line being typed goes */
            ch = uart_getc();
        }
        if (ch >= 0 || (cli.watch_ms && ms_now - watch_t >= cli.watch_ms)) {
            status_snapshot();             /* live values; extrema are consumed only by status_begin */
            if (ch < 0) {
                watch_t = ms_now;
                cli_watch(&cli);
            }
            for (; ch >= 0; ch = uart_getc()) {
                cli_feed(&cli, (char)ch);
                if (ch == '\r' || ch == '\n') {
                    break;                 /* one line per pass (0.6.4, gpt-opt2: seven queued `status` lines blocked
                                              one pass for 0.8 s, past the watchdog; the rest waits in the RX ring) */
                }
            }
        }
        uart_kick();
        if (ms_now != last_ms) {
            last_ms = ms_now;
            if (ack_tick(&ack, &ack_on) || was_active) {
                outputs_dirty = 1;
            }
            was_active = ack.active;
            isense_service();              /* measurement of the last ms: limit, trips, calibration */
            testjig_service();
            power_service(ack_on);         /* power events, D9: overcurrent signal / acknowledge patterns */
            signal_loss_check();
            upd_service();
            temp_service();
            vsense_service();
            if (ms_now - lim_ms >= 1000u) {
                limit_update();            /* the learned connected load drifts */
            }
            if (dark_tick(&dk, ms_now)) {
                outputs_dirty = 1;         /* the dark hold fades: every ms */
            }
            stagger_adapt();
            ultra_clock_tend(&ul, tick_now);   /* the engine's time: continuous without headers, across the wrap */
            if (ultra_on || fx_ch) {       /* (the power-on effect: also before a protocol is there) */
                if (ultra_render(&ul, tick_now, uout)) {
                    outputs_dirty = 1;     /* 1 kHz updates: every PWM period after this uses the new value */
                }
                if (fx_ch) {
                    fx_follow();
                }
            }
            if (!ultra_on && smooth_update(&sm, tick_now)) {
                outputs_dirty = 1;
            }
            fx_service();
            if (ms_now - fps_ms >= 1000u) {
                stat.fps = frames - fps_frames;
                fps_frames = frames;
                fps_ms += 1000u;           /* (whole seconds: the uptime) */
                up_s++;
                energy_second(&en, ms_now, up_s, (uint32_t)pw.short_trips << 16 | pw.ovl_trips, is.i_avg, temp_peak);
            }
            flash_erase_ahead();           /* while low: the stores' next pages */
            /* save 2 s after the last change, not while a pattern is blinking (except its final dark pause), with a
             * token of the settings' write budget: a burst, then one per STORE_REFILL_MS (and when the flash admits
             * it: 0.6.4). The newest values wait, so the last change is never lost (only stored later). */
            if (settings_dirty && ms_now - changed_ms >= SAVE_DEBOUNCE_MS && (!ack.active || ack.seg + 1u == ack.nseg) &&
                (may = flash_may(settings_pending() == 2u, 0u)) != 0u && budget_take(&set_budget, ms_now, 1u)) {
                (void)settings_write(may);
            }
            scene_service();
            if (srun.fading || ss.state == SS_RUN) {
                outputs_dirty = 1;         /* power-on scene cross-fade, soft start (offset, ramp): every ms */
            }
        }
        if (outputs_dirty) {
            outputs_dirty = 0;
            outputs_update(ack_on);
        }
        /* the calibration (a newer test meanwhile: that one; 2 = a cleared record, the factory reset's) */
        if (cal_save_pend && (may = flash_may(cal_pending() == 2u, 0u)) != 0u && budget_take(&cal_budget, ms_now, 1u)) {
            isense_cal_t c;
            uint32_t have = isense_cal_get(&is, &c);
            if (cal_save_pend == 2u || have) {
                if (!have) {
                    uint8_t *z = (uint8_t *)&c;
                    uint32_t k;
                    for (k = 0; k < sizeof c; k++) {
                        z[k] = 0;          /* levels 0: cleared */
                    }
                }
                flash_stall_begin(may);
                if (flash_stall_end(cal_store(&c, &cal_cnt))) {
                    cal_present = (uint8_t)have;
                    cal_save_pend = 0;
                }
            } else {
                cal_save_pend = 0;
            }
        }
        /* (claude-cpu, 0.6.4) nothing left to do: the core sleeps until the next interrupt, at the latest the poll tick
         * (50 us); a flag an interrupt sets after these checks waits for that tick. Only the core stops: DMA, timers,
         * ADC and the UART run on (RM 2.3.2; AHBPCENR SRAMEN = 1 from reset), and a debugger keeps it awake. Not during
         * a flash operation (flash_op runs inside this pass), so its SRAM tick is unaffected. ~4 mA less at 3.3 V
         * (datasheet tables 4-6-1 / 4-7-1: 11.4 -> 7.1 mA at 144 MHz), wake-up 1.4 us. BENCH: the current, the
         * debugger, the slew interrupt's budget at 100 kHz. */
        if (!cap_ready && !fe_ready && ms_now == last_ms) {
            CPU_SLEEP();
        }
    }
}
