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
#include "../core/isense.h"
#include "../core/mcutemp.h"
#include "../core/ocled.h"
#include "../core/pwrev.h"
#include "../core/vsense.h"
#include "../core/proto.h"
#include "../core/scene.h"
#include "../core/smooth.h"
#include "../core/stagger.h"
#include "../core/testjig.h"
#include "../core/ultra.h"
#include "../core/upd_app.h"
#include "../core/ws_decode.h"

#ifndef FW_VERSION
#define FW_VERSION       "0.5.2"   /* release builds: the release version (tools/build_release.sh) */
#endif
#ifndef UPD_APP
#define UPD_APP 0                  /* 1: linked behind the bootloader (vendor/wch/Link_app.ld, build_release.sh) */
#endif
#define UPD_WD_CLEAR_MS  10000u    /* the bootloader's watchdog-loop count is cleared after this long running */
#define LOSS_TIMEOUT_MS  2000u     /* WS modes: signal loss = no valid frame for 2 s */
#define LOSS_FADE_MS     1000u     /* fade-off duration */
#define SAVE_DEBOUNCE_MS 2000u     /* settings saved 2 s after the last change */

/* ------------------------------------------------------------------------------- shared with the poll ISR */
static frx_t frx;                          /* protocol state and per-frame decisions (core/frame_rx.h) */
static volatile gate_plan_t plan;          /* plan of the frame in progress */
static gate_plan_t plan_buf[2];            /* next plan, written by the main loop (double buffer) */
static volatile uint8_t plan_idx;
static ftrack_t ft;                        /* frame tracking (ISR; the main loop only with interrupts off) */
static volatile uint8_t rx_ready;          /* main has set up the plans: the ISR starts tracking frames */
static volatile uint16_t cap_start;        /* ring word 60 us before the frame's first edge (inside the gap) */
static volatile uint16_t cap_words;
static volatile uint8_t cap_ready;
static volatile uint16_t cap_bits;         /* bits to decode */
static volatile uint16_t cap_own;          /* this board's bits under the plan of that frame */
static volatile uint16_t cap_skip;         /* WS2915: the capture starts after the header pause (hdr bits) */
static volatile uint8_t cap_proto;
static volatile uint32_t cap_frame;
static volatile uint8_t fe_ready;          /* frame end: total bits for the classifier */
static volatile uint16_t fe_bits;
static volatile uint32_t fe_frame;
static volatile uint32_t fe_tick;          /* tick of the frame's last edge (0-50 us after it): Ultra clock */
static volatile uint16_t tail_words;       /* tail_copy holds the frame's end (Ultra lengths only), 0 = none */
static uint32_t edge_tick;                 /* ISR: tick and ring position of the latest new edges */
static uint16_t edge_pos;
static volatile uint32_t ms_now, tick_now; /* 1 ms and 50 us time bases */
static volatile uint32_t frames;
/* board current sense (core/isense.h): the tick sums the ADC samples and trips; the main loop limits (isense_apply
 * in outputs_update), calibrates and retries */
static isense_t is;

#define HINT_WORDS 135u                    /* 60 us at 36 MHz, 16 samples per word */
#define TAIL_WORDS 576u                    /* 256 us up to the frame's last edge: 64 bits at 1.5 us + hiccups + tick */
#define TAIL_PAST  4u                      /* words past the ring position of the last-edge tick */
static uint16_t tail_copy[TAIL_WORDS];     /* the end of the frame, for the Ultra header (ws_decode_last) */

/* ring words [start, end) -> dst (at most max words), two straight runs; returns the words copied */
static uint32_t ring_copy(uint16_t *dst, uint32_t start, uint32_t end, uint32_t max)
{
    uint32_t n = (end - start) & (DIN_RING_WORDS - 1u), first, i;
    uint16_t *d = dst;
    const uint16_t *s = &din_ring[start];
    if (n > max) {
        n = max;
    }
    first = DIN_RING_WORDS - start;
    if (first > n) {
        first = n;
    }
    for (i = 0; i + 1u < first; i += 2u, d += 2, s += 2) {      /* start is even: 32-bit copies */
        *(uint32_t *)d = *(const uint32_t *)s;
    }
    for (s = din_ring; i + 1u < n; i += 2u, d += 2, s += 2) {   /* first is even too (ring size even) */
        *(uint32_t *)d = *(const uint32_t *)s;
    }
    if (i < n) {
        *d = *s;
    }
    return n;
}

void poll_isr(void);                       /* 20 kHz tick, entered via isr.S (HPE + mret) */
void poll_isr(void)
{
    static uint16_t sub;
    uint16_t c = DIN_EDGES(), own;
    uint32_t ev;

    POLL_ACK();
    tick_now++;
    if (++sub >= POLL_HZ / 1000u) {
        sub = 0;
        ms_now++;
        hw_adc_ms();                       /* the ADC's injected group: voltage channels / MCU temperature */
    }
    if (is.enabled && isense_tick(&is, ISENSE_RAW())) {
        pwm_force_off(1);                  /* a short: all outputs off now, no table rebuild */
    }
    if (!rx_ready) {
        return;
    }
    /* a provisional (stored) protocol captures an Ultra slot too: a different protocol locks at once (an update
     * session's transparent plan: still its whole 384-bit frame) */
    own = plan.own_bits;
    if (frx.ps.provisional && own < PROTO_DETECT_BITS) {
        own = PROTO_DETECT_BITS;
    }
    ev = ftrack_tick(&ft, c, own, plan.hdr_bits);
    if (ev == 0u) {
        return;
    }
    if (ev & FT_EDGE) {                    /* the last edges so far: where the frame ends (in time and ring) */
        edge_tick = tick_now;
        edge_pos = (uint16_t)din_ring_pos();
    }
    if (ev & FT_CAP_COPY) {                /* one tick after our last bit: the DMA has surely written it */
        cap_words = (uint16_t)ring_copy(din_copy, cap_start, din_ring_pos(), DIN_COPY_WORDS);
        cap_bits = ft.cap_bits;
        cap_ready = 1;
    }
    if (ev & FT_CAP_START) {               /* first edges (0-50 us ago): a ring position inside the reset gap */
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
        gate_plan_t g = plan_buf[plan_idx];
        if (ev & FT_CAP_SHORT) {           /* frame shorter than the capture: decode what came */
            cap_words = (uint16_t)ring_copy(din_copy, cap_start, din_ring_pos(), DIN_COPY_WORDS);
            cap_bits = ft.cap_bits;
            cap_ready = 1;
        }
        if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END) {
            fe_bits = c;                   /* the main loop classifies the frame and plans the next ones */
            fe_frame = frames;
            fe_tick = edge_tick;
            /* the end of a frame of an Ultra length: its last 64 bits are the header (read in the main loop) */
            tail_words = frx_tail_wanted(&frx, c)
                ? (uint16_t)ring_copy(tail_copy, (edge_pos + TAIL_PAST - TAIL_WORDS) & (DIN_RING_WORDS - 2u),
                                      edge_pos + TAIL_PAST, TAIL_WORDS)
                : 0u;
            fe_ready = 1;
        }
        gate_rearm(&g);                    /* the next frame starts >= 280 us after the last edge */
        plan = g;
        if (ev & FT_FRAME_END) {
            frames++;
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
static uint16_t out_now[5];                /* output values in use (for the adaptive stagger) */
static uint16_t built_v[5];                /* value each channel's table holds */
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
/* voltage channels (core/vsense.h): PA4 the supply, PA0 the spare input on TP8 */
static vsense_t vs_sup, vs_aux;
static testjig_t tj;                       /* test jig mode (core/testjig.h): TP9 grounded */

/* every ms: the injected group's voltage pairs of that ms */
static void vsense_service(void)
{
    uint32_t sum2[2], n;
    hw_vsense_take(sum2, &n);
    vsense_add(&vs_sup, sum2[0], n);
    vsense_add(&vs_aux, sum2[1], n);
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
    if (temp_started) {
        mcutemp_sample(&mt, ts2, vref2);
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
static uint8_t isense_calibrate(uint8_t duty_pct)
{
    if (!is.enabled || is.tripped != ISENSE_TRIP_NONE || tj.active) {
        return 0;                          /* (the test jig's pattern keeps the outputs) */
    }
    isense_cal_start(&is, duty_pct == 0u ? 0u : (uint16_t)((uint32_t)duty_pct * 65535u / 100u));
    ack_stop(&ack);                        /* the calibration pattern is the feedback */
    outputs_dirty = 1;
    return 1;
}

static void flash_stall_begin(void);
static void flash_stall_end(uint32_t us);

/* every ms, before the outputs are rendered: the tick's samples of that ms -> limit, trips, calibration */
static void isense_service(void)
{
    uint32_t sum, n, peak, ev;
    if (!is.enabled) {
        return;
    }
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
    }
    if (ev & ISENSE_EV_REAPPLY) {
        outputs_dirty = 1;
    }
    if (ev & ISENSE_EV_CAL_DONE) {
        cal_save_pend = 1;                 /* the test with its curves: its own page, once the content is back */
        if (isense_export(&is, sext.ich25, &sext.izero)) {
            settings_dirty = 1;            /* the 100 % values also in the settings (older firmware), debounced */
            changed_ms = ms_now;
        }
        limit_update();                    /* the connected load is known now */
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

static void outputs_update(uint8_t ack_on)
{
    uint16_t v[5];
    uint8_t i;
    if (tj.active) {
        tj_outputs(&tj, ms_now, v);        /* test jig: the fade over the five channels; data doesn't change it */
    } else if (ack.active) {
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
    isense_apply(&is, v, v);               /* board limit (all channels scaled together), or the calibration pattern */
    for (i = 0; i < 5u; i++) {
        out_now[i] = v[i];
    }
    /* phases spread the extra dither steps / skipped pulses of the five channels over the pattern; only the
     * channels whose value changed are rebuilt */
    {
        static const uint8_t order[5] = { CH_G, CH_R, CH_B, CH_W2, CH_W1 };
        for (i = 0; i < 5u; i++) {
            uint8_t ch = order[i];
            if (!built_ok || built_v[ch] != v[ch]) {
                build(ch, v[ch], (uint8_t)(i * len / 5u));
                built_v[ch] = v[ch];
            }
        }
        built_ok = 1;
    }
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

/* follow-up of an applied command: blink feedback, new settings live, save debounced */
static cfg_result_t handle_result(cfg_result_t r)
{
    if (r.event == CFG_ACK && (r.cmd == CMD_ISENSE_LIMIT || r.cmd == CMD_ISENSE_CAL)) {
        if (!is.enabled) {
            r.event = CFG_NACK;            /* no current sense on this board */
            ack_start(&ack, ACK_P_NACK, 0);
            return r;
        }
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
            if (cal_present) {
                isense_cal_t none;
                uint8_t *z = (uint8_t *)&none;
                uint32_t k;
                for (k = 0; k < sizeof none; k++) {
                    z[k] = 0;
                }
                flash_stall_begin();       /* a cleared record (levels 0) */
                flash_stall_end(cal_store(&none, &cal_cnt));
                cal_present = 0;
            }
            __disable_irq();               /* the tick must not see the half-initialised trip level */
            isense_init(&is, is.enabled, ISENSE_FUSE_MA, ISENSE_BOARD_MA);
            isense_set_gain(&is, ISENSE_GAIN_DEFAULT_Q16 * 100u / ISENSE_AMP_GAIN);
            __enable_irq();
            ul.board_limit_ext = is.enabled;
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

/* A flash write stops the CPU with interrupts off (~2-18 ms) and halves the clocks: drop a capture in progress
 * (its samples are not at 36 MHz) and move the time bases along. The gate is closed meanwhile (downstream boards
 * miss a frame or two); the next idle gap or frame end re-arms it. */
static void flash_stall_begin(void)
{
    __disable_irq();
    ftrack_disarm(&ft);
    cap_ready = 0;
    __enable_irq();
}

static void flash_stall_end(uint32_t us)
{
    __disable_irq();
    ftrack_disarm(&ft);
    cap_ready = 0;
    tick_now += us / ULTRA_TICK_US;
    ms_now += us / 1000u;
    __enable_irq();
    ultra_clock_gap(&ul);
    stat.saves++;
}

static void save_now(void)
{
    settings_dirty = 0;
    flash_stall_begin();
    flash_stall_end(settings_store(&set, &sext));
}

static void scene_save_now(void)
{
    flash_stall_begin();
    flash_stall_end(scene_store(&scene_want));
    scene_stored = scene_want;
}

/* every ms: Ultra take-over deadline, recompute the scene from the lights' SCENE values once things are quiet,
 * write it when due (SCENE_SAVE_MS after the last change, within the write budget: core/scene.h), not while a
 * pattern blinks (except its final dark pause) */
static void scene_service(void)
{
    if (ultra_on && srun.hold && ultra_seen && ms_now - ultra_first_ms >= SCENE_TAKEOVER_MS) {
        scene_takeover(&srun, 0x1Fu, ms_now);     /* the host is there but sends nothing for these channels */
        outputs_dirty = 1;
    }
    if (scene_check && ultra_on && ms_now - scene_check_ms >= SCENE_SAVE_MS) {
        scene_t sc;
        scene_check = 0;
        ultra_scene(&ul, &scene_stored, &sc);
        scene_want = sc;
        scene_major_pend = (uint8_t)scene_major(&scene_stored, &sc);
        scene_changed_ms = scene_check_ms;
    }
    if (!scene_equal(&scene_want, &scene_stored) && (!ack.active || ack.seg + 1u == ack.nseg) &&
        scene_save_due(&sbudget, ms_now, scene_changed_ms, scene_major_pend)) {
        scene_save_now();
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
    if (!ultra_seen) {
        ultra_seen = 1;                    /* the power-on scene holds at most SCENE_TAKEOVER_MS from here */
        ultra_first_ms = ms_now;
    }
    if (slot == 0) {
        return;
    }
    ev = ultra_on_slot(&ul, slot, hdr_tick);
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
        ul.cfg_changed = 0;
        apply_settings();                  /* host-managed: live at once, no blink, not saved (except id 22) */
        stat.config_frames++;
    }
    if (ev & ULTRA_EV_IDENTIFY) {
        if (ul.identify == ULTRA_IDENT_CAL) {
            (void)isense_calibrate(ul.identify_arg);   /* boards without current sense ignore it */
        } else if (ul.identify == ULTRA_IDENT_DARK) {
            dark_hold(&dk, ms_now, (ul.identify_arg ? ul.identify_arg : 60u) * 1000u);
        } else if (ul.identify == ULTRA_IDENT_RELEASE) {
            dark_release(&dk, ms_now);
        } else if (ul.identify == ULTRA_IDENT_VERSION) {
            ack_start_version(&ack, fw_version);
        } else if (ul.identify == ULTRA_IDENT_ON) {      /* (ultra_on_slot drops unknown values) */
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
        ultra_init(&ul, tick_now ^ 0xA5C3u);
        ul.board_limit_ext = is.enabled;   /* CONFIG 21 is the measured limit (isense), not the computed one */
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
        uint32_t a = UPD_APP_DESC + 2u * (UPD_ST_FIRST + upd.info.st_next);
        flash_stall_begin();
        flash_stall_end(hw_flash_half(a, UPD_ST_CONFIRMED));
        upd_app_confirmed(&upd, *(volatile const uint16_t *)a == UPD_ST_CONFIRMED);   /* else: at the next PING */
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

static void upd_capture(void)
{
    static const ws_timing_t tm = WS_TIMING_DEFAULT;
    static uint8_t f[UPD_FRAME_BYTES + 4];
    if (cap_bits < UPD_FRAME_BITS ||
        ws_decode_frame(din_copy, (uint32_t)cap_words * 16u, (uint16_t)UPD_FRAME_BITS, &tm, f) != (int)UPD_FRAME_BITS) {
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
    upd_action(upd_app_poll(&upd, ms_now));
#if UPD_APP
    if (!upd_wd_cleared && ms_now >= UPD_WD_CLEAR_MS && upd_ho_valid(HO)) {
        HO->wd_resets = 0;                 /* running fine: the bootloader starts counting again */
        upd_ho_seal(HO);
        upd_wd_cleared = 1;
    }
#endif
}

static void process_capture(void)
{
    static const ws_timing_t tm = WS_TIMING_DEFAULT;
    static uint8_t bits[PROTO_BITS_BUF];
    uint32_t skip = cap_skip, want = cap_bits - skip;
    proto_t p = (proto_t)cap_proto;
    slot_t slot;
    int n;

    cap_ready = 0;
    if (cap_own == PROTO_UPD_BITS && skip == 0u) {
        upd_capture();                     /* a whole update frame (the transparent plan): not a slot */
        return;
    }
    n = ws_decode_frame(din_copy, (uint32_t)cap_words * 16u, (uint16_t)want, &tm, bits);
    if (n != (int)want) {
        if (n < 0 && -n < 6) {
            stat.dec_err[-n]++;
        }
        return;
    }
    stat.decoded++;
    if (frx_capture(&frx, cap_frame, p, bits, want, cap_own, skip, &slot) & FRX_SHOW_WS) {
        handle_slot(&slot, p);
    }
}

static void process_frame_end(void)
{
    static const ws_timing_t tm = WS_TIMING_DEFAULT;
    uint8_t tail[8], i;
    const uint8_t *t8 = 0;
    frx_end_t e;
    fe_ready = 0;
    if (tail_words != 0u && ws_decode_last(tail_copy, (uint32_t)tail_words * 16u, &tm, tail) >= (int)ULTRA_HDR_BITS) {
        t8 = tail;                         /* the frame's last 64 bits: an Ultra header? */
    }
    frx_frame_end(&frx, fe_frame, fe_bits, t8, &e);
    upd_app_frame_end(&upd, e.ultra_header, e.flags, ms_now);
    if (e.ultra_header && (e.flags & ULTRA_HDR_UPD)) {
        last_data_ms = ms_now;             /* an update session is signal too (lights hold / effects run) */
        signal_lost = 0;
    }
    if (e.after == PROTO_ULTRA && proto_tail_wanted(fe_bits)) {
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
        handle_ultra(e.host_us, fe_tick, e.ultra_slot ? frx.slot : 0);   /* also on the frame that locked Ultra */
    }
    if (frx.ps.mode == PMODE_AUTO && frx.ps.locked != PROTO_NONE && !frx.ps.provisional &&
        frx.ps.locked != set.last_proto) {
        set.last_proto = (uint8_t)frx.ps.locked;   /* sticky auto: written only when the locked protocol changes */
        settings_dirty = 1;
        changed_ms = ms_now;
    }
    i = (uint8_t)(plan_idx ^ 1u);          /* the ISR reads plan_buf[plan_idx] at the next frame end */
    plan_buf[i] = upd_app_transparent(&upd) ? proto_gate_plan_transparent() : e.plan;
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

/* every ms: the power events (core/pwrev.h) and D9. The state comes from the current sense's limiter and trips, or
 * without it from the Ultra power limit (CONFIG 21 on the configured channel currents: computed, no trips). D9 by
 * priority: trip > acknowledge / identify / version > limit > off (core/ocled.h; the feedback channel still shows the
 * acknowledge patterns, outputs_update) */
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
    pwrev_update(&pw, ms_now, &in);
    ocled_update(&ocl, ms_now, in.limiting, in.trip_cause);
    lv = ocled_led(&ocl, ms_now, ack.active, ack_on);
    if (tj.active && ocl.show == OCLED_OFF && !ack.active) {
        lv = tj_led(&tj, ms_now);          /* test jig: on, off at each cycle start (overcurrent / acknowledge win) */
    }
    if (lv != led_now) {
        led_now = lv;
        led_set(lv);
    }
}

/* every ms: the test jig's pad (debounced), and in test mode the pattern (every ms) and its current report */
static void testjig_service(void)
{
    if (tj_pad(&tj, ms_now, TESTJIG_PAD_LOW())) {
        uart_write(tj.active ? "test mode: on (TP9 grounded)\r\n" : "test mode: off\r\n");
        outputs_dirty = 1;
    }
    if (!tj.active) {
        return;
    }
    outputs_dirty = 1;
    if (tj_measure(&tj, ms_now, is.i_ms, is.zero_ma)) {
        cli_test_report(&cli, tj.ma, TJ_PEAK_PCT, tj.cycles, vs_sup.valid ? vs_sup.mv : 0u);
    }
}

static void status_snapshot(void)
{
    uint8_t i;
    stat.uptime_ms = ms_now;
    stat.proto_mode = (uint8_t)frx.ps.mode;
    stat.proto_locked = (uint8_t)proto_active(&frx.ps);
    stat.proto_provisional = frx.ps.provisional;
    stat.frames = frames;
    stat.steps = steps;
    for (i = 0; i < 5u; i++) {
        stat.out[i] = ultra_on ? uout[i] : sm.cur[i];
    }
    stat.isense = is.enabled;
    stat.ilimit50 = sext.ilimit50 == 0xFFu ? 0u : sext.ilimit50;
    if (is.enabled) {
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
        {
            isense_cal_t c;
            uint8_t l;
            stat.cal_levels = isense_cal_get(&is, &c) ? c.levels : 0u;
            stat.cal_count = cal_cnt;
            for (l = 0; l < 4u; l++) {
                stat.cal_pct[l] = c.pct[l];
                for (i = 0; i < 5u; i++) {
                    stat.cal_ma[i][l] = c.ma[i][l];
                }
            }
        }
    }
    stat.lim_ma = clim.limit_ma;
    stat.lim_src = clim.src;
    stat.lim_need = clim.need;
    stat.lim_conn_ma = clim.connected_ma;
    stat.mcu_temp_c10 = mt.c10;
    stat.vdda_mv = mt.vdda_mv;
    stat.test_mode = tj.active;
    stat.vs_supply_mv = vs_sup.valid ? vs_sup.mv : 0u;
    vsense_report(&vs_sup, &stat.vs_supply_min_mv, &stat.vs_supply_max_mv);
    stat.vs_aux_mv = vs_aux.valid ? vs_aux.mv : 0u;
    stat.oc_show = ocl.show;
    stat.oc_cause = ocl.last_cause;
    stat.oc_limiting = ocl.limiting;
    stat.oc_tripped = (uint8_t)(ocl.trip_cause != 0u);
    stat.oc_since_ms = ocl.since_ms;
    stat.oc_hold_ms = ocled_hold_left(&ocl, ms_now);
    stat.pw_state = pwrev_state(&pw);
    pwrev_page(&pw, stat.pw_page);
}

int main(void)
{
    uint32_t last_ms = 0, fps_ms = 0, fps_frames = 0;
    uint8_t ack_on = 0, was_active = 0;
    int ch;

    if (settings_load(&set, &sext) != 0) {
        settings_default(&set);            /* (the extension is cleared too) */
    }
    if (scene_load(&scene_stored) != 0) {
        scene_clear(&scene_stored);        /* none stored: dark until data comes (as before) */
    }
    scene_want = scene_stored;
    scene_boot(&srun, &scene_stored);
    dark_init(&dk);
    ocled_init(&ocl);
    tj_init(&tj, TJ_CYCLE_MS);
    pwrev_init(&pw, ms_now);
    scene_budget_init(&sbudget, 0);
    config_rx_init(&rx);
    frx_init(&frx, (proto_mode_t)set.protocol, (proto_t)set.last_proto);
    smooth_init(&sm);
    ultra_init(&ul, 1);
    hw_init();
    mcutemp_init(&mt);
    vsense_init(&vs_sup, VSENSE_SUPPLY_RT, VSENSE_SUPPLY_RB, VSENSE_VREF_MV);
    vsense_init(&vs_aux, VSENSE_AUX_RT, VSENSE_AUX_RB, VSENSE_VREF_MV);
    hw_adc_init();                         /* PA7 continuously, the injected group from the poll tick */
    {
        __disable_irq();                   /* the tick is running: no half-initialised trip level */
        isense_init(&is, 1, ISENSE_FUSE_MA, ISENSE_BOARD_MA);
        isense_set_gain(&is, ISENSE_GAIN_DEFAULT_Q16 * 100u / ISENSE_AMP_GAIN);   /* board.h: amplifier gain */
        __enable_irq();
    }
    isense_restore(&is, sext.ich25, sext.izero);                         /* stored calibration (100 % values) */
    {
        isense_cal_t c;
        if (cal_load(&c, &cal_cnt) == 0) {
            isense_cal_set(&is, &c);       /* the last test with its curves wins */
            cal_present = 1;
        }
    }
    ul.board_limit_ext = is.enabled;
    stat.reset_flags = hw_reset_flags();
    upd_boot();                            /* UPD_APP: the reset flags from the hand-over, the bootloader's result */
    fw_version = fwver_parse(FW_VERSION);
    if (upd.info.desc_ok && upd.info.version != 0u) {
        fw_version = upd.info.version;     /* release builds: the descriptor, what the boards compare in an update */
    }
    fwver_format(fw_version, fw_version_text);
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
    cli.set = &set;
    cli.scene = &scene_want;
    cli.st = &stat;
    ultra_on = (uint8_t)(proto_active(&frx.ps) == PROTO_ULTRA);
    apply_settings();
    {
        gate_plan_t g = proto_gate_plan(proto_active(&frx.ps));
        plan_buf[0] = plan_buf[1] = g;
        plan = g;
        rx_ready = 1;                      /* gate closed until the tracker arms (idle line) or a frame ends */
    }
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
        while ((ch = uart_getc()) >= 0) {
            status_snapshot();
            cli_feed(&cli, (char)ch);
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
            if (ultra_on) {
                if (ultra_render(&ul, tick_now, uout)) {
                    outputs_dirty = 1;     /* 1 kHz updates: every PWM period after this uses the new value */
                }
            } else if (smooth_update(&sm, tick_now)) {
                outputs_dirty = 1;
            }
            if (ms_now - fps_ms >= 1000u) {
                stat.fps = frames - fps_frames;
                fps_frames = frames;
                fps_ms = ms_now;
            }
            /* save 2 s after the last change, not while a pattern is blinking (except its final dark pause) */
            if (settings_dirty && ms_now - changed_ms >= SAVE_DEBOUNCE_MS &&
                (!ack.active || ack.seg + 1u == ack.nseg)) {
                save_now();
            }
            scene_service();
            if (srun.fading) {
                outputs_dirty = 1;         /* power-on scene cross-fade: every ms */
            }
        }
        if (outputs_dirty) {
            outputs_dirty = 0;
            outputs_update(ack_on);
        }
        if (cal_save_pend) {
            isense_cal_t c;
            cal_save_pend = 0;
            if (isense_cal_get(&is, &c)) {
                flash_stall_begin();
                flash_stall_end(cal_store(&c, &cal_cnt));
                cal_present = 1;
            }
        }
    }
}
