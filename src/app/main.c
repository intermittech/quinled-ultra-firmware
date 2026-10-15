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
#include <string.h>
#include "ch32v20x.h"
#include "../hal/hw.h"
#include "../core/ack.h"
#include "../core/cli.h"
#include "../core/climit.h"
#include "../core/config.h"
#include "../core/cpuload.h"
#include "../core/dark.h"
#include "../core/dither.h"
#include "../core/frame_rx.h"
#include "../core/frame_track.h"
#include "../core/fwver.h"
#include "../core/gamma.h"
#include "../core/health.h"
#include "../core/isense.h"
#include "../core/scratch.h"
#include "../core/lto.h"
#include "../core/mcutemp.h"
#include "../core/ocled.h"
#include "../core/perf.h"
#include "../core/pwrev.h"
#include "../core/rc.h"
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
#define FW_VERSION       "0.6.10"   /* release builds: the release version (tools/build_release.sh) */
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
/* overruns (0.6.5 D4; console `status` / `watch`, display only): [0] captures dropped (the last one not decoded yet),
 * [1] frame ends not handled (the last one still pending), both counted by the tick; [2] render ms late: the main loop's
 * ms came this many ms after the one before, not counted after a flash operation (ovr_on = 0: housekeeping) */
static volatile uint32_t ovr[3];
static uint16_t cpu_peak;                  /* (0.6.5 D3, cpu_second below; 0.6.9: the DIAG page's too) */
#if BOARD_RETURN
static uint16_t rc_peak_ma;                /* (0.6.9) the input current's peak since the last ENERGY answer (bus side) */
#endif
static uint8_t ovr_on;
/* board current sense (core/isense.h): the tick sums the ADC samples and trips; the main loop limits (isense_apply
 * in outputs_update), calibrates and retries */
static isense_t is;
/* (0.6.5, D1) the reset breadcrumbs (core/cli.h bcrumb_t): RAM no reset clears (Link_sections.ld .noinit). Plain
 * stores, no read-modify-write: the tick writes pc, the main loop up_s, isr.S's fault handler pc / cause; read only at
 * boot, before the tick starts */
bcrumb_t bcrumb __attribute__((section(".noinit")));
typedef char bcrumb_offsets[(offsetof(bcrumb_t, pc) == 8u && offsetof(bcrumb_t, cause) == 12u) ? 1 : -1];   /* isr.S */

#define HINT_WORDS 135u                    /* 60 us at 36 MHz, 16 samples per word */
#define TAIL_WORDS 576u                    /* 256 us up to the frame's last edge: 64 bits at 1.5 us + hiccups + tick */
#define TAIL_PAST  4u                      /* words past the ring position of the last-edge tick */
/* the end of the frame, for the Ultra header (ws_decode_last); 4-byte aligned like din_copy: copy_run writes it in
 * 32-bit words (an array of uint16_t is only 2-byte aligned: today it lands on a word by the sizes before it) */
static uint16_t tail_copy[TAIL_WORDS] __attribute__((aligned(4)));
#if BOARD_RETURN
/* the return channel (core/rc.h): a POLL in this board's slot of an RW frame. The main loop arms the window at that
 * frame's end (rc_arm: the schedule from the frame's last edge, the answer built), the poll ISR runs it (rc_tick: turn
 * around, the burst, forward again at the window's end with the gate re-armed) and tracks no frames meanwhile: DIN
 * carries the relayed answers. */
enum { RC_IDLE = 0, RC_ARMED = 1, RC_TURNED = 2, RC_CHECKED = 3 };
static volatile uint8_t rc_state;          /* the main loop sets ARMED (from IDLE, after the rest), the ISR the rest */
static uint8_t rc_send;                    /* ISR: rc_ans still to send in this window */
static volatile uint8_t rc_contention;     /* a burst found the line taken since the last answer went out */
static volatile uint16_t rc_fe_pos;        /* ISR: the ring position at the last-edge tick of the frame end (fe_tick) */
static volatile uint16_t rc_answers, rc_aborts;   /* ISR: answers sent / aborted for contention (ERRORS page) */
static rc_sched_t rc_s;
static uint16_t rc_brr_v;
static uint8_t rc_ans[RC_ANSWER_BYTES];
#endif

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

/* ring words [start, end) -> dst (at most max words), two straight runs; returns the words copied (out of line: with
 * two callers LTO inlines it, and every path of poll_events saves one more register) */
static NOINLINE uint32_t ring_copy(uint16_t *dst, uint32_t start, uint32_t end, uint32_t max)
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
    /* one tick after our last bit (the DMA has surely written it), or at the end of a frame shorter than the capture:
     * decode what came (never both in one tick, nor with FT_CAP_START: core/frame_track.h; 0.6.10: one copy) */
    if ((ev & (FT_CAP_COPY | FT_CAP_SHORT)) && cap_accept) {
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
    } else if ((ev & (FT_CAP_START | FT_CAP_DATA)) == FT_CAP_START) {
        ovr[0]++;                          /* a frame's capture dropped: the last one not decoded yet */
    }
    if (ev & (FT_FRAME_END | FT_BOOT_ARM)) {
        const gate_plan_t *g = &plan_buf[plan_idx];   /* (the main loop writes only the other one) */
        if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END && !fe_ready) {
            uint32_t total = c + ((uint32_t)ft.end_wraps << 16);
            fe_bits = total;                   /* the main loop classifies the frame and plans the next ones */
            fe_frame = frames;
            fe_tick = edge_tick;
#if BOARD_RETURN
            rc_fe_pos = edge_pos;
#endif
            /* the end of a frame of an Ultra length: its last 64 bits are the header (read in the main loop) */
            tail_words = frx_tail_wanted(&frx, total)
                ? (uint16_t)ring_copy(tail_copy, (edge_pos + TAIL_PAST - TAIL_WORDS) & (DIN_RING_WORDS - 2u),
                                      edge_pos + TAIL_PAST, TAIL_WORDS)
                : 0u;
            fe_ready = 1;
        } else if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END) {
            ovr[1]++;                          /* the last frame end still pending: this one is not classified */
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

#if BOARD_RETURN
/* the response window, every tick from rc_arm to its end (core/rc.h): turn around, the turnaround check, this board's
 * burst, forward again. The gate is held closed for the whole window (the relayed answers are edges: a plan would open
 * it, forwarding UART traffic downstream as a frame); at the end it is re-armed with the newest plan, as at a frame
 * end. */
static NOINLINE void rc_tick(void)
{
    uint32_t t = tick_now;
    BARRIER();                             /* acquire the schedule/answer published before RC_ARMED */
    if (rc_state == RC_ARMED) {
        if ((int32_t)(t - rc_s.dir_tick) >= 0) {
            gate_force_closed();
            rc_hw_dir(RC_DIR_ANSWER);
            rc_state = RC_TURNED;
        }
        return;
    }
    if (rc_state == RC_TURNED && (int32_t)(t - rc_s.check_tick) >= 0) {
        rc_state = RC_CHECKED;
        if (rc_hw_send(rc_ans, 0, rc_brr_v)) {
            rc_hw_dir(RC_DIR_QUIET);       /* the segment above is still driven: no relay, no answer this window */
            rc_send = 0;
            rc_contention = 1;
            rc_aborts++;
        }
    } else if (rc_send && (int32_t)(t - rc_s.tx_tick) >= 0) {
        rc_send = 0;
        if (rc_hw_send(rc_ans, RC_ANSWER_BYTES, rc_brr_v)) {
            rc_hw_dir(RC_DIR_QUIET);       /* the line was taken: no answer (the host retries) and, as at the check
                                              (review r6), no more relay into the driver above for this window */
            rc_contention = 1;
            rc_aborts++;
        } else {
            rc_contention = 0;             /* (a CONTENTION before is in this answer's RFLAGS) */
            rc_answers++;
        }
    }
    if ((int32_t)(t - rc_s.end_tick) >= 0) {
        const gate_plan_t *g = &plan_buf[plan_idx];
        rc_hw_dir(RC_DIR_FORWARD);
        gate_rearm(g);
        plan.start_open = g->start_open;
        plan.t1 = g->t1;
        plan.t2 = g->t2;
        plan.own_bits = g->own_bits;
        plan.hdr_bits = g->hdr_bits;
        ftrack_window_end(&ft);
        BARRIER();                         /* finish using the mailbox before main may refill it */
        rc_state = RC_IDLE;
    }
}
#endif

void poll_isr(void);                       /* 20 kHz tick, entered via isr.S (HPE + mret) */
void poll_isr(void)
{
    PERF_T(pt);                            /* (a PERF build: core/perf.h; first, before the acknowledge) */
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
        bcrumb.pc = TRAP_PC();             /* (0.6.5, D1) where the main loop is, once a ms */
        adc_hold = (uint8_t)hw_adc_ms();   /* the ADC's injected group: voltage channels / MCU temperature */
    }
#if BOARD_RETURN
    if (rc_state != RC_IDLE) {
        rc_tick();                         /* the response window: no frame tracking until it ends */
        return;
    }
#endif
    /* (fable-algo) an idle line: ftrack_tick would return no event and change nothing (no edges, the count at 0
     * without wraps, no copy due, armed) */
    if (!rx_ready || (c == 0u && ft.last_cnt == 0u && ft.wraps == 0u && ft.cap_state != 2u && ft.armed)) {
        PERF_ADD(PERF_POLL, pt);
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
    PERF_ADD(PERF_POLL, pt);
}

#if PERF_BUILD
/* `perf` (core/perf.h, a bench build): the counts, the clock and the tick count at the last `perf` */
perf_t perf[PERF_N];
static uint32_t perf_clock0, perf_tick0;
#if PERF_BUILD == 2
/* the SysTick clock: ticks x cycles per tick + SysTick's count; a tick due but not counted yet (its flag up, the count
 * just past 0: the poll ISR before its acknowledge, or interrupts off) counts */
uint32_t perf_systick(void)
{
    uint32_t k = tick_now, c = *(volatile uint32_t *)&SysTick->CNT;
    if ((SysTick->SR & 1u) && c < 144000000u / POLL_HZ / 2u) {
        k++;
    }
    return k * (144000000u / POLL_HZ) + c;
}
#endif

void perf_take(perf_t *dst, uint32_t *clock, uint32_t *ticks)
{
    uint32_t i, now;
    __disable_irq();
    now = PERF_NOW();
    *clock = now - perf_clock0;
    *ticks = tick_now - perf_tick0;
    perf_clock0 = now;
    perf_tick0 = tick_now;
    for (i = 0; i < PERF_N; i++) {
        dst[i] = perf[i];
        perf[i].n = perf[i].max = 0;
        perf[i].sum = 0;
    }
    __enable_irq();
}
#endif

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
/* (0.6.9, B7) the trips' retry policy in the extension's spare bytes 2 / 3 (retry_set), coded so that an erased byte
 * (0xFF) is the default */
#define RETRY_N_X 0xF5u                    /* count ^ this (10 ^ 0xF5 = 0xFF) */
#define RETRY_S_X 0xFEu                    /* seconds ^ this (1 ^ 0xFE = 0xFF) */
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
/* (0.6.6) the console never waits for the UART (hal/hw.c uart_write waits while both 512-byte halves are full): the
 * reply to a line (each <= CLI_PART_ROOM: test_r5_bounds), a `status` / `help` part, a `watch` line and the test jig's
 * report go out only while the UART has CLI_PART_ROOM bytes free (the next line waits in the RX ring meanwhile, and a
 * due watch line or report a pass later); the firmware's own messages (a flash error, test mode, TP9's reset) wait
 * here, in order, and the main loop writes one a pass while the UART has SAY_ROOM free. Nothing is dropped: a fifth
 * message at once (never seen) is written at once, as before. */
#define SAY_ROOM 64u                       /* (every message is shorter) */
#define SAY_Q    4u                        /* (a power of two) */
static int say_q[SAY_Q];
static uint8_t say_h, say_n;
#if !CONSOLE_FULL
#define STREAM_STATUS_S 60u                /* (0.6.7) the release console: the full status every minute */
/* (0.6.9, Quindor) the receiver's release console sends no full status: its return channel's pages (STATUS, LIFETIME,
 * DIAG, ...) carry it; the banner, the live line, the jig's lines and the messages stay (CONSOLE_FULL 1: everything) */
#define STREAM_STATUS   (BOARD != BOARD_DP_RX)
#define STREAM_BOOT_S   5u                 /* ... the first once the live values are valid (the MCU temperature's first
                                              average, ~2 s: the supply's comes before), at the latest at this uptime */
#if STREAM_STATUS
static uint32_t status_s;                  /* ... the uptime of the last one */
static uint8_t status_first;               /* ... 1: the first one is out */
#endif
#endif

static void say(int z)
{
    if (say_n < SAY_Q) {
        say_q[(say_h + say_n++) & (SAY_Q - 1u)] = z;
    } else {
        cli_say(&cli, z);
    }
}
static uint8_t outputs_dirty = 1, settings_dirty, signal_lost;
static uint32_t steps;
static uint8_t len, run_stagger = 0xFF;
static uint32_t last_data_ms, changed_ms;
/* a stored setting changed: saved SAVE_DEBOUNCE_MS after the last change (out of line: eight callers) */
static NOINLINE void settings_changed(void)
{
    settings_dirty = 1;
    changed_ms = ms_now;
}
static scene_budget_t set_budget;          /* write budget of the settings page (store.h STORE_BUDGET_*) */
static uint32_t up_s;                      /* uptime in s: `uptime` and the long ages (ms_now wraps after 49.7 days) */
static uint16_t out_now[5];                /* output values in use, = the value each channel's table holds (0.6.4,
                                              gpt-opt2: one vector; main loop only, no ISR or build() reads it) */
static uint8_t built_ok;
static uint16_t st_o2 = STAGGER_FIXED_O2, st_o3 = STAGGER_FIXED_O3;
static uint8_t st_force;
static uint16_t st_seen[5];
static uint32_t st_ms;
static uint8_t st_moved = 1;               /* build_all ran since stagger_adapt last compared out_now with st_seen (the
                                              compare's answer can't change before: out_now is written there alone) */
/* power-on scene (core/scene.h): shown from power-up until data comes, kept in its own flash log */
#define SCENE_TAKEOVER_MS 15000u           /* Ultra: hold it at most this long after the board's first slot (each light
                                              takes over at its SET: this is for channels the host never sends; 0.6.6: 3 s
                                              after the first header, shorter than the hub's 50-board preamble and than a
                                              refresh cycle to a board powered up while the hub runs) */
static scene_run_t srun;
static scene_t scene_stored;               /* in flash */
static scene_t scene_want;                 /* the latest scene; written when it differs from scene_stored */
static uint8_t scene_pend;                 /* = !scene_equal(&scene_want, &scene_stored), kept where either changes */
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
/* (0.6.5, D2) the event log (core/cli.h evlog_t): main loop only */
static evlog_t evl __attribute__((section(".noinit.evl")));   /* (0.6.9, B2: kept across a reset, evlog_boot) */
typedef char evl_pw_bits[(EVL_OVERLOAD == EVL_LIMIT + 1 && EVL_SHORT == EVL_LIMIT + 2 && EVL_DIP == EVL_LIMIT + 3 &&
                          ULTRA_PW_EV_LIMITED == 1u && ULTRA_PW_EV_OVERLOAD == 2u && ULTRA_PW_EV_SHORT == 4u &&
                          ULTRA_PW_EV_SUPPLY_DIP == 8u) ? 1 : -1];

static void ev_add(uint32_t type, uint32_t ma)
{
    evl_ev_t *e = &evl.e[evl.head = (uint8_t)((evl.head + 1u) & (EVLOG_N - 1u))];
    e->t_s = up_s;
    e->ma = (uint16_t)ma;
    e->type = (uint8_t)type;
}
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
#if BOARD == BOARD_DP_RX
/* (0.6.10, protocol 0.2.6) the receiver's standby. IDENTIFY 7, the hub's planned power-off notice (vdip.planned,
 * PLAN_HOLD_MS): a supply dip starting meanwhile is the planned cut's for its whole length (core/vdip.h in_dip 2:
 * nothing counted or logged, no SUPPLY / STATUS / fault memory / power event) and writes the lifetime record only for
 * something worth storing (energy_due ENERGY_PLANNED: not the power-up alone); the outputs fade dark, an acknowledge
 * pattern too; a calibration stops. Still powered PLAN_HOLD_MS after it: as before. Nothing is kept for the next
 * start: the grace window decides it (every power-up dark until a valid frame or GRACE_MS, then the power-on scene) */
#define PLAN_HOLD_MS 2000u
#define GRACE_MS     1500u
static uint32_t plan_ms;
static uint8_t grace = 1;
static uint32_t grace_ms;                  /* the boot's ms - 1: last_data_ms until a valid frame */
#define PLANNED vdip.planned
#define PLAN_CUT(was) (vdip.in_dip > (was) + 1u)   /* the planned cut starts this ms (in_dip 0 -> 2) */
#else
#define PLANNED 0u
#define PLAN_CUT(was) 0u
#endif
#if MON_EXTRA
static calhist_t chist;                    /* (0.6.6, M2) the calibration history (core/calhist.h), its write pending */
static uint8_t chist_pend;
static uint8_t chist_retry;                /* (review round 6) its last write failed: each try again takes a token of */
static scene_budget_t chist_budget;        /* ... this budget, as the calibration's (CAL_BUDGET_CAP, then one per
                                              CAL_REFILL_MS); before, a failing write ran again every main-loop pass
                                              while low, erasing its page every fourth try */
#endif
static health_t hl;                        /* channel health (core/health.h) */
static sstart_t ss;                        /* soft start (core/sstart.h) */
/* the power-on effect (SCENE_EFFECT): what is stored (lights, their CRC-8), the channels still running it, the
 * engine untouched by a host since boot (it holds the power-on effect), a write pending, its budget */
static uint8_t fx_mask, fx_crc[ULTRA_LIGHTS], fx_ch, ul_used, fx_pend;
static uint8_t ul_stated;                  /* lights a SET reached since power-up (the power-on scene's take-over) */
static scene_budget_t fx_budget;
#define FX_BUDGET_CAP 4u
#define FX_REFILL_MS  21600000u            /* one write per 6 h sustained: >= 6 years at 10k cycles (2 erases each) */
/* the Diff-Power receiver (BOARD=2) has no test pad: the jig's code and console texts aren't built there */
#if BOARD_TESTJIG
static testjig_t tj;                       /* test jig mode (core/testjig.h): TP9 grounded */
static uint8_t tj_rep;                     /* (0.6.6) a cycle's report waits for the UART's room */
#define TJ_ACTIVE() (tj.active)
#else
#define TJ_ACTIVE() 0u
#endif
#if BOARD_RETURN
static uint8_t rc_pend;                    /* frx.slot holds a POLL from the start of frame rc_frame, whose end is not
                                              processed yet */
static uint32_t rc_frame;
static uint16_t rc_uid16;                  /* CRC-16 over the unique id: in every answer */
static uint16_t ls_res;                    /* (0.6.10) the last LIFE_SYNC's: energy_sync's (applied | result << 8) */
static uint16_t rc_windows, rc_missed;     /* windows taken part in; POLLs this board couldn't take part in */
static uint32_t cal_done_s;                /* uptime + 1 at the last calibration test (0: none since power-up) */
/* a window may come or runs: the stagger search and the flash wait (the window's deadline, ULTRA_RW_DIR_MAX_US) */
#define RC_BUSY() (rc_pend || rc_state != RC_IDLE)
#endif

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
/* dip writes (Quindor 2026-10-14): a program into an erased entry of an A/B store, the check written last, never an
 * erase, only with VDD above the PVD's 2.9 V (hw.c flash_op); a cut leaves the previous record (test r5dip). 0: no
 * write at all during a dip (when the bench finds the hold-up shorter than a record's write: README, bench). */
#ifndef FLASH_DIP_WRITES
#define FLASH_DIP_WRITES 1
#endif
#define FM_LOW    1u                       /* flash_may: low (or boot): erases too */
#define FM_PROG   2u                       /* a program into erased flash (a dip, the console's save) */
#define FM_BRIGHT 3u                       /* ... brighter: with the hour's token */
/* (review round 6) after a failed operation no regular write or erase for a while: FLASH_RETRY_MS after the first
 * failure, doubling with each failure in a row up to FLASH_RETRY_MS << FLASH_RETRY_MAX (~11 min); a success after it
 * ends it.
 * A flash that keeps failing (write protection over the record sectors, a worn page, an operation without EOP) must
 * not take the main loop nor wear a page out: 0.6.6 retried an erase ahead / the effects' clear every ms while low
 * and the calibration history's write every pass, so every frame was lost (the tracker disarmed each time), `flash:
 * error` filled the console, and an erase whose status alone failed ran ~60 times a second. A supply dip's write and
 * the console's `save` (FM_PROG) still try at once: the last chance, a user's command. */
#define FLASH_RETRY_MS   10000u
#define FLASH_RETRY_MAX  6u
static uint32_t flash_hi_ms;               /* the last ms the outputs were not low */
static uint32_t flash_err_ms;              /* (r6) the last failed operation; flash_err: failures in a row (0: none) */
static uint8_t flash_err;
static scene_budget_t flash_bright;        /* the writes while brighter */
static uint8_t flash_boot = 1;             /* before the main loop: the outputs are dark */
static uint8_t fx_clear_pend;              /* a factory reset: the power-on effects' erase is due */
static uint8_t reset_save;                 /* (r6, F10) a factory reset: its settings record (the defaults, the void
                                              bits: core/store.h STORE_VOIDS) written at once, a program */
static void void_done(uint8_t b);
static uint8_t flash_recheck = 1;          /* a flash operation ran since flash_erase_ahead last found nothing due: the
                                              stores' positions (core/pagepair.h) change only in one, so until then its
                                              four checks would answer the same */

/* (r6) in the hold-off after a failed operation (above) */
static uint32_t flash_held(void)
{
    return flash_err && ms_now - flash_err_ms < (FLASH_RETRY_MS << (flash_err - 1u));
}

/* may a store write now? need_erase: its next record needs an erase first (only while low); prog: FM_PROG when a
 * program may run whatever the outputs show (a dip, the console's save). Returns FM_LOW / FM_PROG / FM_BRIGHT
 * (flash_stall_begin takes the brighter state's token), 0 = not now. */
static uint32_t flash_may(uint32_t need_erase, uint32_t prog)
{
#if BOARD_RETURN
    if (RC_BUSY()) {
        return 0;                                        /* the return channel's window first */
    }
#endif
    if (prog != FM_PROG && uart_busy()) {
        return 0;                                        /* (the console first: hw.c flash_op) */
    }
    if (prog != FM_PROG && flash_held()) {
        return 0;                                        /* the hold-off after a failed operation (above) */
    }
    if (hw_vdd_low()) {
        /* (review round 6) below the PVD flash_op refuses at once (FLASH_ERR_VDD): nothing is admitted, so no store
         * burns a write token (0.6.6: a pending settings change tried every ms, 16 tokens in 16 ms, then waited 3 h
         * for the next) and no A/B entry is used up by a write that never started; the change stays pending */
        return 0;
    }
    if (flash_boot || (ms_now - flash_hi_ms >= FLASH_LOW_MS && !vdip.in_dip)) {
        return FM_LOW;
    }
    if (need_erase) {
        return 0;
    }
    if (vdip.in_dip) {
        return FLASH_DIP_WRITES ? FM_PROG : 0u;          /* a dip: programs only (or nothing) */
    }
    if (prog == FM_PROG) {
        return FM_PROG;
    }
    return budget_avail(&flash_bright, ms_now) ? FM_BRIGHT : 0u;
}

/* (0.6.9) the status: why a pending write still waits (cli.h cli_status_t save_why; the main reasons, in order) */
static uint8_t save_why(void)
{
    if (!(settings_dirty | cal_save_pend | scene_pend | fx_clear_pend | (fx_pend == 2u))) {
        return 0;
    }
    if (hw_vdd_low()) {
        return 5;
    }
    if (flash_held()) {
        return 4;
    }
    if ((settings_dirty && ms_now - changed_ms < SAVE_DEBOUNCE_MS) ||
        (scene_pend && ms_now - scene_changed_ms < SCENE_SAVE_MS)) {
        return 1;
    }
    return (ms_now - flash_hi_ms < FLASH_LOW_MS && !budget_avail(&flash_bright, ms_now)) ? 2u : 3u;
}

/* every ms while low (and once at power-up): one erase that is due, so a later write needs none. Not before
 * LIFE_PREP_S after power-up (a steady supply) except at power-up itself (before the outputs start: 0.6.3's
 * lifetime erase there keeps wall-switched boards going). */
static void flash_erase_ahead(void)
{
    uint32_t i;
    if ((!flash_recheck && !fx_clear_pend) ||            /* nothing due since the last operation (see above) */
        (!flash_boot && up_s < LIFE_PREP_S) || flash_may(1u, 0u) != FM_LOW) {
        return;
    }
    if (fx_clear_pend) {
        flash_stall_begin(FM_LOW);                       /* (a factory reset: the power-on effects go) */
        fx_clear_pend = (uint8_t)!flash_stall_end(scene_fx_store(&ul, 0, fx_crc));
        if (!fx_clear_pend) {
            void_done(STORE_VOID_FX);
        }
        return;
    }
    for (i = 0; i < REC_COUNT; i++) {                    /* (flashrec.h REC_*: the lifetime record, the settings, the
                                                            calibration, the scene log) */
        if (flash_rec_pending(i)) {
            flash_stall_begin(FM_LOW);
            (void)flash_stall_end(flash_rec_prepare(i));
            return;
        }
    }
    flash_recheck = 0;                                   /* none due: not asked again until a flash operation */
}

/* the lifetime record's writes (core/energy.h ENERGY_DUE_*): a regular one as any store's, a dip's always (into an
 * erased entry; none while its next page waits for an erase) */
static uint32_t life_admit(void)
{
    uint32_t m = flash_may(0u, 0u);
    if (m == FM_BRIGHT && life_room() < 2u) {
        m = 0;                                           /* brighter: the last erased entry stays for a dip write */
    }
    return life_pending() == 2u ? 0u : (FLASH_DIP_WRITES && !hw_vdd_low() ? ENERGY_DUE_DIP : 0u) |
                                       (m != 0u ? ENERGY_DUE_SAVE : 0u);   /* (r6: no dip write below the PVD either) */
}

/* every ms: the injected group's voltage pairs of that ms; the supply dips; the energy and the lifetime record */
static void vsense_service(void)
{
    uint32_t sum2[2], n, i, cut, lit = 0, hi = TJ_ACTIVE() || is.cal_step != 0u || ss.state == SS_RUN;
    hw_vsense_take(sum2, &n);
    vsense_add(&vs_sup, sum2[0], n);
    vsense_add(&vs_aux, sum2[1], n);
    i = vdip.in_dip;
    vdip_ms(&vdip, ms_now, up_s, sum2[0], n, is.i_ms);
    if (i == 1u && !vdip.in_dip) {         /* (0.6.9, B2) the fault memory: a dip ended, the last dip (0.6.10: not
                                              the planned cut's, in_dip 2) */
        const vdip_ev_t *d = vdip_event(&vdip, 0);
        if (d != 0 && d->ref_x != 0u) {
            en.lt.dip_pct = (uint8_t)((uint32_t)d->min_x * 100u / d->ref_x);
            en.lt.dip_cs = (uint8_t)(d->dur_ms / 10u > 255u ? 255u : d->dur_ms / 10u);
            en.lt.dip_a = (uint8_t)(d->i_ma / 100u > 255u ? 255u : d->i_ma / 100u);
        }
    }
    cut = PLAN_CUT(i);
    vdip_z(&vdip, &vs_sup, (uint32_t)is.i_ms);            /* the supply / cable impedance (0.6.5; i_ms >= 0) */
    for (i = 0; i < 5u; i++) {
        uint32_t o = out_now[i];
        lit |= o;
        hi |= o >= FLASH_LOW_LEVEL;
    }
    if (!hi && !ultra_on && sm.dur != 0u) {              /* a WS ramp to a brighter target: the same OR, its test hoisted
                                                            out of the loop (-29 cycles a ms) */
        for (i = 0; i < 5u; i++) {
            hi |= sm.to[i] >= FLASH_LOW_LEVEL;
        }
    }
    if (hi || ((ultra_on || fx_ch) && ultra_rising(&ul, tick_now))) {
        flash_hi_ms = ms_now;                            /* the flash: the outputs are not low (see above) */
    }
    energy_ms(&en, ms_now, vs_sup.mv, is.i_ms, (uint8_t)(lit != 0u));
#if MON_EXTRA
    energy_ch_ms(&en, is.applied, is.i_ms);              /* (0.6.5) energy per channel */
#endif
    n = ENERGY_DUE_NONE;
    if (en.due || vdip.count != en.dips || cut) {        /* (else energy_due answers NONE whatever it may admit, and
                                                            life_admit's flash_may / life_pending reads are saved) */
        n = energy_due(&en, ms_now, vdip.count, life_admit() | (cut ? ENERGY_PLANNED : 0u));
    }
    if (n != ENERGY_DUE_NONE) {
        flash_stall_begin(n == ENERGY_DUE_DIP ? FM_PROG : flash_may(0u, 0u));
        if (flash_stall_end(life_store(&en))) {
            energy_saved(&en, up_s);
        }
    }
}

/* once a second, in the main loop: read the conversion started a second ago, start the next one. Not while a current
 * calibration runs (its 30 ms windows read PA7 alone; the injected group would hold the PA7 results for 84 us). */
#define TEMP_HOT_C10      950               /* (0.6.9, B6) the warning: the die at 95.0 C */
#define TEMP_HOT_HYST_C10 100               /* ... until it is back under 85.0 C */
static uint8_t too_hot;
static uint8_t hl_seen;                    /* (0.6.9, B3) the HEALTH flags a second ago */

static void temp_service(void)
{
    uint32_t ts2 = 0, vref2 = 0;
    if (ms_now - temp_ms < 1000u) {
        return;
    }
    temp_ms = ms_now;
#if MON_EXTRA
    hw_dma_check();                        /* (0.6.5, D6) the DMA's transfer errors, once a second */
#endif
    if (temp_started && !hw_temp_read(&ts2, &vref2)) {
        ts2 = vref2 = 0;                   /* not done: counts as a bad sample */
    }
    {
        isense_cal_t c;
        uint8_t up;
        (void)isense_cal_get(&is, &c);
        health_run(&hl, &is, &c);          /* the running model and the dark current against the stored test */
        up = (uint8_t)(hl.flags & ~hl_seen & (ULTRA_HE_F_DARK | HEALTH_F_CAL));
        hl_seen = hl.flags;
        if (up != 0u) {                    /* (0.6.9, B3) D9's code when either rises: a FET stuck on, a test due */
            ack_start(&ack, ACK_P_FAULT, (up & ULTRA_HE_F_DARK) ? 6u : HEALTH_CODE_CAL_SUGGEST);
        }
    }
    if (temp_started) {
        uint8_t hot;
        mcutemp_sample(&mt, ts2, vref2);
        /* (0.6.9, B6) a warning only (nothing dims): the die over TEMP_HOT_C10 (its absolute offset varies per chip,
         * core/mcutemp.h), back under it by TEMP_HOT_HYST_C10: the event and D9's code once, the HEALTH flag and
         * STATUS's reason meanwhile */
        hot = (uint8_t)(mt.valid && mt.c10 >= TEMP_HOT_C10 - (too_hot ? TEMP_HOT_HYST_C10 : 0));
        if (hot && !too_hot) {
            ev_add(EVL_HOT, 0);
            ack_start(&ack, ACK_P_FAULT, HEALTH_CODE_HOT);
        }
        too_hot = hot;
        hl.flags = (uint8_t)((hl.flags & ~ULTRA_HE_F_HOT) | (hot ? ULTRA_HE_F_HOT : 0u));
        if (mt.valid && mt.c10 > temp_peak) {
            temp_peak = mt.c10;
            temp_peak_s = up_s;
        }
    }
    temp_started = (uint8_t)(is.cal_step == 0u);
    if (temp_started) {
        hw_temp_start();
    }
#if MON_EXTRA
    if (stagger_is_adaptive(eff.stagger) && isense_model_ok(&is)) {
        stagger_cap_second(&stat.cap, stat.ripple_ma, steps);   /* (0.6.5) C10's ripple load: the weights measured */
    }
#endif
}

/* Ultra CONFIG values (host-managed, RAM only) over the stored settings: the byte fields by the settings table (the
 * CONFIG id, the field, its range: config.h settings_desc; 0.6.6, review round 5: the ifs were ~130 B more), the
 * period / preset pair as code */
static void ultra_overlay(settings_t *e)
{
    const setdesc_t *t = settings_desc + SETD_OVL;
    const uint16_t *c = ul.cfg;
    uint32_t v = ul.cfg_valid, i;
#define HAS(id) ((v & (1u << (id))) != 0u)
    if (HAS(ULTRA_CFG_PWM_PERIOD) && c[ULTRA_CFG_PWM_PERIOD] >= PWM_STEPS_MIN) {
        e->pwm_preset = 0xFF;
        e->raw_steps = c[ULTRA_CFG_PWM_PERIOD];
    } else if (HAS(ULTRA_CFG_PWM_PRESET) && c[ULTRA_CFG_PWM_PRESET] < PWM_PRESET_COUNT) {
        e->pwm_preset = (uint8_t)c[ULTRA_CFG_PWM_PRESET];
    }
    for (i = 0; i < SETD_OVL_N; i++, t++) {
        uint32_t id = t->cfg, x = c[id];
        if (HAS(id) && x >= t->lo && x <= t->hi) {
            ((uint8_t *)e)[t->ofs] = (uint8_t)x;
        }
    }
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
    if (is.tripped != ISENSE_TRIP_NONE || TJ_ACTIVE()) {
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
    uint32_t sum, n, peak, ev = 0;
    if (vdip.in_dip || PLANNED) {
        ev = isense_cal_abort(&is, ISENSE_CAL_F_SUPPLY);    /* (0.6.9, A2) a supply dip: a test's currents aren't the
                                                               load's (none running: nothing); 0.6.10: the power-off
                                                               notice, a test started meanwhile too. Before isense_ms
                                                               (review round 8): its last ms would commit the model */
    }
    __disable_irq();
    isense_take(&is, &sum, &n, &peak);
    __enable_irq();
    ev |= PERF_CALL(PERF_ISENSE, isense_ms(&is, sum, n, peak));
#if BOARD_RETURN
    if (is.i_avg > (int32_t)rc_peak_ma) {
        rc_peak_ma = (uint16_t)(is.i_avg > 65535 ? 65535 : is.i_avg);   /* (0.6.9, ENERGY 17) for the hub: the
                                                                            port's PTCs between its polls */
    }
#endif
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
#if BOARD_RETURN
        cal_done_s = up_s + 1u;
#endif
        limit_update();                    /* the connected load is known now */
        (void)isense_cal_get(&is, &c);
        code = health_test_done(&hl, &c);
        ev_add(EVL_CAL, clim.connected_ma);   /* (0.6.5, D2) with the connected load it found */
        if (code != 0u) {
            ack_start(&ack, ACK_P_FAULT, code);   /* D9: the first abnormal channel, or 6 = current while dark */
        }
    }
    if (ev & ISENSE_EV_LATCH) {
        ev_add(EVL_LATCH, is.trip_cause == ISENSE_TRIP_FAST ? is.peak_ma : (uint32_t)(is.i_ms > 0 ? is.i_ms : 0));
    }                                      /* (0.6.9, B7) the retries used up: off until a new command or frame value */
    if (ev & ISENSE_EV_CAL_FAIL) {         /* (0.6.9, A2) the test stopped (is.cal_fail: why): the stored test and the
                                              model as before; logged with the current then, D9 code 8 */
        ev_add(EVL_CAL_FAIL, (uint32_t)(is.i_ms > 0 ? is.i_ms : 0));
        ack_start(&ack, ACK_P_FAULT, HEALTH_CODE_CAL_FAIL);
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
    is.retry_max = (uint8_t)(sext.reserved[2] ^ RETRY_N_X);   /* (0.6.9, B7) the stored retry policy */
    is.retry_s = (uint8_t)(sext.reserved[3] ^ RETRY_S_X);
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
        if (v[ch] >= FLASH_LOW_LEVEL) {
            flash_hi_ms = ms_now;           /* (review round 6) bright from now: a store deciding after this (every
                                               one decides before outputs_update) never sees a stale "low" */
        }
    }
    st_moved = 1;
}

static void outputs_update(uint8_t ack_on)
{
    uint16_t v[5];
    uint8_t i;
#if BOARD_TESTJIG
    if (tj.active) {
        tj_outputs(&tj, ms_now, v);        /* test jig: the fade over the five channels; data doesn't change it */
    } else
#endif
    if (ack.active && !ack.led_only && eff.feedback_ch != 2u) {   /* (0.6.9, A1) feedback off: D9 only, the outputs
                                                                    keep rendering */
        uint16_t lvl = ack_on ? 16384u : 0u;          /* 25 % */
        for (i = 0; i < 5u; i++) {
            v[i] = (eff.feedback_ch == 1u || (eff.feedback_ch == 0u && i == CH_R)) ? lvl : 0u;
        }
        if (PLANNED) {
            dark_apply(&dk, ms_now, v);    /* (0.6.10) the power-off notice's fade: over a pattern too (D9 goes on) */
        }
    } else {
        const uint16_t *o = ultra_on ? uout : sm.cur;   /* (the test out of the loop) */
        for (i = 0; i < 5u; i++) {
            v[i] = o[i];
        }
        scene_mix(&srun, ms_now, v);       /* the power-on scene until data comes, then the cross-fade */
        dark_apply(&dk, ms_now, v);        /* the pin's calibration: dark (a test still overrides it below) */
    }
    is.k_ss = TJ_ACTIVE() ? 65536u                                    /* soft start (not the test jig's pattern) */
                        : ss_factor(&ss, ms_now, (uint8_t)((v[0] | v[1] | v[2] | v[3] | v[4]) != 0u),
                                    (uint8_t)(sext.opts & STORE_OPT_SOFT_START));
    isense_apply(&is, v, v);               /* board limit (all channels scaled together), or the calibration pattern */
    PERF_DO(PERF_TABLES, build_all(v, built_ok));
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
    const uint16_t *o = ultra_on ? uout : sm.cur;
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        v[i] = o[i];
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
    scene_pend = (uint8_t)!scene_equal(&s, &scene_stored);
    scene_major_pend = 1;
    scene_changed_ms = ms_now - (SCENE_SAVE_MS - SAVE_DEBOUNCE_MS);
    return CFG_ACK;
}

/* (0.6.9, B7) the trips' retry policy (config frames 0x15 / 0x16, Ultra CONFIG 24): stored in the settings extension's
 * spare bytes, coded so that an erased byte (0xFF) is the default: 10 retries, 1 s apart */
static uint8_t retry_set(uint32_t count, uint32_t secs)
{
    uint8_t n = (uint8_t)(count ^ RETRY_N_X), t = (uint8_t)(secs ^ RETRY_S_X);
    if (n == sext.reserved[2] && t == sext.reserved[3]) {
        return 0;
    }
    sext.reserved[2] = n;
    sext.reserved[3] = t;
    is.retry_max = (uint8_t)count;
    is.retry_s = (uint8_t)secs;
    settings_changed();
    return 1;
}

/* soft start on / off (console, config frame, Ultra CONFIG 23): stored with the settings; 1 = changed */
static uint8_t soft_start_set(uint32_t on)
{
    uint8_t o = (uint8_t)(on ? sext.opts | STORE_OPT_SOFT_START : sext.opts & ~STORE_OPT_SOFT_START);
    if (o == sext.opts) {
        return 0;
    }
    sext.opts = o;
    settings_changed();
    return 1;
}

static cfg_result_t handle_result(cfg_result_t r)
{
    if (r.event == CFG_ACK && r.cmd == CMD_ISENSE_CAL) {
        if (!isense_calibrate(r.arg)) {
            r.event = CFG_NACK;            /* tripped: not now */
            ack_start(&ack, ACK_P_NACK, 0);
        } else {
            stat.config_frames++;
        }
        return r;
    }
    if (r.event == CFG_ACK && (r.cmd == CMD_ISENSE_LIMIT || r.cmd == CMD_RETRY_COUNT || r.cmd == CMD_RETRY_S ||
                               r.cmd == CMD_SOFT_START || r.cmd == CMD_SCENE || r.cmd == CMD_SCENE_FADE)) {
        /* the values main.c keeps (the settings extension; the scene, not a setting: no settings write): ACK when
         * it changed (config.c: the value code of SOFT_START / SCENE, else 0), SAME when not (one tail: 0.6.10) */
        if (r.cmd == CMD_ISENSE_LIMIT) {
            if (sext.ilimit50 == r.arg || (r.arg == 0u && sext.ilimit50 == 0xFFu)) {
                r.event = CFG_SAME;
            } else {
                sext.ilimit50 = r.arg;     /* stored with the settings */
                limit_update();
                settings_changed();
            }
        } else if (r.cmd == CMD_SOFT_START) {
            r.event = soft_start_set(r.arg) ? CFG_ACK : CFG_SAME;
        } else if (r.cmd == CMD_SCENE || r.cmd == CMD_SCENE_FADE) {
            r.event = scene_command(r.cmd, r.arg);
        } else {
            r.event = retry_set(r.cmd == CMD_RETRY_COUNT ? r.arg : is.retry_max,
                                r.cmd == CMD_RETRY_S ? r.arg : is.retry_s) ? CFG_ACK : CFG_SAME;
        }
        ack_start(&ack, r.event == CFG_ACK ? ACK_P_ACK : ACK_P_SAME, r.event == CFG_ACK ? r.value_code : 0u);
        stat.config_frames += r.event == CFG_ACK;
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
            /* (review round 6, F10: like a reboot) the stores it clears are void until their clears are stored: the
             * settings record (the defaults) carries it, written at once (reset_save; a program, as a user's
             * command); the boot doesn't load a void store (records_load) */
            STORE_VOIDS(&sext) = (uint8_t)~((scene_pend ? STORE_VOID_SCENE : 0u) | (fx_mask ? STORE_VOID_FX : 0u) |
                                            (cal_present ? STORE_VOID_CAL : 0u)
#if MON_EXTRA
                                            | (chist.n ? STORE_VOID_HIST : 0u)
#endif
                                            );
            reset_save = 1;
            /* and the power-on effects and the stored calibration (never the lifetime record): in the flash's next
             * low moment (0.6.4: an erase; 0.6.3 at once, the outputs as they were). The acknowledge pattern's final
             * dark second is usually that moment. */
            fx_clear_pend = 1;
            fx_mask = 0;
            /* (review round 6, fable-sys 04) the power-on effect and the stored scene stop showing: a reset board shows
             * nothing until data, as its flash now says (0.6.6 kept both running from RAM until a power cycle) */
            if (fx_ch) {
                ultra_scene_fx_stop(&ul);
                fx_ch = 0;
            }
            srun.hold = srun.fading = 0;
            outputs_dirty = 1;
            if (cal_present) {
                cal_save_pend = 2;         /* a cleared record (levels 0) */
                cal_present = 0;
            }
#if MON_EXTRA
            chist.n = 0;                   /* (M2) the calibration history begins anew (written in a low moment) */
            chist.tests = 0;
            chist_pend = 1;
#endif
            isense_setup();
        }
        apply_settings();
        settings_changed();
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

#if MON_EXTRA && CONSOLE_FULL
/* (0.6.7, D9) the console's `restore` (core/cli.c checked the length, CRC, version, board): the values as a console
 * change sets them. The settings: none out of range (settings_sanitize), in effect at once, saved after the debounce
 * through the store's budget and the flash policy (pending meanwhile); the calibration (when the backup holds one,
 * not while a test runs): as a finished test leaves it, stored through its budget. 1 = taken. */
static int restore_rec(const uint8_t *d)
{
    settings_t s;
    isense_cal_t c;
    memcpy(&s, d, sizeof s);
    memcpy(&c, d + sizeof s + sizeof sext, sizeof c);
    if (settings_sanitize(&s) ||
        (c.levels != 0u && (is.cal_step != 0u || (c.levels != 1u && c.levels != ISENSE_LEVELS) ||
                            (c.levels == 1u && c.pct[0] - 5u > 95u)))) {
        return 0;                          /* (the calibration: isense_cal_set's own conditions, it would ignore it) */
    }
    set = s;
    {
        uint8_t v = STORE_VOIDS(&sext);    /* (r6: this board's pending clears, not the backup's) */
        memcpy(&sext, d + sizeof s, sizeof sext);
        STORE_VOIDS(&sext) = v;
    }
    proto_set_mode(&frx.ps, (proto_mode_t)set.protocol);
    apply_settings();
    settings_changed();
    if (c.levels != 0u) {
        isense_cal_set(&is, &c);
        cal_present = 1;
        cal_save_pend = 1;
        limit_update();
    }
    return 1;
}
#endif

/* A flash operation (flash_may's answer `may`): the frame tracking stops meanwhile (the tick runs from SRAM: no frames,
 * the gate closed; downstream boards miss a frame or two, the next idle gap or frame end re-arms it), so a capture in
 * progress is dropped. The time bases run on (0.6.4; 0.6.3 added an estimate). */
static NOINLINE void track_stop(void)       /* (before and after it; out of line: -18 B) */
{
    __disable_irq();
    ftrack_disarm(&ft);
    cap_ready = 0;
    __enable_irq();
}

static void flash_stall_begin(uint32_t may)
{
    ovr_on = 0;                                          /* (the render ms it delays: not an overrun) */
#if BOARD_RETURN
    while (rc_state != RC_IDLE) {
        watchdog_kick();                                 /* (a dip's or the console's write) after the window */
    }
    rc_pend = 0;                                         /* this frame's end is lost: no window */
#endif
    if (may == FM_BRIGHT) {
        (void)budget_take(&flash_bright, ms_now, 1u);    /* the hour's write while brighter */
    }
    track_stop();
}

/* ... done with result r (core/flashrec.h): 1 = written */
static uint32_t flash_stall_end(uint32_t r)
{
    track_stop();
    ultra_clock_gap(&ul);
    flash_recheck = 1;                                   /* (whatever the result: a store's position may have moved) */
    if (r == FLASH_OK) {
        stat.saves++;
        if (!flash_held()) {
            flash_err = 0;                 /* (r6) a success ends it, but not a program (FM_PROG: a dip's, a factory
                                              reset's) inside the hold-off: the failing operation would come again */
        }
    } else if (r != FLASH_NONE) {
        flash_err = (uint8_t)(flash_err + (flash_err <= FLASH_RETRY_MAX));   /* (r6: the hold-off, doubling) */
        flash_err_ms = ms_now;
        say(Z(m_flash_err, "flash: error\r\n"));      /* (the record stays pending; its budget asks again) */
#if MON_EXTRA
        stat.fl_err = (uint8_t)r;                        /* (0.6.5, D5) */
        stat.fl_errs++;
#endif
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

#if CONSOLE_FULL
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
    cli_say(&cli, may == 0u ? ZC(m_pending, "pending\r\n") : settings_write(may) ? ZC(ok_crlf, "ok\r\n")
                                                          : ZC(m_failed, "failed\r\n"));
}
#endif

/* (r6, F10) a store voided by a factory reset (core/store.h STORE_VOIDS) is written: valid again, stored with the next
 * settings write */
static void void_done(uint8_t b)
{
    if (!(STORE_VOIDS(&sext) & b)) {
        STORE_VOIDS(&sext) |= b;
        settings_changed();
    }
}

static void scene_save_now(uint32_t may)
{
    flash_stall_begin(may);
    if (flash_stall_end(scene_store(&scene_want))) {
        scene_stored = scene_want;
        scene_pend = 0;
        void_done(STORE_VOID_SCENE);
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
        void_done(STORE_VOID_FX);
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
/* (round 9) inline, as LTO had it: after the hw.c split it left the standalone image's copy out of line (a call and
 * its frame every ms: tools/mainloop.py's ms pass 1,564 -> 1,602 cycles; the app images had it inlined either way,
 * and are byte-identical with the attribute) */
static ALWAYS_INLINE void scene_service(void)
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
        scene_pend = (uint8_t)!scene_equal(&sc, &scene_stored);
        scene_major_pend = (uint8_t)scene_major(&scene_stored, &sc);
        scene_changed_ms = scene_check_ms;
    }
    if (scene_pend && (!ack.active || ack.seg + 1u == ack.nseg) &&
        (may = flash_may(scene_pending() == 2u, 0u)) != 0u && scene_save_due(&sbudget, ms_now, scene_changed_ms, scene_major_pend)) {
        scene_save_now(may);
    }
}

static void handle_slot(const slot_t *slot, proto_t p)
{
    cfg_result_t r = config_process(&rx, &set, slot);
    uint16_t tgt[5];
    uint8_t i, ramp;
    uint32_t d;

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
    for (i = 0, d = 0; i < 5u; i++) {
        d |= (uint32_t)(tgt[i] ^ sm.to[i]);
    }
    if (d != 0u) {
        isense_rearm(&is);                 /* (0.6.9, B7) a new frame value: retry if the retries were used up */
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
    if (slot == 0) {
        return;
    }
    if (!ultra_seen) {
        /* the power-on scene holds at most SCENE_TAKEOVER_MS from this board's first slot (0.6.6: from the first
         * header, which ran out inside the hub's DARK preamble on long pins: 202 frames, ~3.5 s at 50 boards) */
        ultra_seen = 1;
        ultra_first_ms = ms_now;
    }
    ev = ultra_apply_slot(&ul, slot, hdr_tick);       /* frx_capture checked its CRC */
    if (ev & ULTRA_EV_STATE) {
        isense_rearm(&is);                 /* (0.6.9, B7) a new command: retry if the retries were used up */
    }
    if (ev & (ULTRA_EV_STATE | ULTRA_EV_DEF)) {
        /* the power-on scene fades over light by light: a light's channels once it has its state (SET) and its
         * definition (LIGHT_DEF, which a refresh may bring after the SET); an EFFECT runs on the state, and ESPHome
         * sends EFFECT none at its boot ahead of the restored state. The rest at their own state, or SCENE_TAKEOVER_MS.
         * (0.6.6 took over every channel at the first SET / EFFECT of any light: the lights whose LIGHT_DEF / SET were
         * still to come faded towards dark meanwhile: a five-light board on a 50-board pin, or one powered up while
         * the hub runs, went dark for seconds) */
        uint32_t l = (uint32_t)(slot[1] >> 4) % ULTRA_LIGHTS;
        int32_t later = 0;                     /* the SET's start, ms after this frame (the hub's lead): its cross-fade
                                                  starts there, not towards the light's state before it */
        if (slot[0] == ULTRA_SET) {
            ul_stated |= (uint8_t)(1u << l);
            later = (int16_t)ultra_get16(slot + ULTRA_PAYLOAD + 25u);
            later = later < 0 ? 0 : later;
        }
        if ((ul_stated & (1u << l)) && ul.light[l].defined && ul.light[l].mask != 0u) {
            ultra_light_t *L = &ul.light[l];
            if (L->scene_run) {                /* its power-on effect ends with its take-over (a host EFFECT ended it
                                                  already); the other lights' run on until theirs */
                L->scene_run = 0;
                L->fx = ULTRA_FX_NONE;
                L->pend = 0;
            }
            scene_takeover(&srun, L->mask, ms_now + (uint32_t)later);
        }
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
                settings_changed();
                if (v == PMODE_AUTO || v == PMODE_ULTRA) {
                    proto_set_mode(&frx.ps, (proto_mode_t)v);   /* Ultra keeps running; WS modes from the next power-up */
                }
            }
        }
        if ((ul.cfg_changed & (1u << ULTRA_CFG_SOFT_START)) && ul.cfg[ULTRA_CFG_SOFT_START] <= 1u) {
            (void)soft_start_set(ul.cfg[ULTRA_CFG_SOFT_START]);   /* persisted, like 22 */
        }
        if (ul.cfg_changed & (1u << ULTRA_CFG_RETRY)) {
            (void)retry_set(ul.cfg[ULTRA_CFG_RETRY] & 0xFFu, ul.cfg[ULTRA_CFG_RETRY] >> 8);   /* (0.6.9) persisted */
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
#if BOARD == BOARD_DP_RX
            if (!vdip.planned)                 /* (round 9 phase B) not during the power-off notice: dark to the cut */
#endif
            {
                dark_release(&dk, ms_now);
            }
        } else if (ul.identify == ULTRA_IDENT_VERSION) {
            ack_start_version(&ack, fw_version);
#if BOARD == BOARD_DP_RX
        } else if (ul.identify == ULTRA_IDENT_POWER_OFF) {   /* (0.6.10; ultra.c drops it on the dig2analog-ultra) */
            vdip.planned = 1;
            plan_ms = ms_now;
            if (!dk.active || (int32_t)(dk.until - ms_now) < (int32_t)PLAN_HOLD_MS) {
                dark_hold(&dk, ms_now, PLAN_HOLD_MS);   /* dark (0.3 s) before the cut; (round 9 phase B) a longer
                                                           hold (IDENTIFY 3) stays as it is */
            }
#endif
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
/* (0.6.9, Quindor) an update session's SAME / NACK on D9 only, whatever the feedback setting: a host repeats a session
 * (ESPHome's auto_retry) and a board that has it already, or can't take it, keeps its light. A real update's ACK (the
 * new version, upd_boot) keeps the strip pattern; the config / console SAME / NACK are unchanged. */
static void upd_ack_d9(ack_kind_t kind)
{
    ack_start(&ack, kind, 0);
    ack.led_only = 1;
}

static void upd_action(uint32_t a)
{
    switch (a) {
    case UPD_APP_TAKE:
#if UPD_APP
    {
        uint32_t t0 = ms_now;
        HO->request = UPD_REQ_ENTER;       /* the bootloader checks the ANNOUNCE again and installs it */
        upd_ho_seal(HO);
        cli_say(&cli, Z(m_update, "firmware update: into the bootloader\r\n"));
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
        upd_ack_d9(ACK_P_SAME);            /* the session's image is already installed */
        break;
    case UPD_APP_SHOW_REJECTED:
        upd_ack_d9(ACK_P_NACK);            /* not for this board, or older without FORCE */
        break;
    default:
        break;
    }
}

/* Decoded bytes live only through frx_capture / upd_app_frame. Both consume or copy
 * them before handle_slot / upd_action can build tables or program a flash record.
 * No ISR uses core_scratch. Keep the decoder's 16-byte memset alignment. */
typedef char decode_fits_scratch[(PROTO_BITS_BUF <= sizeof(core_scratch) &&
                                 UPD_FRAME_BYTES + 4u <= sizeof(core_scratch)) ? 1 : -1];

static void upd_capture(void)
{
    uint8_t *f = (uint8_t *)core_scratch;
    if (cap_bits < UPD_FRAME_BITS ||
        ws_decode_frame(din_copy, (uint32_t)cap_words * 16u, (uint16_t)UPD_FRAME_BITS, &ws_tm, f) != (int)UPD_FRAME_BITS) {
        return;                            /* the session repeats every block (passes) */
    }
    upd_action(upd_app_frame(&upd, f, ms_now));
}

#if UPD_APP
/* (0.6.8, Quindor: "option A") the bootloader's sectors 0-1 write-protected by the app itself, at its first boot behind
 * the bootloader (RELEASING.md "Production": no CLI flasher sets WRPR through a WCH-LinkE). Once per board, in the jig:
 * the sectors open, VDD above the PVD, the bootloader's info and the descriptor valid, the chip not read-protected.
 * The option bytes are rewritten with WRPR0 & 0xFC (bits 0, 1: 8 KB), RDPR (0xA5), USER, DATA and WRPR1-3 as they
 * were, then a reset loads them (RM 32.6: at a system reset). Never RDPR: never read protection. A power cut between
 * the option erase and the rewrite leaves RDPR erased (read-protected): the jig erases and programs again. VDD below
 * the PVD: the next boot. Before the outputs start (they are dark until data anyway); a failed write: the next boot */
/* (0.6.9, review round 7 R7-06) the option bytes come from the boot's snapshot (main, before hw_init: hw.c
 * hw_option_bytes reads them at a safe flash clock, each against its inverse, twice): one not to be trusted writes
 * nothing (the event, D9 code 9). The rewrite leaves its CRC in prot_chk (.noinit) for the boot after its reset,
 * which checks what the chip loaded: the sectors protected, RDPR 0xA5, every byte as written (the event either way;
 * not as written: D9 code 9) */
static uint16_t ob_snap[8];
static uint8_t ob_ok;
static uint32_t prot_chk __attribute__((section(".noinit.prot")));
#define PROT_MAGIC 0x50520000u                           /* "PR" << 16 | the written bytes' CRC-16 */

static void boot_protect(void)
{
    uint32_t chk = prot_chk;
    prot_chk = 0;
    if (stat.bc.magic == BC_MAGIC && (chk & 0xFFFF0000u) == PROT_MAGIC) {   /* (a reset, not a power-up: RAM kept) */
        chk = ob_ok && !hw_boot_open() && ob_snap[0] == 0xA5u &&
              (chk & 0xFFFFu) == crc16_ccitt((const uint8_t *)ob_snap, sizeof ob_snap);
        ev_add(chk ? EVL_PROT_OK : EVL_PROT_FAIL, 0);
        if (!chk) {
            ack_start(&ack, ACK_P_FAULT, HEALTH_CODE_PROTECT);
        }
        return;
    }
    if (!hw_boot_open() || hw_vdd_low() || !upd.info.desc_ok) {
        return;
    }
    if (!ob_ok) {
        ev_add(EVL_OB_BAD, 0);                           /* no erase on a snapshot not to be trusted */
        ack_start(&ack, ACK_P_FAULT, HEALTH_CODE_PROTECT);
        return;
    }
    if (ob_snap[0] != 0xA5u) {
        return;                                          /* read-protected: never touched */
    }
    ob_snap[4] &= 0xFCu;                                 /* WRPR0: sectors 0 and 1 */
    prot_chk = PROT_MAGIC | crc16_ccitt((const uint8_t *)ob_snap, sizeof ob_snap);
    if (flash_op(HW_OB_ADDR, ob_snap, 8u, HW_OB_ADDR) == FLASH_OK) {
        NVIC_SystemReset();
    }
    prot_chk = 0;
}
#endif

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
        case UPD_RES_SAME:                 /* (the bootloader's own check, after a TAKE: D9 only as above) */
            upd_ack_d9(ACK_P_SAME);
            break;
        case UPD_RES_REJECTED:
            upd_ack_d9(ACK_P_NACK);
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
    uint32_t a = upd_app_poll(&upd, ms_now);
    if (a != 0u) {
        upd_action(a);                     /* (0 = nothing: upd_action's default; the call every ms saved) */
    }
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

#if BOARD_RETURN
/* ------------------------------------------------------------------------------- the return channel's answers
 * Pages: core/ultra_protocol.h and the protocol wiki, Return channel (every page of 0.2.6). Built from live values
 * when the POLL's frame ends. */
#define RC_FEATURES (ULTRA_FEAT_SCENE | ULTRA_FEAT_DIM_TO_WARM | ULTRA_FEAT_DTW_POINTS | ULTRA_FEAT_CURRENT_SENSE | \
                     ULTRA_FEAT_CALIBRATION | ULTRA_FEAT_DARK_HOLD | (UPD_APP ? ULTRA_FEAT_FW_UPDATE : 0u) | \
                     ULTRA_FEAT_BOOT_PROTOCOL | ULTRA_FEAT_STAGGER_ADAPTIVE | ULTRA_FEAT_MCU_TEMP | \
                     ULTRA_FEAT_VERSION_BLINK | ULTRA_FEAT_POWER_EVENTS | ULTRA_FEAT_SUPPLY_DIPS | ULTRA_FEAT_HEALTH | \
                     ULTRA_FEAT_ENERGY | ULTRA_FEAT_SOFT_START | ULTRA_FEAT_SCENE_EFFECT | ULTRA_FEAT_LIFETIME | \
                     ULTRA_FEAT_DIAG | ULTRA_FEAT_RETRY | ULTRA_FEAT_LOSS_SCENE | ULTRA_FEAT_STANDBY | \
                     ULTRA_FEAT_LIFE_SYNC)
/* 0.2.4 hardware capabilities: answers (0x01), relays the boards behind it (0x02), the bus voltage (0x04), the input
 * current (0x08); no board NTC (0x10). (0.6.8: the proposal's 0x20, a second voltage input, is gone from 0.2.4: no
 * page carries its value) */
#define RC_HW_CAPS  (ULTRA_HWCAP_RETURN | ULTRA_HWCAP_RELAY | ULTRA_HWCAP_VBUS | ULTRA_HWCAP_CURRENT)
#define RC_ST_TEMP_BOARD 11u               /* 0.2.4 STATUS offsets (ULTRA_ST_* has 8, 9, 15) */
#define RC_ST_VBUS       13u
#define RC_ST_CURRENT    17u
#define RC_ST_MEASURED   19u
static uint16_t sat16(int32_t v);

/* CRC-16 over (id, value) of every CONFIG id the host set, in id order (STATUS 21, ACKS 21) */
static uint16_t rc_cfg_crc(void)
{
    uint8_t b[ULTRA_CFG_COUNT * 3u];
    uint32_t id, n = 0;
    for (id = 0; id < ULTRA_CFG_COUNT; id++) {
        if (ul.cfg_valid & (1u << id)) {
            b[n] = (uint8_t)id;
            ultra_put16(b + n + 1u, ul.cfg[id]);
            n += 3u;
        }
    }
    return crc16_ccitt(b, n);
}

static const uint8_t rc_u16[11] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };   /* (rc_pack: u16 fields) */

/* (0.6.9, protocol 0.2.5) STATUS 8 bits 4-7: why the light isn't as the host set it, the first that applies */
static uint32_t light_why(void)
{
    const vdip_ev_t *d = vdip_event(&vdip, 0);
    return is.tripped != ISENSE_TRIP_NONE ? (is.latched ? ULTRA_ST_WHY_RETRIES_OUT : is.tripped == ISENSE_TRIP_FAST ?
                                             ULTRA_ST_WHY_SHORT : ULTRA_ST_WHY_OVERLOAD)
         : TJ_ACTIVE() ? ULTRA_ST_WHY_TEST
         : is.cal_step != 0u ? ULTRA_ST_WHY_CALIBRATING
         : too_hot ? ULTRA_ST_WHY_HOT
         : (vdip.in_dip == 1u || (d != 0 && up_s - d->t_s < ULTRA_ST_DIP_RECENT_S)) ? ULTRA_ST_WHY_SUPPLY
         : signal_lost ? ULTRA_ST_WHY_SIGNAL_LOSS
         : ss.state == SS_RUN ? ULTRA_ST_WHY_SOFT_START
         : dk.active ? ULTRA_ST_WHY_DARK_HOLD
         : ack.active ? ULTRA_ST_WHY_PATTERN : ULTRA_ST_WHY_NONE;
}

/* the page's DATA into d (24 bytes, zero before); returns the PAGE to answer (ULTRA_PAGE_UNSUPPORTED: not here) */
static MEASURED uint8_t rc_page(uint8_t page, const uint8_t *arg, uint8_t *d)
{
    isense_cal_t c;
    uint32_t i, j, v[14];
    switch (page) {
    case ULTRA_PAGE_STATUS: {                            /* (0.2.4: 0-7, 11-14, 17-23; 8, 9, 15 before) */
        static const uint8_t fl[14] = { 3, 1, 3, 1, 1, 2, 2, 2, 2, 2, 1, 1, 2, 1 };
        v[0] = fw_version >> 8;                          /* major, minor, patch (0: a development build) */
        v[1] = ((stat.hw_id != 0u ? stat.hw_id : BOARD) << 4) | (stat.hw_rev & 0x0Fu);
        v[2] = up_s;
        v[3] = stat.reset_flags >> 26;                   /* RSTSCKR 26-31: pin, power, software, IWDG, WWDG, low power */
        v[4] = pwrev_state(&pw) | light_why() << ULTRA_ST_WHY_SHIFT;   /* ULTRA_ST_STATE (0.2.5: the reason) */
        v[5] = (uint16_t)mt.c10;                         /* ULTRA_ST_MCU_TEMP */
        v[6] = ULTRA_ST_NA;                              /* RC_ST_TEMP_BOARD: no sensor */
        v[7] = vs_sup.valid ? vs_sup.mv / 10u : 0xFFFFu; /* RC_ST_VBUS */
        v[8] = mt.vdda_mv;                               /* ULTRA_ST_VDDA */
        v[9] = sat16(is.i_avg);                          /* RC_ST_CURRENT */
        v[10] = (vs_sup.valid ? 0x01u : 0u) | (is.enabled ? 0x02u : 0u) | (is.calibrated ? 0x08u : 0u);
        v[11] = eff.pwm_preset;
        v[12] = rc_cfg_crc();
        v[13] = (frx.ps.mode << 4) | (eff.stagger & 0x0Fu);
        rc_pack(d, v, fl, 14);
        break;
    }
    case ULTRA_PAGE_OUTPUTS:
        for (i = 0; i < 5u; i++) {
            v[i] = out_now[i];
            v[5u + i] = sat16((int32_t)isense_ch_ma(&is, i, is.applied[i]));
        }
        rc_pack(d, v, rc_u16, 10);
        for (i = 0; i < ULTRA_LIGHTS; i++) {
            d[20] |= (uint8_t)((ul.light[i].fx != ULTRA_FX_NONE) << i);
            d[21] |= (uint8_t)((ul.light[i].defined != 0u) << i);
        }
        d[22] = srun.hold;
        d[23] = (uint8_t)((is.limiting ? 0x01u : 0u) | (is.tripped != ISENSE_TRIP_NONE ? 0x02u : 0u) |
                          (ul.ch_limited ? 0x04u : 0u) | (dk.active ? 0x08u : 0u) | (ack.active ? 0x10u : 0u) |
                          (ss.state == SS_RUN ? 0x20u : 0u) | (signal_lost ? 0x40u : 0u) |
                          ((TJ_ACTIVE() || is.cal_step != 0u) ? 0x80u : 0u));
        break;
    case ULTRA_PAGE_ACKS: {
        static const uint8_t def[3] = { ULTRA_LIGHT_DEF, ULTRA_SET, ULTRA_EFFECT };
        uint32_t never = 0, q;
        for (j = 0; j < 3u; j++) {
            uint32_t t = arg[j] != 0u ? arg[j] : def[j];
            for (i = 0; i <= ULTRA_LIGHTS; i++) {      /* lights 0-4, then the board (LIGHT 0xF) */
                q = t < ULTRA_TYPES ? ul.last_seq[i][t] : 0x100u;
                if (q > 0xFFu) {
                    never |= 1u << (3u * i + j);
                    q = 0;
                }
                d[3u * i + j] = (uint8_t)q;
            }
        }
        ultra_put24(d + 18, never);
        ultra_put16(d + 21, rc_cfg_crc());
        break;
    }
    case ULTRA_PAGE_ERRORS:
        for (j = 0, i = 1; i < 6u; i++) {
            j += stat.dec_err[i];
        }
        v[0] = frames;
        v[1] = stat.ultra_headers;
        v[2] = stat.ultra_tail_err;
        v[3] = stat.decoded;
        v[4] = j;
        v[5] = rc_windows;
        v[6] = rc_answers;
        v[7] = rc_missed;
        v[8] = rc_aborts;
        v[9] = stat.saves;
        v[10] = stack_used();
        rc_pack(d, v, rc_u16, 11);
        break;
    case ULTRA_PAGE_IDENTITY: {
        static const uint8_t fl[5] = { 4, 1, 1, 2, 4 };
        for (i = 0; i < 12u; i++) {
            d[i] = ((const uint8_t *)HW_UID)[i];       /* the unique id, in address order */
        }
        v[0] = fw_version;                               /* ULTRA_ID_FW_VERSION */
        v[1] = stat.bl_version;
        v[2] = stat.hw_id != 0u ? stat.hw_id : BOARD;
        v[3] = RC_HW_CAPS;
        v[4] = RC_FEATURES;                              /* ULTRA_ID_FEATURES */
        rc_pack(d + 12, v, fl, 5);
        break;
    }
    case ULTRA_PAGE_CALIBRATION: {
        static const uint8_t fl[13] = { 1, 1, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1 };
        v[0] = isense_cal_get(&is, &c) ? c.levels : 0u;
        v[1] = (is.calibrated ? 2u : isense_model_ok(&is) ? 1u : 0u) | (uint32_t)is.cal_fail << 4;   /* (0.2.5: the
                                                           last test's end) */
        v[2] = cal_cnt;
        v[3] = cal_done_s != 0u ? up_s + 1u - cal_done_s : 0xFFFFFFu;
        if (cal_done_s != 0u && v[3] > 0xFFFFFEu) {
            v[3] = 0xFFFFFEu;                            /* known age: saturate below the NONE sentinel */
        }
        v[4] = (uint16_t)(is.zero_valid ? is.zero_ma : 0);
        for (i = 0; i < 5u; i++) {
            v[5u + i] = sat16(is.ch_ma[i]);              /* CH1-CH5 at 100 % */
        }
        v[10] = clim.connected_ma > 65535u ? 65535u : clim.connected_ma;
        v[11] = clim.limit_ma;
        v[12] = (pwrev_src(clim.src) << 4) | clim.need;
        rc_pack(d, v, fl, 13);
        break;
    }
    case ULTRA_PAGE_CAL_TEST:
        if (arg[0] > 1u) {
            return ULTRA_PAGE_UNSUPPORTED;               /* two parts */
        }
        (void)isense_cal_get(&is, &c);                   /* (all 0 without a test) */
        for (i = 0; i < 12u; i++) {                      /* part 0: CH1-CH3, part 1: CH4-CH5 (8 values) */
            j = arg[0] * 12u + i;
            if (j < ISENSE_CH * ISENSE_LEVELS) {
                ultra_put16(d + 2u * i, c.ma[j / ISENSE_LEVELS][j % ISENSE_LEVELS]);
            }
        }
        if (arg[0] != 0u) {
            ultra_put16(d + 16, (uint16_t)c.zero_ma);
            for (i = 0; i < ISENSE_LEVELS; i++) {
                d[18u + i] = c.pct[i];
            }
            d[22] = c.levels;
        }
        break;
    case ULTRA_PAGE_POWER:
        pwrev_page(&pw, d);
        break;
    case ULTRA_PAGE_SUPPLY:
        vdip_page(&vdip, &vs_sup, up_s, vs_sup.valid ? vs_sup.mv : 0u, arg[0], d);
        break;
    case ULTRA_PAGE_HEALTH:
        health_page(&hl, temp_peak, up_s - temp_peak_s, d);
        break;
    case ULTRA_PAGE_ENERGY:
        energy_page(&en, up_s, vs_sup.mv / 10u * (uint32_t)(is.i_avg > 0 ? is.i_avg : 0) / 100u, d);
        ultra_put16(d + ULTRA_EN_VB, vs_aux.valid ? vs_aux.mv / 10u : 0xFFFFu);   /* (0.2.5) after the buck */
        ultra_put16(d + ULTRA_EN_PEAK_MA, rc_peak_ma);
        rc_peak_ma = 0;
        break;
    case ULTRA_PAGE_LIFE_SYNC:                           /* (0.6.10, protocol 0.2.6, ULTRA_LS_*: merged in rc_arm) */
    case ULTRA_PAGE_LIFETIME: {                          /* (0.2.5, ULTRA_LT_*) the record as it is now */
        static const uint8_t fl[11] = { 3, 3, 3, 2, 2, 3, 1, 1, 1, 2, 3 };
        static const uint8_t fs[9] = { 3, 3, 3, 2, 2, 3, 4, 2, 2 };   /* (LIFE_SYNC: Wh, result + applied, records) */
        const life_rec_t *r = &en.lt;
        v[0] = r->on_s / 360u;                           /* 0.1 h */
        v[1] = r->lit_s / 360u;
        v[2] = r->ups;
        v[3] = r->trips[0];                              /* overload, short */
        v[4] = r->trips[1];
        v[5] = r->dips;
        v[6] = r->peak_t;
        v[7] = r->peak_a;
        v[8] = r->wd;
        v[9] = r->updates;
        v[10] = r->writes;
        if (page == ULTRA_PAGE_LIFE_SYNC) {              /* (bytes 16.. its own: ULTRA_LS_A_*) */
            v[6] = energy_total_wh(&en);
            v[7] = ls_res;                               /* (the result, then the applied fields) */
            v[8] = r->writes;
        }
        rc_pack(d, v, page == ULTRA_PAGE_LIFE_SYNC ? fs : fl, page == ULTRA_PAGE_LIFE_SYNC ? 9u : 11u);
        break;
    }
    case ULTRA_PAGE_DIAG:                                /* (0.2.5, ULTRA_DG_*) two parts */
        if (arg[0] > 1u) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        if (arg[0] == 0u) {                              /* the running board */
            static const uint8_t fl[13] = { 1, 1, 2, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1 };
            v[0] = stat.reset_flags >> 26;
            v[1] = stat.bc.magic == BC_MAGIC ? stat.bc.cause : 0xFFu;
            v[2] = stat.bc.pc;
            v[3] = stat.bc.up_s > 0xFFFFFFu ? 0xFFFFFFu : stat.bc.up_s;
            v[4] = stat.cpu_pm;
            v[5] = cpu_peak;
            for (i = 0; i < 3u; i++) {
                j = ovr[i];
                v[6u + i] = j > 0xFFFFu ? 0xFFFFu : j;
            }
            j = vdip_z_mohm(&vdip);
            v[9] = j > 0xFFFFu ? 0xFFFFu : j;
            v[10] = flash_err;
            v[11] = settings_dirty | (cal_save_pend != 0u) << 1 | (uint32_t)scene_pend << 2 | (uint32_t)en.due << 3 |
                    (fx_pend == 2u || fx_clear_pend) << 4 | (uint32_t)en.held << 7;   /* (0.6.10, R5: EXHAUSTED) */
            v[12] = save_why();
            rc_pack(d, v, fl, 13);
            d[22] = d[23] = 0xFFu;                       /* the C10 load: not measured (MON_EXTRA 0) */
        } else {                                         /* the fault memory (core/energy.h), the calibration trend */
            static const uint8_t fl[7] = { 1, 2, 1, 1, 1, 1, 1 };
            const life_rec_t *r = &en.lt;
            v[0] = r->trip;
            v[1] = r->trip_h;
            v[2] = r->dip_pct;
            v[3] = r->dip_cs;
            v[4] = r->dip_a;
            v[5] = r->rst;
            v[6] = r->rst_h;
            rc_pack(d, v, fl, 7);
            for (i = 8; i < 13u; i++) {
                d[i] = 0x80u;                            /* the trend: not known (MON_EXTRA 0: no history) */
            }
            d[13] = d[14] = 0xFFu;
        }
        break;
    default:
        return ULTRA_PAGE_UNSUPPORTED;                   /* FW_UPDATE (reserved), pages this firmware doesn't know */
    }
    return page;
}

/* the frame of a POLL ended (main loop, at once: the window's deadline is ULTRA_RW_DIR_MAX_US after t0). d: samples
 * from t0 (the frame's last falling edge) to the end of the tail copy, -1 = none; odd: the copy's end is
 * TAIL_PAST - odd words past the ring position read at end_tick. Takes part only in an RW frame, with the clock's
 * rate measured (unrated, a 2 % HSI error would move slot 15 by 160 us), and while it can still turn around in time. */
static MEASURED void rc_arm(const frx_end_t *e, uint32_t end_tick, int32_t d, uint32_t odd)
{
    const uint8_t *poll = frx.slot;        /* (rc_poll_parse at the capture; again here: a later capture may have
                                              replaced it when this frame end comes that late) */
    uint32_t i;
    uint8_t page, rf;
    rc_pend = 0;
    if (d < 0 || !e->ultra_sync || (e->flags & (ULTRA_HDR_RW | ULTRA_HDR_EXT)) != ULTRA_HDR_RW ||
        rc_poll_parse(poll) != 0) {
        return;                            /* no window (a POLL is valid only in an RW frame; review r6: and not with
                                              EXT, "must understand": a board ignores every slot of such a frame) */
    }
    if (!ul.clk.rated) {
        rc_missed++;
        return;
    }
    rc_schedule(&rc_s, end_tick, d - (int32_t)(TAIL_PAST - odd) * 16, ul.clk.rate_q16, poll);
    rc_send = 0;
    if (poll[RC_P_INDEX] != ULTRA_POLL_RELAY_ONLY) {     /* (relay only: the page and its arguments are ignored) */
        for (i = 0; i < RC_ANSWER_BYTES; i++) {
            rc_ans[i] = 0;
        }
        if (poll[RC_P_PAGE] == ULTRA_PAGE_POWER) {
            pwrev_ack(&pw, poll + RC_P_ARG);             /* the answer shows the state after it */
        }
        if (poll[RC_P_PAGE] == ULTRA_PAGE_LIFE_SYNC) {
            ls_res = (uint16_t)energy_sync(&en, poll + RC_P_ARG, rc_uid16);   /* (0.6.10; the same) */
        }
        page = rc_page(poll[RC_P_PAGE], poll + RC_P_ARG, rc_ans + ULTRA_RSP_DATA);
        rf = (uint8_t)((pwrev_latched(&pw) ? ULTRA_RSP_EVENT : 0u) | (rc_contention ? ULTRA_RSP_CONTENTION : 0u));
        rc_answer(rc_ans, poll, page, rf, rc_uid16);
        rc_send = 1;
    }
    rc_brr_v = rc_brr(ul.clk.rate_q16, poll[RC_P_RATE]);
    __disable_irq();                       /* (review r6, T14) no tick between the late check and ARMED */
    if ((int32_t)(tick_now + 2u - rc_s.late_tick) > 0) {
        __enable_irq();
        rc_missed++;                       /* a long main-loop pass: it would turn around after ULTRA_RW_DIR_MAX_US */
        return;
    }
    BARRIER();                             /* the schedule and the answer before the ISR sees ARMED */
    rc_state = RC_ARMED;
    __enable_irq();
    rc_windows++;
}
#endif

static void process_capture(void)
{
    uint8_t *bits = (uint8_t *)core_scratch;
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
    n = PERF_CALL(PERF_DECODE, ws_decode_frame(din_copy, (uint32_t)cap_words * 16u, (uint16_t)want, &ws_tm, bits));
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
#if BOARD_RETURN
    if (frx.have_ultra && rc_poll_parse(frx.slot) == 0) {
        rc_pend = 1;                       /* a POLL: a window may follow this frame's end (rc_arm) */
        rc_frame = frx.cap_frame;
    }
#endif
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
#if BOARD_RETURN
    int32_t rc_d = -1;
    uint32_t rc_odd = 0;
#endif
    BARRIER();                             /* tail_copy: read after fe_ready */
    if (tail_words != 0u &&
        PERF_CALL(PERF_HEADER, ws_decode_last(tail_copy, (uint32_t)tail_words * 16u, &ws_tm, tail)) >= (int)ULTRA_HDR_BITS) {
        t8 = tail;                         /* the frame's last 64 bits: an Ultra header? */
    }
#if BOARD_RETURN
    if (rc_pend && fe_frame == rc_frame && tail_words != 0u) {
        rc_d = rc_last_edge(tail_copy, tail_words);   /* t0: the frame's last edge, from the end of the tail copy */
        rc_odd = rc_fe_pos & 1u;
    }
#endif
    frx_frame_end(&frx, fe_frame, total, t8, &e);
    BARRIER();
    fe_ready = 0;
#if BOARD_RETURN
    if (rc_pend) {
        rc_arm(&e, end_tick, rc_d, rc_odd);   /* first: its deadline is ULTRA_RW_DIR_MAX_US after the edge */
    }
#endif
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
        settings_changed();
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
static NOINLINE void stagger_adapt_run(void)
{
    uint16_t peak[5];
    stagger_result_t r;
    uint8_t i, changed = st_force;
    st_moved = 0;
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
    PERF_DO(PERF_STAGGER, stagger_choose_max(out_now, peak, stagger_gap_q16(steps), st_o2, st_o3,
                       (uint16_t)(stagger_is_spread(eff.stagger) ? STAGGER_SPREAD_OMAX : 0u), &r));
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

/* every ms (the test here, out of the search's big frame; the compare of the outputs only after they changed) */
static void stagger_adapt(void)
{
#if BOARD_RETURN
    if (RC_BUSY()) {
        return;                            /* (~0.5 ms: the window's deadline) */
    }
#endif
    if (stagger_is_adaptive(eff.stagger) && ms_now - st_ms >= 50u && (st_moved || st_force)) {
        stagger_adapt_run();
    }
}

/* the stored power-on effects (fx_mask) start in the engine (power-up, signal loss); returns their channels */
static uint8_t fx_start(void)
{
    uint8_t ch, fx = 0;
    ultra_scene_fx_start(&ul, fx_mask, tick_now);
    for (ch = 0; ch < ULTRA_LIGHTS; ch++) {
        if (fx_mask & (1u << ch)) {
            fx |= ul.light[ch].mask;
        }
    }
    return fx;
}

/* (0.6.9, B10) signal loss "scene": the power-on scene and effects as at power-up, faded in from the outputs now; the
 * host's next slot starts the take-over (per light at its SET, or SCENE_TAKEOVER_MS after it). 0: nothing stored */
static uint8_t loss_scene(void)
{
    if ((scene_want.mask & 0x1Fu) == 0u && fx_mask == 0u) {
        return 0;                          /* (the scene as set: the stored one, or one to be stored at a low moment) */
    }
    fx_ch = fx_start();
    scene_return(&srun, &scene_want, fx_ch, out_now, ms_now);
    ultra_seen = 0;
    return 1;
}

static void signal_loss_check(void)
{
    static const uint16_t off[5] = { 0, 0, 0, 0, 0 };
    uint32_t timeout = LOSS_TIMEOUT_MS;
#if BOARD == BOARD_DP_RX
    if (vdip.planned && ms_now - plan_ms >= PLAN_HOLD_MS) {
        vdip.planned = 0;                  /* (0.6.10) still powered: the notice forgotten (a dip it began stays the
                                              planned cut's) */
    }
    if (grace && (last_data_ms != grace_ms || ms_now - grace_ms > GRACE_MS)) {
        grace = 0;                         /* (0.6.10) a valid frame first (Ultra or WS): its state, no scene this
                                              power-up; none: the power-on scene and effects, faded in (B10's) */
        if (last_data_ms == grace_ms) {
            outputs_dirty |= loss_scene();
            signal_lost |= eff.signal_loss == 3u;   /* (round 9 phase B) mode 3's loss scene is this one: the 2 s
                                                        loss path doesn't start it (and its effects) again */
        }
    }
#endif
    if (ultra_on) {
        timeout = (uint32_t)ul.cfg[ULTRA_CFG_LOSS_TIMEOUT] * 100u;
        if (timeout < 500u) {
            timeout = 500u;
        }
    }
    if (ms_now - last_data_ms < timeout) {
        return;
    }
    if (eff.signal_loss == 3u && !signal_lost) {
        signal_lost = 1;
        outputs_dirty = loss_scene();      /* (0.6.9, B10) the power-on scene / effect; nothing stored: hold */
    }
    if (eff.signal_loss == 0u || eff.signal_loss == 3u) {
        /* mode 0 = hold (Ultra: effects keep running), like a WS2805. The host's SET applies again when it is back
         * (every ms of the gap: no slot comes meanwhile): its SEQ may have gone round by whole 256s (a streamed
         * effect, 4.4 s at 58 frames a second), and an 8-bit repeat would leave the light in its old state until the
         * next change. An unchanged SET's re-apply shows nothing. */
        uint32_t i;
        for (i = 0; ultra_on && i < ULTRA_LIGHTS; i++) {
            ul.last_seq[i][ULTRA_SET] = 0x100u;
        }
        return;
    }
    if (signal_lost) {
        return;
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
    uint32_t k;
    in.limit_ma = clim.limit_ma;
    in.limit_src = pwrev_src(clim.src);
    in.ch_limiting = (uint8_t)(ultra_on && ul.ch_limited);
    in.trip_cause = is.tripped;            /* ISENSE_TRIP_FAST / _OVL = ULTRA_PW_CAUSE_SHORT / _OVERLOAD */
    in.limiting = (uint8_t)(is.limiting && in.trip_cause == ISENSE_TRIP_NONE);
    in.i_ma = sat16(is.i_ms);
    in.trip_ma = in.trip_cause == ISENSE_TRIP_FAST ? is.peak_ma : in.i_ma;
    in.factor = (uint16_t)(is.k >= 65535u ? 65535u : is.k);
    in.retry_ms = isense_retry_left_ms(&is);
    in.dips = vdip.count;                  /* (0.6.10: the planned cut's isn't counted, core/vdip.h) */
    pwrev_update(&pw, ms_now, &in);
    for (k = 0, lv = pw.ev; lv != 0u; k++, lv >>= 1) {   /* (0.6.5, D2) the events it counted: limit, overload,
                                                             short, dip */
        if (lv & 1u) {
            ev_add(EVL_LIMIT + k, in.trip_ma);   /* (the board current; a short's: its peak = pwrev's) */
        }
    }
    if (pw.ev & (ULTRA_PW_EV_OVERLOAD | ULTRA_PW_EV_SHORT)) {   /* (0.6.9, B2) the fault memory: the last trip */
        uint32_t a = in.trip_ma / 500u;
        en.lt.trip = (uint8_t)((pw.ev & ULTRA_PW_EV_SHORT ? 1u : 2u) | (a > 63u ? 63u : a) << 2);
        en.lt.trip_h = (uint16_t)(en.lt.lit_s / 3600u > 65535u ? 65535u : en.lt.lit_s / 3600u);
    }
    ocled_update(&ocl, ms_now, up_s, in.limiting, in.trip_cause);
    lv = ocled_led(&ocl, ms_now, ack.active, ack_on);
#if BOARD_TESTJIG
    if ((tj.active || tj.rst >= TJ_RST_ARMED) && ocl.show == OCLED_OFF && !ack.active) {
        lv = tj_led(&tj, ms_now);          /* test jig: on, off at each cycle start; the reset gesture armed: fast
                                              (overcurrent / acknowledge win) */
    }
#endif
    if (lv != led_now) {
        led_now = lv;
        led_set(lv);
    }
}

#if BOARD_TESTJIG
/* every ms: the test jig's pad (debounced), the TP9 reset gesture, and in test mode the pattern (every ms) and its
 * current report */
static void testjig_service(void)
{
#if CONSOLE_FULL
    uint8_t ev = tj_pad(&tj, ms_now, tj_pads(&tj, HW_TP6_LOW(), uart_rx_pos(), HW_TP9_LOW()));
#else
    uint8_t ev = tj_pad(&tj, ms_now, tj_pads(&tj, HW_TP6_LOW(), 0u, HW_TP9_LOW()));   /* (no RX: TP6 a GPIO alone) */
#endif
    if (ev & TJ_EV_MODE) {
        say(tj.active ? ZJ(m_tj_on, "test mode: on\r\n") : ZJ(m_tj_off, "test mode: off\r\n"));
        outputs_dirty = 1;
#if !CONSOLE_FULL
        cli.st_part = 0;                   /* (the release console: the stream pauses, a status in progress ends) */
#endif
    }
    if (ev & TJ_EV_RESET) {
        say(ZJ(m_tj_reset, "factory reset (jig pad)\r\n"));
        (void)cli_apply(CMD_FACTORY_RESET, 0xA5u);     /* as the console's `defaults` */
    }
    if (!tj.active) {
        return;
    }
    outputs_dirty = 1;
    tj_rep |= (uint8_t)tj_measure(&tj, ms_now, is.i_ms, is.zero_ma);
    if (tj_rep && uart_room() >= CLI_PART_ROOM) {   /* (0.6.6: when the UART takes it whole; the cycle's values stay
                                                       until the next one, 1-2 s) */
        tj_rep = 0;
        cli_test_report(&cli, tj.ma, tj.fail, tj.cycles, vs_sup.valid ? vs_sup.mv : 0u);
    }
}
#endif

/* CPU load (core/cpuload.h, 0.6.5 D3): the cycles asleep in the idle WFI since cpu_t0 (tick_now then), the busiest
 * second since the last status (permille). Once a second: the second that ended into stat.cpu_pm. */
#define POLL_CYC (144000000u / POLL_HZ)   /* HCLK cycles per poll tick: SysTick's period (hw.c) */
static uint32_t cpu_idle, cpu_t0;

static void cpu_second(void)
{
    uint32_t t = tick_now, pm = cpu_permille(cpu_idle, t - cpu_t0, POLL_CYC);
    cpu_idle = 0;
    cpu_t0 = t;
    stat.cpu_pm = (uint16_t)pm;
    if (pm > cpu_peak) {
        cpu_peak = (uint16_t)pm;
    }
}

static inline __attribute__((always_inline)) void status_snapshot(void)
{
    uint8_t i;
    /* the live line's values (cli_watch) */
    stat.uptime_s = up_s;
    stat.proto_locked = (uint8_t)proto_active(&frx.ps);
    for (i = 0; i < 5u; i++) {
        stat.out[i] = (ultra_on ? uout : sm.cur)[i];
        stat.i_now_ch_ma[i] = (int32_t)isense_ch_ma(&is, i, is.applied[i]);
    }
    stat.i_ma = is.i_avg;
    stat.i_k_permille = (is.k * 1000u) >> 16;
    stat.mcu_temp_c10 = mt.c10;
    stat.vs_supply_mv = vs_sup.valid ? vs_sup.mv : 0u;
#if CONSOLE_FULL || STREAM_STATUS                /* ... and the full status's (the receiver's release console has none) */
    stat.stack_max = stack_used();
    stat.proto_mode = (uint8_t)frx.ps.mode;
    stat.proto_provisional = frx.ps.provisional;
    stat.frames = frames;
    stat.steps = steps;
    stat.ilimit50 = sext.ilimit50 == 0xFFu ? 0u : sext.ilimit50;
    stat.softstart = (uint8_t)(sext.opts & STORE_OPT_SOFT_START);
    {
        stat.i_peak_ma = is.peak_ma;
        stat.i_zero_ma = is.zero_valid ? is.zero_ma : 0;
        stat.i_limit_ma = is.limit_ma;
        stat.i_trips = is.trips;
        stat.i_tripped = (uint8_t)(is.tripped != ISENSE_TRIP_NONE);
        stat.i_trip_cause = is.trip_cause != ISENSE_TRIP_NONE ? is.trip_cause : stat.i_trip_cause;
        stat.i_cal = (uint8_t)(is.cal_step != 0u);
        stat.cal_fail = is.cal_fail;
        stat.retry_n = is.retry_max;
        stat.retry_s = is.retry_s;
        stat.save_why = save_why();
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
    stat.vdda_mv = mt.vdda_mv;
    stat.mcu_peak_c10 = temp_peak;
    stat.mcu_peak_s = temp_peak_s;
    stat.test_mode = (uint8_t)TJ_ACTIVE();
    /* Extrema are consumed only by status_begin, never by watch or keystrokes. */
    stat.vs_aux_mv = vs_aux.valid ? vs_aux.mv : 0u;
    stat.dip_ref_mv = vsense_mv(&vs_sup, vdip.ref_q12 >> 8, 16u);   /* Q12 as 16 pairs */
    stat.oc_show = ocl.show;
    stat.oc_cause = ocl.last_cause;
    stat.oc_limiting = ocl.limiting;
    stat.oc_tripped = (uint8_t)(ocl.trip_cause != 0u);
    stat.oc_since_s = ocl.since_s;
    stat.oc_hold_ms = ocled_hold_left(&ocl, ms_now);
#if MON_EXTRA
    stat.fl_pend = (uint8_t)(settings_dirty | (cal_save_pend != 0u) << 1 |   /* (0.6.5, D5) */
                             (uint32_t)scene_pend << 2 | (fx_pend == 2u || fx_clear_pend) << 4);
#endif
#endif
}

#if CONSOLE_FULL || STREAM_STATUS                /* (0.6.9: the receiver's release console has no status) */
static void status_begin(void)
{
    stat.cpu_peak = cpu_peak;              /* the busiest second since the last status; again from the last one */
    cpu_peak = stat.cpu_pm;
    vsense_report(&vs_sup, &stat.vs_supply_min_mv, &stat.vs_supply_max_mv);
}
#endif

/* (0.6.7) the records at power-up: the settings, the power-on scene, the engine with the stored power-on effects, the
 * calibration (into core_scratch until isense_setup: returns cal_load's result, 0 = one loaded), the calibration
 * history. (review round 6, F10) A factory reset whose clears aren't all stored yet (core/store.h STORE_VOIDS) voids
 * the stores it cleared: their records don't load and their clears are due again, so a power cut at any moment after a
 * reset never brings back what it cleared. */
static int records_load(void)
{
    uint8_t v;
    int cal;
    if (settings_load(&set, &sext) != 0) {
        settings_default(&set);            /* (the extension is cleared too) */
    } else {
        (void)settings_sanitize(&set);     /* a record of another build: out-of-range fields take their defaults */
    }
    v = (uint8_t)~STORE_VOIDS(&sext);
    if ((v & STORE_VOID_SCENE) || scene_load(&scene_stored) != 0) {
        scene_clear(&scene_stored);        /* none stored: dark until data comes (as before) */
    }
    scene_want = scene_stored;
    scene_pend = scene_major_pend = (uint8_t)((v & STORE_VOID_SCENE) != 0u);   /* (void: its clear again) */
    scene_boot(&srun, &scene_stored);
    ultra_init(&ul, hw_uid_hash());        /* (a seed per board: the power-on effects' randomness) */
    fx_mask = scene_fx_load(&ul, fx_crc);  /* the power-on effects: from here on (the bank's position: always) */
    if (v & STORE_VOID_FX) {
        ultra_init(&ul, hw_uid_hash());    /* (void: not into the engine; its clear again) */
        fx_mask = 0;
        fx_clear_pend = 1;
    }
    cal = cal_load((isense_cal_t *)core_scratch, &cal_cnt);
    if (v & STORE_VOID_CAL) {
        cal = -1;
        cal_save_pend = 2;                 /* (void: a cleared record again) */
    }
#if MON_EXTRA
    (void)calhist_load(&chist);            /* (M2; none: an empty history) */
    if (v & STORE_VOID_HIST) {
        chist.n = 0;
        chist.tests = 0;
        chist_pend = 1;
    }
#endif
    return cal;
}

int main(void)
{
    uint32_t last_ms = 0, fps_ms = ms_now, fps_frames = 0, watch_t = 0, may, life_found, room;
    uint8_t ack_on = 0, was_active = 0;
    int ch, cal_ok;
#if CONSOLE_FULL
    int con_ch = -1;
#endif

    stack_paint();                         /* the high-water mark in `status` */
    evlog_boot(&evl, bcrumb.magic == BC_MAGIC, bcrumb.up_s);   /* (0.6.9, B2) the last run's events kept, or anew */
    stat.bc = bcrumb;                      /* (0.6.5, D1) the last run's breadcrumbs (before the tick runs; cli.c shows
                                              them unless the reset flags say power-on), then this run's */
    bcrumb.magic = BC_MAGIC;
    bcrumb.up_s = bcrumb.cause = 0;    /* (pc: the tick writes it every ms) */
    /* (0.6.5) every record read in one window, before hw_init (nothing runs on the clocks yet): HCLK / 2 for the upper
     * CodeFlash (hw.c hw_records_window); the values stay in RAM from here on */
    hw_records_window(1);
    cal_ok = records_load();
    life_found = life_load(&en.lt);        /* (energy_init takes it from en.lt below) */
    hw_records_window(0);
#if UPD_APP
    ob_ok = (uint8_t)hw_option_bytes(ob_snap);   /* (0.6.9, R7-06) at a safe flash clock: before hw_init (hw.c) */
#endif
#if MON_EXTRA
    stat.fl_rej = flash_rejected() | (uint32_t)ultra_fx_rejected << 24;   /* (0.6.5, D5) the records the loaders
                                              rejected, the effect heads the store lists that did not load */
#endif
#if BOARD == BOARD_DP_RX
    srun.sc.mask = srun.hold = 0;          /* (0.6.10) dark: the grace window (main loop) */
    grace_ms = last_data_ms = ms_now - 1u;
#else
    fx_ch = fx_start();                    /* (the power-on effects) */
    srun.sc.mask |= fx_ch;
    srun.hold |= fx_ch;
#endif
    budget_init(&fx_budget, ms_now, FX_BUDGET_CAP, FX_REFILL_MS);   /* (the time bases: any start value, any wrap) */
    budget_init(&set_budget, ms_now, STORE_BUDGET_CAP, STORE_REFILL_MS);
    budget_init(&cal_budget, ms_now, CAL_BUDGET_CAP, CAL_REFILL_MS);
#if MON_EXTRA
    budget_init(&chist_budget, ms_now, CAL_BUDGET_CAP, CAL_REFILL_MS);   /* (review round 6) */
#endif
    dark_init(&dk);                        /* (ocl, sm: zero in .bss at boot, as their inits leave them) */
#if BOARD_TESTJIG
    tj_init(&tj, TJ_CYCLE_MS);
#endif
    pwrev_init(&pw, ms_now);
    scene_budget_init(&sbudget, ms_now);
    config_rx_init(&rx);
    frx_init(&frx, (proto_mode_t)set.protocol, (proto_t)set.last_proto);
    hw_init();
#if BOARD_RETURN
    rc_uid16 = crc16_ccitt((const uint8_t *)HW_UID, 12);   /* the unique id, in address order */
#endif
    mcutemp_init(&mt);
    health_init(&hl);
    ss_init(&ss, hw_uid_hash());
    vsense_init(&vs_sup, VSENSE_SUPPLY_RT, VSENSE_SUPPLY_RB, VSENSE_VREF_MV);
    vsense_init(&vs_aux, VSENSE_AUX_RT, VSENSE_AUX_RB, VSENSE_VREF_MV);
    hw_adc_init();                         /* PA7 continuously, the injected group from the poll tick */
    isense_setup();
    if (cal_ok == 0) {
        isense_cal_set(&is, (const isense_cal_t *)core_scratch);   /* the last test with its curves wins */
        cal_present = 1;
    }
    stat.reset_flags = hw_reset_flags();
    upd_boot();                            /* UPD_APP: the reset flags from the hand-over, the bootloader's result */
#if UPD_APP
    boot_protect();                        /* (0.6.8) the bootloader's sectors write-protected (once, in the jig) */
#endif
#if UPD_APP
    fw_version = upd.info.desc_ok ? upd.info.version : 0u;   /* the descriptor's: what the boards compare in an update */
#else
    fw_version = fwver_parse(FW_VERSION);
#endif
    fwver_format(fw_version, fw_version_text);
    energy_init(&en, life_found, ms_now, fw_version, stat.reset_flags & RCC_IWDGRSTF);
#if BOARD == BOARD_DP_RX
    en.ev_save = 1u;                       /* (0.6.10: core/energy.h) */
#endif
    if (stat.reset_flags & RCC_IWDGRSTF) { /* (0.6.9, B2) the fault memory: the last abnormal reset (an exception ends
                                              in the watchdog's), with the breadcrumbs of that run */
        en.lt.rst = (uint8_t)(stat.bc.magic == BC_MAGIC && stat.bc.cause != 0u ? 2u | (stat.bc.cause & 63u) << 2 : 1u);
        en.lt.rst_h = (uint8_t)(stat.bc.magic == BC_MAGIC ? (stat.bc.up_s / 3600u > 255u ? 255u : stat.bc.up_s / 3600u)
                                                         : 0u);
    }
    stat.version = fw_version != 0u || UPD_APP ? fw_version_text : FW_VERSION " (development build)";
    stat.bl_version = upd.info.bl_version;
    stat.uid = HW_UID;                     /* (the test jig's line) */
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
#if CONSOLE_FULL
    cli.apply = cli_apply;
    cli.save_now = save_now;
#if MON_EXTRA
    cli.ext = &sext;                       /* (0.6.7, D9) */
    cli.restore = restore_rec;
#endif
    cli.status_begin = status_begin;
#endif
    cli.set = &set;
    cli.eff = &eff;
    cli.scene = &scene_want;
    cli.st = &stat;
    stat.dip = &vdip;
    stat.pw = &pw;
    stat.ev = &evl;                        /* (0.6.5, D2) */
    stat.hl = &hl;
    stat.dip_vs = &vs_sup;
    stat.en = &en;
    stat.ovr = ovr;
#if MON_EXTRA
    stat.hwd = hw_diag;                    /* (0.6.5, D6) */
#if MON_EXTRA
    stat.ch = &chist;                      /* (0.6.6, M2) */
#endif
#endif
    ultra_on = (uint8_t)(proto_active(&frx.ps) == PROTO_ULTRA);
    apply_settings();
    {
        gate_plan_t g = proto_gate_plan(proto_active(&frx.ps));
        plan_buf[0] = plan_buf[1] = g;
        plan = g;
        BARRIER();                         /* (as at the frame end) */
        rx_ready = 1;                      /* gate closed until the tracker arms (idle line) or a frame ends */
    }
    for (ch = 0; ch < (int)REC_COUNT + fx_clear_pend; ch++) {
        flash_erase_ahead();               /* every store's next page: erased now, the outputs still dark (0.6.3: the
                                              lifetime record's; one call erases one page, so one call per store: 0.6.6
                                              had four for the upper layout's five, review round 6) */
    }
    flash_boot = 0;
    budget_init(&flash_bright, ms_now, 1u, FLASH_BRIGHT_MS);
#if CONSOLE_FULL
    cli_banner(&cli);
    cli_say(&cli, ZC(m_type_help, "type help\r\n"));
#else
    cli_banner(&cli);                      /* (0.6.7) the release console: the banner at once, the full status once */
    watch_t = ms_now;                      /* the live values are valid (the stream below: a live line first) */
#endif

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
        if (ultra_on != (uint8_t)proto_active_ultra(&frx.ps)) {   /* (inline: every pass; the call below is rare) */
            protocol_changed(ultra_on ? PROTO_ULTRA : PROTO_NONE, proto_active(&frx.ps));   /* console mode change */
        }
#if CONSOLE_FULL
        if (con_ch < 0 && !cli.st_part) {  /* (0.6.5) a `status` in progress: the input waits in the RX ring */
            con_ch = uart_getc();          /* (0.6.6) the next byte, read ahead: held until the UART has a reply's room */
            if (con_ch == -2) {
                cli.len = 0;               /* a flash operation dropped received bytes: the line being typed goes */
                con_ch = uart_getc();
            }
        }
        if (con_ch >= 0 || say_n != 0u || cli.st_part || cli.watch_ms) {   /* (0.6.6; else the UART isn't asked) */
            room = uart_room();
            if (say_n != 0u && room >= SAY_ROOM) {   /* the firmware's messages, in order, one a pass */
                cli_say(&cli, say_q[say_h]);
                say_h = (uint8_t)((say_h + 1u) & (SAY_Q - 1u));
                say_n--;
                room = uart_room();
            }
            ch = -1;
            if (room >= CLI_PART_ROOM) {   /* room for any reply, a `status` part or a watch line */
                ch = con_ch;
                con_ch = -1;
                if (cli.st_part) {
                    stat.uptime_s = up_s;  /* (0.6.6) the ages against now: the event and dip logs are read live, and
                                              one may come between two parts (a past one only: never a negative age) */
                    (void)cli_poll(&cli);  /* the next part of a `status`: the UART takes it without a wait */
                }
            }
            if (ch >= 0 || (!cli.st_part && cli.watch_ms && ms_now - watch_t >= cli.watch_ms && room >= CLI_PART_ROOM)) {
                status_snapshot();         /* live values; extrema are consumed only by status_begin */
                if (ch < 0) {
                    watch_t = ms_now;
                    cli_watch(&cli);
                }
                for (; ch >= 0; ch = uart_getc()) {
                    cli_feed(&cli, (char)ch);
                    if (ch == '\r' || ch == '\n') {
                        break;             /* one line per pass (0.6.4, gpt-opt2: seven queued `status` lines blocked
                                              one pass for 0.8 s, past the watchdog; the rest waits in the RX ring) */
                    }
                }
            }
        }
#else
        /* (0.6.7) the release console only sends: the full status once the live values are valid after boot
         * (STREAM_BOOT_S) and every STREAM_STATUS_S (in parts), a line of live values every second between (as the
         * full console's `watch`), the firmware's messages; while the test jig runs only its lines (the stream
         * pauses). Each only while the UART has its room: the main loop never waits for it (the firmware's messages
         * as in the full console, one a pass) */
        if (say_n != 0u || cli.st_part || ms_now - watch_t >= 1000u) {
            room = uart_room();
            if (say_n != 0u && room >= SAY_ROOM) {
                cli_say(&cli, say_q[say_h]);
                say_h = (uint8_t)((say_h + 1u) & (SAY_Q - 1u));
                say_n--;
                room = uart_room();
            }
            if (TJ_ACTIVE()) {
                watch_t = ms_now;          /* (paused) */
            } else if (room >= CLI_PART_ROOM) {
#if STREAM_STATUS
                if (cli.st_part) {
                    stat.uptime_s = up_s;  /* (the ages against now: the logs are read live, as in the full console) */
                    (void)cli_poll(&cli);
                } else
#endif
                if (ms_now - watch_t >= 1000u) {
                    watch_t = ms_now;
                    status_snapshot();
#if STREAM_STATUS
                    if (status_first ? up_s - status_s >= STREAM_STATUS_S : (mt.valid || up_s >= STREAM_BOOT_S)) {
                        status_first = 1;
                        status_s = up_s;
                        status_begin();    /* (the extrema: since the last status) */
                        cli.st_part = 1;
                    } else
#endif
                    {
                        cli_watch(&cli);
                    }
                }
            }
        }
#endif
        uart_kick();
        if (ms_now != last_ms) {
            uint32_t m = ms_now;
            if (ovr_on) {
                ovr[2] += m - last_ms - 1u;    /* the ms late (not after a flash operation, nor the first) */
            }
            ovr_on = 1;
            last_ms = m;
            if (ack_tick(&ack, &ack_on) || was_active) {
                outputs_dirty = 1;
            }
            was_active = ack.active;
            isense_service();              /* measurement of the last ms: limit, trips, calibration */
#if BOARD_TESTJIG
            testjig_service();
#endif
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
                if (PERF_CALL(PERF_RENDER, ultra_render(&ul, tick_now, uout))) {
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
                bcrumb.up_s = up_s;        /* (0.6.5, D1) */
                energy_second(&en, ms_now, up_s, (uint32_t)pw.short_trips << 16 | pw.ovl_trips, is.i_avg, temp_peak);
#if MON_EXTRA
                energy_ch_second(&en, &is, vs_sup.mv);
#endif
                cpu_second();
            }
            flash_erase_ahead();           /* while low: the stores' next pages */
            /* save 2 s after the last change, not while a pattern is blinking (except its final dark pause), with a
             * token of the settings' write budget: a burst, then one per STORE_REFILL_MS (and when the flash admits
             * it: 0.6.4). The newest values wait, so the last change is never lost (only stored later). */
            if (reset_save && (may = flash_may(settings_pending() == 2u, FM_PROG)) != 0u) {
                reset_save = 0;            /* (r6, F10) a factory reset's record at once (one try; else as any change) */
                (void)settings_write(may);
            }
            if (settings_dirty && ms_now - changed_ms >= SAVE_DEBOUNCE_MS && (!ack.active || ack.seg + 1u == ack.nseg) &&
                (may = flash_may(settings_pending() == 2u, 0u)) != 0u && budget_take(&set_budget, ms_now, 1u)) {
                (void)settings_write(may);
            }
            scene_service();
            /* the calibration (a newer test meanwhile: that one; 2 = a cleared record, the factory reset's). (review
             * round 6) Here, as every store, before outputs_update: flash_may's "low" is the tables the PWM runs
             * now (vsense_service above); after outputs_update had rebuilt them brighter it still said low (an
             * erase while an output was at 25 %, a program without the hour's token) */
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
                        void_done(STORE_VOID_CAL);
#if MON_EXTRA
                        if (have) {
                            calhist_add(&chist, en.lt.lit_s, &c);   /* (M2) a stored test: into the history */
                            chist_pend = 1;
                        }
#endif
                    }
                } else {
                    cal_save_pend = 0;
                }
            }
#if MON_EXTRA
            /* (0.6.6, M2) the calibration history: its record after a stored test, as any store's (32 half-words) */
            if (chist_pend && (may = flash_may(calhist_pending() == 2u, 0u)) != 0u &&
                (!chist_retry || budget_take(&chist_budget, ms_now, 1u))) {
                flash_stall_begin(may);
                chist_pend = chist_retry = (uint8_t)!flash_stall_end(calhist_store(&chist));
                if (!chist_pend) {
                    void_done(STORE_VOID_HIST);
                }
            }
#endif
            if (srun.fading || srun.in || ss.state == SS_RUN) {
                outputs_dirty = 1;         /* power-on scene cross-fade, soft start (offset, ramp): every ms */
            }
        }
        if (outputs_dirty) {
            outputs_dirty = 0;
            outputs_update(ack_on);
        }
        /* (claude-cpu, 0.6.4) nothing left to do: the core sleeps until the next interrupt, at the latest the poll tick
         * (50 us); a flag an interrupt sets after these checks waits for that tick. Only the core stops: DMA, timers,
         * ADC and the UART run on (RM 2.3.2; AHBPCENR SRAMEN = 1 from reset), and a debugger keeps it awake. Not during
         * a flash operation (flash_op runs inside this pass), so its SRAM tick is unaffected. ~4 mA less at 3.3 V
         * (datasheet tables 4-6-1 / 4-7-1: 11.4 -> 7.1 mA at 144 MHz), wake-up 1.4 us. BENCH: the current, the
         * debugger, the slew interrupt's budget at 100 kHz. The CPU load (0.6.5): stamps before and after it. */
        if (!cap_ready && !fe_ready && ms_now == last_ms) {
            uint32_t k = tick_now, c0 = POLL_CNT(), c1;
            CPU_SLEEP();
            c1 = POLL_CNT();
            cpu_idle += cpu_slept(k != tick_now, c0, c1, POLL_CYC);
        }
    }
}
