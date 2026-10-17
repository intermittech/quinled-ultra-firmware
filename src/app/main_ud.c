/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED Diff-Power Ultra-Digital receiver firmware (BOARD=3; docs/ultradigital-firmware.md).
 *
 * No light engine: the strip's data passes the board through a 227.6 us delay line (hal/hw_dline.c: SPI1 full duplex
 * over a DMA ring; the window logic: core/dline.c), and the board answers the hub's polls on a pixel line with the
 * Ultra-Analog receiver's pages + 0x80 PIXEL. On a plain controller (never polled) it is transparent.
 *
 * Milestone 2 (this file): the shell. The time base and the ADC (20 kHz tick), the records (the lifetime record), the
 * bus / V_B / current with the VREFINT scale, the supply dips, energy and LIFE_SYNC, the flash policy (main.c's,
 * reduced), the output-only console, the status LED, the firmware update's app side, and the answers' page builder
 * (ud_rc_page). The delay line's logic is not wired: its hooks (dl_isr_fault, dl_tick, dl_ram_tick, dl_main, ud_busy)
 * are stubs marked LEAD, and the mux stays direct, so the strip gets RO as it is. */
#include <stddef.h>
#include "ch32v20x.h"
#include "../hal/hw.h"
#include "../hal/hw_dline.h"
#include "../core/ack.h"
#include "../core/bl_protection.h"
#include "../core/cli.h"                   /* bcrumb_t (the reset breadcrumbs: isr.S writes them too) */
#include "../core/cpuload.h"
#include "../core/dline.h"
#include "../core/ud/udfx.h"
#include "../core/ud/udchain.h"
#include "../core/div64.h"
#include "../core/fwver.h"
#include "../core/lto.h"
#include "../core/mcutemp.h"
#include "../core/pwrev.h"
#include "../core/rc.h"
#include "../core/upd_app.h"
#include "../core/vdip.h"
#include "../core/vsense.h"
#include "../core/ws_decode.h"
#include "../core/ultra_update.h"

#if BOARD != BOARD_DP_UD
#error "main_ud.c is the Ultra-Digital receiver's app: BOARD=3"
#endif
#ifndef FW_VERSION
#define FW_VERSION       "0.6.12"   /* release builds: the release version (one for every board of the tree) */
#endif
#ifndef UPD_APP
#define UPD_APP 0                  /* 1: linked behind the bootloader (vendor/wch/Link_app.ld, build_release.sh) */
#endif
#define UPD_WD_CLEAR_S   10u       /* the bootloader's watchdog-loop count is cleared after this long running */
#define PLAN_CUT_MS      10000u    /* (as BOARD 2) a dip starting this long after IDENTIFY 7 is the planned cut's */
#define TEMP_HOT_C10     950       /* the die at 95.0 C: STATUS's reason HOT (a warning: nothing changes) ... */
#define TEMP_HOT_HYST_C10 100      /* ... until it is back under 85.0 C */
#define POLL_CYC (144000000u / POLL_HZ)   /* HCLK cycles per poll tick (SysTick's period, hw_cold.c) */
/* mA per ADC count of the bus current, Q16, for VDDA in mV: VDDA / 4095 x 1000 / (2 mohm x ISENSE_AMP_GAIN), board.h's
 * convention (65536 x 500 / 4095 = 8001.95; 1.6 mA at 3.28 V with the gain 250) */
#define UD_MA_Q16(vdda)  (((uint32_t)(vdda) * 8002u + ISENSE_AMP_GAIN / 2u) / ISENSE_AMP_GAIN)

/* ------------------------------------------------------------------------------- shared with the poll ISR */
static volatile uint32_t ms_now, tick_now; /* 1 ms and 50 us time bases */
static uint32_t poll_sub;                  /* ticks into the ms (poll_isr, flash_poll_isr) */
static uint8_t adc_hold;                   /* the next tick's current sample is held (the temperature sequence) */
static volatile uint32_t cur_sum, cur_n;   /* the bus current's samples (ADC counts) since the main loop's last take */
/* the reset breadcrumbs (core/cli.h bcrumb_t, Link_sections.ld .noinit; isr.S's fault handler writes pc / cause) */
bcrumb_t bcrumb __attribute__((section(".noinit")));
typedef char bcrumb_offsets[(offsetof(bcrumb_t, pc) == 8u && offsetof(bcrumb_t, cause) == 12u) ? 1 : -1];   /* isr.S */

/* ------------------------------------------------------------------------------- the delay line (LEAD)
 * The statistics the window logic keeps (core/dline.c via the hooks below): page 0x80 PIXEL (design note section 8
 * item 5) and the ERRORS page's counters. Written by the tick / the main loop's glue, read by the page builder and the
 * console. */
enum { UD_MODE_DIRECT = 0, UD_MODE_DELAYED = 1, UD_MODE_FAULT = 2 };
typedef struct {
    /* PIXEL (the rest: dline's dl.st) */
    uint16_t fps10;                        /* frames in the last 10 s (= frames per second x 10; second_service) */
    uint8_t  faults;                       /* delay-line faults (OVR, DMA error, RX - TX distance; saturating) */
    uint16_t polls;                        /* polls for this board seen (wrap; answered: dl.st.answered) */
    /* ERRORS (u16, wrap) */
    uint16_t headers;                      /* Ultra frames (behind a marker) with a valid header */
    uint16_t tail_err;                     /* marker frames without one (decode, CRC-8) */
    uint16_t decoded;                      /* slots decoded (CRC-16 good) */
    uint16_t dec_err;                      /* captures that failed to decode */
} ud_stats_t;
static ud_stats_t uds;

/* the window logic (core/dline.c) and its glue. dline decides from the ring alone (the tick's scan); the glue applies its
 * actions to the pins and the UART and hands the capture to the main loop. Contexts: the tick (dl_tick), the SRAM tick
 * during a flash operation (dl_ram_tick), USART3's TC interrupt (dl_tx_done: DE is low), SPI1's error interrupt
 * (dl_isr_fault), the main loop (dl_main: decode, answer, dline_arm). */
static dline_t dl;
static udfx_t ux;                          /* the local pixel effects (core/ud/udfx.h, docs/ultradigital-firmware.md section 11) */
static udc_t uc;                           /* the line mode: STREAM / CHAIN / AUTO deciding (core/ud/udchain.h, section 12) */
static volatile uint8_t dl_ch_ready;       /* DLINE_A_CH_READY not taken by the main loop yet */
static udc_pre_t ucp;                      /* the slot of the frame in progress, decoded as its words arrive (udc_stream) */
static volatile uint32_t dl_ch_tick;       /* the tick it came at */
static volatile uint8_t dl_ev_marker, dl_ev_missed;   /* the tick counts (wrapping), the main loop keeps what it has seen */
static uint8_t ev_marker_seen, ev_missed_seen;
typedef char ud_guard_fits[DL_RING_GUARD == WS_GUARD ? 1 : -1];
static volatile uint8_t dl_ready;          /* FRAME_READY not taken by the main loop yet */
static volatile uint32_t dl_ready_tick;    /* the tick it came at (the clock rate's local stamp, ~1 tick late) */
static volatile uint32_t dl_ready_w;       /* ... and the ring word of that frame's last edge (the host-clock anchor, 0.44 us) */
static volatile uint8_t dl_fault_pend;     /* SPI1 OVR / the SRAM tick's distance check: the next tick restarts */
static volatile uint8_t dl_on;             /* dline_init done: the ticks run the window logic (the tick starts in
                                              hw_init, the ring right after it: code review #1) */
static uint8_t ud_ans[RC_ANSWER_BYTES];
static uint16_t ud_brr;                    /* USART3's divider for the armed answer (ud_frame) */

/* SPI1 OVR (hw_dline.c dl_spi1_isr, above the tick): RX stopped, TX replays a stale ring: the strip line LOW at once
 * (Quindor: invisible on the strip), the tick restarts the ring */
void dl_isr_fault(void)
{
    dl_hw_blank(1);
    dl_fault_pend = 1;
}

/* USART3 TC (hal/hw.c rc_uart_isr, BOARD 3): the burst's last stop bit is out, DE low now: the exact word where RO comes
 * back (dline's RETAKE scan starts there) */
void dl_tx_done(void)
{
    dline_burst_done(&dl, dl_hw_rx_pos());
}

/* the line moved since the tick read TIM4: PA6 HIGH now, or an edge counted. TIM4 is read >= 0.1 us after PA6 (16
 * cycles, >= 0.11 us at 144 MHz, 0.22 at HCLK / 2): a falling edge just before the PA6 read has passed TIM3's input
 * filter and synchroniser by then (dline.h) */
static ALWAYS_INLINE uint32_t dl_live_moved(uint32_t edges)
{
    uint32_t hi = dl_hw_din();
    __asm volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop" ::: "memory");
    return hi | (uint32_t)(dl_hw_edges() != (uint16_t)edges);
}

/* dline's pin actions. LIVE_CHECK (a switch or a release that the visible direct path would show): only while the line is
 * still LOW and no edge came since the tick read TIM4 (both paths LOW: nothing repeated or cut), else none of them and
 * dline is told (asked again later). The SRAM tick uses it too: everything inline / in SRAM */
static ALWAYS_INLINE uint32_t dl_pins(uint32_t a, uint32_t edges)
{
    if ((a & DLINE_A_LIVE_CHECK) && dl_live_moved(edges)) {
        dline_refused(&dl);
        a &= ~(DLINE_A_MUX_DELAYED | DLINE_A_MUX_DIRECT | DLINE_A_BLANK_OFF);
    }
    if (a & DLINE_A_BLANK_ON) {
        dl_hw_blank(1);                    /* (first: a fault's stale ring, SEND's DE rise) */
    }
    if (a & DLINE_A_MUX_DELAYED) {
        dl_hw_mux(1);
    }
    if (a & DLINE_A_MUX_DIRECT) {
        dl_hw_mux(0);
    }
    if ((a & DLINE_A_BLANK_OFF) && !dl_fault_pend) {   /* (an OVR since the tick began: stay LOW) */
        dl_hw_blank(0);
    }
    return a;
}

/* a fault's restart (SPI OVR from dl_isr_fault, or dline's own finding): BLANK_N is LOW already (dl_pins), the ring
 * zeroed and running again, the mux as it is; dline goes on as at BOOT (BLANK_N released at the next quiet run) */
static void dl_restart(void)
{
    dl_hw_restart();
    dline_restart(&dl, dl_hw_rx_pos());
    udfx_restarted(&ux, &dl);              /* (local mode: the encoder starts again from LOW) */
    if (uds.faults < 255u) {
        uds.faults++;
    }
}

static inline void dl_tick(void)
{
    uint32_t e, rx, tx, a;
    if (!dl_on) {
        return;
    }
    e = dl_hw_edges();                     /* (TIM4 before the positions: dline.h) */
    rx = dl_hw_rx_pos();
    tx = dl_hw_tx_pos();
    if (dl_fault_pend) {
        dl_fault_pend = 0;
        a = dl_pins(dline_fault(&dl), e);
    } else {
        a = dline_tick(&dl, dl_ring, rx, tx, e);
        if (a != 0u) {
            a = dl_pins(a, e);
        }
    }
    if (a & DLINE_A_RESTART) {
        dl_restart();
        return;
    }
    if (a & DLINE_A_SEND) {
        if (udfx_answer_blank(&ux)) {
            rc_hw_dir(RC_DIR_ANSWER);      /* BLANK_N LOW, then DE high (hal/hw.c) */
        } else {
            rc_hw_answer_local();          /* a local effect runs: DE only (BLANK_N would freeze it: nothing releases it) */
        }
        (void)rc_hw_send(ud_ans, RC_ANSWER_BYTES, ud_brr);   /* (3 us of line HIGH, then by interrupt; TC: dl_tx_done) */
    }
    if (a & DLINE_A_FRAME_READY) {
        dl_ready_tick = tick_now;
        dl_ready_w = dl.end_w;
        BARRIER();
        dl_ready = 1;
    }
    if (a & DLINE_A_EDGES_ZERO) {
        dl_hw_edges_zero();                /* CHAIN: the gap before the next frame: its bits are TIM4's count (dline owns TIM4) */
    }
    if (a & DLINE_A_CH_READY) {
        dl_ch_tick = tick_now;
        BARRIER();
        dl_ch_ready = 1;
    }
    if (a & DLINE_A_CH_MARKER) {
        dl_ev_marker++;
    }
    if (a & DLINE_A_CH_MISSED) {
        dl_ev_missed++;
    }
    udfx_tick(&ux, &dl, dl_ring);          /* local pixel effects: the mode switches and the encoder's fill, after the scan */
}

/* the same during a flash operation, from SRAM (hw_cold.c flash_op: SPI1's and USART3's interrupts are off): the scan
 * zeroes a marker, a window that is open keeps its zeroing; no capture, no SEND. A fault (dline's) only blanks here:
 * dl_hw_restart is in the flash, so the first tick after the operation restarts (dl_fault_pend) */
void dl_ram_tick(void);
RAMCODE void dl_ram_tick(void)
{
    uint32_t e, rx, tx, a;
    if (!dl_on) {
        return;
    }
    e = dl_hw_edges();
    rx = dl_hw_rx_pos();
    tx = dl_hw_tx_pos();
    a = dline_ram_tick(&dl, dl_ring, rx, tx, e);
    if (a != 0u) {
        a = dl_pins(a, e);
        if (a & DLINE_A_RESTART) {
            dl_hw_blank(1);
            dl_fault_pend = 1;
        }
    }
    if (!dl.restart) {
        udfx_ram_tick(&ux, dl_ring, dl.wpos);   /* the local effect's fill goes on (SRAM code): the flash operation shows nothing */
    }
}

/* the main loop's half: a captured marker frame decoded (ud_decode -> ud_frame: a POLL's answer built into ud_ans with
 * its divider), then dline_arm: 1 = the window may send it */
static uint32_t ud_decode(const uint16_t *cap, uint32_t words, uint32_t end_tick);
static void ud_chain_decode(void);
static inline void dl_main(void)
{
    if (dl.ch_st == DLINE_CH_FRAME && dl.ch_cap) {   /* a frame is coming in: its slot is decoded as the words arrive (the header needs the end) */
        udc_stream(&dl, &ucp);
    }
    if (dl_ch_ready) {
        dl_ch_ready = 0;                   /* (before the decode: it frees the capture before it applies, and the next frame's ready may come while it does) */
        BARRIER();
        ud_chain_decode();
    }
    if (dl_ready) {
        uint32_t ok;
        BARRIER();
        ok = ud_decode(dl.cap, dl.cap_n, dl_ready_tick);
        dl_ready = 0;
        BARRIER();                         /* the answer complete before the window may send it */
        dline_arm(&dl, (int)ok);
    }
}
/* no flash operation while a window may be open or a capture waits (design note section 5) */
static inline uint32_t ud_busy(void)
{
    BARRIER();                             /* (dl.state: the tick's; read fresh) */
    return !dline_idle(&dl) || dl_ready || !dline_ch_idle(&dl) || dl_ch_ready;
}
static void ud_rc_answer(const uint8_t *poll, uint8_t a[RC_ANSWER_BYTES]);
static void upd_action(uint32_t a);

/* ------------------------------------------------------------------------------- main loop state */
static mcutemp_t mt;                       /* the MCU temperature and VDDA from VREFINT (core/mcutemp.h) */
static uint8_t temp_started, too_hot;
static int16_t temp_peak = MCUTEMP_NA;     /* the highest valid MCU temperature since boot */
static vsense_t vs_bus, vs_vb;             /* PA4 the bus, PA0 V_B (core/vsense.h; scaled by VREFINT) */
static vdip_t vdip;                        /* bus dips (core/vdip.h) */
static energy_t en;                        /* input energy, the lifetime record (core/energy.h) */
static pwrev_t pw;                         /* POWER: over the link (core/pwrev.h) */
static upd_app_t upd;                      /* firmware update, app side (core/upd_app.h) */
#if UPD_APP
#define HO ((upd_handover_t *)UPD_HANDOVER_ADDR)  /* outside every linker section (Link_app.ld) */
static uint8_t upd_wd_cleared;
#endif
static uint8_t upd_res, bl_prot;           /* the bootloader's last result (UPD_RES_*) and protection report (0 none,
                                              1 bad option bytes, 2 ok, 3 failed): the console; LEAD: the LED's codes */
static uint32_t fw_version;                /* core/fwver.h: the descriptor's in release builds; 0 = development build */
static char fw_version_text[FWVER_TEXT_MAX];
static uint32_t reset_flags;               /* RCC's (through the hand-over behind the bootloader) */
static bcrumb_t bc;                        /* the last run's breadcrumbs */
static uint32_t up_s, sec_ms;              /* uptime in s, and the ms it last counted */
static uint32_t plan_ms;                   /* IDENTIFY 7's notice (vdip.planned) */
static uint16_t rc_uid16;                  /* CRC-16 over the unique id: in every answer */
static uint16_t ls_res;                    /* the last LIFE_SYNC's: energy_sync's (applied | result << 8) */
static uint8_t ud_lit;                     /* pixel frames flowed in the last second: the energy's "lit" time */
static uint16_t wrong_seen;                /* CHAIN: the non-Ultra frames counted when the LED last blinked */
static uint32_t wrong_s;
static void led_fault(uint8_t code);
static uint32_t fps_ring[10];              /* dline's frames at each of the last 10 s */
static uint8_t fps_i;
/* the bus current (PB0): the last ms, its ~64 ms average (x 64 in i_acc), and the highest average since the last
 * ENERGY answer; mA per count, Q16, follows VREFINT (temp_service) */
static int32_t i_ms, i_avg, i_acc;
static uint16_t i_peak;
static uint32_t i_q16 = UD_MA_Q16(VSENSE_VREF_MV);
/* CPU load (core/cpuload.h) and the main loop's late ms (DIAG) */
static uint32_t cpu_idle, cpu_t0, ovr_late;
static uint16_t cpu_pm, cpu_peak;
static uint8_t ovr_on;
static uint16_t saves;                     /* flash records written (ERRORS) */
static uint8_t con_flash_err;              /* the console's "flash: error" is due */
static uint32_t con_ms, con_status_s;

static uint16_t sat16(int32_t v)
{
    return (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v);
}

/* ------------------------------------------------------------------------------- the ticks */
void poll_isr(void);                       /* 20 kHz tick, entered via isr.S (HPE + mret) */
void poll_isr(void)
{
    POLL_ACK();
    tick_now++;
    if (adc_hold) {
        adc_hold = 0;                      /* (the temperature's 84 us sequence holds the regular conversion) */
    } else {
        cur_sum += ISENSE_RAW();
        cur_n++;
    }
    if (++poll_sub >= POLL_HZ / 1000u) {
        poll_sub = 0;
        ms_now++;
        bcrumb.pc = TRAP_PC();             /* where the main loop is, once a ms */
        adc_hold = (uint8_t)hw_adc_ms();   /* the injected group: the bus and V_B / the MCU temperature */
    }
    dl_tick();
}

/* The tick during a flash operation (hw_cold.c flash_op: through VTF, from SRAM, HCLK halved and SysTick's compare
 * with it): the time bases and the delay line's SRAM half, nothing else */
RAMCODE void flash_poll_isr(void)
{
    POLL_ACK();
    tick_now++;
    if (++poll_sub >= POLL_HZ / 1000u) {
        poll_sub = 0;
        ms_now++;
    }
    dl_ram_tick();
}

/* ------------------------------------------------------------------------------- the flash: when
 * main.c's policy, reduced: no outputs of its own to keep steady (a flash operation halves HCLK; SPI1 and the DMA keep
 * the strip's delay, the SRAM tick its zeroing), so a write may run whenever no poll window can come (ud_busy), the
 * console isn't sending, no failure's hold-off runs and VDD is above the PVD; in a bus dip programs only (the lifetime
 * record's dip write into its pre-erased entry), never an erase. The erase ahead: at boot, then not before
 * LIFE_PREP_S. */
#ifndef FLASH_DIP_WRITES
#define FLASH_DIP_WRITES 1
#endif
#define FLASH_RETRY_MS   10000u            /* (main.c) after a failure: this long, doubling, up to << FLASH_RETRY_MAX */
#define FLASH_RETRY_MAX  6u
static uint32_t flash_err_ms;
static uint8_t flash_err;                  /* failures in a row (DIAG) */
static uint8_t flash_boot = 1, flash_recheck = 1;

static uint32_t flash_held(void)
{
    return flash_err && ms_now - flash_err_ms < (FLASH_RETRY_MS << (flash_err - 1u));
}

static uint32_t flash_may(uint32_t need_erase)
{
    return !ud_busy() && !uart_busy() && !flash_held() && !hw_vdd_low() &&
           !(vdip.in_dip && (need_erase || !FLASH_DIP_WRITES)) &&
           !(need_erase && !udfx_flash_ok(&ux));   /* (an erase stalls the render for 16 ms: only on the stream or in the dark) */
}

static void flash_stall_begin(void)
{
    ovr_on = 0;                            /* (the ms it delays: not late) */
}

/* ... done with result r (core/flashrec.h): 1 = written */
static uint32_t flash_stall_end(uint32_t r)
{
    flash_recheck = 1;
    if (r == FLASH_OK) {
        saves++;
        if (!flash_held()) {
            flash_err = 0;
        }
    } else if (r != FLASH_NONE) {
        flash_err = (uint8_t)(flash_err + (flash_err <= FLASH_RETRY_MAX));
        flash_err_ms = ms_now;
        con_flash_err = 1;
    }
    return r == FLASH_OK;
}

/* the lifetime record's next page erased ahead (its only store) */
static void flash_erase_ahead(void)
{
    if (!flash_recheck || (!flash_boot && up_s < LIFE_PREP_S) || !flash_may(1u)) {
        return;
    }
    if (life_pending() != 0u) {
        flash_stall_begin();
        (void)flash_stall_end(life_prepare());
        return;
    }
    flash_recheck = 0;
}

/* the local-effects record (core/ud/udfx.h): written 2 s after the last change (a burst of CONFIG slots is one record) and at
 * most every 30 s, only on the hub's stream or in the dark (the render never pauses under a lit effect); its next page is
 * erased ahead in such a moment too */
static uint32_t ufx_last_ms;
static void ufx_flash_service(void)
{
    if (!udfx_flash_ok(&ux) || !flash_may(1u)) {
        return;
    }
    if (ux.save_due) {
        if (ms_now - ux.save_ms >= 2000u && (ufx_last_ms == 0u || ms_now - ufx_last_ms >= 30000u)) {
            flash_stall_begin();
            if (flash_stall_end(ufx_store(&ux))) {
                udfx_rec_saved(&ux);
            }
            ufx_last_ms = ms_now | 1u;
        }
    } else if (ufx_pending() == 1u) {
        flash_stall_begin();
        (void)flash_stall_end(ufx_prepare());
    }
}

static uint32_t life_admit(void)
{
    return life_pending() == 2u ? 0u : (FLASH_DIP_WRITES && !hw_vdd_low() && !ud_busy() ? ENERGY_DUE_DIP : 0u) |
                                       (flash_may(0u) ? ENERGY_DUE_SAVE : 0u);
}

/* ------------------------------------------------------------------------------- the measurements */
/* every ms: the bus current of that ms (the tick's samples), its average and peak */
static void current_service(void)
{
    uint32_t s, n;
    __disable_irq();
    s = cur_sum;
    n = cur_n;
    cur_sum = cur_n = 0;
    __enable_irq();
    if (n != 0u) {                         /* (none: a flash operation's ms) */
        i_ms = (int32_t)(s * (i_q16 >> 4) / n >> 12);   /* (<= 21 x 4095 x 7.4k: 32 bits) */
    }
    i_acc += i_ms - (i_acc >> 6);
    i_avg = i_acc >> 6;
    if (i_avg > (int32_t)i_peak) {
        i_peak = sat16(i_avg);
    }
}

/* every ms: POWER's state. The link limit (the port's PTCs) can't be held here: over it is a limit event and STATUS's
 * LIMITING (design note section 6: monitoring only, no trip) */
static void power_service(void)
{
    pwrev_in_t in;
    in.limiting = (uint8_t)(i_avg > (int32_t)ISENSE_BOARD_MA);
    in.ch_limiting = 0;
    in.trip_cause = ULTRA_PW_CAUSE_NONE;
    in.i_ma = sat16(i_ms);
    in.trip_ma = in.i_ma;
    in.limit_ma = ISENSE_BOARD_MA;
    in.limit_src = ULTRA_PW_SRC_BOARD;
    in.factor = 65535u;
    in.retry_ms = 0;
    in.dips = vdip.count;
    pwrev_update(&pw, ms_now, &in);
}

/* every ms: the injected group's pairs of that ms: the bus and V_B, the dips, the energy and the lifetime record */
static void vsense_service(void)
{
    uint32_t sum2[2], n, was, cut;
    hw_vsense_take(sum2, &n);
    vsense_add(&vs_bus, sum2[0], n);
    vsense_add(&vs_vb, sum2[1], n);
    was = vdip.in_dip;
    vdip_ms(&vdip, ms_now, up_s, sum2[0], n, i_ms);
    if (was == 1u && !vdip.in_dip) {       /* the fault memory: a dip ended (not the planned cut's) */
        const vdip_ev_t *d = vdip_event(&vdip, 0);
        if (d != 0 && d->ref_x != 0u) {
            en.lt.dip_pct = (uint8_t)((uint32_t)d->min_x * 100u / d->ref_x);
            en.lt.dip_cs = (uint8_t)(d->dur_ms / 10u > 255u ? 255u : d->dur_ms / 10u);
            en.lt.dip_a = (uint8_t)(d->i_ma / 100u > 255u ? 255u : d->i_ma / 100u);
        }
    }
    cut = vdip.in_dip > was + 1u;          /* the planned cut starts this ms (in_dip 0 -> 2) */
    vdip_z(&vdip, &vs_bus, (uint32_t)i_ms);
    energy_ms(&en, ms_now, vs_bus.mv, i_ms, ud_lit);
    n = ENERGY_DUE_NONE;
    if (en.due || vdip.count != en.dips || cut) {
        n = energy_due(&en, ms_now, vdip.count, life_admit() | (cut ? ENERGY_PLANNED : 0u));
    }
    if (n != ENERGY_DUE_NONE) {
        flash_stall_begin();
        if (flash_stall_end(life_store(&en))) {
            energy_saved(&en, up_s);
        }
    }
}

/* IDENTIFY 7, the hub's planned power-off notice (LEAD: the frame's handler calls it): a dip starting within
 * PLAN_CUT_MS is the planned cut's (core/vdip.h in_dip 2: no dip counted, the record written ENERGY_PLANNED) */
void ud_power_off_notice(void)
{
    vdip.planned = 1;
    plan_ms = ms_now;
}

/* once a second: the conversion started a second ago (MCU temperature, VREFINT), the next one started. VDDA is the
 * LDO's 3.3 V (+-1.5 %, its ripple under the reply burst): every channel is scaled by VREFINT (design note section 6;
 * the last good sample's, mcutemp.c: 2.3-3.7 V). BENCH: VREFINT's own spread (datasheet 1.2 V typ) against a meter */
static void temp_service(void)
{
    uint32_t ts2 = 0, vref2 = 0;
    if (temp_started) {
        if (!hw_temp_read(&ts2, &vref2)) {
            ts2 = vref2 = 0;               /* not done: a bad sample */
        }
        mcutemp_sample(&mt, ts2, vref2);
        too_hot = (uint8_t)(mt.valid && mt.c10 >= TEMP_HOT_C10 - (too_hot ? TEMP_HOT_HYST_C10 : 0));
        if (mt.valid && mt.c10 > temp_peak) {
            temp_peak = mt.c10;
        }
        if (mt.vdda_mv != 0u) {
            vs_bus.vref_mv = vs_vb.vref_mv = mt.vdda_mv;
            i_q16 = UD_MA_Q16(mt.vdda_mv);
        }
    }
    temp_started = 1;
    hw_temp_start();
}

/* bus-side mA per strip-side mA, Q8, for the current limiter: V_B / V_bus / 0.9 (the LED module's buck) */
static uint32_t ux_vk(void)
{
    uint32_t k = 256u;
    if (vs_bus.valid && vs_vb.valid && vs_bus.mv != 0u) {
        k = vs_vb.mv * 284u / vs_bus.mv;
        k = k < 16u ? 16u : k > 1024u ? 1024u : k;
    }
    return k;
}

static void second_service(void)
{
    uint32_t t = tick_now, pm = cpu_permille(cpu_idle, t - cpu_t0, POLL_CYC), f = dl.st.frames;
    cpu_idle = 0;
    cpu_t0 = t;
    cpu_pm = (uint16_t)pm;
    if (pm > cpu_peak) {
        cpu_peak = (uint16_t)pm;
    }
    up_s++;
    bcrumb.up_s = up_s;
    ud_lit = (uint8_t)(f != fps_ring[(fps_i + 9u) % 10u] || (ux.local && !ux.dark));
    ux.load = (uint8_t)(pm / 5u > 200u ? 200u : pm / 5u);   /* page 0x81: the CPU load, 0.5 % steps */
    if (uc.mode == UDC_CHAIN && uc.nonultra != wrong_seen) {   /* CHAIN: frames that are no Ultra frame (a pixel stream): 2 blinks, at most every 30 s */
        wrong_seen = uc.nonultra;
        if (up_s - wrong_s >= 30u || wrong_s == 0u) {
            wrong_s = up_s;
            led_fault(2);
        }
    }
    uds.fps10 = sat16((int32_t)(f - fps_ring[fps_i]));   /* (frames in the last 10 s) */
    fps_ring[fps_i] = f;
    fps_i = (uint8_t)((fps_i + 1u) % 10u);
    energy_second(&en, ms_now, up_s, 0u, i_avg, temp_peak);   /* (no trips on this board) */
    temp_service();
    hw_dma_check();                        /* DMA1 transfer errors -> hw_diag[2 + ch - 1]; LEAD: ch2 / ch3 = a fault */
}

/* ------------------------------------------------------------------------------- the status LED (PB15)
 * On while running. A code (core/ack.h's patterns, the LED only: IDENTIFY 5 the version, an update session's SAME /
 * NACK): 1 s off first, so its first blink shows, then the pattern with its 1 s off, then on again. A code while
 * one runs is dropped (the host's copies of a command show once). */
#define LED_LEAD_MS 1000u
static ack_t led_ack;
static uint16_t led_lead;                  /* ms of the lead-in left (the LED off) */
static uint8_t led_lvl;

static void led_code(ack_kind_t kind)
{
    if (!led_ack.active && led_lead == 0u) {
        ack_start(&led_ack, kind, 0);
        led_lead = LED_LEAD_MS;
    }
}

static void led_fault(uint8_t code)
{
    if (!led_ack.active && led_lead == 0u) {
        ack_start(&led_ack, ACK_P_FAULT, code);
        led_lead = LED_LEAD_MS;
    }
}

static void led_version(void)
{
    if (!led_ack.active && led_lead == 0u) {
        ack_start_version(&led_ack, fw_version);
        led_lead = LED_LEAD_MS;
    }
}

static void led_service(uint32_t ms)       /* every ms; ms = the ms since the last call */
{
    uint8_t on = 1, lvl = 1;
    for (; ms != 0u; ms--) {
        if (led_lead != 0u) {
            led_lead--;
            lvl = 0;
        } else {
            lvl = ack_tick(&led_ack, &on) ? on : 1u;
        }
    }
    if (lvl != led_lvl) {
        led_lvl = lvl;
        led_set(lvl);
    }
}

/* ------------------------------------------------------------------------------- firmware update (app side) */
static void upd_action(uint32_t a)
{
    switch (a) {
    case UPD_APP_TAKE:
#if UPD_APP
    {
        uint32_t t0 = ms_now;
        dl_hw_blank(1);                    /* the strip line LOW and the mux direct before the hand-over (design review
                                              #6): the bootloader blanks only once it stays; the UPD frames meanwhile
                                              never reach the strip */
        dl_hw_mux(0);
        HO->request = UPD_REQ_ENTER;       /* the bootloader checks the ANNOUNCE again and installs it */
        upd_ho_seal(HO);
        uart_write("firmware update: into the bootloader\r\n");
        while (ms_now - t0 < 10u) {        /* the host's lead-in is >= 500 ms: time to send the line */
            uart_kick();
            watchdog_kick();
        }
        NVIC_SystemReset();
    }
#endif
        break;
    case UPD_APP_CONFIRM:
#if UPD_APP
    {
        uint32_t ok = 0;
        if (flash_may(0u)) {               /* (not admitted now: at the next PING, or auto_ok's next try) */
            flash_stall_begin();
            ok = flash_stall_end(hw_flash_half(UPD_APP_DESC + 2u * (UPD_ST_FIRST + upd.info.st_next),
                                               UPD_ST_CONFIRMED));
        }
        upd_app_confirmed(&upd, (int)ok);
    }
#endif
        break;
    case UPD_APP_SHOW_SAME:
        led_code(ACK_P_SAME);              /* the session's image is already installed */
        break;
    case UPD_APP_SHOW_REJECTED:
        led_code(ACK_P_NACK);              /* not for this board, or older without FORCE */
        break;
    default:
        break;
    }
}

/* boot: what the bootloader left in the hand-over block (UPD_APP), the app's descriptor and status records */
static void upd_boot(void)
{
    upd_app_info_t info;
#if UPD_APP
    upd_app_read_info(&info, (const uint8_t *)UPD_APP_DESC, (const uint8_t *)UPD_BL_INFO);
    if (upd_ho_valid(HO)) {
        uint32_t protection = HO->reserved - BL_PROT_BAD;
        reset_flags = HO->reset_flags;     /* the bootloader cleared the RCC flags (its own resets pile up) */
        HO->reset_flags = 0;
        upd_res = (uint8_t)HO->result;
        bl_prot = (uint8_t)(protection <= BL_PROT_FAIL - BL_PROT_BAD ? protection + 1u : 0u);
        HO->result = UPD_RES_NONE;
        HO->request = UPD_REQ_NONE;
        HO->reserved = 0;
        upd_ho_seal(HO);
    }
#else
    info.hw_id = info.hw_rev = info.bl_version = 0;   /* standalone: no update sessions taken */
    info.desc_ok = info.confirmed = 0;
    info.st_next = (uint8_t)UPD_ST_COUNT;
    info.version = 0;
#endif
    upd_app_init(&upd, &info);
}

/* every ms: the transparency timeout / pending results / a delayed confirm; the watchdog-loop count */
static void upd_service(void)
{
    uint32_t a;
    if (up_s >= UPD_APP_AUTO_S && dl.st.frames + uds.decoded >= UPD_APP_AUTO_FRAMES && !(reset_flags & RCC_IWDGRSTF)) {
        upd.auto_ok = 1;                   /* (round 11, B4) a trial image that runs a minute with frames (pixels
                                              through the delay line, or polls) and no watchdog reset confirms itself */
    }
    a = upd_app_poll(&upd, ms_now);
    if (a != 0u) {
        upd_action(a);
    }
#if UPD_APP
    if (!upd_wd_cleared && up_s >= UPD_WD_CLEAR_S && upd_ho_valid(HO)) {
        HO->wd_resets = 0;                 /* running fine: the bootloader starts counting again */
        upd_ho_seal(HO);
        upd_wd_cleared = 1;
    }
#endif
}

/* ------------------------------------------------------------------------------- the return channel's answers
 * The Ultra-Analog receiver's pages (core/ultra_protocol.h, the protocol wiki: Return channel) built from this board's
 * state, + 0x80 PIXEL (hardware-specific). Features and capabilities: design note section 8 (protocol 0.2.7). */
#define UD_FEATURES   (ULTRA_FEAT_MCU_TEMP | (UPD_APP ? ULTRA_FEAT_FW_UPDATE : 0u) | ULTRA_FEAT_VERSION_BLINK | \
                       ULTRA_FEAT_POWER_EVENTS | ULTRA_FEAT_SUPPLY_DIPS | ULTRA_FEAT_ENERGY | ULTRA_FEAT_LIFETIME | \
                       ULTRA_FEAT_DIAG | ULTRA_FEAT_STANDBY | ULTRA_FEAT_LIFE_SYNC)
#define UD_HW_CAPS    (ULTRA_HWCAP_RETURN | ULTRA_HWCAP_VBUS | ULTRA_HWCAP_CURRENT | ULTRA_HWCAP_PIXEL | ULTRA_HWCAP_PIXEL_FX | \
                       ULTRA_HWCAP_CHAIN)
typedef char ud_caps_cd[UD_HW_CAPS == 0x01CDu ? 1 : -1];
#define UD_PAGE_PIXEL ULTRA_PAGE_HW        /* 0x80 */
/* PIXEL DATA (24 bytes; docs/ultradigital-firmware.md section 8.5) */
#define UD_PX_FPS10    0u                  /* u16: frames per second x 10 (the last 10 s) */
#define UD_PX_BITS     2u                  /* u24: bits of the last frame */
#define UD_PX_BITS_MIN 5u                  /* u24: the fewest since the last PIXEL answer (0xFFFFFF: none) */
#define UD_PX_BITS_MAX 8u                  /* u24: the most since then */
#define UD_PX_FRAMES   11u                 /* u24: frames since power-up (wraps: the hub takes differences) */
#define UD_PX_LONG_HIGH 14u                /* u16: long-HIGH events (the line released / undriven) */
#define UD_PX_EDGE_MS  16u                 /* u16: ms since the last edge (saturating: signal loss) */
#define UD_PX_MODE     18u                 /* u8: UD_MODE_* */
#define UD_PX_FAULTS   19u                 /* u8: delay-line faults (saturating) */
#define UD_PX_POLLS    20u                 /* u16: polls seen */
#define UD_PX_ANSWERS  22u                 /* u16: polls answered */

static const uint8_t rc_u16[11] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };   /* (rc_pack: u16 fields) */

/* STATUS 8 bits 4-7: why the board isn't plain: the first that applies */
static uint32_t light_why(void)
{
    const vdip_ev_t *d = vdip_event(&vdip, 0);
    return too_hot ? ULTRA_ST_WHY_HOT
         : (vdip.in_dip == 1u || (d != 0 && up_s - d->t_s < ULTRA_ST_DIP_RECENT_S)) ? ULTRA_ST_WHY_SUPPLY
         : ULTRA_ST_WHY_NONE;
}

static uint32_t ud_hw_id(void)
{
    return upd.info.hw_id != 0u ? upd.info.hw_id : BOARD_HW_ID;
}

/* the page's DATA into d (24 bytes, zero before); returns the PAGE to answer (ULTRA_PAGE_UNSUPPORTED: not here). POWER
 * takes its POLL's acknowledge and LIFE_SYNC its merge first (the answer shows the state after them) */
uint8_t ud_rc_page(uint8_t page, const uint8_t *arg, uint8_t *d)
{
    uint32_t i, j, v[14];
    switch (page) {
    case ULTRA_PAGE_STATUS: {
        static const uint8_t fl[14] = { 3, 1, 3, 1, 1, 2, 2, 2, 2, 2, 1, 1, 2, 1 };
        v[0] = fw_version >> 8;                          /* major, minor, patch (0: a development build) */
        v[1] = ud_hw_id();                               /* ULTRA_ST_HW_ID (0.2.7: the whole byte) */
        v[2] = up_s;
        v[3] = reset_flags >> 26;
        v[4] = pwrev_state(&pw) | light_why() << ULTRA_ST_WHY_SHIFT;
        v[5] = (uint16_t)mt.c10;
        v[6] = ULTRA_ST_NA;                              /* no board sensor */
        v[7] = vs_bus.valid ? vs_bus.mv / 10u : 0xFFFFu;
        v[8] = mt.vdda_mv;
        v[9] = sat16(i_avg);
        v[10] = (vs_bus.valid ? ULTRA_ST_M_VBUS : 0u) | ULTRA_ST_M_CURRENT;
        v[11] = 0xFFu;                                   /* no PWM (0.2.7: not applicable) */
        v[12] = 0xFFFFu;                                 /* the CONFIG CRC over no CONFIG (CRC-16/CCITT-FALSE's init) */
        v[13] = 0;                                       /* modes: none */
        rc_pack(d, v, fl, 14);
        break;
    }
    case ULTRA_PAGE_ERRORS:
        v[0] = dl.st.frames;
        v[1] = uds.headers;
        v[2] = uds.tail_err;
        v[3] = uds.decoded;
        v[4] = uds.dec_err;
        v[5] = dl.st.marks;                              /* marker frames (windows) */
        v[6] = dl.st.answered;
        v[7] = (uint16_t)(uds.polls - dl.st.answered);   /* polls not answered (a late release / main loop, a flash
                                                            operation) */
        v[8] = dl.st.cap_drops;                          /* marker frames not captured */
        v[9] = saves;
        v[10] = stack_used();
        rc_pack(d, v, rc_u16, 11);
        break;
    case ULTRA_PAGE_IDENTITY:                            /* (0.2.7) two parts: 0 the board, 1 the features */
        if (arg[0] > ULTRA_ID_PART_FEATURES) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        if (arg[0] == ULTRA_ID_PART_BOARD) {
            static const uint8_t fl[6] = { 4, 1, 1, 1, 1, 4 };
            for (i = 0; i < 12u; i++) {
                d[i] = ((const uint8_t *)HW_UID)[i];   /* the unique id, in address order */
            }
            v[0] = fw_version;
            v[1] = upd.info.bl_version;
            v[2] = ud_hw_id();
            v[3] = upd.info.hw_rev;
            v[4] = 0;                                    /* (reserved) */
            v[5] = UD_HW_CAPS;
            rc_pack(d + 12, v, fl, 6);
        } else {
            ultra_put32(d + ULTRA_IDF_FEATURES, UD_FEATURES);   /* word 0; words 1-5: none yet */
        }
        break;
    case ULTRA_PAGE_POWER:
        pwrev_ack(&pw, arg);
        pwrev_page(&pw, d);
        break;
    case ULTRA_PAGE_SUPPLY:
        vdip_page(&vdip, &vs_bus, up_s, vs_bus.valid ? vs_bus.mv : 0u, arg[0], d);
        break;
    case ULTRA_PAGE_ENERGY:
        energy_page(&en, up_s, vs_bus.mv / 10u * (uint32_t)(i_avg > 0 ? i_avg : 0) / 100u, d);
        ultra_put16(d + ULTRA_EN_VB, vs_vb.valid ? vs_vb.mv / 10u : 0xFFFFu);   /* V_B (after F4) */
        ultra_put16(d + ULTRA_EN_PEAK_MA, i_peak);
        i_peak = 0;
        break;
    case ULTRA_PAGE_LIFE_SYNC:
        ls_res = (uint16_t)energy_sync(&en, arg, rc_uid16);   /* (one merge a boot: core/energy.h) */
        /* fall through */
    case ULTRA_PAGE_LIFETIME: {
        static const uint8_t fl[11] = { 3, 3, 3, 2, 2, 3, 1, 1, 1, 2, 3 };
        static const uint8_t fs[9] = { 3, 3, 3, 2, 2, 3, 4, 2, 2 };   /* (LIFE_SYNC: Wh, result + applied, records) */
        const life_rec_t *r = &en.lt;
        v[0] = r->on_s / 360u;                           /* 0.1 h */
        v[1] = r->lit_s / 360u;
        v[2] = r->ups;
        v[3] = r->trips[0];
        v[4] = r->trips[1];
        v[5] = r->dips;
        v[6] = r->peak_t;
        v[7] = r->peak_a;
        v[8] = r->wd;
        v[9] = r->updates;
        v[10] = r->writes;
        if (page == ULTRA_PAGE_LIFE_SYNC) {
            v[6] = energy_total_wh(&en);
            v[7] = ls_res;
            v[8] = r->writes;
        }
        rc_pack(d, v, page == ULTRA_PAGE_LIFE_SYNC ? fs : fl, page == ULTRA_PAGE_LIFE_SYNC ? 9u : 11u);
        break;
    }
    case ULTRA_PAGE_DIAG:
        if (arg[0] > 1u) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        if (arg[0] == 0u) {                              /* the running board */
            static const uint8_t fl[13] = { 1, 1, 2, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1 };
            v[0] = reset_flags >> 26;
            v[1] = bc.magic == BC_MAGIC ? bc.cause : 0xFFu;
            v[2] = bc.pc;
            v[3] = bc.up_s > 0xFFFFFFu ? 0xFFFFFFu : bc.up_s;
            v[4] = cpu_pm;
            v[5] = cpu_peak;
            v[6] = 0;                                    /* LEAD: captures dropped, frame ends not handled */
            v[7] = 0;
            v[8] = ovr_late > 0xFFFFu ? 0xFFFFu : ovr_late;
            j = vdip_z_mohm(&vdip);
            v[9] = j > 0xFFFFu ? 0xFFFFu : j;
            v[10] = flash_err;
            v[11] = (uint32_t)en.due << 3 | (uint32_t)en.held << 7;   /* ULTRA_DG_FP_LIFETIME, _EXHAUSTED */
            v[12] = ULTRA_DG_SW_NONE;                    /* (no settings / calibration / scene / effects here) */
            rc_pack(d, v, fl, 13);
            d[22] = d[23] = 0xFFu;                       /* no C10 load */
        } else {                                         /* the fault memory (core/energy.h); no calibration trend */
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
                d[i] = 0x80u;                            /* ULTRA_HE_NA */
            }
            d[13] = d[14] = 0xFFu;
        }
        break;
    case UD_PAGE_PIXEL: {
        static const uint8_t fl[11] = { 2, 3, 3, 3, 3, 2, 2, 1, 1, 2, 2 };
        j = (dl.wpos - dl.edge_w) / DLINE_W_PER_MS;       /* ms since the line's last edge */
        v[0] = uds.fps10;
        v[1] = uc.mode == UDC_CHAIN ? uc.bits : dl.st.bits_last;   /* (CHAIN: TIM4 is zeroed in every gap; the stream scanner's differences are off) */
        v[2] = dl.st.bits_min;                           /* (0xFFFFFFFF: none, sent as 0xFFFFFF) */
        v[3] = dl.st.bits_max;
        v[4] = dl.st.frames;
        v[5] = dl.st.long_high;
        v[6] = j > 0xFFFFu ? 0xFFFFu : j;
        v[7] = dl.direct ? UD_MODE_DIRECT : UD_MODE_DELAYED;
        v[8] = uds.faults;
        v[9] = uds.polls;
        v[10] = dl.st.answered;
        rc_pack(d, v, fl, 11);
        dline_minmax_restart(&dl);                       /* (since the last answer) */
        break;
    }
    case ULTRA_PAGE_HW_FX:                               /* the local pixel effects (core/ud/udfx.c udfx_page), part 3: the line */
        if (arg[0] == ULTRA_PXG_PART_LINE) {
            udc_page(&uc, dl.st.answered, (uint32_t)(ux.local && !ux.dark), ms_now, d);
        } else if (udfx_page(&ux, arg[0], d) != 0) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        break;
    default:
        return ULTRA_PAGE_UNSUPPORTED;                   /* OUTPUTS, ACKS, CALIBRATION, CAL_TEST, HEALTH, FW_UPDATE, ... */
    }
    return page;
}
typedef char ud_pixel_fits[UD_PX_ANSWERS + 2u == ULTRA_RSP_DATA_LEN ? 1 : -1];

/* a valid POLL for this board (index 0, not RELAY_ONLY): the whole answer (SYNC .. CRC) into a. RFLAGS: EVENT while
 * a POWER latch is set; never CONTENTION (no read-back on this board) */
static void ud_rc_answer(const uint8_t *poll, uint8_t a[RC_ANSWER_BYTES])
{
    uint32_t i;
    uint8_t page;
    for (i = 0; i < RC_ANSWER_BYTES; i++) {
        a[i] = 0;
    }
    page = ud_rc_page(poll[RC_P_PAGE], poll + RC_P_ARG, a + ULTRA_RSP_DATA);
    rc_answer(a, poll, page, pwrev_latched(&pw) ? ULTRA_RSP_EVENT : 0u, rc_uid16);
}

/* ------------------------------------------------------------------------------- marker frames (the main loop)
 * The HSI's rate from the headers' TIME (host us at the frame end) against the local tick at the frame end: host us per
 * local tick, Q16 (rc_brr's unit; nominal 50 << 16). The reference moves only after >= 1 s (50 ppm per tick of
 * error), a gap over 30 s starts again; a rate more than 1/16 off nominal is not taken (rc_brr checks it too) */
#define UD_RATE_NOM  (50u << 16)
static uint32_t rate_q16 = UD_RATE_NOM, clk_host, clk_tick;
static uint8_t clk_have;

static void ud_clock(uint32_t host_us, uint32_t end_tick)
{
    uint32_t dt = end_tick - clk_tick, r;
    if (clk_have && dt < POLL_HZ) {
        return;                            /* (under 1 s: the reference stays, the interval grows) */
    }
    if (clk_have && dt <= 30u * POLL_HZ) {
        r = (uint32_t)div64_u((uint64_t)(host_us - clk_host) << 16, dt);
        if (r - (UD_RATE_NOM - UD_RATE_NOM / 16u) < UD_RATE_NOM / 8u) {
            rate_q16 = r;
        }
    }
    clk_have = 1;
    clk_host = host_us;
    clk_tick = end_tick;
}

/* a CRC-good slot that is no POLL: IDENTIFY 5 / 7, a local pixel effect, CONFIG 129-140 (stream and chain frames alike) */
static void ud_slot(const uint8_t *f, uint32_t host)
{
    if (f[0] == ULTRA_IDENTIFY && (f[1] >> 4) == ULTRA_LIGHT_BOARD) {
        if (f[ULTRA_PAYLOAD] == ULTRA_IDENT_POWER_OFF) {
            ud_power_off_notice();         /* (repeated slots: the same notice again, the cut's window from the last) */
        } else if (f[ULTRA_PAYLOAD] == ULTRA_IDENT_VERSION) {
            led_version();                 /* the status LED (its copies: one pattern) */
        }
    } else if (f[0] == ULTRA_PIXEL_FX) {
        udfx_pixel_fx(&ux, f, host);       /* a local pixel effect (applied once per SEQ) */
    } else if (f[0] == ULTRA_CONFIG && (f[1] >> 4) == ULTRA_LIGHT_BOARD) {
        uint32_t j;
        for (j = 0; j < 9u; j++) {         /* CONFIG 129-140 are this board's; the others are for the light boards */
            (void)udfx_config(&ux, f[ULTRA_PAYLOAD + 3u * j], ultra_get16(f + ULTRA_PAYLOAD + 3u * j + 1u));
        }
    }
}

/* a decoded marker frame, f = its bytes (nbits: 320 = one slot + the header, the only Ultra length a single-receiver
 * port uses; 384 = an UPD frame), end_tick = the tick of its last edge. Like frame_rx's Ultra path without the lock
 * (the marker says "for you", the CRCs the rest): the header (MAGIC, VERSION, CRC-8) or nothing; the clock; UPD frames to
 * the update's app side; EXT frames' slots ignored; a CRC-good slot: an RW frame's POLL (this board = index 0) gets its
 * answer built (returns 1), IDENTIFY 7 the standby notice. */
static uint32_t ud_frame(const uint8_t *f, uint32_t nbits, uint32_t end_tick)
{
    const uint8_t *h = f + nbits / 8u - ULTRA_HDR_BYTES;
    uint32_t host, flags;
    if (ultra_parse_header(h, &host) != 0) {
        uds.tail_err++;
        return 0;
    }
    uds.headers++;
    flags = ultra_header_flags(h);
    upd_app_frame_end(&upd, 1u, (uint8_t)flags, ms_now);
    if (flags & ULTRA_HDR_UPD) {
        if (nbits == UPD_FRAME_BITS && !(flags & ULTRA_HDR_RW)) {
            upd_action(upd_app_frame(&upd, f, ms_now));   /* (no clock sync from UPD frames, as frame_rx) */
        }
        return 0;
    }
    ud_clock(host, end_tick);
    udfx_anchor(&ux, host, dl_ready_w);    /* the host clock against the ring's word count (local pixel effects) */
    if (nbits != ULTRA_SLOT_BITS + ULTRA_HDR_BITS || (flags & ULTRA_HDR_EXT) || ultra_slot_check(f) != 0) {
        uds.dec_err += nbits == ULTRA_SLOT_BITS + ULTRA_HDR_BITS && !(flags & ULTRA_HDR_EXT);
        return 0;
    }
    uds.decoded++;
    if (flags & ULTRA_HDR_RW) {
        if (rc_poll_parse(f) != 0 || f[RC_P_INDEX] != 0u) {
            return 0;                      /* not a POLL, or RELAY_ONLY / another index: no other board on this port */
        }
        uds.polls++;
        ud_rc_answer(f, ud_ans);
        ud_brr = rc_brr(rate_q16, f[RC_P_RATE]);
        return 1;
    }
    ud_slot(f, host);
    return 0;
}

/* the capture of a marker frame into its bytes, in one walk (ws_decode_count): its bits counted to the frame's real
 * end, exactly 320 (a POLL / IDENTIFY: one slot + the header) or 384 (an UPD frame), else no frame, so a longer
 * frame whose first 320 bits would pass is refused; the header at the end of those bits. n_samples from the word
 * after the marker's falling edge */
static uint8_t ud_fbuf[UPD_FRAME_BYTES] __attribute__((aligned(4)));
static const ws_timing_t ud_tm = WS_TIMING_DEFAULT;

static uint32_t ud_decode(const uint16_t *cap, uint32_t words, uint32_t end_tick)
{
    int n = ws_decode_count(cap, words * 16u, (uint16_t)UPD_FRAME_BITS, &ud_tm, ud_fbuf);
    if (n == (int)(ULTRA_SLOT_BITS + ULTRA_HDR_BITS) || n == (int)UPD_FRAME_BITS) {
        return ud_frame(ud_fbuf, (uint32_t)n, end_tick);
    }
    uds.dec_err++;
    return 0;
}


/* ------------------------------------------------------------------------------- CHAIN frames (the main loop)
 * A plain Ultra frame on a normal Ultra line (dline's chain scanner captured it raw: head = the slot, tail = the header). Like frame_rx's
 * Ultra path: the header (MAGIC, VERSION, CRC-8) and the frame's length decide that it is one, the clock follows its TIME, a CRC-good slot
 * is this board's own (the first 256 bits it saw): an RW frame's POLL schedules the answer at its Ultra window slot (t0 + 1000 us + index
 * x pitch: no release, no marker), the other slot types act as in ud_frame. UPD frames (384 bits) only feed the update's frame-end notice. */
static void ud_line_apply(uint32_t act)
{
    if (act == UDC_A_CHAIN) {
        dline_ch_start(&dl, 1);            /* BLANK_N still held (BOOT, AUTO deciding): the ring zeroed, the mux switched, the encoder next; else at a latch */
        udfx_chain(&ux, 1);
    } else if (act == UDC_A_STREAM) {
        dline_ch_stop(&dl);
        dline_ch_hold(&dl, 0);             /* (the stream scanner's BOOT recovery goes on) */
        udfx_chain(&ux, 0);
    } else if (act == UDC_A_QUIET) {
        dline_ch_hold(&dl, 0);             /* silence for 2 s: no evidence. The strip is let go (the power-on effect may start), the chain
                                              scanner and the mode stay: two Ultra frames still decide CHAIN, a marker or non-Ultra frames STREAM */
    }
}

static void ud_chain_decode(void)
{
    udc_rx_t rx;
    uint32_t answer = 0, tx_w = 0, win_w = 0, act;
    const uint32_t end_w = dl.ch_end, ready_tick = dl_ch_tick;   /* (the frame's: the capture is freed inside, a later frame overwrites them) */
    (void)udc_classify(&dl, &rx, &ucp);    /* (frees the capture once the slot and the header are out: the next frame may be captured) */
    act = udc_frame(&uc, &rx, ms_now);
    if (rx.cls == UDC_C_NONULTRA) {
        uds.tail_err++;
    } else {
        uds.headers++;
        upd_app_frame_end(&upd, 1u, rx.flags, ms_now);
        if (!(rx.flags & ULTRA_HDR_UPD)) {
            ud_clock(rx.host, ready_tick);
            udfx_anchor(&ux, rx.host, end_w);
            if (rx.cls == UDC_C_SLOT && !(rx.flags & ULTRA_HDR_EXT)) {
                uds.decoded++;
                if (!(rx.flags & ULTRA_HDR_RW)) {
                    ud_slot(rx.slot, rx.host);
                } else if (rc_poll_parse(rx.slot) == 0) {
                    uc.windows++;
                    uds.polls++;
                    udc_window(rx.slot, end_w, ux.wpu24, &tx_w, &win_w);
                    if (rx.slot[RC_P_INDEX] != ULTRA_POLL_RELAY_ONLY) {
                        if (ux.r_have) {   /* (an unmeasured HSI would put the start outside the window: missed, the hub asks again) */
                            ud_rc_answer(rx.slot, ud_ans);
                            ud_brr = rc_brr(rate_q16, rx.slot[RC_P_RATE]);
                            answer = 1;
                        } else {
                            uc.answers_missed++;
                        }
                    }
                }
            } else if (rx.cls == UDC_C_HDR && !(rx.flags & ULTRA_HDR_EXT)) {
                uds.dec_err++;             /* (a right header and length, a slot that is none: a DARK slot of the preamble is no error) */
            }
        }
    }
    if (win_w == 0u && (rx.flags & ULTRA_HDR_RW)) {
        win_w = end_w + DLINE_CH_WIN_W;    /* the window is some other board's: as long as the longest one */
    }
    BARRIER();                             /* the answer complete before the window may send it */
    dline_ch_done(&dl, (int)answer, tx_w, win_w);
    ud_line_apply(act);
}

/* once a ms: the line mode's clock (AUTO's limits, a new CONFIG 140), the tick's events */
static void ud_line_service(void)
{
    const uint32_t active = dl.st.frames + dl.st.marks + dl.st.long_high;   /* (only a change counts) */
    uint8_t n;
    n = dl_ev_marker;
    if (n != ev_marker_seen) {
        ev_marker_seen++;
        ud_line_apply(udc_marker(&uc));
    }
    n = dl_ev_missed;
    if (n != ev_missed_seen) {
        ev_missed_seen++;
        uc.answers_missed++;
    }
    ud_line_apply(udc_service(&uc, udfx_line_cfg(&ux), ms_now, active));
}

/* ------------------------------------------------------------------------------- the console (USART2 TX, PA2)
 * Output only (as the release console): the banner, a line of live values every second, a status line every minute
 * (the first at 5 s), the firmware's messages. Each only while the UART has the room: the main loop never waits.
 * A '%' in a format takes the next value (signed decimal); the texts have no other '%'. TODO: the test jig (TP6). */
#define CON_LINE 160u

static uint32_t con_fmt(char *b, const char *f, const int32_t *v)
{
    char *p = b, t[11];
    for (; *f; f++) {
        if (*f != '%') {
            *p++ = *f;
            continue;
        }
        {
            int32_t x = *v++;
            uint32_t u = x < 0 ? 0u - (uint32_t)x : (uint32_t)x, k = 0;
            if (x < 0) {
                *p++ = '-';
            }
            do {
                t[k++] = (char)('0' + u % 10u);
                u /= 10u;
            } while (u != 0u);
            while (k != 0u) {
                *p++ = t[--k];
            }
        }
    }
    *p = 0;
    return (uint32_t)(p - b);
}

static void con_line(const char *f, const int32_t *v)
{
    char b[CON_LINE];
    (void)con_fmt(b, f, v);
    uart_write(b);
}

static void con_banner(void)
{
    int32_t v[6];
    v[0] = (int32_t)ud_hw_id();
    v[1] = upd.info.hw_rev;
    v[2] = upd.info.bl_version;
    v[3] = (int32_t)(reset_flags >> 26);
    v[4] = upd_res;
    v[5] = bl_prot;
    uart_write("\r\nQuinLED Diff-Power Ultra-Digital receiver ");
    uart_write(fw_version != 0u || UPD_APP ? fw_version_text : FW_VERSION " (development build)");
    con_line("\r\nhw=% rev=% bl=% reset=% upd=% prot=%\r\n", v);
}

/* every ms in the main loop */
static void console_service(void)
{
    int32_t v[11];
    if (con_flash_err && uart_room() >= 16u) {
        con_flash_err = 0;
        uart_write("flash: error\r\n");
    }
    if (ms_now - con_ms < 1000u || uart_room() < CON_LINE) {
        return;
    }
    con_ms = ms_now;
    if (up_s >= 5u && (con_status_s == 0u || up_s - con_status_s >= 60u)) {
        con_status_s = up_s;
        v[0] = (int32_t)(en.lt.on_s / 360u);
        v[1] = (int32_t)en.lt.ups;
        v[2] = (int32_t)energy_total_wh(&en);
        v[3] = (int32_t)en.lt.writes;
        v[4] = vdip.count;
        v[5] = flash_err;
        v[6] = saves;
        v[7] = (int32_t)stack_used();
        v[8] = cpu_peak;
        v[9] = hw_diag[3] + hw_diag[4];             /* DMA1 ch2 / ch3 (the ring's) transfer errors */
        v[10] = (int32_t)ovr_late;
        con_line("status: on_h10=% ups=% wh=% writes=% dips=% flash_err=% saves=% stack=% cpu_peak=% dma_err=% "
                 "late_ms=%\r\n", v);
        return;
    }
    v[0] = (int32_t)up_s;
    v[1] = (int32_t)vs_bus.mv;
    v[2] = (int32_t)vs_vb.mv;
    v[3] = i_avg;
    v[4] = mt.vdda_mv;
    v[5] = mt.c10;
    v[6] = uds.fps10;
    v[7] = (int32_t)dl.st.bits_last;
    v[8] = dl.direct ? UD_MODE_DIRECT : UD_MODE_DELAYED;
    v[9] = uds.faults;
    v[10] = uds.polls;
    con_line("up=% bus=% vb=% i=% vdda=% t=% fps10=% bits=% mode=% faults=% polls=%\r\n", v);
}

/* ------------------------------------------------------------------------------- main */
int main(void)
{
    uint32_t last_ms, life_found, ufx_found;
    uint8_t ufx_rec[UX_REC_BYTES];

    stack_paint();
    bc = bcrumb;                           /* the last run's breadcrumbs (before the tick runs), then this run's */
    bcrumb.magic = BC_MAGIC;
    bcrumb.up_s = bcrumb.cause = 0;
    /* the records in one window before hw_init (hw.c hw_records_window: HCLK / 2 for the upper CodeFlash) */
    hw_records_window(1);
    life_found = life_load(&en.lt);
    ufx_found = ufx_load(ufx_rec);
    hw_records_window(0);
    pwrev_init(&pw, ms_now);
    hw_init();                             /* the pins first: mux direct, strip held LOW, receive, MOSI LOW */
    reset_flags = hw_reset_flags();
    upd_boot();                            /* UPD_APP: the reset flags from the hand-over, the bootloader's result; (round
                                              11, B2) first, its hardware id decides the delay line */
    udfx_init(&ux);                        /* the local pixel effects: the stored record (power-on effect, strip settings) */
    if (ufx_found && udfx_rec_valid(ufx_rec)) {
        udfx_load(&ux, ufx_rec);
    }
    udc_init(&uc, udfx_line_cfg(&ux), ms_now);   /* CONFIG 140 from the record: AUTO (0) unless forced */
    if (!upd_app_wrong_hw(&upd.info, BOARD_HW_ID)) {
        dl_hw_start();                     /* the delay line runs (the mux direct, BLANK_N LOW): dline's BOOT switches
                                              both at the first quiet gap */
        dline_init(&dl, dl_hw_rx_pos());
        if (uc.mode == UDC_CHAIN) {        /* forced CHAIN: the ring zeroed and the mux switched at the first tick, no waiting for a frame */
            dline_ch_start(&dl, 1);
            udfx_chain(&ux, 1);
        } else if (uc.mode == UDC_UNDECIDED) {   /* AUTO: the strip held (BOOT's release waits) until two Ultra frames or the markers say */
            dline_ch_hold(&dl, 1);
            dline_ch_start(&dl, 0);
            udfx_chain(&ux, 2);
        }
        BARRIER();
        dl_on = 1;
    }                                      /* else (another board's bootloader: an SWD mis-flash) the pins stay as
                                              hw_init set them: the strip held LOW, the mux direct, DE low; no ring,
                                              no capture, no answer */
    led_lvl = 1;
    led_set(1);                            /* the status LED: on = running (its codes: led_service) */
    rc_uid16 = crc16_ccitt((const uint8_t *)HW_UID, 12);
    mcutemp_init(&mt);
    vsense_init(&vs_bus, VSENSE_SUPPLY_RT, VSENSE_SUPPLY_RB, VSENSE_VREF_MV);
    vsense_init(&vs_vb, VSENSE_AUX_RT, VSENSE_AUX_RB, VSENSE_VREF_MV);
    hw_adc_init();                         /* PB0 continuously, the injected group from the poll tick */
#if UPD_APP
    fw_version = upd.info.desc_ok ? upd.info.version : 0u;
#else
    fw_version = fwver_parse(FW_VERSION);
#endif
    fwver_format(fw_version, fw_version_text);
    energy_init(&en, life_found, ms_now, fw_version, reset_flags & RCC_IWDGRSTF);
    en.ev_save = 1u;                       /* (as BOARD 2: a standby cycle writes nothing for itself, core/energy.h) */
    if (reset_flags & RCC_IWDGRSTF) {      /* the fault memory: the last abnormal reset, with that run's breadcrumbs */
        en.lt.rst = (uint8_t)(bc.magic == BC_MAGIC && bc.cause != 0u ? 2u | (bc.cause & 63u) << 2 : 1u);
        en.lt.rst_h = (uint8_t)(bc.magic == BC_MAGIC ? (bc.up_s / 3600u > 255u ? 255u : bc.up_s / 3600u) : 0u);
    }
    flash_erase_ahead();                   /* the lifetime record's next page, erased at power-up (the dip write's) */
    flash_boot = 0;
    con_banner();
    last_ms = sec_ms = con_ms = ms_now;

    for (;;) {
        uint32_t fx_more;
        watchdog_kick();
        dl_main();
        fx_more = udfx_render(&ux);        /* one chunk of the next local frame (<= 32 pixels); 1: more to do */
        uart_kick();
        if (ms_now != last_ms) {
            uint32_t m = ms_now, dm = m - last_ms;
            if (ovr_on) {
                ovr_late += dm - 1u;       /* the ms late (not after a flash operation) */
            }
            ovr_on = 1;
            last_ms = m;
            led_service(dm);
            current_service();
            power_service();
            vsense_service();
            upd_service();
            udfx_service(&ux, &dl, ms_now, i_avg > 0 ? (uint32_t)i_avg : 0u, ux_vk());
            ud_line_service();
            if (vdip.planned && ms_now - plan_ms >= PLAN_CUT_MS) {
                vdip.planned = 0;          /* still powered: the notice forgotten */
            }
            if (ms_now - sec_ms >= 1000u) {
                sec_ms += 1000u;           /* (whole seconds: the uptime) */
                second_service();
            }
            flash_erase_ahead();
            ufx_flash_service();
            console_service();
        }
        /* nothing left: the core sleeps until the next interrupt (at the latest the tick, 50 us); DMA, timers, ADC
         * and the UART run on (main.c). The CPU load: stamps before and after it */
        if (ms_now == last_ms && !fx_more) {
            uint32_t k = tick_now, c0 = POLL_CNT(), c1;
            CPU_SLEEP();
            c1 = POLL_CNT();
            cpu_idle += cpu_slept(k != tick_now, c0, c1, POLL_CYC);
        }
    }
}
