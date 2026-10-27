/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <string.h>
#include "../board/ur_digital_m/descriptor_gen.h"
/* Diff-Power-UR-Digital-M firmware (BOARD=3; docs/ur_digital_m-firmware.md).
 *
 * No light engine: the strip's data passes the board through a 227.6 us delay line (mcu/ch32v20x/hw_dline.c: SPI1 full duplex
 * over a DMA ring; the window logic: core/dline.c), and the board answers the hub's polls on a pixel line with the
 * Diff-Power-UR-Analog-M's pages + 0x80 PIXEL. On a plain controller (never polled) it is transparent.
 *
 * Milestone 2 (this file): the shell. The time base and the ADC (20 kHz tick), the records (the lifetime record), the
 * bus / V_B / current with the VREFINT scale, the supply dips, energy and LIFE_SYNC, the flash policy (main.c's,
 * reduced), the output-only console, the status LED, the firmware update's app side, and the answers' page builder
 * (ud_rc_page). The delay line's logic is not wired: its hooks (dl_isr_fault, dl_tick, dl_ram_tick, dl_main, ud_busy)
 * are stubs marked LEAD, and the mux stays direct, so the strip gets RO as it is. */
#include <stddef.h>
#include "../mcu/device.h"
#include "hw.h"
#include "hw_dline.h"
#include "../core/ack.h"
#include "../core/console_record.h"
#include "../core/bl_protection.h"
#include "../core/cli.h"                   /* bcrumb_t (the reset breadcrumbs: isr.S writes them too) */
#include "../core/cpuload.h"
#include "../core/dline.h"
#include "../core/ur_digital_m/udfx.h"
#include "../core/ur_digital_m/udchain.h"
#include "../core/ur_digital_m/udcal.h"
#include "../core/math32.h"
#include "../core/fwver.h"
#include "../core/lto.h"
#include "../core/mcutemp.h"
#include "../core/pwrev.h"
#include "../core/rc.h"
#include "../core/upd_app.h"
#if MCU_CH32V40X
#include "../mcu/ch32v40x/upd_forward.h"
#else
#include "../mcu/ch32v20x/upd_forward.h"
#endif
/* never on a marker port (one receiver); open and close only after >= 350 us of LOW without an edge (the marker quiet
 * rule: above the 280 us reset and the 250 us late-release limit, inside every >= 500 us inter-frame LOW) */
#define UPD_FWD_WANTED() (upd.transparent && dl.ch_on)
#define UPD_FWD_QUIET_W  MCU_US2W(UPD_FORWARD_QUIET_US)
static volatile uint8_t upd_forward_wanted;  /* main: UPD transparency on a CHAIN line (UPD_FWD_WANTED) */
static uint8_t upd_forward_open;             /* the tick: FWD_EN as set */
/* The board at any position of a CHAIN line (docs/ur_digital_m-firmware.md section 12): the gate holds this board's slot and forwards the
 * rest (the slot plan, mcu upd_forward.h), the boards behind answer through its relay window. Main sets gate_slot once CHAIN is decided (AUTO
 * keeps the gate closed until then: unidentified traffic stays blocked; STREAM forwards nothing). The tick re-arms the plan after
 * ULTRA_RW_REARM_US of quiet LOW after a frame (past the hub's release: a re-arm before it would hold the released HIGH from the boards
 * behind), never in a window, never while update-transparent, and only while the line is LOW with no edge since the tick read TIM4 */
#define GATE_REARM_W MCU_US2W(ULTRA_RW_REARM_US)
static volatile uint8_t gate_slot;
#include "../core/vdip.h"
#include "../core/vsense.h"
#include "../core/ws_decode.h"
#include "../core/link_stats.h"
#include "../core/upd_platform.h"
#include "../core/ur_digital_m/udpack.h"
#ifdef MCU_HSI_BOOT_TRIM_STEPS
#include "../core/hsitrim.h"
#endif
#include "../core/ur_digital_m/udjig.h"
#include "../core/console_events.h"
#define UD_JIG BOARD_TESTJIG              /* the test jig mode (core/ur_digital_m/udjig.h) */

#if BOARD != BOARD_UR_DIGITAL_M
#error "main_ur_digital_m.c is the Diff-Power-UR-Digital-M's app: BOARD=3"
#endif
const char firmware_build_identity[] = "0.3 W3 NOT DEPLOYABLE";
#ifndef FW_VERSION
#define FW_VERSION       "0.7.1"   /* release builds: the release version (one for every board of the tree) */
#endif
#ifndef UPD_APP
#define UPD_APP 0                  /* 1: linked behind the bootloader (src/mcu/ch32v20x/Link_app.ld, build_release.sh) */
#endif
#define UPD_WD_CLEAR_S   10u       /* the bootloader's watchdog-loop count is cleared after this long running */
#define PLAN_CUT_MS      10000u    /* (as BOARD 2) a dip starting this long after IDENTIFY 7 is the planned cut's */
#define TEMP_HOT_C10     950       /* the die at 95.0 C: STATUS's reason HOT (a warning: nothing changes) ... */
#define TEMP_HOT_HYST_C10 100      /* ... until it is back under 85.0 C */
#define POLL_CYC (MCU_HCLK_HZ / POLL_HZ)   /* HCLK cycles per poll tick (SysTick's period, hw_cold.c) */
/* Firmware model and pin-role contract. */
/* mA per ADC count, Q16, at vdda mV full scale: 65536 x 1000 / 4095 = 16004, over the sense path's mV per A */
#define UD_MA_Q16(vdda)  (((uint32_t)(vdda) * 16004u + ISENSE_MV_PER_A / 2u) / ISENSE_MV_PER_A)

/* ------------------------------------------------------------------------------- shared with the poll ISR */
static volatile uint32_t ms_now;          /* 1 ms time base */
volatile uint32_t tick_now;               /* 50 us time base, also read by the SRAM flash deadline */
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
#include "ur_digital_m_stats.h"
static ur_digital_m_stats_t uds;

/* the window logic (core/dline.c) and its glue. dline decides from the ring alone (the tick's scan); the glue applies its
 * actions to the pins and the UART and hands the capture to the main loop. Contexts: the tick (dl_tick), the SRAM tick
 * during a flash operation (dl_ram_tick), USART1's TC interrupt (dl_tx_done: DE is low), SPI1's error interrupt
 * (dl_isr_fault), the main loop (dl_main: decode, answer, dline_arm). */
/* dl, ux and the effect engine (udfx.c ux_eng) sit right above gp (Link_sections.ld .bss.gp.*, build_target.sh QL_GP_BSS): their
 * addresses take one instruction (the linker relaxes a lui / lo12 pair to gp-relative only when the whole object is in range: each
 * is under 2 KB, dl's capture is the array below). ld checks the range; tools/size_gate.py measures the result */
static dline_t dl __attribute__((section(".bss.gp.0dl")));
static uint16_t dl_cap[DLINE_CAP_WORDS] __attribute__((aligned(4)));   /* dline's capture buffer (dline_init) */
static udfx_t ux __attribute__((section(".bss.gp.1ux")));   /* the local pixel effects (core/ur_digital_m/udfx.h, docs/ur_digital_m-firmware.md section 11) */
static udcal_t ucal;
static struct {
    uint32_t frame;
    uint16_t rendered;
    uint8_t owned, was_local, faults, lost, turn, take, clear_seq, clear_have;
    volatile uint8_t contaminated;        /* tick ORs; main takes with the ADC batch under IRQ mask */
} cal_io;
static udc_t uc;                           /* the line mode: STREAM / CHAIN / AUTO deciding (core/ur_digital_m/udchain.h, section 12) */
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
static uint16_t ud_brr;                    /* USART1's divider for the armed answer (ud_frame) */

/* SPI1 OVR (hw_dline.c dl_spi1_isr, above the tick): RX stopped, TX replays a stale ring: the strip line LOW at once
 * (polls and faults stay invisible on the strip), the tick restarts the ring */
void dl_isr_fault(void)
{
    dl_hw_blank(1);
    dl_fault_pend = 1;
}

/* USART1 TC (mcu/ch32v20x/hw.c rc_uart_isr, BOARD 3): the burst's last stop bit is out, DE low now: the exact word where RO comes
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
    DL_LIVE_DELAY();
    return hi | (uint32_t)(dl_hw_edges() != (uint16_t)edges);
}

/* dline's pin actions. LIVE_CHECK (a switch or a release that the visible direct path would show): only while the line is
 * still LOW and no edge came since the tick read TIM4 (both paths LOW: nothing repeated or cut), else none of them and
 * dline is told (asked again later). One copy in SRAM: the SRAM tick needs it there, and the 20 kHz tick calls it too
 * (only on a tick with actions: a switch, a window, a fault) */
RAMCODE static uint32_t dl_pins(uint32_t a, uint32_t edges)
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

/* the slot plan's re-arm (dl_tick, after ULTRA_RW_REARM_US of quiet LOW): not in a window, not while update-transparent, the chain
 * scanner idle, not armed already, and the line still LOW with no edge since the tick read TIM4 (a stray edge after it re-arms again
 * at the next quiet LOW) */
static NOINLINE void ud_gate_rearm(uint32_t e)
{
    if (!upd_forward_open && !dl.ch_rel && dl.ch_st == DLINE_CH_IDLE && !gate_slot_armed() && !dl_live_moved(e)) {
        gate_slot_arm();
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
    /* Main publishes UPD transparency; only the sampler's complete LOW run
     * and the live edge check may open the physical downstream gate, and
     * close it (the idle close comes at any time, and a HIGH cut there would
     * reach the next board): nothing writes it while DIN is HIGH. */
    if (upd_forward_open != upd_forward_wanted && dline_low_run(&dl) >= UPD_FWD_QUIET_W && !dl_live_moved(e)) {
        upd_forward_open = upd_forward_wanted;
        upd_forward_set(upd_forward_open);
    }
    if (dl.ch_lz >= GATE_REARM_W && gate_slot) {   /* (the LOW run first: short in a frame) */
        ud_gate_rearm(e);                  /* (out of line: the tick's common path keeps its registers) */
    }
    if (a != 0u) {                         /* (a frame's ticks have no action: none of the tests below) */
        if (a & DLINE_A_CH_TURN) {
            upd_forward_set(0);                /* closed for the window: the relayed answers are edges, a plan would forward them */
            if (rc_hw_relay(1)) {
                dl.ch_arm = 0;                 /* the line above still driven: no relay, no answer in this window */
                uc.relay_busy++;
            }
        }
        if (a & DLINE_A_CH_FWD) {
            (void)rc_hw_relay(0);              /* (the re-arm rule takes the gate from here) */
        }
        if (a & DLINE_A_SEND) {
            if (udfx_answer_blank(&ux)) {
                rc_hw_dir(RC_DIR_ANSWER);      /* BLANK_N LOW, then DE high (mcu/ch32v20x/hw.c) */
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
    }
    udfx_tick(&ux, &dl, dl_ring);          /* local pixel effects: the mode switches and the encoder's fill, after the scan */
#if defined(MCU_SOURCE_SWITCH_LOW_W)
    dl_hw_source(dl.local);                /* the strip's source follows the mode: the serialiser while local, else the replay
                                              (udfx switches only where both are LOW) */
#endif
    if (ux.calibration && (!dline_idle(&dl) || (dl.ch_win && (int32_t)(dl.ch_win-dl.wpos)>0))) {
        cal_io.contaminated=1;
    }
}

/* the same during a flash operation, from SRAM (hw_cold.c flash_op: SPI1's and USART1's interrupts are off): the scan
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
#if !LEAN_TELEMETRY
static pwrev_t pw;                         /* POWER: over the link (core/pwrev.h) */
#endif
static upd_app_t upd;                      /* firmware update, app side (core/upd_app.h) */
#if UPD_APP
#define HO ((upd_handover_t *)UPD_HANDOVER_ADDR)  /* outside every linker section (Link_app.ld) */
static uint8_t upd_wd_cleared;
#endif
static uint8_t upd_res, bl_prot;           /* the bootloader's last result (UPD_RES_*) and protection report (0 none,
                                              1 bad option bytes, 2 ok, 3 failed): the console; LEAD: the LED's codes */
static uint32_t fw_version;                /* core/fwver.h: the descriptor's in release builds; 0 = development build */
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
static uint32_t cal_refill_ms;
static uint8_t cal_tokens=4;
/* CPU load (core/cpuload.h) and the main loop's late ms (DIAG, the console's c record) */
static uint32_t cpu_idle, cpu_t0, ovr_late;
static uint16_t cpu_pm, cpu_peak;
static uint8_t ovr_on;
static uint16_t saves;                     /* flash records written (ERRORS) */
static uint8_t con_flash_err;              /* the console's "flash: error" is due */
static uint32_t con_ms, con_status_s;
static uint8_t con_part;
#if MCU_CH32V40X
static uint32_t con_fp;                    /* the settings' fingerprint (t, p 4..7, the board limit): CONFIG_CHANGED */
static uint8_t con_cfg;                    /* ... its X record is due */
typedef char con_fp_span[offsetof(udfx_t, input_limit_ma) == offsetof(udfx_t, cfg) + sizeof(uint16_t) * (ULTRA_CFG_PX_COUNT + 1u) ? 1 : -1];
#endif
static void con_message(uint32_t value);

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
static uint8_t flash_err;                  /* failures in a row (DIAG, the console's c record) */
static uint8_t flash_boot = 1, flash_recheck = 1;

static uint32_t flash_held(void)
{
    return flash_err && ms_now - flash_err_ms < (FLASH_RETRY_MS << (flash_err - 1u));
}

static uint32_t flash_may(uint32_t need_erase)
{
    return !udcal_active(&ucal) && !ux.calibration && !ud_busy() && !uart_busy() && !flash_held() && !hw_vdd_low() &&
           !(vdip.in_dip && (need_erase || !FLASH_DIP_WRITES)) &&
           (flash_boot || udfx_flash_ok(&ux));   /* (an erase stalls the render for 16 ms: only on the stream or in the dark) */
}

/* a flash operation ran, with result r (core/flashrec.h): 1 = written. The ms it delayed is not late (ovr_on is read
 * only at the main loop's ms step, after every operation) */
static uint32_t flash_done(uint32_t r)
{
    ovr_on = 0;
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
        (void)flash_done(life_prepare());
        return;
    }
    flash_recheck = 0;
}

/* the local-effects record (core/ur_digital_m/udfx.h): written 2 s after the last change (a burst of CONFIG slots is one record) and at
 * most every 30 s, only on the hub's stream or in the dark (the render never pauses under a lit effect); its next page is
 * erased ahead in such a moment too */
static uint32_t ufx_last_ms;
static void ufx_flash_service(void)
{
    if ((ucal.model.flags&ULTRA_UCF_CLEAR_PENDING) || !udfx_flash_ok(&ux) || !flash_may(1u)) {
        return;
    }
    if (ux.save_due) {
        if (ms_now - ux.save_ms >= 2000u && (ufx_last_ms == 0u || ms_now - ufx_last_ms >= 30000u)) {
            if (flash_done(ufx_store(&ux))) {
                udfx_rec_saved(&ux);
            }
            ufx_last_ms = ms_now | 1u;
        }
    } else if (ufx_pending() == 1u) {
        (void)flash_done(ufx_prepare());
    }
}

static void cal_flash_service(void)
{
    /* A program/erase never creates its own dark frame. Stream darkness cannot
     * be proved here, so only a latched local black frame admits this store. */
    if(!(ucal.model.flags&ULTRA_UCF_PENDING) || !flash_may(1u)) return;
    if(ms_now-cal_refill_ms>=21600000u) {
        uint32_t n=(ms_now-cal_refill_ms)/21600000u;
        cal_refill_ms+=n*21600000u;
        cal_tokens=(uint8_t)(n+cal_tokens>4u ? 4u:n+cal_tokens);
    }
    if(!cal_tokens) return;
    cal_tokens--;
    if(flash_done(ucal_store(&ucal))) ucal.model.flags&=0x7Du; /* pending + clear_pending */
}

static uint32_t life_admit(void)
{
    return life_pending() == 2u || !flash_may(0u) ? 0u : ENERGY_DUE_SAVE | ENERGY_DUE_DIP;
}

/* ------------------------------------------------------------------------------- the measurements */
/* every ms: the bus current of that ms (the tick's samples), its average and peak */
static void current_service(void)
{
    uint32_t s, n, contaminated;
    __disable_irq();
    s = cur_sum;
    n = cur_n;
    cur_sum = cur_n = 0;
    contaminated=cal_io.contaminated;
    cal_io.contaminated=0;
    __enable_irq();
    udcal_feed(&ucal,s,n,i_q16,ms_now,contaminated,
               vs_bus.valid ? vs_bus.mv:0u,vs_vb.valid ? vs_vb.mv:0u);
    if (n != 0u) {                         /* (none: a flash operation's ms) */
        i_ms = (int32_t)(s * (i_q16 >> 4) / n >> 12);   /* (<= 21 x 4095 x 7.4k: 32 bits) */
    }
    i_acc += i_ms - (i_acc >> 6);
    i_avg = i_acc >> 6;
    if (i_avg > (int32_t)i_peak) {
        i_peak = sat16(i_avg);
    }
}

#if !LEAN_TELEMETRY
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
#endif

/* every ms: the injected group's pairs of that ms: the bus and V_B, the dips, the energy and the lifetime record */
static void vsense_service(void)
{
    uint32_t sum2[2], n, was, cut;
    hw_vsense_take(sum2, &n);
    /* The temperature conversion replaces one injected group each second.
     * Require a fresh voltage window, not a pair in that deliberate gap. */
    if(vs_bus.ms==VSENSE_WIN_MS-1u && !(vs_bus.n+n)) udcal_abort(&ucal,ULTRA_UCE_SUPPLY);
    vsense_add(&vs_bus, sum2[0], n);
    vsense_add(&vs_vb, sum2[1], n);
    was = vdip.in_dip;
    vdip_ms(&vdip, ms_now, up_s, sum2[0], n, i_ms);
    if (was == 1u && !vdip.in_dip) {       /* the fault memory: a dip ended (not the planned cut's) */
        const vdip_ev_t *d = vdip_event(&vdip, 0);
        if (d != 0 && d->ref_x != 0u) {
            en.lt.dip_pct = (uint8_t)((uint32_t)d->min_x * 100u / d->ref_x);
            en.lt.dip_cs = (uint8_t)(d->dur_ms / 10u > 255u ? 255u : d->dur_ms / 10u);
            en.lt.dip_a = (uint8_t)(d->i_ma / 500u > 255u ? 255u : d->i_ma / 500u);
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
        if (flash_done(life_store(&en))) {
            energy_saved(&en, up_s);
        }
    }
}

/* IDENTIFY 7, the hub's planned power-off notice (LEAD: the frame's handler calls it): a dip starting within
 * PLAN_CUT_MS is the planned cut's (core/vdip.h in_dip 2: no dip counted, the record written ENERGY_PLANNED) */
void ud_power_off_notice(void)
{
    udcal_abort(&ucal,ULTRA_UCE_CANCELLED);
    vdip.planned = 1;
    plan_ms = ms_now;
}

/* once a second: the conversion started a second ago (MCU temperature, VREFINT), the next one started. VDDA is the
 * 3.3 V supply (+-1.5 %, its ripple under the reply burst): every channel is scaled by VREFINT (design note section 6;
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

/* firmware model */
static uint32_t ux_vk(void)
{
    uint32_t k = 256u;
    if (vs_bus.valid && vs_vb.valid && vs_bus.mv != 0u) {
        k = vs_vb.mv * 284u / vs_bus.mv;
        k = k < 16u ? 16u : k > 1024u ? 1024u : k;
    }
    return k;
}

/* the raw frames (the calibration's patterns, the update's black frame): the encoder's buffer is taken, a frame handed over,
 * the normal frames restored; one copy each for cal_work and upd_action */
static NOINLINE uint32_t enc_idle(void)         /* the encoder is in LOW with a whole reset out, nothing handed over */
{
    const ws_enc_t *e = &ux.e;
    BARRIER();
    return !e->ready && e->state == WS_LOW && e->low_w >= e->reset_w + WS_RING_WORDS;
}

static NOINLINE void raw_take(void)             /* the buffer is ours: LOCAL asked for, the first frame a whole reset away */
{
    ux.e.ready = 0;
    ux.first_w = dl.wpos + ux.chip.reset_w;
    BARRIER();
    if (!ux.local) ux.want_local = 1;
}

static NOINLINE void raw_frame(uint32_t nbytes) /* a raw frame of nbytes handed to the encoder: now, no grid */
{
    ws_enc_t *e = &ux.e;
    e->period_w = 0; e->next_w = dl.wpos; e->nbytes = nbytes;
    cal_io.frame = e->frames;
    BARRIER();                                  /* the frame and its metadata complete before the tick takes ready */
    e->ready = 1;
}

static NOINLINE void raw_done(void)             /* the normal frames again: the configured length on the grid, rendered anew */
{
    ws_enc_t *e = &ux.e;
    e->nbytes = ux.npx * ux.chip.bpp; e->period_w = ux.period_w;
    ux.begun = 0; ux.dark = 0; ux.black_sent = 0;
}
#if UD_JIG
/* The test jig (core/ur_digital_m/udjig.h). Its pattern takes the strip as a calibration does (cal_work: the raw frame's
 * owner, the same take and restore; never both: the jig waits for a calibration's restore, one requested while it runs
 * fails BUSY; an update's take wins). Frames keep being decoded, answered and applied (PIXEL_FX, CONFIG: the scene renders them
 * afterwards); the strip shows only the pattern until the pad is released, then one black frame of its pixels, latched */
static udjig_t tj;
enum { TJ_NONE = 0, TJ_TAKEN, TJ_LIVE, TJ_BLACK };
static uint8_t tj_st;                      /* the jig's hold on the strip (cal_io.owned: the take's): taken, the pattern on
                                              it (the cycle runs), the release's black frame handed over */
#define TJ_BUSY() (tj.on || tj_st)
typedef char tj_say_ids[CZ_m_tj_off == CZ_m_tj_on + 1 ? 1 : -1];
#else
#define TJ_BUSY() 0
#endif

/* A job owns only the wire buffer. The normal scene is advanced in separate
 * preparation slices and is rendered at the current host time on restoration.
 * No operation here waits on the encoder or performs flash work. */
static __attribute__((noinline)) uint32_t cal_work(void)
{
    ws_enc_t *e=&ux.e;
    uint32_t p,j,end;
#if UD_JIG
    if(TJ_BUSY()) udcal_abort(&ucal,ULTRA_UCE_BUSY);   /* (a calibration under way restores first; one requested meanwhile fails) */
#endif
    if(!udcal_active(&ucal)) {
        if(cal_io.owned==2u) {
            if(cal_io.take || vdip.in_dip || vdip.planned) return 0;
            ux.calibration=0;cal_io.owned=0;
            if(!cal_io.was_local && !ux.chain) {BARRIER();ux.want_stream=1;}
        }
        /* Acquisition scans both edges and must release the mailbox before
         * the next frame. Render in its gaps; the encoder keeps the current
         * complete output frame while a new one is being prepared. */
        BARRIER();
        if(!TJ_BUSY() || cal_io.take) {
            if(dl.ch_cap && dl.ch_st==DLINE_CH_FRAME && dl.ch_profile>=4u) return 0;
            return udfx_render(&ux);
        }
    }
    if(ud_busy()) return 0;
    if(!cal_io.owned) {
        cal_io.was_local=ux.local;
        cal_io.owned=1; cal_io.faults=uds.faults;cal_io.lost=e->lost;
        ux.calibration=1; ux.want_stream=0; ux.nudge=0;
        raw_take();
#if UD_JIG
        tj_st=(uint8_t)!udcal_active(&ucal);   /* (TJ_TAKEN, or a calibration's take) */
        if(tj_st) return 1;
#endif
        if(ucal.phase!=ULTRA_UCS_RESTORE) ucal.phase=ULTRA_UCS_ACQUIRE;
        return 1;
    }
#if UD_JIG
    if(tj_st) {
        ux.want_stream=0;                 /* (held: a line-mode change may hand the strip back: it is asked for again) */
        BARRIER();
        if(!ux.local) {ux.want_local=1;return 0;}
        if(!enc_idle()) return 0;
        if(tj_st==TJ_BLACK) {
            if(e->frames==cal_io.frame) return 0;
            goto restore;                 /* the black frame latched */
        }
        tj_st=(uint8_t)(tj.on ? TJ_LIVE : TJ_BLACK);
        raw_frame(uj_frame(&tj,ux.pack.ord,ux.chip.bpp,ux_pool));
        return 1;
    }
#endif
    if(!ux.local) return 0;
    if(cal_io.faults!=uds.faults || cal_io.lost!=e->lost || dl_fault_pend) {
        udcal_abort(&ucal,ULTRA_UCE_ENCODER);
        cal_io.faults=uds.faults;cal_io.lost=e->lost;
    }
    if(ucal.pending) {
        if(!enc_idle()) return 0;
        if(ucal.pending==2u) {
            if(e->frames!=cal_io.frame) udcal_latched(&ucal,ms_now);
            return 0;
        }
        if(ucal.pending==1u) {cal_io.rendered=0;ucal.pending=3;}
        end=cal_io.rendered+16u;
        if(end>ucal.cap) end=ucal.cap;
        for(p=cal_io.rendered;p<end;p++) {
            uint8_t value=udcal_pixel(&ucal,p);
            for(j=0;j<ux.chip.bpp;j++)
                ux_pool[p*ux.chip.bpp+j]=(ucal.mask & (1u<<ux.pack.ord[j])) ? value : 0;
        }
        cal_io.rendered=(uint16_t)end;
        if(end==ucal.cap) {
            /* Keep the receiver's nominal/feedback ceiling as a second gate.
             * Raw bytes must never be silently clamped and then accepted as the
             * requested duty. The measured bound in udcal is the primary gate. */
            uint32_t channels=0,ma_ch=ux.cfg[ULTRA_CFG_PX_MA_CH-ULTRA_CFG_PX_FIRST];
            uint32_t nominal;
            for(j=0;j<ux.chip.bpp;j++) channels+=(ucal.mask>>j)&1u;
            if(!ma_ch) ma_ch=20;
            /* K<=32, channels<=5, duty<=255, ma_ch<=65535: first product fits u32. */
            nominal=m32_muldiv(ucal.k*channels*ucal.duty*ma_ch,ux_vk(),255u);
            if(ucal.duty && nominal>(uint32_t)ucal.limit*ux.cl_q8) {
                udcal_abort(&ucal,ULTRA_UCE_LIMIT);return 1;
            }
            ucal.pending=2;
            raw_frame(ucal.cap*ux.chip.bpp);
        }
        return 1;
    }
    if(ucal.phase==ULTRA_UCS_RESTORE && ucal.finish) {
#if UD_JIG
restore:
#endif
        /* No encoder reads the buffer until ready; all old test pixels are black. */
        raw_done();
        e->next_w=dl.wpos+e->reset_w;
        if(ux.lim_bri>16u) ux.lim_bri=16;   /* conservative first restored frame; normal limiter recovers */
        cal_io.owned=(uint8_t)(cal_io.take || vdip.in_dip || vdip.planned ? 2u:0u);
        ux.calibration=(uint8_t)(cal_io.owned!=0u);
        if(!cal_io.owned && !cal_io.was_local && !ux.chain) {BARRIER();ux.want_stream=1;}
#if UD_JIG
        tj_st=TJ_NONE;
        if(udcal_active(&ucal))           /* (the jig's restore: no calibration ran, unless one was requested and failed) */
#endif
        udcal_restored(&ucal);
        return 1;
    }
    if((cal_io.turn^=1u)!=0u) udcal_step(&ucal,i_q16);
    else {e->next_w=dl.wpos;(void)udfx_cal_prepare(&ux);}
    return 0;
}

static uint16_t cal_order(void)
{
    uint32_t j,order=0;
    for(j=0;j<ux.chip.bpp;j++) order|=ux.pack.ord[j]<<(j*3u);
    return (uint16_t)order;
}

static void cal_fp(udcal_model_t *m)      /* the applied fingerprint a calibration belongs to (a request's, the candidate's) */
{
    m->configured=ux.npx;m->chip=(uint8_t)ux.cfg[0];m->bpp=ux.chip.bpp;m->order=cal_order();
}

static void cal_request(const uint8_t *f,uint32_t bad_argument)
{
    udcal_model_t cfg={0};
    uint8_t error=0;
    cal_fp(&cfg);
    cfg.vbus=(uint16_t)vs_bus.mv;cfg.vled=(uint16_t)vs_vb.mv;
    if(!dl_on || uc.mode==UDC_UNDECIDED || upd.run || upd.transparent || ux.reconf || ux.pause_req || ux.want_stream)
        error=ULTRA_UCE_BUSY;
    if(vdip.in_dip || vdip.planned || !vs_bus.valid || !vs_vb.valid || vs_bus.mv-3000u>62000u || vs_vb.mv-3000u>62000u)
        error=ULTRA_UCE_SUPPLY;                  /* (both 3,000-65,000 mV) */
    if(bad_argument) error=ULTRA_UCE_ARGUMENT;
    udcal_request(&ucal,f[2],&cfg,ux.max_px,ux.input_limit_ma,ms_now,error);
}

static NOINLINE uint32_t off5(uint32_t v, uint32_t ref)   /* v more than 5 % away from ref */
{
    return (v>ref?v-ref:ref-v)*100u>ref*5u;
}

static void cal_match(void)
{
    udcal_model_t *m=&ucal.model;
    uint32_t order,lim=ux.input_limit_ma;
    if(udcal_active(&ucal) || ux.reconf) return;
    ucal.limit=(uint16_t)(lim && lim<ISENSE_BOARD_MA?lim:ISENSE_BOARD_MA);
    order=cal_order();
    if(m->configured!=ux.npx || m->chip!=ux.cfg[0] || m->bpp!=ux.chip.bpp || m->order!=order) udcal_invalidate(&ucal);
    if((m->flags&ULTRA_UCF_VALID) && vs_bus.valid && vs_vb.valid) {
        if(off5(vs_bus.mv,m->vbus) || off5(vs_vb.mv,m->vled)) m->flags|=ULTRA_UCF_STALE;
        else m->flags&=0xFBu;
    }
    if(!(m->flags&ULTRA_UCF_VALID)) {
        cal_fp(&ucal.candidate);
        ucal.candidate.flags&=0xF8u;
    }
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
    ud_lit = (uint8_t)(f != fps_ring[fps_i == 0u ? 9u : fps_i - 1u] || (ux.local && !ux.dark));   /* (the last second's) */
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
    fps_i = (uint8_t)(fps_i == 9u ? 0u : fps_i + 1u);
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

static NOINLINE uint32_t led_begin(void)       /* a code may start (none runs, no lead-in): its lead-in begins */
{
    if (led_ack.active || led_lead != 0u) {
        return 0;
    }
    led_lead = LED_LEAD_MS;
    return 1;
}

static void led_code(ack_kind_t kind)
{
    if (led_begin()) {
        ack_start(&led_ack, kind, 0);
    }
}

static void led_fault(uint8_t code)
{
    if (led_begin()) {
        ack_start(&led_ack, ACK_P_FAULT, code);
    }
}

static void led_version(void)
{
    if (led_begin()) {
        ack_start_version(&led_ack, fw_version);
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
#if UD_JIG
    if (tj.on && !led_ack.active && led_lead == 0u) {
        lvl = tj.led;                      /* test mode: the verdict (a code wins) */
    }
#endif
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
    case UPD_APP_CONFIRM:
#if UPD_APP
    {
        ws_enc_t *e = &ux.e;
        uint32_t ok = 0;
        if (a == UPD_APP_CONFIRM && !upd_app_confirm_live(&upd, ms_now)) goto confirm_end;
        if (!cal_io.take) {
            cal_io.take = a == UPD_APP_CONFIRM ? 17u : 1u;
            cal_io.was_local = ux.local;
            udcal_abort(&ucal, ULTRA_UCE_CANCELLED);
        }
        if (udcal_active(&ucal)) return;
        if (cal_io.owned != 2u) {
            ux.calibration = 1;
            ux.want_stream = 0;
            if ((cal_io.take & 15u) == 1u) {
                raw_take();
                cal_io.take++;
            }
            BARRIER();
            if (!ux.local || !enc_idle()) return;
            if ((cal_io.take & 15u) == 2u) {
                const uint32_t n = ux.max_px * ux.chip.bpp;
                memset(ux_pool, 0, n);
                cal_io.take++;
                raw_frame(n);
                return;
            }
            if (e->frames == cal_io.frame) return;
        }
        /* Every configured pixel has received black and its complete latch. */
        if (a == UPD_APP_CONFIRM) {
            ux.dark = 1;
            ux.calibration = 0;
            if (flash_may(0u) && upd_app_confirm_live(&upd, ms_now)) {
                ok = flash_done(hw_flash_half(UPD_APP_DESC + 2u * (UPD_ST_FIRST + upd.info.st_next),
                                         UPD_ST_CONFIRMED));
            }
confirm_end:
            ux.calibration = 0;
            cal_io.take = 0;
            cal_io.owned = 2;
            raw_done();
            upd_app_confirmed(&upd, (int)ok);
            return;
        }
        __disable_irq();                 /* preserve the completed black latch through reset */
        dl_hw_blank(1);
        dl_hw_mux(0);
        HO->request = UPD_REQ_ENTER;       /* the bootloader checks the ANNOUNCE again and installs it */
        upd_ho_seal(HO);
        NVIC_SystemReset();
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
    upd_forward_init();
#if UPD_STATUS_PROFILE
    upd.uid = (const uint8_t *)HW_UID;
#endif
}

/* every ms: the transparency timeout / pending results / a delayed confirm; the watchdog-loop count */
static void upd_service(void)
{
    uint32_t a;
    if(cal_io.take && !udcal_active(&ucal))
        upd_action(cal_io.take & 16u ? UPD_APP_CONFIRM : UPD_APP_TAKE);
    a = upd_app_poll(&upd, ms_now);
    BARRIER();
    upd_forward_wanted = UPD_FWD_WANTED();
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
 * The Diff-Power-UR-Analog-M's pages (core/ultra_protocol.h, the protocol wiki: Return channel) built from this board's
 * state, + 0x80 PIXEL (hardware-specific). Features and capabilities: design note section 8. */
#define UD_FEATURES   (ULTRA_FEAT_MCU_TEMP | (UPD_APP ? ULTRA_FEAT_FW_UPDATE : 0u) | ULTRA_FEAT_VERSION_BLINK | \
                       ULTRA_FEAT_ENERGY | ULTRA_FEAT_LIFETIME | ULTRA_FEAT_STANDBY | ULTRA_FEAT_LIFE_SYNC | \
                       ULTRA_FEAT_PIXEL_CALIBRATION | UD_FEAT_FULL)
/* LEAN_TELEMETRY (core/vdip.h; the V203 Digital-M): no POWER events (the POWER page, STATUS 8 bits 0-3), no SUPPLY
 * page (the dip log) and no DIAG page. The dips are still detected and counted (the flash's dip rules, LIFETIME, the
 * fault memory, STATUS's supply reason) and the supply impedance is on ENERGY; the console's records carry the rest. */
#if LEAN_TELEMETRY
#define UD_FEAT_FULL  0u
#else
#define UD_FEAT_FULL  (ULTRA_FEAT_POWER_EVENTS | ULTRA_FEAT_SUPPLY_DIPS | ULTRA_FEAT_DIAG)
#endif

/* STATUS 8 bits 4-7: why the board isn't plain: the first that applies */
static uint32_t light_why(void)
{
    const vdip_ev_t *d = vdip_event(&vdip, 0);
    return too_hot ? ULTRA_ST_WHY_HOT
         : udcal_active(&ucal) ? ULTRA_ST_WHY_CALIBRATING
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
    uint32_t i, j, v[14], count=0;
    uint32_t fields=0;
    uint8_t *dest=d;
    /* All implemented banks/outputs are bank zero. POWER and LIFE_SYNC use
     * this byte as data, so their parsers retain it. Reject before side effects. */
    if (arg[3] && page != ULTRA_PAGE_POWER && page != ULTRA_PAGE_LIFE_SYNC)
        return ULTRA_PAGE_UNSUPPORTED;
    if(page==ULTRA_PAGE_LINK_STATS)
        return ls_page(arg,d,ms_now,ux.r_have?((ux.wq24+127u)>>8)*125u/2u:0);
    switch (page) {
    case ULTRA_PAGE_DESCRIPTOR: {
        uint32_t offset = 24u * arg[0];
        if (offset >= BOARD_DESCRIPTOR_BYTES) return ULTRA_PAGE_UNSUPPORTED;
        for (i=0; i<24u && offset+i<BOARD_DESCRIPTOR_BYTES; ++i) d[i]=board_descriptor[offset+i];
        break;
    }
    case ULTRA_PAGE_ACKS: {
        /* PIXEL_FX has only explicit segment ACKs. All light columns are NEVER. */
        uint32_t never=0x3FFFFu;
        for (i=0;i<2u;++i) {                 /* the two scopes with a SEQ: row 0 the board's, row 5 the output's */
            const uint32_t row = i != 0u ? 5u : 0u;
            if (!(ux.config_have & (1u<<i))) continue;
            for(j=0;j<3u;++j) {
                if (arg[j]==ULTRA_CONFIG) { d[3u*row+j]=ux.config_seq[i]; never &= ~(1u<<(3u*row+j)); }
            }
        }
        ultra_put24(d+18,never);
        ultra_put16(d+21,ud_px_config_crc(ux.cfg,ux.cfg_valid,ux.input_limit_ma,ux.input_valid));
        break;
    }
    case ULTRA_PAGE_CALIBRATION:
        return arg[1]==1u && !arg[2] && udcal_page(&ucal,arg[0],d)==0 ? page:ULTRA_PAGE_UNSUPPORTED;
    case ULTRA_PAGE_STATUS: {
        const uint32_t fl = RC_WIDTHS4(3, 1, 3, 1) | (RC_WIDTHS4(1, 2, 2, 2) << 8) | (RC_WIDTHS4(2, 2, 1, 1) << 16) | (RC_WIDTHS4(2, 1, 1, 1) << 24);
        v[0] = fw_version >> 8;                          /* major, minor, patch (0: a development build) */
        v[1] = ud_hw_id();                               /* ULTRA_ST_HW_ID (the whole byte) */
        v[2] = up_s;
        v[3] = reset_flags >> 26;
#if LEAN_TELEMETRY
        v[4] = light_why() << ULTRA_ST_WHY_SHIFT;        /* (bits 0-3: no POWER events) */
#else
        v[4] = pwrev_state(&pw) | light_why() << ULTRA_ST_WHY_SHIFT;
#endif
        v[5] = (uint16_t)mt.c10;
        v[6] = ULTRA_ST_NA;                              /* no board sensor */
        v[7] = vs_bus.valid ? vs_bus.mv / 10u : 0xFFFFu;
        v[8] = mt.vdda_mv;
        v[9] = sat16(i_avg);
        v[10] = (vs_bus.valid ? ULTRA_ST_M_VBUS : 0u) | ULTRA_ST_M_CURRENT;
        v[11] = 0xFFu;                                   /* no PWM (not applicable) */
        v[12] = ud_px_config_crc(ux.cfg,ux.cfg_valid,ux.input_limit_ma,ux.input_valid);                                 /* the CONFIG CRC over no CONFIG (CRC-16/CCITT-FALSE's init) */
        v[13] = 0;                                       /* modes: none */
        dest=d; fields=fl; count=14;
        break;
    }
    case ULTRA_PAGE_FW_UPDATE:
        if (arg[0] || arg[1] || arg[2] || !upd.info.desc_ok) return ULTRA_PAGE_UNSUPPORTED;
        upd_app_page(&upd.info,(const uint8_t *)UPD_APP_DESC,d);
        break;
    case ULTRA_PAGE_IDENTITY:                            /* two parts: 0 the board, 1 the features */
        if (arg[0] > ULTRA_ID_PART_TARGET) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        if (arg[0] == ULTRA_ID_PART_TARGET) {
            if (!upd.info.boot) return ULTRA_PAGE_UNSUPPORTED;
            memcpy(d, &upd.info.boot->target, 19u);
            d[19]=ULTRA_RECORD_FORMAT; d[20]=ULTRA_DESC_FORMAT; d[21]=upd.info.boot->entry_10ms;
            break;
        }
        if (arg[0] == ULTRA_ID_PART_BOARD) {
            const uint32_t fl = RC_WIDTHS4(4, 1, 1, 1) | (RC_WIDTHS4(1, 4, 1, 1) << 8);
            memcpy(d, (const uint8_t *)HW_UID, 12u);   /* the unique id, in address order */
            v[0] = fw_version;
            v[1] = upd.info.bl_version;
            v[2] = ud_hw_id();
            v[3] = upd.info.hw_rev;
            v[4] = ULTRA_VERSION;
            v[5] = (BOARD_DESCRIPTOR_BYTES << 16) | BOARD_DESCRIPTOR_CRC;
            dest=d + 12; fields=fl; count=6;
        } else {
            ultra_put32(d + ULTRA_IDF_FEATURES, UD_FEATURES);   /* word 0; words 1-5: none yet */
        }
        break;
#if !LEAN_TELEMETRY
    case ULTRA_PAGE_POWER:
        pwrev_ack(&pw, arg);
        pwrev_page(&pw, d);
        break;
    case ULTRA_PAGE_SUPPLY:
        vdip_page(&vdip, &vs_bus, up_s, vs_bus.valid ? vs_bus.mv : 0u, arg[0], d);
        break;
#endif
    case ULTRA_PAGE_ENERGY:
        energy_page(&en, up_s, vs_bus.mv / 10u * (uint32_t)(i_avg > 0 ? i_avg : 0) / 100u, d);
        ud_put16(d + ULTRA_EN_VB, vs_vb.valid ? vs_vb.mv / 10u : 0xFFFFu);   /* V_B (after the fuse) */
        ud_put16(d + ULTRA_EN_PEAK_MA, i_peak);
        i_peak = 0;
        j = vdip_z_mohm(&vdip);
        ud_put16(d + ULTRA_EN_SUPPLY_Z, j > 0xFFFFu ? 0xFFFFu : j);
        break;
    case ULTRA_PAGE_LIFE_SYNC:
        ls_res = (uint16_t)energy_sync(&en, arg, rc_uid16, BOARD_IN_WH_NUM, BOARD_IN_WH_DEN);   /* (one merge a boot: core/energy.h) */
        /* fall through */
    case ULTRA_PAGE_LIFETIME: {
        const uint32_t fl = RC_WIDTHS4(3, 3, 3, 2) | (RC_WIDTHS4(2, 3, 1, 1) << 8) | (RC_WIDTHS4(1, 2, 3, 1) << 16);
        const uint32_t fs = RC_WIDTHS4(3, 3, 3, 2) | (RC_WIDTHS4(2, 3, 4, 2) << 8) | (RC_WIDTHS4(2, 1, 1, 1) << 16);   /* (LIFE_SYNC: Wh, result + applied, records) */
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
        dest=d; fields=page == ULTRA_PAGE_LIFE_SYNC ? fs : fl; count=page == ULTRA_PAGE_LIFE_SYNC ? 9u : 11u;
        break;
    }
#if !LEAN_TELEMETRY
    case ULTRA_PAGE_DIAG:
        if (arg[0] > 1u) {
            return ULTRA_PAGE_UNSUPPORTED;
        }
        if (arg[0] == 0u) {                              /* the running board */
            const uint32_t fl = RC_WIDTHS4(1, 1, 2, 3) | (RC_WIDTHS4(2, 2, 2, 2) << 8) | (RC_WIDTHS4(2, 2, 1, 1) << 16) | (RC_WIDTHS4(1, 1, 1, 1) << 24);
            v[0] = reset_flags >> 26;
            v[1] = bc.magic == BC_MAGIC ? bc.cause : 0xFFu;
            v[2] = bc.pc;
            v[3] = bc.up_s > 0xFFFFFFu ? 0xFFFFFFu : bc.up_s;
            v[4] = cpu_pm;
            v[5] = cpu_peak;
            v[6] = 0;                                    /* LEAD: captures dropped, frame ends not handled */
            v[7] = 0;
            v[8] = ovr_late > 0xFFFFu ? 0xFFFFu : ovr_late;
            v[9] = 0;                                    /* (17-18 reserved) */
            v[10] = flash_err;
            v[11] = (uint32_t)en.due << 3 | (uint32_t)en.held << 7;   /* ULTRA_DG_FP_LIFETIME, _EXHAUSTED */
            v[12] = ULTRA_DG_SW_NONE;                    /* (no settings / calibration / scene / effects here) */
            dest=d; fields=fl; count=13;
            d[22] = d[23] = 0xFFu;                       /* no ripple load */
        } else {                                         /* the fault memory (core/energy.h); no calibration trend */
            const uint32_t fl = RC_WIDTHS4(1, 2, 1, 1) | (RC_WIDTHS4(1, 1, 1, 1) << 8);
            const life_rec_t *r = &en.lt;
            v[0] = r->trip;
            v[1] = r->trip_h;
            v[2] = r->dip_pct;
            v[3] = r->dip_cs;
            v[4] = r->dip_a;
            v[5] = r->rst;
            v[6] = r->rst_h;
            dest=d; fields=fl; count=7;
            for (i = 8; i < 13u; i++) {
                d[i] = 0x80u;                            /* ULTRA_HE_NA */
            }
            d[13] = d[14] = 0xFFu;
        }
        break;
#endif
    case ULTRA_PAGE_PIXEL:
        if (arg[0] == ULTRA_PIXEL_PART_LINE) {
            udc_page(&uc, dl.st.answered, (uint32_t)(ux.local && !ux.dark), ms_now, d);
        } else if (arg[0] == ULTRA_PIXEL_PART_CONFIG) {
            d[0]=(uint8_t)ux.cfg[0]; d[1]=ux.chip.bpp;
            ud_put16(d+2,cal_order()); ud_put16(d+4,ux.npx);
            ud_put16(d+6,ux.cfg[ULTRA_CFG_PX_FPS-ULTRA_CFG_PX_FIRST]);
            d[8]=(uint8_t)ux.cfg[ULTRA_CFG_PX_SOURCE-ULTRA_CFG_PX_FIRST];
            ud_put16(d+9,ux.cfg[ULTRA_CFG_PX_MAX-ULTRA_CFG_PX_FIRST]);
            ud_put16(d+11,ux.npx*ux.chip.bpp);
            ud_put16(d+13,ud_px_config_crc(ux.cfg,ux.cfg_valid,0,0));
        } else return ULTRA_PAGE_UNSUPPORTED;
        break;
    case ULTRA_PAGE_PIXEL_FX:
        if (arg[1] || udfx_page(&ux,arg[0],d)) return ULTRA_PAGE_UNSUPPORTED;
        break;
    default:
        return ULTRA_PAGE_UNSUPPORTED;                   /* OUTPUTS, ACKS, CAL_TEST, HEALTH, ...; LEAN_TELEMETRY: POWER, SUPPLY, DIAG */
    }
    if(count) rc_pack(dest,v,fields,count);
    return page;
}

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
#if LEAN_TELEMETRY
    rc_answer(a, poll, page, 0u, rc_uid16);             /* (no POWER latches) */
#else
    rc_answer(a, poll, page, pwrev_latched(&pw) ? ULTRA_RSP_EVENT : 0u, rc_uid16);
#endif
}

#ifdef MCU_HSI_BOOT_TRIM_STEPS
#define UD_CLOCK_RATED() clk_rated
#else
#define UD_CLOCK_RATED() 1
#endif
/* ------------------------------------------------------------------------------- marker frames (the main loop)
 * The HSI's rate from the headers' TIME (host us at the frame end) against the local tick at the frame end: host us per
 * local tick, Q16 (rc_brr's unit; nominal 50 << 16). The reference moves only after >= 1 s (50 ppm per tick of
 * error), a gap over 30 s starts again; a rate more than 1/16 off nominal is not taken (rc_brr checks it too) */
#define UD_RATE_NOM  (MCU_POLL_US << 16)
static uint32_t rate_q16 = UD_RATE_NOM, clk_host, clk_tick;
static uint8_t clk_have;
#ifdef MCU_HSI_BOOT_TRIM_STEPS
/* (CH32V40x, decided 2026-10-22) the HSI boots trimmed low; each measured rate moves HSITRIM towards the step at or
 * just below nominal (core/hsitrim.h). A step moves the clock: the rate references start again, and the answers wait
 * for a rate measured at the current trim (clk_rated: the reply's divider and the chain slot times follow it) */
static hsitrim_t htrim;
static uint8_t clk_rated;

static uint32_t ud_trim(uint32_t r)
{
    const int32_t err = (int32_t)m32_muldiv(UD_RATE_NOM, 1000000u, r) - 1000000;
    const uint32_t old = htrim.trim, next = hsitrim_update(&htrim, err);
    int32_t pred = err;
    if (next != old) {
        hw_hsi_trim_set(next);
        pred = err + ((int32_t)next - (int32_t)old) * htrim.step_ppm;
        ux.r_have = 0;                     /* (the effects' word rate, measured again from the next anchors) */
    }
    ws_argb_q16 = m32_muldiv(2u << 16, (uint32_t)(1000000 + pred), 1000000u);   /* HCLK cycles per 10 ns, Q16 */
    return next != old;
}
#endif

static void ud_clock(uint32_t host_us, uint32_t end_tick)
{
    uint32_t dt = end_tick - clk_tick, r;
#ifdef MCU_HSI_BOOT_TRIM_STEPS
    uint32_t stepped = 0;
#endif
    if (clk_have && dt < POLL_HZ) {
        return;                            /* (under 1 s: the reference stays, the interval grows) */
    }
    if (clk_have && dt <= 30u * POLL_HZ) {
        r = m32_muldiv(host_us-clk_host,65536u,dt);
        if (r - (UD_RATE_NOM - UD_RATE_NOM / 16u) < UD_RATE_NOM / 8u) {
            rate_q16 = r;
#ifdef MCU_HSI_BOOT_TRIM_STEPS
            stepped = ud_trim(r);
            clk_rated = (uint8_t)!stepped;
#endif
        }
    }
    clk_have = 1;
    clk_host = host_us;
    clk_tick = end_tick;
#ifdef MCU_HSI_BOOT_TRIM_STEPS
    if (stepped) {
        clk_have = 0;                      /* (the next frame starts a measurement at the new trim) */
    }
#endif
}

/* a CRC-good slot that is no POLL: IDENTIFY 5 / 7, a local pixel effect, CONFIG 64-75 (stream and chain frames alike) */
static void ud_slot(const uint8_t *f, uint32_t host)
{
    const uint32_t what = f[ULTRA_PAYLOAD];   /* an IDENTIFY's code (f[3]) */
    if ((f[0] & (ULTRA_TYPE_MASK | ULTRA_TYPE_BCAST)) == ULTRA_IDENTIFY &&
        (f[1] == 0u ? (what == ULTRA_IDENT_CAL || what == ULTRA_IDENT_RELEASE || what == ULTRA_IDENT_PIXEL_CLEAR)
                    : (f[1] == ULTRA_TARGET_BOARD && (what == ULTRA_IDENT_POWER_OFF || what == ULTRA_IDENT_VERSION)))) {
        uint32_t j,bad=0;
        for(j=ULTRA_PAYLOAD+1u;j<ULTRA_SLOT_BYTES-2u;j++) bad|=f[j];
        switch (what) {
        case ULTRA_IDENT_CAL:
            cal_request(f,bad);
            break;
        case ULTRA_IDENT_RELEASE:
            if(!bad) udcal_abort(&ucal,ULTRA_UCE_CANCELLED);
            break;
        case ULTRA_IDENT_PIXEL_CLEAR:
            if(!bad && (!cal_io.clear_have || cal_io.clear_seq!=f[2])) {
                cal_io.clear_have=1;cal_io.clear_seq=f[2];
                udcal_abort(&ucal,ULTRA_UCE_CANCELLED);udcal_invalidate(&ucal);
                ucal.model.flags=ULTRA_UCF_PENDING|ULTRA_UCF_CLEAR_PENDING;
            }
            break;
        case ULTRA_IDENT_POWER_OFF:
            ud_power_off_notice();         /* (repeated slots: the same notice again, the cut's window from the last) */
            break;
        default:                           /* ULTRA_IDENT_VERSION */
            led_version();                 /* the status LED (its copies: one pattern) */
            break;
        }
    } else if ((f[0] & (ULTRA_TYPE_MASK | ULTRA_TYPE_BCAST)) == ULTRA_PIXEL_FX && (f[1] >> 4) == 0u && ((f[1] & 15u) < UX_SEGS || (f[1] & 15u) == 15u)) {
        uint32_t si=f[1]&15u;
        if(si>=UX_SEGS) si=UX_SEGS;
        if(!(ux.seq_have&(1u<<si)) || ux.seq5[si]!=f[2]) udcal_abort(&ucal,ULTRA_UCE_CANCELLED);
        udfx_pixel_fx(&ux, f, host);       /* a local pixel effect (applied once per SEQ) */
    } else if ((f[0] & (ULTRA_TYPE_MASK | ULTRA_TYPE_BCAST)) == ULTRA_CONFIG && (f[1] == 0u || f[1] == ULTRA_TARGET_BOARD)) {
        uint32_t j, scope = f[1] == ULTRA_TARGET_BOARD;
        if ((ux.config_have & (1u << scope)) && ux.config_seq[scope] == f[2]) return;
        ux.config_seq[scope] = f[2]; ux.config_have |= (uint8_t)(1u << scope);
        for (j = 0; j < 9u; j++) {         /* board/input and output CONFIG scopes have independent SEQs */
            uint32_t id=f[ULTRA_PAYLOAD+3u*j],v=ultra_get16(f+ULTRA_PAYLOAD+3u*j+1u);
            if (scope) {
                if (id != ULTRA_CFG_BOARD_MAX || v == ULTRA_LIMIT_UNKNOWN) continue;
                v = !v || v > ISENSE_BOARD_MA ? ISENSE_BOARD_MA : v;
                if (ux.input_limit_ma != v) udcal_abort(&ucal,ULTRA_UCE_CONFIG);
                ux.input_limit_ma = (uint16_t)v;
                ux.input_valid = 1;
                continue;
            }
            if(udfx_config(&ux,id,v)) {
                udcal_abort(&ucal,ULTRA_UCE_CONFIG);
                if(id<=ULTRA_CFG_PX_PIXELS) udcal_invalidate(&ucal);
            }
        }
    }
}

/* a frame with a right header (flags; host = its TIME; end_tick / w: the tick and the ring word of its last edge), marker or
 * chain: the update's frame-end notice, then the clock and the effects' anchor. An UPD frame (0) cancels a calibration and
 * gets no clock sync (as frame_rx); the caller takes its payload */
static NOINLINE uint32_t ud_header(uint32_t flags, uint32_t host, uint32_t end_tick, uint32_t w)
{
    uds.headers++;
    upd_app_frame_end(&upd, 1u, (uint8_t)flags, ms_now);
    BARRIER();
    upd_forward_wanted = UPD_FWD_WANTED();
    if (flags & ULTRA_HDR_UPD) {
        udcal_abort(&ucal, ULTRA_UCE_CANCELLED);
        return 0;
    }
    ud_clock(host, end_tick);
    udfx_anchor(&ux, host, w);
    return 1;
}

/* a valid POLL for this board: its answer built into ud_ans, with the divider at the POLL's rate */
static NOINLINE void ud_answer(const uint8_t *poll)
{
    ud_rc_answer(poll, ud_ans);
    ud_brr = rc_brr(rate_q16, poll[RC_P_RATE]);
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
    if (ud_parse_header(h, &host) != 0) {
        uds.tail_err++;
        return 0;
    }
    flags = ultra_header_flags(h);
    if (!ud_header(flags, host, end_tick, dl_ready_w)) {   /* (the anchor: the host clock against the ring's word count) */
        if (nbits == UPD_FRAME_BITS && !(flags & ULTRA_HDR_RW)) {
            upd_action(upd_app_frame(&upd, f, ms_now));
        }
        return 0;
    }
    if (nbits != ULTRA_SLOT_BITS + ULTRA_HDR_BITS || (flags & ULTRA_HDR_EXT) || ud_slot_check(f) != 0) {
        uds.dec_err += nbits == ULTRA_SLOT_BITS + ULTRA_HDR_BITS && !(flags & ULTRA_HDR_EXT);
        return 0;
    }
    uds.decoded++;
    if (flags & ULTRA_HDR_RW) {
        if (rc_poll_parse(f) != 0 || f[RC_P_INDEX] != 0u) {
            return 0;                      /* not a POLL, or RELAY_ONLY / another index: no other board on this port */
        }
        uds.polls++;
#ifdef MCU_HSI_BOOT_TRIM_STEPS
        if (!clk_rated) {
            return 0;                      /* (no rate at this trim yet: no answer, the hub asks again) */
        }
#endif
        ud_answer(f);
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
static ws_lock_t line_lock;
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
 * x pitch: no release, no marker), the other slot types act as in ud_frame. UPD frames retain their whole payload. */
static void ud_line_apply(uint32_t act)
{
    if (act == UDC_A_CHAIN) {
        gate_slot = 1;                     /* the tick arms the slot plan in the next quiet LOW */
        dline_ch_start(&dl, 1);            /* BLANK_N still held (BOOT, AUTO deciding): the ring zeroed, the mux switched, the encoder next; else at a latch */
        udfx_chain(&ux, 1);
    } else if (act == UDC_A_STREAM) {
        gate_slot = 0;                     /* a marker port has one receiver: nothing is forwarded (an update's transparency: the tick
                                              closes it, UPD_FWD_WANTED needs CHAIN) */
        BARRIER();
        if (!upd_forward_open) {
            upd_forward_set(0);
        }
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
    uint32_t relay = 0, dir_w = 0, fwd_w = 0;
    const uint32_t end_w = dl.ch_end, ready_tick = dl_ch_tick;   /* (the frame's: the capture is freed inside, a later frame overwrites them) */
    (void)udc_classify(&dl, &rx, &ucp, ms_now);    /* (frees the capture once the slot and the header are out: the next frame may be captured) */
    act = udc_frame(&uc, &rx, ms_now);
    {
        uint32_t lock=ws_line_lock(&line_lock,rx.profile|(rx.bits==ULTRA_HDR_BITS?4u:0u),rx.cls!=UDC_C_NONULTRA,ms_now);
        dl.line_profile=(lock&1u) && line_stats.trained ? line_lock.profile : 255u;
        if(lock&2u) { ux.r_have=ux.a_have=clk_have=0; }
        if(!(lock&1u)) {
            dline_ch_done(&dl,0,0,0,0,0,0);
            ud_line_apply(act);             /* AUTO still observes non-Ultra traffic while acquiring */
            return;
        }
    }
    if (rx.cls == UDC_C_NONULTRA) {
        uds.tail_err++;
    } else if (rx.host = udfx_host_of(&ux, rx.host), !ud_header(rx.flags, rx.host, ready_tick, end_w)) {   /* (TIME as the engine's host us) */
        if (rx.cls == UDC_C_UPDATE && !(rx.flags & ULTRA_HDR_RW))
            upd_action(upd_app_frame(&upd, (const uint8_t *)&rx + offsetof(udc_rx_t, slot), ms_now));
    } else {
        if (rx.cls == UDC_C_SLOT && !(rx.flags & ULTRA_HDR_EXT)) {
            uds.decoded++;
            if (!(rx.flags & ULTRA_HDR_RW)) {
                ud_slot(rx.slot, rx.host);
            } else if (rc_poll_parse(rx.slot) == 0) {
                uc.windows++;
                uds.polls++;
                udc_window(rx.slot, end_w, ux.wpu24, &tx_w, &win_w, &dir_w, &fwd_w);   /* this board's POLL or RELAY_ONLY (the window's end
                                                                                      for the chain scanner in any case) */
                /* the relay window (the boards behind answer through it) and the answer only with a measured rate and a trained line,
                   as on the Analog-M: an unmeasured HSI (+-2.2 %) would put the answer outside its slot and forward-again off by up to
                   ~110 us at R 8, into the hub's retake; missed, the hub asks again */
                if (ux.r_have && ws_line_ready(&line_lock,ms_now) && UD_CLOCK_RATED()) {
                    relay = 1;
                    if (rx.slot[RC_P_INDEX] != ULTRA_POLL_RELAY_ONLY) {
                        ud_answer(rx.slot);
                        answer = 1;
                    }
                } else {
                    uc.answers_missed++;
                }
            }
        } else if (rx.cls == UDC_C_HDR && !(rx.flags & ULTRA_HDR_EXT)) {
            uds.dec_err++;                 /* (a right header and length, a slot that is none: a DARK slot of the preamble is no error) */
        }
    }
    if (win_w == 0u && (rx.flags & ULTRA_HDR_RW)) {
        win_w = end_w + DLINE_CH_WIN_W;    /* the window is some other board's: as long as the longest one */
    }
    BARRIER();                             /* the answer complete before the window may send it */
    dline_ch_done(&dl, (int)answer, tx_w, win_w, relay, dir_w, fwd_w);
    ud_line_apply(act);
}

/* once a ms: the line mode's clock (AUTO's limits, a new CONFIG 75), the tick's events */
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

/* Output-only diagnostics. One bounded record per main-loop pass; live wins each second. */
#define CONSOLE_UR_DIGITAL_M 1
#include "../core/console_fields.inc"
#include "console_ur_digital_m.inc"
static void con_message(uint32_t value) { console_record(uart_write, 0, &value, cf_X); }
static void con_banner(void) { con_b(); console_record(uart_write, 0, HW_UID, cf_i); }
static void console_service(void)
{
    uint32_t p;
    if (uart_room()<CONSOLE_LINE_MAX) return;
    if (con_flash_err) { con_flash_err=0; con_message(201); return; }
#if MCU_CH32V40X
    if (con_cfg) { con_cfg=0; con_message(CZ_m_config); return; }
#endif
#if UD_JIG
    if (tj.say) { tj.say=0; con_message(CZ_m_tj_off-tj.on); return; }   /* X "test mode: on" / "off" */
    if (tj.rep) { tj.rep=0; console_record(uart_write,0,&tj,cf_j); return; }   /* (a cycle's result: j; the supply, UID, firmware: r, i, b) */
#endif
    if (ms_now-con_ms>=1000u) {
        con_ms=ms_now; con_r();
#if MCU_CH32V40X
        {
            uint32_t h=5381u,k;
            const uint8_t *b=(const uint8_t *)ux.cfg;
            for(k=0;k<sizeof ux.cfg+sizeof ux.cfg_valid+sizeof ux.input_limit_ma;k++) h=h*33u+b[k];   /* (adjacent: udfx.h) */
            b=(const uint8_t *)ux.stored;
            for(k=0;k<sizeof ux.stored;k++) h=h*33u+b[k];
            if(h!=con_fp && con_status_s) { con_cfg=1; con_status_s=up_s-60u; }   /* a setting changed: CONFIG_CHANGED, the dump now */
            con_fp=h;
        }
#endif
        if(up_s>=5u && (!con_status_s || up_s-con_status_s>=60u)) { con_status_s=up_s; con_part=1; }
        return;
    }
    p=con_part;
    if(!p) return;
    con_part++;
#if LEAN_TELEMETRY
    if(p==7u) return;                      /* (no P record: no POWER events) */
    if(p==19u) { con_part=24u; return; }   /* (no dip log: 19-23) */
#endif
    if(p<=3) {
        switch(p) { case 1: con_c(); break; case 2: con_t(); break; default: con_k(); break; }
    } else if(p<=10 || p==24 || p==25) {
        static const struct { const void *base; const uint16_t *fields; } records[] = {
            {&ux,cf_l}, {&en.lt,cf_L}, {&en,cf_E},
#if LEAN_TELEMETRY
            {0,0},                         /* (7: skipped above) */
#else
            {&pw,cf_P},
#endif
            {&vdip,cf_V},
            {(const void *)hw_diag,cf_D}, {&bc,cf_F}, {&dl.st,cf_d}, {&uds,cf_z}
        };
        uint32_t k=p<=10 ? p-4 : p-17;
        console_record(uart_write,0,records[k].base,records[k].fields);
    } else if(p>=26u) {
#if MCU_CH32V40X
        if(p==26u) console_record(uart_write,0,&line_stats,cf_K);   /* (the V203's flash has no room for these) */
        else if(p==27u) con_y();
        else
#endif
        con_part=0;
    }
    else {                                 /* 11-14 the segments' settings, 15-18 the stored effects (one record format), 19-22 the dip
                                              log, 23 the deepest dip (one format): one call */
        const void *base; const uint16_t *fields=cf_p; uint32_t idx=p-11u;
        if(p<15u) base=&ux.seg[idx].p;
#if LEAN_TELEMETRY
        else base=&ux.stored[p-15u];       /* (no dip log: 19-23 skipped above) */
#else
        else if(p<19u) base=&ux.stored[p-15u];
        else { fields=cf_v; idx=p-19u; base=p==23u ? (const void *)&vdip.deep : (const void *)&vdip.log[idx]; }
#endif
        console_record(uart_write,idx,base,fields);
    }
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
    (void)ucal_load(&ucal);
    hw_records_window(0);
#if !LEAN_TELEMETRY
    pwrev_init(&pw, ms_now);
#endif
    hw_init();                             /* the pins first: mux direct, strip held LOW, receive, MOSI LOW */
    reset_flags = hw_reset_flags();
    upd_boot();                            /* UPD_APP: the reset flags from the hand-over, the bootloader's result; (round
                                              11, B2) first, its hardware id decides the delay line */
    udfx_init(&ux);                        /* the local pixel effects: the stored record (power-on effect, strip settings) */
    if (ufx_found && udfx_rec_valid(ufx_rec)) {
        udfx_load(&ux, ufx_rec);
    }
    cal_match();                         /* a stored calibration is usable only at this applied fingerprint */
    udc_init(&uc, udfx_line_cfg(&ux), ms_now);   /* CONFIG 75 from the record: AUTO (0) unless forced */
#ifdef MCU_HSI_BOOT_TRIM_STEPS
    hsitrim_init(&htrim, hw_hsi_trim(), (int32_t)MCU_HSI_TRIM_PPM_STEP);
    if (upd.info.boot != 0 && upd.info.boot->target.bytes[UPD_T_MCU] != MCU_UPD_MCU) {
        upd.info.hw_id = 0;                /* another family's bootloader (an SWD mis-flash): never this board's */
    }
#endif
    if (!upd_app_wrong_hw(&upd.info, BOARD_HW_ID)) {
        dl_hw_start();                     /* the delay line runs (the mux direct, BLANK_N LOW): dline's BOOT switches
                                              both at the first quiet gap */
        dline_init(&dl, dl_hw_rx_pos(), dl_cap);
        if (uc.mode == UDC_CHAIN) {        /* forced CHAIN: the ring zeroed and the mux switched at the first tick, no waiting for a frame */
            gate_slot = 1;
            dline_ch_start(&dl, 1);
            udfx_chain(&ux, 1);
        } else if (uc.mode == UDC_UNDECIDED) {   /* AUTO: the strip held (BOOT's release waits) until two Ultra frames or the markers say */
            dline_ch_hold(&dl, 1);
            dline_ch_start(&dl, 0);
            udfx_chain(&ux, 2);
        } else {
            udfx_chain(&ux, 0);             /* publish forced STREAM too; AUTO retains a decided mode */
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
    vsense_init(&vs_bus, VSENSE_SUPPLY_RATIO_Q8, VSENSE_VREF_MV);
    vsense_init(&vs_vb, VSENSE_AUX_RATIO_Q8, VSENSE_VREF_MV);
    hw_adc_init();                         /* PB0 continuously, the injected group from the poll tick */
#if UPD_APP
    fw_version = upd.info.desc_ok ? upd.info.version : 0u;
#else
    fw_version = fwver_parse(FW_VERSION);
#endif
    energy_init(&en, life_found, ms_now, fw_version, reset_flags & RCC_IWDGRSTF);
    en.ev_save = 1u;                       /* (as BOARD 2: a standby cycle writes nothing for itself, core/energy.h) */
    if (reset_flags & RCC_IWDGRSTF) {      /* the fault memory: the last abnormal reset, with that run's breadcrumbs */
        en.lt.rst = (uint8_t)(bc.magic == BC_MAGIC && bc.cause != 0u ? 2u | (bc.cause & 63u) << 2 : 1u);
        en.lt.rst_h = (uint8_t)(bc.magic == BC_MAGIC ? (bc.up_s / 3600u > 255u ? 255u : bc.up_s / 3600u) : 0u);
    }
    flash_erase_ahead();                   /* the lifetime record's next page, erased at power-up (the dip write's) */
    flash_boot = 0;
    upd.startup_ok = !upd_app_wrong_hw(&upd.info, BOARD_HW_ID);
    con_banner();
    last_ms = sec_ms = con_ms = ms_now;

    for (;;) {
        uint32_t fx_more;
        watchdog_kick();
        dl_main();
        fx_more = cal_work();             /* one effect OR calibration slice, never both */
        dl_main();                       /* consume raw input between effect work and millisecond services */
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
#if UD_JIG
            uj_ms(&tj, ms_now, HW_TP6_LOW(), i_avg, tj_st == TJ_LIVE, ux.chip.bpp);   /* the pad, the cycle's currents */
#endif
            dl_main();
#if !LEAN_TELEMETRY
            power_service();
#endif
            vsense_service();
            if(vdip.in_dip || vdip.planned) udcal_abort(&ucal,ULTRA_UCE_SUPPLY);
            upd_service();
            udfx_service(&ux, &dl, ms_now, i_avg > 0 ? (uint32_t)i_avg : 0u, ux_vk());
            cal_match();
            dl_main();
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
            cal_flash_service();
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
