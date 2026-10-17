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
/* The units (round 12, S10). main.c includes them here in this order, as one translation unit: their statics
 * stay static and the image is byte-identical to the single file's (compiled apart, LTO's early inlining
 * differs: a non-static function may be called from elsewhere). Then records_load and main(). The host tests
 * that include main.c (test_soak.c, test_robust_fw.c) get every unit with it.
 *   capture_isr.c   the poll ISR, its shared state, the SRAM tick
 *   loop_state.c    the main loop's state
 *   flash_policy.c  when the flash may be written (flash_may)
 *   sensing.c       supply, dips, energy; temperature
 *   outputs.c       settings in effect, the current sense, the outputs
 *   commands.c      config frame / console commands' results
 *   stores.c        the writes (settings, scene, effects)
 *   frames.c        a frame's own slot (WS, Ultra)
 *   update.c        firmware update, the app side
 *   rc_answer.c     the return channel's answers (BOARD_RETURN)
 *   frame_end.c     the capture's decode, the frame end, the next plan
 *   services.c      periodic services: stagger, signal loss, status */
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
#define FW_VERSION       "0.6.11"   /* release builds: the release version (tools/build_release.sh) */
#endif
#ifndef UPD_APP
#define UPD_APP 0                  /* 1: linked behind the bootloader (vendor/wch/Link_app.ld, build_release.sh) */
#endif
#define UPD_WD_CLEAR_S   10u       /* the bootloader's watchdog-loop count is cleared after this long running */
#define LOSS_TIMEOUT_MS  2000u     /* WS modes: signal loss = no valid frame for 2 s */
#define LOSS_FADE_MS     1000u     /* fade-off duration */
#define SAVE_DEBOUNCE_MS 2000u     /* settings saved 2 s after the last change */
static const ws_timing_t ws_tm = WS_TIMING_DEFAULT;   /* the decoder's bit timing (one copy for its three callers) */

#include "capture_isr.c"
#include "loop_state.c"
#include "flash_policy.c"
#include "sensing.c"
#include "outputs.c"
#include "commands.c"
#include "stores.c"
#include "frames.c"
#include "update.c"
#include "rc_answer.c"
#include "frame_end.c"
#include "services.c"

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
