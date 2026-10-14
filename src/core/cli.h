/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* UART diagnostics console (USART2 on TP5 TX / TP6 RX, 115200 8N1, line based, CR or LF ends a line).
 *   help | ?            command list
 *   status              firmware, uptime, reset cause, stack used, protocol, frame / error counters, frame rate,
 *                       CPU load (the last second, the busiest since the last status), outputs
 *   get                 all settings as key=value
 *   set <key> <value>   change a setting (same validation, blink feedback and saving as config frames)
 *   period <steps>      raw PWM period, 1440..65535 timer counts (144 MHz / steps = PWM frequency)
 *   save                store settings now          defaults   factory reset (all but the lifetime record)
 *   identify            blink the feedback channel / D9
 *   version             the firmware / bootloader lines (as at boot and in status), and blink the version
 *   watch [ms]          one line of live values every 1000 ms (100..60000) until any key
 * status ends with the lifetime record (core/energy.h; kept through factory resets and firmware updates): energy,
 * powered / lit hours, power-ups, trips, dips, the peaks, watchdog resets, updates, records written.
 * Boards with the input current sense: set ilimit <x 50 mA>, set ical <%> (calibration: 0 = the standard test at
 * 25 / 50 / 75 / 100 %), status shows the current, the calibration and the board limit in effect.
 * Every board: status shows the MCU (die) temperature and the 3.3 V rail from the internal sensor / VREFINT, the
 * voltage channels (the supply on PA4 with the input power, the spare input on TP8), the
 * overcurrent signal on D9 (core/ocled.h: limit / trip, since when, the hold left) and the power events
 * (core/pwrev.h: state, counters, latches, the last limit event and trip; what a Diff-Power ultra receiver reports). */
#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include "cz.h"
#include "calhist.h"
#include "config.h"
#include "energy.h"
#include "health.h"
#include "pwrev.h"
#include "scene.h"
#include "stagger.h"
#include "vdip.h"

/* (0.6.5, D1) the reset breadcrumbs: what the board was doing when it reset, in RAM no reset clears (main.c bcrumb,
 * vendor/wch/Link_sections.ld .noinit). The uptime every second, the main loop's PC every ms (the poll tick: mepc),
 * an exception's cause and PC (isr.S HardFault_Handler, then the watchdog resets). status shows the last run's after a
 * watchdog / software / pin reset (`before_reset=`, the PC in decimal: its hex in that release's fw.lst), never after a
 * power-on. */
#define BC_MAGIC 0x31435242u       /* "BRC1" */
typedef struct {
    uint32_t magic;                /* BC_MAGIC: this firmware wrote it since the last reset */
    uint32_t up_s;                 /* uptime, s */
    uint32_t pc;                   /* the main loop's PC at the last ms tick, or the faulting instruction's */
    uint32_t cause;                /* the exception's mcause (0: none; 0 itself, a misaligned fetch, can't happen with
                                      the compressed instructions) */
} bcrumb_t;

typedef struct {
    const char *version;           /* "0.6.0-beta.1", or FW_VERSION + " (development build)" */
    uint8_t  bl_version;           /* the bootloader's (UPD_BL_INFO), 0 = none (standalone build) */
    uint8_t  hw_id, hw_rev;        /* from the bootloader info */
    uint8_t  app_state;            /* 0 standalone, 1 trial (not confirmed), 2 confirmed */
    char     bl_build[9];          /* the bootloader's build id */
    uint32_t uptime_s;             /* s since boot (32 bits: 136 years) */
    uint32_t stack_max;            /* the stack's high-water mark, bytes (painted at boot; 0 = not known) */
    uint32_t reset_flags;          /* RCC RSTSCKR at boot */
    bcrumb_t bc;                   /* (0.6.5, D1) the breadcrumbs the last run left (shown unless reset_flags say
                                      power-on: random RAM) */
    uint8_t  proto_mode, proto_locked, hdr_bits;
    uint8_t  proto_provisional;    /* auto: the locked protocol is the stored one, not confirmed yet */
    uint32_t frames;               /* frame ends seen */
    uint32_t decoded;              /* slots decoded */
    uint32_t dec_err[6];           /* by -WS_ERR_* code */
    uint32_t config_frames;        /* accepted config commands */
    uint32_t ultra_headers;        /* Ultra active: frames of an Ultra length whose tail held a valid header */
    uint32_t ultra_tail_err;       /* ... and those whose tail did not (decode error, gap, CRC) */
    const volatile uint32_t *ovr;  /* overruns (main.c): captures dropped, frame ends not handled, render ms late;
                                      NULL = none */
    uint32_t fps;                  /* frames in the last second */
    uint16_t out[5];               /* current output levels, 16 bit */
    uint32_t steps;                /* PWM period in counts */
    uint32_t saves;                /* flash records written since boot */
    /* (0.6.5, D5) the flash's state (cli.c `flash:`), by store: settings, calibration, scene, lifetime, power-on
     * effect */
    uint8_t  fl_pend;              /* bit per store: a write is waiting (debounce, budget, a low moment) */
    uint8_t  fl_err;               /* the last failed operation's FLASH_ERR_* (core/flashrec.h; 0: none) */
    uint16_t fl_errs;              /* failed operations since boot */
    uint32_t fl_rej;               /* records the loaders rejected at boot, 6 bits per store (the effect store: heads it
                                      lists that did not load) */
    uint16_t stagger_o2, stagger_o3;   /* adaptive stagger: TIM2 / TIM3 offsets, Q16 of the period */
    uint32_t stagger_moves, ripple_ma; /* moves so far, LED_+ ripple estimate (mA rms) */
    stagger_cap_t cap;             /* C10's ripple load (0.6.5; core/stagger.h): updated by main.c once a second */
    /* board current sense (core/isense.h) */
    uint8_t  ilimit50;             /* stored board limit x 50 mA (0 = the board rating) */
    uint8_t  softstart;            /* soft start on (core/sstart.h) */
    uint8_t  i_model;              /* channel model: 0 none, 1 learned, 2 calibrated */
    uint8_t  i_tripped, i_trip_cause;  /* outputs forced off now; last cause (1 short, 2 overload) */
    uint8_t  i_cal;                /* a calibration is running */
    int32_t  i_ma, i_peak_ma, i_zero_ma;   /* ~100 ms average, peak of the last ms, zero (logic + offset) */
    uint32_t i_limit_ma;           /* limit in effect */
    uint32_t i_k_permille;         /* limiter scale of all outputs */
    uint32_t i_trips;
    int32_t  i_ch_ma[5];           /* channel currents at 100 % (model) */
    /* the board limit in effect (core/climit.h), every board */
    uint16_t lim_ma;               /* 0xFFFF = none */
    uint8_t  lim_src, lim_need;    /* CLIMIT_SRC_*, CLIMIT_NEED_* */
    uint32_t lim_conn_ma;          /* connected load (all channels at 100 %), 0 = not known */
    /* the stored current calibration (isense) */
    isense_cal_t cal;              /* the test (levels 0: none; 1 one level, 4 the standard test) */
    uint16_t cal_count;            /* tests stored on this board */
    /* MCU temperature (core/mcutemp.h), every board */
    int16_t  mcu_temp_c10;         /* 0.1 C, MCUTEMP_NA (-32768) = not valid */
    uint16_t vdda_mv;              /* the 3.3 V rail from VREFINT, 0 = not known */
    int16_t  mcu_peak_c10;         /* the highest since boot, 0.1 C (MCUTEMP_NA: none yet) */
    uint32_t mcu_peak_s;           /* uptime then (s) */
    int32_t  i_now_ch_ma[5];       /* each channel's current now: the model at its output, mA (watch) */
    /* voltage channels (core/vsense.h): PA4 the supply, PA0 the spare input on TP8 */
    uint32_t vs_supply_mv, vs_supply_min_mv, vs_supply_max_mv;   /* min / max: since the last status */
    uint32_t vs_aux_mv;
    const energy_t *en;            /* energy and the lifetime record (core/energy.h; always set: no NULL test) */
    const health_t *hl;            /* channel health (core/health.h), NULL = none */
    const vdip_t *dip;             /* supply dips (core/vdip.h), NULL = none */
    const vsense_t *dip_vs;        /* ... their scaling (the supply channel) */
    uint32_t dip_ref_mv;           /* their reference */
    uint8_t  test_mode;            /* test jig mode (core/testjig.h): the test pad is grounded */
    const uint32_t *uid;           /* the chip's 96-bit unique id (hw.h HW_UID; the test jig's line; always set) */
    const volatile uint16_t *hwd;  /* (0.6.5, D6) hal/hw.h hw_diag[HW_DIAG_N]: ADC freshness, DMA errors (NULL:
                                      none) */
    /* the overcurrent signal on D9 (core/ocled.h), every board */
    uint8_t  oc_show;              /* OCLED_*: 0 off, 1 limit, 2 trip */
    uint8_t  oc_cause;             /* the last trip's cause: 1 short, 2 overload */
    uint8_t  oc_limiting, oc_tripped;   /* the conditions now */
    uint32_t oc_since_s;           /* uptime (s) when the shown pattern started */
    uint32_t oc_hold_ms;           /* hold left (0 while the condition is on) */
    /* the power events (core/pwrev.h), every board: what a Diff-Power ultra receiver reports (STATUS byte 8, the
     * POWER page) */
    const pwrev_t *pw;
    /* CPU load (core/cpuload.h): busy share of the last second, of the busiest second since the last status, 0.1 % */
    uint16_t cpu_pm, cpu_peak;
    const struct evlog *ev;        /* (0.6.5, D2) the event log (below), NULL = none */
#if MON_EXTRA
    const calhist_t *ch;           /* (0.6.6, M2) the calibration history (core/calhist.h), NULL = none. (New fields at
                                      the end: test_console_cz.c's random states stay as they were) */
#endif
} cli_status_t;

typedef struct {
    void (*write)(const char *s);
    cfg_result_t (*apply)(uint8_t cmd, uint8_t arg);    /* runs config_apply + the main loop's follow-up */
    void (*save_now)(void);
    void (*status_begin)(void);    /* snapshot and reset extrema only for an actual status command */
    const settings_t *set;
    const scene_t *scene;          /* power-on scene (stored, or about to be) */
    const cli_status_t *st;
    char line[48];
    uint16_t watch_ms;             /* `watch`: a status line every watch_ms (main.c), 0 = off; any key stops it */
    uint8_t len;
    uint8_t  st_part;              /* (0.6.5) `status` / `help` in progress: the next part + 1 (0 = none; cli_poll) */
} cli_t;
/* (0.6.5) the console output's next part (a `status` or `help` in progress): 1 while more remain. main.c calls it
 * while the UART has CLI_PART_ROOM bytes free (every part is shorter), and feeds no new line meanwhile. */
#define CLI_PART_ROOM 512u
uint32_t cli_poll(cli_t *c);

void cli_feed(cli_t *c, char ch);
/* a message of the firmware on the console: z = Z(name, "text\r\n") (core/cz.h; a '%' as "%%"); it writes through
 * c->write: only after main() has set it (it does, before the boot's first flash write) */
void cli_say(cli_t *c, int z);
/* "QuinLED dig2analog-ultra firmware v<version>, Copyright (C) 2026 QuinLED, GPL-3.0, source: ..." and the
 * bootloader / hardware / app line: at boot, first in `status`, and `version` */
void cli_banner(cli_t *c);
/* test jig mode, after each cycle: each channel's current at the peak of pct % (mA), its estimate at 100 %, and the
 * supply (mV, 0 = not read); then (0.6.5) one line for a jig PC:
 *   jig cycle=7 PASS fail=0 a=0.14/0.13/0.40/0.80/0.00 supply=24.01 uid=19088743,2309737967,4275878552 fw=0.6.5
 * PASS when every channel's current at pct % is within core/testjig.h's TJ_PASS_MIN_MA .. TJ_PASS_MAX_MA, else FAIL and
 * fail's bits R 1, G 2, B 4, W1 8, W2 16 (exact mA; the line shows them in A, as the report); the supply in V (0.00 = not
 * read); the chip's unique id (st->uid: its three words at 0x1FFFF7E8, decimal) and the firmware version (st->version:
 * the rest of the line) */
void cli_test_report(cli_t *c, const int32_t ma[5], uint32_t pct, uint32_t cycle, uint32_t supply_mv);
/* `watch`: one line of the live values (the snapshot in st) */
void cli_watch(cli_t *c);
#define HW_DIAG_N 9u                /* (0.6.5, D6) cli_status_t.hwd's length: hal/hw.h's (the same definition: main.c
                                       sees both) */
/* exposed for tests */
void cli_exec(cli_t *c, const char *line);

/* (0.6.5, D2) the event log: the last EVLOG_N events with the uptime they came at (main.c ev_add; RAM, a reset clears
 * it). Fed by what counts them already, no counter of its own: the power events (pwrev_t.ev: a limit event, a trip,
 * a supply dip) and the end of a calibration. `status` ends with them, newest first. EVL_LIMIT .. EVL_DIP are the
 * ULTRA_PW_EV_* bits 0 .. 3 in that order (main.c checks it). */
#define EVLOG_N 8u
enum { EVL_NONE, EVL_LIMIT, EVL_OVERLOAD, EVL_SHORT, EVL_DIP, EVL_CAL };
typedef struct {
    uint32_t t_s;                  /* uptime, s */
    uint16_t ma;                   /* mA: the board current then (a short: its peak), a calibration's connected load */
    uint8_t  type;                 /* EVL_* (EVL_NONE: an empty entry) */
} evl_ev_t;
typedef struct evlog {
    evl_ev_t e[EVLOG_N];           /* a ring: e[head] the newest */
    uint8_t  head;
} evlog_t;

#endif
