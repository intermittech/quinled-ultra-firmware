/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Output-only release: numeric schema 03, one live record per second and a full
 * dump per minute; docs/console-format.md is the transport contract. The jig's
 * report and jig input trigger retain their production format. CONSOLE_FULL selects
 * the standalone text/input bench console without burdening release images. */
#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include "cz.h"
#include "calhist.h"
#include "config.h"
#include "energy.h"
#include "store.h"
#include "health.h"
#include "pwrev.h"
#include "scene.h"
#include "ultra.h"
#include "stagger.h"
#include "vdip.h"

/* the reset breadcrumbs: what the board was doing when it reset, in RAM no reset clears (main.c bcrumb,
 * src/mcu/ch32v20x/Link_sections.ld .noinit). The uptime every second, the main loop's PC every ms (the poll tick: mepc),
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
    const char *version;           /* "0.7.0-beta.1", or FW_VERSION + " (development build)" */
    uint8_t  bl_version;           /* the bootloader's (UPD_BL_INFO), 0 = none (standalone build) */
    uint8_t  hw_id, hw_rev;        /* from the bootloader info */
    uint8_t  app_state;            /* 0 standalone, 1 trial (not confirmed), 2 confirmed */
    char     bl_build[9];          /* the bootloader's build id */
    uint32_t uptime_s;             /* s since boot (32 bits: 136 years) */
    uint32_t stack_max;            /* the stack's high-water mark, bytes (painted at boot; 0 = not known) */
    uint32_t reset_flags;          /* RCC RSTSCKR at boot */
    bcrumb_t bc;                   /* the breadcrumbs the last run left (shown unless reset_flags say
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
    /* the flash's state (cli.c `flash:`), by store: settings, calibration, scene, lifetime, power-on
     * effect */
    uint8_t  fl_pend;              /* bit per store: a write is waiting (debounce, budget, a low moment) */
    uint8_t  fl_err;               /* the last failed operation's FLASH_ERR_* (core/flashrec.h; 0: none) */
    uint16_t fl_errs;              /* failed operations since boot */
    uint32_t fl_rej;               /* records the loaders rejected at boot, 6 bits per store (the effect store: heads it
                                      lists that did not load) */
    uint16_t stagger_o2, stagger_o3;   /* adaptive stagger: TIM2 / TIM3 offsets, Q16 of the period */
    uint32_t stagger_moves, ripple_ma; /* moves so far, LED_+ ripple estimate (mA rms) */
    stagger_cap_t cap;             /* ripple load (core/stagger.h): updated by main.c once a second */
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
    /* voltage channels (core/vsense.h): PA4 the supply, PA0 the spare input on auxiliary input */
    uint32_t vs_supply_mv, vs_supply_min_mv, vs_supply_max_mv;   /* min / max: since the last status */
    uint32_t vs_aux_mv;
    const energy_t *en;            /* energy and the lifetime record (core/energy.h; always set: no NULL test) */
    const health_t *hl;            /* channel health (core/health.h), NULL = none */
    const vdip_t *dip;             /* supply dips (core/vdip.h), NULL = none */
    const vsense_t *dip_vs;        /* ... their scaling (the supply channel) */
    uint32_t dip_ref_mv;           /* their reference */
    uint8_t  test_mode;            /* test jig mode (core/testjig.h): the test pad is grounded */
    const uint32_t *uid;           /* the chip's 96-bit unique id (hw.h HW_UID; the test jig's line; always set) */
    const volatile uint16_t *hwd;  /* mcu/ch32v20x/hw.h hw_diag[HW_DIAG_N]: ADC freshness, DMA errors (NULL:
                                      none) */
    /* the overcurrent signal on status LED (core/ocled.h), every board */
    uint8_t  oc_show;              /* OCLED_*: 0 off, 1 limit, 2 trip */
    uint8_t  oc_cause;             /* the last trip's cause: 1 short, 2 overload */
    uint8_t  oc_limiting, oc_tripped;   /* the conditions now */
    uint32_t oc_since_s;           /* uptime (s) when the shown pattern started */
    uint32_t oc_hold_ms;           /* hold left (0 while the condition is on) */
    /* the power events (core/pwrev.h), every board: what a Diff-Power-UR-Analog-M reports (STATUS byte 8, the
     * POWER page) */
    const pwrev_t *pw;
    /* CPU load (core/cpuload.h): busy share of the last second, of the busiest second since the last status, 0.1 % */
    uint16_t cpu_pm, cpu_peak;
    const struct evlog *ev;        /* the event log (below), NULL = none */
#if MON_EXTRA
    const calhist_t *ch;           /* the calibration history (core/calhist.h), NULL = none. (New fields at
                                      the end: test_console_cz.c's random states stay as they were) */
#endif
    uint8_t  cal_fail;             /* the last test stopped incomplete: ISENSE_CAL_F_* (0: complete / none) */
    uint8_t  retry_n, retry_s;     /* the trips' retry policy */
    uint8_t  save_why;             /* why a pending write waits: 0 nothing pending, 1 debounce, 2 waiting for
                                      dark (the hour's write while bright used), 3 the write budget, 4 a flash error's
                                      hold-off, 5 the supply (main.c save_why) */
    uint32_t fw_word;           /* semantic version word, 0 development */
    /* the limits (the console's Y with ilimit50, lim_*, i_limit_ma, retry_*): the ratings, the user's board limit in
     * effect, each channel's ceiling in effect, a channel capped by the last render */
    uint16_t lim_board_ma, lim_fuse_ma, lim_ch_rated_ma, lim_user_ma;
    uint16_t ch_max_ma[5];
    uint8_t  ch_limited;
} cli_status_t;

typedef struct {
    void (*write)(const char *s);
#if CONSOLE_FULL
    cfg_result_t (*apply)(uint8_t cmd, uint8_t arg);    /* runs config_apply + the main loop's follow-up */
    void (*save_now)(void);
    void (*status_begin)(void);    /* snapshot and reset extrema only for an actual status command */
#endif
    const ultra_t *ul;
    const settings_t *set;
    const settings_t *eff;         /* the settings in effect (Ultra CONFIG over the stored ones) */
    const store_ext_t *ext;        /* the settings' extension (store.h: the current limit, options) */
    /* `restore`: a backup checked here (length, CRC, version, board); main.c applies its settings,
     * extension and calibration (as stored: 16 + 7 + 48 bytes) after its own checks: 1 = taken */
#if MON_EXTRA && CONSOLE_FULL
    int (*restore)(const uint8_t *data);
#endif
    const scene_t *scene;          /* power-on scene (stored, or about to be) */
    const cli_status_t *st;
#if CONSOLE_FULL
    char line[MON_EXTRA ? 168 : 48];   /* (a `restore` line is 158 + some slack; 48 without it) */
    uint16_t watch_ms;             /* `watch`: a status line every watch_ms (main.c), 0 = off; any key stops it */
    uint8_t len;
#endif
    uint8_t  st_part;              /* `status` / `help` in progress: the next part + 1 (0 = none; cli_poll) */
} cli_t;
/* the console output's next part (a `status` or `help` in progress): 1 while more remain. main.c calls it
 * while the UART has CLI_PART_ROOM bytes free (every part is shorter), and feeds no new line meanwhile. */
#if CONSOLE_FULL
#define CLI_PART_ROOM 512u
#else
#define CLI_PART_ROOM 240u
#endif
uint32_t cli_poll(cli_t *c);

#if CONSOLE_FULL
void cli_feed(cli_t *c, char ch);
#endif
/* a message of the firmware on the console: z = Z(name, "text\r\n") (core/cz.h; a '%' as "%%"); it writes through
 * c->write: only after main() has set it (it does, before the boot's first flash write) */
void cli_say(cli_t *c, int z);
/* the banner and the bootloader / hardware / app line: at boot, first in `status`, and `version`. The release:
 * "QuinLED <the board> v<version>, (C) 2026 QuinLED, GPL-3.0, <the source>"; the full console (one short line):
 * "QuinLED Dig2Analog-Plus-5ch v<version> (GPL-3.0)" */
void cli_banner(cli_t *c);
/* test jig mode, after each cycle: one line for a jig PC (the only report). Its format is fixed,
 * jig software parses it (README: Test jig):
 *   jig cycle=7 PASS fail=0 a=0.14/0.13/0.40/0.80/0.00 supply=24.01 uid=19088743,2309737967,4275878552 fw=0.7.0
 * ma: each channel's current at TJ_PEAK_PCT (mA; the line shows A); fail: core/testjig.c's verdict, PASS when 0, else
 * FAIL and its bits R 1, G 2, B 4, W1 8, W2 16 (a channel outside TJ_PASS_MIN_MA .. TJ_PASS_MAX_MA); the supply (mV,
 * 0 = not read: 0.00); the chip's unique id (st->uid: its three words at 0x1FFFF7E8, decimal) and the firmware version
 * (st->version: the rest of the line) */
void cli_test_report(cli_t *c, const int32_t ma[5], uint32_t fail, uint32_t cycle, uint32_t supply_mv);
/* `watch`: one line of the live values (the snapshot in st) */
void cli_watch(cli_t *c);
#define HW_DIAG_N 9u                /* cli_status_t.hwd's length: mcu/ch32v20x/hw.h's (the same definition: main.c
                                       sees both) */
#if CONSOLE_FULL
/* exposed for tests */
void cli_exec(cli_t *c, const char *line);
#endif

/* the event log: the last EVLOG_N events with the uptime they came at (main.c ev_add; RAM no reset but a
 * power-on clears: src/mcu/ch32v20x/Link_sections.ld .noinit; evlog_boot). Fed by what counts them already, no counter of its own: the power events (pwrev_t.ev: a limit event, a trip,
 * a supply dip) and the end of a calibration. `status` ends with them, newest first. EVL_LIMIT .. EVL_DIP are the
 * ULTRA_PW_EV_* bits 0 .. 3 in that order (main.c checks it). */
#define EVLOG_N 8u
enum { EVL_NONE, EVL_LIMIT, EVL_OVERLOAD, EVL_SHORT, EVL_DIP, EVL_CAL, EVL_CAL_FAIL, EVL_LATCH, EVL_OB_BAD,
       EVL_PROT_OK, EVL_PROT_FAIL, EVL_HOT };   /* (a test
                                                       stopped; the retries used up, the outputs off) */
typedef struct {
    uint32_t t_s;                  /* uptime, s */
    uint16_t ma;                   /* mA: the board current then (a short: its peak), a calibration's connected load */
    uint8_t  type;                 /* EVL_* (EVL_NONE: an empty entry) */
} evl_ev_t;
typedef struct evlog {
    uint32_t magic;                /* EVL_MAGIC: kept since the last power-on */
    evl_ev_t e[EVLOG_N];           /* a ring: e[head] the newest */
    uint8_t  head;
} evlog_t;
#define EVL_MAGIC 0x314C5645u      /* "EVL1" */
/* at boot: keep = the RAM was kept (the reset breadcrumbs valid: a watchdog / software / pin reset): the
 * last run's events stay, their times moved back by its uptime (prev_up_s: their ages go on); else a new log */
void evlog_boot(evlog_t *l, uint32_t keep, uint32_t prev_up_s);

#endif
