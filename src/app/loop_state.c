/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* loop_state.c: the main loop's state: the settings and their extension, the stores' pending writes and budgets, the
 * event log, the monitors, the power-on scene and effects; say, settings_changed, ev_add.
 * A unit of src/app/main.c: main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* ------------------------------------------------------------------------------- main loop state */
static settings_t set;                     /* stored settings */
static store_ext_t sext;                   /* stored with them: current limit and calibration (store.h) */
/* the trips' retry policy in the extension's spare bytes 2 / 3 (retry_set), coded so that an erased byte
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
/* the console never waits for the UART (mcu/ch32v20x/hw.c uart_write waits while both 512-byte halves are full): the
 * reply to a line (each <= CLI_PART_ROOM: test_r5_bounds), a `status` / `help` part, a `watch` line and the test jig's
 * report go out only while the UART has CLI_PART_ROOM bytes free (the next line waits in the RX ring meanwhile, and a
 * due watch line or report a pass later); the firmware's own messages (a flash error, test mode, reset input's reset) wait
 * here, in order, and the main loop writes one a pass while the UART has SAY_ROOM free. Nothing is dropped: a fifth
 * message at once (never seen) is written at once (cli_say). */
#define SAY_ROOM 64u                       /* (every message is shorter) */
#define SAY_Q    4u                        /* (a power of two) */
static int say_q[SAY_Q];
static uint8_t say_h, say_n;
#if !CONSOLE_FULL
#define STREAM_STATUS_S 60u                /* the release console: the full status every minute */
/* the receiver's release console sends no full status: its return channel's pages (STATUS, LIFETIME,
 * DIAG, ...) carry it; the banner, the live line, the jig's lines and the messages stay (CONSOLE_FULL 1: everything) */
#define STREAM_STATUS   1
#define STREAM_BOOT_S   5u                 /* ... the first once the live values are valid (the MCU temperature's first
                                              average, ~2 s: the supply's comes before), at the latest at this uptime */
#if STREAM_STATUS
static uint32_t status_s;                  /* ... the uptime of the last one */
static uint8_t status_first;               /* ... 1: the first one is out */
static uint32_t cfg_fp;                    /* ... and early when the settings' fingerprint moves (settings_moved) */
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
static uint16_t out_now[5];                /* output values in use, = the value each channel's table holds (one
                                              vector; main loop only, no ISR or build() reads it) */
static uint8_t built_ok;
static uint16_t st_o2 = STAGGER_FIXED_O2, st_o3 = STAGGER_FIXED_O3;
static uint8_t st_force;
static uint16_t st_seen[5];
static uint32_t st_ms;
static uint8_t st_moved = 1;               /* build_all ran since stagger_adapt last compared out_now with st_seen (the
                                              compare's answer can't change before: out_now is written there alone) */
/* power-on scene (core/scene.h): shown from power-up until data comes, kept in its own flash log */
#define SCENE_TAKEOVER_MS 15000u           /* Ultra: hold it at most this long after the board's first slot (each light
                                              takes over at its SET: this is for channels the host never sends; from
                                              the first slot, not the first header: the hub's 50-board preamble comes
                                              between them, app/frames.c) */
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
/* the overcurrent signal on status LED (core/ocled.h) and status LED's level as set; the power events (core/pwrev.h: the state a
 * Diff-Power-UR-Analog-M reports on its STATUS / POWER pages; local here) */
static ocled_t ocl;
static uint8_t led_now;
static pwrev_t pw;
/* the event log (core/cli.h evlog_t): main loop only */
static evlog_t evl __attribute__((section(".noinit.evl")));   /* (kept across a reset, evlog_boot) */
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
/* voltage channels (core/vsense.h): PA4 the supply, PA0 the spare input on auxiliary input */
static vsense_t vs_sup, vs_aux;
static vdip_t vdip;                        /* supply dips on PA4 (core/vdip.h) */
static energy_t en;                        /* input energy (core/energy.h) */
#if BOARD == BOARD_UR_ANALOG_M
/* the receiver's standby. IDENTIFY 7, the hub's planned power-off notice (vdip.planned,
 * PLAN_CUT_MS): a supply dip starting meanwhile is the planned cut's for its whole length (core/vdip.h in_dip 2:
 * nothing counted or logged, no SUPPLY / STATUS / fault memory / power event) and writes the lifetime record only for
 * something worth storing (energy_due ENERGY_PLANNED: not the power-up alone); the outputs fade dark, an acknowledge
 * pattern too; a calibration stops. Still powered PLAN_HOLD_MS after it: the outputs as before the notice. Nothing is kept for the
 * next start: the grace window decides it (every power-up dark until a valid frame or GRACE_MS, then the power-on
 * scene). A slow decay (big port caps, 48 V): vdip's reference holds from the notice (core/vdip.c), a
 * dip starting up to PLAN_CUT_MS after it is still the cut's, and the outputs stay dark while the supply falls (3 %
 * below the reference, vdip.falling) and through the dip */
#define PLAN_HOLD_MS 2000u                 /* the outputs dark this long from the notice */
#define PLAN_CUT_MS  10000u                /* a dip starting this long after the notice is the cut's */
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
static calhist_t chist;                    /* the calibration history (core/calhist.h), its write pending */
static uint8_t chist_pend;
static uint8_t chist_retry;                /* its last write failed: each try again takes a token of */
static scene_budget_t chist_budget;        /* ... this budget, as the calibration's (CAL_BUDGET_CAP, then one per
                                              CAL_REFILL_MS); before, a failing write ran again every main-loop pass
                                              while low, erasing its page every fourth try */
#endif
static health_t hl;                        /* channel health (core/health.h) */
static sstart_t ss;                        /* soft start (core/sstart.h) */
static uint8_t ul_used, ul_stated;
/* the Diff-Power-UR-Analog-M (BOARD=2) has no test pad: the jig's code and console texts aren't built there */
#if BOARD_TESTJIG
static testjig_t tj;                       /* test jig mode (core/testjig.h): reset input grounded */
static uint8_t tj_rep;                     /* a cycle's report waits for the UART's room */
#define TJ_ACTIVE() (tj.active)
#else
#define TJ_ACTIVE() 0u
#endif
#if BOARD_RETURN
static uint8_t rc_pend;                    /* frx.slot holds a POLL from the start of frame rc_frame, whose end is not
                                              processed yet */
static uint32_t rc_frame;
static uint16_t rc_uid16;                  /* CRC-16 over the unique id: in every answer */
static uint16_t ls_res;                    /* the last LIFE_SYNC's: energy_sync's (applied | result << 8) */
static uint16_t rc_windows, rc_missed;     /* windows taken part in; POLLs this board couldn't take part in */
static uint32_t cal_done_s;                /* uptime + 1 at the last calibration test (0: none since power-up) */
/* a window may come or runs: the stagger search and the flash wait (the window's deadline, ULTRA_RW_DIR_MAX_US) */
#define RC_BUSY() (rc_pend || rc_state != RC_IDLE)
#endif

static void flash_stall_begin(uint32_t may);
static uint32_t flash_stall_end(uint32_t r);
