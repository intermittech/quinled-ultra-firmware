/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* UART diagnostics console (USART2 on TP5 TX / TP6 RX, 115200 8N1, line based, CR or LF ends a line).
 *   help | ?            command list
 *   status              firmware, uptime, reset cause, protocol, frame / error counters, frame rate, outputs
 *   get                 all settings as key=value
 *   set <key> <value>   change a setting (same validation, blink feedback and saving as config frames)
 *   period <steps>      raw PWM period, 1440..65535 timer counts (144 MHz / steps = PWM frequency)
 *   save                store settings now          defaults   factory reset (settings and power-on scene)
 *   identify            blink the feedback channel / D9
 *   version             the firmware / bootloader lines (as at boot and in status), and blink the version
 * Boards with the input current sense: set ilimit <x 50 mA>, set ical <%> (calibration: 0 = the standard test at
 * 25 / 50 / 75 / 100 %), status shows the current, the calibration and the board limit in effect.
 * Every board: status shows the MCU (die) temperature and the 3.3 V rail from the internal sensor / VREFINT. */
#ifndef CLI_H
#define CLI_H

#include <stdint.h>
#include "config.h"
#include "scene.h"

typedef struct {
    const char *version;           /* "0.6.0-beta.1", or FW_VERSION + " (development build)" */
    uint8_t  bl_version;           /* the bootloader's (UPD_BL_INFO), 0 = none (standalone build) */
    uint8_t  hw_id, hw_rev;        /* from the bootloader info */
    uint8_t  app_state;            /* 0 standalone, 1 trial (not confirmed), 2 confirmed */
    char     bl_build[9];          /* the bootloader's build id */
    uint32_t uptime_ms;
    uint32_t reset_flags;          /* RCC RSTSCKR at boot */
    uint8_t  proto_mode, proto_locked, hdr_bits;
    uint8_t  proto_provisional;    /* auto: the locked protocol is the stored one, not confirmed yet */
    uint32_t frames;               /* frame ends seen */
    uint32_t decoded;              /* slots decoded */
    uint32_t dec_err[6];           /* by -WS_ERR_* code */
    uint32_t config_frames;        /* accepted config commands */
    uint32_t ultra_headers;        /* Ultra active: frames of an Ultra length whose tail held a valid header */
    uint32_t ultra_tail_err;       /* ... and those whose tail did not (decode error, gap, CRC) */
    uint32_t fps;                  /* frames in the last second */
    uint16_t out[5];               /* current output levels, 16 bit */
    uint32_t steps;                /* PWM period in counts */
    uint32_t saves;                /* flash records written since boot */
    uint16_t stagger_o2, stagger_o3;   /* adaptive stagger: TIM2 / TIM3 offsets, Q16 of the period */
    uint32_t stagger_moves, ripple_ma; /* moves so far, LED_+ ripple estimate (mA rms) */
    /* board current sense (HW_ISENSE, amplifier present); shown when isense = 1 */
    uint8_t  isense;
    uint8_t  ilimit50;             /* stored board limit x 50 mA (0 = the board rating) */
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
    uint8_t  cal_levels;           /* 0 none, 1 one level, 4 the standard test */
    uint8_t  cal_pct[4];
    uint16_t cal_count;            /* tests stored on this board */
    uint16_t cal_ma[5][4];         /* channel current at each level, mA */
    /* MCU temperature (core/mcutemp.h), every board */
    int16_t  mcu_temp_c10;         /* 0.1 C, MCUTEMP_NA (-32768) = not valid */
    uint16_t vdda_mv;              /* the 3.3 V rail from VREFINT, 0 = not known */
} cli_status_t;

typedef struct {
    void (*write)(const char *s);
    cfg_result_t (*apply)(uint8_t cmd, uint8_t arg);    /* runs config_apply + the main loop's follow-up */
    void (*save_now)(void);
    const settings_t *set;
    const scene_t *scene;          /* power-on scene (stored, or about to be) */
    const cli_status_t *st;
    char line[48];
    uint8_t len;
} cli_t;

void cli_feed(cli_t *c, char ch);
/* "QuinLED dig2analog-ultra firmware v<version>, Copyright (C) 2026 QuinLED, GPL-3.0, source: ..." and the
 * bootloader / hardware / app line: at boot, first in `status`, and `version` */
void cli_banner(cli_t *c);
/* exposed for tests */
void cli_exec(cli_t *c, const char *line);

#endif
