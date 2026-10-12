/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Ultra mode light engine (board side of ULTRA_PROTOCOL.md).
 *
 * The host (ESPHome) sends targets; the board runs everything that changes over time:
 *  - transitions: M (state x brightness), A and B per channel interpolate linearly or with ESPHome's smoothstep,
 *    "via off" for color mode changes, starting at a shared-clock time (a late command catches up);
 *  - exact power-law gamma in fixed point from the full M x A x B product (no 8-bit / table steps),
 *    ESPHome's constant_brightness balance for CW / WW recomputed every update;
 *  - effects: pulse, strobe (steps), flicker, random (ESPHome behaviour) + candle, breathe, color loop,
 *    sunrise, sunset; time-based effects follow the shared clock, so boards stay in phase;
 *  - min / max power per channel, power limit per channel and per board.
 * Clock: each frame header carries the host time (us). The board tracks offset and rate of its own 50 us tick
 * against it, so all boards run on the same time base (HSI +-2 % corrected).
 * render() is called every 1 ms and returns 16-bit outputs for CH1..CH5. */
#ifndef ULTRA_H
#define ULTRA_H

#include <stdint.h>
#include "scene.h"
#include "ultra_protocol.h"

#define ULTRA_TICK_US        50u          /* local tick (20 kHz poll ISR) */
#define ULTRA_EV_CONFIG      0x01u        /* a CONFIG value changed: cfg[] / cfg_changed */
#define ULTRA_EV_IDENTIFY    0x02u
#define ULTRA_EV_LIGHT       0x04u        /* a light command was applied */
#define ULTRA_EV_SCENE       0x08u        /* a light's power-on scene changed (SCENE) */
#define ULTRA_EV_STATE       0x10u        /* a light got a state or an effect (SET / EFFECT): the host has taken over */
#define ULTRA_EV_DEF         0x20u        /* a light (re)defined (LIGHT_DEF): its outputs for a scene may differ */

typedef struct {
    uint16_t m;
    uint16_t a[5];
    uint16_t b[5];
} ultra_vals_t;

typedef struct {                          /* one interpolation from `from` to `to` */
    ultra_vals_t from, to;
    uint32_t t0, dur;                     /* host ms */
    uint8_t smooth, via_off;
} ultra_ramp_t;

typedef struct {
    uint16_t dur, trans;
    ultra_vals_t v;
} ultra_step_t;

typedef struct {
    /* definition (LIGHT_DEF) */
    uint8_t defined, mask, flags;         /* mask: channels with a role */
    uint8_t role[5];
    uint16_t min_p[5], max_p[5];
    uint32_t g_q16;                       /* gamma x 65536 */
    /* state: `cur` from t0 on, `prev` before (a command scheduled in the future) */
    ultra_ramp_t cur, prev;
    /* effect: the active one, and the next one (switches at its start time, also to FX_NONE) */
    uint8_t fx, fx_seq;
    uint8_t pend, pend_fx, pend_seq;
    uint32_t fx_t0, pend_t0;
    uint8_t fxp[ULTRA_FX_PARAMS], pend_p[ULTRA_FX_PARAMS];
    ultra_ramp_t fx_ramp;                 /* random / flicker / candle working values */
    uint32_t fx_next;                     /* next update, host ms */
    uint32_t fx_k;                        /* > 0 after fx_t0 was moved forward by whole periods (long runs) */
    uint32_t rng;
    uint32_t candle_f, candle_t;          /* Q16 factor and target */
    ultra_vals_t last;                    /* values of the last render */
    /* strobe steps */
    uint8_t step_count, step_seq, step_have;   /* step_have: bit mask of received steps for step_seq */
    ultra_step_t step[ULTRA_MAX_STEPS];
    /* power-on scene (SCENE): received since the last init, mode, take-over fade x 100 ms, values */
    uint8_t scene_have, scene_mode, scene_fade;
    ultra_vals_t scene_v;
    /* options (LIGHT_OPT): dim to warm, mireds x 10 of the CW / WW LEDs and of the warm end, red assist (Q16);
     * set points (ULTRA_OPT_POINTS): M and mireds x 10, sorted by M, descending */
    uint8_t opt_flags, opt_n;
    uint16_t opt_cw, opt_ww, opt_warm, opt_assist;
    uint16_t opt_pm[ULTRA_OPT_POINTS_MAX], opt_pt[ULTRA_OPT_POINTS_MAX];
} ultra_light_t;

typedef struct {
    uint8_t synced, rated;                /* rated: rate measured at least once */
    uint8_t gap;                          /* the local tick stopped for a while: restart the rate interval */
    uint32_t ref_tick, last_tick;
    uint64_t ref_us, last_us, ext_us;     /* host time, unwrapped to 64 bit */
    uint32_t rate_q16;                    /* host us per local tick, Q16 */
} ultra_clock_t;

typedef struct {
    ultra_light_t light[ULTRA_LIGHTS];
    ultra_clock_t clk;
    uint16_t last_seq[ULTRA_LIGHTS + 1][ULTRA_TYPES];   /* 0x100 = none */
    uint32_t frame_ms;                    /* host ms of the current frame header */
    uint16_t cfg[ULTRA_CFG_COUNT];
    uint32_t cfg_valid, cfg_changed;      /* bit masks by id */
    uint8_t identify;                     /* last IDENTIFY: ULTRA_IDENT_* 0..4 (unknown values are dropped) */
    uint8_t identify_arg;                 /* its second byte (calibration duty %) */
    uint8_t board_limit_ext;              /* set by main: the board limit (CONFIG 21) is measured (isense), so
                                             power_limit() leaves it out; set again after every ultra_init() */
    uint16_t out[5];
} ultra_t;

void     ultra_init(ultra_t *u, uint32_t seed);
/* frame header (the trailer) received: host time (us) at the end of the frame and the local tick of the frame's
 * last edge (0-50 us after it) */
void     ultra_on_header(ultra_t *u, uint32_t tick, uint32_t host_us);
/* one slot (32 bytes, CRC checked here); returns ULTRA_EV_* */
uint32_t ultra_on_slot(ultra_t *u, const uint8_t *slot, uint32_t tick);
/* outputs at local tick `tick`; returns 1 if any output changed */
uint8_t  ultra_render(ultra_t *u, uint32_t tick, uint16_t *out);
/* host time estimate for a local tick */
uint64_t ultra_host_us(const ultra_t *u, uint32_t tick);
/* all lights to off over dur_ms (signal-loss fade) */
void     ultra_all_off(ultra_t *u, uint32_t tick, uint32_t dur_ms);
/* the local tick did not run for a while (flash write): the next header restarts the rate measurement, the
 * phase error is corrected as usual */
void     ultra_clock_gap(ultra_t *u);
/* Power-on scene from the lights' SCENE values, computed like render() (gamma, constant brightness, min / max
 * power, power limit). Channels of a defined light that has not sent its SCENE yet keep `stored`; channels of no
 * defined light are not part of it. */
void     ultra_scene(const ultra_t *u, const scene_t *stored, scene_t *out);

/* exact gamma: x (Q32 fraction, M x A) -> Q16 output; g_q16 = gamma x 65536 (exposed for tests) */
uint32_t ultra_gamma(uint32_t x_q32, uint32_t g_q16);
/* ESPHome's transition curve 6x^5 - 15x^4 + 10x^3, Q16 */
uint32_t ultra_smoothstep(uint32_t p_q16);

extern const uint32_t ultra_log2_tab[257];
extern const uint32_t ultra_exp2_tab[257];
extern const uint16_t ultra_sin_tab[65];

#endif
