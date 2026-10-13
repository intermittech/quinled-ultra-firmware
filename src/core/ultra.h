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
 *  - min / max power per channel, power limit per channel (the board limit is the current sense's);
 *  - power-on effects (SCENE_EFFECT, 0.6.0): each light's persisted head (below) is stored by main.c, and at
 *    power-up the engine runs the effect on the light's SCENE values without a controller.
 * Clock: each frame header carries the host time (us). The board tracks offset and rate of its own 50 us tick
 * against it, so all boards run on the same time base (HSI +-2 % corrected).
 * render() is called every 1 ms and returns 16-bit outputs for CH1..CH5. */
#ifndef ULTRA_H
#define ULTRA_H

#include <stddef.h>
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

/* 4-byte aligned on the target (24 bytes): its copies (ramps, steps, the last render) are word copies; at 22 bytes, a
 * `to` at 2 mod 4 made memcpy copy byte by byte */
#if defined(__GNUC__)
#define ULTRA_ALIGN4 __attribute__((aligned(4)))
#else
#define ULTRA_ALIGN4
#endif

typedef struct {
    uint16_t m;
    uint16_t a[5];
    uint16_t b[5];
} ULTRA_ALIGN4 ultra_vals_t;

typedef struct {                          /* one interpolation from `from` to `to` */
    ultra_vals_t from, to;
    uint32_t t0, dur;                     /* host ms */
    uint8_t smooth, via_off;
} ultra_ramp_t;

typedef struct {
    uint16_t dur, trans;
    ultra_vals_t v;
} ultra_step_t;

#define ULTRA_SCENE_FX_PARAMS 14u          /* the power-on effect's parameters kept (the effects' longest: 13) */

typedef struct {
    /* the persisted head (ULTRA_HEAD_BYTES; main.c stores it for the power-on effect, hal/hw.c): definition
     * (LIGHT_DEF), options (LIGHT_OPT), power-on scene (SCENE) and effect (SCENE_EFFECT) */
    uint32_t g_q16;                       /* gamma x 65536 */
    ultra_vals_t scene_v;                 /* SCENE values */
    uint16_t min_p[5], max_p[5];
    /* options (LIGHT_OPT): dim to warm, mireds x 10 of the CW / WW LEDs and of the warm end, red assist (Q16);
     * set points (ULTRA_OPT_POINTS): M and mireds x 10, sorted by M, descending */
    uint16_t opt_cw, opt_ww, opt_warm, opt_assist;
    uint16_t opt_pm[ULTRA_OPT_POINTS_MAX], opt_pt[ULTRA_OPT_POINTS_MAX];
    uint8_t mask, flags;                  /* mask: channels with a role */
    uint8_t role[5];
    uint8_t opt_flags, opt_n;
    uint8_t scene_mode, scene_fade;       /* SCENE mode, take-over fade x 100 ms */
    uint8_t scene_fx;                     /* SCENE_EFFECT: the power-on effect (ULTRA_FX_NONE = none) */
    uint8_t scene_fxp[ULTRA_SCENE_FX_PARAMS];
    /* RAM only */
    uint8_t defined;                      /* LIGHT_DEF received since the init */
    uint8_t scene_have, scene_fx_have;    /* SCENE / SCENE_EFFECT received since the init */
    uint8_t scene_run;                    /* the power-on effect runs (from power-up until the take-over) */
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
} ultra_light_t;

#define ULTRA_HEAD_BYTES ((uint32_t)offsetof(ultra_light_t, defined))   /* the persisted head (98 on the target) */
/* the power-on effect store (hal/hw.c writes it, two flash pages): magic (4) | light mask (1) | CRC-8 per light (5) |
 * 2 spare (0), then light i's head at ULTRA_FX_HDR + i x ULTRA_HEAD_BYTES. The header is written last. */
#define ULTRA_FX_MAGIC   0x31584653u      /* "SFX1" */
#define ULTRA_FX_HDR     12u

typedef struct {
    uint8_t synced, rated;                /* rated: rate measured at least once */
    uint8_t gap;                          /* the local tick stopped for a while: restart the rate interval */
    uint32_t ref_tick, last_tick;
    uint64_t ref_us, last_us;             /* host time, unwrapped to 64 bit */
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
    uint8_t ch_limited;                   /* the last ultra_render() capped a channel at its CONFIG 16-20 limit (power
                                             events, pwrev.h); the board limit (CONFIG 21) is the current sense's */
    uint16_t out[5];
} ultra_t;

void     ultra_init(ultra_t *u, uint32_t seed);
/* frame header (the trailer) received: host time (us) at the end of the frame and the local tick of the frame's
 * last edge (0-50 us after it) */
void     ultra_on_header(ultra_t *u, uint32_t tick, uint32_t host_us);
/* one slot (32 bytes, CRC checked here); returns ULTRA_EV_* */
uint32_t ultra_on_slot(ultra_t *u, const uint8_t *slot, uint32_t tick);
/* the same for a slot whose CRC is already checked (frame_rx: frx_capture fills frx.slot only after the check) */
uint32_t ultra_apply_slot(ultra_t *u, const uint8_t *slot, uint32_t tick);
/* outputs at local tick `tick`; returns 1 if any output changed */
uint8_t  ultra_render(ultra_t *u, uint32_t tick, uint16_t *out);
/* host time estimate for a local tick */
uint64_t ultra_host_us(const ultra_t *u, uint32_t tick);
/* all lights to off over dur_ms (signal-loss fade) */
void     ultra_all_off(ultra_t *u, uint32_t tick, uint32_t dur_ms);
/* the local tick did not run for a while (flash write): the next header restarts the rate measurement, the
 * phase error is corrected as usual */
void     ultra_clock_gap(ultra_t *u);
/* every ms (main.c): after ULTRA_TEND_TICKS without a header (none yet: the power-on effect; the host gone while
 * signal loss holds) the clock's reference moves along its line, so the engine's time stays continuous over any
 * uptime and across the 32-bit tick's wrap */
#define ULTRA_TEND_TICKS 0x20000000u      /* 7.5 h of ticks */
void     ultra_clock_tend(ultra_t *u, uint32_t tick);
/* Power-on scene from the lights' SCENE values, computed like render() (gamma, constant brightness, min / max
 * power, power limit). Channels of a defined light that has not sent its SCENE yet keep `stored`; channels of no
 * defined light are not part of it. */
void     ultra_scene(const ultra_t *u, const scene_t *stored, scene_t *out);

/* the power-on effect (SCENE_EFFECT): the lights in `mask` (their heads restored, e.g. from flash) run their effect on
 * their SCENE values from local tick `tick` on, without a controller; stop: the controller takes over (their effect
 * ends, they render only if it defined them) */
void     ultra_scene_fx_start(ultra_t *u, uint8_t mask, uint32_t tick);
/* from a store image: the heads of its valid lights (CRC ok) into u; returns their mask, their CRCs into crc
 * (core/scene.c, with the scene log: the engine itself stays free of the firmware's other modules) */
uint8_t  ultra_fx_load(ultra_t *u, const uint8_t *img, uint8_t crc[ULTRA_LIGHTS]);
void     ultra_scene_fx_stop(ultra_t *u);
/* a light's head is a power-on effect to store: defined, SCENE mode 1 and an effect it can run from power-up */
int      ultra_scene_fx_ok(const ultra_light_t *L);

/* exact gamma: x (Q32 fraction, M x A) -> Q16 output; g_q16 = gamma x 65536 (exposed for tests) */
uint32_t ultra_gamma(uint32_t x_q32, uint32_t g_q16);
/* ESPHome's transition curve 6x^5 - 15x^4 + 10x^3, Q16 */
uint32_t ultra_smoothstep(uint32_t p_q16);
/* the breathe effect's curve: sin^2(pi ph) for ph 0..65536 (Q16), Q16; 0 -> 0, 0.5 -> full, 1 -> 0 (also the test
 * jig's fade, core/testjig.c) */
uint32_t ultra_breathe(uint32_t ph_q16);

extern const uint32_t ultra_log2_poly[7];     /* 1 - log2(2 - u), u^1..u^7, Q29 (ultra_gamma; gen_ultra_tables.py) */
extern const uint32_t ultra_exp2_poly[6];     /* 2^t, Q30 */
extern const uint16_t ultra_sin_tab[65];

#endif
