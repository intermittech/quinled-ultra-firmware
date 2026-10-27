/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Board current sensing.
 *
 * Pure logic, host-tested (test/test_isense.c). Two call sites:
 *  - the 20 kHz poll tick: isense_tick() with the newest ADC result (continuous conversion). It sums the samples
 *    and runs the fast trip; on a trip the caller forces all PWM outputs off at once (hardware, no table rebuild).
 *  - the main loop: isense_ms() once per ms with the ISR's sum of that ms, and isense_apply() wherever the outputs
 *    are recomputed (it scales them to the board limit, or plays the calibration pattern).
 *
 * What it does:
 *  - board current = mean of the samples (exact average at every PWM preset, independent of the ripple);
 *  - soft limit: all five channels scaled by one factor k so the board stays at the limit (colour kept);
 *    with a model of the channel currents (calibration / regression) k follows the content at once (feed-forward,
 *    no pumping), otherwise it is a feedback limiter: instant down, slow release;
 *  - fast trip (ISR): ISENSE_FAST_TICKS ticks at >= 1.5 x fuse rating (a short) -> outputs off;
 *    overload trip (main): the current stays above the limit's trip level for ISENSE_OVL_MS -> outputs off;
 *    retry after 1 s, doubling up to 32 s, reset after 60 s without a trip;
 *  - calibration: all off (zero), then each channel alone at 25 / 50 / 75 / 100 % (the standard test, 0.95 s) or
 *    at one set duty; the model of each channel = its current at 100 % (I_k) and a curve through the 25 / 50 / 75 %
 *    points (supply / cable sag at high current, strips with their own regulators), linear between them;
 *  - refinement: NLMS regression of the measured current on the applied duties through each channel's curve
 *    (16 ms windows), invisible; it refines I_k, the curve shape stays the calibrated one;
 *  - zero tracking: logic current + amplifier offset, measured whenever all outputs are off. */
#ifndef ISENSE_H
#define ISENSE_H

#include <stdint.h>
#include "../mcu/profile.h"

#define ISENSE_CH          5u
#define ISENSE_TICK_HZ     MCU_POLL_HZ     /* isense_tick() rate (poll tick) */
#define ISENSE_ADC_MAX     4095u
#define ISENSE_FAST_TICKS  MCU_TICKS_US(100u)         /* consecutive ticks over the fast-trip level: 50-100 us */
#define ISENSE_OVL_MS      200u       /* overload trip: this long above the trip level */
#define ISENSE_RETRY_COUNT 10u        /* ("10 hiccups with 1 s in between") the retries after a
                                         trip, retry_s apart (ULTRA_CFG_RETRY) */
#define ISENSE_RETRY_S     1u
#define ISENSE_RETRY_RESET_MS 60000u  /* this long without a trip: the count starts again */
#define ISENSE_WIN_MS      16u        /* regression / zero window */
#define ISENSE_CAL_SETTLE_MS 10u      /* calibration, single level: the zero and each step, settle + measure */
#define ISENSE_CAL_MEAS_MS 20u
#define ISENSE_LEVELS      4u         /* the standard test: 25 / 50 / 75 / 100 % per channel */
/* the standard test (wiki: Current sense, calibration): the zero 20 + 30 ms, each level 15 + 30 ms,
 * 50 + 5 x 4 x 45 = 950 ms. The settle covers a slow supply's recovery from the load step; the measure averages
 * >= 65 PWM periods at every preset. */
#ifndef ISENSE_CAL_ZERO_SETTLE_MS
#define ISENSE_CAL_ZERO_SETTLE_MS 20u
#endif
#ifndef ISENSE_CAL_ZERO_MEAS_MS
#define ISENSE_CAL_ZERO_MEAS_MS 30u
#endif
#ifndef ISENSE_CAL_LVL_SETTLE_MS
#define ISENSE_CAL_LVL_SETTLE_MS 15u
#endif
#ifndef ISENSE_CAL_LVL_MEAS_MS
#define ISENSE_CAL_LVL_MEAS_MS 30u
#endif
#define ISENSE_CAL_STEPS_MAX (1u + ISENSE_CH * ISENSE_LEVELS)
#define ISENSE_RELEASE_Q16 128u       /* feedback limiter release per ms: k x (1 + 128/65536), ~350 ms 0.5 -> 1 */

/* events of isense_ms() */
#define ISENSE_EV_TRIP     0x01u      /* overload trip: force the PWM outputs off now */
#define ISENSE_EV_RELEASE  0x02u      /* retry: restore the PWM outputs */
#define ISENSE_EV_REAPPLY  0x04u      /* k or the calibration pattern changed: recompute the outputs (isense_apply) */
#define ISENSE_EV_CAL_DONE 0x08u      /* calibration finished: I_k / zero valid (persist them) */
#define ISENSE_EV_CAL_FAIL 0x10u      /* calibration aborted (cal_fail: why; the stored test and the model unchanged) */
#define ISENSE_EV_LATCH    0x20u      /* the retries used up: the outputs stay off (isense_rearm) */
/* cal_fail: how the last test ended (0: complete, or none since power-up) */
#define ISENSE_CAL_F_TRIP   1u        /* a trip */
#define ISENSE_CAL_F_LIMIT  2u        /* the installation's limit (the limiter's threshold) reached */
#define ISENSE_CAL_F_SUPPLY 3u        /* a supply dip (main.c) */

/* trip causes */
#define ISENSE_TRIP_NONE   0u
#define ISENSE_TRIP_FAST   1u         /* ISR: short-circuit level */
#define ISENSE_TRIP_OVL    2u         /* main: overload the limit could not hold */

typedef struct {
    /* ---- ISR side (isense_tick) ---- */
    volatile uint32_t acc;            /* sum of samples since the main loop's last take */
    volatile uint32_t acc_n;
    volatile uint16_t fast_raw;       /* fast-trip level in ADC counts (0xFFFF = off) */
    volatile uint8_t  fast_cnt;
    volatile uint8_t  tripped;        /* ISENSE_TRIP_*: outputs are forced off */
    volatile uint16_t peak_raw;       /* highest sample since the main loop's last take */
    uint16_t cal_duty;                /* single level: Q16 */

    /* ---- configuration ---- */
    uint32_t ma_per_lsb_q16;          /* gain: mA per ADC count, Q16 (default from the circuit, factory-trimmable) */
    uint16_t fuse_ma;                 /* soldered fuse rating (8000 / 10000) */
    uint16_t board_ma;                /* board rating: default and maximum of the limit */
    uint16_t limit_ma;                /* soft limit in effect (remote: Ultra CONFIG 21 / config frame), <= board_ma */
    uint8_t  enabled;                 /* hardware present (runtime check) */
    uint8_t  win_n;

    /* ---- measurement ---- */
    int32_t  i_ms;                    /* last ms: board current, mA (incl. zero) */
    int32_t  i_avg;                   /* ~100 ms average, mA (status / host) */
    int32_t  i_acc;                   /* ... as it is filtered: i_avg x 64 (6 fraction bits), so it settles on i_ms */
    int32_t  zero_ma;                 /* logic current + amplifier offset, mA */
    uint8_t  zero_valid;
    uint8_t  win_dark;                /* every ms of the window had all outputs at 0 */
    uint16_t peak_ma;                 /* highest sample of the last ms, mA */

    /* ---- limiter ---- */
    uint32_t k;                       /* scale of all outputs, Q16 (65536 = none): min(feed-forward, k_fb) */
    uint32_t k_fb;                    /* feedback limiter state, Q16 */
    uint32_t k_meas;                  /* the k that was in effect during the measured ms */
    uint32_t g;                       /* model gain correction, Q16 */
    uint16_t req[ISENSE_CH];          /* last requested outputs */
    uint16_t applied[ISENSE_CH];      /* last applied outputs (0 while tripped) */
    uint8_t  limiting;                /* k < 1 */
    uint32_t k_ss;                    /* soft start (core/sstart.h, set by main.c): a factor on the outputs only, Q16
                                         (65536 = none); the limiter, `limiting` and k don't see it */

    /* ---- model of the channels: I_k(d) = ch_ma x curve(d) ---- */
    int32_t  ch_ma[ISENSE_CH];        /* LED current of each channel at 100 %, mA */
    uint16_t shape[ISENSE_CH][ISENSE_LEVELS - 1u];  /* current at 25 / 50 / 75 % over that at 100 %, Q15
                                                       (linear: 8192 / 16384 / 24576) */
    uint8_t  calibrated;              /* ch_ma from a calibration (or restored from flash) */
    uint8_t  dark_ms;                 /* consecutive dark ms before the window */
    uint16_t conf;                    /* regression confidence (windows with a small prediction error) */

    /* ---- trips ---- */
    uint8_t  trip_cause;              /* last trip */
    uint8_t  retry_n;                 /* retries since the count started (saturates at 254) */
    uint8_t  retry_max;               /* 0 never retry, 1-254, 255 forever (isense_init: ISENSE_RETRY_COUNT) */
    uint8_t  retry_s;                 /* ... seconds apart (0: 1 s) */
    uint8_t  latched;                 /* the retries used up: tripped until isense_rearm */
    uint32_t trips;                   /* count since boot */
    uint32_t ovl_ms;                  /* ms above the overload level */
    uint32_t off_ms;                  /* ms since the trip */
    uint32_t calm_ms;                 /* ms since the last trip release */

    /* ---- calibration ---- */
    uint8_t  cal_step;                /* 0 idle, 1 zero, 2.. channel (step - 2) / levels, level (step - 2) % levels */
    uint8_t  cal_fail;                /* the last test's end: 0 complete / none, ISENSE_CAL_F_* */
    uint8_t  cal_levels;              /* ISENSE_LEVELS (the standard test) or 1 (one level at cal_duty) */
    uint16_t cal_t;                   /* ms in the step */
    int32_t  cal_sum;                 /* <= 30 ms of i_ms (< 2^21 mA even at a gain of 1): 32 bits */
    uint32_t cal_n;
    int32_t  cal_res[ISENSE_CAL_STEPS_MAX];   /* zero, then each step, mA */
    int16_t  cal_zero;                /* the last test (the stored calibration), mA */
    uint16_t cal_ma[ISENSE_CH][ISENSE_LEVELS];  /* channel current at each level, zero subtracted, mA */
    uint8_t  cal_have;                /* cal_levels of the last test (0 = none: model from flash ext / learned) */
    uint8_t  cal_pct1;                /* single level: its duty, % */

    /* ---- regression / zero windows ---- */
    int32_t  win_i;                   /* sum of i_ms over the window (16 ms: < 2^21 mA) */
    uint32_t win_d[ISENSE_CH];        /* sum of the applied duties (Q16) over the window */
} isense_t;

/* Default firmware conversion: 4.004 mA per ADC count, Q16 (a 200 mV per A sense path). */
#define ISENSE_GAIN_DEFAULT_Q16 262410u

/* a calibration test as stored (core/calstore.h) and reported */
typedef struct {
    uint8_t  levels;                  /* ISENSE_LEVELS: the standard test; 1: one level at pct[0]; 0: none */
    uint8_t  pct[ISENSE_LEVELS];      /* duty of each level, % */
    int16_t  zero_ma;                 /* logic current + amplifier offset, mA */
    uint16_t ma[ISENSE_CH][ISENSE_LEVELS];   /* channel current at each level, zero subtracted, mA (0 = nothing) */
} isense_cal_t;

void isense_init(isense_t *s, uint8_t enabled, uint16_t fuse_ma, uint16_t board_ma);
/* soft limit in mA: 0 = the board rating; clamped to it */
void isense_set_limit(isense_t *s, uint16_t limit_ma);
void isense_set_gain(isense_t *s, uint32_t ma_per_lsb_q16);
/* channel currents at 100 % (e.g. restored from flash); marks the model calibrated */
void isense_set_model(isense_t *s, const int32_t ch_ma[ISENSE_CH], int32_t zero_ma);
/* start a calibration: zero, then each channel alone at 25 / 50 / 75 / 100 % (duty 0, the standard test) or at
 * `duty` (Q16, e.g. 16384 = 25 %; the curve is then linear) */
void isense_cal_start(isense_t *s, uint16_t duty);
/* stop a running test, incomplete (why: ISENSE_CAL_F_*): ISENSE_EV_CAL_FAIL | _REAPPLY, 0 if none ran */
uint32_t isense_cal_abort(isense_t *s, uint8_t why);
/* latched (the retries used up): retry at the next ms, the count from 0 (a new command or frame value) */
void isense_rearm(isense_t *s);
/* length of a calibration in ms (duty as for isense_cal_start) */
uint32_t isense_cal_ms(uint16_t duty);
/* the last test (CAL_DONE) / restore one from flash (the model follows it, marked calibrated) */
int  isense_cal_get(const isense_t *s, isense_cal_t *c);   /* 0: no test since boot / restore */
void isense_cal_set(isense_t *s, const isense_cal_t *c);
/* model: channel current at output v (Q16), mA; the board's LED current for outputs v; the current while a channel
 * is on (its average over v: the stagger weight); the connected load (all channels at 100 %, 0 = no model) */
uint32_t isense_ch_ma(const isense_t *s, uint32_t ch, uint32_t v);
uint32_t isense_model_ma(const isense_t *s, const uint16_t v[ISENSE_CH]);
uint32_t isense_on_ma(const isense_t *s, uint32_t ch, uint32_t v);
uint32_t isense_connected_ma(const isense_t *s);
/* the channel model is trusted (calibrated, or learned with a small error) */
int  isense_model_ok(const isense_t *s);
/* calibration <-> flash (store_ext_t): channel currents x 25 mA (0xFF = none), zero in mA (0xFF = none) */

/* every ms: sum / n = isense_tick() samples of that ms (taken with interrupts off: isense_take) */
uint32_t isense_ms(isense_t *s, uint32_t sum, uint32_t n, uint32_t peak_raw);
/* the outputs to show for the requested ones (may be the same array) */
void isense_apply(isense_t *s, const uint16_t req[ISENSE_CH], uint16_t out[ISENSE_CH]);
/* ms until the retry after a trip (0 = not tripped) */
uint32_t isense_retry_left_ms(const isense_t *s);

/* ---------------------------------------------------------------------------------------------- ISR side */
/* 20 kHz poll tick: raw = newest ADC result. Returns 1 when the outputs must be forced off now (fast trip). */
static inline uint32_t isense_tick(isense_t *s, uint32_t raw)
{
    s->acc += raw;
    s->acc_n++;
    if (raw > s->peak_raw) {
        s->peak_raw = (uint16_t)raw;
    }
    if (raw >= s->fast_raw) {
        if (++s->fast_cnt >= ISENSE_FAST_TICKS && s->tripped == ISENSE_TRIP_NONE) {
            s->tripped = ISENSE_TRIP_FAST;
            return 1;
        }
    } else {
        s->fast_cnt = 0;
    }
    return 0;
}

/* main loop: take the ISR's sums (call with interrupts off) */
static inline void isense_take(isense_t *s, uint32_t *sum, uint32_t *n, uint32_t *peak)
{
    *sum = s->acc;
    *n = s->acc_n;
    *peak = s->peak_raw;
    s->acc = 0;
    s->acc_n = 0;
    s->peak_raw = 0;
}

#endif
