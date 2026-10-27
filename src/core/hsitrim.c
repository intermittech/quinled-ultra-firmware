/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The hub-locked HSI trim: see hsitrim.h. Until a step has been measured, a move up assumes the largest plausible
 * step (it cannot overshoot nominal) and a move down the smallest (it lands below the target); afterwards the learned
 * step, whose sign is the direction. */
#include "hsitrim.h"

void hsitrim_init(hsitrim_t *t, uint32_t trim, int32_t step_ppm)
{
    t->step_ppm = step_ppm;
    t->last_err = 0;
    t->trim = (uint8_t)trim;
    t->last_trim = (uint8_t)trim;
    t->stepped = 0;
    t->locked = 0;
    t->learned = 0;
    t->steps = 0;
}

/* trim units (signed) that move the error by about delta_ppm at |step| ppm a unit in direction dir (+1 / -1) */
static int32_t hsitrim_steps(int32_t delta_ppm, int32_t step, int32_t dir, int32_t round_up)
{
    const int32_t mag = delta_ppm < 0 ? -delta_ppm : delta_ppm;
    int32_t n = round_up ? (mag + step - 1) / step : mag / step;
    if ((delta_ppm < 0) != (dir < 0)) {
        n = -n;
    }
    return n;
}

uint32_t hsitrim_update(hsitrim_t *t, int32_t err_ppm)
{
    int32_t n = 0, next, dir, s;
    if (t->stepped) {                                    /* learn the step from what the last step did */
        const int32_t dt = (int32_t)t->trim - (int32_t)t->last_trim;
        if (dt != 0) {
            const int32_t m = (err_ppm - t->last_err) / dt, a = m < 0 ? -m : m;
            if (a >= HSITRIM_STEP_MIN_PPM && a <= HSITRIM_STEP_MAX_PPM) {
                t->step_ppm = m;
                t->learned = 1;
            }
        }
        t->stepped = 0;
    }
    dir = t->step_ppm < 0 ? -1 : 1;
    s = t->step_ppm < 0 ? -t->step_ppm : t->step_ppm;
    if (err_ppm > -HSITRIM_MARGIN_PPM) {
        /* above the target (or too close): down, the move rounded up so it lands at or below the target */
        n = hsitrim_steps(-HSITRIM_MARGIN_PPM - err_ppm, t->learned ? s : HSITRIM_STEP_MIN_PPM, dir, 1);
    } else {
        const int32_t up = t->learned ? s : HSITRIM_STEP_ASSUME_PPM;
        if (err_ppm + up <= -HSITRIM_MARGIN_PPM - HSITRIM_HYST_PPM) {   /* up, never past the target (hysteresis: no
                                                                          dithering across it on measurement noise) */
            n = hsitrim_steps(-HSITRIM_MARGIN_PPM - HSITRIM_HYST_PPM - err_ppm, up, dir, 0);
        } else if (!t->learned) {
            n = -dir;                                    /* below target, step unknown: one step down (the safe way)
                                                            measures it, then the exact way back up */
        }
    }
    t->locked = (uint8_t)(n == 0);
    if (n == 0) {
        return t->trim;
    }
    next = (int32_t)t->trim + n;
    next = next < 0 ? 0 : next > 31 ? 31 : next;
    if (next == (int32_t)t->trim) {
        t->locked = 1;                                   /* at the end of the range: as close as it gets */
        return t->trim;
    }
    t->last_trim = t->trim;
    t->last_err = err_ppm;
    t->trim = (uint8_t)next;
    t->stepped = 1;
    t->steps++;
    return t->trim;
}
