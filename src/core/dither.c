/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "dither.h"
#include "scratch.h"

/* spread `count` marks evenly over len slots (Bresenham), starting at `phase`; mark -> hi, else lo */
static void spread(uint32_t count, uint32_t len, uint32_t lo, uint32_t hi, uint8_t phase, uint16_t *dst,
                   uint8_t stride)
{
    uint32_t i, acc = 0, slot = phase % len;
    for (i = 0; i < len; i++) {
        uint32_t val = lo;
        acc += count;
        if (acc >= len) {
            acc -= len;
            val = hi;
        }
        dst[slot * stride] = (uint16_t)(val > 0xFFFFu ? 0xFFFFu : val);
        if (++slot == len) {
            slot = 0;
        }
    }
}

/* below the floor with policy SKIP: `pulses` pulses of floor..floor+1 counts carrying `total` counts */
static void skip_pattern(uint32_t total, uint32_t floor, uint32_t len, uint8_t phase, uint16_t *dst, uint8_t stride)
{
    uint32_t pulses = total / floor, i, acc = 0, acc_w = 0, slot = phase % len, base, extra;
    if (pulses == 0u) {                       /* less than one floor pulse in the whole pattern: round */
        if (2u * total < floor) {
            spread(0, len, 0, 0, phase, dst, stride);
            return;
        }
        pulses = 1;
        total = floor;
    }
    base = total / pulses;                    /* >= floor */
    extra = total - base * pulses;            /* pulses that get one count more */
    for (i = 0; i < len; i++) {
        uint32_t val = 0;
        acc += pulses;
        if (acc >= len) {
            acc -= len;
            val = base;
            acc_w += extra;
            if (acc_w >= pulses) {
                acc_w -= pulses;
                val++;
            }
        }
        dst[slot * stride] = (uint16_t)val;
        if (++slot == len) {
            slot = 0;
        }
    }
}

void dither_build(uint16_t v16, uint32_t steps, uint8_t len, const dither_opts_t *o, uint8_t phase,
                  uint16_t *dst, uint8_t stride)
{
    uint32_t num = (uint32_t)v16 * steps;     /* exact duty * 65535; <= 65535 * 65535 */
    uint32_t floor = o->floor_policy == FLOOR_OFF ? 0u : o->floor;
    uint32_t q, k = 0, i;

    if (len == 0u) {
        return;
    }
    if (floor != 0u && v16 != 0u && num < floor * 65535u) {          /* below the minimum pulse */
        if (o->floor_policy == FLOOR_CLAMP || len == 1u) {
            q = (2u * num >= floor * 65535u) ? floor : 0u;
            spread(0, len, q, q, phase, dst, stride);
        } else {
            uint32_t total = (num * len + 32767u) / 65535u;          /* counts over the pattern, < floor * len */
            skip_pattern(total, floor, len, phase, dst, stride);
        }
        return;
    }

    if (len == 1u || o->mode == DITHER_OFF || (o->mode == DITHER_OFF_LOW && v16 < o->threshold16)) {
        q = (num + 32767u) / 65535u;                                 /* nearest native step, no jump */
    } else {
        uint32_t r;
        q = num / 65535u;
        r = num - q * 65535u;
        k = (r * len + 32767u) / 65535u;                             /* r * len <= 65534 * 50 */
        if (k >= len) {
            q++;
            k = 0;
        }
    }
    spread(k, len, q, q + 1u, phase, dst, stride);

    /* ceiling: the off-time near 100 % must not be shorter than the floor either */
    if (floor != 0u && steps > floor) {
        for (i = 0; i < len; i++) {
            uint32_t v = dst[i * stride];
            if (v < steps && v > steps - floor) {
                dst[i * stride] = (uint16_t)(2u * (steps - v) < floor ? steps : steps - floor);
            }
        }
    }
}

/* ---------------------------------------------------------------------------------------------- spread */
#define VAR_MAX 64u

/* the ideal counts of each entry, v x period / 65535 = q + r / 65535, computed once per table: in the main loop's
 * shared scratch (scratch.h; the stagger search uses it at other times) */
uint32_t core_scratch[CORE_SCRATCH_WORDS];
#define var_q (core_scratch)
#define var_r ((uint16_t *)(core_scratch + VAR_MAX))
typedef char var_fits_scratch[(VAR_MAX * 4u + VAR_MAX * 2u <= CORE_SCRATCH_WORDS * 4u) ? 1 : -1];

/* x / 65535 without a divide: exact for x < 65535 x 65536 (v < 65535, period <= 65536) */
static uint32_t div65535(uint32_t x)
{
    return (x + 1u + (x >> 16)) >> 16;
}

/* one pass over the table: entry by entry from `phase`, ideal counts of each period (with the 16-bit remainder
 * carried in rem) plus the carried error acc; floor / ceiling / range decide what is emitted */
static void var_pass(const uint16_t *arr, uint32_t len, uint32_t floor, int nodither, uint8_t phase,
                     uint16_t *dst, uint8_t stride, uint8_t right_aligned, int32_t *acc_io)
{
    uint32_t i, slot = phase % len, rem = 32767u;            /* rounding of the table total: +-0.5 count */
    int32_t acc = *acc_io;
    for (i = 0; i < len; i++) {
        uint32_t p = (uint32_t)arr[slot] + 1u, q = var_q[slot], e;
        int32_t x;
        rem += var_r[slot];
        if (rem >= 65535u) {
            rem -= 65535u;
            q++;
        }
        x = nodither ? (int32_t)(var_q[slot] + (var_r[slot] >= 32768u)) : (int32_t)q + acc;
        if (floor != 0u && x < (int32_t)floor) {
            e = 0;                                           /* too short: skipped, carried on */
        } else if (x >= (int32_t)p) {
            e = p;
        } else if (floor != 0u && (uint32_t)x + floor > p) {
            e = 2u * (p - (uint32_t)x) < floor ? p : p - floor;   /* off-time too short: full on or the floor */
        } else {
            e = x < 0 ? 0u : (uint32_t)x;
        }
        if (!nodither) {
            acc = x - (int32_t)e;
        }
        dst[slot * stride] = (uint16_t)(right_aligned ? p - e : e);
        if (++slot == len) {
            slot = 0;
        }
    }
    *acc_io = acc;
}

/* write either the pulses (off_times = 0) or the off-times (1) given in dst back as compare values */
static void var_store(const uint16_t *arr, uint32_t len, uint16_t *dst, uint8_t stride, int off_times,
                      uint8_t right_aligned)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint32_t p = (uint32_t)arr[i] + 1u, x = dst[i * stride], e = off_times ? p - x : x;
        dst[i * stride] = (uint16_t)(right_aligned ? p - e : e);
    }
}

void dither_build_var(uint16_t v16, uint32_t steps, const uint16_t *arr, uint8_t len, uint8_t min_len,
                      const dither_opts_t *o, uint8_t phase, uint16_t *dst, uint8_t stride, uint8_t right_aligned)
{
    uint32_t floor = o->floor_policy == FLOOR_OFF ? 0u : o->floor, v = v16, rem = 32767u, i, on = 0, sum_p = 0;
    int nodither = len == 1u || o->mode == DITHER_OFF || (o->mode == DITHER_OFF_LOW && v16 < o->threshold16);
    int32_t acc = 0;

    (void)steps;
    if (len == 0u || len > VAR_MAX) {
        return;
    }
    if (v == 0u || v == 65535u) {                            /* off / on all the time */
        for (i = 0; i < len; i++) {
            dst[i * stride] = 0;
        }
        var_store(arr, len, dst, stride, v != 0u, right_aligned);
        return;
    }
    /* fast path: when no period meets the floor or the off-time ceiling, nothing is carried and one pass gives
     * the same table as the full path below (most levels) */
    {
        uint32_t slot = phase % len;
        for (i = 0; i < len; i++) {
            uint32_t p = (uint32_t)arr[slot] + 1u, prod = v * p, q = div65535(prod), r = prod - q * 65535u, x;
            if (nodither) {
                x = q + (r >= 32768u);
            } else {
                rem += r;
                x = q;
                if (rem >= 65535u) {
                    rem -= 65535u;
                    x++;
                }
            }
            if (x > p || (floor != 0u && (x < floor || x + floor > p))) {
                break;
            }
            dst[slot * stride] = (uint16_t)(right_aligned ? p - x : x);
            if (++slot == len) {
                slot = 0;
            }
        }
        if (i == len) {
            return;
        }
        rem = 32767u;
    }
    /* the ideal counts of each entry, and the table's exact on-time (rounded to a count) */
    for (i = 0; i < len; i++) {
        uint32_t p = (uint32_t)arr[i] + 1u, prod = v * p, q = div65535(prod), r = prod - q * 65535u;
        var_q[i] = q;
        var_r[i] = (uint16_t)r;
        rem += r;
        if (rem >= 65535u) {
            rem -= 65535u;
            q++;
        }
        on += q;
        sum_p += p;
    }
    if (floor != 0u) {
        uint32_t ml = min_len ? min_len : 1u, pmin = (len + ml - 1u) / ml, off = sum_p - on;
        /* below the minimum pulse on average: the table's on-time in evenly spaced pulses of at least the floor,
         * at least one per min_len periods (the plain table's rate, >= 2 kHz), else off. CLAMP: 0 or the floor. */
        if (on < len * floor) {
            if (o->floor_policy == FLOOR_CLAMP) {
                uint32_t e = 2u * on >= len * floor ? floor : 0u;
                for (i = 0; i < len; i++) {
                    dst[i * stride] = (uint16_t)e;
                }
            } else {
                if (on < pmin * floor) {
                    on = 2u * on >= pmin * floor ? pmin * floor : 0u;
                }
                skip_pattern(on, floor, len, phase, dst, stride);
            }
            var_store(arr, len, dst, stride, 0, right_aligned);
            return;
        }
        /* the same for the off-time near 100 % (no sliver off-pulses) */
        if (off < len * floor) {
            if (o->floor_policy == FLOOR_CLAMP) {
                uint32_t e = 2u * off >= len * floor ? floor : 0u;
                for (i = 0; i < len; i++) {
                    dst[i * stride] = (uint16_t)e;
                }
            } else {
                if (off < pmin * floor) {
                    off = 2u * off >= pmin * floor ? pmin * floor : 0u;
                }
                skip_pattern(off, floor, len, phase, dst, stride);
            }
            var_store(arr, len, dst, stride, 1, right_aligned);
            return;
        }
    }
    /* the table repeats: a first pass finds the carried error of the steady state, the second writes */
    var_pass(arr, len, floor, nodither, phase, dst, stride, right_aligned, &acc);
    var_pass(arr, len, floor, nodither, phase, dst, stride, right_aligned, &acc);
}
