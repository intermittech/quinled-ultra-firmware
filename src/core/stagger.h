/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Adaptive stagger (STAGGER_ADAPTIVE / _ADAPTIVE_SPREAD): TIM2 / TIM3 phase offsets that minimise the LED_+ ripple current.
 *
 * Pulse layout per PWM period (fractions of the period, Q16, 65536 = one period), as in pwm_start():
 *   TIM1 (reference): R [0, dR), G [-dG, 0) (right-aligned)
 *   TIM2 at o2:       B [o2, o2 + dB)
 *   TIM3 at o3:       W1 [o3, o3 + dW1), W2 [o3 - dW2, o3) (right-aligned)
 * The supply current is sum(I_k x pulse_k). Its mean does not depend on the offsets; its variance is
 * sum(I_k^2 d_k) + 2 sum(I_j I_k overlap_jk) - mean^2, so the search minimises the weighted pairwise overlaps.
 * Overlaps across timers depend only on o2 (TIM1-TIM2), o3 (TIM1-TIM3) and o3 - o2 (TIM2-TIM3): three tables of
 * STAGGER_GRID entries, then the 2D minimum over the grid is additions only, then a finer local refinement.
 * Edge rule: two edges in the same direction (two turn-ons or two turn-offs) of channels on different timers must
 * be at least `gap` apart (simultaneous di/dt, EMI); a turn-on next to a turn-off is fine (the currents cancel).
 * Channel order: R, G, B, W1, W2 (board.h CH_*). Duties: 16-bit output values (65535 = always on). */
#ifndef STAGGER_H
#define STAGGER_H

#include <stdint.h>

#define STAGGER_GRID      24u             /* coarse step T/24 (1/3 and 2/3 on the grid), then T/96 around the best */
#define STAGGER_FIXED_O2  21845u          /* 1/3: the fixed stagger */
#define STAGGER_FIXED_O3  43691u          /* 2/3 */

/* Spread spectrum (STAGGER_*_SPREAD): every PWM period gets its own length, from a table shared by all three timers.
 * The lengths are an even grid of +-6 % around the period, shuffled (pseudo-random, no short repeat inside the
 * table), so the average frequency stays the preset's. At 2.197 kHz the period can only get shorter: grid -12..0 %.
 * The offsets of TIM2 / TIM3 then stay below 85 % of the period (a wrap after TIM1's next one would confuse the
 * phase measurement of the slew). */
#define STAGGER_SPREAD_Q16  3932u         /* 6 % */
#define STAGGER_SPREAD_LEN  32u           /* table entries at least (longer when the dither table is) */
#define STAGGER_SPREAD_OMAX 55705u        /* 0.85 of the period */

typedef struct {
    uint16_t o2, o3;                      /* chosen offsets, Q16 */
    uint32_t ripple_now, ripple_best;     /* AC ripple at the current / chosen offsets, mA rms */
    uint8_t  valid_now;                   /* current offsets satisfy the edge rule */
    uint8_t  move;                        /* worth moving (hysteresis), offsets in o2 / o3 */
} stagger_result_t;

/* AC ripple of the summed LED current, mA rms. peak_ma: LED current of each channel when on. */
uint32_t stagger_ripple_ma(const uint16_t *duty, const uint16_t *peak_ma, uint16_t o2, uint16_t o3);

/* 1 if the offsets satisfy the edge rule (gap Q16) */
int stagger_valid(const uint16_t *duty, uint16_t o2, uint16_t o3, uint16_t gap);

/* best offsets for these duties; move = 1 when they beat the current offsets by > 15 % and > 300 mA rms, or the
 * current offsets break the edge rule and the new ones are valid and not more than 10 % worse */
void stagger_choose(const uint16_t *duty, const uint16_t *peak_ma, uint16_t gap, uint16_t cur_o2, uint16_t cur_o3,
                    stagger_result_t *r);
/* the same, offsets limited to o_max (Q16, 0 = no limit; spread spectrum) */
void stagger_choose_max(const uint16_t *duty, const uint16_t *peak_ma, uint16_t gap, uint16_t cur_o2,
                        uint16_t cur_o3, uint16_t o_max, stagger_result_t *r);

/* spread spectrum period table: arr[k] = period length - 1 of entry k (len entries, len >= 2) */
void stagger_spread_arr(uint32_t steps, uint32_t len, uint16_t *arr);

/* edge gap as a fraction of the period: 0.5 us at a period of `steps` counts of 144 MHz */
uint16_t stagger_gap_q16(uint32_t steps);

/* Phase slew, called once per PWM period of TIM2 / TIM3 (at its update): its wrap is `cur` counts after TIM1's
 * and should be `tgt`; `pending` = length change already written for the period in progress. Returns the length
 * change for the next period (-max_step..max_step); 0 with pending 0 = arrived. allow_longer = 0 when the period
 * cannot be made longer (2.197 kHz: ARR is already 65534), then it only shortens periods, going the long way. */
int32_t stagger_slew(uint32_t cur, uint32_t tgt, int32_t pending, uint32_t steps, uint32_t max_step,
                     int allow_longer);
/* spread spectrum: offsets stay in 0..o_max, so the slew goes straight (never across the period wrap) */
int32_t stagger_slew_straight(uint32_t cur, uint32_t tgt, int32_t pending, uint32_t max_step);

#endif
