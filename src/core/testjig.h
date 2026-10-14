/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Test jig mode (Quindor 2026-10-13: "a clearly visible pattern that the board has booted and all channels are
 * working, quick to check and continue"). A test jig grounds TP9 (PB10, internal pull-up; board.h BOARD_TESTJIG):
 *  - each channel R, G, B, W1, W2 in turn fades up to TJ_PEAK_PCT (Quindor: "No need to go to full, go to 33% max"),
 *    holds TJ_HOLD_MS and fades down again; the whole cycle takes the cycle time (TJ_CYCLE_MS, 1-2 s), repeated while
 *    the pad stays grounded;
 *  - the fade is the Ultra breathe effect's curve (ultra_breathe, sin^2); the values then take the normal output path: the
 *    board limit and the trips (isense), dim floor, dither, stagger, spread spectrum;
 *  - the board current over the end of each channel's hold, minus the zero, is that channel's current at the peak:
 *    after W2 the console prints the five, and the estimate at 100 % (cli_test_report), so a jig can check pass /
 *    fail. Display only;
 *  - D9 is on in test mode, off for TJ_LED_GAP_MS at each cycle start; the overcurrent and acknowledge patterns win;
 *  - data keeps being decoded and forwarded, but doesn't change the outputs while the pad is grounded.
 * Debounce: the pad has to read low for TJ_ENTER_MS in a row (sampled every ms from boot on) to enter, and high for
 * TJ_LEAVE_MS in a row to leave, back to normal operation without a reboot. A floating pad reads high (pull-up);
 * noise would have to hold it low for 50 samples in a row.
 * Factory reset by TP9 (0.6.0), a gesture a jig never makes: on a board that is already running (the pad has read
 * high since power-up; a jig that grounds it from power-up never counts), ground TP9 for TJ_RST_HOLD_MS or longer
 * (test mode runs meanwhile; then D9 blinks fast: armed), release it, and within TJ_RST_WAIT_MS ground it again for
 * TJ_RST_CONFIRM_MS. The second hold resets (main.c: as the console's `defaults`). Anything else (a shorter first hold,
 * no second one in time, a second one released early) does nothing.
 * Pure logic, host-tested (test/test_testjig.c). */
#ifndef TESTJIG_H
#define TESTJIG_H

#include <stdint.h>

#define TJ_ENTER_MS    50u
#define TJ_LEAVE_MS    50u
#ifndef TJ_CYCLE_MS
#define TJ_CYCLE_MS    1500u        /* R -> G -> B -> W1 -> W2, all five; tj_init clamps to 1000..2000 */
#endif
#define TJ_PEAK_PCT    33u          /* the peak of each channel's fade, % of full */
#define TJ_PEAK        ((uint32_t)((65535u * TJ_PEAK_PCT + 50u) / 100u))
#define TJ_HOLD_MS     30u          /* at the peak, per channel */
#define TJ_MEAS_MS     20u          /* the current: averaged over the hold's last 20 ms (the supply settled) */
#define TJ_LED_GAP_MS  100u
#define TJ_RST_HOLD_MS    5000u
#define TJ_RST_WAIT_MS    5000u
#define TJ_RST_CONFIRM_MS 1000u
/* pass / fail (0.6.5, the console's `jig` line, cli_test_report): each channel's current at TJ_PEAK_PCT within these
 * (build options; a jig PC may apply its own limits to the line's values) */
#ifndef TJ_PASS_MIN_MA
#define TJ_PASS_MIN_MA 50u          /* below: the channel or its load open (the zero's noise: ~+-20 mA) */
#endif
#ifndef TJ_PASS_MAX_MA
#define TJ_PASS_MAX_MA 3000u        /* above: ~9 A at full, over the board's 8 A: a short or the wrong load */
#endif
#define TJ_EV_MODE     0x01u        /* tj_pad: test mode turned on or off */
#define TJ_EV_RESET    0x02u        /* ... the reset gesture is complete */
enum { TJ_RST_IDLE = 0, TJ_RST_HOLD, TJ_RST_ARMED, TJ_RST_WAIT, TJ_RST_CONFIRM };

typedef struct {
    uint8_t  active;
    uint8_t  pad_low;               /* the pad's level ... */
    uint16_t run_ms;                /* ... held this long (saturating) */
    uint32_t t0;                    /* the cycle's origin: when test mode began */
    uint32_t cycle_ms, slot_ms, ramp_ms;
    int32_t  acc;                   /* the measuring window: sum of (current - zero), samples, channel */
    uint16_t n;
    uint8_t  meas_ch;
    int32_t  ma[5];                 /* each channel's current at the peak in the last cycle, mA */
    uint32_t cycles;                /* complete cycles (all five measured) */
    uint8_t  rst;                   /* the reset gesture's step (TJ_RST_*); >= TJ_RST_ARMED: D9 blinks fast */
    uint8_t  rst_ok;                /* the pad has read high since power-up */
    uint32_t rst_ms;                /* when the step began */
} testjig_t;

void    tj_init(testjig_t *t, uint32_t cycle_ms);
/* every ms: pad_low = the pad reads low. Returns TJ_EV_MODE when test mode turns on or off, TJ_EV_RESET when the
 * reset gesture is complete. */
uint8_t tj_pad(testjig_t *t, uint32_t now, uint8_t pad_low);
/* test mode: the five 16-bit outputs at now (one channel lit at a time) */
void    tj_outputs(const testjig_t *t, uint32_t now, uint16_t v[5]);
/* test mode, every ms: the board current of the last ms and its zero (mA). Returns 1 when a cycle's five currents
 * are complete (t->ma, t->cycles). */
uint8_t tj_measure(testjig_t *t, uint32_t now, int32_t i_ma, int32_t zero_ma);
/* D9's level in test mode, or while the reset gesture is armed (fast blink) */
uint8_t tj_led(const testjig_t *t, uint32_t now);

#endif
