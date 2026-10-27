/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Test jig mode (decided 2026-10-13: "a clearly visible pattern that the board has booted and all channels are
 * working, quick to check and continue"). A test jig grounds a pad with an internal pull-up (board.h BOARD_TESTJIG,
 * tj_pads, mcu/ch32v20x/hw.h HW_TP6_LOW / HW_TP9_LOW): jig input (PA3, the console's RX) on both boards (held low by the jig
 * or by a UART break, with no byte received meanwhile), on the Dig2Analog-Plus-5ch also reset input (PB10). The fixture: GND,
 * power, console TX (the console's TX: the `jig` line) and jig input. Then (what a jig needs, no fades):
 *  - TJ_GAP_MS all off at the cycle start (the current sense's zero reference), then each channel R, G, B, W1, W2 in
 *    turn at TJ_PEAK_PCT ("No need to go to full, go to 33% max") for its slot (the cycle time TJ_CYCLE_MS,
 *    1-2 s, minus the gap, in five), repeated while the pad stays low; the values take the normal output path: the
 *    board limit and the trips (isense), dim floor, dither, stagger, spread spectrum;
 *  - the board current over each slot's last TJ_MEAS_MS (the supply settled), minus the zero, is that channel's
 *    current; after W2 the console prints the `jig` line (cli_test_report: the five, pass / fail against
 *    TJ_PASS_MIN_MA .. TJ_PASS_MAX_MA, the supply, the chip's UID, the firmware) for a jig PC. Display only;
 *  - status LED: on, off in each cycle's gap until the first verdict; then the last verdict: PASS steady on, FAIL 5 Hz
 *    (a PC-less jig); the overcurrent and acknowledge patterns win;
 *  - data keeps being decoded and forwarded, but doesn't change the outputs while the pad is grounded.
 * Debounce: the pad has to read low for TJ_ENTER_MS in a row (sampled every ms from boot on) to enter, and high for
 * TJ_LEAVE_MS in a row to leave, back to normal operation without a reboot. A floating pad reads high (pull-up);
 * noise would have to hold it low for 50 samples in a row.
 * Factory reset by reset input, a gesture a jig never makes: on a board that is already running (the pad has read
 * high since power-up; a jig that grounds it from power-up never counts), ground reset input for TJ_RST_HOLD_MS or longer
 * (test mode runs meanwhile; then status LED blinks fast: armed), release it, and within TJ_RST_WAIT_MS ground it again for
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
#define TJ_PEAK_PCT    33u          /* each channel's level, % of full */
#define TJ_PEAK        ((uint32_t)((65535u * TJ_PEAK_PCT + 50u) / 100u))
#define TJ_MEAS_MS     20u          /* the current: averaged over the slot's last 20 ms (the supply settled) */
#define TJ_GAP_MS      100u         /* all off at each cycle start */
#define TJ_RST_HOLD_MS    5000u
#define TJ_RST_WAIT_MS    5000u
#define TJ_RST_CONFIRM_MS 1000u
/* pass / fail (the console's `jig` line, cli_test_report): each channel's current at TJ_PEAK_PCT within these
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
    uint32_t cycle_ms, slot_ms;
    int32_t  acc;                   /* the measuring window: sum of (current - zero), samples, channel */
    uint16_t n;
    uint8_t  meas_ch;
    int32_t  ma[5];                 /* each channel's current at the peak in the last cycle, mA */
    uint32_t cycles;                /* complete cycles (all five measured) */
    uint8_t  fail;                  /* the last cycle's verdict: bit n = channel n outside the pass limits */
    uint8_t  rst;                   /* the reset gesture's step (TJ_RST_*); >= TJ_RST_ARMED: status LED blinks fast */
    uint8_t  rst_ok;                /* the pad has read high since power-up */
    uint32_t rst_ms;                /* when the step began */
    uint32_t rx_pos;                /* the console's RX position at the last tj_pads */
} testjig_t;

void    tj_init(testjig_t *t, uint32_t cycle_ms);
/* every ms, the pads for tj_pad: 1 = grounded. jig input (the console's RX) low counts only with no byte received
 * since the last call (rx_pos: the UART's RX position; a jig grounding the pad or holding a break delivers at most one
 * 0x00, console traffic keeps delivering, so it never counts however low the line reads); reset input low (the Dig2Analog-
 * Plus-5ch's) always */
uint8_t tj_pads(testjig_t *t, uint8_t tp6_low, uint32_t rx_pos, uint8_t tp9_low);
/* every ms: pad_low = the pad reads low. Returns TJ_EV_MODE when test mode turns on or off, TJ_EV_RESET when the
 * reset gesture is complete. */
uint8_t tj_pad(testjig_t *t, uint32_t now, uint8_t pad_low);
/* test mode: the five 16-bit outputs at now (one channel lit at a time) */
void    tj_outputs(const testjig_t *t, uint32_t now, uint16_t v[5]);
/* test mode, every ms: the board current of the last ms and its zero (mA). Returns 1 when a cycle's five currents
 * are complete (t->ma, t->cycles, t->fail). */
uint8_t tj_measure(testjig_t *t, uint32_t now, int32_t i_ma, int32_t zero_ma);
/* status LED's level in test mode (the verdict, see above), or while the reset gesture is armed (10 Hz) */
uint8_t tj_led(const testjig_t *t, uint32_t now);

#endif
