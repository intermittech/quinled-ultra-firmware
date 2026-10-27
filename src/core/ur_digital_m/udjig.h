/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Diff-Power-UR-Digital-M's test jig mode (BOARD 3; decided 2026-10-24): the analog boards' contract (core/testjig.h:
 * the pad, the cycle's verdict, the status LED) on a pixel strip, lean for the V203's flash; the verdict goes out as the
 * console's j record (cycle, fail, a[5]: the analog boards' `jig` line's values; decided 2026-10-24, its text did not fit
 * the V203).
 *  - The pad: jig input (hw.h HW_TP6_LOW, pull-up) grounded, or a UART break: test mode after UJ_DEBOUNCE_MS low in a
 *    row, back to normal operation after as long high (no reboot; no reset gesture on this board).
 *  - The cycle: six UJ_SLOT_MS slots (1.5 s), counted only while the pattern is on the strip: all off (the zero), then
 *    each logical channel in turn (R, G, B, W1, W2): the first UJ_PIXELS pixels, whatever the strip's length, with that
 *    channel's raw byte at UJ_LEVEL (no gamma, no limiter), the others 0. One channel of 16 pixels at 25 % draws ~80 mA
 *    at most (20 mA LEDs) on any strip. Channels the chip doesn't have stay dark.
 *  - The current: the board's ~64 ms bus-current average (main_ur_digital_m.c i_avg) at each slot's last ms, settled
 *    to ~2 % of the step after 249 ms, minus the zero slot's. After the fifth
 *    channel: the verdict over the chip's channels (slots 0 .. bpp-1 within UJ_PASS_MIN_MA .. UJ_PASS_MAX_MA; the dark
 *    slots read ~0 and are outside it) and the j record.
 *  - Status LED: on, off in each zero slot until the first verdict; then the last verdict: PASS steady, FAIL 5 Hz.
 * main_ur_digital_m.c puts the pattern on the strip (cal_work: the calibration's raw-frame ownership) and sends the record.
 * Pure logic, host-tested (test/test_udjig.c). */
#ifndef UDJIG_H
#define UDJIG_H

#include <stdint.h>

#define UJ_DEBOUNCE_MS 50u
#define UJ_SLOT_MS     250u
#define UJ_PIXELS      16u
#define UJ_LEVEL       64u          /* 25 %: the calibration's duty ... */
#define UJ_LEVEL_SHIFT 6u           /* ... as a shift */
#ifndef UJ_PASS_MIN_MA
#define UJ_PASS_MIN_MA 5u           /* below: nothing lit (data path or strip open); 16 x ~3 mA on a 5 V strip */
#endif
#ifndef UJ_PASS_MAX_MA
#define UJ_PASS_MAX_MA 400u         /* above: more than the pattern can draw: a short or the wrong load */
#endif

typedef struct {
    /* (the bytes in this order: the ones stored together sit at even offsets, so no merged halfword store is
       misaligned: udjig.c's layout check, the image's site scan) */
    uint8_t  run;                   /* ms the pad has read against the mode (the debounce) */
    uint8_t  on;                    /* test mode */
    uint8_t  fail;                  /* the last verdict: bit n = slot n outside the limits */
    uint8_t  led;                   /* status LED's level in test mode */
    uint8_t  say;                   /* test mode turned on or off: the X record is due (the glue clears it) */
    uint8_t  rep;                   /* a cycle's currents and verdict are complete: the j record is due (... clears it) */
    uint16_t ph;                    /* ms into the cycle */
    int32_t  zero;                  /* the zero slot's mean, mA */
    int32_t  ma[5];                 /* each channel's current in the last cycle, mA */
    uint32_t cycles;                /* complete cycles since test mode began */
} udjig_t;

/* every ms (now): the pad, the bus current of the last ms (mA), live = the pattern is on the strip (the cycle runs), bpp =
 * the chip's channels. Sets say, rep and led */
void     uj_ms(udjig_t *j, uint32_t now, uint32_t pad_low, int32_t i_ma, uint32_t live, uint32_t bpp);
/* the pattern now into fb: UJ_PIXELS pixels of bpp wire bytes (ord[k]: wire byte k's logical channel); all off outside
 * test mode (the release's black frame). Returns the frame's bytes */
uint32_t uj_frame(const udjig_t *j, const uint8_t *ord, uint32_t bpp, uint8_t *fb);

#endif
