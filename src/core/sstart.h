/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Soft start (0.6.0): after power-up (the first light) and after a trip's retry, the outputs ramp from 20 to 100 %
 * over SS_RAMP_MS, starting SS_OFFSET_MAX_MS or less late (the offset comes from the chip's unique id). The boards of a
 * whole pin (a Diff-Power port) then don't switch their load on together. The only 0.6.0 feature that changes the
 * outputs, and only for those ~0.4 s. Applied as a factor on the current sense's output scaling (core/isense.h
 * isense_apply, k_ss): the limiter, its events and D9's limit blink don't see it. Not in test jig mode or during a
 * calibration (their patterns stay as they are). On by default; the console's `softstart` / Ultra CONFIG 23 store it.
 * Pure logic, host-tested (test/test_sstart.c). */
#ifndef SSTART_H
#define SSTART_H

#include <stdint.h>

#define SS_RAMP_MS        300u
#define SS_OFFSET_MAX_MS  100u
#define SS_START_Q16      13107u    /* 20 % */

enum { SS_ARMED = 0, SS_RUN = 1, SS_DONE = 2 };

typedef struct {
    uint8_t  state;                 /* SS_*: waiting for the first light, running (offset, then the ramp), done */
    uint8_t  offset_ms;
    uint32_t t0;                    /* when it ran up (the first light / the retry) */
} sstart_t;

/* uid = a hash of the chip's unique id: the offset 0..SS_OFFSET_MAX_MS */
void     ss_init(sstart_t *s, uint32_t uid);
/* a trip's retry: offset and ramp again */
void     ss_retry(sstart_t *s, uint32_t now);
/* the outputs' factor now, Q16 (65536 = none): lit = an output is asked to be on, on = soft start enabled */
uint32_t ss_factor(sstart_t *s, uint32_t now, uint8_t lit, uint8_t on);

#endif
