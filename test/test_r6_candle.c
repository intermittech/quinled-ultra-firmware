/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included by test_soak.c.
 * r6candle  A power-on candle with no host for 26 days (a niche without ESPHome, examples/
 *           quinled-dig2analog-plus-candle-niche.yaml): the flame must not freeze once the effect's ms clock is 2^31 ms
 *           (24.86 days) old (light_values taking the effect time as negative would hold it at the effect's start, and
 *           a rebase that keeps pulse / strobe / breathe / colour loop running but skips the effects without a period
 *           (candle, flicker, random) would leave them frozen until day 49.7). The same for a candle the host started and left on.
 *           Expected: the flame flickers in a window 24.9 days after power-up as at boot. */
static void sc_r6candle(void) { sc_host_effects(); }
