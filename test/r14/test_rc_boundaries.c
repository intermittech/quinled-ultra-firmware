/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Two actual rc_schedule/rc_brr/rc_answer users on the existing UART/wire model.
 * RCS_HOST_US is the model's hub release only; receiver implementation unchanged. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../src/core/pwrev.h"
static int g_fail, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { g_fail++; printf("FAIL: "); printf(__VA_ARGS__); puts(""); } } while (0)
static unsigned long long rs = 0x140140140140ull;
static unsigned long long rnd(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs; }
static double urand(double a, double b) { return a + (b-a) * (double)(rnd() >> 11) / 9007199254740992.0; }
static unsigned rint_(unsigned n) { return (unsigned)(rnd() % n); }
static double r14_release;
#define RCS_HOST_US r14_release
#include "../test_rc.c"

int main(void)
{
    static const double releases[] = {0, 150, 500, 899, 919, 970, 1000, 1200, 3000};
    static rcs_t c;
    const uint8_t idx[2] = {0, 1};
    unsigned rate, rel, trial, windows = 0, good = 0, quiet = 0;
    for (rate = 0; rate < 3; rate++) for (rel = 0; rel < sizeof releases / sizeof releases[0]; rel++) {
        for (trial = 0; trial < 80; trial++) {
            int k;
            rcs_random_chain(&c, 2, 2, (int)rate);
            r14_release = releases[rel];
            (void)rcs_window(&c, idx, ULTRA_PAGE_STATUS, (uint8_t)trial);
            rcs_hub_uart(&c);
            if (r14_release <= 899) {
                CHECK(c.got_n[0] == 1 && c.got_n[1] == 1 && !memcmp(c.got[0],c.b[0].ans,32) &&
                      !memcmp(c.got[1],c.b[1].ans,32), "two receivers rate %u release %.0f trial %u: got %d/%d",
                      rate, r14_release, trial, c.got_n[0], c.got_n[1]);
                good += 2;
            }
            if (r14_release >= 1000) {
                CHECK(!c.b[0].sent && c.b[0].quiet && !c.got_n[0] && !c.got_n[1],
                      "late release %.0f: first receiver must stop relaying; got %d/%d", r14_release,c.got_n[0],c.got_n[1]);
                quiet++;
            }
            for (k = 0; k < 2; k++) {
                CHECK(!c.b[k].sent || c.b[k].tx_start >= c.t0 + ULTRA_RW_FIRST_US + rc_slot_us((uint8_t)rate)*(unsigned)k,
                      "receiver %d must not send payload early", k);
            }
            CHECK(c.late_fights == 0, "two receivers: payload contention at rate %u release %.0f", rate,r14_release);
            rcs_free(&c); windows++;
        }
    }
    printf("R14 telemetry: %u windows, %u expected answers, %u late-release quiet windows; %d checks, %d failures\n",
           windows, good, quiet, checks, g_fail);
    return g_fail ? 1 : 0;
}
