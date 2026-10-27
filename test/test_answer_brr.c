/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* USART1 at PCLK2 72 MHz on both receiver boards. Verify the three baud
 * dividers, oscillator correction, rounding bounds, fallback and PFIC vector. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "../src/core/rc.h"
#include "../src/target.h"

uint16_t crc16_ccitt(const uint8_t *d, uint32_t n)       /* (rc.c's rc_answer; not under test) */
{
    (void)d;
    (void)n;
    return 0;
}

static int checks, fails;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define WANT_BASE 72u
#define WANT_IRQN 53
#define WANT_CONSOLE 1

/* rate_q16 of a local clock `err` fast (its tick shorter): the host us per tick, Q16 */
static uint32_t rq(double err)
{
    return (uint32_t)(50.0 * 65536.0 / (1.0 + err));
}

int main(void)
{
    const uint32_t nominal = 50u << 16;
    double worst[3] = { 0, 0, 0 };
    uint32_t rate;
    int i;

    CHECK(RC_BRR_BASE == WANT_BASE, "RC_BRR_BASE %u, want %u", (unsigned)RC_BRR_BASE, WANT_BASE);
    CHECK(RC_IRQN == WANT_IRQN, "RC_IRQN %d, want %d", RC_IRQN, WANT_IRQN);
    CHECK(CONSOLE_TX == WANT_CONSOLE, "CONSOLE_TX %d, want %d", CONSOLE_TX, WANT_CONSOLE);
    for (rate = 0; rate < 3; rate++) {
        const uint32_t nom = WANT_BASE << rate;
        CHECK(rc_brr(nominal, (uint8_t)rate) == nom, "rate %u nominal: %u, want %u", rate, rc_brr(nominal, (uint8_t)rate), nom);
        /* the correction: every clock error from -5 % to +5 % in 0.01 % steps (a hundred thousandth of the clock) */
        for (i = -500; i <= 500; i++) {
            const double err = i / 10000.0, exact = nom * (1.0 + err);
            const uint32_t b = rc_brr(rq(err), (uint8_t)rate);
            const double resid = fabs(b / exact - 1.0);
            CHECK(fabs(b - exact) <= 0.5 + 0.01, "rate %u clock %+.2f %%: divider %u, exact %.3f", rate, err * 100, b, exact);
            if (resid > worst[rate]) {
                worst[rate] = resid;
            }
        }
        /* the half step bound as a fraction of the nominal divider */
        CHECK(worst[rate] <= 0.5 / nom * 1.1 + 1e-4, "rate %u worst residual %.3f %% over the half-step bound %.3f %%", rate,
              worst[rate] * 100, 50.0 / nom);
        /* the fallback: a clock 7 % or more off (either way) is not taken, nor a half or double rate */
        CHECK(rc_brr(rq(0.07), (uint8_t)rate) == nom && rc_brr(rq(-0.07), (uint8_t)rate) == nom &&
              rc_brr(rq(0.5), (uint8_t)rate) == nom && rc_brr(nominal / 2u, (uint8_t)rate) == nom &&
              rc_brr(nominal * 2u, (uint8_t)rate) == nom && rc_brr(0xFFFFFFFFu, (uint8_t)rate) == nom,
              "rate %u: an implausible clock gives the nominal divider", rate);
    }
    /* USART1's table at 72 MHz, stated by hand: a 2 % fast clock, a 2.1 % slow one, the plausibility edges */
    CHECK(rc_brr(rq(0.02), 0) == 73u && rc_brr(rq(0.02), 1) == 147u && rc_brr(rq(0.02), 2) == 294u,
          "USART1: +2 %%: %u %u %u, want 73 147 294", rc_brr(rq(0.02), 0), rc_brr(rq(0.02), 1), rc_brr(rq(0.02), 2));
    CHECK(rc_brr(rq(-0.021), 0) == 70u && rc_brr(rq(-0.021), 1) == 141u && rc_brr(rq(-0.021), 2) == 282u,
          "USART1: -2.1 %%: %u %u %u, want 70 141 282", rc_brr(rq(-0.021), 0), rc_brr(rq(-0.021), 1), rc_brr(rq(-0.021), 2));
    CHECK(rc_brr(rq(0.05), 0) == 76u && rc_brr(rq(-0.05), 0) == 68u,
          "USART1 1 Mbaud at +-5 %%: %u %u, want 76 68", rc_brr(rq(0.05), 0), rc_brr(rq(-0.05), 0));
    /* the cost of PCLK2: half the resolution at 1 Mbaud, 0.69 % at worst */
    CHECK(worst[0] > 0.0060 && worst[0] <= 0.0074, "USART1 1 Mbaud worst residual %.3f %% (about 0.69 %%)", worst[0] * 100);
    printf("answer UART %d, rc_brr base %u: worst residual %.2f / %.2f / %.2f %% (1M / 500k / 250k); %d checks, %d failures\n",
           ANSWER_UART, (unsigned)RC_BRR_BASE, worst[0] * 100, worst[1] * 100, worst[2] * 100, checks, fails);
    return fails != 0;
}
