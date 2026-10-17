/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Additional sample-exact delay-line scenarios on the existing independent wire model. */
#include "../../src/core/dline.h"
static uint32_t r14_origin;
static void r14_init(dline_t *d, uint32_t pos);
#define dline_init r14_init
#define main r14_original_dline_main
#include "../test_dline.c"
#undef main
#undef dline_init

static void r14_init(dline_t *d, uint32_t pos)
{
    dline_init(d, pos);
    /* Same ring offsets, simulated word uptime immediately before wrap. */
    d->wpos += r14_origin; d->zpos += r14_origin;
    d->zs += r14_origin; d->os += r14_origin; d->lzs += r14_origin;
    d->edge_w += r14_origin; d->rec_w += r14_origin;
    d->seq = 253;  /* capture ownership crosses 255 -> 1 within the window sequence */
}

int main(void)
{
    static const double clocks[] = {-0.022, 0, 0.022};
    static const double releases[] = {5, 150, 230, 260, 950};
    unsigned wr, c, r, p, rate, slow, scenarios = 0;
    char name[100];
    for (wr = 0; wr < 2; wr++) for (c = 0; c < 3; c++) for (rate = 0; rate < 3; rate++)
    for (r = 0; r < 5; r++) for (p = 0; p < 6; p++) for (slow = 0; slow < 2; slow++) {
        unsigned i;
        r14_origin = wr ? 0xFFFFF000u : 0;
        sprintf(name, "r14 wrap%u clk%+.3f rate%u rel%.0f phase%u slow%u", wr, clocks[c], rate, releases[r], p, slow);
        sc_begin(name, C_WS2812);
        S.hsi = clocks[c]; S.tick_phase = (int)p * 300;
        S.jit_us = 45;
        S.ml_min_us = slow ? 1000 : 20;
        S.ml_us = slow ? 1100 : 300;
        LOW(400);
        for (i = 0; i < 4; i++) {
            PIX(4, 1, 350);
            POLL(releases[r], 3, 20, (int)rate, slow || releases[r] > 250 ? 0 : 1);
        }
        PIX(4, 2, 0);
        go(); scenarios++;
    }
    printf("R14 dline: %u scenarios, %ld checks, %d failures; word/sequence wrap, +/-2.2%% clock, late tick/decode/release\n",
           scenarios, g_checks, g_fail);
    return g_fail ? 1 : 0;
}
