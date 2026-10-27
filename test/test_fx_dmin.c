/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The packer's AUTO rule for dim channels (ULTRA_PXS_AUTO_MIN_LEVEL, 4): in the dither a channel whose wire level is below dmin is rounded, not dithered; at dmin and above it dithers.
 * Built by test\build_fx.cmd against fx_pack.c and the tables (the header's level). */
#include <stdio.h>
#include <string.h>
#include "../src/core/fx.h"
#include "../src/core/ultra_protocol.h"

extern uint32_t fx_gamma16_of(uint32_t v);
static int g_fail, g_checks;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
    uint32_t c, ph, boundary = 0, rounded = 0, dithered = 0;
    printf("test_fx_dmin: ULTRA_PXS_AUTO_MIN_LEVEL = %u\n", (unsigned)ULTRA_PXS_AUTO_MIN_LEVEL);
    CHECK(ULTRA_PXS_AUTO_MIN_LEVEL == 4u, "the header's level is %u, not 4", (unsigned)ULTRA_PXS_AUTO_MIN_LEVEL);
    /* every input level of one channel (green), DITHER with dmin 4 against dmin 0: below wire level 4 (a 16-bit value under 4 x 256) the output is the same in all 256 frames
     * (rounded); at 4 and above it still dithers wherever the value has a fraction */
    for (c = 0; c < 256u; c++) {
        const uint32_t v = fx_gamma16_of(c);
        uint8_t first4 = 0, first0 = 0;
        uint32_t var4 = 0, var0 = 0;
        for (ph = 0; ph < 256u; ph++) {
            fx_pack_cfg_t cfg;
            rgb_t px;
            uint8_t w[3];
            px.r = 0; px.g = (uint8_t)c; px.b = 0;
            memset(&cfg, 0, sizeof cfg);
            cfg.bpp = 3; cfg.ord[1] = 1; cfg.ord[2] = 2; cfg.gamma = 1; cfg.smooth = FX_SM_DITHER; cfg.phase = fx_dither_phase(ph);
            cfg.dmin = 4;
            fx_pack(&px, w, 1, 255, &cfg);
            if (ph == 0u) first4 = w[1];
            var4 += w[1] != first4;
            cfg.dmin = 0;
            fx_pack(&px, w, 1, 255, &cfg);
            if (ph == 0u) first0 = w[1];
            var0 += w[1] != first0;
        }
        if (v < 4u * 256u) {
            CHECK(var4 == 0u, "level %u (value %u, under 4 x 256): %u of 256 frames differ with dmin 4, it must be rounded", c, v, var4);
            if ((v & 255u) != 0u) {
                CHECK(var0 > 0u, "level %u: with dmin 0 it dithers", c);
                rounded++;
            }
        } else if ((v & 255u) != 0u) {
            CHECK(var4 > 0u && var4 == var0, "level %u (value %u, 4 x 256 and up): dmin 4 dithers as dmin 0 does (%u / %u frames)", c, v, var4, var0);
            dithered++;
            if (boundary == 0u) boundary = c;
        }
    }
    CHECK(rounded > 3u && dithered > 100u, "the sweep: %u rounded, %u dithered inputs", rounded, dithered);
    CHECK(fx_gamma16_of(boundary) >= 4u * 256u && boundary > 0u && fx_gamma16_of(boundary - 1u) < 4u * 256u + 256u, "the boundary input %u (value %u)", boundary, fx_gamma16_of(boundary));
    printf("test_fx_dmin: first dithering input %u (16-bit value %u); %u dim inputs rounded, %u dithered\n", boundary, fx_gamma16_of(boundary), rounded, dithered);
    printf("test_fx_dmin: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail != 0;
}
