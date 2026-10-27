/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Dumps frames of the effects 1 .. 31 over a fixed grid of parameters (the catalogue defaults, three intensities, every palette, two
 * times, 100 pixels) as raw RGB bytes, for test/fx_compare.py: the squeeze rounds compare a new engine's frames with an older one's
 * and report the worst channel difference per effect. Build: test\build_fxdump.cmd <outdir> (MSVC). Usage: fx_dump <out.bin> [max effect id]. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/fx.h"

typedef struct { uint8_t id, speed, intensity, palette; uint8_t c1[3], c2[3], c3[3]; uint8_t custom1, custom2; } defaults_t;
static const defaults_t DEF[] = {
#include "fx_defaults.inc"
};
#define NDEF ((int)(sizeof(DEF) / sizeof(DEF[0])))
#define NPX 100

static uint8_t state[4096];

int main(int argc, char **argv)
{
    FILE *f;
    int k, maxid = argc > 2 ? atoi(argv[2]) : 31;
    if (argc < 2 || (f = fopen(argv[1], "wb")) == 0) {
        return 2;
    }
    for (k = 0; k < NDEF && DEF[k].id <= maxid; k++) {
        static const uint8_t lv[3] = { 0, 128, 255 };
        static const uint32_t times[2] = { 700, 5000 };
        uint32_t v, t;
        for (v = 0; v < 3u + 32u; v++) {
            fx_params_t p;
            fx_ctx_t x;
            rgb_t out[NPX];
            memset(&p, 0, sizeof(p));
            p.id = DEF[k].id; p.speed = DEF[k].speed; p.intensity = DEF[k].intensity; p.palette = DEF[k].palette;
            p.c[0].r = DEF[k].c1[0]; p.c[0].g = DEF[k].c1[1]; p.c[0].b = DEF[k].c1[2];
            p.c[1].r = DEF[k].c2[0]; p.c[1].g = DEF[k].c2[1]; p.c[1].b = DEF[k].c2[2];
            p.c[2].r = DEF[k].c3[0]; p.c[2].g = DEF[k].c3[1]; p.c[2].b = DEF[k].c3[2];
            p.bri = 255; p.custom1 = DEF[k].custom1; p.custom2 = DEF[k].custom2; p.seed = 0x1234u;
            if (v >= 1u && v <= 3u) {
                p.intensity = lv[v - 1u];
            } else if (v >= 4u) {
                p.palette = (uint8_t)(v - 4u == 31u ? PAL_COLORS : v - 4u);
                p.c[0].r = 255; p.c[0].g = 40; p.c[0].b = 0; p.c[1].r = 0; p.c[1].g = 20; p.c[1].b = 90; p.c[2].r = 30; p.c[2].g = 255; p.c[2].b = 60;
            }
            fx_begin(&x, &p, NPX, state);
            for (t = 0; t < 2u; t++) {
                fx_frame(&x, times[t]);
                fx_render(&x, out, 0, NPX);
                fwrite(out, sizeof(rgb_t), NPX, f);
            }
        }
    }
    fclose(f);
    return 0;
}
