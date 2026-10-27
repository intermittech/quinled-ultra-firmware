/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The flash-rate check of the V407 effects ("flash comfort"): every built effect at its catalogue
 * defaults (or the ids given), RGB 1-6 / 8 / 60 / 800, RGBW 1 / 3 / 600, RGBCCT 500 pixels (on a few pixels anything local
 * covers the whole strip), 10 minutes of effect time at 42 fps, the undithered 16-bit linear luminance of every pixel
 * (gamma 2.8 applied, no brightness limit).
 *
 * A pixel's transition: its luminance moved by at least 10 % of full scale over 3 frames (~71 ms), the darker side below
 * 80 % (WCAG's general-flash thresholds). A frame is an excursion when transitions in one direction cover >= 80 % of the
 * strip; a flash is a pair of opposing excursions. The gate: at most 3 flashes in any rolling second, for every effect,
 * the flashy ones included (their default rate stays <= 3 Hz too). An engineering check, not a medical certification.
 *
 *   fxv_flash_check [--legacy] [--minutes m] [--detail] [id ...]   exit 1 when an effect fails the gate; --detail: each
 *                                                                  length's worst second and count */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fx_v407/fxv_gen.h"

typedef struct { uint8_t id, speed, intensity, palette; uint8_t c1[3], c2[3], c3[3]; uint8_t custom1, custom2; } def_t;
static const def_t defs[] = {
#include "fx_v407/fxv_defaults.inc"
};

static uint8_t arena[FXV_SLOT_MAX];
static fxv_particle_t slab[FXV_PARTICLES];
static uint32_t hist[4][FXV_N_MAX];
#define FPS 42u
#define T_FS 65280u

static uint32_t lum(fxv_px_t c)
{
    return (2126u * fxv_gamma16f(c.r) + 7152u * fxv_gamma16f(c.g) + 722u * fxv_gamma16f(c.b)) / 10000u;
}

/* returns the most flashes in a rolling second */
static uint32_t check(uint32_t id, uint32_t n, uint32_t fmt, uint32_t minutes, uint32_t *total)
{
    const def_t *d = 0;
    fx_params_t p;
    static fxv_ctx_t x;
    static uint8_t win[FPS];             /* flashes counted at each of the last second's frames */
    uint32_t k, f, frames = minutes * 60u * FPS, last = 0, run_up = 0, run_dn = 0, in_win = 0, worst = 0;
    for (k = 0; k < sizeof defs / sizeof defs[0]; k++) if (defs[k].id == id) d = &defs[k];
    if (!d) return 0;
    memset(&p, 0, sizeof p);
    p.id = (uint8_t)id; p.speed = d->speed; p.intensity = d->intensity; p.palette = d->palette;
    p.c[0].r = d->c1[0]; p.c[0].g = d->c1[1]; p.c[0].b = d->c1[2];
    p.c[1].r = d->c2[0]; p.c[1].g = d->c2[1]; p.c[1].b = d->c2[2];
    p.c[2].r = d->c3[0]; p.c[2].g = d->c3[1]; p.c[2].b = d->c3[2];
    p.custom1 = d->custom1; p.custom2 = d->custom2; p.seed = 0x5EED0000u + id; p.bri = 255;
    fxv_begin(&x, &p, n, fmt, arena, 0);
    if (fxv_meta[id].particles) {                        /* a lone owner: its design count */
        memset(slab, 0, sizeof slab);
        x.p = slab;
        x.pn = fxv_meta[id].particles;
    }
    memset(win, 0, sizeof win);
    *total = 0;
    for (f = 0; f < frames; f++) {
        fxv_px_t px[FXV_CHUNK];
        uint32_t up = 0, dn = 0, i;
        fxv_frame(&x, (f * 1000u + FPS / 2u) / FPS);
        for (i = 0; i < n; i += FXV_CHUNK) {
            const uint32_t m = n - i < FXV_CHUNK ? n - i : FXV_CHUNK;
            uint32_t j;
            fxv_render(&x, px, i, i + m);
            for (j = 0; j < m; j++) {
                const uint32_t l = lum(px[j]), o = hist[(f + 1u) & 3u][i + j];   /* (3 frames ago) */
                if (f >= 3u) {
                    if (l >= o + T_FS / 10u && o < T_FS * 8u / 10u) up++;
                    else if (o >= l + T_FS / 10u && l < T_FS * 8u / 10u) dn++;
                }
                hist[f & 3u][i + j] = l;
            }
        }
        in_win -= win[f % FPS];
        win[f % FPS] = 0;
        if (n && up * 10u >= n * 8u) {                   /* an excursion up (counted at its first frame) */
            if (!run_up && last == 2u) { win[f % FPS] = 1; in_win++; (*total)++; }
            run_up = 1;
            last = 1;
        } else run_up = 0;
        if (n && dn * 10u >= n * 8u) {
            if (!run_dn && last == 1u) { win[f % FPS] = 1; in_win++; (*total)++; }
            run_dn = 1;
            last = 2;
        } else run_dn = 0;
        if (in_win > worst) worst = in_win;
    }
    fxv_end(&x);
    return worst;
}

int main(int argc, char **argv)
{
    static const uint32_t fmts[][2] = { { 3, 1 }, { 3, 2 }, { 3, 3 }, { 3, 4 }, { 3, 5 }, { 3, 6 }, { 3, 8 }, { 3, 60 }, { 3, 800 },
                                        { 4, 1 }, { 4, 3 }, { 4, 600 }, { 5, 500 } };
    const uint32_t nf = (uint32_t)(sizeof fmts / sizeof fmts[0]);
    uint32_t ids[256], nid = 0, legacy = 0, minutes = 10, k, fails = 0, detail = 0;
    int a;
    for (a = 1; a < argc; a++) {
        if (!strcmp(argv[a], "--legacy")) legacy = 1;
        else if (!strcmp(argv[a], "--detail")) detail = 1;
        else if (!strcmp(argv[a], "--minutes") && a + 1 < argc) minutes = (uint32_t)atoi(argv[++a]);
        else if (nid < 256u) ids[nid++] = (uint32_t)atoi(argv[a]);
    }
    if (!nid) {
        for (k = legacy ? 1u : FX_COUNT; k < FXV_FX_COUNT; k++) if (fxv_built(k)) ids[nid++] = k;
    }
    printf("flash check: %u effects, %u min at %u fps, defaults; the gate: <= 3 flashes in any second\n", nid, minutes, FPS);
    for (k = 0; k < nid; k++) {
        uint32_t f, worst = 0, sum = 0;
        if (!fxv_built(ids[k])) { printf("  %3u not built\n", ids[k]); continue; }
        for (f = 0; f < nf; f++) {
            uint32_t t;
            const uint32_t w = check(ids[k], fmts[f][1], fmts[f][0], minutes, &t);
            if (w > worst) worst = w;
            sum += t;
            if (detail) printf("      %3u %u bytes %3u px: most in a second %u, flashes %u\n", ids[k], fmts[f][0], fmts[f][1], w, t);
        }
        printf("  %3u %-6s flashy %u  most in a second %u  flashes in all runs %u  %s\n", ids[k], "", (fxv_meta[ids[k]].flags & FXV_D_FLASHY) != 0u,
               worst, sum, worst <= 3u ? "PASS" : "FAIL");
        fails += worst > 3u;
    }
    printf("%s: %u of %u effects over the gate\n", fails ? "FAIL" : "PASS", fails, nid);
    return fails != 0;
}
