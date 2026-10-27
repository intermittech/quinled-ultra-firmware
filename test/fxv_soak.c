/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 effects layer's soak ("bounds / soak"): every built effect (37+, or the ids given) alone on
 * the segment engine, as the receiver runs it, for hours of host time at a fixed frame rate, with the host clock starting
 * half way before its 32-bit wrap (it wraps in the middle of the run). Every fps + 1 frames (about a second, off any
 * whole-second beat): the whole strip rendered and packed, then checked:
 *   - the arena slot's bytes past the effect's `need` and the slab's records past its range keep their canary bytes;
 *   - every working pixel inside full scale (r, g, b <= 65280);
 *   - no restart (the context's epoch stays 0: a continuous run never has a 2 s discontinuity);
 *   - the longest run of identical strips (informational: a stalled solver shows as hours).
 *
 *   fxv_soak [--hours h] [--fps f] [--n n] [--fmt 3|4|5] [--seed s] [--mode 0..3] [id ...]
 * seed 0 is the zero seed (the layer maps it); default 0x5EED0000 + id. mode: 0 defaults, 1 every control 255, 2 speed
 * 255 and the rest 0, 3 speed 1 and the rest 255. Exit 1 on a failure. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fx_v407/fxv_engine.h"

typedef struct { uint8_t id, speed, intensity, palette; uint8_t c1[3], c2[3], c3[3]; uint8_t custom1, custom2; } def_t;
static const def_t defs[] = {
#include "fx_v407/fxv_defaults.inc"
};

#define CANARY 0xA5u
static uint8_t arena[FXV_ARENA_BYTES];
static fxv_particle_t slab[FXV_PARTICLES];
static fxv_engine_t E;
static fxv_px_t px[FXV_N_MAX];
static uint8_t wire[FXV_N_MAX * 5u];
static fxv_white_t wc;

static int params(fx_params_t *p, uint32_t id, uint32_t mode, int seed_given, uint32_t seed)
{
    uint32_t k;
    memset(p, 0, sizeof *p);
    for (k = 0; k < sizeof defs / sizeof defs[0]; k++) {
        const def_t *d = &defs[k];
        if (d->id != id) continue;
        p->id = d->id; p->speed = d->speed; p->intensity = d->intensity; p->palette = d->palette;
        p->c[0].r = d->c1[0]; p->c[0].g = d->c1[1]; p->c[0].b = d->c1[2];
        p->c[1].r = d->c2[0]; p->c[1].g = d->c2[1]; p->c[1].b = d->c2[2];
        p->c[2].r = d->c3[0]; p->c[2].g = d->c3[1]; p->c[2].b = d->c3[2];
        p->custom1 = d->custom1; p->custom2 = d->custom2;
        if (mode == 1u) { p->speed = 255; p->intensity = 255; p->custom1 = 255; p->custom2 = 255; }
        else if (mode == 2u) { p->speed = 255; p->intensity = 0; p->custom1 = 0; p->custom2 = 0; }
        else if (mode == 3u) { p->speed = 1; p->intensity = 255; p->custom1 = 255; p->custom2 = 255; }
        p->bri = 255;
        p->seed = seed_given ? seed : 0x5EED0000u + id;
        return 1;
    }
    return 0;
}

static uint32_t crc(const uint8_t *b, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu, k, j;
    for (k = 0; k < n; k++) { c ^= b[k]; for (j = 0; j < 8u; j++) c = (c >> 1) ^ ((c & 1u) ? 0xEDB88320u : 0u); }
    return ~c;
}

/* returns the number of failures; prints one line */
static uint32_t soak(uint32_t id, uint32_t n, uint32_t fmt, uint32_t mode, int seed_given, uint32_t seed, double hours, uint32_t fps)
{
    static const uint8_t ords[3][5] = { { 1, 0, 2, 0, 0 }, { 1, 0, 2, 3, 0 }, { 0, 1, 2, 3, 4 } };
    fx_params_t p;
    fx_pack_cfg_t c;
    const uint64_t frames = (uint64_t)(hours * 3600.0 * fps);
    const uint32_t t0 = 0xFFFFFFFFu - (uint32_t)(hours * 1800000.0);   /* the host clock wraps half way */
    uint64_t f, same = 0, worst_same = 0, checks = 0;
    uint32_t fails = 0, last = 0, bad_px = 0, bad_arena = 0, bad_slab = 0, restarts = 0, k, slot, prange = 0;
    const fxv_ctx_t *x;
    if (!params(&p, id, mode, seed_given, seed)) return 0;
    memset(arena, CANARY, sizeof arena);
    memset(slab, CANARY, sizeof slab);
    fxv_engine_init(&E, n, fmt, arena, slab);
    memset(&c, 0, sizeof c);
    c.gamma = 1; c.bpp = (uint8_t)fmt; memcpy(c.ord, ords[fmt - 3u], 5); c.white_mode = FX_W_ACCURATE; c.white_split = 128;
    c.smooth = 2; c.phase = 77; c.dmin = 4;
    fxv_engine_set(&E, 0, &p, t0, t0);
    for (f = 0; f < frames; f++) {
        const uint32_t now = t0 + (uint32_t)((f * 1000u) / fps);
        fxv_engine_frame(&E, now);
        if (f == 0 || (f % (fps + 1u)) != 0u) continue;      /* (off the beat: whole-second periods alias) */
        x = &E.seg[0].cur;
        if (!E.seg[0].active || !x->d) { fails++; printf("  %3u inactive at frame %llu\n", id, (unsigned long long)f); break; }
        slot = x->slot;
        for (k = 0; k < n; k += FXV_CHUNK) fxv_engine_render(&E, px + k, k, k + FXV_CHUNK < n ? k + FXV_CHUNK : n);
        for (k = 0; k < n; k++) if (px[k].r > FXV_FULL || px[k].g > FXV_FULL || px[k].b > FXV_FULL) bad_px++;
        (void)fxv_pack16(px, wire, n, 255, &c, &wc);
        for (k = x->need; k < FXV_SLOT_MAX; k++) if (arena[slot * FXV_SLOT_MAX + k] != CANARY) { bad_arena++; break; }
        prange = (uint32_t)E.pfirst[slot] + E.pcount[slot];
        for (k = prange * (uint32_t)sizeof(fxv_particle_t); k < (uint32_t)sizeof slab; k++)
            if (((const uint8_t *)slab)[k] != CANARY) { bad_slab++; break; }
        if (x->epoch) restarts = x->epoch;
        {
            const uint32_t h = crc(wire, n * fmt);
            if (checks && h == last) { same++; if (same > worst_same) worst_same = same; } else same = 0;
            last = h;
        }
        checks++;
    }
    fails += (bad_px != 0) + (bad_arena != 0) + (bad_slab != 0) + (restarts != 0);
    printf("  %3u n %3u fmt %u mode %u seed %08x: %llu frames, %llu checks; particles %u; bad px %u, arena %u, slab %u, "
           "restarts %u; longest still %llu checks  %s\n", id, n, fmt, mode, p.seed, (unsigned long long)frames,
           (unsigned long long)checks, prange, bad_px, bad_arena, bad_slab, restarts, (unsigned long long)worst_same,
           fails ? "FAIL" : "PASS");
    fflush(stdout);
    return fails;
}

int main(int argc, char **argv)
{
    uint32_t ids[256], nid = 0, n = 33, fmt = 3, fps = 15, mode = 0, seed = 0, k, fails = 0;
    int seed_given = 0, a;
    double hours = 24.0;
    for (a = 1; a < argc; a++) {
        if (!strcmp(argv[a], "--hours") && a + 1 < argc) hours = atof(argv[++a]);
        else if (!strcmp(argv[a], "--fps") && a + 1 < argc) fps = (uint32_t)atoi(argv[++a]);
        else if (!strcmp(argv[a], "--n") && a + 1 < argc) n = (uint32_t)atoi(argv[++a]);
        else if (!strcmp(argv[a], "--fmt") && a + 1 < argc) fmt = (uint32_t)atoi(argv[++a]);
        else if (!strcmp(argv[a], "--mode") && a + 1 < argc) mode = (uint32_t)atoi(argv[++a]);
        else if (!strcmp(argv[a], "--seed") && a + 1 < argc) { seed = (uint32_t)strtoul(argv[++a], 0, 0); seed_given = 1; }
        else ids[nid++] = (uint32_t)atoi(argv[a]);
    }
    if (n > FXV_N_MAX || fmt < 3u || fmt > 5u || !fps) return 2;
    if (!nid) for (k = FX_COUNT; k < FXV_FX_COUNT; k++) if (fxv_built(k)) ids[nid++] = k;
    fxv_white_init(&wc, FXV_K_W_DEFAULT, FXV_K_WW_DEFAULT, FXV_K_CW_DEFAULT);
    printf("soak: %u effects, %.2f h at %u fps, n %u, fmt %u, mode %u, host clock wrapping half way\n", nid, hours, fps, n, fmt, mode);
    for (k = 0; k < nid; k++) fails += soak(ids[k], n, fmt, mode, seed_given, seed, hours, fps);
    printf("%s: %u failures\n", fails ? "FAIL" : "PASS", fails);
    return fails != 0;
}
