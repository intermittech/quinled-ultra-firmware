/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Host tests of the V407 effects layer (src/core/fx_v407; test/build_fxv.cmd): the registry and its maps, the slot
 * contract of every built effect, the 16-bit packer against the legacy one, palettes, gamma, the white mapping,
 * rendering's chunk-order independence and determinism. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fx_v407/fxv_gen.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; if (fails < 40) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static uint32_t rs = 0x12345678u;
static uint32_t rnd(void) { return fxv_rng_next(&rs); }

uint32_t fx_gamma16_of(uint32_t v);
void fx_pal_expand(rgb_t *pal, uint32_t id);

static uint8_t arena[FXV_SLOT_MAX];

static void params_default(fx_params_t *p, uint32_t id)
{
    memset(p, 0, sizeof *p);
    p->id = (uint8_t)id;
    p->speed = 128;
    p->intensity = 128;
    p->palette = PAL_DEFAULT;
    p->c[0].r = 255; p->c[0].g = 200; p->c[0].b = 100;
    p->c[1].r = 0; p->c[1].g = 80; p->c[1].b = 255;
    p->c[2].r = 40; p->c[2].g = 255; p->c[2].b = 0;
    p->bri = 255;
    p->custom1 = 110;
    p->custom2 = 40;
    p->seed = 0xC0FFEEu;
}

static void test_registry(void)
{
    uint32_t i, built = 0;
    CHECK(!fxv_built(0) && fxv_desc(0) == 0, "id 0 is the stream");
    CHECK(!fxv_built(FXV_FX_COUNT) && !fxv_built(255), "ids past the catalogue");
    for (i = 1; i < FX_COUNT; i++) CHECK(fxv_built(i) && (fxv_meta[i].flags & FXV_D_LEGACY), "legacy %u", i);
    for (i = 0; i < 192u; i++) {
        const uint32_t bit = (fxv_map_effects[i >> 3] >> (i & 7u)) & 1u;
        CHECK(bit == (uint32_t)(i < FXV_FX_COUNT && fxv_built(i)), "effect map bit %u", i);
        CHECK(((fxv_map_palettes[i >> 3] >> (i & 7u)) & 1u) == (uint32_t)(i < FXV_PAL_COUNT), "palette map bit %u", i);
        if (i >= FX_COUNT && i < FXV_FX_COUNT && fxv_built(i)) built++;
        if ((fxv_map_flashy[i >> 3] >> (i & 7u)) & 1u) CHECK(fxv_built(i) && (fxv_meta[i].flags & FXV_D_FLASHY), "flashy %u", i);
    }
    CHECK(built == FXV_BUILT_NEW, "built count %u", built);
    CHECK((fxv_map_effects[0] & 1u) == 0u, "bit 0 clear");
    for (i = 0; i < FXV_FX_COUNT; i++) CHECK(fxv_default_palette(i) < FXV_PAL_COUNT || fxv_default_palette(i) == PAL_DEFAULT, "default palette of %u", i);
}

/* every built effect's arena fits S(n) at every n, and begin + frames + render run on it */
static void test_slots(void)
{
    uint32_t id, n;
    for (id = 1; id < FXV_FX_COUNT; id++) {
        const fxv_desc_t *d = fxv_desc(id);
        if (!d) continue;
        for (n = 0; n <= FXV_N_MAX; n++) {
            const uint32_t b = d->bytes(n, 0);
            CHECK(b <= FXV_SLOT_BYTES(n), "effect %u: %u bytes at n %u > S(n) %u", id, b, n, FXV_SLOT_BYTES(n));
            if (b > FXV_SLOT_BYTES(n)) break;
        }
    }
}

/* q = 0 pixels whose channels are multiples of 256 pack to fx_pack's bytes exactly, every mode */
static void test_pack_legacy(void)
{
    static const uint8_t ords[3][5] = { { 1, 0, 2, 0, 0 }, { 1, 0, 2, 3, 0 }, { 0, 1, 2, 3, 4 } };
    rgb_t in[64];
    fxv_px_t px[64];
    uint8_t w1[64 * 5], w2[64 * 5];
    fxv_white_t wc;
    uint32_t it, k;
    fxv_white_init(&wc, FXV_K_W_DEFAULT, FXV_K_WW_DEFAULT, FXV_K_CW_DEFAULT);
    for (it = 0; it < 20000u; it++) {
        fx_pack_cfg_t c;
        const uint32_t n = 1u + rnd() % 64u, bpp = 3u + rnd() % 3u, bri = rnd() % 4u == 0u ? 255u : rnd() & 255u;
        uint32_t s1, s2;
        memset(&c, 0, sizeof c);
        c.gamma = (uint8_t)(rnd() % 4u != 0u);
        c.bpp = (uint8_t)bpp;
        memcpy(c.ord, ords[bpp - 3u], 5);
        c.white_mode = (uint8_t)(rnd() % 4u);
        c.white_split = (uint8_t)rnd();
        c.smooth = (uint8_t)(rnd() % 3u);
        c.phase = (uint8_t)rnd();
        c.px0 = (uint16_t)(rnd() % 800u);
        c.dmin = (uint8_t)(rnd() % 8u);
        for (k = 0; k < n; k++) {
            in[k].r = (uint8_t)rnd(); in[k].g = (uint8_t)rnd(); in[k].b = (uint8_t)rnd();
            if (rnd() % 4u == 0u) in[k].g = in[k].r;
            px[k] = fxv_rgb16(in[k]);
        }
        s1 = fx_pack(in, w1, n, (uint8_t)bri, &c);
        s2 = fxv_pack16(px, w2, n, (uint8_t)bri, &c, &wc);
        CHECK(s1 == s2 && memcmp(w1, w2, n * bpp) == 0, "pack16 != fx_pack (it %u, smooth %u, bpp %u, mode %u)", it,
              c.smooth, bpp, c.white_mode);
    }
}

static void test_gamma_pal(void)
{
    uint32_t v, prev = 0, id, k;
    rgb_t pal[16];
    for (v = 0; v <= FXV_FULL; v++) {
        const uint32_t gv = fxv_gamma16f(v);
        CHECK(gv >= prev, "gamma16f monotonic at %u", v);
        if ((v & 255u) == 0u) CHECK(gv == fx_gamma16_of(v >> 8), "gamma16f exact at %u", v);
        prev = gv;
    }
    CHECK(fxv_gamma16f(0) == 0u && fxv_gamma16f(FXV_FULL) == FXV_FULL && fxv_gamma16f(65535u) == FXV_FULL, "gamma16f ends");
    for (id = 0; id < FXV_PAL_COUNT; id++) {
        fx_pal_expand(pal, id);
        if (id >= FXV_PAL_LEGACY) {
            for (k = 0; k < 16u; k++) {
                CHECK(pal[k].r == fxv_pal_exact[id - FXV_PAL_LEGACY][k][0] && pal[k].g == fxv_pal_exact[id - FXV_PAL_LEGACY][k][1] &&
                      pal[k].b == fxv_pal_exact[id - FXV_PAL_LEGACY][k][2], "palette %u stop %u exact", id, k);
            }
        }
        for (k = 0; k < 16u; k++) {
            const fxv_px_t a = fxv_pal16(pal, k << 12, 1);
            CHECK(a.r == pal[k].r << 8 && a.g == pal[k].g << 8 && a.b == pal[k].b << 8, "pal16 wrap entry %u/%u", id, k);
        }
        {
            const fxv_px_t a = fxv_pal16(pal, 0, 0), b = fxv_pal16(pal, 65535u, 0);
            CHECK(a.r == pal[0].r << 8 && a.b == pal[0].b << 8, "pal16 clamp start %u", id);
            CHECK(b.r == pal[15].r << 8 && b.g == pal[15].g << 8 && b.b == pal[15].b << 8, "pal16 clamp end %u", id);
        }
    }
    for (k = 0; k < 256u; k++) {
        const fxv_px_t c = fxv_kelvin16(k);
        CHECK(c.q == 255u && c.k == k && c.r <= FXV_FULL && c.g <= FXV_FULL && c.b <= FXV_FULL, "kelvin %u", k);
    }
}

/* natural white on white strips: a white of the emitters' own temperature leaves (almost) no RGB */
static void test_white(void)
{
    fxv_white_t wc;
    fx_pack_cfg_t c;
    fxv_px_t px[2];
    uint8_t w[10];
    uint32_t sum;
    fxv_white_init(&wc, FXV_K_W_DEFAULT, FXV_K_WW_DEFAULT, FXV_K_CW_DEFAULT);
    memset(&c, 0, sizeof c);
    c.gamma = 1; c.bpp = 5; c.ord[0] = 0; c.ord[1] = 1; c.ord[2] = 2; c.ord[3] = 3; c.ord[4] = 4;
    c.white_mode = FX_W_ACCURATE; c.smooth = FX_SM_ROUND;
    px[0] = fxv_kelvin16(FXV_K_CW_DEFAULT);
    px[1] = fxv_kelvin16(FXV_K_WW_DEFAULT);
    sum = fxv_pack16(px, w, 2, 255, &c, &wc);
    CHECK(w[0] <= 2 && w[1] <= 2 && w[2] <= 2 && w[4] >= 250 && w[3] <= 2, "RGBCCT cold white -> CW: %u %u %u %u %u", w[0], w[1], w[2], w[3], w[4]);
    CHECK(w[5] <= 2 && w[6] <= 2 && w[7] <= 2 && w[8] >= 250 && w[9] <= 2, "RGBCCT warm white -> WW: %u %u %u %u %u", w[5], w[6], w[7], w[8], w[9]);
    CHECK(sum > 0u, "sum");
    c.bpp = 4;
    px[0] = fxv_kelvin16(FXV_K_W_DEFAULT);
    (void)fxv_pack16(px, w, 1, 255, &c, &wc);
    CHECK(w[0] <= 2 && w[1] <= 2 && w[2] <= 2 && w[3] >= 250, "RGBW white of W's temperature -> W: %u %u %u %u", w[0], w[1], w[2], w[3]);
    c.white_mode = FX_W_OFF;
    (void)fxv_pack16(px, w, 1, 255, &c, &wc);
    CHECK(w[3] == 0u && w[0] == 255u, "white off wins");
}

/* any chunk order and size: the same pixels; two runs: the same bytes */
static void test_chunks(void)
{
    static fxv_px_t a[FXV_N_MAX], b[FXV_N_MAX];
    static fxv_ctx_t x;
    static fxv_particle_t slab[FXV_PARTICLES];
    uint32_t id, it;
    for (id = 1; id < FXV_FX_COUNT; id++) {
        fx_params_t p;
        uint32_t f, n;
        if (!fxv_built(id)) continue;
        for (it = 0; it < 6u; it++) {
            n = it == 0u ? 1u : it == 1u ? 33u : 1u + rnd() % FXV_N_MAX;
            params_default(&p, id);
            p.speed = (uint8_t)rnd();
            p.intensity = (uint8_t)rnd();
            p.custom1 = (uint8_t)rnd();
            p.custom2 = (uint8_t)rnd();
            fxv_begin(&x, &p, n, 3, arena, 0);
            if (fxv_meta[id].particles != 0u) {          /* a lone owner's particle range (as the goldens) */
                x.p = slab;
                x.pn = (uint16_t)(fxv_meta[id].particles < FXV_PARTICLES - FXV_PART_RESERVE ? fxv_meta[id].particles
                                                                                           : FXV_PARTICLES - FXV_PART_RESERVE);
            }
            for (f = 0; f < 40u; f++) fxv_frame(&x, f * 24u + (rnd() & 7u));
            {
                uint32_t k;
                for (k = 0; k < n; k += FXV_CHUNK) fxv_render(&x, a + k, k, k + FXV_CHUNK < n ? k + FXV_CHUNK : n);
                for (k = 0; k < 200u; k++) {
                    const uint32_t from = rnd() % n, cnt = 1u + rnd() % FXV_CHUNK, to = from + cnt < n ? from + cnt : n;
                    fxv_render(&x, b + from, from, to);
                    CHECK(memcmp(a + from, b + from, (to - from) * sizeof(fxv_px_t)) == 0, "effect %u: chunk [%u, %u) differs", id, from, to);
                }
            }
        }
    }
}

/* M2 primitives: stability, conservation, ordering (the exact values are the C <-> JS goldens') */
static void test_primitives(void)
{
    static int16_t a[800], b[800], c[800];
    static fxv_particle_t pp[FXV_PARTICLES];
    static uint16_t start[802], order[FXV_PARTICLES];
    uint32_t it, k, n, st;
    for (it = 0; it < 200u; it++) {                       /* diffusion: within the old range, mass kept within rounding */
        int32_t lo = 32767, hi = -32768, s0 = 0, s1 = 0;
        const uint32_t coef = rnd() % 1025u;
        n = 2u + rnd() % 799u;
        for (k = 0; k < n; k++) { a[k] = (int16_t)(rnd() % 20000u); if (a[k] < lo) lo = a[k]; if (a[k] > hi) hi = a[k]; s0 += a[k]; }
        fxv_diffuse(a, b, n, coef, 0, n, FXV_EDGE_WRAP);
        for (k = 0; k < n; k++) { CHECK(b[k] >= lo && b[k] <= hi, "diffuse maximum principle"); s1 += b[k]; }
        CHECK(s1 <= s0 && s0 - s1 <= (int32_t)n, "diffuse mass %d -> %d (n %u)", s0, s1, n);
    }
    for (it = 0; it < 20u; it++) {                        /* waves at Courant 1 with a little damping stay bounded */
        int16_t *p = a, *q = b, *r = c, *t;
        n = 16u + rnd() % 784u;
        for (k = 0; k < n; k++) a[k] = b[k] = (int16_t)((int32_t)(rnd() % 4000u) - 2000);
        for (st = 0; st < 5000u; st++) {
            fxv_wave_step(p, q, r, n, 4096, 16, 0, n, it % 3u);   /* (clamped to 4000) */
            t = p; p = q; q = r; r = t;
        }
        for (k = 0; k < n; k++) CHECK(q[k] > -30000 && q[k] < 30000, "wave bounded (n %u edge %u): %d", n, it % 3u, q[k]);
    }
    {                                                     /* reaction-diffusion keeps a pattern (neither all u nor all v) */
        static int16_t u[2][800], v[2][800];
        uint32_t cur = 0, nz = 0;
        n = 400;
        for (k = 0; k < n; k++) { v[0][k] = (int16_t)((rnd() & 15u) == 0u ? 2048 : 0); u[0][k] = (int16_t)(4096 - v[0][k]); }
        for (st = 0; st < 3000u; st++) {
            fxv_rd_step(u[cur], v[cur], u[cur ^ 1u], v[cur ^ 1u], n, 1024, 512, 225, 254, 0, n, FXV_EDGE_WRAP);
            cur ^= 1u;
        }
        for (k = 0; k < n; k++) nz += v[cur][k] > 200;
        CHECK(nz > 10u && nz < n - 10u, "reaction-diffusion pattern: %u of %u cells", nz, n);
    }
    for (it = 0; it < 200u; it++) {                       /* bins: every live record once, in bin order */
        const uint32_t cnt = rnd() % 513u, npx = 1u + rnd() % 800u, sh = rnd() % 7u, nb = fxv_bins_count(npx, sh);
        uint32_t live = 0, prev = 0;
        for (k = 0; k < cnt; k++) {
            pp[k].x = (int32_t)(rnd() % ((npx + 40u) << 16)) - (20 << 16);
            pp[k].life = (uint16_t)(rnd() % 3u);
            live += pp[k].life != 0u;
        }
        fxv_bins_build(start, order, pp, cnt, npx, sh);
        CHECK(start[nb] == live, "bins hold the live records: %u of %u", start[nb], live);
        for (k = 0; k < start[nb]; k++) {
            const int32_t px = pp[order[k]].x >> 16;
            uint32_t bb = px < 0 ? 0u : (uint32_t)px >> sh;
            if (bb >= nb) bb = nb - 1u;
            CHECK(pp[order[k]].life != 0u && bb >= prev && k >= start[bb] && k < start[bb + 1u], "bins order");
            prev = bb;
        }
    }
    {                                                     /* envelopes: 0 before / after, full in the hold, smooth edges */
        uint32_t t, last = 0;
        CHECK(fxv_env(0, 120, 50, 300) == 0u && fxv_env(120, 120, 50, 300) == 65535u && fxv_env(470, 120, 50, 300) == 0u, "env ends");
        for (t = 0; t < 120u; t++) { const uint32_t e = fxv_env(t, 120, 50, 300); CHECK(e >= last, "env rises"); last = e; }
        last = 65535u;
        for (t = 170; t < 470u; t++) { const uint32_t e = fxv_env(t, 120, 50, 300); CHECK(e <= last, "env falls"); last = e; }
    }
    {                                                     /* the queue is a FIFO with a hard capacity */
        uint16_t qb[8];
        fxv_queue_t q;
        fxv_queue_init(&q, qb, 5);
        for (k = 0; k < 7u; k++) CHECK(fxv_queue_push(&q, 100u + k) == (uint32_t)(k < 5u), "queue push %u", k);
        for (k = 0; k < 5u; k++) CHECK(fxv_queue_pop(&q) == 100u + k, "queue order");
        CHECK(fxv_queue_pop(&q) == 0xFFFFu, "queue empty");
    }
}

/* canaries: an effect never writes its arena past bytes(n) nor the slab outside its range (records [pn, ...) of a
 * lone owner's design count: the range is exact), over begin, 300 frames with speed / colour changes and renders */
static void test_canaries(void)
{
    static fxv_particle_t slab[FXV_PARTICLES + 16];
    static fxv_px_t o[FXV_CHUNK];
    static fxv_ctx_t x;
    uint32_t id, it, k;
    for (id = 1; id < FXV_FX_COUNT; id++) {
        if (!fxv_built(id)) continue;
        for (it = 0; it < 3u; it++) {
            fx_params_t p;
            const uint32_t n = it == 0u ? 1u : it == 1u ? 150u : FXV_N_MAX, cap = fxv_meta[id].particles;
            uint32_t f, bad = 0;
            params_default(&p, id);
            memset(arena, 0xA5, sizeof arena);
            memset(slab, 0x5A, sizeof slab);
            fxv_begin(&x, &p, n, 3u + it, arena, 0);
            memset(slab + 8, 0, cap * sizeof(fxv_particle_t));
            if (cap) { x.p = slab + 8; x.pn = (uint16_t)cap; }
            for (f = 0; f < 300u; f++) {
                fxv_frame(&x, f * 24u);
                if (f == 100u) fxv_set_params(&x, 255, 7, 200, 9, x.seed);
                if (f % 50u == 49u) for (k = 0; k < n; k += FXV_CHUNK) fxv_render(&x, o, k, k + FXV_CHUNK < n ? k + FXV_CHUNK : n);
            }
            for (k = x.need; k < FXV_SLOT_MAX; k++) bad += arena[k] != 0xA5u;
            for (k = 0; k < 8u * sizeof(fxv_particle_t); k++) bad += ((uint8_t *)slab)[k] != 0x5Au;
            for (k = (8u + cap) * sizeof(fxv_particle_t); k < sizeof slab; k++) bad += ((uint8_t *)slab)[k] != 0x5Au;
            CHECK(bad == 0u, "effect %u at %u px: %u bytes written outside its arena / particle range", id, n, bad);
            fxv_end(&x);
        }
    }
}

int main(void)
{
    test_registry();
    test_slots();
    test_gamma_pal();
    test_pack_legacy();
    test_white();
    test_chunks();
    test_primitives();
    test_canaries();
    printf("%s: test_fxv (%u built new effects, %u palettes)\n", fails ? "FAIL" : "PASS", FXV_BUILT_NEW, FXV_PAL_COUNT);
    return fails != 0;
}
