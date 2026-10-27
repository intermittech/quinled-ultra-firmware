/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Renders local pixel effects to PNG images for review: x = the strip's pixels, y = time going down.
 *
 *   fx_render sheet <out.png> [pixels] [seconds] [fps]          every effect at its defaults, in a grid (id order)
 *   fx_render one <out.png> <id> [pixels] [seconds] [fps] [speed intensity palette]
 *   fx_render pal <out.png>                                      the built-in palettes as bars
 *
 * The colours are the effects' RGB (before the packer's gamma), the way a screen shows them close to how a strip looks.
 * Host tool (no hardware): cl /O2 test/fx_render.c src/core/fx_effects.c fx_math.c fx_tables.c  (test/build_fx.cmd). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/fx.h"

typedef struct { uint8_t id, speed, intensity, palette; uint8_t c1[3], c2[3], c3[3]; uint8_t custom1, custom2; } defaults_t;
static const defaults_t DEF[] = {
#include "fx_defaults.inc"
};

static uint32_t crc_tab[256];
static void crc_init(void)
{
    uint32_t n, k, c;
    for (n = 0; n < 256; n++) {
        c = n;
        for (k = 0; k < 8; k++) {
            c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        }
        crc_tab[n] = c;
    }
}
static uint32_t crc_upd(uint32_t crc, const uint8_t *p, size_t n)
{
    while (n--) {
        crc = crc_tab[(crc ^ *p++) & 255u] ^ (crc >> 8);
    }
    return crc;
}
static void put32(FILE *f, uint32_t v)
{
    fputc((int)(v >> 24), f);
    fputc((int)(v >> 16 & 255), f);
    fputc((int)(v >> 8 & 255), f);
    fputc((int)(v & 255), f);
}
static void chunk(FILE *f, const char *type, const uint8_t *d, uint32_t n)
{
    uint32_t crc = crc_upd(0xFFFFFFFFu, (const uint8_t *)type, 4);
    put32(f, n);
    fwrite(type, 1, 4, f);
    if (n) {
        fwrite(d, 1, n, f);
        crc = crc_upd(crc, d, n);
    }
    put32(f, ~crc);
}

/* an RGB PNG with stored (uncompressed) deflate blocks */
static int write_png(const char *path, const uint8_t *rgb, uint32_t w, uint32_t h)
{
    FILE *f = fopen(path, "wb");
    uint8_t ihdr[13], *raw, *z;
    uint32_t y, rawn = (w * 3u + 1u) * h, zn, pos = 0, a = 1, b = 0, i;
    if (!f) {
        return -1;
    }
    crc_init();
    fwrite("\x89PNG\r\n\x1a\n", 1, 8, f);
    ihdr[0] = (uint8_t)(w >> 24); ihdr[1] = (uint8_t)(w >> 16); ihdr[2] = (uint8_t)(w >> 8); ihdr[3] = (uint8_t)w;
    ihdr[4] = (uint8_t)(h >> 24); ihdr[5] = (uint8_t)(h >> 16); ihdr[6] = (uint8_t)(h >> 8); ihdr[7] = (uint8_t)h;
    ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = ihdr[11] = ihdr[12] = 0;
    chunk(f, "IHDR", ihdr, 13);
    raw = malloc(rawn);
    for (y = 0; y < h; y++) {
        raw[y * (w * 3u + 1u)] = 0;
        memcpy(raw + y * (w * 3u + 1u) + 1u, rgb + (size_t)y * w * 3u, (size_t)w * 3u);
    }
    zn = 2 + rawn + 5 * (rawn / 65535u + 1u) + 4;
    z = malloc(zn);
    z[pos++] = 0x78;
    z[pos++] = 0x01;
    for (i = 0; i < rawn; i++) {
        a = (a + raw[i]) % 65521u;
        b = (b + a) % 65521u;
    }
    for (i = 0; i < rawn;) {
        uint32_t n = rawn - i > 65535u ? 65535u : rawn - i;
        z[pos++] = (uint8_t)(i + n >= rawn);
        z[pos++] = (uint8_t)n; z[pos++] = (uint8_t)(n >> 8);
        z[pos++] = (uint8_t)~n; z[pos++] = (uint8_t)(~n >> 8);
        memcpy(z + pos, raw + i, n);
        pos += n;
        i += n;
    }
    z[pos++] = (uint8_t)(b >> 8); z[pos++] = (uint8_t)b; z[pos++] = (uint8_t)(a >> 8); z[pos++] = (uint8_t)a;
    chunk(f, "IDAT", z, pos);
    chunk(f, "IEND", 0, 0);
    fclose(f);
    free(raw);
    free(z);
    return 0;
}

static void params_from(const defaults_t *d, fx_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->id = d->id; p->speed = d->speed; p->intensity = d->intensity; p->palette = d->palette;
    p->c[0].r = d->c1[0]; p->c[0].g = d->c1[1]; p->c[0].b = d->c1[2];
    p->c[1].r = d->c2[0]; p->c[1].g = d->c2[1]; p->c[1].b = d->c2[2];
    p->c[2].r = d->c3[0]; p->c[2].g = d->c3[1]; p->c[2].b = d->c3[2];
    p->bri = 255; p->custom1 = d->custom1; p->custom2 = d->custom2; p->seed = 0x1234u;
}

/* one effect over time into a w x h area of img (row stride W pixels), each LED sx wide, each frame sy high */
static void render_effect(uint8_t *img, uint32_t W, uint32_t x0, uint32_t y0, const fx_params_t *p, uint32_t n,
                          uint32_t frames, uint32_t fps, uint32_t sx, uint32_t sy)
{
    static rgb_t px[4096];
    static uint8_t state[4096];
    fx_ctx_t x;
    uint32_t f, i, a, b;
    fx_begin(&x, p, (uint16_t)n, state);
    for (f = 0; f < frames; f++) {
        fx_frame(&x, f * 1000u / fps);
        for (a = 0; a < n; a += FX_CHUNK) {              /* in chunks, as the receiver renders */
            b = a + FX_CHUNK < n ? a + FX_CHUNK : n;
            fx_render(&x, px + a, a, b);
        }
        for (i = 0; i < n; i++) {
            uint32_t yy, xx;
            for (yy = 0; yy < sy; yy++) {
                for (xx = 0; xx < sx; xx++) {
                    uint8_t *q = img + (((size_t)(y0 + f * sy + yy)) * W + x0 + i * sx + xx) * 3u;
                    q[0] = px[i].r; q[1] = px[i].g; q[2] = px[i].b;
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "";
    if (argc >= 3 && strcmp(mode, "sheet") == 0) {
        const uint32_t n = argc > 3 ? (uint32_t)atoi(argv[3]) : 100u, sec = argc > 4 ? (uint32_t)atoi(argv[4]) : 6u;
        const uint32_t fps = argc > 5 ? (uint32_t)atoi(argv[5]) : 10u, cols = 4u, sx = 2u, sy = 2u, gap = 8u;
        const uint32_t cnt = sizeof(DEF) / sizeof(DEF[0]), rows = (cnt + cols - 1u) / cols;
        const uint32_t cw = n * sx, ch = sec * fps * sy, W = cols * (cw + gap) + gap, H = rows * (ch + gap) + gap;
        uint8_t *img = malloc((size_t)W * H * 3u);
        uint32_t k;
        memset(img, 40, (size_t)W * H * 3u);
        for (k = 0; k < cnt; k++) {
            fx_params_t p;
            params_from(&DEF[k], &p);
            render_effect(img, W, gap + (k % cols) * (cw + gap), gap + (k / cols) * (ch + gap), &p, n, sec * fps, fps, sx, sy);
        }
        write_png(argv[2], img, W, H);
        printf("%s: %ux%u, %u effects (id order, %u across)\n", argv[2], W, H, cnt, cols);
        free(img);
        return 0;
    }
    if (argc >= 4 && strcmp(mode, "one") == 0) {
        const uint32_t id = (uint32_t)atoi(argv[3]), n = argc > 4 ? (uint32_t)atoi(argv[4]) : 150u;
        const uint32_t sec = argc > 5 ? (uint32_t)atoi(argv[5]) : 10u, fps = argc > 6 ? (uint32_t)atoi(argv[6]) : 20u;
        const uint32_t sx = 4u, sy = 2u, W = n * sx, H = sec * fps * sy;
        uint8_t *img = malloc((size_t)W * H * 3u);
        fx_params_t p;
        uint32_t k;
        for (k = 0; k < sizeof(DEF) / sizeof(DEF[0]); k++) {
            if (DEF[k].id == id) {
                params_from(&DEF[k], &p);
            }
        }
        if (argc > 7) p.speed = (uint8_t)atoi(argv[7]);
        if (argc > 8) p.intensity = (uint8_t)atoi(argv[8]);
        if (argc > 9) p.palette = (uint8_t)atoi(argv[9]);
        render_effect(img, W, 0, 0, &p, n, sec * fps, fps, sx, sy);
        write_png(argv[2], img, W, H);
        printf("%s: %ux%u\n", argv[2], W, H);
        free(img);
        return 0;
    }
    if (argc >= 3 && strcmp(mode, "pal") == 0) {
        extern rgb_t fx_pal_entry(uint32_t id, uint32_t k);
        const uint32_t W = 512, bar = 20, H = PAL_COUNT * (bar + 4);
        uint8_t *img = calloc((size_t)W * H * 3u, 1);
        uint32_t k, x, y;
        for (k = 0; k < PAL_COUNT; k++) {
            for (x = 0; x < W; x++) {
                const uint32_t idx = x * 256u / W, pos = idx * 3855u >> 8, e = pos >> 8, f = pos & 255u;
                const rgb_t a = fx_pal_entry(k, e), c = fx_pal_entry(k, e + 1u > 15u ? 15u : e + 1u);
                for (y = 0; y < bar; y++) {
                    uint8_t *q = img + (((size_t)(k * (bar + 4) + y)) * W + x) * 3u;
                    q[0] = (uint8_t)(a.r + ((int)c.r - a.r) * (int)f / 256);
                    q[1] = (uint8_t)(a.g + ((int)c.g - a.g) * (int)f / 256);
                    q[2] = (uint8_t)(a.b + ((int)c.b - a.b) * (int)f / 256);
                }
            }
        }
        write_png(argv[2], img, W, H);
        free(img);
        return 0;
    }
    fprintf(stderr, "usage: fx_render sheet|one|pal <out.png> ...\n");
    return 2;
}
