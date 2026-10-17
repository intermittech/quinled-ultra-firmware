/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The packer (fx.h): rgb -> the wire bytes of the chip. Brightness, gamma 2.8, auto-white, colour order, the channel sum. */
#include "fx_int.h"

uint32_t fx_pack(const rgb_t *in, uint8_t *wire, uint32_t n, uint8_t bri, const fx_pack_cfg_t *cfg)
{
    const uint8_t *g = cfg->gamma, *ord = cfg->ord;
    const uint32_t bpp = cfg->bpp, mode = cfg->white_mode, split = cfg->white_split, full = bri == 255u;
    uint32_t i, sum = 0;
    for (i = 0; i < n; i++, wire += bpp) {
        uint8_t src[5];
        uint32_t r = in[i].r, gr = in[i].g, b = in[i].b;
        if (!full) {
            r = fx_scale8(r, bri);
            gr = fx_scale8(gr, bri);
            b = fx_scale8(b, bri);
        }
        if (g != 0) {
            r = g[r];
            gr = g[gr];
            b = g[b];
        }
        src[3] = 0;
        src[4] = 0;
        if (bpp >= 4u && (mode == FX_W_ACCURATE || mode == FX_W_BRIGHT)) {
            uint32_t w = r < gr ? r : gr;
            w = w < b ? w : b;
            if (mode == FX_W_ACCURATE) {                  /* accurate: the white's share comes out of RGB */
                r -= w;
                gr -= w;
                b -= w;
            }
            if (bpp == 5u) {                              /* WS2805: the white split warm / cool */
                src[3] = fx_scale8(w, 255u - split);
                src[4] = fx_scale8(w, split);
            } else {
                src[3] = (uint8_t)w;
            }
        }
        src[0] = (uint8_t)r;
        src[1] = (uint8_t)gr;
        src[2] = (uint8_t)b;
        wire[0] = src[ord[0]];
        wire[1] = src[ord[1]];
        wire[2] = src[ord[2]];
        sum += (uint32_t)wire[0] + wire[1] + wire[2];
        if (bpp >= 4u) {
            wire[3] = src[ord[3]];
            sum += wire[3];
            if (bpp == 5u) {
                wire[4] = src[ord[4]];
                sum += wire[4];
            }
        }
    }
    return sum;
}

/* 255 * (k / 16) ^ (1 / 2.8), k = 0 .. 16 */
static const uint8_t root28[17] = { 0, 95, 121, 140, 155, 168, 180, 190, 199, 208, 216, 223, 230, 237, 243, 249, 255 };

uint8_t fx_limit_bri(uint8_t bri, uint32_t sum, uint32_t max_sum)
{
    uint32_t r, k, f, root;
    if (sum <= max_sum) {
        return bri;
    }
    r = (max_sum << 8) / sum;                             /* < 256 */
    k = r >> 4;
    f = r & 15u;
    root = (uint32_t)root28[k] + (((uint32_t)root28[k + 1u] - root28[k]) * f >> 4);
    return (uint8_t)((bri * root) >> 8);
}
