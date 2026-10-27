/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The packer (fx.h): rgb -> the wire bytes of the chip. Brightness, gamma 2.8, auto-white, colour order, the channel sum. */
#include "fx_int.h"
#include "ultra_protocol.h"                   /* ULTRA_PXS_AUTO_MIN_LEVEL: the packer's rounding of dim channels in the AUTO dither is compiled in only when it is not 0 */

/* The table stores gamma minus 256*v, keeping the virtual points in int16_t.
 * The generator verifies every input: within one 16-bit step, exact 8-bit
 * results and endpoints. Interpolating the linear ramp is exact. */
extern const int16_t fx_gamma_q[34];

static uint32_t gamma16(uint32_t v)
{
    const int16_t *p = fx_gamma_q + (v >> 3);
    const int32_t t = (int32_t)(v & 7u), a = p[0];
    const int32_t d1 = (int32_t)p[1] - a, d2 = (int32_t)p[2] - 2 * (int32_t)p[1] + a;
    const int32_t y = ((a * 128 + d1 * 16 * t + d2 * t * (t - 8) + 64) >> 7) + (int32_t)(v * 256u);
    return y > 0 ? (uint32_t)y : 0u;
}

FX_AI uint32_t gamma_ai(uint32_t v)
{
    return (gamma16(v) + 128u) >> 8;
}

uint32_t fx_gamma(uint32_t v)
{
    return gamma_ai(v);
}

uint32_t fx_gamma16_of(uint32_t v)
{
    return gamma16(v);
}

/* the packer, 8 and 16 bits in one loop (the 8-bit path shifts its bytes up by 8 and adds no threshold, so its wire byte is its 8-bit value). 16 bits: gamma, then the limiter's brightness (as gamma of it: (c * b) ^ 2.8 = c ^ 2.8 * b ^ 2.8), then auto-white, all at 16 bits; the wire
 * byte is (value + threshold) >> 8: 128 rounds, the dither's 0..255 (ordered by frame and pixel, 85 apart for the red, green and blue sources; the whites take
 * the red's and green's) makes the average over the pattern's 256 frames exact (the value never passes 65280, so the byte never passes 255) */
static uint32_t pack_all(const rgb_t *in, uint8_t *wire, uint32_t n, uint8_t bri, const fx_pack_cfg_t *cfg)
{
    const uint8_t *ord = cfg->ord;
    const uint32_t g = cfg->gamma, bpp = cfg->bpp, mode = cfg->white_mode, split = cfg->white_split, full = bri == 255u;
    const uint32_t m8 = cfg->smooth == FX_SM_8BIT, dm = cfg->smooth == FX_SM_DITHER ? 255u : 0u, rb = m8 ? 0u : 128u & ~dm;       /* threshold = (phase term & dm) | rb */
    const uint32_t gb = g ? gamma16(bri) : bri * 256u, mb = gb + (gb >> 8);               /* x 65536 / 65280 */
    const uint32_t white = bpp >= 4u && (mode == FX_W_ACCURATE || mode == FX_W_BRIGHT);
#if ULTRA_PXS_AUTO_MIN_LEVEL > 0u
    const uint32_t mv = (uint32_t)cfg->dmin << 8;
#endif
    uint32_t i, sum = 0, d0 = cfg->phase + (uint32_t)cfg->px0 * 53u;
    for (i = 0; i < n; i++, wire += bpp, d0 += 53u) {
        uint8_t t[5];
        uint32_t r, gr, b, w3 = 0, w4 = 0;
        const uint32_t t0 = (d0 & dm) | rb, t1 = ((d0 + 85u) & dm) | rb, t2 = ((d0 + 170u) & dm) | rb;
        if (m8) {                                                      /* the 8-bit path: brightness, then gamma, at 8 bits (<< 8: the rest of the loop is common) */
            r = in[i].r;
            gr = in[i].g;
            b = in[i].b;
            if (!full) {
                r = fx_scale8(r, bri);
                gr = fx_scale8(gr, bri);
                b = fx_scale8(b, bri);
            }
            if (g != 0u) {
                r = gamma_ai(r);
                gr = gamma_ai(gr);
                b = gamma_ai(b);
            }
            r <<= 8;
            gr <<= 8;
            b <<= 8;
        } else {
            if (g != 0u) {
                r = gamma16(in[i].r);
                gr = gamma16(in[i].g);
                b = gamma16(in[i].b);
            } else {
                r = in[i].r * 256u;
                gr = in[i].g * 256u;
                b = in[i].b * 256u;
            }
            if (!full) {
                r = (r * mb + 32768u) >> 16;
                gr = (gr * mb + 32768u) >> 16;
                b = (b * mb + 32768u) >> 16;
            }
        }
        if (white) {
            uint32_t w = r < gr ? r : gr;
            w = w < b ? w : b;
            if (mode == FX_W_ACCURATE) {
                r -= w;
                gr -= w;
                b -= w;
            }
            if (bpp == 5u) {
                w3 = (w * (256u - split)) >> 8;
                w4 = (w * (split + 1u)) >> 8;
            } else {
                w3 = w;
            }
        }
#if ULTRA_PXS_AUTO_MIN_LEVEL > 0u
        t[0] = (uint8_t)((r + (r < mv ? 128u : t0)) >> 8);
        t[1] = (uint8_t)((gr + (gr < mv ? 128u : t1)) >> 8);
        t[2] = (uint8_t)((b + (b < mv ? 128u : t2)) >> 8);
        t[3] = (uint8_t)((w3 + (w3 < mv ? 128u : t0)) >> 8);
        t[4] = (uint8_t)((w4 + (w4 < mv ? 128u : t1)) >> 8);
#else
        t[0] = (uint8_t)((r + t0) >> 8);
        t[1] = (uint8_t)((gr + t1) >> 8);
        t[2] = (uint8_t)((b + t2) >> 8);
        t[3] = (uint8_t)((w3 + t0) >> 8);
        t[4] = (uint8_t)((w4 + t1) >> 8);
#endif
        wire[0] = t[ord[0]];
        wire[1] = t[ord[1]];
        wire[2] = t[ord[2]];
        sum += (uint32_t)t[0] + t[1] + t[2];
        if (bpp >= 4u) {
            wire[3] = t[ord[3]];
            sum += t[3];
            if (bpp == 5u) {
                wire[4] = t[ord[4]];
                sum += t[4];
            }
        }
    }
    return sum;
}

uint32_t fx_pack(const rgb_t *in, uint8_t *wire, uint32_t n, uint8_t bri, const fx_pack_cfg_t *cfg)
{
    return pack_all(in, wire, n, bri, cfg);
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
