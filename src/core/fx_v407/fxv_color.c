/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 effects layer's colour: 16-stop palette sampling, blends, natural white (kelvin), the white mapping for RGBW /
 * RGBCCT strips and the 16-bit packer (fxv_api.h). Integer only; the JS port (fx/v407/common.js) gives the same bytes. */
#include "fxv_gen.h"
#include "../math32.h"
#include "../ultra_protocol.h"       /* ULTRA_PXS_AUTO_MIN_LEVEL (as fx_pack.c) */

#if !defined(__GNUC__)
#define __attribute__(x)
#endif

fxv_px_t fxv_rgb16(rgb_t c)
{
    fxv_px_t o;
    o.r = (uint16_t)(c.r << 8);
    o.g = (uint16_t)(c.g << 8);
    o.b = (uint16_t)(c.b << 8);
    o.q = 0;
    o.k = 0;
    return o;
}

/* cyclic: entry e = u >> 12, 12 fraction bits, entry 15 blends back into 0. Clamped: 0 -> entry 0, 65535 -> entry 15
 * exactly (pos = u x 15 + the rounding that lands 65535 on 15.0) */
fxv_px_t fxv_pal16(const rgb_t *pal, uint32_t u16, uint32_t wrap)
{
    fxv_px_t o;
    rgb_t a, b;
    int32_t f, sh;
    u16 &= 0xFFFFu;
    if (wrap) {
        const uint32_t e = u16 >> 12;
        a = pal[e];
        b = pal[(e + 1u) & 15u];
        f = (int32_t)(u16 & 0xFFFu);
        sh = 4;
    } else {
        const uint32_t pos = u16 * 15u + ((u16 * 15u + 65535u) >> 16), e = pos >> 16;
        a = pal[e];
        b = pal[e < 15u ? e + 1u : 15u];
        f = (int32_t)(pos & 0xFFFFu);
        sh = 8;
    }
    o.r = (uint16_t)(((int32_t)a.r << 8) + ((((int32_t)b.r - a.r) * f) >> sh));
    o.g = (uint16_t)(((int32_t)a.g << 8) + ((((int32_t)b.g - a.g) * f) >> sh));
    o.b = (uint16_t)(((int32_t)a.b << 8) + ((((int32_t)b.b - a.b) * f) >> sh));
    o.q = 0;
    o.k = 0;
    return o;
}

fxv_px_t fxv_blend16(fxv_px_t a, fxv_px_t b, uint32_t w)
{
    fxv_px_t o;
    o.r = (uint16_t)fxv_lerp16(a.r, b.r, w);
    o.g = (uint16_t)fxv_lerp16(a.g, b.g, w);
    o.b = (uint16_t)fxv_lerp16(a.b, b.b, w);
    o.q = (uint8_t)fxv_lerp16(a.q, b.q, w);
    o.k = (uint8_t)fxv_lerp16(a.k, b.k, w);
    return o;
}

fxv_px_t fxv_kelvin16(uint32_t k8)
{
    const uint32_t j = (k8 & 255u) >> 4, f = k8 & 15u;
    const uint8_t *a = fxv_kelvin_tab[j], *b = fxv_kelvin_tab[j + 1u];
    fxv_px_t o;
    o.r = (uint16_t)(((int32_t)a[0] << 8) + ((int32_t)b[0] - a[0]) * (int32_t)f * 16);
    o.g = (uint16_t)(((int32_t)a[1] << 8) + ((int32_t)b[1] - a[1]) * (int32_t)f * 16);
    o.b = (uint16_t)(((int32_t)a[2] << 8) + ((int32_t)b[2] - a[2]) * (int32_t)f * 16);
    o.q = 255;
    o.k = (uint8_t)k8;
    return o;
}

fxv_px_t fxv_scale_px(fxv_px_t c, uint32_t s16)
{
    c.r = (uint16_t)fxv_scale16(c.r, s16);
    c.g = (uint16_t)fxv_scale16(c.g, s16);
    c.b = (uint16_t)fxv_scale16(c.b, s16);
    return c;
}

fxv_px_t fxv_add_px(fxv_px_t a, fxv_px_t b)
{
    fxv_px_t o;
    const uint32_t r = (uint32_t)a.r + b.r, g = (uint32_t)a.g + b.g, bl = (uint32_t)a.b + b.b;
    const int bb = (uint32_t)b.r + b.g + b.b > (uint32_t)a.r + a.g + a.b;
    o.r = (uint16_t)(r > FXV_FULL ? FXV_FULL : r);
    o.g = (uint16_t)(g > FXV_FULL ? FXV_FULL : g);
    o.b = (uint16_t)(bl > FXV_FULL ? FXV_FULL : bl);
    o.q = bb ? b.q : a.q;
    o.k = bb ? b.k : a.k;
    return o;
}

/* gamma 2.8 of a working value: the 8-bit curve's 16-bit points (fxv_gamma_tab = fx_pack.c's gamma16, generated),
 * linear between them (exact at multiples of 256) */
uint32_t fxv_gamma16f(uint32_t v)
{
    uint32_t i, a;
    if (v > FXV_FULL) v = FXV_FULL;
    i = v >> 8;
    a = fxv_gamma_tab[i];
    return a + (((fxv_gamma_tab[i + 1u] - a) * (v & 255u)) >> 8);
}

/* ------------------------------------------------------------------------------- white mapping */
/* an emitter at white k as linear RGB, the largest channel 65535 */
static void white_eq(uint32_t k, uint16_t *o)
{
    const fxv_px_t c = fxv_kelvin16(k > 255u ? 255u : k);
    const uint32_t g[3] = { fxv_gamma16f(c.r), fxv_gamma16f(c.g), fxv_gamma16f(c.b) };
    uint32_t m = g[0], j;
    if (g[1] > m) m = g[1];
    if (g[2] > m) m = g[2];
    for (j = 0; j < 3u; j++) {
        o[j] = (uint16_t)(m != 0u ? m32_muldiv(g[j], 65535u, m) : 0u);
    }
}

static uint32_t inv32(uint32_t w)                       /* 2^32 / w; 0 = the channel never limits (w < 256) */
{
    return w >= 256u ? m32_div(1u, 0u, w) : 0u;
}

void fxv_white_init(fxv_white_t *wc, uint32_t k_w, uint32_t k_ww, uint32_t k_cw)
{
    uint32_t j, c;
    if (k_cw <= k_ww) k_cw = k_ww + 1u;                  /* (the cold white is the cooler one) */
    if (k_cw > 255u) { k_cw = 255u; if (k_ww > 254u) k_ww = 254u; }
    white_eq(k_w, wc->w);
    white_eq(k_ww, wc->ww);
    white_eq(k_cw, wc->cw);
    wc->k_ww = (uint8_t)k_ww;
    wc->k_cw = (uint8_t)k_cw;
    wc->rcp = (256u << 16) / (k_cw - k_ww);
    wc->spare[0] = wc->spare[1] = 0;
    for (c = 0; c < 3u; c++) {
        wc->winv[c] = inv32(wc->w[c]);
        for (j = 0; j < 65u; j++) {
            wc->minv[j][c] = inv32((uint32_t)((int32_t)wc->ww[c] + ((((int32_t)wc->cw[c] - (int32_t)wc->ww[c]) * (int32_t)(4u * j)) >> 8)));
        }
    }
}

/* ------------------------------------------------------------------------------- the packer */
/* one channel to its 16-bit linear level: gamma (the table, linear between its points), the limiter's brightness */
static inline __attribute__((always_inline)) uint32_t lin16(uint32_t v, uint32_t g, uint32_t full, uint32_t mb)
{
    if (v > FXV_FULL) v = FXV_FULL;
    if (g) {
        const uint32_t i = v >> 8, a = fxv_gamma_tab[i];
        v = a + (((fxv_gamma_tab[i + 1u] - a) * (v & 255u)) >> 8);
    }
    return full ? v : (v * mb + 32768u) >> 16;
}

/* the neutral share of a natural-white pixel as the calibrated emitters' level w (<= FXV_FULL); e: their RGB
 * equivalent, inv: 2^32 / e (0: that channel never limits) */
static inline __attribute__((always_inline)) uint32_t white_level(const uint32_t *ch, uint32_t q, const uint32_t *e, const uint32_t *inv)
{
    uint32_t w = FXV_FULL, c;
    for (c = 0; c < 3u; c++) {
        if (inv[c] != 0u) {                              /* w <= a / e = a x (2^32 / e) >> 16, split so it never overflows */
            const uint32_t a = (ch[c] * q) >> 8;
            const uint32_t lim = a * (inv[c] >> 16) + ((a * (inv[c] & 0xFFFFu)) >> 16);
            if (lim < w) w = lim;
        }
    }
    (void)e;
    return w;
}

uint32_t fxv_pack16(const fxv_px_t *in, uint8_t *wire, uint32_t n, uint8_t bri, const fx_pack_cfg_t *cfg,
                    const fxv_white_t *wc)
{
    const uint8_t *ord = cfg->ord;
    const uint32_t g = cfg->gamma, bpp = cfg->bpp, mode = cfg->white_mode, split = cfg->white_split, full = bri == 255u;
    const uint32_t m8 = cfg->smooth == FX_SM_8BIT, dm = cfg->smooth == FX_SM_DITHER ? 255u : 0u, rb = m8 ? 0u : 128u & ~dm;
    const uint32_t gb = g ? fxv_gamma_tab[bri] : bri * 256u, mb = gb + (gb >> 8);
    const uint32_t white = bpp >= 4u && (mode == FX_W_ACCURATE || mode == FX_W_BRIGHT), acc = mode == FX_W_ACCURATE;
#if ULTRA_PXS_AUTO_MIN_LEVEL > 0u
    const uint32_t mv = (uint32_t)cfg->dmin << 8;
#endif
    uint32_t i, sum = 0, d0 = cfg->phase + (uint32_t)cfg->px0 * 53u;
    for (i = 0; i < n; i++, wire += bpp, d0 += 53u) {
        uint8_t t[5];
        uint32_t ch[3], w3 = 0, w4 = 0;
        const uint32_t t0 = (d0 & dm) | rb, t1 = ((d0 + 85u) & dm) | rb, t2 = ((d0 + 170u) & dm) | rb;
        const fxv_px_t px = in[i];
        if (m8) {                                       /* the 8-bit path (fx_pack's): brightness, then gamma, at 8 bits */
            uint32_t r = (px.r > FXV_FULL ? FXV_FULL : px.r) + 128u, gg = (px.g > FXV_FULL ? FXV_FULL : px.g) + 128u;
            uint32_t b = (px.b > FXV_FULL ? FXV_FULL : px.b) + 128u;
            r >>= 8; gg >>= 8; b >>= 8;
            if (!full) { r = fx_scale8(r, bri); gg = fx_scale8(gg, bri); b = fx_scale8(b, bri); }
            if (g) { r = (fxv_gamma_tab[r] + 128u) >> 8; gg = (fxv_gamma_tab[gg] + 128u) >> 8; b = (fxv_gamma_tab[b] + 128u) >> 8; }
            ch[0] = r << 8; ch[1] = gg << 8; ch[2] = b << 8;
        } else {
            ch[0] = lin16(px.r, g, full, mb);
            ch[1] = lin16(px.g, g, full, mb);
            ch[2] = lin16(px.b, g, full, mb);
        }
        if (white) {
            uint32_t w;
            if (px.q == 0u) {                           /* an ordinary colour: the legacy auto-white */
                w = ch[0] < ch[1] ? ch[0] : ch[1];
                w = w < ch[2] ? w : ch[2];
                if (acc) { ch[0] -= w; ch[1] -= w; ch[2] -= w; }
                if (bpp == 5u) { w3 = (w * (256u - split)) >> 8; w4 = (w * (split + 1u)) >> 8; }
                else w3 = w;
            } else if (bpp == 4u) {                     /* natural white: the single white takes the neutral share */
                w = white_level(ch, px.q, 0, wc->winv);
                if (acc) {
                    uint32_t c;
                    for (c = 0; c < 3u; c++) {
                        const uint32_t sb = (w * wc->w[c] + 32768u) >> 16;
                        ch[c] = ch[c] > sb ? ch[c] - sb : 0u;
                    }
                }
                w3 = w;
            } else {                                    /* ... WW + CW, mixed for the pixel's temperature */
                const uint32_t k = px.k, fc = k <= wc->k_ww ? 0u : k >= wc->k_cw ? 256u : ((k - wc->k_ww) * wc->rcp) >> 16;
                const uint32_t jf = fc >> 2, ff = fc & 3u, j1 = jf < 64u ? jf + 1u : 64u;
                uint32_t e[3], inv[3], c;
                for (c = 0; c < 3u; c++) {
                    const uint32_t i0 = wc->minv[jf][c], i1 = wc->minv[j1][c];
                    e[c] = (uint32_t)((int32_t)wc->ww[c] + ((((int32_t)wc->cw[c] - (int32_t)wc->ww[c]) * (int32_t)fc) >> 8));
                    inv[c] = i0 == 0u || i1 == 0u ? 0u : (uint32_t)((int32_t)i0 + ((((int32_t)i1 - (int32_t)i0) * (int32_t)ff) >> 2));
                }
                w = white_level(ch, px.q, e, inv);
                if (acc) {
                    for (c = 0; c < 3u; c++) {
                        const uint32_t sb = (w * e[c] + 32768u) >> 16;
                        ch[c] = ch[c] > sb ? ch[c] - sb : 0u;
                    }
                }
                w4 = (w * fc) >> 8;
                w3 = w - w4;
            }
        }
#if ULTRA_PXS_AUTO_MIN_LEVEL > 0u
        t[0] = (uint8_t)((ch[0] + (ch[0] < mv ? 128u : t0)) >> 8);
        t[1] = (uint8_t)((ch[1] + (ch[1] < mv ? 128u : t1)) >> 8);
        t[2] = (uint8_t)((ch[2] + (ch[2] < mv ? 128u : t2)) >> 8);
        t[3] = (uint8_t)((w3 + (w3 < mv ? 128u : t0)) >> 8);
        t[4] = (uint8_t)((w4 + (w4 < mv ? 128u : t1)) >> 8);
#else
        t[0] = (uint8_t)((ch[0] + t0) >> 8);
        t[1] = (uint8_t)((ch[1] + t1) >> 8);
        t[2] = (uint8_t)((ch[2] + t2) >> 8);
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
