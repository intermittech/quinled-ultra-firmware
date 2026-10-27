/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The C side of the V407 layer's C <-> JS goldens: runs an operation script (dashboards-r13/v3/digital/golden/v407/
 * cases.cjs writes it, check.cjs runs the JS on the same script) and prints one line per output:
 *   <case> <op index> <op> <8 hex digits>   (a CRC-32 over bytes, or a number)
 * Ops (decimal integers):
 *   case <name>
 *   begin n fmt id speed intensity palette c1(r g b) c2 c3 flags custom1 custom2 vstart vlen seed
 *                                  (a particle effect gets a lone owner's range: min(its cap, 512 - 64) records)
 *   frame t_ms                     fxv_frame (begin + every slice)
 *   colors r g b r g b r g b       fxv_set_colors
 *   params speed intensity custom1 custom2 seed
 *   render from to [chunk]         -> CRC of the working pixels (r, g, b as 16-bit LE, q, k), rendered in chunks
 *   pack bri gamma bpp o0..o4 white_mode white_split smooth phase px0 dmin   -> CRC of sum (LE32) + wire bytes of the
 *                                  last render
 *   white k_w k_ww k_cw            the white calibration for pack (the defaults at start)
 *   digest                         -> the effect's state digest
 *   espan from to                  -> the engine's next render step end (fxv_engine_span)
 *   noise seed x t octaves         -> fxv_noise1_q15
 *   pal16 palette u wrap           -> the sampled colour (r, g, b, q, k bytes CRC)
 *   gamma v                        -> fxv_gamma16f
 * Usage: fxv_gold cases.txt gold.txt | fxv_gold --raw <case> cases.txt */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fx_v407/fxv_engine.h"

void fx_pal_expand(rgb_t *pal, uint32_t id);

static fxv_ctx_t x;
static uint8_t arena[FXV_SLOT_MAX];
static fxv_particle_t slab[FXV_PARTICLES];
static fxv_px_t px[FXV_N_MAX];
static uint8_t bytes[FXV_N_MAX * 8], wire[FXV_N_MAX * 5 + 4];
static uint32_t last_from, last_n, index_, v[64], crc_table[256];
static char name[128], op[32], line[4096];
static fxv_white_t wc;
static fxv_engine_t eng;
static uint8_t earena[FXV_ARENA_BYTES];
static fxv_particle_t eslab[FXV_PARTICLES];
static FILE *out;
static const char *raw;

static void emit(const void *data, uint32_t n, uint32_t number, int numeric)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t k, c = 0xFFFFFFFFu;
    if (raw && strcmp(raw, name)) return;
    if (!numeric) for (k = 0; k < n; k++) c = crc_table[(c ^ p[k]) & 255u] ^ (c >> 8);
    fprintf(out, "%s %u %s %08x", name, index_, op, numeric ? number : c ^ 0xFFFFFFFFu);
    if (raw && !numeric) { fputc(' ', out); for (k = 0; k < n; k++) fprintf(out, "%02x", p[k]); }
    fputc('\n', out);
}

static uint32_t ser(const fxv_px_t *s, uint32_t n, uint8_t *d)
{
    uint32_t k;
    for (k = 0; k < n; k++, d += 8) {
        d[0] = (uint8_t)s[k].r; d[1] = (uint8_t)(s[k].r >> 8);
        d[2] = (uint8_t)s[k].g; d[3] = (uint8_t)(s[k].g >> 8);
        d[4] = (uint8_t)s[k].b; d[5] = (uint8_t)(s[k].b >> 8);
        d[6] = s[k].q; d[7] = s[k].k;
    }
    return n * 8u;
}

int main(int argc, char **argv)
{
    FILE *in;
    uint32_t k, j;
    const char *cases = "cases.txt", *gold = "gold.txt";
    raw = argc > 2 && !strcmp(argv[1], "--raw") ? argv[2] : 0;
    if (raw) { if (argc > 3) cases = argv[3]; }
    else { if (argc > 1) cases = argv[1]; if (argc > 2) gold = argv[2]; }
    in = fopen(cases, "r");
    out = raw ? stdout : fopen(gold, "wb");
    if (!in || !out) { perror("gold files"); return 2; }
    for (k = 0; k < 256u; k++) { uint32_t c = k; for (j = 0; j < 8u; j++) c = (c >> 1) ^ ((c & 1u) ? 0xEDB88320u : 0u); crc_table[k] = c; }
    fxv_white_init(&wc, FXV_K_W_DEFAULT, FXV_K_WW_DEFAULT, FXV_K_CW_DEFAULT);
    while (fgets(line, sizeof line, in)) {
        char *p, *end;
        uint32_t count = 0;
        if (sscanf(line, "%31s", op) != 1 || op[0] == '#') continue;
        if (!strcmp(op, "case")) { sscanf(line, "case %127s", name); index_ = 0; continue; }
        index_++;
        p = line + strlen(op);
        memset(v, 0, sizeof v);
        while (count < 64u) { unsigned long val = strtoul(p, &end, 10); if (p == end) break; v[count++] = (uint32_t)val; p = end; }
        if (!strcmp(op, "begin")) {
            fx_params_t q;
            memset(&q, 0, sizeof q);
            q.id = (uint8_t)v[2]; q.speed = (uint8_t)v[3]; q.intensity = (uint8_t)v[4]; q.palette = (uint8_t)v[5];
            for (k = 0; k < 3u; k++) { q.c[k].r = (uint8_t)v[6 + 3 * k]; q.c[k].g = (uint8_t)v[7 + 3 * k]; q.c[k].b = (uint8_t)v[8 + 3 * k]; }
            q.flags = (uint8_t)v[15]; q.custom1 = (uint8_t)v[16]; q.custom2 = (uint8_t)v[17];
            q.vstart = (uint16_t)v[18]; q.vlen = (uint16_t)v[19]; q.seed = v[20]; q.bri = 255;
            fxv_begin(&x, &q, v[0], v[1], arena, 0);
            /* a lone owner's particle range, as the engine gives one context alone (its design cap, at most the slab
             * less the fading reserve); the layer zeroes the records at the start. check.cjs and the card's tail.js: the same */
            if (x.d && fxv_meta[x.id].particles != 0u) {
                const uint32_t cap = fxv_meta[x.id].particles;
                x.p = slab;
                x.pn = (uint16_t)(cap < FXV_PARTICLES - FXV_PART_RESERVE ? cap : FXV_PARTICLES - FXV_PART_RESERVE);
            }
        } else if (!strcmp(op, "frame")) {
            fxv_frame(&x, v[0]);
        } else if (!strcmp(op, "colors")) {
            rgb_t c[3];
            for (k = 0; k < 3u; k++) { c[k].r = (uint8_t)v[3 * k]; c[k].g = (uint8_t)v[3 * k + 1]; c[k].b = (uint8_t)v[3 * k + 2]; }
            fxv_set_colors(&x, c);
        } else if (!strcmp(op, "params")) {
            fxv_set_params(&x, (uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3], v[4]);
        } else if (!strcmp(op, "render")) {
            const uint32_t from = v[0], to = v[1] > x.n ? x.n : v[1], ch = v[2] ? v[2] : FXV_CHUNK;
            last_from = from;
            last_n = to > from ? to - from : 0u;
            memset(px, 0, sizeof px);
            for (k = from; k < to; k += ch) fxv_render(&x, px + (k - from), k, k + ch < to ? k + ch : to);
            emit(bytes, ser(px, last_n, bytes), 0, 0);
        } else if (!strcmp(op, "pack")) {
            fx_pack_cfg_t c;
            uint32_t sum;
            memset(&c, 0, sizeof c);
            c.gamma = (uint8_t)v[1]; c.bpp = (uint8_t)v[2];
            for (k = 0; k < 5u; k++) c.ord[k] = (uint8_t)v[3 + k];
            c.white_mode = (uint8_t)v[8]; c.white_split = (uint8_t)v[9]; c.smooth = (uint8_t)v[10]; c.phase = (uint8_t)v[11];
            c.px0 = (uint16_t)v[12]; c.dmin = (uint8_t)v[13];
            sum = fxv_pack16(px, wire + 4, last_n, (uint8_t)v[0], &c, &wc);
            wire[0] = (uint8_t)sum; wire[1] = (uint8_t)(sum >> 8); wire[2] = (uint8_t)(sum >> 16); wire[3] = (uint8_t)(sum >> 24);
            emit(wire, 4u + last_n * c.bpp, 0, 0);
        } else if (!strcmp(op, "white")) {
            fxv_white_init(&wc, v[0], v[1], v[2]);
        } else if (!strcmp(op, "digest")) {
            emit(0, 0, fxv_digest(&x), 1);
        } else if (!strcmp(op, "noise")) {
            emit(0, 0, (uint32_t)fxv_noise1_q15(v[0], v[1], v[2], v[3]), 1);
        } else if (!strcmp(op, "pal16")) {
            rgb_t pal[16];
            fxv_px_t c;
            uint8_t b[8];
            if (v[0] == PAL_COLORS) memcpy(pal, x.pal, sizeof pal); else fx_pal_expand(pal, v[0]);
            c = fxv_pal16(pal, v[1], v[2]);
            emit(b, ser(&c, 1, b), 0, 0);
        } else if (!strcmp(op, "gamma")) {
            emit(0, 0, fxv_gamma16f(v[0]), 1);
        } else if (!strcmp(op, "hash")) {
            emit(0, 0, fxv_hash32(v[0]) ^ fxv_hash2(v[1], v[2]) ^ fxv_rand(v[0], v[1], v[2]), 1);
        } else if (!strcmp(op, "rng")) {
            uint32_t s = v[0], r = 0;
            for (k = 0; k < v[1]; k++) r ^= fxv_rng_next(&s);
            emit(0, 0, r ^ s, 1);
        } else if (!strcmp(op, "ops16")) {
            emit(0, 0, fxv_smooth16(v[0]) ^ (fxv_lerp16(v[1], v[2], v[0]) << 16) ^ (fxv_scale16(v[1], v[2]) << 8), 1);
        } else if (!strcmp(op, "nz")) {           /* nz seed t octaves x0 step count: the walker == noise1 at every point */
            fxv_nz_t z;
            uint32_t xx = v[3], h = 0;
            fxv_nz_begin(&z, v[0], v[1], v[2]);
            for (k = 0; k < v[5]; k++, xx += v[4]) {
                const int32_t a = fxv_nz_at(&z, xx);
                if (a != fxv_noise1_q15(v[0], xx, v[1], v[2])) { fprintf(stderr, "nz walker != noise1\n"); return 3; }
                h = fxv_hash2(h, (uint32_t)a);
            }
            emit(0, 0, h, 1);
        } else if (!strcmp(op, "field")) {        /* field kind n edge p0 p1 p2 p3 steps seed (kinds below) */
            static int16_t f[4][FXV_N_MAX];
            static uint8_t cb[2][FXV_N_MAX];
            const uint32_t kind = v[0], n = v[1], edge = v[2], steps = v[7];
            uint32_t s = v[8], s2 = v[8] ^ 0xABCDu, st, pos, a = 0, b = 1, c2 = 2;
            for (k = 0; k < n; k++) {
                const uint32_t r = fxv_rng_next(&s);
                f[0][k] = (int16_t)((int32_t)(r >> 20) - 2048);
                f[1][k] = (int16_t)((int32_t)((r >> 4) & 4095u) - 2048);
                cb[0][k] = (uint8_t)((r >> 24) % (kind == 5u ? 2u : v[3] ? v[3] : 1u));
                if (kind == 6u) { f[1][k] = (int16_t)((r & 15u) == 0u ? 2048 : 0); f[0][k] = (int16_t)(4096 - f[1][k]); }
            }
            if (kind == 1u) memcpy(f[1], f[0], sizeof f[0]);        /* wave: still at the start (prev = cur) */
            for (st = 0; st < steps; st++) {
                uint32_t ph;
                for (ph = 0; ph < (kind == 2u ? 2u : 1u); ph++) {
                    for (pos = 0; pos < n;) {
                        const uint32_t len = 1u + fxv_rng_next(&s2) % 64u, to = pos + len < n ? pos + len : n;
                        switch (kind) {
                        case 0: fxv_diffuse(f[a], f[b], n, v[3], pos, to, edge); break;
                        case 1: fxv_wave_step(f[a], f[b], f[c2], n, v[3], v[4], pos, to, edge); break;
                        case 2: fxv_spring_step(f[0], f[1], n, v[3], v[4], ph, pos, to, edge); break;
                        case 3: fxv_ca_cyclic(cb[a & 1u], cb[b & 1u], n, v[3], pos, to, edge); break;
                        case 4: fxv_ca_excitable(cb[a & 1u], cb[b & 1u], n, v[3], pos, to, edge); break;
                        case 5: fxv_ca_elem(cb[a & 1u], cb[b & 1u], n, v[3], pos, to, edge); break;
                        default: fxv_rd_step(f[0 + (a & 1u) * 2u], f[1 + (a & 1u) * 2u], f[0 + (b & 1u) * 2u], f[1 + (b & 1u) * 2u], n,
                                             v[3], v[4], v[5], v[6], pos, to, edge); break;
                        }
                        pos = to;
                    }
                }
                if (kind == 1u) { const uint32_t t = a; a = b; b = c2; c2 = t; }
                else if (kind != 2u) { const uint32_t t = a; a = b; b = t; }
            }
            {
                uint8_t buf[FXV_N_MAX * 4];
                uint32_t m = 0;
                for (k = 0; k < n; k++) {
                    int32_t x0, x1;
                    if (kind >= 3u && kind <= 5u) { buf[m++] = cb[a & 1u][k]; continue; }
                    if (kind == 1u) { x0 = f[b][k]; x1 = 0; }
                    else if (kind == 2u) { x0 = f[0][k]; x1 = f[1][k]; }
                    else if (kind == 6u) { x0 = f[0 + (a & 1u) * 2u][k]; x1 = f[1 + (a & 1u) * 2u][k]; }
                    else { x0 = f[a][k]; x1 = 0; }
                    buf[m++] = (uint8_t)x0; buf[m++] = (uint8_t)((uint32_t)x0 >> 8);
                    buf[m++] = (uint8_t)x1; buf[m++] = (uint8_t)((uint32_t)x1 >> 8);
                }
                emit(buf, m, 0, 0);
            }
        } else if (!strcmp(op, "parts")) {        /* parts count steps accel drag seed npx radius */
            static fxv_particle_t pp[FXV_PARTICLES];
            static uint16_t start[FXV_N_MAX + 2], order[FXV_PARTICLES];
            fxv_px_t o[64];
            uint8_t buf[FXV_PARTICLES * 16 + 64 * 8 + 8];
            const uint32_t cnt = v[0], npx = v[5];
            uint32_t s = v[4], m = 0, a, b, live = 0, i;
            memset(pp, 0, sizeof pp);
            for (k = 0; k < cnt; k++) {
                pp[k].x = (int32_t)(fxv_rng_next(&s) % (npx << 16)) - (8 << 16);
                pp[k].v = (int32_t)(fxv_rng_next(&s) % 131072u) - 65536;
                pp[k].life = (uint16_t)(fxv_rng_next(&s) % 300u);
                pp[k].pal = (uint8_t)fxv_rng_next(&s);
            }
            for (k = 0; k < v[1]; k++) live = fxv_particle_step(pp, cnt, (int32_t)v[2] - 4096, v[3]);
            fxv_bins_build(start, order, pp, cnt, npx, 5);
            memset(o, 0, sizeof o);
            fxv_bins_span(start, npx, 5, 32 - 4, 96 + 4, &a, &b);
            {
                rgb_t rp[16];
                fx_pal_expand(rp, 0);
                for (i = a; i < b; i++) {
                    const fxv_particle_t *q = &pp[order[i]];
                    fxv_splat(o, 32, 64, q->x, v[6], fxv_pal16(rp, (uint32_t)q->pal << 8, 1));
                }
            }
            for (k = 0; k < cnt; k++) {
                memcpy(buf + m, &pp[k], 16);
                m += 16;
            }
            m += ser(o, 64, buf + m);
            buf[m++] = (uint8_t)live; buf[m++] = (uint8_t)(live >> 8); buf[m++] = (uint8_t)a; buf[m++] = (uint8_t)b;
            buf[m++] = (uint8_t)fxv_particle_free(pp, cnt);
            emit(buf, m, 0, 0);
        } else if (!strcmp(op, "queue")) {        /* queue cap ops seed */
            uint16_t qb[64];
            fxv_queue_t q;
            uint32_t s = v[2], h = 0;
            fxv_queue_init(&q, qb, v[0]);
            for (k = 0; k < v[1]; k++) {
                if (fxv_rng_next(&s) & 1u) h = fxv_hash2(h, fxv_queue_push(&q, k));
                else h = fxv_hash2(h, fxv_queue_pop(&q));
            }
            emit(0, 0, h ^ q.count, 1);
        } else if (!strcmp(op, "meta")) {         /* meta id particles tick_ms frame px tick particle: a stand-in owner */
            fxv_meta[v[0]].particles = (uint16_t)v[1]; fxv_meta[v[0]].tick_ms = (uint8_t)v[2];
            fxv_meta[v[0]].c_frame = fxv_meta[v[0]].c_frame_pal = v[3];
            fxv_meta[v[0]].c_px = fxv_meta[v[0]].c_px_pal = (uint16_t)v[4];
            fxv_meta[v[0]].c_tick = (uint16_t)v[5]; fxv_meta[v[0]].c_part = (uint16_t)v[6];
        } else if (!strcmp(op, "einit")) {
            fxv_engine_init(&eng, v[0], v[1], earena, eslab);
        } else if (!strcmp(op, "eset")) {         /* eset s id speed intensity palette c1 c2 c3 bri flags custom1 custom2 seg_start seg_len trans vstart vlen seed now anchor */
            fx_params_t q;
            memset(&q, 0, sizeof q);
            q.id = (uint8_t)v[1]; q.speed = (uint8_t)v[2]; q.intensity = (uint8_t)v[3]; q.palette = (uint8_t)v[4];
            for (k = 0; k < 3u; k++) { q.c[k].r = (uint8_t)v[5 + 3 * k]; q.c[k].g = (uint8_t)v[6 + 3 * k]; q.c[k].b = (uint8_t)v[7 + 3 * k]; }
            q.bri = (uint8_t)v[14]; q.flags = (uint8_t)v[15]; q.custom1 = (uint8_t)v[16]; q.custom2 = (uint8_t)v[17];
            q.seg_start = (uint16_t)v[18]; q.seg_len = (uint16_t)v[19]; q.trans_ms = (uint16_t)v[20];
            q.vstart = (uint16_t)v[21]; q.vlen = (uint16_t)v[22]; q.seed = v[23];
            fxv_engine_set(&eng, v[0], &q, v[24], v[25]);
        } else if (!strcmp(op, "espan")) {        /* espan from to -> the render step's end (fxv_engine_span) */
            emit(0, 0, fxv_engine_span(&eng, v[0], v[1]), 1);
        } else if (!strcmp(op, "eframe")) {
            fxv_engine_frame(&eng, v[0]);
        } else if (!strcmp(op, "erender")) {
            const uint32_t from = v[0], to = v[1];
            last_n = to - from;
            for (k = from; k < to; k += FXV_CHUNK) fxv_engine_render(&eng, px + (k - from), k, k + FXV_CHUNK < to ? k + FXV_CHUNK : to);
            emit(bytes, ser(px, last_n, bytes), 0, 0);
        } else if (!strcmp(op, "estate")) {       /* the bookkeeping: slots, quotas, tier, model, the segments' state */
            uint32_t h = fxv_hash2(eng.slot_used, eng.tier);
            h = fxv_hash2(h, eng.cost);
            for (k = 0; k < FXV_SLOTS; k++) h = fxv_hash2(h, (uint32_t)eng.pfirst[k] << 16 | eng.pcount[k]);
            for (k = 0; k < FXV_MAX_SEG; k++) {
                const fxv_seg_t *sg = &eng.seg[k];
                h = fxv_hash2(h, (uint32_t)sg->active | (uint32_t)sg->bri << 8 | (uint32_t)(uint8_t)sg->xf << 16 | (uint32_t)sg->cur.pn << 24);
                h = fxv_hash2(h, sg->active ? sg->cur.slot | (uint32_t)sg->cur.tier << 8 | (uint32_t)sg->cur.id << 16 : 0u);
            }
            for (k = 0; k < FXV_MAX_XFADE; k++) h = fxv_hash2(h, (uint32_t)eng.xf[k].used | (uint32_t)eng.xf[k].w << 8);
            emit(0, 0, h, 1);
        } else if (!strcmp(op, "env")) {
            emit(0, 0, fxv_env(v[0], v[1], v[2], v[3]), 1);
        } else {
            fprintf(stderr, "unknown op %s\n", op);
            return 2;
        }
    }
    fclose(in);
    if (!raw) fclose(out);
    return 0;
}
