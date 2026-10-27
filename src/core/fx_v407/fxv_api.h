/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 Digital-M effects layer (effects 37 and up, palettes 31 and up): the API every effect module codes against.
 * docs: src/core/fx_v407/README.md (the effect contract, the manifest, the JS fragment contract, the gates).
 *
 * An effect is a descriptor (fxv_desc_t) of six functions on a context (fxv_ctx_t):
 *   bytes(n, tier)          arena bytes it needs for a run of n pixels at particle tier `tier` (<= FXV_SLOT_BYTES(n))
 *   init(x)                 the arena is zeroed and x is set up: build the initial state (bounded, <= ~5,000 cycles)
 *   advance_begin(x)        once per frame, time already advanced: per-frame numbers (x->u[]), start the frame's job
 *   advance_slice(x, quota) bounded work (about `quota` cells / pixels / particle steps); nonzero: call again
 *   render16(x, from, n, o) effect pixels [from, from + n) -> o[0 .. n), n <= FXV_CHUNK; READ-ONLY on the state
 *   destroy(x)              optional: the context ends (particles are released by the layer)
 * plus changed(x) (optional: speed / intensity / custom / colours / seed changed) and digest(x) (optional: a state hash
 * for the C <-> JS goldens).
 *
 * Rules: integer only (no float, no soft-float), no division per pixel, no heap, no globals that
 * change (all state lives in the context, its arena slot and its particle range). Rendering is a pure function of the
 * state after the frame's advance: any chunk order gives the same pixels. Every random draw is tied to the simulation tick
 * and a stable object id (fxv_rand / fxv_hash32), never to render order. Yielding between slices never changes results.
 * The same code runs in the host tests (MSVC) and on the receiver; the JS port (dashboards-r13/v3/digital/fx/v407/) uses
 * the same names and must give the same bytes.
 *
 * Working colour (fxv_px_t): r, g, b on the legacy perceptual scale with 8 fractional bits (8-bit value x 256, full =
 * FXV_FULL = 65280; gamma comes once, in fxv_pack16), plus q = the neutral (white) share of the pixel 0..255 and
 * k = that white's colour temperature (0 = 2000 K .. 255 = 6500 K, fxv_kelvin16). q = 0 is an ordinary colour. */
#ifndef FXV_API_H
#define FXV_API_H

#include <stdint.h>
#include "../fx.h"
#include "../fx_int.h"

/* ---------------------------------------------------------------------------------- geometry and reservations */
#define FXV_N_RGB        800u            /* the longest strips (decided 2026-10-23: 42 fps floor, ~41 fps at full length) */
#define FXV_N_RGBW       600u
#define FXV_N_RGBCCT     500u
#define FXV_N_MAX        FXV_N_RGB
#define FXV_FRAME_BYTES  2500u           /* max(800 x 3, 600 x 4, 500 x 5) wire bytes */
#define FXV_FRAME_ALLOC  2512u           /* one wire frame, 16-byte aligned */
/* S(n): the arena an effect may use for a run of n pixels: 12 bytes a pixel plus 1 KiB of control state */
#define FXV_SLOT_BYTES(n) ((12u * (uint32_t)(n) + 1024u + 15u) & ~15u)
#define FXV_SLOT_MAX     FXV_SLOT_BYTES(FXV_N_MAX)        /* 10,624 */
#define FXV_SLOTS        6u              /* four segments + two fading-out contexts */
#define FXV_ARENA_BYTES  (FXV_SLOTS * FXV_SLOT_MAX)       /* 63,744: fixed at the maximum geometry */
#define FXV_PARTICLES    512u            /* the one global slab (16 bytes a record: 8 KiB) */
#define FXV_PART_RESERVE 64u             /* kept for the contexts fading out */
#define FXV_CHUNK        32u             /* the most pixels one render16 call is asked for */
#define FXV_QUOTA        32u             /* the default work of one advance slice (tune to <= 5,000 cycles) */
#define FXV_TICKS_MAX    16u             /* simulation ticks one frame may run (the rest stays as debt) */
#define FXV_RESYNC_MS    2000u           /* a debt over this much effect time: restart (seed + epoch), no huge catch-up */
#define FXV_FULL         65280u          /* full scale of a working channel (255 x 256) */
#define FXV_FPS_MAX      100u            /* short strips: as fast as the wire allows, at most this */
#define FXV_BUDGET_CYCLES 1000000u       /* a frame's modeled advance + render + pack budget (the particle step-down) */
#define FXV_HOLD_N       12u             /* a strip this short is all one spot: FXV_D_HOLD effects hold their light there */

typedef struct { uint16_t r, g, b; uint8_t q, k; } fxv_px_t;

/* One particle record: 16 bytes. Position and velocity in effect pixels, Q16.16 (signed: a particle may be off the run);
 * velocity is per simulation tick. life 0 = a free record (the layer zeroes records it hands over). */
typedef struct {
    int32_t  x;
    int32_t  v;
    uint16_t age, life;                  /* ticks lived, lifetime in ticks */
    uint8_t  pal;                        /* palette position / colour index (the effect's) */
    uint8_t  phase;                      /* free for the effect */
    uint8_t  flags;                      /* free for the effect */
    uint8_t  kind;                       /* free for the effect */
} fxv_particle_t;
typedef char fxv_particle_is_16_bytes[sizeof(fxv_particle_t) == 16u ? 1 : -1];

typedef struct fxv_ctx_s fxv_ctx_t;

#define FXV_D_FLASHY   0x01u             /* fxv_meta flags: may flash (the default flash rate stays <= 3 Hz) */
#define FXV_D_WHITE    0x02u             /* natural white: sets q / k for white strips */
#define FXV_D_HOLD     0x04u             /* under FXV_HOLD_N pixels the layer holds its light (fxv_core.c: the flash gate) */
#define FXV_D_LEGACY   0x80u             /* effects 1-36 through the adapter (fxv_legacy.c) */

/* The functions of an effect. Its metadata (tick, particle cap, flags, modeled cost) is the manifest's, generated into
 * fxv_meta[id] (fxv_gen.h): one source for C, JS and the scheduler. */
typedef struct {
    uint32_t (*bytes)(uint32_t n, uint32_t tier);
    void     (*init)(fxv_ctx_t *x);
    void     (*advance_begin)(fxv_ctx_t *x);
    uint32_t (*advance_slice)(fxv_ctx_t *x, uint32_t quota);        /* 0: none (an analytic effect) */
    void     (*render16)(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *out);
    void     (*destroy)(fxv_ctx_t *x);                              /* optional */
    void     (*changed)(fxv_ctx_t *x);                              /* optional */
    uint32_t (*digest)(const fxv_ctx_t *x);                         /* optional */
} fxv_desc_t;

/* The context. The layer owns the fields above `u`; the effect owns u[], job[] and its arena. */
#if defined(__GNUC__)
typedef uint32_t fxv_word_t __attribute__((__may_alias__));   /* a context zeroed / moved a word at a time */
#else
typedef uint32_t fxv_word_t;
#endif
struct fxv_ctx_s {
    const fxv_desc_t *d;
    uint8_t  id, speed, intensity, palette;   /* palette: the resolved id (fxv_pal_resolve: 0..FXV_PAL_COUNT-1, PAL_COLORS, or PAL_DEFAULT = own) */
    uint8_t  custom1, custom2, flags, fmt;    /* flags: FX_F_*; fmt: wire bytes per pixel (3, 4, 5) */
    rgb_t    c[3];                       /* the current colours (ramped by the engine) */
    uint8_t  pal_wrap;                   /* 1: the palette is cyclic (fxv_pal wraps), 0: clamp its ends */
    uint8_t  tier;                       /* particle step-down tier (0: the design count) */
    uint16_t n, vlen;                    /* pixels in the run, the virtual length (p->vlen, or n) */
    uint16_t epoch;                      /* restarts after a time discontinuity (mixed into seed) */
    uint8_t  stage, et_frac;             /* (the layer's) */
    uint32_t vbase;                      /* the virtual position of the run's pixel 0 */
    uint32_t seed;                       /* the slot's seed (+ the segment, + the epoch) */
    uint32_t t, t_last;                  /* ms since the anchor (saturating), the t of the previous frame */
    uint32_t et;                         /* speed-integrated time, ms at 1x (wraps; speed 0 stands still) */
    uint32_t de;                         /* et advanced by this frame */
    uint32_t tick;                       /* simulation ticks done (wraps) */
    uint32_t tick_acc;                   /* effect ms not yet a whole tick */
    uint32_t tick_debt;                  /* ticks due and not run yet */
    uint32_t tick_frame;                 /* of those, ticks this frame may still run (fxv_tick_next) */
    uint8_t  *arena;                     /* the slot: FXV_SLOT_MAX bytes, `need` of them zeroed before init */
    uint32_t need;                       /* d->bytes(n, tier) */
    uint32_t zpos;                       /* (the layer's: zeroing position) */
    fxv_particle_t *p;                   /* the particle range (0 when none) */
    uint16_t pn;                         /* records in it (the quota; may change between frames) */
    uint8_t  slot, pending_adv;          /* (the layer's) */
    uint8_t  tick_ms;                    /* fxv_meta[id].tick_ms (0: no simulation ticks) */
    uint8_t  hold, hold_dt;              /* (the layer's: the short-strip hold on, ms since its last pass, <= 255) */
    uint8_t  pal_due;                    /* (the layer's: the palette is built in the first start slice) */
    uint8_t  pal_own;                    /* 1: draw the designed look (Effect default on an effect with its own colours),
                                            0: use pal[] (always built from the chosen palette before the first slice) */
    rgb_t    pal[16];                   /* the active palette, 16 entries */
    uint32_t u[8];                       /* per-frame numbers: free for the effect */
    uint32_t job[4];                     /* the frame's slice-job position: free for the effect */
};
typedef char fxv_ctx_words[sizeof(fxv_ctx_t) % 4u == 0u ? 1 : -1];

/* ---------------------------------------------------------------------------------- the layer (fxv_core.c) */
const fxv_desc_t *fxv_desc(uint32_t id);       /* the effect's descriptor in this build, 0: not built (never Solid) */
uint32_t fxv_built(uint32_t id);               /* 1: id runs here (1-36 through the adapter, 37+ when built) */
uint8_t  fxv_default_palette(uint32_t id);     /* the palette PAL_DEFAULT stands for (PAL_DEFAULT: the effect's own colours) */
uint8_t  fxv_pal_resolve(uint32_t id, uint32_t palette);   /* a slot's palette byte -> x->palette (fxv_begin, the engine) */
/* p: the slot's parameters (p->id must be fxv_built). n pixels, fmt wire bytes per pixel, arena: FXV_SLOT_MAX bytes */
void     fxv_begin(fxv_ctx_t *x, const fx_params_t *p, uint32_t n, uint32_t fmt, uint8_t *arena, uint32_t slot);
void     fxv_frame_begin(fxv_ctx_t *x, uint32_t t_ms);  /* t_ms: since the anchor; then fxv_frame_step until 0 */
void     fxv_clock(fxv_ctx_t *x, uint32_t t_ms);        /* only the clock (et, ticks due) up to t_ms, no advance: a
                                                           parameter change brings the effect to now at the old speed */
uint32_t fxv_frame_step(fxv_ctx_t *x);                  /* one bounded slice (zeroing, init, advance); 0: ready */
void     fxv_frame(fxv_ctx_t *x, uint32_t t_ms);        /* both, to completion */
#define FXV_ST_IDLE 2u                                  /* x->stage: the frame's advance is complete (fxv_core.c ST_IDLE) */
static inline uint32_t fxv_ready(const fxv_ctx_t *x) { return x->stage == FXV_ST_IDLE; }   /* 1: render may run */
void     fxv_render(const fxv_ctx_t *x, fxv_px_t *out, uint32_t from, uint32_t to);   /* to - from <= FXV_CHUNK */
void     fxv_set_colors(fxv_ctx_t *x, const rgb_t c[3]);
void     fxv_set_params(fxv_ctx_t *x, uint8_t speed, uint8_t intensity, uint8_t custom1, uint8_t custom2, uint32_t seed);
void     fxv_end(fxv_ctx_t *x);
uint32_t fxv_digest(const fxv_ctx_t *x);                /* the effect's digest, or 0 */

/* the next simulation tick of this frame: 1 (and x->tick counts it) or 0 (the frame's ticks are done) */
static inline uint32_t fxv_tick_next(fxv_ctx_t *x)
{
    if (x->tick_frame == 0u) return 0u;
    x->tick_frame--;
    x->tick_debt--;
    x->tick++;
    return 1u;
}

/* ---------------------------------------------------------------------------------- maths (fxv_math.c) */
static inline int32_t  fxv_sin16(uint32_t th) { return fx_sin16(th); }   /* 65536 a turn -> -32767 .. 32767 */
static inline int32_t  fxv_cos16(uint32_t th) { return fx_sin16(th + 16384u); }
static inline uint32_t fxv_hash32(uint32_t x) { return fx_hash(x); }    /* lowbias32 */
static inline uint32_t fxv_hash2(uint32_t a, uint32_t b) { return fx_hash2(a, b); }
/* xorshift32 (13 / 17 / 5); a zero state becomes FXV_RNG_ZERO first */
#define FXV_RNG_ZERO 0x6D2B79F5u
static inline uint32_t fxv_rng_next(uint32_t *s)
{
    uint32_t x = *s != 0u ? *s : FXV_RNG_ZERO;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}
/* a draw tied to (seed, tick, object): the same on every receiver and in JS, whatever the render order */
static inline uint32_t fxv_rand(uint32_t seed, uint32_t tick, uint32_t obj)
{
    return fx_hash(seed ^ fx_hash(tick * 0x9E3779B1u + obj * 0x85EBCA6Bu));
}
static inline uint32_t fxv_scale16(uint32_t v, uint32_t s) { return (v * (s + 1u)) >> 16; }   /* v <= 65535, s 65535: v */
static inline uint32_t fxv_lerp16(uint32_t a, uint32_t b, uint32_t f)   /* a -> b by f 0..65535 (65535: b exactly) */
{
    return (uint32_t)((int32_t)a + (((int32_t)b - (int32_t)a) * (int32_t)((f + (f >> 15)) >> 1) >> 15));
}
static inline uint32_t fxv_smooth16(uint32_t f)                         /* 3f^2 - 2f^3 on 0..65535 */
{
    const uint32_t a = f >> 1, s = (a * a) >> 15;                       /* s = f^2 (Q15) */
    const uint32_t r = (s * (98304u - 2u * a)) >> 14;
    return r > 65535u ? 65535u : r;
}
/* value noise, Q15 output -32767 .. 32767: 1-3 octaves of a seeded lattice along x (Q16: 65536 a lattice cell) at time t
 * (Q16 cells); smooth interpolation, no division */
int32_t  fxv_noise1_q15(uint32_t seed, uint32_t x_q16, uint32_t t_q16, uint32_t octaves);
/* the same noise walked along x at one t (render16: pixels upward), the lattice columns cached; gives fxv_noise1_q15's
 * values exactly (any x order works, upward steps are the cheap ones) */
typedef struct { uint32_t seed[3], t[3], xi[3]; int32_t a[3], b[3]; uint16_t tf[3]; uint8_t ok[3], oct; } fxv_nz_t;
void     fxv_nz_begin(fxv_nz_t *z, uint32_t seed, uint32_t t_q16, uint32_t octaves);
int32_t  fxv_nz_at(fxv_nz_t *z, uint32_t x_q16);

/* ---------------------------------------------------------------------------------- fields (fxv_math.c)
 * One step of a 1-D field over cells [from, to) of n, reading only the input planes (immutable during the step) and
 * writing only the output plane: slicing a step into any ranges gives the same plane. Coefficients are Q12 (4096 =
 * 1.0); intermediates 32-bit; results saturate to int16. Edges: what a cell outside [0, n) reads. */
#define FXV_EDGE_CLAMP 0u                /* the end cell repeats (no flux, free end) */
#define FXV_EDGE_ZERO  1u                /* 0 outside (absorbing, a pinned string) */
#define FXV_EDGE_WRAP  2u                /* a ring */
#define FXV_Q12        4096
static inline int32_t fxv_sat16(int32_t v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : v; }
/* diffusion: d = s + coef x (s[i-1] - 2 s[i] + s[i+1]), coef <= 1024 (0.25: stable) */
void     fxv_diffuse(const int16_t *s, int16_t *d, uint32_t n, uint32_t coef_q12, uint32_t from, uint32_t to, uint32_t edge);
/* damped wave: next = cur + (cur - prev)(1 - damp) + c2 x laplacian(cur); c2 <= 4000 (the Courant number below 1:
 * at exactly 1 the alternating mode has a double root and grows) */
void     fxv_wave_step(const int16_t *prev, const int16_t *cur, int16_t *next, uint32_t n, uint32_t c2_q12, uint32_t damp_q12,
                       uint32_t from, uint32_t to, uint32_t edge);
/* a spring chain in two phases per step: phase 0 vel += k x laplacian(pos) - damp x vel (reads pos only), then (after all
 * of phase 0) phase 1 pos += vel. k <= 4096 */
void     fxv_spring_step(int16_t *pos, int16_t *vel, uint32_t n, uint32_t k_q12, uint32_t damp_q12, uint32_t phase,
                         uint32_t from, uint32_t to, uint32_t edge);
/* cellular automata on byte cells (old -> next): an elementary rule (Wolfram's 0..255, cells 0 / 1); cyclic (a cell
 * goes to c + 1 mod states when a neighbour holds it); excitable (Greenberg-Hastings: 0 rests and fires (1) next to a 1,
 * every other state counts up to states - 1 and back to rest) */
void     fxv_ca_elem(const uint8_t *old, uint8_t *next, uint32_t n, uint32_t rule, uint32_t from, uint32_t to, uint32_t edge);
void     fxv_ca_cyclic(const uint8_t *old, uint8_t *next, uint32_t n, uint32_t states, uint32_t from, uint32_t to, uint32_t edge);
void     fxv_ca_excitable(const uint8_t *old, uint8_t *next, uint32_t n, uint32_t states, uint32_t from, uint32_t to, uint32_t edge);
/* Gray-Scott reaction-diffusion, u and v Q12 (0..4096): uvv = u v^2; u' = u + du lap(u) - uvv + feed (1 - u),
 * v' = v + dv lap(v) + uvv - (feed + kill) v; du, dv <= 1024. In 1-D at this resolution feed 225 / kill 254 and
 * 160 / 250 (du 1024, dv 512) keep spots; 150 / 266 dies out (test_fxv) */
void     fxv_rd_step(const int16_t *u, const int16_t *v, int16_t *un, int16_t *vn, uint32_t n, uint32_t du_q12, uint32_t dv_q12,
                     uint32_t feed_q12, uint32_t kill_q12, uint32_t from, uint32_t to, uint32_t edge);

/* ---------------------------------------------------------------------------------- particles and events (fxv_sim.c) */
/* one tick of records [0, n): x += v; v += accel; v -= v x drag (Q16); age++; age >= life: freed (life 0). Returns live */
uint32_t fxv_particle_step(fxv_particle_t *p, uint32_t n, int32_t accel_q16, uint32_t drag_q16);
/* the first free record (life 0) of [0, n), or n */
uint32_t fxv_particle_free(const fxv_particle_t *p, uint32_t n);
/* bins: live records sorted by pixel (x >> 16) into bins of 2^shift pixels, a counting sort (stable by index);
 * start: nb + 1 entries (nb = (n >> shift) + 1), order: one per record. Positions off the run go to the end bins */
uint32_t fxv_bins_count(uint32_t n, uint32_t shift);                      /* nb */
void     fxv_bins_build(uint16_t *start, uint16_t *order, const fxv_particle_t *p, uint32_t pn, uint32_t n, uint32_t shift);
/* the order index range [*a, *b) of records whose bins overlap pixels [lo, hi) (signed: a kernel reaches outside) */
void     fxv_bins_span(const uint16_t *start, uint32_t n, uint32_t shift, int32_t lo, int32_t hi, uint32_t *a, uint32_t *b);
/* add colour c x a triangular kernel (radius r, Q16, >= 0.5 px) centred at x (Q16) into out[0 .. count) = pixels
 * [from, from + count) of the run, saturating; radius 1 px = two-pixel coverage */
void     fxv_splat(fxv_px_t *out, uint32_t from, uint32_t count, int32_t x_q16, uint32_t r_q16, fxv_px_t c);
/* a bounded index queue (ring) over buf[cap]: push 0 when full (the caller counts drops), pop 0xFFFF when empty */
typedef struct { uint16_t *buf; uint16_t cap, head, count, spare; } fxv_queue_t;
void     fxv_queue_init(fxv_queue_t *q, uint16_t *buf, uint32_t cap);
uint32_t fxv_queue_push(fxv_queue_t *q, uint32_t v);
uint32_t fxv_queue_pop(fxv_queue_t *q);
/* an event envelope 0..65535 at t ms after the event: smooth rise over attack, hold, smooth fall over decay */
uint32_t fxv_env(uint32_t t_ms, uint32_t attack_ms, uint32_t hold_ms, uint32_t decay_ms);

/* ---------------------------------------------------------------------------------- colour (fxv_color.c) */
/* 16 entries -> a working colour at u (0..65535 round the palette; wrap 0: clamped ends, entry 0 at 0, 15 at 65535) */
fxv_px_t fxv_pal16(const rgb_t *pal, uint32_t u16, uint32_t wrap);
static inline fxv_px_t fxv_pal(const fxv_ctx_t *x, uint32_t u16) { return fxv_pal16(x->pal, u16, x->pal_wrap); }
fxv_px_t fxv_blend16(fxv_px_t a, fxv_px_t b, uint32_t w);   /* a -> b by w 0..65535 (q, k blended too) */
fxv_px_t fxv_rgb16(rgb_t c);                                 /* an 8-bit colour, q 0 */
fxv_px_t fxv_kelvin16(uint32_t k8);                          /* natural white at k (0 = 2000 K .. 255 = 6500 K): q 255 */
fxv_px_t fxv_scale_px(fxv_px_t c, uint32_t s16);             /* brightness 0..65535 (q, k kept) */
fxv_px_t fxv_add_px(fxv_px_t a, fxv_px_t b);                 /* saturating add (q, k: the brighter one's) */

/* White mapping for the packer: the calibrated emitters as RGB-equivalents (linear, 65535 = the full channel) */
typedef struct {
    uint16_t w[3];                       /* the single white (RGBW) */
    uint16_t ww[3], cw[3];               /* RGBCCT: warm and cold white; white k is the mix with cool share fcw(k) 0..256 */
    uint32_t winv[3];                    /* 2^32 / w[c] (0: the channel never limits) */
    uint32_t minv[65][3];                /* 2^32 / the mix at cool share 4 j (linear between the points) */
    uint32_t rcp;                        /* fcw(k) = (k - k_ww) x rcp >> 16 between the two whites */
    uint8_t  k_ww, k_cw;
    uint8_t  spare[2];
} fxv_white_t;
#define FXV_K_W_DEFAULT   113u           /* the single white: about 4000 K */
#define FXV_K_WW_DEFAULT  40u            /* warm white: about 2700 K */
#define FXV_K_CW_DEFAULT  255u           /* cold white: about 6500 K */
void     fxv_white_init(fxv_white_t *wc, uint32_t k_w, uint32_t k_ww, uint32_t k_cw);
/* working pixels -> wire bytes: gamma 2.8 at 16 bits (interpolated at fractional inputs), the limiter's brightness,
 * white mapping (q > 0: the calibrated emitters; q = 0: the legacy auto-white), dither / round as fx_pack. Pixels with
 * q = 0 whose channels are multiples of 256 give exactly fx_pack's bytes. Returns the wire byte sum. */
uint32_t fxv_pack16(const fxv_px_t *in, uint8_t *wire, uint32_t n, uint8_t bri, const fx_pack_cfg_t *cfg,
                    const fxv_white_t *wc);
uint32_t fxv_gamma16f(uint32_t v16);                         /* gamma 2.8 of a working value 0..65280 -> 0..65280 */

#endif
