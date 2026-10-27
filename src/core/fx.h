/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Local pixel effects on the Diff-Power-UR-Digital-M (BOARD=3): the effect engine (docs/ur_digital_m-effects.md).
 *
 * A pure C module: integer math only (RV32IMAC, no FPU), no division per pixel, no globals that change, no hardware.
 * The same files build for the host tests (MSVC / any C99 compiler) and for the receiver.
 *
 * Layers (each usable alone):
 *   1. fx_ctx_t + fx_begin / fx_frame / fx_render: ONE effect on ONE run of pixels (the prototype's API shape).
 *      An effect renders pixels [from, to) of its run for the frame at effect time t into 8-bit RGB. Every effect is a
 *      closed-form function of (parameters, time, pixel index): any chunk in any order gives the same picture, so a
 *      reversed or mirrored run, and the "race the beam" renderer, need nothing special.
 *   2. fx_engine_t: up to FX_MAX_SEG segments (start / length, reverse, mirror), the transitions (brightness and
 *      colour ramps, the effect crossfade) done locally, the compositor.
 *   3. fx_pack: brightness (the limiter's), gamma 2.8, colour order incl. RGBW / WRGB / WS2805, auto-white, channel sum.
 *
 * Time: fx_frame / fx_engine_frame take HOST milliseconds (the receiver's synced clock, wrapping uint32). An effect's
 * own time is host time minus its anchor (the PIXEL_FX start), so receivers given the same slot render the same picture.
 * Speed 0 freezes an effect; a speed change keeps the picture continuous (the effect time is the integral of the rate).
 *
 * Licences: no WLED code. Algorithms from FastLED's examples (MIT: Pride2015, ColorWaves, TwinkleFOX, Pacifica, Fire2012,
 * DemoReel100) are re-implemented in integer code with attribution (NOTICE.md); the rest is written from the described
 * behaviour. */
#ifndef FX_H
#define FX_H

#include <stdint.h>

/* Set by the BOARD=3 build when trading Pacifica for calibration flash. IDs are
 * never renumbered. The capability map omits it; host/catalogue builds retain it. */
#ifndef FX_NO_PACIFICA
#define FX_NO_PACIFICA 0
#endif

typedef struct { uint8_t r, g, b; } rgb_t;

/* ---------------------------------------------------------------------------------- the catalogue (ids never reused) */
enum {                                 /* effect ids: 1-127 official; core/fx_catalogue.json lists them */
    FX_STREAM       = 0,               /* no local effect (an inactive segment: black here; the hub's stream passes) */
    FX_SOLID        = 1,
    FX_BREATHE      = 2,
    FX_COLORLOOP    = 3,
    FX_RAINBOW      = 4,
    FX_PALETTE      = 5,
    FX_COLORWAVES   = 6,
    FX_PRIDE        = 7,
    FX_SINELON      = 8,
    FX_TWINKLEFOX   = 9,
    FX_COLORTWINKLES = 10,
    FX_CANDLE       = 11,
    FX_CANDLE_MULTI = 12,
    FX_AURORA       = 13,
    FX_BLENDS       = 14,
    FX_FLOW         = 15,
    FX_FIRE         = 16,
    FX_GRADIENT     = 17,
    FX_PACIFICA     = 18,
    FX_SUNRISE      = 19,
    FX_SUNSET       = 20,
    FX_GLITTER      = 21,
    FX_METEOR       = 22,
    FX_FAIRY        = 23,
    FX_PLASMA       = 24,
    FX_NOISE        = 25,
    FX_RUNNING      = 26,
    FX_RIPPLES      = 27,
    FX_WIPE         = 28,
    FX_STRIPES      = 29,
    FX_BULBS        = 30,
    FX_LAVA_LAMP    = 31,
    FX_MOSAIC_DRIFT = 32,
    FX_SILK_WEAVE   = 33,
    FX_CANOPY_SHADE = 34,
    FX_OPAL_FACETS  = 35,
    FX_QUIET_LANDINGS = 36,
    FX_LAST         = 36,
    FX_COUNT        = FX_LAST + 1
};

enum {                                 /* palette ids: 0-127 built-in (16 entries, interpolated); catalogue.json */
    PAL_RAINBOW = 0, PAL_ANALOGOUS = 1, PAL_APRIL_NIGHT = 2, PAL_C9_NEW = 3, PAL_HULT = 4, PAL_MAGRED = 5,
    PAL_ORANGE_TEAL = 6, PAL_RAINBOW_BANDS = 7, PAL_RETRO_CLOWN = 8, PAL_YELMAG = 9,
    PAL_PARTY = 10, PAL_OCEAN = 11, PAL_FOREST = 12, PAL_LAVA = 13, PAL_HEAT = 14, PAL_CLOUD = 15,
    PAL_SUNSET = 16, PAL_AURORA = 17, PAL_FAIRY = 18, PAL_SUNRISE = 19,
    PAL_CANDLELIGHT = 20, PAL_WARM_WHITE = 21, PAL_DAYLIGHT = 22, PAL_CHRISTMAS = 23, PAL_WINTER = 24,
    PAL_HALLOWEEN = 25, PAL_AUTUMN = 26, PAL_PASTEL = 27, PAL_EMBERS = 28, PAL_ROSE = 29, PAL_MOONLIGHT = 30,
    PAL_LAST = 30,
    PAL_COUNT = PAL_LAST + 1,
    PAL_COLORS = 0xFE,                 /* c1 -> c2 -> c3 -> c1 */
    PAL_DEFAULT = 0xFF                 /* Effect default: the effect's default palette, or its own colours (pal_own) */
};

#define FX_F_REVERSE   0x01u           /* PIXEL_FX flags byte (STUDY section 3.1): the bits the engine uses */
#define FX_F_MIRROR    0x02u

#ifndef FX_MAX_SEG
#define FX_MAX_SEG     4u              /* segments (PIXEL_FX LIGHT 0-3); a build may set fewer */
#endif
#ifndef FX_MAX_XFADE
#define FX_MAX_XFADE   2u              /* effect crossfades running at once (the rest switch hard) */
#endif
#define FX_CHUNK       32u             /* the most pixels one fx_engine_render call may be asked for */
#define FX_FIRE_CELLS  256u            /* Fire keeps at most this many heat cells per segment (longer: each covers several pixels) */
#define FX_FIRE_SMOOTH 2u              /* fixed smoothing slots: segment indices 0 and 1 */

typedef struct {                       /* = the PIXEL_FX slot's fields (STUDY section 3.1) + the virtual strip */
    uint8_t  id, speed, intensity, palette;
    rgb_t    c[3];
    uint8_t  bri, flags, custom1, custom2;
    uint16_t seg_start, seg_len;       /* the engine: pixels; len 0 = to the end of the strip */
    uint16_t trans_ms;                 /* the engine: ramp / crossfade time */
    uint16_t vstart, vlen;             /* the run's first pixel in a virtual strip spanning ports / receivers (CONFIG 71/72) */
    uint32_t seed;                     /* random effects: the slot's seed (the host sends custom2 + the segment) */
} fx_params_t;

/* ---------------------------------------------------------------------------------- layer 1: one effect on one run */
typedef struct {
    uint8_t  id, speed, intensity, palette;   /* (id .. c[]: the slot's first 13 bytes, fx_params_t: fx_begin copies them as one) */
    rgb_t    c[3];                     /* the CURRENT colours (the engine ramps them) */
    uint8_t  et_frac;                  /* (here: flags lands on fx_params_t's even offset, so fx_begin's copy stays aligned) */
    uint8_t  flags, custom1, custom2;  /* (the slot's order too) */
    uint8_t  pal_id;                   /* the resolved palette (fx_pal_resolve): pal[] below is built from it */
    uint8_t  pal_own;                  /* 1: draw the designed look (Effect default on an effect with its own colours), 0: use pal[] */
    uint16_t n, vlen;                  /* pixels in the run, the virtual length (p->vlen, or n) */
    uint32_t vbase;                    /* the virtual position of the run's pixel 0 */
    uint32_t seed;
    uint32_t t;                        /* effect time: ms since the anchor */
    uint32_t t_last;                   /* t of the previous fx_frame */
    uint32_t et;                       /* the speed-integrated time (ms at the nominal rate) the effects use */
    uint32_t u[5];                     /* per-effect numbers worked out once a frame (Fire: [0] elapsed, [1] random state) */
    uint8_t  *state;                   /* fx_state_bytes(id, n) bytes, zeroed by fx_begin; 0 when the effect has none */
    uint8_t  *fire_prev;               /* optional extra Fire heat array; caller zeroes it before the first frame */
    rgb_t    pal[16];                  /* the active palette */
} fx_ctx_t;

uint32_t fx_state_bytes(uint8_t id, uint32_t n);         /* bytes of state an effect needs for a run of n pixels (Fire: min(n, 256)) */
uint8_t  fx_default_palette(uint8_t id);                 /* the palette PAL_DEFAULT stands for (PAL_DEFAULT: the effect's own colours) */
/* the palette a slot's palette byte resolves to (fx_begin and the engine's same-effect check): PAL_DEFAULT gives the
 * effect's default, which is PAL_DEFAULT again (pal_own) for an effect whose default is its own colours */
static inline uint8_t fx_pal_resolve(uint8_t id, uint8_t palette)
{
    return palette == PAL_DEFAULT ? fx_default_palette(id) : palette;
}
void     fx_begin(fx_ctx_t *x, const fx_params_t *p, uint16_t n, uint8_t *state);
void     fx_frame(fx_ctx_t *x, uint32_t t_ms);           /* once per frame, before its pixels (t_ms: since the anchor) */
/* Cooperative equivalent: begin once, then step until 0 BEFORE rendering or beginning another frame.
 * Each step cools or diffuses at most 64 Fire cells. Heat/time stay frozen throughout rendering. */
void     fx_frame_begin(fx_ctx_t *x, uint32_t t_ms);
uint32_t fx_frame_step(fx_ctx_t *x);                    /* 1: performed a slice; 0: ready */
void     fx_render(fx_ctx_t *x, rgb_t *out, uint32_t from, uint32_t to);   /* pixels [from, to) -> out[0 .. to - from) */
#if defined(FX_V407) && FX_V407 && !FX_NO_PACIFICA
/* Pacifica's pixels in resumable pieces (the V407 layer renders it ahead in bounded slices, fx_v407/fxv_legacy.c):
 * begin once a frame, then run until 0; each run call steps at most 16 pixels of its block anchor or renders at most
 * max pixels, into out[0 .. n). The same pixels as fx_render. */
typedef struct { uint32_t w[30]; } fx_pac_run_t;
void     fx_pacifica_begin(const fx_ctx_t *x, fx_pac_run_t *r);
uint32_t fx_pacifica_run(const fx_ctx_t *x, fx_pac_run_t *r, rgb_t *out, uint32_t max);
#endif
void     fx_set_colors(fx_ctx_t *x, const rgb_t c[3]);   /* new current colours (rebuilds a c1-c2-c3 palette) */
void     fx_set_speed(fx_ctx_t *x, uint8_t speed);       /* call after fx_frame(x, t) at the change time */

/* ---------------------------------------------------------------------------------- layer 2: segments + transitions */
typedef struct {
    fx_ctx_t cur;
    rgb_t    tc[3];                    /* the last slot's colours, brightness, virtual strip: the targets of the ramps */
    uint8_t  tbri;
    uint16_t tvstart, tvlen;
    uint16_t start, len;               /* the segment's pixels on the strip */
    uint16_t n_eff;                    /* pixels the effect renders (mirror: half) */
    uint16_t st_off;                   /* offset of its state bytes in the pool */
    uint8_t  flags, active;            /* fill the gap before the word-aligned clocks */
    uint32_t anchor;                   /* host ms of the effect's t = 0 (kept when a renewal re-sends the same slot) */
    uint32_t tr_t0;                    /* host ms the ramps started */
    uint32_t tr_inv;                   /* 2^24 / trans_ms (0: no ramp running) */
    uint16_t tr_dur;                   /* trans_ms */
    rgb_t    c0[3];                    /* colours at the ramp start */
    uint8_t  bri0, bri;                /* brightness at the ramp start / now */
    uint8_t  started;                  /* started: the anchor has been reached (a future anchor holds t at 0 until then) */
    int8_t   xf;                       /* index of the crossfade slot, or -1 */
} fx_seg_t;

typedef struct {
    fx_ctx_t prev;                     /* the effect fading out */
    uint32_t t0, inv;                  /* host ms it started, 2^24 / duration */
    uint32_t dur;
    uint32_t anchor;                   /* the old effect's anchor */
    uint8_t  started;
    uint8_t  w;                        /* weight of the NEW effect now, 0..255 (255: done) */
    uint8_t  used;                     /* bit 0 live, bit 1 has a queued replacement */
    uint8_t  w0;                       /* weight at a shortened fade's new origin */
    fx_params_t next;                  /* latest replacement; used bit 1 says it is queued */
    uint32_t next_anchor;
} fx_xfade_t;

typedef struct {
    fx_seg_t   seg[FX_MAX_SEG];
    fx_xfade_t xf[FX_MAX_XFADE];
    uint8_t   *pool;                   /* state bytes for the stateful effects (fx_state_bytes) */
    uint32_t   pool_bytes;
    uint32_t   now;                    /* host ms of the last fx_engine_frame */
    uint16_t   npx;                    /* pixels on the strip */
    uint8_t    deferred;               /* caller drains fx_engine_step after each set/frame_seg/dequeue, before rendering */
} fx_engine_t;

void fx_engine_init(fx_engine_t *e, uint16_t npx, uint8_t *pool, uint32_t pool_bytes);
/* PIXEL_FX for segment s, taking effect at host time now_ms; the effect's t = 0 is at anchor_ms (the slot's start).
 * A RENEWAL (the same effect with every parameter the same, only the anchor new: the hub re-sends START before its signed
 * 32-bit ms runs out, ~every 20 days) changes nothing: the running phase, raw time and picture continue. To restart a
 * time-driven effect (Sunrise / Sunset) send it with any other change (e.g. the seed) or stream (id 0) first.
 * Same effect id: the colours / brightness ramp over trans_ms, speed / intensity / custom change at once (the picture
 * stays continuous). A new effect id (or palette): crossfade over trans_ms (a hard cut when no slot is free).
 * Replacing a live crossfade queues the latest target and finishes the running fade within 250 ms, without changing
 * its current weight. STREAM cancels both. A Fire palette crossfade shares its heat and advances it only once. */
void fx_engine_set(fx_engine_t *e, uint32_t s, const fx_params_t *p, uint32_t now_ms, uint32_t anchor_ms);
void fx_engine_frame(fx_engine_t *e, uint32_t now_ms);   /* once per frame, before its pixels */
void fx_engine_frame_seg(fx_engine_t *e, uint32_t s, uint32_t now_ms); /* same preparation, one segment per main-loop step */
uint32_t fx_engine_dequeue(fx_engine_t *e, uint32_t s, uint32_t now_ms); /* promote a finished fade separately from frame prep */
uint32_t fx_engine_step(fx_engine_t *e);              /* at most one pending Fire slice, including fading-out heat */
/* strip pixels [from, to) (to - from <= FX_CHUNK) composited from all segments (black where none), the segments'
 * brightness applied -> out[0 .. to - from) */
void fx_engine_render(fx_engine_t *e, rgb_t *out, uint32_t from, uint32_t to);
/* 1 when every segment's brightness is 0 and no ramp is running: the strip is dark (flash writes are allowed) */
uint32_t fx_engine_dark(const fx_engine_t *e);
/* Render quota: Fire alone 16, Fire with other layers 2, Pacifica 4, otherwise FX_CHUNK.
 * quota < FX_CHUNK also bounds total pixels across live layers, except Pacifica. */
uint32_t fx_engine_chunk(const fx_engine_t *e, uint32_t quota);

/* ---------------------------------------------------------------------------------- layer 3: the packer */
/* auto-white (the CONFIG 70 / PIXEL_FX numbering): W = min(r,g,b); 1 off, 2 accurate (taken out of RGB), 3 bright (RGB kept); 0 = off too */
enum { FX_W_NONE = 0, FX_W_OFF = 1, FX_W_ACCURATE = 2, FX_W_BRIGHT = 3 };
/* the output step (CONFIG 76, "pixel smoothing"): FX_SM_8BIT is the 8-bit path (brightness, gamma 2.8 and auto-white at 8 bits); the other two work at 16 bits
 * (gamma 2.8 from a 256-entry table, brightness, auto-white) and take the result to the wire's 8 bits: FX_SM_ROUND rounds, FX_SM_DITHER adds an ordered,
 * stateless threshold (frame x pixel) first, so the average over 256 frames is the 16-bit value exactly */
enum { FX_SM_8BIT = 0, FX_SM_ROUND = 1, FX_SM_DITHER = 2 };
typedef struct {
    uint8_t gamma;                     /* 1: gamma 2.8 after the brightness, 0 = none */
    uint8_t bpp;                       /* wire bytes per pixel: 3 RGB, 4 RGBW (WS2814, SK6812), 5 WS2805 */
    uint8_t ord[5];                    /* wire byte k = source channel ord[k]; sources R 0, G 1, B 2, W (WW) 3, CW 4 */
    uint8_t white_mode;                /* FX_W_*: only for bpp 4 / 5 */
    uint8_t white_split;               /* bpp 5: the cool white's (CW, channel 4) share: 0 = all warm (WW, channel 3) ... 255 = all cool */
    uint8_t smooth;                    /* FX_SM_* */
    uint8_t phase;                     /* FX_SM_DITHER: the frame's phase, (the frame index on the host grid) x 97 mod 256 (fx_dither_phase) */
    uint16_t px0;                      /* FX_SM_DITHER: the strip pixel index of in[0] (the caller sets it before each call) */
    uint8_t dmin;                      /* FX_SM_DITHER: a channel whose wire level (the 16-bit value / 256) is below this is rounded, not dithered (0: all dither) */
} fx_pack_cfg_t;
#define fx_dither_phase(frame) ((uint8_t)((frame) * 97u))   /* 97 / 256 ~ 0.379: the golden ratio's fraction, so any run of frames is spread evenly */
/* rgb -> the wire bytes of n pixels: bri (0-255, the limiter's), gamma, auto-white, order. Returns the sum of the
 * wire bytes written (the current estimate). The order lists for the common chips: GRB {1,0,2}, RGB {0,1,2},
 * SK6812 GRBW {1,0,2,3}, WS2814 RGBW {0,1,2,3} / WRGB {3,0,1,2}, WS2805 RGBWC {0,1,2,3,4} */
uint32_t fx_pack(const rgb_t *in, uint8_t *wire, uint32_t n, uint8_t bri, const fx_pack_cfg_t *cfg);
/* the current limiter: the brightness 0-255 that brings a frame whose wire bytes summed to `sum` (packed at brightness
 * `bri`) to about `max_sum`. The sum goes with bri^2.8 through the gamma table, so bri is scaled by the 2.8th root of
 * max_sum / sum (a 16-step table; one division per frame). A frame at or under max_sum keeps bri */
uint8_t  fx_limit_bri(uint8_t bri, uint32_t sum, uint32_t max_sum);
uint32_t fx_gamma(uint32_t v);               /* gamma 2.8 on 0..255: round(255 * (v / 255) ^ 2.8), the 8-bit table (the 16-bit table, rounded) */

#endif
