/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* B7: Fireworks first, then its Countdown finale, Collider, Confetti Cannon,
 * Pinball Relay and Packet Switch. JS: fx/v407/b7_bursts.js.
 *
 * Particle positions are fractions of the virtual strip (65536 = its far end).
 * The shared slab remains the only particle store. The arena contains a 512-byte
 * control block and an 8-byte/pixel working image, composed in bounded slices.
 * Simulation is fixed at 20 effect ms; fractional-tick prediction gives smooth
 * motion at 100 fps. Rendering only copies the completed image, in any order.
 * B7-local candidate for B0: normalised-coordinate, clipped local splats. */
#include "fxv_gen.h"
#include "../math32.h"

typedef struct {
    uint32_t ms, last_t, elapsed, cycle, phase, mode, previous;
    uint32_t left, total, cursor, kind, colour, serial, born;
    int32_t origin;
    uint32_t bpos[10], btime[10], bpal[10], running;
} b7_state_t;
typedef char b7_control_fits[sizeof(b7_state_t) <= 512u ? 1 : -1];

static b7_state_t *b7_state(const fxv_ctx_t *x) { return (b7_state_t *)x->arena; }
static fxv_px_t *b7_image(const fxv_ctx_t *x) { return (fxv_px_t *)(x->arena + 512u); }
static fxv_px_t *b7_small_image(const fxv_ctx_t *x) { return (fxv_px_t *)(x->arena + 512u + 8u * x->n); }
static uint32_t b7_u32(uint32_t a) { return a; }
static int32_t b7_sar(int32_t a, uint32_t b) { return a >> b; }
static uint32_t b7_min(uint32_t a, uint32_t b) { return a < b ? a : b; }
static uint32_t b7_bytes(uint32_t n, uint32_t tier) { (void)tier; return 512u + (n <= 3u ? 16u : 8u) * n; }

/* Map a normalised coordinate to the local image, before calling the shared
 * splat. Off-window objects never overflow signed Q16 pixel coordinates; the
 * window test compares distances, so it cannot wrap at the virtual strip's end. */
static void b7_splat(const fxv_ctx_t *x, int32_t pos, uint32_t radius, fxv_px_t colour)
{
    uint32_t vp, base;
    if (pos < 0 || pos > 65536 || x->n == 0u) return;
    vp = (uint32_t)pos * x->vlen;
    base = x->vbase * 65536u;
    if (vp < base ? base - vp > radius : vp - base > x->n * 65536u + radius) return;
    fxv_splat(b7_image(x), 0u, x->n, (int32_t)(vp - base), radius, colour);
}

static uint32_t b7_nodes(const fxv_ctx_t *x)
{
    const uint32_t want = x->id == 103u ? 2u + x->custom1 * 8u / 255u : 2u + x->custom1 * 6u / 255u;
    return b7_min(want, 2u + x->vlen / 6u);
}

/* The labelled traversal endpoints, with a smooth accelerating control curve.
 * Cancel the common speed-clock rate once per frame, never per particle/pixel. */
static uint32_t b7_velocity(const fxv_ctx_t *x)
{
    const uint32_t slow = x->id == 103u ? 15000u : 12000u;
    const uint32_t fast = x->id == 103u ? 1500u : 1000u;
    const uint32_t curve = x->speed * (510u - x->speed);
    const uint32_t duration = slow - m32_muldiv(curve, slow - fast, 65025u);
    const uint32_t rate = 8u + ((x->speed * 15u) >> 3);
    return m32_muldiv(1310720u, 256u, duration * rate);
}

static void b7_init(fxv_ctx_t *x)
{
    b7_state_t *s = b7_state(x);
    s->last_t = x->t;
    s->cycle = 0xFFFFFFFFu;
    s->previous = 0xFFFFFFFFu;
    s->running = x->speed != 0u;
}

static void b7_changed(fxv_ctx_t *x)
{
    b7_state_t *s = b7_state(x);
    if (x->t < s->last_t) s->elapsed = 0u; /* an explicit new anchor */
    else if (s->running) s->elapsed = b7_u32(s->elapsed + x->t - s->last_t);
    s->last_t = x->t;
    s->running = x->speed != 0u;
}

static void b7_event(fxv_ctx_t *x, uint32_t kind, int32_t origin, uint32_t count, uint32_t colour)
{
    b7_state_t *s = b7_state(x);
    if (s->left != 0u || x->pn == 0u || x->n == 0u) return;
    s->kind = kind; s->origin = origin; s->colour = colour;
    s->left = b7_min(count, b7_min(x->pn, 2u + x->vlen * 2u));
    s->total = s->left; s->cursor = 0u; s->serial++;
}

/* Countdown uses elapsed running time, not the speed-integrated phase: its
 * speed slider labels a duration. Zero speed pauses it, like the other effects.
 * Parameter changes do not reset its elapsed time. No wall-clock claim. */
static void b7_advance(fxv_ctx_t *x)
{
    b7_state_t *s = b7_state(x);
    uint32_t dt = x->t >= s->last_t ? x->t - s->last_t : 0u, duration, finale, elapsed;
    b7_changed(x);
    s->mode = 1u;
    if (x->id == 113u) {
        duration = 10000u + m32_muldiv(x->speed * x->speed, 590000u, 65025u);
        finale = 10000u + x->intensity * 110000u / 255u;
        elapsed = s->elapsed;
        if (x->custom2 >= 64u && x->custom2 < 192u) elapsed %= duration + finale + 10000u;
        s->mode = elapsed < duration ? 0u : elapsed < duration + finale ? 1u : 2u;
        s->phase = elapsed < duration ? elapsed : elapsed - duration;
        if (s->mode == 0u) {
            x->u[2] = m32_muldiv(duration - elapsed, 65536u, duration);
            x->u[3] = fxv_env(elapsed % 1000u, 120u, 0u, 300u);
        } else if (s->mode == 1u && s->previous != 1u) {
            b7_event(x, 2u, 32768, 12u + x->intensity, 40u);
        }
        s->previous = s->mode;
    }
    x->u[0] = 65536u / x->vlen;
    x->u[1] = m32_muldiv(x->tick_acc * 256u + x->et_frac, 65536u, 5120u);
    x->u[5] = x->speed == 0u ? 0u : dt >= 180u ? 65535u : dt * 65535u / 180u;
    if (x->id == 103u || x->id == 106u) x->u[4] = b7_velocity(x);
    x->job[0] = 0u; x->job[1] = 0u; x->job[2] = 0u;
}

static void b7_tick(fxv_ctx_t *x)
{
    b7_state_t *s = b7_state(x);
    uint32_t cycle, phase, r, j, nodes;
    s->ms = b7_u32(s->ms + 20u);
    if (x->id == 94u || x->id == 113u) {
        if (s->mode != 1u) return;
        cycle = x->tick / 120u;
        if (cycle != s->cycle) {
            s->cycle = cycle;
            b7_event(x, 1u, 600, 1u, fxv_rand(x->seed, cycle, 1u) >> 24);
        }
    } else if (x->id == 95u) {
        cycle = x->tick / 160u; phase = x->tick % 160u;
        if (cycle != s->cycle) {
            s->cycle = cycle;
            r = fxv_rand(x->seed, cycle, 2u);
            s->origin = 32768 + b7_sar(((int32_t)(r & 65535u) - 32768) * x->custom1, 10u);
        }
        if (phase == 80u) {
            s->born = x->tick;
            b7_event(x, 2u, s->origin, 4u + x->intensity * 124u / 255u, 0u);
        }
    } else if (x->id == 102u) {
        cycle = x->tick / 180u;
        if (cycle != s->cycle) {
            s->cycle = cycle;
            j = x->custom1 >= 192u || (x->custom1 >= 64u && (cycle & 1u));
            b7_event(x, 3u, j ? 65000 : 536, 4u + x->intensity * 72u / 255u, j);
        }
    } else if (x->id == 103u) {
        nodes = b7_nodes(x);
        for (j = 0; j < nodes; j++) {
            s->bpos[j] = (j + 1u) * 65536u / (nodes + 1u)
                + b7_sar(fxv_sin16(s->ms * 2u + j * 17011u) * 900, 15u);
        }
        /* Replace only missing balls. The desired population never exceeds the
         * current slab quota; a quota increase arrives as zeroed records. */
        s->cursor = 0u;
    } else if (x->id == 106u) {
        nodes = b7_nodes(x);
        for (j = 0; j < nodes; j++) { s->bpos[j] = (j + 1u) * 65536u / (nodes + 1u); s->btime[j] = 0u; }
        cycle = x->tick / (70u - x->intensity / 5u);
        if (cycle != s->cycle) {
            s->cycle = cycle;
            b7_event(x, 5u, 0, 1u, fxv_rand(x->seed, cycle, 5u) >> 24);
        }
    }
}

static void b7_spawn(fxv_ctx_t *x, uint32_t i)
{
    b7_state_t *s = b7_state(x);
    fxv_particle_t *p = &x->p[i];
    const uint32_t r = fxv_rand(x->seed, s->serial, s->total - s->left + 123u);
    p->x = s->origin; p->age = 0u; p->phase = 0u; p->flags = 0u;
    p->pal = (uint8_t)(s->colour + (r >> 24));
    p->kind = (uint8_t)s->kind;
    if (s->kind == 1u) {
        p->v = 1100 + (int32_t)(r & 255u); p->life = 36u + (uint16_t)((r >> 8) & 15u);
        p->pal = (uint8_t)s->colour;
    } else if (s->kind == 2u) {
        p->v = (int32_t)(r & 2047u) - 1024; p->life = 55u + (uint16_t)((r >> 12) & 63u);
    } else if (s->kind == 3u) {
        p->v = 330 + (int32_t)(r & 1023u); if (s->colour) p->v = -p->v;
        p->life = 600u; p->phase = (uint8_t)(r >> 16);
    } else {
        p->v = 330; p->life = 1400u; p->pal = (uint8_t)s->colour;
        if (s->kind == 6u) { p->flags = (uint8_t)(s->colour >> 8); p->phase = (uint8_t)(8u + s->left * 12u); }
    }
    s->left--;
}

static void b7_ball(fxv_ctx_t *x, uint32_t i)
{
    b7_state_t *s = b7_state(x);
    fxv_particle_t *p = &x->p[i];
    const uint32_t target = b7_min(x->pn, b7_min(1u + x->vlen / 9u, 2u + x->intensity * 22u / 255u));
    uint32_t j, r, nodes = b7_nodes(x), magnitude = x->u[4];
    int32_t old;
    if (i >= target) { p->life = 0u; return; }
    if (!p->life) {
        r = fxv_rand(x->seed, i, 103u); p->x = 1000 + (int32_t)(r % 63000u);
        p->v = (i & 1u) ? -(int32_t)magnitude : (int32_t)magnitude;
        p->life = 65535u; p->kind = 4u; p->pal = (uint8_t)(r >> 24); p->phase = 0u;
    }
    if (p->v < 0) p->v += b7_sar(-(int32_t)magnitude - p->v, 4u);
    else p->v += b7_sar((int32_t)magnitude - p->v, 4u);
    old = p->x; p->x += p->v;
    if (p->phase) p->phase--;
    if (p->x < 500) { p->x = 1000 - p->x; p->v = (int32_t)magnitude; }
    if (p->x > 65036) { p->x = 130072 - p->x; p->v = -(int32_t)magnitude; }
    if (p->phase) return;
    for (j = 0; j < nodes; j++) {
        const int32_t bp = (int32_t)s->bpos[j];
        if ((old < bp && p->x >= bp) || (old > bp && p->x <= bp)) {
            s->bpal[j] = p->pal; s->btime[j] = x->tick;
            magnitude = (magnitude * (160u + (x->custom2 >> 1))) >> 8;
            p->v = old < bp ? -(int32_t)magnitude : (int32_t)magnitude;
            p->x = bp - b7_sar((p->x - bp) * (int32_t)(160u + (x->custom2 >> 1)), 8u);
            p->phase = 1u; p->pal = (uint8_t)(p->pal + 29u); break;
        }
    }
}

static void b7_packet(fxv_ctx_t *x, uint32_t i)
{
    b7_state_t *s = b7_state(x);
    fxv_particle_t *p = &x->p[i];
    uint32_t j = p->flags & 127u, nodes = b7_nodes(x), delay;
    int32_t bp;
    p->v = (int32_t)x->u[4];
    /* A changed junction count retires a now-obsolete queued destination. */
    if (j >= nodes && (p->flags & 128u)) { p->phase = 0u; p->flags = (uint8_t)nodes; }
    if (p->phase) {
        p->phase--;
        if (!(p->flags & 128u)) return; /* stagger the split pieces leaving a junction */
        delay = b7_min(s->btime[j]++, 6u);
        p->x = (int32_t)s->bpos[j] - (int32_t)(delay * 1400u);
        if (p->phase) return;
        p->flags = (uint8_t)(j + 1u); p->x = (int32_t)s->bpos[j] + 20;
        if ((j & 1u) && p->kind == 6u) { p->life = 0u; return; } /* rejoin the parent parcel */
        if (!(j & 1u) && p->kind == 5u && x->custom2 >= 64u) {
            b7_event(x, 6u, p->x - 500, 1u + x->custom2 / 96u, p->pal + ((j + 1u) << 8));
        }
        return;
    }
    p->x += p->v;
    if (p->x > 66536) { p->life = 0u; return; }
    if (j >= nodes) return;
    bp = (int32_t)s->bpos[j];
    if (p->x >= bp) { p->x = bp; p->flags = (uint8_t)(j | 128u); p->phase = 12u + (uint8_t)(s->btime[j]++ * 10u); }
}

static void b7_step_particle(fxv_ctx_t *x, uint32_t i)
{
    b7_state_t *s = b7_state(x);
    fxv_particle_t *p = &x->p[i];
    int32_t accel;
    if (x->id == 103u) { b7_ball(x, i); return; }
    if (!p->life) return;
    if (x->id == 113u && s->mode != 1u) { p->life = 0u; return; }
    if (x->id == 106u) { b7_packet(x, i); return; }
    if (p->kind == 7u) { if (++p->age >= p->life) p->life = 0u; return; }
    accel = p->kind == 1u ? -12 : x->id == 95u ? 0 : -(int32_t)(1u + x->custom1 / 14u);
    if (x->id == 102u) accel = 0;
    if (p->kind == 1u && p->age + 1u >= p->life) {
        b7_event(x, 2u, p->x, 12u + x->intensity * 388u / 255u, p->pal);
        s->born = x->tick; p->life = 0u; return;
    }
    fxv_particle_step(p, 1u, accel, x->id == 102u ? 1500u : p->kind == 1u ? 0u : 320u);
    if (x->id == 102u && p->v > -25 && p->v < 25) {
        p->kind = 7u; p->v = 0; p->age = 0u; p->life = 60u + x->custom2 * 3u;
    }
    if (p->x < 0 || p->x > 65536) p->life = 0u;
}

/* Collider with a palette: every collision cycle takes a new pair of palette colours half a palette apart (the left
 * pulse at the pair's base, the right one 128 further, 8-bit palette steps; the base moves 3/8 of the palette per
 * collision, so eight collisions visit the whole palette). Sparks take the colours between the pair (their own
 * random pal), the shockwave the colour in the middle; the motion, the sparks' white flash and the black stay. */
static uint32_t b7_pair(uint32_t cycle) { return (cycle * 96u) & 255u; }

static fxv_px_t b7_colour(const fxv_ctx_t *x, uint32_t pal)
{
    if (x->id == 95u) {
        if (x->pal_own) return fxv_blend16(fxv_rgb16(x->c[0]), fxv_rgb16(x->c[1]), 32768u);
        /* the cycle a spark was born in: born is the last burst's tick, sparks die before the next burst */
        return fxv_pal(x, ((b7_pair(b7_state(x)->born / 160u) + (pal >> 1)) & 255u) << 8);
    }
    return fxv_pal(x, pal << 8);
}

static void b7_paint(fxv_ctx_t *x, uint32_t i)
{
    const fxv_particle_t *p = &x->p[i];
    const b7_state_t *s = b7_state(x);
    uint32_t env, radius, age, life, flutter, j;
    int32_t pos, velocity, bp;
    fxv_px_t c;
    if (!p->life || (x->id == 113u && s->mode != 1u)) return;
    pos = p->x;
    velocity = p->v;
    if (p->kind == 4u) velocity += b7_sar((p->v < 0 ? -(int32_t)x->u[4] : (int32_t)x->u[4]) - p->v, 4u);
    if (!(p->flags & 128u) && p->kind != 7u && !(x->id == 106u && p->phase)) pos += b7_sar(velocity * (int32_t)x->u[1], 16u);
    /* Predict the swept contact too. At the slow end the speed-clock tick is
     * sparse: interpolating through a bumper and snapping back would stutter. */
    if (p->kind == 4u) {
        if (pos < 500) pos = 1000 - pos;
        if (pos > 65036) pos = 130072 - pos;
        for (j = 0; j < b7_nodes(x); j++) {
            bp = (int32_t)s->bpos[j];
            if ((p->x < bp && pos >= bp) || (p->x > bp && pos <= bp)) {
                pos = bp - b7_sar((pos - bp) * (int32_t)(160u + (x->custom2 >> 1)), 8u);
                break;
            }
        }
    } else if (x->id == 106u && !(p->flags & 128u) && (p->flags & 127u) < b7_nodes(x)) {
        bp = (int32_t)s->bpos[p->flags & 127u];
        if (pos > bp) pos = bp;
    }
    age = p->age * 20u + x->tick_acc;
    life = p->life * 20u;
    c = b7_colour(x, p->pal);
    radius = 80000u;
    if (p->kind == 4u || p->kind == 5u || p->kind == 6u) {
        env = 56000u; c.q = 100u; c.k = 128u;
        if (p->kind == 5u) radius = 130000u;
        if (p->kind == 6u) radius = 70000u;
    } else {
        env = fxv_env(age, 120u, p->kind == 1u ? 1000u : 0u, life > 120u ? life - 120u : 1u);
        if (p->kind == 2u) {
            flutter = (uint32_t)(fxv_sin16(s->ms * 41u + i * 9043u) + 32768);
            env = (env * (65535u - ((x->custom2 * flutter) >> 11))) >> 16;
            c = fxv_blend16(c, fxv_kelvin16(205u), p->age < 10u ? (10u - p->age) * 2600u : 0u);
        } else if (p->kind == 3u) {
            flutter = (uint32_t)(fxv_sin16(s->ms * 25u + p->phase * 251u) + 32768);
            env = 38000u + (flutter >> 2);
        } else if (p->kind == 7u) {
            env = 25000u - b7_min(25000u, m32_muldiv(age, 25000u, life));
        } else { c = fxv_kelvin16(150u); radius = 90000u; }
    }
    c = fxv_scale_px(c, env);
    b7_splat(x, pos, radius, c);
    /* A tapered local tail follows the trajectory, rather than retaining
     * frame-rate-dependent previous images. Settled pieces stay distinct. */
    if (p->kind == 1u || p->kind == 2u || p->kind == 4u) {
        c.q = 0u;
        b7_splat(x, pos - b7_sar(p->v, 3u), 70000u, fxv_scale_px(c, 22000u));
    }
}

static void b7_background(fxv_ctx_t *x, uint32_t i)
{
    const b7_state_t *s = b7_state(x);
    fxv_px_t c = fxv_rgb16(x->c[1]);
    uint32_t vp, head, fade, r;
    int32_t spark;
    if (x->id == 94u || x->id == 95u) c = fxv_rgb16((rgb_t){0, 0, 0});
    if (x->id == 113u) {
        c = fxv_rgb16((rgb_t){0, 0, 0});
        vp = (x->vbase + i) * x->u[0];
        if (s->mode == 0u && vp < x->u[2]) {
            fade = 50000u + ((x->u[3] * x->custom1) >> 10);
            c = fxv_scale_px(fxv_rgb16(x->c[0]), b7_min(fade, 65535u)); c.q = 220u; c.k = 70u;
        } else if (s->mode == 1u && s->phase < 300u) {
            c = fxv_scale_px(fxv_kelvin16(150u), fxv_env(s->phase, 120u, 60u, 120u));
        } else if (s->mode == 2u && x->custom2 < 192u) {
            r = fxv_rand(x->seed, x->vbase + i, 113u);
            spark = fxv_sin16(s->elapsed * 6u + (r & 65535u));
            head = spark > 0 ? (uint32_t)spark * 2u : 0u;
            head = (head * head) >> 16;
            head = (head * head) >> 16;
            c = fxv_scale_px(fxv_rgb16(x->c[1]), head >> 1); c.q = 220u; c.k = 55u;
        }
    }
    b7_image(x)[i] = c;
}

static void b7_decorate(fxv_ctx_t *x, uint32_t j)
{
    const b7_state_t *s = b7_state(x);
    uint32_t phase, env, nodes;
    int32_t pos;
    fxv_px_t c;
    if (x->id == 95u) {
        phase = x->tick % 160u;
        phase = phase * 20u + x->tick_acc;
        if (phase < 1600u) {
            pos = j == 0u ? (int32_t)m32_muldiv(phase, (uint32_t)s->origin, 1600u)
                : 65536 - (int32_t)m32_muldiv(phase, 65536u - (uint32_t)s->origin, 1600u);
            b7_splat(x, pos, 130000u, x->pal_own ? fxv_rgb16(x->c[j])
                : fxv_pal(x, ((b7_pair(x->tick / 160u) + j * 128u) & 255u) << 8));
        } else if (phase < 2600u && x->custom2) {
            pos = (int32_t)((phase - 1600u) * 25u);
            if (j == 0u) pos = -pos;
            env = fxv_env(phase - 1600u, 120u, 0u, 880u);
            c = fxv_scale_px(b7_colour(x, 128u), (env * x->custom2) >> 8); c.q = 180u; c.k = 128u;   /* the pair's middle */
            b7_splat(x, s->origin + pos, 100000u, c);
        }
    } else if (x->id == 103u || x->id == 106u) {
        nodes = b7_nodes(x);
        if (j >= nodes) return;
        c = fxv_scale_px(fxv_rgb16(x->c[2]), 14000u);
        if (x->id == 103u && s->btime[j]) {
            env = fxv_env((x->tick - s->btime[j]) * 20u + x->tick_acc, 120u, 80u, 1200u);
            c = fxv_blend16(c, fxv_pal(x, s->bpal[j] << 8), env);
        }
        c.q = 100u; c.k = 128u;
        b7_splat(x, (int32_t)s->bpos[j], 90000u, c);
    }
}

static uint32_t b7_slice(fxv_ctx_t *x, uint32_t quota)
{
    b7_state_t *s = b7_state(x);
    uint32_t k, end;
    /* Particle paint/simulation are deliberately one record per slice. A splat
     * touches at most five nearby pixels; no particle-by-strip raster loop. */
    if (quota == 0u) quota = 1u;
    if (x->job[0] == 0u) {
        if (fxv_tick_next(x)) { b7_tick(x); x->job[0] = 1u; }
        else x->job[0] = 3u;
        x->job[1] = 0u; return 1u;
    }
    if (x->job[0] == 1u) {
        if (x->job[1] < x->pn) { b7_step_particle(x, x->job[1]++); return 1u; }
        x->job[0] = 2u; return 1u;
    }
    if (x->job[0] == 2u) {
        end = b7_min(s->cursor + b7_min(quota, 4u), x->pn);
        for (k = s->cursor; k < end && s->left; k++) if (x->p[k].life == 0u) b7_spawn(x, k);
        s->cursor = end;
        if (!s->left || end == x->pn) { s->left = 0u; x->job[0] = 0u; }
        return 1u;
    }
    if (x->job[0] == 3u) {
        end = b7_min(x->job[1] + b7_min(quota, 8u), x->n);
        for (k = x->job[1]; k < end; k++) b7_background(x, k);
        x->job[1] = end;
        if (end == x->n) { x->job[0] = 4u; x->job[1] = 0u; }
        return 1u;
    }
    if (x->job[0] == 4u) {
        if (x->job[1] < x->pn) { b7_paint(x, x->job[1]++); return 1u; }
        x->job[0] = 5u; x->job[1] = 0u; return 1u;
    }
    if (x->job[1] < (x->id == 95u ? 2u : x->id == 103u || x->id == 106u ? b7_nodes(x) : 0u)) {
        b7_decorate(x, x->job[1]++); return 1u;
    }
    /* At 1-3 pixels a local collision covers the whole strip. A 180 ms
     * exponential envelope removes the subpixel spark shimmer there; the
     * Countdown's deliberate 300 ms finale flash is left exact. */
    if (x->n <= 3u && (x->id == 94u || x->id == 95u) && x->job[2] < x->n) {
        k = x->job[2]++;
        b7_image(x)[k] = fxv_blend16(b7_small_image(x)[k], b7_image(x)[k], x->u[5]);
        b7_small_image(x)[k] = b7_image(x)[k];
        return 1u;
    }
    return 0u;
}

static void b7_render(const fxv_ctx_t *x, uint32_t from, uint32_t count, fxv_px_t *out)
{
    const fxv_px_t *image = b7_image(x);
    uint32_t k;
    for (k = 0; k < count; k++) out[k] = image[from + k];
}

static uint32_t b7_digest(const fxv_ctx_t *x)
{
    const b7_state_t *s = b7_state(x);
    uint32_t i, h = fxv_hash2(s->ms, s->elapsed);
    h = fxv_hash2(h, s->serial);
    for (i = 0; i < x->pn; i++) {
        const fxv_particle_t *p = &x->p[i];
        if (!p->life) continue;
        h = fxv_hash2(h, (uint32_t)p->x); h = fxv_hash2(h, (uint32_t)p->v);
        h = fxv_hash2(h, p->age + ((uint32_t)p->life << 16));
        h = fxv_hash2(h, p->pal + ((uint32_t)p->kind << 8) + ((uint32_t)p->flags << 16) + ((uint32_t)p->phase << 24));
    }
    return h;
}

const fxv_desc_t fxv_d_fireworks = {b7_bytes,b7_init,b7_advance,b7_slice,b7_render,0,b7_changed,b7_digest};
const fxv_desc_t fxv_d_countdown = {b7_bytes,b7_init,b7_advance,b7_slice,b7_render,0,b7_changed,b7_digest};
const fxv_desc_t fxv_d_collider = {b7_bytes,b7_init,b7_advance,b7_slice,b7_render,0,b7_changed,b7_digest};
const fxv_desc_t fxv_d_confetti_cannon = {b7_bytes,b7_init,b7_advance,b7_slice,b7_render,0,b7_changed,b7_digest};
const fxv_desc_t fxv_d_pinball_relay = {b7_bytes,b7_init,b7_advance,b7_slice,b7_render,0,b7_changed,b7_digest};
const fxv_desc_t fxv_d_packet_switch = {b7_bytes,b7_init,b7_advance,b7_slice,b7_render,0,b7_changed,b7_digest};
