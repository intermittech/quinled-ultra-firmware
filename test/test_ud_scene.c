/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#define UDFX_SCENE_TEST
static void scene_hook(unsigned after);
#define main udfx_suite_main
#include "test_udfx.c"
#undef main

static const uint8_t RED[9] = {255,0,0}, BLUE[9] = {0,0,255};
static uint32_t inject_at, inject_after, inject_encoding, injected, hook_on;
static void scene_hook(unsigned after)
{
    if (hook_on && !injected && after == inject_after && U.frames_done == 1u &&
        (inject_encoding ? U.e.state == WS_FRAME && U.e.i >= inject_at : U.begun && U.rendered == inject_at)) {
        send_fx(ULTRA_LIGHT_BOARD, 2, FX_SOLID, inject_encoding == 2 ? 10 : 0, BLUE, 255, 0, 0, 0, 0, udfx_host_us(&U, D.wpos));
        injected = 1;
    }
}

static void scene_start(uint32_t k)
{
    if (!g_step) {
        udfx_anchor(&U, k * 4u / 9u, k);
        send_fx(ULTRA_LIGHT_BOARD, 1, FX_SOLID, 0, RED, 255, 0, 0, 0, 0, k * 4u / 9u);
        g_step = 1;
    }
}

/* The real DMA / decoder model checks pulse widths, byte order, latch, ownership and grid.
 * Inject on both sides of every 32-pixel chunk, and while each quarter is being encoded. */
static void frame_injections(void)
{
    uint32_t chip, side, at, enc;
    sim_t sc = {0};
    world_reserve(MAXW);
    sc.T = 2250u * 100u; sc.steps = 4; sc.cmd = scene_start;
    for (chip = 0; chip < ULTRA_PXC_COUNT; chip++) {
        for (enc = 0; enc < 3; enc++) for (side = 0; side < 2; side++) for (at = 0; at <= 96; at += 32) {
            uint32_t n, k, j, blue = 0;
            uint8_t r[5], b[5];
            make_rx(chip, 0, 96, 50, 0);
            line_clear();
            inject_at = enc ? at * CHIP[chip].bpp : at;
            if (enc && at == 96) inject_at -= 8;           /* the tick consumes several bytes at once */
            inject_after = side; inject_encoding = enc; injected = 0; hook_on = 1;
            simulate(&sc);
            hook_on = 0;
            CHECK(injected, "injection missed chip=%u side=%u at=%u enc=%u", chip, side, at, enc);
            memset(r, 0, sizeof r); memset(b, 0, sizeof b);
            for (j = 0; j < U.chip.bpp; j++) { r[j] = U.pack.ord[j] == 0 ? 255 : 0; b[j] = U.pack.ord[j] == 2 ? 255 : 0; }
            n = decode(out, 0, sc.T - 600, CHIP[chip].s, CHIP[chip].t0h, CHIP[chip].t1h);
            CHECK(n >= 3 && U.e.lost == 0, "too few frames / lost encoder");
            for (k = 0; k < n; k++) {
                const uint8_t *c = !memcmp(fr[k].b, b, U.chip.bpp) ? b : r;
                blue += c == b;
                CHECK(fr[k].nbits == 96u * U.chip.bpp * 8u && fr[k].exact == 1u, "wire length / pulses");
                CHECK(fr[k].low_before >= U.chip.reset_w * 16u, "short reset");
                {
                    const uint32_t h = udfx_host_us(&U, fr[k].start_w);
                    const int32_t err = (int32_t)(h - ((h + 10000u) / 20000u) * 20000u);
                    CHECK(err >= -1 && err <= 1, "command moved the exact frame grid: %d us", err);
                }
                CHECK(k < n_exp && !memcmp(fr[k].b, exp_fb[k], 96u * U.chip.bpp), "encoded bytes differ");
                for (j = 0; j < 96; j++) CHECK(!memcmp(fr[k].b + j * U.chip.bpp, c, U.chip.bpp), "torn scene");
            }
            CHECK(blue > 0, "new scene never displayed");
            CHECK(deadline_misses(sc.T, 0) == 0, "wire deadline missed");
        }
    }
    free(line); free(out); free(owned_t); line = out = 0; owned_t = 0;
}

static void at_host(uint32_t host)
{
    U.local = 1; U.dark = 0; U.e.ready = 0; U.e.state = WS_LOW; U.e.low_w = U.e.reset_w;
    U.e.w = U.e.next_w = U.a_w = 0; U.a_host = host; U.period_us = 0; U.begun = 0;
    while (udfx_render(&U)) {}
}

static void scheduled_scene(void)
{
    uint32_t pass, h;
    for (pass = 0; pass < 2; pass++) {
        h = pass ? 0xFFFFF000u : 100000u;
        make_rx(0, 0, 64, 0, 0); D.wpos = 0;
        udfx_service(&U, &D, 0, 0, 256);
        send_fx(0, 1, FX_SOLID, 0, RED, 255, 0, 0, 64, 0, h);
        send_fx(1, 1, FX_SOLID, 0, BLUE, 255, 0, 32, 32, 0, h);
        at_host(h);
        send_fx(ULTRA_LIGHT_BOARD, 1, FX_SOLID, 10, BLUE, 255, 0, 0, 64, ULTRA_PXF_RESTART | ULTRA_PXF_STORE, h);
        CHECK(U.stored_have == 1 && U.seg[0].restarts == 0, "STORE / RESTART precedence");
        at_host(h + 9999u);
        CHECK(U.seg[1].run && U.seg[0].p.c[0][0] == 255, "future whole-strip canceled early");
        at_host(h + 10000u);
        CHECK(!U.seg[1].run && U.seg[0].restarts == 1 && U.seg[0].p.c[0][2] == 255, "START boundary not atomic");
        send_fx(0, 2, FX_SOLID, 100, RED, 255, 0, 0, 64, ULTRA_PXF_RESTART, h);
        send_fx(0, 3, FX_SOLID, 0, BLUE, 255, 0, 0, 64, 0, h);
        at_host(h + 200000u);
        CHECK(U.seg[0].restarts == 1 && U.seg[0].p.c[0][2] == 255, "superseded START / RESTART ran");
        send_fx(2, 1, FX_STREAM, 100, RED, 0, 0, 0, 0, ULTRA_PXF_STORE, h);
        U.chain = 1; at_host(h + 200001u);
        CHECK(U.stored_have == 0 && fx_engine_dark(&U.eng), "STREAM must cancel every segment and ignore START");
    }
    /* The final 20-ms grid point before host-us wrap must not round to zero. */
    make_rx(0, 0, 64, 50, 0); D.wpos = 0;
    udfx_service(&U, &D, 0, 0, 256);
    send_fx(0, 1, FX_SOLID, 10, BLUE, 255, 0, 0, 64, 0, 4294955000u);
    U.local = 1; U.e.state = WS_LOW; U.e.next_w = U.a_w = 0; U.a_host = 4294960000u;
    while (udfx_render(&U)) {}
    CHECK(U.seg[0].pend_have && !U.seg[0].run, "grid rounding applied START early near wrap");
    /* Positive START can exceed half of the 32-bit microsecond clock. */
    make_rx(0, 0, 64, 0, 0); D.wpos = 0;
    udfx_service(&U, &D, 0, 0, 256);
    send_fx(0, 1, FX_SOLID, 3600000, BLUE, 255, 0, 0, 64, 0, 100500u);
    at_host(100500u);
    CHECK(U.seg[0].pend_have && !U.seg[0].run, "one-hour START applied on receipt");
    /* Advance the continuous reference as the normal service does over the hour. */
    U.hus_ref = 3600100000u; U.hms_ref = 3600100u;
    at_host(3600100499u);
    CHECK(U.seg[0].pend_have && !U.seg[0].run, "START rounded down before its exact us edge");
    at_host(3600100500u);
    CHECK(!U.seg[0].pend_have && U.seg[0].run, "long START missed its exact edge");
}

/* A valid PIXEL_FX command between two real renderer chunks. This is the
 * ordering in main_ud.c: dl_main(), udfx_render(), then service. */
static void partial_frame_command(void)
{
    static const uint8_t red[9] = {255,0,0}, blue[9] = {0,0,255};
    uint32_t i, changed = 0;
    make_rx(0, 0, 64, 50, 1.0);
    dline_init(&D, 0);
    D.direct = 0; D.rec = 0; D.wpos = 11250; D.zs = 0;
    udfx_service(&U, &D, 5, 0, 256);
    udfx_anchor(&U, 5000, D.wpos);
    send_fx(0, 1, FX_SOLID, 0, red, 255, 0, 0, 64, 0, 5000);
    udfx_service(&U, &D, 5, 0, 256);
    (void)udfx_tick(&U, &D, ring);
    CHECK(U.local && U.e.state == WS_LOW, "LOCAL did not begin in LOW");
    while (!U.rendered) udfx_render(&U);                /* frame setup and the first chunk are separate steps */
    CHECK(U.rendered == 32, "first chunk has %u pixels", U.rendered);
    send_fx(0, 2, FX_SOLID, 0, blue, 255, 0, 0, 64, 0, 5000);
    for (i = 0; i < 4; i++) udfx_render(&U);
    CHECK(U.e.ready, "frame was not published");
    for (i = 1; i < 64; i++) changed += memcmp(ux_pool, ux_pool + 3*i, 3) != 0;
    printf("published solid frame: first GRB=%u,%u,%u last=%u,%u,%u; %u pixels differ\n",
           ux_pool[0],ux_pool[1],ux_pool[2],ux_pool[189],ux_pool[190],ux_pool[191],changed);
    CHECK(changed == 0, "one solid frame contains old and new colours");
}

static void interrupted_crossfade(void)
{
    fx_engine_t eng;
    fx_params_t p;
    rgb_t before, after;
    uint8_t pool[1024];
    uint32_t k;
    memset(&p, 0, sizeof p);
    fx_engine_init(&eng, 1, pool, sizeof pool);
    p.id = FX_SOLID; p.bri = 255; p.c[0].r = 255;
    fx_engine_set(&eng, 0, &p, 0, 0);
    p.id = FX_BREATHE; p.c[0].r = 0; p.c[0].b = 255; p.trans_ms = 1000;
    fx_engine_set(&eng, 0, &p, 0, 0);
    fx_engine_frame(&eng, 500);
    fx_engine_render(&eng, &before, 0, 1);
    p.id = FX_GRADIENT;
    for (k=0; k<3; k++) { p.c[k].r=0; p.c[k].g=255; p.c[k].b=0; }
    fx_engine_set(&eng, 0, &p, 500, 500);
    fx_engine_frame(&eng, 500);
    fx_engine_render(&eng, &after, 0, 1);
    printf("crossfade interrupted at the same time: RGB %u,%u,%u -> %u,%u,%u\n",
           before.r,before.g,before.b,after.r,after.g,after.b);
    CHECK(abs((int)before.r-after.r)+abs((int)before.g-after.g)+abs((int)before.b-after.b) <= 3,
          "new transition jumps away from the displayed mixture");
}

static void engine_pixels(fx_engine_t *e, rgb_t *p)
{
    fx_engine_render(e, p, 0, 32);
    fx_engine_render(e, p + 32, 32, 64);
}

static void replacement_policy(void)
{
    uint32_t flags, fire, wrap, t, j;
    for (flags = 0; flags < 4; flags++) for (fire = 0; fire < 2; fire++) for (wrap = 0; wrap < 2; wrap++) {
        fx_engine_t e;
        fx_params_t p = {0};
        uint8_t heat[1024], saved[1024];
        rgb_t before[64], after[64], single;
        const uint32_t base = wrap ? 0xFFFFFF00u : 0u;
        int x0, x1;
        fx_engine_init(&e, 64, heat, sizeof heat);
        p.id = fire ? FX_FIRE : FX_SOLID; p.bri = 255; p.flags = (uint8_t)flags;
        p.speed = fire ? 128 : 0; p.intensity = 180;
        p.palette = PAL_DEFAULT; p.c[0].r = 255; p.seg_len = 32;
        for (j = 0; j < 2; j++) { p.seg_start = (uint16_t)(32*j); fx_engine_set(&e, j, &p, base, base); }
        p.id = FX_BREATHE; p.speed = 0; p.intensity = 0; p.c[0].r = 0; p.c[0].b = 255; p.trans_ms = 65535;
        for (j = 0; j < 2; j++) { p.seg_start = (uint16_t)(32*j); fx_engine_set(&e, j, &p, base, base); }
        fx_engine_frame(&e, base + 500);
        engine_pixels(&e, before);
        memcpy(saved, heat, sizeof heat);
        x0 = e.seg[0].xf; x1 = e.seg[1].xf;
        CHECK(x0 >= 0 && x1 >= 0 && x0 != x1, "both slots must be occupied");
        p.id = FX_GRADIENT; p.trans_ms = 1000;
        for (j = 0; j < 3; j++) { p.c[j].r = 0; p.c[j].g = 255; p.c[j].b = 0; }
        for (j = 0; j < 2; j++) { p.seg_start = (uint16_t)(32*j); fx_engine_set(&e, j, &p, base + 500, base + 500); }
        fx_engine_frame(&e, base + 500);
        engine_pixels(&e, after);
        CHECK(!memcmp(before, after, sizeof before), "replacement jumps (flags=%u fire=%u wrap=%u)", flags, fire, wrap);
        CHECK(!memcmp(saved, heat, sizeof heat), "queued change touched Fire state");
        CHECK(e.xf[x0].t0 + e.xf[x0].dur - (base + 500) <= 250, "finish deadline exceeds 250 ms");
        for (t = 501; t <= 750; t++) {
            fx_engine_frame(&e, base + t);
            engine_pixels(&e, before);
            if (t == 550 || t == 700) {
                /* Third and fourth changes, including one during accelerated finish. */
                p.id = t == 550 ? FX_RAINBOW : (fire ? FX_FIRE : FX_STRIPES);
                for (j = 0; j < 2; j++) { p.seg_start = (uint16_t)(32*j); fx_engine_set(&e, j, &p, base + t, base + t); }
                fx_engine_frame(&e, base + t);
                engine_pixels(&e, after);
                CHECK(!memcmp(before, after, sizeof before), "rapid replacement jumps");
                CHECK(e.xf[x0].t0 + e.xf[x0].dur == base + 750, "later arrival extended the deadline");
            }
            if (t < 750) CHECK(e.seg[0].cur.id == FX_BREATHE && e.seg[1].cur.id == FX_BREATHE, "queued target started early");
            for (j = 0; j < 64; j++) {
                fx_engine_render(&e, &single, j, j+1);
                CHECK(!memcmp(&single, &before[j], sizeof single), "chunk / reversed / mirrored order changed output");
            }
        }
        CHECK(e.seg[0].cur.id == p.id && e.seg[1].cur.id == p.id, "latest target did not start by 250 ms");
        CHECK(e.seg[0].xf >= 0 && e.seg[1].xf >= 0, "replacement lost a crossfade slot");
        CHECK(e.xf[e.seg[0].xf].t0 == base + 750 && e.xf[e.seg[1].xf].t0 == base + 750, "new fade starts at finish deadline");
        if (fire) CHECK(e.seg[0].cur.state != e.seg[1].cur.state, "Fire heat aliased between segments");
        /* STREAM cancels the queued successor as well as both live contexts. */
        p.id = FX_SOLID; p.seg_start = 0; fx_engine_set(&e, 0, &p, base + 800, base + 800);
        p.id = FX_STREAM; fx_engine_set(&e, 0, &p, base + 801, base + 801);
        fx_engine_frame(&e, base + 2000);
        CHECK(!e.seg[0].active && e.seg[0].xf < 0, "STREAM resurrected a queued scene");
    }
    {
        fx_engine_t a, ref;
        fx_params_t p = {0};
        uint8_t heat[1024], refheat[1024], saved[1024];
        rgb_t before[64], after[64];
        p.id = FX_FIRE; p.palette = PAL_DEFAULT; p.speed = 128; p.intensity = 220; p.bri = 255;
        fx_engine_init(&a, 64, heat, sizeof heat); fx_engine_init(&ref, 64, refheat, sizeof refheat);
        fx_engine_set(&a, 0, &p, 0, 0); fx_engine_set(&ref, 0, &p, 0, 0);
        for (t = 0; t <= 3000; t += 20) { fx_engine_frame(&a, t); fx_engine_frame(&ref, t); }
        memcpy(saved, heat, sizeof heat); engine_pixels(&a, before);
        p.palette = PAL_LAVA; p.trans_ms = 1000;
        fx_engine_set(&a, 0, &p, 3000, 0); fx_engine_frame(&a, 3000); engine_pixels(&a, after);
        CHECK(!memcmp(before, after, sizeof before) && !memcmp(saved, heat, sizeof heat), "Fire palette switch cleared / jumped");
        for (t = 3020; t < 4000; t += 20) {
            fx_engine_frame(&a, t); fx_engine_frame(&ref, t);
            CHECK(!memcmp(heat, refheat, 64), "Fire shared heat stepped twice / reset its phase");
        }
    }
    {
        fx_engine_t e;
        fx_params_t p = {0};
        uint8_t heat[1024];
        rgb_t a, b;
        fx_engine_init(&e, 1, heat, sizeof heat);
        p.id = FX_SOLID; p.bri = 255; p.c[0].r = 255;
        fx_engine_set(&e, 0, &p, 0, 0);
        p.id = FX_BREATHE; p.bri = 64; p.c[0].r = 0; p.c[0].b = 255; p.trans_ms = 5000;
        fx_engine_set(&e, 0, &p, 0, 0); fx_engine_frame(&e, 2500);
        fx_engine_render(&e, &a, 0, 1);
        p.id = FX_GRADIENT; p.bri = 220; p.trans_ms = 1000;
        fx_engine_set(&e, 0, &p, 2500, 2500);
        fx_engine_frame(&e, 2500); fx_engine_render(&e, &b, 0, 1);
        CHECK(!memcmp(&a, &b, sizeof a), "retimed brightness jumps at request");
        for (t = 2501; t <= 2750; t++) {
            a = b; fx_engine_frame(&e, t); fx_engine_render(&e, &b, 0, 1);
            CHECK(abs((int)a.r-b.r)+abs((int)a.g-b.g)+abs((int)a.b-b.b) <= 5, "accelerated finish has a step");
        }
        CHECK(e.seg[0].cur.id == FX_GRADIENT && e.seg[0].bri == 64, "handoff did not reach the old destination");
        /* A sparse frame sequence must not add the sampling delay to fade start. */
        p.id = FX_RAINBOW;
        fx_engine_set(&e, 0, &p, 2800, 2800);
        fx_engine_frame(&e, 3100);
        CHECK(e.seg[0].cur.id == FX_RAINBOW && e.xf[e.seg[0].xf].t0 == 3050 && e.xf[e.seg[0].xf].w > 0,
              "sampling after the deadline extended queued latency");
    }
}


/* A08 applies commands before A09 retires a fade. A command arriving after
 * its predecessor's deadline must retain its full transition, even at 1 fps. */
static void regression_1(void)
{
    uint32_t wrap;
    for (wrap = 0; wrap < 2; wrap++) {
        fx_engine_t e;
        fx_params_t p = {0};
        uint8_t heat[1024];
        const uint32_t base = wrap ? 0xFFFFFF00u : 0u;
        fx_engine_init(&e, 64, heat, sizeof heat);
        p.id = FX_SOLID; p.bri = 255; p.c[0].r = 255;
        fx_engine_set(&e, 0, &p, base, base);
        p.id = FX_BREATHE; p.trans_ms = 100;
        fx_engine_set(&e, 0, &p, base + 1000u, base + 1000u);
        fx_engine_frame(&e, base + 1000u);
        p.id = FX_GRADIENT; p.trans_ms = 300;
        fx_engine_set(&e, 0, &p, base + 2000u, base + 2000u);
        fx_engine_frame(&e, base + 2000u);
        CHECK(e.seg[0].tr_t0 == base + 2000u && e.seg[0].tr_inv != 0,
              "new transition backdated to an expired fade");
        CHECK(e.seg[0].xf >= 0 && e.xf[e.seg[0].xf].w == 0,
              "new crossfade skipped its beginning");
    }
}


/* Configuration can arrive in the main-loop pass that freezes a scene. */
static void regression_2(void)
{
    uint32_t chain, pass;
    for (chain = 0; chain < 2; chain++) {
        make_rx(0, 0, 64, 0, 0); dline_init(&D, 0);
        udfx_service(&U, &D, 0, 0, 256);
        U.chain = (uint8_t)chain;
        send_fx(0, 1, FX_PACIFICA, 0, RED, 255, 0, 0, 64, ULTRA_PXF_RESTART, 100000);
        U.local = 1; U.a_host = 100000; U.a_w = D.wpos = 2250; D.direct = 0;
        ws_enc_init(&U.e, &U.chip, ux_pool, U.npx, D.wpos, 0);
        U.e.low_w = U.e.reset_w;
        udfx_render(&U); /* frozen, not applied yet */
        CHECK(U.seg[0].apply == 1 && !U.seg[0].pend_have, "command not frozen");
        udfx_config(&U, ULTRA_CFG_PX_PIXELS, 65);
        udfx_service(&U, &D, 100, 0, 256);
        CHECK(U.reconf && !U.pause_req && !U.want_stream,
              "reconfiguration interrupted frozen scene preparation");
        /* The real tick acknowledges pause or performs the latch-safe handback. */
        for (pass = 0; pass < 24 && U.reconf; pass++) {
            D.wpos += 112;
            udfx_tick(&U, &D, ring);
            udfx_render(&U);
            udfx_service(&U, &D, 101 + pass, 0, 256);
        }
        CHECK(!U.reconf, "reconfiguration did not finish");
        at_host(101000);
        CHECK(U.npx == 65 && U.seg[0].run && U.eng.seg[0].active && U.seg[0].restarts == 1,
              "reconfiguration lost a frozen PIXEL_FX / RESTART");
    }
}


/* A CONFIG rebuild must not attach the old destination's clock to a queued sun. */
static void regression_3(void)
{
    make_rx(0, 0, 64, 0, 0); dline_init(&D, 0);
    udfx_service(&U, &D, 0, 0, 256); U.chain = 1;
    send_fx(0, 1, FX_SOLID, 0, RED, 255, 0, 0, 64, 0, 0); at_host(0);
    send_fx(0, 2, FX_BREATHE, 0, BLUE, 255, 5000, 0, 64, 0, 1000000); at_host(1000000);
    send_fx(0, 3, FX_SUNRISE, 0, RED, 255, 1000, 0, 64, 0, 2000000); at_host(2000000);
    CHECK(U.eng.seg[0].xf >= 0 && (U.eng.xf[U.eng.seg[0].xf].used & 2u), "target not queued");
    udfx_config(&U, ULTRA_CFG_PX_PIXELS, 65);
    U.paused = 1; udfx_service(&U, &D, 2001, 0, 256);
    U.pause_req = U.paused = 0; at_host(2001000);
    CHECK(U.eng.seg[0].cur.id == FX_SUNRISE && U.eng.seg[0].anchor == 2000,
          "CONFIG rebuild gave queued target the old destination's anchor");
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    if (!strcmp(argv[1], "frame")) { partial_frame_command(); frame_injections(); scheduled_scene(); }
    else if (!strcmp(argv[1], "crossfade")) { interrupted_crossfade(); replacement_policy(); regression_3(); regression_2(); regression_1(); }
    else return 2;
    printf("proposal %s: %ld checks, %d failed\n", argv[1], g_checks, g_fail);
    return g_fail != 0;
}
