/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The V407 Digital-M's local effects (core/ur_digital_m/udfx.c with FX_V407 + argbenc.c + the effects layer) on a model of
 * the ARGB serialiser (argb_model_v407.h): 50 us ticks (156 ring words of 0.32 us), the main loop between them (service
 * once a ms, a few render steps a tick). test\build_udfx_v407.cmd [outdir].
 *
 * Checks: two wire frames, the main loop never writes the one on the wire (its bytes are the same at the end as at the
 * start); frames start on the host grid; the grid is the wire time + reset + guard, at most 100 fps and at most the
 * CONFIG 67 ceiling (800 RGB / 600 RGBW / 500 RGBCCT and short strips); an effect this build does not run is refused
 * (the segment keeps its effect, never Solid); every legacy effect runs (Pacifica too); page 0x11 reports the generated
 * maps and counts; a crossfade keeps the grid. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/ur_digital_m/udfx.h"
#include "../src/core/ultra_protocol.h"

static int g_fail;
static long g_checks;
static const char *g_name = "";
#define CHECK(c, ...) do { g_checks++; if (!(c)) { if (g_fail++ < 20) { printf("  FAIL %s:%d [%s] ", __FILE__, __LINE__, g_name); \
    printf(__VA_ARGS__); printf("\n"); } } } while (0)

/* ---- the serialiser model */
uint32_t m_now, m_running, m_start, m_n, m_per, m_fail;
static const uint8_t *m_buf;
static uint8_t m_snap[UX_POOL_BYTES];
static uint32_t n_frames, first_start, last_start, max_dev, owner_bad, grid_w, grid_us, grid_wpu24;
void m_frame_start(const uint8_t *b, uint32_t n, uint32_t per)
{
    if (m_running && m_buf) {                            /* (the previous frame's end: was its buffer left alone?) */
        owner_bad += memcmp(m_snap, m_buf, m_n) != 0;
    }
    m_buf = b;
    m_n = n;
    m_per = per;
    m_start = m_now;
    m_running = 1;
    memcpy(m_snap, b, n);
    if (n_frames == 0u) first_start = m_now;
    else if (grid_w) {
        /* the host grid point k x P in words (period_w is P rounded down: k x period_w drifts off the host grid) */
        const uint32_t k = (m_now - first_start + grid_w / 2u) / grid_w;
        const uint32_t dev = m_now - first_start - (uint32_t)(((uint64_t)k * grid_us * grid_wpu24) >> 24);
        const uint32_t ad = (int32_t)dev < 0 ? 0u - dev : dev;
        if (ad > max_dev) max_dev = ad;
    }
    last_start = m_now;
    n_frames++;
}

static dline_t D;
static udfx_t U;
static uint16_t ring[4];
#define TICK_W 156u

static void send_fx(uint8_t seg, uint8_t seq, uint8_t effect, uint16_t trans, uint32_t w)
{
    ultra_pixel_fx_t f;
    uint8_t s[32];
    memset(&f, 0, sizeof f);
    f.effect = effect;
    f.speed = 150;
    f.intensity = 128;
    f.palette = ULTRA_PX_PAL_DEFAULT;
    f.c[0][0] = 255; f.c[0][1] = 120; f.c[1][2] = 200;
    f.brightness = 255;
    f.transition_ms = trans;
    ultra_slot_pixel_fx(s, 0, seg, seq, &f);
    udfx_pixel_fx(&U, s, udfx_host_us(&U, w));
}

/* run ms milliseconds of ticks and main loop; the tick checks the in-flight buffer each time */
static uint32_t W;
static void run(uint32_t ms)
{
    const uint32_t end = W + ms * MCU_WORDS_MS;
    uint32_t next_ms = W;
    while ((int32_t)(end - W) > 0) {
        uint32_t k;
        W += TICK_W;
        m_now = W;
        D.wpos = W;
        udfx_tick(&U, &D, ring);
        if (m_running && U.e.in_flight == 0) {          /* (finished: check its bytes stayed) */
            owner_bad += memcmp(m_snap, m_buf, m_n) != 0;
            m_running = 0;
        }
        if ((int32_t)(W - next_ms) >= 0) {
            udfx_service(&U, &D, W / MCU_WORDS_MS, 0, 256);
            next_ms += MCU_WORDS_MS;
        }
        for (k = 0; k < 6u; k++) {
            if (!udfx_render(&U)) break;
            if (m_running && memcmp(m_snap, m_buf, m_n) != 0) owner_bad++;   /* the main loop wrote the frame on the wire */
        }
    }
}

static void setup(uint32_t chip, uint32_t px, uint32_t fps)
{
    memset(&D, 0, sizeof D);
    D.state = DLINE_PASS;
    D.zs = 0;
    udfx_init(&U);
    (void)udfx_config(&U, ULTRA_CFG_PX_CHIP, chip);
    (void)udfx_config(&U, ULTRA_CFG_PX_PIXELS, px);
    if (fps) (void)udfx_config(&U, ULTRA_CFG_PX_FPS, fps);
    W = 1000u;
    m_running = 0;
    n_frames = 0;
    max_dev = 0;
    owner_bad = 0;
    run(5);
}

/* the expected grid in host us: the model's wire time + reset + guard, capped at 100 fps / the ceiling */
static uint32_t expect_us(uint32_t px, uint32_t bpp, uint32_t fps)
{
    const uint32_t need = (px * bpp * 8u * MCU_SYM_BIT + 99u) / 100u + U.chip.reset_w * MCU_US_PER_WORD_NUM / MCU_US_PER_WORD_DEN + UX_GUARD_US;
    const uint32_t cap = 1000000u / (fps && fps < 100u ? fps : 100u);
    return need > cap ? need : cap;
}

static void scenario(const char *name, uint32_t chip, uint32_t bpp, uint32_t px, uint32_t fps, uint32_t effect)
{
    uint32_t want_us, frames;
    g_name = name;
    setup(chip, px, fps);
    want_us = expect_us(px, bpp, fps);
    CHECK(U.period_us == want_us, "grid %u us, expected %u", U.period_us, want_us);
    CHECK(U.npx == px && U.max_px == (bpp == 3u ? FXV_N_RGB : bpp == 4u ? FXV_N_RGBW : FXV_N_RGBCCT), "pixels %u max %u", U.npx, U.max_px);
    grid_w = U.period_w;
    grid_us = U.period_us;
    grid_wpu24 = U.wpu24;
    send_fx(0, 1, (uint8_t)effect, 0, W);
    run(300);
    n_frames = 0;
    max_dev = 0;
    run(2000);
    frames = n_frames;
    {
        const uint32_t want = 2000000u / want_us;
        CHECK(frames + 2u >= want && frames <= want + 2u, "%u frames in 2 s, expected ~%u (%u us grid)", frames, want, want_us);
    }
    CHECK(max_dev <= TICK_W + 2u, "a frame %u words off its grid point", max_dev);
    CHECK(owner_bad == 0u, "the main loop wrote the frame on the wire %u times", owner_bad);
    CHECK(U.seg[0].run && U.seg[0].p.effect == effect, "segment 0 runs %u", U.seg[0].p.effect);
    printf("  %-28s %4u px  grid %6u us (%5.1f fps)  %3u frames / 2 s  max grid offset %3u words\n", name, px, U.period_us,
           1e6 / U.period_us, frames, max_dev);
}

static void page_and_refusal(void)
{
    uint8_t d[64];
    uint32_t i;
    g_name = "page 0x11, unbuilt ids";
    setup(0, 60, 0);
    send_fx(0, 1, 38, 0, W);
    run(200);
    memset(d, 0, sizeof d);
    CHECK(udfx_page(&U, ULTRA_PXG_PART_STATE, d) == 0, "part 0");
    CHECK(d[ULTRA_PXG_EFFECTS] == FX_LAST + FXV_BUILT_NEW && d[ULTRA_PXG_PALETTES] == FXV_PAL_COUNT, "counts %u %u", d[ULTRA_PXG_EFFECTS],
          d[ULTRA_PXG_PALETTES]);
    CHECK(((uint32_t)d[ULTRA_PXG_MAX_PIXELS] << 8 | d[ULTRA_PXG_MAX_PIXELS + 1]) == FXV_N_RGB, "max pixels");
    CHECK(d[ULTRA_PXG_RUNNING] == 38, "running %u", d[ULTRA_PXG_RUNNING]);
    memset(d, 0, sizeof d);
    CHECK(udfx_page(&U, ULTRA_PXG_PART_EFFECTS, d) == 0 && memcmp(d, fxv_map_effects, 24) == 0, "part 1 = the effect map");
    memset(d, 0, sizeof d);
    CHECK(udfx_page(&U, ULTRA_PXG_PART_PALETTES, d) == 0 && memcmp(d, fxv_map_palettes, 24) == 0, "part 2 = the palette map");
    for (i = FX_COUNT; i < FXV_FX_COUNT; i++) {          /* an unbuilt effect is refused: segment 0 keeps 38 */
        if (fxv_built(i)) continue;
        send_fx(0, (uint8_t)(2u + i), (uint8_t)i, 0, W);
        run(40);
        CHECK(U.seg[0].p.effect == 38 && U.seg[0].run, "effect %u (not built) changed segment 0 to %u", i, U.seg[0].p.effect);
    }
    for (i = 1; i < FX_COUNT; i++) {                     /* every legacy effect runs here (Pacifica too) */
        send_fx(1, (uint8_t)(200u + i), (uint8_t)i, 0, W);
        run(30);
        CHECK(U.seg[1].p.effect == i && U.seg[1].run, "legacy effect %u refused", i);
    }
    CHECK(owner_bad == 0u, "ownership %u", owner_bad);
}

static void crossfade(void)
{
    uint32_t frames;
    g_name = "crossfade on the grid";
    setup(4, 300, 0);
    grid_w = U.period_w;
    grid_us = U.period_us;
    grid_wpu24 = U.wpu24;
    send_fx(0, 1, 38, 0, W);
    run(300);
    n_frames = 0;
    max_dev = 0;
    send_fx(0, 2, 4, 1000, W);
    run(500);
    send_fx(0, 3, 38, 1000, W);                          /* a replacement while the fade runs: queued */
    run(1500);
    frames = n_frames;
    CHECK(frames + 3u >= 2000000u / U.period_us, "%u frames through the fades", frames);
    CHECK(max_dev <= TICK_W + 2u, "grid offset %u", max_dev);
    CHECK(owner_bad == 0u, "ownership %u", owner_bad);
    CHECK(U.seg[0].p.effect == 38, "the queued target won: %u", U.seg[0].p.effect);
}

static void auto_stream_policy(void)
{
    uint8_t rec[UX_REC_BYTES];
    uint32_t frames;
    g_name = "AUTO replay ownership";
    setup(0, 60, 0);
    CHECK(!U.stream && U.local, "AUTO must permit local output before detection");
    (void)udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_STREAM);
    (void)udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_AUTO);
    run(50);                                          /* no intervening mode-machine decision */
    CHECK(!U.stream && U.local, "unobserved CONFIG writes selected AUTO stream");
    send_fx(0, 1, 38, 0, W);
    run(100);
    CHECK(U.local && U.seg[0].run, "local effect did not start before detection");
    udfx_chain(&U, 0);                                  /* detector selects STREAM */
    run(100);
    CHECK(!U.local && !D.local && !U.want_local, "saved geometry reclaimed AUTO stream");
    frames = n_frames;
    U.stored_have = 1;
    U.stored[0].effect = 38;
    U.stored[0].brightness = 255;
    (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, ULTRA_PXP_POWER_ON | ULTRA_PXP_ON_LOSS);
    (void)udfx_config(&U, ULTRA_CFG_PX_LOSS_TIMEOUT, 5);
    send_fx(0, 2, 38, 0, W);
    run(2500);
    CHECK(!U.local && !U.want_local && n_frames == frames, "effect/loss policy reclaimed AUTO stream");
    CHECK(U.cfg[ULTRA_CFG_PX_SOURCE - ULTRA_CFG_PX_FIRST] == 0u, "saved SOURCE changed");
    udfx_rec_pack(&U, 1, rec);
    (void)udfx_config(&U, ULTRA_CFG_PX_POLICY, 0);         /* forced-mode STOP check has no standalone fallback */
    CHECK(udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_STREAM), "force STREAM");
    run(100);
    CHECK(U.local, "forced STREAM + SOURCE LOCAL changed");
    CHECK(udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_AUTO), "return to AUTO");
    run(100);
    CHECK(!U.local && !U.undec, "sticky STREAM was not replayed on returning to AUTO");
    (void)udfx_config(&U, ULTRA_CFG_PX_LINE_MODE, ULTRA_PXLM_STREAM);
    (void)udfx_config(&U, ULTRA_CFG_PX_SOURCE, 1);
    run(100);
    CHECK(!U.local, "explicit 75=1 / 77=1 replay changed");
    send_fx(0, 3, 38, 0, W);
    run(100);
    CHECK(U.local, "forced STREAM must still allow temporary local FX");
    send_fx(0, 4, ULTRA_PX_FX_STOP, 0, W);
    run(100);
    CHECK(!U.local, "forced STREAM STOP did not return to replay");
    udfx_init(&U);
    udfx_load(&U, rec);
    CHECK(!U.stream && udfx_line_cfg(&U) == ULTRA_PXLM_AUTO && U.stored_have,
          "reboot must retain settings and forget the AUTO decision");
    udfx_chain(&U, 1);
    run(100);
    CHECK(U.chain && U.local, "AUTO CHAIN must retain local ownership");
    CHECK(owner_bad == 0u, "in-flight frame changed during handover");
    printf("  AUTO replay: local handover, loss/FX suppression, forced modes, sticky mode and reboot\n");
}

int main(void)
{
    scenario("RGB 60 px (100 fps cap)", ULTRA_PXC_WS2812B_V5, 3, 60, 0, 38);
    scenario("RGB 800 px (wire-limited)", ULTRA_PXC_WS2812B_V5, 3, 800, 0, 38);
    scenario("RGBW 600 px", ULTRA_PXC_SK6812_RGBW, 4, 600, 0, 38);
    scenario("RGBCCT 500 px", ULTRA_PXC_WS2805_RGBCCT, 5, 500, 0, 38);
    scenario("RGB 300 px, ceiling 30 fps", ULTRA_PXC_WS2812B_V5, 3, 300, 30, 38);
    scenario("RGB 800 px Pacifica", ULTRA_PXC_WS2812B_V5, 3, 800, 0, 18);
    page_and_refusal();
    crossfade();
    auto_stream_policy();
    printf("%s: test_udfx_v407, %ld checks, %d failures\n", g_fail ? "FAIL" : "PASS", g_checks, g_fail);
    return g_fail != 0;
}
