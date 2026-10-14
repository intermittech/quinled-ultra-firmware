/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Power-on scene tests (included by test_main.c): flash log, save rules, take-over cross-fade, and the scene
 * computed by the Ultra engine against what the engine renders once the controller restores the same state. */
#include "../src/core/scene.h"

/* ------------------------------------------------------------------------------------------------ flash model */
static uint8_t sflash[SCENE_LOG_PAGES * SCENE_PAGE_BYTES];
static uint32_t serase[SCENE_LOG_PAGES];
static int s_e339;                     /* erased flash reads 0xE339 (CH32V203 RM 32.5.4) instead of 0xFFFF */

static void sf_erase(uint8_t *p, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i += 2) { p[i] = s_e339 ? 0x39 : 0xFF; p[i + 1] = s_e339 ? 0xE3 : 0xFF; }
}

/* NOR flash: programming can only clear bits (0xE339: the value read back is the value written); a torn write
 * programs a prefix */
static void sf_program(uint32_t idx, const scene_rec_t *r, uint32_t bytes)
{
    uint32_t i;
    for (i = 0; i < bytes; i++) {
        uint8_t *f = &sflash[idx * SCENE_REC_BYTES + i];
        *f = s_e339 ? ((const uint8_t *)r)[i] : (uint8_t)(*f & ((const uint8_t *)r)[i]);
    }
}

/* the firmware's core/flashrec.c logic (0.6.4: the page pair, core/pagepair.h): position found once at boot, then
 * tracked in RAM; the erase that is due comes first (scene_store) */
typedef struct { pp_pos_t pos; uint16_t seq; } slog_t;

static uint32_t sf_index(uint32_t at)
{
    return PP_PAGE(at) * SCENE_RECS_PAGE + PP_ENTRY(at);
}

static int sf_boot(slog_t *l, scene_t *out)
{
    const uint8_t *r = pp_find(&scene_pp, sflash, sflash + SCENE_PAGE_BYTES, &l->pos);
    if (!r) {
        l->seq = 0;
        return -1;
    }
    *out = ((const scene_rec_t *)r)->s;
    l->seq = ((const scene_rec_t *)r)->seq;
    return (int)((r - sflash) / SCENE_REC_BYTES);
}

static void sf_erase_page(slog_t *l)
{
    uint32_t p;
    (void)pp_erase_due(&l->pos, &p);
    sf_erase(sflash + p * SCENE_PAGE_BYTES, SCENE_PAGE_BYTES);
    serase[p]++;
    pp_erased(&l->pos);
}

static uint32_t sf_store(slog_t *l, const scene_t *sc, uint32_t torn_bytes)
{
    scene_rec_t rec;
    uint32_t p, at;
    if (pp_erase_due(&l->pos, &p) == 2u) {
        sf_erase_page(l);
    }
    at = pp_plan(&scene_pp, &l->pos);
    scene_log_pack(sc, ++l->seq, &rec);
    sf_program(sf_index(at), &rec, torn_bytes ? torn_bytes : SCENE_REC_BYTES);
    return sf_index(at);
}

static void scene_rand(scene_t *s)
{
    int i;
    for (i = 0; i < 5; i++) s->v[i] = (uint16_t)rnd();
    s->mask = (uint8_t)(rnd() & 0x1F);
    s->fade = (uint8_t)rnd();
}

static void test_scene_log(int e339)
{
    slog_t l;
    scene_t a, b, back;
    uint32_t n, p, t, emin = 0xFFFFFFFFu, emax = 0;
    int bad = 0, i;

    s_e339 = e339;
    memset(serase, 0, sizeof serase);
    printf("power-on scene: flash log on the page pair (wear levelling, 16-bit sequence wrap, torn writes, erase ahead, "
           "SRAM shadow), erased = %s\n", e339 ? "0xE339" : "0xFFFF");
    sf_erase(sflash, sizeof sflash);
    CHECK(sf_boot(&l, &back) == -1 && l.pos.next == 0 && !l.pos.erase, "empty log");
    /* 70,000 writes: the sequence wraps; every reboot finds the last one and the same next position */
    for (n = 0; n < 70000u; n++) {
        scene_rand(&a);
        sf_store(&l, &a, 0);
        if (n % 997u == 0u || n > 69990u) {
            slog_t l2;
            if (sf_boot(&l2, &back) < 0 || !scene_equal(&back, &a) || l2.pos.next != l.pos.next ||
                l2.pos.erase != l.pos.erase) bad++;
        }
    }
    for (i = 0; i < (int)SCENE_LOG_PAGES; i++) {
        emin = serase[i] < emin ? serase[i] : emin;
        emax = serase[i] > emax ? serase[i] : emax;
    }
    CHECK(bad == 0, "newest record and next position found after every simulated reboot (%d bad)", bad);
    CHECK(emax - emin <= 1u && emax >= 70000u / SCENE_RECS && emax <= 70000u / SCENE_RECS + 1u,
          "even wear: %u..%u erases per page", emin, emax);
    printf("  70,000 writes: %u-%u erases per page (%u records per erase cycle: 10k cycles = %u writes)\n", emin, emax,
           SCENE_RECS, 10000u * SCENE_RECS);
    /* a write torn by a power cut: the previous record stays the newest; the next write does not reuse the torn slot
     * (its page counts as full: the other page, erased first unless it reads erased) */
    scene_rand(&a);
    sf_store(&l, &a, 0);
    scene_rand(&b);
    t = sf_store(&l, &b, 7);
    {
        slog_t l2;
        int k = sf_boot(&l2, &back);
        CHECK(k >= 0 && scene_equal(&back, &a), "torn write ignored");
        CHECK(sf_index(l2.pos.next) != t || l2.pos.erase, "next write skips the torn slot (or erases its page)");
        scene_rand(&b);
        sf_store(&l2, &b, 0);
        CHECK(sf_boot(&l2, &back) >= 0 && scene_equal(&back, &b), "written after the torn slot");
    }
    /* erased page boundary: power cut right after the erase of the next page */
    sf_erase(sflash, sizeof sflash);
    sf_boot(&l, &back);
    for (n = 0; n < SCENE_RECS_PAGE; n++) { scene_rand(&a); sf_store(&l, &a, 0); }
    CHECK(l.pos.erase && sf_index(l.pos.next) == SCENE_RECS_PAGE, "page full: next write erases page 1");
    {
        slog_t l2;
        /* the next page reads erased (here: never used; on the board: erased ahead in a dark / dim moment), so it needs
         * no erase; one holding old records does */
        CHECK(sf_boot(&l2, &back) == (int)SCENE_RECS_PAGE - 1 && scene_equal(&back, &a) &&
              sf_index(l2.pos.next) == SCENE_RECS_PAGE && !l2.pos.erase,
              "boot at a page boundary, the next page erased: no erase");
        sflash[SCENE_RECS_PAGE * SCENE_REC_BYTES + 5u] = 0x12u;   /* an old record's byte */
        CHECK(sf_boot(&l2, &back) == (int)SCENE_RECS_PAGE - 1 && sf_index(l2.pos.next) == SCENE_RECS_PAGE &&
              l2.pos.erase, "boot at a page boundary, the next page used: erase it first");
    }
    /* erase ahead (main.c, while low): the other page is erased while this one fills; the boundary then needs none,
     * and the page holding the newest record is never the one erased */
    {
        slog_t l4;
        int ok = 1;
        sf_boot(&l4, &back);                             /* page 0 full, page 1 dirty: erase due */
        CHECK(pp_erase_due(&l4.pos, &p) == 2u && p == 1u, "erase due: page 1");
        for (n = 0; n < 3u * SCENE_RECS; n++) {
            uint32_t due = pp_erase_due(&l4.pos, &p);
            if (due == 1u) {
                if (p == PP_PAGE(l4.pos.next)) ok = 0;   /* never the page being written */
                sf_erase_page(&l4);
            }
            if (PP_ENTRY(l4.pos.next) == 0u && n > 0u && l4.pos.erase) ok = 0;   /* a boundary that needs an erase */
            scene_rand(&a);
            sf_store(&l4, &a, 0);
            if (n % 5u == 0u) {
                slog_t l5;
                if (sf_boot(&l5, &back) < 0 || !scene_equal(&back, &a)) ok = 0;
            }
        }
        CHECK(ok, "erase ahead: no write waits for an erase, never the newest record's page, every reboot finds it");
    }
    /* SRAM shadow of the zero-wait flash: the log is read once at boot, the position tracked in RAM after that */
    {
        slog_t l3;
        sf_boot(&l3, &back);
        for (n = 0; n < 40u; n++) { scene_rand(&a); sf_store(&l3, &a, 0); }
        CHECK(sf_boot(&l3, &back) >= 0 && scene_equal(&back, &a), "40 writes without re-reading the log: all landed");
    }
}

static void test_scene_rules(void)
{
    scene_t a, b;
    scene_run_t r;
    uint16_t v[5];
    uint32_t t;
    int i, mono = 1, steady = 1;
    printf("power-on scene: save rules, hold, take-over cross-fade\n");

    scene_clear(&a);
    CHECK(a.mask == 0 && a.fade == SCENE_FADE_DEFAULT, "default: no scene, 1 s fade");
    b = a;
    b.mask = 0x18; b.v[3] = 30000; b.v[4] = 20000;
    CHECK(scene_major(&a, &b), "channels join: write soon");
    a = b;
    b.v[3] = 31000;
    CHECK(!scene_major(&a, &b) && !scene_equal(&a, &b), "small change: minor");
    b.v[3] = 30000 + SCENE_MAJOR_STEP + 1;
    CHECK(scene_major(&a, &b), "big change: major");
    b = a; b.v[4] = 0;
    CHECK(scene_major(&a, &b), "a channel turns off: major");
    b = a; b.v[0] = 999;                                 /* a channel outside the mask does not count */
    CHECK(scene_equal(&a, &b), "values outside the mask ignored");
    {
        scene_budget_t bg;
        uint32_t now, writes = 0, major_writes = 0, delayed = 0;
        scene_budget_init(&bg, 0);
        CHECK(!scene_save_due(&bg, 10000, 6000, 1) && scene_save_due(&bg, 11000, 6000, 1), "5 s quiet time");
        /* an automation changing the light every 10 s for 30 days, every 4th change big: the budget caps it */
        for (now = 20000; now < 30u * 86400000u; now += 10000) {
            int major = (now / 10000) % 4 == 0;
            if (scene_save_due(&bg, now, now - 5000, major)) {
                writes++;
                major_writes += (uint32_t)major;
            } else if (major) {
                delayed++;
            }
        }
        printf("  automation changing every 10 s for 30 days: %u writes (%.1f / day, %u big), lifetime at 10k cycles "
               ">= %.0f years\n", writes, writes / 30.0, major_writes, 10000.0 * SCENE_RECS / (writes / 30.0) / 365.0);
        CHECK(writes <= 30u * 96u + SCENE_BUDGET_CAP, "sustained <= 96 writes / day (%u)", writes);
        CHECK(major_writes * 2u >= writes, "big changes keep their reserve (%u of %u)", major_writes, writes);
        /* normal use: 30 changes spread over a day are all written promptly */
        scene_budget_init(&bg, 0);
        writes = 0;
        for (now = 3600000; now < 3600000u + 30u * 2880000u; now += 2880000) writes += (uint32_t)scene_save_due(&bg, now, now - 6000, 0);
        CHECK(writes == 30, "normal use: every change written (%u of 30)", writes);
        (void)delayed;
    }

    /* hold and cross-fade */
    a.mask = 0x19; a.v[0] = 40000; a.v[3] = 30000; a.v[4] = 20000; a.fade = 10;
    scene_boot(&r, &a);
    for (i = 0; i < 5; i++) v[i] = 777;
    CHECK(scene_mix(&r, 5, v) == 0 && v[0] == 40000 && v[3] == 30000 && v[4] == 20000 && v[1] == 777,
          "hold: scene channels show the scene, the others the controller");
    scene_takeover(&r, 0x1F, 1000);
    for (t = 1000; t <= 2100; t++) {
        uint16_t w[5] = { 10000, 0, 0, 30000, 50000 };
        scene_mix(&r, t, w);
        if (t > 1000 && (w[0] > v[0] || w[4] < v[4])) mono = 0;   /* 40000 -> 10000 down, 20000 -> 50000 up */
        if (w[3] != 30000) steady = 0;                             /* same value: no change at all */
        memcpy(v, w, sizeof v);
    }
    CHECK(mono && v[0] == 10000 && v[4] == 50000, "cross-fade monotonic, ends on the controller's values");
    CHECK(steady, "a channel the controller restores to the same value never moves");
    CHECK(r.hold == 0 && r.fading == 0, "done after the fade (%u ms)", a.fade * 100u);
}

/* the scene from the lights' SCENE values = what the engine renders once the host restores the same state */
static void test_scene_engine(void)
{
    static const uint8_t roles[5] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE, ULTRA_ROLE_WARM_WHITE, ULTRA_ROLE_COLD_WHITE };
    static const uint16_t mn[5] = { 0, 0, 0, 0, 0 }, mx[5] = { 65535, 65535, 65535, 65535, 65535 };
    static const uint16_t mn2[5] = { 655, 655, 655, 655, 655 };
    uint8_t s[ULTRA_SLOT_BYTES], ids[3] = { ULTRA_CFG_CH_CURRENT, ULTRA_CFG_CH_CURRENT + 3, ULTRA_CFG_CH_MAX + 3 };
    uint16_t a[5] = { 0, 0, 0, 52000, 21000 }, b[5] = { 65535, 65535, 65535, 65535, 65535 }, vals[3] = { 3000, 5000, 4000 };
    uint16_t out[5], v[5];
    ultra_t u;
    scene_t stored, sc, sc2;
    scene_run_t r;
    uint32_t tick = 20000, t, i;
    int worst = 0, d;

    printf("power-on scene: engine (gamma 2.8, constant brightness, min power, channel limit) vs restored state\n");
    ultra_init(&u, 3);
    ultra_on_header(&u, tick, 1000000);
    ultra_slot_config(s, 1, ids, vals, 3);
    ultra_on_slot(&u, s, tick);
    ultra_slot_light_def(s, 0, 1, roles, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS | ULTRA_DEF_ZERO_MEANS_ZERO, mn2, mx);
    CHECK(ultra_on_slot(&u, s, tick) & ULTRA_EV_DEF, "DEF event");
    ultra_slot_scene(s, 0, 1, 1, 15, 47000, a, b);
    CHECK(ultra_on_slot(&u, s, tick) == ULTRA_EV_SCENE, "SCENE event");
    scene_clear(&stored);
    ultra_scene(&u, &stored, &sc);
    CHECK(sc.mask == 0x1F && sc.fade == 15, "scene covers the light's channels, its fade");
    /* the host restores the same state at boot (SET, no transition): render = scene */
    ultra_slot_set(s, 0, 1, 0, 47000, a, b, 0, 0);
    CHECK(ultra_on_slot(&u, s, tick) & ULTRA_EV_STATE, "STATE event");
    for (t = 0; t < 5; t++) ultra_render(&u, tick + 20u * t, out);
    for (i = 0; i < 5; i++) {
        d = abs((int)out[i] - (int)sc.v[i]);
        worst = d > worst ? d : worst;
    }
    CHECK(worst == 0, "scene == render of the same state (max %d counts)", worst);
    printf("  RGBCCT light, warm 47 %%, channel limit 4 A: scene %u/%u/%u/%u/%u = render, power limit included\n",
           sc.v[0], sc.v[1], sc.v[2], sc.v[3], sc.v[4]);

    /* a defined light whose SCENE has not come yet keeps the stored values; channels of no light are dropped */
    {
        ultra_t u2;
        static const uint8_t r1[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
        ultra_init(&u2, 4);
        ultra_on_header(&u2, tick, 1000000);
        ultra_slot_light_def(s, 0, 1, r1, 1000, 0, mn, mx);
        ultra_on_slot(&u2, s, tick);
        stored.mask = 0x13; stored.v[0] = 1234; stored.v[1] = 555; stored.v[4] = 999; stored.fade = 7;
        ultra_scene(&u2, &stored, &sc2);
        CHECK(sc2.mask == 0x01 && sc2.v[0] == 1234 && sc2.fade == 7, "pending light kept, unowned channels dropped");
        ultra_slot_scene(s, 0, 1, 0, 0, 0, a, b);
        ultra_on_slot(&u2, s, tick);
        ultra_scene(&u2, &stored, &sc2);
        CHECK(sc2.mask == 0, "mode 0: not in the scene");
    }

    /* power cut: the board shows the scene, the host comes back with the same state -> no change at all; with
     * another state -> a monotonic fade over the scene's fade time */
    {
        int k, ok_same = 1, mono = 1;
        for (k = 0; k < 2; k++) {
            ultra_t u3;
            uint16_t prev[5];
            uint8_t up = 0, down = 0;
            uint16_t a2[5] = { 0, 0, 0, 10000, 60000 };
            ultra_init(&u3, 5);
            scene_boot(&r, &sc);
            for (t = 0; t < 300; t++) {                  /* no data yet */
                ultra_render(&u3, t * 20u, v);
                scene_mix(&r, t, v);
                for (i = 0; i < 5; i++) if (v[i] != sc.v[i]) ok_same = 0;
            }
            ultra_on_header(&u3, 300 * 20u, 2000000);
            ultra_slot_config(s, 1, ids, vals, 3);
            ultra_on_slot(&u3, s, 300 * 20u);
            ultra_slot_light_def(s, 0, 1, roles, 2800, ULTRA_DEF_CONSTANT_BRIGHTNESS | ULTRA_DEF_ZERO_MEANS_ZERO, mn2, mx);
            ultra_on_slot(&u3, s, 300 * 20u);
            ultra_slot_set(s, 0, 2, 0, k ? 30000 : 47000, k ? a2 : a, b, 0, 0);
            if (ultra_on_slot(&u3, s, 300 * 20u) & ULTRA_EV_STATE) scene_takeover(&r, 0x1F, 300);
            memcpy(prev, sc.v, sizeof prev);
            for (t = 301; t < 2300; t++) {
                ultra_render(&u3, t * 20u, v);
                scene_mix(&r, t, v);
                for (i = 0; i < 5; i++) {
                    if (!k && v[i] != sc.v[i]) ok_same = 0;
                    if (v[i] > prev[i]) up |= (uint8_t)(1u << i);
                    if (v[i] < prev[i]) down |= (uint8_t)(1u << i);
                }
                memcpy(prev, v, sizeof prev);
            }
            if (up & down) mono = 0;
            if (k && (up | down) == 0) mono = 0;          /* it did move */
        }
        CHECK(ok_same, "same state restored: the outputs never move");
        CHECK(mono, "other state: each channel moves one way only");
        printf("  power cut: scene shown until the host is back; same state = no visible change, other state = %u ms fade\n",
               sc.fade * 100u);
    }
}

static void test_scene_config(void)
{
    settings_t st;
    config_rx_t rx;
    cfg_result_t r;
    printf("power-on scene: config commands\n");
    settings_default(&st);
    config_rx_init(&rx);
    r = config_apply(&rx, &st, CMD_SCENE, 1);
    CHECK(r.event == CFG_ACK && r.value_code == 2 && r.arg == 1, "scene 1: ACK, code 2, main stores the outputs");
    r = config_apply(&rx, &st, CMD_SCENE, 2);
    CHECK(r.event == CFG_NACK, "scene 2: NACK");
    r = config_apply(&rx, &st, CMD_SCENE_FADE, 25);
    CHECK(r.event == CFG_ACK && r.value_code == 0 && r.arg == 25, "fade: ACK only");
}

static void test_scene(void)
{
    test_scene_log(0);
    test_scene_log(1);
    test_scene_rules();
    test_scene_engine();
    test_scene_config();
}
