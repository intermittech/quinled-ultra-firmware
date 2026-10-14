/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Review round 6 (r6-claude-esphome): the power-on scene's take-over against the hub's real start-up and refresh order,
 * on main.c (included by test_soak.c). The hub's numbers: ultra_core.h (quinled-ultra-esphome).
 *
 * r6takeover   The hub's boot on a 50-board pin: it sends ultra_preamble_frames(50) = 202 frames of DARK slots (never
 *              valid) + header first, ~3.5 s, then its commands. 0.6.6 starts the take-over deadline (3 s) at the first
 *              header: it runs out inside the preamble, the scene cross-fades to the engine's outputs (dark: no light
 *              defined yet) and comes back with the hub's LIGHT_DEF / SET after the preamble. Expected: the scene holds
 *              until the board's first slot and the hub's state (= the scene) takes over without a dip.
 * r6takeover5  Five lights with power-on scenes on one board, the hub's boot order per board (first copies first, in the
 *              order queued: the CONFIG pages, then per light LIGHT_DEF, LIGHT_OPT, EFFECT none (setup_state's
 *              stop_effect), SCENE, SCENE_EFFECT, the restored SET). 0.6.6 takes over every channel at the first
 *              light's EFFECT: the other lights fade towards dark until their own LIGHT_DEF / SET come (R6_BOARDS=n:
 *              n boards, default 50). Expected: no channel dips by more than 3 %.
 * r6takeover5d The same with the restored state 8000 over the scene (stored before the last change): each light
 *              cross-fades from its own take-over (one start for all: a light taking over later jumps).
 * r6rejoin5    The same board powers up while the hub runs at rest (a breaker, a swapped board: no preamble), one refresh
 *              item a frame (200 ms) in the hub's order (take_refresh_: CONFIG pages, then per light LIGHT_DEF,
 *              LIGHT_OPT, SET, EFFECT, SCENE, SCENE_EFFECT), from item R6_POS of the cycle (default 30: just after light
 *              4's SET). 0.6.6: the first SET / EFFECT of any light takes over all five; the lights without their
 *              LIGHT_DEF / SET yet go dark for seconds. Expected: no channel dips by more than 3 %. */

#define R6_MAX_BOARDS 50u
static uint32_t r6_boards = R6_MAX_BOARDS;             /* the pin's boards (R6_BOARDS=n in the environment: fewer) */

static uint8_t r6_frame[ULTRA_SLOT_BYTES * R6_MAX_BOARDS + ULTRA_HDR_BYTES];

static void r6_boards_env(void)
{
    const char *e = getenv("R6_BOARDS");
    if (e && atoi(e) >= 1 && atoi(e) <= (int)R6_MAX_BOARDS) {
        r6_boards = (uint32_t)atoi(e);
    }
}

/* one frame of the hub's line as board 0 sees it (every slot + the header trailer), on the wire from now; then the
 * line's reset and the hub's loop gap. slot0: this board's slot (NULL: DARK); the other boards' slots DARK. */
static void r6_hub_frame(const uint8_t *slot0)
{
    uint32_t i, nbits = (ULTRA_SLOT_BYTES * r6_boards + ULTRA_HDR_BYTES) * 8u;
    uint64_t t0 = sim_ns + 7000u, end = t0 + (uint64_t)nbits * 1250u;
    for (i = 0; i < r6_boards; i++) {
        ultra_slot_dark(r6_frame + ULTRA_SLOT_BYTES * i);
    }
    if (slot0) {
        memcpy(r6_frame, slot0, ULTRA_SLOT_BYTES);
    }
    ultra_build_header(r6_frame + ULTRA_SLOT_BYTES * r6_boards, host_us(end));
    wire_frame(r6_frame, nbits, t0);
    while (sim_ns < end + 300000u + 1000000u) {          /* the reset (300 us) and ~1 ms to the next frame */
        run_ticks(1);
    }
}

/* a stored scene on the channels of `mask`, CH1..CH5 at 30000 + 4000 x channel, a 1 s cross-fade */
static void r6_store_scene(uint8_t mask, scene_t *sc)
{
    scene_rec_t rec;
    uint32_t i;
    scene_clear(sc);
    sc->mask = mask;
    for (i = 0; i < 5u; i++) {
        sc->v[i] = (mask & (1u << i)) ? (uint16_t)(30000u + 4000u * i) : 0u;
    }
    sc->fade = 10u;                                      /* default_transition_length 1 s */
    scene_log_pack(sc, 1, &rec);
    memcpy(fl + (SCENE_LOG_ADDR - FL_BASE), &rec, sizeof rec);
}

static void r6_track_low(uint16_t *lo)
{
    uint32_t c;
    for (c = 0; c < 5u; c++) {
        lo[c] = out_now[c] < lo[c] ? out_now[c] : lo[c];
    }
}

/* the largest dip of a scene channel below its scene value, % (and which channel) */
static uint32_t r6_worst_dip(const scene_t *sc, const uint16_t *lo, uint32_t *ch)
{
    uint32_t i, worst = 0;
    *ch = 0;
    for (i = 0; i < 5u; i++) {
        uint32_t dip = (sc->mask & (1u << i)) && lo[i] < sc->v[i] ? (uint32_t)(sc->v[i] - lo[i]) * 100u / sc->v[i] : 0u;
        if (dip > worst) {
            worst = dip;
            *ch = i;
        }
    }
    return worst;
}

static void sc_r6takeover(void)
{
    scene_t sc;
    uint8_t s[32], ids[1] = { ULTRA_CFG_LOSS };
    uint16_t vals[1] = { 0 }, a[5] = { 65535, 65535, 65535, 65535, 65535 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint16_t lo[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint32_t i, k, ch;
    const uint8_t mono[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    const uint16_t z5[5] = { 0, 0, 0, 0, 0 };
    r6_boards_env();
    printf("soak r6takeover: power-on scene on a %u-board pin, the hub's boot preamble (%u frames) first\n", r6_boards,
           ultra_preamble_frames(r6_boards));
    r6_store_scene(0x01u, &sc);                          /* CH1 at 30000 */
    boot(1000u, 20000u, 5000000ull);
    run_ms(1500);                                        /* the hub's ESP32 boots (the scene shows meanwhile) */
    CHECK(out_now[0] == 30000u && srun.hold == 0x01u, "r6takeover: the stored scene shows from power-up (%u)",
          out_now[0]);
    for (k = 0; k < ultra_preamble_frames(r6_boards); k++) {
        r6_hub_frame(0);
        r6_track_low(lo);
    }
    /* the hub's boot commands for this board: 3 CONFIG pages + 4 x 3 for the unused light indices (here a CONFIG and
     * NOPs), LIGHT_DEF, LIGHT_OPT (a NOP here), EFFECT none, SCENE, SCENE_EFFECT (NOPs), SET (the scene's level) */
    for (k = 0; k < 21u; k++) {
        if (k == 0u) {
            ultra_slot_config(s, 7, ids, vals, 1);
        } else if (k == 15u) {
            ultra_slot_light_def(s, 0, 9, mono, 1000, 0, z5, a);
        } else if (k == 17u) {
            ultra_slot_effect(s, 0, 11, ULTRA_FX_NONE, -1000, 0, 0);
        } else if (k == 20u) {
            ultra_slot_set(s, 0, 13, 0, 30000, a, b, 0, -1000);
        } else {
            ultra_slot_nop(s);
        }
        r6_hub_frame(s);
        r6_track_low(lo);
    }
    for (i = 0; i < 150u; i++) {                         /* the take-over's cross-fade (1 s) */
        run_ms(10);
        r6_track_low(lo);
    }
    printf("  lowest CH1 output %u (scene 30000); take-over done: hold %X, fading %X, output %u\n", lo[0], srun.hold,
           srun.fading, out_now[0]);
    CHECK(r6_worst_dip(&sc, lo, &ch) <= 3u, "r6takeover: the scene holds through the hub's preamble and fades over to the "
          "same state without a dip (lowest %u; 0.6.6: the 3 s deadline from the first header runs out inside the "
          "preamble)", lo[0]);
    CHECK(srun.hold == 0u && !srun.fading && out_now[0] == 30000u && ul.light[0].defined,
          "r6takeover: the hub's state took over (%u)", out_now[0]);
    CHECK(events_seen == 0u, "r6takeover: no false events (%X)", events_seen);
}

/* diff: the restored state's level over the scene's (0: the same; r6takeover5d: 8000, the scene stored before the
 * last change): each light cross-fades from its own take-over, no step (0.6.6 + per-light take-over alone: a light
 * taking over later joined the first one's fade at its progress, a jump of ~40 % of the difference) */
static void r6_takeover5_run(const char *name, uint32_t diff)
{
    scene_t sc;
    win_t w;
    uint8_t s[32], ids[1] = { ULTRA_CFG_LOSS };
    uint16_t vals[1] = { 0 }, one[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint16_t lo[5] = { 65535, 65535, 65535, 65535, 65535 };
    const uint16_t z5[5] = { 0, 0, 0, 0, 0 };
    uint32_t k, l, t0, worst, ch, step = 0;
    uint8_t seq = 20;
    r6_boards_env();
    printf("soak %s: five lights with power-on scenes on one board of a %u-board pin, the hub's boot order (state = scene"
           " + %u)\n", name, r6_boards, diff);
    r6_store_scene(0x1Fu, &sc);
    boot(1000u, 20000u, 5000000ull);
    run_ms(1500);
    for (k = 0; k < ultra_preamble_frames(r6_boards); k++) {
        r6_hub_frame(0);
    }
    win_begin(&w);
    for (k = 0; k < 3u; k++) {                           /* the CONFIG pages */
        ultra_slot_config(s, seq++, ids, vals, 1);
        r6_hub_frame(s);
    }
    for (l = 0; l < 5u; l++) {
        uint8_t roles[5] = { 0, 0, 0, 0, 0 };
        roles[l] = ULTRA_ROLE_BRIGHTNESS;
        for (k = 0; k < 6u; k++) {
            switch (k) {
            case 0: ultra_slot_light_def(s, (uint8_t)l, seq++, roles, 1000, 0, z5, one); break;
            case 1: ultra_slot_light_opt(s, (uint8_t)l, seq++, 0, 0, 0, 0, 0); break;
            case 2: ultra_slot_effect(s, (uint8_t)l, seq++, ULTRA_FX_NONE, -1000, 0, 0); break;
            case 3: ultra_slot_scene(s, (uint8_t)l, seq++, 1, 10, (uint16_t)(sc.v[l] + diff), one, one); break;
            case 4: ultra_slot_scene_effect(s, (uint8_t)l, seq++, ULTRA_FX_NONE, 0, 0); break;
            default: ultra_slot_set(s, (uint8_t)l, seq++, 0, (uint16_t)(sc.v[l] + diff), one, one, 0, -1000); break;
            }
            r6_hub_frame(s);
            r6_track_low(lo);
        }
    }
    for (t0 = (uint32_t)(sim_ns / 1000000u); (uint32_t)(sim_ns / 1000000u) - t0 < 1500u;) {
        ultra_slot_nop(s);                               /* the copies (NOP here), the cross-fade's end */
        r6_hub_frame(s);
        r6_track_low(lo);
    }
    win_end();
    worst = r6_worst_dip(&sc, lo, &ch);
    for (k = 0; k < 5u; k++) {
        step = w.maxstep[k] > step ? w.maxstep[k] : step;
    }
    printf("  lowest outputs %u %u %u %u %u (scenes 30000 .. 46000); end: %u %u %u %u %u, hold %X fading %X; largest "
           "1 ms step %u\n", lo[0], lo[1], lo[2], lo[3], lo[4], out_now[0], out_now[1], out_now[2], out_now[3],
           out_now[4], srun.hold, srun.fading, step);
    CHECK(worst <= 3u, "%s: no light dips at the take-over (CH%u by %u %%; 0.6.6 takes over every channel at the first "
          "light's EFFECT, before the others' LIGHT_DEF / SET)", name, ch + 1u, worst);
    CHECK(step <= 40u + diff / 200u, "%s: each light cross-fades from its own take-over, no step (largest %u in 1 ms)",
          name, step);
    CHECK(srun.hold == 0u && !srun.fading && out_now[4] == sc.v[4] + diff, "%s: the hub's states took over", name);
}

static void sc_r6takeover5(void)
{
    r6_takeover5_run("r6takeover5", 0);
}

static void sc_r6takeover5d(void)
{
    r6_takeover5_run("r6takeover5d", 8000);
}

static void sc_r6rejoin5(void)
{
    scene_t sc;
    uint8_t s[32], ids[1] = { ULTRA_CFG_LOSS };
    uint16_t vals[1] = { 0 }, one[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint16_t lo[5] = { 65535, 65535, 65535, 65535, 65535 };
    const uint16_t z5[5] = { 0, 0, 0, 0, 0 };
    const char *e = getenv("R6_POS");
    uint32_t i, k, worst, ch, first = 30u;               /* where in the cycle the board comes in */
    r6_boards = 1;
    if (e && atoi(e) >= 0 && atoi(e) < 33) {
        first = (uint32_t)atoi(e);
    }
    printf("soak r6rejoin5: a five-light board with power-on scenes powers up while the hub runs at rest (from item %u "
           "of the refresh cycle)\n", first);
    r6_store_scene(0x1Fu, &sc);
    boot(1000u, 20000u, 5000000ull);
    run_ms(500);
    for (k = first; k < first + 2u * 33u; k++) {         /* two refresh cycles of 33 items */
        uint32_t pos = k % 33u;
        if (pos < 3u) {
            ultra_slot_config(s, (uint8_t)(100u + pos), ids, vals, 1);
        } else {
            uint32_t l = (pos - 3u) / 6u;
            uint8_t roles[5] = { 0, 0, 0, 0, 0 };
            roles[l] = ULTRA_ROLE_BRIGHTNESS;
            switch ((pos - 3u) % 6u) {
            case 0: ultra_slot_light_def(s, (uint8_t)l, (uint8_t)(10u + l), roles, 1000, 0, z5, one); break;
            case 1: ultra_slot_light_opt(s, (uint8_t)l, (uint8_t)(20u + l), 0, 0, 0, 0, 0); break;
            case 2: ultra_slot_set(s, (uint8_t)l, (uint8_t)(30u + l), 0, sc.v[l], one, one, 0, -32768); break;
            case 3: ultra_slot_effect(s, (uint8_t)l, (uint8_t)(40u + l), ULTRA_FX_NONE, -100000, 0, 0); break;
            case 4: ultra_slot_scene(s, (uint8_t)l, (uint8_t)(50u + l), 1, 10, sc.v[l], one, one); break;
            default: ultra_slot_scene_effect(s, (uint8_t)l, (uint8_t)(60u + l), ULTRA_FX_NONE, 0, 0); break;
            }
        }
        r6_hub_frame(s);
        for (i = 0; i < 199u; i++) {
            run_ms(1);
            r6_track_low(lo);
        }
    }
    worst = r6_worst_dip(&sc, lo, &ch);
    printf("  lowest outputs %u %u %u %u %u (scenes 30000 .. 46000); end %u %u %u %u %u, hold %X fading %X\n", lo[0],
           lo[1], lo[2], lo[3], lo[4], out_now[0], out_now[1], out_now[2], out_now[3], out_now[4], srun.hold,
           srun.fading);
    CHECK(worst <= 3u, "r6rejoin5: no light dips while the refresh brings the lights one after the other (CH%u by %u %%; "
          "0.6.6 takes over all five at the first SET / EFFECT)", ch + 1u, worst);
    CHECK(srun.hold == 0u && !srun.fading && out_now[4] == sc.v[4], "r6rejoin5: the hub's states took over");
    CHECK(events_seen == 0u, "r6rejoin5: no false events (%X)", events_seen);
}

/* r6fxtake  Two lights with power-on candles (CH1, CH2); the host's refresh brings light 0's LIGHT_DEF and SET (no
 *           EFFECT: a host that sends none, or one a frame later) and light 1's only 3 s later. Light 0 takes over at
 *           its SET and its candle ends there (it must not flicker on the host's state); light 1's candle runs on until
 *           its own SET. */
static void sc_r6fxtake(void)
{
    ultra_t u;
    uint8_t s[32], *img;
    uint16_t one[5] = { 65535, 65535, 65535, 65535, 65535 };
    const uint16_t z5[5] = { 0, 0, 0, 0, 0 };
    win_t w0, w1;
    uint32_t i, l;
    printf("soak r6fxtake: two power-on candles, light 0's SET first: its candle ends, light 1's runs on\n");
    ultra_init(&u, 1);
    img = fl + (SCENE_FX_ADDR - FL_BASE);
    for (l = 0; l < 2u; l++) {                           /* the store: light l on CH(l+1), scene 80 %, candle */
        uint8_t roles[5] = { 0, 0, 0, 0, 0 };
        roles[l] = ULTRA_ROLE_BRIGHTNESS;
        ultra_slot_light_def(s, (uint8_t)l, 1, roles, 1000, 0, z5, one);
        (void)ultra_on_slot(&u, s, 0);
        ultra_slot_scene(s, (uint8_t)l, 2, 1, 5, 52428, one, one);
        (void)ultra_on_slot(&u, s, 0);
        ultra_slot_scene_effect(s, (uint8_t)l, 3, ULTRA_FX_CANDLE, P_CANDLE, 4);
        (void)ultra_on_slot(&u, s, 0);
        memcpy(img + ULTRA_FX_HDR + l * ULTRA_HEAD_BYTES, &u.light[l], ULTRA_HEAD_BYTES);
        img[5u + l] = crc8_smbus((const uint8_t *)&u.light[l], ULTRA_HEAD_BYTES);
    }
    img[0] = 0x53; img[1] = 0x46; img[2] = 0x58; img[3] = 0x31;   /* ULTRA_FX_MAGIC, little endian */
    img[4] = 0x03u;
    boot(0, 0, 0);
    CHECK(fx_mask == 0x03u && fx_ch == 0x03u, "r6fxtake: both candles loaded (%X, %X)", fx_mask, fx_ch);
    run_ms(1000);
    r6_boards = 1;
    for (i = 0; i < 20u; i++) {                          /* light 0: LIGHT_DEF, SET (the scene's level), then NOPs */
        uint8_t roles[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
        if (i == 0u) {
            ultra_slot_light_def(s, 0, 11, roles, 1000, 0, z5, one);
        } else if (i == 1u) {
            ultra_slot_set(s, 0, 12, 0, 52428, one, one, 0, -1000);
        } else {
            ultra_slot_nop(s);
        }
        r6_hub_frame(s);
        run_ms(150);
    }
    win_begin(&w0);
    run_ms(1000);
    win_end();
    printf("  after light 0's SET: CH1 %u changes in 1 s (output %u), CH2 %u changes\n", w0.chg[0], out_now[0],
           w0.chg[1]);
    CHECK(w0.chg[0] == 0u && out_now[0] == 52428u, "r6fxtake: light 0's candle ended with its take-over (%u changes)",
          w0.chg[0]);
    CHECK(w0.chg[1] > 100u, "r6fxtake: light 1's candle runs on until its own SET (%u changes)", w0.chg[1]);
    for (i = 0; i < 20u; i++) {                          /* light 1 */
        uint8_t roles[5] = { 0, ULTRA_ROLE_BRIGHTNESS, 0, 0, 0 };
        if (i == 0u) {
            ultra_slot_light_def(s, 1, 21, roles, 1000, 0, z5, one);
        } else if (i == 1u) {
            ultra_slot_set(s, 1, 22, 0, 52428, one, one, 0, -1000);
        } else {
            ultra_slot_nop(s);
        }
        r6_hub_frame(s);
        run_ms(150);
    }
    win_begin(&w1);
    run_ms(1000);
    win_end();
    CHECK(w1.chg[1] == 0u && out_now[1] == 52428u && fx_ch == 0u && !ul.light[0].scene_run && !ul.light[1].scene_run,
          "r6fxtake: light 1's candle ended with its take-over (%u changes, fx_ch %X)", w1.chg[1], fx_ch);
    CHECK(events_seen == 0u, "r6fxtake: no false events (%X)", events_seen);
}

/* r6takeover0  default_transition_length: 0s (the scene's fade 0) and a SET that starts later than it arrives (the
 *              hub's lead: a 1-board pin at the ESP's boot, every pin after a power_supply switch-on: replay at the
 *              preamble's end + the stream lead): 0.6.6 takes over at the SET's arrival, so the light shows its state
 *              before the SET (dark after a power-up) until the SET starts: a blink. Expected: the scene until the SET
 *              starts, then the state. */
static void sc_r6takeover0(void)
{
    scene_t sc;
    scene_rec_t rec;
    uint8_t s[32];
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, lo[5] = { 65535, 65535, 65535, 65535, 65535 };
    const uint8_t mono[5] = { ULTRA_ROLE_BRIGHTNESS, 0, 0, 0, 0 };
    const uint16_t z5[5] = { 0, 0, 0, 0, 0 };
    uint32_t k, ch;
    r6_boards = 1;
    printf("soak r6takeover0: the scene's fade 0, the SET starting 60 ms after it arrives\n");
    scene_clear(&sc);
    sc.mask = 0x01u;
    sc.v[0] = 30000u;
    sc.fade = 0u;
    scene_log_pack(&sc, 1, &rec);
    memcpy(fl + (SCENE_LOG_ADDR - FL_BASE), &rec, sizeof rec);
    boot(1000u, 20000u, 5000000ull);
    run_ms(1000);
    for (k = 0; k < ultra_preamble_frames(1); k++) {
        r6_hub_frame(0);
    }
    for (k = 0; k < 40u; k++) {
        if (k == 0u) {
            ultra_slot_light_def(s, 0, 9, mono, 1000, 0, z5, a);
        } else if (k == 1u) {
            ultra_slot_set(s, 0, 13, 0, 30000, a, a, 0, 60);   /* the same level, starting 60 ms later */
        } else {
            ultra_slot_nop(s);
        }
        r6_hub_frame(s);
        r6_track_low(lo);
    }
    run_ms(200);
    r6_track_low(lo);
    printf("  lowest CH1 %u (scene 30000), end %u\n", lo[0], out_now[0]);
    CHECK(r6_worst_dip(&sc, lo, &ch) <= 3u && out_now[0] == 30000u, "r6takeover0: no blink while the SET waits for its "
          "start (lowest %u; 0.6.6 took over at its arrival: the light's state before it, dark)", lo[0]);
}
