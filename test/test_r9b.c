/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (the probes, kept as the fixes' regression tests) the receiver's grace / notice
 * edges and the first second's dip. Included by test_soak.c; soak_*_tests.exe r9b <name>. Each fails on the code
 * before its fix (c4719e5 / a6ffb59).
 *   fx3      the grace window and signal-loss mode 3: the power-on scene / effects start at GRACE_MS (1.5 s) and the
 *            2 s loss path doesn't start them again (before: loss_scene ran twice, the fade-in and effects restarted)
 *   release  IDENTIFY 4 (RELEASE) during the power-off notice: ignored, dark to the cut (before: the light came back)
 *   hold     IDENTIFY 3 for 60 s, then IDENTIFY 7 without the cut: the 60 s hold stays (before: cut to the notice's 2 s)
 *   dip1s    a supply dip inside the first second after power-up writes the record (before: dip_ok 0 until the first
 *            energy_second: nothing stored, the power-up lost at a cut) */
static void sc_r9b(const char *c)
{
    printf("soak r9b %s: the probes\n", c);
#if BOARD == BOARD_UR_ANALOG_M
    if (!strcmp(c, "fx3")) { sc_host_effects(); return; }
    if (!strcmp(c, "release") || !strcmp(c, "hold")) {
        uint32_t t, mx;
        r8_scene_image();
        boot(0, 0, 0);
        GRACE_WAIT();
        r8_light(32768u, 3000u);
        if (!strcmp(c, "hold")) {
            cmd_identify(ULTRA_IDENT_DARK, 60);          /* the hub's pin calibration: dark for 60 s */
            run_ms(1000);
            CHECK(out_now[0] == 0u && dk.active, "r9b hold: dark under IDENTIFY 3 (%u)", out_now[0]);
        }
        r8_notice(0x59, 50u);
        run_ms(400);
        CHECK(out_now[0] == 0u && vdip.planned, "r9b %s: dark with the notice (%u)", c, out_now[0]);
        if (!strcmp(c, "release")) {
            cmd_identify(ULTRA_IDENT_RELEASE, 0);
            for (t = mx = 0; t < 400u; t++) {
                run_ms(1);
                mx = out_now[0] > mx ? out_now[0] : mx;
            }
            printf("  RELEASE 0.4 s into the notice: the most out %u, planned %u\n", mx, vdip.planned);
            CHECK(mx == 0u && vdip.planned, "r9b release: IDENTIFY 4 ignored during the notice: dark to the cut "
                  "(the most %u)", mx);
        } else {
            run_ms(2200);                                /* 2.6 s after the notice (no cut), 3.6 s into the hold */
            printf("  2.6 s after the notice (no cut): out %u, planned %u, dark hold active %u\n", out_now[0],
                   vdip.planned, dk.active);
            CHECK(out_now[0] == 0u && dk.active, "r9b hold: the 60 s dark hold stays after the notice's 2 s (out %u, "
                  "active %u)", out_now[0], dk.active);
            run_ms(PLAN_CUT_MS);                         /* the notice forgotten after 10 s */
            CHECK(out_now[0] == 0u && dk.active && !vdip.planned, "r9b hold: ... and after the forgotten notice (out "
                  "%u, active %u)", out_now[0], dk.active);
            run_ms(57000u - PLAN_CUT_MS);                /* the hold's end, 60 s after IDENTIFY 3 */
            CHECK(out_now[0] > 30000u && !dk.active, "r9b hold: the light back at the hold's end (%u)", out_now[0]);
        }
        return;
    }
    if (!strcmp(c, "dip1s")) {
        uint32_t w0;
        boot(0, 0, 0);
        w0 = W_LIFE();
        run_ms(600);                                     /* vdip's reference is there (250 ms hold-off) */
        r8_dip(40u);
        run_ms(200);
        printf("  a dip at 0.6 s: counted %u, written %u\n", vdip.count, W_LIFE() - w0);
        CHECK(vdip.count == 1u && W_LIFE() == w0 + 1u, "r9b dip1s: a dip before the first energy_second writes the "
              "record (the power-up stored; %u written)", W_LIFE() - w0);
        return;
    }
    printf("  (unknown case)\n");
#else
    (void)c;
    printf("  (the receiver's)\n");
#endif
}

static int r9b_scenario(const char *s, int argc, char **argv)
{
    if (!strcmp(s, "r9b")) {
        sc_r9b(argc > 2 ? argv[2] : "fx3");
        return 1;
    }
    return 0;
}
