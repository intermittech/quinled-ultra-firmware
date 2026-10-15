/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (review round 9, code size item 10) one settings descriptor table (config.h setdesc_t, config.c settings_desc) replaced
 * five: config.c settings_sanitize's lim[12][3] and cmd_field[13][4], main.c ultra_overlay's ovl[7][4], cli.c get_key's
 * ofs[20] and status_part's so[11]. The five below are 3732a57's, verbatim; each is derived from the one table the way
 * its reader now reads it. Included by test_main.c. */

/* the entry a config-frame command sets, as config_apply / get_key find it (0 = none) */
static const setdesc_t *r9_by_cmd(uint32_t cmd)
{
    uint32_t i;
    for (i = 0; i < SETD_SHOWN; i++) {
        if ((settings_desc[i].cmd & ~SETD_NO_CODE) == cmd) {
            return &settings_desc[i];
        }
    }
    return 0;
}

static void test_r9_setdesc(void)
{
    /* config.c settings_sanitize (3732a57) */
    static const uint8_t lim[12][3] = {
        { offsetof(settings_t, protocol), 0, PMODE_ULTRA },
        { offsetof(settings_t, pwm_preset), 0, PWM_PRESET_COUNT - 1u },
        { offsetof(settings_t, dither_mode), 0, 2 },
        { offsetof(settings_t, dither_thresh), 1, 255 },
        { offsetof(settings_t, feedback_ch), 0, 2 },
        { offsetof(settings_t, stagger), 0, STAGGER_MODES - 1u },
        { offsetof(settings_t, dim_floor), 0, 2 },
        { offsetof(settings_t, min_pulse), 5, 100 },
        { offsetof(settings_t, signal_loss), 0, 3 },   /* (0.6.9, B10: 3 = scene) */
        { offsetof(settings_t, smoothing), 0, 2 },
        { offsetof(settings_t, gamma), 0, 2 },
        { offsetof(settings_t, last_proto), 0, PROTO_ULTRA },
    };
    /* config.c config_apply (3732a57) */
    static const uint8_t cmd_field[CMD_MIN_PULSE][4] = {   /* [cmd - 1]: offset (0xFF: not a byte setting), min, max, code */
        { offsetof(settings_t, protocol), 0, 3, 1 },
        { offsetof(settings_t, pwm_preset), 0, PWM_PRESET_COUNT - 1u, 1 },
        { offsetof(settings_t, dither_mode), 0, 2, 1 },
        { offsetof(settings_t, dither_thresh), 1, 255, 0 },
        { offsetof(settings_t, feedback_ch), 0, 2, 1 },
        { offsetof(settings_t, stagger), 0, STAGGER_MODES - 1u, 1 },
        { offsetof(settings_t, dim_floor), 0, 2, 1 },
        { offsetof(settings_t, signal_loss), 0, 3, 1 },
        { 0xFF, 0, 0, 0 },                                /* CMD_IDENTIFY */
        { 0xFF, 0, 0, 0 },                                /* CMD_FACTORY_RESET */
        { offsetof(settings_t, smoothing), 0, 2, 1 },
        { offsetof(settings_t, gamma), 0, 2, 1 },
        { offsetof(settings_t, min_pulse), 5, 100, 0 },
    };
    /* main.c ultra_overlay (3732a57) */
    static const uint8_t ovl[7][4] = {   /* id, offset, lowest, highest */
        { ULTRA_CFG_DITHER, offsetof(settings_t, dither_mode), 0, 2 },
        { ULTRA_CFG_DITHER_THRESH, offsetof(settings_t, dither_thresh), 1, 255 },
        { ULTRA_CFG_DIM_FLOOR, offsetof(settings_t, dim_floor), 0, 2 },
        { ULTRA_CFG_MIN_PULSE, offsetof(settings_t, min_pulse), 5, 100 },
        { ULTRA_CFG_STAGGER, offsetof(settings_t, stagger), 0, STAGGER_MODES - 1 },
        { ULTRA_CFG_LOSS, offsetof(settings_t, signal_loss), 0, 3 },
        { ULTRA_CFG_FEEDBACK, offsetof(settings_t, feedback_ch), 0, 2 },
    };
    /* cli.c get_key (3732a57; SF: the field's offset, SX: not a settings_t field) */
#define SF(f) ((uint8_t)offsetof(settings_t, f))
#define SX 0xFFu
    static const uint8_t ofs[CMD_SOFT_START] = {   /* [cmd - 1] */
        SF(protocol), SF(pwm_preset), SF(dither_mode), SF(dither_thresh), SF(feedback_ch), SF(stagger),
        SF(dim_floor), SF(signal_loss), SF(gamma), SF(gamma), SF(smoothing), SF(gamma), SF(min_pulse), SX, SX,
        SF(gamma), SF(gamma), SX, SX, SX,  /* (IDENTIFY / FACTORY_RESET / PERIOD_*: not keys; gamma, as the default) */
    };
#undef SF
#undef SX
    /* cli.c status_part (3732a57) */
        static const uint8_t so[11] = {
            offsetof(settings_t, protocol), offsetof(settings_t, pwm_preset), offsetof(settings_t, dither_mode),
            offsetof(settings_t, dither_thresh), offsetof(settings_t, dim_floor), offsetof(settings_t, min_pulse),
            offsetof(settings_t, stagger), offsetof(settings_t, signal_loss), offsetof(settings_t, feedback_ch),
            offsetof(settings_t, smoothing), offsetof(settings_t, gamma) };
    uint32_t i, j, cmd;
    printf("r9 setdesc: the one settings descriptor table against the five it replaced\n");
    CHECK(SETD_N == 12u && SETD_SHOWN == 11u && SETD_OVL == 2u && SETD_OVL_N == 7u, "the table's shape");
    /* settings_sanitize: every field of lim once, with its range; the preset (0xFF allowed) is entry 1 in both */
    for (i = 0; i < 12u; i++) {
        for (j = 0; j < SETD_N && settings_desc[j].ofs != lim[i][0]; j++) {
        }
        CHECK(j < SETD_N && settings_desc[j].lo == lim[i][1] && settings_desc[j].hi == lim[i][2],
              "lim[%u] (offset %u) in the table with its range", (unsigned)i, lim[i][0]);
        CHECK((i == 1u) == (j == 1u), "lim[%u]: the preset's entry", (unsigned)i);
    }
    for (i = 0; i < SETD_N; i++) {
        for (j = 0; j < SETD_N; j++) {
            CHECK(i == j || settings_desc[i].ofs != settings_desc[j].ofs, "entry %u: its field once", (unsigned)i);
        }
    }
    /* config_apply: the commands 1..CMD_MIN_PULSE as cmd_field had them, no other command finds an entry */
    for (cmd = 0; cmd < 256u; cmd++) {
        const setdesc_t *d = r9_by_cmd(cmd);
        const uint8_t *f = (cmd - 1u) < CMD_MIN_PULSE ? cmd_field[cmd - 1u] : 0;
        if (f && f[0] != 0xFFu) {
            CHECK(d && d->ofs == f[0] && d->lo == f[1] && d->hi == f[2] && ((d->cmd & SETD_NO_CODE) == 0u) == (f[3] != 0u),
                  "cmd %u: cmd_field's field, range and value code", (unsigned)cmd);
        } else {
            CHECK(!d, "cmd %u: no byte setting", (unsigned)cmd);
        }
    }
    /* ultra_overlay: entries SETD_OVL .. + SETD_OVL_N - 1 are ovl in order, no other entry has a CONFIG id */
    for (i = 0; i < SETD_N; i++) {
        const setdesc_t *d = &settings_desc[i];
        if (i - SETD_OVL < SETD_OVL_N) {
            const uint8_t *o = ovl[i - SETD_OVL];
            CHECK(d->cfg == o[0] && d->ofs == o[1] && d->lo == o[2] && d->hi == o[3], "ovl[%u]", (unsigned)(i - SETD_OVL));
        } else {
            CHECK(d->cfg == 0u, "entry %u: no CONFIG id", (unsigned)i);
        }
    }
    /* status_part: the console's order */
    for (i = 0; i < 11u; i++) {
        CHECK(settings_desc[i].ofs == so[i], "so[%u]", (unsigned)i);
    }
    /* get_key: ofs by command; a command without an entry took the gamma field (its default) or the switch (SX) */
    for (cmd = 1; cmd <= CMD_SOFT_START; cmd++) {
        const setdesc_t *d = r9_by_cmd(cmd);
        uint8_t o = ofs[cmd - 1u];
        if (d) {
            CHECK(d->ofs == o, "ofs[%u]: the entry's field", (unsigned)(cmd - 1u));
        } else {
            CHECK(o == 0xFFu || o == (uint8_t)offsetof(settings_t, gamma), "ofs[%u]: SX or gamma", (unsigned)(cmd - 1u));
        }
    }
    CHECK(sizeof(settings_t) <= 255u, "offsets fit a byte");
}
