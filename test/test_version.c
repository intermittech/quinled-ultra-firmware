/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The firmware version display (wiki: Console): version text <-> word (core/fwver.c, the
 * rules of tools/release/qdu.py and the ESPHome component's fw_version.h), the blink code (ack_start_version), the
 * console banner and `version`, the identify argument. Included by test_main.c after test_features.c (console
 * helpers). The Ultra side (IDENTIFY 5 taken, 6-255 ignored) is in test_ultra.c. */
#include "../src/core/fwver.h"

static uint32_t ver_ms(const ack_t *a)
{
    uint32_t i, t = 0;
    for (i = 0; i < a->nseg; i++) {
        t += a->ms[i];
    }
    return t;
}

static int ver_segs(const ack_t *a, const uint16_t *want, uint32_t n)
{
    uint32_t i;
    if (a->nseg != n) {
        return 0;
    }
    for (i = 0; i < n; i++) {
        if (a->ms[i] != want[i]) {
            return 0;
        }
    }
    return 1;
}

static uint8_t ver_apply_cmd, ver_apply_arg;
static cfg_result_t ver_apply(uint8_t cmd, uint8_t arg)
{
    ver_apply_cmd = cmd;
    ver_apply_arg = arg;
    return config_apply(&cli_rx, &cli_set, cmd, arg);
}

static void test_version(void)
{
    static const char *const order[] = { "0.5.0", "0.5.1-alpha.0", "0.5.1-alpha.12", "0.5.1-beta.1", "0.5.1-rc.2",
                                         "0.5.1", "0.6.0-alpha.1", "1.0.0", "255.255.255" };
    static const char *const bad[] = { "", "0.4", "1.2.3.4", "1.2.256", "1.2.3-gamma.1", "1.2.3-beta", "1.2.3-beta.64",
                                       "1.2.3 ", "v1.2.x", "1.2.3-alpha.1x" };
    char txt[FWVER_TEXT_MAX];
    uint32_t i, w, prev = 0;
    ack_t a;
    printf("firmware version display (core/fwver.c, ack_start_version, console)\n");

    /* text <-> word, the same cases as the ESPHome component's test (test_sender.cpp) */
    for (i = 0; i < sizeof order / sizeof order[0]; i++) {
        w = fwver_parse(order[i]);
        fwver_format(w, txt);
        CHECK(w != 0u && strcmp(txt, order[i]) == 0, "round trip %s -> %s", order[i], txt);
        CHECK(i == 0 || w > prev, "%s sorts after %s", order[i], order[i - 1]);
        prev = w;
    }
    CHECK(fwver_parse("v0.6.0-beta.1") == 0x00060041u, "v prefix, beta.1 = 0x41");
    CHECK(fwver_parse("0.6.0") == 0x000600FFu, "stable = 0xFF");
    for (i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        CHECK(fwver_parse(bad[i]) == 0u, "rejects '%s'", bad[i]);
    }
    fwver_format(0, txt);
    CHECK(txt[0] == 0, "word 0: no text");

    /* the blink code */
    {
        static const uint16_t beta[] = { 100, 100, 100, 100, 100, 900,   /* 3 fast, 800 ms */
                                         200, 1200,                      /* major 0: 1 short */
                                         700, 300, 200, 300, 200, 1200,  /* minor 6: 1 long + 2 short */
                                         200, 1200,                      /* patch 0 */
                                         200, 300, 200, 1000 };          /* beta: 2 short, 1 s off */
        static const uint16_t stable[] = { 100, 100, 100, 100, 100, 900,
                                           200, 300, 200, 1200,                      /* major 1: 2 short */
                                           200, 300, 200, 300, 200, 1200,            /* minor 2: 3 short */
                                           200, 300, 200, 300, 200, 300, 200, 1000 };/* patch 3: 4 short, 1 s off */
        static const uint16_t dev[] = { 100, 100, 100, 100, 100, 900, 1500, 1000 };
        ack_start_version(&a, fwver_parse("0.6.0-beta.1"));
        CHECK(ver_segs(&a, beta, sizeof beta / sizeof beta[0]) && a.active, "0.6.0-beta.1: 3 fast, 1, 1 long 2 short, 1, "
              "beta 2 short (%u segments, %.1f s)", a.nseg, ver_ms(&a) / 1000.0);
        ack_start_version(&a, fwver_parse("1.2.3"));
        CHECK(ver_segs(&a, stable, sizeof stable / sizeof stable[0]), "1.2.3: 2, 3, 4 short, no pre-release part");
        ack_start_version(&a, 0);
        CHECK(ver_segs(&a, dev, sizeof dev / sizeof dev[0]), "development build: 3 fast + one long flash");
        ack_start_version(&a, fwver_parse("255.255.255-rc.63"));
        CHECK(a.nseg <= ACK_MAX_SEG && a.ms[a.nseg - 1u] == 1000u && ver_ms(&a) < 40000u,
              "the longest code fits (%u of %u segments, %.1f s)", a.nseg, (unsigned) ACK_MAX_SEG, ver_ms(&a) / 1000.0);
        {
            uint8_t on;
            uint32_t ms = 0;
            ack_start_version(&a, fwver_parse("0.6.0-beta.1"));
            while (ack_tick(&a, &on)) {
                ms++;
            }
            CHECK(ms + 1u == ver_ms(&a) || ms == ver_ms(&a), "plays for its length and ends (%u ms)", ms);
        }
    }

    /* the console: banner (status first lines, boot) and `version` = the banner + identify argument 1 */
    {
        cli_t c;
        cli_status_t st;
        memset(&c, 0, sizeof c);
        memset(&st, 0, sizeof st);
        settings_default(&cli_set);
        config_rx_init(&cli_rx);
        c.write = cap_write;
        c.apply = ver_apply;
        c.save_now = cap_save;
        c.set = &cli_set;
        c.st = &st;
        st.steps = 5760;                   /* status prints the PWM frequency (144 MHz / steps) */
        st.version = "0.4 (development build)";
        run_cli(&c, "version\r");
        CHECK(strstr(cli_out, "QuinLED dig2analog-ultra firmware v0.4 (development build), Copyright (C) 2026 QuinLED, "
                              "GPL-3.0, source: github.com/intermittech/quinled-ultra-firmware\r\n") != 0 &&
              strstr(cli_out, "app standalone build") != 0, "banner of a standalone development build");
        CHECK(ver_apply_cmd == CMD_IDENTIFY && ver_apply_arg == CFG_IDENTIFY_VERSION, "`version` blinks the version");
        st.version = "0.6.0-beta.1";
        st.bl_version = 1;
        st.hw_id = 1;
        st.hw_rev = 1;
        st.app_state = 1;
        memcpy(st.bl_build, "1a2b3c4d", 9);
        run_cli(&c, "status\r");
        CHECK(strstr(cli_out, "QuinLED dig2analog-ultra firmware v0.6.0-beta.1, Copyright (C) 2026 QuinLED") == cli_out &&
              strstr(cli_out, "bootloader rev 1 (build 1a2b3c4d), hardware 1 revision 1, app trial (not confirmed yet)") != 0,
              "status starts with the banner and the bootloader line");
        st.app_state = 2;
        run_cli(&c, "version\r");
        CHECK(strstr(cli_out, "app confirmed\r\n") != 0, "confirmed app");
        run_cli(&c, "help\r");
        CHECK(strstr(cli_out, "| version") != 0, "help lists version");
    }

    /* the config frame: identify with argument 1 (CFG_IDENTIFY_VERSION) reaches main.c as such */
    {
        cfg_result_t r;
        config_rx_init(&cli_rx);
        r = config_apply(&cli_rx, &cli_set, CMD_IDENTIFY, CFG_IDENTIFY_VERSION);
        CHECK(r.event == CFG_IDENTIFY && r.arg == CFG_IDENTIFY_VERSION, "config frame identify 1: the version blink");
        r = config_apply(&cli_rx, &cli_set, CMD_IDENTIFY, 0);
        CHECK(r.event == CFG_IDENTIFY && r.arg == 0u, "identify 0: the identify blink, as before");
    }
}
