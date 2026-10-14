/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (0.6.7, Quindor) the release console (core/config.h CONSOLE_FULL 0): it only sends. Included by test_soak.c in the
 * release console's builds (test\build_host.cmd: soak_rel_tests.exe, soak_rc_rel_tests.exe), soak_tests.exe <name>.
 *
 *   conout   boot, an Ultra host's light at 40 %, the supply at 24 V, a short (the trip and its retry) and a supply dip:
 *            65 s of the stream. The banner with its notice at once, the full status at boot once the live values
 *            are valid (after the first live line: the supply, the MCU temperature, the current's zero, the outputs;
 *            every part: a key of each), a line of live values every second, the full status again at the minute,
 *            the UART never overfilled
 *            (tx_blocks); the capture against its golden (CRC-32 and length, the version masked).
 *            SOAK_CON_DUMP=<file> writes the capture as sent.
 *   conin    bytes on TP6 (the full console's RX) change nothing: no reply, no setting, the stream as before
 *   conjig   TP6 held low: test mode; the stream pauses (no status part, no live line) and only the jig lines come,
 *            in their format; released: test mode off and the stream again
 *   condip   a supply dip while the UART sends (uart_busy): the lifetime record's dip write at once
 * The samples (SOAK_CON_DUMP=<file>: the console's bytes as sent on TP5): conout; consamplejig (the release: the
 * stream, then TP6 held low: two PASS cycles, G's load disconnected: a FAIL, released); consamplefull (the full console,
 * CONSOLE_FULL 1: `status` and `get` with a light at 40 %). */
#if !CONSOLE_FULL

#if BOARD == BOARD_DP_RX                    /* (the capture's golden, by board) */
#define CONOUT_GOLDEN_CRC 0x8132a408u
#define CONOUT_GOLDEN_LEN 11380u
#else
#define CONOUT_GOLDEN_CRC 0x9ce67d6du
#define CONOUT_GOLDEN_LEN 11521u
#endif

static uint32_t con_crc32(const char *s, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu, k, i;
    for (i = 0; i < n; i++) {
        c ^= (uint8_t)s[i];
        for (k = 0; k < 8u; k++) {
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        }
    }
    return ~c;
}

static uint32_t con_count(const char *s, const char *what)
{
    uint32_t n = 0;
    while ((s = strstr(s, what)) != NULL) {
        n++;
        s += strlen(what);
    }
    return n;
}

/* the keys of every status part a board always prints (cli.c status_part 0-3, 6, 7) */
static const char *const con_keys[] = {
    "uptime=", "reset=", "stack=", "protocol=", "frames=", "fps=", "decoded=", "cpu=", "config_accepted=", "pwm_hz=",
    "pwm_steps=", "flash_saves=", "out R/G/B/W1/W2=", "current=", "current_peak=", "current_zero=", "current_limit=",
    "current_scale=", "current_trips=", "current_state=", "channel R/G/B/W1/W2=", "input=", "energy=", "tp8=",
    "test_mode=", "power_state=",
};

static void sc_conout(void)
{
    uint32_t i, n, lines, crc, ver_n;
    char *cap, *p;
    const char *st1, *st2, *v;
    size_t vl;
    printf("soak conout: the release console's stream: boot, a light at 40 %%, a short, a dip; 65 s\n");
    soak_wfi = 1;                                        /* the WFI's model (r5cpu): each pass 4 % of a tick, so the */
    soak_busy = 288u;                                    /* stream's cpu figure is a board's, not the soak's 100 % */
    boot(0, 0, 0);
    host_on = 1;                                         /* an Ultra host: a light at 40 % */
    cmd_def(0, 0, 3, 1000);
    cmd_set(0, 26214, 0, 0);
    run_ms(12000);
    short_ticks = 3;                                     /* a short: the fast trip, the retry 1 s later */
    expect_trips = 1;
    run_ms(4000);
    supply_mv = 19000u;                                  /* a 40 ms supply dip */
    run_ms(40);
    supply_mv = 24000u;
    run_ms(65000 - 16040);
    printf("  %u bytes in 65 s, %u blocked\n", con_out_n, tx_blocks);
    CHECK(tx_blocks == 0u, "conout: the stream never waits for the UART (%u bytes past its buffers)", tx_blocks);
    CHECK(strncmp(con_out, "QuinLED ", 8) == 0 && strstr(con_out, ", (C) 2026 QuinLED, GPL-3.0, github.com/intermittech/"
          "quinled-ultra-firmware\r\n") != NULL, "conout: the banner with its notice first (%.60s)", con_out);
    CHECK(con_count(con_out, "QuinLED ") == 3u, "conout: the banner, the full status at boot and at the minute (%u)",
          con_count(con_out, "QuinLED "));
    st1 = strstr(con_out + 8, "QuinLED ");               /* the boot's status: after a live line, with the live */
    st2 = st1 ? strstr(st1 + 8, "QuinLED ") : NULL;      /* values valid (the averages have run); the minute's */
    v = strstr(con_out, "uptime=");                      /* (the first status's keys: the boot's) */
    CHECK(v && strstr(con_out, "  k ") != NULL && strstr(con_out, "  k ") < v, "conout: the boot's status after the "
          "first live line");
    v = strstr(con_out, "supply=");
    CHECK(v && strncmp(v, "supply=0.00 ", 12) != 0, "conout: the boot's status: the supply measured (%.20s)", v);
    v = strstr(con_out, "mcu_temp=");
    CHECK(v && strncmp(v, "mcu_temp=n/a", 12) != 0, "conout: the boot's status: the MCU temperature (%.20s)", v);
    v = strstr(con_out, "current_zero=");
    CHECK(v && strncmp(v, "current_zero=0.00 ", 18) != 0, "conout: the boot's status: the current's zero (%.24s)", v);
    v = strstr(con_out, "out R/G/B/W1/W2=");
    CHECK(v && strncmp(v, "out R/G/B/W1/W2=40.0/40.0/40.0/", 31) == 0, "conout: the boot's status: the outputs (%.40s)",
          v);
    for (i = 0; i < sizeof con_keys / sizeof con_keys[0]; i++) {
        n = con_count(con_out, con_keys[i]);
        CHECK(n == 2u, "conout: the status's %s in both (%u)", con_keys[i], n);
    }
    CHECK(strstr(con_out, "event 1=") != NULL && st2 && strstr(st2, "event 1=") != NULL, "conout: the event log (the "
          "status's last part) at the minute");
    /* the live lines between: each ends "k ... %  dips N\r\n" or "k ... %\r\n", one a second */
    lines = con_count(con_out, "  k ");
    printf("  %u live lines\n", lines);
    CHECK(lines >= 60u && lines <= 64u, "conout: a live line every second between (%u)", lines);
    CHECK(strstr(con_out, "type help") == NULL && strstr(con_out, "error:") == NULL, "conout: no prompt, no error "
          "reply");
    /* the golden: the capture with the version masked (a release changes it) */
    cap = (char *)malloc(2u * con_out_n + 16u);        /* ("<version>" may be longer than the version) */
    vl = strlen(stat.version);
    ver_n = 0;
    for (i = 0, p = cap; i < con_out_n;) {
        if (vl && strncmp(con_out + i, stat.version, vl) == 0) {
            memcpy(p, "<version>", 9);
            p += 9;
            i += (uint32_t)vl;
            ver_n++;
        } else {
            *p++ = con_out[i++];
        }
    }
    crc = con_crc32(cap, (uint32_t)(p - cap));
    printf("  golden: %u bytes (version masked %u times), CRC-32 %08x\n", (uint32_t)(p - cap), ver_n, crc);
    CHECK(crc == CONOUT_GOLDEN_CRC && (uint32_t)(p - cap) == CONOUT_GOLDEN_LEN, "conout: the capture against its "
          "golden (%08x / %u, want %08x / %u)", crc, (uint32_t)(p - cap), CONOUT_GOLDEN_CRC, CONOUT_GOLDEN_LEN);
    free(cap);
}

static void sc_conin(void)
{
    settings_t s0;
    uint32_t w0, n0;
    printf("soak conin: bytes typed on TP6: nothing happens\n");
    boot(0, 0, 0);
    run_ms(6000);                                        /* (the boot's status out: at the latest at STREAM_BOOT_S) */
    s0 = set;
    w0 = W_SETTINGS();
    n0 = con_out_n;
    con_in_r = con_in_w = 0;
    strcpy(con_in, "set dither 1\rstatus\rdefaults\rhelp\r");
    con_in_w = (uint32_t)strlen(con_in);
    run_ms(5000);
    CHECK(memcmp(&s0, &set, sizeof s0) == 0 && W_SETTINGS() == w0, "conin: no setting changed, nothing written");
    CHECK(strstr(con_out + n0, "ok\r\n") == NULL && strstr(con_out + n0, "error:") == NULL &&
          strstr(con_out + n0, "QuinLED ") == NULL, "conin: no reply (only the stream's live lines)");
    CHECK(con_count(con_out + n0, "  k ") >= 4u, "conin: the stream as before (%u lines)", con_count(con_out + n0, "  k "));
}

static void sc_conjig(void)
{
    uint32_t n0, n1, jig;
    const char *j;
    printf("soak conjig: TP6 held low: test mode, the stream pauses, the jig lines only\n");
    boot(0, 0, 0);
    run_ms(3000);
    soak_tp6_low = 1;
    run_ms(200);                                         /* (TJ_ENTER_MS) */
    CHECK(tj.active, "conjig: test mode on");
    n0 = con_out_n;
    run_ms(8000);                                        /* five cycles of the pattern */
    jig = con_count(con_out + n0, "jig cycle=");
    printf("  %u jig lines in 8 s; %s", jig, (j = strstr(con_out + n0, "jig cycle=")) ? j : "(none)\n");
    CHECK(jig >= 4u && con_count(con_out + n0, "  k ") == 0u && strstr(con_out + n0, "uptime=") == NULL,
          "conjig: the jig lines only, the stream paused (%u jig lines, %u live lines)", jig,
          con_count(con_out + n0, "  k "));
    CHECK(j != NULL && strstr(j, " fail=") != NULL && strstr(j, " a=") != NULL && strstr(j, " supply=") != NULL &&
          strstr(j, " uid=") != NULL && strstr(j, " fw=") != NULL, "conjig: the jig line's fields");
    soak_tp6_low = 0;
    run_ms(200);
    n1 = con_out_n;
    run_ms(3000);
    CHECK(!tj.active && strstr(con_out + n0, "test mode: off") != NULL && con_count(con_out + n1, "  k ") >= 2u,
          "conjig: released: test mode off, the stream again (%u lines)", con_count(con_out + n1, "  k "));
}

static void sc_condip(void)
{
    uint32_t w0, k, busy = 0, at = 0, dips0;
    printf("soak condip: a supply dip while the UART sends a line: the dip write at once\n");
    soak_busy_model = 1;                                 /* uart_busy() while bytes are queued, as on the board */
    boot(0, 0, 0);
    run_ms(12000);                                       /* (past LIFE_PREP_S) */
    jump_ms(LIFE_DIP_S * 1000u + 10000u);                /* the lifetime record's dip write: LIFE_DIP_S since the last */
    run_ms(1500);
    for (k = 0; k < 2000u && tx_q < 64u; k++) {
        run_ms(1);                                       /* to a live line on its way (most of it still queued) */
    }
    w0 = W_LIFE();
    dips0 = vdip.count;
    busy = tx_q;
    supply_mv = 19000u;
    for (k = 1; k <= 40u && W_LIFE() == w0; k++) {
        run_ms(1);
        if (W_LIFE() != w0) {
            at = k;
            busy = busy && tx_q != 0u;                   /* the line still on its way when the write was done */
        }
    }
    supply_mv = 24000u;
    printf("  the dip counted %u, the dip write %u ms into it, the UART still sending then %u (%u bytes queued)\n",
           vdip.count - dips0, at, busy != 0u, tx_q);
    CHECK(vdip.count == dips0 + 1u && at != 0u && busy != 0u, "condip: the dip write while the UART sends (dip %u, "
          "write at %u ms, busy %u)", vdip.count - dips0, at, busy != 0u);
    run_ms(2000);
}

static void sc_consamplejig(void)
{
    uint32_t g = ld_ma[1];
    printf("soak consamplejig: the stream, then TP6 held low: PASS, PASS, G disconnected: FAIL\n");
    soak_wfi = 1;
    soak_busy = 288u;
    boot(0, 0, 0);
    run_ms(4500);
    soak_tp6_low = 1;
    run_ms(3200);                                        /* (the entry, two cycles of 1.5 s) */
    ld_ma[1] = 0u;                                       /* G's load disconnected */
    run_ms(1600);
    ld_ma[1] = g;
    soak_tp6_low = 0;
    run_ms(3000);
}
#else
static void sc_consamplefull(void)
{
    printf("soak consamplefull: the full console (CONSOLE_FULL 1): status and get, a light at 40 %%\n");
    soak_wfi = 1;
    soak_busy = 288u;
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 3, 1000);
    cmd_set(0, 26214, 0, 0);
    run_ms(20000);
    console("status");
    console("get");
}
#endif

static int r6o_scenario(const char *s)
{
#if CONSOLE_FULL
    if (!strcmp(s, "consamplefull")) {
        sc_consamplefull();
        return 1;
    }
    return 0;
#else
    if (!strcmp(s, "conout")) {
        sc_conout();
    } else if (!strcmp(s, "conin")) {
        sc_conin();
    } else if (!strcmp(s, "conjig")) {
        sc_conjig();
    } else if (!strcmp(s, "condip")) {
        sc_condip();
    } else if (!strcmp(s, "consamplejig")) {
        sc_consamplejig();
    } else {
        return 0;
    }
    return 1;
#endif
}
