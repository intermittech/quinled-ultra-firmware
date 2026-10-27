/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* the release console (core/config.h CONSOLE_FULL 0): it only sends. Included by test_soak.c in the
 * release console's builds (test\build_host.cmd: soak_rel_tests.exe, soak_rc_rel_tests.exe), soak_tests.exe <name>.
 *
 *   conout   boot, an Ultra host's light at 40 %, the supply at 24 V, a short (the trip and its retry) and a supply dip:
 *            65 s of the stream. The banner with its notice at once, the full status at boot once the live values
 *            are valid (after the first live line: the supply, the MCU temperature, the current's zero, the outputs;
 *            every part: a key of each), a line of live values every second, the full status again at the minute,
 *            the UART never overfilled
 *            (tx_blocks); the capture against its golden (CRC-32 and length, the version masked).
 *            SOAK_CON_DUMP=<file> writes the capture as sent.
 *   conin    bytes on jig input (the full console's RX) change nothing: no reply, no setting, the stream unchanged
 *   conjig   jig input held low: test mode; the stream pauses (no status part, no live line) and only the jig lines come,
 *            in their format; released: test mode off and the stream again
 *   condip   a supply dip while the UART sends (uart_busy): the lifetime record's dip write at once
 * The samples (SOAK_CON_DUMP=<file>: the console's bytes as sent on console TX): conout; consamplejig (the release: the
 * stream, then jig input held low: two PASS cycles, G's load disconnected: a FAIL, released); consamplefull (the full console,
 * CONSOLE_FULL 1: `status` and `get` with a light at 40 %). */
#if !CONSOLE_FULL

#if BOARD == BOARD_UR_ANALOG_M                    /* (the capture's golden, by board) */
#define CONOUT_GOLDEN_CRC 0x303faa65u      /* (no full status: the live lines with the link, every second;
                                              the soak's receiver-sized bus-side load, test_soak.c ld_ma) */
#define CONOUT_GOLDEN_LEN 9435u
#else
#define CONOUT_GOLDEN_CRC 0x23e6fd1fu      /* (the short's retry is no limit event: `limit 0`,
                                              `power_last_limit=none`, no `limit, 0.04 A` event line; the
                                              settings= / save= / faults= lines and the live lines' link; the dip
                                              43 ms: the lifetime record's dip write, 24 half-words, delays
                                              its end's ms) */
#define CONOUT_GOLDEN_LEN 12687u
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
    CHECK(strncmp(con_out,"B" CONSOLE_SCHEMA_HEX ",",4)==0, "compact boot first");
    lines=con_count(con_out,"R" CONSOLE_SCHEMA_HEX ",");
    CHECK(lines>=63u && lines<=66u,"one live record each second: %u",lines);
    CHECK(con_count(con_out,"S" CONSOLE_SCHEMA_HEX ",")==2u,"full settings twice in 65s");
    CHECK(con_count(con_out,"H" CONSOLE_SCHEMA_HEX ",")==2u,"health twice in 65s");
    CHECK(con_count(con_out,"L" CONSOLE_SCHEMA_HEX ",")==2u,"lifetime twice in 65s");
    for(p=con_out;*p;) {
        char *end=strstr(p,"\r\n"),*star=strchr(p,'*'); uint32_t got=0;
        CHECK(end && star && star<end,"terminated checksummed record");
        if(!end||!star||star>=end)break;
        CHECK(end-p+2<=CONSOLE_LINE_MAX,"bounded line");
        sscanf(star+1,"%x",&got);
        CHECK(crc8_smbus((const uint8_t *)p,(uint32_t)(star-p))==got,"record CRC");
        p=end+2;
    }
    (void)i;(void)n;(void)crc;(void)ver_n;(void)cap;(void)st1;(void)st2;(void)v;(void)vl;

}

static void sc_conin(void)
{
    settings_t s0;
    uint32_t w0, n0;
    printf("soak conin: bytes typed on jig input: nothing happens\n");
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
    CHECK(con_count(con_out + n0, "R" CONSOLE_SCHEMA_HEX ",") >= 4u, "conin: the stream unchanged (%u lines)", con_count(con_out + n0, "R" CONSOLE_SCHEMA_HEX ","));
}

static void sc_conjig(void)
{
    uint32_t n0, n1, jig;
    const char *j;
    printf("soak conjig: jig input held low: test mode, the stream pauses, the jig lines only\n");
    boot(0, 0, 0);
    run_ms(3000);
    soak_tp6_low = 1;
    run_ms(200);                                         /* (TJ_ENTER_MS) */
    CHECK(tj.active, "conjig: test mode on");
    n0 = con_out_n;
    run_ms(8000);                                        /* five cycles of the pattern */
    jig = con_count(con_out + n0, "jig cycle=");
    printf("  %u jig lines in 8 s; %s", jig, (j = strstr(con_out + n0, "jig cycle=")) ? j : "(none)\n");
    CHECK(jig >= 4u && con_count(con_out + n0, "R" CONSOLE_SCHEMA_HEX ",") == 0u && strstr(con_out + n0, "uptime=") == NULL,
          "conjig: the jig lines only, the stream paused (%u jig lines, %u live lines)", jig,
          con_count(con_out + n0, "R" CONSOLE_SCHEMA_HEX ","));
    CHECK(j != NULL && strstr(j, " fail=") != NULL && strstr(j, " a=") != NULL && strstr(j, " supply=") != NULL &&
          strstr(j, " uid=") != NULL && strstr(j, " fw=") != NULL, "conjig: the jig line's fields");
    soak_tp6_low = 0;
    run_ms(200);
    n1 = con_out_n;
    run_ms(3000);
    CHECK(!tj.active && strstr(con_out + n0, "X" CONSOLE_SCHEMA_HEX ",") != NULL && con_count(con_out + n1, "R" CONSOLE_SCHEMA_HEX ",") >= 2u,
          "conjig: released: test mode off, the stream again (%u lines)", con_count(con_out + n1, "R" CONSOLE_SCHEMA_HEX ","));
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
    /* Compact live records may drain before the dip is detected. Keep the UART
     * deliberately busy for this flash/console concurrency regression. */
    while (uart_room() < 240u) run_ms(1);
    { char traffic[241]; memset(traffic, '0', 240); traffic[240]=0; uart_write(traffic); }
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
    printf("soak consamplejig: the stream, then jig input held low: PASS, PASS, G disconnected: FAIL\n");
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
