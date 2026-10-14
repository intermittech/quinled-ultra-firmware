/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (0.6.5, D9) the console's backup / restore (core/cli.c), included by test_main.c before main() (test_features.c's
 * console helpers): `backup` prints one `restore <hex>` line (version, board, settings, extension, calibration, CRC);
 * `restore` takes it back through cli_t.restore only when the length, CRC, version and board match (else the reason),
 * upper / lower case hex, leading spaces; random lines: the callback runs exactly when they are valid records.
 * main.c's side (the values applied, saved through the stores): test_soak.c `d9`. */
#include <ctype.h>

static uint8_t d9_got[71];
static uint32_t d9_calls;
static int d9_answer = 1;
static int d9_restore(const uint8_t *d)
{
    memcpy(d9_got, d, sizeof d9_got);
    d9_calls++;
    return d9_answer;
}

static uint32_t d9_hex(const char *s, uint8_t *out, uint32_t max)   /* hex text -> bytes */
{
    uint32_t n = 0;
    unsigned v;
    while (n < max && sscanf(s + 2u * n, "%2x", &v) == 1) {
        out[n++] = (uint8_t)v;
    }
    return n;
}

static void d9_line(char *out, const uint8_t *r, uint32_t n)        /* bytes -> a restore line */
{
    uint32_t i;
    strcpy(out, "restore ");
    for (i = 0; i < n; i++) {
        sprintf(out + 8u + 2u * i, "%02x", r[i]);
    }
    strcat(out, "\r");
}

static void d9_seal(uint8_t *r)                                     /* the CRC of a 73-byte record */
{
    uint16_t k = crc16_ccitt(r, 73u);
    r[73] = (uint8_t)(k >> 8);
    r[74] = (uint8_t)k;
}

static void test_d9(void)
{
    cli_t c;
    cli_status_t st;
    store_ext_t ext;
    uint8_t r[80], q[80];
    char line[200];
    uint32_t i, k, n, valid, calls0, bad = 0;

    printf("D9 backup / restore: the record, its checks, random lines\n");
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    config_rx_init(&cli_rx);
    cli_set.dither_mode = 1;
    cli_set.pwm_preset = 3;
    store_ext_clear(&ext);
    ext.ilimit50 = 100;
    st.version = "test";
    st.steps = 5760;
    st.en = &test_en;
    st.cal.levels = 1;
    st.cal.pct[0] = 50;
    st.cal.zero_ma = -12;
    for (i = 0; i < 5u; i++) {
        st.cal.ma[i][0] = (uint16_t)(1000u + 7u * i);
    }
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.set = &cli_set;
    c.eff = &cli_set;
    c.ext = &ext;
    c.st = &st;
    c.restore = d9_restore;

    run_cli(&c, "backup\r");
    n = (uint32_t)strlen(cli_out);
    CHECK(n == 8u + 150u + 2u && strncmp(cli_out, "restore ", 8) == 0 && strcmp(cli_out + n - 2u, "\r\n") == 0,
          "backup: one restore line of 150 hex digits (%u: %.20s)", n, cli_out);
    CHECK(d9_hex(cli_out + 8, r, 75u) == 75u, "backup: hex");
    CHECK(r[0] == 1u && r[1] == BOARD && memcmp(r + 2, &cli_set, 16) == 0 && memcmp(r + 18, &ext, 7) == 0 &&
          memcmp(r + 25, &st.cal, 48) == 0 && crc16_ccitt(r, 73u) == (uint16_t)(r[73] << 8 | r[74]),
          "backup: version 1, the board, settings + extension + calibration as stored, the CRC");

    cli_out[n - 2u] = '\r';                              /* paste it back */
    cli_out[n - 1u] = 0;
    strcpy(line, cli_out);
    d9_calls = 0;
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "ok\r\n") == 0 && d9_calls == 1u && memcmp(d9_got, r + 2, 71) == 0, "restore: ok, the 71 "
          "data bytes to main.c (%s)", cli_out);
    for (i = 8; line[i] && line[i] != '\r'; i++) {
        line[i] = (char)toupper((unsigned char)line[i]);
    }
    memmove(line + 10, line + 8, strlen(line + 8) + 1u);   /* and with leading spaces */
    line[8] = ' ';
    line[9] = ' ';
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "ok\r\n") == 0 && d9_calls == 2u, "upper case, leading spaces (%s)", cli_out);

    memcpy(q, r, 75);
    q[40] ^= 0x10;                                       /* a changed byte: the CRC */
    d9_line(line, q, 75u);
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: CRC\r\n") == 0 && d9_calls == 2u, "a wrong CRC (%s)", cli_out);
    memcpy(q, r, 75);
    q[0] = 2;                                            /* another version */
    d9_seal(q);
    d9_line(line, q, 75u);
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: version\r\n") == 0 && d9_calls == 2u, "another version (%s)", cli_out);
    memcpy(q, r, 75);
    q[1] = (uint8_t)(BOARD + 1u);                        /* another board */
    d9_seal(q);
    d9_line(line, q, 75u);
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: board\r\n") == 0 && d9_calls == 2u, "another board (%s)", cli_out);
    d9_answer = 0;                                       /* main.c rejects the values */
    d9_line(line, r, 75u);
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: values\r\n") == 0 && d9_calls == 3u, "values main.c rejects (%s)", cli_out);
    d9_answer = 1;
    d9_line(line, r, 74u);                               /* a byte short */
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: length\r\n") == 0, "short (%s)", cli_out);
    d9_line(line, r, 75u);
    strcpy(line + strlen(line) - 1u, "0\r");             /* a digit more */
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: length\r\n") == 0, "a digit too many (%s)", cli_out);
    d9_line(line, r, 75u);
    line[20] = 'g';                                      /* not hex */
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: length\r\n") == 0, "not hex (%s)", cli_out);
    run_cli(&c, "restore\r");
    CHECK(strcmp(cli_out, "error: restore: length\r\n") == 0, "nothing (%s)", cli_out);
    c.restore = 0;
    d9_line(line, r, 75u);
    run_cli(&c, line);
    CHECK(strcmp(cli_out, "error: restore: values\r\n") == 0, "no callback: not taken");
    c.restore = d9_restore;
    st.cal.levels = 0;                                   /* no calibration: its bytes as they are (levels 0) */
    run_cli(&c, "backup\r");
    CHECK(d9_hex(cli_out + 8, q, 75u) == 75u && q[25] == 0u && crc16_ccitt(q, 73u) == (uint16_t)(q[73] << 8 | q[74]),
          "backup without a calibration: levels 0");
    run_cli(&c, "help\r");
    CHECK(strstr(cli_out, "backup | restore <hex>\r\n") != NULL, "help lists them");

    /* random records: valid ones (a correct CRC, version and board) reach main.c, nothing else does */
    rs = 0xD9D9D9D9u;
    for (k = 0; k < 3000u; k++) {
        n = 70u + rint_(10);
        for (i = 0; i < n; i++) {
            q[i] = (uint8_t)rnd();
        }
        if (rint_(2)) {
            q[0] = 1;
            q[1] = BOARD;
        }
        if (n == 75u && rint_(2)) {
            d9_seal(q);
        }
        valid = n == 75u && q[0] == 1u && q[1] == BOARD && crc16_ccitt(q, 73u) == (uint16_t)(q[73] << 8 | q[74]);
        d9_line(line, q, n);
        calls0 = d9_calls;
        run_cli(&c, line);
        bad += (d9_calls != calls0 + valid) || (valid && memcmp(d9_got, q + 2, 71) != 0) ||
               (uint32_t)(strcmp(cli_out, "ok\r\n") == 0) != valid;
    }
    CHECK(bad == 0u, "random lines: the callback exactly for the valid ones (%u wrong)", bad);
}
