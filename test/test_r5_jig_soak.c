/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* r5jig (0.6.5 D10), included by test_soak.c: the test jig's line for a jig PC, on main.c. TP9 grounded from power-up:
 * every cycle (1.5 s) one `jig` line (0.6.7: the only report), with the five currents at 33 % of the soak's loads
 * (1.5 A on R / G / B, 1.2 A on W1 / W2 at full), PASS, the chip's id and the version, D9 steady; then W2's load open
 * (FAIL, bit 16, D9 at 5 Hz) and R's ten times too big (FAIL, bit 1).
 * r5jig6 (0.6.7), both boards: the jig on TP6 (PA3, the console's RX): an unconnected pad (the pull-up), console
 * traffic (the line low most of each ms, bytes arriving), the pad grounded from power-up, a UART break while running
 * (one 0x00 received, then the line held low: the console unharmed after it), the release. The receiver: TP9 never
 * counts (PB10 is its return channel's TX). */

/* the newest `jig` line's fields (the currents in mA); 0 = none */
static int r5j_line(uint32_t *cycle, char *res, uint32_t *fail, uint32_t ma[5], char *uid, char *fw)
{
    const char *p = con_out, *q = 0;
    double a[5], v = 0.0;
    int n;
    uint32_t i;
    while ((p = strstr(p, "jig cycle=")) != NULL) {
        q = p++;
    }
    n = q ? sscanf(q, "jig cycle=%u %4s fail=%u a=%lf/%lf/%lf/%lf/%lf supply=%lf uid=%31s fw=%15s", cycle, res, fail,
                   &a[0], &a[1], &a[2], &a[3], &a[4], &v, uid, fw) : 0;
    for (i = 0; i < 5u; i++) {
        ma[i] = n == 11 ? (uint32_t)(a[i] * 1000.0 + 0.5) : 0u;
    }
    return n == 11 && v > 23.9 && v < 24.1;
}

static void sc_r5jig(void)
{
    uint32_t cycle = 0, fail = 0, ma[5], c0, i, ok;
    char res[8] = "", uid[32] = "", fw[16] = "";
    printf("soak r5jig: the test jig's line for a jig PC (TP9 grounded from power-up)\n");
    soak_gpiob.INDR = 0xFFFFFBFFu;                       /* TP9 (PB10) low */
    boot(0, 0, 0);
    run_ms(5000);
    ok = (uint32_t)r5j_line(&cycle, res, &fail, ma, uid, fw);
    printf("  cycle %u: %s fail %u, %u/%u/%u/%u/%u mA, uid %s, fw %s\n", cycle, res, fail, ma[0], ma[1], ma[2], ma[3],
           ma[4], uid, fw);
    CHECK(ok && tj.active && cycle >= 2u && strcmp(res, "PASS") == 0 && fail == 0u, "r5jig: a passing line each cycle "
          "(%u, cycle %u, %s, fail %u)", ok, cycle, res, fail);
    for (i = 0; i < 5u; i++) {
        uint32_t want = (uint32_t)((uint64_t)ld_ma[i] * TJ_PEAK / 65535u);
        CHECK(ma[i] + 45u >= want && ma[i] <= want + 65u, "r5jig: channel %u: %u mA at 33 %% (the load: %u)", i, ma[i],
              want);
    }
    CHECK(strcmp(uid, "305419896,2596069104,253635900") == 0 && strcmp(fw, FW_VERSION) == 0, "r5jig: uid %s, fw %s",
          uid, fw);
    CHECK(!strstr(con_out, "test @") && strstr(con_out, "jig cycle="), "r5jig: the jig line only (0.6.7)");
    {
        uint32_t lo = 0, k;
        for (k = 0; k < 400u; k++) {                     /* D9: the verdict, PASS: steady */
            run_ms(1);
            lo += !led_now;
        }
        CHECK(lo == 0u && tj.fail == 0u, "r5jig: D9 steady on a PASS (%u ms off)", lo);
    }

    ld_ma[4] = 0;                                        /* W2's load open */
    c0 = cycle;
    run_ms(3200);
    ok = (uint32_t)r5j_line(&cycle, res, &fail, ma, uid, fw);
    CHECK(ok && cycle >= c0 + 2u && strcmp(res, "FAIL") == 0 && fail == 16u && ma[4] < TJ_PASS_MIN_MA,
          "r5jig: W2 open: FAIL, bit 16 (%s, fail %u, %u mA)", res, fail, ma[4]);
    {
        uint32_t flips = 0, k;
        uint8_t was = led_now;
        for (k = 0; k < 400u; k++) {                     /* D9: FAIL: 5 Hz */
            run_ms(1);
            flips += led_now != was;
            was = led_now;
        }
        CHECK(flips >= 3u && flips <= 5u, "r5jig: D9 at 5 Hz on a FAIL (%u changes in 400 ms)", flips);
    }
    ld_ma[4] = 1200;
    ld_ma[0] = 15000;                                    /* R ten times its load: ~5 A at 33 % */
    run_ms(3200);
    ok = (uint32_t)r5j_line(&cycle, res, &fail, ma, uid, fw);
    CHECK(ok && strcmp(res, "FAIL") == 0 && fail == 1u && ma[0] > TJ_PASS_MAX_MA, "r5jig: R too high: FAIL, bit 1 "
          "(%s, fail %u, %u mA)", res, fail, ma[0]);
    ld_ma[0] = 1500;
    /* (0.6.6) statuses typed while the jig runs: its reports wait for the UART's room, none lost, nothing blocks */
    {
        const char *p;
        uint32_t b0 = tx_blocks, c1, n = 0, k;
        run_ms(100);
        (void)r5j_line(&c0, res, &fail, ma, uid, fw);
        con_out_n = 0;
        con_out[0] = 0;
        con_in_r = con_in_w = 0;
        for (k = 0; k < 3u; k++) {
            memcpy(con_in + con_in_w, "status\r", 7u);
            con_in_w += 7u;
        }
        run_ms(4600);
        (void)r5j_line(&c1, res, &fail, ma, uid, fw);
        for (p = con_out; (p = strstr(p, "jig cycle=")) != 0; p++) {
            n++;
        }
        printf("  3 x status while the jig runs: cycles %u..%u, %u jig lines, writes past the UART's buffers: %u\n", c0,
               c1, n, tx_blocks - b0);
        CHECK(c1 >= c0 + 2u && n == c1 - c0 && tx_blocks == b0, "r5jig: a line each cycle during statuses (%u..%u, %u "
              "lines, %u blocked bytes)", c0, c1, n, tx_blocks - b0);
    }
}

static void sc_r5jig6(void)
{
    uint32_t cycle = 0, fail = 0, ma[5], ms, low;
    char res[8] = "", uid[32] = "", fw[16] = "";
    printf("soak r5jig6: the jig on TP6 (the console's RX): unconnected, traffic, grounded, a break, released\n");
    boot(0, 0, 0);
    run_ms(2000);
    CHECK(!tj.active, "r5jig6: TP6 unconnected (the pull-up): no jig");
#if BOARD == BOARD_DP_RX
    soak_gpiob.INDR = 0xFFFFFBFFu;                       /* PB10 low: the receiver's return channel TX, no pad */
    run_ms(500);
    CHECK(!tj.active, "r5jig6: the receiver: TP9 / PB10 never counts");
    soak_gpiob.INDR = 0xFFFFFFFFu;
#endif
    /* console traffic: a stream of 0x00 bytes reads low at almost every sample (start and data bits low; the 1 ms
     * sample rarely meets a stop bit): low throughout here, bytes arriving */
    for (ms = 0, low = 0; ms < 3000u; ms++) {
        soak_tp6_low = 1;
        soak_rx_extra += 11u;                            /* (115200 baud: ~11.5 bytes a ms) */
        run_ms(1);
        low |= tj.active;                                /* (entered at any time) */
    }
    soak_tp6_low = 0;
    run_ms(100);
    CHECK(!low && !tj.active, "r5jig6: 3 s of console traffic: no jig (entered %u)", low);
    /* a UART break: one 0x00 received, then the line held low; the jig line goes out on TX meanwhile */
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    con_in[con_in_w++] = 0;                              /* (the break's character) */
    soak_tp6_low = 1;
    for (ms = 0, low = 0; ms < 100u && !tj.active; ms++) {
        run_ms(1);
        low++;
    }
    CHECK(tj.active && low >= TJ_ENTER_MS && low <= TJ_ENTER_MS + 3u, "r5jig6: a held break enters after %u ms", low);
    run_ms(5000);
    CHECK(tj.active && r5j_line(&cycle, res, &fail, ma, uid, fw) && cycle >= 2u && strcmp(res, "PASS") == 0,
          "r5jig6: the jig line during the break (cycle %u, %s)", cycle, res);
    soak_tp6_low = 0;                                    /* released */
    run_ms(TJ_LEAVE_MS + 5u);
    CHECK(!tj.active, "r5jig6: released: back to normal operation");
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    con_in_w = (uint32_t)strlen(strcpy(con_in, "get\r"));
    run_ms(200);
    CHECK(strstr(con_out, "period=") != NULL, "r5jig6: the console after the break (its 0x00 ignored): %.40s", con_out);
}

/* r5jig6p: TP6 grounded from power-up (the jig fixture: GND, power, TP5, TP6; a scenario of its own: one boot each) */
static void sc_r5jig6p(void)
{
    uint32_t cycle = 0, fail = 0, ma[5];
    char res[8] = "", uid[32] = "", fw[16] = "";
    printf("soak r5jig6p: TP6 grounded from power-up\n");
    soak_tp6_low = 1;
    boot(0, 0, 0);
    run_ms(4000);
    CHECK(tj.active && r5j_line(&cycle, res, &fail, ma, uid, fw) && strcmp(res, "PASS") == 0 && cycle >= 1u,
          "r5jig6p: jig lines (cycle %u, %s)", cycle, res);
    soak_tp6_low = 0;
}

