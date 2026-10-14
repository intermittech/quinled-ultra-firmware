/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* r5jig (0.6.5 D10), included by test_soak.c: the test jig's line for a jig PC, on main.c. TP9 grounded from power-up:
 * every cycle (1.5 s) one `jig` line after the report, with the five currents at 33 % of the soak's loads (1.5 A on R / G
 * / B, 1.2 A on W1 / W2 at full), PASS, the chip's id and the version; then W2's load open (FAIL, bit 16) and R's ten
 * times too big (FAIL, bit 1). */

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
    CHECK(strstr(con_out, "test @33%: R ") && strstr(con_out, "\r\njig cycle="), "r5jig: after the report line");

    ld_ma[4] = 0;                                        /* W2's load open */
    c0 = cycle;
    run_ms(3200);
    ok = (uint32_t)r5j_line(&cycle, res, &fail, ma, uid, fw);
    CHECK(ok && cycle >= c0 + 2u && strcmp(res, "FAIL") == 0 && fail == 16u && ma[4] < TJ_PASS_MIN_MA,
          "r5jig: W2 open: FAIL, bit 16 (%s, fail %u, %u mA)", res, fail, ma[4]);
    ld_ma[4] = 1200;
    ld_ma[0] = 15000;                                    /* R ten times its load: ~5 A at 33 % */
    run_ms(3200);
    ok = (uint32_t)r5j_line(&cycle, res, &fail, ma, uid, fw);
    CHECK(ok && strcmp(res, "FAIL") == 0 && fail == 1u && ma[0] > TJ_PASS_MAX_MA, "r5jig: R too high: FAIL, bit 1 "
          "(%s, fail %u, %u mA)", res, fail, ma[0]);
    ld_ma[0] = 1500;
}
