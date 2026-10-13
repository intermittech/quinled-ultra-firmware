/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Input energy (core/energy.c), included by test_main.c: the integration (24 V x 2 A for an hour = 48 Wh, no drift,
 * no overflow at 75 V x 20 A); when the lifetime value is written (hourly while lit, 30 s after the light goes dark,
 * at a supply dip; never without 1 Wh new, never while it only idles); the write budget over a month of the worst
 * pattern; the log in a page image (the newest valid value, an entry cut short, the 0xE339 erased pattern, the next
 * free entry); the ENERGY page. */
#include "../src/core/energy.h"

/* t0..t1 ms at mv x ma, lit: the writes it asks for (simulated: each one stored at once) */
static uint32_t en_run(energy_t *e, uint32_t t0, uint32_t t1, uint32_t mv, int32_t ma, uint8_t lit, uint16_t dips,
                       uint32_t *dip_writes)
{
    uint32_t t, n = 0, w;
    for (t = t0; t < t1; t++) {
        energy_ms(e, t, mv, ma, lit);
        w = energy_due(e, t, dips);
        if (w != ENERGY_DUE_NONE) {
            energy_saved(e, t);
            n++;
            if (w == ENERGY_DUE_DIP && dip_writes) {
                (*dip_writes)++;
            }
        }
    }
    return n;
}

static void test_energy(void)
{
    energy_t e;
    uint32_t n, t, d, worst;
    uint8_t page[256], pg[24];

    printf("energy: integration, the write rules, the budget, the log, the page\n");
    energy_init(&e, 1000u, 0);
    n = en_run(&e, 0, 3600000u, 24000u, 2000, 1, 0, 0);
    CHECK(e.boot_mwh == 48000u && energy_total_wh(&e) == 1048u, "24 V x 2 A for 1 h: %u mWh, %u Wh total",
          e.boot_mwh, energy_total_wh(&e));
    CHECK(n == 0 && !e.saved, "lit: not written within the first hour");
    n = en_run(&e, 3600000u, 3600001u, 24000u, 2000, 1, 0, 0);
    CHECK(n == 1 && e.saved && e.saved_wh == 1048u, "written after an hour (%u, %u Wh)", n, e.saved_wh);
    /* the largest input: 75 V x 20 A for a minute */
    energy_init(&e, 0, 0);
    (void)en_run(&e, 0, 60000u, 75000u, 20000, 1, 0, 0);
    CHECK(e.boot_mwh == 25000u, "75 V x 20 A for 1 min: %u mWh (25 Wh)", e.boot_mwh);
    energy_init(&e, 0, 0);
    (void)en_run(&e, 0, 1000u, 24000u, -50, 1, 0, 0);
    CHECK(e.boot_mwh == 0u && e.uj == 0u, "a negative current (offset) adds nothing");

    /* dark: 30 s after the light goes off, if 1 Wh came; not before */
    energy_init(&e, 0, 0);
    n = en_run(&e, 0, 600000u, 24000u, 2000, 1, 0, 0);        /* 10 min lit: 8 Wh */
    n += en_run(&e, 600000u, 629000u, 24000u, 30, 0, 0, 0);
    CHECK(n == 0, "dark for 29 s: not yet");
    n = en_run(&e, 629000u, 631000u, 24000u, 30, 0, 0, 0);
    CHECK(n == 1 && e.saved_wh == 8u, "dark for 30 s: written (%u, %u Wh)", n, e.saved_wh);
    /* idling (logic current only): never written again, also after hours */
    n = en_run(&e, 631000u, 631000u + 21600000u, 24000u, 30, 0, 0, 0);
    CHECK(n == 0, "idle for 6 h: %u writes", n);
    /* lit below 1 Wh, dark: nothing */
    energy_init(&e, 0, 0);
    n = en_run(&e, 0, 30000u, 24000u, 2000, 1, 0, 0) + en_run(&e, 30000u, 100000u, 24000u, 30, 0, 0, 0);
    CHECK(n == 0, "0.4 Wh, then dark: nothing written");

    /* a supply dip writes at once (no page erase then); not without 1 Wh new */
    energy_init(&e, 0, 0);
    d = 0;
    n = en_run(&e, 0, 300000u, 24000u, 2000, 1, 0, &d);       /* 4 Wh */
    n += en_run(&e, 300000u, 300001u, 24000u, 2000, 1, 1, &d);
    CHECK(n == 1 && d == 1 && e.saved_wh == 4u, "a dip: written (%u, %u Wh)", d, e.saved_wh);
    n = en_run(&e, 300001u, 300002u, 24000u, 2000, 1, 2, &d);
    CHECK(n == 0, "a second dip right after: nothing new to write");

    /* the budget: the worst pattern (lit 2 min at 384 W, dark 31 s, over and over, a dip each time) for 3 days */
    energy_init(&e, 0, 0);
    n = 0;
    for (t = 0, d = 0; t < 3u * 86400000u; t += 151000u) {
        n += en_run(&e, t, t + 120000u, 48000u, 8000, 1, (uint16_t)d, 0);
        d++;
        n += en_run(&e, t + 120000u, t + 151000u, 48000u, 30, 0, (uint16_t)d, 0);
    }
    worst = (n - ENERGY_BUDGET_CAP) / 3u;
    CHECK(n <= 3u * 24u + ENERGY_BUDGET_CAP + 1u && n >= 3u * 24u, "worst pattern: %u writes in 3 days (24 a day "
          "+ the budget's 4)", n);
    (void)worst;

    /* the log */
    memset(page, 0xFF, sizeof page);
    CHECK(energy_log_find(page) == 0 && energy_log_next(page) == 0, "empty page");
    {
        uint32_t v[2];
        v[0] = 120; v[1] = ~120u; memcpy(page, v, 8);
        v[0] = 150; v[1] = ~150u; memcpy(page + 8, v, 8);
        v[0] = 9999; v[1] = 0xFFFFFFFFu; memcpy(page + 16, v, 8);   /* cut short: the complement never written */
    }
    CHECK(energy_log_find(page) == 150u && energy_log_next(page) == 3u, "the newest valid value: %u, next free %u",
          energy_log_find(page), energy_log_next(page));
    for (t = 0; t < 256u; t += 2u) {
        page[t] = 0x39;
        page[t + 1u] = 0xE3;
    }
    CHECK(energy_log_find(page) == 0 && energy_log_next(page) == 0, "0xE339 erased page");
    for (t = 0; t < 256u; t += 8u) {
        uint32_t v[2] = { t, ~t };
        memcpy(page + t, v, 8);
    }
    CHECK(energy_log_find(page) == 248u && energy_log_next(page) == ENERGY_RECS, "a full page: %u, next %u",
          energy_log_find(page), energy_log_next(page));

    /* the ENERGY page */
    energy_init(&e, 1000u, 0);
    (void)en_run(&e, 0, 3600001u, 24000u, 2000, 1, 0, 0);
    energy_page(&e, 3700000u, 48000u, pg);
    CHECK(ultra_get32(pg + ULTRA_EN_BOOT_MWH) == 48000u && ultra_get32(pg + ULTRA_EN_TOTAL_WH) == 1048u &&
          ultra_get32(pg + ULTRA_EN_POWER_MW) == 48000u && ultra_get24(pg + ULTRA_EN_SAVED_AGE) == 100u,
          "ENERGY page fields");
    energy_init(&e, 0, 0);
    energy_page(&e, 5000u, 0, pg);
    CHECK(ultra_get24(pg + ULTRA_EN_SAVED_AGE) == 0xFFFFFFu, "ENERGY page: not stored since boot");
}
