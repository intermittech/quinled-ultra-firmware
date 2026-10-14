/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (0.6.5, D5) the flash state line, included by test_main.c after test_cal.c (test_features.c's console helpers):
 * pp_find counts the entries it rejects (neither erased nor valid: a cut record, a corrupted one, one of another
 * layout; an erased one, 0xFFFF or 0xE339, is not), and the console's `flash:` line (pending writes by store, rejected
 * records, failed operations with the last FLASH_ERR_*), shown only when one of them is there. main.c's side (the
 * counts at boot, the effect store's heads, the error record, the pending bits): test_soak.c `d5`. */

static void test_d5(void)
{
    static uint8_t pg[2][256];
    const pp_desc_t d = { STORE_SLOT_BYTES, STORE_SLOTS, 4u, 0u, store_valid };
    pp_pos_t pos;
    settings_t s;
    store_ext_t e;
    cli_t c;
    cli_status_t st;
    const uint8_t *r;
    const char *q;

    printf("D5 flash state: rejected records, the flash: line\n");
    memset(pg, 0xFF, sizeof pg);
    settings_default(&s);
    store_ext_clear(&e);
    store_pack_ext(&s, &e, 7u, (store_slot_t *)pg[0]);
    store_pack_ext(&s, &e, 8u, (store_slot_t *)(pg[0] + 32));
    pg[0][64 + 4] = 0x09;                                /* slot 2: a record cut after 5 bytes */
    pg[0][64 + 0] = 0xAA;
    pg[1][0] = 0x39;                                     /* page B slot 0: 0xE339 erased half-words, not rejected */
    pg[1][1] = 0xE3;
    pg[1][2] = 0x39;
    pg[1][3] = 0xE3;
    store_pack_ext(&s, &e, 9u, (store_slot_t *)(pg[1] + 32));
    ((store_slot_t *)(pg[1] + 32))->crc ^= 1u;           /* page B slot 1: a corrupted record */
    r = pp_find(&d, pg[0], pg[1], &pos);
    CHECK(r == pg[0] + 32 && pos.bad == 2u, "pp_find: the newest valid one, 2 rejected (cut, corrupted; the 0xE339 "
          "slot erased) (bad %u)", (uint32_t)pos.bad);
    memset(pg, 0xFF, sizeof pg);
    r = pp_find(&d, pg[0], pg[1], &pos);
    CHECK(r == 0 && pos.bad == 0u, "erased pages: none, nothing rejected");

    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    settings_default(&cli_set);
    config_rx_init(&cli_rx);
    st.version = "test";
    st.steps = 5760;
    st.en = &test_en;
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.set = &cli_set;
    c.eff = &cli_set;
    c.st = &st;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "flash:") == NULL, "all well: no flash line");
    st.fl_pend = 1u | 4u;                                /* settings, scene */
    run_cli(&c, "status\r");
    q = strstr(cli_out, "flash:");
    CHECK(q && strncmp(q, "flash: settings pending, scene pending, errors 0 (-)\r\n", 54) == 0,
          "pending writes (%.60s)", q ? q : "-");
    CHECK(q && strstr(cli_out, "flash_saves=") < q, "after flash_saves=");
    st.fl_pend = 0;
    st.fl_rej = 2u << 6 | 1u << 24;                      /* calibration 2, power-on effect 1 */
    run_cli(&c, "status\r");
    q = strstr(cli_out, "flash:");
    CHECK(q && strncmp(q, "flash: calibration rejected 2, power-on effect rejected 1, errors 0 (-)\r\n", 73) == 0,
          "rejected records (%.80s)", q ? q : "-");
    st.fl_pend = 1u;
    st.fl_rej = 1u << 18 | 1u << 24;                     /* lifetime 1, power-on effect 1 */
    st.fl_errs = 3u;
    st.fl_err = FLASH_ERR_WRP;
    run_cli(&c, "status\r");
    q = strstr(cli_out, "flash:");
    CHECK(q && strncmp(q, "flash: settings pending, lifetime rejected 1, power-on effect rejected 1, "
                          "errors 3 (WRP)\r\n", 90) == 0, "all three (%.100s)", q ? q : "-");
    st.fl_rej = 0;
    st.fl_pend = 0;
    st.fl_errs = 1u;
    st.fl_err = FLASH_ERR_VERIFY;
    run_cli(&c, "status\r");
    q = strstr(cli_out, "flash:");
    CHECK(q && strncmp(q, "flash: errors 1 (VERIFY)\r\n", 26) == 0, "an error alone (%.40s)", q ? q : "-");
    st.fl_err = 9u;                                      /* (not a code: no index past the names) */
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "flash: errors 1 (-)\r\n") != NULL, "an unknown code");
}
