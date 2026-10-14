/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* (0.6.5, D6) the ADC / DMA fault counters' console line (core/cli.c, after `supply=`), included by test_main.c after
 * test_ultra_all_off.c (test_features.c's console helpers): nothing while every counter is 0 (or no counters), else
 * one line with the two ADC counts and the seven DMA channels. The counting itself: test_d6_hw.py (rv32emu). */

static void test_d6(void)
{
    cli_t c;
    cli_status_t st;
    volatile uint16_t h[HW_DIAG_N];
    const char *q;
    uint32_t i;

    printf("D6 ADC / DMA counters: the status line\n");
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
    c.st = &st;
    for (i = 0; i < HW_DIAG_N; i++) {
        h[i] = 0;
    }
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "adc_stale") == NULL, "no counters: no line");
    st.hwd = h;
    run_cli(&c, "status\r");
    CHECK(strstr(cli_out, "adc_stale") == NULL, "all 0: no line");
    h[8] = 1;                                            /* DMA1 channel 7 (UART TX) */
    run_cli(&c, "status\r");
    q = strstr(cli_out, "adc_stale=");
    CHECK(q && strncmp(q, "adc_stale=current 0, voltage 0; dma_errors ch1-7=0/0/0/0/0/0/1\r\n", 64) == 0,
          "one DMA error (%.70s)", q ? q : "-");
    CHECK(q && strstr(cli_out, "supply=") < q && strstr(cli_out, "input=") > q, "after supply=");
    h[0] = 65535;
    h[1] = 3;
    h[2] = 12;
    h[8] = 0;
    run_cli(&c, "status\r");
    q = strstr(cli_out, "adc_stale=");
    CHECK(q && strncmp(q, "adc_stale=current 65535, voltage 3; dma_errors ch1-7=12/0/0/0/0/0/0\r\n", 69) == 0,
          "ADC counts (%.75s)", q ? q : "-");
}
