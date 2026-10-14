/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* r5cpu (0.6.5 D3), included by test_soak.c: the CPU load in `status` and `watch` from main.c's WFI stamps
 * (core/cpuload.h), on the soak's model of the sleep (soak_wfi: the next tick runs inside the WFI, each pass works
 * soak_busy cycles after its tick), so the load is soak_busy / 7,200 exactly. Boots 3 s before the tick count's wrap.
 * A burst shows as the peak (the busiest second since the last status, then again from the last second); a flash
 * write (the SRAM tick, no WFI) counts as busy for its time. */

/* the console's "<key><a>.<b> %" in permille (-1: none) */
static int32_t r5c_con(const char *from, const char *key)
{
    const char *p = strstr(from, key);
    unsigned a, b;
    if (!p || sscanf(p + strlen(key), "%u.%u", &a, &b) != 2) {
        return -1;
    }
    return (int32_t)(a * 10u + b);
}

static void sc_r5cpu(void)
{
    int32_t cpu, peak;
    uint32_t lines = 0, sum = 0;
    uint64_t st0;
    const char *p;
    printf("soak r5cpu: the CPU load in status / watch from the WFI stamps (a modelled sleep), across the tick wrap\n");
    soak_wfi = 1;
    soak_busy = 252u;                                    /* 3.5 % */
    cpu_t0 = 0xFFFFFFFFu - 59999u;                       /* (main.c's first window: from the boot's tick count) */
    boot(0, 0xFFFFFFFFu - 59999u, 0);                    /* the tick count wraps 3 s in */
    run_ms(3500);
    console("status");
    cpu = r5c_con(con_out, "cpu=");
    peak = r5c_con(con_out, ", peak ");
    printf("  steady 252 cycles per tick: cpu=%d, peak %d permille (tick count %08X)\n", (int)cpu, (int)peak,
           (unsigned)tick_now);
    CHECK(cpu == 35 && peak >= 35 && peak <= 45, "r5cpu: steady 3.5 %% across the wrap, the peak the boot's second "
          "(its erases ahead: busy) (cpu %d, peak %d)", (int)cpu, (int)peak);
    CHECK(tick_now < 0x80000000u, "r5cpu: the tick count wrapped (%08X)", (unsigned)tick_now);

    soak_busy = 2952u;                                   /* 41 % for 2.5 s */
    run_ms(2500);
    console("status");
    cpu = r5c_con(con_out, "cpu=");
    peak = r5c_con(con_out, ", peak ");
    CHECK(cpu == 410 && peak == 410, "r5cpu: burst 41 %% (cpu %d, peak %d)", (int)cpu, (int)peak);
    soak_busy = 252u;
    run_ms(2200);
    console("status");
    cpu = r5c_con(con_out, "cpu=");
    peak = r5c_con(con_out, ", peak ");
    CHECK(cpu == 35 && peak == 410, "r5cpu: after the burst: cpu 3.5, peak still 41 (from the last status's second) "
          "(cpu %d, peak %d)", (int)cpu, (int)peak);
    run_ms(1000);
    console("status");
    peak = r5c_con(con_out, ", peak ");
    CHECK(peak == 35, "r5cpu: the next status: the peak again from the last second (%d)", (int)peak);

    /* a settings write (2 s after the change) during a watch (a line at once, then every second: 7 seconds): its
     * stall is busy */
    console("set loss 2");
    st0 = sim_stall_us;
    console("watch 1000");
    run_ms(6000);
    for (p = strstr(con_out, "  cpu "); p; p = strstr(p + 1, "  cpu ")) {
        int32_t v = r5c_con(p, "  cpu ");
        lines++;
        sum += v > 0 ? (uint32_t)v : 0u;
    }
    console("");                                         /* (any key stops the watch) */
    {
        double stall_ms = (double)(sim_stall_us - st0) / 1000.0, want = 35.0 * lines + stall_ms * (1.0 - 0.035);
        printf("  watch: %u lines, sum %u permille-s; flash stall %.1f ms: expected %.1f\n", lines, sum, stall_ms,
               want);
        CHECK(lines == 7u && stall_ms > 0.0 && (sum > want ? sum - want : want - sum) <= 4.0, "r5cpu: the stall counts "
              "as busy (sum %u, want %.1f, %u lines, stall %.1f ms)", sum, want, lines, stall_ms);
    }
}
