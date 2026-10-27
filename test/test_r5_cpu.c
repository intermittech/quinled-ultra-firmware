/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CPU load (core/cpuload.h), included by test_main.c: a cycle-level model of the main loop's idle WFI
 * against the stamps main.c takes around it. The core sleeps until the next interrupt: the poll tick at every tick's
 * start (its interrupt takes 60-160 cycles with the wake-up), or a gate / slew interrupt at any moment (40-200 cycles);
 * after each wake the main loop works for a while and sleeps again. The model knows the true share asleep; the stamps
 * must give it within the stated errors (CPU_ISR_EST per other wake, one tick per race: the tick between the stamp's
 * reads and the WFI), and the result rounded up (<= 1 permille). Windows of one second (20,000 ticks), across the
 * tick count's wrap. */
#include "../src/core/cpuload.h"

#define R5C_CYC 7200u

typedef struct {
    uint64_t t;                  /* cycles since the start */
    uint32_t tick0;              /* the tick count at t = 0 (any value: the wrap) */
    uint32_t idle;               /* the firmware's sum (cpu_slept) */
    uint64_t asleep;             /* the truth */
    uint32_t wakes_other, races;
} r5c_t;

static uint32_t r5c_tick(const r5c_t *m, uint64_t t) { return m->tick0 + (uint32_t)(t / R5C_CYC); }
static uint32_t r5c_cnt(uint64_t t) { return (uint32_t)(t % R5C_CYC); }

/* one window of `ticks` ticks from the model's time now (any phase, as main.c's second): work of busy_lo..busy_hi
 * cycles after each wake, another interrupt before the tick with probability `other` per sleep, a race with
 * probability `race`. Returns the firmware's permille; *truth the model's. */
static uint32_t r5c_window(r5c_t *m, uint32_t ticks, uint32_t busy_lo, uint32_t busy_hi, double other, double race,
                           double *truth)
{
    uint64_t start = m->t;
    uint32_t t_start = r5c_tick(m, start), n;
    m->idle = 0;
    m->asleep = 0;
    while (r5c_tick(m, m->t) - t_start < ticks) {
        uint64_t next_tick = (m->t / R5C_CYC + 1u) * R5C_CYC, wake, after;
        uint32_t k0, c0, c1, k1, isr;
        if (urand(0, 1) < race && m->t + 2u <= next_tick) {
            /* the stamp read just before the tick, its interrupt before the WFI: the core sleeps a whole tick more */
            m->t = next_tick - 2u;
            k0 = r5c_tick(m, m->t);
            c0 = r5c_cnt(next_tick - 1u);
            m->t = next_tick + 60u + rint_(101u);
            next_tick += R5C_CYC;
            m->races++;
        } else {
            k0 = r5c_tick(m, m->t);
            c0 = r5c_cnt(m->t);
        }
        if (urand(0, 1) < other && next_tick - m->t > 300u) {
            wake = m->t + 1u + (uint64_t)rint_((unsigned)(next_tick - m->t - 250u));
            isr = 40u + rint_(161u);
            m->wakes_other++;
        } else {
            wake = next_tick;
            isr = 60u + rint_(101u);
        }
        m->asleep += wake - m->t;
        after = wake + isr;
        c1 = r5c_cnt(after);
        k1 = r5c_tick(m, after);
        m->idle += cpu_slept(k1 != k0, c0, c1, R5C_CYC);
        m->t = after + busy_lo + (busy_hi > busy_lo ? rint_(busy_hi - busy_lo + 1u) : 0u);
    }
    n = r5c_tick(m, m->t) - t_start;
    *truth = 1000.0 - (double)m->asleep * 1000.0 / (double)(m->t - start);
    return cpu_permille(m->idle, n, R5C_CYC);
}

static void test_r5_cpu(void)
{
    r5c_t m;
    uint32_t pm, i;
    double truth;
    printf("cpu load: WFI stamps against a cycle model\n");

    /* the arithmetic */
    CHECK(cpu_slept(1, 700, 0, R5C_CYC) == 6500u, "tick wake: slept to the tick's start");
    CHECK(cpu_slept(0, 700, 1000, R5C_CYC) == 200u, "other wake: less the interrupt's estimate");
    CHECK(cpu_slept(0, 700, 750, R5C_CYC) == (uint32_t)-50, "other wake shorter than the estimate: -50 (mod 2^32)");
    CHECK(cpu_permille(0, 20000, R5C_CYC) == 1000u && cpu_permille(20000u * 7200u, 20000, R5C_CYC) == 0u &&
          cpu_permille(20000u * 3600u, 20000, R5C_CYC) == 500u && cpu_permille(20000u * 6948u, 20000, R5C_CYC) == 35u,
          "permille: 100 / 0 / 50 / 3.5 %%");
    CHECK(cpu_permille((uint32_t)-5000, 20000, R5C_CYC) == 1000u, "a sum below 0 (only short other wakes): 100 %%");
    CHECK(cpu_permille(20000u * 7300u, 20000, R5C_CYC) == 0u, "a sum above the window (estimates): 0 %%, no wrap");
    CHECK(cpu_permille(0u, 20000u, R5C_CYC) == 1000u, "no WFI in the window (a flash operation, a long pass): 100 %%");

    /* idle board: the tick's interrupt and a short pass each tick; the stamps see exactly the time asleep */
    memset(&m, 0, sizeof m);
    m.tick0 = 0xFFFFC000u;                   /* the count wraps in the first window */
    for (i = 0; i < 3u; i++) {
        pm = r5c_window(&m, 20000u, 40u, 120u, 0.0, 0.0, &truth);
        CHECK(fabs(pm - truth) <= 1.5, "idle window %u (wrap): %u permille, true %.2f", i, pm, truth);
    }
    printf("  idle: %.1f %% (true %.2f %%)\n", pm / 10.0, truth / 10.0);

    /* loads near 3.5 / 41 / 92 %: work after each wake, no other interrupts */
    {
        static const uint32_t lo[3] = { 100, 2900, 6000 }, hi[3] = { 300, 3000, 7100 };
        for (i = 0; i < 3u; i++) {
            pm = r5c_window(&m, 20000u, lo[i], hi[i], 0.0, 0.0, &truth);
            CHECK(fabs(pm - truth) <= 1.5, "load %u: %u permille, true %.2f", i, pm, truth);
            printf("  work %u..%u cycles per wake: %.1f %% (true %.2f %%)\n", lo[i], hi[i], pm / 10.0, truth / 10.0);
        }
    }

    /* other interrupts (the gate; slews at 100 kHz, up to 5 per tick): the estimate's error, bounded */
    for (i = 0; i < 3u; i++) {
        static const double p[3] = { 0.05, 0.4, 0.83 };
        uint32_t w0 = m.wakes_other, w;
        double bound;
        pm = r5c_window(&m, 20000u, 30u, 150u, p[i], 0.0, &truth);
        w = m.wakes_other - w0;
        bound = 1.5 + w * 100.0 * 1000.0 / (20000.0 * R5C_CYC);   /* each: |its time - CPU_ISR_EST| <= 100 cycles */
        CHECK(fabs(pm - truth) <= bound, "other wakes %u: %u permille, true %.2f (bound %.1f)", w, pm, truth, bound);
        printf("  %u other wakes/s: %.1f %% (true %.2f %%, bound +-%.1f)\n", w, pm / 10.0, truth / 10.0, bound / 10.0);
    }

    /* races (the tick between the stamp and the WFI): one tick each, too little */
    {
        uint32_t r0 = m.races, r;
        pm = r5c_window(&m, 20000u, 100u, 300u, 0.0, 0.001, &truth);
        r = m.races - r0;
        CHECK(r > 0u && fabs(pm - truth) <= 1.5 + r * 1000.0 / 20000.0, "%u races: %u permille, true %.2f", r, pm,
              truth);
        printf("  %u races in a second: %.1f %% (true %.2f %%)\n", r, pm / 10.0, truth / 10.0);
    }
}
