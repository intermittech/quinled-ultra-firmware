/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* `perf` (core/perf.h) on the host: the core built as a PERF=2 bench build (build_host.cmd: /DPERF_BUILD=2)
 * with a scripted clock in place of main.c's SysTick clock. The counting (perf_add: calls, sum, worst, across the clock's
 * 32-bit wrap), the console's `perf` (every path, the mean, the clock's cycles per tick, at most 1 KB) and that it starts
 * again from 0. The release has none of it: its image is the same byte for byte with and without (prove.sh, md5). */
#include <stdio.h>
#include <string.h>

#include "../src/core/cli.h"
#include "../src/core/config.h"
#include "../src/core/flashrec.h"
#include "../src/core/perf.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
                                       printf("\n"); } } while (0)

/* core/flashrec.c's flash primitive (not used here) */
uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    (void)addr;
    (void)src;
    (void)halfwords;
    (void)erase_addr;
    return FLASH_NONE;
}

/* main.c's side of a PERF build: the counts, the clock, perf_take */
perf_t perf[PERF_N];
static uint32_t clk, clk0, ticks_now, ticks0, takes;
uint32_t perf_systick(void) { return clk; }
void perf_take(perf_t *dst, uint32_t *clock, uint32_t *ticks)
{
    uint32_t i;
    *clock = clk - clk0;
    *ticks = ticks_now - ticks0;
    clk0 = clk;
    ticks0 = ticks_now;
    for (i = 0; i < PERF_N; i++) {
        dst[i] = perf[i];
        perf[i].n = perf[i].max = 0;
        perf[i].sum = 0;
    }
    takes++;
}

static char out[4096];
static void cap(const char *s) { strncat(out, s, sizeof out - strlen(out) - 1); }
static cfg_result_t apply(uint8_t cmd, uint8_t arg)
{
    cfg_result_t r;
    (void)cmd;
    (void)arg;
    memset(&r, 0, sizeof r);
    return r;
}

int main(void)
{
    cli_t c;
    cli_status_t st;
    settings_t set;
    energy_t en;
    uint32_t i, t0;
    const char *p;
    unsigned n, mean, max;
    printf("perf: a PERF=2 build's counting and console command\n");

    /* the counting: one path, durations 10 / 30 / 20 cycles, the third across the clock's wrap */
    clk = 1000u;
    t0 = PERF_NOW();
    clk += 10u;
    perf_add(PERF_RENDER, t0);
    t0 = PERF_NOW();
    clk += 30u;
    perf_add(PERF_RENDER, t0);
    clk = 0xFFFFFFF6u;
    t0 = PERF_NOW();
    clk += 20u;                                          /* (wraps to 10) */
    perf_add(PERF_RENDER, t0);
    CHECK(perf[PERF_RENDER].n == 3u && perf[PERF_RENDER].sum == 60u && perf[PERF_RENDER].max == 30u,
          "counting: 3 calls, sum 60, worst 30 (%u, %u, %u)", perf[PERF_RENDER].n, (uint32_t)perf[PERF_RENDER].sum,
          perf[PERF_RENDER].max);
    /* a long run: the 64-bit sum past 2^32 */
    for (i = 0; i < 3000000u; i++) {
        t0 = PERF_NOW();
        clk += 1700u;
        perf_add(PERF_POLL, t0);
    }
    CHECK(perf[PERF_POLL].sum == 3000000ull * 1700u, "the sum past 2^32 (%llu)",
          (unsigned long long)perf[PERF_POLL].sum);

    /* the console: `perf` (as main.c runs a line: once) */
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    memset(&en, 0, sizeof en);
    settings_default(&set);
    st.version = "test";
    st.steps = 5760;
    st.en = &en;
    c.write = cap;
    c.apply = apply;
    c.set = &set;
    c.eff = &set;
    c.st = &st;
    clk0 = clk - 20000u * 7200u;                         /* the clock counted all the time over 1 s of ticks */
    ticks_now = 20000u;
    out[0] = 0;
    cli_exec(&c, "perf");
    printf("%s", out);
    CHECK(takes == 1u && strlen(out) <= 1024u, "one take, at most 1 KB (one part of the console's output: %u bytes)",
          (uint32_t)strlen(out));
    CHECK(strstr(out, "perf (cycles, SysTick) over 1.0 s: the clock 7200 per poll tick\r\n") != NULL, "the header");
    p = strstr(out, "  render: ");
    CHECK(p && sscanf(p, "  render: %u, mean %u, max %u", &n, &mean, &max) == 3 && n == 3u && mean == 20u &&
          max == 30u, "render: 3, mean 20, max 30");
    p = strstr(out, "  poll ISR: ");
    CHECK(p && sscanf(p, "  poll ISR: %u, mean %u, max %u", &n, &mean, &max) == 3 && n == 3000000u && mean == 1700u &&
          max == 1700u, "poll ISR: the mean of a 64-bit sum");
    for (i = 0, p = out; (p = strstr(p, ": ")) != NULL; p++) {
        i++;
    }
    CHECK(i == PERF_N + 1u && strstr(out, "  stagger: 0, mean 0, max 0\r\n"), "every path (%u), an empty one", i);
    CHECK(perf[PERF_RENDER].n == 0u && perf[PERF_POLL].sum == 0u, "starts again from 0");
    out[0] = 0;
    cli_exec(&c, "perf");
    CHECK(strstr(out, "over 0.0 s: the clock 0 per poll tick") && strstr(out, "  render: 0, mean 0, max 0"),
          "a second perf right after: nothing counted (%s)", out);

    printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
