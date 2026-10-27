/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* `perf`: cycle counts on the silicon, to check tools/measure.py's emulated numbers on the bench. A bench
 * build only (tools/build_target.sh PERF=1 or PERF=2: PERF_BUILD); the release has none of it (the macros below are
 * empty: the same image byte for byte).
 *  - Measured: the poll ISR (every tick, all its paths), the slew ISRs, the gate ISR, the Ultra render, the slot decode
 *    and the Ultra header decode, the table builds (build_all), isense_ms and the stagger search. Per path: the calls,
 *    the mean and the worst since the last `perf` (the console prints them and starts again).
 *  - The clock: PERF=1 the core's cycle counter mcycle (CSR 0xB00; BENCH: that the V4B has it, and whether it counts
 *    in WFI: `perf` prints mcycle's count against the time, 100 % = it runs on while the core sleeps); PERF=2 SysTick
 *    (HCLK, the tick count x POLL_CYC + its count; works on every V4), for when mcycle is missing. Both count HCLK
 *    cycles, the time of the interrupts that preempt a path included (the emulator's numbers have none).
 *  - The stamps cost ~10-20 cycles per path (the poll ISR's every-tick path grows by that in this build); 32-bit stamps
 *    wrap after 29.8 s: a path never takes that long. */
#ifndef PERF_H
#define PERF_H

#include <stdint.h>

#ifndef PERF_BUILD
#define PERF_BUILD 0
#endif

enum {
    PERF_POLL, PERF_SLEW, PERF_GATE, PERF_RENDER, PERF_DECODE, PERF_HEADER, PERF_TABLES, PERF_ISENSE, PERF_STAGGER,
    PERF_N
};
#define PERF_NAMES "poll ISR\0slew ISR\0gate ISR\0render\0slot decode\0header decode\0tables\0isense_ms\0stagger"

typedef struct {
    uint32_t n, max;
    uint64_t sum;
} perf_t;

#if PERF_BUILD
extern perf_t perf[PERF_N];
/* the clock: HCLK cycles (see above) */
#ifndef PERF_NOW
#if PERF_BUILD == 1
static inline uint32_t perf_mcycle(void)
{
    uint32_t c;
    __asm volatile("csrr %0, mcycle" : "=r"(c));
    return c;
}
#define PERF_NOW() perf_mcycle()
#else
uint32_t perf_systick(void);               /* main.c */
#define PERF_NOW() perf_systick()
#endif
#endif
static inline void perf_add(uint32_t i, uint32_t t0)
{
    uint32_t d = PERF_NOW() - t0;
    perf_t *p = &perf[i];
    p->n++;
    p->sum += d;
    if (d > p->max) {
        p->max = d;
    }
}
/* a stamp at the start of a path (a declaration), its end: counted into perf[i]; a call with a value, a statement */
#define PERF_T(v)          uint32_t v = PERF_NOW()
#define PERF_ADD(i, v)     perf_add((i), (v))
#define PERF_CALL(i, call) __extension__({ PERF_T(perf_t0_); __typeof__(call) perf_r_ = (call); \
                                           PERF_ADD((i), perf_t0_); perf_r_; })
#define PERF_DO(i, stmt)   do { PERF_T(perf_t0_); stmt; PERF_ADD((i), perf_t0_); } while (0)
/* the counts since the last call into dst, and again from 0 (main.c; interrupts off meanwhile); the cycles counted by
 * the clock meanwhile (*clock) and the time in poll ticks (*ticks) */
void perf_take(perf_t *dst, uint32_t *clock, uint32_t *ticks);
#else
#define PERF_T(v)
#define PERF_ADD(i, v)
#define PERF_CALL(i, call) (call)
#define PERF_DO(i, stmt)   stmt
#endif

#endif
