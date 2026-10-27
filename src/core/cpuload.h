/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CPU load: the share of the time the core is not asleep in the main loop's idle WFI (main.c), shown in
 * `status` (the last second and the busiest second since the last status) and `watch`.
 *
 * Measured with the poll tick, which wakes the core and runs while it sleeps: before each WFI the main loop notes the
 * tick count and SysTick's count (HCLK cycles into the tick, 0 .. cyc - 1), and again after it.
 *  - Woken by the tick (the count moved on): the core slept from the first stamp to that tick's start, exactly. The
 *    wake-up and the tick's interrupt after it count as busy.
 *  - Woken by another interrupt within the same tick (the forwarding gate, a phase slew): it slept until that
 *    interrupt, which the second stamp follows by the interrupt's own time: CPU_ISR_EST cycles are taken off (the
 *    measured slews: 87 / 165 cycles plus the entry; the gate's are shorter). An error of a few dozen cycles per
 *    such wake, only while frames run or a slew moves.
 *  - Flash operations (hw.c flash_op: no WFI, the SRAM tick) count as busy, for as long as they take.
 *  - A tick between the two reads of a stamp (a window of one instruction) counts one tick (50 us) too much or too
 *    little: rare, and below the display's 0.1 %.
 * The window is a second of ticks, so the result is the time share whatever the clock did meanwhile. Pure logic,
 * host-tested (test/test_r5_cpu.c), and main.c's wiring in the soak (scenario r5cpu). */
#ifndef CPULOAD_H
#define CPULOAD_H

#include <stdint.h>

#define CPU_ISR_EST 100u               /* cycles of a gate / slew interrupt with its entry and exit (estimate) */

/* the cycles asleep in one WFI, from the stamps before it (c0) and after it (c1, SysTick's count) and whether the tick
 * count moved meanwhile (ticked), cyc cycles per tick. Without a tick it may come out below 0 (an interrupt shorter
 * than the estimate): the sum over a second stays right modulo 2^32 (cpu_permille reads it as signed). A second tick
 * before the wake-up (one between c0 and the WFI) counts as one. */
static inline uint32_t cpu_slept(uint32_t ticked, uint32_t c0, uint32_t c1, uint32_t cyc)
{
    return (ticked ? cyc : c1 - CPU_ISR_EST) - c0;
}

/* the load of a window of n ticks of cyc cycles with idle cycles asleep: busy in permille (0..1000, rounded up) */
static inline uint32_t cpu_permille(uint32_t idle, uint32_t n, uint32_t cyc)
{
    idle = (int32_t)idle < 0 ? 0u : idle / n;   /* asleep per tick (n = 0: all ones, no trap on RISC-V: 0 %) */
    return idle >= cyc ? 0u : 1000u - idle * 1000u / cyc;
}

#endif
