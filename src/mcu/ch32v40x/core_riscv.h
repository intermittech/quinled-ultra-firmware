/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32V407 reference manual V1.2, chapter 9 (PFIC p.89-106, SysTick p.80-82, CSRs p.66-78), and
 * the QingKe V3 processor manual V1.5 (V3V core).
 */
#ifndef QUINLED_QINGKE_V3_CORE_H
#define QUINLED_QINGKE_V3_CORE_H

#include <stdint.h>

/* Only the core registers used by this firmware, with byte offsets (RM p.89-90). Vectors reach 103: four words. */
typedef struct {
    uint8_t unused_000[0x48];
    volatile uint32_t CFGR;                 /* 0x048 */
    uint8_t unused_04c[4];
    volatile uint32_t VTFIDR;               /* 0x050: four 8-bit IDs, written as a word */
    uint8_t unused_054[12];
    volatile uint32_t VTFADDR[4];           /* 0x060: [31:1] handler, bit 0 enable */
    uint8_t unused_070[0x90];
    volatile uint32_t IENR[4];              /* 0x100 */
    uint8_t unused_110[0x70];
    volatile uint32_t IRER[4];              /* 0x180 */
    uint8_t unused_190[0xf0];
    volatile uint32_t IPRR[4];              /* 0x280 */
    uint8_t unused_290[0x170];
    volatile uint8_t priority[104];         /* 0x400: [7:4] implemented, bit 7 preempts (CSR 0x804 preempt = 01) */
} ql_pfic_regs;

/* 32-bit counter only: no high words at 0x0c / 0x14 (RM p.80) */
typedef struct {
    volatile uint32_t CTLR;                 /* 0x00 */
    volatile uint32_t SR;                   /* 0x04: bit 0 CNTIF */
    volatile uint32_t CNT;                  /* 0x08 */
    uint8_t unused_0c[4];
    volatile uint32_t CMP;                  /* 0x10 */
} ql_systick_regs;

#define PFIC ((ql_pfic_regs *)(uintptr_t)0xe000e000u)
#define SysTick ((ql_systick_regs *)(uintptr_t)0xe000f000u)

/* The firmware runs in machine mode (startup: mstatus.MPP = 3). CSR 0x800 is read-only on this core unless
 * CPU_RUN_CTLR.non_usta is set (RM p.72-77), so the critical section uses mstatus directly: MIE and MPIE, the
 * same two bits as the V20x's GINTENR section, then fence.i (QK3: after masking). */
static inline __attribute__((always_inline)) void __disable_irq(void)
{
    __asm__ volatile ("csrc mstatus, %0\n\tfence.i" : : "r" (0x88u) : "memory");
}

static inline __attribute__((always_inline)) void __enable_irq(void)
{
    __asm__ volatile ("csrs mstatus, %0" : : "r" (0x88u) : "memory");
}

/* A short critical section that keeps the caller's interrupt state (main loop or a handler): MIE only (QK3 2.2: with
 * nesting on, a handler runs with MIE set until the deepest level; mret restores it from MPIE, which stays as it is).
 * For a read-modify-write of a register another priority level also writes (the GPIO configuration words). */
static inline __attribute__((always_inline)) uint32_t irq_save(void)
{
    uint32_t s;
    __asm__ volatile ("csrrci %0, mstatus, 8\n\tfence.i" : "=r" (s) : : "memory");
    return s;
}

static inline __attribute__((always_inline)) void irq_restore(uint32_t s)
{
    __asm__ volatile ("csrs mstatus, %0" : : "r" (s & 8u) : "memory");
}

/* one pin's CNF / MODE nibble in CFGLR / CFGHR (RM p.128-129), atomically against every other writer of the word */
static inline __attribute__((always_inline)) void gpio_cfg_set(volatile uint32_t *cfgr, uint32_t sh, uint32_t cfg)
{
    const uint32_t s = irq_save();
    *cfgr = (*cfgr & ~(0xFu << sh)) | (cfg << sh);
    irq_restore(s);
}

static inline __attribute__((always_inline)) void NVIC_EnableIRQ(uint32_t id)
{
    PFIC->IENR[id >> 5] = 1u << (id & 31u);
}

static inline __attribute__((always_inline)) void NVIC_DisableIRQ(uint32_t id)
{
    PFIC->IRER[id >> 5] = 1u << (id & 31u);
}

static inline __attribute__((always_inline)) void NVIC_ClearPendingIRQ(uint32_t id)
{
    PFIC->IPRR[id >> 5] = 1u << (id & 31u);
}

static inline __attribute__((always_inline)) void NVIC_SetPriority(uint32_t id, uint8_t value)
{
    PFIC->priority[id] = value;
}

static inline __attribute__((always_inline)) void NVIC_SystemReset(void)
{
    /* PFIC_CFGR: KEYCODE 0xbeef, RESETSYS bit 7 (RM p.94) */
    PFIC->CFGR = (0xbeefu << 16) | (1u << 7);
}

#endif /* QUINLED_QINGKE_V3_CORE_H */
