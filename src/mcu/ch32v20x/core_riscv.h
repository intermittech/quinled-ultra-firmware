/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32FV2x/V3x reference manual V2.5, chapter 9,
 * and its referenced QingKe V4 processor manual V1.5, chapters 3, 5 and 8.
 */
#ifndef QUINLED_QINGKE_CORE_H
#define QUINLED_QINGKE_CORE_H

#include <stdint.h>

/* Keep only the core registers used by this firmware, with byte offsets. */
typedef struct {
    uint8_t unused_000[0x48];
    volatile uint32_t CFGR;                 /* 0x048 */
    uint8_t unused_04c[4];
    volatile uint8_t VTFIDR[2];             /* 0x050 */
    uint8_t unused_052[14];
    volatile uint32_t VTFADDR[2];           /* 0x060 */
    uint8_t unused_068[0x98];
    volatile uint32_t IENR[2];              /* 0x100 */
    uint8_t unused_108[0x78];
    volatile uint32_t IRER[2];              /* 0x180 */
    uint8_t unused_188[0xf8];
    volatile uint32_t IPRR[2];              /* 0x280 */
    uint8_t unused_288[0x178];
    volatile uint8_t priority[54];          /* 0x400; highest used ID is 53 */
} ql_pfic_regs;

typedef struct {
    volatile uint32_t CTLR;                 /* 0x00 */
    volatile uint32_t SR;                   /* 0x04 */
    volatile uint64_t CNT;                  /* 0x08 */
    volatile uint64_t CMP;                  /* 0x10 */
} ql_systick_regs;

#define PFIC ((ql_pfic_regs *)(uintptr_t)0xe000e000u)
#define SysTick ((ql_systick_regs *)(uintptr_t)0xe000f000u)

/* GINTENR maps mstatus.MIE/MPIE. Both bits belong to a critical section.
 * fence.i completes the interrupt-mask change before following instructions.
 * The memory clobber also orders compiler accesses across each boundary.
 */
static inline __attribute__((always_inline)) void __disable_irq(void)
{
    __asm__ volatile ("csrc 0x800, %0\n\tfence.i" : : "r" (0x88u) : "memory");
}

static inline __attribute__((always_inline)) void __enable_irq(void)
{
    __asm__ volatile ("csrs 0x800, %0" : : "r" (0x88u) : "memory");
}

static inline __attribute__((always_inline)) void NVIC_EnableIRQ(uint32_t id)
{
    PFIC->IENR[id >> 5] = 1u << (id & 31u);
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
    /* PFIC_CFGR: KEYCODE=0xbeef, RESETSYS=1 (bit 7). */
    PFIC->CFGR = (0xbeefu << 16) | (1u << 7);
}

#endif /* QUINLED_QINGKE_CORE_H */
