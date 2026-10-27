/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V407 register-level drivers for the Diff-Power-UR-Digital-M (BOARD 3): the V20x HAL interface (hw.h) that
 * main_ur_digital_m.c and the core use, on this family's peripherals. */
#ifndef HW_H
#define HW_H

#include <stdint.h>
#include "../../core/presets.h"
#include "../../core/proto.h"
#include "../../core/config.h"
#include "../../core/scene.h"
#include "../../core/store.h"
#include "../../core/calstore.h"
#include "../../core/energy.h"
#include "../../core/flashrec.h"
#include "../../core/ultra.h"
#include "../../target.h"

#if BOARD != BOARD_UR_DIGITAL_M
#error "The CH32V40x HAL serves the Diff-Power-UR-Digital-M only"
#endif
#if CONSOLE_FULL
#error "CONSOLE_FULL: the V407 board wires no console RX (PA7 is unconnected)"
#endif

#ifndef BARRIER
#define BARRIER() __asm volatile("" ::: "memory")
#endif
#ifndef RAMCODE
#define RAMCODE MCU_RAMCODE
#endif
#ifndef CPU_SLEEP
#define CPU_SLEEP() __asm volatile("wfi" ::: "memory")
#endif
#ifndef TRAP_PC
static inline uint32_t hw_trap_pc(void)
{
    uint32_t v;
    __asm volatile("csrr %0, mepc" : "=r"(v));
    return v;
}
#define TRAP_PC() hw_trap_pc()
#endif

void hw_init(void);
void stack_paint(void);
uint32_t stack_used(void);
uint32_t hw_uid_hash(void);
#ifndef HW_UID
#define HW_UID ((const uint32_t *)0x1FFFF7E8u)   /* ESIG UID registers 1..3 (RM p.577) */
#endif
uint32_t hw_reset_flags(void);
void watchdog_kick(void);
#define POLL_ACK()  (SysTick->SR = 0u)
#define POLL_CNT()  MCU_POLL_COUNT()
/* ADC1: PA2 (IN2) the current sense continuously; the injected group per ms from the poll tick (hw.c) */
void hw_adc_init(void);
uint32_t hw_adc_ms(void);
#define ISENSE_RAW() ((uint32_t)(ADC1->RDATAR & 0xFFFu))
void hw_vsense_take(uint32_t sum2[2], uint32_t *n);
void hw_temp_start(void);
uint8_t hw_temp_read(uint32_t *ts2, uint32_t *vref2);
/* fault counters as the V20x: [0] regular ADC stale, [1] injected late, [2..8] DMA1 channel 1..7 transfer errors */
#define HW_DIAG_N 9u
extern volatile uint16_t hw_diag[HW_DIAG_N];
void hw_dma_check(void);
void led_set(uint8_t on);
/* the test jig's pad on PA12 (input with pull-up) */
#define HW_TP6_LOW() ((uint8_t)((GPIOA->INDR & (1u << 12)) == 0u))
#define HW_TP9_LOW() ((uint8_t)0u)
void flash_poll_isr(void);
void hw_flash_trip(void);
uint32_t hw_vdd_low(void);
/* the user option bytes (RM p.590-591): the same eight byte / inverse pairs as the V20x */
#define HW_OB_ADDR 0x1FFFF800u
uint32_t hw_boot_open(void);
uint32_t hw_option_bytes(uint16_t ob[8]);
void hw_records_window(uint32_t open);
/* the HSI trim (RCC_CTLR HSITRIM[4:0], RM p.23): the hub-locked trim (core hsitrim) moves it */
uint32_t hw_hsi_trim(void);
void hw_hsi_trim_set(uint32_t trim);
/* Return channel: USART7 TX on PA9 (remap 11), input DE on PA11, BLANK_N on PA8 (hw.c) */
enum { RC_DIR_FORWARD = 0, RC_DIR_ANSWER = 1, RC_DIR_QUIET = 2 };
void     rc_hw_dir(uint32_t mode);
uint32_t rc_hw_send(const uint8_t *b, uint32_t n, uint16_t brr);
uint32_t rc_hw_busy(void);
void rc_hw_answer_local(void);
uint32_t rc_hw_relay(uint32_t on);            /* the CHAIN relay window: 1 = turnaround refused, the segment above still driven */
void dl_tx_done(void);
/* the console: USART1 TX on PA6, by its TXE interrupt from a ring */
void uart_write(const char *s);
void uart_kick(void);
uint32_t uart_busy(void);
uint32_t uart_room(void);

#endif
