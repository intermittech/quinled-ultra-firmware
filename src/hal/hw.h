/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V203 register-level drivers for the ultra board. */
#ifndef HW_H
#define HW_H

#include <stdint.h>
#include "../core/presets.h"
#include "../core/proto.h"
#include "../core/config.h"
#include "../core/scene.h"
#include "../core/store.h"
#include "../core/calstore.h"
#include "../core/energy.h"
#include "../core/flashrec.h"
#include "../core/ultra.h"
#include "board.h"

/* A compiler barrier, as __disable_irq / __enable_irq: no memory access moves across it, and values held in
 * registers are read again after it. For the hand-overs between the main loop and the ISRs that need no critical
 * section (the build uses LTO, so a call into another file is no barrier). */
#ifndef BARRIER                                  /* (the host soak test, test/test_soak.c, has its own) */
#define BARRIER() __asm volatile("" ::: "memory")
#endif
/* code that runs from SRAM (vendor/wch/Link_sections.ld .ramtext, copied by the startup with .data): the current trip
 * during a flash operation (hw.c flash_op), when nothing may be fetched from the flash */
#ifndef RAMCODE
#define RAMCODE __attribute__((section(".ramtext"), noinline))
#endif
/* the main loop's idle: the core sleeps until an interrupt (WFI; PFIC_SCTLR SLEEPDEEP = 0: sleep mode) */
#ifndef CPU_SLEEP
#define CPU_SLEEP() __asm volatile("wfi" ::: "memory")
#endif
/* (0.6.5, D1) in an interrupt handler: the interrupted PC (mepc). The poll tick has the lowest priority, so there it is
 * the main loop's place (the reset breadcrumbs, main.c bcrumb). */
#ifndef TRAP_PC
static inline uint32_t hw_trap_pc(void)
{
    uint32_t v;
    __asm volatile("csrr %0, mepc" : "=r"(v));
    return v;
}
#define TRAP_PC() hw_trap_pc()
#endif

/* PWM dither tables, written by the main loop, read by DMA once per PWM period (into the CCRs). pwm_table()
 * tells where a channel's values go: entry i of channel ch is base[i * stride]. */
extern uint16_t pwm_tab1[PATTERN_MAX_LEN * 4];   /* TIM1: G, R; spread: + period */
extern uint16_t pwm_tab2[PATTERN_MAX_LEN * 6];   /* TIM2: B; spread: + period */
extern uint16_t pwm_tab3[PATTERN_MAX_LEN * 6];   /* TIM3: W2, W1; spread: + period */
uint16_t *pwm_table(uint8_t ch, uint8_t *stride);
/* spread spectrum (STAGGER_*_SPREAD): period - 1 of every table entry, else NULL */
const uint16_t *pwm_spread_arr(void);
extern uint16_t din_ring[DIN_RING_WORDS];
extern uint16_t din_copy[DIN_COPY_WORDS];

void hw_init(void);
/* the stack's high-water mark: stack_paint() first thing in main fills the free stack with a pattern, stack_used()
 * gives the bytes below its top ever written since (2048 = all of it: an overflow) */
void stack_paint(void);
uint32_t stack_used(void);
uint32_t hw_uid_hash(void);                      /* a hash of the chip's 96-bit unique id (ESIG, 0x1FFFF7E8) */
#ifndef HW_UID                                   /* (the host soak test, test/soak/ch32v20x.h, has its own) */
#define HW_UID ((const uint32_t *)0x1FFFF7E8u)   /* ... the id itself: three words (RM 31.2: UID registers 1..3) */
#endif
uint32_t hw_reset_flags(void);                   /* RCC reset flags captured at boot */
void watchdog_kick(void);
/* (re)start the PWM timers: period `steps` counts (1440..65535), table length `len`. stagger: TIM1 / TIM2 / TIM3 a
 * third of a period apart, G and W2 right-aligned: R, B, W1 turn on at 0, 1/3, 2/3 of the period, G and W2 turn off
 * at 0 and 2/3 -> no two channels turn on together. */
void pwm_start(uint32_t steps, uint8_t len, uint8_t stagger);
/* main.c, for pwm_start: write every channel's table (pwm_table) for the new period / mode before the timers run */
void pwm_fill(void);
uint8_t pwm_right_aligned(uint8_t ch);          /* channel uses PWM mode 2 (table holds steps - duty) */
/* adaptive stagger: move TIM2 / TIM3 to these offsets (Q16 of the period, relative to TIM1), gradually */
void pwm_set_phase(uint16_t o2, uint16_t o3);
void gate_rearm(const gate_plan_t *g);          /* frame end: set FWD_EN start level and toggle points */
void gate_force_closed(void);
/* DIN falling edges since the last gate_rearm(), and the poll tick acknowledge (main.c, needs ch32v20x.h) */
#define DIN_EDGES() ((uint16_t)TIM4->CNT)
#define POLL_ACK()  (SysTick->SR = 0u)
/* HCLK cycles into the poll tick (SysTick counts up to CMP, then from 0; the low word of its 64-bit CNT) */
#define POLL_CNT()  (*(volatile uint32_t *)&SysTick->CNT)
uint32_t din_ring_pos(void);                    /* words written so far into din_ring (0..RING-1) */
/* ADC1 (hw.c "ADC"): PA7, the current sense (core/isense.h), converted continuously (71.5 + 12.5 cycles at 12 MHz
 * = 7 us); ISENSE_RAW() is the newest result (main.c / the poll tick, needs ch32v20x.h). The injected group, started
 * by hw_adc_ms() from the poll tick once per ms: the voltage channels (core/vsense.h), or once a second the MCU
 * temperature (core/mcutemp.h). */
void hw_adc_init(void);
/* poll tick, every 1 ms: reads the last sequence, starts the next; 1 = the temperature's (84 us without a PA7
 * conversion: the next tick would read this tick's PA7 again) */
uint32_t hw_adc_ms(void);
#define ISENSE_RAW() ((uint32_t)(ADC1->RDATAR & 0xFFFu))
/* the voltage channels since the last take: sums of pairs of conversions (sum2[0] PA4 supply / bus, sum2[1] PA0
 * spare / V_B), n pairs each */
void hw_vsense_take(uint32_t sum2[2], uint32_t *n);
/* MCU temperature: the temperature sensor and VREFINT twice each (~84 us), converted at the next ms after
 * hw_temp_start(); hw_temp_read() gives the sums of the two conversions each (0 = not done). */
void hw_temp_start(void);
uint8_t hw_temp_read(uint32_t *ts2, uint32_t *vref2);
/* (0.6.5, D6) fault counters (wrap): [0] ms in which no regular PA7 conversion started (the current sense stale: the
 * trip saw old samples), [1] injected sequences (voltages / temperature) not done within their ms (hw_adc_ms, the
 * poll tick); [2..8] DMA1 channel 1..7 transfer errors (hw_dma_check, once a second from the main loop: TEIF counted
 * and cleared; a transfer error disables its channel). The console shows them when one is not 0. */
#define HW_DIAG_N 9u
extern volatile uint16_t hw_diag[HW_DIAG_N];
void hw_dma_check(void);
/* all five PWM outputs low at once (OCxM = force inactive; ISR-safe), or back to PWM. The tables keep running.
 * Call pwm_force_off(0) with interrupts off (it must not undo a trip the poll tick raised meanwhile). */
void pwm_force_off(uint8_t off);
/* the poll tick's fast trip (0.6.4, claude-cpu): pwm_force_off(1) inline, so that the tick keeps no stack frame; also
 * the SRAM trip (hw_flash_trip). OCxM = 4 (force inactive) of G / R (TIM1 CH1 / CH2), B (TIM2 CH4), W2 / W1 (TIM3 CH3 /
 * CH4); TIM2 CH3 (B's DMA trigger) and TIM3 CH1 (the DIN capture) stay. The host soak's register model has no PWM
 * timers: the call there. */
#if defined(TIM1)
extern volatile uint8_t pwm_forced;
#define PWM_TRIP() do {                                                                \
        pwm_forced = 1u;                                                               \
        TIM1->CHCTLR1 = (uint16_t)((TIM1->CHCTLR1 & ~0x7070u) | 0x4040u);             \
        TIM2->CHCTLR2 = (uint16_t)((TIM2->CHCTLR2 & ~0x7000u) | 0x4000u);             \
        TIM3->CHCTLR2 = (uint16_t)((TIM3->CHCTLR2 & ~0x7070u) | 0x4040u);             \
    } while (0)
#else
#define PWM_TRIP() pwm_force_off(1)
#endif
void led_set(uint8_t on);
/* (0.6.7) the test jig's pads' levels (core/testjig.h tj_pads: TP6 counts only while no byte arrives): TP6 = PA3, the
 * console's RX (pull-up), both boards; TP9 = PB10 (pull-up), the dig2analog-ultra only (the receiver's return-channel
 * TX). uart_rx_pos: where the UART's RX DMA writes next */
#define HW_TP6_LOW() ((uint8_t)((GPIOA->INDR & (1u << 3)) == 0u))
#if BOARD == BOARD_D2A_ULTRA
#define HW_TP9_LOW() ((uint8_t)((GPIOB->INDR & (1u << 10)) == 0u))
#else
#define HW_TP9_LOW() ((uint8_t)0u)
#endif
#if CONSOLE_FULL
uint32_t uart_rx_pos(void);                     /* (the release console has no RX: TP6 is a GPIO alone) */
#endif
/* the flash: the stores (core/flashrec.h) on hw.c's flash_op. During an operation the tick runs from SRAM: main.c's
 * flash_poll_isr (the time base and the fast trip), which trips through hw_flash_trip (SRAM) */
void flash_poll_isr(void);
void hw_flash_trip(void);
/* (review round 6) VDD below the PVD's 2.9 V (hw_init): flash_op would refuse (FLASH_ERR_VDD), so main.c's flash_may
 * admits no write then: a store's budget token and its A/B entry are not used up by an operation that never starts */
uint32_t hw_vdd_low(void);
/* (0.6.8) the user option bytes (RM 32.6): flash_op(HW_OB_ADDR, ob, 8, HW_OB_ADDR) erases all 8 and programs ob[0..7]
 * (the low bytes: the hardware writes each inverse). hw_boot_open: the bootloader's sectors 0-1 (4 KB each) are not
 * write-protected (FLASH->WPR, loaded at a system reset); hw_option_bytes: the 8 option bytes' low bytes into ob
 * (ob[0] RDPR: 0xA5 no read protection). 0.6.9 (R7-06): read at HCLK / 2 (only before hw_init: it changes the clock),
 * each byte checked against its inverse and read twice; returns 1 when all 8 agree, 0 = not to be trusted */
#define HW_OB_ADDR 0x1FFFF800u
uint32_t hw_boot_open(void);
uint32_t hw_option_bytes(uint16_t ob[8]);
/* (0.6.5) the record pages' reads at boot, before hw_init: open = HCLK / 2 (the upper CodeFlash, board.h), 0 = back */
void hw_records_window(uint32_t open);
#if BOARD_RETURN
/* the return channel (core/rc.h; the window runs in main.c's poll ISR). rc_hw_dir: RC_DIR_ANSWER turn around (the
 * input transceiver drives upstream from the relay node, the output one listens), RC_DIR_QUIET both listen (the
 * turnaround or burst check failed), RC_DIR_FORWARD forward again (a burst still running is cut, the line released) */
enum { RC_DIR_FORWARD = 0, RC_DIR_ANSWER = 1, RC_DIR_QUIET = 2 };
void     rc_hw_dir(uint32_t mode);
/* poll ISR: the line HIGH (USART3 idles high on PB10), after RC_PREAMBLE_CYC DIN read back: LOW = another driver on
 * the segment (1: aborted, the line released); else a 0xFF (core/rc.h RC_LEAD_BYTES) and the n bytes at brr by
 * interrupt, the line released after them
 * (n 0: the turnaround check, the line released at once) */
uint32_t rc_hw_send(const uint8_t *b, uint32_t n, uint16_t brr);
#if BOARD == BOARD_DP_UD
/* (BOARD 3: one transceiver, DE + /RE tied on PB14, RO high-Z while DE is high: nothing to read back). rc_hw_dir:
 * RC_DIR_ANSWER = BLANK_N LOW (PA1), then DE high (the line HIGH: USART3 idles high on PB10); FORWARD / QUIET = a
 * burst still running cut, DE low (BLANK_N stays: the window logic raises it, hw_dline.h dl_hw_blank). rc_hw_send:
 * after RC_PREAMBLE_CYC of HIGH a 0xFF and the n bytes by interrupt, DE low at the last stop bit (TC); returns 0 (no
 * check). rc_hw_busy: the burst still runs (DE high) */
uint32_t rc_hw_busy(void);
void rc_hw_answer_local(void);                 /* the answer while a local effect runs: DE high, BLANK_N left alone (udfx.h) */
void dl_tx_done(void);                         /* main_ud.c: the burst's TC (DE low), from rc_uart_isr */
#endif
#endif
/* UART: TX by DMA from a buffer (non-blocking unless the buffer is full); the full console's RX by DMA into a ring */
#if CONSOLE_FULL
int  uart_getc(void);                           /* -1 if nothing new; -2 once after a flash operation dropped
                                                   received bytes: cancel the line being typed */
#endif
void uart_write(const char *s);
void uart_kick(void);                           /* start sending what uart_write buffered */
uint32_t uart_busy(void);
/* (0.6.5) bytes uart_write takes now without waiting (the buffer being filled, and the other one when idle) */
uint32_t uart_room(void);                       /* something queued or on the line (flash_op waits for it) */

#endif
