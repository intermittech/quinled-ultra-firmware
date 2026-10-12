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
#include "board.h"

/* PWM dither tables, written by the main loop, read by DMA once per PWM period (into the CCRs). pwm_table()
 * tells where a channel's values go: entry i of channel ch is base[i * stride]. */
extern uint16_t pwm_tab1[PATTERN_MAX_LEN * 4];   /* TIM1: G, R (+ B with HW_PWM_3TIMERS 0); spread: + period */
extern uint16_t pwm_tab2[PATTERN_MAX_LEN * 6];   /* TIM2: B (HW_PWM_3TIMERS 1); spread: + period */
extern uint16_t pwm_tab3[PATTERN_MAX_LEN * 6];   /* TIM3: W2, W1; spread: + period */
uint16_t *pwm_table(uint8_t ch, uint8_t *stride);
/* spread spectrum (STAGGER_*_SPREAD): period - 1 of every table entry, else NULL */
const uint16_t *pwm_spread_arr(void);
extern uint16_t din_ring[DIN_RING_WORDS];
extern uint16_t din_copy[DIN_COPY_WORDS];

void hw_init(void);
uint32_t hw_reset_flags(void);                   /* RCC reset flags captured at boot */
void watchdog_kick(void);
/* (re)start the PWM timers: period `steps` counts (1440..65535), table length `len`. stagger:
 *   HW_PWM_3TIMERS 1: TIM1 / TIM2 / TIM3 a third of a period apart, G and W2 right-aligned: R, B, W1 turn on at
 *                     0, 1/3, 2/3 of the period, G and W2 turn off at 0 and 2/3 -> no two channels turn on together;
 *   HW_PWM_3TIMERS 0: TIM3 half a period after TIM1, G / W2 right-aligned -> 4 edge positions (R and B together). */
void pwm_start(uint32_t steps, uint8_t len, uint8_t stagger);
uint8_t pwm_right_aligned(uint8_t ch);          /* channel uses PWM mode 2 (table holds steps - duty) */
/* adaptive stagger: move TIM2 / TIM3 to these offsets (Q16 of the period, relative to TIM1), gradually */
void pwm_set_phase(uint16_t o2, uint16_t o3);
void gate_rearm(const gate_plan_t *g);          /* frame end: set FWD_EN start level and toggle points */
void gate_force_closed(void);
/* DIN falling edges since the last gate_rearm(), and the poll tick acknowledge (main.c, needs ch32v20x.h) */
#if HW_PWM_3TIMERS
#define DIN_EDGES() ((uint16_t)TIM4->CNT)
#define POLL_ACK()  (SysTick->SR = 0u)
#else
#define DIN_EDGES() ((uint16_t)TIM2->CNT)
#define POLL_ACK()  (TIM4->INTFR = (uint16_t)~TIM_UIF)
#endif
uint32_t din_ring_pos(void);                    /* words written so far into din_ring (0..RING-1) */
/* MCU temperature (core/mcutemp.h): ADC1's injected group converts the temperature sensor and VREFINT twice each
 * (~84 us), started by hw_temp_start(); hw_temp_read() gives the sums of the two conversions each (0 = not done). */
void hw_temp_start(void);
uint8_t hw_temp_read(uint32_t *ts2, uint32_t *vref2);
#if HW_ISENSE
/* board current sense (core/isense.h): ADC1 converts PA7 continuously (71.5 + 12.5 cycles at 12 MHz = 7 us);
 * ISENSE_RAW() is the newest result (main.c / the poll tick, needs ch32v20x.h). hw_isense_init() sets up the ADC
 * (PA7 only when the amplifier is present; the temperature always) and returns 1 when it is there. */
uint8_t hw_isense_init(void);
#define ISENSE_RAW() ((uint32_t)(ADC1->RDATAR & 0xFFFu))
/* all five PWM outputs low at once (OCxM = force inactive; ISR-safe), or back to PWM. The tables keep running.
 * Call pwm_force_off(0) with interrupts off (it must not undo a trip the poll tick raised meanwhile). */
void pwm_force_off(uint8_t off);
#else
void hw_temp_init(void);                        /* the ADC for the temperature only */
#endif
void led_set(uint8_t on);
/* settings + their extension (store_ext_t: current sense limit / calibration; all 0xFF from a v1 record) */
int  settings_load(settings_t *s, store_ext_t *ext);
/* 1-2 ms (append), +16 ms when the page needs erasing; interrupts are off meanwhile. Returns that time in us
 * (estimate, BENCH: measure), so the caller can move its time base along. */
uint32_t settings_store(const settings_t *s, const store_ext_t *ext);
/* power-on scene log (same stall as above, ~1 ms + 16 ms every 16th write) */
int  scene_load(scene_t *sc);
uint32_t scene_store(const scene_t *sc);
uint32_t hw_flash_half(uint32_t addr, uint16_t v);    /* firmware update: one status record (stall in us) */
/* current calibration page (core/calstore.h): the newest record (-1: none, or cleared); store one (levels 0 =
 * clear), ~1 ms + 16 ms every 4th write. *count: calibrations stored on this board */
int  cal_load(isense_cal_t *c, uint16_t *count);
uint32_t cal_store(const isense_cal_t *c, uint16_t *count);
/* UART: RX by DMA into a ring, TX by DMA from a buffer (non-blocking unless the buffer is full) */
int  uart_getc(void);                           /* -1 if nothing new */
void uart_write(const char *s);
void uart_kick(void);                           /* start sending what uart_write buffered */

#endif
