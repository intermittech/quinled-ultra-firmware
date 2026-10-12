/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED dig2analog-ultra-5ch rev 0.1: CH32V203F8U6 (QFN20) pin map, see DESIGN_NOTES "MCU pin map". */
#ifndef BOARD_H
#define BOARD_H

/* HW_PWM_3TIMERS = 1 (default): all five channels on their own phase. Needs PB11 wired to PWM_B (in parallel
 * with PB15, which then stays an input). 0: the board as first drawn (B on TIM1 with R / G, 4 edge positions).
 *
 *  pin   function                     HW_PWM_3TIMERS = 1                       = 0                      net
 *  PA0   DIN (divided 5 V data)       unused input                             TIM2_ETR edge counter    DIN
 *  PA6   DIN sampler + edge source    SPI1_MISO 36 MHz + TIM3_CH1 capture      SPI1_MISO                DIN
 *  PA5   SPI1_SCK (unused, NC pad)    SPI1_SCK                                 SPI1_SCK
 *  PA7   board current (HW_ISENSE)    ADC_IN7, continuous conversion           (v0.1: NC pad)           ISENSE
 *  PA1   forwarding gate              GPIO, written by DMA on TIM4 CC2         TIM2_CH2 toggle          FWD_EN
 *  PA2   debug UART TX                USART2_TX                                                         TP5
 *  PA3   debug UART RX                USART2_RX                                                         TP6
 *  PA4   status LED D9                GPIO, active high                                                 STATUS
 *  PA8   PWM G                        TIM1_CH1 (right-aligned)                 TIM1_CH1                 PWM_G
 *  PA9   PWM R                        TIM1_CH2                                 TIM1_CH2                 PWM_R
 *  PB11  PWM B                        TIM2_CH4 (partial remap 2)               input                    PWM_B
 *  PB15  PWM B (first drawn)          input                                    TIM1_CH3N (alone)        PWM_B
 *  PB0   PWM W2                       TIM3_CH3 (right-aligned)                 TIM3_CH3                 PWM_W2
 *  PB1   PWM W1                       TIM3_CH4                                 TIM3_CH4                 PWM_W1
 *  PA13/PA14  SWD                                                                                        TP3 / TP4
 * Timers (HW_PWM_3TIMERS = 1): TIM1 / TIM2 / TIM3 = PWM at 0, 1/3 and 2/3 of the period; TIM4 (no pins) counts
 * DIN edges from TIM3's capture trigger (ITR2); SysTick = 20 kHz poll tick.
 * Gate driver inputs (R10-R14 100 k) and FWD_EN (R29) have pull-downs: everything is off while in reset. */
#ifndef HW_PWM_3TIMERS
#define HW_PWM_3TIMERS 1
#endif

/* HW_ISENSE = 1: the board revision with the input current sense (soldered fuse + 2 mohm shunt + CSA231, gain
 * 100, -> 1 k / 47 nF -> PA7 = ADC_IN7; core/isense.h). Default 0: v0.1 has no shunt (PA7 is an NC pad).
 * With 1: APB2 runs at 72 MHz (ADC <= 14 MHz: 72 / 6 = 12 MHz; TIM1 stays at 144 MHz, the timer clock doubles
 * when the APB2 prescaler is not 1; SPI1 / 2 keeps the 36 MHz DIN sampler). At boot hw_isense_init() checks
 * that the amplifier is really there (PA7 pulled up must stay low), so the same image runs on a v0.1 board. */
#ifndef HW_ISENSE
#define HW_ISENSE 0
#endif
#define ISENSE_FUSE_MA   10000u     /* soldered fuse: 10 A (decided 2026-10-12) */
#define ISENSE_BOARD_MA  8000u      /* board rating: default and maximum of the remote limit */
/* current-sense amplifier gain (V/V) on the 2 mohm shunt: 100 = CSA231PA (SOIC-8), 50 = CSA220NA (SOT-23-5, the
 * top-side layout of fix list v0.1 item 12). Sets the ADC scale (main.c: isense_set_gain). */
#ifndef ISENSE_AMP_GAIN
#define ISENSE_AMP_GAIN  100u
#endif

#define CH_R  0
#define CH_G  1
#define CH_B  2
#define CH_W1 3
#define CH_W2 4

#define DIN_RING_WORDS   2048u      /* 16-bit SPI frames: 32768 samples = 910 us at 36 MHz */
#define DIN_COPY_WORDS   1792u      /* copied per frame: up to 796 us (Ultra: 60 us + 320 bits + latency) */
#define POLL_HZ          20000u     /* poll tick: frame-end detection (core/frame_track.h), ring copy, 1 ms time base */

#define SETTINGS_ADDR    0x0800FF00u   /* last 256-byte page of the 64 KB flash */
#define SCENE_LOG_ADDR   0x0800F700u   /* power-on scene log: SCENE_LOG_PAGES (8) pages below the settings page */
#define CAL_PAGE_ADDR    0x0800F600u   /* current calibration (core/calstore.h): the top page of the app slot. The
                                          linker scripts end the code area here (vendor/wch/Link.ld, Link_app.ld); a
                                          firmware update erases only the new app's length, so the page survives */

#endif
