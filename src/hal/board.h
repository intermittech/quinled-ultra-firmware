/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED dig2analog-ultra-5ch rev 0.4: CH32V203F8U6 (QFN20) pin map (wiki: Pin map). The only dig2analog-ultra
 * revision built. The QuinLED Diff-Power ultra receiver uses the same MCU and pin roles (BOARD_DP_RX: compile-time
 * definitions and core/rxpower.h only so far; there is no receiver hardware yet).
 *
 *  pin      port       function                                                    net
 *   1       PA0        ADC_IN0, injected group (1 kHz): a spare analog input        TP8      (receiver: buck output V_B)
 *  20       PA6        DIN sampler: SPI1_MISO 36 MHz + TIM3_CH1 capture (edge source for TIM4)        DIN
 *   2       PA1        forwarding gate: GPIO, written by DMA on TIM4 CC2                              FWD_EN
 *   3 / 4   PA2 / PA3  console: USART2 TX / RX                                                        TP5 / TP6
 *   5       PA4        ADC_IN4, injected group (1 kHz): the supply, R36 220 k / R37 10 k / C29 1 uF  VSENSE
 *                      (receiver: the bus)
 *   6       PA5        SPI1_SCK, internal only (pad not connected)                                    -
 *   7       PA7        ADC_IN7: board current (shunt + TPA158A3 + RC), continuous conversion          ISENSE
 *  15 / 14  PA9 / PA8  PWM R / G: TIM1_CH2 / TIM1_CH1 (G right-aligned)                               PWM_R / PWM_G
 *  11       PB11       PWM B: TIM2_CH4 (partial remap 2)                                              PWM_B
 *   9 / 8   PB1 / PB0  PWM W1 / W2: TIM3_CH4 / TIM3_CH3 (W2 right-aligned)                            PWM_W1 / PWM_W2
 *  13       PB15       status LED D9 (R30 1 k), active high                                           STATUS
 *  10       PB10       test jig input: pull-up, grounded by the jig (receiver: return-channel TX)    TP9
 *  12 / 18  PB14 / PA10 inputs with pull-down (receiver: input / output transceiver DE)               -
 *  16 / 17  PA13 / PA14 SWD                                                                           TP3 / TP4
 * Timers: TIM1 / TIM2 / TIM3 = PWM at 0, 1/3 and 2/3 of the period; TIM4 (no pins) counts DIN edges from TIM3's
 * capture trigger (ITR2); SysTick = 20 kHz poll tick. Gate driver inputs and FWD_EN have pull-downs: everything is
 * off while in reset.
 *
 * Invariants of this mapping (reference manual V2.5, tables 10-16 / 10-25; hw.c):
 *  - TIM2 partial remap 2 also maps CH1 / ETR to PA0, CH2 to PA1 and CH3 to PB10. TIM2 enables CC4E only: CC1E /
 *    CC2E / CC3E stay 0 (CH3 is B's DMA trigger at count 2, no output; PB10 is USART3_TX on the receiver).
 *  - USART3 (the receiver's return channel) is TX only: RE stays 0, because its default RX pin PB11 is PWM_B.
 *  - SPI1 is a receive-only master: MOSI (PA7, the current sense) is never driven; NSS is software (SSM), so its
 *    default pin PA4 is free for the supply divider.
 *  - TIM1 enables CC1E / CC2E only and TIM3 CC1E (capture) / CC3E / CC4E, so PA10 / PB14 / PB15 / PA7 stay GPIO /
 *    analog. */
#ifndef BOARD_H
#define BOARD_H

#define BOARD_D2A_ULTRA  1          /* QuinLED dig2analog-ultra-5ch rev 0.4 (firmware update: hardware id 1, rev 3) */
#define BOARD_DP_RX      2          /* QuinLED Diff-Power ultra receiver: definitions only, no hardware yet */
#ifndef BOARD
#define BOARD BOARD_D2A_ULTRA
#endif

/* the supply / bus divider on PA4 (ADC_IN4): rev 0.4 has it (Quindor 2026-10-13: the supply in the console only;
 * moved from PA5 the same day, so SPI1's clock pin stays as it was), the receiver reads its bus there */
#define VSENSE_SUPPLY_CH 4u

/* test jig mode (core/testjig.h): TP9 = PB10 with the internal pull-up, grounded by the jig. PB10 because the
 * dig2analog-ultra never needs it otherwise (its data line is one-way: no return channel) and it is 5 V tolerant;
 * PA0 / TP8 stays the spare analog input. The receiver uses PB10 for its return channel's TX: no test pad there yet. */
#if BOARD == BOARD_D2A_ULTRA
#define BOARD_TESTJIG    1
#else
#define BOARD_TESTJIG    0
#endif

/* board current sense (core/isense.h): soldered fuse + 2 mohm shunt + TPA158A3 (gain 100) -> 1 k / 47 nF -> PA7 */
#define ISENSE_FUSE_MA   10000u     /* soldered fuse: 10 A */
#define ISENSE_BOARD_MA  8000u      /* board rating: default and maximum of the remote limit */
/* current-sense amplifier gain (V/V) on the 2 mohm shunt: 100 = TPA158A3. Sets the ADC scale (main.c) */
#ifndef ISENSE_AMP_GAIN
#define ISENSE_AMP_GAIN  100u
#endif

/* voltage channels (core/vsense.h): the divider on each input, top / bottom resistor in ohm. The ADC reference is
 * the 3.3 V rail (the current sense's scale assumes 3.28 V as well). */
#define VSENSE_VREF_MV   3280u
#define VSENSE_SUPPLY_RT 220000u    /* PA4: supply (dig2analog-ultra) / bus (receiver), 1 : 23, full scale 75.4 V */
#define VSENSE_SUPPLY_RB 10000u
#if BOARD == BOARD_D2A_ULTRA
#define VSENSE_AUX_RT    0u         /* PA0: the spare input on TP8, read directly (0 .. 3.3 V) */
#define VSENSE_AUX_RB    1u
#else
#define VSENSE_AUX_RT    120000u    /* PA0: the buck output V_B, 1 : 13, full scale 42.6 V */
#define VSENSE_AUX_RB    10000u
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
#define SCENE_LOG_ADDR   0x0800F700u   /* power-on scene log: SCENE_LOG_PAGES (5) pages, to 0x0800FBFF */
#define SCENE_FX_ADDR    0x0800FC00u   /* the power-on effect (0.6.0): two pages */
#define ENERGY_ADDR      0x0800FE00u   /* the energy log (core/energy.h, 0.6.0): one page */
#define CAL_PAGE_ADDR    0x0800F600u   /* current calibration (core/calstore.h): the top page of the app slot. The
                                          linker scripts end the code area here (vendor/wch/Link.ld, Link_app.ld); a
                                          firmware update erases only the new app's length, so the page survives */

#endif
