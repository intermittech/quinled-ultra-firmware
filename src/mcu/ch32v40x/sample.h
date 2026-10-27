/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The delay line's sampling grid: SPI1 at PCLK2 / 2 = 50 MHz (RM p.358, HSRXEN p.364), 20 ns samples, 16 per ring
 * word (0.32 us). Physical times are the V20x's; the windows cover the pre-lock clock range of clock.h
 * (MCU_CLOCK_MIN/MAX_PPM: the boot trim's -2.4 % with the HSI's +-2.2 % and step margin). Conversions round as named;
 * constant arguments only. */
#ifndef MCU_SAMPLE_H
#define MCU_SAMPLE_H
#include "clock.h"
#define WS_SAMPLE_HZ (MCU_PCLK2_HZ / MCU_SPI_DIV)
#define MCU_SAMPLE_BITS 16u
#define MCU_NS2S(ns) ((unsigned int)((((ns) * 1ULL * WS_SAMPLE_HZ + 999999999u) / 1000000000u)))
#define MCU_US2S(us) ((WS_SAMPLE_HZ / 1000000u) * (us))
#define MCU_US2W(us) ((MCU_US2S(us) + MCU_SAMPLE_BITS - 1u) / MCU_SAMPLE_BITS)
#define MCU_NS2W(ns) ((unsigned int)(((MCU_NS2S(ns) + MCU_SAMPLE_BITS - 1u) / MCU_SAMPLE_BITS)))
#define MCU_WORDS_MS (WS_SAMPLE_HZ / MCU_SAMPLE_BITS / 1000u)
#define MCU_BL_TICKS_PER_SAMPLE (MCU_HCLK_HZ / WS_SAMPLE_HZ)   /* 4 HCLK (SysTick) ticks per DIN sample */
#define MCU_WS_BIT_SAMPLES (MCU_US2S(4u) / 3u)
#define MCU_WS_ZERO_THRESHOLD MCU_NS2S(472u)
#define MCU_WS_HIGH_MAX MCU_NS2S(2222u)
/* line profiles 800/600/400/200 kHz: the V20x's physical decision times and periods (30/42/63/125 samples) */
#define MCU_LINE_THRESHOLDS { MCU_NS2S(600u), MCU_NS2S(833u), MCU_NS2S(1250u), MCU_NS2S(2500u) }
#define MCU_LINE_PERIODS { MCU_NS2S(1250u), MCU_US2S(5u)/3u, MCU_NS2S(2500u), MCU_US2S(5u) }
/* admission windows: the V20x's physical HIGH/LOW extrema, scaled by the slowest / fastest local clock */
#define MCU_LINE_MIN(ns) ((uint16_t)((ns)*1ULL*WS_SAMPLE_HZ*(uint64_t)(1000000 + MCU_CLOCK_MIN_PPM)/1000000000000000ULL))
#define MCU_LINE_MAX(ns) ((uint16_t)(((ns)*1ULL*WS_SAMPLE_HZ*(uint64_t)(1000000 + MCU_CLOCK_MAX_PPM)+999999999999999ULL)/1000000000000000ULL))
#define MCU_LINE_LIMITS { \
 { MCU_LINE_MIN(70u), MCU_LINE_MAX(1180u), MCU_LINE_MIN(60u), MCU_LINE_MAX(1190u) }, \
 { MCU_LINE_MIN(102u), MCU_LINE_MAX(1560u), MCU_LINE_MIN(92u), MCU_LINE_MAX(1570u) }, \
 { MCU_LINE_MIN(165u), MCU_LINE_MAX(2335u), MCU_LINE_MIN(155u), MCU_LINE_MAX(2345u) }, \
 { MCU_LINE_MIN(490u), MCU_LINE_MAX(4510u), MCU_LINE_MIN(480u), MCU_LINE_MAX(4520u) } }
/* the ring: 512 words = 163.84 us nominal (172.8 us at the slowest clock); guard words after it (V20x encoder) */
#define MCU_DLINE_RING_WORDS 512u
#define MCU_DLINE_GUARD_WORDS 64u
/* the strip's source selector (SOURCE_SEL): a switch between the replay and the serialiser needs the incoming LOW run
 * to cover the ring, so the replay (512 words behind RX, 2 words of TX lead) is inside it; 8 words of margin (udfx.c) */
#define MCU_SOURCE_SWITCH_LOW_W (MCU_DLINE_RING_WORDS + 8u)
#define MCU_DIN_RING_WORDS 2048u
#define MCU_DIN_COPY_WORDS 1792u
/* the capture: the chain snapshot ring (a power of two) and, behind it, the acquisition runs. 2048 words = 655 us:
 * the slowest header (64 bits at 200 kHz + tolerance: 1,055 words, ud classify) and a 384-bit 800 kHz marker frame
 * with its latch (~1,700 words) fit */
#define MCU_DLINE_CH_TAIL_WORDS 2048u
#define MCU_DLINE_CH_TAIL_USED 1112u     /* the V20x's 800 words (355.6 us) at 50 MHz */
/* scan margins in ring words (dline.h DLINE_W_LATE): as the V20x, the ring is 512 words here too */
#define MCU_ENCODER_LATE_WORDS 432u
#define MCU_SCAN_LATE_WORDS 500u
#define MCU_NS2S_NEAREST(ns) ((unsigned int)((((ns) * 1ULL * WS_SAMPLE_HZ + 500000000u) / 1000000000u)))
#define MCU_NS2W_NEAREST(ns) ((unsigned int)((((ns) * 1ULL * WS_SAMPLE_HZ + 8000000000ULL) / 16000000000ULL)))
#define MCU_US_PER_WORD_NUM 8u           /* 0.32 us = 8 / 25 */
#define MCU_US_PER_WORD_DEN 25u
#define MCU_US_PER_WORD_Q24 ((uint32_t)((1ULL << 24) * MCU_SAMPLE_BITS * 1000000u / WS_SAMPLE_HZ))
/* the local encoder symbols (udfx chip table): 10 ns units for the ARGB serialiser (argbenc.c converts them to HCLK
 * cycles at the clock estimate); 125 = a nominal 1.25 us bit (800 kHz).
 * fx-b0 must derive the local frame period from this chip symbol duration. */
#define MCU_SYM_NS(ns) (((ns) + 5u) / 10u)
#define MCU_SYM_BIT 125u
/* Keep the V5-class one LOW at 600 ns with the shorter bit (>=580 ns). */
#define MCU_SYM_T1_V5 MCU_SYM_NS(650u)
/* the bootloader's streaming decoder (bootloader/bl_rx.h) on the same 50 MHz grid */
#define MCU_BL_RING_WORDS 4096u          /* 1.31 ms */
/* The bootloader opens its update forwarding (TIM2 CH4 on FWD_EN) only after this long of LOW without an edge, DIN LOW:
 * inside an inter-frame LOW, never partway through a frame (docs/timing.md "Update forwarding and entry"): the
 * protocol's UPD_FORWARD_QUIET_US (ultra_update.h) */
#define MCU_BL_FWD_QUIET_US UPD_FORWARD_QUIET_US
/* End before the earliest host release (+60 us, aim +80), but after the
 * marker-to-frame LOW (<=20 us). Like the app's DLINE_W_LATCH, 46 us also
 * leaves margin over the pre-lock clock range. */
#define MCU_BL_RX_TIMING { MCU_NS2S(600u), MCU_NS2S(4444u), MCU_US2S(46u) }
#define MCU_BL_RX_EDGE_AGE 1
#define MCU_BL_MARK_MIN MCU_US2S(15u)
#define MCU_BL_MARK_MAX MCU_US2S(40u)
/* Declared group frames can be unmarked. Status replies still require a marker;
 * this boot path has no return relay for a downstream status target. */
#define MCU_BL_MARKER_TRACK 1
/* frame period detection from the first bit (bl_rx.c bit()): the V20x windows x 50/36, nominal 62.5/83.3/125/250 */
#define MCU_BL_PER0 62u
#define MCU_BL_PER0_LO 54u
#define MCU_BL_PER0_HI 71u
#define MCU_BL_PER1 83u
#define MCU_BL_PER1_LO 74u
#define MCU_BL_PER1_HI 94u
#define MCU_BL_PER2 125u
#define MCU_BL_PER2_LO 110u
#define MCU_BL_PER2_HI 142u
#define MCU_BL_PER3 250u
#define MCU_BL_PER3_LO 219u
#define MCU_BL_PER3_HI 283u
#endif
