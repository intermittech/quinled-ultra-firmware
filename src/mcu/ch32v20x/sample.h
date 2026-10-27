/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Sample conversions round explicitly; use constant arguments unless documented. */
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
#define MCU_WS_BIT_SAMPLES (MCU_US2S(4u) / 3u)
#define MCU_WS_ZERO_THRESHOLD MCU_NS2S(472u)
#define MCU_WS_HIGH_MAX MCU_NS2S(2222u)
/* Wire-3 line profiles: physical periods and decision times, not CPU clocks.
 * 800's 600 ns decision gives 22 samples; the other midpoints give 30/45/90. */
#define MCU_LINE_THRESHOLDS { MCU_NS2S(600u), MCU_NS2S(833u), MCU_NS2S(1250u), MCU_NS2S(2500u) }
#define MCU_LINE_PERIODS { MCU_NS2S(1250u), MCU_US2S(5u)/3u, MCU_NS2S(2500u), MCU_US2S(5u) }
/* Union of 9.6 MHz PARLIO and qualified 40/80 MHz RMT HIGH/LOW
 * extrema, including TX residual and path admission 220/300/450/750 ns.
 * Count windows include the raw +/-2.2 percent sampler error. */
#define MCU_LINE_MIN(ns) ((uint16_t)((ns)*1ULL*WS_SAMPLE_HZ*978u/1000000000000ULL))
#define MCU_LINE_MAX(ns) ((uint16_t)(((ns)*1ULL*WS_SAMPLE_HZ*1022u+999999999999ULL)/1000000000000ULL))
#define MCU_LINE_LIMITS { \
 { MCU_LINE_MIN(70u), MCU_LINE_MAX(1180u), MCU_LINE_MIN(60u), MCU_LINE_MAX(1190u) }, \
 { MCU_LINE_MIN(102u), MCU_LINE_MAX(1560u), MCU_LINE_MIN(92u), MCU_LINE_MAX(1570u) }, \
 { MCU_LINE_MIN(165u), MCU_LINE_MAX(2335u), MCU_LINE_MIN(155u), MCU_LINE_MAX(2345u) }, \
 { MCU_LINE_MIN(490u), MCU_LINE_MAX(4510u), MCU_LINE_MIN(480u), MCU_LINE_MAX(4520u) } }
#define MCU_DLINE_RING_WORDS 512u
#define MCU_DLINE_GUARD_WORDS 64u
#define MCU_DIN_RING_WORDS 2048u
#define MCU_DIN_COPY_WORDS 1792u
/* Audited algorithm/CPU margins, not wire durations; must be re-certified per family. */
#define MCU_ENCODER_LATE_WORDS 432u
#define MCU_SCAN_LATE_WORDS 500u
#define MCU_NS2S_NEAREST(ns) ((unsigned int)((((ns) * 1ULL * WS_SAMPLE_HZ + 500000000u) / 1000000000u)))
#define MCU_NS2W_NEAREST(ns) ((unsigned int)((((ns) * 1ULL * WS_SAMPLE_HZ + 8000000000ULL) / 16000000000ULL)))
#define MCU_US_PER_WORD_NUM 4u
#define MCU_US_PER_WORD_DEN (WS_SAMPLE_HZ / 1000000u / 4u)
#define MCU_US_PER_WORD_Q24 ((uint32_t)((1ULL << 24) * MCU_SAMPLE_BITS * 1000000u / WS_SAMPLE_HZ))
/* (named for the family interface; the values the shared sources held as literals) */
#define MCU_DLINE_CH_TAIL_WORDS 1024u
#define MCU_DLINE_CH_TAIL_USED 800u
#define MCU_BL_RING_WORDS 2048u
#define MCU_BL_RX_TIMING { 22u, 160u, 3600u }
/* The bootloader opens its update forwarding (TIM2 CH2 on FWD_EN) only after this long of LOW without an edge, DIN LOW:
 * inside an inter-frame LOW, never partway through a frame (docs/timing.md "Update forwarding and entry"): the
 * protocol's UPD_FORWARD_QUIET_US (ultra_update.h) */
#define MCU_BL_FWD_QUIET_US UPD_FORWARD_QUIET_US
#define MCU_BL_MARK_MIN 540u
#define MCU_BL_MARK_MAX 1440u
#define MCU_BL_PER0 45u
#define MCU_BL_PER0_LO 39u
#define MCU_BL_PER0_HI 51u
#define MCU_BL_PER1 60u
#define MCU_BL_PER1_LO 53u
#define MCU_BL_PER1_HI 68u
#define MCU_BL_PER2 90u
#define MCU_BL_PER2_LO 79u
#define MCU_BL_PER2_HI 102u
#define MCU_BL_PER3 180u
#define MCU_BL_PER3_LO 158u
#define MCU_BL_PER3_HI 204u
/* the local encoder symbols (udfx chip table): samples on this grid */
#define MCU_SYM_NS(ns) MCU_NS2S_NEAREST(ns)
#define MCU_SYM_BIT MCU_WS_BIT_SAMPLES
#define MCU_CLOCK_MIN_PPM (-22000)
#define MCU_CLOCK_MAX_PPM 22000
#endif
