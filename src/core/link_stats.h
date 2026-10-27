/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Main-owned LINK_STATS. The ISR only publishes capture samples and metadata;
 * acquire that publication before consuming samples. ls_frame reads the main
 * loop's runs and slot bytes after raw capture can be released. Snapshot/clear
 * and both page reads therefore cannot lose ISR data. */
#ifndef LINK_STATS_H
#define LINK_STATS_H
#include "ws_decode.h"
typedef struct {
    uint32_t sum[2];
    uint16_t min[2], max[2], count[2], period_min, period_max, low_min, low_max;
    uint16_t frames, bad, missing, invalid;
} ls_counts_t;
typedef struct {
    ls_counts_t live;
    uint8_t frozen[48];
    uint32_t snap_ms, train_ms, last_ms, train_end_ms, drops, error_ms;
    uint16_t epoch;
    uint8_t profile, flags, patterns, trained, error_count, bad_windows;
} link_stats_t;
extern link_stats_t line_stats;
/* crc_ok bits: 0 trailer CRC valid, 1 slot CRC valid; drops is cumulative. */
void ls_frame(const ws_line_t *capture,const uint16_t *runs,const uint8_t *slot,int crc_ok,uint32_t ms,uint32_t drops);
uint8_t ls_page(const uint8_t *args,uint8_t *data,uint32_t ms,uint32_t ns_q16);
#endif
