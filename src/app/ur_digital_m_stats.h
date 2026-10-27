/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Application counters shared with the console schema checks. */
#ifndef UD_STATS_H
#define UD_STATS_H
#include <stdint.h>
typedef struct {
    /* PIXEL (the rest: dline's dl.st) */
    uint16_t fps10;                        /* frames in the last 10 s (= frames per second x 10; second_service) */
    uint8_t  faults;                       /* delay-line faults (OVR, DMA error, RX - TX distance; saturating) */
    uint16_t polls;                        /* polls for this board seen (wrap; answered: dl.st.answered) */
    /* ERRORS (u16, wrap) */
    uint16_t headers;                      /* Ultra frames (behind a marker) with a valid header */
    uint16_t tail_err;                     /* marker frames without one (decode, CRC-8) */
    uint16_t decoded;                      /* slots decoded (CRC-16 good) */
    uint16_t dec_err;                      /* captures that failed to decode */
} ur_digital_m_stats_t;
#endif
