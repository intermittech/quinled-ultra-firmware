/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Supply dips (0.6.0): the supply / bus voltage on PA4 (core/vsense.h) falling clearly below its own level, at any
 * supply (12, 24 or 48 V). Monitoring only: nothing here changes the outputs.
 *
 * Every 1 ms reading (one pair of conversions) is compared with a reference, a running mean of the readings (1/4096
 * per ms, ~4 s): a reading below VDIP_ENTER_PCT % of it for VDIP_MIN_MS ms in a row starts a dip (from its first low
 * reading), one above VDIP_LEAVE_PCT % for VDIP_MIN_MS ms in a row ends it. The reference keeps following slowly
 * during a dip, so a supply that settles at a new level ends its dip within seconds. PA4's divider filters with
 * ~10 ms (R36 / R37 / C29), so a dip shorter than a few ms shows smaller than it was.
 * Kept in RAM (a power cycle clears it): the count since boot, the deepest dip (lowest reading against its reference),
 * and the last VDIP_LOG dips with their start, lowest reading, duration and the board current at the lowest reading.
 * Readings stay ADC pair sums here (vsense_mv() turns them into mV for the console / the page).
 * Console only on the dig2analog-ultra; a Diff-Power ultra receiver reports it (protocol 0.2.3: the SUPPLY page,
 * vdip_page, and the POWER page latch ULTRA_PW_EV_SUPPLY_DIP).
 * Pure logic, host-tested (test/test_vdip.c). */
#ifndef VDIP_H
#define VDIP_H

#include <stdint.h>
#include "ultra_protocol.h"
#include "vsense.h"

#define VDIP_LOG         ULTRA_SU_LOG
#define VDIP_HOLDOFF_MS  250u      /* after boot: the divider's RC and the supply settle, then the reference starts */
#define VDIP_ENTER_PCT   90u
#define VDIP_LEAVE_PCT   93u
#define VDIP_MIN_MS      3u

typedef struct {
    uint32_t t_ms;                 /* uptime at its first low reading */
    uint16_t min_x;                /* its lowest reading (pair sum) */
    uint16_t ref_x;                /* the reference when it began (pair sum; 0 = no dip here) */
    uint16_t dur_ms;               /* how long it lasted (65535 = longer) */
    uint16_t i_ma;                 /* the board current at its lowest reading */
} vdip_ev_t;

typedef struct {
    uint32_t ref_q12;              /* the reference: running mean of the pair sums, Q12 (0 = not started) */
    uint16_t run;                  /* ms in a row below ENTER (outside a dip) / above LEAVE (in a dip) */
    uint16_t count;                /* dips since boot (wraps) */
    uint8_t  in_dip;
    uint8_t  n_log;                /* dips in the log */
    uint8_t  head;                 /* the newest's index */
    vdip_ev_t cur;                 /* the dip (or the candidate) in progress */
    vdip_ev_t log[VDIP_LOG];
    vdip_ev_t deep;                /* the deepest (ref_x 0: none yet) */
} vdip_t;

void vdip_init(vdip_t *d);
/* every ms: sum2 = the 1 ms pair sums of PA4, n pairs (0: none this ms), i_ma = the board current of that ms */
void vdip_ms(vdip_t *d, uint32_t now, uint32_t sum2, uint32_t n, int32_t i_ma);
/* the k-th newest logged dip (0 = the newest), NULL past the log */
const vdip_ev_t *vdip_event(const vdip_t *d, uint32_t k);
/* a dip's lowest reading in % of its reference */
uint32_t vdip_pct(const vdip_ev_t *e);
/* SUPPLY page DATA (protocol 0.2.3, ULTRA_SU_*): now_mv = the supply now, arg = the logged dip to show */
void vdip_page(const vdip_t *d, const vsense_t *vs, uint32_t now, uint32_t now_mv, uint8_t arg, uint8_t out[24]);

#endif
