/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Firmware model and pin-role contract. */
#ifndef VDIP_H
#define VDIP_H

#include <stdint.h>
#include "ultra_protocol.h"
#include "vsense.h"

/* LEAN_TELEMETRY (board.json binding feature lean_telemetry, tools/build_target.sh: the V203 Digital-M, for its flash):
 * no SUPPLY page, so no deepest dip and a log of the newest dip only (the fault memory's and STATUS's) */
#ifndef LEAN_TELEMETRY
#define LEAN_TELEMETRY   0
#endif
#define VDIP_PAGE        (!LEAN_TELEMETRY)   /* the SUPPLY page (vdip_page, vdip_pct): its log and the deepest */
#define VDIP_LOG         (VDIP_PAGE ? ULTRA_SU_LOG : 1u)
#define VDIP_HOLDOFF_MS  250u      /* after boot: the divider's RC and the supply settle (this many readings, 1 per ms),
                                      then the reference starts */
#define VDIP_ENTER_PCT   90u
#define VDIP_LEAVE_PCT   93u
#define VDIP_MIN_MS      3u
#define VDIP_FALL_PCT    97u       /* `falling`: a reading below this % of the reference */
#define VDIP_Z_STEADY_MA 20        /* impedance: a window is settled with its current within this (+ 1/256 of it) of
                                      the two windows before */
#define VDIP_Z_STEP_MA   250       /* a step: at least this much between two settled windows ... */
#define VDIP_Z_GAP       250u      /* ... at most this many windows apart (4 s: a fade of up to ~3.9 s) */
#define VDIP_Z_DEN_MAX   65536u    /* the steps' current rise (mA) at which both sums halve */
#define VDIP_Z_MIN_STEPS 4u        /* steps before an impedance is shown */

typedef struct {
    uint32_t t_s;                  /* uptime (s) at its first low reading */
    uint16_t min_x;                /* its lowest reading (pair sum) */
    uint16_t ref_x;                /* the reference when it began (pair sum; 0 = no dip here) */
    uint16_t dur_ms;               /* how long it lasted (65535 = longer) */
    uint16_t i_ma;                 /* the board current at its lowest reading */
} vdip_ev_t;

typedef struct {
    uint32_t ref_q12;              /* the reference: running mean of the pair sums, Q12 (0 = not started) */
    uint32_t start_ms;             /* the ms of cur's first low reading (its length) */
    uint16_t run;                  /* ms in a row below ENTER (outside a dip) / above LEAVE (in a dip); before the
                                      reference: the hold-off's readings */
    uint16_t count;                /* dips since boot (wraps) */
    uint8_t  in_dip;               /* 1: a dip now; 2: a planned cut's (`planned` at its first low reading):
                                      the same for the flash and a calibration, but no count, no log, no deepest */
    uint8_t  n_log;                /* dips in the log */
    uint8_t  head;                 /* the newest's index */
    uint8_t  planned;              /* (the receiver's main.c) a power-off notice holds: a dip starting now is
                                      the planned cut's; the reference holds meanwhile (and through the
                                      cut's dip) */
    vdip_ev_t cur;                 /* the dip (or the candidate) in progress */
    vdip_ev_t log[VDIP_LOG];
#if VDIP_PAGE
    vdip_ev_t deep;                /* the deepest (ref_x 0: none yet) */
#endif
    /* supply / cable impedance (all 0 at boot: no settled window yet) */
    int32_t  znum;                 /* the steps' fall of the supply, mV (the current rising) */
    uint32_t zden;                 /* ... their rise of the current, mA */
    uint16_t zi1, zi2;             /* the two windows before: their current, mA (0xFFFF: around a dip) */
    uint16_t zi_set, zv_set;       /* the last settled window: its current, mA, and supply, mV (16 bits: the difference
                                      is all that counts) */
    uint32_t zma;                  /* the board current over the supply's 16 ms average, mA */
    uint16_t zsteps;               /* steps taken (saturates) */
    uint8_t  zok;                  /* windows the last settled one may still pair with a new one (0: none) */
    uint8_t  cand_planned;         /* `planned` at the candidate's first low reading: its class when it is
                                      confirmed (taken again at each candidate's first low reading) */
    uint8_t  falling;              /* the last reading below VDIP_FALL_PCT % of the reference: with
                                      `planned`, the cut has begun (main.c keeps the outputs dark) */
} vdip_t;

void vdip_init(vdip_t *d);
/* every ms: now / now_s = the ms counter and the uptime in s, sum2 = the 1 ms pair sums of PA4, n pairs (0: none this
 * ms), i_ma = the board current of that ms. No absolute time: any start value, any wrap. */
void vdip_ms(vdip_t *d, uint32_t now, uint32_t now_s, uint32_t sum2, uint32_t n, int32_t i_ma);
/* the k-th newest logged dip (0 = the newest), NULL past the log */
const vdip_ev_t *vdip_event(const vdip_t *d, uint32_t k);
#if VDIP_PAGE
/* a dip's lowest reading in % of its reference */
uint32_t vdip_pct(const vdip_ev_t *e);
#endif
/* every ms after vsense_add of the supply (vs): the board current of that ms, mA (>= 0); at the end of each 16 ms
 * supply average, the impedance's window */
void     vdip_z(vdip_t *d, const vsense_t *vs, uint32_t ma);
/* firmware model */
uint32_t vdip_z_mohm(const vdip_t *d);
#if VDIP_PAGE
/* SUPPLY page DATA (ULTRA_SU_*): now_s = the uptime in s, now_mv = the supply now, arg = the logged
 * dip to show */
void vdip_page(const vdip_t *d, const vsense_t *vs, uint32_t now_s, uint32_t now_mv, uint8_t arg, uint8_t out[24]);
#endif

#endif
