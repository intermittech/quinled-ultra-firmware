/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The flash records (0.6.4: the stores moved here from hal/hw.c, so the host soak runs them as they are on its flash
 * model; hw.c keeps the primitive, flash_op).
 *
 * Every page is read only right after reset (whether a read after programming returns the new data before the next
 * reset is not documented: BENCH); the write positions are tracked in RAM from then on.
 *
 * Erases ahead are separate calls (*_pending / *_prepare): main.c runs them at power-up or in a low moment, so a write
 * while the outputs are bright is a program into erased flash only (Quindor's policy, main.c flash_may). A settings /
 * calibration / scene write erases its next page first when that is due (main.c asks for such a write only while low:
 * flash_may's need_erase); the lifetime record's write never erases (FLASH_NONE while its erase is due).
 *
 * Flash map (hal/board.h): the settings, the calibration, the power-on scene log and the calibration history (M2,
 * 0.6.6) in A/B page pairs (pagepair.h, 0.6.4), the lifetime record in one (core/energy.h, 0.6.3), the power-on
 * effects in two banks of two pages (0.6.5; the 0.6.4 layout: one bank), each rewritten whole. */
#ifndef FLASHREC_H
#define FLASHREC_H

#include <stdint.h>
#include "../hal/board.h"
#include "config.h"
#include "store.h"
#include "calhist.h"
#include "calstore.h"
#include "scene.h"
#include "energy.h"
#include "ultra.h"

/* results of flash_op and the stores */
#define FLASH_NONE    0u            /* nothing done (an erase is pending first, or nothing to do) */
#define FLASH_OK      1u            /* written (EOP, no error) */
#define FLASH_ERR_WRP 2u            /* write protection (WRPRTERR) */
#define FLASH_ERR_EOP 3u            /* the operation ended without EOP */
#define FLASH_ERR_KEY 4u            /* the flash did not unlock */
#define FLASH_ERR_VERIFY 5u         /* a half-word read back differently (FLASH_VERIFY builds only, BENCH) */
#define FLASH_ERR_VDD 6u            /* (0.6.5) VDD below the PVD's 2.9 V: not started (DS table 4-17: >= 2.4 V) */
#define FLASH_ERR_UART 7u           /* (review round 6) the console's byte in flight didn't finish (TC): not started,
                                       so the baud never halves mid-byte (hw.c flash_op); a failure, as the others */

/* the primitive (hal/hw.c; test/test_soak.c: its model): erase the 256-byte page at erase_addr (0: none), then program
 * `halfwords` half-words from src at addr (erased flash). Returns FLASH_OK or FLASH_ERR_*. */
uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr);

/* the four A/B page pairs (0.6.6: one descriptor each, flashrec.c; in main.c's erase order) */
enum { REC_LIFE = 0, REC_SET,
#if BOARD != BOARD_DP_UD              /* BOARD 3 persists only lifetime and local-effects records */
       REC_CAL, REC_SCENE,
#if MON_EXTRA
       REC_CALHIST,                    /* (0.6.6, M2) the calibration history */
#endif
#endif
       REC_COUNT };
/* a pair's erase: 2 = due before its next write, 1 = possible ahead (pagepair.h), 0 = none; ... and that erase */
uint32_t flash_rec_pending(uint32_t rec);
uint32_t flash_rec_prepare(uint32_t rec);
#if BOARD == BOARD_DP_UD
/* the Ultra-Digital receiver has no settings: REC_SET's page pair holds its local-effects record (core/ud/udfx.h, 128 bytes,
 * two a page). ufx_load: the newest valid record into rec (1; 0 = none); ufx_store: the record packed from u with the
 * next sequence number (udfx_rec_pack), programmed into the next erased entry (an erase first when it is due). */
struct udfx_s;
uint32_t ufx_load(uint8_t rec[128]);
uint32_t ufx_store(struct udfx_s *u);
static inline uint32_t ufx_pending(void) { return flash_rec_pending(REC_SET); }
static inline uint32_t ufx_prepare(void) { return flash_rec_prepare(REC_SET); }
#else
/* settings + their extension (store_ext_t: the board current limit): the newest record (0; -1 = none) */
int      settings_load(settings_t *s, store_ext_t *ext);
uint32_t settings_store(const settings_t *s, const store_ext_t *ext);
static inline uint32_t settings_pending(void) { return flash_rec_pending(REC_SET); }   /* an erase: 2 = due before
                                       the next write, 1 = possible ahead (pagepair.h) */
static inline uint32_t settings_prepare(void) { return flash_rec_prepare(REC_SET); }   /* ... that erase */
#endif
#if BOARD != BOARD_DP_UD
/* current calibration (core/calstore.h): the newest record (-1: none, or cleared); store one (levels 0 = clear).
 * *count: calibrations stored on this board (changes only when written) */
int      cal_load(isense_cal_t *c, uint16_t *count);
uint32_t cal_store(const isense_cal_t *c, uint16_t *count);
static inline uint32_t cal_pending(void) { return flash_rec_pending(REC_CAL); }
static inline uint32_t cal_prepare(void) { return flash_rec_prepare(REC_CAL); }
/* power-on scene log (core/scene.h) */
int      scene_load(scene_t *sc);
uint32_t scene_store(const scene_t *sc);
static inline uint32_t scene_pending(void) { return flash_rec_pending(REC_SCENE); }   /* 2: the next record enters a
                                       page that must be erased first; 1: the next page can be erased ahead (it holds
                                       only the oldest records) */
static inline uint32_t scene_prepare(void) { return flash_rec_prepare(REC_SCENE); }
#endif
/* the lifetime record (core/energy.h, two pages; nothing erases them but a full chip erase): the newest stored one
 * into *r (1; 0 = none); store e's into the next erased entry (energy_pack) */
uint32_t life_load(life_rec_t *r);
uint32_t life_store(energy_t *e);
static inline uint32_t life_pending(void) { return flash_rec_pending(REC_LIFE); }
static inline uint32_t life_prepare(void) { return flash_rec_prepare(REC_LIFE); }
uint32_t life_room(void);           /* erased entries left (main.c keeps one for a dip write while bright) */
/* the power-on effects (core/ultra.h ULTRA_FX_*), 0.6.5: two banks of two pages (SCENE_FX_ADDR / SCENE_FX_B_ADDR): the
 * newest valid one into the engine (core/scene.c ultra_fx_load); returns the lights loaded */
uint8_t  scene_fx_load(ultra_t *u, uint8_t crc[ULTRA_LIGHTS]);
/* the power-on effect store (two pages at SCENE_FX_ADDR, core/ultra.h ULTRA_FX_*; loaded with ultra_fx_load): the
 * lights in mask with their CRC-8s (none: the store is erased), the header last. Erases both pages: only where erases
 * are admitted. */
uint32_t scene_fx_store(const ultra_t *u, uint8_t mask, const uint8_t crc[ULTRA_LIGHTS]);
#if MON_EXTRA && BOARD != BOARD_DP_UD
/* (0.6.6, M2) the calibration history (core/calhist.h): the newest record into *h (1; 0 = none: *h untouched); store *h
 * (calhist_pack, programmed from *h itself); the erases as the other pairs' */
uint32_t calhist_load(calhist_t *h);
uint32_t calhist_store(calhist_t *h);
static inline uint32_t calhist_pending(void) { return flash_rec_pending(REC_CALHIST); }
static inline uint32_t calhist_prepare(void) { return flash_rec_prepare(REC_CALHIST); }
#endif
/* firmware update (core/upd_app.h): one half-word into the app descriptor page's status records (no erase) */
uint32_t hw_flash_half(uint32_t addr, uint16_t v);
/* (0.6.5, D5) the records the loaders rejected at boot (pagepair.h pp_pos_t.bad), 6 bits each: settings, calibration
 * << 6, scene log << 12, lifetime record << 18 */
uint32_t flash_rejected(void);

#endif
