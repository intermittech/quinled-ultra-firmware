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
 * Flash map (hal/board.h): the settings, the calibration and the power-on scene log in A/B page pairs (pagepair.h,
 * 0.6.4), the lifetime record in one (core/energy.h, 0.6.3), the power-on effects (two pages, rewritten whole). */
#ifndef FLASHREC_H
#define FLASHREC_H

#include <stdint.h>
#include "config.h"
#include "store.h"
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

/* the primitive (hal/hw.c; test/test_soak.c: its model): erase the 256-byte page at erase_addr (0: none), then program
 * `halfwords` half-words from src at addr (erased flash). Returns FLASH_OK or FLASH_ERR_*. */
uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr);

/* settings + their extension (store_ext_t: the board current limit): the newest record (0; -1 = none) */
int      settings_load(settings_t *s, store_ext_t *ext);
uint32_t settings_store(const settings_t *s, const store_ext_t *ext);
uint32_t settings_pending(void);    /* an erase: 2 = due before the next write, 1 = possible ahead (pagepair.h) */
uint32_t settings_prepare(void);    /* ... that erase */
/* current calibration (core/calstore.h): the newest record (-1: none, or cleared); store one (levels 0 = clear).
 * *count: calibrations stored on this board (changes only when written) */
int      cal_load(isense_cal_t *c, uint16_t *count);
uint32_t cal_store(const isense_cal_t *c, uint16_t *count);
uint32_t cal_pending(void);
uint32_t cal_prepare(void);
/* power-on scene log (core/scene.h) */
int      scene_load(scene_t *sc);
uint32_t scene_store(const scene_t *sc);
uint32_t scene_pending(void);       /* 2: the next record enters a page that must be erased first; 1: the next page
                                       can be erased ahead (it holds only the oldest records) */
uint32_t scene_prepare(void);
/* the lifetime record (core/energy.h, two pages; nothing erases them but a full chip erase): the newest stored one
 * into *r (1; 0 = none); store e's into the next erased entry (energy_pack) */
uint32_t life_load(life_rec_t *r);
uint32_t life_store(energy_t *e);
uint32_t life_pending(void);
uint32_t life_prepare(void);
uint32_t life_room(void);           /* erased entries left (main.c keeps one for a dip write while bright) */
/* the power-on effect store (two pages at SCENE_FX_ADDR, core/ultra.h ULTRA_FX_*; loaded with ultra_fx_load): the
 * lights in mask with their CRC-8s (none: the store is erased), the header last. Erases both pages: only where erases
 * are admitted. */
uint32_t scene_fx_store(const ultra_t *u, uint8_t mask, const uint8_t crc[ULTRA_LIGHTS]);
/* firmware update (core/upd_app.h): one half-word into the app descriptor page's status records (no erase) */
uint32_t hw_flash_half(uint32_t addr, uint16_t v);

#endif
