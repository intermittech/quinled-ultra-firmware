/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#ifndef MCU_LAYOUT_H
#define MCU_LAYOUT_H
/* (decided 2026-10-14) the records in the upper CodeFlash (DS table 2-1: 160 KB
 * non-zero-wait at 0x08010000-0x08037FFF), every one an A/B pair. Each store in its own two
 * 4 KB sectors, A then B, nothing else in them (with every A page in one sector and every B page in the next, a
 * 4 KB sector erase, should the bench need one instead of the 256-byte page erase, would hit every pair at once).
 * Read only in hw.c's boot window (hw_records_window: HCLK / 2, DS table 4-17's 60 MHz flash clock), the values kept
 * in RAM; written through hw.c's flash_op (HCLK / 2 as well). */
#define SETTINGS_ADDR    0x08010000u   /* the settings (core/flashrec.h): page A, page B in the next sector */
#define SETTINGS_B_ADDR  0x08011000u
#define CAL_PAGE_ADDR    0x08012000u   /* the current calibration (core/calstore.h) */
#define CAL_B_ADDR       0x08013000u
#define SCENE_LOG_ADDR   0x08014000u   /* the power-on scene log (core/scene.h) */
#define SCENE_B_ADDR     0x08015000u
#define LIFE_PAGE0_ADDR  0x08016000u   /* the lifetime record (core/energy.h) */
#define LIFE_PAGE1_ADDR  0x08017000u
#define SCENE_FX_ADDR    0x08018000u   /* the power-on effects (core/ultra.h ULTRA_FX_*): bank A, two pages; bank B */
#define SCENE_FX_B_ADDR  0x08019000u
#define CALHIST_ADDR     0x0801A000u   /* the calibration history (M2): its pair */
#define CALHIST_B_ADDR   0x0801B000u
#define RECORDS_BASE     0x08010000u   /* the record area: [RECORDS_BASE, RECORDS_END), twelve sectors */
#define RECORDS_END      0x0801C000u
typedef char records_outside_the_slot[(RECORDS_BASE >= UPD_APP_END) ? 1 : -1];
#endif
