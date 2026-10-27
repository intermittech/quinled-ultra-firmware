/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Layout 4 (protocol docs/update.md, RAM_CODE_MOD = 0): the bootloader 0x08000000-0x08003FFF, metadata 0x08004000
 * (one 4 KB sector, 256 logical bytes), the app body 0x08005000-0x0807FFFF (the zero-wait area's end, DS p.6), the
 * records in the non-zero-wait area: family k at A = 0x08080000 + 8 KB k, B = A + 4 KB (settings, calibration,
 * scenes, lifetime, calibration history, pixel effects). Each 4 KB sector holds one 256-byte logical record page
 * (the V20x's record counts, one erase per page as there). BOARD 3's REC_SET is its pixel-effects record. */
#ifndef MCU_LAYOUT_H
#define MCU_LAYOUT_H
#define MCU_RECORDS_FAMILY(k) (0x08080000u + 0x2000u * (k))
#define SETTINGS_ADDR    MCU_RECORDS_FAMILY(5u)              /* BOARD 3: the pixel-effects record (core/ur_digital_m/udfx.h) */
#define SETTINGS_B_ADDR  (MCU_RECORDS_FAMILY(5u) + 0x1000u)
#define CAL_PAGE_ADDR    MCU_RECORDS_FAMILY(1u)              /* calibration */
#define CAL_B_ADDR       (MCU_RECORDS_FAMILY(1u) + 0x1000u)
#define SCENE_LOG_ADDR   MCU_RECORDS_FAMILY(2u)
#define SCENE_B_ADDR     (MCU_RECORDS_FAMILY(2u) + 0x1000u)
#define LIFE_PAGE0_ADDR  MCU_RECORDS_FAMILY(3u)              /* lifetime */
#define LIFE_PAGE1_ADDR  (MCU_RECORDS_FAMILY(3u) + 0x1000u)
#define CALHIST_ADDR     MCU_RECORDS_FAMILY(4u)
#define CALHIST_B_ADDR   (MCU_RECORDS_FAMILY(4u) + 0x1000u)
#define SCENE_FX_ADDR    SETTINGS_ADDR
#define SCENE_FX_B_ADDR  SETTINGS_B_ADDR
#define RECORDS_BASE     0x08080000u
#define RECORDS_END      0x0808C000u
typedef char records_outside_the_slot[(RECORDS_BASE >= UPD_APP_END) ? 1 : -1];
#endif
