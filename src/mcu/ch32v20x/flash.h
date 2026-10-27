/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#ifndef MCU_FLASH_H
#define MCU_FLASH_H
#include "clock.h"
#define MCU_FLASH_HCLK_DIV 2u
#define MCU_FLASH_HCLK_HZ (MCU_HCLK_HZ / MCU_FLASH_HCLK_DIV)
#define MCU_FLASH_PAGE_BYTES 256u
#define MCU_FLASH_SECTOR_BYTES 4096u
/* Legacy bounded loop guards: poll iteration costs are not cycle-exact deadlines.
 * Retain them for byte identity; a future deadline change needs its own flash audit. */
#define MCU_FLASH_BSY_POLLS 2000000u
#define MCU_FLASH_TC_POLLS 12000u
/* immutable target identity facts (pinned ultra_update.h: MCU 1 V203, package 1 F8U6, layout 1) */
#define MCU_UPD_MCU 1u
#define MCU_UPD_PACKAGE 1u
#define MCU_UPD_LAYOUT 1u
#define MCU_UPD_GEOMETRY 0x88u
#define MCU_UPD_PROGRAM_MS 10u
#define MCU_UPD_ERASE_MS 20u
#define MCU_UPD_ERASE_BYTES 256u
#define MCU_UPD_FEATURES 0x02u           /* (as recorded in the V203 identity) */
#define MCU_BL_WRP_KEEP 0xFCu             /* WRPR0: sectors 0-1 (the 8 KiB slot) protected */
#endif
