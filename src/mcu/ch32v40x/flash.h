/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* CH32V407 flash (RM ch.32 p.579-593; DS table 3-17 p.67): 256 B pages, 4 KB sectors (the standard erase unit),
 * halfword programming; erased reads 0xE339. SCKMOD keeps HCLK / 2 = 100 MHz, the access limit, so a flash
 * operation changes no clock. Waits are bounded by the operation deadlines below (2 x the declared bounds). */
#ifndef MCU_FLASH_H
#define MCU_FLASH_H
#include "clock.h"
#define MCU_FLASH_HCLK_DIV 1u
#define MCU_FLASH_HCLK_HZ (MCU_HCLK_HZ / MCU_FLASH_HCLK_DIV)
#define MCU_FLASH_PAGE_BYTES 256u
#define MCU_FLASH_SECTOR_BYTES 4096u
#define MCU_FLASH_BSY_POLLS 40000000u    /* backstop only: the tick deadline ends a stuck operation first */
#define MCU_FLASH_TC_POLLS 40000u
/* immutable target identity facts (pinned ultra_update.h: MCU 4 V407, package 5 RET6, layout 4) */
#define MCU_UPD_MCU 4u
#define MCU_UPD_PACKAGE 5u
#define MCU_UPD_LAYOUT 4u
#define MCU_UPD_GEOMETRY 0x8Cu           /* 256-byte program, 4096-byte erase */
#define MCU_UPD_PROGRAM_MS 30u           /* DS: page 15 ms max */
#define MCU_UPD_ERASE_MS 100u            /* DS: sector 50 ms max */
#define MCU_UPD_ERASE_BYTES 4096u
/* target features: the status profile (docs/update.md: required for bootloaders of 16 KiB or more;
 * decided 2026-10-22) and the 4 KB erase unit */
#define MCU_UPD_FEATURES 0x03u           /* UPD_FEATURE_STATUS | UPD_FEATURE_ERASE_4K */
#define MCU_BL_WRP_KEEP 0xF0u             /* WRPR0: sectors 0-3 (the 16 KiB slot) protected (RM p.591) */
#define MCU_BL_USER_KEEP 0xBFu            /* USER bit 6 RAM_CODE_MOD = 0: the 512 KB / 200 KB split of layout 4 */
#endif
