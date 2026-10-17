/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Local bootloader/app diagnostics in the existing hand-over's reserved word.
 * No descriptor, status, info-block or hand-over layout/checksum changes. Old apps ignore this word.
 * Pending is consumed by the bootloader only, before any app has run; reports by the app at upd_boot. */
#ifndef BL_PROTECTION_H
#define BL_PROTECTION_H
#define BL_PROT_PENDING 0x50520000u       /* low 16 bits: CRC-16 of the eight uint16_t option values */
/* Consecutive report values map to the app's OB_BAD / PROT_OK / PROT_FAIL events (checked at compile time). */
#define BL_PROT_BAD     0x50530008u
#define BL_PROT_OK      0x50530009u
#define BL_PROT_FAIL    0x5053000Au
#endif
