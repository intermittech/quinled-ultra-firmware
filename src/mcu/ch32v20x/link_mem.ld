/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32FV2x/V3x reference manual V2.5, CH32V203 datasheet
 * V3.0 and the project-owned target/update/record and startup contracts.
 */
/* Code uses the zero-wait flash alias. Persistent records are excluded.
 * App RAM leaves the first 32 bytes for the bootloader handover.
 */
MEMORY {
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x10000
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x5000
}
