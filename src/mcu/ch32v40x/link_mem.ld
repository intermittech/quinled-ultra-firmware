/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32V407 reference manual V1.2 (memory map p.5-6, flash p.579) and datasheet V1.3 (p.6) and the
 * project-owned target/update/record and startup contracts.
 */
/* Standalone (SWD) image: code from the zero-wait alias 0 (512 KB with RAM_CODE_MOD = 0, DS p.6), records excluded
 * (0x08080000+). App RAM: the whole 200 KB SRAM of RAM_CODE_MOD = 0 (RM p.6; startup.S refuses mode 1). */
MEMORY {
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x80000
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x32000
}
