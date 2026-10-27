/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32V407 reference manual V1.2 (memory map p.5-6, flash p.579) and datasheet V1.3 (p.6) and the
 * project-owned target/update/record and startup contracts.
 */
/* Behind the 16 KB bootloader and the 4 KB metadata sector: the body slot 0x08005000-0x0807FFFF (layout 4) at
 * the link alias. App RAM: the 200 KB SRAM of RAM_CODE_MOD = 0 (RM p.6; startup.S refuses mode 1) less the first 32
 * bytes, the bootloader's hand-over. */
MEMORY {
    FLASH (rx) : ORIGIN = 0x00005000, LENGTH = 0x7b000
    RAM (rwx) : ORIGIN = 0x20000020, LENGTH = 0x31fe0
}
