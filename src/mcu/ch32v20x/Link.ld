/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED
 * Written from the CH32FV2x/V3x reference manual V2.5, CH32V203 datasheet
 * V3.0 and the project-owned target/update/record and startup contracts.
 */
/* Select the memory contract, then the common firmware section ABI. */
ENTRY(_start)
__stack_size = DEFINED(QL_STACK_SIZE) ? QL_STACK_SIZE : 0x800; PROVIDE(_stack_size = __stack_size);
INCLUDE link_mem.ld
INCLUDE Link_sections.ld
