/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compact 64-bit division (core/div64.c): the target's __udivdi3 / __divdi3. */
#ifndef DIV64_H
#define DIV64_H

#include <stdint.h>

uint64_t div64_u(uint64_t n, uint64_t d);
int64_t  div64_s(int64_t n, int64_t d);

#endif
