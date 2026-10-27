/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Numeric diagnostic stream; schema generated from tools/console_schema.json. */
#ifndef CONSOLE_RECORD_H
#define CONSOLE_RECORD_H
#include <stdint.h>
#include "console_format.h"
void console_record(void (*write)(const char *), uint32_t index,
                    const void *base, const uint16_t *layout);
#endif
