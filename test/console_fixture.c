/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Host capture driver: every schema record, native typed accessors, edge values. */
#include <stddef.h>
#include <stdio.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include <string.h>
#include "../src/core/cli.h"
#include "../src/core/console_record.h"
#include "../src/core/link_stats.h"
#include "../src/core/ur_digital_m/udcal.h"
#include "../src/core/ur_digital_m/udfx.h"
#include "../src/core/ur_digital_m/udjig.h"
#include "../src/app/ur_digital_m_stats.h"
#define CONSOLE_UR_DIGITAL_M 1
#include "../src/core/console_fields.inc"
static void output(const char *s) { fputs(s,stdout); }
#include "console_fixture.inc"
int main(void) {
#ifdef _WIN32
_setmode(_fileno(stdout),_O_BINARY);
#endif
fixture(); return 0; }
