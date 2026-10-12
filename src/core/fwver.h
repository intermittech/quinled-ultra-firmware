/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Firmware version text <-> version word (the update descriptor's format, ultra_update.h):
 *   major << 24 | minor << 16 | patch << 8 | pre,  pre = 0x00 + n alpha.n, 0x40 + n beta.n, 0x80 + n rc.n, 0xFF stable
 * As a number it sorts like semver. The same rules as tools/release/qdu.py and the ESPHome component's fw_version.h.
 * 0 = not a version ("0.4", a development build): research/firmware-version-display.md. */
#ifndef FWVER_H
#define FWVER_H

#include <stdint.h>

#define FWVER_TEXT_MAX 24u                     /* "255.255.255-alpha.63" + NUL */

/* "x.y.z" or "x.y.z-alpha.n" / "-beta.n" / "-rc.n" (x, y, z <= 255, n <= 63), an optional leading 'v'; else 0 */
uint32_t fwver_parse(const char *s);
/* the text of a version word (buf >= FWVER_TEXT_MAX); 0 gives "" */
void     fwver_format(uint32_t w, char *buf);

#endif
