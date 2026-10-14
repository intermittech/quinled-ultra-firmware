/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compressed console text. tools/gen_console_text.py (run by every build) reads each Z(name, "text") in the
 * sources, compresses all of them together (byte-pair tokens, at most 128) and writes cz_text.h into the build's
 * output folder: CZ_<name> = the text's offset in the compressed blob, minus CZ_BIAS (a 12-bit signed immediate at
 * every use). The C compiler never sees the text: Z(name, "text") is CZ_<name>.
 *
 * Only the console (core/cli.c) decodes it: character by character into its line buffer (cz_next: a few bytes of
 * state, no output buffer), so only the cold console path pays the CPU. Rules for the text:
 *   - 7-bit characters only; a format (cf) uses the console's %-codes ("%%" for a '%');
 *   - the NUL-separated name lists ("a\0b\0c") and the settings keys keep their separators and command bytes as
 *     plain bytes (bytes below 0x20 other than LF are never part of a token, a CR only as CR LF): nth / skip /
 *     key_cmd walk them compressed;
 *   - one name, one text (the same name may be used again with the same text). */
#ifndef CZ_H
#define CZ_H

#include <stdint.h>
#include "cz_text.h"

#define Z(name, text) CZ_##name
#define ZP(name, text) CZ_##name           /* (a PERF build's text: tools/gen_console_text.py --perf) */
#define ZU(name, text) CZ_##name           /* (an upper-layout build's text, MON_EXTRA: --upper) */
#define ZJ(name, text) CZ_##name           /* (the test jig's text: not with --no-jig, the receiver's build) */
#define ZL(name, text) CZ_##name           /* (0.6.4's layout's instead: without --upper) */

#endif
