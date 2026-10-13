/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Function attributes for the LTO build (tools/build_target.sh). Host builds (MSVC): no effect.
 *
 * MEASURED: a function tools/measure.py or tools/flash_stores.py runs by name (tools/rv32emu.py, on the compiled ELF).
 * LTO inlines a function into its only caller or clones it for constant arguments (a ".constprop" copy without some
 * parameters), so the name may be gone, or take other arguments. A measurement build (MEASURE=1) keeps these out of
 * line, under their own name and signature; the release inlines them (out of line there too would cost ~100-700 B).
 * measure.py runs the ISRs, which are never inlined, in the release image itself.
 *
 * NOINLINE / ALWAYS_INLINE: where LTO's choice makes a measured path slower than the build before LTO (where the
 * function was in another file, or had another caller). */
#ifndef LTO_H
#define LTO_H

#if defined(__GNUC__) && defined(MEASURE_BUILD)
#define MEASURED __attribute__((noinline, noclone, used))
#else
#define MEASURED
#endif

#if defined(__GNUC__)
#define NOINLINE __attribute__((noinline))
#define ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define NOINLINE
#define ALWAYS_INLINE __inline
#endif

#endif
