/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Main-loop scratch RAM shared by work that never runs at the same time: the adaptive stagger's offset search
 * (stagger.c: three 24-entry cost tables), the spread-spectrum dither table (dither.c: the ideal counts of up
 * to 64 entries) and the flash record being programmed (mcu/ch32v20x/hw.c: settings 32, scene 16, calibration 64 bytes). Each
 * fills what it uses at the start of its call and is called only from the main loop, one after the other (never from
 * an interrupt, never nested), so one buffer serves all: 288 bytes of RAM (the fixed grid offsets are in flash).
 * main.c also decodes captures here: frx_capture / upd_app_frame consume or copy
 * the bytes before a handler can reuse the scratch for tables or a flash record. */
#ifndef SCRATCH_H
#define SCRATCH_H

#include <stdint.h>

#define CORE_SCRATCH_WORDS 72u
/* GCC target: the decoder's memset uses its fast path at this alignment. */
#if defined(__GNUC__)
#define SCRATCH_ALIGN __attribute__((aligned(16)))
#else
#define SCRATCH_ALIGN
#endif
extern uint32_t core_scratch[CORE_SCRATCH_WORDS] SCRATCH_ALIGN;

#endif
