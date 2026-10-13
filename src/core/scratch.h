/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Main-loop scratch RAM shared by work that never runs at the same time: the adaptive stagger's offset search
 * (stagger.c: four 24-entry cost / offset tables), the spread-spectrum dither table (dither.c: the ideal counts of up
 * to 64 entries) and the flash record being programmed (hal/hw.c: settings 32, scene 16, calibration 64 bytes). Each
 * fills what it uses at the start of its call and is called only from the main loop, one after the other (never from
 * an interrupt, never nested), so one buffer serves all: 384 bytes of RAM. */
#ifndef SCRATCH_H
#define SCRATCH_H

#include <stdint.h>

#define CORE_SCRATCH_WORDS 96u
extern uint32_t core_scratch[CORE_SCRATCH_WORDS];

#endif
