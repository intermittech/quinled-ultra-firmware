/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#ifndef FW_TARGET_H
#define FW_TARGET_H
#include "mcu/profile.h"
#include "board/boards.h"
#define DIN_RING_WORDS MCU_DIN_RING_WORDS
#define DIN_COPY_WORDS MCU_DIN_COPY_WORDS
#define POLL_HZ MCU_POLL_HZ
/* A supported pin binding validates BOARD/ANSWER_UART even in host builds. */
#if defined(MCU_CH32V40X) && MCU_CH32V40X == 1
#include "mcu/ch32v40x/bindings.h"
#else
#include "mcu/ch32v20x/bindings.h"
#endif
#ifndef __ASSEMBLER__
#include "core/upd_platform.h"
#if defined(MCU_CH32V40X) && MCU_CH32V40X == 1
#include "mcu/ch32v40x/layout.h"
#else
#include "mcu/ch32v20x/layout.h"
#endif
#endif
#endif
