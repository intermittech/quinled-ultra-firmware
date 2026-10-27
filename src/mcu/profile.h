/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The build selects one registered MCU family (tools/build_target.sh: -DMCU_<FAMILY>=1); direct/host builds use the
 * V20x profile. */
#ifndef MCU_PROFILE_H
#define MCU_PROFILE_H
#if defined(MCU_CH32V40X) && MCU_CH32V40X == 1
#include "ch32v40x/clock.h"
#include "ch32v40x/sample.h"
#include "ch32v40x/systick.h"
#include "ch32v40x/irq.h"
#include "ch32v40x/flash.h"
#else
#ifndef MCU_CH32V20X
#define MCU_CH32V20X 1
#endif
#if MCU_CH32V20X != 1
#error "Unsupported MCU profile"
#endif
#include "ch32v20x/clock.h"
#include "ch32v20x/sample.h"
#include "ch32v20x/systick.h"
#include "ch32v20x/irq.h"
#include "ch32v20x/flash.h"
#endif
#endif
