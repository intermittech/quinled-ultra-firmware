/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compile-only offsets; never linked into firmware. */
#if BOARD == 3
#include "../src/app/main_ur_digital_m.c"
#else
#include "../src/app/main.c"
#endif
#include <stddef.h>
#define O(t, f) (unsigned)offsetof(t, f)
__attribute__((used, section(".rodata.update_offsets"))) const unsigned update_offsets[] = {
    O(upd_app_t, ping_seen), O(upd_app_t, ping_ms), O(upd_app_t, startup_ok),
    O(upd_app_t, confirm_done), O(upd_app_t, transparent), O(upd_app_t, info),
    O(upd_app_info_t, confirmed), O(upd_app_info_t, desc_ok), O(upd_app_info_t, st_next),
#if BOARD == 3
    O(udfx_t, e), O(udfx_t, max_px), O(udfx_t, npx), O(udfx_t, chip),
    O(udfx_t, local), O(udfx_t, dark), O(udfx_t, calibration),
    O(ws_enc_t, ready), O(ws_enc_t, state), O(ws_enc_t, nbytes), O(ws_enc_t, frames),
    O(ws_enc_t, low_w), O(ws_enc_t, reset_w), O(ws_chip_t, bpp),
    O(__typeof__(cal_io), take), O(__typeof__(cal_io), owned),
    O(dline_t, ch_hold),
#else
    O(ack_t, active), O(ack_t, led_only),
#endif
};
