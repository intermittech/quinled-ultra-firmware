/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Pure boot services: RV32 ILP32, caller-owned buffers, no mutable state. */
#ifndef BOOT_API_H
#define BOOT_API_H
#include "ultra_update.h"
#define BOOT_API_MAGIC 0x31414251u
#define BOOT_API_VERSION 1u
#define BOOT_API_V1_BYTES 20u
#define BOOT_API_META ((BOOT_API_VERSION << 16) | BOOT_API_V1_BYTES)
typedef struct {
    uint32_t magic;
    uint16_t bytes, version;
    uint32_t desc, frame, reserved;
} boot_api_t;
typedef char boot_api_size[(sizeof(boot_api_t) == BOOT_API_V1_BYTES) ? 1 : -1];

static inline int upd_fields_ok(const uint8_t *f)
{
    uint32_t type = f[0], i, first = 0;
    if (type != UPD_PING && !f[1]) return 0;
    if (type == UPD_ANNOUNCE)
        return !f[4] && !(f[5] & ~UPD_F_FORCE) && upd_get16(f+2) < 2u;
    if (type == UPD_DATA) return !f[5];
    if (f[2] || f[3] || f[4] || f[5]) return 0;
    if (type == UPD_END) first = 8;
#if defined(MCU_CH32V40X) && MCU_CH32V40X == 1
    else if (type == UPD_SELECT) first = 28;         /* UID[12], target[16], zero[4] */
    else if (type == UPD_STATUS) {                    /* UID[12], answer index 0, R = 1, rate, zero[17] */
        if (f[18] || f[19] != 1u || f[20] >= 3u) return 0;
        first = 15;
    }
#endif
    else if (type != UPD_PING && type != UPD_ABORT) return 0;
    for (i = first; i < 32u; ++i) if (f[6u+i]) return 0;
    return 1;
}

#if defined(__riscv) && UPD_APP
#include "boot_api_layout.h"
static inline const boot_api_t *boot_api(void) {
    return (const boot_api_t *)MCU_BOOT_API_TABLE_ADDR;
}
static inline int boot_api_ok(void) {
    const boot_api_t *t = boot_api();
    return t->magic == BOOT_API_MAGIC && t->version == BOOT_API_VERSION &&
        t->bytes >= BOOT_API_V1_BYTES && !(t->bytes & 3u) &&
        t->bytes <= MCU_BOOT_API_ENTRY_END - MCU_BOOT_API_TABLE_ADDR;
}
static inline int boot_api_call(uint32_t entry, const uint8_t *p) {
    if (entry < MCU_BOOT_API_ENTRY_MIN || entry >= MCU_BOOT_API_ENTRY_END || (entry & 1u)) return -2;
    return ((int (*)(const uint8_t *))entry)(p);
}
static inline int boot_api_desc(const uint8_t *d, const upd_target_t *target, uint32_t version) {
    (void)target; (void)version;
    if (!boot_api_ok()) return UPD_ERROR_UNSUPPORTED;
    return boot_api_call(boot_api()->desc, d);
}
static inline int boot_api_frame(const uint8_t *f) {
    if (!boot_api_ok()) return -2;
    return boot_api_call(boot_api()->frame, f);
}
#else
/* Portable reference for host tests and standalone builds. */
static inline int boot_api_desc(const uint8_t *d, const upd_target_t *target, uint32_t version) {
    return upd_desc_check(d, target, version);
}
static inline int boot_api_frame(const uint8_t *f) {
    return !upd_check_frame(f) && upd_fields_ok(f) ? 0 : -1;
}
#endif
#endif
