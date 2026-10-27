/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Compact big-endian telemetry writes, shared by the digital pages; the BOARD 3 code's CRC-16. */
#ifndef UDPACK_H
#define UDPACK_H
#include "../lto.h"
#include "../ultra_protocol.h"
/* The firmware's one CRC-16 copy: config.c crc16_ccitt (= the header's ultra_crc16). The header's inline is a static function
 * per file, and LTO merges only the copies of files built with the same flags (udcal.c, config.c and upd_app.c are
 * -msave-restore, main_ur_digital_m.c, udchain.c and udfx.c are not: two copies). The tests, built without config.c (the host
 * suites, the target benches), keep the inline (UR_DIGITAL_M_IMAGE: tools/build_target.sh, the firmware image); the two header
 * helpers the digital code needs are written on ud_crc16 here, as in the header. */
#if defined(UR_DIGITAL_M_IMAGE)
uint16_t crc16_ccitt(const uint8_t *d, uint32_t n);                /* core/config.h */
#define ud_crc16 crc16_ccitt
#else
#define ud_crc16 ultra_crc16
#endif
static inline int ud_slot_check(const uint8_t *s)                   /* = ultra_slot_check: 0 = CRC ok */
{
    return ud_crc16(s, 30) == ultra_get16(s + 30) ? 0 : -1;
}
uint16_t ud_px_config_crc(const uint16_t *values, uint16_t valid, uint16_t input_ma, uint8_t input_valid);
/* a frame header's MAGIC, VERSION and CRC-8, and its TIME (= ultra_parse_header): on the firmware's one CRC-8 copy
 * (config.c crc8_smbus; the header's inline would be a second one) */
#if defined(UR_DIGITAL_M_IMAGE)
uint8_t crc8_smbus(const uint8_t *d, uint32_t n);                   /* core/config.h */
int ud_parse_header(const uint8_t *h, uint32_t *time_us);           /* udchain.c */
#else
#define ud_parse_header ultra_parse_header
#endif
/* a CRC-16 value an erased (0xFFFF) or a half-programmed (0xE339) record would show: the firmware's one copy (config.c
 * crc16_erased); the tests keep the inline */
#if defined(UR_DIGITAL_M_IMAGE)
int crc16_erased(uint32_t c);                                       /* core/config.h */
#define ud_crc_erased crc16_erased
#else
static inline int ud_crc_erased(uint32_t c) { return c == 0xFFFFu || c == 0xE339u; }
#endif
/* a page part's u16 fields from u16 struct members, one call: t[k].dst = the page byte (UD_F16_B: from base b, else from base
 * a), t[k].src = the member's byte offset (offsetof; even: the members are aligned). The bytes are the same as one ud_put16 per
 * field, at a fraction of the code */
typedef struct { uint8_t dst, src; } ud_f16_t;
#define UD_F16_B 0x80u
#if defined(UR_DIGITAL_M_IMAGE)
void ud_put16(uint8_t *d, uint32_t v);
void ud_put16s(uint8_t *d, const void *a, const void *b, const ud_f16_t *t, uint32_t n);
#else
static NOINLINE void ud_put16(uint8_t *d,uint32_t v)
{
    ultra_put16(d,v);
}
#if defined(__GNUC__)
__attribute__((unused))                           /* (udfx.c has no table of its own) */
#endif
static NOINLINE void ud_put16s(uint8_t *d, const void *a, const void *b, const ud_f16_t *t, uint32_t n)
{
    for (; n != 0u; n--, t++) {
        const uint8_t *s = (const uint8_t *)((t->dst & UD_F16_B) ? b : a) + t->src;
        ud_put16(d + (t->dst & 0x7Fu), *(const uint16_t *)(const void *)s);
    }
}
#endif
#endif
