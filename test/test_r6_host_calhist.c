/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The calibration history's record and a CRC that reads like erased flash.
 * test\build_host.cmd builds and runs it (r6_calhist.exe)
 *
 * The CRC is programmed last, so a record cut before its last half-word leaves the CRC erased (0xFFFF, or 0xE339 if
 * the chip's erased value is that), and a CRC-16 matches it 1 time in 65,536; the packers never write those values and
 * the checks reject them (config.h crc16_erased), for the settings, the calibration, the lifetime record and the
 * calibration history (M2, calhist.c). Without that, a torn history record whose CRC happens to read erased would be
 * taken as the newest (its erased entries shown as 65.54 A), and calhist_pack could write a valid record whose CRC is
 * 0xFFFF: both checks below fail then. */
#include <stdio.h>
#include <string.h>
#include "../src/core/calhist.h"
#include "../src/core/config.h"
#include "../src/core/flashrec.h"

uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    (void)addr;
    (void)src;
    (void)halfwords;
    (void)erase_addr;
    return FLASH_NONE;                                   /* (the stores aren't used here) */
}

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
                                       printf("\n"); } } while (0)

static void base_rec(calhist_t *h, uint32_t seq)
{
    memset(h, 0, sizeof *h);
    h->magic = CALHIST_MAGIC;
    h->seq = seq;
    h->tests = 3;
    h->n = 2;
    h->e[0].lit_h = 12;
    h->e[0].ma[0] = 3000;
    h->e[1].lit_h = 1012;
    h->e[1].ma[0] = 2700;
}

/* rsv2 for which the CRC over bytes 0..61 (seq as it will be) is `want` (one exists: CRC-16 is a bijection on the last
 * 16 bits of its input) */
static int find_rsv2(calhist_t *h, uint16_t want)
{
    uint32_t v;
    for (v = 0; v < 65536u; v++) {
        h->rsv2 = (uint16_t)v;
        if (crc16_ccitt((const uint8_t *)h, CALHIST_REC_BYTES - 2u) == want) {
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    static const uint16_t erased[2] = { 0xFFFFu, 0xE339u };
    calhist_t h;
    uint32_t k;
    printf("r6 calhist: a history record whose CRC reads like erased flash\n");
    for (k = 0; k < 2u; k++) {
        /* a record cut before its CRC's half-word: bytes 0..61 complete, the CRC erased, and their CRC equal to it */
        base_rec(&h, 7u);
        CHECK(find_rsv2(&h, erased[k]), "a CRC of 0x%04X found", erased[k]);
        h.crc = erased[k];
        CHECK(!calhist_valid((const uint8_t *)&h), "a record whose CRC reads erased (0x%04X) is not valid", erased[k]);
        /* the packer: the record it writes never has such a CRC, and stays valid */
        base_rec(&h, 7u);
        h.seq = 8u;                                      /* (calhist_pack increments it: the CRC of the 8) */
        CHECK(find_rsv2(&h, erased[k]), "a CRC of 0x%04X found", erased[k]);
        h.seq = 7u;
        calhist_pack(&h);
        CHECK(h.seq == 8u && !crc16_erased(h.crc), "calhist_pack wrote CRC 0x%04X", h.crc);
        CHECK(calhist_valid((const uint8_t *)&h), "the packed record is valid");
        CHECK(h.n == 2u && h.tests == 3u && h.e[1].ma[0] == 2700u, "the packed record keeps its history");
        printf("  erased value 0x%04X: packed CRC 0x%04X\n", erased[k], h.crc);
    }
    printf("r6 calhist: %s (%d failures)\n", g_fail ? "FAILED" : "passed", g_fail);
    return g_fail ? 1 : 0;
}
