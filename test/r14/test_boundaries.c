/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Boundary and interruption tests on the production code, used unchanged.
 * Reuse the existing NOR flash model and store adapters, not their test cases. */
#define main r6_pagepair_original_main
#include "../test_r6_host_pagepair.c"
#undef main
#include "../../src/core/frame_rx.h"
#include "../../src/core/upd_app.h"

static unsigned checks;
#undef CHECK
#define CHECK(c, ...) do { checks++; if (!(c)) { g_fail++; printf("FAIL: "); printf(__VA_ARGS__); puts(""); } } while (0)

static upd_bl_info_t update_boot = {{'Q','D','B','2'},3,
    {{1,1,1,1,3,0,0,UPD_REV_D2A_R04,8,0,0x21,0,0,0,0xDF,0}}, {0x88,10,20},5,{0}};

static void descriptor(uint8_t *d, unsigned hw, unsigned rev)
{
    memset(d, 0, UPD_DESC_BYTES);
    memcpy(d, "QDU6", 4);
    d[UPD_D_VERSION] = UPD_DESC_VERSION;
    memcpy(d + UPD_D_TARGET, update_boot.target.bytes, 7);
    d[UPD_D_TARGET] = (uint8_t)hw;
    d[UPD_D_REV_MIN] = d[UPD_D_REV_MAX] = (uint8_t)rev;
    upd_put32(d + UPD_D_FW_VERSION, 0x00070AFFu);
    upd_put32(d + UPD_D_BODY_LEN, 256);
    upd_put32(d + UPD_D_BODY_CRC, 0x12345678);
    upd_put32(d + UPD_D_LOAD, UPD_APP_BODY);
    upd_put32(d + UPD_D_ENTRY, UPD_APP_ENTRY);
    d[UPD_D_MIN_BL] = 2;
    upd_put32(d + UPD_D_CRC, upd_crc32_words(UINT32_MAX, d, 15));
}

static void app_init(upd_app_t *u, unsigned hw)
{
    upd_app_info_t info = {0};
    info.hw_id = (uint8_t)hw;
    info.hw_rev = hw == 1 ? 3 : 0;
    info.bl_version = 3;
    update_boot.target.bytes[UPD_T_BOARD] = (uint8_t)hw;
    update_boot.target.bytes[UPD_T_REVISION] = info.hw_rev;
    info.boot = &update_boot;
    info.desc_ok = info.confirmed = 1;
    info.version = 0x00060AFFu;
    upd_app_init(u, &info);
}

static uint32_t send_upd(upd_app_t *u, uint8_t type, uint8_t session, unsigned half,
                         const uint8_t *d, uint32_t ms)
{
    uint8_t f[UPD_FRAME_BYTES];
    upd_build_frame(f, type, session, half, 0, 0, d, ms * 1000u);
    upd_app_frame_end(u, 1, UPD_HDR_FLAG, ms);
    return upd_app_frame(u, f, ms);
}

static void updates(int abort_case)
{
    upd_app_t u;
    uint8_t d[64], f[UPD_FRAME_BYTES];
    unsigned hw, image_hw, bit;
    uint32_t r;
    for (hw = 1; hw <= 3; hw++) {
        for (image_hw = 1; image_hw <= 3; image_hw++) {
            app_init(&u, hw);
            descriptor(d, image_hw, image_hw == 1 ? 3 : 0);
            (void)send_upd(&u, UPD_ANNOUNCE, 7, 0, d, 10);
            r = send_upd(&u, UPD_ANNOUNCE, 7, 1, d + 32, 11);
            CHECK((r == UPD_APP_TAKE) == (hw == image_hw), "hardware %u image %u action %u", hw, image_hw, r);
        }
    }
    descriptor(d, 1, 3);
    for (bit = 0; bit < UPD_FRAME_BYTES * 8u; bit++) {
        app_init(&u, 1);
        upd_build_frame(f, UPD_ANNOUNCE, 9, 0, 0, 0, d, 0);
        f[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        CHECK(upd_app_frame(&u, f, 0) == UPD_APP_NONE && !u.have_half,
              "corrupt update frame bit %u must not retain a descriptor half", bit);
    }
    app_init(&u, 1);
    (void)send_upd(&u, UPD_ANNOUNCE, 9, 0, d, UINT32_MAX - 1000u);
    (void)send_upd(&u, UPD_ANNOUNCE, 9, 0, d, UINT32_MAX - 999u);
    /* (its 256-byte layout-1 image holds 2 x 510 + 100 = 1120 ms from that half, then the 2 s idle close) */
    (void)upd_app_poll(&u, UINT32_MAX - 999u + 1120u + UPD_IDLE_CLOSE_MS);
    CHECK(!u.have_half && !u.transparent, "interrupted ANNOUNCE expires across millisecond wrap");
    r = send_upd(&u, UPD_ANNOUNCE, 9, 1, d + 32, UINT32_MAX - 999u + 1120u + UPD_IDLE_CLOSE_MS + 1u);
    CHECK(r == UPD_APP_NONE, "second half after timeout alone cannot take an update");
    app_init(&u, 1);
    u.info.confirmed = u.confirm_done = 0;
    u.startup_ok = 1;
    r = upd_app_poll(&u, 60000);
    CHECK(r == UPD_APP_NONE, "startup without idle PING cannot confirm");
    if (abort_case) {
        app_init(&u, 1);
        (void)send_upd(&u, UPD_ANNOUNCE, 42, 0, d, 100);
        (void)send_upd(&u, UPD_ABORT, 42, 0, NULL, 101);
        r = send_upd(&u, UPD_ANNOUNCE, 42, 1, d + 32, 102);
        printf("ANNOUNCE half 0 -> ABORT -> half 1: action=%u (NONE=%u TAKE=%u), takes=%u\n",
               r, UPD_APP_NONE, UPD_APP_TAKE, u.takes);
        CHECK(r == UPD_APP_NONE, "ABORT must discard the cancelled session's incomplete descriptor");
    }
}

static void frames(void)
{
    uint8_t s[ULTRA_SLOT_BYTES], h[ULTRA_HDR_BYTES], bad[ULTRA_SLOT_BYTES];
    frx_t rx;
    frx_end_t e;
    slot_t ws;
    unsigned n, bits, bit;
    ultra_slot_nop(s);
    ultra_build_header(h, ULTRA_TIME_WRAP);               /* TIME at its wrap: an invalid header, as a bad CRC */
    for (n = 1; n <= 64; n++) {
        frx_init(&rx, PMODE_ULTRA, PROTO_NONE);
        (void)frx_capture(&rx, 0, PROTO_ULTRA, s, ULTRA_SLOT_BITS, ULTRA_SLOT_BITS, 0, &ws);
        frx_frame_end(&rx, 0, ULTRA_HDR_BITS + n * ULTRA_SLOT_BITS, h, &e);
        CHECK(!e.ultra_slot && !e.ultra_header, "TIME = ULTRA_TIME_WRAP: no header, no slot, chain %u", n);
    }
    ultra_build_header(h, ULTRA_TIME_WRAP - 1u);          /* the highest TIME */
    for (n = 1; n <= 64; n++) {
        for (bits = 0; bits <= ULTRA_SLOT_BITS; bits++) {
            frx_init(&rx, PMODE_ULTRA, PROTO_NONE);
            (void)frx_capture(&rx, UINT32_MAX, PROTO_ULTRA, s, bits, ULTRA_SLOT_BITS, 0, &ws);
            frx_frame_end(&rx, UINT32_MAX, ULTRA_HDR_BITS + n * ULTRA_SLOT_BITS, h, &e);
            CHECK(e.ultra_slot == (bits == ULTRA_SLOT_BITS), "slot boundary %u bits, chain %u", bits, n);
        }
        frx_init(&rx, PMODE_ULTRA, PROTO_NONE);
        (void)frx_capture(&rx, 0, PROTO_ULTRA, s, ULTRA_SLOT_BITS, ULTRA_SLOT_BITS, 0, &ws);
        frx_frame_end(&rx, 0, ULTRA_HDR_BITS + n * ULTRA_SLOT_BITS - 1, h, &e);
        CHECK(!e.ultra_slot, "truncated chain frame %u must not apply a slot", n);
    }
    for (bit = 0; bit < 256; bit++) {
        memcpy(bad, s, sizeof s);
        bad[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        frx_init(&rx, PMODE_ULTRA, PROTO_NONE);
        (void)frx_capture(&rx, 1, PROTO_ULTRA, bad, 256, 256, 0, &ws);
        frx_frame_end(&rx, 1, 320, h, &e);
        CHECK(!e.ultra_slot, "slot checksum single-bit corruption %u", bit);
    }
    for (bit = 0; bit < 64; bit++) {
        memcpy(bad, h, sizeof h);
        bad[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        frx_init(&rx, PMODE_ULTRA, PROTO_NONE);
        (void)frx_capture(&rx, 1, PROTO_ULTRA, s, 256, 256, 0, &ws);
        frx_frame_end(&rx, 1, 320, bad, &e);
        CHECK(!e.ultra_slot && !e.ultra_header, "header single-bit corruption %u", bit);
    }
    printf("frame boundaries: 1..64 slots, captures 0..256 bits, all header/slot single-bit corruptions\n");
}

static void counters(void)
{
    energy_t a = {0}, b = {0};
    uint32_t k, start = UINT32_MAX - 2500u;
    energy_init(&a, 0, 0, 0, 0);
    energy_init(&b, 0, start, 0, 0);
    for (k = 0; k <= 10000; k++) {
        energy_ms(&a, k, 75000, 20000, (uint8_t)(k < 4000));
        energy_ms(&b, start + k, 75000, 20000, (uint8_t)(k < 4000));
        if (k % 1000 == 0) {
            energy_second(&a, k, k / 1000, 0, 20000, 800);
            energy_second(&b, start + k, k / 1000, 0, 20000, 800);
            CHECK(a.boot_mwh == b.boot_mwh && a.lt.lit_s == b.lt.lit_s && a.due == b.due,
                  "energy behavior through 49.7-day millisecond wrap at %u", k);
        }
    }
    energy_init(&a, 0, 0, 0, 0);
    a.boot_mwh = UINT32_MAX - 10u;
    a.mwh_seen = a.boot_mwh - 990u;
    a.base_wh = UINT32_MAX - 1u;
    for (k = 0; k < 20000; k++) energy_ms(&a, k, 75000, 20000, 1);
    energy_second(&a, 20000, 20, 0, 20000, 800);
    CHECK(energy_total_wh(&a) == UINT32_MAX, "lifetime Wh saturates while boot_mwh wraps: %u", energy_total_wh(&a));
    a.base_on_s = UINT32_MAX - 5u;
    a.lt.lit_s = UINT32_MAX;
    energy_second(&a, 20001, 20, 0, 20000, 800);
    CHECK(a.lt.on_s == UINT32_MAX && a.lt.lit_s == UINT32_MAX, "powered/lit seconds stay at their ceiling");
    printf("counter boundaries: ms wrap, boot_mwh wrap, lifetime Wh/on_s/lit_s saturation; LIFE_REC_BYTES=%u sizeof=%zu\n",
           LIFE_REC_BYTES, sizeof(life_rec_t));
}

static void cuts(void)
{
    unsigned i, initial, k, cases = 0;
    for (i = 0; i < NSTORES; i++) {
        const store_if_t *s = &stores[i];
        for (initial = 1; initial <= 2u * s->recs + 1; initial++) {
            for (k = 0; k <= s->bytes / 2u + 1; k++) {
                uint32_t v = 0, n, r;
                f_mode = F_NONE; cut = 0; guard_lo = guard_hi = 0;
                erase_pair(s);
                (void)s->load(&v);
                for (n = 1; n <= initial; n++) {
                    (void)prep(i);
                    CHECK(s->write(n) == FLASH_OK, "%s seed %u", s->name, n);
                }
                guard_lo = place_of(i, initial); guard_hi = guard_lo + s->bytes;
                f_mode = F_CUT; f_k = (int)k;
                (void)prep(i);
                r = s->write(initial + 1);
                f_mode = F_NONE; cut = 0;
                CHECK(s->load(&v) && (v == initial || v == initial + 1),
                      "%s cut %u after %u records: loaded %u, write=%u", s->name, k, initial, v, r);
                guard_lo = place_of(i, v); guard_hi = guard_lo + s->bytes;
                (void)prep(i);
                CHECK(s->write(1000u + initial) == FLASH_OK, "%s resume after cut %u", s->name, k);
                CHECK(s->load(&v) && v == 1000u + initial, "%s resume survives reboot after cut %u", s->name, k);
                cases++;
            }
        }
        printf("power-cut sweep: %s, all halfword cut points at every entry across two pages\n", s->name);
    }
    CHECK(!bad_prog && !bad_erase, "cuts: bad programs=%u bad erases=%u", bad_prog, bad_erase);
    printf("power-cut cases=%u, bad programs=%u, bad erases=%u\n", cases, bad_prog, bad_erase);
}

int main(int argc, char **argv)
{
    int abort_only = argc > 1 && strcmp(argv[1], "abort") == 0;
    updates(1);                         /* the ABORT case always (it failed on 893303f; upd_app.c drops the half now) */
    if (!abort_only) {
        frames(); counters();
        fl = (uint8_t *)VirtualAlloc((void *)(uintptr_t)FL_BASE, FL_SIZE, 0x3000u, 0x04u);
        if (fl != (uint8_t *)(uintptr_t)FL_BASE) return 2;
        memset(fl, 0xFF, FL_SIZE);
        cuts();
    }
    printf("R14 boundaries: %u checks, %d failures\n", checks, g_fail);
    return g_fail ? 1 : 0;
}
