/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The protocol repository's C99 sender (status profile) as a library for the closed loop of test_bl_status_v407.py:
 * the host side frames and answer checks are the reference implementation's, unmodified. */
#include <string.h>
#include "ultra_sender_update.h"
#include "ultra_sender_profile.h"

#define EXPORT __declspec(dllexport)

static unsigned char image[600000];
static unsigned length;
static ultra_update_t update;
static ultra_sender_config_t config;
static ultra_sender_t line;
static ultra_sender_board_t boards[1];
static ultra_sender_item_t items[32];

static int read_body(void *context, uint32_t off, uint8_t *out, uint32_t n)
{
    (void)context;
    if (off > length || n > length - off) return -1;
    memcpy(out, image + 64 + off, n);
    return 0;
}

/* blinfo: the bootloader's 32-byte info at 0x08000010 (QDB2, version, target[16], geometry[3], entry_10ms, build) */
EXPORT int s_begin(const unsigned char *img, unsigned n, const unsigned char *blinfo, const unsigned char *uid,
                   unsigned session, unsigned rate, unsigned profile, long long now)
{
    upd_descriptor_t descriptor;
    upd_target_t target;
    ultra_update_layout_t layout;
    if (n <= 64 || n > sizeof image) return -100;
    memcpy(image, img, n);
    length = n - 64;
    memset(&layout, 0, sizeof layout);
    memcpy(&descriptor, image, 64);
    memcpy(&target, blinfo + 5, 16);
    layout.metadata_base = 0x08004000;              /* layout 4: the 4 KiB metadata sector, the body behind it */
    layout.metadata_bytes = 0x1000;
    layout.bootloader_bytes = 16384;
    layout.boot_version = blinfo[4];
    memcpy(layout.geometry, blinfo + 21, 3);
    layout.entry_10ms = blinfo[24];
    ultra_sender_config_init(&config, 1, 0);
    config.clear_unused = 0;
    if (ultra_timing_profile(&config.timing, (uint8_t)profile, 40000000, 500)) return -101;
    if (ultra_sender_init(&line, &config, boards, items, 32)) return -102;
    line.preamble_left = 0;
    return ultra_update_begin_mode(&update, &line, &descriptor, &target, &layout, uid, ULTRA_PORT_DIFF_POWER, (uint8_t)session, 1,
                              (uint8_t)rate, ULTRA_UPDATE_SELECTED, now, read_body, NULL);
}

EXPORT int s_next(long long now, unsigned char *frame) { return ultra_update_next(&update, now, frame); }
EXPORT int s_answer(const unsigned char *a, long long now) { return ultra_update_answer(&update, a, now); }
EXPORT long long s_due(void) { return update.due_us; }
EXPORT long long s_poll_end(void) { return update.poll_end_us; }
EXPORT int s_state(void) { return update.state; }
EXPORT int s_boot_complete(void) { return update.state >= ULTRA_UPDATE_QUIET && update.state < ULTRA_UPDATE_FAILED; }
EXPORT int s_failed(void) { return update.state >= ULTRA_UPDATE_FAILED; }
EXPORT int s_error(void) { return update.error; }
EXPORT unsigned s_plan(unsigned i)
{
    return i == 0 ? update.plan.status : i == 1 ? update.plan.entry_ms : i == 2 ? update.plan.length : 0u;
}
