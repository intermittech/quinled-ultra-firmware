/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Drive the protocol repository's C99 sender; output its actual timed frames. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ultra_sender_update.h"
#include "ultra_sender_profile.h"

static unsigned char image[57152], boot[32];
static unsigned length;
static int read_body(void *context, uint32_t off, uint8_t *out, uint32_t n)
{
    (void)context;
    if (off > length || n > length - off) return -1;
    memcpy(out, image + 64 + off, n);
    return 0;
}

int main(int argc, char **argv)
{
    FILE *f;
    upd_descriptor_t descriptor;
    upd_target_t target;
    ultra_update_layout_t layout = {0};
    ultra_update_t update;
    ultra_sender_config_t config;
    ultra_sender_t line;
    ultra_sender_board_t boards[1];
    ultra_sender_item_t items[32];
    uint8_t frame[48], uid[12] = {0};
    unsigned i, n, profile;
    int64_t now = 1000000;
    if (argc != 6) return 2;
    f = fopen(argv[1], "rb"); if (!f) return 3;
    n = (unsigned)fread(image, 1, sizeof image, f); fclose(f);
    if (n <= 64) return 4;
    length = n - 64;
    f = fopen(argv[2], "rb"); if (!f) return 5;
    fseek(f, 16, SEEK_SET); n = (unsigned)fread(boot, 1, 32, f); fclose(f);
    if (n != 32) return 6;
    memcpy(&descriptor, image, 64); memcpy(&target, boot + 5, 16);
    layout.metadata_base = 0x08002000; layout.metadata_bytes = 256;
    layout.bootloader_bytes = 8192; layout.boot_version = boot[4];
    memcpy(layout.geometry, boot+21, 3); layout.entry_10ms = boot[24];
    profile = (unsigned)atoi(argv[3]);
    ultra_sender_config_init(&config, 1, 0);
    config.clear_unused = 0;
    if (ultra_timing_profile(&config.timing, (uint8_t)profile, 40000000, 500)) return 7;
    if (ultra_sender_init(&line, &config, boards, items, 32)) return 8;
    line.preamble_left = 0;
    if (ultra_update_begin(&update, &line, &descriptor, &target, &layout, uid, ULTRA_PORT_DIFF_POWER,
                          (uint8_t)atoi(argv[5]), (uint8_t)atoi(argv[4]), 2, now, read_body, NULL)) return 9;
    while (update.state < ULTRA_UPDATE_QUIET) {
        int result;
        now = update.due_us;
        result = ultra_update_next(&update, now, frame);
        if (result == 0) continue;
        if (result != 48) return 10;
        printf("%lld %u %u %u %u ", (long long)update.transport.frame_start_us, line.timing.clock_hz,
               line.timing.bit_ticks, line.timing.zero_ticks, line.timing.one_ticks);
        for (i = 0; i < 48; ++i) printf("%02x", frame[i]);
        putchar('\n');
    }
    return 0;
}
