/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The app's side of the firmware update (see upd_app.h). */
#include "upd_app.h"

void upd_app_read_info(upd_app_info_t *info, const uint8_t *desc_page, const uint8_t *bl_info)
{
    uint32_t i;
    info->hw_id = info->hw_rev = info->bl_version = 0;
    info->desc_ok = info->confirmed = 0;
    info->st_next = (uint8_t)UPD_ST_COUNT;
    info->version = 0;
    if (bl_info[0] == 'Q' && bl_info[1] == 'D' && bl_info[2] == 'B' && bl_info[3] == 'L') {
        info->bl_version = bl_info[4];
        info->hw_id = bl_info[5];
        info->hw_rev = bl_info[6];
    }
    if (info->bl_version == 0u ||
        upd_desc_check(desc_page, upd_crc32_words(0xFFFFFFFFu, desc_page, 15u), info->hw_id, info->hw_rev,
                       info->bl_version) != 0) {
        return;
    }
    info->desc_ok = 1;
    info->version = upd_get32(desc_page + UPD_D_FW_VERSION);
    for (i = 0; i < UPD_ST_COUNT; i++) {
        const uint8_t *p = desc_page + 2u * (UPD_ST_FIRST + i);
        uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
        if (upd_erased16(v)) {
            info->st_next = (uint8_t)i;
            break;
        }
        if (v == UPD_ST_CONFIRMED) {
            info->confirmed = 1;
        }
    }
}

void upd_app_init(upd_app_t *u, const upd_app_info_t *info)
{
    uint8_t *b = (uint8_t *)u;
    uint32_t i;
    for (i = 0; i < sizeof(*u); i++) {
        b[i] = 0;
    }
    u->info = *info;
    u->confirm_done = info->confirmed;
}

static uint32_t ping_check(upd_app_t *u)
{
    if (u->ping_seen && u->normal_seen && !u->confirm_done && u->info.desc_ok && u->info.bl_version != 0u &&
        u->info.st_next < UPD_ST_COUNT) {
        u->confirm_done = 1;               /* asked once; upd_app_confirmed(0) re-arms it */
        return UPD_APP_CONFIRM;
    }
    return UPD_APP_NONE;
}

void upd_app_frame_end(upd_app_t *u, uint8_t ultra_header, uint8_t flags, uint32_t now_ms)
{
    if (!ultra_header) {
        return;                            /* WS frames, or a header that didn't decode: no change */
    }
    if (flags & UPD_HDR_FLAG) {
        u->last_upd_ms = now_ms;
        if (u->run < 255u) {
            u->run++;
        }
        if (u->run >= UPD_APP_ENTER_RUN) {
            u->transparent = 1;
        }
    } else {
        u->run = 0;
        u->normal_seen = 1;
        u->transparent = 0;                /* the host sends normal frames again */
    }
}

static uint32_t evaluate(upd_app_t *u)
{
    uint32_t v = upd_get32(u->pending + UPD_D_FW_VERSION);
    int rc;
    u->have_half = 0;
    if (u->info.bl_version == 0u) {
        return UPD_APP_NONE;               /* no bootloader: forward only */
    }
    rc = upd_desc_check(u->pending, upd_crc32_words(0xFFFFFFFFu, u->pending, 15u), u->info.hw_id, u->info.hw_rev,
                        u->info.bl_version);
    if (rc == -1) {
        return UPD_APP_NONE;               /* halves of two images: wait for the next pair */
    }
    u->sessions_seen++;
    u->session = u->pending_session;
    u->decided = 1;
    if (rc != 0) {
        u->result = UPD_APP_SHOW_REJECTED;
    } else if (u->info.desc_ok && v == u->info.version && !u->force) {
        u->result = UPD_APP_SHOW_SAME;
    } else if (u->info.desc_ok && v < u->info.version && !u->force) {
        u->result = UPD_APP_SHOW_REJECTED;
    } else {
        u->takes++;
        return UPD_APP_TAKE;               /* the bootloader checks it again and installs it */
    }
    return UPD_APP_NONE;
}

uint32_t upd_app_frame(upd_app_t *u, const uint8_t *f, uint32_t now_ms)
{
    uint32_t type, idx, session;
    if (upd_check_frame(f) != 0) {
        return UPD_APP_NONE;
    }
    u->frames++;
    u->last_upd_ms = now_ms;
    type = f[UPD_B_TYPE];
    session = f[UPD_B_SESSION];
    idx = upd_get16(f + UPD_B_INDEX);
    switch (type) {
    case UPD_ANNOUNCE: {
        uint32_t i;
        if (idx > 1u || (u->decided && session == u->session)) {
            return UPD_APP_NONE;           /* repeats of a session already decided */
        }
        if (session != u->pending_session || u->have_half == 0u) {
            u->pending_session = (uint8_t)session;
            u->have_half = 0;
            u->force = 0;
        }
        for (i = 0; i < UPD_CHUNK_BYTES; i++) {
            u->pending[idx * UPD_CHUNK_BYTES + i] = f[UPD_B_DATA + i];
        }
        u->have_half = (uint8_t)(u->have_half | (1u << idx));
        u->force = (uint8_t)(u->force | ((f[UPD_B_FLAGS] & UPD_F_FORCE) != 0u));
        return u->have_half == 3u ? evaluate(u) : UPD_APP_NONE;
    }
    case UPD_END:
    case UPD_ABORT:
        if (u->decided && session == u->session && u->result != 0u) {
            uint32_t r = type == UPD_END ? u->result : UPD_APP_NONE;
            u->result = 0;
            return r;
        }
        return UPD_APP_NONE;
    case UPD_PING:
        u->ping_seen = 1;
        return ping_check(u);
    default:
        return UPD_APP_NONE;               /* DATA: not for the app */
    }
}

uint32_t upd_app_poll(upd_app_t *u, uint32_t now_ms)
{
    if (u->transparent && now_ms - u->last_upd_ms >= UPD_APP_IDLE_MS) {
        u->transparent = 0;
        u->run = 0;
        if (u->result != 0u) {             /* END missed: show the result now */
            uint32_t r = u->result;
            u->result = 0;
            return r;
        }
    }
    return ping_check(u);                  /* a PING seen before the first normal frame confirms now */
}

void upd_app_confirmed(upd_app_t *u, int ok)
{
    if (ok) {
        u->info.confirmed = 1;
        u->info.st_next++;
    } else {
        u->confirm_done = 0;
        u->ping_seen = 0;                  /* try again at the next PING */
    }
}
