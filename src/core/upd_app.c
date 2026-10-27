/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The app's side of the firmware update (see upd_app.h). */
#include "upd_app.h"
#include "boot_api.h"
#include <string.h>

void upd_app_read_info(upd_app_info_t *info, const uint8_t *desc_page, const uint8_t *bl_info)
{
    uint32_t i;
    info->hw_id = info->hw_rev = info->bl_version = 0;
    info->desc_ok = info->confirmed = 0;
    info->st_next = (uint8_t)UPD_ST_COUNT;
    info->version = 0;
    info->boot = 0;
    info->status_flags = info->trials = 0;
    if (bl_info[0] == 'Q' && bl_info[1] == 'D' && bl_info[2] == 'B' && bl_info[3] == '2') {
        info->boot = (const upd_bl_info_t *)bl_info;
        info->bl_version = info->boot->bl_version;
        info->hw_id = info->boot->target.bytes[UPD_T_BOARD];
        info->hw_rev = info->boot->target.bytes[UPD_T_REVISION];
    }
    if (info->bl_version == 0u ||
        boot_api_desc(desc_page, &info->boot->target, info->bl_version) != 0) {
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
        if(v == UPD_ST_VERIFIED) info->status_flags |= ULTRA_FU_F_VERIFIED;
        if(v == UPD_ST_TRIAL_QUIET || v == UPD_ST_TRIAL_SEEN) info->trials++;
        if(v == UPD_ST_TRIAL_SEEN) info->status_flags |= ULTRA_FU_F_TRIAL_SEEN;
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

static uint32_t ping_check(upd_app_t *u, uint32_t now_ms)
{
    if (!upd_app_confirm_live(u, now_ms)) u->ping_seen = 0;
    if (u->ping_seen && u->startup_ok && !u->confirm_done && u->info.desc_ok &&
        u->info.bl_version != 0u && u->info.st_next < UPD_ST_COUNT) {
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
        u->run = u->decided = u->have_half = 0;
        u->result = 0;
        u->transparent = 0;                /* the host sends normal frames again */
        u->ping_seen = 0;
        u->forward_busy = 0;
#if UPD_STATUS_PROFILE
        u->selected = 0;
#endif
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
    rc = boot_api_desc(u->pending, &u->info.boot->target, u->info.bl_version);
    if (rc == UPD_ERROR_DESCRIPTOR) {
        return UPD_APP_NONE;               /* halves of two images: wait for the next pair */
    }
    u->sessions_seen++;
    u->session = u->pending_session;
    u->decided = 1;
#if UPD_STATUS_PROFILE
    if (!u->selected && rc != 0) {
        u->result = 0;                    /* foreign/invalid group image: forward only */
        return UPD_APP_NONE;
    }
#endif
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
    uint32_t type, idx, session, i;
    int check = boot_api_frame(f);
    if (check == -2) return UPD_APP_SHOW_REJECTED;
    if (check != 0) {
        return UPD_APP_NONE;
    }
    type = f[UPD_B_TYPE];
    session = f[UPD_B_SESSION];
    idx = upd_get16(f + UPD_B_INDEX);
    /* Forwarders see foreign-family broadcasts too: never close during an ANNOUNCE's declared erase lead (from its
     * final half); the two-second idle close counts from the hold's end, and DATA/END/ABORT of that session restore
     * it. This grants forwarding only, never flash. The hold is the protocol's (ultra_update.h upd_transit_hold_ms:
     * the first half's erase units, any forwarder-checkable invalidity the full layout-1 slot); a second half
     * without its first gets the largest. */
    if (type == UPD_ANNOUNCE) {
        const uint32_t hold = idx == 0u ? upd_transit_hold_ms(f + UPD_B_DATA) :
                              u->forward_busy && session == u->forward_session ? u->forward_hold_ms : UPD_TRANSIT_HOLD_MAX_MS;
        u->forward_busy = 1;
        u->forward_session = (uint8_t)session;
        u->forward_hold_ms = hold;
        u->forward_busy_until = now_ms + hold;
    } else if (session == u->forward_session && (type == UPD_DATA || type == UPD_END || type == UPD_ABORT))
        u->forward_busy = 0;
    if (type == UPD_DATA) return UPD_APP_NONE;
#if UPD_STATUS_PROFILE
    /* the status profile (docs/update.md): only the node an UPD_SELECT names (UID and immutable target) takes the
     * session's ANNOUNCE; the others do not reboot. UPD_STATUS is the bootloader's. */
    if (type == UPD_SELECT) {
        u->selected = (u->uid && u->info.boot && !memcmp(f + UPD_B_DATA, u->uid, 12) &&
                       !memcmp(f + UPD_B_DATA + 12, u->info.boot->target.bytes, 16)) ? 1u : 2u;
        u->sel_session = (uint8_t)session;
        u->have_half = 0;
        return UPD_APP_NONE;
    }
    /* A selection belongs only to its session: the next session's PING or ANNOUNCE ends it (so does the end of
     * transparency), and that later session may be a blind one. The excluded session itself never enters. */
    if (u->selected && session != u->sel_session && (type == UPD_PING || type == UPD_ANNOUNCE) && session != 0u)
        u->selected = 0;
    if (type == UPD_STATUS || (type == UPD_ANNOUNCE && u->selected == 2u))
        return UPD_APP_NONE;
#endif
    u->frames++;
    u->last_upd_ms = now_ms;
    switch (type) {
    case UPD_ANNOUNCE: {
        u->ping_seen = 0;
        if (u->decided && session == u->session) {
            return UPD_APP_NONE;           /* repeats of a session already decided */
        }
        if (session != u->pending_session || u->have_half == 0u) {
            u->pending_session = (uint8_t)session;
            u->have_half = 0;
            u->force = f[UPD_B_FLAGS];
        } else if (u->force != f[UPD_B_FLAGS] ||
                   ((u->have_half & (1u << idx)) &&
                    memcmp(u->pending + idx * UPD_CHUNK_BYTES, f + UPD_B_DATA, UPD_CHUNK_BYTES))) {
            u->session = (uint8_t)session;
            u->decided = 1;
            u->have_half = u->result = 0;
            return UPD_APP_NONE;           /* conflicting repeats require a new session */
        }
        for (i = 0; i < UPD_CHUNK_BYTES; i++) {
            u->pending[idx * UPD_CHUNK_BYTES + i] = f[UPD_B_DATA + i];
        }
        u->have_half = (uint8_t)(u->have_half | (1u << idx));
        return u->have_half == 3u ? evaluate(u) : UPD_APP_NONE;
    }
    case UPD_END:
    case UPD_ABORT:
        if (type == UPD_ABORT && session == u->pending_session) {
            u->have_half = u->result = 0;
            u->session = (uint8_t)session;
            u->decided = 1;                /* cancellation requires a new session */
        }
        if (u->decided && session == u->session && u->result != 0u) {
            if (type == UPD_END &&
                (upd_get32(f + UPD_B_DATA) != upd_get32(u->pending + UPD_D_BODY_CRC) ||
                 upd_get32(f + UPD_B_DATA + 4) != upd_get32(u->pending + UPD_D_BODY_LEN)))
                return UPD_APP_NONE;
            uint32_t r = type == UPD_END ? u->result : UPD_APP_NONE;
            u->result = 0;
            return r;
        }
        return UPD_APP_NONE;
    case UPD_PING:
        if (session != 0u || u->have_half || u->decided) return UPD_APP_NONE;
        if (!u->ping_seen) u->ping_ms = now_ms;
        u->ping_seen = 1;
        return ping_check(u, now_ms);
    default:
        return UPD_APP_NONE;               /* DATA: not for the app */
    }
}

uint32_t upd_app_poll(upd_app_t *u, uint32_t now_ms)
{
    if (u->transparent && now_ms - u->last_upd_ms >= UPD_IDLE_CLOSE_MS &&
        (!u->forward_busy || (int32_t)(now_ms - u->forward_busy_until) >= (int32_t)UPD_IDLE_CLOSE_MS)) {
        u->transparent = 0;
        u->run = u->decided = u->have_half = u->forward_busy = 0;
#if UPD_STATUS_PROFILE
        u->selected = 0;
#endif
        if (u->result != 0u) {             /* END missed: show the result now */
            uint32_t r = u->result;
            u->result = 0;
            return r;
        }
    }
    return ping_check(u, now_ms);
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

void upd_app_page(const upd_app_info_t *info, const uint8_t *desc, uint8_t *out)
{
    uint32_t i;
    out[0]=ULTRA_FU_FORMAT_VERSION;
    out[1]=(uint8_t)(info->status_flags | (info->confirmed ? ULTRA_FU_F_CONFIRMED:0u));
    out[2]=info->trials;
    for(i=0;i<12u;++i) out[4+i]=desc[UPD_D_FW_VERSION+i];
    for(i=0;i<4u;++i) out[16+i]=desc[UPD_D_CRC+i];
}
