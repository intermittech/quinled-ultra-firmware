/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The app's side of the firmware update over the data line (wiki: Updating over the data line; formats:
 * ultra_update.h; the other side: bootloader/). Portable, host-tested (test/test_update.c).
 *
 * UPD frames (BLOCK + Ultra header trailer with the UPD flag, 384 bits) are forwarded and acted on:
 *  - UPD headers at frame ends (every board reads the tail of 384-bit frames): after UPD_APP_ENTER_RUN of them in a
 *    row the board goes TRANSPARENT: its gate forwards everything and it captures the whole 384-bit frame (so the
 *    boards behind it see the session too, one after the other). Lights keep running; UPD frames count as signal.
 *  - ANNOUNCE (both descriptor halves): the same checks as the bootloader (upd_desc_check + version policy).
 *    Take it -> UPD_APP_TAKE: main.c sets UPD_REQ_ENTER in the hand-over block and resets into the bootloader.
 *    Not for this board / not newer -> stay transparent and show "already active" / NACK at the session's END.
 *  - PING: a freshly installed app (status not CONFIRMED) that has completed its startup checks
 *    confirms itself -> UPD_APP_CONFIRM: main.c appends UPD_ST_CONFIRMED to the descriptor page's status records.
 *    Normal operation without an idle PING never authorizes confirmation.
 *  - Back to normal: at the first normal Ultra header, or UPD_IDLE_CLOSE_MS without UPD frames. An ANNOUNCE (any image,
 *    any family) holds transparency through that image's declared erase lead (counted from its descriptor, upd_app.c)
 *    until its session's DATA / END / ABORT: a transit board keeps forwarding through a foreign erase silence.
 * Without a bootloader (a standalone build: info.bl_version 0) it only forwards; nothing is taken or confirmed. */
#ifndef UPD_APP_H
#define UPD_APP_H

#include <stdint.h>
#include "upd_platform.h"

#define UPD_APP_ENTER_RUN  2u          /* UPD headers in a row before going transparent */

typedef struct {
    uint8_t  hw_id, hw_rev, bl_version;   /* the bootloader's info (UPD_BL_INFO); bl_version 0 = no bootloader */
    uint8_t  desc_ok;                     /* this app's descriptor is valid */
    uint8_t  confirmed;                   /* UPD_ST_CONFIRMED is recorded */
    uint8_t  st_next;                     /* first erased status record (UPD_ST_COUNT = full) */
    uint8_t status_flags, trials;
    const upd_bl_info_t *boot;           /* immutable protected identity, no RAM copy */
    uint32_t version;                     /* this app's version word (0 without a descriptor) */
} upd_app_info_t;

/* actions for main.c */
enum {
    UPD_APP_NONE = 0,
    UPD_APP_TAKE,                         /* request UPD_REQ_ENTER and reset into the bootloader */
    UPD_APP_CONFIRM,                      /* append UPD_ST_CONFIRMED at info.st_next (then upd_app_confirmed) */
    UPD_APP_SHOW_SAME,                    /* the session's image is already installed: "already active" */
    UPD_APP_SHOW_REJECTED,                /* not for this board / older without FORCE: NACK */
};

typedef struct {
    upd_app_info_t info;
    uint8_t  run;                         /* UPD headers in a row */
    uint8_t  transparent;
    uint8_t  startup_ok;                  /* main completed initialization and checked the board identity */
    uint8_t  ping_seen;
    uint8_t  confirm_done;
    uint8_t  have_half, pending_session, force;
    uint8_t  pending[UPD_DESC_BYTES];
    uint8_t  decided;                     /* `session` is decided (skipped) */
    uint8_t  session;
    uint8_t  result;                      /* UPD_APP_SHOW_* for the END of `session`, 0 = shown / none */
    uint32_t last_upd_ms;
    uint32_t ping_ms;                    /* first idle PING; repeats never extend an active request */
    uint32_t sessions_seen, takes, frames;   /* statistics */
#if UPD_STATUS_PROFILE
    const uint8_t *uid;                   /* this MCU's 12-byte UID (main sets it) */
    uint8_t  selected, sel_session;       /* reset: declared (0); SELECT: this UID (1) / excluded (2) */
#endif
    uint8_t  forward_busy;                /* explicit flag: a wrapped deadline may equal zero */
    uint8_t  forward_session;             /* only this session's DATA / END / ABORT ends the hold */
    uint32_t forward_busy_until;          /* declared erase silence; DATA / END / ABORT ends it */
    uint32_t forward_hold_ms;             /* that session's hold, from its first ANNOUNCE half */
} upd_app_t;

void upd_app_page(const upd_app_info_t *info, const uint8_t *desc, uint8_t *out);
void upd_app_init(upd_app_t *u, const upd_app_info_t *info);
/* every frame end: was there a valid Ultra header, and its FLAGS */
void upd_app_frame_end(upd_app_t *u, uint8_t ultra_header, uint8_t flags, uint32_t now_ms);
/* a whole 384-bit frame captured while transparent; returns an UPD_APP_* action */
uint32_t upd_app_frame(upd_app_t *u, const uint8_t *frame, uint32_t now_ms);
/* every ms: the transparent timeout; returns an UPD_APP_* action (a skipped session's result if END was missed) */
uint32_t upd_app_poll(upd_app_t *u, uint32_t now_ms);
/* main.c verified the CONFIRMED record (ok = 1) or could not (0: retry at the next idle PING). */
void upd_app_confirmed(upd_app_t *u, int ok);
static inline int upd_app_confirm_live(const upd_app_t *u, uint32_t now_ms) {
    return u->ping_seen && now_ms - u->ping_ms < UPD_DISPATCH_MS;   /* (the host's dispatch allowance) */
}
static inline int upd_app_transparent(const upd_app_t *u) { return u->transparent; }
/* behind a bootloader of another hardware id (an image for another board, flashed over SWD): 1 = the
 * app keeps its outputs dark (main.c) or its delay line off with the line released (main_ur_digital_m.c, BOARD 3) */
static inline int upd_app_wrong_hw(const upd_app_info_t *info, uint32_t hw_id)
{
    return info->bl_version != 0u && info->hw_id != hw_id;
}

/* Read the descriptor page (64 + 192 bytes) and bootloader info (32 bytes) into *info. */
void upd_app_read_info(upd_app_info_t *info, const uint8_t *desc_page, const uint8_t *bl_info);

#endif
