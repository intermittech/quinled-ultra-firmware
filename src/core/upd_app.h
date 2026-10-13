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
 *  - PING: a freshly installed app (status not CONFIRMED) that has also seen normal Ultra frames since boot
 *    confirms itself -> UPD_APP_CONFIRM: main.c appends UPD_ST_CONFIRMED to the descriptor page's status records.
 *  - Back to normal: at the first normal Ultra header, or UPD_APP_IDLE_MS without UPD frames.
 * Without a bootloader (a standalone build: info.bl_version 0) it only forwards; nothing is taken or confirmed. */
#ifndef UPD_APP_H
#define UPD_APP_H

#include <stdint.h>
#include "ultra_update.h"

#define UPD_APP_ENTER_RUN  2u          /* UPD headers in a row before going transparent */
#define UPD_APP_IDLE_MS    2000u       /* transparent ends this long after the last UPD frame */

typedef struct {
    uint8_t  hw_id, hw_rev, bl_version;   /* the bootloader's info (UPD_BL_INFO); bl_version 0 = no bootloader */
    uint8_t  desc_ok;                     /* this app's descriptor is valid */
    uint8_t  confirmed;                   /* UPD_ST_CONFIRMED is recorded */
    uint8_t  st_next;                     /* first erased status record (UPD_ST_COUNT = full) */
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
    uint8_t  normal_seen;                 /* a normal Ultra header since boot */
    uint8_t  ping_seen;
    uint8_t  confirm_done;
    uint8_t  have_half, pending_session, force;
    uint8_t  pending[UPD_DESC_BYTES];
    uint8_t  decided;                     /* `session` is decided (skipped) */
    uint8_t  session;
    uint8_t  result;                      /* UPD_APP_SHOW_* for the END of `session`, 0 = shown / none */
    uint32_t last_upd_ms;
    uint32_t sessions_seen, takes, frames;   /* statistics */
} upd_app_t;

void upd_app_init(upd_app_t *u, const upd_app_info_t *info);
/* every frame end: was there a valid Ultra header, and its FLAGS */
void upd_app_frame_end(upd_app_t *u, uint8_t ultra_header, uint8_t flags, uint32_t now_ms);
/* a whole 384-bit frame captured while transparent; returns an UPD_APP_* action */
uint32_t upd_app_frame(upd_app_t *u, const uint8_t *frame, uint32_t now_ms);
/* every ms: the transparent timeout; returns an UPD_APP_* action (a skipped session's result if END was missed) */
uint32_t upd_app_poll(upd_app_t *u, uint32_t now_ms);
/* main.c wrote the CONFIRMED record (ok = 1) or could not (0: tried again at the next PING) */
void upd_app_confirmed(upd_app_t *u, int ok);
static inline int upd_app_transparent(const upd_app_t *u) { return u->transparent; }

/* read the descriptor page (64 + 192 bytes) and the bootloader info (16 bytes) into *info */
void upd_app_read_info(upd_app_info_t *info, const uint8_t *desc_page, const uint8_t *bl_info);

#endif
