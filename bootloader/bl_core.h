/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED dig2analog-ultra-5ch bootloader: portable logic (host-tested in bootloader/test).
 *
 * Boot decision (bl_decide), the update session (bl_sess_*), the erase plan, commit and verify. Flash and the
 * other board functions go through the bl_plat_* interface: bl_hw.c on the board, a flash model in the tests.
 * Formats and the flash map: src/core/ultra_update.h. Design: research/firmware-update.md section 3.4. */
#ifndef BL_CORE_H
#define BL_CORE_H

#include <stdint.h>
#include "ultra_update.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BL_VERSION
#define BL_VERSION 1u
#endif
#ifndef BL_HW_ID
#define BL_HW_ID UPD_HW_ID_D2A_ULTRA_5CH
#endif
#ifndef BL_HW_REV
#define BL_HW_REV UPD_REV_PB11                 /* production builds one bootloader per board revision */
#endif

#define BL_TRIAL_MS          150u              /* listen window of an unconfirmed app's boot */
#define BL_WAIT_MS           3000u             /* requested session: wait this long for an ANNOUNCE */
#define BL_IDLE_MS           3000u             /* a session ends this long after its last UPD frame */
#define BL_WD_LIMIT          3u                /* watchdog resets in a row -> recovery */
#ifndef BL_POOL_PAGES
#define BL_POOL_PAGES        48u               /* partly received / waiting pages kept in RAM (12.5 KB) */
#endif
#define BL_MAX_PAGES         ((UPD_APP_BODY_MAX + UPD_PAGE_BYTES - 1u) / UPD_PAGE_BYTES)   /* 214 */
#define BL_ST_RESERVE        4u                /* status records kept free for VERIFIED / CONFIRMED */

/* ------------------------------------------------------------------------------------------------ platform */
const uint8_t *bl_plat_ptr(uint32_t addr);                 /* flash contents (memory mapped on the board) */
int  bl_plat_erase_sector(uint32_t addr);                  /* 4 KB, 0 = ok */
int  bl_plat_erase_page(uint32_t addr);                    /* 256 B (fast mode), 0 = ok */
int  bl_plat_program_page(uint32_t addr, const uint8_t *d); /* 256 B (fast page program), 0 = ok */
int  bl_plat_program_half(uint32_t addr, uint16_t v);     /* standard half-word programming, 0 = ok */
uint32_t bl_plat_crc32(uint32_t addr, uint32_t nwords);    /* upd_crc32_words from 0xFFFFFFFF (CRC unit) */
int  bl_plat_power_ok(void);                               /* VDD high enough to write flash (PVD) */
void bl_plat_kick(void);                                   /* watchdog */

/* ------------------------------------------------------------------------------------------------ app state */
typedef struct {
    uint8_t  desc_ok;          /* descriptor valid (magic, CRC, layout, this board) */
    uint8_t  verified;         /* UPD_ST_VERIFIED recorded */
    uint8_t  confirmed;        /* UPD_ST_CONFIRMED recorded */
    uint8_t  strikes;          /* UPD_ST_TRIAL_SEEN records (unconfirmed boots with Ultra frames) */
    uint8_t  trials;           /* trial records of any kind */
    uint8_t  st_next;          /* index of the first erased status record (UPD_ST_COUNT = full) */
    uint32_t version;          /* firmware version word (0 if !desc_ok) */
    uint32_t body_len, body_crc;
} bl_app_t;

/* descriptor checks shared by the boot path and the session: 0 = usable on this board */
int  bl_desc_check(const uint8_t *d);
void bl_app_read(bl_app_t *a);

/* ------------------------------------------------------------------------------------------------ boot decision */
enum { BL_RST_POWER = 1, BL_RST_WATCHDOG = 2, BL_RST_SOFT = 3 };
typedef enum {
    BL_ACT_JUMP = 0,           /* start the app now (no clock or peripheral touched) */
    BL_ACT_VERIFY,             /* check the body CRC after a commit, then decide again */
    BL_ACT_TRIAL,              /* unconfirmed app: listen BL_TRIAL_MS, then start it */
    BL_ACT_SESSION,            /* the app asked: listen for a session */
    BL_ACT_RECOVERY,           /* no usable app (or a watchdog loop): listen until a session installs one */
} bl_action_t;

/* ho: the hand-over block as found in RAM (validated and updated in place). reset: BL_RST_* of this reset
 * (the caller reads RCC->RSTSCKR and clears it, so the flags describe exactly this reset); rcc_flags: those raw
 * flags, kept in ho->reset_flags for the app */
bl_action_t bl_decide(const bl_app_t *a, upd_handover_t *ho, uint32_t reset, uint32_t rcc_flags);
int  bl_ho_valid(const upd_handover_t *ho);
void bl_ho_seal(upd_handover_t *ho);

/* after BL_ACT_VERIFY: 0 = body CRC ok and VERIFIED recorded; else the descriptor page was erased */
int  bl_verify(bl_app_t *a);
/* BL_ACT_VERIFY in full mode: verify, then BL_ACT_JUMP (= leave through a reset with UPD_REQ_BOOT_ONCE, the clocks
 * are already up), BL_ACT_TRIAL (unconfirmed) or BL_ACT_RECOVERY (bad body) */
bl_action_t bl_after_verify(bl_app_t *a);

/* ------------------------------------------------------------------------------------------------ session */
typedef enum {
    BL_S_LISTEN = 0,           /* no session yet (trial window, requested session, recovery) */
    BL_S_RECEIVE,              /* accepted: app area erased, pages arrive */
    BL_S_SKIP,                 /* rejected or not needed: wait for the end, then back to the app */
} bl_state_t;

typedef enum { BL_GATE_OPEN = 0, BL_GATE_ULTRA } bl_gate_t;   /* plan for the next frame */

enum { BL_POOL_FREE = 0, BL_POOL_PARTIAL, BL_POOL_COMPLETE };   /* COMPLETE: waits for a quiet moment */
typedef struct {
    uint16_t page;             /* page number in the body */
    uint8_t  mask;             /* chunks present */
    uint8_t  used;             /* BL_POOL_* */
    uint8_t  data[UPD_PAGE_BYTES];
} bl_pool_t;

typedef struct {
    bl_action_t mode;          /* why the bootloader listens: TRIAL, SESSION or RECOVERY */
    bl_state_t state;
    bl_app_t app;              /* the installed app as read at boot */
    uint32_t t_start, t_last;  /* ms: entered listening, last UPD frame */
    uint8_t  session;          /* current session id */
    uint8_t  force;
    uint8_t  have_half;        /* bit 0 / 1: ANNOUNCE halves of `pending` received */
    uint8_t  pending_session;
    uint8_t  pending[UPD_DESC_BYTES];
    uint8_t  desc[UPD_DESC_BYTES];   /* the accepted descriptor */
    uint32_t npages;
    uint8_t  written[(BL_MAX_PAGES + 7u) / 8u];
    uint32_t nwritten;
    uint8_t  skip_result;      /* UPD_RES_SAME / _REJECTED for BL_S_SKIP */
    uint8_t  seen_ultra;       /* a normal Ultra frame was seen (trial: ends the window) */
    bl_gate_t gate;
    /* exit request (bl_sess_poll returns 1): what to write into the hand-over before the reset */
    uint32_t exit_request, exit_result, exit_version;
    /* statistics (UART / tests) */
    uint32_t frames, upd_frames, bad_frames, pages_programmed, flash_errors, evicted;
    bl_pool_t pool[BL_POOL_PAGES];
} bl_sess_t;

void bl_sess_init(bl_sess_t *s, bl_action_t mode, const bl_app_t *app, uint32_t now_ms);
/* one complete frame: nbits, and its last min(nbits, 384) bits right-aligned in tail[48] (tail[47] = last byte;
 * nbits < 384 leaves the unused leading bytes undefined). Flash work happens in here (it blocks). */
void bl_sess_frame(bl_sess_t *s, uint32_t nbits, const uint8_t *tail, uint32_t now_ms);
/* timeouts; 1 = leave now: write s->exit_* into the hand-over and reset */
int  bl_sess_poll(bl_sess_t *s, uint32_t now_ms);
/* recovery blink on the feedback channel (1 Hz, on while 1) */
int  bl_sess_blink(const bl_sess_t *s, uint32_t now_ms);
/* after bl_sess_poll returned 1: the hand-over for the next boot (a result stays until the app takes it) */
void bl_ho_exit(upd_handover_t *ho, const bl_sess_t *s);

/* erase plan for [UPD_APP_DESC, UPD_APP_BODY + body_len): 4 KB sectors where a whole sector is inside the app
 * area, else 256-byte pages. Writes up to max entries (address | 1 = page erase), returns the count. */
uint32_t bl_erase_plan(uint32_t body_len, uint32_t *ops, uint32_t max);

#ifdef __cplusplus
}
#endif

#endif
