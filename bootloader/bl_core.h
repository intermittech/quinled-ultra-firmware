/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED Ultra ABI-3 bootloader: production logic exercised in rv32emu.
 *
 * Boot decision (bl_decide), the update session (bl_sess_*), cursor erase, commit and verify. Flash and the
 * other board functions go through the bl_plat_* interface: bl_hw.c on the board, a flash model in the tests.
 * Formats and the flash map: src/core/ultra_update.h. Design: wiki, Updating over the data line. */
#ifndef BL_CORE_H
#define BL_CORE_H

#include <stdint.h>
#include "upd_platform.h"
#include "bl_protection.h"
#include "../src/mcu/profile.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BL_VERSION
#define BL_VERSION 3u
#endif
#ifndef BL_HW_ID
#define BL_HW_ID UPD_HW_ID_D2A_PLUS_5CH
#endif
#ifndef BOARD
#define BOARD BL_HW_ID
#endif
#include "../src/board/boards.h"
extern const upd_bl_info_t bl_info;
#ifndef BL_HW_REV
#define BL_HW_REV BOARD_REVISION              /* production builds one bootloader per board revision */
#endif

#define BL_TRIAL_MS          150u              /* listen window of an unconfirmed app's boot */
#define BL_WAIT_MS           3000u             /* requested session: wait this long for an ANNOUNCE */
#define BL_IDLE_MS           UPD_IDLE_TIMEOUT_MS
#define BL_WD_LIMIT          3u                /* watchdog resets in a row -> recovery */
#define BL_RECOVERY_MS       60000u            /* confirmed app: retry after an idle recovery window */
#define BL_ESCAPE_TICKS      (MCU_RESET_HSI_HZ / 50u)   /* DIN continuously high for 20 ms at the reset HSI (V20x 8 MHz) */
#if defined(UPD_STATUS_PROFILE) && !defined(BL_POOL_PAGES)
#define BL_POOL_PAGES        1u                /* declared multipass pool; status has a separate staging block */
#endif
#ifndef BL_POOL_PAGES
#define BL_POOL_PAGES        48u               /* partly received / waiting pages kept in RAM (12.5 KB) */
#endif
#define BL_MAX_PAGES         ((UPD_APP_BODY_MAX + UPD_PAGE_BYTES - 1u) / UPD_PAGE_BYTES)
#define BL_ST_RESERVE        4u                /* status records kept free for VERIFIED / CONFIRMED */

/* ------------------------------------------------------------------------------------------------ platform */
const uint8_t *bl_plat_ptr(uint32_t addr);                 /* flash contents (memory mapped on the board) */
int  bl_plat_erase_page(uint32_t addr);                    /* 256 B (fast mode), 0 = ok */
int  bl_plat_program_page(uint32_t addr, const uint8_t *d); /* 256 B (fast page program), 0 = ok */
int  bl_plat_program_half(uint32_t addr, uint16_t v);     /* standard half-word programming, 0 = ok */
void bl_plat_begin(uint32_t deadline_ms);                 /* one shared physical deadline for an operation */
uint32_t bl_plat_ms(void);
int bl_plat_layout_ok(void);                              /* loaded protection and fixed V203 geometry */
uint32_t bl_plat_crc32(uint32_t addr, uint32_t nwords);    /* upd_crc32_words from 0xFFFFFFFF (CRC unit) */
int  bl_plat_power_ok(void);                               /* VDD high enough to write flash (PVD) */
void bl_plat_kick(void);                                   /* watchdog */
void bl_plat_watchdog_start(void);                         /* before any app startup code */
int bl_plat_din_high(void);                                /* reset-clock DIN input */
uint32_t bl_plat_boot_ticks(void);                         /* 8 MHz ticks, starts on first use */
int bl_plat_options_read(uint16_t ob[8]);                  /* twice, byte/inverse checked, at reset clock */
int bl_plat_boot_open(void);                              /* loaded WPR bits 0-1 */
int bl_plat_options_write(const uint16_t ob[8]);           /* erase + all eight low bytes, preserve RDPR */
uint32_t bl_plat_ticks(void);                              /* (status profile) SysTick at HCLK */
const uint8_t *bl_plat_uid(void);                          /* (status profile) the MCU's 12-byte UID */

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
__attribute__((noipa)) int bl_desc_check(const uint8_t *d);
__attribute__((noipa)) void bl_app_read(bl_app_t *a);
void bl_trial_record(bl_app_t *a, int seen);

/* ------------------------------------------------------------------------------------------------ boot decision */
enum { BL_RST_POWER = 1, BL_RST_WATCHDOG = 2, BL_RST_SOFT = 3 };
/* RCC->RSTSCKR flags (= RCC_*RSTF of ch32v20x.h; bl_main.c checks that they match) */
#define BL_RCC_PINRSTF       0x04000000u
#define BL_RCC_PORRSTF       0x08000000u
#define BL_RCC_SFTRSTF       0x10000000u
#define BL_RCC_IWDGRSTF      0x20000000u
#define BL_RCC_WWDGRSTF      0x40000000u
#define BL_RCC_LPWRRSTF      0x80000000u
#define BL_RESET_KIND_API    1
/* BL_RST_* of this reset from the RSTSCKR flags. The specific flags decide first: an internal reset (software,
 * watchdog) can raise PINRSTF as well, because the reset structure drives the NRST line (so on the STM32F1 this RCC
 * mirrors; the QFN20 has no NRST pin, so PINRSTF can only come from inside). A power-on classification wipes the
 * hand-over block, so a software reset must never be taken for one: the app's UPD_REQ_ENTER and the bootloader's
 * own UPD_REQ_BOOT_ONCE would be lost at every reset. PINRSTF alone still counts as power-like. */
uint32_t bl_reset_kind(uint32_t rcc_flags);
typedef enum {
    BL_ACT_JUMP = 0,           /* start the app now, with the independent watchdog running */
    BL_ACT_VERIFY,             /* check the body CRC after a commit, then decide again */
    BL_ACT_TRIAL,              /* unconfirmed app: listen BL_TRIAL_MS, then start it */
    BL_ACT_SESSION,            /* the app asked: listen for a session */
    BL_ACT_RECOVERY,           /* no usable app: indefinite; confirmed watchdog loop: BL_RECOVERY_MS */
    BL_ACT_RESET,              /* options written: reset to load and check them before any app */
} bl_action_t;

/* ho: the hand-over block as found in RAM (validated and updated in place). reset: BL_RST_* of this reset
 * (the caller reads RCC->RSTSCKR and clears it, so the flags describe exactly this reset); rcc_flags: those raw
 * flags, kept in ho->reset_flags for the app */
bl_action_t bl_decide(const bl_app_t *a, upd_handover_t *ho, uint32_t reset, uint32_t rcc_flags);
/* Shared by main and the board model: early watchdog, decision, DIN escape and protection before any app. */
bl_action_t bl_boot(bl_app_t *a, upd_handover_t *ho, uint32_t reset, uint32_t rcc_flags);
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
    BL_S_ERASE,
    BL_S_FAILED,               /* same session cannot restart; new ANNOUNCE required */
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
    uint32_t erase_cursor, erase_end;
    uint8_t  written[(BL_MAX_PAGES + 7u) / 8u];
    uint32_t nwritten;
    uint8_t  skip_result;      /* UPD_RES_SAME / _REJECTED for BL_S_SKIP */
    uint8_t  seen_ultra;       /* a normal Ultra frame was seen (trial: ends the window) */
    bl_gate_t gate;
    /* exit request (bl_sess_poll returns 1): what to write into the hand-over before the reset */
    uint32_t exit_request, exit_result, exit_version;
    /* transaction counters */
    uint32_t frames, upd_frames, bad_frames, pages_programmed, flash_errors, evicted;
#if UPD_STATUS_PROFILE
    /* the status profile (bl_session.c; docs/update.md): selection, rate lock, poll-driven flash, replies */
    struct {
        uint32_t frame_t0;             /* local ticks of the handled frame's last falling edge (bl_main.c) */
        uint32_t t_host, t_ticks;      /* the newest valid UPD frame end: host us (trailer TIME) / local ticks */
        uint32_t ref_host, ref_ticks;  /* the rate measurement's reference observation */
        uint32_t span_host, span_ticks;/* the locked rate: span_ticks local ticks per span_host host us */
        uint8_t  have_ref, locked;
        uint8_t  selected, sel_session;/* 0 declared; SELECT locks status mode: 1 mine, 2 excluded */
        uint8_t  state, error;         /* UPD_STATE_COMMITTED once committed; the FAILED reason */
        uint8_t  op, op_due;           /* the pending blocking operation; announced BUSY (starts at op_at) */
        uint8_t  revoked, mask, reply; /* mask: chunks of the block at `committed`; reply: answer[] to send */
        uint16_t brr;                  /* the answer UART's divider at the locked rate */
        uint32_t op_at, reply_at;      /* local ticks */
        uint32_t committed;            /* readback-verified body prefix (bytes) */
        uint8_t  answer[32];
        uint8_t  stage[UPD_PAGE_BYTES];
    } st;
    /* the declared profile's erase lead (bl_session.c): no inactivity until lead_end (ms) */
    uint32_t lead_end;
    uint8_t  lead;
#endif
    bl_pool_t pool[BL_POOL_PAGES];
} bl_sess_t;

__attribute__((noipa)) void bl_sess_init(bl_sess_t *s, bl_action_t mode, const bl_app_t *app, uint32_t now_ms);
/* one complete frame: nbits, and its last min(nbits, 384) bits right-aligned in tail[48] (tail[47] = last byte;
 * nbits < 384 leaves the unused leading bytes undefined). Flash work happens in here (it blocks). */
__attribute__((noipa)) void bl_sess_frame(bl_sess_t *s, uint32_t nbits, const uint8_t *tail, uint32_t now_ms);
/* timeouts; 1 = leave now: write s->exit_* into the hand-over and reset */
__attribute__((noipa)) int bl_sess_poll(bl_sess_t *s, uint32_t now_ms);
/* after bl_sess_poll returned 1: the hand-over for the next boot (a result stays until the app takes it) */
void bl_ho_exit(upd_handover_t *ho, const bl_sess_t *s);

#ifdef __cplusplus
}
#endif

#endif
