/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* App/boot handover and the family flash layout; wire objects live in the pinned shared header. */
#ifndef UPD_PLATFORM_H
#define UPD_PLATFORM_H
#include "ultra_update.h"
#define UPD_HDR_MAGIC ULTRA_MAGIC
#define UPD_FLASH_BASE 0x08000000u
#define UPD_BL_BASE UPD_FLASH_BASE
#define UPD_BL_INFO UPD_BL_INFO_ADDR
#if defined(MCU_CH32V40X) && MCU_CH32V40X == 1
/* layout 4: the 16 KiB bootloader, the metadata in its own 4 KiB erase sector, the body to the zero-wait end */
#define UPD_BL_SIZE UPD_V407_BL_BYTES
#define UPD_APP_DESC UPD_V407_METADATA
#define UPD_APP_BODY UPD_V407_BODY
#define UPD_APP_END UPD_V407_END
#define UPD_ERASE_BYTES 4096u
#define UPD_STATUS_PROFILE 1   /* docs/update.md: the status profile (UPD_SELECT, UPD_STATUS replies) */
#else
#define UPD_BL_SIZE UPD_V203_BL_BYTES
#define UPD_APP_DESC UPD_V203_METADATA
#define UPD_APP_BODY UPD_V203_BODY
#define UPD_APP_END UPD_V203_END
#define UPD_ERASE_BYTES 256u
#endif
#define UPD_APP_BODY_MAX (UPD_APP_END - UPD_APP_BODY)
#define UPD_APP_ENTRY UPD_APP_BODY
#define UPD_PAGE_BYTES 256u
#define UPD_SECTOR_BYTES 4096u
#define UPD_CHUNKS_PER_PAGE UPD_CHUNKS_PER_PROGRAM
#define UPD_HW_ID_D2A_PLUS_5CH 1u
#define UPD_HW_ID_DP_ULTRA_RX 2u
#define UPD_HW_ID_DP_ULTRA_DIGITAL 3u
#define UPD_REV_D2A_R04 4u
#define upd_get16 ultra_get16
#define upd_get32 ultra_get32
#define upd_put16 ultra_put16
#define upd_put32 ultra_put32
#define upd_crc8 ultra_crc8
#define upd_crc16 ultra_crc16
static inline int upd_check_header(const uint8_t *h) {
    return ultra_get16(h)==ULTRA_MAGIC &&
        (h[2]==((UPD_HDR_VERSION<<4)|UPD_HDR_FLAG) ||
         h[2]==((UPD_HDR_VERSION<<4)|UPD_HDR_FLAG|ULTRA_HDR_RW)) &&
        ultra_crc8(h,7)==h[7] ? 0:-1;
}
/* ------------------------------------------------------------------------------------------------ hand-over */
/* 32 bytes at the start of RAM, outside every linker section: survives software and watchdog resets (not power
 * loss; valid only when magic and check match). */
#define UPD_HANDOVER_ADDR    0x20000000u
#define UPD_HO_MAGIC         0x51444F48u  /* "QDOH" */
typedef struct {
    uint32_t magic;
    uint32_t request;        /* app / bootloader -> bootloader at the next boot */
    uint32_t result;         /* bootloader -> app: what the last session did (UPD_RES_*) */
    uint32_t result_version; /* the version the result is about */
    uint32_t wd_resets;      /* watchdog resets in a row; the app clears it after 10 s of normal running */
    uint32_t reset_flags;    /* RCC->RSTSCKR of the resets since the app last looked: the bootloader clears the
                                RCC flags at every boot (its own software resets would pile up otherwise) and
                                ORs them in here; the app reads and clears this instead of RCC->RSTSCKR */
    uint32_t reserved;
    uint32_t check;          /* ~(magic ^ request ^ result ^ result_version ^ wd_resets) */
} upd_handover_t;
#define UPD_REQ_NONE         0u
#define UPD_REQ_ENTER        0x55504421u  /* "UPD!": the app saw an ANNOUNCE it wants: stay and listen */
#define UPD_REQ_BOOT_ONCE    0x424F4F54u  /* "BOOT": jump without a listen window (set by the bootloader) */
#define UPD_RES_NONE         0u
#define UPD_RES_UPDATED      1u           /* app: play ACK + version */
#define UPD_RES_SAME         2u           /* app: play "already active" (1 short) */
#define UPD_RES_REJECTED     3u           /* app: play NACK (1 long) */
#define UPD_RES_INCOMPLETE   4u           /* app: play "2 long" (staging only: old firmware kept) */

static inline uint32_t upd_ho_sum(const upd_handover_t *ho)
{
    return ~(ho->magic ^ ho->request ^ ho->result ^ ho->result_version ^ ho->wd_resets);
}
static inline int upd_ho_valid(const upd_handover_t *ho)
{
    return ho->magic == UPD_HO_MAGIC && ho->check == upd_ho_sum(ho);
}
static inline void upd_ho_seal(upd_handover_t *ho)
{
    ho->magic = UPD_HO_MAGIC;
    ho->check = upd_ho_sum(ho);
}


#endif
