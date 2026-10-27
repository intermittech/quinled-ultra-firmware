/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader identity, boot decision, protection and installed-image verification. */
#include "bl_core.h"
#include "boot_api.h"
#include "boot_api_layout.h"
#ifndef BL_BUILD_ID
#define BL_BUILD_ID "dev0000"
#endif
/* Bytes, never native integer layout: the protected ABI-3 identity. */
#define BL_BE32(v) (uint8_t)((v)>>24), (uint8_t)((v)>>16), (uint8_t)((v)>>8), (uint8_t)(v)
#if defined(__GNUC__)
__attribute__((section(".blinfo"), used))
#endif
const upd_bl_info_t bl_info = {
    { 'Q', 'D', 'B', '2' }, BL_VERSION,
    {{ BL_HW_ID, MCU_UPD_MCU, MCU_UPD_PACKAGE, MCU_UPD_LAYOUT,
       UPD_BOOT_ABI, MCU_UPD_FEATURES, BOARD_VARIANT, BL_HW_REV,
       BL_BE32(UPD_APP_BODY), BL_BE32(UPD_APP_BODY_MAX) }},
    { MCU_UPD_GEOMETRY, MCU_UPD_PROGRAM_MS, MCU_UPD_ERASE_MS }, 5, BL_BUILD_ID
};
/* The family's identity facts (src/mcu/<family>/flash.h) are the pinned header's profiles. */
#if defined(MCU_CH32V40X) && MCU_CH32V40X == 1
typedef char bl_family_is_layout4[(MCU_UPD_MCU == UPD_MCU_V407 && MCU_UPD_PACKAGE == UPD_PACKAGE_RET6 &&
    MCU_UPD_LAYOUT == UPD_LAYOUT_V407 && MCU_UPD_GEOMETRY == UPD_V407_GEOMETRY && MCU_UPD_PROGRAM_MS == UPD_V407_PROGRAM_MS &&
    MCU_UPD_ERASE_MS == UPD_V407_ERASE_MS && UPD_ERASE_BYTES == MCU_UPD_ERASE_BYTES) ? 1 : -1];
#else
typedef char bl_family_is_layout1[(MCU_UPD_MCU == UPD_MCU_V203 && MCU_UPD_PACKAGE == UPD_PACKAGE_F8U6 &&
    MCU_UPD_LAYOUT == UPD_LAYOUT_V203 && MCU_UPD_GEOMETRY == UPD_V203_GEOMETRY && MCU_UPD_PROGRAM_MS == UPD_V203_PROGRAM_MS &&
    MCU_UPD_ERASE_MS == UPD_V203_ERASE_MS && UPD_ERASE_BYTES == MCU_UPD_ERASE_BYTES) ? 1 : -1];
#endif

static void zero(void *p, uint32_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) {
        *b++ = 0;
    }
}

static uint32_t half_at(uint32_t addr)
{
    const uint8_t *p = bl_plat_ptr(addr);
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8);          /* flash is read little-endian, like the CPU */
}

/* ------------------------------------------------------------------------------------------------ descriptor */
__attribute__((noinline, used)) int bl_desc_check(const uint8_t *d)
{
    return upd_desc_check(d, &bl_info.target, BL_VERSION);
}

void bl_app_read(bl_app_t *a)
{
    const uint8_t *d = bl_plat_ptr(UPD_APP_DESC);
    uint32_t i;
    zero(a, sizeof(*a));
    a->st_next = (uint8_t)UPD_ST_COUNT;
    if (bl_desc_check(d) != 0) {
        return;
    }
    a->desc_ok = 1;
    a->version = upd_get32(d + UPD_D_FW_VERSION);
    a->body_len = upd_get32(d + UPD_D_BODY_LEN);
    a->body_crc = upd_get32(d + UPD_D_BODY_CRC);
    for (i = 0; i < UPD_ST_COUNT; i++) {
        uint32_t v = half_at(UPD_APP_DESC + 2u * (UPD_ST_FIRST + i));
        if (upd_erased16(v)) {
            a->st_next = (uint8_t)i;
            break;
        }
        if (v == UPD_ST_VERIFIED) {
            a->verified = 1;
        } else if (v == UPD_ST_CONFIRMED) {
            a->confirmed = 1;
        } else if (v == UPD_ST_TRIAL_SEEN || v == UPD_ST_TRIAL_QUIET) {
            a->trials++;
            a->strikes = (uint8_t)(a->strikes + (v == UPD_ST_TRIAL_SEEN));
        }                                                     /* anything else: a torn record, skipped */
    }
}

static int st_append(bl_app_t *a, uint16_t v, uint32_t keep_free)
{
    if ((uint32_t)a->st_next + keep_free >= UPD_ST_COUNT || !bl_plat_power_ok()) {
        return -1;
    }
    bl_plat_begin(2u * MCU_UPD_PROGRAM_MS);
    if (bl_plat_program_half(UPD_APP_DESC + 2u * (UPD_ST_FIRST + a->st_next), v) != 0 ||
        half_at(UPD_APP_DESC + 2u * (UPD_ST_FIRST + a->st_next)) != v) {
        return -1;
    }
    a->st_next++;
    return 0;
}

void bl_trial_record(bl_app_t *a, int seen)
{
    (void)st_append(a, seen ? UPD_ST_TRIAL_SEEN : UPD_ST_TRIAL_QUIET, BL_ST_RESERVE);
}

/* ------------------------------------------------------------------------------------------------ hand-over */
int bl_ho_valid(const upd_handover_t *ho)
{
    return upd_ho_valid(ho);                                  /* the app uses the same helpers */
}

void bl_ho_seal(upd_handover_t *ho)
{
    upd_ho_seal(ho);
}

/* ------------------------------------------------------------------------------------------------ boot decision */
uint32_t bl_reset_kind(uint32_t f)
{
    if (f & (BL_RCC_IWDGRSTF | BL_RCC_WWDGRSTF)) {
        return BL_RST_WATCHDOG;
    }
    if (f & BL_RCC_SFTRSTF) {
        return BL_RST_SOFT;                                   /* also when PINRSTF came with it */
    }
    if (f & (BL_RCC_PORRSTF | BL_RCC_PINRSTF | BL_RCC_LPWRRSTF)) {
        return BL_RST_POWER;
    }
    return BL_RST_SOFT;                                       /* no flag: they were taken already */
}

bl_action_t bl_decide(const bl_app_t *a, upd_handover_t *ho, uint32_t reset, uint32_t rcc_flags)
{
    uint32_t req;
    bl_action_t act;
    if (!bl_ho_valid(ho) || reset == BL_RST_POWER) {
        zero(ho, sizeof(*ho));                                /* power-on: RAM is random, nothing carries over */
    }
    ho->reset_flags |= rcc_flags;                             /* for the app (the bootloader clears the RCC flags) */
    if (reset == BL_RST_WATCHDOG && ho->wd_resets < 255u) {
        ho->wd_resets++;
    }
    req = ho->request;
    ho->request = UPD_REQ_NONE;
    if (req == UPD_REQ_ENTER) {
        act = BL_ACT_SESSION;
    } else if (!a->desc_ok || ho->wd_resets >= BL_WD_LIMIT) {
        act = BL_ACT_RECOVERY;
    } else if (req == UPD_REQ_BOOT_ONCE) {
        act = BL_ACT_JUMP;                                    /* set by this bootloader in this RAM-continuous run,
                                                                 after the body passed its CRC (the VERIFIED record
                                                                 may be missing: VDD was low), a trial window or a
                                                                 skipped session; power-on wipes it */
    } else if (!a->verified) {
        act = BL_ACT_VERIFY;
    } else if (a->confirmed) {
        act = BL_ACT_JUMP;
    } else {
        act = BL_ACT_TRIAL;
    }
    bl_ho_seal(ho);
    return act;
}

/* At reset HSI, before clocks/sampler/output startup. An untrusted snapshot never reaches the option erase.
 * The pending CRC is kept in the hand-over (app RAM is not safe across the bootloader's own reset). */
static int protect(upd_handover_t *ho)
{
    uint16_t ob[8];
    uint32_t check = ho->reserved;
    int ok = bl_plat_options_read(ob), open = bl_plat_boot_open();
    if ((check & 0xFFFF0000u) == BL_PROT_PENDING) {
        ok = ok && !open && ob[0] == 0xA5u &&
             (check & 0xFFFFu) == upd_crc16((const uint8_t *)ob, sizeof ob);
        ho->reserved = ok ? BL_PROT_OK : BL_PROT_FAIL;
        return ok ? 0 : -1;
    }
    if (!ok) {
        ho->reserved = BL_PROT_BAD;
        return -1;
    }
    if (ob[0] != 0xA5u) {
        return -1;                             /* never lift or enable read protection */
    }
    if (!open) {
        return 0;
    }
    if (!bl_plat_power_ok()) {
        return -1;                             /* dark recovery until a safe reboot; no unprotected app */
    }
    ob[4] &= MCU_BL_WRP_KEEP;
#ifdef MCU_BL_USER_KEEP
    ob[1] &= MCU_BL_USER_KEEP;
#endif
    ho->reserved = BL_PROT_PENDING | upd_crc16((const uint8_t *)ob, sizeof ob);
    if (bl_plat_options_write(ob) == 0) {
        return 1;
    }
    ho->reserved = BL_PROT_FAIL;
    return -1;
}

bl_action_t bl_boot(bl_app_t *a, upd_handover_t *ho, uint32_t reset, uint32_t rcc_flags)
{
    bl_action_t act;
    int prot;
    bl_plat_watchdog_start();
    bl_app_read(a);
    act = bl_decide(a, ho, reset, rcc_flags);
    if (reset == BL_RST_POWER && bl_plat_din_high()) {
        uint32_t start = bl_plat_boot_ticks();
        while (bl_plat_din_high()) {
            if (bl_plat_boot_ticks() - start >= BL_ESCAPE_TICKS) {
                act = BL_ACT_SESSION;
                break;
            }
        }
    }
    prot = protect(ho);
    if (prot > 0) {
        /* Protection's reset must not lose a requested session or the power-on escape. */
        if (act == BL_ACT_SESSION) {
            ho->request = UPD_REQ_ENTER;
        }
        bl_ho_seal(ho);
        return BL_ACT_RESET;
    }
    if (prot < 0) {
        a->desc_ok = 0;                         /* disallow every session exit/timeout jump this boot */
        return BL_ACT_RECOVERY;
    }
    return act;
}

int bl_verify(bl_app_t *a)
{
    if (!a->desc_ok) {
        return -1;
    }
    if (bl_plat_crc32(UPD_APP_BODY, a->body_len / 4u) == a->body_crc) {
        (void)st_append(a, UPD_ST_VERIFIED, 0);               /* low VDD: not recorded, checked again next boot */
        a->verified = 1;
        return 0;
    }
    if (bl_plat_power_ok()) {
        bl_plat_begin(2u * MCU_UPD_ERASE_MS);
        (void)bl_plat_erase_page(UPD_APP_DESC);               /* a bad body never boots: no descriptor */
    }
    a->desc_ok = 0;
    return -1;
}

bl_action_t bl_after_verify(bl_app_t *a)
{
    if (bl_verify(a) != 0) {
        return BL_ACT_RECOVERY;
    }
    return a->confirmed ? BL_ACT_JUMP : BL_ACT_TRIAL;
}

extern int bl_api_frame(const uint8_t *);
__attribute__((section(".boot_api"),used))
const uintptr_t bl_api[] = {
    BOOT_API_MAGIC, BOOT_API_META,
    (uintptr_t)bl_desc_check + MCU_BOOT_API_FLASH_BASE - MCU_BOOT_API_LINK_BASE,
    (uintptr_t)bl_api_frame + MCU_BOOT_API_FLASH_BASE - MCU_BOOT_API_LINK_BASE,
    0
};
