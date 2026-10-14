/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The flash records: see flashrec.h. */
#include <stdint.h>
#include "flashrec.h"
#include "pagepair.h"
#include "scratch.h"
#include "lto.h"
#include "../hal/board.h"

/* the record being programmed: in the main loop's scratch (core/scratch.h; the lifetime record: in place), written as
 * its own type and read by flash_op as half-words (flash_op starts with a compiler barrier) */
typedef char rec_fits_scratch[(sizeof(cal_rec_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(store_slot_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(scene_rec_t) <= CORE_SCRATCH_WORDS * 4u) ? 1 : -1];

/* a pair's page and an entry's address */
static uint32_t pp_addr(uint32_t page0, uint32_t page1, uint32_t at, uint32_t bytes)
{
    return (PP_PAGE(at) ? page1 : page0) + PP_ENTRY(at) * bytes;
}

/* a pair's erase: the one due, else one ahead (pagepair.h) */
static uint32_t pp_prepare(pp_pos_t *pos, uint32_t page0, uint32_t page1)
{
    uint32_t p, r;
    if (!pp_erase_due(pos, &p)) {
        return FLASH_NONE;
    }
    r = flash_op(p ? page1 : page0, 0, 0u, p ? page1 : page0);
    if (r == FLASH_OK) {
        pp_erased(pos);
    }
    return r;
}

/* a store's position: the erase that is due first (main.c asks for it only while low: flash_may), then the entry.
 * PP_NONE: *r says why (that erase failed) */
static uint32_t pp_write_at(const pp_desc_t *d, pp_pos_t *pos, uint32_t page0, uint32_t page1, uint32_t *r)
{
    uint32_t p;
    *r = pp_erase_due(pos, &p) == 2u ? pp_prepare(pos, page0, page1) : FLASH_OK;
    return *r == FLASH_OK ? pp_plan(d, pos) : PP_NONE;
}

/* ---- settings: an A/B pair (0.6.4; 0.6.3 erased its one page when full, and a power cut right after that erase
 * lost the stored settings), 8 records of 32 bytes per page, the same records as before (page A is 0.6.3's page) */
static const pp_desc_t set_pp = { STORE_SLOT_BYTES, STORE_SLOTS, 4u, 0u, store_valid };
static pp_pos_t set_pos;
static uint32_t set_seq;

MEASURED int settings_load(settings_t *s, store_ext_t *ext)
{
    const store_slot_t *r = (const store_slot_t *)pp_find(&set_pp, (const uint8_t *)SETTINGS_ADDR,
                                                          (const uint8_t *)SETTINGS_B_ADDR, &set_pos);
    if (!r) {
        store_ext_clear(ext);
        return -1;
    }
    *s = r->s;
    *ext = r->ext;
    set_seq = r->seq;
    return 0;
}

MEASURED uint32_t settings_store(const settings_t *s, const store_ext_t *ext)
{
    uint32_t r, at = pp_write_at(&set_pp, &set_pos, SETTINGS_ADDR, SETTINGS_B_ADDR, &r);
    if (at == PP_NONE) {
        return r;
    }
    store_pack_ext(s, ext, ++set_seq, (store_slot_t *)core_scratch);
    return flash_op(pp_addr(SETTINGS_ADDR, SETTINGS_B_ADDR, at, STORE_SLOT_BYTES), (const uint16_t *)core_scratch,
                    STORE_SLOT_BYTES / 2u, 0u);
}

uint32_t settings_pending(void)
{
    uint32_t p;
    return pp_erase_due(&set_pos, &p);
}

uint32_t settings_prepare(void)
{
    return pp_prepare(&set_pos, SETTINGS_ADDR, SETTINGS_B_ADDR);
}

/* ---- current calibration: an A/B pair (0.6.4), 4 records of 64 bytes per page (page A is 0.6.3's page) */
static const pp_desc_t cal_pp = { CAL_REC_BYTES, CAL_RECS, 4u, 0u, cal_valid };
static pp_pos_t cal_pos;
static uint32_t cal_seq;
static uint16_t cal_cnt;

MEASURED int cal_load(isense_cal_t *c, uint16_t *count)
{
    const cal_rec_t *r = (const cal_rec_t *)pp_find(&cal_pp, (const uint8_t *)CAL_PAGE_ADDR,
                                                    (const uint8_t *)CAL_B_ADDR, &cal_pos);
    if (!r) {
        cal_cnt = 0;
        *count = 0;
        return -1;
    }
    *c = r->c;
    cal_seq = r->seq;
    *count = cal_cnt = r->count;
    return c->levels != 0u ? 0 : -1;
}

MEASURED uint32_t cal_store(const isense_cal_t *c, uint16_t *count)
{
    uint32_t r, at = pp_write_at(&cal_pp, &cal_pos, CAL_PAGE_ADDR, CAL_B_ADDR, &r);
    uint16_t n = (uint16_t)(cal_cnt + (c->levels != 0u));
    if (at == PP_NONE) {
        return r;
    }
    cal_pack(c, ++cal_seq, n, (cal_rec_t *)core_scratch);
    r = flash_op(pp_addr(CAL_PAGE_ADDR, CAL_B_ADDR, at, CAL_REC_BYTES), (const uint16_t *)core_scratch,
                 CAL_REC_BYTES / 2u, 0u);
    if (r == FLASH_OK) {
        *count = cal_cnt = n;                            /* (a failed write is not counted) */
    }
    return r;
}

uint32_t cal_pending(void)
{
    uint32_t p;
    return pp_erase_due(&cal_pos, &p);
}

uint32_t cal_prepare(void)
{
    return pp_prepare(&cal_pos, CAL_PAGE_ADDR, CAL_B_ADDR);
}

/* ---- power-on scene log: an A/B pair (0.6.4, core/scene.h scene_pp; 0.6.3: a ring that erased the page it entered),
 * 16 records of 16 bytes per page */
static pp_pos_t scene_pos;
static uint16_t scene_seq;

MEASURED int scene_load(scene_t *sc)
{
    const scene_rec_t *r = (const scene_rec_t *)pp_find(&scene_pp, (const uint8_t *)SCENE_LOG_ADDR,
                                                        (const uint8_t *)SCENE_B_ADDR, &scene_pos);
    if (!r) {
        return -1;
    }
    *sc = r->s;
    scene_seq = r->seq;
    return 0;
}

MEASURED uint32_t scene_store(const scene_t *sc)
{
    uint32_t r, at = pp_write_at(&scene_pp, &scene_pos, SCENE_LOG_ADDR, SCENE_B_ADDR, &r);
    if (at == PP_NONE) {
        return r;
    }
    scene_log_pack(sc, ++scene_seq, (scene_rec_t *)core_scratch);
    return flash_op(pp_addr(SCENE_LOG_ADDR, SCENE_B_ADDR, at, SCENE_REC_BYTES), (const uint16_t *)core_scratch,
                    SCENE_REC_BYTES / 2u, 0u);
}

uint32_t scene_pending(void)
{
    uint32_t p;
    return pp_erase_due(&scene_pos, &p);
}

uint32_t scene_prepare(void)
{
    return pp_prepare(&scene_pos, SCENE_LOG_ADDR, SCENE_B_ADDR);
}

/* ---- the lifetime record (core/energy.h): its A/B pair */
static life_pos_t life_pos;

MEASURED uint32_t life_load(life_rec_t *r)
{
    return life_find((const uint8_t *)LIFE_PAGE0_ADDR, (const uint8_t *)LIFE_PAGE1_ADDR, r, &life_pos);
}

MEASURED uint32_t life_store(energy_t *e)
{
    uint32_t at = life_plan(&life_pos);
    if (at == LIFE_PLAN_NONE) {
        return FLASH_NONE;
    }
    energy_pack(e);                                      /* (programmed from e->lt itself) */
    return flash_op(pp_addr(LIFE_PAGE0_ADDR, LIFE_PAGE1_ADDR, at, LIFE_REC_BYTES), (const uint16_t *)&e->lt,
                    LIFE_REC_BYTES / 2u, 0u);
}

MEASURED uint32_t life_pending(void)
{
    uint32_t p;
    return pp_erase_due(&life_pos, &p);
}

uint32_t life_room(void)
{
    return life_free(&life_pos);
}

MEASURED uint32_t life_prepare(void)
{
    return pp_prepare(&life_pos, LIFE_PAGE0_ADDR, LIFE_PAGE1_ADDR);
}

/* ---- the power-on effect store (core/ultra.h ULTRA_FX_*): both pages erased, the heads, the header last */
#ifdef __riscv                                           /* (the heads' layout is the target's; host tests differ) */
typedef char fx_store_fits[(ULTRA_FX_HDR + ULTRA_LIGHTS * ULTRA_HEAD_BYTES <= 512u && ULTRA_HEAD_BYTES == 98u) ? 1 : -1];
#endif

#if UPPER_RECORDS
/* two banks of two pages (0.6.5, the upper layout): a store rewrites the bank that doesn't hold the newest one, its
 * header (written last) with the next 16-bit sequence number in its two spare bytes (0.6.0-0.6.4 wrote 0 there), so a
 * power cut during a store keeps the effects stored before it */
static const uint32_t fx_bank[2] = { SCENE_FX_ADDR, SCENE_FX_B_ADDR };
static uint8_t fx_cur;                                   /* the bank holding the newest store (none: 0) */
static uint16_t fx_seq;

static uint32_t fx_hdr(uint32_t b, uint32_t *seq)
{
    const uint8_t *h = (const uint8_t *)(uintptr_t)fx_bank[b];
    *seq = (uint32_t)h[10] | (uint32_t)h[11] << 8;
    return ((uint32_t)h[0] | (uint32_t)h[1] << 8 | (uint32_t)h[2] << 16 | (uint32_t)h[3] << 24) == ULTRA_FX_MAGIC;
}

MEASURED uint8_t scene_fx_load(ultra_t *u, uint8_t crc[ULTRA_LIGHTS])
{
    uint32_t sa, sb, a = fx_hdr(0, &sa), b = fx_hdr(1, &sb);
    fx_cur = (uint8_t)(b && (!a || (int16_t)(uint16_t)(sb - sa) > 0));
    fx_seq = (uint16_t)(fx_cur ? sb : sa);
    return ultra_fx_load(u, (const uint8_t *)(uintptr_t)fx_bank[fx_cur], crc);
}

MEASURED uint32_t scene_fx_store(const ultra_t *u, uint8_t mask, const uint8_t crc[ULTRA_LIGHTS])
{
    uint8_t *h = (uint8_t *)core_scratch;
    uint32_t i, nb = fx_cur ^ 1u, base = fx_bank[nb], r = flash_op(base, 0, 0u, base);   /* page 0: its header goes */
    if (mask != 0u && r == FLASH_OK) {
        r = flash_op(base + 256u, 0, 0u, base + 256u);
    }
    for (i = 0; i < ULTRA_LIGHTS && r == FLASH_OK; i++) {
        if (mask & (1u << i)) {
            r = flash_op(base + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES, (const uint16_t *)&u->light[i],
                         ULTRA_HEAD_BYTES / 2u, 0u);
        }
    }
    if (r != FLASH_OK) {
        return r;                                        /* no header after a failed head */
    }
    core_scratch[0] = ULTRA_FX_MAGIC;
    core_scratch[2] = (uint32_t)(uint16_t)(fx_seq + 1u) << 16;   /* bytes 10-11: the sequence (8-9: crc[3-4] below) */
    h[4] = mask;                                         /* (mask 0: a valid header that loads nothing: cleared) */
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        h[5u + i] = crc[i];
    }
    /* the header last, its magic last of all: a header cut short has no magic, so the bank doesn't count (with the
     * magic first, an erased sequence number 0xFFFF could look newer than the other bank's) */
    r = flash_op(base + 4u, (const uint16_t *)(h + 4), (ULTRA_FX_HDR - 4u) / 2u, 0u);
    if (r == FLASH_OK) {
        r = flash_op(base, (const uint16_t *)h, 2u, 0u);
    }
    if (r == FLASH_OK) {
        fx_cur = (uint8_t)nb;
        fx_seq++;
    }
    return r;
}
#else
/* 0.6.4's layout: one bank, rewritten in place (a cut during a store loses the effects, never anything else) */
MEASURED uint8_t scene_fx_load(ultra_t *u, uint8_t crc[ULTRA_LIGHTS])
{
    return ultra_fx_load(u, (const uint8_t *)SCENE_FX_ADDR, crc);
}

MEASURED uint32_t scene_fx_store(const ultra_t *u, uint8_t mask, const uint8_t crc[ULTRA_LIGHTS])
{
    uint8_t *h = (uint8_t *)core_scratch;
    uint32_t i, r = flash_op(SCENE_FX_ADDR, 0, 0u, SCENE_FX_ADDR);   /* erases page 0: the header goes */
    if (mask == 0u || r != FLASH_OK) {
        return r;
    }
    r = flash_op(SCENE_FX_ADDR + 256u, 0, 0u, SCENE_FX_ADDR + 256u);
    for (i = 0; i < ULTRA_LIGHTS && r == FLASH_OK; i++) {
        if (mask & (1u << i)) {
            r = flash_op(SCENE_FX_ADDR + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES, (const uint16_t *)&u->light[i],
                         ULTRA_HEAD_BYTES / 2u, 0u);
        }
    }
    if (r != FLASH_OK) {
        return r;                                        /* no header after a failed head */
    }
    core_scratch[0] = ULTRA_FX_MAGIC;
    core_scratch[2] = 0u;                                /* the 2 spare bytes (10, 11): 0, not leftover scratch */
    h[4] = mask;
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        h[5u + i] = crc[i];
    }
    return flash_op(SCENE_FX_ADDR, (const uint16_t *)h, ULTRA_FX_HDR / 2u, 0u);   /* the header last */
}
#endif

uint32_t hw_flash_half(uint32_t addr, uint16_t v)
{
    return flash_op(addr, &v, 1u, 0u);
}

uint32_t flash_rejected(void)
{
    return set_pos.bad | (uint32_t)cal_pos.bad << 6 | (uint32_t)scene_pos.bad << 12 | (uint32_t)life_pos.bad << 18;
}
