/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The flash records: see flashrec.h. */
#include <stdint.h>
#include "flashrec.h"
#include "pagepair.h"
#include "scratch.h"
#include "lto.h"
#if BOARD == BOARD_DP_UD
#include "ud/udfx.h"
#endif
#include "../hal/board.h"

/* the record being programmed: in the main loop's scratch (core/scratch.h; the lifetime record: in place), written as
 * its own type and read by flash_op as half-words (flash_op starts with a compiler barrier) */
typedef char rec_fits_scratch[(sizeof(cal_rec_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(store_slot_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(scene_rec_t) <= CORE_SCRATCH_WORDS * 4u) ? 1 : -1];

/* ---- the A/B page pairs (pagepair.h; 0.6.6: + the calibration history), one descriptor and one path each (0.6.6,
 * review round 5: a copy of the path per store with its two page addresses was ~190 B more). REC_* (flashrec.h) in
 * main.c's erase order. */
typedef struct {
    const pp_desc_t *pp;
    uint32_t page[2];                                    /* A, B (hal/board.h) */
} rec_t;

#if BOARD == BOARD_DP_UD
static int ufx_valid(const uint8_t *r)                   /* the Ultra-Digital receiver's local-effects record */
{
    return udfx_rec_valid(r);
}
static const pp_desc_t set_pp = { UX_REC_BYTES, 2u, 4u, 0u, ufx_valid };   /* (REC_SET's pair: BOARD 3 has no settings) */
#else
static const pp_desc_t set_pp = { STORE_SLOT_BYTES, STORE_SLOTS, 4u, 0u, store_valid };
#endif
#if BOARD != BOARD_DP_UD
static const pp_desc_t cal_pp = { CAL_REC_BYTES, CAL_RECS, 4u, 0u, cal_valid };
#if MON_EXTRA
static const pp_desc_t hist_pp = { CALHIST_REC_BYTES, CALHIST_RECS, 4u, 0u, calhist_valid };
#endif
#endif

static const rec_t recs[REC_COUNT] = {
    /* the lifetime record (core/energy.h): its A/B pair */
    [REC_LIFE] = { &life_pp, { LIFE_PAGE0_ADDR, LIFE_PAGE1_ADDR } },
    /* settings: an A/B pair (0.6.4; 0.6.3 erased its one page when full, and a power cut right after that erase lost
     * the stored settings), 8 records of 32 bytes per page, the same records as before (page A is 0.6.3's page) */
    [REC_SET] = { &set_pp, { SETTINGS_ADDR, SETTINGS_B_ADDR } },
#if BOARD != BOARD_DP_UD
    /* current calibration: an A/B pair (0.6.4), 4 records of 64 bytes per page (page A is 0.6.3's page) */
    [REC_CAL] = { &cal_pp, { CAL_PAGE_ADDR, CAL_B_ADDR } },
    /* power-on scene log: an A/B pair (0.6.4, core/scene.h scene_pp; 0.6.3: a ring that erased the page it entered),
     * 16 records of 16 bytes per page */
    [REC_SCENE] = { &scene_pp, { SCENE_LOG_ADDR, SCENE_B_ADDR } },
#if MON_EXTRA
    /* the calibration history (core/calhist.h, M2, 0.6.6): an A/B pair, 4 records of 64 bytes per page */
    [REC_CALHIST] = { &hist_pp, { CALHIST_ADDR, CALHIST_B_ADDR } },
#endif
#endif
};
static pp_pos_t rec_pos[REC_COUNT];
#if BOARD != BOARD_DP_UD
static uint32_t set_seq;
static uint32_t cal_seq;
static uint16_t cal_cnt, scene_seq;
#endif

/* a pair's page p (0 / 1), and an entry's address */
static uint32_t rec_page(uint32_t i, uint32_t p)
{
    return recs[i].page[p];
}

static uint32_t rec_addr(uint32_t i, uint32_t at)
{
    return rec_page(i, PP_PAGE(at)) + PP_ENTRY(at) * recs[i].pp->bytes;
}

/* the newest valid record (NULL: none) and the pair's write position */
static const uint8_t *rec_find(uint32_t i)
{
    return pp_find(recs[i].pp, (const uint8_t *)(uintptr_t)rec_page(i, 0), (const uint8_t *)(uintptr_t)rec_page(i, 1),
                   &rec_pos[i]);   /* (uintptr_t: the 64-bit host build) */
}

MEASURED uint32_t flash_rec_pending(uint32_t i)   /* (tools/flash_stores.py: by name) */
{
    uint32_t p;
    return pp_erase_due(&rec_pos[i], &p);
}

/* a pair's erase: the one due, else one ahead (pagepair.h) */
MEASURED uint32_t flash_rec_prepare(uint32_t i)
{
    uint32_t p, r;
    if (!pp_erase_due(&rec_pos[i], &p)) {
        return FLASH_NONE;
    }
    r = flash_op(rec_page(i, p), 0, 0u, rec_page(i, p));
    if (r == FLASH_OK) {
        pp_erased(&rec_pos[i]);
    }
    return r;
}

/* a store's position: the erase that is due first (main.c asks for it only while low: flash_may), then the entry.
 * PP_NONE: *r says why (that erase failed) */
static uint32_t rec_write_at(uint32_t i, uint32_t *r)
{
    *r = flash_rec_pending(i) == 2u ? flash_rec_prepare(i) : FLASH_OK;
    return *r == FLASH_OK ? pp_plan(recs[i].pp, &rec_pos[i]) : PP_NONE;
}

/* the record (its bytes: the descriptor's) programmed at `at` (rec_write_at's, pp_plan's), and the pair's position told
 * how it went (0.6.6, review round 5: pagepair.h pp_done; a failed write never counts as the newest record) */
static uint32_t rec_program(uint32_t i, uint32_t at, const void *src)
{
    uint32_t r = flash_op(rec_addr(i, at), (const uint16_t *)src, recs[i].pp->bytes / 2u, 0u);
    pp_done(&rec_pos[i], at, r == FLASH_OK);
    return r;
}

#if BOARD == BOARD_DP_UD
/* ---- the local-effects record (core/ud/udfx.h): two 128-byte records a page, the CRC last */
MEASURED uint32_t ufx_load(uint8_t rec[128])
{
    const uint8_t *r = rec_find(REC_SET);
    uint32_t i;
    if (!r) {
        return 0;
    }
    for (i = 0; i < UX_REC_BYTES; i++) {
        rec[i] = r[i];
    }
    return 1;
}

MEASURED uint32_t ufx_store(struct udfx_s *u)
{
    uint32_t r, at = rec_write_at(REC_SET, &r), buf[UX_REC_BYTES / 4u];   /* (the stack: this board has no scratch RAM) */
    if (at == PP_NONE) {
        return r;
    }
    udfx_rec_pack(u, u->rec_seq + 1u, (uint8_t *)buf);
    r = rec_program(REC_SET, at, buf);
    if (r == FLASH_OK) {
        u->rec_seq++;
    }
    return r;
}
#else
/* ---- settings */
MEASURED int settings_load(settings_t *s, store_ext_t *ext)
{
    const store_slot_t *r = (const store_slot_t *)rec_find(REC_SET);
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
    uint32_t r, at = rec_write_at(REC_SET, &r);
    if (at == PP_NONE) {
        return r;
    }
    store_pack_ext(s, ext, ++set_seq, (store_slot_t *)core_scratch);
    return rec_program(REC_SET, at, core_scratch);
}
#endif

#if BOARD != BOARD_DP_UD
/* ---- current calibration */
MEASURED int cal_load(isense_cal_t *c, uint16_t *count)
{
    const cal_rec_t *r = (const cal_rec_t *)rec_find(REC_CAL);
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
    uint32_t r, at = rec_write_at(REC_CAL, &r);
    uint16_t n = (uint16_t)(cal_cnt + (c->levels != 0u));
    if (at == PP_NONE) {
        return r;
    }
    cal_pack(c, ++cal_seq, n, (cal_rec_t *)core_scratch);
    r = rec_program(REC_CAL, at, core_scratch);
    if (r == FLASH_OK) {
        *count = cal_cnt = n;                            /* (a failed write is not counted) */
    }
    return r;
}

/* ---- power-on scene log */
MEASURED int scene_load(scene_t *sc)
{
    const scene_rec_t *r = (const scene_rec_t *)rec_find(REC_SCENE);
    if (!r) {
        return -1;
    }
    *sc = r->s;
    scene_seq = r->seq;
    return 0;
}

MEASURED uint32_t scene_store(const scene_t *sc)
{
    uint32_t r, at = rec_write_at(REC_SCENE, &r);
    if (at == PP_NONE) {
        return r;
    }
    scene_log_pack(sc, ++scene_seq, (scene_rec_t *)core_scratch);
    return rec_program(REC_SCENE, at, core_scratch);
}
#endif

/* ---- the lifetime record (core/energy.h): its write never erases (FLASH_NONE while its erase is due) */
MEASURED uint32_t life_load(life_rec_t *r)
{
    const uint8_t *best = rec_find(REC_LIFE);
    if (best) {
        *r = *(const life_rec_t *)best;
    }
    return best != 0;
}

MEASURED uint32_t life_store(energy_t *e)
{
    uint32_t at = pp_plan(&life_pp, &rec_pos[REC_LIFE]);
    if (at == PP_NONE) {
        return FLASH_NONE;
    }
    energy_pack(e);                                      /* (programmed from e->lt itself) */
    return rec_program(REC_LIFE, at, &e->lt);
}

uint32_t life_room(void)
{
    return pp_free(&life_pp, &rec_pos[REC_LIFE]);
}

#if MON_EXTRA && BOARD != BOARD_DP_UD
/* ---- the calibration history (M2, 0.6.6): programmed from *h itself, as the lifetime record */
MEASURED uint32_t calhist_load(calhist_t *h)
{
    const calhist_t *r = (const calhist_t *)rec_find(REC_CALHIST);
    if (r) {
        *h = *r;
    }
    return r != 0;
}

MEASURED uint32_t calhist_store(calhist_t *h)
{
    uint32_t r, at = rec_write_at(REC_CALHIST, &r);
    if (at == PP_NONE) {
        return r;
    }
    calhist_pack(h);
    return rec_program(REC_CALHIST, at, h);
}
#endif

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
#if BOARD == BOARD_DP_UD
    return rec_pos[REC_SET].bad | (uint32_t)rec_pos[REC_LIFE].bad << 18;
#else
    return rec_pos[REC_SET].bad | (uint32_t)rec_pos[REC_CAL].bad << 6 | (uint32_t)rec_pos[REC_SCENE].bad << 12 |
           (uint32_t)rec_pos[REC_LIFE].bad << 18;
#endif
}
