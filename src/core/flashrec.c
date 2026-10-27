/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The flash records: see flashrec.h. */
#include <stdint.h>
#include <string.h>
#include "flashrec.h"
#include "pagepair.h"
#include "scratch.h"
#include "lto.h"
#if BOARD == BOARD_UR_DIGITAL_M
#include "ur_digital_m/udfx.h"
#include "ur_digital_m/udcal.h"
#endif
#include "../target.h"

/* the record being programmed: in the main loop's scratch (core/scratch.h; the lifetime record: in place), written as
 * its own type and read by flash_op as half-words (flash_op starts with a compiler barrier) */
typedef char rec_fits_scratch[(sizeof(cal_rec_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(store_slot_t) <= CORE_SCRATCH_WORDS * 4u &&
                               sizeof(scene_rec_t) <= CORE_SCRATCH_WORDS * 4u) ? 1 : -1];

/* ---- the A/B page pairs (pagepair.h; with the calibration history), one descriptor each and one path for all (a
 * copy of the path per store with its two page addresses would be ~190 B more). REC_* (flashrec.h) in main.c's erase
 * order. */
typedef struct {
    const pp_desc_t *pp;
    uint32_t page[2];                                    /* A, B (mcu/ch32v20x/layout.h) */
} rec_t;

#if BOARD == BOARD_UR_DIGITAL_M
static int ufx_valid(const uint8_t *r)                   /* the Diff-Power-UR-Digital-M's local-effects record */
{
    return udfx_rec_valid(r);
}
static const pp_desc_t set_pp = { 256u, UX_REC_BYTES, 2u, 4u, 0u, ufx_valid };   /* (REC_SET's pair: BOARD 3 has no settings) */
static const pp_desc_t udcal_pp = { 256u, UC_REC_BYTES, 4u, 4u, 0u, udcal_valid };
static uint32_t ucal_seq;
#else
static const pp_desc_t set_pp = { 256u, STORE_SLOT_BYTES, STORE_SLOTS, 4u, 0u, store_valid };
#endif
#if BOARD != BOARD_UR_DIGITAL_M
static const pp_desc_t cal_pp = { 256u, CAL_REC_BYTES, CAL_RECS, 4u, 0u, cal_valid };
#if MON_EXTRA
static const pp_desc_t hist_pp = { 256u, CALHIST_REC_BYTES, CALHIST_RECS, 4u, 0u, calhist_valid };
#endif
#endif

static const rec_t recs[REC_COUNT] = {
    /* the lifetime record (core/energy.h): its A/B pair */
    [REC_LIFE] = { &life_pp, { LIFE_PAGE0_ADDR, LIFE_PAGE1_ADDR } },
    /* settings: an A/B pair (one page erased when full would lose the stored settings to a power cut right after
     * that erase), 8 records of 32 bytes per page */
    [REC_SET] = { &set_pp, { SETTINGS_ADDR, SETTINGS_B_ADDR } },
#if BOARD == BOARD_UR_DIGITAL_M
    [REC_UDCAL] = { &udcal_pp, { CAL_PAGE_ADDR, CAL_B_ADDR } },
#endif
#if BOARD != BOARD_UR_DIGITAL_M
    /* current calibration: an A/B pair, 4 records of 64 bytes per page */
    [REC_CAL] = { &cal_pp, { CAL_PAGE_ADDR, CAL_B_ADDR } },
    /* power-on scene log: an A/B pair (core/scene.h scene_pp),
     * 16 records of 16 bytes per page */
    [REC_SCENE] = { &scene_pp, { SCENE_LOG_ADDR, SCENE_B_ADDR } },
#if MON_EXTRA
    /* the calibration history (core/calhist.h, M2): an A/B pair, 4 records of 64 bytes per page */
    [REC_CALHIST] = { &hist_pp, { CALHIST_ADDR, CALHIST_B_ADDR } },
#endif
#endif
};
static pp_pos_t rec_pos[REC_COUNT];
#if BOARD != BOARD_UR_DIGITAL_M
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
 * how it went (pagepair.h pp_done; a failed write never counts as the newest record) */
static uint32_t rec_program(uint32_t i, uint32_t at, const void *src)
{
    uint32_t r = flash_op(rec_addr(i, at), (const uint16_t *)src, recs[i].pp->bytes / 2u, 0u);
    pp_done(&rec_pos[i], at, r == FLASH_OK);
    return r;
}

#if BOARD == BOARD_UR_DIGITAL_M
uint32_t ucal_load(void *c)
{
    const uint8_t *r=rec_find(REC_UDCAL);
    if(!r) return 0;
    ucal_seq=(uint32_t)r[4]|(uint32_t)r[5]<<8|(uint32_t)r[6]<<16|(uint32_t)r[7]<<24;
    udcal_load((udcal_t *)c,r);
    return 1;
}
uint32_t ucal_store(const void *c)
{
    uint32_t r,at,buf[UC_REC_BYTES/4u];
    at=rec_write_at(REC_UDCAL,&r);
    if(at==PP_NONE) return r;
    udcal_pack((const udcal_t *)c,ucal_seq+1u,(uint8_t *)buf);
    r=rec_program(REC_UDCAL,at,buf);
    if(r==FLASH_OK) ucal_seq++;
    return r;
}
/* ---- the local-effects record (core/ur_digital_m/udfx.h): two 128-byte records a page, the CRC last */
MEASURED uint32_t ufx_load(uint8_t rec[128])
{
    const uint8_t *r = rec_find(REC_SET);
    if (!r) {
        return 0;
    }
    memcpy(rec, r, UX_REC_BYTES);
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

#if BOARD != BOARD_UR_DIGITAL_M
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

#if MON_EXTRA && BOARD != BOARD_UR_DIGITAL_M
/* ---- the calibration history (M2): programmed from *h itself, as the lifetime record */
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

uint32_t hw_flash_half(uint32_t addr, uint16_t v)
{
    uint32_t r = flash_op(addr, &v, 1u, 0u);
    /* Update status uses the physical flash address, after BSY clears. */
    return r == FLASH_OK && *(volatile const uint16_t *)(uintptr_t)addr != v ? FLASH_ERR_VERIFY : r;
}

uint32_t flash_rejected(void)
{
#if BOARD == BOARD_UR_DIGITAL_M
    return rec_pos[REC_SET].bad | (uint32_t)rec_pos[REC_LIFE].bad << 18;
#else
    return rec_pos[REC_SET].bad | (uint32_t)rec_pos[REC_CAL].bad << 6 | (uint32_t)rec_pos[REC_SCENE].bad << 12 |
           (uint32_t)rec_pos[REC_LIFE].bad << 18;
#endif
}
