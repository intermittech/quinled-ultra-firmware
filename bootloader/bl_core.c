/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader logic: boot decision, update session, erase plan, commit, verify (see bl_core.h). */
#include "bl_core.h"

static void zero(void *p, uint32_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) {
        *b++ = 0;
    }
}

static void copy(void *d, const void *s, uint32_t n)
{
    uint8_t *a = (uint8_t *)d;
    const uint8_t *b = (const uint8_t *)s;
    while (n--) {
        *a++ = *b++;
    }
}

static uint32_t half_at(uint32_t addr)
{
    const uint8_t *p = bl_plat_ptr(addr);
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8);          /* flash is read little-endian, like the CPU */
}

/* ------------------------------------------------------------------------------------------------ descriptor */
/* crc = upd_crc32_words over the first 60 bytes (computed by the caller: CRC unit on flash, software on RAM) */
static int desc_check_crc(const uint8_t *d, uint32_t crc)
{
    return upd_desc_check(d, crc, BL_HW_ID, BL_HW_REV, BL_VERSION);   /* the app checks with the same rules */
}

int bl_desc_check(const uint8_t *d)
{
    return desc_check_crc(d, upd_crc32_words(0xFFFFFFFFu, d, 15u));
}

void bl_app_read(bl_app_t *a)
{
    const uint8_t *d = bl_plat_ptr(UPD_APP_DESC);
    uint32_t i;
    zero(a, sizeof(*a));
    a->st_next = (uint8_t)UPD_ST_COUNT;
    if (desc_check_crc(d, bl_plat_crc32(UPD_APP_DESC, 15u)) != 0) {
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
    if (bl_plat_program_half(UPD_APP_DESC + 2u * (UPD_ST_FIRST + a->st_next), v) != 0) {
        return -1;
    }
    a->st_next++;
    return 0;
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
    ob[4] &= 0xFCu;
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

/* ------------------------------------------------------------------------------------------------ erase plan */
uint32_t bl_erase_plan(uint32_t body_len, uint32_t *ops, uint32_t max)
{
    uint32_t a = UPD_APP_DESC, end = UPD_APP_BODY + body_len, n = 0;
    end = (end + UPD_PAGE_BYTES - 1u) & ~(UPD_PAGE_BYTES - 1u);
    if (end > UPD_APP_END) {
        end = UPD_APP_END;
    }
    while (a < end && n < max) {
        if ((a & (UPD_SECTOR_BYTES - 1u)) == 0u && a + UPD_SECTOR_BYTES <= UPD_APP_END) {
            ops[n++] = a;                                     /* whole sector inside the app area (desc first) */
            a += UPD_SECTOR_BYTES;
        } else {
            ops[n++] = a | 1u;                                /* page: the sector is shared with the scene log */
            a += UPD_PAGE_BYTES;
        }
    }
    return n;
}

/* ------------------------------------------------------------------------------------------------ session */
static void leave(bl_sess_t *s, uint32_t request, uint32_t result, uint32_t version)
{
    s->exit_request = request;
    s->exit_result = result;
    s->exit_version = version;
    s->state = BL_S_LISTEN;
    s->mode = BL_ACT_JUMP;                                    /* marks "leave" for bl_sess_poll */
}

static void to_recovery(bl_sess_t *s)
{
    s->mode = BL_ACT_RECOVERY;
    s->state = BL_S_LISTEN;
}

static void pool_clear(bl_sess_t *s)
{
    uint32_t i;
    for (i = 0; i < BL_POOL_PAGES; i++) {
        s->pool[i].used = BL_POOL_FREE;
    }
}

void bl_sess_init(bl_sess_t *s, bl_action_t mode, const bl_app_t *app, uint32_t now_ms)
{
    zero(s, (uint32_t)((const uint8_t *)&s->pool[0] - (const uint8_t *)s));   /* the pool data is not cleared */
    pool_clear(s);
    s->mode = mode;
    s->state = BL_S_LISTEN;
    s->app = *app;
    s->t_start = now_ms;
    s->t_last = now_ms;
    s->gate = BL_GATE_OPEN;
}

static uint32_t chunks_in_page(const bl_sess_t *s, uint32_t page)
{
    uint32_t rest = upd_get32(s->desc + UPD_D_BODY_LEN) - page * UPD_PAGE_BYTES;
    uint32_t n = (rest + UPD_CHUNK_BYTES - 1u) / UPD_CHUNK_BYTES;
    return n > UPD_CHUNKS_PER_PAGE ? UPD_CHUNKS_PER_PAGE : n;
}

static void accept(bl_sess_t *s)
{
    uint32_t ops[24], n, i, len = upd_get32(s->pending + UPD_D_BODY_LEN);
    if (!bl_plat_power_ok()) {
        return;                                               /* try again on the next ANNOUNCE */
    }
    /* Revoke the magic before an erase can touch the body in the SAME sector. A partial erase may leave the
     * descriptor and its CONFIRMED record intact. Zero cannot erase back to 'QD' (0x4451) by flipping bits
     * towards the chip's 0xE339 erased readout (0x4451 needs bits absent from both zero and 0xE339).
     * A failed/torn revoke never reaches an erase; its old body is still intact. */
    s->app.desc_ok = 0;
    if (bl_plat_program_half(UPD_APP_DESC, 0) != 0) {
        s->flash_errors++;
        to_recovery(s);
        return;
    }
    n = bl_erase_plan(len, ops, 24u);                         /* full slot: 13 sectors + 7 pages */
    for (i = 0; i < n; i++) {
        int rc;
        bl_plat_kick();
        rc = (ops[i] & 1u) ? bl_plat_erase_page(ops[i] & ~1u) : bl_plat_erase_sector(ops[i]);
        if (rc != 0) {
            s->flash_errors++;
            to_recovery(s);
            return;
        }
    }
    copy(s->desc, s->pending, UPD_DESC_BYTES);
    s->session = s->pending_session;
    s->npages = (len + UPD_PAGE_BYTES - 1u) / UPD_PAGE_BYTES;
    zero(s->written, sizeof(s->written));
    s->nwritten = 0;
    pool_clear(s);
    s->state = BL_S_RECEIVE;
}

static int same_desc(const uint8_t *a, const uint8_t *b)
{
    uint32_t i;
    for (i = 0; i < UPD_DESC_BYTES; i++) {
        if (a[i] != b[i]) {
            return 0;
        }
    }
    return 1;
}

/* both ANNOUNCE halves of `pending` are in */
static void evaluate(bl_sess_t *s)
{
    int rc = bl_desc_check(s->pending);
    uint32_t v = upd_get32(s->pending + UPD_D_FW_VERSION);
    s->have_half = 0;
    if (rc == -1) {
        return;                                               /* halves of two different images: wait */
    }
    if (s->state == BL_S_RECEIVE && same_desc(s->pending, s->desc)) {
        s->session = s->pending_session;                      /* the host restarted the same image: keep pages */
        return;
    }
    /* the session id changes only with an accept (accept() sets it after the erase) or a skip: if accept() can't
     * erase now (VDD low), a running session keeps its id and the new session's chunks are not mixed into it */
    if (rc != 0) {
        if (s->state == BL_S_RECEIVE) {
            return;                                           /* an image for other hardware while ours streams in:
                                                                 not ours to answer, no reason to drop the install */
        }
        s->skip_result = UPD_RES_REJECTED;
    } else if (s->app.desc_ok && v == s->app.version && !s->force) {
        s->skip_result = UPD_RES_SAME;
    } else if (s->app.desc_ok && v < s->app.version && !s->force) {
        s->skip_result = UPD_RES_REJECTED;
    } else {
        accept(s);
        return;
    }
    copy(s->desc, s->pending, UPD_DESC_BYTES);                /* for the result's version */
    s->session = s->pending_session;
    s->state = BL_S_SKIP;
}

static uint32_t popcount8(uint32_t m)
{
    uint32_t n = 0;
    while (m) {
        n += m & 1u;
        m >>= 1;
    }
    return n;
}

/* the entry of `page`, else a free one, else the least complete partial page is dropped (it comes again in the
 * next pass): a full pool must never block the page that is streaming now */
static bl_pool_t *pool_get(bl_sess_t *s, uint32_t page)
{
    uint32_t i, best_n = 9u;
    bl_pool_t *free_e = 0, *victim = 0;
    for (i = 0; i < BL_POOL_PAGES; i++) {
        bl_pool_t *e = &s->pool[i];
        if (e->used != BL_POOL_FREE && e->page == page) {
            return e;
        }
        if (e->used == BL_POOL_FREE) {
            if (free_e == 0) {
                free_e = e;
            }
        } else if (e->used == BL_POOL_PARTIAL && popcount8(e->mask) < best_n) {
            best_n = popcount8(e->mask);
            victim = e;
        }
    }
    if (free_e == 0) {
        free_e = victim;                                      /* 0: the pool is all complete pages */
        s->evicted += (victim != 0);
    }
    if (free_e != 0) {
        free_e->used = BL_POOL_PARTIAL;
        free_e->page = (uint16_t)page;
        free_e->mask = 0;
    }
    return free_e;
}

/* programs up to `max` complete pages (2 ms each); 1 = all done */
static int program_pending(bl_sess_t *s, uint32_t max)
{
    uint32_t i;
    for (i = 0; i < BL_POOL_PAGES; i++) {
        bl_pool_t *e = &s->pool[i];
        uint32_t page = e->page;
        if (e->used != BL_POOL_COMPLETE) {
            continue;
        }
        if (max == 0u || !bl_plat_power_ok()) {
            return 0;                                         /* more later; VDD low: kept, retried later */
        }
        bl_plat_kick();
        if (bl_plat_program_page(UPD_APP_BODY + page * UPD_PAGE_BYTES, e->data) != 0) {
            s->flash_errors++;                                /* a page can't be programmed twice: give up */
            to_recovery(s);
            return 0;
        }
        s->written[page >> 3] = (uint8_t)(s->written[page >> 3] | (1u << (page & 7u)));
        s->nwritten++;
        s->pages_programmed++;
        e->used = BL_POOL_FREE;
        max--;
    }
    return 1;
}

static void put_chunk(bl_sess_t *s, uint32_t idx, const uint8_t *data)
{
    uint32_t page = idx / UPD_CHUNKS_PER_PAGE, c = idx % UPD_CHUNKS_PER_PAGE, need, i;
    bl_pool_t *e;
    if (page >= s->npages) {
        return;
    }
    need = chunks_in_page(s, page);
    if (c >= need) {
        return;
    }
    if (((s->written[page >> 3] >> (page & 7u)) & 1u) == 0u && (e = pool_get(s, page)) != 0 &&
        e->used == BL_POOL_PARTIAL) {
        copy(e->data + c * UPD_CHUNK_BYTES, data, UPD_CHUNK_BYTES);
        e->mask = (uint8_t)(e->mask | (1u << c));
        if (e->mask == (1u << need) - 1u) {
            for (i = need * UPD_CHUNK_BYTES; i < UPD_PAGE_BYTES; i++) {
                e->data[i] = 0xFFu;                           /* past the end of the body */
            }
            e->used = BL_POOL_COMPLETE;
        }
    }
    if (c == need - 1u) {
        /* the last chunk of a page span: the host pauses now (UPD_PAGE_PAUSE_US), time for one page program. Chunks
         * come in page order, so a span completes at most one page; a page whose last chunk got lost waits for the
         * next pause (every span triggers, also of pages already written) or the END. */
        (void)program_pending(s, 1u);
    }
}

/* END, ABORT or the idle timeout */
static void finish(bl_sess_t *s, int abort)
{
    uint32_t v = upd_get32(s->desc + UPD_D_FW_VERSION), i;
    if (s->state == BL_S_SKIP) {
        if (s->app.desc_ok) {
            leave(s, s->app.verified ? UPD_REQ_BOOT_ONCE : UPD_REQ_NONE,
                  abort ? UPD_RES_NONE : s->skip_result, v);
        } else {
            to_recovery(s);
        }
        return;
    }
    if (s->state != BL_S_RECEIVE) {
        return;
    }
    if (!abort) {
        (void)program_pending(s, BL_POOL_PAGES);              /* pages still waiting for a quiet moment */
    }
    if (s->state != BL_S_RECEIVE) {
        return;                                               /* a flash error */
    }
    if (abort || s->nwritten != s->npages) {
        to_recovery(s);                                       /* the app area is partly written: no app */
        return;
    }
    if (!bl_plat_power_ok()) {
        return;                                               /* commit later (next END or the timeout) */
    }
    for (i = 0; i < UPD_DESC_BYTES / 2u; i++) {               /* the descriptor last = the commit */
        bl_plat_kick();
        if (bl_plat_program_half(UPD_APP_DESC + 2u * i, (uint16_t)(s->desc[2u * i] | (s->desc[2u * i + 1u] << 8))) != 0) {
            s->flash_errors++;
            to_recovery(s);
            return;
        }
    }
    leave(s, UPD_REQ_NONE, UPD_RES_UPDATED, v);               /* next boot: VERIFY (CRC after a reset) */
}

static void on_block(bl_sess_t *s, const uint8_t *f)
{
    uint32_t type = f[UPD_B_TYPE], session = f[UPD_B_SESSION], idx = upd_get16(f + UPD_B_INDEX);
    const uint8_t *data = f + UPD_B_DATA;
    switch (type) {
    case UPD_ANNOUNCE:
        if (idx > 1u || (s->state != BL_S_LISTEN && session == s->session)) {
            return;                                           /* repeated ANNOUNCE of the running session */
        }
        if (session != s->pending_session || s->have_half == 0u) {
            s->pending_session = (uint8_t)session;
            s->have_half = 0;
            s->force = 0;
        }
        copy(s->pending + idx * UPD_CHUNK_BYTES, data, UPD_CHUNK_BYTES);
        s->have_half = (uint8_t)(s->have_half | (1u << idx));
        s->force = (uint8_t)(s->force | ((f[UPD_B_FLAGS] & UPD_F_FORCE) != 0u));
        if (s->have_half == 3u) {
            evaluate(s);
        }
        break;
    case UPD_DATA:
        if (s->state == BL_S_RECEIVE && session == s->session) {
            put_chunk(s, idx, data);
        }
        break;
    case UPD_END:
    case UPD_ABORT:
        if (s->state != BL_S_LISTEN && session == s->session) {
            if (type == UPD_END && s->state == BL_S_RECEIVE &&
                (upd_get32(data) != upd_get32(s->desc + UPD_D_BODY_CRC) ||
                 upd_get32(data + 4) != upd_get32(s->desc + UPD_D_BODY_LEN))) {
                return;                                       /* an END of another image: ignore */
            }
            finish(s, type == UPD_ABORT);
        }
        break;
    default:
        break;                                                /* PING and unknown types: nothing to do here */
    }
}

static int normal_ultra(const uint8_t *h)
{
    return upd_get16(h) == UPD_HDR_MAGIC && (h[2] >> 4) == UPD_HDR_VERSION && (h[2] & UPD_HDR_FLAG) == 0u &&
           upd_crc8(h, 7) == h[7];
}

void bl_sess_frame(bl_sess_t *s, uint32_t nbits, const uint8_t *tail, uint32_t now_ms)
{
    s->frames++;
    if (nbits == UPD_FRAME_BITS && upd_check_frame(tail) == 0) {
        s->upd_frames++;
        s->t_last = now_ms;
        s->gate = BL_GATE_OPEN;                               /* a session: everyone forwards everything */
        on_block(s, tail);
        return;
    }
    if (nbits >= 64u && normal_ultra(tail + UPD_FRAME_BYTES - UPD_HDR_BYTES)) {
        s->seen_ultra = 1;
        /* a board stuck in recovery takes its 256 bits like an app would, so the boards behind it keep working */
        s->gate = (s->mode == BL_ACT_RECOVERY && s->state == BL_S_LISTEN) ? BL_GATE_ULTRA : BL_GATE_OPEN;
        return;
    }
    if (nbits == UPD_FRAME_BITS) {
        s->bad_frames++;
    }
    s->gate = BL_GATE_OPEN;
}

int bl_sess_poll(bl_sess_t *s, uint32_t now_ms)
{
    if (s->mode == BL_ACT_JUMP) {
        return 1;                                             /* leave() was called */
    }
    switch (s->state) {
    case BL_S_LISTEN:
        if (s->mode == BL_ACT_TRIAL && (s->seen_ultra || now_ms - s->t_start >= BL_TRIAL_MS)) {
            (void)st_append(&s->app, s->seen_ultra ? UPD_ST_TRIAL_SEEN : UPD_ST_TRIAL_QUIET, BL_ST_RESERVE);
            leave(s, UPD_REQ_BOOT_ONCE, UPD_RES_NONE, 0);
        } else if (s->mode == BL_ACT_SESSION && now_ms - s->t_last >= BL_WAIT_MS) {
            if (s->app.desc_ok && s->app.verified) {
                leave(s, UPD_REQ_BOOT_ONCE, UPD_RES_NONE, 0);
            } else if (s->app.desc_ok) {
                leave(s, UPD_REQ_NONE, UPD_RES_NONE, 0);      /* reset through VERIFY (or recovery if invalid) */
            } else {
                to_recovery(s);
            }
        } else if (s->mode == BL_ACT_RECOVERY && s->app.desc_ok && s->app.verified && s->app.confirmed &&
                   now_ms - s->t_start >= BL_RECOVERY_MS) {
            leave(s, UPD_REQ_BOOT_ONCE, UPD_RES_NONE, 0);
        }
        break;
    case BL_S_RECEIVE:
    case BL_S_SKIP:
        if (s->state != BL_S_LISTEN && now_ms - s->t_last >= BL_IDLE_MS) {
            finish(s, 0);
            s->t_last = now_ms;                               /* a deferred commit (VDD low) retries in 3 s */
        }
        break;
    }
    return s->mode == BL_ACT_JUMP;
}

void bl_ho_exit(upd_handover_t *ho, const bl_sess_t *s)
{
    ho->request = s->exit_request;
    if (s->exit_result != UPD_RES_NONE) {
        ho->result = s->exit_result;
        ho->result_version = s->exit_version;
    }
    if (s->exit_result == UPD_RES_UPDATED ||
        (s->exit_request == UPD_REQ_BOOT_ONCE && s->app.desc_ok && s->app.verified && s->app.confirmed)) {
        ho->wd_resets = 0;
    }
    bl_ho_seal(ho);
}

int bl_sess_blink(const bl_sess_t *s, uint32_t now_ms)
{
    return s->mode == BL_ACT_RECOVERY && s->state == BL_S_LISTEN && ((now_ms / 500u) & 1u) == 0u;
}
