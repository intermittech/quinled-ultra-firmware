/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* ABI-3 declared-profile receiver. No persistent resume and no A/B rollback.
 * Only validated metadata committed after full body verification is bootable. */
#include "bl_core.h"
#include "boot_api.h"
#include <string.h>

static int equal(const uint8_t *a, const uint8_t *b, uint32_t n)
{
    while (n--) if (*a++ != *b++) return 0;
    return 1;
}

static void pool_clear(bl_sess_t *s)
{
    uint32_t i;
    for (i = 0; i < BL_POOL_PAGES; ++i) s->pool[i].used = BL_POOL_FREE;
}

static void leave(bl_sess_t *s, uint32_t request, uint32_t result)
{
    s->exit_request = request;
    s->exit_result = result;
    s->exit_version = upd_get32(s->desc + UPD_D_FW_VERSION);
    s->mode = BL_ACT_JUMP;
}

static void fail(bl_sess_t *s)
{
#if UPD_STATUS_PROFILE
    /* Shared ANNOUNCE failures must cancel status work too. Keep a more specific
     * error supplied by st_fail; shared repeat failures are conflicts. */
    if (!s->st.error) s->st.error = UPD_ERROR_CONFLICT;
    s->st.op = 0;
    s->st.op_due = 0;
    s->st.reply = 0;
#endif
    s->state = BL_S_FAILED;
    s->have_half = 0;
    pool_clear(s);
    if (!s->app.desc_ok) s->mode = BL_ACT_RECOVERY;
}

void bl_sess_init(bl_sess_t *s, bl_action_t mode, const bl_app_t *app, uint32_t now)
{
    memset(s, 0, (uint32_t)((uint8_t *)s->pool - (uint8_t *)s));
    pool_clear(s);
    s->mode = mode;
    s->app = *app;
    s->t_start = s->t_last = now;
#if UPD_STATUS_PROFILE
    /* No persistent journal: even a reset before revocation lost its session. */
    s->st.state = UPD_STATE_RESTART_REQUIRED;
#endif
}

static uint32_t page_bytes(const bl_sess_t *s, uint32_t page)
{
    uint32_t n = upd_get32(s->desc + UPD_D_BODY_LEN) - page * 256u;
    return n < 256u ? n : 256u;
}

#if UPD_STATUS_PROFILE
static void st_evaluate(bl_sess_t *s);

/* docs/update.md: inactivity excludes the whole declared erase lead, from the final boot ANNOUNCE: the image's erase
 * units (metadata sector + body) x 2 x erase_max + 100 ms. The real erase takes a fraction of it (256 KiB: 2.0 of
 * 13.1 s; full slot: 3.7 of 24.9 s). Layout 1's largest lead (9.06 s) stays below the idle timeout. */
static void lead_start(bl_sess_t *s)
{
    const uint32_t units = (UPD_APP_BODY - UPD_APP_DESC) / UPD_ERASE_BYTES +
                           (upd_get32(s->desc + UPD_D_BODY_LEN) + UPD_ERASE_BYTES - 1u) / UPD_ERASE_BYTES;
    s->lead_end = bl_plat_ms() + units * 2u * MCU_UPD_ERASE_MS + 100u;
    s->lead = 1;
}
#endif
static void evaluate(bl_sess_t *s)
{
    uint32_t len = upd_get32(s->pending + UPD_D_BODY_LEN);
    uint32_t version = upd_get32(s->pending + UPD_D_FW_VERSION);
#if UPD_STATUS_PROFILE
    if (s->st.selected == 1u) {
        st_evaluate(s);
        return;
    }
#endif
    s->session = s->pending_session;
    s->have_half = 0;
    if (bl_desc_check(s->pending) ||
        ((len + 255u) & ~255u) > UPD_APP_BODY_MAX ||
        !bl_plat_layout_ok() || !bl_plat_power_ok()) {
        fail(s);
        return;
    }
    memcpy(s->desc, s->pending, 64);
#if UPD_STATUS_PROFILE
    lead_start(s);
#endif
    if (s->app.desc_ok && version <= s->app.version && !s->force) {
        s->skip_result = version == s->app.version ? UPD_RES_SAME : UPD_RES_REJECTED;
        s->state = BL_S_SKIP;
        return;
    }
    /* Layout 4 revokes by erasing and verifying its dedicated metadata sector.
     * Its flash must never be programmed over an existing descriptor. */
#if !UPD_STATUS_PROFILE
    /* A torn revoke never permits a body erase. The old body is still intact. */
    bl_plat_begin(2u * MCU_UPD_PROGRAM_MS);
    if (bl_plat_program_half(UPD_APP_DESC, 0) ||
        bl_plat_ptr(UPD_APP_DESC)[0] || bl_plat_ptr(UPD_APP_DESC)[1]) {
        s->app.desc_ok = 0;
        fail(s);
        return;
    }
    s->app.desc_ok = 0;
#endif
    s->npages = (len + 255u) / 256u;
    s->erase_cursor = UPD_APP_DESC;
#if UPD_ERASE_BYTES == 256u
    s->erase_end = UPD_APP_BODY + s->npages * 256u;
#else
    s->erase_end = (UPD_APP_BODY + s->npages * 256u + UPD_ERASE_BYTES - 1u) & ~(UPD_ERASE_BYTES - 1u);
#endif
    s->nwritten = 0;
    memset(s->written, 0, sizeof s->written);
    pool_clear(s);
    s->state = BL_S_ERASE;
    s->t_last = bl_plat_ms();
}

/* One minimum erase unit per poll; no truncated plan or record-region overlap. */
static void erase_next(bl_sess_t *s)
{
    uint32_t i, a = s->erase_cursor;
    const uint8_t *p = bl_plat_ptr(a);
    bl_plat_begin(2u * MCU_UPD_ERASE_MS);
#if UPD_STATUS_PROFILE
    /* Even a torn metadata erase makes the old app unsafe. Body erases follow
     * only after this entire sector has read back as erased. */
    if (a == UPD_APP_DESC) s->app.desc_ok = 0;
#endif
    if (!bl_plat_power_ok() || bl_plat_erase_page(a)) goto error;
    for (i = 0; i < UPD_ERASE_BYTES; i += 2u)
        if (!upd_erased16(p[i] | ((uint32_t)p[i+1] << 8))) goto error;
    s->erase_cursor += UPD_ERASE_BYTES;
    if (s->erase_cursor == s->erase_end) s->state = BL_S_RECEIVE;
    s->t_last = bl_plat_ms();
    return;
error:
    ++s->flash_errors;
    fail(s);
}

/* Bounded multipass pool: retain partial pages; an overflowing pool can evict
 * a partial page, which a later complete pass supplies again. */
static bl_pool_t *pool_get(bl_sess_t *s, uint32_t page)
{
    uint32_t i;
    bl_pool_t *free_entry = 0, *partial = 0;
    for (i = 0; i < BL_POOL_PAGES; ++i) {
        bl_pool_t *e = &s->pool[i];
        if (e->used && e->page == page) return e;
        if (!e->used) free_entry = e;
        else if (e->used == BL_POOL_PARTIAL) partial = e;
    }
    if (!free_entry) { free_entry = partial; s->evicted += partial != 0; }
    if (free_entry) {
        free_entry->page = (uint16_t)page;
        free_entry->mask = 0;
        free_entry->used = BL_POOL_PARTIAL;
#if UPD_STATUS_PROFILE
        for (i = 0; i < 256u; ++i) free_entry->data[i] = (uint8_t)(i & 1u ? 0xe3u : 0x39u);
#else
        memset(free_entry->data, 0xff, 256);
#endif
    }
    return free_entry;
}

/* Exactly one page at a declared block pause, including pauses in later passes. */
static void program_pending(bl_sess_t *s)
{
    uint32_t i;
    for (i = 0; i < BL_POOL_PAGES; ++i) {
        bl_pool_t *e = &s->pool[i];
        uint32_t address = UPD_APP_BODY + e->page * 256u;
        if (e->used != BL_POOL_COMPLETE) continue;
        bl_plat_begin(2u * MCU_UPD_PROGRAM_MS);
        if (!bl_plat_power_ok() || bl_plat_program_page(address, e->data) ||
            !equal(bl_plat_ptr(address), e->data, page_bytes(s, e->page))) {
            ++s->flash_errors;
            fail(s);
            return;
        }
        s->written[e->page >> 3] |= (uint8_t)(1u << (e->page & 7u));
        ++s->nwritten;
        ++s->pages_programmed;
        e->used = BL_POOL_FREE;
        s->t_last = bl_plat_ms();
        return;
    }
}

static int put_chunk(bl_sess_t *s, uint32_t index, const uint8_t *data)
{
    uint32_t off = index * 32u, len = upd_get32(s->desc + UPD_D_BODY_LEN);
    uint32_t page = index / 8u, c = index % 8u, n, i, need;
    bl_pool_t *e;
    if (off >= len) return 0;
    n = len - off;
    if (n > 32u) n = 32u;
    for (i = n; i < 32u; ++i) if (data[i]) return 0;
    need = (page_bytes(s, page) + 31u) / 32u;
    if (s->written[page >> 3] & (1u << (page & 7u))) {
        if (!equal(bl_plat_ptr(UPD_APP_BODY + off), data, n)) { fail(s); return 1; }
    } else if ((e = pool_get(s, page)) != 0) {
        if ((e->mask & (1u << c)) && !equal(e->data + c * 32u, data, n)) {
            fail(s);
            return 1;
        }
        memcpy(e->data + c * 32u, data, n);
        e->mask |= (uint8_t)(1u << c);
        if (e->mask == (1u << need) - 1u) e->used = BL_POOL_COMPLETE;
    }
    if (c == need - 1u) program_pending(s);
    return 1;
}

static void finish(bl_sess_t *s)
{
    uint32_t i, length = upd_get32(s->desc + UPD_D_BODY_LEN);
    if (s->state == BL_S_SKIP) {
        leave(s, s->app.verified ? UPD_REQ_BOOT_ONCE : UPD_REQ_NONE, s->skip_result);
        return;
    }
    if (s->state != BL_S_RECEIVE || s->nwritten != s->npages) return;
    if (bl_plat_crc32(UPD_APP_BODY, length / 4u) != upd_get32(s->desc + UPD_D_BODY_CRC)) {
        fail(s);
        return;
    }
    /* One shared deadline covers ALL metadata writes and readback. Magic last;
     * a cut before it leaves recovery, after it leaves a complete verified body. */
    bl_plat_begin(2u * MCU_UPD_PROGRAM_MS);
    if (!bl_plat_power_ok()) { fail(s); return; }
    for (i = 2; i < 64u; i += 2u)
        if (bl_plat_program_half(UPD_APP_DESC + i,
                                (uint16_t)(s->desc[i] | ((uint16_t)s->desc[i+1] << 8)))) goto error;
    if (!equal(bl_plat_ptr(UPD_APP_DESC + 2u), s->desc + 2u, 62)) goto error;
    if (bl_plat_program_half(UPD_APP_DESC + 64u, UPD_ST_VERIFIED) ||
        bl_plat_ptr(UPD_APP_DESC + 64u)[0] != (UPD_ST_VERIFIED & 255u) ||
        bl_plat_ptr(UPD_APP_DESC + 64u)[1] != (UPD_ST_VERIFIED >> 8) ||
        bl_plat_program_half(UPD_APP_DESC, 0x4451u) ||
        !equal(bl_plat_ptr(UPD_APP_DESC), s->desc, 64)) goto error;
    leave(s, UPD_REQ_BOOT_ONCE, UPD_RES_UPDATED);
    return;
error:
    ++s->flash_errors;
    fail(s);
}

#if UPD_STATUS_PROFILE
/* ------------------------------------------------------------------------------------------------ status profile
 * docs/update.md "Status profile": only the selected bootloader (UPD_SELECT: this UID and its immutable target, in
 * that session) acts on the session's frames and answers its polls, on the answer UART at the host-locked rate
 * (bl_main.c: hw_reply). Flash work is poll-driven: a poll with an operation pending is answered BUSY with the
 * operation's whole deadline, and the operation starts once that answer's window has closed (marker port: frame end
 * + 900 + pitch, + 80); one erase unit (the first, the metadata sector, is the revoke), one 256-byte block, or the
 * body verification with the metadata commit per BUSY. One 256-byte staging block. COMMITTED holds until a PING of
 * the session acknowledges it, then the reboot. */
enum { ST_OP_NONE = 0, ST_OP_ERASE, ST_OP_PROGRAM, ST_OP_COMMIT };
static const uint16_t st_op_ms[4] = { 0u, 2u * MCU_UPD_ERASE_MS, 2u * MCU_UPD_PROGRAM_MS,
                                      100u + 2u * MCU_UPD_PROGRAM_MS };
#define ST_REPLY_US 450u               /* the answer's enable after the poll's end: released >= 20 us, <= 900 us */

static void st_fail(bl_sess_t *s, uint8_t e)
{
    s->st.error = e;
    s->st.op = ST_OP_NONE;
    s->st.op_due = 0;
    fail(s);
}

static uint32_t st_absdiff(uint32_t a, uint32_t b)
{
    return a > b ? a - b : b - a;
}

/* host us -> local ticks at the locked rate */
static uint32_t st_ticks(const bl_sess_t *s, uint32_t us)
{
    return (uint32_t)((uint64_t)us * s->st.span_ticks / s->st.span_host);
}

/* every valid UPD frame: its trailer TIME (host us at its end) against the local ticks of its last falling edge.
 * Locked once the observations span >= 1 s (docs/timing.md: +-200 ppm, >= 1000 ms); a gap over 2 s, a rate outside
 * the clock's range or a deviation over 200 ppm (+ 5 us of edge error) from the locked rate restarts it. The
 * reference moves on every 4 s, the lock staying. */
static void st_observe(bl_sess_t *s, const uint8_t *f)
{
    const uint32_t th = upd_get32(f + 43), tl = s->st.frame_t0;
    if (s->st.have_ref && th != s->st.t_host && th - s->st.t_host <= 2000000u) {
        const uint32_t dh = th - s->st.ref_host, dl = tl - s->st.ref_ticks;
        if (dh < 8000000u && dl / 206u <= dh && dl / 185u >= dh &&
            (!s->st.locked || st_absdiff(dl, st_ticks(s, dh)) <= dl / 5000u + 1000u)) {
            if (dh >= 1000000u) {
                s->st.span_host = dh;
                s->st.span_ticks = dl;
                s->st.locked = 1;
                if (dh >= 4000000u) {
                    s->st.ref_host = th;
                    s->st.ref_ticks = tl;
                }
            }
            s->st.t_host = th;
            s->st.t_ticks = tl;
            return;
        }
    }
    s->st.have_ref = 1;
    s->st.locked = 0;
    s->st.ref_host = s->st.t_host = th;
    s->st.ref_ticks = s->st.t_ticks = tl;
}

/* chunks in the block at `committed` */
static uint32_t st_need(const bl_sess_t *s)
{
    uint32_t left = upd_get32(s->desc + UPD_D_BODY_LEN) - s->st.committed;
    return left >= 256u ? 8u : (left + 31u) / 32u;
}

/* the UPD_STATUS answer: PAGE 0x86, SEQ = the session, UID16, the 24-byte status (ultra_update.h UPD_S_*) */
static void st_answer(bl_sess_t *s)
{
    uint8_t *a = s->st.answer, *v = a + ULTRA_RSP_DATA;
    const uint32_t len = upd_get32(s->desc + UPD_D_BODY_LEN);
    uint32_t state = s->st.state, credit = 0, prefix = 0, retry = 0, i;
    if (s->state == BL_S_FAILED) state = UPD_STATE_FAILED;
    else if (s->st.op) { state = UPD_STATE_BUSY; retry = st_op_ms[s->st.op]; }
    else if (s->st.state == UPD_STATE_COMMITTED) state = UPD_STATE_COMMITTED;
    else if (s->state == BL_S_RECEIVE) {
        state = s->st.mask ? UPD_STATE_RECEIVE : UPD_STATE_READY;
        if (s->st.committed < len) {
            credit = st_need(s);
            for (i = 0; i < 8u; ++i) credit -= (s->st.mask >> i) & 1u;
        }
    }
    for (i = 0; i < 8u && (s->st.mask & (1u << i)); ++i) prefix += 32u;
    if (prefix > len - s->st.committed) prefix = len - s->st.committed;
    memset(a, 0, 32);
    a[0] = ULTRA_RSP_SYNC;
    a[1] = ULTRA_PAGE_UPD_STATUS;
    a[3] = s->st.sel_session;
    ultra_put16(a + 4, ultra_crc16(bl_plat_uid(), 12));
    v[UPD_S_VERSION] = UPD_STATUS_VERSION;
    v[UPD_S_STATE] = (uint8_t)state;
    v[UPD_S_ERROR] = state == UPD_STATE_FAILED ? s->st.error : 0u;
    v[UPD_S_CREDIT] = (uint8_t)credit;
    v[UPD_S_SESSION] = s->st.sel_session;
    v[UPD_S_FLAGS] = s->st.revoked ? UPD_SF_REVOKED : 0u;
    ultra_put16(v + UPD_S_CHUNK_MASK, s->st.mask);
    ultra_put32(v + UPD_S_ACCEPTED, s->st.committed + prefix);
    ultra_put32(v + UPD_S_COMMITTED, s->st.committed);
    ultra_put32(v + UPD_S_RETRY_MS, retry);
    ultra_put32(v + UPD_S_IMAGE_CRC, upd_get32(s->desc + UPD_D_BODY_CRC));
    ultra_put16(a + 30, ultra_crc16(a, 30));
}

/* the descriptor (both ANNOUNCE halves of the selected session): the same checks as the declared profile; then the
 * erase units wait for their polls (training never revokes or erases) */
static void st_evaluate(bl_sess_t *s)
{
    const uint32_t len = upd_get32(s->pending + UPD_D_BODY_LEN), version = upd_get32(s->pending + UPD_D_FW_VERSION);
    int rc;
    s->session = s->pending_session;
    s->have_half = 0;
    memcpy(s->desc, s->pending, 64);
    rc = bl_desc_check(s->pending);
    if (rc || ((len + 255u) & ~255u) > UPD_APP_BODY_MAX) { st_fail(s, rc ? (uint8_t)rc : UPD_ERROR_BOUNDS); return; }
    if (!bl_plat_layout_ok()) { st_fail(s, UPD_ERROR_TARGET); return; }
    if (!bl_plat_power_ok()) { st_fail(s, UPD_ERROR_SUPPLY); return; }
    if (s->app.desc_ok && version <= s->app.version && !s->force) { st_fail(s, UPD_ERROR_DESCRIPTOR); return; }
    s->npages = (len + 255u) / 256u;
    s->erase_cursor = UPD_APP_DESC;
    s->erase_end = (UPD_APP_BODY + s->npages * 256u + UPD_ERASE_BYTES - 1u) & ~(UPD_ERASE_BYTES - 1u);
    s->st.committed = 0;
    s->st.mask = 0;
    s->st.op = ST_OP_ERASE;
    s->st.op_due = 0;
    s->st.state = UPD_STATE_PREPARED;
    s->state = BL_S_ERASE;
}

/* a blocking operation, after its BUSY answer's window */
static void st_run(bl_sess_t *s)
{
    const uint32_t op = s->st.op, len = upd_get32(s->desc + UPD_D_BODY_LEN);
    uint32_t i;
    if (s->state == BL_S_FAILED || !op) return;
    s->st.op = ST_OP_NONE;
    s->st.op_due = 0;
    if (!bl_plat_power_ok()) { st_fail(s, UPD_ERROR_SUPPLY); return; }
    if (op == ST_OP_ERASE) {
        const uint32_t a = s->erase_cursor;
        const uint8_t *p = bl_plat_ptr(a);
        bl_plat_begin(2u * MCU_UPD_ERASE_MS);
        if (bl_plat_erase_page(a)) goto flash;
        for (i = 0; i < UPD_ERASE_BYTES; i += 2u)
            if (!upd_erased16(p[i] | ((uint32_t)p[i+1] << 8))) goto flash;
        if (a == UPD_APP_DESC) {
            s->st.revoked = 1;
            s->app.desc_ok = 0;
        }
        s->erase_cursor = a + UPD_ERASE_BYTES;
        if (s->erase_cursor == s->erase_end) s->state = BL_S_RECEIVE;
        else s->st.op = ST_OP_ERASE;
    } else if (op == ST_OP_PROGRAM) {
        const uint32_t a = UPD_APP_BODY + s->st.committed;
        uint32_t n = len - s->st.committed;
        if (n > 256u) n = 256u;
        for (i = n; i < 256u; ++i) s->st.stage[i] = (uint8_t)(i & 1u ? 0xE3u : 0x39u);   /* the erased fill */
        bl_plat_begin(2u * MCU_UPD_PROGRAM_MS);
        if (bl_plat_program_page(a, s->st.stage) || !equal(bl_plat_ptr(a), s->st.stage, n)) goto flash;
        s->st.committed += n;
        s->st.mask = 0;
        ++s->pages_programmed;
    } else {
        if (bl_plat_crc32(UPD_APP_BODY, len / 4u) != upd_get32(s->desc + UPD_D_BODY_CRC)) {
            st_fail(s, UPD_ERROR_CRC);
            return;
        }
        /* One shared deadline covers ALL metadata writes and readback. Magic last. */
        bl_plat_begin(2u * MCU_UPD_PROGRAM_MS);
        for (i = 2; i < 64u; i += 2u)
            if (bl_plat_program_half(UPD_APP_DESC + i, (uint16_t)(s->desc[i] | ((uint16_t)s->desc[i+1] << 8))))
                goto flash;
        if (!equal(bl_plat_ptr(UPD_APP_DESC + 2u), s->desc + 2u, 62)) goto flash;
        if (bl_plat_program_half(UPD_APP_DESC + 64u, UPD_ST_VERIFIED) ||
            bl_plat_ptr(UPD_APP_DESC + 64u)[0] != (UPD_ST_VERIFIED & 255u) ||
            bl_plat_ptr(UPD_APP_DESC + 64u)[1] != (UPD_ST_VERIFIED >> 8) ||
            bl_plat_program_half(UPD_APP_DESC, 0x4451u) ||
            !equal(bl_plat_ptr(UPD_APP_DESC), s->desc, 64)) goto flash;
        s->st.state = UPD_STATE_COMMITTED;
    }
    s->t_last = bl_plat_ms();
    return;
flash:
    ++s->flash_errors;
    st_fail(s, UPD_ERROR_FLASH);
}

static void st_chunk(bl_sess_t *s, uint32_t idx, const uint8_t *data)
{
    const uint32_t len = upd_get32(s->desc + UPD_D_BODY_LEN), off = idx * 32u, block = s->st.committed;
    uint32_t k, n, i, padding = 0;
    if (s->state != BL_S_RECEIVE || off >= len) return;
    n = len - off;
    if (n > 32u) n = 32u;
    for (i = n; i < 32u; ++i) padding |= data[i];
    /* Compare repeats before the busy/credit gate, including a fully staged
     * block and readback-verified flash. Final transport padding is also fixed. */
    if (off < block) {
        if (padding || !equal(bl_plat_ptr(UPD_APP_BODY + off), data, n)) st_fail(s, UPD_ERROR_CONFLICT);
        return;
    }
    if (off >= block + 256u) return;
    k = (off - block) / 32u;
    if (s->st.mask & (1u << k)) {
        if (padding || !equal(s->st.stage + 32u * k, data, n)) st_fail(s, UPD_ERROR_CONFLICT);
        return;
    }
    if (s->st.op || padding) return;
    memcpy(s->st.stage + 32u * k, data, n);
    s->st.mask = (uint8_t)(s->st.mask | (1u << k));
    if (s->st.mask == (1u << st_need(s)) - 1u) s->st.op = ST_OP_PROGRAM;
}

/* SELECT, the selected session's frames (ANNOUNCE excepted: the shared halves logic, then st_evaluate) and the
 * polls; 1 = handled here */
static int st_block(bl_sess_t *s, const uint8_t *f)
{
    const uint32_t type = f[0], session = f[1], len = upd_get32(s->desc + UPD_D_BODY_LEN);
    const uint8_t *data = f + 6;
    if (type == UPD_SELECT) {
        const int mine = equal(data, bl_plat_uid(), 12) && equal(data + 12, bl_info.target.bytes, 16);
        if (!(mine && s->st.selected == 1u && session == s->st.sel_session)) {
            const uint8_t initial = s->st.state == UPD_STATE_RESTART_REQUIRED;
            /* Keep clock observations and the installed-app facts. Everything
             * belonging to the transfer, including queued replies, is gone. */
            s->state = BL_S_LISTEN;
            s->session = s->pending_session = s->have_half = s->force = 0;
            memset(s->pending, 0, sizeof s->pending);
            memset(s->desc, 0, sizeof s->desc);
            memset(s->written, 0, sizeof s->written);
            s->npages = s->nwritten = s->erase_cursor = s->erase_end = 0;
            s->pages_programmed = s->flash_errors = s->evicted = 0;
            s->skip_result = s->lead = 0;
            s->exit_request = s->exit_result = s->exit_version = 0;
            pool_clear(s);
            memset(&s->st.state, 0, sizeof s->st - ((uint8_t *)&s->st.state - (uint8_t *)&s->st));
            s->st.state = initial ? UPD_STATE_RESTART_REQUIRED : UPD_STATE_IDLE;
            /* 0 = declared group (reset default), 1 = selected status,
             * 2 = explicitly excluded. Exclusion never becomes a broadcast. */
            s->st.selected = mine ? 1u : 2u;
            s->st.sel_session = (uint8_t)session;
        }
        s->t_last = bl_plat_ms();
        return 1;
    }
    /* A selection belongs only to its session (docs/update.md): the next session's PING or ANNOUNCE ends it, and that
     * session may be a blind one. A selected transfer still running ignores it (its own inactivity ends it first);
     * COMMITTED was its end: the new app boots. Other sessions' DATA, END, ABORT and polls are stale. */
    if (s->st.selected && session && session != s->st.sel_session && (type == UPD_PING || type == UPD_ANNOUNCE)) {
        if (s->st.selected == 2u || s->state == BL_S_LISTEN || s->state == BL_S_FAILED) s->st.selected = 0;
        else if (s->st.state == UPD_STATE_COMMITTED) {
            leave(s, UPD_REQ_BOOT_ONCE, UPD_RES_UPDATED);
            return 1;
        }
    }
    if (!s->st.selected) return type == UPD_STATUS;  /* declared: never answer or reserve poll-driven work */
    if (s->st.selected != 1u || session != s->st.sel_session) {
        if (session == s->st.sel_session) s->t_last = bl_plat_ms();   /* (excluded: that session is still running) */
        return 1;
    }
    s->t_last = bl_plat_ms();
    if (type != UPD_STATUS && type != UPD_ABORT && s->state == BL_S_FAILED) return 1;
    if (type == UPD_ANNOUNCE) return 0;
    if (type == UPD_STATUS) {
        if (!equal(data, bl_plat_uid(), 12) || !s->st.locked) return 1;   /* (unlocked: silence) */
        if (s->state != BL_S_FAILED && s->st.op && !s->st.op_due) {
            s->st.op_due = 1;
            s->st.op_at = s->st.frame_t0 + st_ticks(s, ULTRA_RW_DIR_MAX_US + ultra_rw_slot_us(data[14]) + ULTRA_RW_END_US);
        }
        st_answer(s);
        s->st.reply = 1;
        s->st.reply_at = s->st.frame_t0 + st_ticks(s, ST_REPLY_US);
        s->st.brr = (uint16_t)((((uint64_t)s->st.span_ticks << data[14]) + s->st.span_host) /
                               (2u * (uint64_t)s->st.span_host));
    } else if (type == UPD_DATA) {
        st_chunk(s, upd_get16(f + 2), data);
    } else if (type == UPD_END) {
        if (s->state == BL_S_RECEIVE && !s->st.op && s->st.state != UPD_STATE_COMMITTED && s->st.committed == len &&
            upd_get32(data) == upd_get32(s->desc + UPD_D_BODY_CRC) && upd_get32(data + 4) == len)
            s->st.op = ST_OP_COMMIT;
    } else if (type == UPD_PING) {
        if (s->st.state == UPD_STATE_COMMITTED) leave(s, UPD_REQ_BOOT_ONCE, UPD_RES_UPDATED);   /* acknowledged */
    } else if (type == UPD_ABORT) {
        if (s->state != BL_S_FAILED) st_fail(s, UPD_ERROR_SESSION);
        if (s->app.desc_ok) leave(s, UPD_REQ_NONE, UPD_RES_NONE);
    }
    return 1;
}
#endif

/* The shared codec checks CRC/header/type. Semantic invalidity is a no-op. */
__attribute__((noinline, used)) int bl_api_frame(const uint8_t *f)
{
    return !upd_check_frame(f) && upd_fields_ok(f) ? 0 : -1;
}

static void on_block(bl_sess_t *s, const uint8_t *f, uint32_t now)
{
    uint32_t type = f[0], session = f[1], idx = upd_get16(f + 2);
    const uint8_t *data = f + 6;
#if UPD_STATUS_PROFILE
    if (st_block(s, f)) return;
#endif
    if (type == UPD_ANNOUNCE) {
        if (s->state == BL_S_RECEIVE || s->state == BL_S_ERASE || s->state == BL_S_SKIP) {
            if (session != s->session) return;
            if (f[5] != s->force || !equal(s->desc + 32u * idx, data, 32)) fail(s);
#if UPD_STATUS_PROFILE
            else if (!s->st.selected) lead_start(s);   /* (the lead counts from the final ANNOUNCE) */
#endif
        } else {
            if (s->state == BL_S_FAILED && session == s->session) return;
            if (session != s->pending_session) {
                s->pending_session = (uint8_t)session;
                s->have_half = 0;
            }
            if (s->have_half && (f[5] != s->force ||
                ((s->have_half & (1u << idx)) && !equal(s->pending + 32u * idx, data, 32)))) {
                s->session = (uint8_t)session;
                fail(s);
                return;
            }
            s->force = f[5];
            memcpy(s->pending + 32u * idx, data, 32);
            s->have_half |= (uint8_t)(1u << idx);
            if (s->have_half == 3u) evaluate(s);
        }
    } else {
        if (session != s->session && !(s->have_half && session == s->pending_session)) return;
        if (type == UPD_ABORT) {
            s->session = (uint8_t)session;
            fail(s);
            if (s->app.desc_ok) leave(s, UPD_REQ_NONE, UPD_RES_NONE);
        } else if (type == UPD_DATA && s->state == BL_S_RECEIVE) {
            if (!put_chunk(s, idx, data)) return;
        } else if (type == UPD_END) {
            if (upd_get32(data) != upd_get32(s->desc + UPD_D_BODY_CRC) ||
                upd_get32(data + 4) != upd_get32(s->desc + UPD_D_BODY_LEN)) return;
            finish(s);
        } else if (type == UPD_DATA && s->state == BL_S_SKIP) {
            /* (a skipping node waits for END: its session's DATA passes are activity) */
        } else if (type != UPD_PING || !session) return;
    }
    /* A flash operation may have advanced physical time since the frame. */
    s->t_last = bl_plat_ms();
    (void)now;
}

void bl_sess_frame(bl_sess_t *s, uint32_t nbits, const uint8_t *tail, uint32_t now)
{
    ++s->frames;
    s->gate = BL_GATE_OPEN;
    if (s->mode == BL_ACT_JUMP) return;
    if (nbits == UPD_FRAME_BITS && !bl_api_frame(tail)) {
        ++s->upd_frames;
#if UPD_STATUS_PROFILE
        st_observe(s, tail);
#endif
        on_block(s, tail, now);
    } else if (nbits >= 64u && upd_get16(tail+40) == ULTRA_MAGIC &&
               (tail[42] >> 4) == ULTRA_VERSION && !(tail[42] & UPD_HDR_FLAG) &&
               upd_crc8(tail+40, 7) == tail[47]) {
        s->seen_ultra = 1;
        if (s->mode == BL_ACT_RECOVERY && (s->state == BL_S_LISTEN || s->state == BL_S_FAILED))
            s->gate = BL_GATE_ULTRA;
    } else ++s->bad_frames;
}

int bl_sess_poll(bl_sess_t *s, uint32_t now)
{
#if UPD_STATUS_PROFILE
    /* A selection whose session went quiet for the idle time has ended (a running selected transfer fails instead). */
    if (s->st.selected && (s->st.selected == 2u || s->state == BL_S_LISTEN || s->state == BL_S_FAILED) &&
        now - s->t_last >= BL_IDLE_MS)
        s->st.selected = 0;
    if (s->state == BL_S_FAILED) {
        /* Failure never authorizes flash work. Keep its reported error until
         * ABORT or inactivity returns to an app whose metadata is still intact. */
        if (now - s->t_last >= BL_IDLE_MS && s->app.desc_ok)
            leave(s, UPD_REQ_NONE, UPD_RES_NONE);
        return s->mode == BL_ACT_JUMP;
    }
    if (s->st.op && s->st.op_due && (int32_t)(bl_plat_ticks() - s->st.op_at) >= 0) {
        st_run(s);
        now = bl_plat_ms();                             /* (the operation took its time: t_last is after `now`) */
    }
    if (s->st.op_due) s->t_last = now;                  /* (an announced busy interval is no inactivity) */
    if (s->lead) {                                      /* (nor is the declared erase lead: idle from its end) */
        const uint32_t until = (int32_t)(now - s->lead_end) < 0 ? now : s->lead_end;
        if ((int32_t)(until - s->t_last) > 0) s->t_last = until;
        if (until == s->lead_end) s->lead = 0;
    }
    if (s->st.selected && s->state == BL_S_ERASE) {
        if (now - s->t_last >= BL_IDLE_MS) {
            st_fail(s, UPD_ERROR_SESSION);
            if (s->app.desc_ok) leave(s, UPD_REQ_NONE, UPD_RES_NONE);
        }
        return s->mode == BL_ACT_JUMP;
    }
    /* A lost poll/ACK must not remove the host's committed status endpoint. */
    if (s->st.state == UPD_STATE_COMMITTED && s->state == BL_S_RECEIVE)
        return s->mode == BL_ACT_JUMP;
    if (s->mode == BL_ACT_JUMP) return 1;
#endif
    if (s->state == BL_S_ERASE) erase_next(s);
    else if (s->state == BL_S_LISTEN) {
        if ((s->mode == BL_ACT_TRIAL && (s->seen_ultra || now - s->t_start >= BL_TRIAL_MS)) ||
            (s->mode == BL_ACT_SESSION && now - s->t_last >= BL_WAIT_MS) ||
            (s->mode == BL_ACT_RECOVERY && now - s->t_start >= BL_RECOVERY_MS && s->app.confirmed)) {
            if (s->app.desc_ok) {
                if (s->mode == BL_ACT_TRIAL) bl_trial_record(&s->app, s->seen_ultra);
                leave(s, s->app.verified ? UPD_REQ_BOOT_ONCE : UPD_REQ_NONE, UPD_RES_NONE);
            }
            else s->mode = BL_ACT_RECOVERY;
        }
    } else if (now - s->t_last >= BL_IDLE_MS) {
        /* Inactivity is never END and cannot authorize commit. */
#if UPD_STATUS_PROFILE
        st_fail(s, UPD_ERROR_SESSION);
#else
        fail(s);
#endif
        if (s->app.desc_ok) leave(s, UPD_REQ_NONE, UPD_RES_NONE);
    }
    return s->mode == BL_ACT_JUMP;
}

void bl_ho_exit(upd_handover_t *ho, const bl_sess_t *s)
{
    ho->request = s->exit_request;
    ho->result = s->exit_result;
    ho->result_version = s->exit_version;
    if (s->exit_result == UPD_RES_UPDATED) ho->wd_resets = 0;
    bl_ho_seal(ho);
}
