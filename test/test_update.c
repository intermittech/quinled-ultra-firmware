/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The app's side of the firmware update (core/upd_app.c): transparency, ANNOUNCE decisions, results, PING / confirm,
 * the descriptor page reader. Included by test_main.c. The bootloader side has its own tests (bootloader/test). */
#include "../src/core/upd_app.h"

static upd_bl_info_t tu_boot = {{'Q','D','B','2'},1,
    {{1,1,1,1,3,0,0,UPD_REV_D2A_R04,8,0,0x21,0,0,0,0xDF,0}}, {0x88,10,20},5,{0}};

static void tu_desc(uint8_t *d, uint32_t version, uint8_t hw_id, uint16_t revmask, uint8_t min_bl)
{
    uint32_t i;
    for (i = 0; i < UPD_DESC_BYTES; i++) {
        d[i] = 0;
    }
    d[0] = 'Q'; d[1] = 'D'; d[2] = 'U'; d[3] = '6';
    d[UPD_D_VERSION] = UPD_DESC_VERSION;
    d[UPD_D_TARGET] = hw_id;
    for(i=0; i<16u && !(revmask & (1u<<i)); ++i) {}
    d[UPD_D_REV_MIN] = d[UPD_D_REV_MAX] = (uint8_t)i;
    memcpy(d+UPD_D_TARGET+1,tu_boot.target.bytes+1,6);
    upd_put32(d + UPD_D_FW_VERSION, version);
    upd_put32(d + UPD_D_BODY_LEN, 30548u);
    upd_put32(d + UPD_D_BODY_CRC, 0x12345678u);
    upd_put32(d + UPD_D_LOAD, UPD_APP_BODY);
    upd_put32(d + UPD_D_ENTRY, UPD_APP_ENTRY);
    d[UPD_D_MIN_BL] = min_bl;
    upd_put32(d + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d, 15u));
}

#define TU_V(a, b, c, pre) (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (pre))
#define TU_REVS (1u << UPD_REV_D2A_R04)

/* one session's frames into the app, as the board sees them: frame end (header) first decides transparency for
 * the frames after it; captured frames (384 bits) only while transparent */
typedef struct { upd_app_t u; uint32_t now, actions[8], nact; } tu_board_t;

static void tu_frame(tu_board_t *b, const uint8_t *f, int whole)
{
    uint32_t a = UPD_APP_NONE;
    b->now += 1;
    if (whole && upd_app_transparent(&b->u)) {
        a = upd_app_frame(&b->u, f, b->now);
    }
    upd_app_frame_end(&b->u, 1, (uint8_t)(f[UPD_BLOCK_BYTES + 2] & 0x0Fu), b->now);
    if (a != UPD_APP_NONE && b->nact < 8u) {
        b->actions[b->nact++] = a;
    }
}

static void tu_upd(tu_board_t *b, uint8_t type, uint8_t session, uint32_t idx, uint8_t flags, const uint8_t *data)
{
    uint8_t f[UPD_FRAME_BYTES];
    upd_build_frame(f, type, session, idx, 0, flags, data, b->now);
    tu_frame(b, f, 1);
}

static void tu_announce(tu_board_t *b, uint8_t session, const uint8_t *desc, uint8_t flags, uint32_t pairs)
{
    uint32_t i;
    for (i = 0; i < 2u * pairs; i++) {
        tu_upd(b, UPD_ANNOUNCE, session, i & 1u, flags, desc + 32u * (i & 1u));
    }
}

static void tu_end(tu_board_t *b, uint8_t session)
{
    uint32_t i;
    uint8_t payload[32] = {0};
    memcpy(payload, b->u.pending + UPD_D_BODY_CRC, 4);
    memcpy(payload + 4, b->u.pending + UPD_D_BODY_LEN, 4);
    for (i = 0; i < 5u; i++) {
        tu_upd(b, UPD_END, session, 0, 0, payload);
    }
}

static void tu_init(tu_board_t *b, uint32_t version, int bootloader, int confirmed)
{
    upd_app_info_t info = {0};
    info.hw_id = bootloader ? UPD_HW_ID_D2A_PLUS_5CH : 0;
    info.hw_rev = UPD_REV_D2A_R04;
    info.bl_version = bootloader ? 1 : 0;
    info.desc_ok = (uint8_t)(bootloader && version != 0u);
    info.confirmed = (uint8_t)confirmed;
    info.st_next = 2;
    info.version = version;
    info.boot = bootloader ? &tu_boot : 0;
    memset(b, 0, sizeof(*b));
    upd_app_init(&b->u, &info);
}

static int tu_has(const tu_board_t *b, uint32_t a)
{
    uint32_t i;
    for (i = 0; i < b->nact; i++) {
        if (b->actions[i] == a) {
            return 1;
        }
    }
    return 0;
}

/* still transparent at every poll from `from` to `from + ms` (100 ms steps, the last one exact) */
static int tu_hold_through(tu_board_t *b, uint32_t from, uint32_t ms)
{
    uint32_t t;
    for (t = 0; t < ms; t += 100u) {
        upd_app_poll(&b->u, from + t);
        if (!upd_app_transparent(&b->u)) return 0;
    }
    upd_app_poll(&b->u, from + ms);
    return upd_app_transparent(&b->u);
}

/* the transit hold an ANNOUNCE's first half implies, counted independently of upd_app.c: the layout's metadata unit and
 * the body's erase units, 2 x 255 ms each + 100 ms; anything outside a known layout's slot: the largest (V203) */
static uint32_t tu_hold_of(const uint8_t *d)
{
    uint32_t layout = d[UPD_D_TARGET + UPD_T_LAYOUT], len = upd_get32(d + UPD_D_BODY_LEN), load = upd_get32(d + UPD_D_LOAD);
    uint32_t unit = layout == 1u ? 256u : layout == 4u ? 4096u : 0u;
    uint32_t body = layout == 1u ? 0x08002100u : 0x08005000u, end = layout == 1u ? 0x08010000u : 0x08080000u;
    if (!unit || !len || load != body || len > end - body) return 114340u;
    return (1u + (len + unit - 1u) / unit) * 510u + 100u;
}

/* a foreign descriptor half 0 of the given layout / body: the hold reads only these fields */
static void tu_desc_for(uint8_t *d, const uint8_t *base, uint32_t layout, uint32_t len, uint32_t load)
{
    memcpy(d, base, UPD_DESC_BYTES);
    d[UPD_D_TARGET + UPD_T_LAYOUT] = (uint8_t)layout;
    upd_put32(d + UPD_D_BODY_LEN, len);
    upd_put32(d + UPD_D_LOAD, load);
}

/* Transit forwarding through a foreign family's erase lead (docs/timing.md "Update forwarding and entry"; decided
 * 2026-10-23): an ANNOUNCE holds transparency for its own declared lead, counted from its descriptor (62 KiB V407:
 * 8770 ms; a full V407 slot: 63340 ms; a full V203 slot or anything unknown: 114340 ms) from its final half; the 2000 ms
 * idle close counts from the hold's end; this session's DATA / END / ABORT, an ordinary header restore the 2000 ms rule.
 * Never a TAKE for a foreign image. */
static void test_update_hold(const uint8_t *d_foreign, const uint8_t *d_new)
{
    static const uint32_t t0s[] = { 1000u, 0xFFFFFFFFu - 5000u };   /* (the second wraps inside the hold) */
    static const uint8_t ends[] = { UPD_DATA, UPD_END, UPD_ABORT };
    static const struct { uint32_t layout, len, load, want; const char *what; } hc[] = {
        { 4u, 63488u, 0x08005000u, 8770u, "62 KiB V407" },
        { 4u, 0x08080000u - 0x08005000u, 0x08005000u, 63340u, "full V407 slot" },
        { 1u, 0x08010000u - 0x08002100u, 0x08002100u, 114340u, "full V203 slot" },
        { 1u, 30548u, 0x08002100u, 61810u, "30548-byte V203 image" },
        { 9u, 63488u, 0x08005000u, 114340u, "unknown layout" },
        { 4u, 0u, 0x08005000u, 114340u, "empty body" },
        { 4u, 0x08080000u - 0x08005000u + 1u, 0x08005000u, 114340u, "body above the slot" },
        { 4u, 63488u, 0x08002100u, 114340u, "body at another address" },
    };
    uint8_t d[UPD_DESC_BYTES], z[32] = {0};
    tu_board_t b;
    uint32_t i, k, c, t, hold;
    printf("firmware update, transit hold through the ANNOUNCE's declared erase lead\n");
    for (c = 0; c < sizeof hc / sizeof hc[0]; c++) {
        tu_desc_for(d, d_foreign, hc[c].layout, hc[c].len, hc[c].load);
        hold = tu_hold_of(d);
        CHECK(hold == hc[c].want, "%s: the independent count %u ms, want %u", hc[c].what, (unsigned)hold,
              (unsigned)hc[c].want);
        for (i = 0; i < 2u; i++) {
            tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
            b.now = t0s[i];
            tu_upd(&b, UPD_PING, 0x21, 0, 0, 0);
            tu_upd(&b, UPD_PING, 0x21, 0, 0, 0);
            tu_announce(&b, 0x21, d, 0, 1);
            t = b.now;                                   /* (the final half) */
            CHECK(!tu_has(&b, UPD_APP_TAKE) && tu_hold_through(&b, t, hold + UPD_IDLE_CLOSE_MS - 1u),
                  "%s, t0 %08X: transparent through the hold + 1999 ms, no TAKE", hc[c].what, (unsigned)t0s[i]);
            upd_app_poll(&b.u, t + hold + UPD_IDLE_CLOSE_MS);
            CHECK(!upd_app_transparent(&b.u), "%s, t0 %08X: closed 2000 ms after the hold's end", hc[c].what,
                  (unsigned)t0s[i]);
        }
    }
    /* half 1 without half 0: the largest hold */
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_upd(&b, UPD_PING, 0x22, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0x22, 0, 0, 0);
    tu_desc_for(d, d_foreign, 4u, 63488u, 0x08005000u);
    tu_upd(&b, UPD_ANNOUNCE, 0x22, 1, 0, d + 32);
    t = b.now;
    CHECK(tu_hold_through(&b, t, 114340u + UPD_IDLE_CLOSE_MS - 1u), "half 1 alone: held 114340 ms + 1999");
    upd_app_poll(&b.u, t + 114340u + UPD_IDLE_CLOSE_MS);
    CHECK(!upd_app_transparent(&b.u), "half 1 alone: closed at 116340 ms");
    /* DATA at 3500 ms into a 62 KiB lead and at 49700 ms into a full V407 slot's: still open, then the 2000 ms rule */
    for (c = 0; c < 2u; c++) {
        uint32_t at = c ? 49700u : 3500u;
        for (i = 0; i < 2u; i++) {
            tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
            b.now = t0s[i];
            tu_upd(&b, UPD_PING, 0x23, 0, 0, 0);
            tu_upd(&b, UPD_PING, 0x23, 0, 0, 0);
            tu_desc_for(d, d_foreign, 4u, c ? 0x08080000u - 0x08005000u : 63488u, 0x08005000u);
            tu_announce(&b, 0x23, d, 0, 1);
            t = b.now;
            CHECK(tu_hold_through(&b, t, at - 1u), "%u ms lead: open until its DATA", (unsigned)at);
            b.now = t + at - 1u;
            tu_upd(&b, UPD_DATA, 0x23, 0, 0, z);
            t = b.now;
            upd_app_poll(&b.u, t + UPD_IDLE_CLOSE_MS - 1u);
            CHECK(upd_app_transparent(&b.u), "DATA at %u ms: open 1999 ms after it", (unsigned)at);
            upd_app_poll(&b.u, t + UPD_IDLE_CLOSE_MS);
            CHECK(!upd_app_transparent(&b.u), "DATA at %u ms, t0 %08X: the 2000 ms rule again", (unsigned)at,
                  (unsigned)t0s[i]);
        }
    }
    /* this session's END / ABORT end the hold; another session's DATA / END / ABORT (stale) keep it */
    tu_desc_for(d, d_foreign, 4u, 63488u, 0x08005000u);
    for (k = 0; k < 3u; k++) {
        tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
        tu_upd(&b, UPD_PING, 0x24, 0, 0, 0);
        tu_upd(&b, UPD_PING, 0x24, 0, 0, 0);
        tu_announce(&b, 0x24, d, 0, 1);
        tu_upd(&b, ends[k], 0x25, 0, 0, z);              /* another session's: stale */
        t = b.now;
        CHECK(tu_hold_through(&b, t, 8770u + UPD_IDLE_CLOSE_MS - 2u), "another session's type %u keeps the hold",
              (unsigned)ends[k]);
        tu_upd(&b, ends[k], 0x24, 0, 0, z);
        t = b.now;
        upd_app_poll(&b.u, t + UPD_IDLE_CLOSE_MS);
        CHECK(!upd_app_transparent(&b.u), "this session's type %u: the 2000 ms rule again", (unsigned)ends[k]);
    }
    /* a repeated pair re-arms the hold from its final half */
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_upd(&b, UPD_PING, 0x26, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0x26, 0, 0, 0);
    tu_announce(&b, 0x26, d, 0, 1);
    b.now += 5000u;
    tu_announce(&b, 0x26, d, 0, 1);
    t = b.now;
    CHECK(tu_hold_through(&b, t, 8770u + UPD_IDLE_CLOSE_MS - 1u), "a repeated pair: held from its final half");
    upd_app_poll(&b.u, t + 8770u + UPD_IDLE_CLOSE_MS);
    CHECK(!upd_app_transparent(&b.u), "a repeated pair: closed 2000 ms after its hold");
    /* an ordinary header closes and ends the hold */
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_upd(&b, UPD_PING, 0x27, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0x27, 0, 0, 0);
    tu_announce(&b, 0x27, d, 0, 1);
    upd_app_frame_end(&b.u, 1, 0, ++b.now);
    CHECK(!upd_app_transparent(&b.u), "an ordinary header closes during the hold");
    tu_upd(&b, UPD_PING, 0x28, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0x28, 0, 0, 0);
    upd_app_poll(&b.u, b.now + UPD_IDLE_CLOSE_MS);
    CHECK(!upd_app_transparent(&b.u), "... and ends the hold (the next acquisition idles out at 2000 ms)");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_upd(&b, UPD_PING, 0x29, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0x29, 0, 0, 0);
    tu_announce(&b, 0x29, d_new, 0, 2);
    CHECK(tu_has(&b, UPD_APP_TAKE), "its own image: TAKE, as without a hold (the hold grants forwarding only)");
}

/* An update session through a chain of 10 boards at the wire level (test_wire.c: the real tracker, gate plans and
 * frame decisions, and upd_app as main.c runs it). The lead-in is ANNOUNCE halves, as the hub sends it: a board only
 * sees whole UPD frames once the one before it is transparent, so they turn transparent one after the other; each
 * takes the ANNOUNCE and resets into the bootloader (dark for a moment, then open for everything). */
static void test_update_chain(void)
{
    static const struct { proto_mode_t mode; proto_t sticky; int conf, pre, other_hw; const char *name; } sc[] = {
        { PMODE_ULTRA, PROTO_NONE, 0, 3, 0, "Ultra mode" },
        { PMODE_AUTO, PROTO_ULTRA, 1, 3, 0, "auto, locked to Ultra" },
        { PMODE_AUTO, PROTO_ULTRA, 0, 0, 0, "auto, Ultra stored (provisional), session right after power-up" },
        { PMODE_WS2805, PROTO_NONE, 0, 0, 0, "forced WS2805" },
        { PMODE_ULTRA, PROTO_NONE, 0, 3, 1, "Ultra mode, image for other hardware" },
    };
    const int nb = 10, ends = 6;
    uint8_t d[64];
    size_t s;
    int f, k;
    printf("firmware update through a chain of %d boards (wire level, lead-in = ANNOUNCE halves)\n", nb);
    for (s = 0; s < sizeof sc / sizeof sc[0]; s++) {
        wres_t r;
        int takes = 0, last_take = -1, forwarded = 0, shows = 0, order = 1;
        tu_desc(d, TU_V(0, 5, 0, 0xFF), (uint8_t)(sc[s].other_hw ? 2 : UPD_HW_ID_D2A_PLUS_5CH), TU_REVS, 1);
        wupd_from = sc[s].pre;
        for (f = wupd_from; f < WF_MAX; f++) {
            int h = (f - wupd_from) & 1;
            if (f < WF_MAX - ends) upd_build_frame(wupd_host[f], UPD_ANNOUNCE, 0x21, (uint32_t)h, 0, 0, d + 32 * h, 0);
            else {
                uint8_t payload[32] = {0};
                memcpy(payload, d + UPD_D_BODY_CRC, 4);
                memcpy(payload + 4, d + UPD_D_BODY_LEN, 4);
                upd_build_frame(wupd_host[f], UPD_END, 0x21, 0, 0, 0, payload, 0);
            }
        }
        memset(&wupd_info, 0, sizeof wupd_info);
        wupd_info.hw_id = UPD_HW_ID_D2A_PLUS_5CH;
        wupd_info.hw_rev = UPD_REV_D2A_R04;
        wupd_info.bl_version = 1;
        wupd_info.boot = &tu_boot;
        wupd_info.desc_ok = 1;
        wupd_info.confirmed = 1;
        wupd_info.st_next = 2;
        wupd_info.version = TU_V(0, 4, 0, 0xFF);
        wconfirmed = sc[s].conf;
        whost_build(WK_UPD, nb, WF_MAX, 200000.0, 0, 0);
        r = wchain(nb, sc[s].mode, sc[s].sticky, 0);
        for (k = 0; k < nb; k++) {
            if (wb[k].take_hf >= 0) {
                takes++;
                if (wb[k].take_hf > last_take) last_take = wb[k].take_hf;
            }
            forwarded += wb[k].upd_ok > 0;
            shows += wb[k].upd_shows == 1;
            if (k > 0 && wb[k].upd_first_hf >= 0 && wb[k].upd_first_hf < wb[k - 1].upd_first_hf) order = 0;
        }
        if (!sc[s].other_hw) {
            printf("  %s: %d / %d boards take it, the last at UPD frame %d (%.1f frames per board; the hub's lead-in "
                   "of 500 ms + 15 ms per board is ~%d frames)\n", sc[s].name, takes, nb, last_take - wupd_from,
                   (double)(last_take - wupd_from + 1) / nb, (500000 + 15000 * nb) / 809);
            CHECK(takes == nb, "%s: %d / %d boards take the ANNOUNCE", sc[s].name, takes, nb);
        } else {
            printf("  %s: %d take it, %d / %d see the whole session, %d show NACK at END\n", sc[s].name, takes,
                   forwarded, nb, shows);
            CHECK(takes == 0 && forwarded == nb && shows == nb,
                  "%s: nobody takes it, all %d boards see the whole session, all show NACK at END", sc[s].name, nb);
        }
        CHECK(order, "%s: board k sees whole UPD frames no earlier than board k-1", sc[s].name);
        if (sc[s].mode == PMODE_ULTRA && sc[s].pre > 0) {
            CHECK(r.garbage == 0, "%s: normal Ultra frames before the session shown right (garbage %d)", sc[s].name,
                  r.garbage);
        }
    }
    wupd_from = WF_MAX;                    /* back to plain wire chains */
    memset(&wupd_info, 0, sizeof wupd_info);
    wconfirmed = 0;
}

/* the update frames' CRCs are ultra_protocol.h's one copy (ULTRA_CRC_EXTERN / UPD_CRC_ULTRA): against the
 * bitwise definitions the bootloader, the tools and the hub keep (ultra_update.h without the define), random data */
static uint16_t ref_crc16(const uint8_t *d, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        uint32_t b;
        c ^= (uint16_t)((uint16_t)*d++ << 8);
        for (b = 0; b < 8u; b++) {
            c = (uint16_t)((c & 0x8000u) ? (((uint32_t)c << 1) ^ 0x1021u) : ((uint32_t)c << 1));
        }
    }
    return c;
}

static uint8_t ref_crc8(const uint8_t *d, uint32_t n)
{
    uint8_t c = 0;
    while (n--) {
        uint32_t b;
        c ^= *d++;
        for (b = 0; b < 8u; b++) {
            c = (uint8_t)((c & 0x80u) ? (((uint32_t)c << 1) ^ 0x07u) : ((uint32_t)c << 1));
        }
    }
    return c;
}

static void test_update_crc(void)
{
    uint8_t d[64];
    uint32_t i, k, x = 0x2545F491u, bad16 = 0, bad8 = 0;
    for (k = 0; k < 20000u; k++) {
        uint32_t n = k % 65u;
        for (i = 0; i < n; i++) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            d[i] = (uint8_t)x;
        }
        bad16 += upd_crc16(d, n) != ref_crc16(d, n) || ultra_crc16(d, n) != ref_crc16(d, n);
        bad8 += upd_crc8(d, n) != ref_crc8(d, n) || ultra_crc8(d, n) != ref_crc8(d, n);
    }
    CHECK(bad16 == 0u && bad8 == 0u, "the CRC-16 / CRC-8 of the update frames = the bitwise reference (%u / %u of 20000 differ)",
          (unsigned)bad16, (unsigned)bad8);
}

static void test_update(void)
{
    test_update_crc();
    tu_board_t b;
    uint8_t d_new[64], d_same[64], d_old[64], d_hw[64], d_rev[64], d_bl[64];
    printf("firmware update, app side (core/upd_app.c)\n");
    tu_desc(d_new, TU_V(0, 5, 0, 0xFF), 1, TU_REVS, 1);
    tu_desc(d_same, TU_V(0, 4, 0, 0xFF), 1, TU_REVS, 1);
    tu_desc(d_old, TU_V(0, 3, 9, 0xFF), 1, TU_REVS, 1);
    tu_desc(d_hw, TU_V(0, 5, 0, 0xFF), 2, TU_REVS, 1);
    tu_desc(d_rev, TU_V(0, 5, 0, 0xFF), 1, 1u << (UPD_REV_D2A_R04 - 1u), 1);
    tu_desc(d_bl, TU_V(0, 5, 0, 0xFF), 1, TU_REVS, 2);

    /* transparency */
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    CHECK(!upd_app_transparent(&b.u), "one UPD header: not yet transparent");
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    CHECK(upd_app_transparent(&b.u), "%u UPD headers in a row: transparent", (unsigned)UPD_APP_ENTER_RUN);
    upd_app_frame_end(&b.u, 1, 0, ++b.now);
    CHECK(!upd_app_transparent(&b.u), "a normal Ultra header: back to normal");
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    upd_app_frame_end(&b.u, 0, 0, ++b.now);
    CHECK(upd_app_transparent(&b.u), "a WS frame (no Ultra header) changes nothing");
    CHECK(upd_app_poll(&b.u, b.now + UPD_IDLE_CLOSE_MS - 2u) == UPD_APP_NONE && upd_app_transparent(&b.u),
          "still transparent before the timeout");
    upd_app_poll(&b.u, b.now + UPD_IDLE_CLOSE_MS);
    CHECK(!upd_app_transparent(&b.u), "2 s without UPD frames: back to normal");
    {
        gate_plan_t g = proto_gate_plan_transparent();
        CHECK(g.start_open == 1 && g.t1 == 0 && g.t2 == 0 && g.own_bits == UPD_FRAME_BITS,
              "transparent gate plan: open, no toggle, 384-bit capture");
    }

    /* decisions */
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x10, d_new, 0, 3);
    CHECK(tu_has(&b, UPD_APP_TAKE), "newer image: TAKE (reset into the bootloader)");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x11, d_same, 0, 8);
    CHECK(b.nact == 0, "same version: nothing during the session");
    tu_end(&b, 0x11);
    CHECK(b.nact == 1 && b.actions[0] == UPD_APP_SHOW_SAME, "... and 'already active' once at END");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x12, d_old, 0, 3);
    tu_end(&b, 0x12);
    CHECK(b.nact == 1 && b.actions[0] == UPD_APP_SHOW_REJECTED, "older version: NACK at END");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x13, d_old, UPD_F_FORCE, 3);
    CHECK(tu_has(&b, UPD_APP_TAKE), "older version with FORCE: TAKE");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x14, d_same, UPD_F_FORCE, 3);
    CHECK(tu_has(&b, UPD_APP_TAKE), "same version with FORCE: TAKE");
    {
        const uint8_t *bad[3];
        const char *what[3] = { "other hardware", "other board revision", "needs a newer bootloader" };
        uint32_t k;
        bad[0] = d_hw; bad[1] = d_rev; bad[2] = d_bl;
        for (k = 0; k < 3u; k++) {
            tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
            tu_announce(&b, (uint8_t)(0x20 + k), bad[k], UPD_F_FORCE, 3);
            tu_end(&b, (uint8_t)(0x20 + k));
            CHECK(b.nact == 1 && b.actions[0] == UPD_APP_SHOW_REJECTED, "%s: NACK even with FORCE", what[k]);
        }
    }
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x15, d_same, 0, 3);
    tu_announce(&b, 0x16, d_new, 0, 2);
    CHECK(tu_has(&b, UPD_APP_TAKE), "a new session with another image is evaluated again");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x17, d_same, 0, 3);
    tu_upd(&b, UPD_ABORT, 0x17, 0, 0, 0);
    tu_end(&b, 0x17);
    CHECK(b.nact == 0, "ABORT: no pattern");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);        /* an ABORT between the halves: the half (and its FORCE) dropped */
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);               /* (transparent: whole frames from here) */
    tu_upd(&b, UPD_ANNOUNCE, 0x1C, 0, UPD_F_FORCE, d_old);
    CHECK(b.u.have_half == 1u && b.u.force, "half 0 with FORCE taken");
    tu_upd(&b, UPD_ABORT, 0x1C, 0, 0, 0);
    tu_upd(&b, UPD_ANNOUNCE, 0x1C, 1, 0, d_old + 32u);
    CHECK(b.nact == 0 && b.u.have_half == 0u,
          "ABORT prevents reuse of the cancelled session");
    tu_upd(&b, UPD_ANNOUNCE, 0x1C, 0, 0, d_old);
    tu_end(&b, 0x1C);
    CHECK(b.nact == 0, "cancelled session stays cancelled");
    tu_announce(&b, 0x1F, d_old, 0, 2);
    tu_end(&b, 0x1F);
    CHECK(b.nact == 1 && b.actions[0] == UPD_APP_SHOW_REJECTED,
          "a new session evaluates without the cancelled FORCE");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_ANNOUNCE, 0x1D, 0, 0, d_new);
    tu_upd(&b, UPD_ABORT, 0x1E, 0, 0, 0);              /* another session's ABORT: the half stays */
    tu_upd(&b, UPD_ANNOUNCE, 0x1D, 1, 0, d_new + 32u);
    CHECK(tu_has(&b, UPD_APP_TAKE), "another session's ABORT leaves the half: TAKE");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x18, d_same, 0, 3);
    CHECK(upd_app_poll(&b.u, b.now + tu_hold_of(d_same) + UPD_IDLE_CLOSE_MS) == UPD_APP_SHOW_SAME,
          "END missed: the result at the timeout (an ANNOUNCE without DATA: 2000 ms after its hold's end)");
    tu_init(&b, 0, 1, 0);
    tu_announce(&b, 0x19, d_old, 0, 3);
    CHECK(tu_has(&b, UPD_APP_TAKE), "no valid descriptor of its own: any compatible image is taken");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 0, 1);
    tu_announce(&b, 0x1A, d_new, 0, 3);
    tu_end(&b, 0x1A);
    CHECK(b.nact == 0 && upd_app_transparent(&b.u), "no bootloader (standalone build): forward only");
    {
        uint8_t f[UPD_FRAME_BYTES];
        tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
        tu_upd(&b, UPD_PING, 0, 0, 0, 0);
        tu_upd(&b, UPD_PING, 0, 0, 0, 0);
        upd_build_frame(f, UPD_ANNOUNCE, 0x1B, 0, 0, 0, d_new, 0);
        f[10] ^= 1u;                       /* a bit error: CRC-16 fails */
        CHECK(upd_app_frame(&b.u, f, b.now) == UPD_APP_NONE && b.u.have_half == 0, "a corrupted block is ignored");
    }
    {
        uint8_t f[UPD_FRAME_BYTES], altered[32];
        uint32_t k;
        for (k = 0; k < 5u; ++k) {
            tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
            upd_build_frame(f, UPD_ANNOUNCE, k ? 61 : 0, k == 1 ? 2 : 0,
                            k == 2 ? 1 : 0, k == 3 ? 2 : 0, d_new, 0);
            if (k == 4) upd_build_frame(f, UPD_PING, 0, 0, 0, 0, d_new, 0);
            CHECK(upd_app_frame(&b.u, f, b.now) == UPD_APP_NONE && !b.u.have_half && !b.u.ping_seen,
                  "ABI-3 malformed control %u cannot change session or confirm", k);
        }
        for (k = 0; k < 2u; ++k) {
            tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
            b.u.transparent = 1;
            tu_upd(&b, UPD_ANNOUNCE, 62, 0, 0, d_new);
            memcpy(altered, d_new, 32); altered[16] ^= 1;
            tu_upd(&b, UPD_ANNOUNCE, 62, k, k ? UPD_F_FORCE : 0, k ? d_new+32 : altered);
            tu_announce(&b, 62, d_new, 0, 2);
            CHECK(!tu_has(&b, UPD_APP_TAKE), "conflicting repeated bytes/FORCE poison session %u", k);
            tu_announce(&b, 63, d_new, 0, 2);
            CHECK(tu_has(&b, UPD_APP_TAKE), "fresh session recovers from conflict %u", k);
        }
        tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
        tu_announce(&b, 64, d_same, 0, 3);
        tu_upd(&b, UPD_END, 64, 0, 0, 0);
        CHECK(!b.nact, "END with wrong image tuple cannot finish session");
        tu_end(&b, 64);
        CHECK(tu_has(&b, UPD_APP_SHOW_SAME), "matching END finishes skipped session");
    }

    /* A hub restart / 8-bit session wrap may reuse an old session id. */
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x35, d_same, 0, 3);
    tu_end(&b, 0x35);
    upd_app_frame_end(&b.u, 1, 0, ++b.now);
    tu_announce(&b, 0x35, d_new, 0, 3);
    CHECK(tu_has(&b, UPD_APP_TAKE), "session id reused after normal traffic must evaluate the new image");
    tu_init(&b, TU_V(0, 4, 0, 0xFF), 1, 1);
    tu_announce(&b, 0x35, d_same, 0, 3);
    b.now += tu_hold_of(d_same) + UPD_IDLE_CLOSE_MS;
    (void)upd_app_poll(&b.u, b.now);
    tu_announce(&b, 0x35, d_new, 0, 3);
    CHECK(tu_has(&b, UPD_APP_TAKE), "session id reused after timeout must evaluate the new image");

    /* PING / confirm */
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    CHECK(b.nact == 0, "PING before startup checks complete: not yet");
    upd_app_frame_end(&b.u, 1, 0, ++b.now);
    CHECK(upd_app_poll(&b.u, b.now) == UPD_APP_NONE, "normal traffic is not a startup check");
    b.u.startup_ok = 1;
    CHECK(upd_app_poll(&b.u, b.now) == UPD_APP_NONE, "normal traffic cancels stale PING authority");
    b.u.transparent = 1;
    b.nact = 0;
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    CHECK(tu_has(&b, UPD_APP_CONFIRM), "startup checks + fresh idle PING: CONFIRM");
    CHECK(upd_app_poll(&b.u, b.now) == UPD_APP_NONE, "... once");
    upd_app_confirmed(&b.u, 0);
    CHECK(upd_app_poll(&b.u, b.now) == UPD_APP_NONE, "write failed: not again without a new PING");
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    CHECK(tu_has(&b, UPD_APP_CONFIRM), "the next PING tries again");
    upd_app_confirmed(&b.u, 1);
    CHECK(b.u.info.confirmed && b.u.info.st_next == 3, "confirmed; the next status record moves on");
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 1);
    upd_app_frame_end(&b.u, 1, 0, ++b.now);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    CHECK(b.nact == 0 && upd_app_poll(&b.u, b.now) == UPD_APP_NONE, "already confirmed: nothing written");

    /* The first PING bounds dispatch even when time wraps or copies arrive. */
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 0);
    b.u.transparent = 1;
    b.now = 0xfffffff0u;
    tu_upd(&b, UPD_PING, 0, 0, 0, 0);
    {
        uint32_t first = b.u.ping_ms;
        b.now += 30u;
        tu_upd(&b, UPD_PING, 0, 0, 0, 0);
        CHECK(b.u.ping_ms == first, "copies do not extend the dispatch deadline");
        CHECK(upd_app_confirm_live(&b.u, first + UPD_DISPATCH_MS - 1u), "last dispatch millisecond");
        CHECK(!upd_app_confirm_live(&b.u, first + UPD_DISPATCH_MS), "dispatch expires exactly at bound");
        b.u.startup_ok = 1;
        CHECK(upd_app_poll(&b.u, first + UPD_DISPATCH_MS) == UPD_APP_NONE && !b.u.ping_seen,
              "late startup never authorizes a confirmation");
        b.now = first + UPD_DISPATCH_MS;
        tu_upd(&b, UPD_PING, 0, 0, 0, 0);
        CHECK(tu_has(&b, UPD_APP_CONFIRM), "fresh idle PING retries after expiry");
    }

    /* ABI-3: elapsed uptime and normal traffic never substitute for idle PING. */
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 0);
    upd_app_frame_end(&b.u, 1, 0, ++b.now);
    CHECK(upd_app_poll(&b.u, 120000) == UPD_APP_NONE, "normal traffic without idle PING never confirms");
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 1);
    b.u.ping_seen = b.u.startup_ok = 1;
    CHECK(upd_app_poll(&b.u, ++b.now) == UPD_APP_NONE, "already confirmed: nothing written");
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 0, 0);
    b.u.ping_seen = b.u.startup_ok = 1;
    CHECK(upd_app_poll(&b.u, ++b.now) == UPD_APP_NONE, "standalone build cannot confirm");
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 0);
    b.u.info.st_next = UPD_ST_COUNT;
    b.u.ping_seen = b.u.startup_ok = 1;
    CHECK(upd_app_poll(&b.u, ++b.now) == UPD_APP_NONE, "full status area cannot confirm");

    /* behind another hardware's bootloader: main.c dark, main_ur_digital_m.c's delay line off */
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 1, 1);
    CHECK(!upd_app_wrong_hw(&b.u.info, UPD_HW_ID_D2A_PLUS_5CH) && upd_app_wrong_hw(&b.u.info, UPD_HW_ID_DP_ULTRA_RX) &&
          upd_app_wrong_hw(&b.u.info, UPD_HW_ID_DP_ULTRA_DIGITAL), "B2: the bootloader's hardware id (1) against the "
          "build's");
    tu_init(&b, TU_V(0, 5, 0, 0xFF), 0, 0);
    CHECK(!upd_app_wrong_hw(&b.u.info, UPD_HW_ID_D2A_PLUS_5CH) &&
          !upd_app_wrong_hw(&b.u.info, UPD_HW_ID_DP_ULTRA_DIGITAL), "B2: no bootloader (standalone): never wrong");

    /* the descriptor page reader */
    {
        uint8_t page[UPD_PAGE_BYTES], bl[32];
        upd_app_info_t info;
        uint32_t i;
        for (i = 0; i < UPD_PAGE_BYTES; i++) {
            page[i] = (uint8_t)((i & 1u) ? 0xE3u : 0x39u);   /* erased, as the RM says it reads */
        }
        memcpy(page, d_same, 64);
        page[64] = (uint8_t)(UPD_ST_VERIFIED & 0xFFu);
        page[65] = (uint8_t)(UPD_ST_VERIFIED >> 8);
        memcpy(bl, &tu_boot, sizeof(tu_boot));     /* bootloader 1, hardware 1, revision 3 (rev 0.4) */
        upd_app_read_info(&info, page, bl);
        CHECK(info.desc_ok && info.version == TU_V(0, 4, 0, 0xFF) && !info.confirmed && info.st_next == 1 &&
              info.bl_version == 1 && info.hw_rev == UPD_REV_D2A_R04, "descriptor page: version, records, bootloader");
        page[66] = (uint8_t)(UPD_ST_CONFIRMED & 0xFFu);
        page[67] = (uint8_t)(UPD_ST_CONFIRMED >> 8);
        memset(page + 68, 0xFF, 4);         /* the other erased reading */
        upd_app_read_info(&info, page, bl);
        CHECK(info.confirmed && info.st_next == 2, "CONFIRMED found, 0xFFFF counts as erased too");
        bl[0] = 0;
        upd_app_read_info(&info, page, bl);
        CHECK(info.bl_version == 0 && !info.desc_ok, "no bootloader info: standalone");
    }

    test_update_hold(d_hw, d_new);
    test_update_chain();
}
