/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Bootloader host tests (bootloader\test\build_bl_test.cmd, MSVC).
 *
 * A NOR flash model (erased = 0xE339E339 as RM 32.5.4, no double programming, the bootloader / scene log /
 * settings areas must never be touched, power cuts at any flash operation), a simulated board (the bootloader's
 * boot flow as bl_main.c runs it, plus a minimal app model), a host that sends update sessions with frame loss and
 * bit errors, and a waveform model for the streaming decoder. */
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../bl_core.h"
#include "../bl_rx.h"

#define RCC_FLAG_TEST_IWDG 0x20000000u        /* RCC_IWDGRSTF */

static int fails, checks;
#define CHECK(c, ...)                                                                                              \
    do {                                                                                                           \
        checks++;                                                                                                  \
        if (!(c)) {                                                                                                \
            fails++;                                                                                               \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                                                          \
            printf(__VA_ARGS__);                                                                                   \
            printf("\n");                                                                                          \
        }                                                                                                          \
    } while (0)

static uint32_t rng = 0x12345678u;
static uint32_t rnd(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

/* =============================================================================================== flash model */
#define FL_BASE 0x08000000u
#define FL_SIZE 0x10000u
static uint8_t fl[FL_SIZE];
static uint8_t fl_prog[FL_SIZE / 2u];     /* half-word programmed since its erase (or torn) */
static uint32_t fl_ops, fl_cut_at;        /* power cut when fl_ops reaches fl_cut_at (0 = never) */
static uint32_t fl_busy_us;               /* flash time since the simulator last looked */
static int fl_power = 1, fl_violations;
static jmp_buf cut_jmp;

static void fl_fill_erased(uint32_t off, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        fl[off + i] = (uint8_t)(((off + i) & 1u) ? 0xE3u : 0x39u);
        fl_prog[(off + i) / 2u] = 0;
    }
}

static void fl_torn(uint32_t off, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        fl[off + i] = (uint8_t)rnd();
        fl_prog[(off + i) / 2u] = 1;
    }
}

static void fl_op(uint32_t off, uint32_t n)
{
    fl_ops++;
    if (fl_cut_at != 0u && fl_ops == fl_cut_at) {
        fl_torn(off, n);                  /* half done: anything */
        longjmp(cut_jmp, 1);
    }
}

static void fl_violation(const char *what, uint32_t addr)
{
    fl_violations++;
    printf("  flash violation: %s at 0x%08X\n", what, (unsigned)addr);
}

const uint8_t *bl_plat_ptr(uint32_t addr) { return &fl[addr - FL_BASE]; }

int bl_plat_erase_sector(uint32_t a)
{
    if ((a & 4095u) != 0u || a < UPD_APP_DESC || a + 4096u > UPD_APP_END) {
        fl_violation("sector erase", a);
        return -1;
    }
    fl_op(a - FL_BASE, 4096u);
    fl_fill_erased(a - FL_BASE, 4096u);
    fl_busy_us += 16000u;
    return 0;
}

int bl_plat_erase_page(uint32_t a)
{
    if ((a & 255u) != 0u || a < UPD_APP_DESC || a + 256u > UPD_APP_END) {
        fl_violation("page erase", a);
        return -1;
    }
    fl_op(a - FL_BASE, 256u);
    fl_fill_erased(a - FL_BASE, 256u);
    fl_busy_us += 16000u;
    return 0;
}

int bl_plat_program_page(uint32_t a, const uint8_t *d)
{
    uint32_t i, off = a - FL_BASE;
    if ((a & 255u) != 0u || a < UPD_APP_BODY || a + 256u > UPD_APP_END) {
        fl_violation("page program", a);
        return -1;
    }
    for (i = 0; i < 128u; i++) {
        if (fl_prog[off / 2u + i]) {
            fl_violation("page programmed twice", a);
            return -1;
        }
    }
    fl_op(off, 256u);
    memcpy(&fl[off], d, 256u);
    memset(&fl_prog[off / 2u], 1, 128u);
    fl_busy_us += 2000u;
    return 0;
}

int bl_plat_program_half(uint32_t a, uint16_t v)
{
    uint32_t off = a - FL_BASE;
    if ((a & 1u) != 0u || a < UPD_APP_DESC || a >= UPD_APP_BODY) {
        fl_violation("half-word program outside the descriptor page", a);
        return -1;
    }
    if (fl_prog[off / 2u]) {
        fl_violation("half-word programmed twice", a);
        return -1;
    }
    fl_op(off, 2u);
    fl[off] = (uint8_t)v;
    fl[off + 1u] = (uint8_t)(v >> 8);
    fl_prog[off / 2u] = 1;
    fl_busy_us += 60u;
    return 0;
}

uint32_t bl_plat_crc32(uint32_t addr, uint32_t nwords)
{
    return upd_crc32_words(0xFFFFFFFFu, &fl[addr - FL_BASE], nwords);
}

int bl_plat_power_ok(void) { return fl_power; }
void bl_plat_kick(void) {}

/* =============================================================================================== images */
typedef struct {
    uint8_t desc[UPD_DESC_BYTES];
    uint8_t body[UPD_APP_BODY_MAX];
    uint32_t len;
} image_t;

static void make_image(image_t *im, uint32_t len, uint32_t version, uint8_t hw_id, uint16_t revmask, uint8_t min_bl,
                       uint32_t seed)
{
    uint32_t i, s = seed | 1u;
    memset(im, 0, sizeof(*im));
    im->len = len;
    for (i = 0; i < len; i++) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        im->body[i] = (uint8_t)s;
    }
    memcpy(im->desc, "QDU5", 4);
    im->desc[UPD_D_DESC_VERSION] = UPD_DESC_VERSION;
    im->desc[UPD_D_HW_ID] = hw_id;
    upd_put16(im->desc + UPD_D_HW_REV_MASK, revmask);
    upd_put32(im->desc + UPD_D_FW_VERSION, version);
    upd_put32(im->desc + UPD_D_BODY_LEN, len);
    upd_put32(im->desc + UPD_D_BODY_CRC, upd_crc32_words(0xFFFFFFFFu, im->body, len / 4u));
    upd_put32(im->desc + UPD_D_LOAD_ADDR, UPD_APP_BODY);
    upd_put32(im->desc + UPD_D_ENTRY, UPD_APP_ENTRY);
    im->desc[UPD_D_MIN_BL] = min_bl;
    memcpy(im->desc + UPD_D_BUILD_ID, "test-image", 10);
    upd_put32(im->desc + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, im->desc, 15u));
}

#define V(a, b, c, pre) (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | ((uint32_t)(c) << 8) | (pre))
#define REV_OK (1u << UPD_REV_D2A_R04)

/* production: chip erased, bootloader + app + VERIFIED + CONFIRMED written by the programmer */
static void factory(const image_t *im)
{
    uint32_t i;
    fl_fill_erased(0, FL_SIZE);
    memset(fl, 0x5A, UPD_BL_SIZE);                    /* the bootloader itself (never touched) */
    memset(fl_prog, 1, UPD_BL_SIZE / 2u);
    memcpy(&fl[UPD_APP_DESC - FL_BASE], im->desc, UPD_DESC_BYTES);
    memcpy(&fl[UPD_APP_BODY - FL_BASE], im->body, im->len);
    for (i = 0; i < UPD_DESC_BYTES / 2u; i++) {
        fl_prog[(UPD_APP_DESC - FL_BASE) / 2u + i] = 1;
    }
    for (i = 0; i < (im->len + 1u) / 2u; i++) {
        fl_prog[(UPD_APP_BODY - FL_BASE) / 2u + i] = 1;
    }
    fl[UPD_APP_DESC - FL_BASE + 64] = (uint8_t)(UPD_ST_VERIFIED & 0xFFu);
    fl[UPD_APP_DESC - FL_BASE + 65] = (uint8_t)(UPD_ST_VERIFIED >> 8);
    fl[UPD_APP_DESC - FL_BASE + 66] = (uint8_t)(UPD_ST_CONFIRMED & 0xFFu);
    fl[UPD_APP_DESC - FL_BASE + 67] = (uint8_t)(UPD_ST_CONFIRMED >> 8);
    fl_prog[(UPD_APP_DESC - FL_BASE) / 2u + 32u] = 1;
    fl_prog[(UPD_APP_DESC - FL_BASE) / 2u + 33u] = 1;
    memset(&fl[0xF700], 0x77, 0x900);                 /* scene log + settings: must survive */
}

static int flash_has(const image_t *im)
{
    return memcmp(&fl[UPD_APP_DESC - FL_BASE], im->desc, UPD_DESC_BYTES) == 0 &&
           memcmp(&fl[UPD_APP_BODY - FL_BASE], im->body, im->len) == 0;
}

static int keep_areas_intact(void)
{
    uint32_t i;
    for (i = 0; i < UPD_BL_SIZE; i++) {
        if (fl[i] != 0x5A) {
            return 0;
        }
    }
    for (i = 0xF700; i < FL_SIZE; i++) {
        if (fl[i] != 0x77) {
            return 0;
        }
    }
    return 1;
}

/* =============================================================================================== board model */
typedef struct {
    int in_app;                /* the app runs (else the bootloader listens) */
    int app_half;              /* app model: ANNOUNCE halves seen */
    int app_takes;             /* app model: reboot into the bootloader on an ANNOUNCE */
    int app_confirmed;
    uint64_t now_us, busy_until_us;
    uint32_t boots, sessions_entered, exits, jumps;
    bl_action_t last_act;
} board_t;
static board_t B;
static bl_app_t BA;
static bl_sess_t BS;
static upd_handover_t HO;

static uint32_t ms(void) { return (uint32_t)(B.now_us / 1000u); }

/* the boot flow of bl_main.c; soft resets loop here */
static void board_boot(uint32_t reset)
{
    for (;;) {
        bl_action_t act;
        B.boots++;
        B.now_us += 3000u;                            /* reset + code load (gate closed meanwhile) */
        bl_app_read(&BA);
        act = bl_decide(&BA, &HO, reset, 0);
        if (act == BL_ACT_VERIFY) {
            fl_busy_us = 0;
            act = bl_after_verify(&BA);
            B.now_us += fl_busy_us;
            if (act == BL_ACT_JUMP) {
                HO.request = UPD_REQ_BOOT_ONCE;
                bl_ho_seal(&HO);
                reset = BL_RST_SOFT;
                continue;
            }
        }
        B.last_act = act;
        if (act == BL_ACT_JUMP) {
            B.in_app = 1;
            B.jumps++;
            B.app_half = 0;
            B.app_confirmed = BA.confirmed;
            return;
        }
        if (act == BL_ACT_SESSION) {
            B.sessions_entered++;
        }
        B.in_app = 0;
        bl_sess_init(&BS, act, &BA, ms());
        return;
    }
}

/* flash work runs at HCLK / 2, so bl_main resyncs the decoder after every flash operation; it needs a 100 us gap:
 * frames that end before then are lost (a page program in the host's page pause loses nothing) */
static void board_stall(uint32_t us)
{
    if (us > 0u) {
        B.busy_until_us = B.now_us + us + 100u;
    }
}

static void board_poll(void)
{
    if (B.in_app) {
        return;
    }
    fl_busy_us = 0;
    if (bl_sess_poll(&BS, ms())) {
        B.exits++;
        bl_ho_exit(&HO, &BS);
        B.now_us += fl_busy_us;
        board_boot(BL_RST_SOFT);
    } else {
        board_stall(fl_busy_us);
    }
}

static void app_model(const uint8_t *tail, uint32_t nbits)
{
    if (nbits != UPD_FRAME_BITS || upd_check_frame(tail) != 0) {
        return;
    }
    if (tail[UPD_B_TYPE] == UPD_ANNOUNCE && upd_get16(tail + UPD_B_INDEX) <= 1u) {
        B.app_half |= 1 << upd_get16(tail + UPD_B_INDEX);
        if (B.app_half == 3 && B.app_takes) {
            HO.request = UPD_REQ_ENTER;
            bl_ho_seal(&HO);
            board_boot(BL_RST_SOFT);
        }
    } else if (tail[UPD_B_TYPE] == UPD_PING && !B.app_confirmed && BA.desc_ok) {
        /* the app appends CONFIRMED at the first free record it found at boot */
        if (bl_plat_program_half(UPD_APP_DESC + 2u * (UPD_ST_FIRST + BA.st_next), UPD_ST_CONFIRMED) == 0) {
            BA.st_next++;
        }
        B.app_confirmed = 1;
    }
}

/* one frame as the board sees it at its end; tail = last 48 bytes right-aligned */
static void board_frame(const uint8_t *tail, uint32_t nbits)
{
    if (B.now_us < B.busy_until_us) {
        return;                                       /* busy with flash work: the frame is lost */
    }
    if (B.in_app) {
        app_model(tail, nbits);
        return;
    }
    fl_busy_us = 0;
    bl_sess_frame(&BS, nbits, tail, ms());
    board_stall(fl_busy_us);
}

/* =============================================================================================== host model */
static uint32_t loss_ppm, flip_ppm;                   /* frame loss / single-bit errors per million frames */
static uint32_t host_frames, host_lost;

static void host_frame_bits(const uint8_t *f, uint32_t nbytes, uint32_t gap_us)
{
    uint8_t tail[48];
    uint32_t nbits = nbytes * 8u;
    memset(tail, 0, sizeof(tail));
    memcpy(tail + 48u - nbytes, f, nbytes);
    B.now_us += (uint64_t)nbits * 1325u / 1000u;      /* 1.325 us per bit */
    host_frames++;
    if (rnd() % 1000000u < loss_ppm) {
        host_lost++;
    } else {
        if (rnd() % 1000000u < flip_ppm) {
            uint32_t b = rnd() % nbits;
            tail[48u - nbytes + b / 8u] ^= (uint8_t)(0x80u >> (b % 8u));
        }
        board_frame(tail, nbits);
    }
    B.now_us += 300u + gap_us;
    board_poll();
}

static void host_upd(uint8_t type, uint8_t session, uint32_t index, uint8_t pass, uint8_t flags,
                     const uint8_t *data, uint32_t gap_us)
{
    uint8_t f[UPD_FRAME_BYTES];
    upd_build_frame(f, type, session, index, pass, flags, data, (uint32_t)B.now_us);
    host_frame_bits(f, UPD_FRAME_BYTES, gap_us);
}

static void host_idle(uint32_t us)
{
    while (us > 0u) {
        uint32_t step = us > 1000u ? 1000u : us;
        B.now_us += step;
        us -= step;
        board_poll();
    }
}

/* a normal Ultra frame (wire VERSION 2): n slots of NOP + the trailer */
static void host_normal(uint32_t nslots)
{
    uint8_t f[48];
    uint32_t nbytes = 32u * nslots + 8u;
    uint8_t *h;
    if (nbytes > 48u) {
        nbytes = 48u;                                 /* only the tail matters to the bootloader */
    }
    memset(f, 0, sizeof(f));
    h = f + nbytes - 8u;
    upd_put16(h, UPD_HDR_MAGIC);
    h[2] = (uint8_t)(UPD_HDR_VERSION << 4);
    upd_put32(h + 3, (uint32_t)B.now_us);
    h[7] = upd_crc8(h, 7);
    {
        uint8_t tail[48];
        uint32_t nbits = (32u * nslots + 8u) * 8u;
        memset(tail, 0, sizeof(tail));
        memcpy(tail + 48u - nbytes, f, nbytes);
        B.now_us += (uint64_t)nbits * 1325u / 1000u;
        board_frame(tail, nbits);
        B.now_us += 300u;
        board_poll();
    }
}

static void host_ping(void)
{
    uint32_t i;
    for (i = 0; i < 20u; i++) {
        host_upd(UPD_PING, 0, 0, 0, 0, 0, 0);
    }
}

typedef struct {
    uint32_t lead_ms, passes, end_ms, page_pause_us, announce_every;
    uint8_t flags;
} sess_opts_t;
static const sess_opts_t SESS_DEFAULT = { 500u, 3u, 250u, 2700u, 4u, 0u };

static void host_session(const image_t *im, uint8_t session, const sess_opts_t *o)
{
    uint64_t t0 = B.now_us;
    uint32_t i = 0, p, k, npages = (im->len + 255u) / 256u;
    uint8_t endd[32];
    while (B.now_us - t0 < (uint64_t)o->lead_ms * 1000u) {
        host_upd(UPD_ANNOUNCE, session, i & 1u, 0, o->flags, im->desc + 32u * (i & 1u), 0);
        i++;
    }
    for (p = 0; p < o->passes; p++) {
        for (k = 0; k < npages; k++) {
            uint32_t page = (p & 1u) ? npages - 1u - k : k, c;
            for (c = 0; c < 8u; c++) {
                uint32_t idx = page * 8u + c, off = idx * 32u;
                uint8_t d[32];
                if (off >= im->len) {
                    break;
                }
                memset(d, 0xFF, sizeof(d));
                memcpy(d, im->body + off, im->len - off < 32u ? im->len - off : 32u);
                host_upd(UPD_DATA, session, idx, (uint8_t)((p << 4) | o->passes), 0, d,
                         (c == 7u || off + 32u >= im->len) ? o->page_pause_us : 0u);
            }
            if (o->announce_every != 0u && k % o->announce_every == o->announce_every - 1u) {
                host_upd(UPD_ANNOUNCE, session, 0, 0, o->flags, im->desc, 0);
                host_upd(UPD_ANNOUNCE, session, 1, 0, o->flags, im->desc + 32, 0);
            }
        }
    }
    memset(endd, 0, sizeof(endd));
    memcpy(endd, im->desc + UPD_D_BODY_CRC, 4);
    memcpy(endd + 4, im->desc + UPD_D_BODY_LEN, 4);
    t0 = B.now_us;
    while (B.now_us - t0 < (uint64_t)o->end_ms * 1000u) {
        host_upd(UPD_END, session, 0, 0, 0, endd, 0);
    }
}

/* after a session: what the hub does (1 s wait, resync = normal frames, PING burst) */
static void host_after(void)
{
    uint32_t i;
    host_idle(1000000u);
    for (i = 0; i < 20u; i++) {
        host_normal(2);
    }
    host_ping();
    for (i = 0; i < 10u; i++) {
        host_normal(2);
    }
}

static void board_power_on(void)
{
    uint32_t i;
    for (i = 0; i < sizeof(HO) / 4u; i++) {
        ((uint32_t *)&HO)[i] = rnd();                 /* RAM after power loss */
    }
    B.busy_until_us = 0;
    board_boot(BL_RST_POWER);
}

static void board_reset_model(void)
{
    memset(&B, 0, sizeof(B));
    B.app_takes = 1;
    loss_ppm = flip_ppm = 0;
    fl_power = 1;
    fl_cut_at = 0;
    fl_violations = 0;
}

static int running(const image_t *im)
{
    bl_app_t a;
    bl_app_read(&a);
    return B.in_app && flash_has(im) && a.desc_ok && a.verified;
}

/* =============================================================================================== tests */
static void test_erase_plan(void)
{
    uint32_t lens[] = { 4, 252, 256, 3836, 3840, 3844, 4096, 30548, 43400, UPD_APP_BODY_MAX - 4, UPD_APP_BODY_MAX };
    uint32_t i;
    printf("erase plan\n");
    for (i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
        uint32_t ops[32], n = bl_erase_plan(lens[i], ops, 32), k, covered_end = UPD_APP_DESC, ok = 1;
        CHECK(n > 0 && ops[0] == UPD_APP_DESC, "len %u: first op erases the descriptor sector", (unsigned)lens[i]);
        for (k = 0; k < n; k++) {
            uint32_t a = ops[k] & ~1u, sz = (ops[k] & 1u) ? 256u : 4096u;
            ok &= (a == covered_end) && (a + sz <= UPD_APP_END) && (sz == 256u || (a & 4095u) == 0u);
            covered_end = a + sz;
        }
        CHECK(ok && covered_end >= UPD_APP_BODY + lens[i], "len %u: contiguous, inside the app area, covers the body",
              (unsigned)lens[i]);
        CHECK(n <= 24u, "len %u: %u ops fit the session's array", (unsigned)lens[i], (unsigned)n);
    }
}

static void test_desc_check(void)
{
    image_t *im = (image_t *)malloc(sizeof(image_t));
    uint8_t d[64];
    printf("descriptor checks\n");
    make_image(im, 1024, V(1, 0, 0, 0xFF), 1, REV_OK, 1, 1);
    CHECK(bl_desc_check(im->desc) == 0, "valid");
    memcpy(d, im->desc, 64); d[3] = 'X';
    CHECK(bl_desc_check(d) == -1, "magic");
    memcpy(d, im->desc, 64); d[20] ^= 1;
    CHECK(bl_desc_check(d) == -1, "CRC");
#define FIELD_TEST(mod, what)                                                                                      \
    do {                                                                                                           \
        memcpy(d, im->desc, 64);                                                                                   \
        mod;                                                                                                       \
        upd_put32(d + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, d, 15u));                                            \
        CHECK(bl_desc_check(d) == -2, "rejects %s", what);                                                         \
    } while (0)
    FIELD_TEST(d[UPD_D_HW_ID] = 2, "another hardware id");
    FIELD_TEST(upd_put16(d + UPD_D_HW_REV_MASK, 1u << (UPD_REV_D2A_R04 - 1u)), "another revision");
    FIELD_TEST(upd_put32(d + UPD_D_BODY_LEN, UPD_APP_BODY_MAX + 4u), "a body that doesn't fit");
    FIELD_TEST(upd_put32(d + UPD_D_BODY_LEN, 1022), "an odd length");
    FIELD_TEST(upd_put32(d + UPD_D_BODY_LEN, 0), "length 0");
    FIELD_TEST(upd_put32(d + UPD_D_LOAD_ADDR, 0x08000000u), "another load address");
    FIELD_TEST(d[UPD_D_MIN_BL] = BL_VERSION + 1u, "a newer bootloader requirement");
    FIELD_TEST(d[UPD_D_DESC_VERSION] = 2, "descriptor version 2");
    free(im);
}

static void test_decide(void)
{
    bl_app_t a;
    upd_handover_t ho;
    printf("boot decision\n");
    memset(&a, 0, sizeof(a));
    memset(&ho, 0xA5, sizeof(ho));
    CHECK(bl_decide(&a, &ho, BL_RST_POWER, 0) == BL_ACT_RECOVERY, "no app -> recovery");
    a.desc_ok = 1;
    CHECK(bl_decide(&a, &ho, BL_RST_POWER, 0) == BL_ACT_VERIFY, "not verified -> verify");
    a.verified = 1;
    CHECK(bl_decide(&a, &ho, BL_RST_POWER, 0) == BL_ACT_TRIAL, "unconfirmed -> trial window");
    a.confirmed = 1;
    CHECK(bl_decide(&a, &ho, BL_RST_POWER, 0) == BL_ACT_JUMP, "confirmed -> jump");
    ho.request = UPD_REQ_ENTER;
    bl_ho_seal(&ho);
    CHECK(bl_decide(&a, &ho, BL_RST_SOFT, 0) == BL_ACT_SESSION, "requested -> session");
    CHECK(ho.request == UPD_REQ_NONE, "request consumed");
    ho.request = UPD_REQ_ENTER;
    bl_ho_seal(&ho);
    CHECK(bl_decide(&a, &ho, BL_RST_POWER, 0) == BL_ACT_JUMP, "a request does not survive a power-on");
    a.confirmed = 0;
    ho.request = UPD_REQ_BOOT_ONCE;
    bl_ho_seal(&ho);
    CHECK(bl_decide(&a, &ho, BL_RST_SOFT, 0) == BL_ACT_JUMP, "boot once skips the trial window");
    CHECK(bl_decide(&a, &ho, BL_RST_SOFT, 0) == BL_ACT_TRIAL, "... only once");
    a.confirmed = 1;
    memset(&ho, 0, sizeof(ho));
    bl_ho_seal(&ho);
    CHECK(bl_decide(&a, &ho, BL_RST_WATCHDOG, RCC_FLAG_TEST_IWDG) == BL_ACT_JUMP, "watchdog 1");
    CHECK(bl_decide(&a, &ho, BL_RST_WATCHDOG, 0) == BL_ACT_JUMP, "watchdog 2");
    CHECK(bl_decide(&a, &ho, BL_RST_WATCHDOG, 0) == BL_ACT_RECOVERY, "watchdog 3 -> recovery");
    CHECK(bl_decide(&a, &ho, BL_RST_SOFT, 0) == BL_ACT_RECOVERY, "stays in recovery until a power-on");
    CHECK((ho.reset_flags & RCC_FLAG_TEST_IWDG) != 0u, "reset flags kept for the app");
    CHECK(bl_decide(&a, &ho, BL_RST_POWER, 0) == BL_ACT_JUMP, "power-on clears the watchdog count");
}

static image_t im_old, im_new, im_other;

static void test_clean_update(void)
{
    uint32_t i;
    printf("clean update (30.5 KB, 3 passes)\n");
    board_reset_model();
    make_image(&im_old, 30548, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 11);
    make_image(&im_new, 30548, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 22);
    factory(&im_old);
    board_power_on();
    CHECK(B.in_app && B.last_act == BL_ACT_JUMP, "factory board boots straight into the app");
    for (i = 0; i < 5; i++) {
        host_normal(3);
    }
    {
        uint64_t t0 = B.now_us;
        host_session(&im_new, 0x42, &SESS_DEFAULT);
        printf("  session %.2f s, %u frames\n", (double)(B.now_us - t0) / 1e6, (unsigned)host_frames);
    }
    CHECK(B.sessions_entered == 1, "the app handed over once (%u)", (unsigned)B.sessions_entered);
    CHECK(HO.result == UPD_RES_UPDATED && HO.result_version == V(0, 5, 0, 0xFF), "result UPDATED 0.5.0 for the app");
    CHECK(running(&im_new), "new app verified and running");
    {
        bl_app_t a;
        bl_app_read(&a);
        CHECK(!a.confirmed && a.trials == 1, "first boot was a trial (%u trials)", (unsigned)a.trials);
    }
    host_after();
    {
        bl_app_t a;
        bl_app_read(&a);
        CHECK(a.confirmed, "confirmed after the PING");
    }
    board_power_on();
    CHECK(B.in_app && B.last_act == BL_ACT_JUMP, "next power-on: no window");
    CHECK(keep_areas_intact(), "bootloader, scene log and settings untouched");
    CHECK(fl_violations == 0, "no flash rule broken");
}

static void test_loss(void)
{
    static const uint32_t loss[] = { 1000, 10000, 50000, 100000 };
    uint32_t li, run;
    printf("frame loss (30.5 KB, 3 passes, 20 runs each, + 1 %% bit errors)\n");
    for (li = 0; li < 4u; li++) {
        uint32_t ok = 0, rec = 0, bad = 0;
        for (run = 0; run < 20u; run++) {
            bl_app_t a;
            board_reset_model();
            rng = 0x9E3779B9u * (run + 1u) + li;
            make_image(&im_old, 30548, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 100 + run);
            make_image(&im_new, 30548, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 200 + run);
            factory(&im_old);
            board_power_on();
            loss_ppm = loss[li];
            flip_ppm = 10000;
            host_session(&im_new, (uint8_t)(run + 1u), &SESS_DEFAULT);
            loss_ppm = flip_ppm = 0;
            host_after();
            bl_app_read(&a);
            if (running(&im_new)) {
                ok++;
            } else if (!B.in_app && BS.mode == BL_ACT_RECOVERY && !a.desc_ok) {
                rec++;
                if (rec <= 2u) {
                    uint32_t u = 0, q;
                    for (q = 0; q < BL_POOL_PAGES; q++) {
                        u += BS.pool[q].used;
                    }
                    printf("    run %u: %u / %u pages, pool %u used, frames %u upd %u bad %u, host lost %u of %u\n",
                           (unsigned)run, (unsigned)BS.nwritten, (unsigned)BS.npages, (unsigned)u, (unsigned)BS.frames,
                           (unsigned)BS.upd_frames, (unsigned)BS.bad_frames, (unsigned)host_lost, (unsigned)host_frames);
                }
            } else {
                bad++;
            }
            CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules (loss %u ppm, run %u)", (unsigned)loss[li],
                  (unsigned)run);
        }
        printf("  loss %4.1f %%: %u updated, %u in recovery (retry), %u other\n", loss[li] / 10000.0, (unsigned)ok,
               (unsigned)rec, (unsigned)bad);
        CHECK(bad == 0, "every board ends updated or in recovery");
        if (loss[li] <= 10000u) {
            CHECK(ok == 20u, "loss %u ppm: all updated", (unsigned)loss[li]);
        }
    }
}

static void test_rejects(void)
{
    static uint8_t snap[FL_SIZE];
    struct { image_t *im; uint8_t flags; uint32_t result; const char *what; } cases[6];
    static image_t im_same, im_older, im_hw, im_big, im_minbl, im_rev;
    uint32_t i;
    printf("rejected and skipped images\n");
    make_image(&im_old, 20000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 31);
    im_same = im_old;
    make_image(&im_older, 20000, V(0, 4, 9, 0xFF), 1, REV_OK, 1, 32);
    make_image(&im_hw, 20000, V(0, 6, 0, 0xFF), 2, REV_OK, 1, 33);
    make_image(&im_rev, 20000, V(0, 6, 0, 0xFF), 1, 1u << (UPD_REV_D2A_R04 - 1u), 1, 34);
    make_image(&im_minbl, 20000, V(0, 6, 0, 0xFF), 1, REV_OK, BL_VERSION + 1u, 35);
    make_image(&im_big, 1024, V(0, 6, 0, 0xFF), 1, REV_OK, 1, 36);
    upd_put32(im_big.desc + UPD_D_BODY_LEN, UPD_APP_BODY_MAX + 256u);
    upd_put32(im_big.desc + UPD_D_CRC, upd_crc32_words(0xFFFFFFFFu, im_big.desc, 15u));
    cases[0].im = &im_same; cases[0].flags = 0; cases[0].result = UPD_RES_SAME; cases[0].what = "same version";
    cases[1].im = &im_older; cases[1].flags = 0; cases[1].result = UPD_RES_REJECTED; cases[1].what = "older version";
    cases[2].im = &im_hw; cases[2].flags = 0; cases[2].result = UPD_RES_REJECTED; cases[2].what = "other hardware";
    cases[3].im = &im_rev; cases[3].flags = 0; cases[3].result = UPD_RES_REJECTED; cases[3].what = "other revision";
    cases[4].im = &im_minbl; cases[4].flags = 0; cases[4].result = UPD_RES_REJECTED; cases[4].what = "newer bootloader";
    cases[5].im = &im_big; cases[5].flags = 0; cases[5].result = UPD_RES_REJECTED; cases[5].what = "too big";
    for (i = 0; i < 6u; i++) {
        sess_opts_t o = SESS_DEFAULT;
        board_reset_model();
        factory(&im_old);
        board_power_on();
        memcpy(snap, fl, FL_SIZE);
        o.passes = 1;
        host_session(cases[i].im, (uint8_t)(10u + i), &o);
        host_idle(500000u);
        CHECK(memcmp(snap, fl, FL_SIZE) == 0, "%s: flash untouched", cases[i].what);
        CHECK(B.in_app && HO.result == cases[i].result, "%s: back in the app with result %u (got %u, in_app %d)",
              cases[i].what, (unsigned)cases[i].result, (unsigned)HO.result, B.in_app);
    }
    {
        sess_opts_t o = SESS_DEFAULT;
        printf("  FORCE installs the same and an older version\n");
        board_reset_model();
        factory(&im_old);
        board_power_on();
        o.flags = UPD_F_FORCE;
        host_session(&im_older, 20, &o);
        host_after();
        CHECK(running(&im_older), "forced downgrade installed");
        host_session(&im_older, 21, &o);
        host_after();
        CHECK(running(&im_older) && HO.result == UPD_RES_UPDATED, "forced reinstall of the same version");
        CHECK(fl_violations == 0, "no flash rule broken");
    }
}

static void test_abort_timeout(void)
{
    sess_opts_t o = SESS_DEFAULT;
    printf("abort, host gone mid-session, late joiner\n");
    board_reset_model();
    make_image(&im_old, 16000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 41);
    make_image(&im_new, 16000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 42);
    factory(&im_old);
    board_power_on();
    o.passes = 0;                                     /* lead-in, then nothing: host gone */
    o.end_ms = 0;
    host_session(&im_new, 50, &o);
    host_idle(4000000u);
    CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY, "host gone after the erase: recovery (dark, listening)");
    CHECK(bl_sess_blink(&BS, 0) == 1 && bl_sess_blink(&BS, 600) == 0, "recovery blinks at 1 Hz");
    host_session(&im_new, 51, &SESS_DEFAULT);
    host_after();
    CHECK(running(&im_new), "the next session installs it");

    /* ABORT */
    board_reset_model();
    factory(&im_old);
    board_power_on();
    o = SESS_DEFAULT;
    o.passes = 1;
    o.end_ms = 0;
    host_session(&im_new, 52, &o);
    host_upd(UPD_ABORT, 52, 0, 0, 0, 0, 0);
    host_idle(100000u);
    CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY, "ABORT after the erase: recovery");

    /* late joiner: the board powers up during pass 1 (misses the lead-in) */
    board_reset_model();
    factory(&im_old);
    B.app_takes = 1;
    {
        /* board off: frames are not delivered until power-on in the middle of pass 1 */
        uint64_t t0;
        sess_opts_t lo = SESS_DEFAULT;
        image_t *im = &im_new;
        uint32_t npages = (im->len + 255u) / 256u, k, c, p, i = 0;
        B.in_app = 1;
        B.busy_until_us = ~0ull;                      /* "busy" = unpowered */
        t0 = B.now_us;
        while (B.now_us - t0 < 500000u) {
            host_upd(UPD_ANNOUNCE, 53, i & 1u, 0, 0, im->desc + 32u * (i & 1u), 0);
            i++;
        }
        for (p = 0; p < lo.passes; p++) {
            for (k = 0; k < npages; k++) {
                uint32_t page = (p & 1u) ? npages - 1u - k : k;
                if (p == 0u && k == npages / 3u) {
                    board_power_on();                 /* powers up here */
                }
                for (c = 0; c < 8u; c++) {
                    uint32_t idx = page * 8u + c, off = idx * 32u;
                    uint8_t d[32];
                    if (off >= im->len) {
                        break;
                    }
                    memset(d, 0xFF, 32);
                    memcpy(d, im->body + off, im->len - off < 32u ? im->len - off : 32u);
                    host_upd(UPD_DATA, 53, idx, 0, 0, d, c == 7u ? 2700u : 0u);
                }
                if (k % 4u == 3u) {
                    host_upd(UPD_ANNOUNCE, 53, 0, 0, 0, im->desc, 0);
                    host_upd(UPD_ANNOUNCE, 53, 1, 0, 0, im->desc + 32, 0);
                }
            }
        }
        {
            uint8_t endd[32];
            memset(endd, 0, 32);
            memcpy(endd, im->desc + UPD_D_BODY_CRC, 4);
            memcpy(endd + 4, im->desc + UPD_D_BODY_LEN, 4);
            for (i = 0; i < 300u; i++) {
                host_upd(UPD_END, 53, 0, 0, 0, endd, 0);
            }
        }
        host_after();
        CHECK(running(&im_new), "late joiner: caught an interleaved ANNOUNCE, finished in the later passes");
    }
    CHECK(fl_violations == 0 && keep_areas_intact(), "no flash rule broken");
}

/* power cut at every flash operation of an update, then power-on and a clean retry */
static void test_power_cuts(uint32_t len, uint32_t stride)
{
    uint32_t total, k, cases = 0, after_ok = 0, ran_bad = 0, old_kept = 0, recov = 0, newer = 0;
    printf("power cut at flash operation k (image %u B, every %u. op)\n", (unsigned)len, (unsigned)stride);
    /* count the operations of a clean run */
    board_reset_model();
    make_image(&im_old, len, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 61);
    make_image(&im_new, len, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 62);
    factory(&im_old);
    board_power_on();
    fl_ops = 0;
    host_session(&im_new, 70, &SESS_DEFAULT);
    host_after();
    total = fl_ops;
    CHECK(running(&im_new), "clean reference run");
    for (k = 1; k <= total; k += stride) {
        bl_app_t a;
        board_reset_model();
        factory(&im_old);
        board_power_on();
        fl_ops = 0;
        fl_cut_at = k;
        if (setjmp(cut_jmp) == 0) {
            host_session(&im_new, 71, &SESS_DEFAULT);
            host_after();
        }
        fl_cut_at = 0;
        cases++;
        board_power_on();                             /* power back */
        bl_app_read(&a);
        if (B.in_app) {
            /* whatever runs must be an intact image: old, or new and checked */
            if (flash_has(&im_old)) {
                old_kept++;
            } else if (flash_has(&im_new)) {
                newer++;
            } else {
                ran_bad++;
            }
        } else {
            recov++;
        }
        host_session(&im_new, 72, &SESS_DEFAULT);
        host_after();
        board_power_on();
        if (running(&im_new)) {
            after_ok++;
        }
        CHECK(fl_violations == 0, "cut at op %u: flash rules", (unsigned)k);
        CHECK(keep_areas_intact(), "cut at op %u: bootloader / scene log / settings intact", (unsigned)k);
    }
    printf("  %u ops in a clean run, %u cut points: after the cut %u ran the old app, %u the new one, %u in recovery,"
           " %u ran a broken image; %u / %u updated by the retry\n", (unsigned)total, (unsigned)cases,
           (unsigned)old_kept, (unsigned)newer, (unsigned)recov, (unsigned)ran_bad, (unsigned)after_ok, (unsigned)cases);
    CHECK(ran_bad == 0, "never runs a broken image");
    CHECK(after_ok == cases, "a retry always succeeds");
}

static void test_trial_and_watchdog(void)
{
    uint32_t i;
    bl_app_t a;
    printf("trial window, strikes, status area, watchdog loop\n");
    board_reset_model();
    make_image(&im_old, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 81);
    make_image(&im_new, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 82);
    factory(&im_old);
    board_power_on();
    host_session(&im_new, 90, &SESS_DEFAULT);
    host_idle(2000000u);                              /* no PING: stays unconfirmed */
    for (i = 0; i < 3u; i++) {                        /* power-ons with frames: the window ends at the first frame */
        uint64_t t0;
        board_power_on();
        CHECK(!B.in_app && BS.mode == BL_ACT_TRIAL, "unconfirmed power-on %u: trial window", (unsigned)i);
        t0 = B.now_us;
        host_normal(2);
        CHECK(B.in_app && B.now_us - t0 < 5000u, "the first Ultra frame ends the window");
    }
    bl_app_read(&a);
    CHECK(a.strikes == 3 && a.trials == 4, "3 strikes recorded (+1 quiet trial after the update): %u / %u",
          (unsigned)a.strikes, (unsigned)a.trials);
    for (i = 0; i < 200u; i++) {                      /* many quiet power-ons */
        board_power_on();
        host_idle(200000u);
    }
    bl_app_read(&a);
    CHECK(a.st_next == UPD_ST_COUNT - BL_ST_RESERVE, "trial records stop %u before the end (%u)",
          (unsigned)BL_ST_RESERVE, (unsigned)a.st_next);
    host_ping();
    bl_app_read(&a);
    CHECK(a.confirmed, "CONFIRMED still fits");
    board_power_on();
    CHECK(B.in_app && B.last_act == BL_ACT_JUMP, "confirmed: no window");
    /* watchdog loop */
    board_boot(BL_RST_WATCHDOG);
    board_boot(BL_RST_WATCHDOG);
    CHECK(B.in_app, "2 watchdog resets: still the app");
    board_boot(BL_RST_WATCHDOG);
    CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY, "3rd watchdog reset: recovery");
    host_normal(3);
    CHECK(BS.gate == BL_GATE_ULTRA, "recovery takes its Ultra slot (gate plan)");
    host_upd(UPD_PING, 0, 0, 0, 0, 0, 0);
    CHECK(BS.gate == BL_GATE_OPEN, "UPD frames pass whole");
    {
        uint8_t ws[48];
        memset(ws, 0x55, sizeof(ws));
        bl_sess_frame(&BS, 40, ws, ms());
        CHECK(BS.gate == BL_GATE_OPEN, "WS frames pass whole");
    }
    board_power_on();
    CHECK(B.in_app, "power-on: the app again");
    CHECK(fl_violations == 0, "no flash rule broken");
}

static void test_verify_catches_bad_body(void)
{
    bl_app_t a;
    printf("a committed image with a bad body never runs\n");
    board_reset_model();
    make_image(&im_old, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 91);
    make_image(&im_new, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 92);
    factory(&im_old);
    board_power_on();
    host_session(&im_new, 95, &SESS_DEFAULT);         /* installs + verifies */
    /* corrupt the body under a valid, unverified descriptor (as a stale read-back would) */
    fl_fill_erased(UPD_APP_DESC - FL_BASE + 64, 192);
    fl[UPD_APP_BODY - FL_BASE + 100] ^= 0xFF;
    board_power_on();
    bl_app_read(&a);
    CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY && !a.desc_ok, "CRC mismatch: descriptor erased, recovery");
}

static void test_low_power(void)
{
    printf("VDD low (PVD): nothing written, done later\n");
    board_reset_model();
    make_image(&im_old, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 93);
    make_image(&im_new, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 94);
    factory(&im_old);
    board_power_on();
    fl_power = 0;
    {
        sess_opts_t o = SESS_DEFAULT;
        o.passes = 1;
        host_session(&im_new, 96, &o);
    }
    host_idle(500000u);
    CHECK(flash_has(&im_old), "low VDD during the whole session: old app untouched");
    fl_power = 1;
    host_session(&im_new, 97, &SESS_DEFAULT);
    host_after();
    CHECK(running(&im_new), "VDD fine: installed");
    CHECK(fl_violations == 0, "no flash rule broken");
}

/* a new session's ANNOUNCE while VDD is low must not hand the running session's pages to the new image */
static void test_switch_low_power(void)
{
    sess_opts_t o = SESS_DEFAULT;
    uint32_t written_before, i;
    uint8_t d[32];
    printf("new session while VDD is low: the running session keeps its id\n");
    board_reset_model();
    make_image(&im_old, 8000, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 121);
    make_image(&im_new, 8000, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 122);
    make_image(&im_other, 8000, V(0, 6, 0, 0xFF), 1, REV_OK, 1, 123);
    factory(&im_old);
    board_power_on();
    o.passes = 0;
    o.end_ms = 0;
    host_session(&im_new, 0x41, &o);                  /* lead-in only: accepted, erased, receiving */
    CHECK(!B.in_app && BS.state == BL_S_RECEIVE && BS.session == 0x41, "receiving session 0x41");
    written_before = BS.nwritten;
    fl_power = 0;
    host_upd(UPD_ANNOUNCE, 0x42, 0, 0, 0, im_other.desc, 0);
    host_upd(UPD_ANNOUNCE, 0x42, 1, 0, 0, im_other.desc + 32, 0);
    CHECK(BS.session == 0x41, "VDD low: still session 0x41 (got 0x%02X)", BS.session);
    for (i = 0; i < 8u; i++) {                        /* the new session's chunks for page 0 */
        memcpy(d, im_other.body + 32u * i, 32);
        host_upd(UPD_DATA, 0x42, i, 0, 0, d, i == 7u ? 2700u : 0u);
    }
    CHECK(BS.nwritten == written_before, "the other image's chunks were not taken");
    fl_power = 1;
    host_session(&im_other, 0x43, &SESS_DEFAULT);
    host_after();
    CHECK(running(&im_other), "VDD fine again: the new session installs");
    CHECK(fl_violations == 0, "no flash rule broken");
}

/* ------------------------------------------------------------------------------------------------ decoder */
typedef struct {
    uint16_t *w;
    uint32_t nw, cap, bit;     /* bit: samples in the current (last) word */
    double frac, fs;
} wave_t;

static void wave_put(wave_t *wv, uint32_t level, double ns)
{
    double s = ns * 1e-9 * wv->fs + wv->frac;
    uint32_t n = (uint32_t)s, i;
    wv->frac = s - n;
    for (i = 0; i < n; i++) {
        if (wv->bit == 0u) {
            if (wv->nw == wv->cap) {
                wv->cap = wv->cap ? wv->cap * 2u : 4096u;
                wv->w = (uint16_t *)realloc(wv->w, wv->cap * 2u);
            }
            wv->w[wv->nw++] = 0;
        }
        if (level) {
            wv->w[wv->nw - 1u] |= (uint16_t)(0x8000u >> wv->bit);
        }
        wv->bit = (wv->bit + 1u) & 15u;
    }
}

static double jit(double ns) { return ns * ((int)(rnd() % 2001u) - 1000) / 1000.0; }

/* the hub's '0' high / low (ns): 250 / 1050 since the 50-board defaults, 340 / 960 before */
static double wv_h0 = 250.0, wv_l0 = 1050.0;

/* frame bits at the hub's timing with edge jitter; hiccup: a long low inside the frame at bit `hic` */
static void wave_frame(wave_t *wv, const uint8_t *bytes, uint32_t nbits, double j, uint32_t hic, double hic_ns)
{
    uint32_t b;
    for (b = 0; b < nbits; b++) {
        int one = (bytes[b / 8u] >> (7u - b % 8u)) & 1;
        double h = one ? 700.0 : wv_h0, l = one ? 650.0 : wv_l0, e = jit(j);
        wave_put(wv, 1, h + e);
        wave_put(wv, 0, l - e + (b == hic ? hic_ns : 0.0));
    }
}

typedef struct {
    uint32_t n, nbits[64];
    uint8_t tail[64][48], bad[64];
} got_t;

static void wave_decode(const wave_t *wv, got_t *g, int resync_at)
{
    bl_rx_t r;
    static const bl_rx_timing_t t = BL_RX_TIMING_36MHZ;
    uint32_t pos = 0;
    bl_rx_init(&r, &t);
    g->n = 0;
    while (pos < wv->nw) {
        bl_frame_t f;
        uint32_t n = 1u + rnd() % BL_RX_MAX_FEED;
        if (n > wv->nw - pos) {
            n = wv->nw - pos;
        }
        if (resync_at >= 0 && pos <= (uint32_t)resync_at && pos + n > (uint32_t)resync_at) {
            bl_rx_resync(&r);
        }
        if (bl_rx_feed(&r, wv->w + pos, n, &f) && g->n < 64u) {
            g->nbits[g->n] = f.nbits;
            g->bad[g->n] = f.bad;
            memcpy(g->tail[g->n], f.tail, 48);
            g->n++;
        }
        pos += n;
    }
}

static int tail_matches(const uint8_t *tail, const uint8_t *bytes, uint32_t nbits)
{
    uint32_t k = nbits < 384u ? nbits : 384u, j;
    for (j = 0; j < k; j++) {
        uint32_t sb = nbits - k + j, db = 384u - k + j;
        if (((tail[db / 8u] >> (7u - db % 8u)) & 1u) != ((bytes[sb / 8u] >> (7u - sb % 8u)) & 1u)) {
            return 0;
        }
    }
    return 1;
}

static void test_decoder(void)
{
    static uint8_t bytes[4][2100];
    static const uint32_t lens[4] = { 384, 64 + 256 * 50, 40, 64 + 256 };
    double hsi[3] = { -0.022, 0.0, 0.022 };
    uint32_t hi, rep, f, ok = 0, total = 0, tm;
    got_t *g = (got_t *)malloc(sizeof(got_t));
    printf("streaming decoder (36 MHz, HSI -2.2 / 0 / +2.2 %%, edge jitter +-60 ns, RMT hiccups, hub '0' 250 and 340 ns)\n");
    for (tm = 0; tm < 6u; tm++) {
        hi = tm % 3u;
        wv_h0 = tm < 3u ? 250.0 : 340.0;
        wv_l0 = tm < 3u ? 1050.0 : 960.0;
        for (rep = 0; rep < 20u; rep++) {
            wave_t wv;
            memset(&wv, 0, sizeof(wv));
            wv.fs = 36e6 * (1.0 + hsi[hi]);
            wave_put(&wv, 0, 150000.0);
            for (f = 0; f < 4u; f++) {
                uint32_t i;
                for (i = 0; i < sizeof(bytes[f]); i++) {
                    bytes[f][i] = (uint8_t)rnd();
                }
                wave_frame(&wv, bytes[f], lens[f], 60.0, (rep & 1u) ? lens[f] / 2u : 0xFFFFFFFFu, 20000.0);
                wave_put(&wv, 0, 300000.0);
            }
            wave_decode(&wv, g, -1);
            total += 4u;
            for (f = 0; f < 4u && f < g->n; f++) {
                if (g->nbits[f] == lens[f] && !g->bad[f] && tail_matches(g->tail[f], bytes[f], lens[f])) {
                    ok++;
                }
            }
            CHECK(g->n == 4u, "4 frames (got %u)", (unsigned)g->n);
            free(wv.w);
        }
    }
    wv_h0 = 250.0;
    wv_l0 = 1050.0;
    printf("  %u / %u frames exact (lengths 384, 12864 = 50 boards, 40, 320)\n", (unsigned)ok, (unsigned)total);
    CHECK(ok == total, "every frame decoded exactly");
    {
        /* resync in the middle of a frame: that frame is dropped, the next one is exact */
        wave_t wv;
        uint32_t i, mid;
        memset(&wv, 0, sizeof(wv));
        wv.fs = 36e6;
        wave_put(&wv, 0, 150000.0);
        for (i = 0; i < 48u; i++) {
            bytes[0][i] = (uint8_t)rnd();
        }
        mid = wv.nw + 300u;
        wave_frame(&wv, bytes[0], 384, 30.0, 0xFFFFFFFFu, 0);
        wave_put(&wv, 0, 300000.0);
        wave_frame(&wv, bytes[0], 384, 30.0, 0xFFFFFFFFu, 0);
        wave_put(&wv, 0, 300000.0);
        wave_decode(&wv, g, (int)mid);
        CHECK(g->n == 1u && g->nbits[0] == 384u && tail_matches(g->tail[0], bytes[0], 384),
              "resync mid-frame: the cut frame is dropped, the next one is exact");
        free(wv.w);
    }
    {
        /* a too-long high pulse marks the frame bad */
        wave_t wv;
        memset(&wv, 0, sizeof(wv));
        wv.fs = 36e6;
        wave_put(&wv, 0, 150000.0);
        wave_frame(&wv, bytes[0], 100, 0, 0xFFFFFFFFu, 0);
        wave_put(&wv, 1, 5000.0);
        wave_put(&wv, 0, 300000.0);
        wave_decode(&wv, g, -1);
        CHECK(g->n == 1u && g->bad[0], "a 5 us pulse marks the frame bad");
        free(wv.w);
    }
    free(g);
}

/* end to end at sample level: a short image through waveform -> decoder -> session */
static void test_e2e_waveform(void)
{
    image_t *im = &im_new;
    static const bl_rx_timing_t t = BL_RX_TIMING_36MHZ;
    bl_rx_t r;
    wave_t wv;
    uint32_t i, pos = 0, npages, p, k, c;
    uint8_t f[UPD_FRAME_BYTES], endd[32];
    printf("end to end through the decoder (4 KB image, HSI +1.8 %%, jitter +-50 ns)\n");
    board_reset_model();
    make_image(&im_old, 4096, V(0, 4, 0, 0xFF), 1, REV_OK, 1, 101);
    make_image(im, 4096, V(0, 5, 0, 0xFF), 1, REV_OK, 1, 102);
    factory(&im_old);
    /* the board starts in the requested session (as after the app's hand-over) */
    for (i = 0; i < sizeof(HO) / 4u; i++) {
        ((uint32_t *)&HO)[i] = 0;
    }
    HO.request = UPD_REQ_ENTER;
    bl_ho_seal(&HO);
    board_boot(BL_RST_SOFT);
    memset(&wv, 0, sizeof(wv));
    wv.fs = 36e6 * 1.018;
    wave_put(&wv, 0, 200000.0);
    for (i = 0; i < 40u; i++) {
        upd_build_frame(f, UPD_ANNOUNCE, 7, i & 1u, 0, 0, im->desc + 32u * (i & 1u), i);
        wave_frame(&wv, f, 384, 50.0, 0xFFFFFFFFu, 0);
        wave_put(&wv, 0, 300000.0);
    }
    wave_put(&wv, 0, 300000.0);                       /* time for the erase */
    npages = (im->len + 255u) / 256u;
    for (p = 0; p < 2u; p++) {
        for (k = 0; k < npages; k++) {
            for (c = 0; c < 8u; c++) {
                upd_build_frame(f, UPD_DATA, 7, k * 8u + c, (uint8_t)p, 0, im->body + (k * 8u + c) * 32u, 0);
                wave_frame(&wv, f, 384, 50.0, 0xFFFFFFFFu, 0);
                wave_put(&wv, 0, c == 7u ? 3000000.0 : 300000.0);   /* page pause 3 ms */
            }
        }
    }
    memset(endd, 0, 32);
    memcpy(endd, im->desc + UPD_D_BODY_CRC, 4);
    memcpy(endd + 4, im->desc + UPD_D_BODY_LEN, 4);
    for (i = 0; i < 20u; i++) {
        upd_build_frame(f, UPD_END, 7, 0, 0, 0, endd, 0);
        wave_frame(&wv, f, 384, 50.0, 0xFFFFFFFFu, 0);
        wave_put(&wv, 0, 300000.0);
    }
    bl_rx_init(&r, &t);
    {
    uint64_t base = B.now_us;
    while (pos < wv.nw && !B.in_app) {
        bl_frame_t fr;
        uint32_t n = wv.nw - pos < 64u ? wv.nw - pos : 64u;
        int got = bl_rx_feed(&r, wv.w + pos, n, &fr);
        pos += n;
        B.now_us = base + (uint64_t)((double)pos * 16.0 / wv.fs * 1e6);
        if (got) {
            fl_busy_us = 0;
            bl_sess_frame(&BS, fr.nbits, fr.tail, ms());
            if (fl_busy_us > 0u) {
                /* the board was busy: skip the samples of that time and resync, as bl_main does */
                pos += (uint32_t)((double)fl_busy_us * 1e-6 * wv.fs / 16.0);
                bl_rx_resync(&r);
            }
        }
        board_poll();
    }
    }
    host_idle(200000u);                               /* trial window */
    CHECK(B.in_app && flash_has(im), "installed through the sample-level path (frames %u, UPD %u, bad %u)",
          (unsigned)BS.frames, (unsigned)BS.upd_frames, (unsigned)BS.bad_frames);
    free(wv.w);
}

/* an image made by tools/release/mkimage.py installs through the C code (descriptor and CRC agree) */
static void test_release_image(const char *path)
{
    FILE *f = fopen(path, "rb");
    static uint8_t buf[UPD_DESC_BYTES + UPD_APP_BODY_MAX + 4];
    size_t n;
    printf("release image %s\n", path);
    if (f == 0) {
        CHECK(0, "cannot open %s", path);
        return;
    }
    n = fread(buf, 1, sizeof(buf), f);
    fclose(f);
    CHECK(n > UPD_DESC_BYTES && bl_desc_check(buf) == 0, "the Python descriptor passes the bootloader's checks");
    board_reset_model();
    make_image(&im_old, 4096, V(0, 0, 1, 0xFF), 1, REV_OK, 1, 111);   /* an older app on the board */
    memcpy(im_new.desc, buf, UPD_DESC_BYTES);
    im_new.len = (uint32_t)(n - UPD_DESC_BYTES);
    memcpy(im_new.body, buf + UPD_DESC_BYTES, im_new.len);
    factory(&im_old);
    board_power_on();
    host_session(&im_new, 0x77, &SESS_DEFAULT);
    host_after();
    CHECK(running(&im_new), "installed and verified (body CRC from Python = CRC unit model)");
}

#ifndef BL_TEST_NO_MAIN
int main(void)
{
    const char *img = getenv("BL_TEST_IMAGE");
    if (img != 0 && *img != 0) {
        test_release_image(img);
        printf("\n%d checks, %d failed\n", checks, fails);
        return fails ? 1 : 0;
    }
    test_erase_plan();
    test_desc_check();
    test_decide();
    test_clean_update();
    test_rejects();
    test_abort_timeout();
    test_trial_and_watchdog();
    test_verify_catches_bad_body();
    test_low_power();
    test_switch_low_power();
    test_loss();
    test_power_cuts(6000, 1);
    test_power_cuts(30548, 7);
    test_decoder();
    test_e2e_waveform();
    printf("\n%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
#endif
