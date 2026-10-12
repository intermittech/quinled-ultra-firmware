/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Wire-level chain test (included by test_main.c): what every board of a chain really sees.
 *
 * Every bit on a wire keeps its own time (rise; fall = rise + 0.35 / 0.70 us), so a closed gate leaves a LOW gap of
 * exactly the blocked bits, as U2 does. Each board runs the real frame tracker (frame_track.h) on a 50 us tick with
 * its own phase and the hardware edge count, re-arms its gate with the real plans (proto_gate_plan, one frame later
 * via the double buffer), and makes the real per-frame decisions (frame_rx.c: capture, frame end, sticky, lock),
 * with a main-loop latency model. The decoder is modelled by its rules: the capture is the first own bits of the
 * frame and fails if it contains a reset gap (>= 50 us); the Ultra header is the last 64 complete bits in a window of
 * 576 words (256 us) ending just after the last-edge tick, also failing on a gap inside it. The waveform decoder
 * itself is tested on sampled waveforms (test_decoder, test_ultra_e2e).
 *
 * Content: every WS chip slot and every Ultra slot carries its board index and host frame number, so a board that
 * shows another board's bits, or bits cut at the wrong place, counts as garbage.
 * Negative control: the format 1 layout (header first, withdrawn, gate open 64 / closed 256 / open) is run at the wire level; boards
 * after the first never see a frame that holds the header and their slot - the bug this layout change fixes.
 * Firmware update (WK_UPD, run by test_update.c): every board also runs the app's update side (core/upd_app.c) as
 * main.c does - UPD headers make it transparent, it then captures whole 384-bit frames; an ANNOUNCE it takes resets
 * it into the bootloader (gate closed, then open for everything). */
#include "../src/core/frame_rx.h"
#include "../src/core/upd_app.h"

#define WB_MAX     10
#define WF_MAX     64
#define W_TICK     50.0
#define W_TAIL_US  (576.0 * 16.0 / 36.0)       /* tail window: 576 words of 16 samples at 36 MHz = 256 us */
#define W_GAP_US   50.0                       /* decoder gap_min: a reset gap */

enum { WK_WS2805 = 0, WK_WS2915 = 1, WK_ULTRA = 2, WK_LEGACY = 3, WK_SWITCH = 4,   /* SWITCH: ultra_switch_frame, then Ultra */
       WK_UPD = 5 };                                                                 /* UPD: Ultra frames, then wupd_host[] */

typedef struct { double rise; uint16_t frame, idx; uint8_t v; } wbit_t;

static wbit_t *wwire[WB_MAX + 1];
static int wn[WB_MAX + 1];
#define WX_MAX     2                          /* broadcast / extension slots after the last board's slot */
static uint8_t whost[WF_MAX][32 * (WB_MAX + WX_MAX) + 8];
static int whost_bits[WF_MAX], wframes;
static double whost_end[WF_MAX];
static int wkind, wboards, wpre, wconfirmed;   /* wconfirmed: the stored protocol counts as confirmed (locked) */
static int wpre_nop;                          /* preamble of NOP slots (valid CRC) instead of DARK slots */
static int wmixed;                            /* per-board modes below instead of wchain's */
static proto_mode_t wmix_mode[WB_MAX];
static proto_t wmix_sticky[WB_MAX];
static int wmix_conf[WB_MAX];                 /* ... and the stored protocol counts as confirmed */
static int wgap_board = -1, wgap_frame;       /* this board powers up inside the first >= 300 us gap of that frame */
static int wext;                             /* Ultra frames: broadcast slots (types 0x80..) after the board slots */
static int wflags;                           /* Ultra frames: header EXT on f % 3 == 1, HINT on f % 3 == 2 */
/* WK_UPD: host frames from wupd_from on are these UPD frames; the boards' update info (bl_version 0: no bootloader) */
#define WUPD_UART_US 10000.0                  /* main.c: TAKE sends its console line, then resets */
#define WUPD_BOOT_US 2000.0                   /* reset + bootloader start until its gate opens */
static uint8_t wupd_host[WF_MAX][UPD_FRAME_BYTES];
static int wupd_from = WF_MAX;
static upd_app_info_t wupd_info;

/* the hub's switch sequence is the firmware's config protocol */
typedef char wire_switch_cfg_check[(ULTRA_WSCFG_MAGIC == CONFIG_MAGIC && ULTRA_WSCFG_PROTOCOL == CMD_PROTOCOL &&
                                    ULTRA_WSCFG_COPIES >= CONFIG_REPEAT) ? 1 : -1];

/* host frame f is an Ultra frame (slots + header) */
static int wframe_ultra(int f) { return wkind == WK_ULTRA || (wkind == WK_SWITCH && f >= (int)ULTRA_SWITCH_WS_FRAMES); }

/* own generator: the shared one (test_main.c) feeds the tests after this one, which must not depend on how many
 * chains run here */
static unsigned long long wrs = 0x2545F4914F6CDD1DULL;
static double wurand(double a, double b)
{
    wrs ^= wrs << 13; wrs ^= wrs >> 7; wrs ^= wrs << 17;
    return a + (b - a) * (double)(wrs >> 11) / 9007199254740992.0;
}

static double wfall(const wbit_t *b) { return b->rise + (b->v ? 0.70 : 0.35); }

static void ws2805_bytes(int k, int f, uint8_t *o)
{
    o[0] = (uint8_t)(0x10 + k); o[1] = (uint8_t)(f & 0x7F); o[2] = 0x55; o[3] = (uint8_t)(0xAA ^ k); o[4] = (uint8_t)((f >> 7) & 0x7F);
}

static void ws2915_bytes(int k, int f, uint8_t *o)
{
    static const uint8_t tail[5] = { 0x10, 0x20, 0x30, 0x40, 0x50 };
    int i;
    o[0] = (uint8_t)(0x20 + k); o[1] = (uint8_t)(f & 0x7F); o[2] = 0x33; o[3] = (uint8_t)(0x44 ^ k); o[4] = (uint8_t)((f >> 7) & 0x7F);
    for (i = 0; i < 5; i++) o[5 + i] = (uint8_t)(tail[i] ^ (i == 4 ? k : 0));
}

/* broadcast slot e of host frame f: DARK during the preamble, like the board slots */
static void wbcast_slot(int e, int f, uint8_t *s)
{
    int i;
    if (f < wpre) { ultra_slot_dark(s); return; }
    ultra_slot_begin(s, (uint8_t)(ULTRA_TYPE_BCAST + e), ULTRA_LIGHT_BOARD, 0, (uint8_t)f);
    for (i = 0; i < (int)ULTRA_PAYLOAD_LEN; i++) s[ULTRA_PAYLOAD + i] = (uint8_t)(0x5A ^ (i * 7) ^ e ^ f);
    ultra_slot_seal(s);
}

static uint8_t wflag_of(int f) { return (uint8_t)(!wflags ? 0 : f % 3 == 1 ? ULTRA_HDR_EXT : f % 3 == 2 ? ULTRA_HDR_HINT : 0); }

static void wultra_slot(int k, int f, uint8_t *s)
{
    static const uint16_t one[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint16_t a[5] = { 0, 0, 0, 0, 0 };
    if (f < wpre) { if (wpre_nop) ultra_slot_nop(s); else ultra_slot_dark(s); return; }
    a[0] = (uint16_t)f;
    a[1] = (uint16_t)k;
    ultra_slot_set(s, 0, (uint8_t)f, 0, (uint16_t)(1000 + k), a, one, 0, 0);
}

/* host frames on wire 0. period_us > 0: frame starts every period (a WS controller), 0: back to back (300 us) */
static void whost_build(int kind, int boards, int nframes, double start_us, double period_us, int pre)
{
    int f, i, b, n;
    double t = start_us;
    wkind = kind; wboards = boards; wframes = nframes; wpre = pre;
    if (!wwire[0]) for (i = 0; i <= WB_MAX; i++) wwire[i] = (wbit_t *)malloc(sizeof(wbit_t) * (size_t)WF_MAX * (256 * (WB_MAX + WX_MAX) + 64 + 8));
    wn[0] = 0;
    for (f = 0; f < nframes; f++) {
        uint8_t *fr = whost[f];
        double t0 = period_us > 0 ? start_us + f * period_us : t;
        int wsf = kind == WK_SWITCH && f < (int)ULTRA_SWITCH_WS_FRAMES;   /* a WS config frame of the switch sequence */
        n = 0;
        if (wsf) {
            n = (int)ultra_switch_frame(fr, (uint32_t)f, (uint32_t)boards);
        } else if (kind == WK_UPD && f >= wupd_from) {
            memcpy(fr, wupd_host[f], UPD_FRAME_BYTES);                         /* its own header (TIME unused) */
            n = UPD_FRAME_BYTES;
        } else if (kind == WK_WS2805) {
            for (b = 0; b < boards; b++, n += 5) ws2805_bytes(b, f, fr + n);
        } else if (kind == WK_WS2915) {
            fr[0] = 0xFF; fr[1] = 0xFF; fr[2] = 0xFF; fr[3] = 0x80; n = 4;      /* gains 31, 7 check bits '0' */
            for (b = 0; b < boards; b++, n += 10) ws2915_bytes(b, f, fr + n);
        } else {
            int x = kind == WK_LEGACY ? 0 : wext, h = kind == WK_LEGACY ? 0 : 32 * (boards + x);   /* VERSION 2: slots, then the header */
            for (b = 0; b < boards; b++) wultra_slot(b, f, fr + (kind == WK_LEGACY ? 8 : 0) + 32 * b);
            for (b = 0; b < x; b++) wbcast_slot(b, f, fr + 32 * (boards + b));
            n = 32 * (boards + x) + 8;
            ultra_build_header_flags(fr + h, 0, kind == WK_LEGACY ? 0 : wflag_of(f));   /* TIME filled in below */
        }
        whost_bits[f] = n * 8;
        /* bit times: a WS controller 1.25 us per bit, the hub 1.30 / 1.35 us ('0' / '1') */
        for (i = 0; i < 2; i++) {
            double tt = t0;
            int j;
            wn[0] -= i ? n * 8 : 0;
            for (j = 0; j < n * 8; j++) {
                wbit_t *w = &wwire[0][wn[0]++];
                w->v = (uint8_t)((fr[j >> 3] >> (7 - (j & 7))) & 1u);
                w->rise = tt;
                w->frame = (uint16_t)f;
                w->idx = (uint16_t)j;
                tt += kind == WK_WS2805 || kind == WK_WS2915 ? 1.25 : (w->v ? 1.35 : 1.30);
            }
            whost_end[f] = wfall(&wwire[0][wn[0] - 1]);
            if (kind >= WK_ULTRA && !wsf && !(kind == WK_UPD && f >= wupd_from) && i == 0) {   /* TIME = host clock at the end */
                int h = kind == WK_LEGACY ? 0 : 32 * (boards + wext);
                ultra_build_header_flags(fr + h, (uint32_t)(kind == WK_LEGACY ? t0 : whost_end[f]),
                                         kind == WK_LEGACY ? 0 : wflag_of(f));
                continue;                                                      /* again with the real header bits */
            }
            break;
        }
        t = whost_end[f] + 300.0;
    }
}

typedef struct {
    int kind;                                  /* 0 capture, 1 frame end */
    double ready;
    uint32_t frame, total, nbits, own, skip;
    proto_t proto;
    int valid, tail_ok, from, to, hf;          /* hf: host frame of the capture's first bit */
    uint8_t bytes[UPD_FRAME_BYTES + 4], tail[8];   /* an Ultra slot, or a whole update frame */
    double fe_tick;
} wjob_t;
typedef char wjob_bytes_check[UPD_FRAME_BYTES + 4 >= PROTO_BITS_BUF ? 1 : -1];

typedef struct {
    frx_t frx;
    ftrack_t ft;
    gate_plan_t plan, buf[2];
    int idx, rearmed, legacy, cap_skip;
    double rearm_t, closed_t, boot, phase, main_free, edge_t;
    int cnt_from, cnt_ptr, fwd_ptr;
    uint32_t frames;
    int cap_first;
    uint32_t cap_frame;
    proto_t cap_proto;
    uint16_t cap_own;
    uint8_t last_proto;
    wjob_t job[8];
    int njob;
    int first_f, shown, garbage, dark, writes, slot_frames, legacy_slot_frames, frames_seen;
    double sync_err;
    config_rx_t crx;                           /* config frames, as main.c handle_slot */
    settings_t set;
    int acks, relock_ws;                       /* config commands taken; re-locks to a WS protocol */
    upd_app_t upd;                             /* the update side, as main.c */
    double take_t;                             /* TAKE (the main loop's time), -1 = none */
    int take_hf, upd_first_hf, upd_ok, upd_shows;   /* host frames of the TAKE / first whole UPD frame; valid ones */
    int bcast_ignored, ext_frames;             /* a broadcast slot taken as its own and ignored; EXT frames synced */
} wboard_t;

static wboard_t wb[WB_MAX];
static const gate_plan_t w_closed = { 0, 0, 0, 0, 0 };
static const gate_plan_t w_legacy = { 1, 64, 320, 320, 64 };   /* format 1: header open, own slot closed, rest open */

static int wgate_open(const gate_plan_t *g, int n)
{
    int open = g->start_open;
    if (g->t1 && n >= g->t1) open = !open;
    if (g->t2 && n >= g->t2) open = !open;
    return open;
}

static int wdebug;                            /* WIRE_DEBUG: print every garbage frame */
static const char *wpath = "", *wscen = "";
static uint32_t wdbg_total, wdbg_frame;
static int wdbg_tail, wdbg_hdr;

static void wshow(wboard_t *b, int ok, int f)
{
    if (!ok) {
        b->garbage++;
        if (wdebug)
            printf("    dbg garbage [%s]: board %d, via %s, host frame %d, board frame %u (total %u bits, tail %d, header %d), "
                   "mode %d locked %d provisional %d\n", wscen, (int)(b - wb), wpath, f, wdbg_frame, wdbg_total, wdbg_tail,
                   wdbg_hdr, b->frx.ps.mode, b->frx.ps.locked, b->frx.ps.provisional);
        return;
    }
    if (b->first_f < 0) b->first_f = f;
    b->shown++;
}

static void wshow_ws(wboard_t *b, int k, const slot_t *s, proto_t p)
{
    uint8_t e[10];
    int f = (s->raw[1] & 0x7F) | ((s->raw[4] & 0x7F) << 7), n = p == PROTO_WS2805 ? 5 : 10, i, zero = 1;
    for (i = 0; i < n; i++) zero &= s->raw[i] == 0;
    if (wkind >= WK_ULTRA && zero) { b->dark++; return; }   /* a NOP slot read as WS: dark, not a colour */
    if (p == PROTO_WS2805) ws2805_bytes(k, f, e); else ws2915_bytes(k, f, e);
    wshow(b, f < wframes && s->raw_len == n && memcmp(s->raw, e, (size_t)n) == 0, f);
}

static void wshow_ultra(wboard_t *b, int k, const uint8_t *s)
{
    if (s[0] == ULTRA_NOP) return;                                  /* NOP preamble (DARK slots are never valid) */
    if (wdebug && !(s[0] == ULTRA_SET && ultra_get16(s + ULTRA_PAYLOAD) == 1000u + (uint32_t)k)) {
        int kk, f = (int)ultra_get16(s + ULTRA_PAYLOAD + 2);
        printf("    dbg ULTRA slot of board %d applied by board %d (host frame %d); frame %d along the chain:\n",
               (int)ultra_get16(s + ULTRA_PAYLOAD) - 1000, k, f, f);
        for (kk = 0; kk <= k; kk++) {
            int i, n = 0, first = -1, last = -1, gaps = 0;
            double prev = -1;
            for (i = 0; i < wn[kk]; i++) {
                if (wwire[kk][i].frame != f) continue;
                if (first < 0) first = wwire[kk][i].idx;
                if (prev >= 0 && wwire[kk][i].rise - prev >= W_GAP_US) gaps++;
                prev = wfall(&wwire[kk][i]);
                last = wwire[kk][i].idx;
                n++;
            }
            printf("      wire %d (input of board %d): %d bits, idx %d..%d, %d gaps >= 50 us; board %d boot %.0f us, plan open %d t1 %d\n",
                   kk, kk, n, first, last, gaps, kk, wb[kk].boot, wb[kk].plan.start_open, wb[kk].plan.t1);
        }
    }
    wshow(b, s[0] == ULTRA_SET && ultra_get16(s + ULTRA_PAYLOAD) == 1000u + (uint32_t)k, (int)ultra_get16(s + ULTRA_PAYLOAD + 2));
}

/* a broadcast slot (type >= 0x80) captured as the board's own slot (the hub's board count too low): the real engine
 * must ignore it and keep its state */
static ultra_t wu, wu0;
static void wbcast_taken(wboard_t *b, const uint8_t *s)
{
    ultra_init(&wu, 1);
    memcpy(&wu0, &wu, sizeof wu);
    if (ultra_on_slot(&wu, s, 0) != 0 || memcmp(&wu, &wu0, sizeof wu) != 0) b->garbage++;
    else b->bcast_ignored++;
}

static void wpack(const wbit_t *in, int from, int n, uint8_t *out)
{
    int i;
    memset(out, 0, (size_t)(n + 7) / 8);
    for (i = 0; i < n; i++) if (in[from + i].v) out[i >> 3] |= (uint8_t)(0x80u >> (i & 7));
}

/* a reset gap (>= 50 us between a fall and the next rise) inside bits [from, to) */
static int whas_gap(const wbit_t *in, int from, int to)
{
    int i;
    for (i = from + 1; i < to; i++) if (in[i].rise - wfall(&in[i - 1]) >= W_GAP_US) return 1;
    return 0;
}

/* a WS slot to show: a config frame holds the outputs (and a protocol command switches the mode at once), as main.c */
static void wws(wboard_t *b, int k, const slot_t *s, proto_t p)
{
    cfg_result_t r = config_process(&b->crx, &b->set, s);
    if (r.event != CFG_NONE) {
        if (r.event == CFG_ACK) {
            b->acks++;
            if (r.cmd == CMD_PROTOCOL) proto_set_mode(&b->frx.ps, (proto_mode_t)b->set.protocol);
        }
        return;
    }
    wshow_ws(b, k, s, p);
}

static void wloop_job(wboard_t *b, int k, wjob_t *j, double t)
{
    if (j->kind == 0) {
        slot_t ws;
        if (j->own == PROTO_UPD_BITS && j->skip == 0u) {   /* transparent: a whole update frame (main.c upd_capture) */
            if (j->valid && j->nbits >= UPD_FRAME_BITS) {
                uint32_t a = upd_app_frame(&b->upd, j->bytes, (uint32_t)(t / 1000.0));
                if (upd_check_frame(j->bytes) == 0 && b->upd_ok++ == 0) b->upd_first_hf = j->hf;
                if (a == UPD_APP_TAKE && b->take_t < 0) { b->take_t = t; b->take_hf = j->hf; }
                if (a == UPD_APP_SHOW_SAME || a == UPD_APP_SHOW_REJECTED) b->upd_shows++;
            }
            return;
        }
        if (j->valid && (frx_capture(&b->frx, j->frame, j->proto, j->bytes, j->nbits, j->own, j->skip, &ws) & FRX_SHOW_WS)) {
            wpath = "capture (locked)";
            wdbg_frame = j->frame; wdbg_total = 0; wdbg_tail = -1; wdbg_hdr = -1;
            wws(b, k, &ws, j->proto);
        }
        return;
    }
    {
        frx_end_t e;
        frx_frame_end(&b->frx, j->frame, j->total, j->tail_ok ? j->tail : NULL, &e);
        wpath = "frame end (stored protocol confirmed)";
        wdbg_frame = j->frame; wdbg_total = j->total; wdbg_tail = j->tail_ok; wdbg_hdr = e.ultra_header;
        if (e.show_ws) wws(b, k, &b->frx.ws, (proto_t)b->frx.ws_proto);
        if (e.after != e.before && (e.after == PROTO_WS2805 || e.after == PROTO_WS2915_32 || e.after == PROTO_WS2915_26))
            b->relock_ws++;
        if (e.close_gate) { b->closed_t = t; }
        if (e.ultra_sync) {
            double err = fabs(j->fe_tick - (double)e.host_us);
            if (err > b->sync_err) b->sync_err = err;
            if (e.flags & ULTRA_HDR_EXT) b->ext_frames++;
            if (e.ultra_slot) {
                if (b->frx.slot[0] >= ULTRA_TYPE_BCAST) wbcast_taken(b, b->frx.slot);
                else wshow_ultra(b, k, b->frx.slot);
            }
        }
        if (b->frx.ps.mode == PMODE_AUTO && b->frx.ps.locked != PROTO_NONE && !b->frx.ps.provisional &&
            b->frx.ps.locked != b->last_proto) {
            b->last_proto = (uint8_t)b->frx.ps.locked;
            b->writes++;
        }
        upd_app_frame_end(&b->upd, e.ultra_header, e.flags, (uint32_t)(t / 1000.0));
        b->idx ^= 1;
        b->buf[b->idx] = upd_app_transparent(&b->upd) ? proto_gate_plan_transparent() : e.plan;
    }
}

static void wloop(wboard_t *b, int k, double t)
{
    while (b->njob) {
        wjob_t *j = &b->job[0];
        double start = j->ready > b->main_free ? j->ready : b->main_free;
        double done = start + (j->kind == 0 ? 20.0 + 0.74 * j->nbits : 10.0 + (j->tail_ok ? 120.0 : 0.0));
        if (done > t) return;
        b->main_free = done;
        wloop_job(b, k, j, done);
        memmove(&b->job[0], &b->job[1], sizeof(wjob_t) * (size_t)(--b->njob));
    }
}

/* one board: input wire k, output wire k + 1 */
static void wsim_board(int k, proto_mode_t mode, proto_t sticky, int legacy)
{
    wboard_t *b = &wb[k];
    const wbit_t *in = wwire[k];
    wbit_t *out = wwire[k + 1];
    int n = wn[k], nout = 0;
    double t, end = (n ? wfall(&in[n - 1]) : 0) + 2000.0;

    memset(b, 0, sizeof *b);
    upd_app_init(&b->upd, &wupd_info);
    b->take_t = -1;
    b->take_hf = b->upd_first_hf = -1;
    if (wmixed) { mode = wmix_mode[k]; sticky = wmix_sticky[k]; }
    config_rx_init(&b->crx);
    settings_default(&b->set);
    b->set.protocol = (uint8_t)mode;
    b->boot = 24000.0 + wurand(0, 6000.0);
    b->phase = wurand(0, W_TICK);
    if (k == wgap_board) {                       /* power-up inside a LOW gap of a frame (the gates before it) */
        int i;
        for (i = 1; i < n; i++)
            if (in[i].frame == wgap_frame && in[i - 1].frame == wgap_frame && in[i].rise - wfall(&in[i - 1]) >= 300.0) {
                b->boot = wfall(&in[i - 1]) + 100.0;
                break;
            }
    }
    b->legacy = legacy;
    frx_init(&b->frx, mode, sticky);
    if (wconfirmed || (wmixed && wmix_conf[k])) b->frx.ps.provisional = 0;
    b->last_proto = (uint8_t)sticky;
    b->plan = b->buf[0] = b->buf[1] = legacy ? w_legacy : proto_gate_plan(proto_active(&b->frx.ps));
    b->closed_t = -1;
    b->first_f = -1;
    while (b->cnt_ptr < n && wfall(&in[b->cnt_ptr]) <= b->boot) b->cnt_ptr++;   /* the counter starts at power-up */
    b->cnt_from = b->cnt_ptr;
    while (b->fwd_ptr < n && in[b->fwd_ptr].rise <= b->boot) b->fwd_ptr++;     /* gate pulled low until then */
    for (t = b->boot + b->phase; t < end; t += W_TICK) {
        uint32_t ev;
        uint16_t c, own;
        if (b->take_t >= 0 && t >= b->take_t + WUPD_UART_US) {   /* reset into the bootloader: dark, then all open */
            for (; b->fwd_ptr < n && in[b->fwd_ptr].rise <= t; b->fwd_ptr++)
                if (in[b->fwd_ptr].rise >= b->take_t + WUPD_UART_US + WUPD_BOOT_US) out[nout++] = in[b->fwd_ptr];
            continue;
        }
        wloop(b, k, t);
        /* bits that started since the last tick go through the gate as it is now */
        for (; b->fwd_ptr < n && in[b->fwd_ptr].rise <= t; b->fwd_ptr++) {
            const wbit_t *w = &in[b->fwd_ptr];
            int cnt = b->fwd_ptr - b->cnt_from;
            if (b->rearmed && cnt >= 0 && !(b->closed_t >= 0 && w->rise > b->closed_t) && wgate_open(&b->plan, cnt))
                out[nout++] = *w;
        }
        while (b->cnt_ptr < n && wfall(&in[b->cnt_ptr]) <= t) b->cnt_ptr++;
        c = (uint16_t)(b->cnt_ptr - b->cnt_from);
        own = b->plan.own_bits;                    /* as main.c poll_isr */
        if (b->frx.ps.provisional && own < PROTO_DETECT_BITS) own = PROTO_DETECT_BITS;
        ev = ftrack_tick(&b->ft, c, own, b->plan.hdr_bits);
        if (ev & FT_EDGE) b->edge_t = t;
        if (ev & FT_CAP_START) {
            if (ev & FT_CAP_DATA) {
                b->cap_skip = b->plan.hdr_bits;          /* WS2915 data after the header pause */
            } else {
                b->cap_skip = 0;
                b->cap_frame = b->frames;
                b->cap_proto = proto_active(&b->frx.ps);
                b->cap_own = b->plan.own_bits;
            }
            b->cap_first = b->cnt_from + b->cap_skip;
        }
        if ((ev & (FT_CAP_COPY | FT_CAP_SHORT)) && !legacy && b->njob < 8) {
            wjob_t *j = &b->job[b->njob++];
            int want = b->ft.cap_bits - b->cap_skip;
            j->kind = 0; j->ready = t; j->frame = b->cap_frame; j->proto = b->cap_proto; j->own = b->cap_own;
            j->skip = (uint32_t)b->cap_skip;
            j->nbits = (uint32_t)(want > 0 ? want : 0);
            j->hf = b->cap_first < n ? in[b->cap_first].frame : -1;
            /* the decoder starts in the idle line before the capture and stops at a reset gap inside it */
            j->valid = want > 0 && b->cap_first + want <= b->cnt_ptr && !whas_gap(in, b->cap_first, b->cap_first + want) &&
                       (b->cap_first == 0 || in[b->cap_first].rise - wfall(&in[b->cap_first - 1]) >= 60.0 ||
                        b->cap_first == b->cnt_from);
            if (j->valid) wpack(in, b->cap_first, want, j->bytes);
        }
        if (ev & (FT_FRAME_END | FT_BOOT_ARM)) {
            if ((ev & (FT_FRAME_END | FT_PARTIAL)) == FT_FRAME_END) {
                int from = b->cnt_from, to = b->cnt_ptr, w0 = to, nb = to - from;
                b->frames_seen++;
                /* wire check: VERSION 2 = this board's slot first and the header last, of one host frame */
                if (nb >= 320 && in[from].frame == in[to - 1].frame) {
                    int hf = in[from].frame, base = 256 * (wboards + wext);
                    if (in[from].idx == 256 * k && in[from + 255].idx == 256 * k + 255 && in[to - 64].idx == base &&
                        in[to - 1].idx == base + 63) b->slot_frames++;
                    if (in[from].idx == 0 && in[from + 63].idx == 63 && in[from + 64].idx == 64 + 256 * k &&
                        in[from + 319].idx == 64 + 256 * k + 255 && in[from + 319].frame == hf) b->legacy_slot_frames++;
                }
                if (!legacy && b->njob < 8) {
                    wjob_t *j = &b->job[b->njob++];
                    j->kind = 1; j->ready = t; j->frame = b->frames; j->total = (uint32_t)nb; j->fe_tick = b->edge_t;
                    j->tail_ok = 0;
                    if (frx_tail_wanted(&b->frx, (uint32_t)nb)) {
                        /* the decoder's window: 576 words ending just after the last-edge tick */
                        double ws = b->edge_t + 1.8 - W_TAIL_US;
                        while (w0 > from && in[w0 - 1].rise >= ws) w0--;
                        if (to - w0 >= 64 && !whas_gap(in, w0, to)) {
                            wpack(in, to - 64, 64, j->tail);
                            j->tail_ok = 1;
                        }
                    }
                }
            }
            b->plan = b->buf[b->idx];
            b->rearmed = 1;
            b->rearm_t = t;
            b->closed_t = -1;
            b->cnt_from = b->cnt_ptr;
            if (ev & FT_FRAME_END) b->frames++;
        }
    }
    wn[k + 1] = nout;
}

typedef struct {
    int worst_first, garbage, dark, writes, missing, slot_frames_min, legacy_hits, all_shown, acks, relock, fwd_mis, fwd_closed;
    double sync;
} wres_t;

/* Forwarding of board k for every Ultra host frame it saw from its first bit after power-up: the ideal output is
 * everything but the first 256 bits it saw. Nothing (gate closed) is safe; anything else is a misaligned forward. */
static void wfwd(int k, int *mis, int *closed)
{
    const wbit_t *in = wwire[k], *out = wwire[k + 1];
    int n = wn[k], no = wn[k + 1], i = 0, o = 0;
    while (i < n) {
        int f = in[i].frame, a = i, oa, cnt;
        while (i < n && in[i].frame == f) i++;
        while (o < no && out[o].frame < f) o++;
        oa = o;
        while (o < no && out[o].frame == f) o++;
        if (!wframe_ultra(f) || in[a].rise <= wb[k].boot) continue;
        cnt = o - oa;
        if (cnt == 0) {
            if (i - a > 256) (*closed)++;
        } else if (!(cnt == i - a - 256 && out[oa].idx == in[a + 256].idx && out[o - 1].idx == in[i - 1].idx)) {
            (*mis)++;
        }
    }
}

/* the last board's output: per host frame exactly the broadcast slots and the header (bits 256 N .. 256 (N + k) + 63) */
static void wtail_count(int nb, int *frames, int *ok)
{
    const wbit_t *o = wwire[nb];
    int n = wn[nb], i = 0, f;
    for (f = 0; f < wframes; f++) {
        int a, c = 0;
        (*frames)++;
        while (i < n && o[i].frame < f) i++;
        a = i;
        while (i < n && o[i].frame == f) { i++; c++; }
        if (c == 256 * wext + 64 && o[a].idx == 256 * nb && o[i - 1].idx == 256 * (nb + wext) + 63) (*ok)++;
    }
}

static wres_t wchain(int boards, proto_mode_t mode, proto_t sticky, int legacy)
{
    wres_t r;
    int k;
    memset(&r, 0, sizeof r);
    r.slot_frames_min = 1 << 30;
    for (k = 0; k < boards; k++) {
        wsim_board(k, mode, sticky, legacy);
        wfwd(k, &r.fwd_mis, &r.fwd_closed);
        r.acks += wb[k].acks;
        r.relock += wb[k].relock_ws;
        r.garbage += wb[k].garbage;
        r.dark += wb[k].dark;
        r.writes += wb[k].writes;
        if (wb[k].first_f < 0) r.missing++;
        else if (wb[k].first_f > r.worst_first) r.worst_first = wb[k].first_f;
        if (wb[k].slot_frames < r.slot_frames_min) r.slot_frames_min = wb[k].slot_frames;
        if (k > 0) r.legacy_hits += wb[k].legacy_slot_frames;
        if (wb[k].sync_err > r.sync) r.sync = wb[k].sync_err;
    }
    return r;
}

/* Opt-in sweep (TEST_SWEEP=<seeds>, no number: 200): the randomized chain scenarios (power-up moments, tick phases, frame starts)
 * again for seeds 1..n, 1-10 boards each; prints every failing seed and case. Same pass rules as test_wire(). */
static int wire_sweep_seed(unsigned long long seed)
{
    static const proto_t st[] = { PROTO_WS2805, PROTO_WS2915_32, PROTO_WS2915_26 };
    static const double per[] = { 0, 20000.0, 200000.0 };
    static const int wrong[][2] = { { 0, PROTO_ULTRA }, { 1, PROTO_ULTRA }, { 0, PROTO_WS2915_32 }, { 1, PROTO_WS2805 },
                                    { 2, PROTO_WS2805 }, { 2, PROTO_WS2915_32 }, { 2, PROTO_WS2915_26 } };
    static const struct { proto_mode_t mode; proto_t sticky; int conf, ws, second; } sw[] = {
        { PMODE_WS2805, PROTO_NONE, 0, 1, 0 }, { PMODE_WS2915, PROTO_NONE, 0, 1, 1 }, { PMODE_AUTO, PROTO_WS2805, 1, 1, 0 },
        { PMODE_AUTO, PROTO_WS2915_32, 1, 1, 1 }, { PMODE_ULTRA, PROTO_NONE, 0, 0, 0 }, { PMODE_AUTO, PROTO_ULTRA, 1, 0, 0 } };
    int nb, i, k, fails = 0;
    wres_t r;
    wrs = 0x2545F4914F6CDD1DULL ^ (seed * 0x9E3779B97F4A7C15ULL);
    for (nb = 1; nb <= WB_MAX; nb++) {
        int pre = 4 * nb + 2;
        /* fresh auto + preamble */
        wscen = "fresh";
        whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre);
        r = wchain(nb, PMODE_AUTO, PROTO_NONE, 0);
        if (r.garbage || r.missing || r.worst_first != pre) {
            fails++;
            printf("  wire sweep FAIL seed %llu: fresh + preamble, %d boards: garbage %d missing %d first %d\n", seed, nb,
                   r.garbage, r.missing, r.worst_first);
        }
        /* wrong stored protocol */
        for (i = 0; i < 7; i++) {
            if (wrong[i][0] == WK_ULTRA) whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre);
            else whost_build(wrong[i][0], nb, WF_MAX, 3000.0, 20000.0, 0);
            r = wchain(nb, PMODE_AUTO, (proto_t)wrong[i][1], 0);
            if (r.garbage || r.missing || r.writes != nb) {
                fails++;
                printf("  wire sweep FAIL seed %llu: stored %d, sent kind %d, %d boards: garbage %d missing %d writes %d\n",
                       seed, wrong[i][1], wrong[i][0], nb, r.garbage, r.missing, r.writes);
            }
        }
        /* live switch, DARK preamble */
        for (i = 0; i < 2; i++) {
            wconfirmed = 1;
            whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre);
            r = wchain(nb, PMODE_AUTO, i ? PROTO_WS2915_32 : PROTO_WS2805, 0);
            wconfirmed = 0;
            if (r.garbage || r.missing || r.worst_first != pre) {
                fails++;
                printf("  wire sweep FAIL seed %llu: live switch from %s, %d boards: garbage %d missing %d first %d\n", seed,
                       i ? "WS2915" : "WS2805", nb, r.garbage, r.missing, r.worst_first);
            }
        }
        /* stale stored WS protocol, hub running, no preamble */
        wscen = "stale";
        for (i = 0; i < 3; i++) {
            int pp;
            for (pp = 0; pp < 3; pp++) {
                int nf = per[pp] > 100000.0 ? 32 : per[pp] > 0 ? 40 : WF_MAX;
                whost_build(WK_ULTRA, nb, nf, wurand(0, 30000.0), per[pp], 0);
                r = wchain(nb, PMODE_AUTO, st[i], 0);
                if (r.garbage || r.missing) {
                    fails++;
                    printf("  wire sweep FAIL seed %llu: stale %d, period %.0f us, %d boards: garbage %d missing %d\n", seed,
                           st[i], per[pp], nb, r.garbage, r.missing);
                }
            }
        }
        /* switch_boards_to_ultra */
        wscen = "switch";
        for (i = 0; i < 6; i++) {
            int pre2 = (int)ULTRA_SWITCH_WS_FRAMES + pre, ok = 1, c = 0;
            wconfirmed = sw[i].conf;
            whost_build(WK_SWITCH, nb, WF_MAX, 200000.0, 0, pre2);
            r = wchain(nb, sw[i].mode, sw[i].sticky, 0);
            wconfirmed = 0;
            for (k = 0; k < nb; k++) {
                if (wb[k].acks != sw[i].ws) ok = 0;
                if (wb[k].garbage > c) c = wb[k].garbage;
            }
            if (!ok || r.missing || r.worst_first != pre2 || r.relock || (!sw[i].second && c) ||
                c > (int)ULTRA_WSCFG_COPIES || (!sw[i].ws && r.dark)) {
                fails++;
                printf("  wire sweep FAIL seed %llu: switch case %d, %d boards: acks ok %d missing %d first %d relock %d "
                       "colour %d\n", seed, i, nb, ok, r.missing, r.worst_first, r.relock, c);
            }
        }
    }
    return fails;
}

static void wire_sweep(int seeds)
{
    int sd, bad = 0, fails = 0;
    for (sd = 1; sd <= seeds; sd++) {
        int f = wire_sweep_seed((unsigned long long)sd);
        fails += f;
        bad += f != 0;
    }
    printf("wire sweep: %d seeds x (fresh + preamble, 7 wrong stored protocols, 2 live switches, 9 stale cases, 6 switch "
           "cases) x 1-10 boards: %d failing seeds, %d failing chains\n", seeds, bad, fails);
    CHECK(bad == 0, "wire sweep: %d failing seeds", bad);
}

static void test_wire(void)
{
    static const char *const kn[] = { "WS2805", "WS2915", "Ultra" };
    static const proto_t kp[] = { PROTO_WS2805, PROTO_WS2915_32, PROTO_ULTRA };
    static const proto_mode_t km[] = { PMODE_WS2805, PMODE_WS2915, PMODE_ULTRA };
    int nb, kind, bad, worst;
    wres_t r;

    printf("wire: bit- and gap-accurate frames through chains of 1-10 boards (real tracker, gate plans, decisions)\n");
    if (getenv("WIRE_DEBUG_SEED")) {                 /* one sweep seed again, with every garbage frame printed */
        wdebug = 1;
        wire_sweep_seed((unsigned long long)atoi(getenv("WIRE_DEBUG_SEED")));
        wdebug = 0;
        wrs = 0x2545F4914F6CDD1DULL;
    }
    if (getenv("TEST_SWEEP")) {
        wire_sweep(atoi(getenv("TEST_SWEEP")) > 0 ? atoi(getenv("TEST_SWEEP")) : 200);   /* 200: the confirmation run */
        wrs = 0x2545F4914F6CDD1DULL;                     /* the normal run below as always */
    }

    /* negative control: the format 1 layout (header first). Board 0 sees header + slot; every later board only gets a
     * header-only frame and then its slot as a frame without header (the closed gate's 320 us LOW ends the frame) */
    bad = 0;
    for (nb = 2; nb <= WB_MAX; nb++) {
        whost_build(WK_LEGACY, nb, 8, 40000.0, 0, 0);
        r = wchain(nb, PMODE_ULTRA, PROTO_NONE, 1);
        if (r.legacy_hits != 0 || wb[0].legacy_slot_frames == 0) bad++;
        if (wb[1].frames_seen < 2 * wb[0].frames_seen - 2) bad++;           /* board 1: two frames per frame */
    }
    CHECK(bad == 0, "format 1 layout: boards after the first never see the header with their slot (%d)", bad);
    printf("  format 1 layout (header first): board 1 splits every frame in two, boards 1..n never get header + slot "
           "-> the bug is reproduced\n");

    /* VERSION 2 layout, forced Ultra: every board sees its slot first and the header last, in every frame */
    bad = 0;
    worst = 0;
    for (nb = 1; nb <= WB_MAX; nb++) {
        whost_build(WK_ULTRA, nb, 24, 40000.0, 0, 0);
        r = wchain(nb, PMODE_ULTRA, PROTO_NONE, 0);
        if (r.slot_frames_min < 24 || r.garbage || r.missing || r.worst_first != 0 || r.sync > 52.0) bad++;
        for (int k = 0; k < nb; k++) if (wb[k].shown != 24) bad++;
        if (r.sync > worst) worst = (int)r.sync;
    }
    CHECK(bad == 0, "VERSION 2 forced Ultra: every board, every frame, its own slot (%d)", bad);
    printf("  VERSION 2 (header last), forced Ultra, 1-10 boards: every board applies its own slot from every frame; "
           "clock reference within %d us of the frame end\n", worst);

    /* broadcast / extension slots (reserved, ultra_protocol.h): k = 1, 2 slots of types 0x80.. after the last board's
     * slot. 256 x (N + k) + 64 bits is an Ultra length, so lock and classification don't change: every board applies
     * only its own slot, and the k slots + the header leave the last board as sent (what a later firmware would read).
     * Forced Ultra; fresh auto + preamble; live switch from boards locked to WS (the broadcast slots DARK in the
     * preamble too). Then the hub counts one board too few: the last board takes the first broadcast slot as its own
     * slot, and the real engine ignores it. */
    {
        int kx, bad3 = 0, tails = 0, tails_ok = 0, ign = 0, k2;
        for (kx = 1; kx <= WX_MAX; kx++) {
            wext = kx;
            for (nb = 1; nb <= WB_MAX; nb++) {
                int pre = 4 * nb + 2;
                whost_build(WK_ULTRA, nb, 24, 40000.0, 0, 0);
                r = wchain(nb, PMODE_ULTRA, PROTO_NONE, 0);
                if (r.slot_frames_min < 24 || r.garbage || r.missing || r.worst_first != 0 || r.fwd_mis || r.sync > 52.0) bad3++;
                for (k2 = 0; k2 < nb; k2++) if (wb[k2].shown != 24) bad3++;
                wtail_count(nb, &tails, &tails_ok);
                whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre);
                r = wchain(nb, PMODE_AUTO, PROTO_NONE, 0);
                if (r.garbage || r.missing || r.worst_first != pre || r.writes != nb) bad3++;
                for (k2 = 0; k2 < 2; k2++) {
                    wconfirmed = 1;
                    whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre);
                    r = wchain(nb, PMODE_AUTO, k2 ? PROTO_WS2915_32 : PROTO_WS2805, 0);
                    wconfirmed = 0;
                    if (r.garbage || r.missing || r.worst_first != pre) bad3++;
                }
                if (nb >= 2) {
                    whost_build(WK_ULTRA, nb - 1, 24, 40000.0, 0, 0);
                    r = wchain(nb, PMODE_ULTRA, PROTO_NONE, 0);
                    if (r.garbage || r.missing != 1 || wb[nb - 1].first_f >= 0 || wb[nb - 1].bcast_ignored != 24) bad3++;
                    ign += wb[nb - 1].bcast_ignored;
                }
            }
        }
        wext = 0;
        CHECK(bad3 == 0 && tails_ok == tails, "broadcast slots: every board its own slot only, the tail passes untouched "
              "(%d, %d of %d tails)", bad3, tails_ok, tails);
        printf("  broadcast slots (1-2 of type 0x80.. after the last slot), 1-10 boards, forced / fresh auto + preamble / "
               "live switch from WS: every board applies only its own slot; the last board passes %d of %d frames' "
               "broadcast slots + header on untouched; hub one board short: the last board ignores the broadcast slot "
               "as its own (%d frames, engine unchanged)\n", tails_ok, tails, ign);
    }

    /* header flags: EXT ("must understand", no extension supported yet) frames apply no slot but keep the lock and the
     * clock sync; HINT frames apply as usual. Every 3rd frame EXT, every 3rd HINT */
    {
        int bad4 = 0, k2;
        wflags = 1;
        for (nb = 1; nb <= WB_MAX; nb++) {
            int pre = 4 * nb + 2, first = pre % 3 == 1 ? pre + 1 : pre;
            whost_build(WK_ULTRA, nb, 24, 40000.0, 0, 0);
            r = wchain(nb, PMODE_ULTRA, PROTO_NONE, 0);
            if (r.garbage || r.missing || r.sync > 52.0 || r.fwd_mis) bad4++;
            for (k2 = 0; k2 < nb; k2++) if (wb[k2].shown != 16 || wb[k2].ext_frames != 8) bad4++;
            whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre);
            r = wchain(nb, PMODE_AUTO, PROTO_NONE, 0);
            if (r.garbage || r.missing || r.worst_first != first || r.writes != nb) bad4++;
        }
        wflags = 0;
        CHECK(bad4 == 0, "header EXT / HINT through chains of 1-10 boards (%d)", bad4);
        printf("  header flags, 1-10 boards (every 3rd frame EXT, every 3rd HINT): EXT frames apply no slot, the lock and "
               "the clock sync stay; HINT frames apply; fresh auto locks through them\n");
    }

    /* boot, controller already sending at 50 fps / the hub starting later: forced and stored (sticky) protocols
     * show the first complete frame after the 24-30 ms reset on every board */
    for (kind = 0; kind < 3; kind++) {
        int sticky;
        for (sticky = 0; sticky < 2; sticky++) {
            bad = 0;
            for (nb = 1; nb <= WB_MAX; nb++) {
                int lim;
                if (kind == WK_ULTRA) whost_build(WK_ULTRA, nb, 16, 200000.0, 0, 0);
                else whost_build(kind, nb, 16, 3000.0, 20000.0, 0);
                r = wchain(nb, sticky ? PMODE_AUTO : km[kind], sticky ? kp[kind] : PROTO_NONE, 0);
                lim = kind == WK_ULTRA ? 0 : 2;                                /* frames at 3, 23, 43 ms: 43 ms */
                if (r.garbage || r.missing || r.worst_first > lim || r.writes) bad++;
            }
            CHECK(bad == 0, "%s %s: first complete frame on every board (%d)", sticky ? "stored" : "forced", kn[kind], bad);
        }
    }
    printf("  forced / stored WS2805, WS2915, Ultra, 1-10 boards: every board shows the first complete frame after "
           "its reset, no flash write\n");

    /* fresh boards (nothing stored), auto: WS locks after 3 frames per board, Ultra after one */
    for (kind = 0; kind < 2; kind++) {
        whost_build(kind, WB_MAX, WF_MAX, 200000.0, 20000.0, 0);
        r = wchain(WB_MAX, PMODE_AUTO, PROTO_NONE, 0);
        CHECK(!r.garbage && !r.missing && r.worst_first <= 4 * WB_MAX + 1 && r.writes == WB_MAX,
              "fresh auto %s: %d, %d garbage, %d writes", kn[kind], r.worst_first, r.garbage, r.writes);
        printf("  fresh auto %s, 10 boards at 50 fps: last board from frame %d, no garbage\n", kn[kind], r.worst_first);
    }
    for (nb = 1; nb <= WB_MAX; nb++) {
        whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, 0);
        r = wchain(nb, PMODE_AUTO, PROTO_NONE, 0);
        CHECK(!r.garbage && !r.missing && r.writes == nb, "fresh auto Ultra, %d boards (%d garbage)", nb, r.garbage);
        for (int k = 0; k < nb; k++) CHECK(wb[k].first_f <= 2 * k + 2, "fresh Ultra board %d of %d from frame %d", k, nb, wb[k].first_f);
        if (nb == WB_MAX) printf("  fresh auto Ultra, back to back: board k shows its slot from frame <= 2k + 2 (10 boards: %d)\n", r.worst_first);
    }
    for (nb = 1; nb <= WB_MAX; nb++) {
        int pre = 4 * nb + 2;
        whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre);
        r = wchain(nb, PMODE_AUTO, PROTO_NONE, 0);
        CHECK(!r.garbage && !r.missing && r.worst_first == pre, "fresh auto Ultra + preamble %d, %d boards: first command %d",
              pre, nb, r.worst_first);
    }
    printf("  fresh auto Ultra with the preamble (4N + 2 DARK frames): every board applies the first command frame\n");

    /* the stored protocol is wrong (all boards), 1-10 boards: nothing misaligned is ever shown, all re-lock */
    {
        static const int cases[][2] = { { 0, PROTO_ULTRA }, { 1, PROTO_ULTRA }, { 0, PROTO_WS2915_32 }, { 1, PROTO_WS2805 },
                                        { 2, PROTO_WS2805 }, { 2, PROTO_WS2915_32 }, { 2, PROTO_WS2915_26 } };
        int i, worst2 = 0, runs = 0, running = 0;
        bad = 0;
        for (i = 0; i < 7; i++) {
            for (nb = 1; nb <= WB_MAX; nb++) {
                int ultra = cases[i][0] == WK_ULTRA;
                if (ultra) whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, 4 * nb + 2);
                else whost_build(cases[i][0], nb, WF_MAX, 3000.0, 20000.0, 0);
                r = wchain(nb, PMODE_AUTO, (proto_t)cases[i][1], 0);
                if (r.garbage || r.missing || r.writes != nb) {
                    bad++;
                    printf("  FAIL detail: stored %d, sent %s, %d boards: garbage %d missing %d writes %d\n", cases[i][1],
                           kn[cases[i][0]], nb, r.garbage, r.missing, r.writes);
                }
                if (r.worst_first > worst2) worst2 = r.worst_first;
                runs++;
                if (ultra) {
                    /* the hub was already running: no preamble, refresh frames 200 ms apart */
                    whost_build(WK_ULTRA, nb, 24, 3000.0, 200000.0, 0);
                    r = wchain(nb, PMODE_AUTO, (proto_t)cases[i][1], 0);
                    running += r.garbage;
                    if (r.missing) bad++;
                }
            }
        }
        CHECK(bad == 0, "stored protocol wrong: no garbage, all re-lock (%d)", bad);
        CHECK(running == 0, "stored WS protocol, hub already running: no garbage (%d)", running);
        printf("  stored protocol wrong (%d chains: 7 protocol pairs x 1-10 boards): nothing misaligned shown, all "
               "re-locked (last board by frame %d); hub already running: %d misaligned\n", runs, worst2, running);
    }

    /* live switch: boards locked (confirmed) to WS2805 / WS2915 by a running WS controller, then an Ultra hub takes
     * over without a power cycle. A locked board shows its WS window at once (it cannot know yet), anywhere in the
     * slots; it re-locks on the trailer headers (3 frames: its capture is its WS window, never an Ultra slot), and
     * the board after it sees Ultra lengths only from then on. DARK slots show dark; NOP slots (CRC) show colour. */
    {
        int k2, dark4 = 0, late4 = 0, late2 = 0, garb = 0, garb_nop = 0, miss = 0;
        wconfirmed = 1;
        for (k2 = 0; k2 < 2; k2++) {
            for (nb = 1; nb <= WB_MAX; nb++) {
                int pre4 = 4 * nb + 2, pre2 = 2 * nb + 3;
                proto_t st = k2 ? PROTO_WS2915_32 : PROTO_WS2805;
                whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre4);
                r = wchain(nb, PMODE_AUTO, st, 0);
                garb += r.garbage; dark4 += r.dark; miss += r.missing;
                if (r.worst_first != pre4) late4++;
                whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre2);
                r = wchain(nb, PMODE_AUTO, st, 0);
                if (r.worst_first != pre2 || r.missing) late2++;
                wpre_nop = 1;
                whost_build(WK_ULTRA, nb, WF_MAX, 200000.0, 0, pre4);
                r = wchain(nb, PMODE_AUTO, st, 0);
                garb_nop += r.garbage;
                wpre_nop = 0;
            }
        }
        {                                                /* the margin: the shortest preamble that works, 10 boards */
            int pmin[2];
            for (k2 = 0; k2 < 2; k2++) {
                for (pmin[k2] = 2 * WB_MAX; pmin[k2] < 4 * WB_MAX + 2; pmin[k2]++) {
                    whost_build(WK_ULTRA, WB_MAX, WF_MAX, 200000.0, 0, pmin[k2]);
                    r = wchain(WB_MAX, PMODE_AUTO, k2 ? PROTO_WS2915_32 : PROTO_WS2805, 0);
                    if (!r.garbage && !r.missing && r.worst_first == pmin[k2]) break;
                }
            }
            printf("  live switch, 10 boards: shortest working preamble %d (WS2805) / %d (WS2915) DARK frames, sent 42\n",
                   pmin[0], pmin[1]);
        }
        wconfirmed = 0;
        CHECK(garb == 0 && miss == 0 && late4 == 0, "live switch WS -> Ultra, preamble 4N + 2 DARK: no colour, all from "
              "the first command (%d garbage, %d missing, %d late)", garb, miss, late4);
        CHECK(garb_nop > 0, "negative control: a NOP preamble shows colour on locked WS boards");
        printf("  live switch (boards locked to WS2805 / WS2915, then the Ultra hub), 1-10 boards: preamble of 4N + 2 "
               "DARK frames -> every board from the first command, no colour (%d dark frames shown); 2N + 3 -> %d of 20 "
               "chains late; NOP slots instead of DARK -> %d frames of colour\n", dark4, late2, garb_nop);
    }

    /* dig2analog_ultra.switch_boards_to_ultra: the hub's WS config frames "protocol = Ultra" (ultra_switch_frame:
     * WS2805, then WS2915) through running chains, then the start-up preamble and the commands */
    {
        static const struct { proto_mode_t mode; proto_t sticky; int conf, ws, second; const char *name; } sc[] = {
            { PMODE_WS2805, PROTO_NONE, 0, 1, 0, "forced WS2805" },
            { PMODE_WS2915, PROTO_NONE, 0, 1, 1, "forced WS2915" },
            { PMODE_AUTO, PROTO_WS2805, 1, 1, 0, "auto, locked to WS2805" },
            { PMODE_AUTO, PROTO_WS2915_32, 1, 1, 1, "auto, locked to WS2915" },
            { PMODE_ULTRA, PROTO_NONE, 0, 0, 0, "forced Ultra" },
            { PMODE_AUTO, PROTO_ULTRA, 1, 0, 0, "auto, locked to Ultra" },
        };
        int i, k, cmax[6] = { 0 }, bad2 = 0;
        for (i = 0; i < 6; i++) {
            for (nb = 1; nb <= WB_MAX; nb++) {
                int pre = (int)ULTRA_SWITCH_WS_FRAMES + 4 * nb + 2, acks_ok = 1, c = 0;
                wconfirmed = sc[i].conf;
                whost_build(WK_SWITCH, nb, WF_MAX, 200000.0, 0, pre);
                r = wchain(nb, sc[i].mode, sc[i].sticky, 0);
                wconfirmed = 0;
                for (k = 0; k < nb; k++) {
                    if (wb[k].acks != sc[i].ws) acks_ok = 0;
                    if (wb[k].garbage > c) c = wb[k].garbage;
                }
                if (c > cmax[i]) cmax[i] = c;
                if (!acks_ok || r.missing || r.worst_first != pre || r.relock || (!sc[i].second && c) ||
                    c > (int)ULTRA_WSCFG_COPIES || (!sc[i].ws && r.dark)) {
                    bad2++;
                    printf("  FAIL detail: switch, %s, %d boards: acks ok %d, missing %d, first %d (want %d), re-locks %d, "
                           "colour %d, dark %d\n", sc[i].name, nb, acks_ok, r.missing, r.worst_first, pre, r.relock, c, r.dark);
                }
            }
        }
        CHECK(bad2 == 0, "switch_boards_to_ultra: every WS board takes it once, no re-lock, all from the first command (%d)", bad2);
        printf("  switch_boards_to_ultra (WS config frames, WS2805 then WS2915, then the preamble), 1-10 boards: every board "
               "from the first command, no re-lock anywhere. Forced / auto WS2805: taken, no colour. Forced / auto WS2915: "
               "taken, colour on at most %d / %d frames per board (the WS2805 config frames). Ultra boards: ignored\n",
               cmax[1], cmax[3]);

        /* mixed pins (the limit): boards behind a board in Ultra mode or of the other WS format */
        {
            static const char *const mn[] = { "Ultra board, then 3 forced WS2805", "3 forced WS2805, then a forced WS2915",
                                              "forced WS2915, then 3 forced WS2805" };
            int m;
            for (m = 0; m < 3; m++) {
                int sw = 0, ws = 0;
                for (k = 0; k < 4; k++) {
                    proto_mode_t md = m == 0 ? (k == 0 ? PMODE_ULTRA : PMODE_WS2805)
                                    : m == 1 ? (k < 3 ? PMODE_WS2805 : PMODE_WS2915) : (k == 0 ? PMODE_WS2915 : PMODE_WS2805);
                    wmix_mode[k] = md;
                    wmix_sticky[k] = PROTO_NONE;
                    wmix_conf[k] = 0;
                }
                wmixed = 1;
                whost_build(WK_SWITCH, 4, WF_MAX, 200000.0, 0, (int)ULTRA_SWITCH_WS_FRAMES + 18);
                r = wchain(4, PMODE_AUTO, PROTO_NONE, 0);
                wmixed = 0;
                for (k = 0; k < 4; k++) {
                    if (wmix_mode[k] != PMODE_ULTRA) { ws++; sw += wb[k].acks; }
                }
                printf("  switch, mixed pin (%s): %d of %d WS boards switched\n", mn[m], sw, ws);
            }
        }
    }

    /* the residual risk: the hub is already running (no preamble), and boards power up at random moments with a stale
     * stored WS protocol (sticky, not forced): back-to-back frames, 50 fps, refresh at rest. Then the same with only
     * some boards stale and the others running in Ultra. */
    {
        static const proto_t st[] = { PROTO_WS2805, PROTO_WS2915_32, PROTO_WS2915_26 };
        static const double per[] = { 0, 20000.0, 200000.0 };
        static const char *const pn[] = { "back to back", "50 fps", "refresh every 200 ms" };
        int i, pp, run, k, garb = 0, mis = 0, closed = 0, miss = 0, chains = 0, boards = 0, last[3] = { 0, 0, 0 };
        for (i = 0; i < 3; i++)
            for (pp = 0; pp < 3; pp++)
                for (nb = 1; nb <= WB_MAX; nb++)
                    for (run = 0; run < 4; run++) {
                        int nf = per[pp] > 100000.0 ? 32 : per[pp] > 0 ? 40 : WF_MAX;
                        whost_build(WK_ULTRA, nb, nf, wurand(0, 30000.0), per[pp], 0);
                        r = wchain(nb, PMODE_AUTO, st[i], 0);
                        garb += r.garbage; mis += r.fwd_mis; closed += r.fwd_closed; miss += r.missing;
                        chains++; boards += nb;
                        if (nb == WB_MAX) {                  /* frames until the last board shows (after its power-up) */
                            int f0 = 0;
                            while (f0 < nf && whost_end[f0] < 30000.0) f0++;
                            if (r.worst_first - f0 > last[pp]) last[pp] = r.worst_first - f0;
                        }
                    }
        CHECK(garb == 0 && miss == 0, "stale stored WS protocol, hub running: nothing shown (%d), all boards in Ultra (%d)",
              garb, miss);
        printf("  stale stored WS2805 / WS2915 (32, 26) on every board, hub running with real slots, no preamble, random "
               "power-up moments (%d chains, %d boards): 0 frames shown, every board in Ultra; 10 boards: the last one "
               "%d / %d / %d frames after power-up (%s / %s / %s). %d misaligned forwards (a stale board forwards with its "
               "stored WS plan until the frames it gets are aligned; nobody shows them), %d frames closed\n", chains, boards,
               last[0], last[1], last[2], pn[0], pn[1], pn[2], mis, closed);

        garb = mis = closed = miss = chains = boards = 0;
        for (pp = 0; pp < 3; pp++)
            for (nb = 2; nb <= WB_MAX; nb++)
                for (run = 0; run < 6; run++) {
                    int nf = per[pp] > 100000.0 ? 32 : per[pp] > 0 ? 40 : WF_MAX, stale = 0;
                    for (k = 0; k < nb; k++) {
                        int sk = wurand(0, 1) < 0.5;
                        wmix_mode[k] = PMODE_AUTO;
                        wmix_sticky[k] = sk ? st[(int)wurand(0, 2.999)] : PROTO_ULTRA;
                        wmix_conf[k] = !sk;
                        stale += sk;
                    }
                    wmixed = 1;
                    whost_build(WK_ULTRA, nb, nf, wurand(0, 30000.0), per[pp], 0);
                    r = wchain(nb, PMODE_AUTO, PROTO_NONE, 0);
                    wmixed = 0;
                    garb += r.garbage; mis += r.fwd_mis; closed += r.fwd_closed; miss += r.missing;
                    chains++; boards += stale;
                }
        CHECK(garb == 0 && miss == 0, "some boards stale, the others in Ultra: nothing shown (%d), all in Ultra (%d)", garb,
              miss);
        printf("  some boards stale (random half), the others running in Ultra (%d chains, %d stale boards): 0 frames shown, "
               "every board in Ultra; %d misaligned forwards, %d closed\n", chains, boards, mis, closed);
    }

    /* a board FORCED to WS (boot_protocol ws2805 / ws2915) on the Ultra hub: it shows its WS window of every frame */
    {
        int m;
        for (m = 0; m < 2; m++) {
            int k, shown = 0, frames = 0;
            whost_build(WK_ULTRA, 3, 24, 40000.0, 0, 0);
            r = wchain(3, m ? PMODE_WS2915 : PMODE_WS2805, PROTO_NONE, 0);
            for (k = 0; k < 3; k++) { shown += wb[k].garbage + wb[k].dark; frames += wb[k].frames; }
            printf("  forced %s on the Ultra hub (3 boards, 24 frames): %d WS windows shown (of %d frames), %d as colour, "
                   "%d Ultra slots applied\n", m ? "WS2915" : "WS2805", shown, frames, r.garbage, 3 - r.missing);
        }
    }

    /* regression (found by the sweep, 2026-10-12): 6 boards with a stale stored WS2915 (32) protocol each pass the 32
     * header bits and block 80, so the 7th sees 32 bits, a ~600 us LOW gap, then the rest of the frame. With a boot
     * arm after 3 idle ticks, a board powering up inside that gap took the rest as a frame: 32 + 6 x 80 = 512 bits =
     * 2 slots, so it started at board 2's slot (CRC ok) and ended with the header, and the one-frame Ultra lock
     * applied board 2's slot on board 6. Fixed: the boot arm needs 10 ms of idle line (FT_BOOT_TICKS). */
    {
        int gb;
        wgap_board = 6;
        wgap_frame = 0;                              /* frames from 31 ms: every board before it is up */
        whost_build(WK_ULTRA, 7, 24, 31000.0, 0, 0);
        r = wchain(7, PMODE_AUTO, PROTO_WS2915_32, 0);
        gb = wb[6].garbage;
        wgap_board = -1;
        CHECK(gb == 0 && r.missing == 0, "power-up inside a WS2915 gap: no other board's slot (%d), all boards in Ultra", gb);
        printf("  7 boards with a stale stored WS2915, the 7th powers up inside the 600 us gap of a frame: %d frames of "
               "another board's slot (boot arm after 10 ms of idle line)\n", gb);
    }
}
