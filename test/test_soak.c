/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Time-warp soak test (0.6.2): the whole firmware - src/app/main.c itself with every core module - on the host, on a
 * model of the hardware below hal/hw.h, started a little before every counter's wrap and run for simulated days.
 *
 * The model: the 20 kHz poll tick calls main.c's poll_isr; the DIN wire carries real WS bit waveforms (T0H 300 ns,
 * T1H 750 ns, 1.25 us per bit) of an Ultra hub's frames (the board's slot + the header trailer) that the SPI DMA
 * samples at 36 MHz into din_ring and the edge counter counts (TIM4, reset by gate_rearm), so the firmware's own frame
 * tracker, ring copies, decoder, frame_rx, proto and the engine see them as on the board. The current sense reads a
 * load model (the outputs x the strips' currents + the logic), PA4 a 24 V supply (dips on request), the temperature
 * sensor a steady die. The flash stores are hal/hw.c's (same records, same pages, at the real addresses), counted per
 * page; a write stalls the model as on the chip (2 ms, +16 ms per erase). The console is a byte queue.
 * main() runs in a fiber: every pass of its loop kicks the watchdog, which hands back to the scenario here, so the
 * scenarios read as plain code (run, jump, send, check). A jump moves time on by up to a second at once (the board's
 * time bases as a flash stall does, then 20 real ticks), for the sparse stretches of days.
 *
 * Scenarios (soak_tests.exe <name>; build_host.cmd runs each in its own process: a fresh boot):
 *   wrap   boot 65 s before the 1 ms counter's wrap, 0.2 s before the 20 kHz tick's, 30 s before the host clock's: a
 *          20 s transition across the ms wrap, a breathe effect across all three, the absolute-deadline control
 *   long   52 days: a 28-day stretch dark (energy), a dark hold (IDENTIFY 3) whose fade must not replay 2^32 ms later,
 *          supply dips (one 100 ms after the ms wrap), 3 days without a host (signal loss holds: the effect runs on),
 *          the tick's 20 wraps, the host's ms wrap; the console's uptime and ages at the end; every flash write
 *   fx     no host ever: the power-on effects (candle, breathe) from the stored effect, 3 days, past the tick's wrap
 *   cfg    an automation: Ultra CONFIG 23 (stored) flipped every second for a day, then every minute for 3 days;
 *          calibrations (IDENTIFY 2) every minute for 2 hours: the settings / calibration write budgets, and that the
 *          last value is the one stored
 * Steps are bounded by 2 ms of an effect's slope: the host clock runs 0.6 % fast, so the engine's ms (integer)
 * advances by 2 in one board ms every ~170 ms.
 * Each checks: no stuck or false events (trips, limit events, D9, dips, signal loss), no lost transitions, no flash
 * writes but the expected ones, outputs continuous across every wrap. The negative control (wrap): an absolute
 * deadline `now >= t_end` armed before the ms wrap fires at once, where the firmware's `now - t0 >= d` waits. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define main fw_main
#include "../src/app/main.c"
#undef main
#include "../src/core/scratch.h"

/* kernel32 (no windows.h: its macros would meet the firmware's names) */
__declspec(dllimport) void *__stdcall VirtualAlloc(void *addr, size_t size, unsigned long type, unsigned long prot);
__declspec(dllimport) void *__stdcall ConvertThreadToFiber(void *param);
__declspec(dllimport) void *__stdcall CreateFiber(size_t stack, void(__stdcall *start)(void *), void *param);
__declspec(dllimport) void __stdcall SwitchToFiber(void *fiber);

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
                                       printf("\n"); } } while (0)

/* ================================================================================================ hardware model */
soak_tim_t soak_tim4;
soak_systick_t soak_systick;
soak_adc_t soak_adc1;
soak_gpio_t soak_gpiob = { 0xFFFFFFFFu };                /* TP9 pulled up: no test jig */

uint16_t pwm_tab1[PATTERN_MAX_LEN * 4], pwm_tab2[PATTERN_MAX_LEN * 6], pwm_tab3[PATTERN_MAX_LEN * 6];
uint16_t din_ring[DIN_RING_WORDS], din_copy[DIN_COPY_WORDS];
static uint16_t sim_tab[5][256];

static void *fib_main, *fib_fw;
static uint64_t sim_ns, sim_ticks;                       /* real time, real ticks since the start */
static uint64_t ring_words;                              /* SPI words the DMA wrote */
static int ring_dirty;                                   /* din_ring holds a frame's samples somewhere */
static uint32_t edges_total, edges_rearm;                /* DIN falling edges; at the last gate re-arm */
static uint8_t wf_bits[64];                              /* the frame on the wire (MSB first) */
static uint32_t wf_nbits, wf_next_fall;
static uint64_t wf_t0;
static int wf_on;
static uint32_t vs_pending, supply_mv = 24000u;          /* injected group pairs since the last take; PA4's input */
static uint8_t sim_forced;                               /* pwm_force_off */
static uint32_t sim_led_on, sim_rearms;                  /* D9 switched on; gate re-arms */
static uint32_t short_ticks;                             /* ticks of a short still to come (the fast trip) */
static uint32_t sim_stalls;                              /* flash writes (the CPU stops) */

static uint64_t wf_fall(uint32_t i)
{
    return wf_t0 + (uint64_t)i * 1250u + (((wf_bits[i >> 3] >> (7u - (i & 7u))) & 1u) ? 750u : 300u);
}

static int wf_level(uint64_t t)
{
    uint64_t d;
    uint32_t i;
    if (!wf_on || t < wf_t0) {
        return 0;
    }
    d = t - wf_t0;
    i = (uint32_t)(d / 1250u);
    return i < wf_nbits && t < wf_fall(i);
}

/* the DMA and the edge counter up to real time t */
static void wave_to(uint64_t t)
{
    uint64_t target = t * 9u / 4000u, wend = wf_t0 + (uint64_t)wf_nbits * 1250u;
    if (target - ring_words > DIN_RING_WORDS && (!wf_on || wend < ring_words * 16u * 250u / 9u)) {
        if (ring_dirty) {
            memset(din_ring, 0, sizeof din_ring);        /* a long stretch of an idle line */
            ring_dirty = 0;
        }
        ring_words = target - DIN_RING_WORDS;
    }
    while (ring_words < target) {
        uint64_t s0 = ring_words * 16u, tw0 = s0 * 250u / 9u, tw1 = (s0 + 16u) * 250u / 9u;
        uint16_t w = 0;
        uint32_t j;
        if (wf_on && tw1 >= wf_t0 && tw0 <= wend) {
            for (j = 0; j < 16u; j++) {
                if (wf_level((s0 + j) * 250u / 9u)) {
                    w |= (uint16_t)(0x8000u >> j);
                }
            }
        }
        din_ring[ring_words % DIN_RING_WORDS] = w;
        ring_dirty |= w != 0u;
        ring_words++;
    }
    while (wf_on && wf_next_fall < wf_nbits && wf_fall(wf_next_fall) <= t) {
        edges_total++;
        wf_next_fall++;
    }
    if (wf_on && wf_next_fall == wf_nbits && t > wend + 1000000u) {
        wf_on = 0;
    }
}

static void wire_frame(const uint8_t *bytes, uint32_t nbits, uint64_t t0)
{
    memcpy(wf_bits, bytes, (nbits + 7u) / 8u);
    wf_nbits = nbits;
    wf_t0 = t0;
    wf_next_fall = 0;
    wf_on = 1;
}

/* the load: logic 45 mA + each output's duty x its strip (a short: 4095, the fast trip) */
static const uint32_t ld_ma[5] = { 1500, 1500, 1500, 1200, 1200 };
static uint32_t load_raw(void)
{
    uint32_t ma = 45u, i, g = is.ma_per_lsb_q16 ? is.ma_per_lsb_q16 : 262410u, raw;
    if (short_ticks) {
        short_ticks--;
        return 4095u;
    }
    for (i = 0; i < 5u && !sim_forced; i++) {
        ma += (uint32_t)out_now[i] * ld_ma[i] / 65535u;
    }
    raw = (uint32_t)(((uint64_t)ma << 16) / g);
    return raw > 4095u ? 4095u : raw;
}

/* PA4: one pair of conversions of mv at the supply (220 k / 10 k, 3.28 V) */
static uint32_t pair_of(uint32_t mv)
{
    return 2u * (uint32_t)((double)mv * 10.0 / 230.0 / 3280.0 * 4095.0 + 0.5);
}

/* ---- hal/hw.h */
uint16_t *pwm_table(uint8_t ch, uint8_t *stride) { *stride = 1; return sim_tab[ch < 5u ? ch : 0u]; }
const uint16_t *pwm_spread_arr(void) { return 0; }
void hw_init(void) {}
void stack_paint(void) {}
uint32_t stack_used(void) { return 0x2A0u; }
uint32_t hw_uid_hash(void) { return 0x5EED1234u; }
uint32_t hw_reset_flags(void) { return 0x0C000000u; }   /* power-on + pin */
void pwm_start(uint32_t s, uint8_t l, uint8_t st) { (void)s; (void)l; (void)st; }
uint8_t pwm_right_aligned(uint8_t ch) { (void)ch; return 0; }
void pwm_set_phase(uint16_t o2, uint16_t o3) { (void)o2; (void)o3; }
void gate_rearm(const gate_plan_t *g) { (void)g; edges_rearm = edges_total; sim_rearms++; }
void gate_force_closed(void) {}
uint32_t din_ring_pos(void) { return (uint32_t)(ring_words % DIN_RING_WORDS); }
void hw_adc_init(void) {}
void hw_adc_ms(void) { vs_pending++; }
void hw_vsense_take(uint32_t sum2[2], uint32_t *n)
{
    *n = vs_pending;
    sum2[0] = vs_pending * pair_of(supply_mv);
    sum2[1] = vs_pending * 2u * 2048u;
    vs_pending = 0;
}
void hw_temp_start(void) {}
uint8_t hw_temp_read(uint32_t *ts2, uint32_t *vref2) { *ts2 = 2u * 1710u; *vref2 = 2u * 1489u; return 1; }
void pwm_force_off(uint8_t off) { sim_forced = off; }
void led_set(uint8_t on) { sim_led_on += on != 0u; }
void watchdog_kick(void) { SwitchToFiber(fib_main); }   /* every main loop pass: back to the scenario */

/* ---- console */
static char con_in[512], con_out[1u << 16];
static uint32_t con_in_r, con_in_w, con_out_n;
int uart_getc(void) { return con_in_r < con_in_w ? (int)(unsigned char)con_in[con_in_r++] : -1; }
void uart_write(const char *s)
{
    while (*s && con_out_n + 1u < sizeof con_out) {
        con_out[con_out_n++] = *s++;
    }
    con_out[con_out_n] = 0;
}
void uart_kick(void) {}

/* ---- flash: the real addresses (VirtualAlloc), the stores as hal/hw.c, writes / erases per page */
#define FL_BASE 0x08000000u
static uint8_t *fl;
static uint32_t fl_writes[256], fl_erases[256];
static void sim_stall(uint32_t us);

static uint32_t flash_program(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i, us = 2000u;
    if (erase_addr != 0u) {
        memset(fl + (erase_addr - FL_BASE), 0xFF, 256u);
        fl_erases[(erase_addr - FL_BASE) / 256u]++;
        us += 16000u;
    }
    for (i = 0; i < halfwords; i++) {
        ((uint16_t *)(uintptr_t)addr)[i] = src[i];
    }
    if (halfwords) {
        fl_writes[(addr - FL_BASE) / 256u]++;
    }
    sim_stall(us);                                       /* the CPU stops, real time goes on */
    return us;
}

static uint8_t store_free, cal_free_, energy_next_;
static uint32_t store_seq, cal_seq_;
static uint16_t cal_cnt_;
static uint32_t scene_next_;
static uint8_t scene_erase_;
static uint16_t scene_seq_;

static uint32_t page_append(uint32_t page, uint8_t *next, uint32_t bytes, uint32_t n, uint32_t may_erase)
{
    uint32_t idx = *next, erase = 0;
    if (idx >= n) {
        if (!may_erase) {
            return 0;
        }
        erase = page;
        idx = 0;
    }
    *next = (uint8_t)(idx + 1u);
    return flash_program(page + idx * bytes, (const uint16_t *)core_scratch, bytes / 2u, erase);
}

int settings_load(settings_t *s, store_ext_t *ext)
{
    uint32_t seq;
    store_free = (uint8_t)store_next_free((const uint8_t *)SETTINGS_ADDR);
    if (store_find_ext((const uint8_t *)SETTINGS_ADDR, s, ext, &seq) < 0) {
        store_ext_clear(ext);
        return -1;
    }
    store_seq = seq;
    return 0;
}

uint32_t settings_store(const settings_t *s, const store_ext_t *ext)
{
    store_pack_ext(s, ext, ++store_seq, (store_slot_t *)core_scratch);
    return page_append(SETTINGS_ADDR, &store_free, STORE_SLOT_BYTES, STORE_SLOTS, 1u);
}

int scene_load(scene_t *sc)
{
    int newest = scene_log_find((const uint8_t *)SCENE_LOG_ADDR, sc, &scene_seq_);
    scene_next_ = (uint32_t)scene_log_next((const uint8_t *)SCENE_LOG_ADDR, newest, &scene_erase_);
    return newest < 0 ? -1 : 0;
}

uint32_t scene_store(const scene_t *sc)
{
    scene_rec_t *rec = (scene_rec_t *)core_scratch;
    uint32_t idx = scene_next_, erase = 0;
    if (scene_erase_) {
        erase = SCENE_LOG_ADDR + (idx / SCENE_RECS_PAGE) * SCENE_PAGE_BYTES;
    }
    scene_log_pack(sc, ++scene_seq_, rec);
    scene_next_ = (idx + 1u) % SCENE_RECS;
    scene_erase_ = (uint8_t)(scene_next_ % SCENE_RECS_PAGE == 0u);
    return flash_program(SCENE_LOG_ADDR + idx * SCENE_REC_BYTES, (const uint16_t *)rec, SCENE_REC_BYTES / 2u, erase);
}

int cal_load(isense_cal_t *c, uint16_t *count)
{
    cal_free_ = (uint8_t)cal_next_free((const uint8_t *)CAL_PAGE_ADDR);
    if (cal_find((const uint8_t *)CAL_PAGE_ADDR, c, &cal_seq_, &cal_cnt_) < 0) {
        cal_cnt_ = 0;
        *count = 0;
        return -1;
    }
    *count = cal_cnt_;
    return c->levels != 0u ? 0 : -1;
}

uint32_t cal_store(const isense_cal_t *c, uint16_t *count)
{
    if (c->levels != 0u) {
        cal_cnt_++;
    }
    cal_pack(c, ++cal_seq_, cal_cnt_, (cal_rec_t *)core_scratch);
    *count = cal_cnt_;
    return page_append(CAL_PAGE_ADDR, &cal_free_, CAL_REC_BYTES, CAL_RECS, 1u);
}

uint32_t energy_load(void)
{
    energy_next_ = (uint8_t)energy_log_next((const uint8_t *)ENERGY_ADDR);
    return energy_log_find((const uint8_t *)ENERGY_ADDR);
}

uint32_t energy_store(uint32_t wh, uint8_t may_erase)
{
    core_scratch[0] = wh;
    core_scratch[1] = ~wh;
    return page_append(ENERGY_ADDR, &energy_next_, ENERGY_REC_BYTES, ENERGY_RECS, may_erase);
}

uint32_t energy_clear(void)
{
    energy_next_ = 0;
    return flash_program(ENERGY_ADDR, 0, 0u, ENERGY_ADDR);
}

uint32_t scene_fx_store(const ultra_t *u, uint8_t mask, const uint8_t crc[ULTRA_LIGHTS])
{
    uint8_t *h = (uint8_t *)core_scratch;
    uint32_t i, us = flash_program(SCENE_FX_ADDR, 0, 0u, SCENE_FX_ADDR);
    if (mask == 0u) {
        return us;
    }
    us += flash_program(SCENE_FX_ADDR + 256u, 0, 0u, SCENE_FX_ADDR + 256u);
    core_scratch[2] = 0u;
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        if (mask & (1u << i)) {
            us += flash_program(SCENE_FX_ADDR + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES, (const uint16_t *)&u->light[i],
                                ULTRA_HEAD_BYTES / 2u, 0u);
        }
        h[5u + i] = crc[i];
    }
    core_scratch[0] = ULTRA_FX_MAGIC;
    h[4] = mask;
    return us + flash_program(SCENE_FX_ADDR, (const uint16_t *)h, ULTRA_FX_HDR / 2u, 0u);
}

uint32_t hw_flash_half(uint32_t addr, uint16_t v) { return flash_program(addr, &v, 1u, 0u); }

static uint32_t page_of(uint32_t addr) { return (addr - FL_BASE) / 256u; }
static uint32_t writes_in(uint32_t addr, uint32_t pages)
{
    uint32_t n = 0, i;
    for (i = 0; i < pages; i++) {
        n += fl_writes[page_of(addr) + i];
    }
    return n;
}
#define W_SETTINGS() writes_in(SETTINGS_ADDR, 1u)
#define W_SCENE()    writes_in(SCENE_LOG_ADDR, SCENE_LOG_PAGES)
#define W_FX()       writes_in(SCENE_FX_ADDR, 2u)
#define W_ENERGY()   writes_in(ENERGY_ADDR, 1u)
#define W_CAL()      writes_in(CAL_PAGE_ADDR, 1u)

/* ================================================================================================ the host (hub) */
static int host_on;
static uint32_t host_period = 400u;                      /* ticks between frames: 50 fps */
static uint64_t host_next, host_us0;
static const int64_t host_ppm = 6000;                    /* its clock 0.6 % fast against the board's */
static uint8_t hq[256][32];
static uint32_t hq_head, hq_tail, hq_rep, host_frames;
static uint8_t useq = 1;

static uint32_t host_us(uint64_t t_ns)
{
    uint64_t us = t_ns / 1000u;
    return (uint32_t)(host_us0 + us + us * (uint64_t)host_ppm / 1000000u);
}

static void host_send(const uint8_t *s)
{
    memcpy(hq[hq_tail++ % 256u], s, 32);
}

/* a frame: the board's slot (each command 3 times, else a NOP) and the header with the host time at the frame end */
static void host_frame(void)
{
    uint8_t f[40];
    uint64_t t0 = sim_ns + 7000u;
    if (hq_head != hq_tail) {
        memcpy(f, hq[hq_head % 256u], 32);
        if (++hq_rep >= 3u) {
            hq_rep = 0;
            hq_head++;
        }
    } else {
        ultra_slot_nop(f);
    }
    ultra_build_header(f + 32, host_us(t0 + 320u * 1250u));
    wire_frame(f, 320u, t0);
    host_frames++;
}

static uint8_t seq_next(void) { return useq++; }

static void cmd_def(uint8_t l, uint8_t ch_first, uint8_t n, uint16_t gamma_x1000)
{
    static const uint8_t rgb[3] = { ULTRA_ROLE_RED, ULTRA_ROLE_GREEN, ULTRA_ROLE_BLUE };
    uint8_t s[32], roles[5] = { 0, 0, 0, 0, 0 };
    uint16_t mn[5] = { 0, 0, 0, 0, 0 }, mx[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint32_t i;
    for (i = 0; i < n; i++) {
        roles[ch_first + i] = n == 3u ? rgb[i] : (uint8_t)ULTRA_ROLE_BRIGHTNESS;
    }
    ultra_slot_light_def(s, l, seq_next(), roles, gamma_x1000, 0, mn, mx);
    host_send(s);
}

static void cmd_set(uint8_t l, uint16_t m, uint32_t dur, uint8_t flags)
{
    uint8_t s[32];
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    ultra_slot_set(s, l, seq_next(), flags, m, a, b, dur, 0);
    host_send(s);
}

static void cmd_effect(uint8_t l, uint8_t fx, const uint8_t *p, uint32_t n)
{
    uint8_t s[32];
    ultra_slot_effect(s, l, seq_next(), fx, 0, p, n);
    host_send(s);
}

static void cmd_scene(uint8_t l, uint16_t m)
{
    uint8_t s[32];
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    ultra_slot_scene(s, l, seq_next(), 1, 10, m, a, b);
    host_send(s);
}

static void cmd_config(uint8_t id, uint16_t v)
{
    uint8_t s[32];
    ultra_slot_config(s, seq_next(), &id, &v, 1u);
    host_send(s);
}

static void cmd_identify(uint8_t what, uint8_t arg)
{
    uint8_t s[32];
    if (what == ULTRA_IDENT_DARK) {
        ultra_slot_hold_dark(s, seq_next(), arg);
    } else if (what == ULTRA_IDENT_CAL) {
        ultra_slot_calibrate(s, seq_next(), arg);
    } else {
        ultra_slot_identify(s, seq_next(), what);
    }
    host_send(s);
}

static const uint8_t P_BREATHE[7] = { 0, 0x0F, 0xA0, 0x27, 0x10, 0xFF, 0xFF };   /* 4 s, 0.15 .. 1 */
static const uint8_t P_CANDLE[4] = { 0x9C, 0x40, 0, 100 };                     /* intensity 0.61, speed 100 % */

/* ================================================================================================ running it */
static uint32_t ms0_board;                               /* ms_now at boot */
typedef struct {
    uint32_t n, chg[5], maxstep[5], down[5], up[5];
    uint16_t prev[5], first[5], last[5];
    int64_t eh_min, eh_max;                              /* the engine's time per sample (us): ~1000 */
    uint32_t stalls;                                     /* flash stalls seen (a step over one is not counted) */
    uint64_t eh_prev, at_max[5];                         /* (sim ms of each channel's largest step) */
} win_t;
static win_t *win;                                       /* sampling every ms while set */
static uint32_t events_seen;                             /* the false-event watch below, accumulated */

static void win_sample(void)
{
    uint32_t i;
    if (!win) {
        return;
    }
    if (win->n != 0u && win->stalls != sim_stalls) {     /* a flash write stopped the CPU: its step is a jump */
        win->stalls = sim_stalls;
        win->eh_prev = ultra_host_us(&ul, tick_now);
        for (i = 0; i < 5u; i++) {
            win->prev[i] = win->last[i] = out_now[i];
        }
        return;
    }
    win->stalls = sim_stalls;
    {
        uint64_t h = ultra_host_us(&ul, tick_now);
        int64_t dh = (int64_t)(h - win->eh_prev);
        if (win->n == 0u) {
            win->eh_min = 0x7FFFFFFFFFFFLL;
            win->eh_max = -0x7FFFFFFFFFFFLL;
        } else {
            if (dh < win->eh_min) win->eh_min = dh;
            if (dh > win->eh_max) win->eh_max = dh;
        }
        win->eh_prev = h;
    }
    for (i = 0; i < 5u; i++) {
        uint16_t v = out_now[i];
        if (win->n == 0u) {
            win->first[i] = v;
        } else {
            uint32_t d = (uint32_t)abs((int)v - (int)win->prev[i]);
            win->chg[i] += d != 0u;
            if (d > win->maxstep[i]) {
                win->maxstep[i] = d;
                win->at_max[i] = sim_ns / 1000000u;
            }
            win->down[i] += v < win->prev[i];
            win->up[i] += v > win->prev[i];
        }
        win->prev[i] = win->last[i] = v;
    }
    win->n++;
}

/* the false-event watch, after every pass: no trip, limit event, D9, limiting (the load stays below every limit) */
static uint32_t expect_trips;
static void watch_events(void)
{
    events_seen |= (pw.limit_events != 0u) | (pw.ovl_trips != 0u) << 1 | ((uint32_t)pw.short_trips != expect_trips) << 2 |
                   (is.limiting != 0u) << 3 | (ocl.show != OCLED_OFF && expect_trips == 0u) << 4;
}

static void pass(void)
{
    SwitchToFiber(fib_fw);                               /* one main loop pass, up to its next watchdog kick */
    watch_events();
}

static void sim_tick(void)
{
    sim_ns += 50000u;
    sim_ticks++;
    wave_to(sim_ns);
    soak_tim4.CNT = (uint16_t)(edges_total - edges_rearm);
    soak_adc1.RDATAR = load_raw();
    poll_isr();
    if (host_on && sim_ticks >= host_next && !wf_on) {
        host_frame();
        host_next = sim_ticks + host_period;
    }
}

static void sim_stall(uint32_t us)
{
    sim_stalls++;
    sim_ns += (uint64_t)us * 1000u;
    sim_ticks += us / 50u;
    wave_to(sim_ns);
}

static void run_ticks(uint64_t n)
{
    while (n--) {
        sim_tick();
        pass();
        if (sim_ticks % 20u == 0u) {
            win_sample();
        }
    }
}

static void run_ms(uint64_t ms) { run_ticks(ms * 20u); }

/* sparse: ms (1..1000) at once, as a flash stall moves the time bases, then 20 real ticks; no frame in flight */
static void jump_ms(uint32_t ms)
{
    while (wf_on) {
        run_ticks(1);
    }
    if (ms > 1u) {
        sim_ns += (uint64_t)(ms - 1u) * 1000000u;
        sim_ticks += 20u * (uint64_t)(ms - 1u);
        tick_now += 20u * (ms - 1u);
        ms_now += ms - 1u;
        vs_pending += ms - 1u;
    }
    {
        int h = host_on, i;
        host_on = 0;
        for (i = 0; i < 19; i++) {                       /* the poll ISR alone (nothing for the main loop) */
            sim_tick();
        }
        host_on = h;
    }
    run_ticks(1);
}

/* sparse over a stretch, host bursts (15 frames) every `burst_s` seconds of real time if the host is there */
static uint64_t burst_next_ns;
static void run_sparse(uint64_t seconds, uint32_t burst_s)
{
    uint64_t s;
    for (s = 0; s < seconds; s++) {
        if (host_on && burst_s && sim_ns >= burst_next_ns) {
            burst_next_ns = sim_ns + (uint64_t)burst_s * 1000000000u;
            host_next = sim_ticks;
            run_ms(300);                                 /* 15 frames at 50 fps */
            jump_ms(700);
        } else {
            int h = host_on;
            host_on = 0;
            jump_ms(1000);
            host_on = h;
        }
    }
}

static uint64_t now_s(void) { return sim_ns / 1000000000u; }

static void console(const char *cmd)
{
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    while (*cmd && con_in_w + 2u < sizeof con_in) {
        con_in[con_in_w++] = *cmd++;
    }
    con_in[con_in_w++] = '\r';
    run_ms(50);
}

static void __stdcall fw_entry(void *p)
{
    (void)p;
    (void)fw_main();
}

/* power-up at these time bases (ms0, tick0) with the host's clock at host0 us (the statics main.c sets from ms_now at
 * boot on the chip, where ms_now is 0: set here from ms0, as a boot at that time base) */
static void boot(uint32_t ms0, uint32_t tick0, uint64_t host0)
{
    ms_now = ms0;
    tick_now = tick0;
    ms0_board = ms0;
    host_us0 = host0;
    last_data_ms = changed_ms = st_ms = scene_changed_ms = scene_check_ms = ultra_first_ms = temp_ms = lim_ms = ms0;
    fib_main = ConvertThreadToFiber(0);
    fib_fw = CreateFiber(1u << 20, fw_entry, 0);
    SwitchToFiber(fib_fw);                               /* the boot, up to the main loop's first kick */
}

static void flash_init(void)
{
    fl = (uint8_t *)VirtualAlloc((void *)(uintptr_t)FL_BASE, 0x10000u, 0x3000u, 0x04u);   /* RESERVE|COMMIT, RW */
    if (fl != (uint8_t *)(uintptr_t)FL_BASE) {
        printf("soak: cannot map the flash model at 0x%08X\n", FL_BASE);
        exit(2);
    }
    memset(fl, 0xFF, 0x10000u);
}

/* the console's `uptime=` in s (-1: none) */
static int64_t con_uptime(void)
{
    const char *p = strstr(con_out, "uptime=");
    long d = 0, h, m, s;
    if (!p) {
        return -1;
    }
    p += 7;
    if (strchr(p, 'd') && strchr(p, 'd') < strchr(p, '\r')) {
        d = strtol(p, (char **)&p, 10);
        p += 2;
    }
    h = strtol(p, (char **)&p, 10);
    m = strtol(p + 1, (char **)&p, 10);
    s = strtol(p + 1, (char **)&p, 10);
    return (int64_t)d * 86400 + h * 3600 + m * 60 + s;
}

#define SOAK_PROGRESS(...) do { printf("  [%7.3f d] ", (double)sim_ns / 864e11); printf(__VA_ARGS__); printf("\n"); \
                                 fflush(stdout); } while (0)

static void win_begin(win_t *w) { memset(w, 0, sizeof *w); win = w; }
static void win_end(void) { win = 0; }

/* ================================================================================================ scenarios */
/* wrap: the three wraps within the first 65 s */
static void sc_wrap(void)
{
    win_t w, r;
    uint32_t armed, abs_end, fired_abs = 0, fired_rel = 0, t, i, steady_ok = 1;
    uint16_t lo, hi;
    printf("soak wrap: boot 65 s before the ms wrap, 10 s before the tick's, 30 s before the host clock's\n");
    boot(0xFFFF0000u, 0xFFFFF000u - 200000u, 0xFFFFFFFFull - 30000000ull);
    host_on = 1;
    cmd_def(0, 0, 3, 2800);                              /* RGB on CH1-3 */
    cmd_def(1, 3, 1, 1000);                              /* CH4, gamma 1: linear */
    cmd_def(2, 4, 1, 1000);                              /* CH5 */
    cmd_set(0, 32768, 0, 0);
    cmd_set(1, 13107, 0, 0);                             /* 0.2 */
    cmd_set(2, 65535, 0, 0);
    cmd_effect(2, ULTRA_FX_BREATHE, P_BREATHE, 7);
    run_ms(5000);
    CHECK(ultra_on && ul.light[1].defined && ul.light[2].fx == ULTRA_FX_BREATHE, "wrap: Ultra locked, lights defined, "
          "breathe running (%u, %u)", ultra_on, ul.light[2].fx);
    lo = out_now[3];
    /* steady channels across the tick wrap (0.2 s in) and the host's (30 s in), breathe as the reference */
    win_begin(&r);
    run_ms(40000);
    win_end();
    for (i = 0; i < 3u; i++) {
        steady_ok &= r.chg[i] == 0u;
    }
    CHECK(steady_ok && r.chg[3] == 0u, "wrap: steady channels didn't move across the tick / host clock wraps");
    CHECK(r.maxstep[4] <= 100u && r.chg[4] > 30000u, "wrap: breathe continuous (max step %u at %u ms, %u changes, "
          "engine us per ms %d..%d)", r.maxstep[4], (uint32_t)r.at_max[4], r.chg[4], (int)r.eh_min, (int)r.eh_max);
    /* a 20 s transition 0.2 -> 0.9 on CH4 across the ms wrap (at 65.5 s); the absolute-deadline control */
    while (ms_now - ms0_board < 55000u) {
        run_ms(1);
    }
    cmd_set(1, 58982, 20000, 0);                         /* linear */
    armed = ms_now;
    abs_end = ms_now + 30000u;                           /* wraps: 30 s from 55 s is past 65.5 s */
    win_begin(&w);
    for (t = 0; t < 26000u; t++) {
        run_ms(1);
        if (!fired_abs && ms_now >= abs_end) fired_abs = ms_now - armed;    /* (the negative control) */
        if (!fired_rel && ms_now - armed >= 30000u) fired_rel = ms_now - armed;
    }
    win_end();
    while (!fired_rel) {
        run_ms(1);
        if (ms_now - armed >= 30000u) fired_rel = ms_now - armed;
    }
    hi = out_now[3];
    printf("  transition 0.2 -> 0.9 across the ms wrap: %u -> %u, %u ms up, %u down, max step %u; breathe max step %u\n",
           lo, hi, w.up[3], w.down[3], w.maxstep[3], w.maxstep[4]);
    CHECK(w.down[3] == 0u && w.up[3] > 15000u && w.maxstep[3] <= 6u && abs((int)hi - 58982) <= 2,
          "wrap: the transition ran smoothly to its end across the ms wrap");
    CHECK(w.maxstep[4] <= r.maxstep[4] + 2u && w.chg[4] > 20000u, "wrap: breathe continuous across the ms wrap");
    printf("  negative control: `now >= t0 + 30 s` armed 10.5 s before the wrap fired after %u ms; `now - t0 >= 30 s`"
           " after %u ms\n", fired_abs, fired_rel);
    CHECK(fired_abs < 1000u && fired_rel == 30000u, "wrap: the absolute deadline misfires, the firmware's form holds");
    CHECK(events_seen == 0u && sim_led_on == 0u && vdip.count == 0u && !signal_lost && is.trips == 0u,
          "wrap: no false events (%X, D9 %u, dips %u, loss %u, trips %u)", events_seen, sim_led_on, vdip.count,
          signal_lost, is.trips);
    CHECK(W_SETTINGS() == 1u && W_SCENE() == 0u && W_FX() == 0u && W_CAL() == 0u, "wrap: flash writes: settings %u "
          "(the sticky protocol: 1), scene %u, effects %u, calibration %u", W_SETTINGS(), W_SCENE(), W_FX(), W_CAL());
    console("status");
    CHECK(con_uptime() >= 80 && con_uptime() <= (int64_t)now_s() + 1, "wrap: uptime %d s (sim %u s)",
          (int)con_uptime(), (uint32_t)now_s());
}

/* long: 52 days */
static void sc_long(void)
{
    win_t w, ref, d1;
    uint32_t e0, e_dark, dips = 0, base_writes, d, steady = 1;
    uint64_t t_rel;
    printf("soak long: 52 days, bursts of frames every 20 min, sampled every second, windows every ms\n");
    boot(0xFFFF0000u, 0xFFFFF000u, 0xFFFFFFFFull - 30000000ull);
    host_on = 1;
    cmd_def(0, 0, 3, 2800);
    cmd_def(1, 3, 1, 1000);
    cmd_def(2, 4, 1, 1000);
    cmd_set(0, 32768, 0, 0);
    cmd_set(1, 32768, 0, 0);
    cmd_set(2, 65535, 0, 0);
    cmd_effect(2, ULTRA_FX_BREATHE, P_BREATHE, 7);
    cmd_scene(0, 32768);
    run_ms(10000);
    win_begin(&ref);
    run_ms(10000);
    win_end();
    CHECK(ref.chg[4] > 5000u && ref.chg[0] == 0u, "long: breathe running, RGB steady");
    run_sparse(43200u - 20u, 1200u);                     /* to 0.5 days */
    /* the dark hold (the hub's pin calibration): 10 s, released by itself */
    cmd_identify(ULTRA_IDENT_DARK, 10);
    host_next = sim_ticks;
    run_ms(3000);
    CHECK(dk.active && out_now[0] == 0u && out_now[4] == 0u, "long: dark hold");
    run_ms(12000);
    CHECK(!dk.active && out_now[0] != 0u, "long: the hold ran out, back on");
    t_rel = sim_ns;                                      /* the release (+- a few ms) */
    run_sparse(86400u * 3u / 2u, 1200u);                 /* to day 2 */
    /* dark from day 2 to day 30: energy written once (30 s dark), then nothing; then lit again */
    run_ms(180000);                                      /* 3 min every ms: >= 1 Wh not stored yet */
    e0 = W_ENERGY();
    cmd_set(0, 0, 0, 0);
    cmd_set(1, 0, 0, 0);
    cmd_set(2, 0, 0, 0);
    cmd_effect(2, ULTRA_FX_NONE, P_BREATHE, 0);
    host_next = sim_ticks;
    run_ms(40000);                                       /* the write 30 s after dark */
    CHECK(out_now[0] == 0u && out_now[3] == 0u && out_now[4] == 0u && W_ENERGY() - e0 == 1u, "long: all dark, "
          "the energy written 30 s later (%u)", W_ENERGY() - e0);
    run_sparse(28u * 86400u, 1200u);                     /* to day 30 */
    e_dark = W_ENERGY() - e0;
    cmd_set(0, 32768, 0, 0);
    cmd_set(1, 32768, 0, 0);
    cmd_set(2, 65535, 0, 0);
    cmd_effect(2, ULTRA_FX_BREATHE, P_BREATHE, 7);
    host_next = sim_ticks;
    run_ms(2000);
    base_writes = W_ENERGY();
    run_ms(180000);                                      /* 3 min lit, every ms (the energy integrates per ms): 2.7 Wh */
    run_sparse(3600u - 180u, 1200u);
    printf("  energy writes: %u in the 28 dark days (the write 30 s after dark, if 1 Wh was new), %u in the first lit "
           "hour after\n", e_dark, W_ENERGY() - base_writes);
    CHECK(e_dark == 1u, "long: dark for 28 days: %u energy writes (the one 30 s after dark)", e_dark);
    CHECK(W_ENERGY() - base_writes >= 1u, "long: lit again after 28 days dark: written within the hour (0.6.1: "
          "not for up to 24.8 days)");
    supply_mv = 19000u;                                  /* a 20 ms dip (a relay) */
    run_ms(20);
    supply_mv = 24000u;
    dips++;
    run_ms(2000);
    CHECK(vdip.count == dips, "long: the dip at day 30 (%u)", vdip.count);
    run_sparse(10u * 86400u - 3600u - 2u, 1200u);        /* to day 40 */
    SOAK_PROGRESS("day 40: the host goes");
    /* the host goes away for 3.5 days (signal loss: hold, the default): candle (CH4) and breathe (CH5) run on, on the
     * board's own time: windows every 12 h, at 2^31 ticks after the last header (0.6.1: the engine's time jumped)
     * and at the tick's wrap */
    cmd_effect(1, ULTRA_FX_CANDLE, P_CANDLE, 4);
    host_next = sim_ticks;
    run_ms(3000);
    host_on = 0;
    run_ms(100);
    {
        uint32_t ref_tick = ul.clk.ref_tick, wraps = 0, at31 = 0;
        win_begin(&d1);
        run_ms(10000);
        win_end();
        for (d = 0; d < 7u; d++) {
            uint32_t to31 = (uint32_t)(ref_tick + 0x80000000u - tick_now) / 20000u;   /* s to those points */
            uint32_t towrap = (uint32_t)(0u - tick_now) / 20000u, step = 43200u - 10u;
            if (!at31 && to31 < step + 10u && to31 > 5u) {
                step = to31 - 5u;
                at31 = 1;
            } else if (towrap < step + 10u && towrap > 5u) {
                step = towrap - 5u;
                wraps++;
            }
            run_sparse(step, 0);
            win_begin(&w);
            run_ms(10000);
            win_end();
            steady &= w.chg[3] * 2u >= d1.chg[3] && w.maxstep[4] <= ref.maxstep[4] + 2u && w.chg[4] > 2000u &&
                      w.chg[0] == 0u;
        }
        CHECK(steady && at31 && wraps && d1.chg[3] > 5000u && !signal_lost, "long: 3.5 days without a host: candle and "
              "breathe ran on, continuous, also at 2^31 ticks without a header (%u) and the tick's wrap (%u)", at31,
              wraps);
    }
    host_on = 1;
    burst_next_ns = sim_ns;
    cmd_effect(1, ULTRA_FX_NONE, P_CANDLE, 0);
    host_next = sim_ticks;
    run_ms(3000);
    /* the three points left, in time order: the engine's ms wrap (the host's time in ms, 32 bits: 49.4 days, its
     * clock 0.6 % fast), the board's ms wrap (with a dip 100 ms after it), 2^32 ms after the dark hold's release (its
     * fade must not replay) */
    {
        uint64_t when[3], hu = ultra_host_us(&ul, tick_now), wrap_us = ((hu / 1000u >> 32) + 1u) * 4294967296000ull;
        uint32_t done = 0, k, i, em0;
        when[0] = sim_ns + (wrap_us - hu) * 1000000u / 1006u;
        when[1] = sim_ns + (uint64_t)(uint32_t)(0u - ms_now) * 1000000u;
        when[2] = t_rel + 4294967296000000ull;
        SOAK_PROGRESS("host back: the engine's ms wrap at %.3f d, the board's at %.3f d, the hold + 2^32 ms at %.3f d",
                      when[0] / 864e11, when[1] / 864e11, when[2] / 864e11);
        for (k = 0; k < 3u; k++) {
            uint32_t e = 3;
            for (i = 0; i < 3u; i++) {
                if (!(done & (1u << i)) && (e == 3u || when[i] < when[e])) e = i;
            }
            done |= 1u << e;
            if (when[e] > sim_ns + 30000000000ull) {
                run_sparse((when[e] - sim_ns) / 1000000000u - 30u, 1200u);
            }
            SOAK_PROGRESS("point %u", e);
            host_next = sim_ticks;
            win_begin(&w);
            if (e == 0u) {                               /* the engine's ms wrap */
                em0 = (uint32_t)(ultra_host_us(&ul, tick_now) / 1000u);
                run_ms(60000);
                win_end();
                CHECK(em0 > 0xFFFF0000u && (uint32_t)(ultra_host_us(&ul, tick_now) / 1000u) < 0x10000u &&
                      w.chg[0] == 0u && w.chg[3] == 0u && w.maxstep[4] <= ref.maxstep[4] + 2u && w.chg[4] > 20000u,
                      "long: across the engine's ms wrap: continuous (engine ms %u -> %u, RGB %u changes, breathe %u)",
                      em0, (uint32_t)(ultra_host_us(&ul, tick_now) / 1000u), w.chg[0], w.maxstep[4]);
            } else if (e == 1u) {                        /* the board's ms wrap, a dip 100 ms after it */
                while ((uint32_t)(0u - ms_now) > 1000u && ms_now > 0x80000000u) {
                    run_ms(1);
                }
                while (ms_now > 0x80000000u) {
                    run_ms(1);
                }
                run_ms(100);
                supply_mv = 19000u;
                run_ms(15);
                supply_mv = 24000u;
                dips++;
                run_ms(15000);
                win_end();
                CHECK(vdip.count == dips, "long: a dip 100 ms after the ms wrap is seen (%u of %u; 0.6.1 was blind "
                      "there)", vdip.count, dips);
                CHECK(w.chg[0] == 0u && w.chg[3] == 0u && w.maxstep[4] <= ref.maxstep[4] + 2u && w.chg[4] > 5000u,
                      "long: outputs continuous across the ms wrap (RGB %u changes, breathe max step %u)", w.chg[0],
                      w.maxstep[4]);
            } else {                                     /* 2^32 ms after the dark hold */
                run_ms(60000);
                win_end();
                CHECK(w.chg[0] == 0u && w.chg[3] == 0u && w.first[0] == w.last[0], "long: 2^32 ms after the dark "
                      "hold: no fade replayed (RGB %u changes, CH4 %u; 0.6.1 faded up from dark again)", w.chg[0],
                      w.chg[3]);
            }
        }
    }
    if (now_s() < 52u * 86400u) {
        run_sparse(52u * 86400u - now_s(), 1200u);
    }
    console("status");
    printf("  after %u s: uptime %d s, %u frames, flash writes: settings %u, scene %u, effects %u, energy %u, "
           "calibration %u\n", (uint32_t)now_s(), (int)con_uptime(), host_frames, W_SETTINGS(), W_SCENE(), W_FX(),
           W_ENERGY(), W_CAL());
    CHECK(con_uptime() + 300 >= (int64_t)now_s() && con_uptime() <= (int64_t)now_s(), "long: uptime after 52 days "
          "(%d s, sim %u s)", (int)con_uptime(), (uint32_t)now_s());
    CHECK(strstr(con_out, "peak ") && strstr(con_out, "51d ") != NULL, "long: the MCU peak's age in days (51d+)");
    CHECK(strstr(con_out, "supply_dip 1=2d ") != NULL, "long: the newest dip's age (2 days)");
    {
        const char *d2 = strstr(con_out, "supply_dip 2=");
        char line[96];
        uint32_t n = 0;
        while (d2 && d2[n] && d2[n] != '\r' && n + 1u < sizeof line) {
            line[n] = d2[n];
            n++;
        }
        line[n] = 0;
        printf("  console: %s\n", d2 ? line : "(no supply_dip 2)");
        CHECK(d2 && (strncmp(d2 + 13, "21d 22:", 7) == 0 || strncmp(d2 + 13, "21d 23:", 7) == 0),
              "long: the day-30 dip's age (22 days)");
    }
    CHECK(events_seen == 0u && sim_led_on == 0u && is.trips == 0u, "long: no false events (%X, D9 %u, trips %u)",
          events_seen, sim_led_on, is.trips);
    CHECK(W_SETTINGS() == 1u && W_SCENE() == 1u && W_FX() == 0u && W_CAL() == 0u, "long: flash writes settings %u "
          "(1), scene %u (1), effects %u, calibration %u", W_SETTINGS(), W_SCENE(), W_FX(), W_CAL());
    CHECK(W_ENERGY() <= 24u * 52u + ENERGY_BUDGET_CAP, "long: energy writes within the budget (%u)", W_ENERGY());
}

/* fx: the power-on effects without any host, 3 days */
static void sc_fx(void)
{
    ultra_t u;
    uint8_t s[32], crc[ULTRA_LIGHTS], *img;
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, mn[5] = { 0, 0, 0, 0, 0 };
    uint8_t roles[5] = { 0, 0, 0, ULTRA_ROLE_BRIGHTNESS, 0 }, roles5[5] = { 0, 0, 0, 0, ULTRA_ROLE_BRIGHTNESS };
    win_t ref, w;
    uint32_t i;
    printf("soak fx: no host, the stored power-on effects (breathe CH4, candle CH5) for 3 days\n");
    /* the effect store as a host's SCENE_EFFECT left it (hal/hw.c's layout, core/ultra.h ULTRA_FX_*) */
    ultra_init(&u, 1);
    ultra_slot_light_def(s, 0, 1, roles, 1000, 0, mn, a);
    (void)ultra_on_slot(&u, s, 0);
    ultra_slot_light_def(s, 1, 2, roles5, 1000, 0, mn, a);
    (void)ultra_on_slot(&u, s, 0);
    ultra_slot_scene(s, 0, 3, 1, 10, 65535, a, a);
    (void)ultra_on_slot(&u, s, 0);
    ultra_slot_scene(s, 1, 4, 1, 10, 52428, a, a);
    (void)ultra_on_slot(&u, s, 0);
    ultra_slot_scene_effect(s, 0, 5, ULTRA_FX_BREATHE, P_BREATHE, 7);
    (void)ultra_on_slot(&u, s, 0);
    ultra_slot_scene_effect(s, 1, 6, ULTRA_FX_CANDLE, P_CANDLE, 4);
    (void)ultra_on_slot(&u, s, 0);
    img = fl + (SCENE_FX_ADDR - FL_BASE);
    for (i = 0; i < 2u; i++) {
        crc[i] = crc8_smbus((const uint8_t *)&u.light[i], ULTRA_HEAD_BYTES);
        memcpy(img + ULTRA_FX_HDR + i * ULTRA_HEAD_BYTES, &u.light[i], ULTRA_HEAD_BYTES);
        img[5u + i] = crc[i];
    }
    img[0] = 0x53; img[1] = 0x46; img[2] = 0x58; img[3] = 0x31;   /* ULTRA_FX_MAGIC, little endian */
    img[4] = 0x03u;
    img[10] = img[11] = 0;
    boot(0, 0, 0);                                       /* a real boot: tick 0, the wrap 2.49 days in */
    CHECK(fx_mask == 0x03u && fx_ch == 0x18u, "fx: both effects loaded (mask %X, channels %X)", fx_mask, fx_ch);
    run_ms(5000);
    win_begin(&ref);
    run_ms(20000);
    win_end();
    run_sparse(0x100000000ull / 20000u - 30u, 0);        /* to 5 s before the tick's wrap */
    win_begin(&w);
    run_ms(65000);
    win_end();
    printf("  20 s from boot: breathe %u changes (max step %u), candle %u; 65 s across the tick wrap: %u (%u), %u\n",
           ref.chg[3], ref.maxstep[3], ref.chg[4], w.chg[3], w.maxstep[3], w.chg[4]);
    CHECK(ref.chg[4] > 10000u && w.chg[4] * 20u >= ref.chg[4] * 65u / 2u, "fx: the candle keeps flickering across the "
          "tick's wrap (0.6.1: froze for 2.49 days)");
    CHECK(w.maxstep[3] <= ref.maxstep[3] + 2u && w.chg[3] * 20u >= ref.chg[3] * 65u / 2u, "fx: breathe continuous");
    run_sparse(86400u / 2u, 0);
    CHECK(events_seen == 0u && sim_led_on == 0u && W_SETTINGS() + W_SCENE() + W_FX() + W_CAL() == 0u &&
          W_ENERGY() <= 3u * 24u + ENERGY_BUDGET_CAP, "fx: no false events, no flash writes but the energy (%u, %u)",
          W_SETTINGS() + W_SCENE() + W_FX() + W_CAL(), W_ENERGY());
}

/* cfg: an automation flipping a stored setting; calibrations on a schedule */
static void sc_cfg(void)
{
    settings_t s2;
    store_ext_t e2;
    uint32_t k, w0, w1, last = 0, c0, lost;
    printf("soak cfg: CONFIG 23 every second for a day, then every minute for 3 days; calibrations every minute\n");
    boot(0xFFFF0000u, 0xFFFFF000u, 0xFFFFFFFFull - 30000000ull);
    host_on = 1;
    cmd_def(0, 0, 3, 2800);
    cmd_set(0, 0, 0, 0);                                 /* dark: no energy writes (their stall drops a frame) */
    run_ms(3000);
    w0 = W_SETTINGS();                                   /* (the sticky protocol at the lock) */
    host_on = 0;
    for (k = 0, lost = 0; k < 86400u; k++) {             /* every second: one frame with the command */
        last = k & 1u;
        cmd_config(ULTRA_CFG_SOFT_START, (uint16_t)last);
        hq_rep = 2;                                      /* (sent once) */
        host_frame();
        run_ms(2);
        lost += (uint32_t)(sext.opts & STORE_OPT_SOFT_START) != last;
        jump_ms(998);
    }
    w1 = W_SETTINGS() - w0;
    CHECK(w1 == 0u && lost == 0u, "cfg: flipped every second for a day: %u writes (the 2 s debounce never ran out), "
          "%u frames lost", w1, lost);
    run_sparse(10u, 0);
    CHECK(W_SETTINGS() - w0 == 1u, "cfg: ... the last value written 2 s after the end (%u)", W_SETTINGS() - w0);
    w0 = W_SETTINGS();
    for (k = 0; k < 3u * 1440u; k++) {                   /* every minute for 3 days */
        last = (k & 1u) ^ 1u;
        cmd_config(ULTRA_CFG_SOFT_START, (uint16_t)last);
        hq_rep = 2;
        host_frame();
        run_ms(2);
        jump_ms(998);
        run_sparse(59u, 0);
    }
    w1 = W_SETTINGS() - w0;
    printf("  every minute for 3 days: %u settings writes (budget %u + 8 a day), %u erases of the page\n", w1,
           STORE_BUDGET_CAP, fl_erases[page_of(SETTINGS_ADDR)]);
    CHECK(w1 >= STORE_BUDGET_CAP + 3u * 8u - 2u && w1 <= STORE_BUDGET_CAP + 3u * 8u + 1u, "cfg: within the budget (%u)",
          w1);
    run_sparse(3u * 3600u + 60u, 0);                     /* the next token at the latest */
    CHECK(settings_load(&s2, &e2) == 0 && (uint32_t)(e2.opts & STORE_OPT_SOFT_START) == last &&
          (sext.opts & STORE_OPT_SOFT_START) == last, "cfg: the last value is the one stored (%u)", last);
    CHECK(!settings_dirty, "cfg: nothing left pending");
    /* calibrations every minute for 2 hours: 4 stored at once, then one per 6 h; the newest is the one stored */
    host_on = 1;
    c0 = W_CAL();
    for (k = 0; k < 120u; k++) {
        cmd_identify(ULTRA_IDENT_CAL, (uint8_t)(k & 1u ? 50u : 0u));
        host_next = sim_ticks;
        run_ms(3000);                                    /* the test (~1 s) */
        run_sparse(57u, 0);
    }
    printf("  120 calibrations in 2 h: %u stored\n", W_CAL() - c0);
    CHECK(W_CAL() - c0 == CAL_BUDGET_CAP, "cfg: calibrations: %u stored (the budget's %u)", W_CAL() - c0,
          CAL_BUDGET_CAP);
    run_sparse(6u * 3600u, 1200u);
    {
        isense_cal_t c = { 0 };
        uint16_t n;
        CHECK(W_CAL() - c0 == CAL_BUDGET_CAP + 1u && cal_load(&c, &n) == 0 && c.levels == 1u && c.pct[0] == 50u,
              "cfg: the newest test stored when the token came (%u, levels %u)", W_CAL() - c0, c.levels);
    }
    CHECK((events_seen & ~0x18u) == 0u, "cfg: no false events (%X)", events_seen);
}

int main(int argc, char **argv)
{
    const char *sc = argc > 1 ? argv[1] : "wrap";
    flash_init();
    if (strcmp(sc, "wrap") == 0) {
        sc_wrap();
    } else if (strcmp(sc, "long") == 0) {
        sc_long();
    } else if (strcmp(sc, "fx") == 0) {
        sc_fx();
    } else if (strcmp(sc, "cfg") == 0) {
        sc_cfg();
    } else {
        printf("soak: unknown scenario %s\n", sc);
        return 2;
    }
    printf("soak %s: %s (%d failures), %u s simulated\n", sc, g_fail ? "FAILED" : "passed", g_fail, (uint32_t)now_s());
    return g_fail ? 1 : 0;
}
