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
 *   long   52 days: a 28-day stretch dark (the lifetime record: one write a day, no lit second), a dark hold
 *          (IDENTIFY 3) whose fade must not replay 2^32 ms later, supply dips (one 100 ms after the ms wrap), 3 days
 *          without a host (signal loss holds: the effect runs on), the tick's 20 wraps, the host's ms wrap; the
 *          console's uptime, ages and lifetime line at the end; every flash write
 *   fx     no host ever: the power-on effects (candle, breathe) from the stored effect, 3 days, past the tick's wrap
 *   cfg    an automation: Ultra CONFIG 23 (stored) flipped every second for a day, then every minute for 3 days;
 *          calibrations (IDENTIFY 2) every minute for 2 hours: the settings / calibration write budgets, and that the
 *          last value is the one stored; then a factory reset (console `defaults`): everything cleared but the
 *          lifetime record (0.6.3), whose pages it never touches
 *   spec   the scene's take-over deadline (15 s), CONFIG 22's persistence rules, IDENTIFY 3 with argument 0 (60 s), signal
 *          loss "off" and the host's refresh afterwards (review round 4, fable-spec)
 *   race   (r4, claude-bugs) a capture the main loop takes late, after the next frame changed its tags: a glitch, then
 *          a WS2915 header pause (0.6.2: the decoder asked for 65,505 bits, ~8 KB of RAM zeroed on the chip)
 *   tripreset (r4, claude-bugs) a factory reset (`defaults`) while a short holds the outputs off: they come back with
 *          the retry (0.6.2: the reset cleared the trip but not the forced-off outputs, dark until a power cycle)
 *   longws (r4, claude-bugs) WS2805 / WS2915 frames longer than the 16-bit edge counter counts (1,700 / 900 chips):
 *          auto mode locks and shows them (0.6.2: the count modulo 65,536 fit no protocol, never locked, dark)
 *   minmax (r4, claude-bugs) `status` typed key by key: its supply min / max cover the time since the last command
 *          (0.6.2: every key restarted them, a dip before the typing never showed)
 *   r4rx / r4reset / r4gate (r4, gpt-bugs) the ISR against owned mailboxes; defaults while tripped; an update's
 *          silent end restores the gate (test_r4_*.c); r4h7 / r4wraps / r4flash (test_r4.c); v* (gpt-verify)
 * Steps are bounded by 2 ms of an effect's slope: the host clock runs 0.6 % fast, so the engine's ms (integer)
 * advances by 2 in one board ms every ~170 ms.
 * Each checks: no stuck or false events (trips, limit events, D9, dips, signal loss), no lost transitions, no flash
 * writes but the expected ones, no half-word programmed that wasn't erased, outputs continuous across every wrap.
 * The negative control (wrap): an absolute
 * deadline `now >= t_end` armed before the ms wrap fires at once, where the firmware's `now - t0 >= d` waits. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/ws_decode.h"
static unsigned v_oversize;
static int v_decode(const uint16_t *buf, uint32_t nsamples, uint16_t nbits, const ws_timing_t *tm, uint8_t *out)
{
    if (nbits > 384u) { v_oversize++; return WS_ERR_TRUNCATED; }
    return ws_decode_frame(buf, nsamples, nbits, tm, out);
}
#define ws_decode_frame v_decode
#define main fw_main
#include "../src/app/main.c"
#undef main
#undef ws_decode_frame
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
soak_gpio_t soak_gpioa = { 0xFFFFFFFFu };                /* (0.6.7) TP6 (PA3) pulled up, the console's idle line */
const uint32_t soak_uid[3] = { 0x12345678u, 0x9ABCDEF0u, 0x0F1E2D3Cu };

uint16_t pwm_tab1[PATTERN_MAX_LEN * 4], pwm_tab2[PATTERN_MAX_LEN * 6], pwm_tab3[PATTERN_MAX_LEN * 6];
uint16_t din_ring[DIN_RING_WORDS], din_copy[DIN_COPY_WORDS];
static uint16_t sim_tab[5][256];

static void *fib_main, *fib_fw;
static uint64_t sim_ns, sim_ticks;                       /* real time, real ticks since the start */
static uint64_t ring_words;                              /* SPI words the DMA wrote */
static int ring_dirty;                                   /* din_ring holds a frame's samples somewhere */
static uint32_t edges_total, edges_rearm;                /* DIN falling edges; at the last gate re-arm */
static uint8_t wf_bits[16400];                           /* the frame on the wire (MSB first; r4: up to 131,200 bits) */
static uint32_t wf_nbits, wf_next_fall;
static uint64_t wf_t0;
static int wf_on;
static uint32_t vs_pending, supply_mv = 24000u;          /* injected group pairs since the last take; PA4's input */
static uint8_t sim_forced;                               /* pwm_force_off */
static uint32_t sim_led_on, sim_rearms;                  /* D9 switched on; gate re-arms */
static uint32_t short_ticks;                             /* ticks of a short still to come (the fast trip) */
static uint32_t sim_stalls;                              /* flash writes (the CPU stops) */
static uint64_t sim_stall_us;                            /* ... their time (r5cpu: busy, no WFI) */

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

/* the load: logic 45 mA + each output's duty x its strip (a short: 4095, the fast trip). (0.6.9) The Diff-Power
 * receiver measures its bus side, before its buck, and a port's PTCs allow 2.25 A: a receiver-sized load, ~100 W of
 * strips seen from a ~48 V bus, all five at 100 %: 2.1 A (under its rating) */
#if BOARD == BOARD_DP_RX
static uint32_t ld_ma[5] = { 450, 450, 450, 360, 360 };
#else
static uint32_t ld_ma[5] = { 1500, 1500, 1500, 1200, 1200 };   /* (r5jig: an open / overloaded channel) */
#endif
static uint32_t soak_leak_ma;                            /* (0.6.9) current with every output off (a FET stuck on) */
static uint32_t load_raw(void)
{
    uint32_t ma = 45u + soak_leak_ma, i, g = is.ma_per_lsb_q16 ? is.ma_per_lsb_q16 : 262410u, raw;
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
static uint32_t soak_reset_flags = 0x0C000000u;          /* power-on + pin (0.6.5, D1: a scenario sets others) */
uint32_t hw_reset_flags(void) { return soak_reset_flags; }
void pwm_start(uint32_t s, uint8_t l, uint8_t st) { (void)s; (void)l; (void)st; }
uint8_t pwm_right_aligned(uint8_t ch) { (void)ch; return 0; }
void pwm_set_phase(uint16_t o2, uint16_t o3) { (void)o2; (void)o3; }
static uint8_t sim_gate_forced;                          /* gate_force_closed since the last gate_rearm */
void gate_rearm(const gate_plan_t *g) { (void)g; edges_rearm = edges_total; sim_rearms++; sim_gate_forced = 0; }
void gate_force_closed(void) { sim_gate_forced = 1; }
uint32_t din_ring_pos(void) { return (uint32_t)(ring_words % DIN_RING_WORDS); }
void hw_adc_init(void) {}
/* the temperature's injected sequence (hw_temp_start, then the next ms): 84 us without a PA7 conversion, so the
 * next tick reads the same RDATAR (sim_tick keeps it); the voltages' sequence (28 us) ends before the next tick */
static uint8_t soak_temp_req, soak_adc_held;
uint32_t hw_adc_ms(void)
{
    if (soak_temp_req) {
        soak_temp_req = 0;
        soak_adc_held = 1;
        return 1;
    }
    vs_pending++;
    return 0;
}
void hw_vsense_take(uint32_t sum2[2], uint32_t *n)
{
    *n = vs_pending;
    sum2[0] = vs_pending * pair_of(supply_mv);
    sum2[1] = vs_pending * 2u * 2048u;
    vs_pending = 0;
}
void hw_temp_start(void) { soak_temp_req = 1; }
static uint32_t soak_ts = 1710u;                         /* the temperature sensor's count (1710: ~30 C; 0.6.9) */
uint8_t hw_temp_read(uint32_t *ts2, uint32_t *vref2) { *ts2 = 2u * soak_ts; *vref2 = 2u * 1489u; return 1; }
volatile uint16_t hw_diag[HW_DIAG_N];                    /* (0.6.5, D6: hal/hw.c's counters, none here) */
void hw_dma_check(void) {}
void pwm_force_off(uint8_t off) { sim_forced = off; }
void led_set(uint8_t on) { sim_led_on += on != 0u; }
void watchdog_kick(void) { SwitchToFiber(fib_main); }   /* every main loop pass: back to the scenario */

/* ---- console */
static char con_in[512], con_out[1u << 16];
static uint32_t con_in_r, con_in_w, con_out_n;
/* (0.6.6) con_rx_paced = 1: con_in's bytes arrive at 115200 baud (11.52 a ms, sim_tick) into the board's 64-byte RX
 * DMA ring (hal/hw.c uart_rx: 63 bytes unread at most); one arriving onto 63 unread overwrites one: rx_overruns */
static uint32_t con_rx_paced, con_in_due, rx_acc, rx_overruns;
int uart_getc(void)
{
    return con_in_r < (con_rx_paced ? con_in_due : con_in_w) ? (int)(unsigned char)con_in[con_in_r++] : -1;
}
/* (0.6.7) the jig's TP6: soak_tp6_low drives PA3 (main.c reads it through hal/hw.h HW_TP6_LOW); the UART's RX
 * position: the console's input (con_in) plus soak_rx_extra, traffic the console doesn't see */
static uint8_t soak_tp6_low;
static uint32_t soak_rx_extra;
uint32_t uart_rx_pos(void)
{
    return (con_rx_paced ? con_in_due : con_in_w) + soak_rx_extra;
}
/* (0.6.5) the UART's 2 x 512-byte TX buffers draining at 115200 baud (11.52 bytes a ms, sim_tick): a write beyond
 * them would make uart_write wait on the board (the main loop stalls): counted in tx_blocks. (0.6.7) The release
 * console's: 2 x 256 (hal/hw.c UART_TX_SIZE) */
#define SOAK_TX_CAP (CONSOLE_FULL ? 1024u : 512u)
/* (0.6.9, Quindor) the receiver's release console sends no full status (main.c STREAM_STATUS): the checks that read
 * the status (`uptime=`, `supply_dip`, `lifetime=`) check the same values in RAM there */
#define SOAK_NO_STATUS (!CONSOLE_FULL && BOARD == BOARD_DP_RX)
/* (0.6.10) the receiver's power-up: dark until a valid frame or GRACE_MS, then the power-on scene and effects faded in
 * (SCENE_IN_MS; main.c signal_loss_check): a scenario about the power-on scene / effects or their take-over waits that
 * out after its boot (the dig2analog-ultra shows them at once: nothing to wait) */
#if BOARD == BOARD_DP_RX
#define GRACE_WAIT() run_ms(GRACE_MS + SCENE_IN_MS + 10u)
#else
#define GRACE_WAIT() ((void)0)
#endif
static uint32_t tx_q, tx_acc, tx_blocks;
uint32_t uart_room(void) { return tx_q >= SOAK_TX_CAP ? 0u : SOAK_TX_CAP - tx_q; }
static FILE *con_dump;                                   /* SOAK_CON_DUMP=<file>: every byte the console wrote */
void uart_write(const char *s)
{
    static int con_dump_init;
    if (!con_dump_init) {
        const char *f = getenv("SOAK_CON_DUMP");
        con_dump_init = 1;
        con_dump = f ? fopen(f, "wb") : NULL;
    }
    if (con_dump) {
        fputs(s, con_dump);
        fflush(con_dump);
    }
    while (*s) {
        tx_blocks += tx_q >= SOAK_TX_CAP;
        tx_q++;
        if (con_out_n + 1u < sizeof con_out) {
            con_out[con_out_n++] = *s;
        }
        s++;
    }
    con_out[con_out_n] = 0;
}
void uart_kick(void) {}

/* ---- flash: the real addresses (VirtualAlloc), writes / erases per page. 0.6.4: the stores are core/flashrec.c's
 * themselves; this is hal/hw.c's flash_op: an erase or a program blocks the main loop for a test duration (erase
 * 16 ms, a half-word FL_HW_US) while the tick runs as the SRAM path does (flash_poll_isr: time bases, fast trip).
 * Counted: erases / programs while an output is at or above the flash's "low" level (15 %), and half-words programmed
 * that were not erased (the chip would refuse them). */
#define FL_BASE  0x08000000u
#define FL_SIZE  (RECORDS_END - FL_BASE)      /* (0.6.5) the 64 KB and the record sectors above */
#define FL_HW_US 100u                                    /* a test duration, not a measurement (BENCH) */
static uint8_t *fl;
static uint32_t fl_writes[FL_SIZE / 256u], fl_erases[FL_SIZE / 256u], fl_bad, fl_lit_erases, fl_bright_erases, fl_bright_progs;
static uint32_t fl_fail;                                 /* a test: the next operations on the settings pages fail */
static int32_t fl_cut = -1;                              /* (0.6.5) units (an erase, a half-word) before the power goes */
static uint8_t sim_vdd_low;                              /* (0.6.5) VDD below the PVD: flash_op starts nothing */
static uint32_t fl_outside;                              /* (0.6.5) operations outside the records / status records */
/* (review round 6) fl_ops: every flash_op call; fl_fault(addr, halfwords, erase_addr) != 0: that operation fails with
 * that code (FLASH_ERR_*), the flash unchanged; fl_fail_all: every operation fails (write protection on the records) */
static uint32_t fl_ops, fl_fail_all;
static uint32_t (*fl_fault)(uint32_t addr, uint32_t halfwords, uint32_t erase_addr);
static void sim_stall(uint32_t us);

static uint32_t fl_bright(void)
{
    uint32_t i, b = 0;
    for (i = 0; i < 5u; i++) {
        b |= out_now[i] >= FLASH_LOW_LEVEL;
    }
    return b;
}

#if BOARD_RETURN
static uint32_t rc_flash_in_window;                      /* flash operations while a response window ran (must be 0) */
#endif
uint32_t flash_op(uint32_t addr, const uint16_t *src, uint32_t halfwords, uint32_t erase_addr)
{
    uint32_t i, us = halfwords * FL_HW_US, a = erase_addr ? erase_addr : addr;
    fl_ops++;
    if (sim_vdd_low) {
        return FLASH_ERR_VDD;                            /* (hw.c: the PVD checked first) */
    }
    if (fl_fault) {
        uint32_t e = fl_fault(addr, halfwords, erase_addr);
        if (e != 0u) {
            sim_stall(erase_addr ? 16000u : FL_HW_US);   /* (the operation ran and failed) */
            return e;
        }
    }
    fl_outside += !((a >= RECORDS_BASE && a + 2u * halfwords <= RECORDS_END) ||
                    (a >= UPD_APP_DESC + UPD_DESC_BYTES && a + 2u * halfwords <= UPD_APP_DESC + UPD_PAGE_BYTES));
#if BOARD_RETURN
    rc_flash_in_window += rc_state != RC_IDLE;
#endif
    if (fl_fail_all) {
        sim_stall(FL_HW_US);                             /* (r6) refused at once (WRPRTERR), the clock switches' time */
        return FLASH_ERR_WRP;
    }
    if (fl_fail && (((addr | erase_addr) & ~0xFFu) == SETTINGS_ADDR || ((addr | erase_addr) & ~0xFFu) == SETTINGS_B_ADDR)) {
        fl_fail--;
        sim_stall(FL_HW_US);
        return FLASH_ERR_WRP;
    }
    if (erase_addr != 0u) {
        if (fl_cut == 0) {
            return FLASH_ERR_EOP;                        /* (0.6.5) the power is gone: nothing more changes */
        }
        fl_cut -= fl_cut > 0;
        memset(fl + (erase_addr - FL_BASE), 0xFF, 256u);
        fl_erases[(erase_addr - FL_BASE) / 256u]++;
        us += 16000u;
        fl_lit_erases += (out_now[0] | out_now[1] | out_now[2] | out_now[3] | out_now[4]) != 0u;
        fl_bright_erases += fl_bright();
    }
    for (i = 0; i < halfwords; i++) {
        if (fl_cut == 0) {
            return FLASH_ERR_EOP;
        }
        fl_cut -= fl_cut > 0;
        fl_bad += ((uint16_t *)(uintptr_t)addr)[i] != 0xFFFFu;   /* (the chip would refuse it) */
        ((uint16_t *)(uintptr_t)addr)[i] = src[i];
    }
    if (halfwords) {
        fl_writes[(addr - FL_BASE) / 256u]++;
        fl_bright_progs += fl_bright();
    }
    sim_stall(us);                                       /* the main loop waits, the tick runs on */
    return FLASH_OK;
}

/* (review round 6) soak_busy_model = 1: uart_busy() while TX bytes are queued (tx_q), as on the board; 0 (the
 * default, as before): never busy */
static uint8_t soak_busy_model;
uint32_t uart_busy(void) { return soak_busy_model && tx_q != 0u; }
uint32_t hw_vdd_low(void) { return sim_vdd_low; }         /* (r6) the PVD, as flash_op reads it */
void hw_flash_trip(void) { pwm_force_off(1); }
static uint32_t sim_window;                              /* (0.6.5) the boot window for the record reads: opened */
void hw_records_window(uint32_t open) { sim_window += open; }

static uint32_t page_of(uint32_t addr) { return (addr - FL_BASE) / 256u; }
static uint32_t writes_in(uint32_t addr, uint32_t pages)
{
    uint32_t n = 0, i;
    for (i = 0; i < pages; i++) {
        n += fl_writes[page_of(addr) + i];
    }
    return n;
}
#define W_SETTINGS() (writes_in(SETTINGS_ADDR, 1u) + writes_in(SETTINGS_B_ADDR, 1u))
#define W_SCENE()    (writes_in(SCENE_LOG_ADDR, 1u) + writes_in(SCENE_B_ADDR, 1u))
#define W_FX()       (writes_in(SCENE_FX_ADDR, 2u) + (SCENE_FX_B_ADDR != SCENE_FX_ADDR ? writes_in(SCENE_FX_B_ADDR, 2u) : 0u))
#define W_LIFE()     (writes_in(LIFE_PAGE0_ADDR, 1u) + writes_in(LIFE_PAGE1_ADDR, 1u))
#define W_CAL()      (writes_in(CAL_PAGE_ADDR, 1u) + writes_in(CAL_B_ADDR, 1u))
#if MON_EXTRA
#define W_HIST()     (writes_in(CALHIST_ADDR, 1u) + writes_in(CALHIST_B_ADDR, 1u))   /* (0.6.6, M2) */
#endif

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
    soak_gpioa.INDR = soak_tp6_low ? 0xFFFFFFF7u : 0xFFFFFFFFu;   /* (0.6.7) TP6 = PA3 */
    for (tx_acc += 576u; tx_acc >= 1000u; tx_acc -= 1000u) {   /* 11.52 bytes a ms */
        tx_q -= tx_q != 0u;
    }
    for (rx_acc += con_rx_paced ? 576u : 0u; rx_acc >= 1000u; rx_acc -= 1000u) {
        if (con_in_due < con_in_w) {
            rx_overruns += con_in_due - con_in_r >= 63u;
            con_in_due++;
        }
    }
    wave_to(sim_ns);
    soak_tim4.CNT = (uint16_t)(edges_total - edges_rearm);
    if (soak_adc_held) {
        soak_adc_held = 0;                               /* (hw_adc_ms: the temperature's sequence holds PA7) */
    } else {
        soak_adc1.RDATAR = load_raw();
    }
    poll_isr();
    if (host_on && sim_ticks >= host_next && !wf_on) {
        host_frame();
        host_next = sim_ticks + host_period;
    }
}

static void sim_stall(uint32_t us)
{
    sim_stalls++;
    sim_stall_us += us / 50u * 50u;
    for (; us >= 50u; us -= 50u) {                     /* (0.6.4: the tick runs from SRAM meanwhile) */
        sim_ns += 50000u;
        sim_ticks++;
        wave_to(sim_ns);
        if (soak_adc_held) {
            soak_adc_held = 0;
        } else {
            soak_adc1.RDATAR = load_raw();
        }
        flash_poll_isr();
    }
}

/* the main loop's WFI (soak/ch32v20x.h CPU_SLEEP): nothing (a pass ends, the next tick comes before the next one),
 * or with soak_wfi (0.6.5 D3, scenario r5cpu) the sleep itself: the next tick runs inside it, SysTick's count is
 * soak_busy at the WFI (the pass's work after its tick) and 120 after it (the tick's interrupt) */
static int soak_wfi;
static uint32_t soak_busy, soak_slept;
void soak_sleep(void)
{
    if (soak_wfi) {
        sim_tick();
        soak_systick.CNT = 120u;
        soak_slept = 1;                                  /* (run_ticks: this tick is done) */
    }
}

static void run_ticks(uint64_t n)
{
    while (n--) {
        if (soak_slept) {
            soak_slept = 0;                              /* (the last pass slept through it) */
        } else {
            sim_tick();
        }
        soak_systick.CNT = soak_busy;
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
    while (cli.st_part) {
        run_ms(10);                                      /* (0.6.5) a status in parts: to its end */
    }
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
    fl = (uint8_t *)VirtualAlloc((void *)(uintptr_t)FL_BASE, FL_SIZE, 0x3000u, 0x04u);   /* RESERVE|COMMIT, RW */
    if (fl != (uint8_t *)(uintptr_t)FL_BASE) {
        printf("soak: cannot map the flash model at 0x%08X\n", FL_BASE);
        exit(2);
    }
    memset(fl, 0xFF, FL_SIZE);
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

#include "test_r5_cpu_soak.c"                   /* (0.6.5 D3: r5cpu) */

/* long: 52 days */
static void sc_long(void)
{
    win_t w, ref, d1;
    uint32_t e0, e_dark, dips = 0, base_writes, d, steady = 1, lit0;
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
    /* dark from day 2 to day 30: the lifetime record written 30 s after dark, then once a day (the powered time alone);
     * no lit second counted (lit_ms passes 2^31 ms back on day 27); then lit again */
    run_ms(BOARD == BOARD_DP_RX ? 600000u : 180000u);    /* 3 min every ms: >= 1 Wh not stored yet (0.6.9: the
                                                            receiver's bus-side load, 10 min) */
    e0 = W_LIFE();
    cmd_set(0, 0, 0, 0);
    cmd_set(1, 0, 0, 0);
    cmd_set(2, 0, 0, 0);
    cmd_effect(2, ULTRA_FX_NONE, P_BREATHE, 0);
    host_next = sim_ticks;
    run_ms(40000);                                       /* the write 30 s after dark */
    CHECK(out_now[0] == 0u && out_now[3] == 0u && out_now[4] == 0u && W_LIFE() - e0 == 1u, "long: all dark, "
          "the lifetime record written 30 s later (%u)", W_LIFE() - e0);
    lit0 = en.lt.lit_s;
    run_sparse(28u * 86400u, 1200u);                     /* to day 30 */
    e_dark = W_LIFE() - e0;
    cmd_set(0, 32768, 0, 0);
    cmd_set(1, 32768, 0, 0);
    cmd_set(2, 65535, 0, 0);
    cmd_effect(2, ULTRA_FX_BREATHE, P_BREATHE, 7);
    host_next = sim_ticks;
    run_ms(2000);
    base_writes = W_LIFE();
    run_ms(180000);                                      /* 3 min lit, every ms (the energy integrates per ms): 2.7 Wh */
    run_sparse(3600u - 180u, 1200u);
    CHECK(e_dark >= 28u && e_dark <= 29u, "long: dark for 28 days: %u lifetime writes (30 s after dark, then daily)",
          e_dark);
    CHECK(en.lt.lit_s - lit0 >= 3600u - 30u && en.lt.lit_s - lit0 <= 3600u + 30u, "long: no lit second in the 28 dark "
          "days, the lit hour since (%u s)", en.lt.lit_s - lit0);
    supply_mv = 19000u;                                  /* a 20 ms dip (a relay) */
    run_ms(20);
    supply_mv = 24000u;
    dips++;
    run_ms(2000);
    CHECK(vdip.count == dips, "long: the dip at day 30 (%u)", vdip.count);
    printf("  lifetime writes: %u in the 28 dark days (30 s after dark, then the daily one), %u in the lit hour after "
           "and its dip\n", e_dark, W_LIFE() - base_writes);
    CHECK(W_LIFE() - base_writes >= 1u, "long: lit again after 28 days dark: the new Wh stored at the dip (0.6.1: the "
          "energy not for up to 24.8 days)");
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
#if CONSOLE_FULL
    console("status");
#else
    while (cli.st_part) {                                /* the release console takes no command (0.6.9: a status */
        run_ms(10);                                      /* the stream had begun before the capture lost its first */
    }                                                    /* part): the stream's next full status, whole */
    con_out_n = 0;
    con_out[0] = 0;
#if !SOAK_NO_STATUS
    while (!cli.st_part) {
        run_ms(10);
    }
    while (cli.st_part) {
        run_ms(10);
    }
#endif
#endif
#if SOAK_NO_STATUS
    printf("  after %u s: uptime %u s (RAM: no status on this console), %u frames, flash writes: settings %u, scene %u, "
           "effects %u, lifetime %u, calibration %u\n", (uint32_t)now_s(), up_s, host_frames, W_SETTINGS(), W_SCENE(),
           W_FX(), W_LIFE(), W_CAL());
    CHECK((uint64_t)up_s + 300u >= now_s() && up_s <= now_s(), "long: uptime after 52 days (%u s, sim %u s)", up_s,
          (uint32_t)now_s());
    CHECK(up_s - temp_peak_s >= 51u * 86400u && vdip_event(&vdip, 0) != 0 &&
          up_s - vdip_event(&vdip, 0)->t_s >= 2u * 86400u && up_s - vdip_event(&vdip, 0)->t_s < 3u * 86400u,
          "long: the MCU peak's age (51 days+) and the newest dip's (2 days), from RAM");
#else
    printf("  after %u s: uptime %d s, %u frames, flash writes: settings %u, scene %u, effects %u, lifetime %u, "
           "calibration %u\n", (uint32_t)now_s(), (int)con_uptime(), host_frames, W_SETTINGS(), W_SCENE(), W_FX(),
           W_LIFE(), W_CAL());
    CHECK(con_uptime() + 300 >= (int64_t)now_s() && con_uptime() <= (int64_t)now_s(), "long: uptime after 52 days "
          "(%d s, sim %u s)", (int)con_uptime(), (uint32_t)now_s());
    CHECK(strstr(con_out, "peak ") && (strstr(con_out, "51d ") != NULL || strstr(con_out, "52d ") != NULL),
          "long: the MCU peak's age in days (51d+; 52d: the release console's next status, up to a minute on)");
    CHECK(strstr(con_out, "supply_dip 1=2d ") != NULL, "long: the newest dip's age (2 days)");
#endif
#if !SOAK_NO_STATUS
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
#endif
    CHECK(events_seen == 0u && sim_led_on == 0u && is.trips == 0u, "long: no false events (%X, D9 %u, trips %u)",
          events_seen, sim_led_on, is.trips);
    CHECK(W_SETTINGS() == 1u && W_SCENE() == 1u && W_FX() == 0u && W_CAL() == 0u, "long: flash writes settings %u "
          "(1), scene %u (1), effects %u, calibration %u", W_SETTINGS(), W_SCENE(), W_FX(), W_CAL());
    CHECK(W_LIFE() <= 8u * 52u + LIFE_BUDGET_CAP, "long: lifetime writes within the budget (%u)", W_LIFE());
    printf("  lifetime erases: %u / %u, %u while lit; the 22 days lit to the end without a dark moment: written only "
           "at the dips\n", fl_erases[page_of(LIFE_PAGE0_ADDR)], fl_erases[page_of(LIFE_PAGE1_ADDR)], fl_lit_erases);
    {                                                    /* the lifetime record: RAM and flash agree with the run */
        life_rec_t r;
        life_pos_t p;
        const char *lt = strstr(con_out, "lifetime=");
        uint32_t found = life_find((const uint8_t *)LIFE_PAGE0_ADDR, (const uint8_t *)LIFE_PAGE1_ADDR, &r, &p);
        char line[200];
        uint32_t n = 0;
        while (lt && lt[n] && lt[n] != '\r' && n + 1u < sizeof line) {
            line[n] = lt[n];
            n++;
        }
        line[n] = 0;
        printf("  console: %s\n", lt ? line : "(no lifetime line)");
        CHECK(found && r.writes == en.lt.writes && r.ups == 1u && en.lt.on_s == up_s && r.on_s + 3u * 86400u >=
              up_s && r.dips == dips && en.lt.dips == dips, "long: the stored record, from the last dip (writes %u / "
              "%u, %u s of %u, dips %u)", r.writes, en.lt.writes, r.on_s, up_s, r.dips);
        CHECK(en.lt.lit_s > 23u * 86400u && en.lt.lit_s < 25u * 86400u, "long: lit about 24 of the 52 days (%u s)",
              en.lt.lit_s);
#if !SOAK_NO_STATUS
        CHECK(lt && strstr(lt, ", 1 power-ups, trips 0 / 0, dips 2, ") != NULL, "long: the lifetime line");
#endif
    }
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
    GRACE_WAIT();                                        /* (0.6.10: the receiver's grace window) */
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
          W_LIFE() <= 3u * 8u + LIFE_BUDGET_CAP, "fx: no false events, no flash writes but the lifetime record (%u, "
          "%u)", W_SETTINGS() + W_SCENE() + W_FX() + W_CAL(), W_LIFE());
}

#include "test_r5_ovr_soak.c"                   /* (0.6.5 D4: r5ovr) */
#include "test_r6_takeover.c"                    /* (review round 6: r6takeover*, r6rejoin5, r6fxtake) */

/* cfg: an automation flipping a stored setting; calibrations on a schedule */
static void sc_cfg(void)
{
    settings_t s2;
    store_ext_t e2;
    uint32_t k, w0, w1, last = 0, c0, lost, l0, s0;
    printf("soak cfg: CONFIG 23 every second for a day, then every minute for 3 days; calibrations every minute\n");
    boot(0xFFFF0000u, 0xFFFFF000u, 0xFFFFFFFFull - 30000000ull);
    host_on = 1;
    cmd_def(0, 0, 3, 2800);
    cmd_set(0, 0, 0, 0);                                 /* dark: no lifetime writes (their stall drops a frame) */
    run_ms(3000);
    w0 = W_SETTINGS();                                   /* (the sticky protocol at the lock) */
    l0 = W_LIFE();
    s0 = sim_stalls;
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
    CHECK(w1 <= lost && lost <= sim_stalls - s0 && lost <= 2u, "cfg: flipped every second for a day: %u writes (the "
          "2 s debounce never ran out but where a frame was lost), %u frames lost (only in the day's flash operations: "
          "%u, the lifetime record's daily write %u and an erase ahead in a dark moment)", w1, lost, sim_stalls - s0,
          W_LIFE() - l0);
    k = settings_dirty;                                  /* (0.6.10: a lost frame's flip may have left the last value
                                                            the stored one: then nothing is due) */
    run_sparse(10u, 0);
    CHECK(W_SETTINGS() - w0 == w1 + k && !settings_dirty && (uint32_t)(sext.opts & STORE_OPT_SOFT_START) == last,
          "cfg: ... the last value written 2 s after the end (%u, %u due)", W_SETTINGS() - w0, k);
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
#if MON_EXTRA
    {                                                    /* (0.6.6, M2) the calibration history: a record per stored test */
        calhist_t r;
        memset(&r, 0, sizeof r);
        printf("  calibration history: %u tests, %u entries, %u records\n", chist.tests, chist.n, W_HIST());
        CHECK(chist.tests == CAL_BUDGET_CAP + 1u && chist.n == CALHIST_N && W_HIST() == CAL_BUDGET_CAP + 1u &&
              calhist_load(&r) && r.seq == chist.seq && r.tests == chist.tests && !chist_pend,
              "cfg: the calibration history: %u tests, %u records, stored as in RAM", chist.tests, W_HIST());
    }
#endif
    CHECK((events_seen & ~0x18u) == 0u, "cfg: no false events (%X)", events_seen);
    /* a factory reset (console `defaults`, as a config frame or TP9's gesture): settings, scene, effects, calibration
     * cleared; the lifetime record kept, its pages untouched */
    {
        uint32_t e0p, e1p, wl = W_LIFE();
        uint32_t ups = en.lt.ups, on = en.lt.on_s, wh, wr, found;
        life_rec_t r;
        life_pos_t p;
        cmd_set(0, 65535, 0, 0);                         /* lit 10 min: something to store */
        host_next = sim_ticks;
        run_ms(600000);
        cmd_set(0, 0, 0, 0);
        host_next = sim_ticks;
        run_ms(40000);                                   /* written 30 s after dark */
        wh = energy_total_wh(&en);
        wr = en.lt.writes;
        CHECK(W_LIFE() >= wl + 1u && wh > 0u && cal_present, "cfg: lit, then dark: stored (%u Wh), a calibration "
              "stored", wh);
        wl = W_LIFE();
        e0p = fl_erases[page_of(LIFE_PAGE0_ADDR)];
        e1p = fl_erases[page_of(LIFE_PAGE1_ADDR)];
        console("defaults");
        /* Render the reset's acknowledgement at normal cadence: sparse 1 Hz sampling can
         * keep presenting bright segments and never give flash admission a continuous low window. */
        run_ms(30000);
        found = life_find((const uint8_t *)LIFE_PAGE0_ADDR, (const uint8_t *)LIFE_PAGE1_ADDR, &r, &p);
        printf("  factory reset: lifetime %u Wh, %u power-ups, %u writes, pages erased %u / %u times before, %u / %u "
               "after\n", energy_total_wh(&en), en.lt.ups, en.lt.writes, e0p, e1p, fl_erases[page_of(LIFE_PAGE0_ADDR)],
               fl_erases[page_of(LIFE_PAGE1_ADDR)]);
        CHECK(strstr(con_out, "ok") && !cal_present && fx_mask == 0u && sext.ilimit50 == 0xFFu && scene_want.mask == 0u,
              "cfg: factory reset: calibration, effects, limit, scene cleared");
        CHECK(fl_erases[page_of(LIFE_PAGE0_ADDR)] == e0p && fl_erases[page_of(LIFE_PAGE1_ADDR)] == e1p &&
              W_LIFE() == wl, "cfg: factory reset: the lifetime pages untouched");
#if MON_EXTRA
        {                                                /* (0.6.6, M2) the calibration history begins anew */
            calhist_t hr;
            CHECK(calhist_load(&hr) && hr.n == 0u && hr.tests == 0u && chist.n == 0u && !chist_pend,
                  "cfg: factory reset: the calibration history begins anew (%u entries stored)", hr.n);
        }
#endif
        CHECK(en.lt.ups == ups && en.lt.on_s >= on && energy_total_wh(&en) - wh <= 1u && en.lt.writes == wr && found &&
              r.writes == wr && wh - r.wh <= 1u && r.ups == ups, "cfg: factory reset: the lifetime record kept (RAM %u Wh, "
              "flash %u Wh, %u writes)", energy_total_wh(&en), r.wh, r.writes);
    }
}

#include "test_verify.c"
#include "test_r6_seqwrap.c"                     /* (review round 6: r6seqwrap) */
#include "test_r4.c"
#include "test_r4_rx.c"                         /* (gpt-bugs: its own regressions, besides gpt-verify's) */
#include "test_r4_reset.c"
#include "test_r4_gate.c"
#include "test_r5.c"                             /* (0.6.5: the upper flash, dip writes) */
#include "test_soak_rc.c"                        /* (BOARD=2: the return channel; soak_rc.exe) */
#include "test_r6_gamma0.c"                      /* (review round 6: r6gamma0) */

/* spec (review round 4): main.c behaviours of README / ULTRA_PROTOCOL.md that had no test. The power-on scene's
 * take-over SCENE_TAKEOVER_MS after the first slot without a light command (r6: 15 s; 0.6.6: 3 s after the header); CONFIG 22 written only when
 * it differs from the stored value, auto / Ultra applied at once, WS2805 stored only; IDENTIFY 3 with argument 0 =
 * 60 s; signal loss "off" (CONFIG 8 = 1), then the host back without a reboot repeating its state with the SEQ it
 * already sent: the light must come back (0.6.2: it stays off until the next change). */
static void sc_spec(void)
{
    scene_t sc;
    scene_rec_t rec;
    uint8_t s[32];
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint32_t w0;
    printf("soak spec: the scene's take-over deadline, CONFIG 22 persistence, IDENTIFY 3 = 60 s, signal loss off + "
           "refresh\n");
    scene_clear(&sc);
    sc.mask = 0x01u;                                     /* a stored scene: CH1 at 46 %, a 1 s cross-fade */
    sc.v[0] = 30000u;
    scene_log_pack(&sc, 1, &rec);
    memcpy(fl + (SCENE_LOG_ADDR - FL_BASE), &rec, sizeof rec);
    boot(1000u, 20000u, 5000000ull);
    GRACE_WAIT();                                        /* (0.6.10: the receiver's grace window) */
    run_ms(600);                                         /* (the soft start's 0.4 s) */
    CHECK(out_now[0] == 30000u && srun.hold == 0x01u, "spec: the stored scene shows from power-up (%u)", out_now[0]);
    host_on = 1;                                         /* an Ultra hub sending NOP frames: no light command */
    run_ms(SCENE_TAKEOVER_MS - 500u);
    CHECK(ultra_on && out_now[0] == 30000u && srun.hold == 0x01u, "spec: the scene holds %u s after the first slot "
          "(%u)", SCENE_TAKEOVER_MS / 1000u - 1u, out_now[0]);
    run_ms(1000);
    CHECK(srun.hold == 0u && out_now[0] < 30000u, "spec: the take-over %u s after the first slot (%u)",
          SCENE_TAKEOVER_MS / 1000u, out_now[0]);
    run_ms(1500);
    CHECK(out_now[0] == 0u && !srun.fading, "spec: faded to dark (no light defined) over the scene's 1 s (%u)",
          out_now[0]);
    /* CONFIG 22: persisted only when it differs; auto / Ultra at once, a WS protocol from the next power-up */
    w0 = W_SETTINGS();
    cmd_config(ULTRA_CFG_BOOT_PROTOCOL, PMODE_AUTO);
    run_ms(3000);
    CHECK(W_SETTINGS() == w0 && frx.ps.mode == PMODE_AUTO, "spec: CONFIG 22 = the stored value: no write (%u)",
          W_SETTINGS() - w0);
    cmd_config(ULTRA_CFG_BOOT_PROTOCOL, PMODE_ULTRA);
    run_ms(3000);
    CHECK(W_SETTINGS() == w0 + 1u && frx.ps.mode == PMODE_ULTRA && set.protocol == PMODE_ULTRA && ultra_on,
          "spec: CONFIG 22 = 3: forced Ultra at once, one write (%u)", W_SETTINGS() - w0);
    cmd_config(ULTRA_CFG_BOOT_PROTOCOL, PMODE_AUTO);
    run_ms(3000);
    CHECK(W_SETTINGS() == w0 + 2u && frx.ps.mode == PMODE_AUTO && proto_active(&frx.ps) == PROTO_ULTRA && ultra_on,
          "spec: CONFIG 22 = 0: auto at once, the lock kept, one write (%u)", W_SETTINGS() - w0);
    cmd_config(ULTRA_CFG_BOOT_PROTOCOL, PMODE_WS2805);
    run_ms(3000);
    CHECK(W_SETTINGS() == w0 + 3u && set.protocol == PMODE_WS2805 && frx.ps.mode == PMODE_AUTO && ultra_on,
          "spec: CONFIG 22 = 2: stored only, Ultra keeps running (%u)", W_SETTINGS() - w0);
    cmd_config(ULTRA_CFG_BOOT_PROTOCOL, PMODE_AUTO);
    run_ms(3000);
    CHECK(W_SETTINGS() == w0 + 4u && set.protocol == PMODE_AUTO, "spec: CONFIG 22 = 0 again: one write (%u)",
          W_SETTINGS() - w0);
    /* IDENTIFY 3 with argument 0: dark for 60 s, then back by itself */
    cmd_def(0, 0, 3, 1000);                              /* RGB on CH1-3, gamma 1: out = M */
    cmd_set(0, 65535, 0, 0);
    run_ms(2000);
    CHECK(out_now[0] == 65535u, "spec: the light on (%u)", out_now[0]);
    cmd_identify(ULTRA_IDENT_DARK, 0);
    run_ms(1000);
    CHECK(dk.active && out_now[0] == 0u, "spec: IDENTIFY 3, argument 0: dark (%u)", out_now[0]);
    run_ms(58000);
    CHECK(dk.active && out_now[0] == 0u, "spec: still dark 59 s in (%u)", out_now[0]);
    run_ms(2500);
    CHECK(!dk.active && out_now[0] == 65535u, "spec: back by itself after 60 s (%u)", out_now[0]);
    /* signal loss "off": the host away for > 3 s, then back without a reboot (its clock ran on), repeating the state */
    cmd_config(ULTRA_CFG_LOSS, 1);
    ultra_slot_set(s, 0, 200, 0, 32768, a, b, 0, 0);     /* the host's state, SEQ 200 */
    host_send(s);
    run_ms(2000);
    CHECK(out_now[0] == 32768u && !signal_lost, "spec: the host's state (%u)", out_now[0]);
    host_on = 0;
    run_ms(4000);
    CHECK(signal_lost && out_now[0] == 0u, "spec: signal loss, CONFIG 8 = 1: off (%u, lost %u)", out_now[0],
          signal_lost);
    host_on = 1;
    host_next = sim_ticks;
    host_send(s);                                        /* the refresh: the same command, the same SEQ */
    run_ms(2000);
    CHECK(!signal_lost && out_now[0] == 32768u, "spec: the host's refresh after a signal-loss off brings the light "
          "back (%u; 0.6.2: it stays off until the next change)", out_now[0]);
    CHECK(events_seen == 0u && is.trips == 0u, "spec: no false events (%X)", events_seen);
}

/* race (r4): a capture the main loop takes late, after the next frame's start changed the capture's tags. A WS2915
 * board behind another one (the header passes, then a pause while the boards before it block their bits) gets a
 * glitch (one edge: a capture of 1 bit), and the main loop is busy (the stagger search, ~0.5 ms) until the next frame
 * has passed its header pause: the poll ISR has set cap_skip = 32 for that frame while cap_bits is still the glitch's.
 * 0.6.2: want = 1 - 32 wraps, the decoder zeroes ~8 KB from dec.slot (the settings, the frame tracker, the
 * console's state ...). */
static const uint8_t WS15_HDR[4] = { 0xFF, 0xFF, 0xFF, 0x80 };    /* gains 31, check bits 0 */

/* the header, a pause (the boards before this one block their bits), then `data_bits` of the 80 data bits; with
 * isr_only the main loop does not run meanwhile */
static void ws2915_frame(const uint8_t data[10], uint32_t data_bits, int isr_only)
{
    uint64_t t0 = sim_ns + 7000u, t1 = t0 + 32u * 1250u + 200000u;   /* a 200 us pause after the header */
    wire_frame(WS15_HDR, 32u, t0);
    while (sim_ns < t1) {
        sim_tick();
        if (!isr_only) {
            pass();
        }
    }
    t1 = sim_ns + 7000u;                                 /* (the ring is sampled up to now: the data starts after) */
    wire_frame(data, data_bits, t1);
    while (sim_ns < t1 + (uint64_t)data_bits * 1250u + (isr_only ? 0u : 400000u)) {
        sim_tick();
        if (!isr_only) {
            pass();
        }
    }
}

static void sc_race(void)
{
    static const uint8_t data[10] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x24, 0x68 };
    static const uint8_t glitch[1] = { 0x80 };
    uint32_t i, k;
    uint8_t min_pulse;
    printf("soak race: a glitch, then a WS2915 frame's header pause, while the main loop is late\n");
    boot(0, 0, 0);
    console("set protocol 1");                           /* forced WS2915 (32-bit header) */
    for (k = 0; k < 4u; k++) {
        ws2915_frame(data, 80u, 0);
        run_ms(20);
    }
    CHECK(proto_active(&frx.ps) == PROTO_WS2915_32 && plan.hdr_bits == 32u && sm.to[0] == 0x1234u &&
          sm.to[4] == 0x2468u, "race: WS2915 frames with a header pause shown (%u, hdr %u, %04X %04X)",
          (uint32_t)proto_active(&frx.ps), (uint32_t)plan.hdr_bits, sm.to[0], sm.to[4]);
    min_pulse = set.min_pulse;
    for (k = 0; k < 20u; k++) {
        wire_frame(glitch, 1u, sim_ns + 7000u);          /* one edge: a frame of 1 bit, its capture 1 bit */
        for (i = 0; i < 10u; i++) {
            sim_tick();                                  /* the poll ISR alone: the main loop is busy */
        }
        ws2915_frame(data, 4u + k, 1);                   /* the next frame: past its header pause (cap_skip 32) */
        pass();                                          /* the main loop takes the glitch's capture now */
        ws2915_frame(data, 80u, 0);
        run_ms(20);
    }
    printf("  20 late captures: %u decoder calls past the buffer\n", v_oversize);
    CHECK(v_oversize == 0u, "race: no decode past dec.slot (%u calls; 0.6.2: ~8 KB of RAM zeroed on the chip)",
          v_oversize);
    CHECK(sm.to[0] == 0x1234u && set.min_pulse == min_pulse && stat.version != 0, "race: still showing the frames, "
          "settings and console state intact");
}

/* d1 (0.6.5): the reset breadcrumbs (main.c bcrumb). The block as the run before left it (a watchdog reset after a
 * load access fault, 1d 1:02:03 up, the faulting PC), the reset flags a watchdog's: `status` shows that run's line,
 * and this run's breadcrumbs start again (the magic, the uptime every second, the tick's PC every ms, no fault). */
static void sc_d1(void)
{
    const char *p;
    printf("soak d1: the reset breadcrumbs after a watchdog reset\n");
    bcrumb.magic = BC_MAGIC;
    bcrumb.up_s = 90123u;
    bcrumb.pc = 0x00004A2Eu;
    bcrumb.cause = 5u;
    soak_reset_flags = 0x20000000u;                      /* IWDGRSTF */
    boot(0, 0, 0);
    CHECK(bcrumb.magic == BC_MAGIC && bcrumb.up_s == 0u && bcrumb.cause == 0u, "d1: this run's block starts (magic "
          "%08X, up %u, cause %u)", bcrumb.magic, bcrumb.up_s, bcrumb.cause);
    run_ms(3500);
    printf("  after 3.5 s: up %u s (uptime %u), pc %08X, cause %u\n", bcrumb.up_s, up_s, bcrumb.pc, bcrumb.cause);
    CHECK(bcrumb.up_s == up_s && up_s == 3u && bcrumb.pc == 0x00003A1Cu && bcrumb.cause == 0u, "d1: the uptime every "
          "second, the tick's PC (%u %u %08X)", bcrumb.up_s, up_s, bcrumb.pc);
    console("status");
    p = strstr(con_out, "before_reset=");
    printf("  status: %.47s", p ? p : "(no before_reset line)\n");
    CHECK(p && strncmp(p, "before_reset=1d 1:02:03 up, pc 18990, fault 5\r\n", 47) == 0, "d1: the last run's line");
    CHECK(strstr(con_out, "reset=watchdog") != NULL, "d1: the reset line says watchdog");
}
#if BOARD_TESTJIG                                /* (the receiver has no test pad: no jig there) */
#include "test_r5_jig_soak.c"                    /* (0.6.5 D10: r5jig) */
#endif

/* tripreset (r4): a factory reset while a trip holds the outputs off (a short; the retry 1 s later). The reset sets the
 * current sense up again (isense_setup): 0.6.2 cleared the trip there, but the outputs stayed forced off (only the
 * retry releases them, and there was no trip left to retry): the board stayed dark, `status` said ok, until a power
 * cycle. */
static void sc_tripreset(void)
{
    uint32_t k;
    printf("soak tripreset: `defaults` while a short's trip holds the outputs off\n");
    boot(0, 0, 0);
    run_ms(500);
    for (k = 0; k < 2u; k++) {
        short_ticks = 3;                                 /* a short: the fast trip forces the outputs off */
        expect_trips = k + 1u;
        run_ms(k == 0u ? 300u : 20u);
        CHECK(sim_forced && is.tripped != ISENSE_TRIP_NONE, "tripreset: tripped, outputs forced off (%u, %u)",
              sim_forced, (uint32_t)is.tripped);
        if (k == 0u) {
            console("defaults");                         /* the factory reset, during the retry wait */
            run_ms(3000);                                /* the retry (1 s), the soft start */
            printf("  after the reset and 3 s: outputs forced %u, tripped %u, status says %s\n", sim_forced,
                   (uint32_t)is.tripped, is.tripped ? "tripped" : "ok");
            CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE, "tripreset: the outputs come back after the reset "
                  "(forced %u, tripped %u)", sim_forced, (uint32_t)is.tripped);
        }
    }
    run_ms(3000);
    CHECK(!sim_forced && is.tripped == ISENSE_TRIP_NONE, "tripreset: a later trip retries as before (forced %u)",
          sim_forced);
}

/* d2 (0.6.5): the event log (main.c ev_add, core/cli.h evlog_t), fed by what counts the events already: a short (the
 * fast trip: pwrev's short trip with its current; its retry comes back with the limiter's feedback factor still low
 * from the short's sample: 0.6.9 counted a limit event for it too, 0.6.10 not, part of the trip), a supply dip
 * (24 V -> 19 V for 40 ms:
 * vdip's count, with the board current then), a calibration (`set ical 50`: its end). `status` ends with them, newest
 * first, with their ages: one line per counted event. */
static const char *d2_line(const char *s, uint32_t n)    /* the n-th line (1..) of s */
{
    while (s && --n) {
        s = strstr(s, "\r\n");
        s = s ? s + 2 : 0;
    }
    return s;
}

static void sc_d2(void)
{
    char want[80];
    uint32_t t_short, t_dip, t_cal, now, cal0;
    const char *l, *ev;
    printf("soak d2: the event log from a short, a supply dip and a calibration\n");
    boot(0, 0, 0);
    run_ms(2000);
    console("status");
    CHECK(strstr(con_out, "event ") == NULL, "d2: nothing logged yet");
    short_ticks = 3;                                     /* a short: the fast trip, the retry 1 s later */
    expect_trips = 1;
    t_short = up_s;
    run_ms(4000);
    supply_mv = 19000u;
    t_dip = up_s;
    run_ms(40);
    supply_mv = 24000u;
    run_ms(2000);
    cal0 = cal_cnt;
    console("set ical 50");
    run_ms(isense_cal_ms((uint16_t)(50u * 65535u / 100u)) + 200u);
    t_cal = up_s;
    jump_ms(1000);
    run_ms(1500);
    CHECK(is.cal_step == 0u && cal_cnt == cal0 + 1u, "d2: the calibration ran and was stored (%u -> %u)", cal0,
          cal_cnt);
    console("status");
    now = up_s;
    ev = strstr(con_out, "event 1=");
    printf("%s", ev ? ev : "(no event lines)\n");
    CHECK(pw.short_trips == 1u && vdip.count == 1u && pw.limit_events == 0u, "d2: pwrev / vdip counted one trip, one "
          "dip, no limit event (0.6.10: the retry's is the trip's) (%u %u %u)", (uint32_t)pw.short_trips,
          (uint32_t)vdip.count, (uint32_t)pw.limit_events);
    snprintf(want, sizeof want, "event 1=0:00:%02u ago, calibration, %u.%02u A\r\n", now - t_cal,
             (clim.connected_ma + 5u) / 1000u, (clim.connected_ma + 5u) / 10u % 100u);
    CHECK(ev && strncmp(d2_line(ev, 1), want, strlen(want)) == 0, "d2: 1 the calibration, the connected load (%s)",
          want);
    snprintf(want, sizeof want, "event 2=0:00:%02u ago, supply dip, ", now - t_dip);
    CHECK((l = d2_line(ev, 2)) && strncmp(l, want, strlen(want)) == 0, "d2: 2 the dip, with the current (%s)",
          want);
    snprintf(want, sizeof want, "event 3=0:00:%02u ago, short, %u.%02u A\r\n", now - t_short,
             ((uint32_t)pw.trip_ma + 5u) / 1000u, ((uint32_t)pw.trip_ma + 5u) / 10u % 100u);   /* (as cf rounds) */
    CHECK((l = d2_line(ev, 3)) && strncmp(l, want, strlen(want)) == 0, "d2: 3 the short with its current, no limit "
          "event for its retry (%s)", want);
    CHECK((l = d2_line(ev, 4)) && *l == 0, "d2: nothing else, the end of status");
}

/* longws (r4): WS frames longer than the 16-bit edge counter (TIM4) counts. A WS2805 line of 1,700 chips (68,000 bits,
 * 85 ms a frame), then a WS2915 line of 900 chips (32 + 72,000 bits); auto mode, nothing stored, this board first on
 * the line. 0.6.2: the frame end took the count modulo 65,536 (2,464 / 6,496 bits: no protocol's length), so auto
 * mode never locked and the board stayed dark. */
static uint8_t long_bits[16400];

static void long_frame(uint32_t nbits)
{
    wire_frame(long_bits, nbits, sim_ns + 7000u);
    run_ms((uint64_t)nbits * 1250u / 1000000u + 30u);    /* the frame, then a 30 ms gap */
}

static void sc_longws(void)
{
    static const uint8_t c805[5] = { 0x40, 0x80, 0xC0, 0x20, 0x10 };
    uint8_t c915[14] = { 0xFF, 0xFF, 0xFF, 0x80, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x24, 0x68 };
    uint32_t i, k;
    printf("soak longws: WS2805 / WS2915 frames of 68,000 / 72,032 bits (the edge counter wraps at 65,536)\n");
    for (i = 0; i < sizeof long_bits; i++) {
        long_bits[i] = (uint8_t)(i * 37u + 11u);         /* the chips after this board: anything */
    }
    memcpy(long_bits, c805, sizeof c805);
    boot(0, 0, 0);
    for (k = 0; k < 6u; k++) {
        long_frame(1700u * 40u);
    }
    printf("  WS2805, 68,000 bits: active %u, frames %u, out %04X %04X\n", (uint32_t)proto_active(&frx.ps), frames,
           sm.to[0], sm.to[4]);
    CHECK(proto_active(&frx.ps) == PROTO_WS2805 && sm.to[0] == 0x4040u && sm.to[4] == 0x1010u, "longws: WS2805 "
          "locked and shown (active %u, %04X %04X)", (uint32_t)proto_active(&frx.ps), sm.to[0], sm.to[4]);
    memcpy(long_bits, c915, sizeof c915);
    for (k = 0; k < 6u; k++) {
        long_frame(32u + 900u * 80u);
    }
    printf("  WS2915, 72,032 bits: active %u, frames %u, out %04X %04X\n", (uint32_t)proto_active(&frx.ps), frames,
           sm.to[0], sm.to[4]);
    CHECK(proto_active(&frx.ps) == PROTO_WS2915_32 && sm.to[0] == 0x1234u && sm.to[4] == 0x2468u, "longws: WS2915 "
          "locked and shown (active %u, %04X %04X)", (uint32_t)proto_active(&frx.ps), sm.to[0], sm.to[4]);
    c915[4] = 0x43;                                      /* 1,638 chips: 131,072 bits, the count ends on 0 */
    c915[13] = 0x21;
    memcpy(long_bits, c915, sizeof c915);
    k = frames;
    long_frame(32u + 1638u * 80u);
    long_frame(32u + 1638u * 80u);
    printf("  WS2915, 131,072 bits: %u frames ended of 2, out %04X %04X\n", frames - k, sm.to[0], sm.to[4]);
    CHECK(frames - k == 2u && sm.to[0] == 0x4334u && sm.to[4] == 0x2421u, "longws: a frame of 2 x 65,536 bits ends and "
          "is shown (%u frame ends, %04X %04X)", frames - k, sm.to[0], sm.to[4]);
    CHECK(set.last_proto == PROTO_WS2915_32 && sim_led_on == 0u && events_seen == 0u, "longws: the sticky protocol "
          "follows, no false events");
}

/* d5 (0.6.5): the flash state line. Before the boot: a settings record and one cut after it (a power loss while it
 * was programmed), an effect store whose header lists two lights with wrong CRCs. `status`: both rejected, no error;
 * a setting changed: "settings pending" until the debounced write; a write that fails (write protection): its error
 * and code, the change still pending, then written by the retry. */
static const char *d5_line(void)
{
    const char *p = strstr(con_out, "flash:");
    static char l[160];
    size_t n = p ? strcspn(p, "\r") : 0;
    memcpy(l, p ? p : "", n);
    l[n] = 0;
    return l;
}

static void sc_d5(void)
{
    settings_t s;
    store_ext_t e;
    uint8_t *fx = fl + (SCENE_FX_ADDR - FL_BASE);
    uint32_t k;
    printf("soak d5: the flash state: rejected records, a pending save, a failed write\n");
    settings_default(&s);
    store_ext_clear(&e);
    store_pack_ext(&s, &e, 5u, (store_slot_t *)(fl + (SETTINGS_ADDR - FL_BASE)));
    memset(fl + (SETTINGS_ADDR - FL_BASE) + STORE_SLOT_BYTES, 0x5A, 10);   /* the next record, cut */
    fx[0] = (uint8_t)(ULTRA_FX_MAGIC & 0xFFu);           /* (little endian, as hw.c writes the word) */
    fx[1] = (uint8_t)((ULTRA_FX_MAGIC >> 8) & 0xFFu);
    fx[2] = (uint8_t)((ULTRA_FX_MAGIC >> 16) & 0xFFu);
    fx[3] = (uint8_t)(ULTRA_FX_MAGIC >> 24);
    fx[4] = 0x03;                                        /* lights 0 and 1 listed; their heads (erased) fail the CRC */
    fx[5] = 0x11;
    fx[6] = 0x22;
    boot(0, 0, 0);
    run_ms(1000);
    console("status");
    printf("  boot: %s\n", d5_line());
    CHECK(strcmp(d5_line(), "flash: settings rejected 1, power-on effect rejected 2, errors 0 (-)") == 0, "d5: the "
          "rejected records (%s)", d5_line());
    console("set dither 1");
    run_ms(500);
    console("status");
    printf("  a setting changed: %s\n", d5_line());
    CHECK(strcmp(d5_line(), "flash: settings pending, settings rejected 1, power-on effect rejected 2, errors 0 "
                            "(-)") == 0, "d5: pending (%s)", d5_line());
    run_ms(3000);
    console("status");
    CHECK(!settings_dirty && strstr(d5_line(), "pending") == NULL, "d5: written after the debounce (%s)", d5_line());
    fl_fail = 1;                                         /* the next settings operation fails: write protection */
    console("set dither 2");
    for (k = 0; k < 100u && fl_fail != 0u; k++) {
        run_ms(100);                                     /* (the debounce, the acknowledge blink's end) */
    }
    console("status");
    printf("  a failed write: %s\n", d5_line());
    CHECK(fl_fail == 0u && stat.fl_errs == 1u && strstr(d5_line(), "errors 1 (WRP)") != NULL,
          "d5: the error and its code (%s)", d5_line());
    run_ms(11000);                                       /* (r6: FLASH_RETRY_MS of hold-off after a failure) */
    console("status");
    CHECK(!settings_dirty && set.dither_mode == 2u && strstr(d5_line(), "pending") == NULL, "d5: the retry wrote it");
}

/* minmax (r4): the console's `supply=... (min, max)` covers the time since the last command (the console's snapshot is
 * taken at a command's line end). Typed key by key (a person, ~150 ms a key), 0.6.2 restarted the min / max at every
 * key (the snapshot on any byte), so a dip before the typing never showed. */
static void console_typed(const char *cmd, uint32_t ms_per_key)
{
    con_out_n = 0;
    con_out[0] = 0;
    con_in_r = con_in_w = 0;
    for (; *cmd; cmd++) {
        con_in[con_in_w++] = *cmd;
        run_ms(ms_per_key);
    }
    con_in[con_in_w++] = '\r';
    run_ms(50);
    while (cli.st_part) {
        run_ms(10);                                      /* (0.6.5) a status in parts: to its end */
    }
}

/* the min / max of the `supply=` line, mV (-1: not found) */
static void con_supply_minmax(int32_t *mn, int32_t *mx)
{
    const char *p = strstr(con_out, "supply=");
    unsigned a, b, c, d;
    *mn = *mx = -1;
    if (p && sscanf(p, "supply=%*u.%*u V (min %u.%u V, max %u.%u V)", &a, &b, &c, &d) == 4) {
        *mn = (int32_t)(a * 1000u + b * 10u);
        *mx = (int32_t)(c * 1000u + d * 10u);
    }
}

static void sc_minmax(void)
{
    int32_t mn, mx;
    printf("soak minmax: `status` typed key by key after a sag of the supply\n");
    boot(0, 0, 0);
    run_ms(2000);
    console("status");                                   /* the window starts here */
    supply_mv = 20000u;                                  /* the supply sags to 20 V for 200 ms */
    run_ms(200);
    supply_mv = 24000u;
    run_ms(2000);
    console_typed("status", 150u);
    con_supply_minmax(&mn, &mx);
    printf("  typed: min %d mV, max %d mV\n", (int)mn, (int)mx);
    CHECK(mn >= 19900 && mn <= 20100 && mx >= 23900, "minmax: the sag since the last status shows (min %d mV)",
          (int)mn);
    console_typed("status", 150u);
    con_supply_minmax(&mn, &mx);
    CHECK(mn >= 23900 && mn <= 24100, "minmax: the next status starts again (min %d mV)", (int)mn);
}


#include "test_r5_retry.c"
#include "test_r5_age.c"                         /* (0.6.6: r5age) */
/* (review round 6) the reviewers' scenario files, each with its own dispatcher; r6_scenarios tries them in turn */
#include "test_r6_fable.c"                       /* (fable-fw: r6vdd ...) */
#include "test_r6_gpt.c"                         /* (gpt-fw: r6gcut ...) */
#include "test_r6_claude.c"                      /* (claude-fw: r6race ...) */
#include "test_r6_sys.c"                         /* (fable-sys: r6e2e ...) */
#include "test_r6_console.c"                     /* (0.6.7: the release console: conout ...; the samples) */
#include "test_r7.c"                             /* (0.6.9, review round 7: r7ack ...) */
#include "test_r8.c"                             /* (0.6.10: r8evtype) */
#include "test_r9b.c"                            /* (0.6.10, review round 9 phase B: r9b) */
#include "test_r11.c"                            /* (round 11: r11confirm) */
static int r6_scenarios(const char *sc, int argc, char **argv)
{
    return r6f_scenario(sc) || r6g_scenario(sc) || r6c_scenario(sc) || r6s_scenario(sc, argc, argv) ||
           r6o_scenario(sc) || r7_scenario(sc, argc, argv) || r8_scenario(sc, argc, argv) ||
           r9b_scenario(sc, argc, argv) || r11_scenario(sc, argc, argv);
}

#if MON_EXTRA
/* d9 (0.6.5): backup / restore through main.c (restore_rec). Settings changed, a calibration (`set ical 50`), both
 * stored; `backup`; a factory reset (`defaults`: settings, extension and calibration cleared); the backup line pasted
 * back: the settings and the calibration as before, in effect at once, then stored by the stores' normal paths (the
 * settings after the debounce, the calibration with its budget: read back from the flash model with a pp_find of the
 * test's own, the stores' positions untouched); the line with one digit changed: rejected, nothing changed. */
static int d9_cal_eq(const isense_cal_t *a, const isense_cal_t *b)
{
    return a->levels == b->levels && memcmp(a->pct, b->pct, sizeof a->pct) == 0 && a->zero_ma == b->zero_ma &&
           memcmp(a->ma, b->ma, sizeof a->ma) == 0;
}

static void sc_d9(void)
{
    const pp_desc_t sd = { STORE_SLOT_BYTES, STORE_SLOTS, 4u, 0u, store_valid };
    const pp_desc_t cd = { CAL_REC_BYTES, CAL_RECS, 4u, 0u, cal_valid };
    pp_pos_t tmp;
    const store_slot_t *sr;
    const cal_rec_t *cr;
    char bk[200];
    isense_cal_t c0, c1;
    settings_t s0;
    store_ext_t e0;
    size_t n;
    printf("soak d9: backup, factory reset, restore\n");
    boot(0, 0, 0);
    run_ms(1000);
    console("set dither 1");
    run_ms(4000);
    console("set preset 3");
    run_ms(4000);
    console("set ilimit 100");
    run_ms(4000);
    console("set ical 50");
    run_ms(isense_cal_ms((uint16_t)(50u * 65535u / 100u)) + 4000u);
    CHECK(isense_cal_get(&is, &c0) && c0.levels == 1u && !settings_dirty && cal_save_pend == 0u, "d9: changed, "
          "calibrated, both stored");
    s0 = set;
    e0 = sext;
    console("backup");
    n = strcspn(con_out, "\r");
    CHECK(strncmp(con_out, "restore ", 8) == 0 && n == 158u, "d9: the backup line (%u)", (uint32_t)n);
    memcpy(bk, con_out, n);
    bk[n] = 0;
    console("defaults");
    run_ms(10000);                                       /* the factory reset's pattern, its writes */
    CHECK(set.dither_mode != s0.dither_mode && sext.ilimit50 == 0xFFu && !isense_cal_get(&is, &c1) && !settings_dirty &&
          cal_save_pend == 0u, "d9: the factory reset cleared and stored it");
    {
        char d = bk[60];
        bk[60] = (char)(d == '0' ? '1' : '0');           /* one digit changed */
        console(bk);
        CHECK(strcmp(con_out, "error: restore: CRC\r\n") == 0 && set.dither_mode != s0.dither_mode && !settings_dirty,
              "d9: a changed line: rejected, nothing changed (%s)", con_out);
        bk[60] = d;
    }
    console(bk);
    printf("  restore: %s", con_out);
    CHECK(strcmp(con_out, "ok\r\n") == 0 && memcmp(&set, &s0, sizeof set) == 0 && memcmp(&sext, &e0, sizeof sext) == 0,
          "d9: the settings and the extension back, in effect at once");
    CHECK(isense_cal_get(&is, &c1) && d9_cal_eq(&c1, &c0) && cal_present,
          "d9: the calibration back (its write: at once while dark, %u)", (uint32_t)cal_save_pend);
    CHECK(settings_dirty && eff.dither_mode == 1u && steps == settings_steps(&s0), "d9: in effect; the settings' "
          "write pending (the debounce)");
    run_ms(6000);
    sr = (const store_slot_t *)pp_find(&sd, (const uint8_t *)SETTINGS_ADDR, (const uint8_t *)SETTINGS_B_ADDR, &tmp);
    cr = (const cal_rec_t *)pp_find(&cd, (const uint8_t *)CAL_PAGE_ADDR, (const uint8_t *)CAL_B_ADDR, &tmp);
    CHECK(!settings_dirty && sr && memcmp(&sr->s, &s0, sizeof s0) == 0 && memcmp(&sr->ext, &e0, sizeof e0) == 0,
          "d9: the settings stored by the store (newest record = the backup's)");
    CHECK(cal_save_pend == 0u && cr && d9_cal_eq(&cr->c, &c0), "d9: the calibration stored by its store");
}
#endif
#include "test_r6_candle.c"                      /* (review round 6: r6candle) */

int main(int argc, char **argv)
{
    const char *sc = argc > 1 ? argv[1] : "wrap";
    flash_init();
    if (strcmp(sc, "r5retry") == 0) {
        sc_r5retry();
    } else if (strcmp(sc, "r5age") == 0) {
        sc_r5age();
    } else if (strcmp(sc, "r6candle") == 0) {
        sc_r6candle();
    } else if (verify_scenario(sc) || r4_scenario(sc) || r5_scenario(sc) || rc_scenario(sc) ||
               r6_scenarios(sc, argc, argv)) {
    } else if (strcmp(sc, "wrap") == 0) {
        sc_wrap();
    } else if (strcmp(sc, "r5cpu") == 0) {
        sc_r5cpu();
    } else if (strcmp(sc, "d1") == 0) {
        sc_d1();
    } else if (strcmp(sc, "long") == 0) {
        sc_long();
    } else if (strcmp(sc, "fx") == 0) {
        sc_fx();
    } else if (strcmp(sc, "r5ovr") == 0) {
        sc_r5ovr();
    } else if (strcmp(sc, "r6takeover") == 0) {
        sc_r6takeover();
    } else if (strcmp(sc, "r6takeover5") == 0) {
        sc_r6takeover5();
    } else if (strcmp(sc, "r6rejoin5") == 0) {
        sc_r6rejoin5();
    } else if (strcmp(sc, "r6takeover5d") == 0) {
        sc_r6takeover5d();
    } else if (strcmp(sc, "r6fxtake") == 0) {
        sc_r6fxtake();
    } else if (strcmp(sc, "r6takeover0") == 0) {
        sc_r6takeover0();
    } else if (strcmp(sc, "cfg") == 0) {
        sc_cfg();
    } else if (strcmp(sc, "d2") == 0) {
        sc_d2();
    } else if (strcmp(sc, "spec") == 0) {
        sc_spec();
    } else if (strcmp(sc, "r4rx") == 0) {
        sc_r4rx();
    } else if (strcmp(sc, "r4reset") == 0) {
        sc_r4reset();
    } else if (strcmp(sc, "r4gate") == 0) {
        sc_r4gate();
    } else if (strcmp(sc, "d5") == 0) {
        sc_d5();
    } else if (strcmp(sc, "race") == 0) {
        sc_race();
    } else if (strcmp(sc, "r6gamma0") == 0) {
        sc_r6gamma0();
#if BOARD_TESTJIG
    } else if (strcmp(sc, "r5jig") == 0) {
        sc_r5jig();
    } else if (strcmp(sc, "r5jig6") == 0) {
        sc_r5jig6();
    } else if (strcmp(sc, "r5jig6p") == 0) {
        sc_r5jig6p();
#endif
    } else if (strcmp(sc, "tripreset") == 0) {
        sc_tripreset();
    } else if (strcmp(sc, "longws") == 0) {
        sc_longws();
#if MON_EXTRA
    } else if (strcmp(sc, "d9") == 0) {
        sc_d9();
#endif
    } else if (strcmp(sc, "minmax") == 0) {
        sc_minmax();
    } else if (strcmp(sc, "r6seqwrap") == 0) {
        sc_r6seqwrap();
    } else {
        printf("soak: unknown scenario %s\n", sc);
        return 2;
    }
    CHECK(fl_bad == 0u, "%s: no half-word programmed that wasn't erased (%u)", sc, fl_bad);
    CHECK(fl_outside == 0u, "%s: flash operations only on the record pages and the update status records (%u)", sc,
          fl_outside);
    CHECK(fl_bright_erases == 0u, "%s: erases only while every output is below 15 %% (%u erases while brighter, %u "
          "while lit)", sc, fl_bright_erases, fl_lit_erases);
    printf("soak %s: %s (%d failures), %u s simulated\n", sc, g_fail ? "FAILED" : "passed", g_fail, (uint32_t)now_s());
    return g_fail ? 1 : 0;
}
