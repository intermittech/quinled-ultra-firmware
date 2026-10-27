/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Diff-Power-UR-Digital-M's delay line (BOARD 3; docs/ur_digital_m-firmware.md sections 3-5 and
 * the design review of 2026-10-15): the window logic over the 512-word DMA ring. Pure logic, host-tested
 * (test/test_dline.c: a sample-exact model of the ring, the mux, BLANK_N, DE and a hub); mcu/ch32v20x/hw_dline.c runs the
 * hardware, main_ur_digital_m.c the glue (the actions below) and the decode.
 *
 * The ring: SPI1 samples PA6 at 36 MHz into 16-bit words (MSB = the earliest sample); RX DMA writes word k into
 * ring[k % 512], TX DMA replays ring[k % 512] as output word k + 512 (D = 227.6 us; it reads a word 2 words ahead of
 * RX's write). What the strip sees is decided by zeroing ring words before TX reads them (deadline: ~510 words after
 * the write, ~226 us); the tick runs every 50 us, so every decision here is taken from the ring itself, word by word,
 * not from the pin or a timer: the same scan serves the 20 kHz tick and the flash operation's SRAM tick (no TIM3
 * thresholds, which the SRAM tick can't serve; a 20 us marker between two SRAM ticks is still in the ring).
 *
 * The scan (every word written since the last tick, once):
 *  - PASS: >= DLINE_W_LONG all-HIGH words in a row = a long HIGH (pixel highs <= 1.35 us give <= 3): zero it from the
 *    rising edge on (the samples before the edge in that word stay) -> MARK. A LOW run of DLINE_W_LATCH words = a latch
 *    (once per run): a frame ended (counted when the line was active since the last one), its bits = TIM4's falling
 *    edges since the last latch (only when the count is exact: below). DLINE_W_QUIET LOW words through the newest
 *    word while direct: ask for the mux switch (MUX_DELAYED: both paths LOW, nothing repeated or cut).
 *  - MARK: the marker's HIGH, zeroed; its falling-edge word ends it (-> FRAME) within 60 us, else HOLD.
 *  - FRAME: from the word after the marker's falling-edge word (ws_decode needs a LOW start; the capture starts at an
 *    even word for 32-bit copies: then that word, zeroed, comes first), each word copied into the capture
 *    (DLINE_CAP_WORDS, only while the main loop holds none) and zeroed. LOW >= DLINE_W_LATCH words: the frame end (-> END); a long HIGH first: the
 *    hub released the line within 46 us of its last bit (-> REPLY). FRAME_READY: the main loop decodes the capture.
 *  - END: a long HIGH (DLINE_W_LONG all-HIGH words) starting before t_end + 250 us is the release (zeroed -> REPLY); a
 *    shorter HIGH is pixels (the hub broke its >= 300 us after a frame): PASS at that word, unzeroed; t_end + 250 us:
 *    PASS whatever comes (a non-RW frame, or a hub too late: no answer), without the main loop.
 *  - REPLY: the released line (zeroed). The tick sends (SEND) once the answer is armed (dline_arm), the release has
 *    lasted >= 20 us and t_end + 900 us has not passed; the release's first 1 -> 0 transition without our burst: the
 *    hub took the line back (-> PASS at that word); still HIGH 5 ms after t_end: HOLD.
 *  - BURST: DE high: own-transmit echo is unavailable, the ring reads LOW. The release's 1 -> 0 (our DE rise) ends its
 *    zeroing. dline_burst_done (DE low) gives the DE-low word -> RETAKE.
 *  - RETAKE: from the DE-low word: a HIGH within DLINE_W_RO_EN words (the idle-HIGH line, any length) is zeroed through
 *    its first 1 -> 0 transition (the hub's retake), that word included; a line still LOW there: the hub had taken it
 *    already -> PASS at once. The zeroing never runs past that word (the hub's pixels come >= 20 us after its retake).
 *  - HOLD: a long HIGH that isn't a marker (the line released / undriven), zeroed through its first 1 -> 0 transition.
 * BLANK_N (belt and braces) is LOW from SEND to the window's end; in delayed mode the ring is already zero there.
 *
 * Polls must never show on the strip, so the mux goes direct -> delayed only behind BLANK_N or a live check:
 *  - BOOT (dline_init) and after a fault: BLANK_N LOW (the strip line held LOW), the mux as it is. At the first quiet
 *    run (>= DLINE_W_QUIET LOW words through the newest word: the ring's last 512 words are LOW) the mux goes delayed
 *    and BLANK_N is released in the same tick (both paths LOW). A hub gives >= 350 us LOW before every marker: 231 us
 *    + a tick 50 us late < 300. No quiet run within 50 ms (a plain controller at full rate with short resets): BLANK_N
 *    is released in direct mode inside a latch gap (>= DLINE_W_GAP LOW words) and the mux goes delayed at a later quiet
 *    run (DIRECT-PASS). Those two need the live check (DLINE_A_LIVE_CHECK): the direct path is visible.
 *  - A fault (the glue's report: SPI OVR, a DMA error; or found here: RX - TX distance outside 1-3 words, RX not moving
 *    for two ticks): BLANK_N LOW and RESTART (the glue zeroes and restarts the ring, the mux stays: delayed replays the
 *    zeroed ring), then BLANK_N held until a quiet run, as BOOT, but without the 50 ms fallback once the mux is delayed
 *    (never back to direct: the delayed path is a whole stream). No scan until dline_restart.
 *
 * Bits per frame: TIM4 counts PA6's falling edges (= bits). A latch's count is exact when the line stayed LOW from the
 * latch to the newest word of the tick that found it and through the word RX was writing then (TIM4 is read before the
 * RX position: an edge counted is in the ring or in that word; checked at the next tick); otherwise (a reset under
 * ~100 us: the next frame already started) that frame's bits are not recorded. A
 * window's end re-bases the count the same way. TIM4 is 16 bits: frames over 65,535 bits wrap.
 *
 * Contexts: dline_tick (the 20 kHz tick), dline_ram_tick (the SRAM tick during a flash operation; it and everything it
 * calls is DLINE_RAMCODE or inline), dline_fault / dline_restart / dline_refused (the tick's context) don't preempt each
 * other; dline_burst_done (the USART1 TC interrupt) may preempt them; dline_arm and the capture belong to the main loop
 * between FRAME_READY and dline_arm. */
#ifndef DLINE_H
#define DLINE_H

#include <stdint.h>
#include "../mcu/profile.h"
#include "ws_decode.h"

#define DLINE_RING_WORDS   MCU_DLINE_RING_WORDS        /* = hw_dline.h DL_RING_WORDS (a power of two: the positions' mask) */
#define DLINE_RING_MASK    (DLINE_RING_WORDS - 1u)
#define DLINE_CAP_WORDS    (MCU_DLINE_CH_TAIL_WORDS + WS_LINE_WORDS)   /* the chain snapshot + the acquisition runs */
#define DLINE_W_PER_MS     MCU_WORDS_MS       /* ring words per ms (36 MHz / 16) */

/* thresholds in whole ring words (16 samples, 0.444 us) */
#define DLINE_W_LONG       MCU_US2W(10u)
#define DLINE_W_MARK_MAX   MCU_US2W(60u)        /* a marker's HIGH ends within 60 us, else HOLD */
#define DLINE_W_LATCH      MCU_US2W(46u)        /* LOW words: a latch / a frame end (46 us; a 50 us reset gives >= 111) */
#define DLINE_W_QUIET      MCU_US2W(231u)        /* LOW words through the newest: the mux may switch (231 us > D = 512) */
#define DLINE_W_GAP        MCU_US2W(50u)        /* LOW words through the newest: inside a latch gap (50 us; the fallback) */
#define DLINE_W_FALLBACK   MCU_US2W(50000u)     /* BOOT / fault: no quiet run within 50 ms: release BLANK_N direct */
#define DLINE_W_END        MCU_US2W(250u)        /* no release by t_end + 250 us: PASS */
#define DLINE_W_RELEASED   MCU_US2W(20u)         /* the release's all-HIGH words before SEND (20 us) */
#define DLINE_W_REPLY_MAX  (MCU_US2S(900u) / MCU_SAMPLE_BITS - 1u)       /* SEND no later than t_end + 900 us (16 x 2024 + 15 samples < 32,400) */
#define DLINE_W_HOLD       MCU_US2W(5000u)      /* the released line HIGH 5 ms after t_end: HOLD */
#define DLINE_W_RO_EN      MCU_NS2W(1777u)          /* after DE low: a HIGH starting within 1.8 us is the idle-HIGH line (the receive output's
                                          enable from high-Z), a line LOW through it was taken by the hub */
#define DLINE_W_TX_MAX     MCU_NS2W(1422222u)       /* no burst-done 1.42 ms after SEND (> 3 + 1,320 us at 250 kbaud): DE low */
#define DLINE_W_LATE       MCU_SCAN_LATE_WORDS        /* the scan this far behind RX: the words are (nearly) out of the TX: lost */
#define DLINE_EDGE_MASK    0xFFFFu     /* TIM4's width */

/* CHAIN line mode (docs/ur_digital_m-firmware.md section 12): the line carries plain Ultra frames (N slots + the header trailer, no
 * markers, no pixels) and the board's encoder owns every ring word, so the raw frame is copied by a second scanner (dl_ch_scan) in
 * the 20 kHz tick, before the encoder replaces the words and before the stream scanner (which keeps running: markers, windows) looks
 * at them. cap[0..1024) is an ISR-written circular raw snapshot; ch_hn publishes progress after its words.
 * The main loop consumes that progress incrementally, before the snapshot wraps. cap[1024..DLINE_CAP_WORDS)
 * holds acquisition runs owned only by the main loop. The ISR freezes raw capture and metadata at ch_ready.
 * Acquisition releases raw ownership once runs are complete; locked decoding releases after reading the trailer.
 * The next frame can then refill raw storage while the main loop aggregates the separate acquisition runs. */
#define DLINE_CH_HEAD      0u
#define DLINE_CH_TAIL      MCU_DLINE_CH_TAIL_WORDS   /* raw snapshot (1024 words at 36 MHz, 2048 at 50 MHz), consumed incrementally */
#define DLINE_CH_TAIL_USED MCU_DLINE_CH_TAIL_USED   /* slowest profile, including raw clock tolerance; they end at ch_tw: 8 LOW words
                                          after the last pulse when the LOW run ended the frame (DLINE_CH_TAIL_CUT before the snapshot's end) */
#define DLINE_CH_TAIL_CUT  (DLINE_W_LATCH - 8u)
/* A valid RW release follows >=60 us LOW. The 46 us frame-end threshold
 * therefore publishes the capture before release; an earlier HIGH is invalid. */
#define DLINE_W_CH_LATE    MCU_US2W(80u)        /* an armed answer is sent at most this many words (80 us = ULTRA_RW_LATE_US) after its start word */
#define DLINE_CH_WIN_W     (MCU_WORDS_MS * 25u)   /* a window after an RW frame lasts at most 24.2 ms (16 slots at 250 kbaud) */

/* states */
enum {
    DLINE_PASS = 0, DLINE_MARK, DLINE_HOLD, DLINE_FRAME, DLINE_END, DLINE_REPLY, DLINE_BURST, DLINE_RETAKE
};

enum { DLINE_CH_IDLE = 0, DLINE_CH_FRAME, DLINE_CH_HIGH };
typedef char dline_ch_fits[(DLINE_CH_TAIL + WS_LINE_WORDS <= DLINE_CAP_WORDS) ? 1 : -1];

/* actions (bits) for the glue, from dline_tick / dline_ram_tick / dline_fault. Order: BLANK_ON first, then the mux,
 * then BLANK_OFF; SEND after BLANK_ON */
#define DLINE_A_MUX_DELAYED 0x01u      /* MUX_SEL delayed */
#define DLINE_A_MUX_DIRECT  0x02u      /* MUX_SEL direct (not sent: never back to direct once delayed; a glue may keep
                                          it for its own faults) */
#define DLINE_A_BLANK_ON    0x04u      /* BLANK_N LOW (SEND; a fault) */
#define DLINE_A_BLANK_OFF   0x08u      /* BLANK_N released (the window's end; a quiet run after BOOT / a fault) */
#define DLINE_A_FRAME_READY 0x10u      /* a marker frame is captured: cap[0 .. cap_n), its last edge at word end_w */
#define DLINE_A_SEND        0x20u      /* DE high and the armed answer's burst now (BLANK_ON comes with it) */
#define DLINE_A_WINDOW_END  0x40u      /* back to PASS (a flash write may follow: dline_idle) */
#define DLINE_A_RESTART     0x80u      /* a fault: zero and restart the ring (the mux as it is), then dline_restart */
#define DLINE_A_EDGES_ZERO  0x200u     /* CHAIN: the line is in a gap (LOW >= a latch): the glue zeroes TIM4 (dl_hw_edges_zero): the
                                          next frame's bits are TIM4's count. dline is TIM4's only writer (a gate in a later phase
                                          re-arms from here too) */
#define DLINE_A_CH_READY    0x400u     /* CHAIN: a frame is captured (ch_*): the main loop decodes it, then dline_ch_arm / _release */
#define DLINE_A_CH_MARKER   0x800u     /* CHAIN: a marker (a HIGH of 3.6-60 us that ends) outside a window: a pixel-line hub is talking */
#define DLINE_A_CH_MISSED   0x1000u    /* CHAIN: an armed answer's start word passed (a late tick, a flash operation) */
#define DLINE_A_CH_TURN     0x2000u    /* CHAIN relay window: turn around now (the gate closed, port 2 listens, the
                                          input drives the line above from the relay node, unless that line is still driven) */
#define DLINE_A_CH_FWD      0x4000u    /* CHAIN relay window: forward again (the window's end; a stop or a fault inside one) */
#define DLINE_W_CH_TURN_LATE MCU_US2W(400u)   /* a turnaround later than ULTRA_RW_DIR_MAX_US (900 = RC_DIR_US 500 + 400) is not taken */
#define DLINE_A_LIVE_CHECK  0x100u     /* apply this call's MUX / BLANK_OFF only if PA6 is LOW and TIM4 still reads the
                                          edges passed in, TIM4 read >= ~0.1 us after PA6 (its input filter's delay: a
                                          whole pulse just before could be in neither); no edge since: the visible
                                          direct path is in its gap. Else apply none of them and call dline_refused
                                          (asked again at a later tick) */

typedef struct {
    uint32_t frames;                   /* frames (latches after activity, outside windows) since power-up */
    uint32_t bits_last;                /* falling edges (bits) of the last measured frame */
    uint32_t bits_min, bits_max;       /* ... since dline_minmax_restart (min 0xFFFFFFFF: none) */
    uint32_t bits_n;                   /* frames measured */
    uint16_t long_high;                /* HOLDs: a HIGH > 60 us that isn't a marker, a released line > 5 ms */
    uint16_t marks;                    /* marker frames (windows) */
    uint16_t answered;                 /* SENDs (wraps) */
    uint8_t  cap_drops;                /* marker frames not captured: the main loop still had one, or a flash op
                                          (saturating) */
    uint8_t  late;                     /* ticks that came too late for the ring (saturating) */
    uint8_t  faults;                   /* delay-line faults: reported (dline_fault) or found (saturating) */
} dline_stats_t;

typedef struct {
    /* ---- the scan (the ticks) ---- */
    volatile uint8_t line_profile;     /* main publishes 255 for acquisition, otherwise a qualified profile */
    uint8_t ch_profile;                /* immutable profile of the captured frame */
    uint32_t wpos;                     /* RX words written since start (free-running: the slot is wpos & mask) */
    uint32_t zpos;                     /* the next word the scan looks at */
    uint32_t zs, os;                   /* PASS / END: where the current all-LOW / all-HIGH word run started */
    uint32_t lzs;                      /* the LOW run of the last latch or window end (the next latch counts a frame
                                          only if the line was active since) */
    uint32_t sync_zs;                  /* the LOW run of this tick's latch / window end (its TIM4 count is exact if
                                          the run reaches the tick's newest word) */
    uint32_t run;                      /* MARK: its all-HIGH words; FRAME: LOW words; REPLY: the release's HIGH words */
    uint32_t orun;                     /* FRAME: all-HIGH words in a row */
    uint32_t m0, m1;                   /* FRAME: the last two words with an edge (t_end) */
    uint32_t mark_w, end_w, rel_w;     /* the marker's rising-edge word, the frame's last edge (t_end), the release's */
    uint32_t send_w, txend_w;          /* SEND's newest word, DE low's word */
    uint32_t edge_w;                   /* the word of the line's last edge (signal loss: (wpos - edge_w) / 2250 ms) */
    uint32_t rec_w;                    /* BOOT / the restart after a fault: since this word (the 50 ms fallback) */
    uint32_t last_rx;                  /* the RX position at the last tick (RX not moving: a fault) */
    uint16_t v1;                       /* FRAME: the word m1 */
    uint32_t pend_w;                   /* ... the word RX was writing at that latch / window end */
    uint16_t base;                     /* TIM4 at the last exact latch / window end */
    uint16_t pend_e;                   /* TIM4 at a latch / window end, exact if word pend_w comes in all LOW */
    uint16_t cap_n;                    /* captured words */
    uint8_t  state;                    /* DLINE_* */
    uint8_t  direct;                   /* the mux is direct: 1 (BOOT, the fallback) */
    uint8_t  rec;                      /* BOOT / after a fault: BLANK_N held until a quiet run (or the fallback) */
    uint8_t  restart;                  /* a fault: no scan until dline_restart (1: found by the SRAM tick, the RESTART not
                                          asked for yet; 2: asked for) */
    uint8_t  stall;                    /* ticks in a row without RX moving */
    uint8_t  bl;                       /* BLANK_N as the glue was told (blank || rec) */
    uint8_t  undo;                     /* LIVE_CHECK: direct | rec << 1 | bl << 2 before it (dline_refused) */
    uint8_t  blank;                    /* the window holds BLANK_N LOW (SEND .. its end) */
    uint8_t  local;                    /* local pixel effects: the encoder (core/ur_digital_m/udfx.c) owns every ring word from its
                                          position on, so the window never asks for BLANK_N (it would cut the effect's
                                          frame) and a fault's recovery needs no quiet run. Set and cleared by udfx_tick
                                          only in PASS, behind a latch */
    uint8_t  hi;                       /* BURST: the release still HIGH in the ring; RETAKE: inside the idle HIGH */
    uint8_t  txdone;                   /* BURST: DE is low (txend_w) */
    uint8_t  cap_ok;                   /* FRAME: every word went into the capture */
    uint8_t  cap_own;                  /* FRAME: the capture is this frame's (the main loop holds none) */
    uint8_t  base_ok;                  /* base is exact */
    uint8_t  sync;                     /* a latch (2) / window end (1) in this tick, its TIM4 count pending */
    uint8_t  pend_k;                   /* a latch (2) / window end (1) waiting for word pend_w */
    uint8_t  seq, cur_seq;             /* FRAME_READY's number; the current window's (0: not captured) */
    volatile uint8_t  cap_ready;       /* FRAME_READY given, until dline_arm (no new capture meanwhile) */
    volatile uint8_t  tx_flag;         /* dline_burst_done -> the tick */
    volatile uint16_t tx_raw;          /* ... RX's position at DE low */
    volatile uint32_t arm;             /* dline_arm: seq << 2 | 1 (answer) / 2 (none) */
    dline_stats_t st;
    uint32_t fall_w;                   /* FRAME: the marker's falling-edge word */
    uint32_t cap_w0;                   /* the ring word of cap[0] (even: 32-bit copies; the word after the marker's
                                          falling-edge word, or that word itself, zeroed) */
    uint16_t *cap;                     /* the capture of the last marker frame: the caller's DLINE_CAP_WORDS, 4-byte aligned
                                          (dline_init); a pointer, so that this state stays small (gp's window, main_ur_digital_m.c) */
    /* ---- CHAIN (dl_ch_scan; ch_on 0: none of it runs; after the capture: the offsets of everything above stay as they were) */
    uint32_t cpos;                     /* the next word the chain scanner looks at */
    uint32_t ch_end;                   /* the handed-over frame's last edge (t0): the ring word */
    uint32_t ch_tx;                    /* the armed answer: the word it starts at */
    uint32_t ch_win;                   /* the window's end (no marker is counted before it) */
    uint32_t ch_m1;                    /* FRAME: the last word with a HIGH sample */
    uint16_t ch_hn;                    /* published raw words since ch_cjs - 1; not a linear buffer length */
    uint16_t ch_tn;                    /* the frame's words in the snapshot, the last DLINE_CH_TAIL of them valid (saturating) */
    uint32_t ch_tw;                    /* the snapshot's end: the ring word just past the frame's last word copied */
    uint32_t ch_js;                    /* FRAME: the frame's first word */
    uint32_t ch_cjs;                   /* ... of the frame that is in the capture (the main loop's tag for the slot it decoded early) */
    uint16_t ch_bits;                  /* the handed-over frame's bits: TIM4 */
    uint16_t ch_lz;                    /* IDLE / FRAME: the LOW run in words; HIGH: the HIGH run (saturating) */
    uint8_t  ch_on;                    /* the chain scanner runs */
    uint8_t  ch_st;                    /* DLINE_CH_* */
    uint8_t  ch_o;                     /* FRAME: all-HIGH words in a row */
    uint8_t  ch_cap;                   /* FRAME: the words go to the capture */
    uint8_t  ch_enter;                 /* 1: zero the ring and switch the mux at the next tick (BLANK_N held); 2: done, udfx_tick takes it */
    uint8_t  ch_hold;                  /* AUTO deciding: the stream scanner's BOOT recovery releases neither BLANK_N nor the mux */
    uint8_t  ch_skip;                  /* the SRAM tick ran (a flash operation): the scanner starts again at the newest word */
    volatile uint8_t ch_req;           /* main loop -> tick: 1 start, 2 stop (|4: enter) */
    volatile uint8_t ch_ready;         /* raw snapshot/metadata owned until dline_ch_free */
    volatile uint8_t ch_arm;           /* dline_ch_done publishes an answer waiting for ch_tx */
    volatile uint8_t ch_rel;           /* the relay window: 1 armed (turn at ch_dir), 2 turned (forward again at ch_fwd), 0 none */
    uint32_t ch_dir, ch_fwd;           /* ... the ring words of the turnaround and of forward-again */
} dline_t;

/* SRAM placement of the flash operation's tick and what it calls (the host: nothing) */
#ifndef DLINE_RAMCODE
#if defined(__GNUC__) && defined(__riscv)
#define DLINE_RAMCODE MCU_RAMCODE
#else
#define DLINE_RAMCODE
#endif
#endif

/* BOOT: the mux direct and BLANK_N LOW (the app sets both before the delay line runs); rx_pos: RX DMA's next word
 * (hw_dline.h dl_hw_rx_pos) */
void     dline_init(dline_t *d, uint32_t rx_pos, uint16_t *cap);   /* cap: DLINE_CAP_WORDS, 4-byte aligned */
/* the 20 kHz tick: edges = TIM4 (dl_hw_edges), read BEFORE the positions; tx_pos = dl_hw_tx_pos(). Returns DLINE_A_* */
uint32_t dline_tick(dline_t *d, uint16_t *ring, uint32_t rx_pos, uint32_t tx_pos, uint32_t edges);
/* the same during a flash operation (SRAM; no capture, no SEND). Never returns RESTART (it needs the flash): a fault
 * found here returns BLANK_ON, scans no more, and the next dline_tick (or dline_fault) returns the RESTART */
DLINE_RAMCODE uint32_t dline_ram_tick(dline_t *d, uint16_t *ring, uint32_t rx_pos, uint32_t tx_pos, uint32_t edges);
/* main loop, after FRAME_READY: the capture decoded; ok = a valid RW POLL for this board, its answer built (the glue
 * sends it at SEND). Every FRAME_READY needs one call (it releases the capture) */
void     dline_arm(dline_t *d, int ok);
/* the burst's last stop bit is out and DE is low (USART1 TC): rx_pos = dl_hw_rx_pos() at that moment */
void     dline_burst_done(dline_t *d, uint32_t rx_pos);
/* the glue's fault report (SPI OVR, a DMA error; from the tick's context): the window dropped, BLANK_N LOW. Returns
 * DLINE_A_RESTART | BLANK_ON (| WINDOW_END); 0 when the RESTART was asked for already */
uint32_t dline_fault(dline_t *d);
/* after the glue's restart (the ring zeroed, running again from RX position rx_pos; the mux unchanged) */
void     dline_restart(dline_t *d, uint32_t rx_pos);
/* inline always: the SRAM tick's glue calls these during a flash operation (an outlined copy would be in the flash) */
#if defined(__GNUC__)
#define DLINE_INLINE static inline __attribute__((always_inline))
#else
#define DLINE_INLINE static __inline
#endif
/* the glue didn't apply a LIVE_CHECK action set (the line not in its gap any more): the state before it again */
DLINE_INLINE void dline_refused(dline_t *d)
{
    d->direct = (uint8_t)(d->undo & 1u);
    d->rec = (uint8_t)(d->undo >> 1 & 1u);
    d->bl = (uint8_t)(d->undo >> 2 & 1u);
}
/* the page sent min / max: start again */
void     dline_minmax_restart(dline_t *d);

/* ---- CHAIN (main loop side; the tick takes the requests) */
/* start the chain scanner at the next tick; enter: BOOT's wait is over (dline_ch_hold ends too): at the next tick the ring is zeroed and
 * the mux switched while BLANK_N is still held; udfx_tick starts the encoder (d->ch_enter == 2) and BLANK_N follows */
void     dline_ch_start(dline_t *d, int enter);
void     dline_ch_stop(dline_t *d);
/* AUTO deciding (1): the stream scanner's BOOT recovery releases neither BLANK_N nor the mux until 0 */
DLINE_INLINE void dline_ch_hold(dline_t *d, int on) { d->ch_hold = (uint8_t)(on != 0); }
/* CH_READY: the capture is no longer needed (the slot and the header are decoded): the next frame may be captured. Call it as early as
 * the raw words are out, before the rest of the work: a hub at its 300 us reset starts the next frame 300 us after this one ends */
void     dline_ch_free(dline_t *d);
/* after CH_READY and the work: this board's answer starts at word tx_w (answer = 1) and the window ends at win_w (0: no RW frame);
 * the relay window: relay = a valid POLL for this board or RELAY_ONLY: turnaround at word dir_w (TURN), forward again at fwd_w (FWD);
 * the boards behind answer through it. One publication: the tick never sees the window without its answer (a busy turnaround drops
 * the answer). Every CH_READY needs one call (and one dline_ch_free) */
void     dline_ch_done(dline_t *d, int answer, uint32_t tx_w, uint32_t win_w, uint32_t relay, uint32_t dir_w, uint32_t fwd_w);
/* the frame's last words (the header at their end), in time order, in the snapshot ring: ring[(start + k) & (DLINE_CH_TAIL - 1)], k < *words
 * (<= DLINE_CH_TAIL_USED). The next frame's capture overwrites the ring from its first word on, and so these words: read the header BEFORE dline_ch_free
 * (a capture starts at the earliest when the capture is free; after an RW frame the released line's HIGH starts one 150 us after the frame) */
DLINE_INLINE const uint16_t *dline_ch_tail(const dline_t *d, uint32_t *start, uint32_t *words)
{
    const uint32_t n = d->ch_tn < DLINE_CH_TAIL_USED ? d->ch_tn : DLINE_CH_TAIL_USED;
    *start = (d->ch_tw - n) & (DLINE_CH_TAIL - 1u);
    *words = n;
    return d->cap + DLINE_CH_HEAD;
}
/* no flash operation while a chain frame is being captured, decoded, or an answer waits for its word */
DLINE_INLINE int dline_ch_idle(const dline_t *d)
{
    return !d->ch_on || (d->ch_st == DLINE_CH_IDLE && !d->ch_ready && !d->ch_arm && !d->ch_rel);
}
/* no window open and no capture pending: a flash operation may start (it must stay off a window) */
DLINE_INLINE int dline_idle(const dline_t *d) { return d->state == DLINE_PASS && !d->cap_ready; }
/* the incoming line has been LOW for this many words through the newest (0: not in PASS, or not LOW now): a latch is
 * DLINE_W_LATCH. udfx_tick switches modes only inside one */
DLINE_INLINE uint32_t dline_low_run(const dline_t *d)
{
    return d->state == DLINE_PASS && d->zs != d->wpos ? d->wpos - d->zs : 0u;
}
/* the hal's RX / TX distance check (TX 1-3 words ahead of RX's write; else RX stalled: the ring is stale) */
DLINE_INLINE int dline_dist_ok(uint32_t rx_pos, uint32_t tx_pos)
{
    return ((tx_pos - rx_pos) & DLINE_RING_MASK) - 1u < 3u;
}

#endif
