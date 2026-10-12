/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED dig2analog-ultra-5ch bootloader (research/firmware-update.md, README.md here).
 *
 * Every reset starts here, at HSI 8 MHz:
 *   fast path: descriptor CRC (CRC unit, 15 words) + status records -> confirmed app: jump (well under 0.1 ms, no
 *              clock, no peripheral touched, so a Diff-Power switch-on is as fast as without a bootloader);
 *   full mode: 144 MHz, DIN sampler, gate, watchdog: verify a fresh image, an unconfirmed app's trial window, a
 *              session the app asked for, or recovery. It always leaves through a software reset.
 * No interrupts are used. */
#include "ch32v20x.h"
#include "bl_core.h"
#include "bl_hw.h"
#include "bl_rx.h"

#ifndef BL_BUILD_ID
#define BL_BUILD_ID "dev00000"
#endif

__attribute__((section(".blinfo"), used)) const upd_bl_info_t bl_info = {
    { 'Q', 'D', 'B', 'L' }, BL_VERSION, BL_HW_ID, BL_HW_REV, 0, BL_BUILD_ID
};

#define HO ((upd_handover_t *)UPD_HANDOVER_ADDR)
#define GAP_TICKS     (100u * HW_TICKS_US)        /* DIN quiet this long: the frame ended (re-arm the gate) */
#define STALL_TICKS   (800u * HW_TICKS_US)        /* the ring holds 0.91 ms: after a longer stall, resync */

static bl_sess_t sess __attribute__((section(".noinit")));
static bl_rx_t rx __attribute__((section(".noinit")));
static bl_frame_t frame __attribute__((section(".noinit")));

static void report(const char *what, uint32_t v)
{
    hw_puts(what);
    hw_puthex(v);
    hw_puts("\r\n");
}

int main(void)
{
    static const bl_rx_timing_t timing = BL_RX_TIMING_36MHZ;
    bl_app_t app;
    bl_action_t act;
    uint32_t flags = hw_reset_flags_take(), reset, rd = 0, last_feed, gate_tick, now;
    uint16_t gate_cnt = 0;

    reset = (flags & (RCC_IWDGRSTF | RCC_WWDGRSTF)) ? BL_RST_WATCHDOG
          : (flags & (RCC_PORRSTF | RCC_PINRSTF | RCC_LPWRRSTF)) ? BL_RST_POWER : BL_RST_SOFT;
    hw_crc_on();
    bl_app_read(&app);
    act = bl_decide(&app, HO, reset, flags);
    if (act == BL_ACT_JUMP) {
        hw_jump_app();
    }

    hw_full_init();
    report("QDBL action ", act);
    report("app version ", app.version);
    if (act == BL_ACT_VERIFY) {
        act = bl_after_verify(&app);
        report("verified ", act);
        if (act == BL_ACT_JUMP) {
            HO->request = UPD_REQ_BOOT_ONCE;
            bl_ho_seal(HO);
            hw_soft_reset();
        }
    }
    bl_sess_init(&sess, act, &app, hw_ms());
    bl_rx_init(&rx, &timing);
    hw_gate_open();
    rd = hw_ring_pos();
    last_feed = gate_tick = hw_ticks();

    for (;;) {
        uint32_t wr, n;
        hw_kick();
        now = hw_ticks();

        /* gate: the frame end in real time from the edge counter; plan for the next frame from the decoder */
        {
            uint16_t c = hw_edges();
            if (c != gate_cnt) {
                gate_cnt = c;
                gate_tick = now;
            } else if (c != 0u && now - gate_tick >= GAP_TICKS) {
                if (sess.gate == BL_GATE_ULTRA) {
                    hw_gate_ultra();
                } else {
                    hw_gate_open();
                }
                gate_cnt = 0;
            }
        }

        /* decoder: at most BL_RX_MAX_FEED words per pass, so the gate check runs often */
        wr = hw_ring_pos();
        if (hw_flash_stall || now - last_feed > STALL_TICKS) {
            hw_flash_stall = 0;
            bl_rx_resync(&rx);                    /* flash work (sampled at half speed) or the ring wrapped */
            rd = wr;
        }
        last_feed = now;
        n = (wr - rd) & (HW_RING_WORDS - 1u);
        if (n > BL_RX_MAX_FEED) {
            n = BL_RX_MAX_FEED;
        }
        if (n > HW_RING_WORDS - rd) {
            n = HW_RING_WORDS - rd;
        }
        if (n != 0u) {
            int got = bl_rx_feed(&rx, &hw_ring[rd], n, &frame);
            rd = (rd + n) & (HW_RING_WORDS - 1u);
            if (got) {
                bl_sess_frame(&sess, frame.bad ? 0u : frame.nbits, frame.tail, hw_ms());
            }
        }

        if (bl_sess_poll(&sess, hw_ms())) {
            report("exit request ", sess.exit_request);
            report("result ", sess.exit_result);
            report("upd frames ", sess.upd_frames);
            report("pages ", sess.pages_programmed);
            bl_ho_exit(HO, &sess);
            hw_soft_reset();
        }
        hw_feedback(bl_sess_blink(&sess, hw_ms()));
    }
}
