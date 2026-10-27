/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* QuinLED Ultra bootloader (wiki: Updating over the data line; README.md here).
 *
 * Every reset starts here, at HSI 8 MHz:
 *   fast path: start IWDG, descriptor/status + option-byte protection check -> confirmed app: jump at HSI;
 *              DIN high for 20 ms on power-on requests a 3 s session window (low has no qualification wait);
 *   full mode: 144 MHz, DIN sampler, gate, watchdog: verify a fresh image, an unconfirmed app's trial window, a
 *              session the app asked for, or recovery (the status LED blinks slowly, the outputs stay dark).
 *              It always leaves through a software reset.
 *   first provisioning: protect sectors 0-1, reset and read back all option bytes before running an app.
 * No interrupts are used. */
#include "../src/mcu/device.h"
#include "bl_core.h"
#include "bl_hw.h"
#include "bl_rx.h"

#define HO ((upd_handover_t *)UPD_HANDOVER_ADDR)
#define GAP_TICKS     (ULTRA_RW_REARM_US * HW_TICKS_US)   /* DIN quiet this long after a frame: re-arm the slot plan */
#define STALL_TICKS   (800u * HW_TICKS_US)        /* the ring holds 0.91 ms: after a longer stall, resync */

static bl_sess_t sess __attribute__((section(".noinit")));
static bl_rx_t rx __attribute__((section(".noinit")));
static bl_frame_t frame __attribute__((section(".noinit")));

/* bl_reset_kind (bl_core.c, host-tested) reads the RSTSCKR flags by its own names: they must be the chip's */
typedef char bl_rcc_flags_match[(BL_RCC_PINRSTF == RCC_PINRSTF && BL_RCC_PORRSTF == RCC_PORRSTF &&
                                 BL_RCC_SFTRSTF == RCC_SFTRSTF && BL_RCC_IWDGRSTF == RCC_IWDGRSTF &&
                                 BL_RCC_WWDGRSTF == RCC_WWDGRSTF && BL_RCC_LPWRRSTF == RCC_LPWRRSTF) ? 1 : -1];

int main(void)
{
    static const bl_rx_timing_t timing = BL_RX_TIMING;
    bl_app_t app;
    bl_action_t act;
    uint32_t flags = hw_reset_flags_take(), reset, rd = 0, last_feed, gate_tick, now, ms;
    uint16_t gate_cnt = 0;
    uint8_t shut = 1;                         /* FWD_EN held LOW since reset, until the first quiet LOW */

#if BL_HW_ID == 3
    hw_blank_early();                         /* (before anything else: bl_hw.c) */
#endif
    reset = bl_reset_kind(flags);             /* software / watchdog first: they may raise PINRSTF as well */
    hw_crc_on();
    act = bl_boot(&app, HO, reset, flags);
    if (act == BL_ACT_RESET) {
        hw_soft_reset();
    }
    if (act == BL_ACT_JUMP) {
        hw_jump_app();
    }

    hw_full_init();
    if (act == BL_ACT_VERIFY) {
        act = bl_after_verify(&app);
        if (act == BL_ACT_JUMP) {
            HO->request = UPD_REQ_BOOT_ONCE;
            bl_ho_seal(HO);
            hw_soft_reset();
        }
    }
    bl_sess_init(&sess, act, &app, hw_ms());
    bl_rx_init(&rx, &timing);
    rd = hw_ring_pos();
    last_feed = gate_tick = hw_ticks();

    for (;;) {
        uint32_t wr, n;
        /* DIN before the edge count (and the counter's input delay): a LOW read here, with no edge counted after it,
         * was LOW since gate_tick; read after the count, a fall in between would pass a held HIGH as quiet */
        const int din_high = bl_plat_din_high();
        hw_kick();
        now = hw_ticks();

        /* gate: the frame end in real time from the edge counter; plan for the next frame from the decoder */
        {
            uint16_t c = hw_edges();
            /* Full forwarding opens only after MCU_BL_FWD_QUIET_US of LOW without an edge (the time read after the
             * count: never longer than the line was quiet), DIN LOW: the first opening, and every later return to
             * it after a frame the slot plan held (FWD_EN LOW). The slot plan re-arms after GAP_TICKS of quiet LOW
             * after a frame (past the hub's release). Nothing writes the gate while DIN is HIGH: a released or held
             * HIGH is never cut or started late. */
            if (c != gate_cnt) {
                gate_cnt = c;
                gate_tick = hw_ticks();
            } else if (!din_high && (shut || c != 0u) &&
                       now - gate_tick >= (shut || sess.gate != BL_GATE_ULTRA ? MCU_BL_FWD_QUIET_US * HW_TICKS_US
                                                                               : GAP_TICKS)) {
                shut = 0;
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
#if UPD_STATUS_PROFILE
        now = hw_ticks();                          /* (the samples up to wr ended about now) */
#endif
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
#if UPD_STATUS_PROFILE
                /* Group transfers accept CHAIN frames; the selected status
                 * endpoint still has only a qualified marker reply path. */
                if (!frame.marker && frame.tail[0] == UPD_STATUS) frame.bad = 1;
                /* The saved frame edge remains valid if the release rose in
                 * this feed; rx.run would then describe the released HIGH. */
                sess.st.frame_t0 = now - (((wr - rd) & (HW_RING_WORDS - 1u)) * 16u + frame.end_ago) * MCU_BL_TICKS_PER_SAMPLE;
#endif
                bl_sess_frame(&sess, frame.bad ? 0u : frame.nbits, frame.tail, hw_ms());
#if UPD_STATUS_PROFILE
                if (sess.st.reply) {
                    sess.st.reply = 0;
                    if (hw_reply(sess.st.answer, sess.st.reply_at, sess.st.brr, sess.st.frame_t0))
                        sess.st.op_due = 0;        /* unsent BUSY: require a fresh poll before flash work */
                    hw_flash_stall = 1;            /* (the ring ran on unread meanwhile: resync) */
                }
#endif
                if (!rx.locked_period && !frame.bad &&
                    (sess.state == BL_S_ERASE || sess.state == BL_S_RECEIVE))
                    rx.locked_period = frame.period;
            }
        }

        ms = hw_ms();
        /* recovery (no usable app) while no update runs: the status LED blinks slowly, 1 s on / 1 s off, from the
         * session's ms time base; otherwise it stays off. Only the LED: the outputs stay dark */
        hw_led(sess.mode == BL_ACT_RECOVERY && (sess.state == BL_S_LISTEN || sess.state == BL_S_FAILED) &&
               !(ms / 1000u & 1u));
        if (bl_sess_poll(&sess, ms)) {
            bl_ho_exit(HO, &sess);
            hw_soft_reset();
        }
        if (sess.state == BL_S_FAILED) rx.locked_period = 0;
    }
}
