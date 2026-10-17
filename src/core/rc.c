/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Diff-Power ultra receiver's return channel: see rc.h. */
#include "rc.h"
#include "config.h"                        /* crc16_ccitt: the one copy of ultra_crc16 */
#include "lto.h"
#include "../hal/board.h"

/* tools: measured by name in the receiver's measurement build (the dig2analog-ultra's images stay as they were) */
#if BOARD_RETURN
#define RC_MEASURED MEASURED
#else
#define RC_MEASURED
#endif

RC_MEASURED int rc_poll_parse(const uint8_t *s)
{
    return ultra_poll_valid(s);            /* (round 11, F1: the header's, one copy of the rule; slot FLAGS: unknown bits
                                              are ignored, as in every slot) */
}

RC_MEASURED int32_t rc_last_edge(const uint16_t *w, uint32_t words)
{
    uint32_t k = words, v, b;
    while (k-- != 0u) {
        v = w[k];
        if (v != 0u) {
            for (b = 0; !(v & 1u); b++) {      /* the last high sample: the lowest set bit (MSB = earliest) */
                v >>= 1;
            }
            return (int32_t)((words - 1u - k) * 16u + b);   /* its sample's end is the falling edge */
        }
    }
    return -1;
}

RC_MEASURED void rc_schedule(rc_sched_t *s, uint32_t edge_tick, int32_t back_samples, uint32_t rate_q16,
                             const uint8_t *poll)
{
    /* an event fires at the first tick T with (T - edge_tick) >= its offset from t0 - t0's distance back. inv = local
     * ticks per host us, Q32 (one division); offsets <= 23.6 ms: the truncation is < 1e-5 tick. Samples -> ticks at
     * the nominal 36 MHz: the HSI's error over <= 256 us is < 0.15 us. */
    uint32_t inv = (uint32_t)((1ull << 48) / rate_q16), pitch = rc_slot_us(poll[RC_P_RATE]);
    uint32_t back = (uint32_t)(back_samples * 65536 / (int32_t)RC_SAMPLES_TICK);
    uint32_t idx = poll[RC_P_INDEX] == ULTRA_POLL_RELAY_ONLY ? 0u : poll[RC_P_INDEX];
#define RC_AT(us) (edge_tick + (((uint32_t)(((uint64_t)(us) * inv) >> 16) - back + 0xFFFFu) >> 16))
    s->dir_tick = RC_AT(RC_DIR_US);
    s->check_tick = RC_AT(RC_CHECK_US);
    s->late_tick = RC_AT(ULTRA_RW_DIR_MAX_US);
    s->tx_tick = RC_AT(ULTRA_RW_FIRST_US + pitch * idx);
    s->end_tick = RC_AT(ULTRA_RW_FIRST_US + pitch * poll[RC_P_SLOTS] + RC_END_US);
#undef RC_AT
}

RC_MEASURED uint16_t rc_brr(uint32_t rate_q16, uint8_t rate)
{
    /* PCLK1 runs on the HSI as the tick does: a tick of rate_q16 / 65536 host us instead of 50 means the clock is
     * 50 x 65536 / rate_q16 times the nominal; the divider for 1 Mbaud at 144 MHz is 144 (one step = 0.7 %: the
     * residual <= 0.35 %; 500 / 250 kbaud: 288 / 576, <= 0.17 / 0.09 %, each at its own resolution: review r6, 0.6.6
     * rounded at 1 Mbaud's and shifted, 0.35 % at every rate, 4.6 us over a 250 kbaud burst) */
    uint32_t nom = 144u << rate, brr = (nom * 3276800u + rate_q16 / 2u) / rate_q16;   /* (576 x 3276800 < 2^31) */
    if (brr < nom - nom / 18u || brr > nom + nom / 18u) {
        brr = nom;                         /* not plausible (> 5.5 % off): nominal */
    }
    return (uint16_t)brr;
}

void rc_pack(uint8_t *d, const uint32_t *v, const uint8_t *len, uint32_t k)
{
    uint32_t i, n, x;
    for (i = 0; i < k; i++) {
        x = v[i];
        for (n = len[i]; n-- != 0u;) {
            d[n] = (uint8_t)x;
            x >>= 8;
        }
        d += len[i];
    }
}

RC_MEASURED void rc_answer(uint8_t a[RC_ANSWER_BYTES], const uint8_t *poll, uint8_t page, uint8_t rflags,
                           uint16_t uid16)
{
    a[0] = ULTRA_RSP_SYNC;
    a[1] = page;
    a[2] = (uint8_t)((poll[RC_P_INDEX] << 4) | (rflags & 0x0Fu));
    a[3] = poll[2];                        /* the POLL's SEQ */
    ultra_put16(a + 4, uid16);
    ultra_put16(a + 30, crc16_ccitt(a, 30));
}
