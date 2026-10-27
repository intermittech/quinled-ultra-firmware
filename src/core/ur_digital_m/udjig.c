/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Digital-M's test jig mode: see udjig.h. */
#include <stddef.h>
#include "udjig.h"

typedef char uj_level_shift[UJ_LEVEL == 1u << UJ_LEVEL_SHIFT ? 1 : -1];
typedef char uj_layout[offsetof(udjig_t, run) == 0u && offsetof(udjig_t, on) == 1u && offsetof(udjig_t, say) == 4u &&
                       offsetof(udjig_t, ph) == 6u ? 1 : -1];   /* (udjig.h: no odd merged halfword store) */

void uj_ms(udjig_t *j, uint32_t now, uint32_t pad_low, int32_t i_ma, uint32_t live, uint32_t bpp)
{
    const uint32_t ph = j->ph, x = ph % UJ_SLOT_MS, s = ph / UJ_SLOT_MS;
    uint32_t i;
    int32_t m;
    if ((pad_low != 0u) == j->on) {
        j->run = 0;
    } else if (++j->run >= UJ_DEBOUNCE_MS) {
        j->on ^= 1u;                                     /* (on: a new cycle from the zero; off: the LED's state forgotten) */
        j->run = 0;
        j->ph = 0;
        j->cycles = 0;
        j->say = 1;
        return;
    }
    j->led = (uint8_t)(j->cycles ? !j->fail || ((now / 100u) & 1u)   /* the last verdict: PASS steady, FAIL 5 Hz */
                                 : s != 0u);             /* no verdict yet: off in each zero slot */
    if (!j->on || !live) {
        return;
    }
    j->ph = (uint16_t)(ph + 1u == 6u * UJ_SLOT_MS ? 0u : ph + 1u);
    if (x != UJ_SLOT_MS - 1u) {
        return;
    }
    m = i_ma;                                            /* (the slot's last ms: the board's average has settled) */
    if (s == 0u) {
        j->zero = m;
        return;
    }
    j->ma[s - 1u] = m - j->zero;
    if (s < 5u) {
        return;
    }
    j->cycles++;
    j->fail = 0;
    for (i = 0; i < bpp && i < 5u; i++) {
        m = j->ma[i];
        j->fail |= (uint8_t)((m < (int32_t)UJ_PASS_MIN_MA || m > (int32_t)UJ_PASS_MAX_MA) << i);
    }
    j->rep = 1;
}

uint32_t uj_frame(const udjig_t *j, const uint8_t *ord, uint32_t bpp, uint8_t *fb)
{
    const uint32_t ch = j->on ? j->ph / UJ_SLOT_MS - 1u : 5u;   /* (the zero slot: ~0, no channel) */
    uint32_t i, k;
    for (i = 0; i < UJ_PIXELS; i++) {
        for (k = 0; k < bpp; k++) {
            *fb++ = (uint8_t)((ord[k] == ch) << UJ_LEVEL_SHIFT);
        }
    }
    return UJ_PIXELS * bpp;
}
