/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* A board's own descriptor (src/board/<board>/descriptor_gen.h, copied in as desc.h; MCU_CH32V40X picks the V407
 * Digital-M's) through the pinned protocol's sender decoder: it decodes, and its LINK flags are what the board says
 * (test/run_m1a_vectors.py builds one per board and MCU; WANT_LINK_FLAGS, if set: the expected LINK 0 flags); and the
 * LIFE_SYNC energy step the board merges (BOARD_IN_WH_NUM / BOARD_IN_WH_DEN, core/energy.c's arithmetic) is the
 * sender's allowance from that descriptor's domain 0 for every powered step (0.1 h, 0..30 days). */
#include <stdio.h>
#include "ultra_sender_descriptor.h"
#include "desc.h"

int main(void)
{
    ultra_descriptor_t d;
    ultra_desc_link_t l;
    ultra_desc_domain_t dom;
    uint32_t dt, bad = 0, ref = 0;
    int rc = ultra_descriptor_decode(&d, board_descriptor, (uint16_t)BOARD_DESCRIPTOR_BYTES);
    int lk = rc == 0 ? ultra_descriptor_link(&d, 0u, &l) : -1;
    int dm = rc == 0 ? ultra_descriptor_domain(&d, 0u, &dom) : -1;
#ifdef WANT_LINK_FLAGS
    int ok = rc == 0 && lk == 0 && l.flags == WANT_LINK_FLAGS;
#else
    int ok = rc == 0 && lk == 0;
#endif
    for (dt = 0; dm == 0 && dt <= 7200u; dt++) {
        ref = ultra_lifetime_energy_allowance(dom.rating_ma, dom.vmax_10mv, dt, 0xFFFFFFFFu);
        bad += (BOARD_IN_WH_NUM * (dt + 10u) + BOARD_IN_WH_DEN - 1u) / BOARD_IN_WH_DEN != ref;
    }
    ok = ok && dm == 0 && bad == 0 && BOARD_IN_WH_NUM != 0u;
    printf("board %u descriptor (%u B): decode %d, LINK 0 flags %u; LIFE_SYNC energy step %u/%u Wh per 0.1 h = the "
           "sender's allowance (%u mA x %u x 10 mV: %u Wh at 30 days), %u differ: %s\n", (unsigned)board_descriptor[6],
           (unsigned)BOARD_DESCRIPTOR_BYTES, rc, lk == 0 ? (unsigned)l.flags : 0u, (unsigned)BOARD_IN_WH_NUM,
           (unsigned)BOARD_IN_WH_DEN, dm == 0 ? dom.rating_ma : 0u, dm == 0 ? dom.vmax_10mv : 0u, (unsigned)ref,
           (unsigned)bad, ok ? "ok" : "FAIL");
    return ok ? 0 : 1;
}
