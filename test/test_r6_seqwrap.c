/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Included by test_soak.c.
 * r6seqwrap  Signal loss "hold" (the default), the data cable out for 5 s while the hub streams an ESPHome effect to
 *            the light (a step a frame, each a SET with the next 8-bit SEQ, ultra_core.h light_stream); the effect stops
 *            meanwhile, and the hub's last SET (the light's state) carries after a multiple of 256 steps the SEQ this
 *            board applied last. When the cable is back, a board that takes that SET (and every refresh of it) as a
 *            repeat keeps the old state until the light changes again (1 in 256 per such gap; off / fade off reset the
 *            SEQs already). Expected: the board shows the hub's state. */
static void sc_r6seqwrap(void)
{
    uint8_t s[32];
    uint16_t a[5] = { 65535, 65535, 65535, 65535, 65535 }, b[5] = { 65535, 65535, 65535, 65535, 65535 };
    uint32_t k;
    printf("soak r6seqwrap: hold, a 5 s gap in which the light's SET SEQ goes round by 256\n");
    boot(0, 0, 0);
    host_on = 1;
    cmd_def(0, 0, 1, 1000);                              /* CH1, gamma 1: out = M */
    ultra_slot_set(s, 0, 10, 0, 20000, a, b, 0, 0);      /* the stream's step the board applies last: SEQ 10 */
    host_send(s);
    run_ms(2000);
    CHECK(out_now[0] == 20000u, "r6seqwrap: the light at the stream's step (%u)", out_now[0]);
    host_on = 0;                                         /* the cable out: 256 more steps (SEQ 11 .. 265), unseen */
    run_ms(5000);
    CHECK(!signal_lost && out_now[0] == 20000u, "r6seqwrap: hold: the light holds through the gap (%u)", out_now[0]);
    host_on = 1;
    host_next = sim_ticks;
    k = 10u + 256u;                                      /* the state after the effect: SEQ 266, on the wire 10 */
    ultra_slot_set(s, 0, (uint8_t)(k & 0xFFu), 0, 50000, a, b, 0, 0);
    for (k = 0; k < 10u; k++) {
        host_send(s);                                    /* its copies and refreshes */
    }
    run_ms(3000);
    CHECK(out_now[0] == 50000u, "r6seqwrap: the hub's state after the gap (%u; 20000: the 8-bit SEQ taken as a "
          "repeat of the one it applied last)", out_now[0]);
    CHECK(events_seen == 0u, "r6seqwrap: no false events (%X)", events_seen);
}
