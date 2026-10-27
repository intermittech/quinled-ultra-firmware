# CRC-driven lower-rate indication

This firmware implements a controller-facing proposal on QuinLED Ultra
`LINK_STATS` page `0x13`: DATA byte 2 bit `0x10` is `RATE_REQUEST`.
Its allocation in the protocol and the controllers' support are still open.
It means “this link may need a lower rate”; it does not diagnose a cable or
change the active profile.

A window with at least four failed header/slot validations counts as bad;
an undecodable header also fails validation. Three
consecutive one-second windows latch the indication. Missing an entire window
breaks the streak; delayed calls retain the original window boundaries. The
millisecond counter may wrap. A bad slot CRC counts even with a valid trailer;
the intentional all-zero DARK preamble does not count as a bad slot.

The flag appears in both parts of one frozen snapshot. Clearing reported
frame/error counters does not clear the latch. A new acquisition epoch clears
it. Isolated failures and two bad windows do not request a change.

A controller should read both parts, verify their common epoch, freshness and
advertised profile support, and decide whether to retrain at a supported lower
rate. It must handle the whole affected chain coherently. The receiver never
changes the wire rate on its own, and the existing return-window qualification
still applies. A header-only tentative lock cannot answer this or any poll.

Follow-up: agree the flag allocation and semantics in the protocol,
then add controller/ESPHome/HA reporting and retraining policy. This work does
not remove any effect, so the effect catalogue and the capability maps stay as they are.
Firmware regression coverage is in `test/test_line_state.c`.
