# V407 update scope

The declared group profile accepts matching target/revision ANNOUNCE without
SELECT, on marked or unmarked frames. It uses the declared erase/program/END
pauses, checks repeated data, and never replies to status polls. Layout 4 erases
and verifies the metadata sector before any body erase; programming over an
existing descriptor is forbidden. Inactivity is ten seconds without a frame of
the session, excluding the whole declared erase lead (counted from the final
boot ANNOUNCE: the image's erase units x 2 x erase_max + 100 ms). A node that
skips the image (same or older version) stays through its session's DATA
passes and leaves at END. ABORT and inactivity return only to an intact
installed app. Revoked or incomplete images remain in recovery.

SELECT binds status mode for its session only: this UID may answer and reserve
flash work, while another UID excludes this node. The excluded session never
becomes a blind write. The next session's PING or ANNOUNCE ends a selection, and
that session may be a blind one: excluded, not yet announced and FAILED nodes
continue in the declared profile; a COMMITTED node boots its verified image; a
selected transfer that is still running ignores other sessions. Ten seconds
without a frame of the selected session also end it. Session-0 PINGs and other
sessions' DATA, END, ABORT and polls are stale. Reset clears selection. Selected
status replies require the qualified single-receiver marker envelope; unmarked
status polls are silent. There is no downstream status return relay.

The V407 default assembly (variant 0) always has the chain port fitted. V407
LINK is 0x37: RETURN, RELAY, FORWARD, MARKER and UPDATE_FORWARD. On a CHAIN
line the app holds its own slot, forwards the rest (the slot plan, re-armed
after 200 us of quiet LOW after a frame) and relays the answers of the boards
behind it in every window of a valid POLL; a STREAM port forwards nothing. For
updates the app opens its downstream gate fully after two UPD headers; app and
bootloader open it only inside a quiet inter-frame LOW, after 350 us of LOW
with no edge (inside the port gap of at least 500 us, before the next frame).
The app holds forwarding through update entry; the bootloader holds the gate
through flash work and keeps the slot plan in recovery. Reset/unpowered states
close it. Both MCU families' apps and bootloaders forward group updates. A
bootloader relays no answers: its status replies use the marker path only.

Group senders start with at least 1000 ms of nonzero-session UPD PING, then the
entry ANNOUNCE pair, the whole-port entry wait (largest entry bound + largest
forwarding blackout + port processing gap, from the pair's physical end), at
least 1000 ms of training and the repeated boot ANNOUNCE pair before the
declared erase lead. Selected sessions use the same additive rule for their
entry, reboot and acknowledgement waits. The firmware test harness pins the
protocol 0.3 release sender, d5f17d2.

After the last UPD frame, the host holds LOW at least 2100 ms before ordinary
traffic; the app closes at its 2000 ms transparent timeout. After each ANNOUNCE,
transit apps hold forwarding for the sender's upd_transit_hold_ms() of its first
half: the erase units (from its layout and body length) x 2 x 255 ms + 100 ms,
counted from the final half, then close 2000 ms later. A first half that fails a
forwarder check (magic, format, flags, revision order, board 0 / 255, ABI,
feature bits, layout, load, body length) and a second half alone get 114340 ms.
Examples: 62 KiB layout-4 image 8770 ms, full layout-4 slot 63340 ms, full
layout-1 slot 114340 ms. DATA/END/ABORT of that session restore the ordinary
idle timer; the host still holds LOW until the hold + 2100 ms after the last
ANNOUNCE in case a hop missed them. Without an idle-close gap, at most the first
ordinary frame can pass; its decoded trailer closes the gate before the next
frame.

Tests cover production boot transactions/cuts/faults, decoder/main/UART marker
timing, and coupled application-update/GPIO/boot models with both family orders.
The app/boot model boundary is the public UPD_APP_TAKE action; output blackout
and real startup are separate gates. Model evidence does not replace physical
mixed-chain, interrupted-flash, electrical or oscillator qualification.
