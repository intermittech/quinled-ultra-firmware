# The QuinLED Ultra protocol in this firmware

**The specification** is the QuinLED Ultra protocol repository, intermittech/quinled-ultra-protocol: the header
`include/ultra_protocol.h` with every constant and builder, `CHANGELOG.md`, the test vectors, and its wiki (frame
layout, slot types, effects, CONFIG ids, the shared clock, the compatibility rules, the return channel, host
behaviour). `src/core/ultra_protocol.h` here is a byte-identical copy (protocol 0.2.9, wire VERSION 2), and so is the
ESPHome component's: its host test checks the two match. This file keeps only what is this firmware's own: what it
implements, how it renders, and its firmware update frames.

## What it implements

- **QuinLED Dig2Analog-Ultra-5ch** (`BOARD=1`, the default): Ultra mode next to the WS2915 and WS2805 emulation (CONFIG
  22 picks the boot protocol; WS config frames switch a board forced to a WS protocol: the wiki, Host behaviour). Every
  slot type, effect and CONFIG id (the wiki's Config ids are this board's), the power-on scene and effects,
  signal-loss mode 3, firmware updates behind the bootloader (below). No return channel: it ignores the slots
  of RW frames, so it never answers a POLL, and it ignores IDENTIFY 7 (the planned power-off notice).
- **QuinLED Diff-Power Ultra-Analog receiver** (`BOARD=2`): all of the above and the return channel: pages
  STATUS, OUTPUTS, ACKS, ERRORS, IDENTITY, CALIBRATION, CAL_TEST, POWER, SUPPLY, HEALTH, ENERGY, LIFETIME and DIAG,
  LIFE_SYNC (0x0E: the receiver's steps and the counters' 32-bit room, wiki: Return channel); IDENTIFY
  7 (standby). IDENTITY (two parts): part 0 hardware id 2, the revision, capabilities 0x0000000F
  (answers, relays, bus voltage, input current); part 1 features word 0 0x007FFFFF behind the bootloader (bits 0-22;
  0x007FFFBF without bit 6, the update, in a standalone build), words 1-5 0. STATUS 3: the hardware id.
- **QuinLED Diff-Power Ultra-Digital receiver** (`BOARD=3`, in development): a pixel port's delay line, polled
  behind a marker with the release-keyed window (the protocol wiki: Return channel, digital ports). The Ultra-Analog
  receiver's pages it can fill, + 0x80 PIXEL; IDENTIFY 7. IDENTITY part 0: hardware id 3, capabilities 0x1CD
  (answers, bus voltage, input current, PIXEL, local effects, chain mode; no relay), its features: [docs/ultradigital-firmware.md](docs/ultradigital-firmware.md)
  section 8.

## How it renders a SET (beyond the specification)

- Every value is a 16-bit fraction (0..65535 = 0..1) as ESPHome's `LightColorValues`; the board interpolates M (state
  x brightness), A (the channel's color) and B (the color brightness) separately, as ESPHome does, and renders every
  1 ms: gamma on the full 48-bit product, constant brightness, min / max, the per-channel power limits.
- The gamma: fixed-point log2 / exp2 polynomials (`tools/gen_ultra_tables.py`), under 0.7 counts of 65535 from
  `pow()` for gamma 0.45-3.0, monotonic; an output that should be on is never rounded to 0.
- The outputs go to the PWM dither tables; every PWM period after an update uses the new value (a 10 s fade from 0 to
  100 % at gamma 2.8: 8,721 distinct steps, the largest 0.031 %).
- The board limit (CONFIG 21) holds on the measured input current (the board current sense): after rendering, all
  channels are scaled by one factor. The adaptive stagger weighs the measured channel currents.
- Its timing and size, measured on the compiled firmware (`tools/measure.py`, an RV32IMC emulator): README, "Compiled
  code, measured".

## Firmware update frames

Release builds put an 8 KB bootloader in front of the app. The hub sends a signed image to every board of a pin in
one session; Ultra mode only. Formats and constants: `src/core/ultra_update.h` (frozen: every bootloader parses it
for life; the ESPHome component keeps an identical copy). How to use it: wiki, Updating over the data line.
Bootloader: [bootloader/README.md](bootloader/README.md).

**Frame:** BLOCK (40 bytes) + HEADER (8, VERSION 2, FLAGS = UPD) = **384 bits**, a length no other frame uses. It
is sent like any Ultra frame. 384 is not an Ultra length, so a board classifies it as "odd" and keeps its lock.

| Bytes | Field | |
|---|---|---|
| 0 | TYPE | 0x81 ANNOUNCE, 0x82 DATA, 0x83 END, 0x84 ABORT, 0x85 PING |
| 1 | SESSION | the same in every frame of one session |
| 2-3 | INDEX | ANNOUNCE: descriptor half 0 / 1 (bytes 0-31 / 32-63); DATA: 32-byte chunk number of the body |
| 4 | PASS | pass << 4 \| passes (information only) |
| 5 | FLAGS | ANNOUNCE: 0x01 FORCE (also the same or an older version) |
| 6-37 | DATA | 32 bytes; END: body CRC-32 and body length |
| 38-39 | CRC-16 | CCITT-FALSE over bytes 0-37 |

**Image** = a 64-byte descriptor (magic `QDU5`, hardware id, board-revision mask, version, body length and CRC-32,
oldest bootloader that can install it, build id, CRC-32) + the body: the app linked at 0x0800_2100. The hub checks
the image's ECDSA P-256 signature before it sends anything; the boards check the CRCs.

**Hardware ids and board revisions** (`ultra_update.h`). An image installs only on the hardware id and the
revisions in its mask: the app and the bootloader both check it, and the hub picks, per pin, the image whose mask
holds the line's `board_revision`.

| Hardware id | Board | Revisions (bit of the mask) |
|---|---|---|
| 1 | QuinLED Dig2Analog-Ultra-5ch | 3 = rev 0.4 (`UPD_REV_D2A_R04`, mask 0x8). 0-2 stay unused: drawings that were never built |
| 2 | QuinLED Diff-Power Ultra-Analog receiver | reserved, no hardware yet |

**Session, as the hub sends it** (`update_session.h` in the ESPHome component):
1. Lead-in: ANNOUNCE halves, alternating, for 500 ms + 15 ms per board.
2. Passes (3 by default): every 256-byte flash page as 8 DATA chunks, then a 2.7 ms pause while the boards program
   the page; odd passes run backwards; both ANNOUNCE halves again every 4 pages (boards that joined late).
3. END, repeated for 250 ms: the bootloaders that have the whole image commit it and reset into it.

PING frames are sent outside sessions (after one, and when the line resyncs).

**What a board does:**
- **App, every 384-bit frame:** reads its tail. After 2 UPD headers in a row the board goes **transparent**: its gate
  forwards whole frames and it captures whole frames. A board sees whole UPD frames only once the board before it is
  transparent, so the chain joins one board after the other (the wire-level test: ~5 frames per board; the lead-in
  allows ~80). Lights keep running, and UPD frames count as signal. Back to normal at the first normal Ultra header,
  or 2 s after the last UPD frame.
- **ANNOUNCE (both halves):** the app makes the bootloader's checks: hardware id, revision mask, bootloader version,
  and a newer version unless FORCE.
  - Taken: it leaves a request in a RAM hand-over block and resets into the bootloader, which installs the image
    during the session. Its gate forwards everything meanwhile.
  - Not taken: the board stays transparent and blinks at END, on its status LED only (the outputs keep
    their light, so a repeated session doesn't flash the strips): 1 short = already this version, 1 long
    (NACK) = rejected.
- **After the install:** the new app's first boot blinks ACK + its minor version + 1. It confirms itself (a status
  record in its descriptor page) on whichever comes first:
  - the first PING after it has also seen normal Ultra frames since boot;
  - without a PING (a host that sends none: WLED, the sender library): once it has run 60 s (`UPD_APP_AUTO_S`) with
    at least 100 frames of any protocol decoded (`UPD_APP_AUTO_FRAMES`), if this boot wasn't a watchdog reset.

  Until it is confirmed, every boot gives the bootloader a listen window (≤ 150 ms, ended by the first normal Ultra
  frame), so a new session can still reach it; a confirmed app's boots have none. An app that ends in 3 watchdog
  resets in a row falls back to recovery: dark, a slow red blink, waiting for a session.
