# Changelog

A summary of the firmware releases for QuinLED Ultra boards. While below 1.0 versions are numbered 0.x.y. This
repository holds the current release only.

## 0.7.1, 2026-10-27 (source release)

- Ultra protocol 0.3.1-draft (wire VERSION 3, unchanged).
- Boards mix freely on a CHAIN line: the Diff-Power Ultra Receiver Digital-Medium (CH32V203 and CH32V407) works at
  any position, several per line, next to Analog-Medium receivers. It holds its own slot, forwards the rest and
  relays the answers of the boards behind it, once its line rate is measured and the line trained. A board that finds
  the line above still driven at the turnaround neither relays nor answers; PIXEL part 0 counts it (BUSY).
- Every board's Ultra slot plan re-arms only after 200 µs of quiet LOW after a frame (`ULTRA_RW_REARM_US`), never
  before the hub has released the line; the bootloaders do the same. The WS2805 / WS2915 modes keep their frame end.
- AUTO line mode: a detected plain pixel stream replays the incoming waveform whatever the saved pixel count, source
  or stored effects; the decision holds until a reboot or a forced mode.
- Digital-Medium: on a line it has not qualified yet it skips the sampled words without an edge, so it keeps up with
  a frame that follows a configuration.
- A stored pixel calibration stays valid after a reboot up to the board's own pixel limits: 800 RGB / 600 RGBW / 500
  five-channel pixels on the CH32V407 (it was capped at the CH32V203's 600 / 500 / 400).
- Digital-Medium: the pixel calibration's deadline (descriptor `cal_ms`) is the board's own full-length worst case
  plus 15 %: 6,000 ms on the CH32V203 (400 five-channel pixels), 6,500 ms on the CH32V407 (500).

## 0.7.0, 2026-10-24 (source release)

- Ultra protocol 0.3 (wire VERSION 3): per-port line profiles, full TARGET bytes, update ABI 3.
- Three boards: QuinLED Dig2Analog-Plus-5ch, Diff-Power Ultra Receiver Analog-Medium and Digital-Medium; the
  Digital-Medium also on the CH32V407. Board facts and descriptors come from one file per board.
- Bootloaders with protected boot services: blind group updates per chip family and exact target, forwarding through
  every app and bootloader, an optional status profile; a new app confirms itself only after its startup checks.
- Analog effects run on the host (SET with transitions); scenes are the power-on and signal-loss levels.
- Digital-Medium on the CH32V407: 78 more pixel effects (ids 37-114) and 18 more palettes (ids 31-48).
- Every pixel effect takes a palette (catalogue version 4), also the effects that come with their own colours; they
  use the full palette. Effect default keeps each effect's own look, and the chosen palette stays when the effect
  changes.
- Short strips: effects that would flash a strip of a few pixels as a whole run a calm version there.
- Candle is a calm flame that dips now and then, as in a draught; Candle Multi on one pixel is the same flame.
- Test jig mode on the Digital-Medium (both MCUs): the jig pad held low lights each channel in turn on the first 16 pixels; the
  verdict on the status LED and in a `j` console record.
- One shared start lands on the same tick on every board, whatever its boot time: header TIME wraps on a whole ms
  and START counts from the header's host ms (protocol 0.3).
- The lifetime energy merges from the hub again (LIFE_SYNC): each step is bounded by the board's input rating and
  maximum input voltage from its descriptor, exactly; an unknown input merges no energy (the other counters still do).
- Console schema 05: the limits in their own record (`Y`; `y` on the CH32V407 Digital-Medium), the input line's
  statistics (`K`), the second voltage each second, and a full dump at once after a setting changes.
- A Digital-Medium is the last board of its Diff-Power chain, one per port.
- A board in recovery (no usable app) blinks its status LED slowly, 1 s on and 1 s off, while it waits for an
  update; the outputs stay dark.
- Digital-Medium on the CH32V203: smaller effect and record code for more free flash, with the same output.
- Telemetry: the supply impedance is on the ENERGY page on every board. For flash room the Digital-Medium on the
  CH32V203 leaves out the POWER events, the SUPPLY dip log and the DIAG page (its features say so); it still detects
  and counts supply dips, and its console records carry its diagnostics for the test jig.
- Analog-Medium: the current readings, the limits and the over-current trip use the board's own current-sense scale.
- The wiki's Test-Pads page shows the test pads used to program and test a board.
- Own MCU register definitions and startup code, no vendor SDK files; SPDX licence tags on every source file.
- No images or factory files yet: packaging stays off until the bench qualification is done.
- The cross-repo tests run by default: the protocol copies and pins, the vectors, the protocol's receiver simulators
  and the ESPHome receiver contracts against this tree.

## 0.6.12, 2026-10-17

- Digital-Medium receiver: runs as the last board of a normal Ultra line (chain line mode, protocol 0.2.9).
- 36 local pixel effects and 31 palettes; smooth dimming (16-bit gamma and brightness with dithering).

## 0.6.11, 2026-10-17

- Digital-Medium receiver: local pixel effects (26 effects, 20 palettes) from one command; protocol 0.2.8.
- Bootloader hardening, IDENTITY in two parts, the size and cycle gate.

## 0.6.10, 2026-10-15

- Analog-Medium receiver: standby on a hub-switched Diff-Power port, lifetime counters restored from the hub (LIFE_SYNC).

## 0.6.9, 2026-10-15

- Protocol 0.2.5 pages (LIFETIME, DIAG), fault memory, the retry policy after a trip, signal loss "scene".

## 0.6.8, 2026-10-14

- Protocol 0.2.4 return-channel timing; the app write-protects the bootloader at its first boot.

## 0.6.7, 2026-10-14

- Output-only console, a second supported toolchain, fixes to the flash records.

## 0.6.6, 2026-10-14

- The return channel on the Analog-Medium receiver; RAM, flash and CPU optimizations.

## 0.6.5, 2026-10-14

- Settings, scene, calibration and lifetime records in the upper flash, each as an A/B pair; bootloader 2;
  diagnostics and monitoring.

## 0.6.4, 2026-10-14

- Lifetime counters that survive factory resets and updates (0.6.3); flash saves that never disturb the outputs.

## 0.6.2, 2026-10-14

- Long-run safety: every counter wrap and the flash wear checked by a time-warp soak test.

## 0.6.1, 2026-10-13

- Link-time optimization: 4.1 KB more flash headroom, same behaviour.

## 0.6.0, 2026-10-13

- Monitoring (supply dips, temperature peak, channel health), soft start and power-on effects (protocol 0.2.3).

## 0.5.0 - 0.5.5, 2026-10-13

- The Dig2Analog-Plus-5ch rev 0.4, a test-jig mode, the supply voltage in the console, an optimization pass.

## 0.4, 2026-10-13

- The first release: WS2805 / WS2915 emulation and Ultra mode, staggered high-frequency PWM with 16-bit dithering,
  power-on scene, dim-to-warm, current sense and calibration, the data-line bootloader, host tests. Overcurrent and
  power-event reporting (protocol 0.2.2) and the firmware version display (protocol 0.2.1) followed the same day.
