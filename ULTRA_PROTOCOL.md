# QuinLED dig2analog Ultra protocol 0.2 (wire VERSION 2)

**Ultra mode** is the third protocol of the QuinLED dig2analog-ultra-5ch board, next to WS2915 and WS2805
emulation. In Ultra mode the board works as a co-processor of the controller (ESPHome):

- The controller sends **what** should happen: a target state with a transition length, an effect with its
  parameters, or a board setting.
- The board computes **how** it happens, every 1 ms: transitions, gamma, effects, min/max power and the power
  limit. All of it is 16-bit or wider, with no 8-bit stage.

Master header with all constants and builders: [`src/core/ultra_protocol.h`](src/core/ultra_protocol.h).
The ESPHome component keeps an identical copy, and its host test checks that the two match.

## Link and frame

- **Physical layer:** one-way, the same WS28xx pulses as WS2915 (800 kbit/s, MSB first). There is no extra wire.
- **Daisy chain:** each board takes the **first slot** of what it sees. Its hardware gate holds the output LOW for
  those 256 bits, as in WS2805 mode, and then forwards the rest: the slots of the boards after it and the header.
  The position in the chain is the address.

```
SLOT board 0 (32 bytes) | SLOT board 1 (32) | ... | SLOT board N-1 (32) | HEADER (8, a trailer) | reset >= 280 us
```

Frame bits = 256 × N + 64. Every board sees one contiguous frame: its own slot at the start, the header at the end.

**Chain length:** officially up to **50 boards per data pin** (`../research/chain-50-boards.md`): 12,864 bits =
16.7-17.4 ms per frame. The hub sends '0' = 250 ns and '1' = 700 ns high (1.30 / 1.35 µs per bit), centred on the
boards' 472 ns decision. The boards pass bits on without re-timing, so this leaves room for the pulse-width change of
50 forwarding boards.

**Why the header is last.** In format 1 (withdrawn, never released) the header came first and every board
forwarded it, then blocked its slot.
The closed gate left a 320 µs LOW gap after the header on the next board's input, which is longer than a WS reset,
so that board saw a header-only frame and then a frame without a header. Boards after the first never got their
slot together with a header. The firmware's wire-level chain test (`test/test_wire.c`) reproduces this with the
format 1 layout and passes with wire VERSION 2.

**Reading a frame.** The board decodes its slot from the start of the frame (a 256-bit capture, as before). The
header is read at the frame end: when DIN has been quiet for 100-150 µs, the poll ISR copies the last 576 words
(256 µs) of the DIN sample ring up to the last edge, and the main loop decodes the last 64 complete bits
(`ws_decode_last`). The board reads this tail for frames of an Ultra length (256 × n + 64) and of the update length
(384), and for every frame while `auto` mode is not locked yet or only provisionally (from flash).

Those lengths can coincide with WS2805 / WS2915 frame lengths, so a frame counts as Ultra by its **header**, not its
length:

- the header must pass magic, version and CRC-8, and the frame must have an Ultra length;
- in `auto` mode a board locks after **one** frame whose header and its own slot both pass their CRCs (CRC-8 +
  CRC-16, so a WS frame almost never fakes it: ~2⁻⁴⁴ per frame, and its length must fit too). Frames with a valid
  header but no valid slot for this board count like WS frames: 3 in a row. `protocol 3` forces Ultra;
- a frame that ends with a valid Ultra header is never taken as WS, whatever its length.

**Start-up.** A board remembers the protocol it locked last (written to flash only when it changes) and starts with
it after power-up, so it gates and decodes from the first frame. The first frame of another protocol drops it (the
gate closes at once) and the normal lock follows. Before its commands, the controller sends a **preamble of 4N + 2
frames of DARK slots** (N = boards; `ultra_preamble_frames()`), full frames with the header at the end:

- **DARK slot:** 256 zero bits. It is never a valid slot (the CRC-16 of zeros isn't zero), so every board ignores it.
- A fresh board (nothing stored) locks on 3 headers. It forwards with the Ultra plan one frame later, so the next
  board sees Ultra lengths ~4 frames after its predecessor.
- A board that remembers Ultra confirms it with the first frame.
- A board that remembers WS2805 / WS2915 drops it on the first frame (stored protocols wait for the frame end before
  showing anything).
- A board that is still **locked** to WS2805 / WS2915 (an Ultra controller takes over a running chain without a power
  cycle) shows its WS window at once, before it can know. Along a chain the windows of locked WS2805 boards cover
  every bit of a slot, so any 1 bit there would show as a colour on some board. With DARK slots they show dark until
  they lock on the headers. NOP slots (all zero but their CRC) would show the CRC as colours.
- Wire-level chain test, 1-10 boards: every board applies the first command after the preamble, also when every board
  was locked to WS2805 or WS2915. 10 locked boards need 40 frames; 42 are sent. A header-only frame wouldn't work:
  the first board takes the first 256 bits of every frame as its slot.

Idle frames (keep-alive, refresh at rest) use NOP slots.

**Boards that power up while the hub is running** (no preamble, e.g. on their own supply) with a stale stored
WS2805 / WS2915 protocol:
- They show nothing until they lock to Ultra. A stored protocol waits for the frame end, and a frame that ends in a
  valid Ultra header is never WS.
- Until its input is aligned, such a board forwards with its stored WS plan. Those frames are cut at the wrong place,
  but no board shows them.
- Chain test, 1–10 boards, every board stale or a random half (the others running in Ultra), back-to-back frames,
  50 fps and a 200 ms refresh, random power-up moments: 0 frames shown, every board in Ultra. The 10th of 10 stale
  boards shows its slot ~19 frames after power-up, about 4 s at a 200 ms refresh.

**A board forced to WS2805 / WS2915** (`boot_protocol`, config frame, console) treats Ultra frames as WS:
- it shows its WS window of every frame, here the start of its Ultra slot, as a colour;
- it forwards the rest cut at the wrong place, so the boards after it get no valid slot.

The next section brings such boards back.

## Switching boards forced to WS2805 / WS2915

The hub reaches a WS-forced board with the board's own WS config frames: `ultra_switch_frame()` in
`ultra_protocol.h`, sent by ESPHome's `dig2analog_ultra.switch_boards_to_ultra`. It runs only on request.

| Frames | Content |
|---|---|
| 0–3 | WS2805 format: every board's 40-bit slot = the config message "protocol = 3 (Ultra)" (`MAGIC 0xD2A5, CMD 0x01, ARG 3, CRC-8`) |
| 4 | WS2805 format, all zero (the normal frame that ends a config command) |
| 5–8 | WS2915 format: 32-bit header (gains 31), then per board the message and its complement |
| 9 | WS2915 format, all zero |
| then | the start-up preamble (4N + 2 DARK frames) and everything again, with CONFIG 22 = the boot protocol wanted (auto or Ultra) |

- **Taking it:** a board of a frame's format takes the command after 3 identical copies. It switches to forced
  Ultra at once, so it ignores the second format, and blinks its acknowledge pattern. It then follows the preamble.
- **Lengths alternate by 8 bits** (1 or 2 zero bytes of padding).
  - Every board sees a frame minus whole bytes: 40, 80 or 256 bits per board before it.
  - So of two consecutive frames, at most one has a WS2805 or WS2915 length at any board.
  - No board counts 3 frames of one WS protocol in a row, so none re-locks: boards in Ultra mode, and boards in auto
    locked to the other WS format, keep their protocol.
- **Side effect:** a board of the second format (WS2915) shows the first format's config frames as its slot: a colour
  flash of up to 4 frames (a few ms), then the zero frame.
- **Limit:** a board in Ultra mode blocks the first 256 bits of every frame, so the WS boards behind it get the
  frames cut at the wrong place. The same goes for boards behind a board of the other WS format. They stay as they
  are, also when it is sent again or after a power cycle; switch them on their own (put them first on the pin, a WS
  controller, the console).
- **Chain test** (1–10 boards: forced or auto-locked WS2805 / WS2915, forced or auto-locked Ultra):
  - every WS board takes it exactly once, every board applies the first command after the preamble, and no board
    re-locks;
  - WS2805 boards show no colour; WS2915 boards show colour on ≤ 4 frames; Ultra boards ignore it;
  - mixed pins: 0 of 3 WS boards behind an Ultra board are switched, 3 of 4 with a WS2915 board behind three WS2805
    boards.

| Header field | Bytes | |
|---|---|---|
| MAGIC | 2 | `0x5AD2` |
| VERSION << 4 \| FLAGS | 1 | VERSION 2 (the wire format of protocol 0.2); FLAGS 0x01 RW, 0x02 HINT, 0x04 EXT, 0x08 UPD: see [Compatibility rules](#compatibility-rules), [Reserved](#reserved-return-channel) and [Firmware update](#firmware-update-frames) (0 in normal frames) |
| TIME | 4 | host clock in µs (low 32 bits) **at the end of the frame**: the falling edge of the header's last bit |
| CRC-8 | 1 | SMBus (poly 0x07) over the first 7 bytes |

| Slot field | Bytes | |
|---|---|---|
| TYPE | 1 | see below |
| LIGHT << 4 \| FLAGS | 1 | light 0-4 of this board, `0xF` = the board itself (5-14 reserved); FLAGS: SET only, else 0 |
| SEQ | 1 | sequence number per (light, type) |
| PAYLOAD | 27 | big-endian |
| CRC-16 | 2 | CCITT-FALSE over the first 30 bytes |

**SEQ** numbers make every command idempotent. The board applies a (light, type) command once, and ignores
copies with the same SEQ.

- The controller sends every command 3 times, so a corrupted frame loses nothing.
- At rest it keeps repeating the current state (*refresh*), so a board that missed something, or just
  powered up, catches up within a couple of seconds.

## Shared clock

Every header carries the controller's µs clock at the frame end. The controller computes it from the frame's start
and its bits on the wire ('0' and '1' have different lengths). Each board:

- notes its own 50 µs tick at the frame end: the tick that saw the last edges (0-50 µs after the last bit; the
  wire-level chain test measures ≤ 51 µs on every board of a 10-board chain);
- tracks the offset and rate of its HSI clock against the controller's: the first rate measurement after
  ≥ 0.5 s is taken as is, later ones are averaged, and phase errors are corrected by ¼ per header;
- treats a jump of more than 200 ms as a controller reboot: it moves all running times along and restarts its
  SEQ tracking.

SET and EFFECT carry their start time relative to the frame's TIME. The controller fills it in per frame from
one absolute start time (default: 30 ms after the command, `start_delay`), so:

- all copies of a command and all boards start at the same moment;
- a late copy catches up where the others already are;
- effects run in phase on every board.

Measured in the host test, two boards with HSI +2.0 % and −1.7 % stay within 80 µs of each other.

## Slot types

| Type | Name | Payload |
|---|---|---|
| 0 | NOP | — |
| 1 | LIGHT_DEF | roles CH1-CH5 (nibbles, 3 bytes), gamma × 1000 (2), flags (1), min power ×5 (2 each), max power ×5 (2 each) |
| 2 | SET | M (2), A ×5 (2 each), B ×5 (2 each), duration ms (3), start ms (int16) |
| 3 | EFFECT | effect (1), start ms (int32), parameters (≤ 22) |
| 4 | STEP | index << 4 \| count (1), duration ms (2), transition ms (2), M, A ×5, B ×5 (like SET) |
| 5 | CONFIG | up to 9 × (id (1), value (2)) |
| 6 | IDENTIFY | 1 = blink the identify pattern (30 s), 0 = stop; 2 = current calibration (boards with the input current sense: zero, then each channel alone; payload byte 2 = 0: the standard test at 25 / 50 / 75 / 100 %, 0.95 s; 5..100: at that duty only, 0.18 s; visible; stored on the board; other boards ignore it); 3 = DARK: every output fades to dark (0.3 s) and stays dark for payload byte 2 s (0 = 60 s) or until 4 = RELEASE, which fades back; the light engine keeps running underneath, so states and effects continue. The hub calibrates a pin with them: DARK to every board, then one board's test after the other in chain order, then RELEASE. 5 = VERSION (protocol 0.2.1): the board blinks its firmware version once on the feedback channel (3 fast, then major, minor, patch as value codes 1.2 s apart, then alpha 1 / beta 2 / rc 3 short; a development build: one long flash); a board without it ignores the slot, so no blink means older firmware. Values 6-255 and calibration arguments 1-4 / 101-255 are reserved (the slot is ignored) |
| 7 | POLL | *reserved* for the return channel, see [Reserved](#reserved-return-channel); boards ignore it |
| 8 | SCENE | mode (1: 0 not in the scene, 1 these values; 2-255 reserved), fade × 100 ms (1), M (2), A ×5 (2 each), B ×5 (2 each) |
| 9 | LIGHT_OPT | flags (1: 0x01 dim to warm, 0x02 set points), CW LED mireds × 10 (2), WW LED mireds × 10 (2), warm end mireds × 10 (2), red assist (2, 0..65535 = 0..100 %), set points: 4 × (M (2), mireds × 10 (2)), unused = 0 |
| 0x0A-0x7F | — | *reserved* for later per-board commands; boards ignore them |
| 0x80-0xFF | — | *reserved* for [broadcast / extension slots](#broadcast--extension-slots-reserved) only; never applied as a board's own slot |

A slot of all zero bits (DARK, the preamble) is not a type: its CRC fails and every board ignores it.

**Roles:** 0 none, 1 brightness, 2 red, 3 green, 4 blue, 5 white, 6 cold white, 7 warm white; 8-15 reserved (e.g.
amber, UV, lime).

**LIGHT_DEF flags:** 0x01 `zero_means_zero`, 0x02 `constant_brightness`; the other bits reserved.

**SET flags** (low nibble of byte 1): 0x01 smoothstep (ESPHome's 6x⁵ − 15x⁴ + 10x³; otherwise linear),
0x02 via off (a color mode change: fade to 0, switch, fade up); 0x04, 0x08 reserved.

### SET: what the board computes

All values are 16-bit fractions (0..65535 = 0..1), following ESPHome's `LightColorValues`:

- **M** = state × brightness
- **A** = the channel's color (red / green / blue, white, cold white, warm white; 1 for a brightness channel)
- **B** = color brightness on RGB channels, otherwise 1

The board interpolates M, A and B separately, as ESPHome interpolates its color values:

- off → on: the colors jump and only M rises;
- on → off: only M falls;
- via off: as above.

Every 1 ms it computes, per channel:

```
out = gamma(M × A × B)                                        exact power law on the full 48-bit product
CW/WW with constant_brightness:
out = g(M) × max(g(cw), g(ww)) × g(this) / (g(cw) + g(ww))      ESPHome's formula
out = min + out × (max − min)    (0 stays 0 with zero_means_zero)
power limit per channel, then per board (CONFIG 11-21)
```

Boards with the input current sense (next board revision, `HW_ISENSE`; DESIGN_NOTES "Board current sense") hold
CONFIG 21 on the measured input current instead, after rendering; the per-channel limits (16-20) stay estimates
from 11-15. Adaptive stagger then uses the measured channel currents as weights.

The gamma uses fixed-point log2/exp2 tables with interpolation: error under 0.7 counts of 65535 against
`pow()` for gamma 0.45 to 3.0, monotonic. An output that should be on is never rounded to 0.

The outputs go to the PWM dither tables. Every PWM period after an update uses the new value. A 10 s fade
from 0 to 100 % at gamma 2.8 makes 8,721 distinct output steps; the largest is 0.031 %.

### LIGHT_OPT: dim to warm

With the flag set on a light that has cold and warm white channels, the board makes the colour temperature follow
the brightness, every 1 ms. It uses M, so transitions, fades and effects warm up and cool down smoothly on the board.

```
own   = CW mireds + (WW mireds − CW mireds) × g(ww) / (g(cw) + g(ww))   the light's colour temperature (its A values)
target= own + (warm end − own) × (1 − M)                               linear in mireds; never colder than `own`
WW share f = (target − CW) / (WW − CW), at most 1
constant_brightness:  CW = g(M) × max(g(cw), g(ww)) × (1 − f),  WW = ... × f      (CW + WW unchanged)
otherwise:            the brighter of the two stays at g(M) × max(g(cw), g(ww))   (ESPHome's normalisation)
below the WW LED (target > WW mireds): WW only, plus red assist on the light's red channel:
                      red = WW × assist × (target − WW) / (warm end − WW),  green = red / 4,
                      only while the light's RGB part is off
```

At M = 1 the outputs equal those without dim to warm (host test: ≤ 2 counts). The power limit applies after it
and includes the red assist. ESPHome's colour temperature stays the light's temperature at full brightness.

**Set points** (flag 0x02): instead of the straight line to the warm end, up to 4 (M, mireds) points, in any order:
- the target goes linearly in mireds from point to point;
- above the highest point it goes up to `own` at M = 1, unless a point is at M = 1;
- below the lowest point it holds that point's temperature;
- never colder than `own`.

The warm end field carries the warmest point; it scales the red assist. ESPHome's YAML: `dim_to_warm: points:` with
`brightness` and `color_temperature` (`2700 K`, `2700K` or mireds); without a 100 % point HA's colour temperature is
the top of the curve, and in every case the light is never colder than it.

### SCENE: power-on scene

What a light's channels show from the board's power-up until the controller takes over. The board stores it in
flash, in a log separate from the settings, and writes only when the resulting outputs change: 5 s after the last
change, within a write budget that averages at most 96 writes a day.

- **mode** 0: the light's channels are not in the scene (dark at power-up). 1: show M / A / B, interpreted like SET.
- The board computes the outputs with the light's LIGHT_DEF (gamma, constant brightness, min / max power) and the
  power limit (CONFIG 11-21). This is the same code path as rendering, so a controller that restores the same state
  gives identical outputs.
- **fade:** after power-up, the first SET or EFFECT the board receives starts a cross-fade from the scene to the
  rendered outputs over `fade` × 100 ms (smoothstep). With no light command, the cross-fade starts 3 s after the first
  Ultra header. A same-state restore shows no change; a different state fades in.
- Channels of a defined light whose SCENE has not arrived yet keep their stored value. Channels of no defined light
  are dropped.

ESPHome sends it for lights with `power_on_scene: true`: the state its `restore_mode` / `initial_state` will restore
at the next boot. Every other light sends mode 0.

### Effects

| Id | Effect | Parameters | Like ESPHome's |
|---|---|---|---|
| 0 | none | — | (stops the effect at its start time) |
| 1 | pulse | on ms (3), off ms (3), interval ms (3), min (2), max (2) | `pulse` |
| 2 | strobe | step count (1); the steps follow as STEP slots | `strobe` (≤ 8 colors) |
| 3 | flicker | alpha (2), intensity (2), interval ms (2) | `flicker` |
| 4 | random | transition ms (3), interval ms (3) | `random` |
| 5 | candle | intensity (2), speed % (2) | — |
| 6 | breathe | period ms (3), min (2), max (2) | — |
| 7 | color loop | period ms (3), saturation (2) | — |
| 8 | sunrise | duration ms (3) | — |
| 9 | sunset | duration ms (3) | — |

Ids 10-127 are reserved for later effects, 128-255 for custom ones. The payload after an effect's parameters is spare
(0).

How effects interact with the light's state:

- Effects work on top of the light's state. A light that is off stays off.
- An effect switches at its start time, including when it is stopped (`none`).
- If a transition starts at the same moment, which is how ESPHome turns a light off during an effect, the
  transition starts from what the effect showed, as in ESPHome.
- Sunrise ends at the light's state. Sunset ends off, and the light stays off until the next SET.
- Periodic effects keep their exact phase over any uptime: the board moves their start forward by whole
  periods.

### CONFIG ids

Host-managed: the controller sends them at boot and repeats them. They live in RAM only and are never written
to flash. When the board leaves Ultra mode, its stored settings apply again. Exception: ids marked *persisted* are
stored settings; the board writes them to flash only when the value differs from the stored one.

| Id | Setting | Value |
|---|---|---|
| 1 | PWM preset | 0..14 (2.197 kHz .. 100 kHz) |
| 2 | PWM period | 1440..65535 counts of 144 MHz (0 = use the preset) |
| 3 | dithering | 0 on, 1 off at low levels, 2 off |
| 4 | dither threshold | 1..255 = 0.1 .. 25.5 % |
| 5 | dim floor | 0 skip, 1 clamp, 2 off |
| 6 | minimum pulse | 5..100 × 10 ns |
| 7 | stagger | 0 off, 1 fixed (thirds of a period), 2 fixed + spread-spectrum PWM (every period ±6 %), 3 adaptive (phases follow the colour mix for the least LED_+ ripple; CONFIG 11-15 are the weights), 4 adaptive + spread spectrum (the default) |
| 8 | signal loss | 0 hold (effects keep running), 1 off, 2 fade off |
| 9 | signal loss timeout | × 100 ms (default 30 = 3 s) |
| 10 | feedback channel | 0 red, 1 all, 2 off |
| 11-15 | LED current CH1-CH5 at 100 % | mA |
| 16-20 | limit CH1-CH5 | mA, 0 = none |
| 21 | limit for the board | mA, 0 = none. Boards with the input current sense: the **measured** board current is held to it (all channels scaled together, 0 = the board rating), and the estimate from 11-15 is skipped |
| 22 | boot protocol (*persisted*) | 0 auto, 1 WS2915, 2 WS2805, 3 Ultra. Auto and Ultra apply at once; WS2915 / WS2805 from the board's next power-up on (it then ignores Ultra frames) |

Ids 23-127 are reserved for later settings, 128-255 for hardware-specific ones (e.g. a Diff-Power receiver, a
current-sense revision). An unused pair is (0, 0).

## Compatibility rules

Nothing has shipped yet, so these rules are part of protocol 0.2 from the start. **A board ignores what it doesn't know**, so a
later feature never disturbs boards with older firmware. **The hub sends only defined values:** the builders in
`ultra_protocol.h` can't produce anything else, and the ESPHome host test checks every frame the hub builds (also
from 20,000 random calls with undefined inputs).

| What a board gets | What it does | Host test |
|---|---|---|
| Slot TYPE 0x0A-0xFF | ignores the slot | `test_ultra.c` `test_ultra_compat` |
| LIGHT 5-14 | ignores the slot, whatever its type | same |
| CONFIG id 23-255 (or 0) | skips that pair; the slot's other pairs apply | same |
| Effect id 10-255 | runs `none`: the light's plain state (a running effect stops at the start time) | same |
| Role 8-15 | the channel isn't part of the light (as role 0) | same |
| Unknown flag bits: slot FLAGS (other than SET's 0x01 / 0x02), LIGHT_DEF / LIGHT_OPT flags, the reserved nibble after CH5's role | ignored: the engine's state is byte-identical to the same slot without them | same |
| Spare payload bytes: LIGHT_DEF 26, LIGHT_OPT 25-26, SCENE 24-26, IDENTIFY after its argument, EFFECT after its parameters | ignored | same |
| IDENTIFY value 6-255 (5 before 0.2.1), calibration argument 1-4 / 101-255 | ignores the slot | same |
| SCENE mode 2-255 | ignores the slot (the stored scene stays) | same |
| Header FLAGS 0x02 HINT | processes the frame as usual | `test_ultra_compat` (frame_rx), `test_wire.c` |
| Header FLAGS 0x04 EXT | keeps its lock and the clock sync, ignores every slot of the frame (as for RW) | same |
| Another VERSION | rejects the frame: no lock, no clock sync, no slot | `test_ultra_compat` (frame_rx) |

**Rules for later protocol parts:**

- **0 means the old behaviour.** The host sends spare bytes and reserved bits as 0, so a field added in them must mean
  "as before" at 0.
- **Header flags.**
  - **HINT (0x02), optional:** frame-level information a board may use; a board that doesn't know it processes the
    frame as usual.
  - **EXT (0x04), must understand:** the frame uses an extension. A board without it keeps its lock and its clock
    sync but ignores every slot of that frame, as for RW / UPD. The extension says what it is (e.g. in a broadcast
    slot).
  - **New frame-level features go behind EXT** (or RW / UPD), unless boards that don't know them can process the
    frame's slots safely.
  - None of them is used yet; the hub sends 0 in normal frames.
- **`ULTRA_VERSION` changes only for an incompatible change.** A board rejects every frame of another version (no
  lock, no clock, no slot), so a new version means updating every board on the pin. (A bootloader ends its listen
  window at the first normal version-2 header; with another version it waits its full 150 ms.)
- **The host can't ask a board what it supports** ([capability reporting](#capability-reporting-reserved) is
  reserved for QuinLED Diff-Power ultra receivers; the dig2analog-ultra never answers). It relies on the fallbacks above: an unknown command does nothing, an unknown effect shows the plain
  state, an unknown role leaves the channel dark.

**Reserved ranges:**

| Field | Defined | Reserved |
|---|---|---|
| Slot TYPE | 0x00-0x09 | 0x0A-0x7F later per-board commands; 0x80-0xFF broadcast / extension slots only |
| LIGHT | 0-4, 0xF (the board) | 5-14 (e.g. groups, virtual lights) |
| Role | 0-7 | 8-15 (e.g. amber, UV, lime) |
| Effect | 0-9 | 10-127 official, 128-255 custom |
| CONFIG id | 1-22 | 23-127 official, 128-255 hardware-specific (e.g. a Diff-Power receiver, a current-sense revision; 128 = the MCU temperature offset, reserved) |
| Header FLAGS | 0x02 HINT, 0x04 EXT (no hint or extension defined yet); 0x01 RW, 0x08 UPD (reserved below) | — |
| Return-channel page | 0x00-0x07 | 0x08-0x7F official, 0x80-0xFD hardware-specific; 0xFE BUSY, 0xFF UNSUPPORTED |
| Feature bit (IDENTITY) | 0-9 | 10-23 official, 24-31 hardware-specific |

### Broadcast / extension slots (reserved)

```
SLOT 0 | ... | SLOT N-1 | BCAST 0 | ... | BCAST k-1 | HEADER        frame bits = 256 × (N + k) + 64
```

- Every board forwards everything after its own slot, so slots after the last board's slot reach every board.
- They carry types 0x80-0xFF only, in the normal slot format (CRC-16). During the start-up preamble they are DARK, like
  the board slots.
- The length is still an Ultra length and the header still ends the frame, so classification, lock and clock sync
  don't change.
- **Today's boards never read them**, and a board never applies a type ≥ 0x80 as its own slot: e.g. when the hub's
  board count is one too low, the last board takes the first broadcast slot as its own and ignores it.
- **Wire-level chain test** (`test_wire.c`, 1-10 boards, k = 1 and 2; forced Ultra, fresh auto with the preamble, a
  live switch from boards locked to WS2805 / WS2915):
  - every board applies only its own slot;
  - the last board passes the broadcast slots and the header on untouched (480 of 480 frames);
  - with the board count one short, the last board ignores the broadcast slot every frame and its engine's state
    stays unchanged.

**What reading them would cost a later firmware** (estimate, CH32V203; the board reads the header from a copy of the
DIN samples at the frame end):

| | 1 slot | 2 slots |
|---|---|---|
| Sample copy (36 MHz SPI = 2.25 words per µs; 256 bits at up to 1.5 µs each) | +864 words = +1.7 KB | +3.4 KB |
| DIN ring (910 µs) | long enough (window 640 µs, read 100-150 µs after the frame end) | too short (1.02 ms): +1.3 KB, or copy mid-frame by bit position |
| RAM free today (20 KB − 18.7 KB with the stack = 1.3 KB) | short by ~0.4 KB: fits after trimming the slot copy buffer (3.5 KB = 1,792 words; the longest capture is 1,433 words) by ~0.7 KB | doesn't fit |
| The copy at the frame end (today 24.5 µs in the poll ISR) | +27 µs: past one 50 µs tick, so a DMA memory-to-memory copy (a free DMA1 channel, ~0 CPU) | same |
| Decode + CRC in the main loop | ~200 µs per frame (like the own slot) | ~400 µs |
| Wire time on the hub | +340 µs per frame | +680 µs |

- **Cheaper:** decode the slot from the DIN ring while the frame still arrives (no copy, ~0 RAM), with a main-loop
  deadline of ~0.5 ms.
- **Suggested:** one broadcast slot at most, 27 bytes per frame for every board.

### Capability reporting (reserved)

For QuinLED Diff-Power ultra receivers only (the dig2analog-ultra never reports back): the return channel's
IDENTITY page (0x04, `ULTRA_ID_*`, `ULTRA_FEAT_*`) carries the firmware version and a 32-bit feature bitmap:

| Off | Size | Field |
|---|---|---|
| 0 | 12 | MCU unique ID |
| 12 | 4 | firmware version word: major << 24 \| minor << 16 \| patch << 8 \| pre-release (`ultra_update.h`); 0 = unknown |
| 16 | 1 | bootloader version (0 = none) |
| 17 | 1 | hardware id |
| 18 | 2 | hardware capabilities ([research/return-channel.md](../research/return-channel.md) §4.5) |
| 20 | 4 | firmware features, below |

| Bit | Name | Feature |
|---|---|---|
| 0 | SCENE | power-on scene (SCENE) |
| 1 | DIM_TO_WARM | LIGHT_OPT dim to warm with red assist |
| 2 | DTW_POINTS | LIGHT_OPT set points |
| 3 | CURRENT_SENSE | the board current sense in use (CONFIG 21 = the measured current) |
| 4 | CALIBRATION | IDENTIFY 2, pages CALIBRATION / CAL_TEST |
| 5 | DARK_HOLD | IDENTIFY 3 / 4 |
| 6 | FW_UPDATE | UPD sessions (the app behind a bootloader) |
| 7 | BOOT_PROTOCOL | CONFIG 22 |
| 8 | STAGGER_ADAPTIVE | CONFIG 7 values 3 / 4 |
| 9 | MCU_TEMP | STATUS carries the MCU temperature and the 3.3 V rail (below) |
| 10 | VERSION_BLINK | IDENTIFY 5 (protocol 0.2.1) |
| 11-23 | — | later official features (e.g. a broadcast slot, an EXT extension) |
| 24-31 | — | hardware-specific |

- Firmware 0.4's features are 0x3A7 (bits 0, 1, 2, 5, 7, 8, 9), plus bits 3 and 4 with the board current sense and
  bit 6 behind a bootloader; a Diff-Power ultra receiver would report them.
- **Until Diff-Power receivers answer, the host can't read this** (a dig2analog-ultra never answers). It sends what is configured, and a board without a feature
  falls back as above (e.g. firmware without SCENE would show nothing at power-up).

## Reserved: return channel

For **QuinLED Diff-Power ultra receivers only** (their RS-485 pair turns around). The QuinLED dig2analog-ultra-5ch
never reports back, in any revision: its data line is one-way, and what it measures (current, voltage, temperature,
calibration) stays on the board.

Not implemented. These values are reserved so later firmware and hosts can add it without another change to the
frame layout. Constants: `ultra_protocol.h` (`ULTRA_HDR_RW`, `ULTRA_POLL`, `ULTRA_RW_*`, `ULTRA_RSP_*`,
`ULTRA_PAGE_*`). Design: [research/return-channel.md](../research/return-channel.md) §4.

| FLAGS bit | Name | Meaning | What a board of this firmware does |
|---|---|---|---|
| 0x01 | RW | A response window follows this frame; every slot is a POLL | Clock sync from the header; ignores every slot; keeps its lock (an RW frame is a normal Ultra frame for auto-detect) |
| 0x02 | HINT | Optional frame-level hint ([Compatibility rules](#compatibility-rules)); none defined yet | Processes the frame as usual |
| 0x04 | EXT | Must understand: the frame uses an extension; none defined yet | Clock sync from the header; ignores every slot; keeps its lock |
| 0x08 | UPD | Firmware update frame, see [Firmware update frames](#firmware-update-frames) | No clock sync, no slot; keeps its lock |

RW and UPD are never set together: the host never sends that, and a board ignores such a frame (as an UPD frame).

- **Slot type 7, POLL:** valid only in an RW frame. Payload: answer index 0-15 (0xFF = relay only), R = response
  slots in the window (1-16), page, return rate, page arguments. A board of this firmware drops POLL slots in any
  frame.
- **Response window timing** is defined from **t0 = the frame end**: the falling edge of the RW frame's last bit (the
  header's last bit), as each board sees it. With the header as a trailer, t0 is also the moment the board can read
  RW. Its own POLL slot was decoded earlier, from the frame start.

| After t0 | Who | What | Constant |
|---|---|---|---|
| ≤ 150 µs | host / sender | stops driving | `ULTRA_RW_RELEASE_US` |
| 250-900 µs | every board taking part | turns around: forwarding gate held CLOSED (DOUT LOW), upstream port drives | `ULTRA_RW_DIR_MIN_US`, `_MAX_US` |
| s_i = 1000 + 450 × i µs | the board with index i | one 32-byte response slot, UART 8N1 LSB first, 1 Mbaud | `ULTRA_RW_FIRST_US`, `_SLOT_US`, `_BAUD` |
| W = 1000 + 450 × R µs | every board taking part | back to forward mode | |
| ≥ W + 200 µs | host | next frame | `ULTRA_RW_HOLDOFF_US` |

- **Response slot:** SYNC `0xA5` \| PAGE \| INDEX << 4 \| RFLAGS \| SEQ \| UID16 (2) \| DATA (24) \| CRC-16 (2).
- **Pages:** 0x00 STATUS, 0x01 OUTPUTS, 0x02 ACKS, 0x03 ERRORS, 0x04 IDENTITY ([layout](#capability-reporting-reserved)),
  0x05 FW_UPDATE (research/return-channel.md §4.5), and for the current calibration (`research/current-calibration.md`)
  below; 0x08-0x7F reserved for later pages, 0x80-0xFD hardware-specific:

  **0x00 STATUS**, the fields the firmware measures already (console `status`; `ULTRA_ST_*`). A Diff-Power ultra
  receiver reports them; on the dig2analog-ultra they stay local:

  | Off | Size | Field |
  |---|---|---|
  | 9 | 2 | MCU temperature, int16, 0.1 °C; **0x8000 = not valid** (no good conversion yet, or 5 bad ones in a row) |
  | 15 | 2 | the 3.3 V rail (VDDA from VREFINT), mV; 0 = not known |

  - The CH32V203's internal sensor (ADC channel 16) against VREFINT (channel 17), converted once a second and
    filtered (~8 s), with the datasheet's typical V25 = 1.40 V and 4.3 mV/°C (`core/mcutemp.h`).
  - It is the die temperature, a few °C above the board. **The absolute offset varies per chip** (V25 1.34-1.46 V,
    slope 3.8-4.7 mV/°C, VREFINT 1.17-1.23 V: up to ~±20 °C), so it is a trend and over-temperature value. A
    one-point calibration offset is reserved as CONFIG 128 (hardware-specific range, int16 0.1 °C; not settable
    yet, boards skip it). VDDA carries VREFINT's ±2.5 %.
  - Feature bit 9 (MCU_TEMP) says a board fills them.

  **0x06 CALIBRATION** (boards with the input current sense; others answer 0xFF = not supported):

  | Off | Size | Field |
  |---|---|---|
  | 0 | 1 | stored test: 0 none, 1 one level, 4 the standard test (25 / 50 / 75 / 100 %) |
  | 1 | 1 | model now: 0 none, 1 learned, 2 calibrated (and refined since) |
  | 2 | 2 | tests stored on this board (count) |
  | 4 | 3 | age of the stored test, s (0xFFFFFF = from before this power-up) |
  | 7 | 2 | zero (logic current + amplifier offset), mA |
  | 9 | 10 | CH1–CH5 current at 100 % (the model now), mA |
  | 19 | 2 | connected load (sum of the five), mA |
  | 21 | 2 | board limit in effect, mA (0xFFFF = none) |
  | 23 | 1 | its source << 4 \| needed: source 0 board rating, 1 user (CONFIG 21 / stored), 2 PWM preset (C10), 3 fuse; needed 0 load not known, 1 no (the connected load can't reach it), 2 yes |

  **0x07 CAL_TEST:** the stored test in two parts; POLL payload byte 4 = the part.

  | Part | Off | Size | Field |
  |---|---|---|---|
  | 0 | 0 | 24 | CH1, CH2, CH3: the current at each of the four levels, mA (16-bit each, zero subtracted) |
  | 1 | 0 | 16 | CH4, CH5: the same |
  | 1 | 16 | 2 | zero, mA |
  | 1 | 18 | 4 | the four levels' duties, % (one-level test: the duty, then 0) |
  | 1 | 22 | 1 | levels (as CALIBRATION byte 0) |
  | 1 | 23 | 1 | reserved |

  - With the current sense, OUTPUTS 10–19 carry each channel's current from the model at its output (the curve), and
    STATUS 17 the measured total. STATUS 19 gets flag 0x08: the channel currents come from a calibrated model.
  - **Per-channel power** = V+ (STATUS 13) × the OUTPUTS current; while V+ isn't measured, the host uses its configured
    supply voltage.
  - **When to poll:** CALIBRATION after a test (~1 s after the command) and every few minutes; CAL_TEST on demand.

- **Forwarding gate during the window: held CLOSED** (DOUT LOW), not high-Z.
  - On a Diff-Power receiver, DIN sees every answer relayed back.
  - A gate plan re-armed during the window would open mid-window and forward that UART traffic downstream as frames.
  - So the board holds the gate closed from its turn-around until the window ends, then re-arms with the newest plan.
- **Timing reference:** response slots are timed from **the locally measured t0**, the last edge the tail copy finds
  in the DIN ring, not from the shared clock's phase. The shared clock is too coarse (50 µs tick) and lags by a frame.
- **When to poll:**
  - No POLL before the chain is locked (after the start-up DARK preamble), and none during an update session.
  - The answers carry UID16, so the host notices when the indices shifted (a board added, removed or not locked).
- **Decode deadline:** a board must know RW and its POLL by t0 + 900 µs.
  - Its slot is decoded during the frame.
  - The header follows from the tail copy (in the poll ISR, at t0 + 100-150 µs) and its decode (130 µs). That is
    less work after t0 than format 1's whole-capture decode.
  - It is tight anyway: the adaptive stagger search blocks the main loop for up to ~0.5 ms. Firmware with the return
    channel must hold that search back while a frame end is pending (not done yet: a follow-up for that firmware).

## Firmware update frames

Release builds put an 8 KB bootloader in front of the app. The hub sends a signed image to every board of a pin in
one session; Ultra mode only. Formats and constants: `src/core/ultra_update.h` (frozen: every bootloader parses it
for life; the ESPHome component keeps an identical copy). Design and decisions:
[research/firmware-update.md](../research/firmware-update.md). Bootloader: [bootloader/README.md](bootloader/README.md).

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
  - Not taken: the board stays transparent and blinks at END: 1 short = already this version, 1 long (NACK) =
    rejected.
- **After the install:** the new app's first boot blinks ACK + its minor version + 1. It confirms itself (a status
  record in its descriptor page) at the first PING after it has also seen normal Ultra frames. Until then, every
  boot gives the bootloader a listen window (≤ 150 ms, ended by the first normal Ultra frame), so a new session can
  still reach it. An app that ends in 3 watchdog resets in a row falls back to recovery: dark, a slow red blink,
  waiting for a session.

## Versions

Protocol releases are numbered 0.x.y while nothing is released as 1.0; the MINOR number is the wire VERSION
(header byte 2). The master header and the changelog are in intermittech/quinled-ultra-protocol.

- **Format 1** (firmware 0.3, withdrawn, never released, not run on hardware): the header first.
- **Protocol 0.2.0** (wire VERSION 2):
  - the header as a trailer;
  - TIME at the frame end;
  - DARK preamble of full frames;
  - SCENE 7 → 8 and LIGHT_OPT 8 → 9, so POLL can take 7;
  - FLAGS RW / UPD and the return-channel constants reserved.

  Format 1 and VERSION 2 reject each other's headers (version nibble), so the controller and the boards must
  match.
- **Protocol 0.2.1:** IDENTIFY 5, the firmware version as blinks (feature bit 10). Compatible both ways.
- **From VERSION 2 on, the wire VERSION changes only for an incompatible change** (the release then becomes
  0.3.0; [Compatibility rules](#compatibility-rules)).
  Everything else goes into the reserved ranges, behind HINT / EXT, or into broadcast slots.
- **Firmware update frames** use the UPD flag reserved in 0.2.0; no new wire VERSION. Their header VERSION stays 2
  for good, whatever the version of normal frames becomes, because bootloaders are never updated.

## Board resources (CH32V203, 144 MHz)

| | |
|---|---|
| Decoding the board's slot (256 bits, from the frame start) | 192 µs |
| Reading the header trailer: tail copy in the poll ISR / decode of the last 64 bits | 25 µs / 130 µs |
| SET slot (CRC + new ramp) | 10 µs |
| Clock sync per header | 2 µs |
| Engine per 1 ms update | 15-43 µs typical (21 µs RGBWW + dim to warm with red assist), 69 µs worst measured (5 lights with an effect each) |
| Flash / RAM | 40.3 KB / 18.7 KB of 61.5 / 20 KB (whole firmware, including 2 KB stack and the 1.1 KB tail buffer; 43.9 KB flash with the board current sense) |

Measured with `tools/measure.py`, which runs the compiled functions in an RV32IMC emulator.
