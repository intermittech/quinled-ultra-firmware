# Console schema 05

Firmware 0.7.1, protocol 0.3 (wire 3).
The output-only default console uses ASCII on jig pad, 115200 8N1. Every second it
emits R (analog) or r (digital). It starts a full dump after the first valid
measurements, then every 60 seconds, and at once when a setting changes: X 9
(CONFIG_CHANGED) then the dump (settings, CONFIG, scenes, light definitions; the
fingerprint is checked once a second). Each dump is split into bounded records;
settings and counters are read in main context, one record per pass. A dump is
not an atomic snapshot. Each record's fields are sampled together; interrupt
counters use naturally aligned volatile loads. No console work runs in an ISR.
Normal diagnostics pause while the production jig runs (its X records and results only).

`<record>05,<index>,<hex value>,...*<CRC8>\r\n`

Integers are uppercase hexadecimal, without prefixes or leading zeroes. Signed
fields use two's complement at the width in the schema. The index is hex too.
CRC-8/SMBUS: polynomial 07, init 00, no reflection, xorout 00, over all ASCII
bytes before `*`, including the type and schema. The checksum is exactly two
hex digits. Maximum record length is 240 bytes including CRLF. Reject incomplete,
overlong, bad-checksum, unknown-schema and wrong-field-count records; resynchronize
at CRLF. There are no explanatory strings or units in default diagnostics.

The sole field table is [console_schema.json](../tools/console_schema.json).
It holds record IDs, accessor order, widths, signedness, names, units/scales,
meanings and jig explanation text. `gen_console_schema.py --check` runs in every
target build. Generated C tables use offsetof and compile-time width checks;
[Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M) native field walks and the small remaining gather arrays come from the same schema. Layout/accessor changes
must update the schema; changing wire order/units/meaning requires a new schema
ID and new fixtures. Correcting explanation text to match an unchanged scale
does not change the wire meaning or schema ID (lifetime peak/dip current: 0.5 A).
The schema ID is independent of the firmware version word.
Arrays are in increasing channel/light/index order. X contains an event number
whose explanations are in the schema. Zero pointers emit zero fields; health
validity flags, calibration level count and event type distinguish unavailable data.

| Record | Meaning | Index |
|---|---|---|
| B | boot | 0 |
| R | real time | 0 |
| S | stored settings | 0 |
| T | effective settings | 0 |
| I | protection and calibration state | 0 |
| Y | limits: ratings, user limit, limit in effect and its source, connected load, channel ceilings, retries | 0 |
| C | counters | 0 |
| M | monitoring | 0 |
| A | calibration | 0 |
| a | calibration channel | channel/light 0..4 |
| H | health | 0 |
| L | lifetime | 0 |
| E | energy | 0 |
| e | channel energy | 0 |
| P | power events (not on the V203 Digital-M) | 0 |
| V | supply summary | 0 |
| v | supply dip (not on the V203 Digital-M) | log slots 0..3, deepest 4 |
| N | event log | event ring slot 0..7 (sort by t_s) |
| O | stored power-on and signal-loss levels | 0 |
| F | reset breadcrumbs | 0 |
| Q | overruns | 0 |
| D | hardware diagnostics | 0 |
| U | ultra config | 0 |
| h | calibration history entry | history entry, first then newest |
| X | message | 0 |
| G | light definition | channel/light 0..4 |
| g | light options | channel/light 0..4 |
| J | logical power-on and signal-loss levels | channel/light 0..4 |
| b | digital boot | 0 |
| r | digital real time | 0 |
| c | digital system counters, calibration phase/error/elapsed and line mode | 0 |
| d | digital line counters | 0 |
| z | digital decoded counters | 0 |
| t | pixel configuration | 0 |
| k | digital calibration model | 0 |
| j | digital jig cycle: cycle, fail bits, each channel's current | 0 |
| y | digital limits: ratings, user limit, mA per channel, the limiter's trim and brightness | 0 |
| l | digital output state | 0 |
| p | pixel segment | live segment 0..3, stored segment 4..7 |
| i | unique identity | 0 |
| W | Ultra applied config state | 0 |
| s | stored extension | 0 |
| K | link statistics: line profile, rate acquisition, frames, errors, bit / LOW extremes, overruns | 0 |

U contains every analog Ultra CONFIG slot, including reserved zeros. S/T contain
all stored/effective WS settings; s contains the stored extension. G/g/J contain
light definitions, options and both scenes. J includes each fade into its level;
O exposes the stored physical PWM levels. SET supplies the takeover transition. Digital t contains
all pixel CONFIG slots; p contains every live and stored segment command. k/l
include calibration and output state; c contains the calibration job state.
The generated native-walk tables encode record ID/count and runs of up to four
adjacent fields. Both target compilers check the named struct spans; typed host
captures check the decoded values. This representation does not change wire bytes.
Native widths in t/k/l/d/z preserve every value without widening their storage
to temporary u32 arrays. Signed k.idle is a 16-bit two's-complement value.
K carries the input line's
LINK_STATS counters (the return channel's page reads the same state). R carries
the second voltage (Analog-M V_B, Dig2Analog auxiliary input) each second. L.wh is the last stored lifetime Wh, while E.base_wh plus
unmerged boot mWh represents current lifetime energy (see energy.h).

The analog boards' production jig record remains the exact textual exception:
`jig cycle=... PASS|FAIL fail=... a=.../.../.../.../... supply=... uid=...,...,... fw=...\r\n`.
Its rounding, spacing, clamping, version text and jig pad trigger are unchanged
(one formatter, core/jig_line.inc; test/test_udjig.c compares it byte for byte with
the earlier one). Firmware events around test mode use X records (2 on, 3 off).
The Digital-M (V203 and V407) reports each jig cycle as the j record instead: the
cycle, the fail bits and a[5] as signed mA (the supply, UID and firmware are in
r, i and b; i now comes at its boot too). a[n] is the bus current of 16 pixels
with channel n (R, G, B, W1, W2) at 25 %, minus the all-off slot's; channels past
the chip's are dark, read ~0 and are outside the verdict (5-400 mA;
core/ur_digital_m/udjig.h).

The console uses USART2 on PA2; receiver replies use USART1 on PA9.
CONSOLE_FULL=1 retains the separate text/input bench console, compiled only for
standalone SWD builds on boards 1/2. It does not contribute text tables, decoder,
input DMA or command code to release images. BOARD 3 remains output-only. Its V203 image emits neither K, y nor
CONFIG_CHANGED (no flash room); the V407 image emits them.

[Golden capture](../test/fixtures/console/all-records.txt) is byte-exact output
from the compiled C emitter on the host, covering every record type. These are
synthetic typed values (including negative values and unknown enum numbers) for
parser testing, not a report of an operating board.
[Decoded inputs](../test/fixtures/console/expected.json) are independent fixture
values. `test/test_console_schema.py` checks all values, framing, lengths and
single-byte corruption. `tools/decode_console.py` is a strict reference decoder
for the next quinled-jig worker. The jig repository is not modified.

The Dig2Analog history is now two entries: h index 0 is the first calibration,
index 1 the latest. CAL2 uses 40-byte records, six per page; upgrading from CALH
starts a fresh history. Calibration and lifetime stores are unaffected. [Diff-Power Ultra Receiver Analog-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Analog-M)
has no history store: A/a carry the last calibration, H the health flags and
comparison with the preceding calibration and running model. HEALTH retains
classes/reasons, test and model deviations, dark-current difference, temperature
peak and age; DIAG retains first-to-latest trend where available. This checkpoint
uses wire-3 HEALTH dark-current units of 2 mA. Status LED indication and all safety
computations remain enabled.
