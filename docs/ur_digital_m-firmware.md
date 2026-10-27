# [Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M) application (BOARD 3)

Firmware 0.7.1, protocol 0.3 (wire 3). Physical timing qualification on the bench
is still open.

On both CH32V203 and CH32V407, AUTO line mode
(CONFIG 75=0, the default) works with a base sender's plain pixel stream.
At least four non-Ultra frames and at least 500 ms of activity select STREAM;
a pixel-line marker selects it immediately. AUTO then replays the incoming
waveform regardless of saved pixel count (CONFIG 66), SOURCE (CONFIG 77), or
stored effects. Local effects and loss policy cannot take over in this state.
The decision sticks until reboot or an explicit forced-mode change; reboot
detects again. Two consecutive valid Ultra frames still select CHAIN (three
for header-only frames). Silence alone leaves AUTO undecided.

A momentary signal drop keeps the last setting by default; local effects keep
running and a pixel strip holds its last latched frame. Returning data resumes
normally with no blink. CONFIG 73 can explicitly enable a stored effect on loss;
its factory value is zero.

The board has one pixel output and four local segments. The CH32V203 has a
2,000-byte shared framebuffer and takes 600 RGB, 500 RGBW or 400 five-channel
pixels; the CH32V407 has two 2,500-byte wire frames and takes 800 RGB, 600 RGBW
or 500 five-channel pixels. A stored calibration is valid up to the same limits. Chip,
channel order, gamma, white handling, frame rate and strip geometry use CONFIG
64..77 on TARGET 0. Board TARGET FF accepts input-current ceiling 21 separately.
Both scopes have independent duplicate SEQs; applied-value CRCs exclude rejected
entries. Zero ceilings mean the board's rated defaults, never unlimited current.

CONFIG 77 selects output source: 0 local, 1 streamed ingress 0. Source 1 requires
explicit STREAM line mode (CONFIG 75=1); unsupported routes are rejected. AUTO
and CHAIN keep the saved source at 0; AUTO's detected STREAM overrides its local
output policy as described above. Forced STREAM (75=1) and CHAIN (75=2) retain
their configured-source behavior. A PIXEL_FX may temporarily run locally with source
1 selected; STOP returns to that configured stream at a latch. With source 0,
STOP leaves black. Whole-output TARGET 0F updates the four reported segment SEQs.
ACKS columns selecting PIXEL_FX always report NEVER/SEQ zero; read PIXEL_FX state
for running IDs and last accepted segment SEQs.

On a CHAIN line the board (CH32V203 and CH32V407) works at any position, mixed
with other receivers. Port 2's gate holds the board's own 256-bit slot and forwards the
rest. It re-arms after 200 us of quiet LOW after a frame, never in a released
HIGH. Every valid POLL (own index or RELAY_ONLY) opens a relay window: at t0 +
500 us port 2 listens and the input drives the line above, so the boards behind
answer through this board; forwarding returns at W + 6 us. If the line above is
still driven at the turnaround, the board neither relays nor answers (PIXEL part
0 BUSY counts it). A STREAM port keeps one receiver and forwards nothing.

PIXEL page 10 part 0 reports line counters/mode, part 1 configuration. PIXEL_FX
page 11 reports state and effect/palette membership. Only output 0 and membership
group 0 are supported. Page 80 is merged into PIXEL; private 81 and duplicate
ERRORS 03 are unsupported. Other public telemetry uses the shared wire-3 header.
The supply impedance is on ENERGY (bytes 19-20), as on every board.

The V203 board (board.json binding feature `lean_telemetry`, for its flash) has no
POWER, SUPPLY or DIAG page and says so in IDENTITY part 1; STATUS 8 bits 0-3 are
0. It still detects and counts supply dips (the flash's dip rules, LIFETIME, the
fault memory, STATUS's supply reason) and keeps the newest dip for them. Its
console sends no P and no v records; the c, b, F, E, L and V records carry what
the DIAG page showed. The V407 board has every page.

The MCU layer runs SPI RX/TX DMA over a 512-word ring. Nominal replay delay is
227.6 us; the board descriptor bounds it at 234 us including oscillator error.
The mux changes only in a safe low/latch interval. DMA/SPI faults blank output,
clear the ring and recover at a low boundary. Local-to-stream switching drains
the local frame; table changes use the pause-request/acknowledgement handshake.
Barrier publication protects tables, grid anchors, pause state and ownership.

The local limiter uses calibrated current and configured input/output ceilings.
It cannot limit unmeasured streamed pixels. Descriptor enforcement therefore
marks firmware enforcement LOCAL_ONLY. Input/output operating-voltage bounds
remain unknown in the descriptor and require installation qualification.

Digital calibration is a bounded, resumable job; its descriptor budget (`cal_ms`) is
the board's own full-length worst case with a 15 % margin: 6 s on the CH32V203
(400 five-byte pixels), 6.5 s on the CH32V407 (500).
It owns the encoder temporarily, rejects invalid configuration, aborts on a
changed accepted configuration (including a lowered input ceiling), and restores
the prior owner through black/latch boundaries. No calibration store creates a
black frame merely to write flash.

Test jig mode: the jig pad held low, or a UART break, for 50 ms enters it, 50 ms
high leaves it, with no reboot. Like a calibration it owns the encoder: the first
16 pixels with one channel at a time at 25 % (raw), all off between, a 1.5 s cycle,
so any strip draws ~80 mA at most. Each channel's bus current against the dark slot
gives the j console record and the status LED's verdict (PASS steady, FAIL 5 Hz).
Frames keep being decoded, answered and applied; on release a black frame is
latched before the scene or the stream returns. A calibration requested meanwhile
fails BUSY.

Flash admission requires local mode, a proven black frame, no queued stream,
no ready frame, encoder LOW and a complete latch plus ring drainage. A raw
stream never admits flash. Effect records are 128 B, format 5, output count 1,
two per 256 B page. Calibration records are 64 B, format 3, output count 1.
Both use independent A/B pairs. Diff-Power-UR-Digital-M native struct records are deliberately not
adopted: their 176 B size halves density with insufficient flash benefit.

The full effect catalogue and palettes are in `src/core/fx_catalogue.json`;
Pacifica is excluded by the application build's documented calibration trade.
The pure engine remains host-testable independently. Replies use USART1 TX
on PA9 at PCLK2 72 MHz; all receiver deployment candidates require WCH XW.
