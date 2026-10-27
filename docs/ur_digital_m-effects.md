# Local pixel effects: the effect engine

The [Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M) can run pixel effects itself, started by the hub over the QuinLED Ultra protocol, instead
of streaming every pixel. This page is the effect engine: a pure C module in `src/core/fx*.c` (integer math only, no hardware, builds
for the host tests and for the receiver). How the delay line puts the effect on the strip, and the protocol slot, are in
[ur_digital_m-firmware.md](ur_digital_m-firmware.md); the effect and palette list the hub builds its entities from is
[fx_catalogue.json](../src/core/fx_catalogue.json); third-party material is in [NOTICE.md](../NOTICE.md).

## Layers (`src/core/fx.h`)

| Layer | What |
|---|---|
| 1. `fx_ctx_t` | ONE effect on ONE run of pixels: `fx_begin`, `fx_frame(t)` once a frame, `fx_render(from, to)` into an RGB chunk. |
| 2. `fx_engine_t` | Up to 4 segments (start / length, reverse, mirror), brightness and colour ramps, the effect crossfade, the compositor. |
| 3. `fx_pack` | Brightness, gamma 2.8, colour order, auto-white, the channel sum, at 16 bits with a rounding or dither to the wire's 8 (CONFIG 76) or at 8 bits; `fx_limit_bri` for the current limiter. |

Every effect is a closed-form function of (parameters, effect time, pixel index). A frame can be rendered in chunks of up to 32 pixels, in
any order, and gives the same pixels; a reversed or mirrored segment, the "race the beam" renderer and two receivers given the same slot
and host clock all follow from that. Nothing is kept per pixel except Fire's heat (up to 256 bytes per Fire segment, plus a previous-heat array for segment indices 0 and 1; the one effect that is stateful by design: a receiver that
joins later shows other flames). Pride, Colorwaves and Pacifica keep no state either: their running sums are closed-form integrals of their
beats, so a receiver that starts the same slot later draws the same picture.

**Time.** The engine takes host milliseconds (the synced clock, wrapping `uint32_t`); an effect's own time is host time minus its anchor
(the PIXEL_FX start). Animations run on a speed-integrated time, linear in the slider like WLED's: speed 128 is about 1x, 255 about 1.9x, 1 about 0.04x, **0 freezes the
effect**, and changing the speed never makes the picture jump. Sunrise and Sunset use the raw time since their anchor.

**Renewal.** The slot's START is a signed 32-bit millisecond anchor, so the hub re-sends it about every 20 days. `fx_engine_set` with the
same effect, palette, geometry and every parameter the same, and only a new anchor, changes nothing: the running time base, phase and picture
continue (tested for every effect, with anchors up to 19.7 days old, and over 60 days of running past 2^31 and 2^32 ms of host time). Any other
change together with a new anchor takes the new anchor (Sunrise / Sunset restart; the animations' phase still continues). To restart a
sunrise, change something (the seed will do) or STOP (id 0) first. Effect time keeps counting past 49 days; the animations' own
speed-integrated time wraps once every ~50 days at 1x (a one-off jump of a few effects, once).

**Transitions** (`trans_ms` in the slot, done on the receiver):

- the same effect and palette: colours and brightness ramp linearly; speed, intensity and the customs change at once;
- another effect (or palette): a crossfade over `trans_ms` (the old effect keeps running while it fades out). Two crossfades can run at
  once; one that finds no free slot switches hard;
- replacing a running crossfade keeps its two live effects and queues only the latest command. If more than 250 ms
  remain, the remaining blend and brightness/colour ramps are retimed from their current values to finish in 250 ms.
  Further replacements keep that deadline. The queued crossfade starts from the completed destination, with its
  requested transition duration and original anchor; frame sampling after the deadline includes elapsed progress.
  This adds at most 250 ms of engine wait, in addition to the controller's frame-boundary latency (output is sampled
  at the configured frame rate). STREAM cancels the fade and its queue. Geometry changes retain their hard-switch
  semantics at handoff. Each occupied fade slot owns its queue, so two replacements can wait independently;
- Fire's fading-out heat stays owned until that fade ends. Fire-to-Fire palette changes blend palettes over the same
  heat field, advanced once by the new context, with no clear or double update;
- from dark the colours snap and only the brightness fades in; "off" is brightness 0 with a transition, and `fx_engine_dark()` is true
  once it is done (flash writes are allowed then);
- effect id 0 (STOP; resumes the configured output source) on a segment makes it inactive (black there).

**Segments.** `seg_start` / `seg_len` (0 = to the end), clipped to the strip; a later segment is drawn over an earlier one. Mirror renders
half the segment and mirrors it, reverse flips it; both are cheap (+17 / +9 cycles per pixel). `FX_MAX_SEG` and `FX_MAX_XFADE` are build options (the defaults, 4 and 2: 1,056 B of RAM on RV32). `vstart` / `vlen` place a run on a virtual
strip that spans ports and receivers (CONFIG 71 / 72).

**Packer.** `ord[5]` says which source channel (R 0, G 1, B 2, W / WW 3, CW 4) goes to each wire byte: GRB `{1,0,2}`, SK6812 GRBW
`{1,0,2,3}`, WS2814 RGBW `{0,1,2,3}` or WRGB `{3,0,1,2}`, WS2805 `{0,1,2,3,4}`. Auto-white takes W = min(r, g, b) after the gamma, out of RGB
(accurate) or on top (brighter); on the 5-channel chip the white splits warm / cool by `white_split`. The sum of the wire bytes it returns
is the current estimate; `fx_limit_bri(bri, sum, max_sum)` gives the brightness for the next frame (it accounts for the gamma).

**Smooth dimming** (`fx_pack_cfg_t.smooth`, CONFIG 76; [ur_digital_m-firmware.md](ur_digital_m-firmware.md) section 14). The 8-bit path (`FX_SM_8BIT`) scales the colour by the limiter's brightness, applies the gamma (the 16-bit table rounded: the 8-bit table, `round(255 * (v / 255) ^ 2.8)`) and the auto-white at 8 bits. The
16-bit paths look the gamma up in a 256-entry table of `65280 * (v / 255) ^ 2.8` (65280 = 255 x 256: a wire level is 256 units), multiply by the
limiter's brightness as the gamma of it (`(c * b) ^ 2.8 = c ^ 2.8 * b ^ 2.8`), take the white out of the 16-bit values and add a threshold before the shift
to 8 bits: 128 rounds (`FX_SM_ROUND`), the ordered 0..255 threshold of the dither (`FX_SM_DITHER`) spreads the fraction over frames. The threshold is
`(frame x 97 + pixel x 53 + source channel x 85) mod 256`: stateless, from the host frame index and the pixel's place on the strip, so receivers on one grid
make the same bytes, a frame cut into chunks equals the whole one, and over 256 frames the average is exactly the 16-bit value (97 / 256 is the golden ratio's
fraction, so any run of frames is spread evenly). One loop does all three: the 8-bit path shifts its bytes up by 8 and adds no threshold. Cycles per pixel at the full brightness (rv32emu):
RGB 82 -> 116 (16-bit), RGBW 105 -> 140, five channels 117 -> 155 (the limiter's brightness below 255 adds ~9).

## Driving it

```c
fx_engine_init(&eng, npx, pool, pool_bytes);               /* pool: four current + two previous heat slots, up to 1,536 B */
fx_engine_set(&eng, light, &params, host_ms, anchor_ms);   /* a PIXEL_FX slot */
fx_engine_frame(&eng, host_ms);                            /* once per frame */
for (a = 0; a < npx; a += FX_CHUNK) {                      /* chunks <= 32 pixels, any order */
    fx_engine_render(&eng, rgb, a, b);                     /* rgb[0 .. b-a) */
    sum += fx_pack(rgb, fb + a * bpp, b - a, limiter_bri, &cfg);
}
```

The receiver sets `eng.deferred = 1`: after each command, dequeue or per-segment frame preparation it calls
`fx_engine_step` once per main-loop pass until it returns zero. A Fire slice cools at most 64 cells or diffuses at most
64 cells and sparks; the RNG order, integer arithmetic and four-step catch-up/drop rule are unchanged. Commands,
parameters and frame time are frozen before preparation. The previous wire frame remains intact during preparation;
rendering starts only when every job is complete, then the completed heat stays frozen through all render chunks.
The synchronous `fx_frame` / `fx_engine_frame` APIs finish preparation before returning. Fire renders in 16-pixel
calls alone, 2-pixel calls with other layers (including a crossfade); this also bounds compositor and packer latency.

**Fire smoothing.** Segment indices **0 and 1** reserve the two previous-heat slots; indices **2 and 3** always use
the original stepping. This assignment depends only on the segment index, not command order or which effects are
active. Changing another segment, its length or its effect never moves a running slot. A fading-out Fire retains
its slot until the fade ends; a Fire palette fade shares both heat arrays and the interpolation fraction.
The four current-heat slots are followed by two previous-heat slots, each `min(npx, 256)` bytes. A smaller caller
pool still supports unsmoothed Fire wherever a current-heat slot fits. The receiver provisions 1,536 B: 1,024 B
current heat plus **512 B** for smoothing. Each context also has a 4-byte optional previous-heat pointer on RV32.

For these two slots, spatial interpolation is performed on each heat array, then the two heat samples are blended
before the palette lookup. The blend fraction is `(elapsed_effect_ms * 256 + et_frac) / 30`: speed zero freezes it,
and speed changes preserve it. This delays the picture by one heat step while keeping the cooling, diffusion,
sparks and RNG sequence. At speed 16 the picture changes between heat steps instead of holding for about 202 ms.

## Effects

Speed, Intensity, Palette (default) and the colour roles per effect (a `-` is not used):

| Id | Effect | Speed | Intensity | Palette | Colours | Custom 1 / 2 |
|---|---|---|---|---|---|---|
| 1 | Solid | - | - | - | Colour | - |
| 2 | Breathe | Rate | Depth | - | Colour, Low colour | - |
| 3 | Colorloop | Rate | Saturation | - | - | - |
| 4 | Rainbow | Speed (a round in ~1.9 s at 128) | Size (128: one per strip, doubling every 32) | - | - | - |
| 5 | Palette | Speed (a round in ~3.9 s at 128) | Size (128: one per strip, doubling every 32) | Party | - | - |
| 6 | Colorwaves | Speed | Wave density | Party | - | - |
| 7 | Pride 2015 | Speed | Wave density | - | - | - |
| 8 | Sinelon | Speed | Trail (higher: shorter tail) | Rainbow | Background | Dots (0-127 one, 128-255 two) |
| 9 | Twinklefox | Speed | Density | C9 New | Background | Incandescent cooling (0-127 on, 128-255 off) |
| 10 | Colortwinkles | Speed | Density | Party | Background | - |
| 11 | Candle | Flicker rate | Flicker amount | - | Flame, Dim colour | - |
| 12 | Candle Multi | Flicker rate | Flicker amount | - | Flame, Dim colour | - |
| 13 | Aurora | Speed | Curtain size | Aurora | - | - |
| 14 | Blends | Speed | Ripple tightness (higher: shorter ripples) | Party | - | - |
| 15 | Flow | Speed (a round in ~2 s at 128) | Zones (higher: more, shorter) | Rainbow | - | - |
| 16 | Fire 2012 | Speed | Sparking | Heat | - | Cooling |
| 17 | Gradient | Scroll speed (0 still) | Repeats (0: one gradient, no loop) | - | Colour 1, 2, 3 | - |
| 18 | Pacifica | Speed | Whitecaps | - | - | - |
| 19 | Sunrise | Duration (10 s .. 1 h) | Spread along the strip | Sunrise | - | - |
| 20 | Sunset | Duration (10 s .. 1 h) | Spread along the strip | Sunrise | - | - |
| 21 | Glitter | Speed | Sparkle density | Rainbow | Sparkle | - |
| 22 | Meteor | Speed | Tail length | - | Meteor, Background | - |
| 23 | Fairy | Speed | Density | Fairy | Background | - |
| 24 | Plasma | Speed | Scale | Party | - | - |
| 25 | Noise | Speed | Scale | Lava | - | - |
| 26 | Running | Speed | Wavelength | - | Colour 1, 2 | - |
| 27 | Ripples | Speed | Ripple size (how far a ring spreads, 8-72 px) | Winter | Background | Rain (how many drops fall, 0: none) |
| 28 | Wipe | Speed (a cycle is ~4.2 s at 128) | Hold (0: none, 255: three quarters of the 4 s cycle) | Christmas | Colour 1, 2 | Colours (0-127 colour 1 and 2 take turns, 128-255 a new palette colour each wipe) |
| 29 | Stripes | Speed (0 still; ~20 px/s at 128) | Stripe size (higher: finer, 2-504 px period) | - | Colour 1, 2 | Share of colour 1 (128: half) |
| 30 | Bulbs | Shift speed (0 still; a hop every ~0.7 s at 128) | Spacing (higher: closer bulbs) | C9 New | Unlit | Bulb size (share of the spacing), Colour step (64: a quarter of the palette) |
| 31 | Lava Lamp | Speed | Blob size | Sunset | Background | - |
| 32 | Mosaic Drift | Colour change speed | Tile size (higher: smaller tiles) | Analogous | - | Whiten (mixes neutral white in) |
| 33 | Silk Weave | Speed | Ribbon spacing (higher: closer) | - | Ribbon 1, Ribbon 2, Ground | White at the crossings |
| 34 | Canopy Shade | Sway speed | Shadow depth | - | Wash, Shadow | Leaf spacing (higher: closer) |
| 35 | Opal Facets | Reflection speed | Facet width (higher: narrower) | Pastel | - | Reflection strength |
| 36 | Quiet Landings | Bounce pace | Height (share of the strip) | Winter | Ground | Pearls (0-85 one, 86-170 two, 171-255 three) |

Pride 2015, Colorwaves, Twinklefox, Pacifica, Fire 2012 and Sinelon are re-implementations of Mark Kriegsman's FastLED examples (MIT) at the
paces of WLED's namesakes (Rainbow, Colorloop, Palette, Flow, Glitter, Breathe, Meteor, Pride, Colorwaves too); the rest is this firmware's own (Ripples, Wipe, Stripes, Bulbs, Lava Lamp and the five after them (Mosaic Drift, Silk Weave, Canopy Shade, Opal Facets, Quiet Landings) are closed forms of the effect time like the others, so receivers that join late draw the same picture). Ids are never reused: `test/fx_ledger.json` lists every id issued with its name.

**Palettes** (16 entries, interpolated; 254 = colour 1 -> 2 -> 3, 255 = the effect's default; origins and licences in [NOTICE.md](../NOTICE.md): Analogous, Hult,
Magred and Yelmag are our own look-alikes of their namesakes; stored as RGB565, at most 4 of 255 off the source stops): 0 Rainbow, 1 Analogous, 2 April Night, 3 C9 New,
4 Hult, 5 Magred, 6 Orange & Teal, 7 Rainbow Bands, 8 Retro Clown, 9 Yelmag, 10 Party, 11 Ocean, 12 Forest, 13 Lava, 14 Heat, 15 Cloud,
16 Sunset, 17 Aurora, 18 Fairy, 19 Sunrise, 20 Candlelight, 21 Warm White (about 1900-3000 K), 22 Daylight (2700-6500 K), 23 Christmas, 24 Winter,
25 Halloween, 26 Autumn, 27 Pastel, 28 Embers, 29 Rose, 30 Moonlight. On RGBW strips (WS2814, SK6812) the packer's auto-white puts min(r, g, b) on the white LED, so the cool and pastel
palettes use it (Moonlight 39 %, Pastel 33 %, Daylight 28 %, Winter 24 % of the light) and the warm whites hardly do (Candlelight 5 %,
Warm White 4 %: little blue); a warm white that drives a warm-white LED would need a different packer rule.

## Tests and measurements

| What | How |
|---|---|
| Host tests (MSVC): math, every effect, chunk and order invariance, determinism, fuzzing, golden frames, segments, transitions, packer | `test\build_fx.cmd` (`test_fx.exe --golden` writes a new golden table after a deliberate change) |
| Catalogue = header = generated files, ids never reused | `python test/test_fx_catalogue.py` (run by `build_fx.cmd`) |
| Cycles per pixel, code size and RAM on the CH32V203 (rv32emu model) | `python tools/measure_fx.py` |
| Look: PNG images, x = pixels, y = time | `test\build_fx.cmd build\fx render`, then `fx_render sheet|one|pal out.png` |
| Regenerate tables and catalogue | `python tools/gen_fx_tables.py` (`--check` verifies) |
| Frames of two engine builds, the worst channel difference per effect (for size squeezes) | `test\build_fxdump.cmd` in both trees, `fx_dump.exe out.bin`, `python test/fx_compare.py old.bin new.bin` |

`tools/measure_fx.py` reports per-function sizes and cycles with xPack GCC 15.2 and XW=0 flags (a non-LTO benchmark);
`tools/size_gate.py` measures the linked release images, including both compilers and every answer-UART setting.
The palettes are RGB565 (16 x 2 B each); gamma 2.8 is one 256-entry 16-bit table (its rounded values are the 8-bit table).
Helpers are shared except in Pacifica, which keeps them inline (the CPU worst case). On RV32 `sizeof(fx_engine_t)` is 1,056 B plus the Fire pool.
Effect files enable scalar replacement for RGB structs; effect renderers share register-save/restore stubs.
Preparation uses direct dispatch, while the render table keeps one function pointer per effect.
The generic controller (with Pacifica enabled) renders it in 4-pixel steps to bound main-loop latency; the benchmark's 32-pixel batches measure the engine alone.

On chain lines, the controller limits a render call to a total quota of 12 pixels
across live effect contexts. Crossfade sources count
as contexts too; five or six live contexts use one pixel per call. The quota applies as
soon as AUTO decides CHAIN, including a standalone frame already being rendered.
STREAM and Pacifica keep their existing quotas. Repeated setup can still exceed the effect-step budget for mirrored/overlapping
Pacifica and six live Mosaic Drift contexts. Those configurations require further
scheduling work to guarantee the same bound.
The effect quota is not a complete main-loop bound; decoding and millisecond services add work.

The BOARD 3 calibration build defines `FX_NO_PACIFICA=1`. It advertises 35 effects and omits ID 18 from
PIXEL_FX page 0x11's effect bitmap. Pacifica commands and saved presets are ignored; its numeric ID is never reassigned.
The generic engine and host catalogue retain all 36 IDs. Other effects and all 31 palettes are unchanged.
