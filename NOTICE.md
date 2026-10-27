<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Copyright and licence

Copyright (C) 2026 QuinLED.

The project-authored firmware, bootloader, MCU register definitions, core helpers,
startup and linker scripts are licensed under the GNU General Public License,
version 3 or (at your option) any later version (`GPL-3.0-or-later`). See [LICENSE](LICENSE).

The source tree ships no WCH SDK files. The MCU support in
[`src/mcu/ch32v20x`](src/mcu/ch32v20x/PROVENANCE.md) was independently written from
hardware manuals and the project's own interface and memory-layout requirements.
Hardware addresses, register fields and interrupt numbers describe the MCU.
Each new support file records its factual sources; the provenance inventory
identifies the files covered by this replacement. The V40x support in `src/mcu/ch32v40x`
contains no WCH SDK files either.

The build uses external compiler toolchains, whose licences govern those tools
and any linked runtime material separately. They are not bundled in this tree.

The firmware says so itself: its console banner shows
"QuinLED <board> vX.Y.Z, (C) 2026 QuinLED, GPL-3.0, github.com/intermittech/quinled-ultra-firmware".

## Third-party material in the local pixel effects

This covers both effect engines: the V203 one (`src/core/fx*.c`, effects 1-36, palettes 0-30) and the V407 Digital-M one
(`src/core/fx_v407/`, which runs effects 1-36 through an adapter and adds effects 37-114 and palettes 31-48).

The firmware is GPL-3.0-or-later. Neither engine contains WLED program code: every effect is written from its
described behaviour in this firmware's own integer code (Flow's idea, palette zones running in opposite directions, is WLED's Flow behaviour; its zone law and code are our own; Sinelon, Blends and
the others are re-compositions). What does come from WLED is the colour stops of three palettes (below), which both
catalogues ship. The items below are what *is* derived from, and under which terms.
`tools/gen_fx_tables.py` holds the palette data with the same origins; `src/core/fx_catalogue.json` (V203) and
`src/core/fx_v407/catalogue_v407.json` (V407) list an origin per effect and per palette.

## FastLED (MIT) — algorithms re-implemented in integer code

Pride 2015, Colorwaves, TwinkleFOX, Pacifica, Fire 2012 and the sinelon idea are re-implementations (fixed-point, no floating point,
no per-pixel history, closed forms for the sums along the strip) of Mark Kriegsman's examples in the FastLED library; the sixteen-entry
palettes Rainbow, Rainbow Bands, Party, Ocean, Forest, Lava, Heat, Cloud and Pacifica's three are FastLED's colour tables.

- Pride2015: "Animated, ever-changing rainbows", by Mark Kriegsman (FastLED example Pride2015).
- Colorwaves: Pride2015's scheme (the same beats and sums) with a palette instead of the hue, after Mark Kriegsman's gist
  "ColorWavesWithPalettes" (https://gist.github.com/kriegsman/8281905786e8b2632aeb, no licence stated; the algorithm is Pride2015's, MIT).
- TwinkleFOX: "Twinkling 'holiday' lights that fade in and out", Mark Kriegsman, December 2015 (the per-pixel pseudo random chain is
  replaced by a hash of the pixel index).
- Pacifica: "Gentle, blue-green ocean waves", December 2019, Mark Kriegsman and Mary Corey March.
- Fire2012: Mark Kriegsman (FastLED example Fire2012).
- sinelon: FastLED example DemoReel100, Mark Kriegsman (the fade is computed instead of kept per pixel).
- Palettes: FastLED colorpalettes (RainbowColors_p, RainbowStripeColors_p, PartyColors_p, OceanColors_p, ForestColors_p, LavaColors_p,
  HeatColors_p, CloudColors_p) and the three Pacifica palettes.

> The MIT License (MIT)
>
> Copyright (c) 2013 FastLED
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
> (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
> publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
> so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
> MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
> FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
> CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Palettes taken from other projects (colour stops as data)

**Retro Clown** is the gradient "bhw1_01" by Blackheartedwolf (2011) from the cpt-city archive
(http://seaviewsensing.com/pub/cpt-city/bhw/bhw1/), licensed under the Creative Commons Attribution 3.0 licence
(https://creativecommons.org/licenses/by/3.0/): adapted here (resampled to 16 entries; the colour values are the original's). It is
separately licensed data under CC BY 3.0; it is not part of the GPL-licensed code.

**April Night, Orange & Teal and C9 New** (palettes 2, 6 and 3 in both catalogues) are palettes by Aircoookie (Christian Schwinne) from WLED, added while WLED was MIT-licensed
(2018-2020; the same stops are in WLED v0.14.4, still MIT). Only their colour stops (numbers) are used, resampled to sixteen entries;
no WLED code is used. C9 New derives from the C9 colours of Mark Kriegsman's TwinkleFOX gist
(https://gist.github.com/kriegsman/756ea6dcae8e30845b5a). WLED's licence for these:

> MIT License
>
> Copyright (c) 2016 Christian Schwinne
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
> (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
> publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
> so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
> MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
> FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
> CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**Names.** The palette names Analogous, April Night, C9 New, Hult, Magred, Orange & Teal, Rainbow Bands, Retro Clown and Yelmag follow
WLED's palette list, the set chosen for this firmware. For **Analogous, Hult, Magred and Yelmag** the stops are NOT WLED's or cpt-city's:
they are our own look-alikes (the same mood and colour progression, stops chosen by eye from the description), because the licences of
the originals (cpt-city "free to use" without a redistribution right for Hult, an ambiguous GPL / GFDL record for the three by Nevit
Dilmen, and WLED's 2025 gamma-adjusted numbers under the EUPL-1.2) do not clearly allow copying them into this firmware.

## Public domain

- `fx_hash` is lowbias32 by Chris Wellons (Hash Prospector), released into the public domain.

## Own work (GPL-3.0-or-later)

Everything else: the engine, the effects Solid, Breathe, Colorloop, Rainbow, Palette, Sinelon's trail, Colortwinkles, Candle,
Candle Multi, Aurora, Blends, Flow (its zone law is our own), Gradient, Sunrise, Sunset, Glitter, Meteor, Fairy, Plasma, Noise, Running, Ripples, Wipe, Stripes,
Bulbs, Lava Lamp, Mosaic Drift, Silk Weave, Canopy Shade, Opal Facets, Quiet Landings, the palettes Sunset, Aurora, Fairy, Sunrise, Candlelight, Warm White, Daylight, Christmas, Winter, Halloween, Autumn, Pastel, Embers,
Rose, Moonlight (the colour temperatures of the whites follow the common black-body-to-RGB table, public numerical data; every other stop was chosen by eye) and the look-alikes Analogous, Hult, Magred and Yelmag, the packer, the tables' generators and the tests; and the V407 engine (`src/core/fx_v407/`) with its effects 37-114 and palettes 31-48 (Iridescent to Graphite Citrus), whose catalogue origins are "own" or name the public science or published algorithm an effect models.

## Trademark

"QuinLED" is a trademark of its owner, and this license does not grant any right to it. You may build, change and
share this firmware under the GPL, but a derived product may not be called QuinLED or presented as one. Keep the
credit to the original project in the source and documentation.
