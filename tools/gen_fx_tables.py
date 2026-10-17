# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Writes src/core/fx_tables.c (the quarter sine, the gamma 2.8 table, the built-in palettes, Pacifica's three) and the
effect / palette catalogue (src/core/fx_catalogue.json: ids, names, parameters, defaults, origins). The catalogue is the
list the hub (ESPHome) builds its effect and palette entities from; the ids in it match src/core/fx.h (test_fx.c and
test/test_fx_catalogue.py check that). Ids are NEVER reused.

    python tools/gen_fx_tables.py [--copy <path to also write the catalogue to>]

Palette sources, licences (NOTICE.md lists them):
  FastLED  16-entry palettes (colorpalettes.cpp) and Pacifica's (examples/Pacifica), MIT
  cpt-city Retro Clown: gradient bhw1_01 by Blackheartedwolf (2011), CC BY 3.0, resampled
  WLED     April Night, C9 New, Orange & Teal: by Aircoookie, added while WLED was MIT-licensed (colour stops only)
  own      Aurora, Sunset, Fairy, Sunrise, Candlelight, Warm White, Daylight, Christmas, Winter, Halloween, Autumn, Pastel,
           Embers, Rose, Moonlight, and look-alikes for Analogous, Hult, Magred, Yelmag (named after the WLED /
           cpt-city palettes, whose stops are NOT used: their licences do not allow it)
  Both (not used): the names follow WLED's palette list, which Quindor chose.
Each palette is 16 entries, entry k at 0..255 position k * 17, linear in RGB between the source stops."""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "src", "core"))
TEST = os.path.normpath(os.path.join(HERE, "..", "test"))
CRLF, LF = b"\r\n", b"\n"


def hexpal(vals):
    assert len(vals) == 16
    return [((v >> 16) & 255, (v >> 8) & 255, v & 255) for v in vals]


def resample(stops):
    """[(pos 0..255, (r, g, b))] -> 16 entries at pos k * 17 (linear; flat outside the stops)"""
    out = []
    for k in range(16):
        x = k * 17
        if x <= stops[0][0]:
            out.append(stops[0][1])
            continue
        if x >= stops[-1][0]:
            out.append(stops[-1][1])
            continue
        for (p0, c0), (p1, c1) in zip(stops, stops[1:]):
            if p0 <= x <= p1:
                f = 0.0 if p1 == p0 else (x - p0) / (p1 - p0)
                out.append(tuple(int(round(a + (b - a) * f)) for a, b in zip(c0, c1)))
                break
    return out


NAMED = dict(Black=0x000000, Maroon=0x800000, DarkRed=0x8B0000, Red=0xFF0000, Orange=0xFFA500, White=0xFFFFFF,
             Blue=0x0000FF, DarkBlue=0x00008B, SkyBlue=0x87CEEB, LightBlue=0xADD8E6, MidnightBlue=0x191970,
             Navy=0x000080, MediumBlue=0x0000CD, SeaGreen=0x2E8B57, Teal=0x008080, CadetBlue=0x5F9EA0,
             DarkCyan=0x008B8B, CornflowerBlue=0x6495ED, Aquamarine=0x7FFFD4, Aqua=0x00FFFF, LightSkyBlue=0x87CEFA,
             DarkGreen=0x006400, DarkOliveGreen=0x556B2F, Green=0x008000, ForestGreen=0x228B22, OliveDrab=0x6B8E23,
             MediumAquamarine=0x66CDAA, LimeGreen=0x32CD32, YellowGreen=0x9ACD32, LightGreen=0x90EE90,
             LawnGreen=0x7CFC00)


def named(s):
    return hexpal([NAMED[n] for n in s.split()])


FASTLED = "FastLED colorpalettes.cpp, MIT (Daniel Garcia and contributors)"
LOOK = ("our own look-alike of the palette of this name (WLED / cpt-city have one): same mood and colour progression, "
        "stops chosen by eye from its description, no values copied")


def cpt(path, who):
    return "cpt-city, http://seaviewsensing.com/pub/cpt-city/%s: %s" % (path, who)


WLED = ("palette by Aircoookie (Christian Schwinne) from WLED, added while WLED was MIT-licensed (Copyright (c) 2016 Christian Schwinne; "
        "same stops in v0.14.4); colour stops only")
OWN = "own, made for this firmware (GPL-3.0-or-later)"

# id, name, 16 entries, origin, cyclic (the last entry blends back into the first without a seam)
PALETTES = [
    (0, "Rainbow", hexpal([0xFF0000, 0xD52A00, 0xAB5500, 0xAB7F00, 0xABAB00, 0x56D500, 0x00FF00, 0x00D52A,
                           0x00AB55, 0x0056AA, 0x0000FF, 0x2A00D5, 0x5500AB, 0x7F0081, 0xAB0055, 0xD5002B]),
     FASTLED + ": RainbowColors_p", True),
    (1, "Analogous", resample([(0, (40, 10, 255)), (70, (100, 0, 240)), (140, (170, 0, 170)), (200, (225, 0, 80)),
                               (255, (255, 10, 10))]), LOOK, False),
    (2, "April Night", resample([(0, (1, 5, 45)), (10, (1, 5, 45)), (25, (5, 169, 175)), (40, (1, 5, 45)),
                                 (61, (1, 5, 45)), (76, (45, 175, 31)), (91, (1, 5, 45)), (112, (1, 5, 45)),
                                 (127, (249, 150, 5)), (143, (1, 5, 45)), (162, (1, 5, 45)), (178, (255, 92, 0)),
                                 (193, (1, 5, 45)), (214, (1, 5, 45)), (229, (223, 45, 72)), (244, (1, 5, 45)),
                                 (255, (1, 5, 45))]), WLED, True),
    (3, "C9 New", resample([(0, (255, 5, 0)), (60, (255, 5, 0)), (61, (196, 57, 2)), (120, (196, 57, 2)),
                            (121, (6, 126, 2)), (180, (6, 126, 2)), (181, (4, 30, 114)), (255, (4, 30, 114))]),
     WLED + "; derived from Mark Kriegsman's C9 colours (https://gist.github.com/kriegsman/756ea6dcae8e30845b5a)", False),
    (4, "Hult", resample([(0, (240, 210, 250)), (40, (250, 180, 245)), (100, (230, 90, 225)), (170, (70, 140, 210)),
                          (230, (20, 180, 175)), (255, (20, 180, 175))]), LOOK, False),
    (5, "Magred", resample([(0, (0, 0, 0)), (70, (90, 0, 100)), (130, (230, 0, 230)), (190, (250, 0, 110)),
                            (255, (245, 0, 10))]), LOOK, False),
    (6, "Orange & Teal", resample([(0, (0, 150, 92)), (55, (0, 150, 92)), (200, (255, 72, 0)), (255, (255, 72, 0))]),
     WLED, False),
    (7, "Rainbow Bands", hexpal([0xFF0000, 0x000000, 0xAB5500, 0x000000, 0xABAB00, 0x000000, 0x00FF00, 0x000000,
                                 0x00AB55, 0x000000, 0x0000FF, 0x000000, 0x5500AB, 0x000000, 0xAB0055, 0x000000]),
     FASTLED + ": RainbowStripeColors_p", True),
    (8, "Retro Clown", resample([(0, (244, 168, 48)), (26, (244, 168, 48)), (117, (230, 78, 92)), (255, (173, 54, 228))]),
     cpt("bhw/bhw1/bhw1_01.c3g", "gradient \"bhw1_01\" by Blackheartedwolf (2011), Creative Commons Attribution 3.0 "
         "(https://creativecommons.org/licenses/by/3.0/); resampled to 16 entries"), False),
    (9, "Yelmag", resample([(0, (0, 0, 0)), (40, (95, 0, 5)), (85, (235, 0, 10)), (125, (250, 0, 110)),
                            (170, (235, 10, 240)), (212, (250, 120, 140)), (255, (250, 240, 20))]), LOOK, False),
    (10, "Party", hexpal([0x5500AB, 0x84007C, 0xB5004B, 0xE5001B, 0xE81700, 0xB84700, 0xAB7700, 0xABAB00,
                          0xAB5500, 0xDD2200, 0xF2000E, 0xC2003E, 0x8F0071, 0x5F00A1, 0x2F00D0, 0x0007F9]),
     FASTLED + ": PartyColors_p", True),
    (11, "Ocean", named("MidnightBlue DarkBlue MidnightBlue Navy DarkBlue MediumBlue SeaGreen Teal CadetBlue Blue "
                        "DarkCyan CornflowerBlue Aquamarine SeaGreen Aqua LightSkyBlue"),
     FASTLED + ": OceanColors_p", False),
    (12, "Forest", named("DarkGreen DarkGreen DarkOliveGreen DarkGreen Green ForestGreen OliveDrab Green SeaGreen "
                         "MediumAquamarine LimeGreen YellowGreen LightGreen LawnGreen MediumAquamarine ForestGreen"),
     FASTLED + ": ForestColors_p", False),
    (13, "Lava", named("Black Maroon Black Maroon DarkRed DarkRed Maroon DarkRed DarkRed DarkRed Red Orange White "
                       "Orange Red DarkRed"), FASTLED + ": LavaColors_p", False),
    (14, "Heat", hexpal([0x000000, 0x330000, 0x660000, 0x990000, 0xCC0000, 0xFF0000, 0xFF3300, 0xFF6600, 0xFF9900,
                         0xFFCC00, 0xFFFF00, 0xFFFF33, 0xFFFF66, 0xFFFF99, 0xFFFFCC, 0xFFFFFF]),
     FASTLED + ": HeatColors_p (the black-body ramp of HeatColor)", False),
    (15, "Cloud", named("Blue DarkBlue DarkBlue DarkBlue DarkBlue DarkBlue DarkBlue DarkBlue Blue DarkBlue SkyBlue "
                        "SkyBlue LightBlue White LightBlue SkyBlue"), FASTLED + ": CloudColors_p", False),
    (16, "Sunset", resample([(0, (60, 0, 80)), (77, (200, 0, 60)), (140, (255, 80, 0)), (204, (255, 200, 40)),
                             (255, (60, 0, 80))]), OWN, True),
    (17, "Aurora", resample([(0, (1, 5, 45)), (60, (0, 120, 40)), (110, (0, 230, 60)), (150, (20, 255, 120)),
                             (190, (0, 150, 150)), (225, (60, 30, 160)), (255, (1, 5, 45))]), OWN, True),
    (18, "Fairy", resample([(0, (255, 190, 90)), (70, (255, 140, 60)), (130, (255, 220, 160)), (190, (255, 120, 150)),
                            (255, (255, 190, 90))]), OWN, True),
    (19, "Sunrise", resample([(0, (0, 0, 0)), (40, (30, 2, 4)), (90, (110, 10, 6)), (140, (215, 48, 6)),
                              (180, (255, 110, 18)), (215, (255, 175, 70)), (240, (255, 225, 160)),
                              (255, (255, 250, 235))]), OWN + " (Sunrise / Sunset: a warm sky from dark to day)", False),
    # round 15 (ud-fx-more): ambient, seasonal and warm/cool white palettes, all our own (colour temperatures from the
    # black-body approximation, stops chosen by eye)
    (20, "Candlelight", resample([(0, (60, 12, 0)), (70, (180, 60, 5)), (140, (255, 140, 30)), (200, (255, 190, 90)),
                                  (255, (255, 225, 160))]), OWN + " (candle flame, amber to warm white)", False),
    (21, "Warm White", resample([(0, (255, 147, 41)), (85, (255, 169, 87)), (170, (255, 190, 120)),
                                 (255, (255, 147, 41))]), OWN + " (warm whites, about 1900 K - 3000 K; on RGBW strips the white LED carries little of them: auto-white takes min(r, g, b), and a warm white has almost no blue)", True),
    (22, "Daylight", resample([(0, (255, 169, 87)), (64, (255, 209, 163)), (128, (255, 249, 253)), (192, (255, 209, 163)),
                               (255, (255, 169, 87))]), OWN + " (whites from 2700 K to 6500 K, colour temperatures from the common black-body RGB table; the cool end puts ~30 % of its light on the white LED of RGBW strips)", True),
    (23, "Christmas", resample([(0, (220, 0, 0)), (30, (220, 0, 0)), (64, (0, 160, 20)), (94, (0, 160, 20)),
                                (128, (255, 170, 90)), (158, (255, 170, 90)), (192, (255, 110, 0)), (222, (255, 110, 0)),
                                (255, (220, 0, 0))]), OWN + " (red, green, warm white, gold)", True),
    (24, "Winter", resample([(0, (70, 130, 255)), (60, (170, 215, 255)), (120, (240, 248, 255)), (180, (130, 200, 235)),
                             (255, (70, 130, 255))]), OWN + " (ice blue and white)", True),
    (25, "Halloween", resample([(0, (255, 90, 0)), (64, (120, 0, 170)), (128, (10, 0, 18)), (192, (70, 200, 0)),
                                (255, (255, 90, 0))]), OWN + " (orange, purple, green)", True),
    (26, "Autumn", resample([(0, (110, 10, 0)), (60, (200, 50, 0)), (120, (255, 130, 10)), (180, (220, 170, 20)),
                             (220, (150, 60, 5)), (255, (110, 10, 0))]), OWN + " (leaves: rust, orange, gold)", True),
    (27, "Pastel", resample([(0, (255, 170, 190)), (50, (255, 210, 170)), (100, (250, 250, 180)), (150, (180, 240, 200)),
                             (200, (170, 200, 250)), (235, (210, 180, 245)), (255, (255, 170, 190))]),
     OWN + " (soft pink, peach, lemon, mint, sky, lilac)", True),
    (28, "Embers", resample([(0, (0, 0, 0)), (60, (70, 6, 0)), (120, (190, 40, 0)), (180, (255, 100, 5)),
                             (230, (255, 170, 40)), (255, (255, 210, 110))]), OWN + " (glowing coals)", False),
    (29, "Rose", resample([(0, (90, 0, 40)), (70, (200, 20, 90)), (130, (255, 70, 130)), (190, (255, 150, 170)),
                           (255, (90, 0, 40))]), OWN + " (deep rose to blush)", True),
    (30, "Moonlight", resample([(0, (8, 12, 40)), (80, (40, 60, 130)), (150, (150, 170, 225)), (210, (225, 232, 250)),
                                (255, (8, 12, 40))]), OWN + " (night blue to silver)", True),
]
PACIFICA = [
    hexpal([0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117, 0x000019, 0x00001C,
            0x000026, 0x000031, 0x00003B, 0x000046, 0x14554B, 0x28AA50]),
    hexpal([0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117, 0x000019, 0x00001C,
            0x000026, 0x000031, 0x00003B, 0x000046, 0x0C5F52, 0x19BE5F]),
    hexpal([0x000208, 0x00030E, 0x000514, 0x00061A, 0x000820, 0x000927, 0x000B2D, 0x000C33, 0x000E39, 0x001040,
            0x001450, 0x001860, 0x001C70, 0x002080, 0x1040BF, 0x2060FF]),
]

# ------------------------------------------------------------------------------------------------ the effects
W = (255, 255, 255)
WARM = (255, 140, 30)
# id, name, summary, speed, intensity, palette?, colours (names), custom1, custom2, defaults, state B/px, origin
# defaults: speed, intensity, palette (None = not used / 255), c1, c2, c3, custom1, custom2
EFFECTS = [
    (1, "Solid", "One colour; fades smoothly to another (the transition).", None, None, False, ["Colour"], None, None,
     (128, 128, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (2, "Breathe", "The colour swells and fades like breathing.", "Rate", "Depth", False, ["Colour", "Low colour"],
     None, None, (128, 255, 255, WARM, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (3, "Colorloop", "The whole strip cycles through the hues.", "Rate", "Saturation", False, [], None, None,
     (128, 255, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (4, "Rainbow", "A rainbow moves along the strip.", "Speed", "Size (128: one rainbow per strip, doubling every 32)", False, [], None, None,
     (128, 128, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (5, "Palette", "The chosen palette moves along the strip.", "Speed", "Size (128: one palette per strip, doubling every 32)", True, [], None,
     None, (128, 128, 10, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (6, "Colorwaves", "Slow, ever-changing waves of palette colour.", "Speed", "Wave density", True, [], None, None,
     (128, 128, 10, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "FastLED ColorWavesWithPalettes (Mark Kriegsman), MIT"),
    (7, "Pride 2015", "Animated, ever-changing rainbows (not palette-based).", "Speed", "Wave density", False, [],
     None, None, (128, 128, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "FastLED Pride2015 (Mark Kriegsman), MIT"),
    (8, "Sinelon", "A dot sweeps back and forth, trailing a fading tail.", "Speed", "Trail (higher: shorter tail)", True,
     [None, "Background"], "Dots (0-127: one, 128-255: two)", None, (128, 128, 0, W, (0, 0, 0), (0, 0, 0), 0, 0), 0,
     "FastLED DemoReel100 sinelon (Mark Kriegsman, MIT), tail computed instead of faded"),
    (9, "Twinklefox", "Twinkling holiday lights that fade in and out.", "Speed", "Density", True, [None, "Background"],
     "Incandescent cooling (0-127: on, 128-255: off)", None, (128, 150, 3, W, (0, 0, 0), (0, 0, 0), 0, 0), 0,
     "FastLED TwinkleFOX (Mark Kriegsman), MIT"),
    (10, "Colortwinkles", "Pixels light up in random palette colours and fade away.", "Speed", "Density", True,
     [None, "Background"], None, None, (128, 100, 10, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (11, "Candle", "The whole strip flickers like a candle.", "Flicker rate", "Flicker amount", False, ["Flame", "Dim colour"], None,
     None, (128, 128, 255, WARM, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (12, "Candle Multi", "Every pixel flickers on its own like a candle.", "Flicker rate", "Flicker amount", False,
     ["Flame", "Dim colour"], None, None, (128, 128, 255, WARM, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (13, "Aurora", "Drifting curtains of northern-light colour.", "Speed", "Curtain size", True, [], None, None,
     (128, 128, 17, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (14, "Blends", "A smooth rippling wash of the palette's colours that drifts along.", "Speed", "Ripple tightness (higher: shorter ripples)", True, [],
     None, None, (128, 190, 10, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (15, "Flow", "Zones that each show the palette, flowing in turn in opposite directions.", "Speed", "Zones (higher: more, shorter zones)", True,
     [], None, None, (128, 128, 0, (0, 0, 0), (0, 0, 0), (0, 0, 0), 0, 0), 0, "own, from the described behaviour"),
    (16, "Fire 2012", "Flames rising from the start of the strip.", "Speed", "Sparking", True, [],
     "Cooling", None, (128, 120, 14, W, (0, 0, 0), (0, 0, 0), 85, 0), 1,
     "FastLED Fire2012 (Mark Kriegsman), MIT"),
    (17, "Gradient", "Colour 1 to colour 2 to colour 3 across the strip.", "Scroll speed (0: still)",
     "Repeats (0: one gradient, no loop)", False, ["Colour 1", "Colour 2", "Colour 3"], None, None,
     (0, 0, 255, (255, 0, 0), (255, 160, 0), (0, 0, 255), 0, 0), 0, "own"),
    (18, "Pacifica", "Gentle blue-green ocean waves.", "Speed", "Whitecaps", False, [], None, None,
     (128, 128, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "FastLED Pacifica (Mark Kriegsman, Mary Corey March), MIT"),
    (19, "Sunrise", "A slow wake-up: dark, red, orange, warm white.", "Duration (10 s .. 1 h)",
     "Spread along the strip", True, [], None, None, (128, 0, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (20, "Sunset", "The wake-up backwards: warm white to dark, and it stays dark.", "Duration (10 s .. 1 h)",
     "Spread along the strip", True, [], None, None, (128, 0, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (21, "Glitter", "A palette backdrop with short sparkles.", "Speed", "Sparkle density", True, ["Sparkle"], None,
     None, (128, 40, 0, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (22, "Meteor", "A meteor with a shimmering tail runs along the strip.", "Speed", "Tail length", False,
     ["Meteor", "Background"], None, None, (128, 128, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (23, "Fairy", "Sparse, soft, slow twinkles like fairy lights.", "Speed", "Density", True, [None, "Background"], None,
     None, (100, 60, 18, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (24, "Plasma", "Smooth overlapping waves through the palette.", "Speed", "Scale", True, [], None, None,
     (128, 128, 10, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (25, "Noise", "Drifting smooth noise through the palette.", "Speed", "Scale", True, [], None, None,
     (128, 128, 13, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (26, "Running", "Waves of colour 1 on colour 2 run along the strip.", "Speed", "Wavelength", False,
     ["Colour 1", "Colour 2"], None, None, (128, 128, 255, (255, 120, 0), (0, 0, 30), (0, 0, 0), 0, 0), 0, "own"),
    (27, "Ripples", "Raindrops on a pond: soft rings spread out and fade.", "Speed", "Ripple size", True, [None, "Background"],
     "Rain (how many of the drops fall; 0: none)", None, (128, 128, 255, W, (0, 0, 0), (0, 0, 0), 200, 0), 0, "own"),
    (28, "Wipe", "One colour wipes over the other along the strip, then back.", "Speed", "Hold (0: none, 255: three quarters of the cycle)", True,
     ["Colour 1", "Colour 2"], "Colours (0-127: Colour 1 and 2 alternate, 128-255: a new palette colour each wipe)", None,
     (128, 64, 255, (255, 0, 0), (0, 150, 0), (0, 0, 0), 0, 0), 0, "own"),
    (29, "Stripes", "Hard stripes of colour 1 and colour 2 move along the strip (candy cane).", "Speed (0: still)",
     "Stripe size (higher: finer)", False, ["Colour 1", "Colour 2"], "Share of Colour 1 (128: half)", None,
     (128, 190, 255, (255, 0, 0), W, (0, 0, 0), 128, 0), 0, "own"),
    (30, "Bulbs", "A string of coloured bulbs; the colours hop along it.", "Shift speed (0: still)",
     "Spacing (higher: closer bulbs)", True, [None, "Unlit"], "Bulb size (share of the spacing, 128: half)",
     "Colour step (64: a quarter of the palette)", (40, 200, 255, W, (0, 0, 0), (0, 0, 0), 128, 64), 0, "own"),
    (31, "Lava Lamp", "Big soft blobs of palette colour drift and merge.", "Speed", "Blob size", True, [None, "Background"],
     None, None, (128, 200, 255, W, (0, 0, 0), (0, 0, 0), 0, 0), 0, "own"),
    (32, "Mosaic Drift", "Fixed adjoining tiles that slowly melt from one colour to the next.", "Colour change speed",
     "Tile size (higher: smaller tiles)", True, [], "Whiten (mixes neutral white in; 0: none)", None,
     (128, 180, 1, W, (0, 0, 0), (0, 0, 0), 90, 0), 0, "own"),
    (33, "Silk Weave", "Two broad ribbons of light slide through each other and turn pearly where they cross.", "Speed",
     "Ribbon spacing (higher: closer)", False, ["Ribbon 1", "Ribbon 2", "Ground"], "White at the crossings", None,
     (128, 100, 255, (255, 150, 110), (110, 160, 255), (20, 12, 14), 130, 0), 0, "own"),
    (34, "Canopy Shade", "A pale wash with soft leaf-like shadows that breathe and lean a little.", "Sway speed", "Shadow depth",
     False, ["Wash", "Shadow"], "Leaf spacing (higher: closer)", None, (100, 180, 255, (255, 236, 200), (40, 70, 30), (0, 0, 0), 150, 0),
     0, "own"),
    (35, "Opal Facets", "Fixed glass-like facets in palette colours; a pearly reflection rolls across each one.", "Reflection speed",
     "Facet width (higher: narrower)", True, [], "Reflection strength", None, (110, 170, 255, W, (0, 0, 0), (0, 0, 0), 190, 0), 0, "own"),
    (36, "Quiet Landings", "One to three soft pearls rise and fall in arcs and touch down without a trail.", "Bounce pace",
     "Height (share of the strip)", True, [None, "Ground"], "Pearls (0-85: one, 86-170: two, 171-255: three)", None,
     (128, 160, 255, W, (0, 0, 0), (0, 0, 0), 128, 0), 0, "own"),
]
EFFECT_PAL_DEFAULT = {5: 10, 6: 10, 8: 0, 9: 3, 10: 10, 13: 17, 14: 10, 15: 0, 16: 14, 19: 19, 20: 19, 21: 0, 23: 18,
                      24: 10, 25: 13, 27: 24, 28: 23, 30: 3, 31: 16, 32: 1, 35: 27, 36: 24}


def q16():
    return [int(round(32767 * math.sin(math.pi / 2 * k / 64))) for k in range(65)] + [32767]


def cpal(name, p):
    L = ["    { /* %s */" % name]
    for k in range(0, 16, 4):
        L.append("        " + " ".join("{%3d, %3d, %3d}," % c for c in p[k:k + 4]))
    L.append("    },")
    return L


def q565(c):
    """the nearest RGB565 value of (r, g, b) 0..255 (the decoder replicates the top bits: 31 -> 255, 63 -> 255)"""
    def best(v, bits):
        top = (1 << bits) - 1
        def ex(q):
            return (q << (8 - bits)) | (q >> (2 * bits - 8))
        return min(range(top + 1), key=lambda q: (abs(ex(q) - v), q))
    return (best(c[0], 5) << 11) | (best(c[1], 6) << 5) | best(c[2], 5)


def un565(v):
    r, g, b = v >> 11, (v >> 5) & 63, v & 31
    return ((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2))


def cpal565(name, p):
    L = ["    { /* %s */" % name]
    for k in range(0, 16, 8):
        L.append("        " + " ".join("0x%04X," % q565(c) for c in p[k:k + 8]))
    L.append("    },")
    return L


def pal_error(p):
    """the largest channel error (0..255) of the 565 palette against the source entries"""
    return max(abs(a - b) for c in p for a, b in zip(un565(q565(c)), c))


def write_tables():
    for pid, (i, name, p, origin, cyc) in enumerate(PALETTES):
        assert i == pid and len(p) == 16, name
    L = ["/* generated by tools/gen_fx_tables.py: don't edit */", '#include "fx_int.h"', "",
         "const uint16_t fx_qsin16[66] = {" + ", ".join(map(str, q16())) + "};", ""]
    g16 = []
    for j in range(256):
        g = int(round(65280 * (j / 255) ** 2.8))
        want = int(round(255 * (j / 255) ** 2.8))          # the 8-bit table of 0.6.12: (g + 128) >> 8 must be it at every level
        g = min(max(g, want * 256 - 128), want * 256 + 127)
        g16.append(g)
    L += ["/* gamma 2.8 at 16 bits: round(65280 * (v / 255) ^ 2.8), 65280 = 255 * 256, moved by a step where (value + 128) >> 8 would miss round(255 * (v / 255) ^ 2.8) (fx_pack.c) */",
          "const uint16_t fx_gamma16[256] = {"]
    for k in range(0, 256, 16):
        L.append("    " + ", ".join("%5d" % x for x in g16[k:k + 16]) + ",")
    L += ["};", ""]
    L += ["/* the palettes' 16 entries as RGB565 (fx_unpack565 in fx_int.h): the largest channel error against the source stops is %d of 255 */" %
          max(pal_error(p) for i, n, p, o, c in PALETTES), "const uint16_t fx_pals565[PAL_COUNT][16] = {"]
    for i, name, p, origin, cyc in PALETTES:
        L += cpal565(name, p)
    L += ["};", "", "const rgb_t fx_pal_pacifica[3][16] = {"]
    for k, p in enumerate(PACIFICA):
        L += cpal("pacifica %d" % (k + 1), p)
    L += ["};", "", "const uint8_t fx_pal_default_tab[FX_COUNT] = {"]
    d = [EFFECT_PAL_DEFAULT.get(i, 0) for i in range(len(EFFECTS) + 1)]
    L.append("    " + ", ".join(map(str, d)) + ",")
    L.append("};")
    with open(os.path.join(SRC, "fx_tables.c"), "w", newline="\n") as f:
        f.write("\n".join(L) + "\n")


def write_catalogue(copies):
    def c3(c):
        return list(c)
    effects = []
    for (i, name, summary, sp, it, pal, cols, c1, c2, dflt, st, origin) in EFFECTS:
        d = dict(zip(("speed", "intensity", "palette", "c1", "c2", "c3", "custom1", "custom2"), dflt))
        d["palette"] = EFFECT_PAL_DEFAULT.get(i, 255) if pal else 255
        for k in ("c1", "c2", "c3"):
            d[k] = c3(d[k])
        effects.append(dict(id=i, name=name, summary=summary,
                            colors=sum(1 for c in cols if c), color_slots=(list(cols) + [None, None, None])[:3],
                            speed=sp is not None, speed_label=sp, intensity=it is not None, intensity_label=it,
                            palette=pal, custom1=c1, custom2=c2,
                            defaults=d, state_bytes_per_pixel=st, origin=origin))
    pals = [dict(id=i, name=n, origin=o, cyclic=c) for i, n, p, o, c in PALETTES]
    cat = dict(catalogue_version=3,
               note="ids are never reused; 1-127 official, 128-255 custom. palette 254 = colour 1 -> 2 -> 3, 255 = the "
                    "effect's own default. colors = how many of c1..c3 the effect uses, color_slots = their labels by position (null: that slot is unused); speed / intensity / palette: true when used; the "
                    "labels say what they do; custom1 / custom2: null or a label. A retired id would stay with retired: true.",
               palette_special=dict(colors=254, default=255), effects=effects, palettes=pals)
    for path in copies:
        with open(path, "w", newline="\n") as f:
            json.dump(cat, f, indent=1)
            f.write("\n")


def write_defaults():
    """test/fx_defaults.inc: the catalogue's defaults as C initialisers (the host tests and fx_render.c)"""
    L = ["/* generated by tools/gen_fx_tables.py: don't edit. {id, speed, intensity, palette, c1, c2, c3, custom1, custom2} */"]
    for (i, name, summary, sp, it, pal, cols, c1, c2, d, st, origin) in EFFECTS:
        pid = EFFECT_PAL_DEFAULT.get(i, 255) if pal else 255
        L.append("{ %d, %d, %d, %d, {%s}, {%s}, {%s}, %d, %d }, /* %s */" % (
            i, d[0], d[1], pid, ", ".join(map(str, d[3])), ", ".join(map(str, d[4])), ", ".join(map(str, d[5])),
            d[6], d[7], name))
    with open(os.path.join(TEST, "fx_defaults.inc"), "w", newline="\n") as f:
        f.write("\n".join(L) + "\n")


def check():
    """regenerate into a temporary folder and compare with the files in the tree: 0 = up to date"""
    global SRC, TEST
    import tempfile
    real = (SRC, TEST)
    bad = 0
    with tempfile.TemporaryDirectory() as d:
        SRC = TEST = d
        write_tables()
        write_defaults()
        write_catalogue([os.path.join(d, "fx_catalogue.json")])
        SRC, TEST = real
        for name, dest in (("fx_tables.c", SRC), ("fx_defaults.inc", TEST), ("fx_catalogue.json", SRC)):
            with open(os.path.join(d, name), "rb") as f1, open(os.path.join(dest, name), "rb") as f2:
                if f1.read().replace(CRLF, LF) != f2.read().replace(CRLF, LF):
                    print("%s is out of date: run tools/gen_fx_tables.py" % name)
                    bad = 1
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--copy", action="append", default=[], help="also write the catalogue here")
    ap.add_argument("--check", action="store_true", help="only check that the generated files are up to date")
    a = ap.parse_args()
    if a.check:
        sys.exit(check())
    write_tables()
    write_defaults()
    write_catalogue([os.path.join(SRC, "fx_catalogue.json")] + a.copy)


if __name__ == "__main__":
    main()
