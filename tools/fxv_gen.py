# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 Digital-M profile of tools/gen_fx_tables.py: effects 37+ and palettes 31+ (src/core/fx_v407/README.md).

Inputs (firmware-owned, never written by the generation step):
  src/core/fx_v407/catalogue_v407.json   the approved catalogue (114 effects, 49 palettes, design objects, ha_group);
                                          --import <path> copies an approved file in after validating it
  src/core/fx_v407/manifest/*.json        one manifest per effect batch: which ids the batch implements, their C / JS
                                          symbols and metadata (tick, particles, flashy, white, modeled cost)
  src/core/fx_v407/pal_notes/*.json       per batch (legacy.json: 1-36) every effect's palette_slots and palette_note
                                          (tools/gen_fx_tables.py load_pal_notes); merged into the ESPHome catalogue
Outputs (generated: never hand-merge, regenerate after merging batches):
  src/core/fx_v407/fxv_gen.h, fxv_gen.c  ids, counts, the registry (only built ids), metadata, default palettes, exact
                                          palettes, cyclic flags, capability maps (page 0x11), the kelvin table
  src/core/fx_v407/fxv_gen.json          the same data for the JS assembler (dashboards-r13/v3/digital/fx/v407) and the
                                          C <-> JS golden case generator
  src/core/fx_v407/pixel_fx_catalogue_v407.json   the ESPHome-facing catalogue: the approved one plus the resolved
                                          default_palette_id (or default_palette "own"), palette_slots / palette_note and
                                          flashy flag of every effect, and the built map

    python tools/gen_fx_tables.py --profile v407 [--check] [--import <approved catalogue>] [--strict-notes]

--strict-notes: fail when an effect has no palette note yet (the integration; while batches work, a missing note is
listed and the effect's catalogue entry goes without palette_slots / palette_note).

The legacy (V203) outputs of gen_fx_tables.py are not touched by this profile."""
import glob
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
FXV = os.path.normpath(os.path.join(HERE, "..", "src", "core", "fx_v407"))
CAT_IN = os.path.join(FXV, "catalogue_v407.json")
MANIFESTS = os.path.join(FXV, "manifest")
LEGACY_FX = 37                  # ids 0..36 (0 = the stream) are the legacy engine's (fx.h FX_COUNT)
LEGACY_PAL = 31                 # palettes 0..30 are the legacy RGB565 tables (fx.h PAL_COUNT)
MAP_IDS = 192                   # page 0x11 parts 1 / 2: 24 bytes, ids 0..191
BATCHES = ("b1_analytic", "b2_surfaces", "b3_natural_particles", "b4_mechanics", "b5_cell_transport", "b6_vignettes",
           "b7_bursts")
# Plan section 5: the ids each batch owns (a manifest may list only its own)
BATCH_IDS = {
    "b1_analytic": (37, 38, 39, 43, 44, 47, 52, 54, 63, 76, 77, 79, 80, 97, 98, 99, 101, 109, 114),
    "b2_surfaces": (40, 41, 42, 48, 49, 51, 65, 66, 82, 96),
    "b3_natural_particles": (45, 46, 50, 53, 56, 57, 67, 75, 90, 107, 112),
    "b4_mechanics": (55, 61, 62, 68, 78, 81, 86, 87, 92),
    "b5_cell_transport": (64, 69, 71, 72, 73, 74, 84, 88, 89, 91, 105),
    "b6_vignettes": (58, 59, 60, 70, 83, 85, 93, 100, 104, 108, 110, 111),
    "b7_bursts": (94, 95, 102, 103, 106, 113),
}
FLASHY_REQUIRED = {113}         # Countdown keeps its single finale flash: it must carry the flag (PLAN section 6)
# modeled cycles of the legacy effects through the adapter (render + 16-bit expand, per pixel; per frame);
# measured with tools/measure_fxv.py where marked, the rest a conservative common figure
LEGACY_COST_PX = {4: 90, 6: 230, 7: 230, 8: 270, 9: 320, 10: 270, 12: 310, 13: 340, 16: 170, 18: 760, 24: 260, 25: 300, 30: 220, 31: 300, 32: 280, 33: 320, 34: 220}   # (measure_fxv.py: 18 701, 13 307, 16 167, 4 83)
LEGACY_COST_PX_DEFAULT = 200
PACK_COST_PX = 280              # fxv_pack16 per pixel (tools/measure_fxv.py: 231-273 colour at 3-5 bytes)
# effects 1-36 on the V407 adapter: a render + pack step's worst cycles a pixel (tools/measure_fxv_slices.py, every corner
# and format, forward and reversed + dimmed, 400 cycles of headroom); c_step = the larger of this and the frame model's px
LEGACY_STEP = {1: 384, 2: 384, 3: 384, 4: 384, 5: 454, 6: 555, 7: 555, 8: 625, 9: 625, 10: 625, 11: 384, 12: 625, 13: 714,
               14: 500, 15: 500, 16: 500, 17: 454, 18: 384, 19: 454, 20: 454, 21: 500, 22: 500, 23: 625, 24: 625, 25: 714,
               26: 500, 27: 555, 28: 454, 29: 500, 30: 555, 31: 714, 32: 714, 33: 625, 34: 555, 35: 555, 36: 555}
PACK_COST_WHITE_PX = 560        # ... a natural-white pixel on RGBW / RGBCCT (379-557)
# effects 1-36 on a chosen palette where that costs more than their Effect default (the own-colour ones; render cycles a
# pixel as LEGACY_COST_PX, the step's worst as LEGACY_STEP; tools/measure_fxv*.py --palette 10 / 254, the worse of both)
LEGACY_COST_PX_PAL = {1: 202, 2: 203, 4: 124, 7: 222, 11: 203, 12: 381, 18: 940, 26: 241, 33: 457, 34: 310}   # (18: its render-ahead advance)
LEGACY_STEP_PAL = {1: 519, 2: 520, 4: 415, 11: 520, 12: 718, 17: 482, 22: 551, 26: 552, 33: 801, 34: 629}
# effects 1-36: modeled cycles of a frame's advance (prep + slices), on a chosen palette where more
LEGACY_COST_FRAME = {6: 780, 7: 780, 11: 490, 16: 27600, 18: 2860, 28: 460, 31: 500}
LEGACY_COST_FRAME_DEFAULT = 400
LEGACY_COST_FRAME_PAL = {18: 3200}


def kelvin_srgb(kelvin):
    """Tanner Helland's blackbody fit (sRGB-encoded, 0..255 floats)"""
    t = kelvin / 100.0
    if t <= 66:
        r = 255.0
        g = 99.4708025861 * math.log(t) - 161.1195681661
        b = 0.0 if t <= 19 else 138.5177312231 * math.log(t - 10) - 305.0447927307
    else:
        r = 329.698727446 * ((t - 60) ** -0.1332047592)
        g = 288.1221695283 * ((t - 60) ** -0.0755148492)
        b = 255.0
    return [min(255.0, max(0.0, v)) for v in (r, g, b)]


def kelvin_table():
    """17 points, k = 16 j (k 0 = 2000 K, 255 = 6500 K, so point 16 = 6517 K), on the firmware's perceptual scale: the
    packer applies gamma 2.8, so an sRGB (2.2) value v becomes 255 (v / 255)^(2.2 / 2.8) and lands on the same light"""
    out = []
    for j in range(17):
        kel = 2000.0 + 4500.0 * (16 * j) / 255.0
        out.append([int(round(255.0 * (v / 255.0) ** (2.2 / 2.8))) for v in kelvin_srgb(kel)])
    return out


def fx_tables_c():
    with open(os.path.join(FXV, "..", "fx_tables.c"), encoding="utf-8") as f:
        return f.read()


def c_array(text, name):
    i = text.index(name)
    body = text[text.index("{", i) + 1:text.index("};", i)]
    return [int(t, 0) for t in body.replace(chr(10), " ").split(",") if t.strip() and not t.strip().startswith("/*")]


def legacy_gamma16():
    """fx_pack.c gamma16(v) for v 0..255, from the residual table in fx_tables.c (the C formula, bit for bit)"""
    q = c_array(fx_tables_c(), "fx_gamma_q[34]")
    out = []
    for v in range(256):
        p, t = v >> 3, v & 7
        a = q[p]
        d1, d2 = q[p + 1] - a, q[p + 2] - 2 * q[p + 1] + a
        y = ((a * 128 + d1 * 16 * t + d2 * t * (t - 8) + 64) >> 7) + v * 256
        out.append(max(0, y))
    return out


def legacy_palettes():
    """fx_effects.c pal_expand for palettes 0..30 from fx_pal_data (RGB565 stops, 8-bit blends), bit for bit"""
    text = fx_tables_c()
    i = text.index("fx_pal_data[]")
    body = text[text.index("{", i) + 1:text.index("};", i)]
    data = []
    for line in body.split(chr(10)):
        line = line.split("*/")[-1]
        data += [int(t, 16) for t in line.replace(",", " ").split()]
    def un565(v):
        r, g, b = v >> 11, (v >> 5) & 63, v & 31
        return [(r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)]
    def blend(a, b, f):
        g = f + (f >> 7)
        return [x + (((y - x) * g) >> 8) for x, y in zip(a, b)]
    pals, pos = [], 0
    for _ in range(LEGACY_PAL):
        n = data[pos]
        pos += 1
        pal = [None] * 16
        if n == 0:
            for k in range(16):
                pal[k] = un565(data[pos] | data[pos + 1] << 8)
                pos += 2
        else:
            ca = un565(data[pos] | data[pos + 1] << 8)
            pos += 2
            a = 0
            for k in range(1, n):
                b = 15 if k == n - 1 else data[pos]
                if k != n - 1:
                    pos += 1
                cb = un565(data[pos] | data[pos + 1] << 8)
                pos += 2
                for e in range(a, b + 1):
                    pal[e] = blend(ca, cb, ((e - a) * 255) // (b - a))
                a, ca = b, cb
        pals.append(pal)
    assert pos == len(data), "fx_pal_data parse"
    return pals


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def validate_catalogue(cat):
    effects = {e["id"]: e for e in cat["effects"]}
    pals = {p["id"]: p for p in cat["palettes"]}
    assert sorted(effects) == list(range(1, max(effects) + 1)), "effect ids must be 1..N without gaps"
    assert sorted(pals) == list(range(0, max(pals) + 1)), "palette ids must be 0..N without gaps"
    assert max(effects) < MAP_IDS and max(pals) < MAP_IDS
    order = cat["ha_group_order"]
    for i, e in effects.items():
        assert e.get("ha_group") in order and e["ha_group"] != "Other", "effect %d: ha_group" % i
        assert "palette" not in e, "effect %d: no per-effect palette flag (every effect takes a palette)" % i
        if e.get("default_palette") is not None:
            assert e["default_palette"] == "own" and e["defaults"]["palette"] == 255, "effect %d: default_palette" % i
            assert i < LEGACY_FX or e["design"].get("default_palette_id") is None, "effect %d: own and an id" % i
        if i >= LEGACY_FX:
            assert "design" in e, "effect %d: design object" % i
            dp = e["design"].get("default_palette_id")
            assert dp is None or dp in pals, "effect %d: default_palette_id" % i
    for i, p in pals.items():
        if i >= LEGACY_PAL:
            st = p["stops"]
            assert len(st) == 16 and all(len(c) == 3 and all(0 <= v <= 255 and v == int(v) for v in c) for c in st), \
                "palette %d: 16 RGB stops" % i
        assert isinstance(p.get("cyclic"), bool), "palette %d: cyclic" % i
    return effects, pals


def default_palette(e, legacy_tab):
    i = e["id"]
    if i < LEGACY_FX:
        return legacy_tab[i]
    dp = e["design"].get("default_palette_id")
    if dp is not None:
        return dp
    if e.get("default_palette") == "own":
        return 255                           # Effect default = the effect's own colours (pal_own)
    p = e["defaults"]["palette"]
    return p if p < 254 else 0


def legacy_default_tab():
    sys.path.insert(0, HERE)
    import gen_fx_tables as g                # the legacy table: EFFECT_PAL_DEFAULT (own colours: 255)
    return [0] + [g.EFFECT_PAL_DEFAULT.get(i, 255) for i in range(1, LEGACY_FX)]


def load_manifests(effects):
    out, seen = [], {}
    for path in sorted(glob.glob(os.path.join(MANIFESTS, "*.json"))):
        m = load(path)
        b = m["batch"]
        assert b in BATCHES and os.path.basename(path) == b + ".json", "%s: batch name" % path
        assert m["c"] == b + ".c" and m["js"] == b + ".js", "%s: module names" % path
        for f in m["effects"]:
            i = f["id"]
            assert i in BATCH_IDS[b], "%s: id %d belongs to another batch" % (path, i)
            assert i not in seen, "id %d in two manifests" % i
            assert f["symbol"].isidentifier() and f["symbol"] == f["symbol"].lower(), "id %d: symbol" % i
            for k in ("built", "flashy", "white"):
                assert isinstance(f.get(k), bool), "id %d: %s" % (i, k)
            assert isinstance(f.get("hold", False), bool), "id %d: hold" % i
            assert 0 <= f.get("tick_ms", 0) <= 255 and 0 <= f.get("particles", 0) <= 512
            if f["particles"]:
                assert f["particles"] <= max(effects[i]["design"]["particles"], 1) * 2, "id %d: particles vs design" % i
            c = f["cost"]
            assert all(0 <= c[k] < 65536 for k in ("px", "tick", "particle")), "id %d: cost" % i
            assert all(0 <= c.get(k, 0) < 2 ** 32 for k in ("frame", "frame_pal")), "id %d: cost frame" % i   # (uint32_t c_frame)
            assert 0 <= c.get("step", 0) < 65536, "id %d: cost step" % i
            assert set(c) <= {"frame", "px", "tick", "particle", "step", "frame_pal", "px_pal", "step_pal"}, "id %d: cost keys" % i
            assert all(0 <= c[k] < 65536 for k in ("px_pal", "step_pal") if k in c), "id %d: cost *_pal" % i
            seen[i] = dict(f, batch=b)
        out.append(m)
    for i in FLASHY_REQUIRED:
        if i in seen and seen[i]["built"]:
            assert seen[i]["flashy"], "id %d must be flagged flashy" % i
    return out, seen


def bits(ids):
    m = [0] * (MAP_IDS // 8)
    for i in ids:
        m[i >> 3] |= 1 << (i & 7)
    return m


def build(cat):
    effects, pals = validate_catalogue(cat)
    legacy_tab = legacy_default_tab()
    manifests, man = load_manifests(effects)
    n_fx, n_pal = max(effects) + 1, max(pals) + 1
    built = sorted(i for i, f in man.items() if f["built"])
    eff_map = bits(list(range(1, LEGACY_FX)) + built)
    pal_map = bits(range(n_pal))
    defaults = [0] + [default_palette(effects[i], legacy_tab) for i in range(1, n_fx)]
    meta = []
    for i in range(n_fx):
        if i == 0:
            meta.append(dict(tick_ms=0, particles=0, flags=0, frame=0, px=0, tick=0, particle=0, step=0,
                             frame_pal=0, px_pal=0, step_pal=0))
        elif i < LEGACY_FX:
            px = LEGACY_COST_PX.get(i, LEGACY_COST_PX_DEFAULT) + PACK_COST_PX
            px_pal = max(LEGACY_COST_PX.get(i, LEGACY_COST_PX_DEFAULT), LEGACY_COST_PX_PAL.get(i, 0)) + PACK_COST_PX
            frame = LEGACY_COST_FRAME.get(i, LEGACY_COST_FRAME_DEFAULT)
            meta.append(dict(tick_ms=0, particles=0, flags=0x80, frame=frame, px=px, tick=0, particle=0,
                             step=max(px, LEGACY_STEP[i]), frame_pal=max(frame, LEGACY_COST_FRAME_PAL.get(i, 0)), px_pal=px_pal,
                             step_pal=max(px_pal, LEGACY_STEP[i], LEGACY_STEP_PAL.get(i, 0))))
        elif i in man and man[i]["built"]:
            f = man[i]
            c = f["cost"]
            px = c["px"] + (PACK_COST_WHITE_PX if f["white"] else PACK_COST_PX)
            px_pal = max(c["px"], c.get("px_pal", 0)) + (PACK_COST_WHITE_PX if f["white"] else PACK_COST_PX)
            # step: one render + pack step's worst cycles a pixel (any format; fxv_engine_span sizes a step by it), when measured
            # above the frame model's px (tools/measure_fxv_slices.py); else px
            meta.append(dict(tick_ms=f.get("tick_ms", 0), particles=f.get("particles", 0),
                             flags=(1 if f["flashy"] else 0) | (2 if f["white"] else 0) | (4 if f.get("hold") else 0),
                             frame=f["cost"]["frame"],
                             px=px, tick=f["cost"]["tick"], particle=f["cost"]["particle"],
                             step=max(px, f["cost"].get("step", 0)),
                             # palette mode (pal_own 0): the manifest's *_pal where it costs more, never below the own
                             frame_pal=max(c["frame"], c.get("frame_pal", 0)), px_pal=px_pal,
                             step_pal=max(px_pal, c.get("step", 0), c.get("step_pal", 0))))
        else:
            meta.append(dict(tick_ms=0, particles=0, flags=0, frame=0, px=0, tick=0, particle=0, step=0,
                             frame_pal=0, px_pal=0, step_pal=0))
    flashy = [i for i in built if man[i]["flashy"]]
    data = dict(
        profile="v407", catalogue_version=int(str(cat["catalogue_version"]).split("+")[0]),
        catalogue_version_text=str(cat["catalogue_version"]),
        catalogue_board=cat.get("catalogue_version_board", ""),
        catalogue_sha256=hashlib.sha256(json.dumps(cat, sort_keys=True).encode()).hexdigest(),
        fx_count=n_fx, pal_count=n_pal, legacy_fx=LEGACY_FX, legacy_pal=LEGACY_PAL,
        built=built, flashy=flashy, effect_map=eff_map, palette_map=pal_map,
        default_palette=defaults, cyclic=[1 if pals[i]["cyclic"] else 0 for i in range(n_pal)],
        exact_palettes=[pals[i]["stops"] for i in range(LEGACY_PAL, n_pal)],
        kelvin=kelvin_table(), meta=meta, gamma16=legacy_gamma16(), legacy_palettes=legacy_palettes(),
        batches=[dict(batch=m["batch"], c=m["c"], js=m["js"],
                      effects=[dict(id=f["id"], symbol=f["symbol"], built=f["built"]) for f in m["effects"]])
                 for m in manifests],
        names={str(i): effects[i]["name"] for i in range(1, n_fx)},
        defaults={str(i): dict(effects[i]["defaults"], palette=255 if effects[i]["defaults"]["palette"] == 255 else
                                effects[i]["defaults"]["palette"]) for i in range(1, n_fx)},
        groups={str(i): effects[i]["ha_group"] for i in range(1, n_fx)},
    )
    return data, effects


def c_bytes(vals, per=16, fmt="%d"):
    rows = []
    for k in range(0, len(vals), per):
        rows.append("    " + ", ".join(fmt % v for v in vals[k:k + per]) + ",")
    return rows


def write_c(data, dest):
    n_fx, n_pal = data["fx_count"], data["pal_count"]
    syms = []
    for b in data["batches"]:
        for f in b["effects"]:
            if f["built"]:
                syms.append((f["id"], f["symbol"]))
    syms.sort()
    H = ["/* SPDX-License-Identifier: GPL-3.0-or-later", " * Copyright (C) 2026 QuinLED */", "/* generated by tools/gen_fx_tables.py --profile v407 from catalogue_v407.json and the manifests: don't edit */",
         "#ifndef FXV_GEN_H", "#define FXV_GEN_H", "", "#include \"fxv_api.h\"", "",
         "#define FXV_FX_COUNT      %du                  /* effect ids 0..%d (0 = the stream) */" % (n_fx, n_fx - 1),
         "#define FXV_PAL_COUNT     %du                   /* palettes 0..%d */" % (n_pal, n_pal - 1),
         "#define FXV_PAL_LEGACY    %du                   /* 0..%d: the legacy RGB565 tables; the rest exact 16-stop */" % (
             LEGACY_PAL, LEGACY_PAL - 1),
         "#define FXV_CATALOGUE_VERSION %du" % data["catalogue_version"],
         "#define FXV_BUILT_NEW     %du                   /* effects 37+ built in this tree */" % len(data["built"]), "",
         "typedef struct {",
         "    uint16_t particles;                   /* design cap of particle records */",
         "    uint8_t  tick_ms;                     /* simulation tick, effect ms (0: none) */",
         "    uint8_t  flags;                       /* FXV_D_* */",
         "    uint32_t c_frame;                     /* modeled cycles: per frame (the advance; 32 bits: Plasma Globe all-max ~120,000) */",
         "    uint16_t c_px, c_tick, c_part;        /* ... per pixel (render + pack), per tick, per particle tick */",
         "    uint16_t c_step;                      /* one render + pack step's worst cycles a pixel (fxv_engine_span sizes a step by it; >= c_px) */",
         "    uint32_t c_frame_pal;                 /* the same on a chosen palette (pal_own 0; the manifest's *_pal, else equal) */",
         "    uint16_t c_px_pal, c_step_pal;",
         "} fxv_meta_t;", "",
         "extern const fxv_desc_t *const fxv_registry[FXV_FX_COUNT - FX_COUNT];   /* ids 37.., 0: not built */",
         "#ifndef FXV_META_CONST",
         "#define FXV_META_CONST const                /* (host tests and the golden oracle build it mutable: stand-in owners) */",
         "#endif",
         "extern FXV_META_CONST fxv_meta_t fxv_meta[FXV_FX_COUNT];",
         "extern const uint8_t fxv_pal_default_tab[FXV_FX_COUNT];",
         "extern const uint8_t fxv_pal_exact[FXV_PAL_COUNT - FXV_PAL_LEGACY][16][3];",
         "extern const uint8_t fxv_pal_cyclic[(FXV_PAL_COUNT + 7u) / 8u];",
         "extern const uint8_t fxv_map_effects[24], fxv_map_palettes[24], fxv_map_flashy[24];",
         "extern const uint8_t fxv_kelvin_tab[17][3];",
         "extern const uint16_t fxv_gamma_tab[257];        /* fx_pack.c's gamma16 of 0..255 (+ the last again): the packer's curve */", ""]
    for i, s in syms:
        H.append("extern const fxv_desc_t fxv_d_%s;   /* %d %s */" % (s, i, data["names"][str(i)]))
    H += ["", "#endif", ""]
    C = ["/* SPDX-License-Identifier: GPL-3.0-or-later", " * Copyright (C) 2026 QuinLED */", "/* generated by tools/gen_fx_tables.py --profile v407 from catalogue_v407.json and the manifests: don't edit */",
         "#include \"fxv_gen.h\"", "",
         "const fxv_desc_t *const fxv_registry[FXV_FX_COUNT - FX_COUNT] = {"]
    by = dict(syms)
    for i in range(LEGACY_FX, n_fx):
        C.append("    %s,   /* %d %s */" % ("&fxv_d_" + by[i] if i in by else "0", i, data["names"][str(i)]))
    C += ["};", "", "FXV_META_CONST fxv_meta_t fxv_meta[FXV_FX_COUNT] = {"]
    for i, m in enumerate(data["meta"]):
        C.append("    { %d, %d, 0x%02X, %d, %d, %d, %d, %d, %d, %d, %d },%s" % (
            m["particles"], m["tick_ms"], m["flags"], m["frame"], m["px"], m["tick"], m["particle"], m["step"],
            m["frame_pal"], m["px_pal"], m["step_pal"], "" if i == 0 else "   /* %d */" % i))
    C += ["};", "", "const uint8_t fxv_pal_default_tab[FXV_FX_COUNT] = {"]
    C += c_bytes(data["default_palette"], 20)
    C += ["};", "", "const uint8_t fxv_pal_exact[FXV_PAL_COUNT - FXV_PAL_LEGACY][16][3] = {"]
    for k, st in enumerate(data["exact_palettes"]):
        C.append("    { /* %d */ %s }," % (LEGACY_PAL + k, ", ".join("{%d, %d, %d}" % tuple(c) for c in st)))
    cyc = [0] * ((n_pal + 7) // 8)
    for i, c in enumerate(data["cyclic"]):
        cyc[i >> 3] |= c << (i & 7)
    C += ["};", "", "const uint8_t fxv_pal_cyclic[(FXV_PAL_COUNT + 7u) / 8u] = {"] + c_bytes(cyc, 16, "0x%02X") + ["};", ""]
    C += ["const uint8_t fxv_map_effects[24] = {"] + c_bytes(data["effect_map"], 12, "0x%02X") + ["};"]
    C += ["const uint8_t fxv_map_palettes[24] = {"] + c_bytes(data["palette_map"], 12, "0x%02X") + ["};"]
    C += ["const uint8_t fxv_map_flashy[24] = {"] + c_bytes(bits(data["flashy"]), 12, "0x%02X") + ["};", ""]
    C += ["/* natural white, k = 16 j (2000 K .. 6517 K), perceptual for the packer's gamma 2.8 */",
          "const uint8_t fxv_kelvin_tab[17][3] = {"]
    C += ["    " + ", ".join("{%d, %d, %d}" % tuple(c) for c in data["kelvin"][k:k + 6]) + "," for k in range(0, 17, 6)]
    C += ["};", ""]
    g = data["gamma16"] + data["gamma16"][-1:]
    C += ["/* gamma 2.8 at 16 bits of 0..255 (fx_pack.c's gamma16, bit for bit) and entry 256 = entry 255 (the interpolation's end) */",
          "const uint16_t fxv_gamma_tab[257] = {"] + c_bytes(g, 16) + ["};", ""]
    with open(os.path.join(dest, "fxv_gen.h"), "w", newline="\n") as f:
        f.write("\n".join(H))
    with open(os.path.join(dest, "fxv_gen.c"), "w", newline="\n") as f:
        f.write("\n".join(C))


def write_json(data, dest):
    with open(os.path.join(dest, "fxv_gen.json"), "w", newline="\n") as f:
        json.dump(data, f, indent=1, sort_keys=True)
        f.write("\n")


STRICT_NOTES = False


def write_esphome(cat, data, dest):
    """the approved catalogue plus the resolved default palette (default_palette_id, or default_palette "own"), the palette
    notes (pal_notes/*.json) and flashy flag of every effect, and the built map"""
    sys.path.insert(0, HERE)
    import gen_fx_tables as g
    out = json.loads(json.dumps(cat))
    out["catalogue_version"] = data["catalogue_version"]          # ESPHome's catalogue.py: the integer protocol version,
    out["board_catalogue_version"] = data["catalogue_board"]      # the board revision text beside it (as wire3's copy)
    flashy = set(data["flashy"])
    notes = g.load_pal_notes({e["id"]: [c for c in e["color_slots"] if c] for e in out["effects"]})
    missing = [e["id"] for e in out["effects"] if e["id"] not in notes]
    if missing:
        print("palette notes missing (src/core/fx_v407/pal_notes): %s" % " ".join(map(str, missing)))
        assert not STRICT_NOTES, "--strict-notes: every effect needs palette_slots + palette_note"
    for e in out["effects"]:
        i = e["id"]
        rest = {k: e.pop(k) for k in list(e) if k in ("custom1", "custom2", "defaults", "state_bytes_per_pixel", "origin",
                                                      "design", "ha_group")}
        e.pop("default_palette", None)
        if data["default_palette"][i] == 255:
            e["default_palette"] = "own"
        else:
            e["default_palette_id"] = data["default_palette"][i]
        if i in notes:
            e["palette_slots"], e["palette_note"] = notes[i]
        e.update(rest)
        if i >= data["legacy_fx"]:
            e["flashy"] = i in flashy
    out["firmware_built"] = dict(effects="".join("%02x" % b for b in data["effect_map"]),
                                 palettes="".join("%02x" % b for b in data["palette_map"]))
    with open(os.path.join(dest, "pixel_fx_catalogue_v407.json"), "w", newline="\n", encoding="utf-8") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
        f.write("\n")


def write_defaults(cat, dest):
    """the catalogue's defaults as C initialisers for the host tools (test/fxv_flash_check.c):
    {id, speed, intensity, palette, c1, c2, c3, custom1, custom2}"""
    L = ["/* SPDX-License-Identifier: GPL-3.0-or-later", " * Copyright (C) 2026 QuinLED */", "/* generated by tools/gen_fx_tables.py --profile v407: don't edit. {id, speed, intensity, palette, c1, c2, c3, custom1, custom2} */"]
    for e in cat["effects"]:
        d = e["defaults"]
        L.append("{ %d, %d, %d, %d, {%s}, {%s}, {%s}, %d, %d }, /* %s */" % (
            e["id"], d["speed"], d["intensity"], d["palette"], ", ".join(map(str, d["c1"])), ", ".join(map(str, d["c2"])),
            ", ".join(map(str, d["c3"])), d["custom1"], d["custom2"], e["name"]))
    with open(os.path.join(dest, "fxv_defaults.inc"), "w", newline="\n", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


OUTPUTS = ("fxv_gen.h", "fxv_gen.c", "fxv_gen.json", "pixel_fx_catalogue_v407.json", "fxv_defaults.inc")


def generate(dest):
    cat = load(CAT_IN)
    data, _ = build(cat)
    write_c(data, dest)
    write_json(data, dest)
    write_esphome(cat, data, dest)
    write_defaults(cat, dest)


def check():
    bad = 0
    with tempfile.TemporaryDirectory() as d:
        generate(d)
        for name in OUTPUTS:
            with open(os.path.join(d, name), "rb") as f1:
                new = f1.read().replace(b"\r\n", b"\n")
            try:
                with open(os.path.join(FXV, name), "rb") as f2:
                    old = f2.read().replace(b"\r\n", b"\n")
            except OSError:
                old = None
            if new != old:
                print("%s is out of date: run tools/gen_fx_tables.py --profile v407" % name)
                bad = 1
    return bad


def import_catalogue(path):
    cat = load(path)
    validate_catalogue(cat)
    shutil.copyfile(path, CAT_IN)
    print("imported %s -> %s" % (path, CAT_IN))


def main(argv):
    global STRICT_NOTES
    STRICT_NOTES = "--strict-notes" in argv
    if "--import" in argv:
        import_catalogue(argv[argv.index("--import") + 1])
    if "--check" in argv:
        return check()
    generate(FXV)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
