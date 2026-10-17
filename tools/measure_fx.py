# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Cycles and code sizes of the local-effects engine on the CH32V203 (RV32IMC): builds test/rv_fx_bench.c + src/core/fx*.c with
the xPack GCC 15.2 at the firmware's XW=0 flags (-Os -mtune=size, tools/build_target.sh's OPT, no LTO), runs the entry points in
tools/rv32emu.py (an instruction-class cycle model of the QingKe V4: ALU 1, load 2, taken branch / jump 2, div 17) and prints
cycles per pixel for every effect, the packer and the engine layer, the code size per effect and the RAM.

    python tools/measure_fx.py [--out build/measure_fx.txt] [--npx 300]    (FX_OPT=-O2 in the environment: -O2 instead of -Os)

Real builds and a real instruction-level emulator; the numbers are the model's (not a scope's). The XW=1 compiler (WCH GCC) is not
installed here: it makes the same code a little smaller."""
import argparse
import glob
import importlib.util
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
BUILD = os.path.join(FW, "build", "rv")
CORE = ["fx_math", "fx_effects", "fx_engine", "fx_pack", "fx_tables"]
OPT = (os.environ.get("FX_OPT", "-Os") + " -fno-partial-inlining -malign-data=natural --param=max-inline-insns-auto=5 --param=max-hoist-depth=0 "
       "--param=max-grow-copy-bb-insns=3 --param=gcse-unrestricted-cost=0 -fno-tree-sra -fno-jump-tables -fno-ipa-sra "
       "-fno-shrink-wrap-separate -fno-ipa-modref -ffunction-sections -fdata-sections -msmall-data-limit=400").split()

# effect id, name, the functions of its code (static ones that were inlined are simply absent)
EFFECTS = [
    (1, "Solid", ["fx_solid"]),
    (2, "Breathe", ["fx_breathe", "prep_breathe"]),
    (3, "Colorloop", ["fx_colorloop", "prep_colorloop"]),
    (4, "Rainbow", ["fx_rainbow", "prep_scroll"]),
    (5, "Palette", ["fx_palette"]),
    (6, "Colorwaves", ["fx_pride", "prep_pride"]),
    (7, "Pride 2015", []),
    (8, "Sinelon", ["fx_sinelon", "prep_sinelon", "dot_bri"]),
    (9, "Twinklefox", ["fx_twinklefox"]),
    (10, "Colortwinkles", ["twinkle", "fx_colortwinkles"]),
    (11, "Candle", ["fx_candle", "prep_candle", "flick", "flame"]),
    (12, "Candle Multi", ["fx_candle_multi"]),
    (13, "Aurora", ["fx_aurora", "prep_aurora"]),
    (14, "Blends", ["fx_blends", "prep_blends", "blend_cell"]),
    (15, "Flow", ["fx_flow", "prep_flow"]),
    (16, "Fire 2012", ["fx_fire", "prep_fire", "fire_step"]),
    (17, "Gradient", ["fx_gradient", "prep_gradient"]),
    (18, "Pacifica", ["fx_pacifica", "prep_pacifica", "pac_start", "pac_sum"]),
    (19, "Sunrise", ["fx_sun", "prep_sun"]),
    (20, "Sunset", []),
    (21, "Glitter", ["fx_glitter", "prep_glitter"]),
    (22, "Meteor", ["fx_meteor", "prep_meteor"]),
    (23, "Fairy", ["fx_fairy"]),
    (24, "Plasma", ["fx_plasma"]),
    (25, "Noise", ["fx_noise", "prep_noise"]),
    (26, "Running", ["fx_running", "prep_running"]),
    (27, "Ripples", ["fx_ripples"]),
    (28, "Wipe", ["fx_wipe", "prep_wipe"]),
    (29, "Stripes", ["fx_stripes", "prep_stripes", "window_cov", "period16"]),
    (30, "Bulbs", ["fx_bulbs", "prep_bulbs"]),
    (31, "Lava Lamp", ["fx_lava", "prep_lava"]),
    (32, "Mosaic Drift", ["fx_mosaic", "mosaic_tile", "prep_cells", "cell_step"]),
    (33, "Silk Weave", ["fx_silk", "prep_silk"]),
    (34, "Canopy Shade", ["fx_canopy"]),
    (35, "Opal Facets", ["fx_opal"]),
    (36, "Quiet Landings", ["fx_landings"]),
]
SHARED = {30: "(with Stripes: window_cov, period16)", 6: "(Colorwaves and Pride share fx_pride / prep_pride)", 7: "(with Colorwaves)", 20: "(with Sunrise)",
          23: "(with Colortwinkles: twinkle())"}
ENGINE = ["fx_engine_init", "fx_engine_set", "fx_engine_frame", "fx_engine_render", "fx_engine_dark", "seg_params",
          "drop_xfade", "begin_effect", "ramp256", "lerp256", "eff_index", "render_segment", "state_off",
          "fx_engine_frame_seg", "fx_engine_dequeue", "ramp_frame", "xfade_weight"]
CORE_FNS = ["fx_begin", "fx_frame", "fx_render", "fx_set_colors", "fx_set_speed", "fx_state_bytes", "fx_default_palette",
            "build_palette", "fx_rate256", "fx_defs"]
MATH = ["fx_sin16", "fx_sin8", "fx_nz1", "fx_nz1_begin", "nz_col", "lat", "fx_pal_at", "fx_pal_lin", "fx_hue", "fx_sat",
        "fx_blend", "fx_scale_rgb", "xorshift"]
PACK = ["fx_pack", "fx_limit_bri", "root28", "fx_gamma", "gamma_ai"]
TABLES = ["fx_qsin16", "fx_pals565", "fx_pal_pacifica", "fx_pal_default_tab"]


def tool(name):
    base = os.environ.get("LOCALAPPDATA", "")
    found = sorted(glob.glob(os.path.join(base, "xPacks", "xpack-riscv-none-elf-gcc-*", "bin", "riscv-none-elf-" + name + "*")))
    found = [f for f in found if not f.endswith("-gcc-ar.exe")]
    if not found:
        sys.exit("xPack riscv-none-elf-%s not found under %%LOCALAPPDATA%%/xPacks" % name)
    return found[0]


def build():
    os.makedirs(BUILD, exist_ok=True)
    gcc = tool("gcc")
    arch = ["-march=rv32imc_zicsr_zifencei", "-mabi=ilp32", "-mtune=size"]
    objs = []
    for f in CORE:
        o = os.path.join(BUILD, f + ".o")
        subprocess.run([gcc] + arch + OPT + ["-Wall", "-Wextra", "-std=gnu11", "-I", os.path.join(FW, "src", "core"), "-c",
                                             os.path.join(FW, "src", "core", f + ".c"), "-o", o], check=True, timeout=120)
        objs.append(o)
    elf = os.path.join(BUILD, "fx_bench.elf")
    r = subprocess.run([gcc] + arch + OPT + ["-std=gnu11", "-ffreestanding", "-nostdlib", "-nostartfiles", "-I",
                                             os.path.join(FW, "src", "core"), "-T", os.path.join(FW, "test", "fx_rv.ld"),
                                             "-Wl,--gc-sections", "-Wl,--entry=b_begin", "-o", elf,
                                             os.path.join(FW, "test", "rv_fx_bench.c")] + objs + ["-lgcc"],
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        sys.exit(r.stdout + r.stderr)
    return elf, objs


def obj_sizes(objs):
    out = {}
    for o in objs:
        r = subprocess.run([tool("size"), o], capture_output=True, text=True, check=True)
        text, data, bss = [int(v) for v in r.stdout.splitlines()[1].split()[:3]]
        out[os.path.basename(o)] = (text, data, bss)
    return out


def emu_module():
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location("rv32emu_fw", os.path.join(FW, "tools", "rv32emu.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=os.path.join(FW, "build", "measure_fx.txt"))
    ap.add_argument("--npx", type=int, default=300)
    a = ap.parse_args()
    n = a.npx
    elf, objs = build()
    m = emu_module()
    e = m.Emu(elf)
    L = []

    def p(s=""):
        L.append(s)
        print(s)

    def call(fn, args):
        e.call(fn, args)
        return e.cycles(), e.ret

    p("Local effects engine: cycles at 144 MHz (rv32emu model) and sizes; xPack GCC 15.2, XW=0 flags, -Os -mtune=size; %d px" % n)
    p("")
    p("1. Effects at the catalogue defaults: cycles per pixel per frame (the per-frame prep + the render in chunks of 32)")
    p("   %-14s %9s %9s %9s %9s   %s" % ("effect", "prep/frm", "render/px", "total/px", "worst/px", "% CPU at 31,250 px/s (RGB line rate)"))
    rows = {}
    for (eid, name, fns) in EFFECTS:
        call("b_begin", [eid, n])
        tot = []
        pre = []
        ren = []
        for f in range(8):
            t = 1000 + 1237 * f
            cp, _ = call("b_frame", [t])
            cr, _ = call("b_render", [n])
            pre.append(cp)
            ren.append(cr)
            tot.append((cp + cr) / n)
        k = slice(3, 8)                                    # (fire builds up its heat, the state warms)
        mean = sum(tot[k]) / 5.0
        rows[eid] = mean
        p("   %-14s %9.0f %9.1f %9.1f %9.1f   %5.1f %%" % (name, sum(pre[k]) / 5.0, sum(ren[k]) / 5.0 / n, mean, max(tot[k]),
                                                       mean * 31250 / 144e6 * 100))
    allm = list(rows.values())
    p("   mean of the %d effects: %.1f cycles/px, worst effect %.1f" % (len(allm), sum(allm) / len(allm), max(allm)))
    p("")
    p("2. Packer (gamma + bri + order + auto-white), cycles per pixel")
    for bri in (255, 200):
        for sm, snm in ((0, "8-bit path (CONFIG 141 = 3)"), (1, "16-bit, rounded"), (2, "16-bit, dithered")):
            for bpp, nm in ((3, "RGB (WS2812B)"), (4, "RGBW (SK6812 / WS2814)"), (5, "5 ch (WS2805)")):
                c, _ = call("b_pack", [n, bpp, sm, bri])
                p("   %-24s %-28s bri %3d %6.1f   (%.1f %% CPU at the line rate for that chip)" % (nm, snm, bri, c / n, c / n * {3: 31250, 4: 23400, 5: 18750}[bpp] / 144e6 * 100))
    c, _ = call("b_limit", [3000000])
    p("   fx_limit_bri (once a frame): %d cycles" % c)
    p("")
    p("3. Engine layer (fx_engine_frame + fx_engine_render in chunks of 32) over the plain effect: Rainbow / Aurora, cycles per pixel")
    for eid in (4, 13):
        base = None
        for kind, label in ((0, "one segment"), (1, "reverse"), (2, "mirror"), (3, "two segments"), (4, "crossfade running"),
                            (5, "brightness ramp")):
            call("b_eng_setup", [eid, n, kind])
            cs = []
            for f in range(5):
                cf, _ = call("b_eng_frame", [1000 + 997 * f])
                cr, _ = call("b_eng_render", [n])
                cs.append((cf + cr) / n)
            v = sum(cs[2:]) / 3.0
            base = v if base is None else base
            p("   effect %-3d %-18s %7.1f cycles/px (%+.1f vs one segment)" % (eid, label, v, v - base))
    p("")
    p("   per call: the same engine rendered in calls of 32 / 8 / 2 / 1 pixels (the receiver's race-the-beam renderer calls it with the few pixels a")
    p("   tick frees; the call overhead and the effects' set-up per call are why it should batch to >= 32 pixels), cycles per pixel:")
    for eid, nm in ((4, "Rainbow"), (13, "Aurora"), (18, "Pacifica"), (8, "Sinelon")):
        row = []
        for ch in (32, 8, 2, 1):
            call("b_eng_setup", [eid, n, 0])
            call("b_eng_frame", [2000])
            cr, _ = call("b_eng_render_chunks", [n, ch])
            row.append(cr / n)
        p("   %-10s %s" % (nm, "   ".join("%7.1f" % v for v in row)))
    p("")
    syms = e.syms
    p("4. Code size (bytes; symbol sizes in the linked bench, only what is used; static functions the compiler inlined are inside their callers)")
    total_fx = 0
    for (eid, name, fns) in EFFECTS:
        sz = sum(syms[f][1] for f in fns if f in syms)
        total_fx += sz
        p("   %-14s %5d  %s%s" % (name, sz, " ".join(f for f in fns if f in syms), "  " + SHARED[eid] if eid in SHARED else ""))
    ssz = lambda names: sum(syms[f][1] for f in names if f in syms)
    p("   effects total: %d (%d effects, %.0f per effect)" % (total_fx, len(EFFECTS), total_fx / len(EFFECTS)))
    p("   fx_begin / fx_frame / fx_render + palette build + dispatch table: %d" % ssz(CORE_FNS))
    p("   engine (segments, ramps, crossfade, compositor): %d" % ssz(ENGINE))
    p("   math (sin16 / sin8, noise, palette lookup, hue, blend): %d" % ssz(MATH))
    p("   packer + limiter: %d" % ssz(PACK))
    p("   tables: %d (%s)" % (ssz(TABLES), ", ".join("%s %d" % (t, syms[t][1]) for t in TABLES if t in syms)))
    code = total_fx + ssz(CORE_FNS) + ssz(ENGINE) + ssz(MATH) + ssz(PACK) + ssz(TABLES)
    p("   TOTAL (flash): %d bytes" % code)
    osz = obj_sizes(objs)
    p("   by object (text + rodata, before gc-sections; includes unused helpers): " + ", ".join("%s %d" % (k, v[0]) for k, v in osz.items()) +
      "; sum %d" % sum(v[0] for v in osz.values()))
    p("")
    p("5. RAM")
    sz = [call("b_sizeof", [k])[1] for k in range(5)]
    p("   fx_ctx_t (one effect on one run) %d B, fx_seg_t %d, fx_xfade_t %d, fx_params_t %d; fx_engine_t (%d segments, %d crossfades) %d B"
      % (sz[0], sz[1], sz[2], sz[4], 4, 2, sz[3]))
    p("   state pool: only Fire uses it: min(segment pixels, 256) bytes per Fire segment (the pool is sized by the caller)")
    p("   frame buffer + pool at the max pixel count: RGB 1,875 px: %d B; RGBW 1,500 px: %d B; WS2805 1,250 px: %d B (+ the pool)"
      % (1875 * 3 + sz[3], 1500 * 4 + sz[3], 1250 * 5 + sz[3]))
    with open(a.out, "w", newline="\n") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
