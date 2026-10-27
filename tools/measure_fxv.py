# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Cycles of the V407 effects layer (src/core/fx_v407) in rv32emu: builds test/rv_fxv_bench.c + src/core/fx*.c +
src/core/fx_v407/*.c with the xPack GCC 15.2 at the firmware's flags (-Os -mtune=size, tools/build_target.sh's OPT, the
effect files' -ftree-sra -fno-caller-saves -fno-tree-forwprop, no LTO; the V407 builds XW=0), runs the entry points in
tools/rv32emu.py (its V4B-derived cost table, as the other V407 cycle tests) and prints, for every built effect (37+) and
some legacy ones: the frame's advance, its largest slice, render cycles per pixel; the 16-bit packer per pixel; the
segment engine's overhead; and the CPU model per frame at 800 RGB / 600 RGBW / 500 RGBCCT and short strips at 100 fps.

    python tools/measure_fxv.py [--out <file>] [--build <dir>] [--ids 38,4,18]

The model's numbers, not a scope's. Budgets: a frame's advance + render + pack <= 1,000,000 cycles
at the 42 fps floor, one slice <= ~5,000 cycles; the main loop has ~29 % of 200 MHz (58 M cycles a second)."""
import argparse
import glob
import importlib.util
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
OPT = ("-Os -fno-partial-inlining -malign-data=natural --param=max-inline-insns-auto=5 --param=max-hoist-depth=0 "
       "--param=max-grow-copy-bb-insns=3 --param=gcse-unrestricted-cost=0 -fno-tree-sra -fno-jump-tables -fno-ipa-sra "
       "-fno-shrink-wrap-separate -fno-ipa-modref -ffunction-sections -fdata-sections -msmall-data-limit=400").split()
FXFLAGS = ["-ftree-sra", "-fno-caller-saves", "-fno-tree-forwprop"]
LEGACY_REF = [4, 13, 16, 18]
MAIN_HZ = 58_000_000                 # the main loop's share of 200 MHz (71 % modeled poll load)
BUDGET = 1_000_000


def tool(name):
    base = os.environ.get("LOCALAPPDATA", "")
    found = sorted(glob.glob(os.path.join(base, "xPacks", "xpack-riscv-none-elf-gcc-*", "bin", "riscv-none-elf-" + name + "*")))
    found = [f for f in found if not f.endswith("-gcc-ar.exe")]
    if not found:
        sys.exit("xPack riscv-none-elf-%s not found under %%LOCALAPPDATA%%/xPacks" % name)
    return found[0]


def build(out):
    os.makedirs(out, exist_ok=True)
    gcc = tool("gcc")
    arch = ["-march=rv32imc_zicsr_zifencei", "-mabi=ilp32", "-mtune=size"]
    core = os.path.join(FW, "src", "core")
    srcs = [os.path.join(core, f + ".c") for f in ("fx_math", "fx_effects", "fx_engine", "fx_pack", "fx_tables", "math32")]
    srcs += sorted(glob.glob(os.path.join(core, "fx_v407", "*.c")))
    objs = []
    for s in srcs:
        o = os.path.join(out, os.path.basename(s)[:-2] + ".o")
        subprocess.run([gcc] + arch + OPT + FXFLAGS + ["-DFX_V407=1", "-Wall", "-std=gnu11", "-I", core, "-c", s, "-o", o],
                       check=True, timeout=180)
        objs.append(o)
    ld = os.path.join(out, "fxv_rv.ld")
    with open(ld, "w") as f:
        f.write("MEMORY { FLASH (rx) : ORIGIN = 0, LENGTH = 224K\n RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 200K }\n"
                "SECTIONS { .text : { KEEP(*(.text.b_*)) *(.text .text.*) *(.rodata .rodata.* .srodata .srodata.*) } > FLASH\n"
                " .data : { *(.data .data.* .sdata .sdata.*) } > RAM AT > FLASH\n"
                " .bss (NOLOAD) : { *(.bss .bss.* .sbss .sbss.* COMMON) } > RAM\n"
                " __global_pointer$ = ORIGIN(RAM) + 0x800;\n /DISCARD/ : { *(.comment) *(.riscv.attributes) } }\n")
    elf = os.path.join(out, "fxv_bench.elf")
    r = subprocess.run([gcc] + arch + OPT + FXFLAGS + ["-DFX_V407=1", "-std=gnu11", "-ffreestanding", "-nostdlib", "-nostartfiles",
                                                      "-I", core, "-T", ld, "-Wl,--gc-sections", "-Wl,--entry=b_begin", "-o", elf,
                                                      os.path.join(FW, "test", "rv_fxv_bench.c")] + objs + ["-lgcc"],
                       capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        sys.exit(r.stdout + r.stderr)
    return elf


def emu_module():
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location("rv32emu_fw", os.path.join(FW, "tools", "rv32emu.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.RAM_SIZE = 200 * 1024                              # (the V407's 200 KB SRAM: the arena, the slab, the frames)
    return m


def built_ids():
    with open(os.path.join(FW, "src", "core", "fx_v407", "fxv_gen.json"), encoding="utf-8") as f:
        g = json.load(f)
    return g["built"], g


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=None)
    ap.add_argument("--build", default=os.path.join(FW, "build", "rv_fxv"))
    ap.add_argument("--ids", default="")
    ap.add_argument("--palette", type=int, default=256, help="every effect on this palette (default: its Effect default)")
    a = ap.parse_args()
    built, gen = built_ids()
    ids = [int(v) for v in a.ids.split(",") if v] or built + LEGACY_REF
    elf = build(a.build)
    m = emu_module()
    e = m.Emu(elf)
    e.call("b_palette", [a.palette])
    L, res = [], {"effects": {}, "pack": {}, "engine": {}, "model": {}}

    def p(s=""):
        L.append(s)
        print(s)

    def cyc(fn, args=()):
        e.call(fn, list(args))
        return e.cycles()

    p("V407 effects layer: rv32emu cycles (V4B-derived cost table), xPack GCC 15.2 -Os, the firmware's flags; 200 MHz")
    p("")
    p("1. Effects at their catalogue defaults, alone: per frame (advance = frame start + every slice), the largest slice,")
    p("   render cycles per pixel (chunks of 32), at 60 and 800 px (RGB); frames 4-11 of a 42 fps run")
    p("   %-5s %-22s %6s %10s %9s %9s %10s %10s" % ("id", "name", "n", "advance", "max slice", "render/px", "frame", "% of 1 M"))
    for eid in ids:
        name = gen["names"].get(str(eid), str(eid))[:22]
        for n in (60, 800):
            cyc("b_begin", [eid, n, 3])
            adv, mx, ren = [], 0, []
            for f in range(12):
                t = 2000 + f * 24
                c = cyc("b_frame_begin", [t])
                while True:
                    s = cyc("b_step")
                    c += s
                    mx = max(mx, s)
                    if not e.ret:
                        break
                r = cyc("b_render", [n, 32])
                if f >= 4:
                    adv.append(c)
                    ren.append(r / n)
            av, rp = max(adv), max(ren)
            frame = av + rp * n
            res["effects"].setdefault(str(eid), {})[str(n)] = dict(advance=av, max_slice=mx, render_px=round(rp, 1), frame=round(frame))
            p("   %-5d %-22s %6d %10d %9d %9.1f %10.0f %9.1f %%" % (eid, name, n, av, mx, rp, frame, frame / BUDGET * 100))
    p("")
    p("2. The 16-bit packer (gamma at fractional inputs, brightness, white mapping, dither / round), cycles per pixel, 32 px")
    cyc("b_pack", [32, 3, 1, 200, 0])                    # (warm-up: the first call builds the white calibration once)
    for bpp in (3, 4, 5):
        for q in (0, 255):
            row = []
            for sm in (0, 1, 2):
                for bri in (255, 200):
                    row.append(cyc("b_pack", [32, bpp, sm, bri, q]) / 32.0)
            res["pack"]["%d_%s" % (bpp, "white" if q else "colour")] = [round(v, 1) for v in row]
            p("   %d bytes, %-14s 8-bit %5.1f / %5.1f  round %5.1f / %5.1f  dither %5.1f / %5.1f   (bri 255 / 200)" % (
                bpp, "natural white" if q else "colour", *row))
    # the packer a frame costs: dithered at brightness 200 (the worst of round / dither), colour or natural white
    pk = {(bpp, w): res["pack"]["%d_%s" % (bpp, "white" if w else "colour")][5] for bpp in (3, 4, 5) for w in (0, 1)}
    p("")
    p("3. The segment engine with Kelvin Drift (38) at 800 px: frame start (dequeue + segment frames + every step) and the")
    p("   composited render, cycles per pixel")
    for kind, label in ((0, "one segment"), (1, "reverse"), (2, "mirror"), (3, "four overlapping segments"), (4, "two crossfades (38 -> 4)")):
        cyc("b_eng_setup", [38, 800, kind, 4])
        fr, rr = [], []
        for f in range(6):
            c = cyc("b_eng_frame_begin", [2000 + f * 24])
            while True:
                c += cyc("b_eng_step")
                if not e.ret:
                    break
            fr.append(c)
            rr.append(cyc("b_eng_render", [800]) / 800.0)
        res["engine"][label] = dict(frame=max(fr), render_px=round(max(rr), 1))
        p("   %-28s frame start %8d   render %6.1f cycles/px" % (label, max(fr), max(rr)))
    p("")
    p("4. The CPU model per frame: advance + (render + pack at that format, dithered, bri 200) x n with one effect, against")
    p("   1,000,000 cycles (the 42 fps floor's budget) and the main loop's 58 M cycles a second at the frame's grid")
    p("   (UX_GUARD_US 100, the chips' 1.33 us bits)")
    grid = lambda n, bpp: max(10000, (n * bpp * 8 * 133 + 99) // 100 + 302 + 100)
    for eid in ids:
        for n, bpp, lab in ((800, 3, "RGB 800"), (600, 4, "RGBW 600"), (500, 5, "RGBCCT 500"), (60, 3, "RGB 60 @100 fps")):
            r = res["effects"][str(eid)]["800" if n > 60 else "60"]
            white = bool(gen["meta"][eid]["flags"] & 2)
            fr = r["advance"] + (r["render_px"] + pk[(bpp, white)]) * n
            us = grid(n, bpp)
            avail = MAIN_HZ * us / 1e6
            res["model"]["%d_%s" % (eid, lab)] = dict(frame=round(fr), grid_us=us, main_loop=round(avail))
            p("   %-5d %-16s frame %9.0f cycles  %5.1f %% of 1 M  grid %6d us  %5.1f %% of the main loop's %8.0f" % (
                eid, lab, fr, fr / BUDGET * 100, us, fr / avail * 100, avail))
    out = a.out or os.path.join(a.build, "measure_fxv.txt")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    with open(os.path.splitext(out)[0] + ".json", "w", encoding="utf-8", newline="\n") as f:
        json.dump(res, f, indent=1)
    print("\n-> " + out)


if __name__ == "__main__":
    main()
