# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The Ultra-Digital receiver's local pixel effects (core/ud/udfx.c, wsenc.c, the effect engine) on the CH32V203: cycles and sizes.

Builds test/ud_fx_bench.c with the target compiler (the xPack GCC 15.2, -Os, as the firmware; $RISCV_GCC overrides) and runs
its entry points in tools/rv32emu.py (an instruction-class cycle model of the QingKe V4: ALU 1, load 2, taken branch 2,
div 17). The rows are the numbers docs/ultradigital-firmware.md section 11 gives; the asserts are the budget: the tick
inside the study's 385 cycles +25 %, a render pass inside what the 15 % CPU at the full line rate allows.

    python test/test_ud_fx_cycles.py [--elf-out <path>]        (tools/build_release.sh runs it for UD=1)
"""
import argparse
import glob
import os
import subprocess
import sys
import tempfile
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
from rv32emu import Emu  # noqa: E402

HCLK = 144_000_000
NPX = 300
CHIPS = [("WS2812B V5 (48 / 13 / 25)", 0, 3), ("SK6812 RGBW (48 / 12 / 26)", 3, 4), ("WS2805 RGBCCT (48 / 13 / 25)", 5, 5),
         ("WS2811 400 kHz (90 / 18 / 43)", 7, 3)]
fails = 0


def target_gcc():
    if os.environ.get("RISCV_GCC"):
        return os.environ["RISCV_GCC"]
    xp = sorted(glob.glob(os.path.join(os.environ.get("LOCALAPPDATA", ""), "xPacks", "xpack-riscv-none-elf-gcc-*",
                                       "bin", "riscv-none-elf-gcc*")))
    if not xp:
        raise SystemExit("no target compiler: set RISCV_GCC")
    return xp[-1]


def build(out):
    fx = FW / "src/core"
    srcs = [FW / "test/ud_fx_bench.c", FW / "src/core/ud/udfx.c", FW / "src/core/ud/wsenc.c", FW / "src/core/dline.c"]
    srcs += sorted(fx.glob("fx*.c"))
    inc = ["-I" + str(fx)]
    cmd = [target_gcc(), "-march=rv32imc_zicsr", "-mabi=ilp32", "-Os", "-ffunction-sections", "-fdata-sections", "-fno-common",
           "-ffreestanding", "-nostdlib", "-nostartfiles", "-Wall", "-Wextra", "-Wno-unused-parameter", "-fno-jump-tables",
           "-T", str(FW / "test/ud_fx_rv.ld"), "-o", str(out)] + inc + [str(s) for s in srcs] + ["-lgcc"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise SystemExit(r.stdout + r.stderr)


def check(ok, what):
    global fails
    if not ok:
        fails += 1
        print("FAIL:", what)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf-out", default=None)
    a = ap.parse_args()
    td = tempfile.mkdtemp()
    elf = a.elf_out or os.path.join(td, "ud_fx_bench.elf")
    build(elf)
    e = Emu(elf)

    def cyc(fn, args=()):
        e.call(fn, list(args))
        return e.cycles(), e.ret

    nfx = cyc("b_fx_count")[1]
    print("Local pixel effects on the CH32V203 (rv32emu model, -Os, 144 MHz), %d px\n" % NPX)
    print("1. The encoder in the 20 kHz tick (udfx_tick: the mode check, the nudge, ws_ring_fill), 400 ticks of 112 / 113 words")
    for name, chip, bpp in CHIPS:
        cyc("b_setup", [chip, NPX, 1, 0])
        c, _ = cyc("b_ticks", [400])
        per_tick = c / 400.0
        px = 400 * 112.5 / (bpp * (24 if chip != 7 else 45))
        check(per_tick < (500 if chip != 7 else 2000), "%s: the tick costs %.0f cycles" % (name, per_tick))
        print("   %-34s %6.0f cycles / tick (%.1f us, %.1f %% of a tick)   %5.0f cycles / px" %
              (name, per_tick, per_tick / HCLK * 1e6, per_tick / (HCLK / 20000.0) * 100.0, c / px))
    cyc("b_setup", [0, NPX, 1, 0])
    cyc("b_ticks", [20])
    c, _ = cyc("b_low_tick")
    print("   %-34s %6d cycles" % ("a LOW tick (113 zero words)", c))

    print("\n2. The render, per pixel per frame (a pass over %d px: fx_frame, fx_render, fx_pack, the limiter), mean of 4 frames" % NPX)
    worst_px, rest_px = 0, 0
    PACIFICA = 18                      # the one outlier (HANDOFF-fx-api / NOTES-effects question 5): its own, larger limit
    for fx in range(1, nfx):
        cyc("b_setup", [0, NPX, fx, 0])
        cs = []
        for f in range(6):
            cyc("b_service", [3 + f])
            c, _ = cyc("b_render_frame")
            c2, _ = cyc("b_render_frame")
            cs.append(c2 / NPX)
        mean = sum(cs[2:]) / 4.0
        worst_px = max(worst_px, mean)
        if fx != PACIFICA:
            rest_px = max(rest_px, mean)
        print("   effect %2d   %7.1f cycles / px" % (fx, mean))
    for name, chip, bpp in CHIPS[1:3]:
        cyc("b_setup", [chip, NPX, 1, 0])
        cyc("b_render_frame")
        c, _ = cyc("b_render_frame")
        print("   solid on %-28s %7.1f cycles / px (the packer's %d bytes)" % (name, c / NPX, bpp))
    print("   (a whole frame rendered in chunks of 32 px; the main loop races the beam, section 2b)")
    print("   the worst effect %.0f cycles / px: at the full line rate (31,250 px / s) %.1f %% CPU" %
          (worst_px, worst_px * 31250 / HCLK * 100.0))
    print("   the worst but Pacifica %.0f cycles / px: %.1f %% CPU" % (rest_px, rest_px * 31250 / HCLK * 100.0))
    check(rest_px * 31250 / HCLK < 0.15, "the render at the full line rate takes %.1f %% CPU (budget 15 %%)" %
          (rest_px * 31250 / HCLK * 100.0))
    check(worst_px * 31250 / HCLK < 0.20, "Pacifica at the full line rate takes %.1f %% CPU (limit 20 %%)" %
          (worst_px * 31250 / HCLK * 100.0))

    print("")
    print("2b. The render as the main loop calls it, racing the beam (render in chunks of 32 px once they are free), %d px WS2812B V5," % NPX)
    print("    frames back to back (fps 0), 1,200 ticks after 300 of warm-up; the CPU is render + the 20 kHz tick")
    worst_race, worst_race_rest = 0.0, 0.0
    for fx in range(1, nfx):
        cyc("b_setup", [0, NPX, fx, 0])
        cyc("b_race_reset")
        rc = tc = 0
        f0 = calls0 = 0
        for k in range(1500):
            c, _ = cyc("b_race_tick")
            if k >= 300:
                tc += c
            c, _ = cyc("b_race_render")
            if k == 300:
                f0 = cyc("b_race_frames")[1]
                calls0 = cyc("b_race_calls")[1]
            if k > 300:
                rc += c
        fdone = cyc("b_race_frames")[1] - f0
        calls = cyc("b_race_calls")[1] - calls0
        per_px = rc / (fdone * NPX) if fdone else float("nan")
        cpu = (rc + tc) / (1199 / 20000.0) / HCLK * 100.0
        check(fdone >= 4, "effect %d: only %d frames in 60 ms" % (fx, fdone))
        check(calls / max(fdone, 1) <= NPX / 32.0 + 4.0, "effect %d: %.0f render chunks a frame (chunks of 32 px expected)" % (fx, calls / max(fdone, 1)))
        worst_race = max(worst_race, cpu)
        if fx != PACIFICA:
            worst_race_rest = max(worst_race_rest, cpu)
        print("   effect %2d   %7.1f cycles / px   %5.1f chunks / frame   %5.1f %% CPU (render + tick)" %
              (fx, per_px, calls / max(fdone, 1), cpu))
    check(worst_race_rest < 20.0, "racing the beam: %.1f %% CPU at the full line rate (budget 20 %%)" % worst_race_rest)
    check(worst_race < 30.0, "Pacifica racing the beam: %.1f %% CPU at the full line rate (limit 30 %%)" % worst_race)
    print("   the worst but Pacifica %.1f %% CPU, Pacifica %.1f %%" % (worst_race_rest, worst_race))

    print("\n3. The rest")
    cyc("b_setup", [0, NPX, 1, 30])
    c_srv, _ = cyc("b_service", [100])
    c_anc, _ = cyc("b_anchor", [1])
    c_anc2, _ = cyc("b_anchor", [2])
    c_pg = [cyc("b_page", [p])[0] for p in (0, 1, 2)]
    c_fx, _ = cyc("b_pixel_fx", [3])
    c_lut, _ = cyc("b_setup", [3, NPX, 1, 0])
    print("   udfx_service (a ms)        %6d cycles (%.2f %% of a ms)" % (c_srv, c_srv / (HCLK / 1000.0) * 100.0))
    print("   udfx_anchor                %6d / %d cycles (the second measures the rate)" % (c_anc, c_anc2))
    print("   page 0x81 parts 0 / 1 / 2  %6d / %d / %d cycles" % tuple(c_pg))
    print("   a PIXEL_FX applied         %6d cycles" % c_fx)
    check(c_srv < 6000, "udfx_service takes %d cycles" % c_srv)

    print("\n4. Code size at -Os (bytes)")
    syms = e.syms
    def sz(names):
        return sum(syms[n][1] for n in names if n in syms)
    code = lambda n: syms[n][0] < 0x20000000        # (flash: not the pool, the buffers)
    ud = [n for n in syms if n.startswith(("udfx_", "ux_")) and code(n)]
    ws = [n for n in syms if n.startswith("ws_") and code(n)]
    fxs = [n for n in syms if n.startswith(("fx_", "sin8", "hue8", "pal_at", "rnd8", "fire_frame", "hash32")) and code(n)]
    print("   udfx.c    %6d   wsenc.c %6d (ws_ring_fill %d: .ramtext)   effect engine %6d" %
          (sz(ud), sz(ws), sz(["ws_ring_fill"]), sz(fxs)))
    print("\nlocal-effects cycle rows: %s" % ("passed" if not fails else "FAILED (%d)" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
