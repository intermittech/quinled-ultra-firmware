# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 planning models against the effects (tools/measure_fxv.py's build and cost table): every effect alone at
each control corner (the slice gate's: 0 defaults, 1 every control 255, 2 speed 255 and the rest 0, 3 speed 1 and the
rest 255), at 60 RGB, 500 RGBCCT, 600 RGBW and 800 RGB, on its Effect default and on each --palettes palette. Frames
at 42 fps (24 ms) from the effect's t = 0 (no catch-up: a tick effect runs the model's ticks a frame); the first is
not charged (it starts the effect); then long runs: 60 px at every corner (--long seconds, every frame rendered) and
800 px at corners 0 and 1 (--long800 seconds, every 4th frame rendered): the effects' later phases, their whole cycle
at the corners' speeds. Every run also reports the largest single call (the frame start or one advance slice: an
engine step, <= 5,000 with the engine's own share). A model is conservative when every
charged frame stays within it (px = c_px less the pack's allowance, 280; natural white 560):

  advance + render  <=  c_frame + ticks_42 x (c_tick + c_part x particles) + px x n   (the frame: fxv_engine.c quota_tier)
  render a pixel    <=  px      at 500-800 px (at 60 px a pixel also carries its render calls' fixed parts, 2 of 32
                                px: the frame's total above charges them)
  advance           <=  c_frame                    (effects 1-36: their advance does not grow with the strip; not
                                                    Pacifica 18, which renders ahead in it)

with the model fxv_engine.c takes for the context (pal_own 1: c_frame / c_px, else c_frame_pal / c_px_pal). An effect
whose advance works a pixel at a time (a field) carries that work in px. Exit 1 on any miss.

    python tools/measure_fxv_models.py [--ids 7,18] [--palettes 10,13,254] [--frames 16] [--long 20] [--long800 10] [--jobs n]
                                       [--out <file>] [--recheck <json>]

--recheck: no measurement; a previous run's records (its .json) against the models fxv_gen.json gives now (the
measurements do not depend on the cost fields: a model change needs no new run, a code change does)."""
import argparse
import importlib.util
import json
import multiprocessing
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
LENGTHS = ((60, 3), (500, 5), (600, 4), (800, 3))
PACK_PX, PACK_WHITE_PX = 280, 560          # tools/fxv_gen.py PACK_COST_PX / PACK_COST_WHITE_PX
FX_PACIFICA = 18

E = None
GEN = None
FRAMES = 16
LONG_S = 20
LONG_N = 60
LONG800_S = 10
SLICE_WARN = 4800                          # a single advance call over this leaves the engine step no room under 5,000


def load(name):
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location(name + "_fw", os.path.join(HERE, name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def setup(elf, frames, long_s, long800_s):
    global E, GEN, FRAMES, LONG_S, LONG800_S
    mf = load("measure_fxv")
    E = mf.emu_module().Emu(elf)
    GEN = mf.built_ids()[1]
    FRAMES = frames
    LONG_S = long_s
    LONG800_S = long800_s


def cyc(fn, args=()):
    E.call(fn, list(args))
    return E.cycles()


def ticks_42(speed, tick_ms):                # fxv_engine.c ticks_42
    rate = 0 if speed == 0 else 8 + ((speed * 15) >> 3)
    return (23810 * rate // 256 + 999) // 1000 // tick_ms + 1 if tick_ms else 0


def judge(gen, eid, pal, n, bpp, mode, adv, rp, tot, call=0, long_run=False):
    """one measured setup against the model fxv_engine.c plans the context with"""
    m = gen["meta"][eid]
    own = pal == 256 and gen["default_palette"][eid] == 255
    fr, px = (m["frame"], m["px"]) if own else (m["frame_pal"], m["px_pal"])
    px_part = px - (PACK_WHITE_PX if m["flags"] & 2 else PACK_PX)
    speed = gen["defaults"][str(eid)]["speed"] if mode == 0 else (1 if mode == 3 else 255)
    bound = fr + ticks_42(speed, m["tick_ms"]) * (m["tick"] + m["particle"] * m["particles"])
    ratios = [tot / (bound + px_part * n)] + ([rp / px_part] if n >= 500 else [])
    if eid < gen["legacy_fx"] and eid != FX_PACIFICA:
        ratios.append(adv / bound)
    return dict(id=eid, palette=pal, own=own, n=n, bpp=bpp, mode=mode, long=long_run, advance=adv, render_px=round(rp, 2),
                total=tot, call=call, frame_model=bound, px_model=px_part, total_model=bound + px_part * n,
                miss=max(ratios) > 1.0, ratio=round(max(ratios), 4))


def job(task):
    eid, pal = task
    E.call("b_palette", [pal])
    rows = []
    runs = [(n, bpp, FRAMES, False, 1, range(4)) for n, bpp in LENGTHS]
    if LONG_S:
        runs.append((LONG_N, 3, LONG_S * 1000 // 24, True, 1, range(4)))
    if LONG800_S:
        runs.append((800, 3, LONG800_S * 1000 // 24, True, 4, range(2)))
    for n, bpp, frames, long_run, every, modes in runs:
        for mode in modes:
            cyc("b_corner", [mode])
            cyc("b_begin", [eid, n, bpp])
            adv = rp = tot = call = 0
            for f in range(frames):
                c = cl = cyc("b_frame_begin", [24 * f])   # (from the effect's t = 0: no start-up debt to catch up)
                while True:
                    s = cyc("b_step")
                    c, cl = c + s, max(cl, s)
                    if not E.ret:
                        break
                if f % every and f != frames - 1:
                    adv, call = max(adv, c), max(call, cl) if f else call
                    continue
                r = cyc("b_render", [n, 32])
                if f == 0:
                    continue
                adv, rp, tot, call = max(adv, c), max(rp, r / n), max(tot, c + r), max(call, cl)
            rows.append(judge(GEN, eid, pal, n, bpp, mode, adv, rp, tot, call, long_run))
    cyc("b_corner", [0])
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", default="")
    ap.add_argument("--palettes", default="10,254", help="besides the Effect default (0..48, 254 Colors 1-2-3)")
    ap.add_argument("--frames", type=int, default=16)
    ap.add_argument("--long", type=int, default=20, help="seconds of the long run at 60 px (0: none)")
    ap.add_argument("--long800", type=int, default=10, help="seconds of the long run at 800 px (0: none)")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--build", default=os.path.join(FW, "build", "rv_fxv_models"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--recheck", default=None, help="a previous run's .json against today's models (no measurement)")
    a = ap.parse_args()
    mf = load("measure_fxv")
    built, gen = mf.built_ids()
    ids = [int(v) for v in a.ids.split(",") if v] or list(range(1, gen["legacy_fx"])) + built
    pals = [256] + [int(v) for v in a.palettes.split(",") if v]
    if a.recheck:
        with open(a.recheck, encoding="utf-8") as f:
            old = json.load(f)
        recs = {}
        for r in old:
            recs.setdefault((r["id"], r["palette"]), []).append(r)
        tasks = sorted(recs)
        elf = None
    else:
        elf = mf.build(a.build)
        tasks = [(i, p) for i in ids for p in pals]
    out = a.out or os.path.join(a.build, "measure_fxv_models.txt")
    L, rows = [], []

    def p(s=""):
        L.append(s)
        print(s, flush=True)

    p("V407 planning models: measured <= model, each effect alone, corners 0-3 at 60 RGB / 500 RGBCCT / 600 RGBW / 800 RGB,")
    p("Effect default + palettes %s, %d frames at 24 ms (the first not charged); rv32emu, tools/measure_fxv.py's build" %
      (a.palettes, a.frames))
    p("   %-4s %-20s %-7s %8s %8s %7s %7s %6s %6s  worst at (ratio: measured / model, the worst of the three; call: the"
      " largest single advance call)" % ("id", "name", "mode", "advance", "frame", "rnd/px", "px", "ratio", "call"))
    if a.recheck:
        results = ([judge(gen, r["id"], r["palette"], r["n"], r["bpp"], r["mode"], r["advance"], r["render_px"], r["total"],
                          r.get("call", 0), r.get("long", False)) for r in recs[t]] for t in tasks)
    elif a.jobs <= 1:
        setup(elf, a.frames, a.long, a.long800)
        results = map(job, tasks)
    else:
        pool = multiprocessing.Pool(a.jobs, initializer=setup, initargs=(elf, a.frames, a.long, a.long800))
        results = pool.imap(job, tasks)
    for r in results:
        rows += r
        w = max(r, key=lambda x: x["ratio"])
        adv = max(x["advance"] for x in r)
        rpx = max(x["render_px"] for x in r)
        name = gen["names"].get(str(w["id"]), str(w["id"]))[:20]
        mode = "default" if w["palette"] == 256 else "pal %d" % w["palette"]
        call = max(x["call"] for x in r)
        p("   %-4d %-20s %-7s %8d %8d %7.1f %7d %6.3f %6d  %d px mode %d%s%s%s" % (
            w["id"], name, mode, adv, w["frame_model"], rpx, w["px_model"], w["ratio"], call, w["n"], w["mode"],
            " (long)" if w["long"] else "", "  MISS" if any(x["miss"] for x in r) else "", "  SLICE" if call > SLICE_WARN else ""))
    misses = [x for x in rows if x["miss"]]
    calls = [x for x in rows if x["call"] > SLICE_WARN]
    p("")
    if calls:
        p("SLICE: %d setups where one advance call (a frame start or a slice) is over %d (an engine step <= 5,000):" % (
            len(calls), SLICE_WARN))
        for x in sorted(calls, key=lambda x: -x["call"])[:60]:
            p("   %d pal %s %d px mode %d%s: %d" % (x["id"], "default" if x["palette"] == 256 else x["palette"], x["n"],
                                                x["mode"], " (long)" if x["long"] else "", x["call"]))
    if misses:
        p("FAIL: %d measured over the model:" % len(misses))
        for x in misses:
            p("   %d pal %s %d px mode %d: total %d / %d, render %.2f / %d, advance %d / %d" % (
                x["id"], "default" if x["palette"] == 256 else x["palette"], x["n"], x["mode"], x["total"],
                x["total_model"], x["render_px"], x["px_model"], x["advance"], x["frame_model"]))
    else:
        p("PASS: every effect's measured advance and render within its model (%d setups)" % len(rows))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    with open(os.path.splitext(out)[0] + ".json", "w", encoding="utf-8", newline="\n") as f:
        json.dump(rows, f, indent=0)
    print("\n-> " + out)
    return 1 if misses or calls else 0


if __name__ == "__main__":
    sys.exit(main())
