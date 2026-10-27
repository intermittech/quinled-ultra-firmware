# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 effects layer's integration gate in rv32emu (the build and cost table of tools/measure_fxv.py): effects on
the segment engine, run step by step as the receiver runs it (udfx.c): a segment set (engine_set) is a step, each
segment's dequeue and frame start is a step, then every engine slice (quotas, moves, zeroing, init, advance, catch-up);
the render + pack steps walk the strip by the engine's own spans (fxv_engine_span: one run each), fxv_engine_render then
fxv_pack16 dithered at brightness 200, accurate white.

    python tools/measure_fxv_slices.py [--out <file>] [--build <dir>] [--ids 38,94] [--jobs n] [--no-mix] [--mix-only]
                                       [--no-worst] [--heavy k] [--palette p] [--partners 50,27] [--late k]

--palette p: every effect on palette p (0..48, 254 Colors 1-2-3) instead of its Effect default (the palette round's G3).

1. Every effect alone (the 78 new ones and the 36 legacy ones through the adapter), each control corner (0 defaults,
   1 every control 255, 2 speed 255 and the rest 0, 3 speed 1 and the rest 255) at 800 RGB, 600 RGBW and 500 RGBCCT,
   plus defaults at 60 RGB and all-max at two odd lengths (250 RGBCCT: a mirrored 500; 229 RGB). Frames: 7 at 42 fps
   (24 ms) from the start, the 8th rendered (steady = its render + the most advance of frames 2-8) and rendered again
   reversed at segment brightness 128, then a 1.9 s jump (the largest catch-up without the 2 s restart) rendered, then
   a 2.5 s jump (the seed + epoch restart).
2. Heavy mixes: four overlapping full-length segments, then segments 0 and 1 crossfade (3 s) to two more effects; the
   integration counter-test's mixes (four Marble Veins all-max fading to Tinsel / Incense Smoke ...).
3. The worst-case generator: the heaviest effects (the largest c_step and the largest measured render steps of part 1)
   pairwise as a crossfade (every ordered pair of two of them) at each format, in two layouts: on top of three other
   full-length segments, and as a mirrored, dimmed segment under three short segments (1-3 pixels, one reversed, one
   crossfading) that cut its render steps into many short runs.
4. --partners: targeted crossfades, every effect of --ids (all by default) to and from each partner, in part 3's two
   layouts at each format, all-max (50 Koi Pond: a heavy step; 27 Opal Facets: the largest fixed part a render step).
   --late k: parts 3 and 4 render k more frames 500 ms apart while the fade runs (later phases, catch-up).
Gate (exit 1): any slice over 5,000 cycles (SLICE). A frame over 1,000,000 at the 42 fps floor is flagged (FRAME) and
reported: the accepted exceptions are known (Thunderstorm / Ink Bloom at 800 RGB, catch-up frames, heavy
mixes at a lower fps)."""
import argparse
import importlib.util
import json
import multiprocessing
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
SLICE_MAX = 5000
BUDGET = 1_000_000
FULL = ((800, 3), (600, 4), (500, 5))
CONFIGS = [(n, bpp, mode) for n, bpp in FULL for mode in range(4)] + [(60, 3, 0), (250, 5, 1), (229, 3, 1)]
FMT = {3: "RGB", 4: "RGBW", 5: "RGBCCT"}
REV_DIM = 1 | (128 << 8)                 # b_eng_params / b_eng_look: reversed, segment brightness 128
MIXES = [("four heavy, all-max", (94, 51, 62, 57), 1, (112, 90), 1144),
         ("four particle effects, defaults", (94, 57, 112, 90), 0, (107, 53), 1144),
         ("four fields, all-max", (51, 66, 71, 92), 1, (86, 62), 1144),
         ("counter-test: Marble Veins x4", (63, 63, 63, 63), 1, (109, 52), 1144),
         ("counter-test: Koi Pond x4", (50, 50, 50, 50), 1, (63, 109), 1144),
         ("counter-test: Wind Chimes x4", (61, 61, 61, 61), 1, (78, 81), 1144)]

E = None
GEN = None


def load(name):
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location(name + "_fw", os.path.join(HERE, name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def setup(elf, palette=256):
    global E, GEN
    mf = load("measure_fxv")
    E = mf.emu_module().Emu(elf)
    GEN = mf.built_ids()[1]
    E.call("b_palette", [palette])


def cyc(fn, args=()):
    E.call(fn, list(args))
    return E.cycles()


def put(s, eid, mode, now, start_len=0, look=0):
    cyc("b_eng_params", [eid, mode, start_len, look])
    return cyc("b_eng_set", [s, now])


def frame(t, render, n):
    """one frame at host ms t as udfx.c's steps: (engine-side slices' max, their sum, render + pack max, sum, steps)"""
    smax = ssum = 0

    def step(c):
        nonlocal smax, ssum
        smax, ssum = max(smax, c), ssum + c

    def drain():
        while True:
            step(cyc("b_eng_step"))
            if not E.ret:
                return
    drain()
    for s in range(4):
        step(cyc("b_eng_dequeue", [s, t]))
        drain()
    for s in range(4):
        step(cyc("b_eng_seg", [s, t]))
        drain()
    rmax, rsum, steps = (walk(n) if render else (0, 0, 0))
    return smax, ssum, rmax, rsum, steps


def walk(n):
    """the frame's render + pack steps by the engine's spans: (max, sum, steps)"""
    rmax = rsum = steps = k = 0
    while k < n:
        cyc("b_eng_span", [k, n])
        end = E.ret
        if not k < end <= n:
            raise RuntimeError("span %d -> %d of %d" % (k, end, n))
        r = cyc("b_slice", [k, end - k])
        rmax, rsum, steps, k = max(rmax, r), rsum + r, steps + 1, end
    return rmax, rsum, steps


def effect_rows(eid):
    rows = []
    for n, bpp, mode in CONFIGS:
        cyc("b_eng_init", [n, bpp])
        adv = put(0, eid, mode, 1000)
        rp = warm = start = 0
        t = 1000
        for f in range(7):
            smax, ssum, _, _, _ = frame(t, False, n)
            adv = max(adv, smax)
            if f == 0:
                start = ssum                                # (the start: zeroing, init, the first advance)
            else:
                warm = max(warm, ssum)                      # (frames with and without simulation ticks)
            t += 24
        smax, ssum, rmax, rsum, steps = frame(t, True, n)
        steady = max(warm, ssum) + rsum
        adv, rp = max(adv, smax), max(rp, rmax)
        cyc("b_eng_look", [0, REV_DIM])
        rlook, _, _ = walk(n)
        cyc("b_eng_look", [0, 0])
        cyc("b_eng_span", [0, n])
        chunk = E.ret
        t += 1900
        smax, ssum, rmax, rsum, _ = frame(t, True, n)
        catch = ssum + rsum
        cmax = smax
        adv, rp = max(adv, smax), max(rp, rmax)
        t += 2500
        smax, _, _, _, _ = frame(t, False, n)              # the restart (seed + epoch)
        adv = max(adv, smax)
        rows.append(dict(id=eid, n=n, bpp=bpp, mode=mode, chunk=chunk, steps=steps, adv_max=adv, rp_max=rp, rp_look=rlook,
                         start=start, catchup_slice=cmax, steady=steady, catchup=catch))
    return rows


def mix_row(label, four, mode, fade, t_fade, n, bpp):
    cyc("b_eng_init", [n, bpp])
    smax = 0
    for s, fid in enumerate(four):
        smax = max(smax, put(s, fid, mode, 1000))
    t = 1000
    for f in range(6):
        smax = max(smax, frame(t, False, n)[0])
        t += 24
    for s, fid in enumerate(fade):
        smax = max(smax, put(s, fid, mode | (3000 << 8), t_fade))
    rows = []
    for f in range(4):
        t += 24
        rows.append(frame(t, f == 3, n))
    smax = max([smax] + [r[0] for r in rows])
    rmax = rows[-1][2]
    fr = rows[-1][1] + rows[-1][3]
    tier = cyc("b_eng_tier") and E.ret
    model = cyc("b_eng_cost") and E.ret
    return dict(label=label, n=n, bpp=bpp, tier=tier, model=model, adv_max=smax, rp_max=rmax, steps=rows[-1][4], frame=fr)


def worst_row(layout, a, b, n, bpp, late=0):
    """a crossfading to b (the run's two layers) at all-max: on top of three full-length segments, or mirrored + dimmed
    under three short segments (the render steps cut into short runs); late: that many more rendered frames 500 ms
    apart while the fade runs (the effects' later phases, catch-up included)"""
    cyc("b_eng_init", [n, bpp])
    smax = 0
    if layout == "top":
        for s, fid in enumerate((63, 109, 52)):
            smax = max(smax, put(s, fid, 1, 1000))
        smax = max(smax, put(3, a, 1, 1000))
        top = 3
    else:
        smax = max(smax, put(0, a, 1, 1000, 0, 2 | (128 << 8)))
        smax = max(smax, put(1, 80, 1, 1000, (5 << 16) | 1))
        smax = max(smax, put(2, 50, 1, 1000, (9 << 16) | 2))
        smax = max(smax, put(3, 63, 1, 1000, (14 << 16) | 3, 1))
        top = 0
    t = 1000
    for f in range(3):
        smax = max(smax, frame(t, False, n)[0])
        t += 24
    smax = max(smax, put(top, b, 1 | (3000 << 8), t, 0, 0 if layout == "top" else 2 | (128 << 8)))
    if layout != "top":
        smax = max(smax, put(2, 109, 1 | (3000 << 8), t, (9 << 16) | 2))
    for f in range(2):
        t += 24
        smax = max(smax, frame(t, False, n)[0])
    t += 24
    s2, _, rmax, rsum, steps = frame(t, True, n)
    smax = max(smax, s2)
    for f in range(late):
        t += 500
        s2, _, r2, _, _ = frame(t, True, n)
        smax, rmax = max(smax, s2), max(rmax, r2)
    return dict(layout=layout, a=a, b=b, n=n, bpp=bpp, adv_max=smax, rp_max=rmax, steps=steps)


def _job(args):
    kind = args[0]
    if kind == "fx":
        return effect_rows(args[1])
    if kind == "mix":
        return mix_row(*args[1:])
    return worst_row(*args[1:])


def run(jobs, elf, tasks, palette=256):
    if jobs <= 1:
        setup(elf, palette)
        for t in tasks:
            yield _job(t)
        return
    with multiprocessing.Pool(jobs, initializer=setup, initargs=(elf, palette)) as pool:
        for r in pool.imap(_job, tasks):
            yield r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None)
    ap.add_argument("--build", default=os.path.join(FW, "build", "rv_fxv_slices"))
    ap.add_argument("--ids", default="")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--no-mix", action="store_true")
    ap.add_argument("--mix-only", action="store_true")
    ap.add_argument("--no-worst", action="store_true")
    ap.add_argument("--heavy", type=int, default=8, help="effects in the worst-case generator's heavy set")
    ap.add_argument("--palette", type=int, default=256, help="every effect on this palette (default: its Effect default)")
    ap.add_argument("--partners", default="", help="part 4: every effect to and from these ids as a crossfade")
    ap.add_argument("--late", type=int, default=0, help="parts 3 and 4: rendered frames 500 ms apart after the fade's start")
    a = ap.parse_args()
    mf = load("measure_fxv")
    built, gen = mf.built_ids()
    legacy = list(range(1, gen["legacy_fx"]))
    ids = [] if a.mix_only else [int(v) for v in a.ids.split(",") if v] or built + legacy
    elf = mf.build(a.build)
    L, res = [], {"effects": {}, "mix": {}, "worst": [], "elf": elf}
    out = a.out or os.path.join(a.build, "measure_fxv_slices.txt")
    over = []

    def p(s=""):
        L.append(s)
        print(s, flush=True)

    p("V407 integration gate: rv32emu cycles (tools/rv32emu.py cost table), xPack GCC 15.2 -Os, the firmware's flags")
    p("effects on the engine, step by step as udfx.c; slice = a segment set, a dequeue, a frame start, an engine step or a")
    p("render + pack step (one run: fxv_engine_span); gate %d / frame %d (flagged)" % (SLICE_MAX, BUDGET))
    if a.palette < 256:
        p("every effect on palette %d (--palette), not its Effect default" % a.palette)
    worst = {"slice": (0, ""), "frame": (0, "")}
    measured = {}
    if ids:
        p("")
        p("1. Every effect alone")
        p("   %-4s %-20s %-10s %4s %5s %5s %8s %8s %8s %8s %9s %9s %9s %s" % (
            "id", "name", "format", "mode", "chunk", "steps", "adv max", "rp max", "rev+dim", "catchup", "steady", "catch-up",
            "% of 1 M", "flag"))
        for rows in run(a.jobs, elf, [("fx", i) for i in ids], a.palette):
            for r in rows:
                eid, n, bpp, mode = r["id"], r["n"], r["bpp"], r["mode"]
                name = gen["names"].get(str(eid), str(eid))[:20]
                lab = "%s %d" % (FMT[bpp], n)
                full = n >= 500
                sl = max(r["adv_max"], r["rp_max"], r["rp_look"])
                flag = []
                if sl > SLICE_MAX:
                    flag.append("SLICE")
                    over.append("%d %s mode %d: %d" % (eid, lab, mode, sl))
                if full and max(r["steady"], r["catchup"]) > BUDGET:
                    flag.append("FRAME")
                res["effects"].setdefault(str(eid), []).append(r)
                measured[eid] = max(measured.get(eid, 0), r["rp_max"], r["rp_look"])
                if sl > worst["slice"][0]:
                    worst["slice"] = (sl, "%d %s mode %d" % (eid, lab, mode))
                if full and max(r["steady"], r["catchup"]) > worst["frame"][0]:
                    worst["frame"] = (max(r["steady"], r["catchup"]), "%d %s mode %d" % (eid, lab, mode))
                p("   %-4d %-20s %-10s %4d %5d %5d %8d %8d %8d %8d %9d %9d %8.1f%% %s" % (
                    eid, name, lab, mode, r["chunk"], r["steps"], r["adv_max"], r["rp_max"], r["rp_look"], r["catchup_slice"],
                    r["steady"], r["catchup"], max(r["steady"], r["catchup"]) / BUDGET * 100, " ".join(flag)))
        p("")
        p("largest slice %d (%s); largest full-length frame %d (%s)" % (worst["slice"][0], worst["slice"][1],
                                                                          worst["frame"][0], worst["frame"][1]))
    res["worst_single"] = worst
    if not a.no_mix:
        p("")
        p("2. Heavy mixes: four overlapping full-length segments, then segments 0 and 1 crossfade (3 s) to two more effects")
        p("   %-34s %-10s %4s %9s %8s %8s %5s %9s %8s %s" % ("mix", "format", "tier", "model", "adv max", "rp max", "steps", "frame",
                                                         "% of 1 M", "flag"))
        tasks = [("mix", lab, four, mode, fade, tf, n, bpp) for lab, four, mode, fade, tf in MIXES for n, bpp in FULL]
        for r in run(a.jobs, elf, tasks, a.palette):
            lab = "%s %d" % (FMT[r["bpp"]], r["n"])
            sl = max(r["adv_max"], r["rp_max"])
            flag = "SLICE" if sl > SLICE_MAX else ""
            if flag:
                over.append("mix %s %s: %d" % (r["label"], lab, sl))
            res["mix"]["%s %s" % (r["label"], lab)] = r
            p("   %-34s %-10s %4d %9d %8d %8d %5d %9d %7.1f%% %s" % (r["label"][:34], lab, r["tier"], r["model"], r["adv_max"],
                                                               r["rp_max"], r["steps"], r["frame"], r["frame"] / BUDGET * 100, flag))
    if not a.no_worst:
        steps = {i: gen["meta"][i]["step_pal" if a.palette < 256 else "step"] for i in built + legacy}
        heavy = sorted(steps, key=lambda i: -steps[i])[:a.heavy]
        for i in sorted(measured, key=lambda i: -measured[i])[:max(2, a.heavy // 2)]:
            if i not in heavy:
                heavy.append(i)
        p("")
        p("3. Worst-case generator: heavy set %s (c_step %s); every ordered pair a -> b as a crossfade at all-max," %
          (",".join(map(str, heavy)), ",".join(str(steps[i]) for i in heavy)))
        p("   on top of three full-length segments ('top'), or mirrored + dimmed under three short segments ('cut')")
        p("   %-5s %-4s %-4s %-10s %8s %8s %5s %s" % ("lay", "a", "b", "format", "adv max", "rp max", "steps", "flag"))
        tasks = [("worst", lay, x, y, n, bpp, a.late) for lay in ("top", "cut") for n, bpp in FULL for x in heavy for y in heavy
                 if x != y]
        wmax = (0, "")
        for r in run(a.jobs, elf, tasks, a.palette):
            lab = "%s %d" % (FMT[r["bpp"]], r["n"])
            sl = max(r["adv_max"], r["rp_max"])
            flag = "SLICE" if sl > SLICE_MAX else ""
            if flag:
                over.append("worst %s %d -> %d %s: %d" % (r["layout"], r["a"], r["b"], lab, sl))
            if sl > wmax[0]:
                wmax = (sl, "%s %d -> %d %s" % (r["layout"], r["a"], r["b"], lab))
            res["worst"].append(r)
            p("   %-5s %-4d %-4d %-10s %8d %8d %5d %s" % (r["layout"], r["a"], r["b"], lab, r["adv_max"], r["rp_max"], r["steps"], flag))
        p("")
        p("worst-case generator: largest slice %d (%s), %d setups" % (wmax[0], wmax[1], len(tasks)))
    partners = [int(v) for v in a.partners.split(",") if v]
    if partners:
        subjects = [int(v) for v in a.ids.split(",") if v] or built + legacy
        tasks = [("worst", lay, x, y, n, bpp, a.late) for lay in ("top", "cut") for n, bpp in FULL for q in partners
                 for i in subjects if i != q for x, y in ((i, q), (q, i))]
        p("")
        p("4. Targeted crossfades: every effect to and from %s at all-max, layouts and formats as part 3" % a.partners)
        tmax, tbad = (0, ""), 0
        for r in run(a.jobs, elf, tasks, a.palette):
            lab = "%s %d" % (FMT[r["bpp"]], r["n"])
            sl = max(r["adv_max"], r["rp_max"])
            if sl > SLICE_MAX:
                tbad += 1
                over.append("partner %s %d -> %d %s: %d" % (r["layout"], r["a"], r["b"], lab, sl))
                p("   %-5s %-4d %-4d %-10s %8d %8d %5d SLICE" % (r["layout"], r["a"], r["b"], lab, r["adv_max"], r["rp_max"], r["steps"]))
            if sl > tmax[0]:
                tmax = (sl, "%s %d -> %d %s" % (r["layout"], r["a"], r["b"], lab))
            res.setdefault("partners", []).append(r)
        p("targeted crossfades: largest slice %d (%s), %d setups, %d over" % (tmax[0], tmax[1], len(tasks), tbad))
    p("")
    if over:
        p("FAIL: %d over %d cycles: %s" % (len(over), SLICE_MAX, "; ".join(over[:40])))
    else:
        p("PASS: every slice <= %d cycles" % SLICE_MAX)
    res["over"] = over
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    with open(os.path.splitext(out)[0] + ".json", "w", encoding="utf-8", newline="\n") as f:
        json.dump(res, f, indent=1)
    print("\n-> " + out)
    return 1 if over else 0


if __name__ == "__main__":
    sys.exit(main())
