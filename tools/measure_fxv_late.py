# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 effects' later phases against the planning models and the step limit (after the b3b5 counter-check's
method): dashboards-r13/v3/digital/golden/v407/late_states.cjs runs each effect alone far into its cycle in the exact
JS mirror (default effect times 20 / 60 / 120 / 200 s, every control corner, 60 and 800 px, Effect default and chosen
palettes) and writes each state; here rv32emu (tools/measure_fxv.py's build) restores it into the bench, checks the
render is byte-identical to the JS one (CRC), then measures the next frame at 42 fps: its advance, the largest single
call (the frame start or one advance slice: an engine step), the render. tools/measure_fxv_models.py's rules: the
frame's advance + render within c_frame + ticks + px x n, the render a pixel within px at 500-800 px, a single call
<= 4,800 (the engine's own share on top stays under 5,000). Exit 1 on a miss, an over-long call or a CRC mismatch.

    node <dashboards>/golden/v407/late_states.cjs --qfx <the worktree's assembled qfx.js> --out states.jsonl
    python tools/measure_fxv_late.py states.jsonl [--jobs n] [--out <file>]"""
import argparse
import base64
import importlib.util
import json
import multiprocessing
import os
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
E = M = GEN = None


def load(name):
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location(name + "_fw", os.path.join(HERE, name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def setup(elf):
    global E, M, GEN
    mf = load("measure_fxv")
    M = load("measure_fxv_models")
    E = mf.emu_module().Emu(elf)
    GEN = mf.built_ids()[1]


def cyc(fn, args=()):
    E.call(fn, list(args))
    return E.cycles()


def measure(line):
    r = json.loads(line)
    n = r["n"]
    E.call("b_palette", [r["palette"]])
    E.call("b_corner", [r["mode"]])
    E.call("b_begin", [r["id"], n, 3])
    parts = [base64.b64decode(r["arena"]), base64.b64decode(r["slab"]), struct.pack("<8I", *r["u"]),
             struct.pack("<4I", *r["job"]), struct.pack("<13I", *r["scalars"])]
    for k, data in enumerate(parts):
        if data:
            E.call("b_snap_ref", [k])
            E.poke(E.ret, data)
    E.call("b_snap_restore", [])
    E.call("b_render", [n, 32])
    E.call("b_snap_ref", [5])
    ok = zlib.crc32(E.peek(E.ret, n * 8)) == r["crc"]
    c = call = cyc("b_frame_begin", [r["time"] + 24])
    while True:
        s = cyc("b_step")
        c, call = c + s, max(call, s)
        if not E.ret:
            break
    rend = cyc("b_render", [n, 32])
    E.call("b_corner", [0])
    row = M.judge(GEN, r["id"], r["palette"] if r["palette"] != 255 else 256, n, 3, r["mode"], c, rend / n, c + rend, call, True)
    row.update(time=r["time"], et=r["et"], crc_ok=ok)
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("states")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--build", default=os.path.join(FW, "build", "rv_fxv_late"))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    mf = load("measure_fxv")
    elf = mf.build(a.build)
    with open(a.states, encoding="utf-8") as f:
        lines = [ln for ln in f.read().splitlines() if ln]
    out = a.out or os.path.join(a.build, "measure_fxv_late.txt")
    if a.jobs <= 1:
        setup(elf)
        rows = [measure(ln) for ln in lines]
    else:
        with multiprocessing.Pool(a.jobs, initializer=setup, initargs=(elf,)) as pool:
            rows = list(pool.imap(measure, lines, chunksize=4))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(os.path.splitext(out)[0] + ".json", "w", encoding="utf-8", newline="\n") as f:
        json.dump(rows, f, indent=0)
    M = load("measure_fxv_models")                      # (the workers' globals are not this process's)
    gen = mf.built_ids()[1]
    bad_crc = [x for x in rows if not x["crc_ok"]]
    misses = [x for x in rows if x["miss"] and x["crc_ok"]]
    calls = [x for x in rows if x["call"] > M.SLICE_WARN and x["crc_ok"]]
    L = ["V407 later phases (JS mirror states restored in rv32emu, CRC-checked): %d states, %d effects" % (
        len(rows), len({x["id"] for x in rows}))]
    worst = {}
    for x in rows:
        w = worst.get(x["id"])
        if w is None or x["ratio"] > w["ratio"]:
            worst[x["id"]] = x
    for i in sorted(worst):
        x = worst[i]
        L.append("   %-4d %-20s ratio %6.3f  call %5d  %s px mode %d %s at %d s effect time" % (
            i, gen["names"].get(str(i), "")[:20], x["ratio"], max(y["call"] for y in rows if y["id"] == i), x["n"], x["mode"],
            "default" if x["palette"] == 256 else "pal %d" % x["palette"], x["et"] // 1000))
    for label, lst in (("CRC mismatch (the state did not transfer; not measured)", bad_crc), ("measured over the model", misses),
                       ("a single call over %d" % M.SLICE_WARN, calls)):
        if lst:
            L.append("%s: %d" % (label, len(lst)))
            for x in sorted(lst, key=lambda x: -x["ratio"])[:40]:
                L.append("   %d %s %d px mode %d t %d: total %d / %d, render %.2f / %d, advance %d, call %d" % (
                    x["id"], "default" if x["palette"] == 256 else "pal %d" % x["palette"], x["n"], x["mode"], x["time"],
                    x["total"], x["total_model"], x["render_px"], x["px_model"], x["advance"], x["call"]))
    L.append("PASS" if not (bad_crc or misses or calls) else "FAIL")
    print("\n".join(L))
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    return 1 if bad_crc or misses or calls else 0


if __name__ == "__main__":
    sys.exit(main())
