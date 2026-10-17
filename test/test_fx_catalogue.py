# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The effect catalogue's consistency (python test/test_fx_catalogue.py):
  - the generated files (fx_tables.c, fx_defaults.inc, fx_catalogue.json) are what tools/gen_fx_tables.py writes now;
  - every effect / palette id in the catalogue is the id of its FX_* / PAL_* constant in src/core/fx.h, and the other way round;
  - ids are never reused or renamed: test/fx_ledger.json lists every id ever issued with its name; the catalogue must keep each
    one with its name, and a new id must be added to the ledger first (python test/test_fx_catalogue.py --add-new);
  - the parameters named in the catalogue are consistent (a palette flag on a palette effect, defaults in range)."""
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
CORE = os.path.join(FW, "src", "core")
LEDGER = os.path.join(HERE, "fx_ledger.json")


def enum_ids(text, prefix, stop):
    """{NAME: value} of the enum constants with the prefix (explicit numbers only)"""
    out = {}
    for m in re.finditer(r"\b(%s[A-Z0-9_]+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)" % prefix, text):
        if m.group(1) not in stop:
            out[m.group(1)] = int(m.group(2), 0)
    return out


def main():
    bad = []
    r = subprocess.run([sys.executable, os.path.join(FW, "tools", "gen_fx_tables.py"), "--check"], capture_output=True, text=True)
    if r.returncode != 0:
        bad.append("generated files out of date: " + r.stdout.strip())
    cat = json.load(open(os.path.join(CORE, "fx_catalogue.json")))
    hdr = open(os.path.join(CORE, "fx.h")).read()
    fx = enum_ids(hdr, "FX_", {"FX_LAST", "FX_COUNT", "FX_MAX_SEG", "FX_MAX_XFADE", "FX_CHUNK"})
    fx = {k: v for k, v in fx.items() if not k.startswith("FX_F_") and not k.startswith("FX_W_")}
    pal = enum_ids(hdr, "PAL_", {"PAL_LAST", "PAL_COUNT", "PAL_COLORS", "PAL_DEFAULT"})
    ids = [e["id"] for e in cat["effects"]]
    pids = [p["id"] for p in cat["palettes"]]
    if len(set(ids)) != len(ids) or len(set(pids)) != len(pids):
        bad.append("duplicate ids in the catalogue")
    if sorted(v for k, v in fx.items() if k != "FX_STREAM") != sorted(ids):
        bad.append("fx.h FX_* ids %s != catalogue %s" % (sorted(fx.values()), sorted(ids)))
    if sorted(pal.values()) != sorted(pids):
        bad.append("fx.h PAL_* ids %s != catalogue %s" % (sorted(pal.values()), sorted(pids)))
    # names: FX_COLORLOOP -> "Colorloop": compare with spaces and punctuation dropped
    norm = lambda s: re.sub(r"[^a-z0-9]", "", s.lower())
    for e in cat["effects"]:
        want = [k for k, v in fx.items() if v == e["id"]]
        if not want or norm(want[0][3:]) != norm(e["name"]):
            if not (want and norm(e["name"]).startswith(norm(want[0][3:])[:6])):
                bad.append("effect %d: header %s vs catalogue name %s" % (e["id"], want, e["name"]))
        d = e["defaults"]
        if e["palette"] and not (0 <= d["palette"] < 128):
            bad.append("effect %d uses a palette but has no default palette" % e["id"])
        if not e["palette"] and d["palette"] != 255:
            bad.append("effect %d: palette default without a palette" % e["id"])
        for k in ("speed", "intensity", "custom1", "custom2"):
            if not 0 <= d[k] <= 255:
                bad.append("effect %d default %s out of range" % (e["id"], k))
        if not 1 <= e["id"] <= 127:
            bad.append("effect id %d outside 1-127 (official)" % e["id"])
    for p in cat["palettes"]:
        if not 0 <= p["id"] <= 127:
            bad.append("palette id %d outside 0-127" % p["id"])
    # the ledger: ids are never reused or renamed
    ledger = json.load(open(LEDGER)) if os.path.exists(LEDGER) else {"effects": {}, "palettes": {}}
    now = {"effects": {str(e["id"]): e["name"] for e in cat["effects"]}, "palettes": {str(p["id"]): p["name"] for p in cat["palettes"]}}
    for kind in ("effects", "palettes"):
        for i, name in ledger[kind].items():
            if i not in now[kind]:
                bad.append("%s id %s (%s) was issued and is gone: ids are never reused" % (kind, i, name))
            elif now[kind][i] != name:
                bad.append("%s id %s was %r and is now %r" % (kind, i, name, now[kind][i]))
        for i, name in now[kind].items():
            if i not in ledger[kind]:
                if "--add-new" in sys.argv:
                    ledger[kind][i] = name
                    print("ledger: added %s %s %s" % (kind, i, name))
                else:
                    bad.append("%s id %s (%s) is new: add it to test/fx_ledger.json (--add-new) once it is final" % (kind, i, name))
    if "--add-new" in sys.argv:
        with open(LEDGER, "w", newline="\n") as f:
            json.dump(ledger, f, indent=1, sort_keys=True)
            f.write("\n")
    for b in bad:
        print("FAIL", b)
    print("test_fx_catalogue: %d effects, %d palettes, %d problems" % (len(ids), len(pids), len(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
