# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The frozen record corpus: every store's flash pages as one firmware build wrote them, and the check
that a later build reads them back the same and writes on after them. On the compiled firmware in tools/rv32emu.py
(tools/flash_stores.py's flash model), so the pages are the target's own bytes.

    python tools/record_corpus.py write <fw.elf> <dir> <label> [<source>]   a build's corpus: <dir>/pages.hex and
                                                                          <dir>/manifest.json
    python tools/record_corpus.py check <fw.elf> <dir> [<dir> ...]          each corpus into a build (test/records/*/:
                                                                          every frozen version)

<fw.elf>: a measurement build (MEASURE=1: the stores out of line, core/lto.h MEASURED), the upper record layout
(the record layout). <source>: what built it (a commit).

The writes (seeded, the same for every build; a pair's erase only when it is due, so both pages hold records): the
settings 11 records (page A full, B begun), the power-on scene log 21 (A full, B begun), the calibration 6, the
lifetime record 13 (both pages full, then A erased and begun again), the calibration history 6 (builds with it,
MON_EXTRA), the power-on effects twice (bank B all five lights, then bank A lights 0, 2 and 4: the newest). pages.hex: every store's pages, 32 bytes a line
("address hex"); manifest.json: what the writing build's own loaders read back from them, as the next boot does.

The check, per corpus: the area erased, the pages in; the build's loaders must read what the manifest says (or the
incompatible SFX1 effects must be rejected; the other stores must match);
then one more record per store, and a fresh boot of the build on that flash reads the new ones. Exit 1 on a failure."""
import hashlib
import json
import os
import random
import struct
import sys

from flash_stores import L066, UPPER_SPAN, fresh
from rv32emu import RAM_BASE, RAM_SIZE

# Explicitly incompatible SFX1 fixtures: the SFX2 loader must reject them.
SFX1_CORPORA = {"0.6.10", "0.6.11", "0.6.12"}
REC = {"life": 0, "settings": 1, "cal": 2, "scene": 3, "calhist": 4}   # core/flashrec.h REC_*
HEAD = 100                                              # ULTRA_HEAD_BYTES on the target (flashrec.c asserts it)
G_INIT = struct.pack("<I", 183501)                       # ultra_init's g_q16 (2.8) at the start of each light


def crc8(b):
    """core/config.h crc8_smbus: polynomial 0x07, init 0, MSB first"""
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


class Board:
    """a build on the flash model: the scratch buffer (din_ring), the engine's lights (ul) and their stride"""
    def __init__(self, elf):
        self.e = fresh(elf)
        e = self.e
        self.buf = e.addr("din_ring") + 64
        self.ul = e.addr("ul")
        e.call("ultra_init", [self.ul, 1234])
        ram = e.peek(self.ul, min(5 * 2048, RAM_BASE + RAM_SIZE - self.ul))
        at = [i for i in range(0, len(ram) - 3, 4) if ram[i:i + 4] == G_INIT]
        if len(at) < 5 or at[0] != 0 or len(set(at[k + 1] - at[k] for k in range(4))) != 1:
            raise SystemExit("%s: the lights' stride not found (ultra_init's gamma at %s)" % (elf, at[:6]))
        self.stride = at[1]
        self.has_hist = "calhist_load" in e.syms

    def call(self, fn, args):
        self.e.call(fn, args)
        return self.e.ret

    def prepare(self, store):
        """a pair's erase when it is due before the next write (2; the stores do that themselves, the lifetime record
        never: main.c erases it ahead), not one possible ahead (1): both pages keep their records"""
        if self.call("flash_rec_pending", [REC[store]]) == 2:
            self.call("flash_rec_prepare", [REC[store]])

    def loads(self):
        """what the loaders read (as main.c's records_load at power-up): {store: {...}}"""
        e, b, r = self.e, self.buf, {}
        e.poke(b, bytes(900))
        r["settings"] = {"ret": self.call("settings_load", [b, b + 32]), "set": e.peek(b, 16).hex(),
                         "ext": e.peek(b + 32, 7).hex()}
        ret = self.call("cal_load", [b + 64, b + 120])
        c = e.peek(b + 64, 48)
        r["cal"] = {"ret": ret, "cal": (c[:5] + c[6:]).hex(), "count": e.peek(b + 120, 2).hex()}   # (5: padding)
        r["scene"] = {"ret": self.call("scene_load", [b + 128]), "scene": e.peek(b + 128, 12).hex()}
        r["life"] = {"ret": self.call("life_load", [b + 160]), "rec": e.peek(b + 160, 48).hex()}
        if self.has_hist:
            r["calhist"] = {"ret": self.call("calhist_load", [b + 224]), "rec": e.peek(b + 224, 64).hex()}
        e.call("ultra_init", [self.ul, 1234])
        mask = self.call("scene_fx_load", [self.ul, b + 300])
        r["fx"] = {"mask": mask, "crc": e.peek(b + 300, 5).hex(),
                   "heads": [e.peek(self.ul + i * self.stride, HEAD).hex() if mask >> i & 1 else None for i in range(5)]}
        return r

    def area(self):
        return self.e.peek(UPPER_SPAN[0], UPPER_SPAN[1])


def heads(rnd):
    """five power-on effect heads that core/scene.c head_ok takes (the target's layout: core/ultra.h ultra_light_t)"""
    out = []
    for _ in range(5):
        role = [rnd.randrange(8) for _ in range(5)]
        h = struct.pack("<I", rnd.randrange(65536, 4294967))                      # g_q16 (gamma <= 65.535)
        h += bytes(rnd.getrandbits(8) for _ in range(22)) + bytes(2)              # scene_v (24: aligned 4)
        h += struct.pack("<10H", *[rnd.getrandbits(16) for _ in range(10)])        # min_p, max_p
        h += struct.pack("<4H", rnd.getrandbits(16), rnd.getrandbits(16), 5000, rnd.getrandbits(16))  # cw ww warm assist
        h += struct.pack("<4H4H", 60000, 20000, 0, 0, 3000, 4000, 0, 0)           # opt_pm, opt_pt: 2 points
        h += bytes([sum(1 << i for i in range(5) if role[i]), rnd.getrandbits(2)] + role)   # mask, flags, role
        h += bytes([rnd.getrandbits(2), 2, 1, rnd.getrandbits(8), rnd.randrange(3, 10)])  # opt_flags opt_n mode fade fx
        h += bytes(rnd.getrandbits(8) for _ in range(14))                         # scene_fxp
        h += bytes(2)                                                         # base_channel, reserved
        assert len(h) == HEAD
        out.append(h)
    return out


def write(bd, rnd, n_set, n_scene, n_cal, n_life, n_hist, fx_masks):
    """the stores' writes; returns what was written last per store (the check's expectation after its writes)"""
    e, b, w = bd.e, bd.buf, {}
    bd.loads()                                           # (the positions, as records_load at power-up)
    for _ in range(n_set):
        s, x = bytes(rnd.getrandbits(8) for _ in range(16)), bytes(rnd.getrandbits(8) for _ in range(7))
        e.poke(b, s + bytes(16) + x)
        bd.call("settings_store", [b, b + 32])
        bd.prepare("settings")
        w["settings"] = {"set": s.hex(), "ext": x.hex()}
    for _ in range(n_scene):
        sc = bytes(rnd.getrandbits(8) for _ in range(10)) + bytes([rnd.getrandbits(5), rnd.getrandbits(8)])
        e.poke(b + 128, sc)
        bd.call("scene_store", [b + 128])
        bd.prepare("scene")
        w["scene"] = {"scene": sc.hex()}
    for _ in range(n_cal):
        c = struct.pack("<B4Bxh", rnd.choice([1, 4]), 25, 50, 75, 100, rnd.getrandbits(15) - 100)
        c += struct.pack("<20H", *[rnd.getrandbits(16) for _ in range(20)])
        e.poke(b + 64, c)
        bd.call("cal_store", [b + 64, b + 120])
        bd.prepare("cal")
        w["cal"] = {"cal": (c[:5] + c[6:]).hex(), "count": e.peek(b + 120, 2).hex()}
    en = e.addr("en")
    if n_life:
        e.call("life_load", [en])
        e.call("energy_init", [en, e.ret, 0, 0x00060A00, 0])
    for _ in range(n_life):
        lt = bytearray(e.peek(en, 48))
        lt[4:20] = bytes(rnd.getrandbits(8) for _ in range(16))                   # on_s, lit_s, ups, dips
        lt[28:45] = bytes(rnd.getrandbits(8) for _ in range(17))                  # trips .. rst_h
        e.poke(en, bytes(lt))
        bd.call("life_store", [en])
        bd.prepare("life")
        w["life"] = {"rec": e.peek(en, 48).hex()}
    if bd.has_hist:
        h = b + 224
        e.poke(h, bytes(64))
        e.call("calhist_load", [h])
        for _ in range(n_hist):
            e.poke(h + 8, bytes(rnd.getrandbits(8) for _ in range(52)))
            e.poke(h + 10, bytes([rnd.getrandbits(8) % 5]))                      # (n <= CALHIST_N)
            bd.call("calhist_store", [h])
            bd.prepare("calhist")
            w["calhist"] = {"rec": e.peek(h, 64).hex()}
    for m in fx_masks:
        hs = heads(rnd)
        for i, hd in enumerate(hs):
            e.poke(bd.ul + i * bd.stride, hd)
        e.poke(b + 300, bytes(crc8(hd) for hd in hs))
        bd.call("scene_fx_store", [bd.ul, m, b + 300])
        w["fx"] = {"mask": m, "crc": bytes(crc8(hd) if m >> i & 1 else 0 for i, hd in enumerate(hs)).hex(),
                   "heads": [hd.hex() if m >> i & 1 else None for i, hd in enumerate(hs)]}
    return w


def pages_hex(area):
    out = []
    for store, (pa, pb, n) in L066.items():
        for base in (pa, pb):
            for a in range(base, base + n, 32):
                off = a - UPPER_SPAN[0]
                out.append("%08X %s" % (a, area[off:off + 32].hex()))
    return "\n".join(out) + "\n"


def md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def same(got, want):
    """a load against what is expected: every field it has (the effects' CRCs: the loaded lights' only)"""
    if "mask" in want:
        m, a, b = want["mask"], bytes.fromhex(got["crc"]), bytes.fromhex(want["crc"])
        return got["mask"] == m and got["heads"] == want["heads"] and all(a[i] == b[i] for i in range(5) if m >> i & 1)
    return all(got.get(k) == v for k, v in want.items())


def cmd_write(elf, out, label, source=""):
    bd = Board(elf)
    write(bd, random.Random(12), 11, 21, 6, 13, 6, [0x1F, 0x15])
    area = bd.area()
    back = Board(elf)                                    # the next boot
    back.e.poke(UPPER_SPAN[0], area)
    man = {"label": label, "source": source, "elf_md5": md5(elf), "made_by": "tools/record_corpus.py write",
           "layout": "upper (src/mcu/ch32v20x/layout.h)", "stores": back.loads()}
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "pages.hex"), "w", newline="\n") as f:
        f.write(pages_hex(area))
    with open(os.path.join(out, "manifest.json"), "w", newline="\n") as f:
        json.dump(man, f, indent=1, sort_keys=True)
        f.write("\n")
    print("%s: corpus %s from %s (%s): the loads %s" % (out, label, elf, source or "-", ", ".join(
        "%s %s %d" % (k, "mask" if "mask" in v else "ret", v.get("mask", v.get("ret")) & 0xFF)
        for k, v in man["stores"].items())))


def load_corpus(d):
    with open(os.path.join(d, "manifest.json")) as f:
        man = json.load(f)
    area = bytearray(b"\xff" * UPPER_SPAN[1])
    with open(os.path.join(d, "pages.hex")) as f:
        for line in f:
            a, h = line.split()
            off = int(a, 16) - UPPER_SPAN[0]
            area[off:off + 32] = bytes.fromhex(h)
    return man, bytes(area)


def cmd_check(elf, dirs):
    fails = 0
    for d in dirs:
        man, area = load_corpus(d)
        label = man["label"]
        bd = Board(elf)
        bd.e.poke(UPPER_SPAN[0], area)
        got = bd.loads()
        res = []
        for store, want in man["stores"].items():
            if store not in got:
                res.append("%s: not in this build" % store)
                continue
            ok = same(got[store], want)
            if store == "fx" and label in SFX1_CORPORA:
                ok = got[store]["mask"] == 0 and all(h is None for h in got[store]["heads"])
                res.append("fx SFX1: %s" % ("rejected" if ok else "UNSAFELY ACCEPTED"))
                fails += not ok
                continue
            res.append("%s: %s" % (store, "read back" if ok else "DIFFERENT"))
            fails += not ok
            if not ok:
                print("  %s %s: got %s, the manifest %s" % (label, store, got[store], want))
        w = write(bd, random.Random(77), 1, 1, 1, 1, 1, [0x0A])               # writes on: one record each
        nb = Board(elf)
        nb.e.poke(UPPER_SPAN[0], bd.area())
        after = nb.loads()
        for store, want in w.items():
            ok = same(after[store], want)
            res.append("%s: writes on %s" % (store, "ok" if ok else "FAILED"))
            fails += not ok
            if not ok:
                print("  %s %s after one more write: got %s, written %s" % (label, store, after[store], want))
        print("corpus %s (%s, %s) into %s: %s" % (label, man.get("source", "-"), d, elf, "; ".join(res)))
    print("record corpus: %s (%d failures)" % ("passed" if not fails else "FAILED", fails))
    return 1 if fails else 0


if __name__ == "__main__":
    if len(sys.argv) >= 5 and sys.argv[1] == "write":
        cmd_write(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5] if len(sys.argv) > 5 else "")
    elif len(sys.argv) >= 4 and sys.argv[1] == "check":
        sys.exit(cmd_check(sys.argv[2], sys.argv[3:]))
    else:
        raise SystemExit(__doc__)
