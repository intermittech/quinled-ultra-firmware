# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The flash record paths in tools/rv32emu.py, on the compiled firmware: two builds compared.

    python tools/flash_stores.py <old fw.elf> <new fw.elf> [<new image>]

Every record area starts erased (0xFF). Settings (19 stores: page wraps), the scene log (97: across its pages),
the calibration page (9 tests, then read back as the next boot does), the energy log (40 entries, erase allowed
every 5th) and the power-on effect store (twice: all lights, then none; then energy_clear). Compared per build: the
bytes of every area, every peripheral write in order, every return value and what the loads read back.
An LTO build inlines these functions (core/lto.h MEASURED): pass its measurement build (MEASURE=1) as <new fw.elf>,
and the image itself as <new image>. The settings path also runs through main.c's save_now (the console's `save`),
which every build keeps: that part runs in <new image> (else <new fw.elf>), with the settings poked into main.c's
set / sext. (After a review tool by Fable, 2026-10-13.)
"""
import hashlib
import random
import struct
import sys

from rv32emu import Emu

REG = {"cal": (0x0800F600, 0x100), "scene": (0x0800F700, 0x500), "fx": (0x0800FC00, 0x200),
       "energy": (0x0800FE00, 0x100), "settings": (0x0800FF00, 0x100)}


def fresh(elf):
    e = Emu(elf)
    for a, n in REG.values():
        e.poke(a, b"\xff" * n)
    return e


def stores(elf):
    e = fresh(elf)
    rnd = random.Random(3)
    buf = e.addr("din_ring") + 64
    log = []
    e.call("settings_load", [buf, buf + 32])
    log.append(("load", e.ret))
    for _ in range(19):
        e.poke(buf, bytes(rnd.getrandbits(8) for _ in range(16)))
        e.poke(buf + 32, bytes(rnd.getrandbits(8) for _ in range(7)))
        e.call("settings_store", [buf, buf + 32])
        log.append(("set", e.ret))
    e.call("scene_load", [buf + 64])
    log.append(("sload", e.ret))
    for _ in range(97):
        e.poke(buf + 64, bytes(rnd.getrandbits(8) for _ in range(10)) + bytes([rnd.getrandbits(5), rnd.getrandbits(8)]))
        e.call("scene_store", [buf + 64])
        log.append(("scene", e.ret))
    e.call("scene_load", [buf + 80])
    log.append(("sload2", e.ret, e.peek(buf + 80, 12)))
    e.call("cal_load", [buf + 128, buf + 200])
    log.append(("cload", e.ret))
    for _ in range(9):
        levels = [0, 1, 4][rnd.getrandbits(8) % 3]
        c = struct.pack("<B4Bxh", levels, 25, 50, 75, 100, rnd.getrandbits(15) - 100)
        c += struct.pack("<20H", *[rnd.getrandbits(16) for _ in range(20)])
        e.poke(buf + 128, c)
        e.call("cal_store", [buf + 128, buf + 200])
        log.append(("cal", e.ret, e.peek(buf + 200, 2)))
    e.call("cal_load", [buf + 300, buf + 400])                 # what the next boot reads back
    back = e.peek(buf + 300, 48)
    log.append(("cload2", e.ret, e.peek(buf + 400, 2), back[:5] + back[6:]))   # (byte 5: padding)
    e.call("energy_load", [])
    log.append(("eload", e.ret))
    for k in range(40):
        e.call("energy_store", [1000 + k * 37, int(k % 5 == 0)])
        log.append(("en", e.ret))
    e.call("energy_load", [])
    log.append(("eload2", e.ret))
    ul = e.addr("ul")                                          # the power-on effects: the engine's light heads
    e.poke(ul, bytes(rnd.getrandbits(8) for _ in range(5 * 98)))
    crc = buf + 500
    e.poke(crc, bytes(rnd.getrandbits(8) for _ in range(5)))
    e.call("scene_fx_store", [ul, 0x1F, crc])
    log.append(("fx", e.ret))
    fx_all = e.peek(*REG["fx"])
    e.call("scene_fx_store", [ul, 0, crc])
    log.append(("fx0", e.ret))
    e.call("energy_clear", [])
    log.append(("eclear", e.ret))
    img = {r: e.peek(a, n) for r, (a, n) in REG.items()}
    img["fx (all lights)"] = fx_all
    return log, img, e.writes


def save_now(elf):
    """main.c's save_now in the image: 19 saves of poked settings (page wraps)"""
    e = fresh(elf)
    rnd = random.Random(5)
    s, x = e.sym("set"), e.sym("sext")
    assert (s[1], x[1]) == (16, 7), (s, x)
    log = []                                                   # (void: the flash and the writes are compared)
    for _ in range(19):
        e.poke(s[0], bytes(rnd.getrandbits(8) for _ in range(16)))
        e.poke(x[0], bytes(rnd.getrandbits(8) for _ in range(7)))
        log.append(e.call("save_now", []) > 0)
    return log, {"settings": e.peek(*REG["settings"])}, e.writes


def compare(what, a, b):
    print("%s: %d calls; %s" % (what, len(a[0]), ", ".join("%s %s" % (r, hashlib.md5(v).hexdigest()[:8])
                                                          for r, v in b[1].items())))
    ok = a[0] == b[0] and a[1] == b[1] and a[2] == b[2]
    print("  returns / read-backs identical: %s; areas identical: %s; peripheral writes identical: %s (%d)" %
          (a[0] == b[0], {r: a[1][r] == b[1][r] for r in a[1]}, a[2] == b[2], len(b[2])))
    return ok


if __name__ == "__main__":
    old, new = sys.argv[1], sys.argv[2]
    img = sys.argv[3] if len(sys.argv) > 3 else new
    ok = compare("stores", stores(old), stores(new))
    ok &= compare("save_now (settings, %s)" % img, save_now(old), save_now(img))
    print("IDENTICAL" if ok else "DIFFERENT")
    sys.exit(0 if ok else 1)
