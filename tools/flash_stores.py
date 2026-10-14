# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The flash record paths in tools/rv32emu.py, on the compiled firmware: two builds compared.

    python tools/flash_stores.py <old fw.elf> <new fw.elf> [<new image>]

Every record area starts erased (0xFF). Settings (19 stores: page wraps), the scene log (97: across its pages),
the calibration page (9 tests, then read back as the next boot does), the lifetime record (0.6.3: 40 records, the
pending erase done after each except every 5th, as main.c does while dark, then read back as the next boot does; up to
0.6.2: the energy log, 40 entries, erase allowed every 5th, and energy_clear) and the power-on effect store (twice: all
lights, then none). Compared per build: the bytes of every area, every peripheral write in order, every return value
and what the loads read back. Builds on both sides of 0.6.3 compare the areas they share (settings, calibration,
effects).
An LTO build inlines these functions (core/lto.h MEASURED): pass its measurement build (MEASURE=1) as <new fw.elf>,
and the image itself as <new image>. The settings path also runs through main.c's save_now (the console's `save`),
which every build keeps: that part runs in <new image> (else <new fw.elf>), with the settings poked into main.c's
set / sext. (After a review tool by Fable, 2026-10-13.)
0.6.4 changed the flash path itself (hal/hw.c flash_op: HCLK / 2, the SRAM tick, the result codes; A/B pairs for the
settings, calibration and scene log, erases ahead from main.c): its peripheral writes and record positions differ from
0.6.3 by design, so compare 0.6.4 builds with each other (the record formats are 0.6.3's).
0.6.6: the flash controller as flash_op uses it (FlashEmu): the keys unlock, STATR says EOP, a STRT with PAGE_ER erases
the 256-byte page at ADDR, a program only clears bits. Before, every operation returned FLASH_ERR_EOP (the bytes were
written, no page erased), so the stores' error handling ran instead of their normal path.
0.6.6 moved each store into its own two 4 KB sectors (src/hal/board.h): an upper-layout build's map is found by where
its calibration goes, the areas are compared per store and page, and the flash ADDR register's values (the erases) as
(store, page, offset), so 0.6.5 and 0.6.6 builds compare with each other.
"""
import hashlib
import random
import struct
import sys

from rv32emu import Emu

import os
UPPER = os.environ.get("UPPER_RECORDS", "1") != "0"
# the upper layouts (src/hal/board.h): each store's page A, page B and their size. 0.6.5: every A page in the sector
# 0x08010000, every B page in 0x08011000; 0.6.6: each store its own two sectors
L065 = {"settings": (0x08010000, 0x08011000, 0x100), "cal": (0x08010100, 0x08011100, 0x100),
        "scene": (0x08010200, 0x08011200, 0x100), "life (its two pages)": (0x08010300, 0x08011300, 0x100),
        "fx": (0x08010400, 0x08011400, 0x200)}
L066 = {"settings": (0x08010000, 0x08011000, 0x100), "cal": (0x08012000, 0x08013000, 0x100),
        "scene": (0x08014000, 0x08015000, 0x100), "life (its two pages)": (0x08016000, 0x08017000, 0x100),
        "fx": (0x08018000, 0x08019000, 0x200)}
UPPER_SPAN = (0x08010000, 0xC000)                    # the upper record area, erased at the start
# 0.6.4's layout
REG = {"cal": (0x0800F600, 0x100), "scene": (0x0800F700, 0x500), "fx": (0x0800FC00, 0x200),
       "energy": (0x0800FE00, 0x100), "settings": (0x0800FF00, 0x100)}
LIFE_PAGES = (0x0800FB00, 0x0800FE00)
LIFE = "life_store"           # 0.6.3: the lifetime record in two pages (0x0800FB00 = the scene log's 5th until then)


FLASH_R = 0x40022000                 # the flash controller (RM 32.4): KEYR +4, STATR +0x0C, CTLR +0x10, ADDR +0x14,
STATR_EOP, CTLR_PG, CTLR_STRT, CTLR_PAGE_ER = 0x20, 0x01, 0x40, 0x00020000   # MODEKEYR +0x24
CTLR_LOCK, CTLR_FAST_LOCK = 0x80, 0x8000


class FlashEmu(Emu):
    """+ flash_op's controller: the keys unlock, every operation ends with EOP, a page erase sets 256 bytes to 0xFF,
    a program (PG) only clears bits"""
    def __init__(self, elf):
        super().__init__(elf)
        self.mmio[FLASH_R + 0x10] = CTLR_LOCK | CTLR_FAST_LOCK

    def rd(self, a, n):
        if a == FLASH_R + 0x0C:
            return STATR_EOP
        return super().rd(a, n)

    def wr(self, a, n, v):
        ctl = self.mmio.get(FLASH_R + 0x10, 0)
        if a == FLASH_R + 0x04:
            self.mmio[FLASH_R + 0x10] = ctl & ~CTLR_LOCK
        elif a == FLASH_R + 0x24:
            self.mmio[FLASH_R + 0x10] = ctl & ~CTLR_FAST_LOCK
        elif a == FLASH_R + 0x10:
            if v & CTLR_STRT and v & CTLR_PAGE_ER:
                self.poke(self.mmio.get(FLASH_R + 0x14, 0) & ~0xFF, b"\xff" * 256)
            self.mmio[a] = v & 0xFFFFFFFF
        elif a == FLASH_R + 0x14:
            self.mmio[a] = v & 0xFFFFFFFF
        elif 0x08000000 <= a < 0x08040000 and ctl & CTLR_PG:
            v &= super().rd(a, n)                        # (NOR flash: a program clears bits)
        super().wr(a, n, v)


def fresh(elf):
    e = FlashEmu(elf)
    if UPPER:
        e.poke(UPPER_SPAN[0], b"\xff" * UPPER_SPAN[1])
    else:
        for a, n in REG.values():
            e.poke(a, b"\xff" * n)
    return e


def layout(elf):
    """an upper-layout build's map: 0.6.6's if a calibration record (a cleared one) goes to 0x08012000"""
    e = fresh(elf)
    buf = e.addr("din_ring") + 64
    e.call("cal_load", [buf, buf + 200])
    e.poke(buf, bytes(48))
    e.call("cal_store", [buf, buf + 200])
    return L066 if e.peek(0x08012000, 4) != b"\xff" * 4 else L065


def canon(lay, writes):
    """the flash ADDR register's values (the erases' pages) as (store, page, offset): the same in either layout"""
    def c(a):
        for s, (pa, pb, n) in lay.items():
            for p, base in enumerate((pa, pb)):
                if base <= a < base + n:
                    return (s, p, a - base)
        return a
    return [(a, c(v)) if a == FLASH_R + 0x14 else (a, v) for a, v in writes] if lay else writes


def stores(elf):
    lay = layout(elf) if UPPER else None
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
    if LIFE in e.syms:                                         # 0.6.3: the lifetime record
        en = e.addr("en")
        e.call("life_load", [en])
        log.append(("lload", e.ret))
        e.call("energy_init", [en, e.ret, 0, 0x00060300, 0])
        for k in range(40):
            e.poke(en + 4, struct.pack("<I", 1000 + k * 37))       # (on_s: the record changes each time)
            e.call("life_store", [en])
            log.append(("life", e.ret))
            if k % 5 != 4:                                     # (every 5th time the erase waits: the store fails)
                # 0.6.6: one path for the four record pairs, flash_rec_pending / flash_rec_prepare(REC_LIFE = 0);
                # before: life_pending / life_prepare
                rec = "flash_rec_pending" in e.syms
                e.call("flash_rec_pending" if rec else "life_pending", [0] if rec else [])
                if e.ret:
                    e.call("flash_rec_prepare" if rec else "life_prepare", [0] if rec else [])
                    log.append(("prep", e.ret))
        e.call("life_load", [buf + 600])
        log.append(("lload2", e.ret, e.peek(buf + 600, 40)))
    else:
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
    fx_all = (e.peek(lay["fx"][0], 0x200) + e.peek(lay["fx"][1], 0x200)) if lay else e.peek(*REG["fx"])
    e.call("scene_fx_store", [ul, 0, crc])
    log.append(("fx0", e.ret))
    if LIFE not in e.syms:
        e.call("energy_clear", [])
        log.append(("eclear", e.ret))
    if lay:
        img = {r: e.peek(a, n) + e.peek(b, n) for r, (a, b, n) in lay.items()}
    else:
        img = {r: e.peek(a, n) for r, (a, n) in REG.items()}
        img["life (its two pages)"] = e.peek(LIFE_PAGES[0], 0x100) + e.peek(LIFE_PAGES[1], 0x100)
    img["fx (all lights)"] = fx_all
    return log, img, canon(lay, e.writes)


def save_now(elf):
    """main.c's save_now in the image: 19 saves of poked settings (page wraps)"""
    lay = layout(elf) if UPPER else None
    e = fresh(elf)
    rnd = random.Random(5)
    s, x = e.sym("set"), e.sym("sext")
    assert (s[1], x[1]) == (16, 7), (s, x)
    if "uart_write" in e.syms and any(k == "cli" or k.startswith("cli.lto_priv.") for k in e.syms):
        # (save_now's reply goes through cli_say, which writes through cli.write: set as main() does at boot)
        e.poke(e.sym("cli")[0], struct.pack("<I", e.sym("uart_write")[0]))
    log = []                                                   # (void: the flash and the writes are compared)
    for _ in range(19):
        e.poke(s[0], bytes(rnd.getrandbits(8) for _ in range(16)))
        e.poke(x[0], bytes(rnd.getrandbits(8) for _ in range(7)))
        if "uart_busy" in e.syms or any(k.startswith("uart_tx_len") for k in e.syms):
            # 0.6.4: save_now waits (<= 100 ms of ms_now, which the emulator never advances) until the console's
            # output is sent: the last call's line taken as sent, the DMA done, USART2 TC set
            t = [k for k in e.syms if k.startswith("uart_tx_len")]
            if t:
                e.poke(e.sym(t[0])[0], bytes(4))                # (uart_tx_len: u32)
            e.mmio[0x40020084] = 0                                 # DMA1 channel 7 CNTR
            e.mmio[0x40004400] = 0xC0                              # USART2 STATR: TXE, TC
        log.append(e.call("save_now", []) > 0)
    a = lay["settings"] if lay else (0x0800FF00, 0x0800F900)   # the settings' A / B pages
    return log, {"settings": e.peek(a[0], 0x100) + e.peek(a[1], 0x100)}, canon(lay, e.writes)


def compare(what, a, b, shared=None):
    """shared: only these areas, and not the calls / writes (builds on both sides of a format change)"""
    print("%s: %d calls; %s" % (what, len(a[0]), ", ".join("%s %s" % (r, hashlib.md5(v).hexdigest()[:8])
                                                          for r, v in b[1].items())))
    if shared:
        same = {r: (a[1][r][:s] == b[1][r][:s]) for r, s in shared.items()}
        print("  (across the 0.6.3 format change) shared areas identical: %s" % same)
        return all(same.values())
    ok = a[0] == b[0] and a[1] == b[1] and a[2] == b[2]
    print("  returns / read-backs identical: %s; areas identical: %s; peripheral writes identical: %s (%d)" %
          (a[0] == b[0], {r: a[1][r] == b[1][r] for r in a[1]}, a[2] == b[2], len(b[2])))
    return ok


if __name__ == "__main__":
    old, new = sys.argv[1], sys.argv[2]
    img = sys.argv[3] if len(sys.argv) > 3 else new
    sa, sb = stores(old), stores(new)
    if any(k[0] == "life" for k in sa[0]) != any(k[0] == "life" for k in sb[0]):
        ok = compare("stores", sa, sb, {"settings": 256, "cal": 256, "fx": 512, "fx (all lights)": 512})
    else:
        ok = compare("stores", sa, sb)
    ok &= compare("save_now (settings, %s)" % img, save_now(old), save_now(img))
    print("IDENTICAL" if ok else "DIFFERENT")
    sys.exit(0 if ok else 1)
