# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The Ultra-Digital receiver's local-effects record on its flash path (tools/flash_stores.py's flash model, the compiled
firmware in tools/rv32emu.py): ufx_store / ufx_load over the A/B page pair (REC_SET's, two 128-byte records a page), and
main_ud.c's rule that the record is written only on the hub's stream or in the dark.

    python test/test_ud_record_flash.py <ud-api fw.elf>
    (XW=0 LTO=0 EXTRA_CFLAGS="-DBOARD=3 -fno-inline -fno-ipa-cp -fno-ipa-sra" OUT=build/ud-api bash tools/build_target.sh)

1. 20 stores with a changing configuration: every one reads back as the newest (sequence numbers up, both pages used, the
   erase of the other page when a page is full), and a fresh boot of the build on that flash reads the last.
2. A power cut at every half-word of a store: the record being programmed (CRC last, never an erased value) is cut short at
   each of its 64 half-words: the boot loader reads the record before it, never the cut one; the write after the cut
   (the cut entry is used up) lands in the next entry and wins.
3. ufx_flash_service (the main loop's call): a lit local effect writes nothing and erases nothing; a dark strip, or the
   hub's stream, writes after the 2 s debounce; at most one write per 30 s."""
import os
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
from flash_stores import FLASH_R, UPPER_SPAN, fresh  # noqa: E402

REC_A, REC_B = 0x08010000, 0x08011000                   # src/hal/board.h SETTINGS_ADDR / SETTINGS_B_ADDR (REC_SET's pair)
UX_REC = 128
checks = fails = 0


def check(ok, what):
    global checks, fails
    checks += 1
    if not ok:
        fails += 1
        print("FAIL:", what)


elf = sys.argv[1] if len(sys.argv) > 1 else str(FW / "build/ud-api/fw.elf")
e = fresh(elf)
A = e.addr
ux = A("ux")
buf = A("dl_ring")                                      # scratch RAM: the delay ring (not running here)


def seq_of(rec):
    return int.from_bytes(rec[4:8], "little")


def load():
    e.poke(buf, bytes(UX_REC))
    found = e.call("ufx_load", [buf]) or e.ret
    rec = e.peek(buf, UX_REC)
    return (rec if e.ret else None)


def set_config(k):
    e.call("udfx_config", [ux, 131, 10 + k])           # CONFIG 131: pixel count


def store():
    e.call("ufx_store", [ux])
    return e.ret


e.call("udfx_init", [ux])
check(load() is None, "an erased area has no record")
# ---------------------------------------------------------------------------------------------- 1. many stores
last = None
for k in range(20):
    set_config(k)
    r = store()
    check(r == 1, "store %d returned %d (FLASH_OK 1)" % (k, r))
    rec = load()
    e.call("udfx_rec_valid", [buf])
    check(rec is not None and e.ret == 1, "store %d: the record reads back valid" % k)
    check(rec is not None and rec[8 + 4:8 + 6] == (10 + k).to_bytes(2, "little"), "store %d: the newest configuration" % k)
    if last is not None and rec is not None:
        check(seq_of(rec) == seq_of(last) + 1, "store %d: sequence %d after %d" % (k, seq_of(rec), seq_of(last)))
    last = rec
pages = [e.peek(REC_A, 256), e.peek(REC_B, 256)]
check(pages[0][:2] != b"\xff\xff" and pages[1][:2] != b"\xff\xff", "both pages hold records after 20 stores")
check(pages[0][:2] == b"\xa1\xd5" or pages[1][:2] == b"\xa1\xd5", "the magic (little-endian 0xD5A1) is where it should be")
e2 = fresh(elf)                                        # a fresh boot on this flash
e2.poke(REC_A, pages[0])
e2.poke(REC_B, pages[1])
e2.call("ufx_load", [buf])
check(e2.ret == 1 and e2.peek(buf + 12, 2) == (29).to_bytes(2, "little"), "a fresh boot reads the last record (pixels 29)")

# ---------------------------------------------------------------------------------------------- 2. a power cut in a store
e = fresh(elf)
A = e.addr
ux, buf = A("ux"), A("dl_ring")
e.call("udfx_init", [ux])
for k in range(5):
    set_config(k)
    store()
base = [e.peek(REC_A, 256), e.peek(REC_B, 256)]
good = load()
check(good is not None and good[12:14] == (14).to_bytes(2, "little"), "the record before the cuts")
set_config(40)
store()
new_rec = None
for pg, a in enumerate((REC_A, REC_B)):
    now = e.peek(a, 256)
    if now != base[pg]:
        off = next(i for i in range(0, 256, UX_REC) if now[i:i + UX_REC] != base[pg][i:i + UX_REC])
        new_rec, new_at = now[off:off + UX_REC], a + off
check(new_rec is not None and new_rec[12:14] == (50).to_bytes(2, "little"), "the store went to a new entry")
cut_ok = 0
for j in range(0, UX_REC // 2):                        # j half-words of the new record programmed, the rest erased
    ec = fresh(elf)
    ec.poke(REC_A, base[0])
    ec.poke(REC_B, base[1])
    ec.poke(new_at, new_rec[:2 * j] + b"\xff" * (UX_REC - 2 * j))
    ec.call("ufx_load", [buf])
    got = ec.peek(buf, UX_REC) if ec.ret else None
    ok = got == good if j < UX_REC // 2 else got == new_rec
    cut_ok += ok
    check(ok, "a cut after %d of 64 half-words: the loader read %s" %
          (j, "nothing" if got is None else "the %s record" % ("new" if got == new_rec else "older" if got == good else "WRONG")))
ec = fresh(elf)
ec.poke(REC_A, base[0])
ec.poke(REC_B, base[1])
ec.poke(new_at, new_rec)
ec.call("ufx_load", [buf])
check(ec.ret == 1 and ec.peek(buf, UX_REC) == new_rec, "the whole record reads back")
print("  a cut at each of 64 half-words: %d read the record before it, the whole one reads back" % cut_ok)

# ---------------------------------------------------------------------------------------------- 3. the policy: lit / dark / stream
import glob, subprocess, struct, tempfile  # noqa: E402,E401
src = open(FW / "test/test_ud_pages.py", encoding="utf-8").read()
g = dict(glob=glob, os=os, subprocess=subprocess, tempfile=tempfile, struct=struct, Path=Path, FW=FW)
exec(compile(src[src.index("def target_gcc"):src.index("def every_symbol")], "offsets", "exec"), g)   # its offsets()
O = g["offsets"](elf)

e = fresh(elf)
A = e.addr
ux, ms_now = A("ux"), A("ms_now")
e.call("udfx_init", [ux])
log = []
orig_wr = e.wr


def wr(a, n, v):
    if a == FLASH_R + 0x10 and v & 0x40 and v & 0x20000:   # STRT with PAGE_ER: an erase
        log.append("erase")
    elif UPPER_SPAN[0] <= a < UPPER_SPAN[0] + UPPER_SPAN[1] and "program" not in log:   # a half-word into the records
        log.append("program")
    return orig_wr(a, n, v)


e.wr = wr


def pages_now():
    return e.peek(REC_A, 256) + e.peek(REC_B, 256)


def service(ms):
    e.wr(ms_now, 4, ms)
    log.clear()
    e.call("ufx_flash_service")
    return list(log)


e.call("udfx_config", [ux, 131, 12])                    # the record is due since ux.ms = 0
check(e.rd(ux + O["ux_save_due"], 1) == 1, "a CONFIG makes the record due")
e.wr(ux + O["ux_local"], 1, 1)                           # a lit LOCAL strip
check(service(5000) == [] and pages_now() == bytes([0xFF]) * 512, "a lit local effect: no flash operation at all")
e.wr(ux + O["ux_local"], 1, 0)
check(service(1000) == [], "on the stream: the 2 s debounce (due since 0 ms, 1 s later: not yet)")
w = service(5000)
check("program" in w and "erase" not in w and e.rd(ux + O["ux_save_due"], 1) == 0, "on the stream, after 2 s: one program: %s" % w)
check(load() is not None, "... and the record reads back")
e.call("udfx_config", [ux, 131, 13])
e.wr(ux + O["ux_save_ms"], 4, 5000)
w = service(9000)
check(w == [], "a second record within 30 s of the first: held (%s, last %d)" % (w, e.rd(A("ufx_last_ms"), 4)))
e.wr(ux + O["ux_local"], 1, 1)
e.wr(ux + O["ux_dark"], 1, 1)                            # LOCAL but dark: allowed
w = service(40000)
check(w == ["program"] and e.rd(ux + O["ux_save_due"], 1) == 0, "a dark local strip, 35 s later: the program: %s (due %d, save_ms %d, ms %d)" % (w, e.rd(ux + O["ux_save_due"], 1), e.rd(ux + O["ux_save_ms"], 4), e.rd(ms_now, 4)))
e.wr(ux + O["ux_dark"], 1, 0)
e.call("udfx_config", [ux, 131, 14])
e.wr(ux + O["ux_save_ms"], 4, 70000)
check(service(120000) == [], "lit again, a record due: nothing is written")
# the erase ahead: when one page holds the newest records and the other is due for an erase, it happens in the dark only
e.wr(ux + O["ux_local"], 1, 0)
for k in range(8):                                       # fill: the pages wrap (2 a page), an erase when it is due
    e.call("udfx_config", [ux, 131, 100 + k])
    e.wr(ux + O["ux_save_ms"], 4, 200000 + k * 40000)
    ops = service(200000 + k * 40000 + 31000)
    check("program" in ops, "store %d: %s" % (k, ops))
    last_ops = ops
print("  policy: lit = no flash, debounce 2 s, one write per 30 s, dark and stream write; the pages wrapped with erases when due")
rec = load()
check(rec is not None and rec[12:14] == (107).to_bytes(2, "little"), "the last record after the wraps")

print("ud record flash: %d checks, %d failed" % (checks, fails))
sys.exit(1 if fails else 0)
