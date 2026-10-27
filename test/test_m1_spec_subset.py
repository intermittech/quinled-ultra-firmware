# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run unchanged framing/checksum codecs and the widened pitch against spec goldens.

Usage: python test/test_m1_spec_subset.py <LTO=0 API ELF> <protocol repo>
This is deliberately a named subset: it does not validate wire-3 command dispatch,
page payload semantics, descriptors, update compatibility or sample decoding.
The ELF needs -fno-inline -fno-ipa-cp -fno-ipa-sra, as the UR_DIGITAL_M page harness does.
"""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
from rv32emu import Emu

elf, spec = sys.argv[1], pathlib.Path(sys.argv[2])
e = Emu(elf)
ring = "din_ring" if "din_ring" in e.syms else "dl_ring"
scratch = e.addr(ring)
assert e.syms[ring][1] >= 1056
has_slot_validator = "ultra_slot_check" in e.syms
cases = json.loads((spec / "test-vectors/vectors.json").read_text())["vectors"]
checks = 0
for case in cases:
    data = bytes.fromhex(case["hex"])
    kind = case["kind"]
    if kind not in ("slot", "header", "answer", "descriptor"):
        continue
    assert len(data) <= 1024
    e.poke(scratch, data)
    if kind == "header":
        e.call("ultra_crc8", [scratch, 7])
        assert e.ret == data[7], case["name"]
    else:
        offset, length, expected = (6, len(data) - 6, int.from_bytes(data[4:6], "big")) if kind == "descriptor" else (0, 30, int.from_bytes(data[30:32], "big"))
        e.call("ultra_crc16", [scratch + offset, length])
        assert e.ret == expected, case["name"]
    checks += 1
    if kind == "slot" and has_slot_validator:
        e.call("ultra_slot_check", [scratch])
        assert e.ret == 0, case["name"]
        e.poke(scratch, bytes([data[0] ^ 1]))
        e.call("ultra_slot_check", [scratch])
        assert e.ret != 0, case["name"]
        checks += 2
    if kind == "answer":
        fields = case["fields"]
        poll = bytearray(32)
        poll[2], poll[3] = fields["seq"], fields["index"]
        e.poke(scratch + 1024, poll)
        e.poke(scratch, bytes(6) + bytes.fromhex(fields["data"]) + bytes(2))
        e.call("rc_answer", [scratch, scratch + 1024, fields["page"], fields["flags"], fields["uid16"]])
        assert e.peek(scratch, 32) == data, case["name"]
        checks += 1
for row in json.loads((spec / "test-vectors/vectors_timing.json").read_text())["rates"]:
    e.call("ultra_rw_slot_us", [row["rate"]])
    assert e.ret == row["pitch"]
    assert 1000 + 16 * e.ret == row["window16"]
    checks += 2
for rate in (3, 255):
    e.call("ultra_rw_slot_us", [rate])
    assert e.ret == 0
    checks += 1
print(f"M1 checkpoint spec subset: {checks} checks, 0 failures ({elf})")
print("Additional slot CRC validator:", "checked" if has_slot_validator else "not linked in this image (CRC codec checked)")
print("Scope: framing/checksum codec and pitch only; full wire-3 receiver validation remains pending.")
