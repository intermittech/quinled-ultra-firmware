# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Target identity across MCU families: the actual bootloader descriptor checkers (bl_desc_check, the QBA1 descriptor
entry the applications call too) of a CH32V203 and a CH32V407 Digital-M bootloader, run in rv32emu. Each accepts a
valid descriptor for its own immutable target and refuses the other family's (UPD_ERROR_TARGET: MCU, package and
layout bytes differ), whatever the bodies' bounds. The two Digital-M builds carry the same board revision (the
pre-production boards' provisional v0r1, 0x01; final hardware becomes v1r1, 0x11: revision byte = major << 4 | minor):
the families differ by MCU / package / layout, never by the revision.

    python test/test_v407_identity.py <V203 bootloader folder> <V407 bootloader folder>   (each with bl.elf / bl.bin)
"""
import pathlib
import struct
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'tools'))
from rv32emu import Emu  # noqa: E402


def seal(d):
    crc = 0xffffffff
    for i in range(0, 60, 4):
        crc ^= int.from_bytes(d[i:i + 4], 'little')
        for _ in range(32):
            crc = ((crc << 1) ^ (0x04c11db7 if crc & 0x80000000 else 0)) & 0xffffffff
    d[60:64] = crc.to_bytes(4, 'big')
    return d


def descriptor(target, length=16):
    """a valid v2 descriptor for this 16-byte immutable target"""
    base = struct.unpack('>I', target[8:12])[0]
    d = bytearray(64)
    d[:4] = b'QDU6'
    d[4] = 2
    d[6] = d[7] = target[7]
    d[8:15] = target[:7]
    d[15] = 2
    struct.pack_into('>IIIII', d, 16, 0x00030000, length, 0xdeadbeef, base, base)
    d[36:39] = bytes([2, 3, 1])
    return seal(d)


def main():
    folders = [pathlib.Path(a) for a in sys.argv[1:3]]
    emus, targets = [], []
    for f in folders:
        emus.append(Emu(str(f / 'bl.elf')))
        info = (f / 'bl.bin').read_bytes()[0x10:0x30]
        assert info[:4] == b'QDB2', f
        targets.append(info[5:21])
    assert targets[0][1] == 1 and targets[1][1] == 4, 'expected a V203 (MCU 1) and a V407 (MCU 4) bootloader'
    checks = 0
    assert targets[0][0] == targets[1][0] == 3, 'both the Digital-M (board 3)'
    assert targets[0][7] == targets[1][7] == 0x01, f'revisions {targets[0][7]:#04x} / {targets[1][7]:#04x}: both v0r1'
    checks += 2
    for i, e in enumerate(emus):
        for j, t in enumerate(targets):
            e.poke(0x20004000, bytes(descriptor(t)))
            e.call('bl_desc_check', [0x20004000])
            expected = 0 if i == j else 2
            assert e.ret == expected, (i, j, e.ret)
            checks += 1
            # the other family's tuple with this family's bounds is still refused: identity, not geometry
            if i != j:
                x = descriptor(targets[i])
                x[9:12] = t[1:4]
                e.poke(0x20004000, bytes(seal(x)))
                e.call('bl_desc_check', [0x20004000])
                assert e.ret == 2, (i, j, e.ret)
                checks += 1
    print(f'{checks} cross-family descriptor checks PASS: V203 {targets[0][:8].hex()} / V407 {targets[1][:8].hex()}')


if __name__ == '__main__':
    main()
