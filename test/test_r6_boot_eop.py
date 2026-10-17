# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run real compiled bootloader flash helpers with EOP/WRP/unlock failures.
Usage: python test/test_r6_boot_eop.py build/bootloader/bl.elf [hardware_id]
The optional hardware id (1, 2 or 3) selects the integration fixture's descriptor;
default 1 preserves the original test. Receiver revisions are 0, main board is 3.
"""
from pathlib import Path
import struct
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rv32emu import Emu

FLASH = 0x40022000
hw_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
assert hw_id in (1, 2, 3), 'hardware id must be 1, 2 or 3'
hw_rev = 3 if hw_id == 1 else 0

def word_crc(c, w):
    c ^= w
    for _ in range(32):
        c = ((c << 1) ^ (0x04c11db7 if c & 0x80000000 else 0)) & 0xffffffff
    return c

def crc32(data):
    c = 0xffffffff
    for (w,) in struct.iter_unpack('<I', data):
        c = word_crc(c, w)
    return c

def crc(data, width, poly, init):
    c = init
    for b in data:
        c ^= b << (width-8)
        for _ in range(8):
            c = ((c << 1) ^ (poly if c & (1 << (width-1)) else 0)) & ((1 << width)-1)
    return c

class Flash(Emu):
    def __init__(self, elf, mode):
        super().__init__(elf)
        self.mode = mode
        self.keys = []
        self.once = False
        self.mmio[FLASH + 0x10] = 0x8080
        self.mmio[FLASH + 0xc] = 0

    def done(self):
        self.mmio[FLASH + 0xc] = {'ok': 0x20, 'no_eop': 0, 'locked': 0, 'wrp': 0x10}[self.mode]
        ok = self.mode == 'ok'
        if self.once:
            self.once = False
            self.mode = 'ok'
        return ok

    def wr(self, a, n, v):
        if a == 0x40023008:
            self.mmio[0x40023000] = 0xffffffff
            return
        if a == 0x40023000:
            self.mmio[a] = word_crc(self.mmio.get(a, 0xffffffff), v)
            return
        if a in (FLASH + 4, FLASH + 0x24):
            self.keys.append((a, v))
            if len(self.keys) > 1 and self.keys[-2] == (a, 0x45670123) and v == 0xcdef89ab and self.mode != 'locked':
                self.mmio[FLASH + 0x10] &= ~(0x80 if a == FLASH + 4 else 0x8000)
            return
        if a == FLASH + 0xc:
            self.mmio[a] &= ~(v & 0x30)
            return
        if a == FLASH + 0x10:
            if v & 0x40 and v & (2 | 0x20000):
                if self.done():
                    base = self.mmio.get(FLASH + 0x14, 0)
                    if base >= 0x08002000:
                        self.poke(base, b'\xff' * (4096 if v & 2 else 256))
                v &= ~0x40
            if v & 0x200000 and v & 0x10000:
                self.done()
            self.mmio[a] = v
            return
        if 0x08002000 <= a < 0x08010000:
            if self.mmio[FLASH+0x10] & 1:
                self.done()
            return
        if self._mem(a, n)[0] is None:
            self.mmio[a] = v
        super().wr(a, n, v)

failed = 0
for fn, args in [('bl_plat_erase_sector', [0x08002000]), ('bl_plat_erase_page', [0x08002000]),
                 ('bl_plat_program_half', [0x08002040, 0xa5b7]), ('bl_plat_program_page', [0x08002100, 0x20003000])]:
    for mode in ['ok', 'no_eop', 'locked', 'wrp']:
        e = Flash(sys.argv[1], mode)
        e.call(fn, args)
        ok = (e.ret == 0) == (mode == 'ok')
        print(fn, mode, 'return', hex(e.ret), 'PASS' if ok else 'FAIL')
        failed += not ok

# Integrate the actual compiled session and reset decision, not just its HAL:
# first sector erase is a no-op without EOP; any later erase succeeds. A power
# cut immediately after accept must never boot the damaged old confirmed body.
body = bytes((i*71+3) & 255 for i in range(6000))
desc = bytearray(64)
desc[:6] = b'QDU5\x01' + bytes([hw_id])
struct.pack_into('>HIIIII', desc, 6, 1 << hw_rev, 0x000606ff, len(body), crc32(body), 0x08002100, 0x2100)
desc[28] = 2
struct.pack_into('>I', desc, 60, crc32(desc[:60]))
e = Flash(sys.argv[1], 'no_eop')
e.poke(0x08002000, desc + b'\xb7\xa5\xc3\xa5' + b'\xff'*(256-68))
e.poke(0x08002100, body)
app, frame = 0x20004700, 0x20004740
e.call('bl_app_read', [app])
assert e.peek(app, 3) == b'\x01\x01\x01', 'confirmed factory fixture'
sess = e.addr('sess')
e.call('bl_sess_init', [sess, hw_rev, app, 0])
desc[10] = 7
struct.pack_into('>I', desc, 60, crc32(desc[:60]))
e.once = True
for half in (0, 1):
    f = bytearray(48)
    f[0], f[1] = 0x81, 1
    struct.pack_into('>H', f, 2, half)
    f[6:38] = desc[half*32:(half+1)*32]
    struct.pack_into('>H', f, 38, crc(f[:38], 16, 0x1021, 0xffff))
    f[40:43] = b'\x5a\xd2\x28'
    f[47] = crc(f[40:47], 8, 7, 0)
    e.poke(frame, f)
    e.call('bl_sess_frame', [sess, 384, frame, half])
assert not e.once, 'session must attempt the descriptor-sector erase'
e.call('bl_app_read', [app])
e.call('bl_decide', [app, 0x20000000, 1, 0x08000000])
body_ok = e.peek(0x08002100, len(body)) == body
ok = e.ret != 0 or body_ok
print('reset after failed descriptor-sector erase: action', e.ret, 'old body intact', body_ok, 'PASS' if ok else 'FAIL')
failed += not ok
sys.exit(bool(failed))
