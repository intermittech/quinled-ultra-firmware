# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 app's declared group and selected status update entry. A fresh app
accepts matching group ANNOUNCE; SELECT binds this UID or explicitly excludes it,
and excluded/wrong-session ANNOUNCE never falls back to group entry. core/upd_app.c compiled for RV32 (-DMCU_CH32V40X=1, the
pure frame checks) and run in rv32emu with the real V407 bootloader's identity.

    python test/test_upd_select_v407.py <V407 bl.bin> <out dir> [--src <firmware tree>]
"""
import argparse
import json
import pathlib
import struct
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'test'))
from ur_digital_m_target import target_gcc   # noqa: E402

BENCH = r'''
#include "upd_app.h"
upd_app_t u;
upd_app_info_t info;
uint8_t bl[32], page[256], uid[12], fr[48];
void *memcpy(void *d, const void *s, unsigned n) { unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }
int memcmp(const void *x, const void *y, unsigned n)
{ const unsigned char *a = x, *b = y; for (; n--; a++, b++) if (*a != *b) return *a - *b; return 0; }
void b_init(void) { }
void b_setup(void)
{
    upd_app_read_info(&info, page, bl);
    upd_app_init(&u, &info);
#ifdef UPD_STATUS_PROFILE
    u.uid = uid;
#endif
}
unsigned b_frame(void) { return upd_app_frame(&u, fr, 1000u); }
void b_end(unsigned flags) { upd_app_frame_end(&u, 1u, (uint8_t)flags, 1000u); }
unsigned b_poll(unsigned now) { return upd_app_poll(&u, now); }
'''
UID = bytes(range(0x31, 0x3d))
TAKE = 1


def crc16(d):
    c = 0xffff
    for v in d:
        c ^= v << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021 if c & 0x8000 else c << 1) & 0xffff
    return c


def crc8(d):
    c = 0
    for v in d:
        c ^= v
        for _ in range(8):
            c = ((c << 1) ^ 7 if c & 0x80 else c << 1) & 0xff
    return c


def crc32(data):
    c = 0xffffffff
    for (v,) in struct.iter_unpack('<I', data):
        c ^= v
        for _ in range(32):
            c = ((c << 1) ^ 0x04c11db7 if c & 0x80000000 else c << 1) & 0xffffffff
    return c


def frame(op, session, index, data, flags=0):
    b = bytearray(48)
    b[0:6] = bytes([op, session, index >> 8, index & 255, 0, flags])
    b[6:38] = data.ljust(32, b'\0')
    b[38:40] = crc16(b[:38]).to_bytes(2, 'big')
    b[40:43] = b'\x5a\xd2' + bytes([0x39 if op == 0x86 else 0x38])
    b[43:47] = (12345).to_bytes(4, 'big')
    b[47] = crc8(b[40:47])
    return bytes(b)


def descriptor(target):
    base = struct.unpack('>I', target[8:12])[0]
    d = bytearray(64)
    d[:5] = b'QDU6\x02'
    d[6] = d[7] = target[7]
    d[8:15] = target[:7]
    d[15] = 3
    struct.pack_into('>IIIII', d, 16, 0x00030100, 1024, 0x12345678, base, base)
    d[36:39] = bytes([2, 3, 1])
    d[60:64] = crc32(bytes(d[:60])).to_bytes(4, 'big')
    return bytes(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bl', type=pathlib.Path)
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    a = ap.parse_args()
    src = a.src.resolve()
    sys.path.insert(0, str(src / 'tools'))
    from rv32emu import Emu
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'select_bench.c').write_text(BENCH)
    elf = a.out / 'select_bench.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-nostdlib',
                    '-ffunction-sections', '-Wl,--gc-sections', '-DMCU_CH32V40X=1', '-DBOARD=3', '-I', str(src / 'src/core'), '-I', str(src / 'src'), '-Wl,-e,b_init', '-T', str(src / 'test/fx_rv.ld'),
                    str(a.out / 'select_bench.c'), str(src / 'src/core/upd_app.c'), '-o', str(elf)],
                   check=True, timeout=120)
    blinfo = a.bl.read_bytes()[0x10:0x30]
    target = blinfo[5:21]
    desc = descriptor(target)

    def run(steps):
        e = Emu(str(elf))
        e.poke(e.addr('bl'), blinfo)
        e.poke(e.addr('page'), b'\x39\xe3' * 128)               # no installed app
        e.poke(e.addr('uid'), UID)
        e.call('b_setup')
        out = []
        for f in steps:
            if f[0] == 'end':                                   # a decoded header: 8 = UPD, 0 = ordinary
                e.call('b_end', [f[1]])
                continue
            if f[0] == 'poll':
                e.call('b_poll', [f[1]])
                continue
            e.poke(e.addr('fr'), f)
            e.call('b_frame')
            out.append(e.ret)
        return out

    sel = lambda s, uid=UID, tgt=target: frame(0x87, s, 0, uid + tgt + bytes(4))    # noqa: E731
    ann = lambda s: [frame(0x81, s, 0, desc[:32]), frame(0x81, s, 1, desc[32:])]     # noqa: E731
    other = bytes(12)
    abort = lambda s: frame(0x84, s, 0, b'')                                        # noqa: E731
    ping = lambda s: frame(0x85, s, 0, b'')                                         # noqa: E731
    cases = [
        ('selected: ANNOUNCE taken', [sel(9)] + ann(9), TAKE),
        ('no SELECT: declared group entry', ann(9), TAKE),
        ('another UID selected', [sel(9, uid=other)] + ann(9), 0),
        ('another target', [sel(9, tgt=bytes(16))] + ann(9), 0),
        ('selection moved to another node', [sel(9), sel(9, uid=other)] + ann(9), 0),
        ('a status poll is the bootloader\'s', [sel(9), frame(0x86, 9, 0, UID + bytes([0, 1, 0]))] + ann(9), TAKE),
        # docs/update.md: a selection belongs only to its session and cannot gate a later blind session
        ('a later session is blind', [sel(9)] + ann(10), TAKE),
        ('another UID, ABORT, later blind session', [sel(9, uid=other), abort(9)] + ann(10), TAKE),
        ('another UID, later session\'s PING preamble, blind', [sel(9, uid=other), ping(10), ping(10)] + ann(10), TAKE),
        ('self, ABORT, later blind session', [sel(9), ann(9)[0], abort(9)] + ann(10), TAKE),
        ('another UID: its session never enters, also after ABORT', [sel(9, uid=other), abort(9)] + ann(9), 0),
        ('another UID: stale DATA/END/ABORT of other sessions keep it', [sel(9, uid=other), frame(0x82, 10, 0, b''),
            abort(10), frame(0x83, 10, 0, b'')] + ann(9), 0),
        ('another UID: an idle confirmation PING keeps it', [sel(9, uid=other), ping(0)] + ann(9), 0),
        ('another UID: ordinary traffic ends it', [sel(9, uid=other), ('end', 0)] + ann(9), TAKE),
        ('another UID: the idle close ends it', [('end', 8), ('end', 8), sel(9, uid=other), ('poll', 3001)] + ann(9),
            TAKE),
        ('another UID: no idle close while transparent', [('end', 8), ('end', 8), sel(9, uid=other), ('poll', 2999)] +
            ann(9), 0),
    ]
    bad = []
    rows = {}
    for name, steps, want in cases:
        got = run(steps)[-1]
        rows[name] = got
        if got != want:
            bad.append((name, got, want))
    print(json.dumps(dict(cases=rows, failures=bad)))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
