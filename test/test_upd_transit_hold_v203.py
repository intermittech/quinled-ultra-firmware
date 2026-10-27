# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V203 apps' transit hold (BOARD 1 / 2 / 3: core/upd_app.c without the V407 status profile), the cases of r96's
test_upd_transit_hold_v407.py: a foreign ANNOUNCE keeps update forwarding open for that ANNOUNCE's declared erase lead.

The hold counts the erase units from the descriptor as the sender does (layout 1: 256 B units, layout 4: 4096 B, plus
the metadata unit) x 2 x 255 ms (the descriptor carries no erase bound; the u8 maximum is never shorter) + 100 ms,
from the final ANNOUNCE half; the two-second idle close counts from its end, and DATA / END / ABORT of that session
restore the two-second rule (decided 2026-10-23). A first half failing a forwarder check (magic, format, flags,
revision order, board 0 / 255, ABI, feature bits, layout, load, body length zero / not whole words / beyond the slot)
and a second half without its first get the largest known hold, 114340 ms. core/upd_app.c compiled for RV32 as the
V203 apps build it. The holds also equal the sender's reference upd_transit_hold_ms() (include/
ultra_update.h at the protocol pin, test_protocol_copy.py PROTOCOL_REF, compiled the same way) for the smallest,
a mid and the full layout-4 image and invalid ones.

    python test/test_upd_transit_hold_v203.py <out dir> [--src <firmware tree>] [--proto <protocol checkout>]
"""
import argparse
import json
import pathlib
import struct
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'test'))
from ur_digital_m_target import target_gcc                       # noqa: E402
from test_upd_select_v407 import frame, crc32                    # noqa: E402
from test_protocol_copy import PROTOCOL_REF, default_proto       # noqa: E402

BENCH = r'''
#include "upd_app.h"
upd_app_t u;
upd_app_info_t info;
uint8_t bl[32], page[256], fr[48];
void *memcpy(void *d, const void *s, unsigned n) { unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }
int memcmp(const void *x, const void *y, unsigned n)
{ const unsigned char *a = x, *b = y; for (; n--; a++, b++) if (*a != *b) return *a - *b; return 0; }
void b_init(void) { }
void b_setup(void) { upd_app_read_info(&info, page, bl); upd_app_init(&u, &info); }
unsigned b_frame(unsigned now) { upd_app_frame_end(&u, 1u, 8u, now); return upd_app_frame(&u, fr, now); }
unsigned b_poll(unsigned now) { upd_app_poll(&u, now); return u.transparent; }
'''
V203_BODY, V203_END, V407_BODY, V407_END = 0x08002100, 0x08010000, 0x08005000, 0x08080000
FALLBACK = 224 * 510 + 100
IDLE = 2000


REF_BENCH = r'''
#include "ultra_update.h"
uint8_t h[32];
void b_init(void) { }
unsigned b_hold(void) { return upd_transit_hold_ms(h); }
'''


def desc(layout, length, load, **edit):
    """a foreign descriptor (another board); `edit`: byte offset -> value (e.g. o5=1 sets the flags byte)"""
    d = bytearray(64)
    d[:5] = b'QDU6\x02'
    d[8:15] = bytes([0x7e, 2, 3, layout, 3, 0, 0])
    struct.pack_into('>IIIII', d, 16, 0x00030100, length, 0x12345678, load, load)
    for key, value in edit.items():
        d[int(key[1:])] = value
    d[60:64] = crc32(bytes(d[:60])).to_bytes(4, 'big')
    return bytes(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    ap.add_argument('--proto', type=pathlib.Path, default=default_proto(),
                    help='protocol checkout holding PROTOCOL_REF (the reference sender header)')
    a = ap.parse_args()
    src = a.src.resolve()
    sys.path.insert(0, str(src / 'tools'))
    from rv32emu import Emu
    a.out.mkdir(parents=True, exist_ok=True)
    gcc = [target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-nostdlib',
           '-ffunction-sections', '-Wl,--gc-sections', '-Wl,-e,b_init', '-T', str(src / 'test/fx_rv.ld')]
    (a.out / 'hold_bench.c').write_text(BENCH)
    elf = a.out / 'hold_bench.elf'
    subprocess.run(gcc + ['-DMCU_CH32V20X=1', '-I', str(src / 'src/core'),
                          '-I', str(src / 'src'), str(a.out / 'hold_bench.c'), str(src / 'src/core/upd_app.c'),
                          '-o', str(elf)], check=True, timeout=120)
    inc = a.out / 'ref-include'
    inc.mkdir(exist_ok=True)
    for name in ('ultra_protocol.h', 'ultra_update.h'):
        text = subprocess.run(['git', '-C', str(a.proto), 'show', f'{PROTOCOL_REF}:include/{name}'], capture_output=True,
                              check=True, timeout=60).stdout
        (inc / name).write_bytes(text)
    (a.out / 'ref_bench.c').write_text(REF_BENCH)
    ref_elf = a.out / 'ref_bench.elf'
    subprocess.run(gcc + ['-I', str(inc), str(a.out / 'ref_bench.c'), '-o', str(ref_elf)], check=True, timeout=120)
    ref = Emu(str(ref_elf))
    def reference(d):
        ref.poke(ref.addr('h'), d[:32]); ref.call('b_hold'); return ref.ret
    rows = []

    def bench(t0):
        e = Emu(str(elf))
        e.poke(e.addr('page'), b'\x39\xe3' * 128)       # no bootloader identity: forward only
        e.call('b_setup')
        def send(f, t):
            e.poke(e.addr('fr'), f); e.call('b_frame', [t & 0xffffffff])
        def open_at(t):
            e.call('b_poll', [t & 0xffffffff]); return bool(e.ret)
        for k in (0, 10): send(frame(0x85, 7, 0, b''), t0 + k)  # transparent after two UPD headers
        return send, open_at

    def hold_case(name, d, hold, halves=(0, 1), t0=1000, data_at=None):
        send, open_at = bench(t0)
        last = t0 + 20
        for k, half in enumerate(halves):
            send(frame(0x81, 7, half, d[half * 32:(half + 1) * 32]), t0 + 20 + 10 * k); last = t0 + 20 + 10 * k
        if data_at is None:
            ok = open_at(last + hold + IDLE - 1) and not open_at(last + hold + IDLE)
            detail = dict(closes_after_ms=hold + IDLE)
        else:
            held = open_at(last + data_at - 1)
            send(frame(0x82, 7, 0, b''), last + data_at)
            ok = held and open_at(last + data_at + IDLE - 1) and not open_at(last + data_at + IDLE)
            detail = dict(open_at_data_ms=data_at)
        rows.append(dict(case=name, hold_ms=hold, ok=bool(ok), **detail)); print(json.dumps(rows[-1]), flush=True)

    k62 = desc(4, 63488, V407_BODY)
    hold_case('V407 62 KiB, abandoned', k62, 17 * 510 + 100)
    hold_case('V407 62 KiB, DATA after its 3500 ms lead', k62, 17 * 510 + 100, data_at=3500)
    full = desc(4, V407_END - V407_BODY, V407_BODY)
    hold_case('V407-class full slot, 200 ms bound: DATA at 49700 ms', full, 124 * 510 + 100, data_at=49700)
    hold_case('V407 full slot, abandoned', full, 124 * 510 + 100)
    hold_case('V203 full slot, abandoned', desc(1, V203_END - V203_BODY, V203_BODY), 224 * 510 + 100)
    hold_case('V203 4 KiB, abandoned', desc(1, 4096, V203_BODY), 17 * 510 + 100)
    for name, d in (('unknown layout', desc(2, 4096, V407_BODY)), ('empty body', desc(4, 0, V407_BODY)),
                    ('body above the slot', desc(4, V407_END - V407_BODY + 1, V407_BODY)),
                    ('load outside the slot', desc(4, 4096, V407_BODY + 4096))):
        hold_case(name + ': the largest known hold', d, FALLBACK)
    for name, d in (('body not whole words', desc(4, 4098, V407_BODY)), ('bad magic', desc(4, 4096, V407_BODY, o0=0x58)),
                    ('descriptor format 3', desc(4, 4096, V407_BODY, o4=3)), ('flags 1', desc(4, 4096, V407_BODY, o5=1)),
                    ('revision 255..1', desc(4, 4096, V407_BODY, o6=255, o7=1)), ('board 0', desc(4, 4096, V407_BODY, o8=0)),
                    ('board 255', desc(4, 4096, V407_BODY, o8=255)), ('ABI 2', desc(4, 4096, V407_BODY, o12=2)),
                    ('feature bits 0x80', desc(4, 4096, V407_BODY, o13=0x80))):
        hold_case(name + ': the largest known hold', d, FALLBACK)
    hold_case('second half without its first: the largest known hold', k62, FALLBACK, halves=(1,))
    hold_case('smallest layout-1 image (4 B)', desc(1, 4, V203_BODY), 1120)
    # the sender's reference for every descriptor it checks
    for name, d, want in (('smallest layout-1 (4 B)', desc(1, 4, V203_BODY), 1120),
                          ('smallest layout-4 (4 B)', desc(4, 4, V407_BODY), 1120),
                          ('mid: 62 KiB layout-4', k62, 17 * 510 + 100), ('full layout-4', full, 124 * 510 + 100),
                          ('full layout-1', desc(1, V203_END - V203_BODY, V203_BODY), FALLBACK),
                          ('invalid: bad magic', desc(4, 4096, V407_BODY, o0=0x58), FALLBACK),
                          ('invalid: not whole words', desc(4, 4098, V407_BODY), FALLBACK),
                          ('invalid: load outside the slot', desc(4, 4096, V407_BODY + 4096), FALLBACK)):
        got = reference(d)
        rows.append(dict(case='reference: ' + name, reference_ms=got, expected_ms=want, ok=got == want))
        print(json.dumps(rows[-1]), flush=True)
        hold_case('equals the reference: ' + name, d, got)
    hold_case('across the ms wrap', k62, 17 * 510 + 100, t0=(1 << 32) - 5000)

    send, open_at = bench(1000)                          # a repeated pair re-arms the hold from its final half
    for t in (1020, 1030, 4020, 4030): send(frame(0x81, 7, (t // 10) % 2, k62[((t // 10) % 2) * 32:][:32]), t)
    hold = 17 * 510 + 100
    ok = open_at(4030 + hold + IDLE - 1) and not open_at(4030 + hold + IDLE)
    rows.append(dict(case='a repeated pair counts from its final half', hold_ms=hold, ok=bool(ok)))
    print(json.dumps(rows[-1]), flush=True)

    send, open_at = bench(1000)                          # only that session's ABORT restores the 2 s rule
    for k in (0, 1): send(frame(0x81, 7, k, k62[k * 32:(k + 1) * 32]), 1020 + 10 * k)
    send(frame(0x84, 8, 0, b''), 1040)
    stale = open_at(1040 + IDLE + 100)
    send(frame(0x84, 7, 0, b''), 1050)
    ok = stale and open_at(1050 + IDLE - 1) and not open_at(1050 + IDLE)
    rows.append(dict(case='a stale ABORT keeps the hold, its own ABORT restores the 2 s rule', ok=bool(ok)))
    print(json.dumps(rows[-1]), flush=True)

    (a.out / 'transit-hold.json').write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} transit-hold cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__':
    sys.exit(main())
