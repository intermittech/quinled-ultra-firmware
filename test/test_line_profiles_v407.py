# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""CH32V40x (50 MHz sampling): the lossless-run acquisition decoder across the local clock range, sample phases and
50 us tick phases, compiled for the family (-DMCU_CH32V40X=1) and run in rv32emu. The clock range is the
pre-lock decision (2026-10-22): the HSI trimmed 12 steps low (-2.4 % nominal) with the factory +-2.2 %, i.e.
-4.55..-0.25 %, plus the locked clock and step-size margin. Model cycles are not a silicon timing certificate.

    python test/test_line_profiles_v407.py <out dir>
"""
import json
import math
import pathlib
import struct
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'tools'))
sys.path.insert(0, str(FW / 'test'))
from rv32emu import Emu                      # noqa: E402
from ur_digital_m_target import target_gcc   # noqa: E402

FS_MHZ = 50.0
PERIODS = [1250, 5000 / 3, 2500, 5000]
EXPECT_K = [30, 42, 63, 125]                  # MCU_LINE_THRESHOLDS at 50 MHz
DISTORTION = [218, 292, 440, 588]             # the V20x test's per-profile HIGH-width distortion, ns
# ppm of nominal: pre-lock slowest, pre-lock nominal, pre-lock fastest, locked, locked + one step margin, and the
# admission window's own edges (MCU_CLOCK_MIN/MAX_PPM)
PPMS = [-45500, -24000, -2500, 0, 2000, -52000, 4000]
TICK_WORDS = 156                               # 50 us of 0.32 us words

BENCH = r'''
#include "../src/core/ws_decode.h"
ws_line_t state;
uint16_t runs[WS_LINE_WORDS], samples[160];
uint8_t decoded[48];
ws_rise_t rise;
void b_reset(void) { ws_line_init(&state); }
void b_feed(unsigned n) { ws_line_feed(&state,runs,samples,n); }
int b_decode(unsigned tail) { return ws_line_decode(&state,runs,tail?64:384,decoded,tail); }
unsigned b_profile(void) { return state.profile; }
unsigned b_threshold(unsigned p) { return ws_line_threshold(p); }
'''


def build(out):
    src = out / 'line_v407_bench.c'
    src.write_text(BENCH)
    elf = out / 'line_v407.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-DBOARD=3',
                    '-DMCU_CH32V40X=1', '-I', str(FW / 'src'), '-fno-builtin', '-fno-common', '-nostdlib',
                    '-Wl,-e,b_reset', '-T', str(FW / 'test/fx_rv.ld'), str(src), str(FW / 'src/core/ws_decode.c'),
                    '-o', str(elf)], check=True, timeout=120)
    return elf


def frame_words(bits, period, f_mhz, phase, distortion_ns):
    quantum = 1000.0 / f_mhz
    first = 16 + phase / 16
    end = math.ceil(first + len(bits) * period / quantum) + 32
    words = [0] * ((end + 15) // 16)
    for i, bit in enumerate(bits):
        start = first + i * period / quantum
        high = (period * (.75 if bit else .25) + (10 if distortion_ns > 0 else -10) + distortion_ns) / quantum
        for j in range(math.ceil(start), math.ceil(start + high)):
            words[j // 16] |= 0x8000 >> (j % 16)
    return words


def run(e, words, phase):
    samples = e.addr('samples')
    e.call('b_reset')
    pos, chunk, peak = 0, 1 + phase * 9, 0
    while pos < len(words):
        n = min(chunk, len(words) - pos)
        e.poke(samples, struct.pack('<' + 'H' * n, *words[pos:pos + n]))
        e.call('b_feed', [n])
        peak = max(peak, e.cycles())
        pos += n
        chunk = TICK_WORDS
    return peak


def main():
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    e = Emu(str(build(out)))
    dest = e.addr('decoded')
    data = bytes([0x55, 0xaa, 0, 255]) * 12
    bits = [(x >> (7 - i)) & 1 for x in data for i in range(8)]
    rows, checks, failures = [], 0, []
    for profile, (period, k, dmax) in enumerate(zip(PERIODS, EXPECT_K, DISTORTION)):
        e.call('b_threshold', [profile])
        assert e.ret == k, (profile, e.ret)
        peak = 0
        for ppm in PPMS:
            f = FS_MHZ * (1 + ppm / 1e6)
            for phase in range(16):
                for distortion in (-dmax, dmax):
                    words = frame_words(bits, period, f, phase, distortion)
                    peak = max(peak, run(e, words, phase))
                    e.call('b_profile')
                    ok = e.ret == profile
                    e.call('b_decode', [0])
                    ok = ok and e.ret == 384 and e.peek(dest, 48) == data
                    e.call('b_decode', [1])
                    ok = ok and e.ret == 64 and e.peek(dest, 8) == data[-8:]
                    checks += 1
                    if not ok:
                        failures.append(dict(profile=profile, ppm=ppm, phase=phase, distortion=distortion))
        fmin, fmax = FS_MHZ * (1 - 0.052), FS_MHZ * (1 + 0.004)
        rows.append(dict(profile=profile, threshold=k,
                         M0_ns=round(1000 * k / fmax - (period / 4 + 10 + dmax) - 1000 / fmin, 3),
                         M1_ns=round(period * .75 - 10 - dmax - 1000 * k / fmin - 1000 / fmin, 3),
                         feed_peak_model_cycles=peak))
    result = dict(mcu='ch32v40x', sample_mhz=FS_MHZ, ppm=PPMS, cases=checks, failures=failures, profiles=rows)
    (out / 'profiles_v407.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(cases=checks, failures=len(failures), profiles=rows), indent=2))
    if failures:
        print('FAIL', failures[:8])
        sys.exit(1)


if __name__ == '__main__':
    main()
