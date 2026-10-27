# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The Diff-Power-UR-Digital-M's local-effects chip presets (core/ur_digital_m/udfx.c ux_chips) against the chips' datasheet windows,
with the HSI off by up to +-2.2 % (CH32V203 datasheet table 4-13,
-40..85 C): the generated pulses scale with it (the encoder counts SPI1 samples of 27.8 ns at 36 MHz, which the HSI sets).

    python tools/ur_digital_m_timing_check.py

Reads the preset table out of the source, so it checks what the firmware has. Exit 1 when a preset leaves a window."""
import re
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
TS = 1e3 / 36.0                                          # ns per sample (SPI1 at 36 MHz)
WORD_NS = 16 * TS                                        # a ring word
HSI = 0.022

# window per chip family: (T0H, T1H, T0L, T1L, period) as (min, max) ns, None = no limit; the least reset, us
V5 = "WorldSemi V5 class (WS2812B-V5/2020, WS2811 V2.2, WS2814, WS2805, WS2915)"
WIN = {
    V5: (((220, 380), (580, 1000), (580, 1000), (580, 1000), (1250, None)), 280),
    "WS2812B classic": (((250, 550), (650, 950), (700, 1000), (300, 600), (650, 1850)), 50),
    "SK6812 classic Rev 02": (((150, 450), (450, 750), (750, 1050), (450, 750), (650, 1850)), 80),
    "SK6812 Rev B/1": (((250, 400), (650, 1000), (800, None), (200, None), (1200, None)), 200),
}
# preset id (protocol ULTRA_PXC_*) -> name and the windows it must be inside
CHIPS = [
    ("WS2812B V5", [V5]),
    ("WS2812B classic", ["WS2812B classic"]),
    ("SK6812 RGB", ["SK6812 classic Rev 02", "SK6812 Rev B/1"]),
    ("SK6812 RGBW", ["SK6812 classic Rev 02", "SK6812 Rev B/1"]),
    ("WS2814 RGBW", [V5]),
    ("WS2805 RGBCCT", [V5]),
    ("WS2815 (V5 class assumed: no datasheet in the repository)", [V5]),
]


def presets():
    import subprocess
    sys.path.insert(0, str(FW / 'test'))
    from ur_digital_m_target import target_gcc
    src = '#include "src/core/ur_digital_m/udfx.c"\n'
    cmd = [target_gcc(), '-E', '-P', '-DCH32V20x_D6', '-I'+str(FW), '-I'+str(FW/'src/core'), '-x', 'c', '-']
    text = subprocess.check_output(cmd, input=src, text=True)
    body = text[text.index('ux_chips['):]
    body = body[:body.index('};')]
    # All expressions are expanded integer arithmetic supplied by this repository.
    rows = re.findall(r'\{ ([^{}]+), \{', body)
    result = []
    for row in rows:
        values = []
        for expr in row.split(',')[:6]:
            expr = re.sub(r'\((?:unsigned int|uint32_t)\)', '', expr)
            expr = re.sub(r'(?<=\d)[uUlL]+', '', expr).replace('/', '//')
            if not re.fullmatch(r'[0-9 ()+*/-]+', expr): raise ValueError(expr)
            values.append(eval(expr, {"__builtins__": {}}, {}))
        result.append(tuple(values))
    return result


def inside(v, w, scale):
    lo, hi = w
    a, b = v * (1 - scale), v * (1 + scale)
    return (lo is None or a >= lo) and (hi is None or b <= hi), a, b


def main():
    rows = presets()
    if len(rows) != len(CHIPS):
        print("the table has %d presets, this check %d" % (len(rows), len(CHIPS)))
        return 1
    bad = 0
    for (s, t0h, t1h, bpp, reset_w, latch_w), (name, fams) in zip(rows, CHIPS):
        vals = [t0h * TS, t1h * TS, (s - t0h) * TS, (s - t1h) * TS, s * TS]
        reset_us = reset_w * WORD_NS / 1000.0
        latch_us = latch_w * WORD_NS / 1000.0
        print("%-58s %2d / %2d / %2d samples: T0H %.0f  T1H %.0f  T0L %.0f  T1L %.0f  bit %.0f ns, reset %.0f us" %
              (name, s, t0h, t1h, *vals, reset_us))
        for f in fams:
            w, rmin = WIN[f]
            res = [inside(v, wi, HSI) for v, wi in zip(vals, w)]
            ok = all(r[0] for r in res)
            rok = reset_us * (1 - HSI) >= rmin
            lok = latch_us * (1 - HSI) >= min(WIN[g][1] for g in fams) and latch_w <= reset_w   # (the mode switches wait for a LOW this long: the weakest family's latch)
            worst = ", ".join("%s %.0f-%.0f" % (n, r[1], r[2]) for n, r in zip(("T0H", "T1H", "T0L", "T1L", "bit"), res)
                              if not r[0])
            print("    %-62s %s%s%s" % (f, "inside at +-2.2 %" if ok else "OUT: " + worst,
                                        "" if rok else "   RESET %.0f us at the fast HSI < %d" % (reset_us * (1 - HSI), rmin),
                                        "" if lok else "   LATCH %.0f us at the fast HSI < %d or above the reset" %
                                        (latch_us * (1 - HSI), min(WIN[g][1] for g in fams))))
            bad += (not ok) + (not rok) + (not lok)
    print("timing check: %s" % ("every preset inside its windows at +-2.2 %" if not bad else "FAILED (%d)" % bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
