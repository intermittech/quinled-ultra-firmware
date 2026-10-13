# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Reference values of the WS2805 gamma (8-bit input -> 16-bit output) for gamma 2.2 and 2.8:
round(65535 * (in / 255) ^ gamma), an input above 0 never 0.

src/core/gamma.c doesn't hold these as tables (to save flash): it computes them with the board's power curve
(ultra_gamma), within 1 count of this formula; the host test (test/test_cal.c, test_gamma_exact) checks all 512.
This script prints the reference, e.g. to compare.
"""


def table(g):
    vals = []
    for i in range(256):
        v = round(65535 * (i / 255) ** g)
        if i > 0 and v == 0:
            v = 1                      # keep "on" distinguishable from off; the dim floor handles tiny levels
        vals.append(v)
    return vals


if __name__ == "__main__":
    for name, g in (("gamma 2.2", 2.2), ("gamma 2.8", 2.8)):
        print(name)
        vals = table(g)
        for k in range(0, 256, 12):
            print("    " + ", ".join("%5d" % v for v in vals[k:k + 12]))
