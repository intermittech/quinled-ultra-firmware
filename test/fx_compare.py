# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compares two fx_dump.bin files (same grid: effects 1 .. N, 35 parameter variants, 2 times, 100 px, RGB bytes): the worst channel
difference per effect, and how many pixels differ by more than 8. For the squeeze rounds: `python test/fx_compare.py old.bin new.bin`."""
import sys

VARIANTS, TIMES, NPX = 35, 2, 100
a = open(sys.argv[1], 'rb').read()
b = open(sys.argv[2], 'rb').read()
n = min(len(a), len(b))
per = VARIANTS * TIMES * NPX * 3
print("effects compared: %d (old %d bytes, new %d bytes)" % (n // per, len(a), len(b)))
worst_all = 0
for e in range(n // per):
    x, y = a[e * per:(e + 1) * per], b[e * per:(e + 1) * per]
    worst = max(abs(p - q) for p, q in zip(x, y))
    big = sum(1 for i in range(0, per, 3) if max(abs(x[i + j] - y[i + j]) for j in range(3)) > 8)
    print("effect %2d: worst channel difference %3d, pixels over 8: %d of %d" % (e + 1, worst, big, per // 3))
    worst_all = max(worst_all, worst)
print("worst of all: %d" % worst_all)
