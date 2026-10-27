# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Relocated ADDI's zero-immediate alias must not cause a false mismatch."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from order_proof import norm_ops


class RelocatedAlias(unittest.TestCase):
    def test_alias(self):
        rel = [('R_RISCV_LO12_I', '.LC5')]
        self.assertEqual(norm_ops('mv', 'a0,a0', rel),
                         norm_ops('addi', 'a0,a0,128', rel))

    def test_differences_remain_visible(self):
        rel = [('R_RISCV_LO12_I', '.LC5')]
        a = norm_ops('mv', 'a0,a0', rel)
        self.assertNotEqual(a, norm_ops('addi', 'a1,a0,128', rel))
        self.assertNotEqual(a, norm_ops('addi', 'a0,a0,128',
                                      [('R_RISCV_LO12_I', '.LC6')]))
        self.assertNotEqual(norm_ops('mv', 'a0,a0', []),
                            norm_ops('addi', 'a0,a0,128', []))


if __name__ == '__main__':
    unittest.main()
