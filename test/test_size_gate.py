# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Hard SRAM/flash boundary checks, including code copied from flash into RAM."""
import contextlib
import io
import pathlib
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
import size_gate as gate


class HardMemoryLimits(unittest.TestCase):
    def run_gate(self, name, flash, ram, ramtext=292):
        row = next(r for r in gate.BUDGET if r[0] == name)
        sections = {".text": flash - ramtext, ".ramtext": ramtext,
                    ".bss": ram - ramtext}
        with patch.object(gate, "BUDGET", [row]), patch.object(gate, "sections", return_value=sections):
            with contextlib.redirect_stdout(io.StringIO()):
                return gate.gate_sizes("unused", False)

    def test_entire_flash_slot_is_available_but_never_overflows(self):
        for name in ("app", "app_xw0"):
            self.assertEqual(self.run_gate(name, 57088, 10000), 0)
            self.assertEqual(self.run_gate(name, 57089, 10000), 1)

    def test_receivers_report_target_but_enforce_slot(self):
        for name in ('ur_analog_m','ur_digital_m'):
            target=gate.LTO_CHURN+gate.FINAL_FREE
            self.assertEqual(self.run_gate(name,57088-target,10000),0)
            self.assertEqual(self.run_gate(name,57089-target,10000),0)
            self.assertEqual(self.run_gate(name,57088,10000),0)
            self.assertEqual(self.run_gate(name,57089,10000),1)

    def test_xpack_receivers_measure_overflow_but_keep_sram_floor(self):
        for name in ("ur_analog_m_xw0", "ur_digital_m_xw0"):
            self.assertEqual(self.run_gate(name, 59000, 10000), 0)
            self.assertEqual(self.run_gate(name, 59000, 18000), 1)

    def test_analog_floor_includes_ramtext_stack_and_reserved_holes(self):
        limit = 20480 - 2048 - 112 - 1024
        self.assertEqual(self.run_gate("ur_analog_m_xw0", 56000, limit), 0)
        self.assertEqual(self.run_gate("ur_analog_m_xw0", 56000, limit + 1), 1)

    def test_digital_floor_includes_large_ramtext(self):
        limit = 20480 - 2048 - 112 - 1536
        self.assertEqual(self.run_gate("ur_digital_m_xw0", 55000, limit, 4460), 0)
        self.assertEqual(self.run_gate("ur_digital_m_xw0", 55000, limit + 1, 4460), 1)

    def test_data_load_counts_twice_stack_does_not(self):
        self.assertEqual(gate.memory_usage({".init": 4, ".text": 100, ".ramtext": 20,
                                           ".data": 8, ".bss": 40, ".noinit": 16,
                                           ".stack": 2048}), (132, 84))


if __name__ == "__main__":
    unittest.main()
