# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Board ownership, binding completeness and generated-fact consistency."""
import importlib.util
import sys
sys.path.insert(0,str(__import__("pathlib").Path(__file__).resolve().parents[1]/"tools"))
from pathlib import Path
import unittest

FW = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('targets', FW/'tools/gen_targets.py')
targets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(targets)

class BoardDefinitions(unittest.TestCase):
    def test_owned_generated_facts(self):
        reg = targets.definitions()
        self.assertEqual({b['id'] for b in reg['boards'].values()}, {1, 2, 3})
        for path, content in targets.generate(reg).items():
            self.assertEqual((FW/path).read_text(), content, path)
        for name, b in reg['boards'].items():
            self.assertIn(b['default_assembly'], [a['id'] for a in b['assemblies']])
            self.assertEqual(b['link']['profile_mask'], 15)
            self.assertFalse(b['link']['timing_qualified'])
            self.assertGreater(b['sense_full_scale_ma'], b['current']['board_ma'])
            self.assertTrue(b['bindings'], name)

    def test_d2a_has_no_return_or_optional_forwarding(self):
        b = targets.definitions()['boards']['d2a_plus']
        self.assertTrue(b['link']['always_forwards'])
        self.assertFalse(b['return'])
        self.assertEqual([a['channel_ma'] for a in b['assemblies']], [3000, 5000])
        self.assertTrue(all('chain_port' not in a for a in b['assemblies']))

    def test_digital_m_default_has_fitted_update_port(self):
        # Port 2 is always fitted on the V203 r2 and the V407 (decided 2026-10-23): one assembly, variant 0
        # (the immutable target is unchanged). LINK: the board at any position of a CHAIN line, V203 and V407 (decided
        # 2026-10-25): RETURN | RELAY | FORWARD | MARKER | UPDATE_FWD, never CHAIN_END.
        b = targets.definitions()['boards']['ur_digital_m']
        for mcu, binding in b['bindings'].items():
            bound = targets.bound_board(b, binding.get('overrides', {}))
            self.assertEqual(bound['assemblies'], [{'id': 0, 'chain_port': True}], mcu)
            self.assertEqual(bound['default_assembly'], 0, mcu)
            flags = bound['descriptor']['link']['flags']
            self.assertEqual(flags, 0x37, mcu)
            self.assertTrue(flags & 0x20 and flags & 0x04 and not flags & 0x08, mcu)
        text = (FW/'src/board/ur_digital_m/descriptor_gen.h').read_text()
        for part in text.split('#else'):
            self.assertIn('0x11, 0x00, 0x37, 0x07', part)   # the LINK record's flags byte in both generated images

    def test_life_sync_energy_step(self):
        """domain 0's rating x maximum voltage as Wh per 0.1 h in lowest terms; unknown: none; past 32 bits: no build"""
        import board_descriptor as bd
        reg = targets.definitions()
        for name in ('ur_analog_m', 'ur_digital_m'):
            self.assertEqual(bd.input_wh(reg['boards'][name]), (99, 8), name)   # 2,250 mA x 55.00 V
        b = reg['boards']['ur_analog_m']
        def with_domain0(**kw):
            c = __import__('copy').deepcopy(b)
            c['descriptor']['domains'][0].update(kw)
            return c
        self.assertEqual(bd.input_wh(with_domain0(max_10mv=65535, min_10mv=65535)), (0, 1))
        self.assertEqual(bd.input_wh(with_domain0(rating_ma=65535)), (0, 1))
        self.assertEqual(bd.input_wh(with_domain0(rating_ma=65000, max_10mv=4800)), (312, 1))
        with self.assertRaises(ValueError):
            bd.input_wh(with_domain0(rating_ma=2251, max_10mv=5501))   # 12,382,751 / 10^6: x 7,210 past 32 bits

if __name__ == '__main__':
    unittest.main()
