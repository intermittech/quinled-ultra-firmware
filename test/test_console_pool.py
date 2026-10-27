# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Encoded substring sharing must preserve bytes, offsets, and determinism."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('console_text', Path(__file__).resolve().parents[1] / 'tools/gen_console_text.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


class PoolTest(unittest.TestCase):
    def test_names_tokens_and_embedded_nuls(self):
        texts = [b'hello world', b'world', b'red\0green\0blue', b'green\0blue', b'blue', b'', b'world']
        seqs, pairs, _ = gen.compress(texts)
        offsets, blob = gen.pool_sequences(seqs)
        self.assertLess(len(blob), sum(map(len, seqs)))
        for text, seq, off in zip(texts, seqs, offsets):
            encoded = blob[off:off + len(seq)]
            self.assertEqual(encoded, bytes(seq))
            self.assertEqual(b''.join(gen.expand(x, pairs) for x in encoded), text + b'\0')
        self.assertEqual(gen.pool_sequences(seqs), (offsets, blob))

    def test_exact_shared_suffix(self):
        offsets, blob = gen.pool_sequences([b'a\0b\0', b'b\0', b'\0', b'a\0'])
        self.assertEqual(blob, b'a\0b\0')
        self.assertEqual(offsets, [0, 2, 1, 0])


if __name__ == '__main__':
    unittest.main()
