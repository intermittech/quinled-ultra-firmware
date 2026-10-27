# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
import json,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
from decode_console import decode,crc8,SCHEMA
class ConsoleTest(unittest.TestCase):
 def test_goldens_and_corruption(self):
    path=ROOT/'test/fixtures/console';lines=(path/'all-records.txt').read_bytes().splitlines(keepends=True)
    expected=json.loads((path/'expected.json').read_text());self.assertEqual([decode(x) for x in lines],expected)
    self.assertEqual({x['record'] for x in expected},{x['record'] for x in SCHEMA['records']})
    for line in lines:
        for i in range(len(line)-2):
            bad=bytearray(line);bad[i]^=1
            with self.assertRaises((ValueError,KeyError)):decode(bytes(bad))
    with self.assertRaises(ValueError):decode(lines[0][:-1])
    sid=format(SCHEMA['schema'],'02X').encode()+b',';other=format(SCHEMA['schema']+1,'02X').encode()+b','
    with self.assertRaises(ValueError):decode(lines[0].replace(sid,other,1))
if __name__=='__main__':unittest.main()
