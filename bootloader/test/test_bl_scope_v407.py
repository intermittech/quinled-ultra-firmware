# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Selected CHAIN status is silent through decoder, main loop and UART backend.

Declared CHAIN reception is qualified by the group tests. The regular wire
matrix covers marked replies at all compliant release times and three clocks.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    a = p.parse_args()
    root = Path(__file__).resolve().parents[2]
    board = json.loads((root/'src/board/ur_digital_m/board.json').read_text())
    flags = board['bindings']['ch32v40x'].get('overrides', {}).get('descriptor', {}).get('link', {}).get(
        'flags', board['descriptor']['link']['flags'])
    # the app at any position of a CHAIN line: RETURN | RELAY | FORWARD | MARKER | UPDATE_FORWARD. RELAY is the app's
    # ordinary return window (the sender's RW plan); a bootloader relays no status: its selected CHAIN status stays
    # silent (the wire run below), status replies go over the marker path
    assert flags == 0x37, 'RETURN | RELAY | FORWARD | MARKER | UPDATE_FORWARD'
    assert not flags & 0x08, 'not a chain end'
    with tempfile.TemporaryDirectory() as td:
        r = subprocess.run([sys.executable, str(Path(__file__).with_name('test_bl_wire_v407.py')),
                            str(a.elf), td, '--hsi', '0', '--release-us', '80', '--unmarked'], timeout=300)
        sys.exit(r.returncode)
