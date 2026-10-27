# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The firmware's copies of the protocol release files are byte-identical to that release (one master protocol header;
the copies are checked by a test). PROTOCOL_REF is the release commit, the firmware's one protocol pin: every copy is
compared byte for byte with `git show PROTOCOL_REF:<path>` in a protocol checkout that has it (--proto, else
$ULTRA_PROTOCOL_REPO, else ../quinled-ultra-protocol; required), and MD5 with each copy's digest. Every other place
that names the protocol commit (PINS) must name this one.

    python test/test_protocol_copy.py [--proto <protocol checkout>]

Moving to a new release: copy its files, set PROTOCOL_REF, MD5 and the PINS places, run this."""
import argparse
import hashlib
import os
import re
from pathlib import Path
import subprocess
import sys

FW = Path(__file__).resolve().parents[1]
PROTOCOL_REF = 'd5f17d2b0150dc86b1d952b2ddfb4efaa1543f08'     # protocol 0.3.1-draft (wire 3)
COPIES = {                                                     # firmware path: (protocol path, md5)
    'src/core/ultra_protocol.h': ('include/ultra_protocol.h', '0be48b8f7159ac9a7f5aa6f26190e95e'),
    'src/core/ultra_update.h': ('include/ultra_update.h', 'd44b1b2e0526bdfd7c2d1ae77a2dbf06'),
    'test/protocol/shared_vectors.h': ('test-vectors/shared_vectors.h', '0ebb734db4f80483f89310946db6ec71'),
    'test/protocol/vectors.json': ('test-vectors/vectors.json', '9f0a40fa6261c70538ee7a8d92a10d45'),
    'test/protocol/golden_traces.h': ('test-vectors/golden_traces.h', '7e04d5691f85dcf0862766d8607417ba'),
}
PINS = {                                                       # firmware path: the pattern whose group names the commit
    'test/run_m1a_vectors.py': r"^PIN='([0-9a-f]+)'",
    'bootloader/test/prepare_abi3_senders.py': r'^PIN = "([0-9a-f]+)"',
    'bootloader/test/test_bl_status_v407.py': r'^PIN = "([0-9a-f]+)"',
    'bootloader/STATUS-SCOPE.md': r'protocol 0\.3 release sender, ([0-9a-f]{7,40})\.',
}


def default_proto():
    return Path(os.environ.get('ULTRA_PROTOCOL_REPO', FW.parent / 'quinled-ultra-protocol'))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--proto', type=Path, default=default_proto(), help='a protocol repository checkout that has PROTOCOL_REF')
    a = p.parse_args()
    bad = []
    if not (a.proto / '.git').exists():
        print(f'  FAIL the protocol checkout is missing ({a.proto}): --proto or ULTRA_PROTOCOL_REPO')
        return 1
    for path, pattern in PINS.items():
        m = re.search(pattern, (FW / path).read_text(encoding='utf-8'), re.M)
        if not m:
            bad.append(f'{path}: no protocol pin found ({pattern})')
        elif not PROTOCOL_REF.startswith(m.group(1)):
            bad.append(f'{path} pins protocol {m.group(1)}, PROTOCOL_REF is {PROTOCOL_REF[:7]}: move them together')
    for fw, (proto, md5) in COPIES.items():
        data = (FW / fw).read_bytes()
        got = hashlib.md5(data).hexdigest()
        if got != md5:
            bad.append(f'{fw}: md5 {got}, the release has {md5}')
        ref = subprocess.run(['git', '-C', str(a.proto), 'show', f'{PROTOCOL_REF}:{proto}'], capture_output=True,
                             timeout=60)
        if ref.returncode != 0:
            bad.append(f'{proto} at {PROTOCOL_REF[:7]}: {ref.stderr.decode(errors="replace").strip()}')
        elif ref.stdout != data:
            bad.append(f'{fw} differs from {PROTOCOL_REF[:7]}:{proto}')
    for b in bad:
        print('  FAIL ' + b)
    print(f'protocol copies: {"FAIL" if bad else "ok"}, {len(COPIES)} files against release {PROTOCOL_REF[:7]} '
          f'(byte for byte with {a.proto}), {len(PINS)} other pins')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
