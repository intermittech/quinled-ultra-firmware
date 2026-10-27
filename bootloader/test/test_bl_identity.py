# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Verify immutable bytes in real boot binaries against firmware-owned targets."""
import json, pathlib, sys
root=pathlib.Path(__file__).resolve().parents[2]
folder=pathlib.Path(sys.argv[1]); count=0
build_id=next(line.split()[1] for line in (root/'bootloader/BL_BUILD_ID').read_text().splitlines() if line.startswith('id '))
assert len(build_id)==7
for board in (1,2,3):
    facts=next(json.loads(p.read_text()) for p in (root/'src/board').glob('*/board.json') if json.loads(p.read_text())['id']==board)
    for compiler in ('xpack','wch'):
        blob=(folder/f'bl{board}-{compiler}/bl.bin').read_bytes(); info=blob[16:48]
        assert len(blob)<=8192
        assert info[:5]==b'QDB2\x03'
        assert info[5:13]==bytes([board,1,1,1,3,2,facts['default_assembly'],facts['revision']])
        assert info[13:21]==bytes.fromhex('080021000000df00')
        assert info[21:25]==bytes.fromhex('880a1405')
        assert info[25:32]==build_id.encode('ascii')
        count+=1
print(f'{count} compiled boot identities and slots PASS')
