# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Preempt udfx_service at its LOCAL request store in the compiled BOARD=3 API image.

python test/test_ur_digital_m_publish.py build/ur_digital_m-api/fw.elf
The tick must see the frame grid's first word when it sees the request.
"""
import re
import subprocess
import sys
import tempfile
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'tools'))
from rv32emu import Emu
from ur_digital_m_target import target_gcc


def offsets():
    fields = [('udfx_t', 'want_local'), ('udfx_t', 'local'), ('udfx_t', 'first_w'),
              ('udfx_t', 'seg'), ('ux_seg_t', 'run'), ('udfx_t', 'e'),
              ('ws_enc_t', 'next_w'), ('dline_t', 'wpos'), ('dline_t', 'direct'),
              ('dline_t', 'rec'), ('dline_t', 'zs')]
    with tempfile.TemporaryDirectory() as td:
        c, asm = Path(td) / 'offsets.c', Path(td) / 'offsets.s'
        c.write_text('#include <stddef.h>\n#include "ur_digital_m/udfx.h"\nconst unsigned offsets[] = {' +
                     ','.join('offsetof(%s,%s)' % f for f in fields) + '};\n')
        subprocess.run([target_gcc(), '-march=rv32imc', '-mabi=ilp32', '-S',
                        '-I' + str(FW / 'src/core'), str(c), '-o', str(asm)], check=True, timeout=60)
        values = [int(v) for v in re.findall(r'\.word\s+(\d+)', asm.read_text())]
    assert len(values) == len(fields)
    return dict(zip([t + '.' + f for t, f in fields], values))


class Capture(Emu):
    watch = None
    snapshot = None

    def wr(self, a, n, v):
        super().wr(a, n, v)
        if a == self.watch and v == 1 and self.snapshot is None:
            self.snapshot = bytes(self.ram)


def main():
    o = offsets()
    e = Capture(sys.argv[1])
    u, d, ring = e.addr('ux'), e.addr('dl'), e.addr('dl_ring')
    e.call('udfx_init', [u])
    e.call('dline_init', [d, 0, e.addr('dl_cap')])
    e.call('udfx_config', [u, 66, 40])
    e.call('udfx_config', [u, 67, 50])
    e.call('udfx_service', [u, d, 0, 0, 256])  # derive the config before the effect command
    e.wr(d + o['dline_t.wpos'], 4, 11250)     # 5 ms, inside a quiet LOW run
    e.wr(d + o['dline_t.zs'], 4, 0)
    e.wr(d + o['dline_t.direct'], 1, 0)
    e.wr(d + o['dline_t.rec'], 1, 0)
    e.call('udfx_anchor', [u, 5000, 11250])
    e.wr(u + o['udfx_t.seg'] + o['ux_seg_t.run'], 1, 1)
    e.watch = u + o['udfx_t.want_local']
    e.call('udfx_service', [u, d, 5, 0, 256])
    assert e.snapshot is not None, 'no request to enter local'
    # The interrupt is allowed immediately after that store. The rest of the
    # interrupted function cannot retroactively repair the encoder's next_w.
    e.watch = None
    e.ram[:] = e.snapshot
    first = e.rd(u + o['udfx_t.first_w'], 4)
    e.call('udfx_tick', [u, d, ring])
    assert e.rd(u + o['udfx_t.local'], 1), 'tick did not take the request'
    next_w = e.rd(u + o['udfx_t.e'] + o['ws_enc_t.next_w'], 4)
    e.call('udfx_host_us', [u, next_w])
    error = min(e.ret % 20000, 20000 - e.ret % 20000)
    print('LOCAL publication: first_w=%d next_w=%d host_us=%d grid_error_us=%d' %
          (first, next_w, e.ret, error))
    assert first and error <= 1, 'tick consumed LOCAL before first_w was published'
    print('test_ud_publish: passed')


if __name__ == '__main__':
    main()
