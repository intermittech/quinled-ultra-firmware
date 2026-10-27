# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""M1a source contracts; complements compiled preemption and flash-cut tests."""
from pathlib import Path
import re
FW=Path(__file__).resolve().parents[1]
def read(p):return (FW/p).read_text()
flash=read('src/app/flash_policy.c')
assert '#define FLASH_LOW_LEVEL 1u' in flash and '#define FLASH_LOW_MS 500u' in flash
assert 'for (i=0;i<BOARD_CHANNELS;++i) if (out_now[i]) return 0;' in flash
assert 'FM_BRIGHT' not in flash and 'flash_bright' not in read('src/app/stores.c')
ur_digital_m=read('src/core/ur_digital_m/udfx.c')
gate=ur_digital_m[ur_digital_m.index('uint32_t udfx_flash_ok'):ur_digital_m.index('void udfx_service')]
for required in ['UX_BARRIER()', 'u->local && u->dark', '!u->want_stream', '!u->stream_pending', '!u->e.ready', 'u->e.state == WS_LOW', 'u->e.reset_w + WS_RING_WORDS']:
 assert required in gate,required
assert re.search(r'UX_BARRIER\(\);[^\n]*\n\s*u->want_stream =',ur_digital_m)
assert re.search(r'if \(u->paused\) \{\s*UX_BARRIER\(\);',ur_digital_m)
assert re.search(r'UX_BARRIER\(\);[^\n]*\n\s*u->pause_req = 0;',ur_digital_m)
assert re.search(r'UX_BARRIER\(\);[^\n]*\n\s*u->paused = \(uint8_t\)',ur_digital_m)
assert 'MCU_US2W(100000u)' in read('src/core/ur_digital_m/udfx.h')
app=read('src/core/upd_app.c')
assert 'if (session != 0u || u->have_half || u->decided) return UPD_APP_NONE;' in app
assert '0.3 development build, wire 3; NOT DEPLOYABLE' in read('src/app/main.c')
assert '0.3 W3 NOT DEPLOYABLE' in read('src/app/main_ur_digital_m.c')
ver=[re.search(r'#define FW_VERSION +"([^"]+)"',read(p)).group(1) for p in ('src/app/main.c','src/app/main_ur_digital_m.c')]
assert ver[0]==ver[1],ver                          # one version for every board of the tree
num=tuple(int(x) for x in re.match(r'(\d+)\.(\d+)\.(\d+)$',ver[0]).groups())
assert (0,6,12)<num<(1,0,0),ver[0]                 # above the published 0.6.12 (an app refuses an older image), below 1.0
print('M1a dark admission, ownership barriers, physical-unit timeout and update-session contracts passed')
