# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run the pinned public codecs and complete firmware vector harness with MSVC.

python test/run_m1a_vectors.py --spec <proto checkout> --out <temporary directory>
"""
import argparse,os,subprocess
from pathlib import Path
PIN='d5f17d2b0150dc86b1d952b2ddfb4efaa1543f08'
FW=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
a=p.parse_args();spec=a.spec.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
actual=subprocess.check_output(['git','-c','safe.directory='+spec.as_posix(),'-C',str(spec),'rev-parse','HEAD'],text=True).strip()
assert actual==PIN,(actual,PIN)
for h in ['ultra_protocol.h','ultra_update.h']:
 assert (FW/'src/core'/h).read_bytes()==(spec/'include'/h).read_bytes(),h+' differs from pinned spec'
fixtures=FW/'test/protocol'
for name in ['shared_vectors.h','vectors.json','golden_traces.h']:
 assert (fixtures/name).read_bytes()==(spec/'test-vectors'/name).read_bytes(),name+' differs from pinned spec'
vswhere=Path(os.environ.get('ProgramFiles(x86)',r'C:\Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
vs=Path(subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip())
flags=f'/nologo /O2 /W4 /DUPD_CRC_ULTRA /I"{FW}/src/core" /I"{spec}/include" /I"{fixtures}" /I"{spec}/test-vectors"'
jobs=[('codec-c','',[spec/'test-vectors/test_vectors.c']),('codec-cpp','/TP',[spec/'test-vectors/test_vectors.c']),
      ('firmware','',[FW/q for q in ['test/test_m1a_vectors.c','src/core/ws_decode.c','src/core/upd_app.c','src/core/rc.c','src/core/math32.c','src/core/config.c','src/core/presets.c']])]
# every board's own descriptor through the pinned sender's decoder (the Digital-M's LINK: RETURN | RELAY | FORWARD | MARKER |
# UPDATE_FWD = 55, at any position of a CHAIN line, V203 and V407)
sender=sorted((spec/'sender/src').glob('*.c'))
for board,mcu,want in [('ur_digital_m','/DMCU_CH32V40X=1',55),('ur_digital_m','',55),('ur_analog_m','',None)]:
 jobs.append(('descriptor-'+board+('-v407' if mcu else ''),'/I"'+str(spec/'sender/src')+'" /I. '+mcu+(' /DWANT_LINK_FLAGS='+str(want)+'u' if want else ''),
              [FW/'test/test_board_descriptors.c',*sender],FW/'src/board'/board/'descriptor_gen.h'))
for name,extra,sources,*desc in jobs:
 d=out/name;d.mkdir(exist_ok=True)
 if desc:(d/'desc.h').write_bytes(desc[0].read_bytes())
 cmd='@echo off\ncall "'+str(vs/'VC/Auxiliary/Build/vcvars64.bat')+'" >nul\ncl '+flags+' '+extra+' /Fe:run.exe '+' '.join('"'+str(s)+'"' for s in sources)+'\n'
 (d/'build.cmd').write_text(cmd)
 with (d/'build.log').open('w') as log:
  subprocess.run(['cmd','/c',str(d/'build.cmd')],cwd=d,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
 args=[str(d/'run.exe')]+([str(fixtures/'vectors.json')] if name.startswith('codec') else [])
 result=subprocess.run(args,cwd=d,capture_output=True,text=True,check=True,timeout=120)
 (d/'run.log').write_text(result.stdout);print(name+': '+result.stdout.strip().splitlines()[-1])
