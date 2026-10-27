@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Build and run the host tests with MSVC (Visual Studio Build Tools). Usage: test\build_host.cmd [outdir]
rem (default outdir: build\host). host_tests.exe (everything in src\core), isense_tests.exe (board current sense
rem against a board / fuse model, test_isense.c) and soak_tests.exe (the whole firmware past every counter's wrap,
rem test_soak.c).
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\host"
set "PF86=%ProgramFiles(x86)%"
if "%PF86%"=="" set "PF86=C:\Program Files (x86)"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR (
  echo MSVC not found
  exit /b 2
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
rem Sequential host execution cannot expose compiler reordering across ISR hand-overs.
rem Python 3 checks the documented source contracts as a separate safety guard.
python "%HERE%test_shared_contract.py"
if %errorlevel% neq 0 exit /b 1
rem the return channel's ISR hand-overs: their barriers
python "%HERE%test_r6_rc_contract.py"
if %errorlevel% neq 0 exit /b 1
python "%HERE%test_m1a_contract.py"
if %errorlevel% neq 0 exit /b 1
python "%HERE%test_m1b_contract.py"
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%" mkdir "%OUT%"
rem USART1 dividers on both receivers and negative controls for unsupported UART selectors
call "%HERE%build_answer_uart.cmd" "%OUT%"
if %errorlevel% neq 0 exit /b 1
call "%HERE%build_line_state.cmd" "%OUT%"
if %errorlevel% neq 0 exit /b 1
rem The target's compact libc loops, renamed against the host libc (all alignments and overlaps).
cl /nologo /O2 /W4 /Fe:"%OUT%\mem_tests.exe" /Fo:"%OUT%\\" "%HERE%test_mem.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\mem_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem The acquisition feed (core/ws_decode.c ws_line_feed) against its word-by-word reference: the Digital-M build (it skips the
rem words without an edge) and the plain one, byte for byte (test_line_feed.c).
if not exist "%OUT%\line_feed" mkdir "%OUT%\line_feed"
if not exist "%OUT%\line_feed3" mkdir "%OUT%\line_feed3"
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\line_feed_tests.exe" /Fo:"%OUT%\line_feed\\" "%HERE%test_line_feed.c" "%HERE%..\src\core\ws_decode.c"
if %errorlevel% neq 0 exit /b 1
cl /DBOARD=3 /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\line_feed3_tests.exe" /Fo:"%OUT%\line_feed3\\" "%HERE%test_line_feed.c" "%HERE%..\src\core\ws_decode.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\line_feed_tests.exe"
if %errorlevel% neq 0 exit /b 1
"%OUT%\line_feed3_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem the console's compressed text (core/cz.h): tools/gen_console_text.py writes %OUT%\gen\cz_text.h (--upper: the
rem MON_EXTRA texts; genrc: the receiver's reduced monitoring texts)
python "%HERE%..\tools\gen_console_text.py" "%OUT%\gen" --upper --full
if %errorlevel% neq 0 exit /b 1
python "%HERE%..\tools\gen_console_text.py" "%OUT%\genrc" --full
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%\isense" mkdir "%OUT%\isense"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /DVDIP_PLANNED=1 /DENERGY_HUB=1 /I "%OUT%\gen" /Fe:"%OUT%\host_tests.exe" /Fo:"%OUT%\\" ^
   "%HERE%test_main.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /Fe:"%OUT%\isense_tests.exe" /Fo:"%OUT%\isense\\" ^
   "%HERE%test_isense.c" "%HERE%..\src\core\isense.c" "%HERE%..\src\core\math32.c"
if %errorlevel% neq 0 exit /b 1
rem the core as a PERF=2 bench build (core/perf.h): the counting and the console's `perf` (test_r5_perf.c)
if not exist "%OUT%\perf" mkdir "%OUT%\perf"
python "%HERE%..\tools\gen_console_text.py" "%OUT%\genperf" --perf --upper --full
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /DPERF_BUILD=2 /I "%OUT%\genperf" /Fe:"%OUT%\perf_tests.exe" /Fo:"%OUT%\perf\\" ^
   "%HERE%test_r5_perf.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\perf_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem the same against a 100 mV / A sense path (ISENSE_MV_PER_A 100)
if not exist "%OUT%\isense50" mkdir "%OUT%\isense50"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DISENSE_TEST_MV_PER_A=100u /DISENSE_TEST_BW=50e3 /Fe:"%OUT%\isense_tests_g50.exe" ^
   /Fo:"%OUT%\isense50\\" "%HERE%test_isense.c" "%HERE%..\src\core\isense.c" "%HERE%..\src\core\math32.c"
if %errorlevel% neq 0 exit /b 1
rem the time-warp soak (test/test_soak.c): src\app\main.c itself on a model of the hardware, past every
rem counter's wrap; each scenario in its own process (a fresh boot), ~30 s together
if not exist "%OUT%\soak" mkdir "%OUT%\soak"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /I "%HERE%soak" /I "%HERE%..\src\mcu\ch32v20x" /I "%OUT%\gen" /Fe:"%OUT%\soak_tests.exe" /Fo:"%OUT%\soak\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
rem the same as the Diff-Power-UR-Analog-M (BOARD=2): its return channel (test_soak_rc.c) and every scenario above;
rem built as the receiver builds (tools/build_target.sh): no MON_EXTRA monitoring, the reduced monitoring texts (genrc)
rem (OUTPUTS, ACKS and ERRORS always built in)
if not exist "%OUT%\soak_rc" mkdir "%OUT%\soak_rc"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /DBOARD=2 /DMON_EXTRA=0 /I "%HERE%soak" /I "%HERE%..\src\mcu\ch32v20x" /I "%OUT%\genrc" /Fe:"%OUT%\soak_rc_tests.exe" /Fo:"%OUT%\soak_rc\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\host_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem the CPU load from the WFI stamps, on the soak's model of the sleep (test_r5_cpu_soak.c)
"%OUT%\soak_tests.exe" r5cpu
if errorlevel 1 exit /b 1
rem LIGHT_DEF gamma 0: linear, as ESPHome's gamma_correct: 0
"%OUT%\soak_tests.exe" r6gamma0
if errorlevel 1 exit /b 1
rem the reset breadcrumbs after a watchdog reset (test_soak.c d1)
"%OUT%\soak_tests.exe" d1
if %errorlevel% neq 0 exit /b 1
"%OUT%\isense_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem the event log: a short, a supply dip, a calibration (test_soak.c d2)
"%OUT%\soak_tests.exe" d2
if %errorlevel% neq 0 exit /b 1
rem the power-on scene's take-over against the hub's start-up (50 boards) and refresh order
for %%s in (r6takeover r6takeover0 r6takeover5 r6takeover5d r6rejoin5 r6fxtake) do (
  "%OUT%\soak_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
rem the overrun counters on main.c (test_r5_ovr_soak.c)
"%OUT%\soak_tests.exe" r5ovr
if errorlevel 1 exit /b 1
rem a power-on candle past 2^31 ms of effect time
"%OUT%\soak_tests.exe" r6candle
if errorlevel 1 exit /b 1
"%OUT%\isense_tests_g50.exe"
if %errorlevel% neq 0 exit /b 1
rem the flash state: rejected records at boot, a pending save, a failed write (test_soak.c d5)
"%OUT%\soak_tests.exe" d5
if %errorlevel% neq 0 exit /b 1
rem signal loss hold: the host's SET after a gap that took its SEQ round by 256
"%OUT%\soak_tests.exe" r6seqwrap
if %errorlevel% neq 0 exit /b 1
for %%s in (wrap fx cfg long spec vunit vultra vrx vlong vreset vgate vconsole vclockback r4h7 r4wraps r4flash r4rx r4reset r4gate fwdopen race tripreset longws minmax r5dip r5fx r5console r5jig r5retry r5age r5jig6 r5jig6p) do (
  "%OUT%\soak_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
rem (the receiver: every scenario but r5jig, reset input it doesn't have (r5jig6: its jig input jig), and d5, MON_EXTRA's flash state line)
for %%s in (rcpages rcpower rcfail wrap fx cfg long spec vunit vultra vrx vlong vreset vgate vconsole vclockback r4h7 r4wraps r4flash r4rx r4reset r4gate fwdopen race tripreset longws minmax r5dip r5fx r5console r5cpu r5ovr d1 d2 r5retry r5age r5jig6 r5jig6p) do (
  "%OUT%\soak_rc_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
rem backup, factory reset, restore: the values back, saved by the stores (test_soak.c backup)
"%OUT%\soak_tests.exe" backup
if %errorlevel% neq 0 exit /b 1
rem the reviewers' tests: host programs with their own main (test_r6_host_<t>.c with src\core) and
rem soak scenarios (test_r6_*.c in test_soak.c) on the main board and the receiver
if not exist "%OUT%\r6" mkdir "%OUT%\r6"
for %%t in (pagepair calhist rc) do (
  cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /I "%OUT%\gen" /Fe:"%OUT%\r6_%%t.exe" /Fo:"%OUT%\r6\\" ^
     "%HERE%test_r6_host_%%t.c" "%HERE%..\src\core\*.c"
  if errorlevel 1 exit /b 1
  "%OUT%\r6_%%t.exe"
  if errorlevel 1 exit /b 1
)
rem the page-transaction prototype (test\pagetx: not in the firmware): writes, cuts, the control
if not exist "%OUT%\pagetx" mkdir "%OUT%\pagetx"
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\pagetx_tests.exe" /Fo:"%OUT%\pagetx\\" ^
   "%HERE%pagetx\test_pagetx.c" "%HERE%pagetx\pagetx.c"
if errorlevel 1 exit /b 1
"%OUT%\pagetx_tests.exe"
if errorlevel 1 exit /b 1
rem (code size) core/ultra.c's maths in 32 bits against the 64-bit code it replaced, every 16-bit
rem input exhaustively (test_r9_math.c: its own main, it includes ultra.c itself; `r9_math.exe long`: the 2^32 sweeps)
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /Fe:"%OUT%\r9_math.exe" /Fo:"%OUT%\r6\\" ^
   "%HERE%test_r9_math.c" "%HERE%..\src\core\config.c" "%HERE%..\src\core\presets.c" "%HERE%..\src\core\math32.c" "%HERE%..\src\core\ultra_tables.c"
if errorlevel 1 exit /b 1
"%OUT%\r9_math.exe"
if errorlevel 1 exit /b 1
set "R6=r6vdd r6erase r6fxclear r6erasef r6flashfail r6e2e r6ws2ultra r6boot r6boot5"
for %%s in (%R6% r6gcut r6gadmit r6gretry r6race r6hist) do (
  "%OUT%\soak_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
for %%s in (%R6% r6rcdim r6rc r6takeover r6takeover0 r6takeover5 r6takeover5d r6rejoin5 r6fxtake r6candle r6seqwrap r6gamma0) do (
  "%OUT%\soak_rc_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
rem the factory reset like a reboot: by the pad, with the host lit, from the console; a power cut at any ms
for %%v in (tp9 host console) do (
  "%OUT%\soak_tests.exe" r6fxreset %%v
  if errorlevel 1 exit /b 1
  "%OUT%\soak_rc_tests.exe" r6fxreset %%v
  if errorlevel 1 exit /b 1
)
rem the release console (CONSOLE_FULL 0: it only sends; the builds above: the full console, the bench's): the
rem main board and the receiver with the release's texts; the console's own scenarios (test_r6_console.c: the
rem stream and its golden, typed bytes ignored, the jig pausing it, a dip write mid-line) and every scenario that
rem types no command
python "%HERE%..\tools\gen_console_text.py" "%OUT%\genrel" --upper
if %errorlevel% neq 0 exit /b 1
python "%HERE%..\tools\gen_console_text.py" "%OUT%\genrelrc"
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%\soak_rel" mkdir "%OUT%\soak_rel"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=0 /I "%HERE%soak" /I "%HERE%..\src\mcu\ch32v20x" /I "%OUT%\genrel" /Fe:"%OUT%\soak_rel_tests.exe" /Fo:"%OUT%\soak_rel\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%\soak_rc_rel" mkdir "%OUT%\soak_rc_rel"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=0 /DBOARD=2 /DMON_EXTRA=0 /I "%HERE%soak" /I "%HERE%..\src\mcu\ch32v20x" /I "%OUT%\genrelrc" /Fe:"%OUT%\soak_rc_rel_tests.exe" /Fo:"%OUT%\soak_rc_rel\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
set "REL=conout conin conjig condip fx long spec vunit vultra vrx vlong vreset vgate vclockback r4h7 r4wraps r4rx r4reset r4gate tripreset longws r5fx r5retry r5age r5jig6p r6erase r6erasef r6ws2ultra r6boot r6boot5 r6takeover r6takeover0 r6takeover5 r6takeover5d r6rejoin5 r6fxtake r6candle r6seqwrap r6gamma0"
for %%s in (%REL% r5jig r6gcut r6gadmit r6gretry) do (
  "%OUT%\soak_rel_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
for %%s in (%REL% rcpages rcpower rcfail r6rc) do (
  "%OUT%\soak_rc_rel_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
for %%v in (tp9 host) do (
  "%OUT%\soak_rel_tests.exe" r6fxreset %%v
  if errorlevel 1 exit /b 1
  "%OUT%\soak_rc_rel_tests.exe" r6fxreset %%v
  if errorlevel 1 exit /b 1
)
rem the approved items' scenarios (test_r7.c): the main board, the receiver and
rem the release console's builds
for %%e in (soak_tests soak_rc_tests soak_rel_tests soak_rc_rel_tests) do (
  for %%s in ("r7ack 0" "r7ack 2" "r7cal ok" "r7cal limit" "r7cal supply" "r7cal trip" r7retry r7retryws "r7loss ultra scene" "r7loss ultra none" "r7loss ws scene" "r7loss ws none" r7fault "r7con ultra" "r7con ws" r7pages r7hot r7why r7b3 "r7upd 0" "r7upd 1" r8evtype "r8sb worth" "r8sb nothing" "r8sb darkwait" "r8sb missed" "r8sb brief" "r8sb copies" "r8sb bright" "r8sb slow" "r8sb cal" "r8sb pending" "r8sb race" "r8sb expire" "r8sb cross" "r8sb pre" "r8sb ack" "r8sb cal_edge" "r8sb pvd" "r8sb pgfail" "r8sb exhaust" "r8sb sync" "r8sb syncgate" "r8sb syncneg" "r8sb g_frame" "r8sb g_none" "r8sb g_noslot" "r8sb g_ws" "r8sb g_app" "r8sb app" "r9b fx3" "r9b release" "r9b hold" "r9b dip1s" "r11confirm ws" "r11confirm ultra" "r11confirm wd" "r11confirm none" r11hwid) do (
    "%OUT%\%%e.exe" %%~s
    if errorlevel 1 exit /b 1
  )
)
rem the wall-switch pattern: 12 power cycles, each its own process (test_r6_sys.c r6wall)
call "%HERE%run_r6wall.cmd" "%OUT%\soak_tests.exe" 12
if errorlevel 1 exit /b 1
exit /b 0
