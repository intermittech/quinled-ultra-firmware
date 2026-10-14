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
if not exist "%OUT%" mkdir "%OUT%"
rem the console's compressed text (core/cz.h): tools/gen_console_text.py writes %OUT%\gen\cz_text.h (--upper: the
rem default record layout's texts; genlow: 0.6.4's layout, UPPER_RECORDS=0)
python "%HERE%..\tools\gen_console_text.py" "%OUT%\gen" --upper
if %errorlevel% neq 0 exit /b 1
python "%HERE%..\tools\gen_console_text.py" "%OUT%\genlow"
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%\isense" mkdir "%OUT%\isense"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /I "%OUT%\gen" /Fe:"%OUT%\host_tests.exe" /Fo:"%OUT%\\" ^
   "%HERE%test_main.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /Fe:"%OUT%\isense_tests.exe" /Fo:"%OUT%\isense\\" ^
   "%HERE%test_isense.c" "%HERE%..\src\core\isense.c"
if %errorlevel% neq 0 exit /b 1
rem 0.6.5 D7: the core as a PERF=2 bench build (core/perf.h): the counting and the console's `perf` (test_r5_perf.c)
if not exist "%OUT%\perf" mkdir "%OUT%\perf"
python "%HERE%..\tools\gen_console_text.py" "%OUT%\genperf" --perf --upper
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DPERF_BUILD=2 /I "%OUT%\genperf" /Fe:"%OUT%\perf_tests.exe" /Fo:"%OUT%\perf\\" ^
   "%HERE%test_r5_perf.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\perf_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem the same against a gain-50 amplifier (CSA220NA, board.h ISENSE_AMP_GAIN 50)
if not exist "%OUT%\isense50" mkdir "%OUT%\isense50"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DISENSE_TEST_GAIN=50u /DISENSE_TEST_BW=50e3 /Fe:"%OUT%\isense_tests_g50.exe" ^
   /Fo:"%OUT%\isense50\\" "%HERE%test_isense.c" "%HERE%..\src\core\isense.c"
if %errorlevel% neq 0 exit /b 1
rem the time-warp soak (0.6.2, test/test_soak.c): src\app\main.c itself on a model of the hardware, past every
rem counter's wrap; each scenario in its own process (a fresh boot), ~30 s together
if not exist "%OUT%\soak" mkdir "%OUT%\soak"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /I "%HERE%soak" /I "%OUT%\gen" /Fe:"%OUT%\soak_tests.exe" /Fo:"%OUT%\soak\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
rem the same as the Diff-Power ultra receiver (BOARD=2): its return channel (test_soak_rc.c) and every scenario above;
rem built as the receiver builds (tools/build_target.sh): no MON_EXTRA monitoring, 0.6.4 layout's texts (genlow), the
rem three 0.2.4-only pages UNSUPPORTED (RC_PAGES_024 0); soak_rc024: the same with those pages (rcpages, for the review)
if not exist "%OUT%\soak_rc" mkdir "%OUT%\soak_rc"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DBOARD=2 /DMON_EXTRA=0 /DRC_PAGES_024=0 /I "%HERE%soak" /I "%OUT%\genlow" /Fe:"%OUT%\soak_rc_tests.exe" /Fo:"%OUT%\soak_rc\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%\soak_rc024" mkdir "%OUT%\soak_rc024"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DBOARD=2 /DMON_EXTRA=0 /DRC_PAGES_024=1 /I "%HERE%soak" /I "%OUT%\genlow" /Fe:"%OUT%\soak_rc024_tests.exe" /Fo:"%OUT%\soak_rc024\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\soak_rc024_tests.exe" rcpages
if %errorlevel% neq 0 exit /b 1
"%OUT%\host_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem 0.6.5 D3: the CPU load from the WFI stamps, on the soak's model of the sleep (test_r5_cpu_soak.c)
"%OUT%\soak_tests.exe" r5cpu
if errorlevel 1 exit /b 1
rem (0.6.5, D1) the reset breadcrumbs after a watchdog reset (test_soak.c d1)
"%OUT%\soak_tests.exe" d1
if %errorlevel% neq 0 exit /b 1
"%OUT%\isense_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem (0.6.5, D2) the event log: a short, a supply dip, a calibration (test_soak.c d2)
"%OUT%\soak_tests.exe" d2
if %errorlevel% neq 0 exit /b 1
rem 0.6.5 D4: the overrun counters on main.c (test_r5_ovr_soak.c)
"%OUT%\soak_tests.exe" r5ovr
if errorlevel 1 exit /b 1
"%OUT%\isense_tests_g50.exe"
if %errorlevel% neq 0 exit /b 1
rem (0.6.5, D5) the flash state: rejected records at boot, a pending save, a failed write (test_soak.c d5)
"%OUT%\soak_tests.exe" d5
if %errorlevel% neq 0 exit /b 1
rem (0.6.5) 0.6.4's flash layout (UPPER_RECORDS=0, the bench fallback): the soak's flash scenarios on its record map
if not exist "%OUT%\soak_lower" mkdir "%OUT%\soak_lower"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DUPPER_RECORDS=0 /I "%HERE%soak" /I "%OUT%\genlow" /Fe:"%OUT%\soak_tests_lower.exe" /Fo:"%OUT%\soak_lower\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
for %%s in (wrap fx cfg r4flash r5console r5retry r5age) do (
  "%OUT%\soak_tests_lower.exe" %%s
  if errorlevel 1 exit /b 1
)
for %%s in (wrap fx cfg long spec vunit vultra vrx vlong vreset vgate vconsole vclockback r4h7 r4wraps r4flash r4rx r4reset r4gate race tripreset longws minmax r5dip r5fx r5console r5jig r5retry r5age) do (
  "%OUT%\soak_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
rem (the receiver: every scenario but r5jig, the test pad it doesn't have, and d5, MON_EXTRA's flash state line)
for %%s in (rcpages rcpower rcfail wrap fx cfg long spec vunit vultra vrx vlong vreset vgate vconsole vclockback r4h7 r4wraps r4flash r4rx r4reset r4gate race tripreset longws minmax r5dip r5fx r5console r5cpu r5ovr d1 d2 r5retry r5age) do (
  "%OUT%\soak_rc_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
