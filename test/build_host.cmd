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
if not exist "%OUT%\isense" mkdir "%OUT%\isense"
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\host_tests.exe" /Fo:"%OUT%\\" ^
   "%HERE%test_main.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\isense_tests.exe" /Fo:"%OUT%\isense\\" ^
   "%HERE%test_isense.c" "%HERE%..\src\core\isense.c"
if %errorlevel% neq 0 exit /b 1
rem the same against a gain-50 amplifier (CSA220NA, board.h ISENSE_AMP_GAIN 50)
if not exist "%OUT%\isense50" mkdir "%OUT%\isense50"
cl /nologo /O2 /W4 /wd4996 /DISENSE_TEST_GAIN=50u /DISENSE_TEST_BW=50e3 /Fe:"%OUT%\isense_tests_g50.exe" ^
   /Fo:"%OUT%\isense50\\" "%HERE%test_isense.c" "%HERE%..\src\core\isense.c"
if %errorlevel% neq 0 exit /b 1
rem the time-warp soak (0.6.2, test/test_soak.c): src\app\main.c itself on a model of the hardware, past every
rem counter's wrap; each scenario in its own process (a fresh boot), ~30 s together
if not exist "%OUT%\soak" mkdir "%OUT%\soak"
cl /nologo /O2 /W4 /wd4996 /I "%HERE%soak" /Fe:"%OUT%\soak_tests.exe" /Fo:"%OUT%\soak\\" ^
   "%HERE%test_soak.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\host_tests.exe"
if %errorlevel% neq 0 exit /b 1
"%OUT%\isense_tests.exe"
if %errorlevel% neq 0 exit /b 1
"%OUT%\isense_tests_g50.exe"
if %errorlevel% neq 0 exit /b 1
for %%s in (wrap fx cfg long spec vunit vultra vrx vlong vreset vgate vconsole vclockback r4h7 r4wraps r4flash r4rx r4reset r4gate race tripreset longws minmax) do (
  "%OUT%\soak_tests.exe" %%s
  if errorlevel 1 exit /b 1
)
