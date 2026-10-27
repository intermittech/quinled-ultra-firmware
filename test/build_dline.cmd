@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem The Diff-Power-UR-Digital-M's delay line + poll window (core/dline.c) on its sample-exact model (test_dline.c), MSVC.
rem Usage: test\build_dline.cmd [outdir] [scenario [trace-scenario [seed]]]   (default outdir: build\host; no scenario:
rem all, ~2 s; `*`: all). Two builds: dline.c as it is and its word-by-word reference (DLINE_WORDS_ONLY: no 4-word block
rem paths); both must pass and print the same (every scenario's state digest included): the block paths change nothing.
rem DLINE_CFLAGS (environment): extra defines, e.g. "/DMCU_CH32V40X=1 /DFS_MHZ=50.0 /DDLINE_TEST_PERIOD_PPM=47700" (V407).
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
if not exist "%OUT%\dline" mkdir "%OUT%\dline"
if not exist "%OUT%\dline_ref" mkdir "%OUT%\dline_ref"
cl /nologo /O2 /W4 /wd4996 %DLINE_CFLAGS% /Fe:"%OUT%\dline_tests.exe" /Fo:"%OUT%\dline\\" ^
   "%HERE%test_dline.c" "%HERE%..\src\core\dline.c" "%HERE%..\src\core\ws_decode.c"
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /wd4127 /DDLINE_WORDS_ONLY %DLINE_CFLAGS% /Fe:"%OUT%\dline_ref_tests.exe" /Fo:"%OUT%\dline_ref\\" ^
   "%HERE%test_dline.c" "%HERE%..\src\core\dline.c" "%HERE%..\src\core\ws_decode.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\dline_tests.exe" %2 %3 %4 > "%OUT%\dline_out.txt"
set RC=%errorlevel%
type "%OUT%\dline_out.txt"
if %RC% neq 0 exit /b 1
"%OUT%\dline_ref_tests.exe" %2 %3 %4 > "%OUT%\dline_ref_out.txt"
if %errorlevel% neq 0 (
  type "%OUT%\dline_ref_out.txt"
  echo dline: the word-by-word reference failed
  exit /b 1
)
fc "%OUT%\dline_out.txt" "%OUT%\dline_ref_out.txt" >nul
if %errorlevel% neq 0 (
  fc "%OUT%\dline_out.txt" "%OUT%\dline_ref_out.txt"
  echo dline: the block paths differ from the word-by-word reference
  exit /b 1
)
echo dline: the block paths and the word-by-word reference agree (every scenario's digest)
