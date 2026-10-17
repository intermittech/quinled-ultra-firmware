@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem The Ultra-Digital receiver CHAIN line mode (the real core/dline.c chain scanner, core/ud/udchain.c, udfx.c, wsenc.c, the effect engine src/core/fx*.c, ws_decode.c) on a word-exact ring model (test_udchain.c), MSVC.
rem Usage: test\build_udchain.cmd [outdir] [scenario prefix]   (default outdir: build\host; no prefix: all, ~1 min)
rem Two builds: dline.c as it is and its word-by-word reference (DLINE_WORDS_ONLY: the chain scanner takes no word pairs, the stream scanner no 4-word
rem blocks); both must pass and print the same (the digest of everything captured included): the fast paths change nothing.
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
if not exist "%OUT%\udchain" mkdir "%OUT%\udchain"
if not exist "%OUT%\udchain_ref" mkdir "%OUT%\udchain_ref"
set "FXSRC=%HERE%..\src\core\fx_math.c %HERE%..\src\core\fx_effects.c %HERE%..\src\core\fx_engine.c %HERE%..\src\core\fx_pack.c %HERE%..\src\core\fx_tables.c"
set "FXINC=/I%HERE%..\src\core"
set "SRC="%HERE%test_udchain.c" "%HERE%..\src\core\dline.c" "%HERE%..\src\core\ud\udfx.c" "%HERE%..\src\core\ud\wsenc.c" "%HERE%..\src\core\ud\udchain.c" "%HERE%..\src\core\ws_decode.c" %FXSRC%"
cl /nologo /O2 /W4 /wd4996 %FXINC% /Fe:"%OUT%\udchain_tests.exe" /Fo:"%OUT%\udchain\\" %SRC%
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /wd4127 %FXINC% /DDLINE_WORDS_ONLY /Fe:"%OUT%\udchain_ref_tests.exe" /Fo:"%OUT%\udchain_ref\\" %SRC%
if %errorlevel% neq 0 exit /b 1
"%OUT%\udchain_tests.exe" %2 %3 %4 > "%OUT%\udchain_out.txt"
set RC=%errorlevel%
type "%OUT%\udchain_out.txt"
if %RC% neq 0 exit /b 1
"%OUT%\udchain_ref_tests.exe" %2 %3 %4 > "%OUT%\udchain_ref_out.txt"
if %errorlevel% neq 0 (
  type "%OUT%\udchain_ref_out.txt"
  echo udchain: the word-by-word reference failed
  exit /b 1
)
fc "%OUT%\udchain_out.txt" "%OUT%\udchain_ref_out.txt" >nul
if %errorlevel% neq 0 (
  fc "%OUT%\udchain_out.txt" "%OUT%\udchain_ref_out.txt"
  echo udchain: the fast paths differ from the word-by-word reference
  exit /b 1
)
echo udchain: the fast paths and the word-by-word reference agree (every scenario's results and the digest of every capture)
