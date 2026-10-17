@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem The Ultra-Digital receivers local pixel effects (core/ud/udfx.c, wsenc.c, the real core/dline.c, the effect engine src/core/fx*.c) on a word-exact ring model (test_udfx.c), MSVC.
rem Usage: test\build_udfx.cmd [outdir] [scenario prefix]   (default outdir: build\host; no prefix: all)
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
if not exist "%OUT%\udfx" mkdir "%OUT%\udfx"
set "FXSRC=%HERE%..\src\core\fx_math.c %HERE%..\src\core\fx_effects.c %HERE%..\src\core\fx_engine.c %HERE%..\src\core\fx_pack.c %HERE%..\src\core\fx_tables.c"
set "FXINC=/I%HERE%..\src\core"
cl /nologo /O2 /W4 /wd4996 %FXINC% /Fe:"%OUT%\udfx_tests.exe" /Fo:"%OUT%\udfx\\" ^
   "%HERE%test_udfx.c" "%HERE%..\src\core\dline.c" "%HERE%..\src\core\ud\udfx.c" "%HERE%..\src\core\ud\wsenc.c" %FXSRC%
if %errorlevel% neq 0 exit /b 1
"%OUT%\udfx_tests.exe" %2 %3 %4 > "%OUT%\udfx_out.txt"
set RC=%errorlevel%
type "%OUT%\udfx_out.txt"
exit /b %RC%
