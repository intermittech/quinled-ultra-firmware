@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem The V407 effects layer (src/core/fx_v407, FX_V407=1) on the host, MSVC. Usage: test\build_fxv.cmd [outdir] [cases.txt]
rem   test_fxv.exe = the layer's tests (registry / maps, slot contract, pack16 vs fx_pack, palettes, white, chunks);
rem   fxv_gold.exe = the C side of the C <-> JS goldens (dashboards-r13/v3/digital/golden/v407); with a cases file it runs too.
rem   fxv_flash_check.exe = the flash-rate gate of every built effect at its defaults (run it: [--legacy] [--minutes m] [ids])
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\fxv"
set "PF86=%ProgramFiles(x86)%"
if "%PF86%"=="" set "PF86=C:\Program Files (x86)"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR (
  echo MSVC not found
  exit /b 2
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%\obj" mkdir "%OUT%\obj"
if not exist "%OUT%\gold" mkdir "%OUT%\gold"
set CORE=%HERE%..\src\core
set "FXSRC=%CORE%\fx_math.c %CORE%\fx_effects.c %CORE%\fx_engine.c %CORE%\fx_pack.c %CORE%\fx_tables.c %CORE%\math32.c"
set FXV=
for %%f in ("%CORE%\fx_v407\*.c") do call set "FXV=%%FXV%% "%%f""
cl /nologo /O2 /W4 /wd4996 /DFX_V407=1 /DFXV_META_CONST= /I"%CORE%" /Fe:"%OUT%\test_fxv.exe" /Fo:"%OUT%\obj\\" "%HERE%test_fxv.c" %FXSRC% %FXV%
if %errorlevel% neq 0 exit /b 1
"%OUT%\test_fxv.exe"
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /DFX_V407=1 /DFXV_META_CONST= /I"%CORE%" /Fe:"%OUT%\fxv_gold.exe" /Fo:"%OUT%\gold\\" "%HERE%fxv_gold.c" %FXSRC% %FXV%
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%\flash" mkdir "%OUT%\flash"
cl /nologo /O2 /W4 /wd4996 /DFX_V407=1 /I"%CORE%" /Fe:"%OUT%\fxv_flash_check.exe" /Fo:"%OUT%\flash\\" "%HERE%fxv_flash_check.c" %FXSRC% %FXV%
if %errorlevel% neq 0 exit /b 1
if not "%~2"=="" (
  "%OUT%\fxv_gold.exe" "%~2" "%OUT%\gold.txt"
  if %errorlevel% neq 0 exit /b 1
)
exit /b 0
