@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem The V407 Digital-M's local effects (udfx.c FX_V407, argbenc.c on a serialiser model, the effects layer), MSVC.
rem Usage: test\build_udfx_v407.cmd [outdir]
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\udfx_v407"
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
set CORE=%HERE%..\src\core
set "FXSRC=%CORE%\fx_math.c %CORE%\fx_effects.c %CORE%\fx_engine.c %CORE%\fx_pack.c %CORE%\fx_tables.c %CORE%\math32.c"
set FXV=
for %%f in ("%CORE%\fx_v407\*.c") do call set "FXV=%%FXV%% "%%f""
cl /nologo /O2 /W4 /wd4996 /wd4100 /wd4127 /DBOARD=3 /DMCU_CH32V40X=1 /DFX_V407=1 /DWS_ARGB_HW=\"argb_model_v407.h\" /I"%HERE%." /I"%CORE%" /Fe:"%OUT%\test_udfx_v407.exe" /Fo:"%OUT%\obj\\" "%HERE%test_udfx_v407.c" "%CORE%\ur_digital_m\udfx.c" "%CORE%\ur_digital_m\argbenc.c" %FXSRC% %FXV%
if %errorlevel% neq 0 exit /b 1
"%OUT%\test_udfx_v407.exe"
exit /b %errorlevel%
