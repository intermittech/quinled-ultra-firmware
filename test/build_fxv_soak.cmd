@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem The V407 effects layer's soak on the host, MSVC: builds fxv_soak.exe (test\fxv_soak.c). Usage: test\build_fxv_soak.cmd [outdir]
rem then: fxv_soak.exe [--hours h] [--fps f] [--n n] [--fmt 3|4|5] [--seed s] [--mode 0..3] [id ...]
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\fxv_soak"
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
cl /nologo /O2 /W4 /wd4996 /DFX_V407=1 /I"%CORE%" /Fe:"%OUT%\fxv_soak.exe" /Fo:"%OUT%\obj\\" "%HERE%fxv_soak.c" %FXSRC% %FXV%
exit /b %errorlevel%
