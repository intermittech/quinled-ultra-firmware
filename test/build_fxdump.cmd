@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Builds test\fx_dump.exe (MSVC) from the firmware tree this script is in. Usage: test\build_fxdump.cmd [outdir]
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\fxdump"
set "PF86=%ProgramFiles(x86)%"
if "%PF86%"=="" set "PF86=C:\Program Files (x86)"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR (
  echo MSVC not found
  exit /b 2
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%" mkdir "%OUT%"
set CORE=%HERE%..\src\core
cl /nologo /O2 /W3 /D_CRT_SECURE_NO_WARNINGS /Fe:"%OUT%\fx_dump.exe" /Fo:"%OUT%\\" "%HERE%fx_dump.c" "%CORE%\fx_effects.c" "%CORE%\fx_engine.c" "%CORE%\fx_math.c" "%CORE%\fx_tables.c" "%CORE%\fx_pack.c"
exit /b %errorlevel%
