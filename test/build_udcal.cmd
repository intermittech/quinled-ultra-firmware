@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\udcal"
set "PF86=%ProgramFiles(x86)%"
if "%PF86%"=="" set "PF86=C:\Program Files (x86)"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
if not defined VSDIR for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR exit /b 2
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%" mkdir "%OUT%"
cl /DBOARD=3 /nologo /O2 /W4 /wd4127 /I"%HERE%..\src\core" /Fe:"%OUT%\udcal.exe" /Fo:"%OUT%\\" "%HERE%test_udcal.c" "%HERE%..\src\core\ur_digital_m\udcal.c" "%HERE%..\src\core\math32.c"
if errorlevel 1 exit /b 1
"%OUT%\udcal.exe"
if errorlevel 1 exit /b 1
rem the V407 Digital-M (its own pixel limits: the stored calibration's cap)
if not exist "%OUT%\v407" mkdir "%OUT%\v407"
cl /DBOARD=3 /DMCU_CH32V40X=1 /DFX_V407=1 /nologo /O2 /W4 /wd4127 /I"%HERE%..\src\core" /Fe:"%OUT%\udcal_v407.exe" /Fo:"%OUT%\v407\\" "%HERE%test_udcal.c" "%HERE%..\src\core\ur_digital_m\udcal.c" "%HERE%..\src\core\math32.c"
if errorlevel 1 exit /b 1
"%OUT%\udcal_v407.exe"
exit /b %errorlevel%
