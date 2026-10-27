@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\pwrev-digital"
set "PF86=%ProgramFiles(x86)%"
if "%PF86%"=="" set "PF86=C:\Program Files (x86)"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR exit /b 2
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%" mkdir "%OUT%"
cl /nologo /O2 /W4 /Fe:"%OUT%\pwrev.exe" /Fo:"%OUT%\\" "%HERE%test_pwrev_digital.c" "%HERE%..\src\core\rc.c" "%HERE%..\src\core\math32.c" "%HERE%..\src\core\config.c" "%HERE%..\src\core\presets.c"
if errorlevel 1 exit /b 1
"%OUT%\pwrev.exe"
exit /b %errorlevel%
