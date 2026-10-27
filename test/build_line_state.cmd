@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Production acquisition and LINK_STATS boundary tests, MSVC.
setlocal
set "HERE=%~dp0"
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\host"
set "PF86=%ProgramFiles(x86)%"
if "%PF86%"=="" set "PF86=C:\Program Files (x86)"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR exit /b 2
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%\line_state" mkdir "%OUT%\line_state"
cl /nologo /O2 /W4 /DUPD_CRC_ULTRA /DBOARD=2 /Fe:"%OUT%\line_state.exe" /Fo:"%OUT%\line_state\\" ^
 "%HERE%test_line_state.c" "%HERE%..\src\core\ws_decode.c" "%HERE%..\src\core\link_stats.c" "%HERE%..\src\core\math32.c"
if errorlevel 1 exit /b 1
"%OUT%\line_state.exe"
exit /b %errorlevel%
