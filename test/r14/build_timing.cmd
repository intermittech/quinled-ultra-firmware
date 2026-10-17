@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
set HERE=%~dp0
set "OUT=%HERE%..\..\build\r14-timing"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR exit /b 2
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%" mkdir "%OUT%"
if "%1"=="rc" goto rc
cl /nologo /O2 /W4 /wd4996 /wd4505 /Zi /fsanitize=address /Fe:"%OUT%\dline.exe" /Fo:"%OUT%\\" /Fd:"%OUT%\dline.pdb" "%HERE%test_dline_boundaries.c" "%HERE%..\..\src\core\dline.c" "%HERE%..\..\src\core\ws_decode.c" /link /INCREMENTAL:NO
if errorlevel 1 exit /b 2
"%OUT%\dline.exe"
set DLRC=%errorlevel%
:rc
if not defined DLRC set DLRC=0
cl /nologo /O2 /W4 /wd4996 /wd4505 /Zi /fsanitize=address /Fe:"%OUT%\rc.exe" /Fo:"%OUT%\\" /Fd:"%OUT%\rc.pdb" "%HERE%test_rc_boundaries.c" "%HERE%..\..\src\core\rc.c" "%HERE%..\..\src\core\config.c" "%HERE%..\..\src\core\pwrev.c" "%HERE%..\..\src\core\presets.c" /link /INCREMENTAL:NO
if errorlevel 1 exit /b 2
"%OUT%\rc.exe"
if errorlevel 1 exit /b 1
exit /b %DLRC%
