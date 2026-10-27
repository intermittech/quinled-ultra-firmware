@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
set HERE=%~dp0
set "OUT=%HERE%..\..\build\r14-host"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR exit /b 2
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%" mkdir "%OUT%"
python "%HERE%..\..\tools\gen_console_text.py" "%OUT%\gen" --upper --full
if errorlevel 1 exit /b 2
cl /nologo /O2 /W4 /wd4996 /wd4505 /Zi /fsanitize=address /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /DENERGY_HUB=1 /I "%OUT%\gen" /Fe:"%OUT%\boundaries.exe" /Fo:"%OUT%\\" /Fd:"%OUT%\boundaries.pdb" "%HERE%test_boundaries.c" "%HERE%..\..\src\core\*.c" /link /INCREMENTAL:NO
if errorlevel 1 exit /b 2
"%OUT%\boundaries.exe" %*
exit /b %errorlevel%
