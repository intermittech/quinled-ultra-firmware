@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\..\tmp\console-host"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR exit /b 2
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%" mkdir "%OUT%"
python "%HERE%..\tools\gen_console_schema.py" --check
if errorlevel 1 exit /b 1
python "%HERE%..\tools\gen_console_fixture.py" --check
if errorlevel 1 exit /b 1
cl /nologo /O2 /W4 /DUPD_CRC_ULTRA /DCONSOLE_FULL=0 /I"%HERE%..\src\core" /Fe:"%OUT%\capture.exe" /Fo:"%OUT%\\" "%HERE%console_fixture.c" "%HERE%..\src\core\console_record.c" "%HERE%..\src\core\config.c" "%HERE%..\src\core\presets.c"
if errorlevel 1 exit /b 1
"%OUT%\capture.exe" > "%OUT%\capture.txt"
fc /b "%OUT%\capture.txt" "%HERE%fixtures\console\all-records.txt" >nul
if errorlevel 1 exit /b 1
python "%HERE%test_console_schema.py"
exit /b %errorlevel%
