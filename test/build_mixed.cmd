@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Build and run the Diff-Power mixed-chain simulation (test\test_mixed.c) with MSVC. Usage: test\build_mixed.cmd
setlocal
set HERE=%~dp0
set "PF86=%ProgramFiles(x86)%"
if "%PF86%"=="" set "PF86=C:\Program Files (x86)"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR (
  echo MSVC not found
  exit /b 2
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
set "OUT=%HERE%..\build\mixed"
if not exist "%OUT%" mkdir "%OUT%"
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\mixed_tests.exe" /Fo:"%OUT%\\" ^
   "%HERE%test_mixed.c" "%HERE%..\src\core\frame_rx.c" "%HERE%..\src\core\proto.c"
if errorlevel 1 exit /b 1
"%OUT%\mixed_tests.exe" %*
