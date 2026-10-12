@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Build and run the host tests with MSVC (Visual Studio Build Tools). Usage: test\build_host.cmd [outdir]
rem (default outdir: build\host). Two programs: host_tests.exe (everything in src\core) and isense_tests.exe
rem (board current sense against a board / fuse model, test_isense.c).
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\host"
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
if not exist "%OUT%\isense" mkdir "%OUT%\isense"
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\host_tests.exe" /Fo:"%OUT%\\" ^
   "%HERE%test_main.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\isense_tests.exe" /Fo:"%OUT%\isense\\" ^
   "%HERE%test_isense.c" "%HERE%..\src\core\isense.c"
if %errorlevel% neq 0 exit /b 1
rem the same against a gain-50 amplifier (CSA220NA, board.h ISENSE_AMP_GAIN 50)
if not exist "%OUT%\isense50" mkdir "%OUT%\isense50"
cl /nologo /O2 /W4 /wd4996 /DISENSE_TEST_GAIN=50u /DISENSE_TEST_BW=50e3 /Fe:"%OUT%\isense_tests_g50.exe" ^
   /Fo:"%OUT%\isense50\\" "%HERE%test_isense.c" "%HERE%..\src\core\isense.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\host_tests.exe"
if %errorlevel% neq 0 exit /b 1
"%OUT%\isense_tests.exe"
if %errorlevel% neq 0 exit /b 1
"%OUT%\isense_tests_g50.exe"
