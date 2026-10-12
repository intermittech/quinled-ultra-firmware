@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Build and run the isense host test (core/isense.c against the board model) with MSVC.
rem Usage: test\build_isense.cmd [output dir]   (default %TEMP%\isense_host; keeps clear of build\host)
setlocal
set HERE=%~dp0
set OUT=%~1
if "%OUT%"=="" set "OUT=%TEMP%\isense_host"
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
cl /nologo /O2 /W4 /wd4996 /Fe:"%OUT%\isense_tests.exe" /Fo:"%OUT%\\" "%HERE%test_isense.c" "%HERE%..\src\core\isense.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\isense_tests.exe"
