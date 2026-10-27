@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem USART1 divider tests on both receiver boards, plus rejected selectors.
rem Usage: test\build_answer_uart.cmd [outdir]   (default outdir: build\host). build_host.cmd runs it.
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
call :one 2 1
if %errorlevel% neq 0 exit /b 1
call :one 3 1
if %errorlevel% neq 0 exit /b 1
for %%B in (2 3) do for %%U in (2 3) do (
  cl /nologo /EP /DBOARD=%%B /DANSWER_UART=%%U "%HERE%test_answer_brr.c" >"%OUT%\reject-%%B-%%U.log" 2>&1
  if not errorlevel 1 exit /b 1
  findstr /C:"Receiver boards require ANSWER_UART=1" "%OUT%\reject-%%B-%%U.log" >nul
  if errorlevel 1 exit /b 1
)
exit /b 0

:one
if not exist "%OUT%\answer_uart_%1" mkdir "%OUT%\answer_uart_%1"
cl /nologo /O2 /W4 /wd4996 /wd4127 /DUPD_CRC_ULTRA /DBOARD=%1 /DANSWER_UART=%2 /Fe:"%OUT%\answer_brr_%1.exe" /Fo:"%OUT%\answer_uart_%1\\" ^
   "%HERE%test_answer_brr.c" "%HERE%..\src\core\rc.c" "%HERE%..\src\core\math32.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\answer_brr_%1.exe"
if %errorlevel% neq 0 exit /b 1
exit /b 0
