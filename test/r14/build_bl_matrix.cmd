@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
set HERE=%~dp0
set "OUT=%HERE%..\..\build\r14-bl-matrix"
set "REL=%HERE%..\..\build\release\0.6.11-rc.14"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSDIR=%%i
if not defined VSDIR exit /b 2
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
for %%h in (1 2 3) do (
  call :one %%h
  if errorlevel 1 exit /b 1
)
exit /b 0
:one
set REV=0
if "%1"=="1" set REV=3
if not exist "%OUT%\%1" mkdir "%OUT%\%1"
cl /nologo /O2 /W4 /wd4996 /wd4505 /Zi /fsanitize=address /DBL_HW_ID=%1 /DBL_HW_REV=%REV% /I"%HERE%..\..\bootloader" /I"%HERE%..\..\src\core" /Fe:"%OUT%\%1\matrix.exe" /Fo:"%OUT%\%1\\" /Fd:"%OUT%\%1\matrix.pdb" "%HERE%test_bl_matrix.c" "%HERE%..\..\bootloader\bl_core.c" "%HERE%..\..\bootloader\bl_rx.c" /link /INCREMENTAL:NO
if errorlevel 1 exit /b 2
"%OUT%\%1\matrix.exe" "%REL%\quinled-dig2analog-ultra-5ch-0.6.11-rc.14-rev04.bin" "%REL%\quinled-diff-power-ultra-analog-0.6.11-rc.14-rx-rev0.bin" "%REL%\quinled-diff-power-ultra-digital-0.6.11-rc.14-ud-rev0.bin"
exit /b %errorlevel%
