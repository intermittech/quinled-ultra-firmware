@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Build and run the effect engine's host tests (MSVC). Usage: test\build_fx.cmd [outdir] [render]
rem   outdir default build\fx; test_fx.exe = the engine's tests (determinism, golden frames, segments, transitions, packer);
rem   with "render" also fx_render.exe (PNG strips: fx_render sheet|one|pal ...).
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\fx"
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
set CORE=%HERE%..\src\core
cl /nologo /O2 /W4 /D_CRT_SECURE_NO_WARNINGS /Fe:"%OUT%\test_fx.exe" /Fo:"%OUT%\\" "%HERE%test_fx.c" "%CORE%\fx_math.c" "%CORE%\fx_engine.c" "%CORE%\fx_pack.c" "%CORE%\fx_tables.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\test_fx.exe"
if %errorlevel% neq 0 exit /b 1
python "%HERE%test_fx_catalogue.py"
if %errorlevel% neq 0 exit /b 1
if /i "%~2"=="render" (
  cl /nologo /O2 /W3 /D_CRT_SECURE_NO_WARNINGS /Fe:"%OUT%\fx_render.exe" /Fo:"%OUT%\\" "%HERE%fx_render.c" "%CORE%\fx_effects.c" "%CORE%\fx_math.c" "%CORE%\fx_tables.c"
  if %errorlevel% neq 0 exit /b 1
)
exit /b 0
