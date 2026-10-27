@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
set "OUT=%~1"
if "%OUT%"=="" set "OUT=build\ur_digital_m-scene"
set "PF86=%ProgramFiles(x86)%"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
set "PATH=%PF86%\Microsoft Visual Studio\Installer;%PATH%"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if not exist "%OUT%" mkdir "%OUT%"
cl /DBOARD=3 /nologo /O2 /W3 /D_CRT_SECURE_NO_WARNINGS /I"%~dp0..\src\core" /Fe:"%OUT%\scene.exe" /Fo:"%OUT%\\" "%~dp0test_ur_digital_m_scene.c" "%~dp0..\src\core\dline.c" "%~dp0..\src\core\ur_digital_m\udfx.c" "%~dp0..\src\core\ur_digital_m\wsenc.c" "%~dp0..\src\core\fx_math.c" "%~dp0..\src\core\fx_effects.c" "%~dp0..\src\core\fx_engine.c" "%~dp0..\src\core\fx_pack.c" "%~dp0..\src\core\fx_tables.c" "%~dp0..\src\core\math32.c"
if errorlevel 1 exit /b 1
if "%~2"=="" (
    "%OUT%\scene.exe" frame
    if errorlevel 1 exit /b 1
    "%OUT%\scene.exe" crossfade
) else (
    "%OUT%\scene.exe" %2
)
exit /b %errorlevel%
