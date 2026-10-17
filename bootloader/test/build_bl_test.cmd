@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Build and run the bootloader host tests with MSVC:
rem   bl_tests.exe      bl_core + bl_rx against a flash / board / host model (test_bl.c)
rem   sender_tests.exe  the ESPHome component's update sender through the same model (test_sender.cpp), when
rem                     quinled-ultra-esphome sits next to the firmware; also checks its copy of ultra_update.h
rem Usage: bootloader\test\build_bl_test.cmd [output dir]   (default %TEMP%\bl_host; keeps clear of build\host)
rem ULTRA_ESPHOME=<quinled-ultra-esphome folder>: use that one. Otherwise it is looked for next to the firmware
rem   repository (..\..\..\quinled-ultra-esphome), then next to the hardware project (..\..\..\..\quinled-ultra-esphome).
rem BL_TEST_IMAGE=<image from tools\release\mkimage.py>: also install that image (both programs).
setlocal
set HERE=%~dp0
set OUT=%~1
if "%OUT%"=="" set "OUT=%TEMP%\bl_host"
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
if not exist "%OUT%\sender" mkdir "%OUT%\sender"
cl /nologo /O2 /W4 /wd4996 /I"%HERE%.." /I"%HERE%..\..\src\core" /Fe:"%OUT%\bl_tests.exe" /Fo:"%OUT%\\" ^
   "%HERE%test_bl.c" "%HERE%..\bl_core.c" "%HERE%..\bl_rx.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\bl_tests.exe"
if %errorlevel% neq 0 exit /b 1
rem 0.6.5: the same for 0.6.4's flash layout (UPPER_RECORDS=0: the bench fallback, bootloader 1)
if not exist "%OUT%\lower" mkdir "%OUT%\lower"
cl /nologo /O2 /W4 /wd4996 /DUPPER_RECORDS=0 /I"%HERE%.." /I"%HERE%..\..\src\core" /Fe:"%OUT%\bl_tests_lower.exe" ^
   /Fo:"%OUT%\lower\\" "%HERE%test_bl.c" "%HERE%..\bl_core.c" "%HERE%..\bl_rx.c"
if %errorlevel% neq 0 exit /b 1
rem (a release image needs its own layout's bootloader: BL_TEST_IMAGE is for the default layout's run above)
setlocal
set "BL_TEST_IMAGE="
"%OUT%\bl_tests_lower.exe"
if %errorlevel% neq 0 exit /b 1
endlocal
rem (round 11) the hardware-id-3 bootloader's marker skip (bl_rx.c BL_RX_MARKER, the Ultra-Digital receiver): UPD
rem frames behind a 15-40 us marker decode with BL_HW_ID=3; hardware 1 (the negative control) takes the marker as a
rem bad first bit
if not exist "%OUT%\marker3" mkdir "%OUT%\marker3"
if not exist "%OUT%\marker1" mkdir "%OUT%\marker1"
cl /nologo /O2 /W4 /wd4996 /DBL_HW_ID=3 /I"%HERE%.." /Fe:"%OUT%\bl_marker_hw3.exe" /Fo:"%OUT%\marker3\\" ^
   "%HERE%test_bl_marker.c" "%HERE%..\bl_rx.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\bl_marker_hw3.exe"
if %errorlevel% neq 0 exit /b 1
cl /nologo /O2 /W4 /wd4996 /I"%HERE%.." /Fe:"%OUT%\bl_marker_hw1.exe" /Fo:"%OUT%\marker1\\" ^
   "%HERE%test_bl_marker.c" "%HERE%..\bl_rx.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\bl_marker_hw1.exe"
if %errorlevel% neq 0 exit /b 1
set "ESPH="
if defined ULTRA_ESPHOME set "ESPH=%ULTRA_ESPHOME%\components\dig2analog_ultra"
if not defined ESPH if exist "%HERE%..\..\..\quinled-ultra-esphome\components\dig2analog_ultra\update_session.h" (
  set "ESPH=%HERE%..\..\..\quinled-ultra-esphome\components\dig2analog_ultra"
)
if not defined ESPH set "ESPH=%HERE%..\..\..\..\quinled-ultra-esphome\components\dig2analog_ultra"
if not exist "%ESPH%\update_session.h" (
  echo quinled-ultra-esphome not found at ULTRA_ESPHOME or next to the firmware: sender test skipped
  exit /b 0
)
fc /b "%HERE%..\..\src\core\ultra_update.h" "%ESPH%\ultra_update.h" >nul
if errorlevel 1 (
  echo FAIL: quinled-ultra-esphome ultra_update.h differs from the master src\core\ultra_update.h
  exit /b 1
)
rem 4505 / 4611: test_bl.c's own tests are unused here; its setjmp is only used by those
cl /nologo /O2 /W4 /wd4996 /wd4505 /wd4611 /EHsc /std:c++17 /I"%HERE%.." /I"%HERE%." /I"%HERE%..\..\src\core" /I"%ESPH%" ^
   /Fe:"%OUT%\sender_tests.exe" /Fo:"%OUT%\sender\\" "%HERE%test_sender.cpp" "%HERE%..\bl_core.c" "%HERE%..\bl_rx.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\sender_tests.exe"
