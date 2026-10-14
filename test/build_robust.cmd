@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Build the randomized robustness tests with MSVC and AddressSanitizer. Usage: test\build_robust.cmd [outdir]
rem robust_tests.exe (test_robust.c: every input path of src\core, property-based) and robust_fw.exe
rem (test_robust_fw.c: the whole firmware, src\app\main.c on a model of the hardware, random traffic / console / flash).
rem Run them with the arguments the files describe; the default run of each is in the final lines here.
setlocal
set HERE=%~dp0
set "OUT=%~1"
if "%OUT%"=="" set "OUT=%HERE%..\build\robust"
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
if not exist "%OUT%\fw" mkdir "%OUT%\fw"
rem the console's compressed text (core/cz.h): tools/gen_console_text.py writes %OUT%\gen\cz_text.h
python "%HERE%..\tools\gen_console_text.py" "%OUT%\gen" --upper --full
if %errorlevel% neq 0 exit /b 1
rem /fsanitize=address: heap / global / stack overflows and use after free abort the run with a report
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /Zi /fsanitize=address /I "%OUT%\gen" /Fe:"%OUT%\robust_tests.exe" /Fo:"%OUT%\\" /Fd:"%OUT%\robust_tests.pdb" ^
   "%HERE%test_robust.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
rem (4457 / 4312: main.c built for the 64-bit host with UPD_APP = 1: a shadowed name, flash addresses cast to pointers)
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /wd4457 /wd4312 /Zi /fsanitize=address /I "%HERE%soak" /I "%OUT%\gen" /Fe:"%OUT%\robust_fw.exe" /Fo:"%OUT%\fw\\" ^
   /Fd:"%OUT%\robust_fw.pdb" "%HERE%test_robust_fw.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
if "%2"=="nobuild-only" exit /b 0
"%OUT%\robust_tests.exe" regress
if %errorlevel% neq 0 exit /b 1
"%OUT%\robust_fw.exe" regress
if %errorlevel% neq 0 exit /b 1
rem (0.6.8) the bootloader's write protection at the boot (main.c boot_protect), each case its own process
for %%c in (open protected vdd nodesc rdp fail) do (
  "%OUT%\robust_fw.exe" wrp %%c
  if errorlevel 1 exit /b 1
)
"%OUT%\robust_tests.exe" all 2000
if %errorlevel% neq 0 exit /b 1
"%OUT%\robust_fw.exe" 20
if %errorlevel% neq 0 exit /b 1
rem (review round 6, gpt-protocol) the POWER unit test with AddressSanitizer: its ACK buffer holds the nine bytes
if not exist "%OUT%\r6rc" mkdir "%OUT%\r6rc"
cl /nologo /Od /Zi /fsanitize=address /DUPD_CRC_ULTRA /DCONSOLE_FULL=1 /I "%OUT%\gen" /Fe:"%OUT%\r6rc_asan.exe" /Fo:"%OUT%\r6rc\\" ^
   /Fd:"%OUT%\r6rc_asan.pdb" "%HERE%test_r6_host_rc.c" "%HERE%..\src\core\*.c" /link /INCREMENTAL:NO
if %errorlevel% neq 0 exit /b 1
"%OUT%\r6rc_asan.exe" ack-buffer
if %errorlevel% neq 0 exit /b 1
rem (0.6.7) the whole firmware with the release console (CONSOLE_FULL 0: it only sends; random bytes on TP6 ignored)
python "%HERE%..\tools\gen_console_text.py" "%OUT%\genrel" --upper
if %errorlevel% neq 0 exit /b 1
if not exist "%OUT%\fwrel" mkdir "%OUT%\fwrel"
cl /nologo /O2 /W4 /wd4996 /DUPD_CRC_ULTRA /DCONSOLE_FULL=0 /wd4457 /wd4312 /Zi /fsanitize=address /I "%HERE%soak" /I "%OUT%\genrel" /Fe:"%OUT%\robust_fw_rel.exe" /Fo:"%OUT%\fwrel\\" ^
   /Fd:"%OUT%\robust_fw_rel.pdb" "%HERE%test_robust_fw.c" "%HERE%..\src\core\*.c"
if %errorlevel% neq 0 exit /b 1
"%OUT%\robust_fw_rel.exe" 20
if %errorlevel% neq 0 exit /b 1
