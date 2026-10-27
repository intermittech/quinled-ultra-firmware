@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem Many whole-firmware fuzz runs, one process each (a fresh boot): run_robust_fw.cmd <first seed> <count> [sim-seconds] [exe]
rem Prints every run's summary line; exits 1 when any run failed. (test\build_robust.cmd builds robust_fw.exe.)
setlocal enabledelayedexpansion
set FIRST=%1
set COUNT=%2
set SECS=%3
if "%SECS%"=="" set SECS=3
set EXE=%4
if "%EXE%"=="" set "EXE=%~dp0..\build\robust\robust_fw.exe"
set FAILS=0
set /a LAST=%FIRST%+%COUNT%-1
for /l %%s in (%FIRST%,1,%LAST%) do (
  "%EXE%" %%s %SECS%
  if errorlevel 1 set /a FAILS+=1
)
echo robust_fw: %COUNT% runs from seed %FIRST%, %FAILS% failed
if %FAILS% neq 0 exit /b 1
