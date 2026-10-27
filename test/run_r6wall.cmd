@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
rem the wall-switch pattern on main.c: N sessions, each its own process (a real power
rem cut: the RAM goes, the flash image stays in a file). Usage: test\run_r6wall.cmd <soak_tests.exe> [sessions]
setlocal
set "EXE=%~1"
set "N=%~2"
if "%N%"=="" set "N=40"
set "IMG=%TEMP%\r6wall_%RANDOM%.bin"
if exist "%IMG%" del "%IMG%"
for /l %%s in (1,1,%N%) do (
  "%EXE%" r6wall %%s "%IMG%" > "%IMG%.log" 2>&1
  if errorlevel 1 (
    type "%IMG%.log"
    echo r6wall: session %%s FAILED
    exit /b 1
  )
)
type "%IMG%.log"
echo r6wall: %N% sessions passed
del "%IMG%" "%IMG%.log"
exit /b 0
