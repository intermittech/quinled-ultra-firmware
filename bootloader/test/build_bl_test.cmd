@echo off
rem SPDX-License-Identifier: GPL-3.0-or-later
rem Copyright (C) 2026 QuinLED
setlocal
python "%~dp0run_abi3.py" %*
exit /b %errorlevel%
