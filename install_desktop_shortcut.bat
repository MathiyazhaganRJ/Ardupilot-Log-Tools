@echo off
title Install ArduPilot Flight Log Analyser Shortcut
cd /d "%~dp0"

echo ========================================================
echo  ArduPilot Flight Log Analyser - Desktop Setup
echo ========================================================
echo.
echo Installing desktop shortcut for current user...
echo.

python create_desktop_shortcut.py

if %errorlevel% equ 0 (
    echo.
    echo ========================================================
    echo  SUCCESS: Shortcut created on your Desktop!
    echo ========================================================
    echo You can now double-click 'ArduPilot Flight Log Analyser'
    echo from your Desktop anytime to launch the application.
) else (
    echo.
    echo ========================================================
    echo  ERROR: Failed to create shortcut.
    echo ========================================================
    echo Please verify Python 3.10+ is installed on this system.
)

echo.
timeout /t 5
