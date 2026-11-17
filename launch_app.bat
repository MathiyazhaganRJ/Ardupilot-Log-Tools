@echo off
title ArduPilot Flight Log Analyser
cd /d "%~dp0"

where pythonw >nul 2>nul
if %errorlevel% equ 0 (
    start "" pythonw.exe app.py
    exit /b 0
)

where python >nul 2>nul
if %errorlevel% equ 0 (
    start "" python app.py
    exit /b 0
)

echo Python is not detected in your PATH. Please install Python 3.10+ from python.org.
pause

