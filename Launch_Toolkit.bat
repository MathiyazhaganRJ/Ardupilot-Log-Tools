@echo off
title ArduPilot Toolkit Launcher
color 0B
echo ========================================================
echo         Starting ArduPilot Data Toolkit...
echo ========================================================
echo.
:: Start Streamlit silently in the background
start /B streamlit run 1_Log_Manager.py --server.headless true

:: Give the server 3 seconds to boot up
timeout /t 3 /nobreak >nul

:: Open as a standalone Native App Window (no tabs, no URL bar)
start msedge --app=http://localhost:8501

echo.
echo 🟢 System is running. 
echo ❌ Close this black console window to shut down the server when you are done.
pause
