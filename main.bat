@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ==============================================================
echo    TURRET FCS // GERCEK ZAMANLI ATIS KONTROL SISTEMI
echo ==============================================================
echo.

if exist "venv\Scripts\python.exe" (
    "venv\Scripts\python.exe" turret_fcs\main.py
) else if exist "..\venv\Scripts\python.exe" (
    "..\venv\Scripts\python.exe" turret_fcs\main.py
) else (
    python turret_fcs\main.py
)

pause
