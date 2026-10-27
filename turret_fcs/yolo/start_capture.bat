@echo off
cd /d "%~dp0"
echo ========================================================
echo   TURRET FCS - C270 CANLI EKRANLI VERI TOPLAYICI
echo ========================================================
echo.
if exist "..\..\venv\Scripts\python.exe" (
    "..\..\venv\Scripts\python.exe" capture_dataset.py
) else if exist "..\venv\Scripts\python.exe" (
    "..\venv\Scripts\python.exe" capture_dataset.py
) else if exist "venv\Scripts\python.exe" (
    "venv\Scripts\python.exe" capture_dataset.py
) else (
    python capture_dataset.py
)
pause
