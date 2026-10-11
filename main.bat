@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ==============================================================
echo    TURRET FCS // GERCEK ZAMANLI ATIS KONTROL SISTEMI
echo ==============================================================
echo.

if exist "venv\Scripts\python.exe" (
    echo [INFO] Sanal ortam devrede: venv\Scripts\python.exe
    "venv\Scripts\python.exe" turret_fcs\main.py
) else if exist "..\venv\Scripts\python.exe" (
    echo [INFO] Sanal ortam devrede: ..\venv\Scripts\python.exe
    "..\venv\Scripts\python.exe" turret_fcs\main.py
) else (
    echo [UYARI] venv bulunamadi, sistem genelindeki Python kullaniliyor...
    python turret_fcs\main.py
)

echo.
echo [FCS] Oturum sonlandirildi.
pause
