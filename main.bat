@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"

echo ==============================================================
echo    TURRET FCS // GERCEK ZAMANLI ATIS KONTROL SISTEMI
echo ==============================================================
echo.

if not exist "venv\Scripts\python.exe" (
    echo [HATA] Sanal ortam bulunamadi: venv\Scripts\python.exe
    echo        Once 'setup_env.bat' betigini calistirin.
    if not defined CI pause
    exit /b 1
)

echo [INFO] Sanal ortam devrede: venv\Scripts\python.exe
venv\Scripts\python.exe -m turret_fcs.main %*

echo.
echo [FCS] Oturum sonlandirildi.
if not defined CI pause
endlocal
