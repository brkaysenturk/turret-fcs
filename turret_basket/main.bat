@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0\.."

set PYTHONDONTWRITEBYTECODE=1

if not exist "venv\Scripts\python.exe" (
    echo [HATA] Sanal ortam bulunamadi: venv\Scripts\python.exe
    echo        Once 'setup_env.bat' betigini calistirin.
    if not defined CI pause
    exit /b 1
)

echo TURRET BASKET // ACTIVATED
venv\Scripts\python.exe -m turret_basket.main %*
echo TURRET BASKET // DEACTIVATED
endlocal
