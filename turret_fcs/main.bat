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

echo TURRET FCS // ACTIVATED
venv\Scripts\python.exe -m turret_fcs.main %*
echo TURRET FCS // DEACTIVATED
endlocal
