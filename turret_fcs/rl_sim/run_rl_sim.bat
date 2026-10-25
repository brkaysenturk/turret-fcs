@echo off
setlocal
chcp 65001 > nul
cd /d "%~dp0\..\.."

if exist "venv\Scripts\python.exe" (
    set "PY=venv\Scripts\python.exe"
) else (
    set "PY=python"
)

cls
echo ========================================================
echo        TURRET FCS // REINFORCEMENT LEARNING PANEL
echo ========================================================
echo  [1] Modeli Egit (PPO - 250.000 Adim)
echo  [2] Egitilmis Modeli Canli Izle (Interaktif Simulasyon)
echo  [3] P vs. RL Karsilastirma Grafigi Uret (Tez Icin)
echo  [4] Cikis
echo ========================================================
set /p choice="Seciminiz (1/2/3/4): "

if "%choice%"=="1" (
    echo.
    echo [INFO] PPO Egitimi Baslatiliyor...
    "%PY%" turret_fcs\rl_sim\train_rl.py
    pause
) else if "%choice%"=="2" (
    echo.
    echo [INFO] Canli Simulasyon Aciliyor ^('q' ile cikabilirsiniz^)...
    "%PY%" turret_fcs\rl_sim\test_rl.py
    pause
) else if "%choice%"=="3" (
    echo.
    echo [INFO] P vs RL Karsilastirma Testi Calistiriliyor...
    "%PY%" analysis\generate_benchmark_plots.py
    pause
) else (
    exit /b
)
