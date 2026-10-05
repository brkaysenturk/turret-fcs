@echo off
chcp 65001 > nul
cls
echo ========================================================
echo        TURRET FCS // REINFORCEMENT LEARNING PANELİ      
echo ========================================================
echo  [1] Modeli Eğit (PPO - 150.000 Adım)
echo  [2] Eğitilmiş Modeli Canlı İzle (İnteraktif Simülasyon)
echo  [3] PID vs. RL Karşılaştırma Grafiği Üret (Tez İçin)
echo  [4] Çıkış
echo ========================================================
set /p choice="Seçiminiz (1/2/3/4): "

if "%choice%"=="1" (
    echo.
    echo [INFO] PPO Eğitimi Başlatılıyor...
    python train_rl.py
    pause
) else if "%choice%"=="2" (
    echo.
    echo [INFO] Canlı Simülasyon Açılıyor ('q' ile çıkabilirsiniz)...
    python test_rl.py
    pause
) else if "%choice%"=="3" (
    echo.
    echo [INFO] PID vs RL Karşılaştırma Testi Çalıştırılıyor...
    python compare_pid_vs_rl.py
    pause
) else (
    exit /b
)
