@echo off
chcp 65001 >nul
echo ==============================================================
echo    TURRET FCS // OTOMATIK ORTAM KURULUMU (SETUP ENV)
echo ==============================================================
echo.

where python >nul 2>nul
if %errorlevel% neq 0 (
    echo [HATA] Sistemde Python bulunamadi! Lutfen Python 3.10 veya 3.12 kurun.
    pause
    exit /b 1
)

echo [1/3] Sanal ortam olusturuluyor (venv)...
if not exist "venv" (
    python -m venv venv
    echo [OK] venv olusturuldu.
) else (
    echo [INFO] venv zaten mevcut.
)

echo.
echo [2/3] Pip guncelleniyor...
call venv\Scripts\activate.bat
python -m pip install --upgrade pip

echo [3/3] Bagimliliklar yukleniyor (requirements.txt)...
python -m pip install -r requirements.txt
python -m pip install pytest
python -m pip install --no-deps -e .

echo.
echo ==============================================================
echo  [TAMAMLANDI] Tum kutuphaneler basariyla kuruldu!
echo  Artik 'main.bat' ile sistemi calistirabilirsiniz.
echo ==============================================================
pause
