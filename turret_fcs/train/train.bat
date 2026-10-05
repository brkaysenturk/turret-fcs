@echo off
cd /d "%~dp0"

echo [BILGI] Veri setleri birlestiriliyor (merge_datasets.py)...
"..\..\venv\Scripts\python.exe" merge_datasets.py

echo.
echo [BILGI] Birlestirilmis veri seti ile egitim baslatiliyor (train.py)...
"..\..\venv\Scripts\python.exe" train.py

echo.
echo [TAMAMLANDI] Egitim bitti!
pause
