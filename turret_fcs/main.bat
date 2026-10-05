@echo off
cd /d "%~dp0"

:: Eger bat dosyasi turret_fcs klasorunun icindeyse dogrudan buradadir, degilse icine gir
if exist "main.py" (
    rem Zaten main.py dosyasinin oldugu klasordeyiz
) else if exist "turret_fcs\main.py" (
    cd turret_fcs
)

:: Sanal ortami (venv) bul ve calistir
if exist "..\venv\Scripts\python.exe" (
    "..\venv\Scripts\python.exe" main.py
) else if exist "venv\Scripts\python.exe" (
    "venv\Scripts\python.exe" main.py
) else (
    python main.py
)

pause