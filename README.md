# Otonom Asimetrik Hava Tehdidi Tespit ve Atış Kontrol Sistemi (Turret FCS)
### YTU Bitirme Çalışması & Tasarım Projesi

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![STM32 Nucleo](https://img.shields.io/badge/Hardware-STM32F401RE-002B49.svg)](https://www.st.com/)
[![Webots Simulator](https://img.shields.io/badge/Simulation-Webots%20R2025a-red.svg)](https://cyberbotics.com/)
[![YOLOv8](https://img.shields.io/badge/YOLO-v8n-00FFFF.svg)](https://ultralytics.com/)
[![Reinforcement Learning](https://img.shields.io/badge/RL-PPO%20(SB3)-orange.svg)](https://stable-baselines3.readthedocs.io/)

Bu proje; mikro/mini İHA'lar, kamikaze dronlar ve asimetrik hava tehditlerine karşı geliştirilmiş; **Kamera + ToF Lidar Sensör Füzyonu**, **YOLOv8 Derin Öğrenme Hedef Tespiti**, **Genişletilmiş Kalman Filtresi (EKF) Yörünge Kestirimi** ve **Hibrit PID + PPO Takviyeli Öğrenme (RL)** tabanlı otonom bir Elektro-Optik Atış Kontrol Sistemidir (FCS).

---

## 🎯 Temel Özellikler ve Yetenekler

* **Çift Modlu Algılama ve Sensör Füzyonu:** RGB Kamera (Logitech C270) ve TF-Luna ToF Lidar sensörünün yüksek hızlı veri füzyonu.
* **Yapay Zeka Destekli Hedefleme:** Gerçek zamanlı YOLOv8n nesne tespiti ve ByteTrack tabanlı çoklu hedef takibi.
* **Balistik ve Kinematik Kestirim:** Hedefin 3 boyutlu hız ve ivmesini modelleyen Kalman Filtresi ile önleme açısı (lead angle) ve yerçekimi balistik düşüş kompanzasyonu.
* **Hibrit Çift Katmanlı Kontrol Mimarisi:**
  * **PID Kontrolcü:** Düşük gecikmeli, kararlı açısal merkezleme.
  * **PPO Derin Takviyeli Öğrenme (RL):** Ani manevra yapan tehditlere karşı çevik ve adaptif açısal hız tahmini.
* **Askeri Standartta Sayısal HUD (Heads-Up Display):** Hedef menzili, yaklaşma hızı, kilit durumu, balistik nişangah ve atış penceresi gösterimi.
* **Gömülü Donanım Mimarisi:** STM32F401RE tabanlı 100 Hz UART haberleşmeli, donanımsal limit anahtarlı servo kontrolcü.
* **Dijital İkiz Simülasyonu:** Webots R2025a fizik motorunda 1:1 ölçekli aerodinamik ve mekanik simülasyon doğrulaması.

---

## 🏛️ Sistem ve Yazılım Mimarisi

```
+-----------------------------------------------------------------------------------+
|                            GÖRSEL VE SENSÖR GİRDİSİ                                |
|  [ Kamera (1280x720 @ 30 FPS) ]           [ TF-Luna ToF Lidar (UART Mesafe) ]     |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                             HEDEF TESPİT VE İZLEME                                 |
|  * YOLOv8n Derin Öğrenme Modeli (Balon / Mini İHA Tespiti)                        |
|  * ByteTrack / Kalman Filtresi (3D Konum, Hız ve İvme Kestirimi)                   |
|  * Askeri HUD Arayüzü & Hedef Önceliklendirme (FCS Manager)                       |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                           KONTROL VE NİŞAN ALMA KATMANI                           |
|  * Balistik Düşüş & Önleme Açısı Hesaplama (Ballistics Engine)                    |
|  * Hibrit Kontrolcü: Klasik PID + PPO Takviyeli Öğrenme (RL Controller)           |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                        GÖMÜLÜ DONANIM / FİZİKSEL EYLEME                           |
|  * STM32F401RE Mikrodenetleyici (100 Hz Kontrol Döngüsü)                          |
|  * Çift Eksen Pan-Tilt Servo Sürücüleri (Yüksek Torklu Açısal Yönlendirme)        |
|  * Fırlatıcı ve Tetik Kontrol Birimi                                              |
+-----------------------------------------------------------------------------------+
```

---

## 📊 Webots Simülasyon Doğrulaması (10 Atışlık Performans Analizi)

Sistem Webots R2025a ortamında farklı mesafelerdeki (1.2m - 5.0m) ve farklı irtifalardaki hareketli hava hedeflerine karşı test edilmiş ve **%100 vuruş hassasiyeti (10/10)** doğrulanmıştır:

| Parametre | Değer |
| :--- | :--- |
| **Toplam Angajman / Başarı** | 10 Atış / 10 İsabet (%100 Başarı) |
| **Ortalama Hedefe Kilitlenme Süresi** | 0.42 saniye |
| **Ortalama Açısal Hata (Hedefe İsabette)** | 0.18 derece |
| **Maksimum Test Menzili** | 5.0 metre |
| **Kontrol Tepki Frekansı** | 100 Hz (STM32) / 30+ FPS (Python FCS) |

---

## 🚀 Hızlı Başlangıç ve Kurulum

### 1. Gereksinimler
* Windows 10/11 x64
* Python 3.10 veya 3.12 (*"Add Python to PATH"* seçili olmalıdır)
* STM32 Nucleo-F401RE Kartı & Logitech C270 Web Kamerası

### 2. Otomatik Ortam Kurulumu
Depoyu klonlayın ve ana dizindeki kurulum betiğini çalıştırın:
```bash
git clone https://github.com/brkysnturk/turret_fcs.git
cd turret_fcs
setup_env.bat
```
Bu betik sanal ortamı (`venv`) kurar, `pip`'i günceller ve [requirements.txt](requirements.txt) içerisindeki tüm bağımlılıkları yükler.

### 3. Sistemi Çalıştırma
Donanım kablolarını taktıktan sonra tek tıkla sistemi başlatın:
```bash
turret_fcs\main.bat
```

### 🎮 Kontrol Tuşları
* **`ESC` :** Programı ve telemetri kaydını güvenli kapatma (CSV dosyasını anında kaydeder).
* **`SPACE` (Boşluk):** Acil Durdurma (E-STOP) - Motorları ve kilitlenmeyi anında frenler.
* **`Y` :** Manuel Ateş İzni (Manual Fire Trigger).
* **`M` :** Kontrol Modu Değiştirici (`HYBRID` <-> `PID` <-> `RL`).

---

## 📈 Akademik Dinamik Analiz ve Sistem Tanıma

Taret kontrol dinamiklerini ve sistem tanıma (SOPDT) sonuçlarını incelemek için `analysis/` araçları kullanılır:

```bash
# 1. 6 Panelli Detaylı Dinamik ve Kontrol Kararlılık Analizi (300 DPI)
python analysis/plot_tuned_system.py

# 2. 4 Panelli Sistem Tanıma, S-Düzlemi Root Locus ve Doğrulama
python analysis/system_identification.py
```
*(Akademik tez veya jüri sunumu için MATLAB ortamında `analysis/turret_fcs_analysis.m` dosyasını çalıştırabilirsiniz).*

---

## 📂 Dizin Yapısı

```
turret_fcs/
├── requirements.txt           # Python kütüphane bağımlılıkları
├── setup_env.bat              # Otomatik sanal ortam (venv) ve kütüphane kurulum aracı
├── README.md                  # Proje mimarisi ve teknik dokümantasyon
├── analysis/                  # Akademik kontrol ve sistem tanıma araçları
│   ├── plot_tuned_system.py   # Python 6 panelli dinamik analiz motoru
│   ├── system_identification.py # Basamak eğri uyumu ve S-düzlemi root locus
│   ├── turret_fcs_analysis.m  # MATLAB tez analiz ikizi
│   └── figures/               # Yüksek çözünürlüklü (300 DPI) yayın grafikleri
├── logs/                      # Gerçek donanım telemetri test kayıtları (.csv)
├── turret_fcs/                # Gerçek donanım çalışma kodları
│   ├── main.py                # Ana atış kontrol döngüsü ve hedef koordinasyonu
│   ├── main.bat               # Tek tıkla donanım başlatıcı
│   ├── telemetry.py           # Kare-kare donanım telemetri kaydedicisi
│   ├── fcs_manager.py         # Hedef seçim, kilit ve balistik hesap motoru
│   ├── fcs_comm.py            # STM32 UART binary paket haberleşme sürücüsü
│   ├── kalman_filter.py       # Kalman filtreli 3D yörünge kestirimi
│   ├── turret_hud.py          # Askeri standartta dinamik HUD arayüzü
│   ├── rl_controller.py       # PPO takviyeli öğrenme denetleyicisi
│   ├── pid_controller.py      # Optimize PD kontrolcü
│   └── camera_thread.py       # Sıfır gecikmeli çok iş parçacıklı kamera sürücüsü
├── stm32/                     # STM32F401RE Nucleo gömülü yazılım projesi (CubeIDE)
│   └── turret_main/           # Ana taret donanım projesi (.ioc, Core/Src, Core/Inc)
│       └── Core/Src/          # fcs_comm.c, servo_driver.c, luna_read.c, flywheel_solenoid.c, main.c
├── webots/                    # Webots dijital ikiz simülasyonu
│   ├── controllers/           # Simülasyon kontrolcüleri
│   ├── worlds/                # 3D simülasyon dünyası
│   └── analysis/              # Performans grafik motorları ve telemetri verileri
└── weights/                   # Eğitilmiş yapay zeka model ağırlıkları (best.pt)
```

---

## 📜 Lisans & Akademik Referans
Bu proje Yıldız Teknik Üniversitesi Bitirme Çalışması & Tasarım Projesi kapsamında geliştirilmiştir.
Tüm hakları proje ekibine aittir.
