# Otonom Asimetrik Hava Tehdidi Tespit ve Atış Kontrol Sistemi (Turret FCS)

## Yıldız Teknik Üniversitesi — Kontrol ve Otomasyon Mühendisliği Bölümü

### Bitirme Çalışması & Tasarım Projesi (v1.1 Production Release)

[![Release](https://img.shields.io/badge/Release-v1.1-success.svg)](https://github.com/brkysnturk/turret_fcs/releases/tag/v1.1)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![STM32 Nucleo](https://img.shields.io/badge/Hardware-STM32F401RE-002B49.svg)](https://www.st.com/)
[![Webots Simulator](https://img.shields.io/badge/Simulation-Webots%20R2025a-red.svg)](https://cyberbotics.com/)
[![YOLOv8](https://img.shields.io/badge/YOLO-v8n-00FFFF.svg)](https://ultralytics.com/)
[![Reinforcement Learning](https://img.shields.io/badge/RL-11D%20PPO%20(SB3)-orange.svg)](https://stable-baselines3.readthedocs.io/)
[![Control Theory](https://img.shields.io/badge/Control-Type--1%20P%20%2B%202--DOF%20Feedforward-brightgreen.svg)](https://en.wikipedia.org/wiki/Proportional_control)

Bu proje; mikro/mini İHA'lar ve asimetrik hava tehditlerine karşı geliştirilmiş; **Logitech C270 RGB Kamera + Benewake TF-Luna ToF LiDAR Sensör Füzyonu**, **YOLOv8n Derin Öğrenme Hedef Tespiti**, **ByteTrack Çoklu Hedef Takibi**, **Adaptif Kalman Filtresi Hız/Konum Kestirimi**, **2-DOF Referans Hızı İleri Beslemeli (Velocity Feedforward) Tip-1 P Kontrolcü**, **Menzil Kapılama (Range Gating)** ve **11-Boyutlu Eylem Geçmişli PPO Derin Pekiştirmeli Öğrenme (RL)** tabanlı otonom bir Elektro-Optik Atış Kontrol Sistemidir (FCS).

---

## 🚀 v1.1 Güncellemesi İle Gelen Yenilikler

* **Kalman Tabanlı 2-DOF Hız İleri Beslemesi (Velocity Feedforward):**
  * Klasik PID'deki gürültülü sonlu farklar türevi yerine, Kalman filtresinin kestirdiği filtrelenmiş hedef açısal hızı doğrudan 2 serbestlik dereceli (2-DOF) referans hızı ileri beslemesi olarak aktarılır.
  * Tip-1 ayrık integratör döngüsünde hareketli hedeflere karşı oluşan rampa hız geride kalma hatasını (velocity lag) ortadan kaldırır.
  * `p_controller.py` içerisinden `ENABLE_D_TERM = True / False` anahtarı ile anında açılıp kapatılabilir.
* **Menzil Kapılama (Range Gating) Koruması:**
  * Hedef namlu merkezinden uzaktayken (`angular_error > 3.0°`) TF-Luna LiDAR ışınının arkadaki duvara çarpıp mesafeyi aniden şişirmesini engeller; son bilinen geçerli hedef mesafesini dondurur (`last_valid_target_dist_m`).
* **11-Boyutlu Eylem Geçmişli PPO Pekiştirmeli Öğrenme Ajanı:**
  * Simülasyon-gerçeklik boşluğunu (Sim-to-Real gap) kapatmak için 2 adımlı eylem gecikme belleği (`act_t-1, act_t-2`) ve filtrelenmiş hata türevi gözlem uzayına dahil edildi.
  * Servo gecikmesi, video akış dalgalanması ve tespit titremesini içeren Alan Rastgelelendirmesi (Domain Randomization) altında eğitilen PPO ajanı; simülasyon ve gerçek dünya koşullarında yüksek kararlılık ve takip doğruluğu sağlar (+%45.7 doğruluk artışı).
* **Ters Kayan Pencere (Sliding Window) UART Ayrıştırıcısı:**
  * PC-STM32 arasındaki 115200 Baud ikili veri hattında elektriksel parazit veya paket bölünmesi durumunda faz kayması ve gecikme yaşanmasını tamamen önleyen, ters yönde taramalı sağlam ayrıştırıcı devreye alındı.
* **Çok Katmanlı Donanımsal ve Yazılımsal Güvenlik (Fail-Safe):**
  * **300 ms STM32 Heartbeat Watchdog:** PC kilitlenirse veya iletişim kesilirse fırlatıcı motorlar ve solenoid anında acil frenlenir (`Launcher_EmergencyStop()`).
  * **350 ms Kamera Donma Bekçisi:** Video akışı kesilirse hayali hedefe atış yapılmasını önler.
  * **Mandallı (Latched) E-Stop:** Boşluk (Space) tuşu ile tek hamlede sistemi acil frenler.
  * **120 ms Kenar Tetiklemeli Solenoid Mandalı:** Tek onayla birden fazla dart atılmasını donanımsal olarak engeller.
* **Lissajous Eğrisi Dinamik Devriye Taraması:**
  * Hedef menzil dışında veya kayıpken hava sahası Lissajous sonsuzluk yörüngesi ile taranır (Pan: ±35°, Tilt: ±8°); hedef tespit edilmediğinde sakin merkeze dönüş gerçekleştirilir.

---

## 🎯 Temel Özellikler ve Yetenekler

* **Çift Modlu Algılama ve Sensör Füzyonu:** RGB Kamera (Logitech C270, 720p @ 30 FPS) ve TF-Luna ToF LiDAR sensörünün (100 Hz UART) yüksek hızlı veri füzyonu.
* **Yapay Zeka Destekli Hedefleme:** Gerçek zamanlı YOLOv8n nesne tespiti ve ByteTrack tabanlı kilit sadakatli (lock persistence) çoklu hedef izleme algoritması.
* **Balistik ve Kinematik Kestirim (Lead Pursuit):** Hedefin görüntü düzlemindeki konum ve hız vektörünü kestiren Kalman Filtresi (`kalman_filter.py`), dinamik mermi uçuş süresi önleme açısı (lead angle) ve yerçekimi balistik düşüş (süper-elevasyon) telafisi.
* **Çok Katmanlı Üçlü Kontrol Mimarisi (`[M]` Tuşu ile Değiştirilebilir):**
  * **`[P CONTROLLER]`:** Tip-1 ayrık integratör karakteristiği, 2-DOF Kalman hız ileri beslemesi (velocity feedforward) ve yumuşak ölü bölge (soft deadband) ile çalışan titreşimsiz kararlı açısal merkezleme.
  * **`[HYBRID CONTROLLER (RL:X%)]`:** Kararlı P omurgasını korurken, hedef manevralarında gecikmeyi yenmek için devreye giren dinamik PPO takviyesi (maksimum %55 AI desteği). Kilit oturduğunda AI desteği sönümlenir.
  * **`[RL CONTROLLER]`:** 11-boyutlu eylem geçmişli ve 80 ms donanım gecikme tamponlu tam otonom PPO ajanının doğrudan açısal hız kontrolü.
* **Taktiksel Operatör HUD (Heads-Up Display):**
  * **Renk Disiplini:** Devriye modunda **Yeşil**, Takipte **Açık Mavi**, Hedef kilitlendiğinde ve atış anında **Kırmızı**.
  * **Operatör Yetkilendirme Paneli:** `TARGET LOCKED` / `AUTHORIZE FIRE? [Y/N]`.
  * **Kademeli Vuruş Olasılığı (Hit Probability):** Gri (< %40), Amber (%40 - %69), Kırmızı (>= %70).
  * **Taktiksel Event Log Konsolu:** Tüm sistem olayları (hedef takibi, kilitlenme, atış, devriye, mod değişimi, LiDAR durumu ve E-STOP) ekranın sol üstünde dinamik durum akışıyla gösterilir.
  * **Muharebe Sadeleştirmesi (Decluttering):** Kilit ve atış esnasında operatörün dikkatini dağıtmamak için minimal arayüze geçer.
* **Gömülü Donanım Mimarisi:** STM32F401RE Nucleo tabanlı 100 Hz ikili (binary) UART haberleşmeli, donanımsal limit korumalı çift MG996R servo sürücüsü, 300 ms heartbeat güvenlik bekçili volan ve solenoid dart itici kontrolü.
* **Dijital İkiz Simülasyonu:** Webots R2025a fizik motorunda kinematik ve dinamik simülasyon doğrulaması.

---

## 🏛️ Sistem ve Donanım Mimarisi

Aşağıdaki şemalar sistemin PC yazılım boru hattını, STM32 gömülü mimarisini, Webots simülasyon motorunu ve elektriksel güç dağıtım katmanını göstermektedir:

### 1. Turret FCS Yazılım Pipeline Şeması

![Turret FCS Mimari Şeması](turret_fcs_architecture.png)

### 2. STM32F401RE Gömülü Çekirdek & Sürücü Mimarisi

![STM32 Gömülü Mimari](stm32_architecture.png)

### 3. Webots Dijital İkiz Simülasyon Mimarisi

![Webots Simülasyon Mimarisi](webots_architecture.png)

### 4. Donanım ve Güç Dağıtım Şeması

Projenin elektriksel güç dağıtımı, sensör entegrasyonu ve aktüatör bağlantıları [turret_fcs_hardware.svg](turret_fcs_hardware.svg) dosyasında detaylandırılmıştır.

### 5. 3D Mekanik Montaj ve Donanım Tasarımı

Taretin 3D yazıcıdan üretilecek açık beşikli, minimal optik paralakslı ve gizli servo hornlu elektromekanik montaj tasarımı:

![Turret FCS 3D Mekanik Montaj](turret_cad_render_annotated.jpg)

```text
+-----------------------------------------------------------------------------------+
|                            GÖRSEL VE SENSÖR GİRDİSİ                               |
|  [ Logitech C270 (1280x720 @ 30 FPS) ]     [ Benewake TF-Luna LiDAR (100 Hz UART) ]|
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                             HEDEF TESPİT VE İZLEME                                |
|  * YOLOv8n Derin Öğrenme Modeli (Hava Hedefi / Mini İHA Tespiti)                 |
|  * ByteTrack / Kalman Filtresi (3D Konum, Hız ve İvme Kestirimi)                  |
|  * Menzil Kapılama (Range Gating) Duvar Parazit Filtreleme                        |
|  * Taktiksel Askeri HUD Arayüzü & Hedef Önceliklendirme (FCS Manager)             |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                           KONTROL VE NİŞAN ALMA KATMANI                           |
|  * Balistik Düşüş & Dinamik Önleme Açısı Hesaplama (Ballistics Engine)            |
|  * Çok Modlu Kontrol: P + 2-DOF Hız İleri Beslemesi / Hibrit / Saf 11-D RL        |
|  * 80 ms Taşıyıcı Gecikme Telafisi & Eylem Geçmişi Hafızası (Action History)       |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                        GÖMÜLÜ DONANIM / FİZİKSEL EYLEME                           |
|  * STM32F401RE Nucleo Mikrodenetleyici (100 Hz UART Kontrol Döngüsü)              |
|  * 300 ms Heartbeat Emniyet Watchdog'u (Otomatik Acil Fren / Kesme)               |
|  * Çift Eksen Pan-Tilt MG996R Servoları (50 Hz PWM Açısal Yönlendirme)            |
|  * Çift DC Volan Motoru (23.500 RPM) ve 120 ms Mandallı Solenoid Dart İticisi     |
+-----------------------------------------------------------------------------------+
```

---

## 🔄 Algoritmik Durum Akış Şeması (Flowchart)

Sistemin arama, tespit, takip, kilitlenme ve atış sekansları [turret_fcs_flowchart.svg](turret_fcs_flowchart.svg) dosyasında modellenmiştir:

* **DURUM 1 (Devriye / Arama):** Hedef yokken Lissajous parametrik tarama eğrileriyle (Pan: ±35°, Tilt: ±8°) hava sahası taranır; HUD yeşil renktedir.
* **DURUM 2 (Tespit & Mesafe):** Hedef algılandığında TF-Luna LiDAR ile 100 Hz mesafe ve sinyal gücü doğrulanır; Menzil Kapılama ile duvar okumaları filtrelenir; hedef kimliği kilitlenir.
* **DURUM 3 (Takip & Lead):** Kalman Filtresi hız kestirimi ve 2-DOF hız ileri beslemesi devreye girer; HUD açık mavi renge geçer.
* **DURUM 4 (Hibrit / P Kontrol):** Tip-1 P kontrolcü (hız ileri beslemeli) ve 11-boyutlu PPO takviyesi ile açısal takip hatası minimuma indirilir.
* **DURUM 5 (Hedefe Kilitlendi):** Hata toleransı sağlandığında ve kararlılık sayacı dolduğunda hedef kilitlenir (`TARGET LOCKED`), koyu amber operatör paneli açılır, `[Y]` atış yetkisi beklenir.
* **DURUM 6 (Ateşleme):** Operatör onayıyla 120 ms yükselen kenar kilidiyle solenoid dartı fırlatır; ekranda 3 saniyelik balistik geri bildirim gösterilir.

---

## 🛡️ Güvenlik ve Fail-Safe (Arıza Korumalı) Mimarisi

Sistem, fiziksel test güvenliği ve donanım koruması için çok katmanlı güvenlik önlemleri içerir:

1. **300 ms Donanımsal Heartbeat Watchdog:** PC ile STM32 arasındaki seri iletişim veya yazılım döngüsü 300 ms boyunca kesilirse, STM32 fırlatıcı volan motorlarını ve solenoid darbesini anında otomatik keser (`Launcher_EmergencyStop()`).
2. **Mandallı (Latched) Acil Durdurma (E-STOP):** Operatör `SPACE` tuşuna bastığında sistem mandallı durdurma moduna geçer. Kamerada hedef olsa dahi motorlara komut verilmez, servolar doğrudan mekanik merkez (90°, 90°) pozisyonuna kilitlenir. Tekrar `SPACE` tuşuna basılana kadar sistem uyanmaz.
3. **Kamera Donma / Kablo Kopma Koruması:** Web kamerasından 0.35 saniye boyunca yeni kare gelmezse, hayali bir konuma kilitlenmeyi önlemek için kamera iş parçacığı ana döngüyü güvenli beklemeye geçirir.
4. **Çift Atış Önleyici Yükselen Kenar Kilidi:** Solenoid darbesi 120 ms ile sınırlandırılmıştır ve tek onayda yalnızca tek bir dart atışı yapacak şekilde yükselen kenar tetiklemesi ile kilitlenmiştir.
5. **Açılış ve Hata Korumalı Pin Durumları:** STM32 ilk enerjilendiğinde ve `Error_Handler` durumunda PB0/PB1 tetik pinleri donanımsal PULL-DOWN ile kesin olarak 0V (LOW) seviyesinde tutulur.

---

## 📊 Karşılaştırmalı Performans Analizi (P Kontrolcü vs PPO RL)

Sistem, gerçek donanımın **80 ms taşıyıcı gecikmesi** altında 3 farklı hedef manevra senaryosunda (Doğrusal, Zigzag ve Kaçınma Manevrası) karşılaştırılmıştır:

![P Controller vs PPO RL Benchmark Karşılaştırması](turret_fcs/rl_sim/p_vs_rl_comparison.png)

| Test Senaryosu | Klasik P Kontrolcü Ortalama Hata | 11-D PPO Takviyeli Öğrenme (RL) Hata | Hata İyileştirme Oranı |
| :--- | :---: | :---: | :---: |
| **Senaryo 1 (Doğrusal Uçuş)** | 54.21 piksel | 28.45 piksel | **+%47.5** |
| **Senaryo 2 (Zigzag Manevrası)** | 62.10 piksel | 34.12 piksel | **+%45.1** |
| **Senaryo 3 (Kaçınma Manevrası)** | 68.64 piksel | 37.81 piksel | **+%44.9** |
| **Genel Benchmark Ortalaması** | **61.65 piksel** | **33.46 piksel** | **+%45.7 Doğruluk Artışı** |

---

## 🚀 Hızlı Başlangıç ve Kurulum

### 1. Donanım ve Yazılım Gereksinimleri

* Windows 10/11 x64
* Python 3.10 veya 3.12 (*"Add Python to PATH"* seçili olmalıdır)
* STM32 Nucleo-F401RE Kartı & Logitech C270 Web Kamerası
* Benewake TF-Luna LiDAR Sensörü

### 2. Otomatik Ortam Kurulumu

Depoyu klonlayın ve ana dizindeki kurulum betiğini çalıştırın:

```bash
git clone https://github.com/brkysnturk/turret_fcs.git
cd turret_fcs
setup_env.bat
```

Bu betik sanal ortamı (`venv`) oluşturur, `pip`'i günceller ve [requirements.txt](requirements.txt) içerisindeki tüm bağımlılıkları eksiksiz yükler.

### 3. Sistemi Tek Tıkla Başlatma

Donanım kablolarını taktıktan sonra ana dizindeki başlatıcıyı çalıştırın:

```bash
main.bat
```

### 🎮 Operatör Kontrol Tuşları

* **`ESC` / `Q` :** Programı ve telemetri kaydını güvenli kapatma (`logs/telemetry_test_*.csv` diske kaydedilir).
* **`SPACE` (Boşluk) :** Mandallı Acil Durdurma (E-STOP) — Motorları 90° merkeze çeker ve mandallar (Tekrar basınca takibe döner).
* **`M` :** Kontrol Modu Döngüsü (`[P CONTROLLER]` <-> `[HYBRID CONTROLLER (RL:X%)]` <-> `[RL CONTROLLER]`).
* **`P` :** Lissajous parametrik devriye taramasını açma / kapatma.
* **`Y` :** Manuel Atış Yetkisi (Authorize Fire).
* **`N` :** Hedef Bırakma / Kilit İptali.

---

## 📈 Akademik Dinamik Analiz ve Sistem Tanıma

Taret kontrol dinamiklerini, FOPDT sistem tanıma sonuçlarını ve Padé kök-yer eğrilerini incelemek için `analysis/` araçları kullanılır:

```bash
# 1. 6 Panelli Detaylı Dinamik ve Kontrol Kararlılık Analizi (300 DPI)
python analysis/plot_tuned_system.py

# 2. 4 Panelli Sistem Tanıma, S-Düzlemi Root Locus ve Doğrulama
python analysis/system_identification.py

# 3. P Kontrolcü vs PPO RL Karşılaştırma Grafiği
python turret_fcs/rl_sim/compare_pid_vs_rl.py
```

*(Akademik tez veya jüri sunumu için MATLAB ortamında `analysis/turret_fcs_analysis.m` dosyasını çalıştırabilirsiniz).*

---

## 📂 Dizin Yapısı

```text
turret_fcs/
├── requirements.txt           # Python kütüphane bağımlılıkları (YOLO, Torch, SB3, Optuna, vb.)
├── setup_env.bat              # Otomatik sanal ortam (venv) ve kütüphane kurulum aracı
├── main.bat                   # Tek tıkla gerçek zamanlı atış kontrol başlatıcısı
├── README.md                  # Proje mimarisi ve teknik dokümantasyon
├── turret_fcs_architecture.png# 4K Turret FCS ana mimari şeması (v1.1)
├── stm32_architecture.png     # 4K STM32 gömülü sürücü ve kesme mimarisi (v1.1)
├── webots_architecture.png    # 4K Webots dijital ikiz simülasyon mimarisi (v1.1)
├── turret_fcs_flowchart.svg   # Vektörel algoritmik durum akış şeması (Flowchart)
├── turret_fcs_hardware.svg    # Vektörel elektriksel donanım ve güç dağıtım şeması
├── analysis/                  # Akademik kontrol ve sistem tanıma araçları
│   ├── plot_tuned_system.py   # Python 6 panelli dinamik analiz motoru
│   ├── system_identification.py # FOPDT basamak eğri uyumu ve Padé kök-yer eğrisi
│   ├── turret_fcs_analysis.m  # MATLAB akademik analiz ikizi
│   └── figures/               # Yüksek çözünürlüklü (300 DPI) yayın grafikleri
├── logs/                      # Gerçek donanım telemetri test kayıtları (.csv)
├── turret_fcs/                # Gerçek donanım çalışma kodları
│   ├── main.py                # Ana atış kontrol döngüsü ve hedef koordinasyonu
│   ├── telemetry.py           # Kare-kare donanım telemetri kaydedicisi
│   ├── fcs_manager.py         # Hedef seçim, kilit, balistik ve menzil kapılama motoru
│   ├── fcs_comm.py            # Ters kayan pencereli STM32 UART binary paket haberleşme sürücüsü
│   ├── kalman_filter.py       # Kalman filtreli 3D yörünge ve hız kestirimi (2-DOF Feedforward)
│   ├── turret_hud.py          # Askeri standartta dinamik taktiksel HUD arayüzü
│   ├── rl_controller.py       # 11-boyutlu eylem geçmişli PPO RL çıkarım motoru
│   ├── p_controller.py        # 2-DOF hız ileri beslemeli ve yumuşak ölü bölgeli Tip-1 P kontrolcü
│   ├── patrol_generator.py    # Lissajous eğrisi parametrik devriye motoru
│   ├── target_selector.py     # Öncelikli hedef filtreleme algoritması
│   ├── camera_thread.py       # Sıfır gecikmeli, watchdog korumalı kamera sürücüsü
│   └── rl_sim/                # Pekiştirmeli öğrenme ortamı ve eğitim araçları
│       ├── turret_env.py      # 11-D Gymnasium taret ortamı (Transport Delay & Action History)
│       ├── optimize_rl.py     # Optuna TPE Bayesyen Hiperparametre Optimizasyonu
│       ├── train_rl.py        # PPO model eğitim kodu (Stable-Baselines3)
│       ├── compare_pid_vs_rl.py # P vs RL akademik benchmark karşılaştırması
│       └── models/            # Eğitilmiş üretim PPO modeli (turret_ppo_final.zip)
├── stm32/                     # STM32F401RE Nucleo gömülü yazılım projesi (CubeIDE)
│   └── turret_main/           # Ana taret donanım projesi (.ioc, Core/Src, Core/Inc)
│       └── Core/Src/          # fcs_comm.c, servo_driver.c, luna_read.c, flywheel_solenoid.c, main.c
├── webots/                    # Webots dijital ikiz simülasyonu
│   ├── controllers/           # Simülasyon kontrolcüleri (webots_main, webots_turret_hud, vb.)
│   ├── worlds/                # 3D simülasyon dünyası ve aktörler
│   └── analysis/              # Webots performans analiz motorları ve telemetri verileri
└── weights/                   # Eğitilmiş yapay zeka model ağırlıkları (best.pt)
```

---

## 📜 Lisans & Akademik Referans

Bu proje Yıldız Teknik Üniversitesi Kontrol ve Otomasyon Mühendisliği Bölümü Bitirme Çalışması & Tasarım Projesi kapsamında geliştirilmiştir. Tüm hakları proje ekibine aittir.
