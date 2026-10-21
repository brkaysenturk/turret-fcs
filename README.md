# Otonom Asimetrik Hava Tehdidi Tespit ve Atış Kontrol Sistemi (Turret FCS)

## Yıldız Teknik Üniversitesi — Kontrol ve Otomasyon Mühendisliği Bölümü

### Bitirme Çalışması & Tasarım Projesi (v1.3 Production Release)

[![Release](https://img.shields.io/badge/Release-v1.3-success.svg)](https://github.com/brkysnturk/turret_fcs/releases/tag/v1.3)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![CI](https://github.com/brkysnturk/turret_fcs/actions/workflows/ci.yml/badge.svg)](https://github.com/brkysnturk/turret_fcs/actions)
[![STM32 Nucleo](https://img.shields.io/badge/Hardware-STM32F401RE-002B49.svg)](https://www.st.com/)
[![Webots Simulator](https://img.shields.io/badge/Simulation-Webots%20R2025a-red.svg)](https://cyberbotics.com/)
[![YOLOv8](https://img.shields.io/badge/YOLO-v8n-00FFFF.svg)](https://ultralytics.com/)
[![Reinforcement Learning](https://img.shields.io/badge/RL-LQR--Reward%20PPO%20(SB3)-orange.svg)](https://stable-baselines3.readthedocs.io/)
[![Control Theory](https://img.shields.io/badge/Control-Soft--Deadband%20Type--1%20P-brightgreen.svg)](https://en.wikipedia.org/wiki/Proportional_control)

Bu proje; mikro/mini İnsansız Hava Araçları (İHA) ve asimetrik hava tehditlerine karşı geliştirilmiş; **Logitech C270 RGB Kamera (720p @ 30 FPS) + Benewake TF-Luna ToF LiDAR (100 Hz UART) Sensör Füzyonu**, **YOLOv8n Derin Öğrenme Hedef Tespiti**, **ByteTrack Çoklu Hedef Takibi**, **Ego-Hareket Telafili CWNA Adaptif Kalman Filtresi**, **Yumuşak Ölü Bölgeli (Soft Deadband) Tip-1 P Kontrolcü**, **Kapalı Form 2B Gauss Vuruş Olasılığı (Marcum Q / ncx2 CDF)**, **Menzil Kapılama (Range Gating)** ve **11-Boyutlu Eylem Geçmişli PPO Derin Pekiştirmeli Öğrenme (RL)** tabanlı tam otonom bir Elektro-Optik Atış Kontrol Sistemidir (FCS).

---

## 🚀 v1.3 Sürüm Detayları ve Temel Mühendislik İnovasyonları

`v1.3` sürümü; kontrol teorisi, balistik hesaplama, stokastik vuruş olasılığı, taktiksel arayüz (HUD) ve yazılım mimarisi açısından sistemi endüstriyel ve akademik olgunluğa ulaştıran kapsamlı bir üretim sürümüdür:

### 1. Tam 2B Gauss Vuruş Olasılığı Modeli (`turret_fcs/lock_manager.py`)
* **Kapalı Form Marcum Q / Non-Central Chi-Squared Entegrasyonu:**
  * Eski sezgisel ofset zayıflatma yaklaşımı yerine, dairesel hedef alanı üzerinde eksantrik 2B izotropik Gauss integrali kapalı formda `scipy.stats.ncx2.cdf(1, df=2, nc=(r_ofset / sigma)^2)` formülasyonuyla modellendi.
  * Hata sıfırken Rayleigh dağılımına `p_sifir = 1 - exp(-theta_hedef^2 / (2 * sigma_toplam^2))` tam olarak indirgenir; hata büyüdükçe eksantrik Gauss integrali fiziksel saçılımı kesin doğrulukla hesaplar.
* **Dinamik Hedef Açısal Boyutu:**
  * Sabit yarıçap varsayımı (0.15 m) kaldırıldı. YOLO tespit sınırlayıcı kutusundan (bounding box) piksel boyutları (`bw, bh`) ve kamera odak uzaklığı (`fx, fy`) kullanılarak anlık hedef açısal yarıçapı `theta_hedef = 0.5 * (atan(bw / (2 * fx)) + atan(bh / (2 * fy)))` olarak dinamik hesaplanır.
* **Gecikme ve Uçuş Süresi Ayrıştırması:**
  * 80 ms donanım gecikmesi uçuş süresinden ayrıştırıldı; toplam hedef saçılımı `sigma_hareket = 0.25 * omega * (t_ucus + 0.08 saniye)` ve silah dağılımı `sigma_silah = 1.25 derece` üzerinden toplam varyans `sigma_toplam = sqrt(sigma_silah^2 + sigma_hareket^2)` olarak modellenir.
* **Dinamik Histerezis Kilit Penceresi:**
  * Sabit 3.0° tolerans yerine hedef hızına duyarlı dinamik giriş eşiği (`<15 px/s: 1.60°`, `15-45 px/s: 2.20°`, `>=45 px/s: 3.00°`) ve histerezis çıkış eşiği (`lock_exit = lock_enter + 0.60°` -> 2.20° - 3.60°) ile 10 karelik kararlılık sayacı entegre edildi. Vuruş olasılığı ise 2B Gauss (ncx2) modeliyle anlık hesaplanır.

### 2. Adaptif Kalman Filtresi Kalibrasyonu ve Hız Doğruluğu (`turret_fcs/kalman_filter.py`)
* **Sürekli Beyaz Gürültü İvme Modeli (CWNA):**
  * Durum vektörü `X = [x, y, vx, vy]^T` olarak modellenmiş olup, yenilik (innovation) büyüklüğüne göre ivme gürültü varyansı `sigma_a = 4.0 px/s^2` (sabit hız) ile `280.0 px/s^2` (ani manevra) arasında dinamik ayarlanır.
  * Kovaryans güncellemesinde sayısal simetri ve pozitif yarı-tanımlılığı garanti eden Joseph formülasyonu kullanılır.
* **Düşük Hız Kalibrasyonu (Low-Speed Velocity Pass):**
  * Ego-hareket tabanlı hız ölü bölgesi genişletmesi kaldırıldı; kalibre edilmiş statik hız ölü bölgesi (6.0 - 14.0 px/s) doğrudan çıkış katmanına uygulandı. Böylece 1.0 derece/s (24.6 px/s) gibi yavaş hedef manevraları %100 doğrulukla filtreden geçer ve önleme açısı (lead) sıfıra çökmez.
* **Gelişmiş Açısal Dönüşüm:**
  * Piksel hızından açısal hıza geçişte doğrusal küçük açı yaklaşımı yerine `omega = degrees(atan2(v_px, f))` trigonometrik bağıntısı kullanılarak optik eksen kenarlarındaki distorsiyon önlendi.
* **Genişletilmiş Sıçrama Eşiği:**
  * Sıçrama (jump) eşiği 90 px'den 240 px (~9.8 derece) seviyesine çıkarılarak 1.0 s (1000 ms) volan hızlanması sırasındaki balistik önleme hareketlerinde filtrenin yanlışlıkla sıfırlanması engellendi.

### 3. Tip-1 Oransal (P) Kontrol Teorisi Omurgası (`turret_fcs/p_controller.py`)
* **Açık Çevrim Donanımsal İntegratör (Pole at z=1):**
  * STM32 gömülü mimarisi, gelen açısal hız/adım komutlarını her 20 ms'de (`50 Hz PWM`) biriktirerek donanımsal konum üretir: `theta(k) = theta(k-1) + delta_theta(k)`.
  * Bu yapı sistem açık çevrim transfer fonksiyonuna z=1 kutbu (saf integratör) ekler. Dolayısıyla kontrol döngüsü doğal olarak Tip-1 sistemdir.
  * Statik hedefler için yazılımsal Ki integratörüne ihtiyaç duyulmadan kalıcı durum hatası teorik ve pratik olarak sıfırdır (`e_ss = 0`). İntegratör sarılması (windup) ve limit çevrim salınımları tamamen elimine edilmiştir.
* **Rampa Hata Telafisi:**
  * Sabit hızlı manevra yapan hedeflerde oluşan rampa takip hatası `e_ss = omega / Kv`, Kalman filtresinden gelen hız vektörüyle üretilen ileri beslemeli dinamik önleme açısı (lead angle) ile kapatılır.
* **Eksenel Parametreler ve Yumuşak Ölü Bölge (Soft Deadband):**
  * Pan ekseni: `Kp_pan = 0.10` (30 FPS'de `Kp_s = 3.0 1/s`), `deadband_pan = 0.85 derece`.
  * Tilt ekseni: `Kp_tilt = 0.05` (30 FPS'de `Kp_s = 1.5 1/s`), `deadband_tilt = 0.95 derece`.
  * Yumuşak ölü bölge formülasyonu sayesinde ölü bölge sınırında eylem sıfırdan pürüzsüz başlar, mekanik dişli vuruntusu (chattering) engellenir.

### 4. 11-Boyutlu Eylem Geçmişli PPO Pekiştirmeli Öğrenme Ajanı (`turret_fcs/rl_controller.py`)
* **80 ms Donanım Gecikme Telafisi:**
  * Fiziksel servo yanıtı ve seri iletişimden kaynaklanan 80 ms (yaklaşık 2-3 görüntü karesi) taşıyıcı gecikmeyi Markovian karar sürecine dönüştürmek için 2 adımlı eylem geçmişi gözlem vektörüne dahil edildi.
  * 11-Boyutlu Durum Uzayı: `[error_x, error_y, d_err_x, d_err_y, dist_norm, vx_raw, vy_raw, act_hist_1 (pan, tilt), act_hist_2 (pan, tilt)]`.
* **Bryson-Normalize Kuadratik LQR Ödül Fonksiyonu:**
  * Çelişkili ceza terimleri yerine durum, eylem ve jerk büyüklüklerini regüle eden saf kuadratik LQR ödülü:
    `r = - (e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2) + bullseye_bonus`
* **Optuna TPE Bayesyen Optimizasyonu ve 250k Eğitim:**
  * 20 sabit tohum üzerinde Medyan + IQR metrikleriyle optimize edilen `turret_ppo_final.zip` modeli; P kontrolcünün rampa gecikmesine uğradığı 8-10 derece/s hedef hızlarında %100 kilitlenme sadakati sunar.
* **Operatör Denetimli Bağımsız Seçim:**
  * Kararsız geçiş dinamiklerine yol açan otomatik hibrit anahtarlama yerine, operatörün `[M]` tuşu ile duruma göre seçebildiği saf `[P CONTROLLER]` (hassas ve sakin merkezleme) ve `[RL CONTROLLER]` (yüksek hızlı çevik takip) mimarisi uygulanır.

### 5. İki Aşamalı Balistik Atış Protokolü, Slew İleri Beslemesi ve Donanım Senkronizasyonu
* **Aşama 1 (1.0 s Volan Hızlanma - SPINUP & Sıfır Gecikmeli İleri Besleme):**
  * Operatör atış yetkisi verdiğinde (`[Y]`), çift DC volan motoru 23.500 RPM hızına çıkarılır. Bu sırada taret Kalman önleme noktasına (lead angle) yönelir.
  * **Zero-Lag Aim-Point Feedforward:** Rampanın kalıcı durum takip hatasını (`e_ss = omega / K`) yok etmek için doğrudan türevsel önleme adımı eklenir: `u = delta_slew + u_P` (`delta_slew = applied_lead - prev_applied_lead`). Böylece taret 1.0 saniye içerisinde tam süper-elevasyon açısına gecikmesiz ulaşır. Hedef takibi ve Kalman filtreleme arka planda kesintisiz devam eder.
* **Aşama 2 (80 ms Solenoid Darbesi - FIRING):**
  * Önleme açısına ulaşıldığında STM32 donanımsal zamanlayıcısı 80 ms yükselen kenar darbeli solenoid fırlatıcıyı tetikler ve tek bir dart fırlatılır.
* **Aşama 3 (350 ms Merkeze Geri Dönüş - RECOVER):**
  * Solenoid geri çekilirken taret hedef merkezine (boresight) geri döner.
* **Aşama 4 (1.5 s Atış Değerlendirme ve Kilit Muafiyeti - EVAL & GRACE):**
  * EVAL 1.5 s ekranda gösterilir; atış sonrası kilit muafiyeti 0.75 s (22 kare @ 30 FPS) olup EVAL'in ilk yarısında çalışır. Taret optik ekseni hedefe yeniden oturana kadar yapay kilit kaybı bayrağı üretilmez.

### 6. Taktiksel MIL-STD Operatör HUD ve Muharebe Sadeleştirmesi (Combat Declutter)
* **Sorumlulukların Ayrılığı (SRP) Kapsüllemesi (`turret_fcs/turret_hud.py`):**
  * HUD çizim mantığı `main.py` ve `webots_main.py` döngülerinden arındırılarak `TurretHUD` sınıfı içerisindeki modüler metodlara (`draw_mode_banner`, `draw_footer`, `draw_event_log`) taşındı.
* **Muharebe Sadeleştirmesi (Combat Decluttering):**
  * Atış izni istendiğinde (`AUTHORIZE FIRE? [Y/N]`) ve atış protokolü boyunca ekran karmaşasını önlemek ve operatörün hedefe odaklanmasını sağlamak amacıyla:
    * Sol üst Telemetri/Durum paneli gizlenir.
    * Üst orta mod afişi gizlenir.
    * Sağ üst FPS göstergesi gizlenir.
    * Sol alt taktiksel olay günlüğü (event log) tamamen kaldırılır.
    * Alt kontrol lejantı kilitte sadeleşir, atışta tamamen gizlenir.
* **Taktiksel Askeri Terminoloji:**
  * Atış değerlendirme afişi uluslararası askeri standartlara uygun şekilde İngilizce olarak sunulur: `ENGAGEMENT COMPLETE / ENGAGEMENT SUCCESSFUL`.

### 7. Telemetri v2 ve Taktiksel Metadata Sidecar Mimarisi (`turret_fcs/telemetry.py`)
* **40 Kolonluk Genişletilmiş CSV Şeması:**
  * `schema_version`, `frame_id`, `latency_ms`, `vx_raw`, `vy_raw`, `innov_px`, `sigma_a`, `kp_eff_pan/tilt`, `db_eff_pan/tilt`, `servo_pan_fb/tilt_fb`, `sensor_temp_c`, `lock_thr` alanlarını içeren yüksek çözünürlüklü deterministik veri kaydı.
* **JSON Metadata Sidecar (`.meta.json`):**
  * Her oturum açılışında sistem konfigürasyonu, donanım kimliği ve Git SHA kodunu; kapanışta ise toplam kare sayısı, çalışma süresi, kilit oranı ve medyan hata istatistiklerini otomatik özetler.
* **Milisaniye Hassasiyetli Olay Kütüğü (`_events.csv`):**
  * Taktiksel durum geçişleri, operatör yetkilendirmeleri ve atış sekansları olay bazlı olarak ayrı bir dosyada zaman damgasıyla tutulur.

### 8. SSOT (Single Source of Truth) ve Otomatik Test Paketi
* **Merkezi Yapılandırma (`turret_fcs/config.py`):**
  * Optik parametreler, kontrolcü kazançları, aktüatör limitleri (Pan 0.55°/adım, Tilt 0.22°/adım) ve balistik katsayılar tek bir saf matematik leaf modülünde (`CONFIG`) konsolide edildi. Model dosya yolları `CONFIG.paths` üzerinden tekilleştirildi.
* **Otomatik Test Paketi (`tests/`):**
  * 19 otomatik Pytest birim ve tutarlılık testi ile CI boru hattı üzerinden sürekli doğrulanır.
* **Standart Karşılaştırma Betiği:**
  * P ve RL karşılaştırmaları üretim `PController` ve `RLController` sınıfları üzerinden standart `turret_fcs/rl_sim/compare_p_vs_rl.py` betiğiyle 300 DPI yayın kalitesinde (`p_vs_rl_final_benchmark.png`) üretilir.

---

## 🚀 Önceki Sürüm İnovasyonları

### v1.2
* **Otonom Devriye Birikimli Adımlayıcısı:** 80-100 FPS yüksek döngü hızında MG996R mekanik ölü bölgesine takılmayı önleyen birikimli adım mekanizması (`min_dispatch_deg = 0.95 derece`) ile 90-90 merkezine eksiksiz dönüş.
* **Takip Histerezisi ve Anlık Güven (% CONF):** Arama modunda `conf = 0.28`, kilitli takipte hareket bulanıklığına karşı `conf = 0.16` dinamik güven eşiği.
* **Asimetrik Eylemsizlik Sönümlemesi:** Dikey eksendeki 300g pirinç volan namlu ağırlığının yarattığı yerçekimi momenti ve dişli boşluğu için asimetrik yumuşak ölü bölge (Pan 0.85°, Tilt 0.95°) ve adaptif sönümleme.
* **Smith Predictor Eylem Belleği:** Fiziksel testte %50.2 kilitlenme oranı, 13.16 saniye (1.061 kare @ 80.6 FPS) kesintisiz kilitlenme başarımı.
* **Hedef Kayıp (Dropout) Toleransı:** 4 karelik (`max_lost_frames`) dinamik tolerans penceresi ve aktüatör durum gözlemcisine `was_lost` bayrağı.

### v1.1
* **Menzil Kapılama (Range Gating):** Hedef namlu merkezinden uzaktayken (`angular_error > 3.0°`) TF-Luna LiDAR ışınının arkadaki duvara çarpmasını önleyen mesafe dondurma mekanizması (`last_valid_target_dist_m`).
* **Ters Kayan Pencere (Sliding Window) UART Ayrıştırıcısı:** 115200 Baud ikili veri hattında faz kayması ve gecikmeyi önleyen sağlam ayrıştırıcı.
* **300 ms STM32 Heartbeat Watchdog:** İletişim koptuğunda fırlatıcı motorlar ve solenoid için acil donanımsal frenleme (`Launcher_EmergencyStop()`).
* **Lissajous Parametrik Devriye Taraması:** Hedef yokken hava sahasını tarayan sonsuzluk yörüngesi (Pan: +/-35°, Tilt: +/-8°).

---

## 🎯 Temel Yetenekler ve Donanım Parametreleri

| Alt Sistem / Bileşen | Donanım & Yazılım Özelliği | Çalışma Parametresi / Başarım |
| :--- | :--- | :--- |
| **Görsel Algılama** | Logitech C270 HD Web Kamerası | 1280x720 @ 30 FPS, dFOV: ~56.6° (Optik Standart: 55°, Yatay 48.8°, Dikey 28.6°) |
| **Menzil Algılama** | Benewake TF-Luna ToF LiDAR | 850 nm VCSEL, 0.2 - 8.0 m menzil, 100 Hz UART |
| **Yapay Zeka Çıkarımı** | Ultralytics YOLOv8n + ByteTrack | Mini İHA / Balon tespiti, 8-15 ms çıkarım (GPU) |
| **Filtreleme & Füzyon** | Ego-Hareket Telafili CWNA Kalman | Joseph form kovaryans, adaptif Q, 100 Hz tahmin |
| **Vuruş Olasılığı** | Kapalı Form 2B Gauss (Marcum Q) | ncx2 CDF, dinamik hedef boyutu, histerezis kilit |
| **Birincil Kontrolcü** | Tip-1 Yumuşak Ölü Bölgeli P | Pan Kp=0.10 (3.0 1/s), Tilt Kp=0.05 (1.5 1/s) |
| **İkincil Kontrolcü** | 11-Boyutlu PPO Pekiştirmeli Öğrenme | 80 ms gecikme tamponu, LQR ödül fonksiyonu |
| **Mikrodenetleyici** | STM32F401RE Nucleo (ARM Cortex-M4) | 84 MHz, 100 Hz ikili paket UART, 300 ms Watchdog |
| **Aktüatörler** | Çift MG996R Metal Dişli Servo | 50 Hz PWM, Pan: +/-45°, Tilt: -20° / +30° |
| **Fırlatma Mekanizması** | Çift Fırçasız DC Volan + Solenoid | 23.500 RPM, 80 ms donanımsal solenoid darbesi |
| **Dijital İkiz** | Cyberbotics Webots R2025a | ODE fizik motoru, tam kinematik/dinamik model |

---

## 🏛️ Sistem ve Donanım Mimarisi

Aşağıdaki mimari şemalar sistemin PC yazılım boru hattını, STM32 gömülü mimarisini, Webots simülasyon motorunu ve elektriksel güç dağıtım katmanını göstermektedir:

### 1. Turret FCS Yazılım Pipeline Şeması
![Turret FCS Mimari Şeması](docs/assets/turret_fcs_architecture.png)

### 2. STM32F401RE Gömülü Çekirdek & Sürücü Mimarisi
![STM32 Gömülü Mimari](docs/assets/stm32_architecture.png)

### 3. Webots Dijital İkiz Simülasyon Mimarisi
![Webots Simülasyon Mimarisi](docs/assets/webots_architecture.png)

### 4. Donanım ve Güç Dağıtım Şeması
Projenin elektriksel güç dağıtımı, sensör entegrasyonu ve aktüatör bağlantıları [turret_fcs_hardware.svg](docs/assets/turret_fcs_hardware.svg) dosyasında detaylandırılmıştır.

### 5. 3D Mekanik Montaj ve Donanım Tasarımı
Taretin 3D yazıcıdan üretilen açık beşikli, minimal optik paralakslı ve gizli servo montajlı elektromekanik tasarımı:

![Turret FCS Elektromekanik Donanım Yerleşimi](docs/assets/mechanical_design.jpg)

![Turret FCS 3D Mekanik Montaj](docs/assets/turret_cad_render_annotated.jpg)

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
|  * ByteTrack Çoklu Hedef Takibi & Anlık Güven Histerezisi                         |
|  * CWNA Adaptif Kalman Filtresi (Joseph Form, Düşük Hız Kalibrasyonu)             |
|  * Menzil Kapılama (Range Gating) Duvar Parazit Filtreleme                        |
|  * Taktiksel MIL-STD HUD Arayüzü & Muharebe Sadeleştirmesi (Combat Declutter)    |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                           KONTROL VE NİŞAN ALMA KATMANI                           |
|  * Kapalı Form 2B Gauss Vuruş Olasılığı (Marcum Q / ncx2 CDF)                     |
|  * Dinamik Önleme Açısı (Lead Pursuit) & Balistik Süper-Elevasyon Hesabı          |
|  * Bağımsız Modlar: Yumuşak Ölü Bölgeli Tip-1 P / 11-Boyutlu LQR-PPO RL           |
|  * 80 ms Taşıyıcı Gecikme Telafisi & Eylem Geçmişi Hafızası                       |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                        GÖMÜLÜ DONANIM / FİZİKSEL EYLEME                           |
|  * STM32F401RE Nucleo Mikrodenetleyici (100 Hz İkili UART Protokolü)              |
|  * 300 ms Donanımsal Heartbeat Watchdog (Otomatik Güvenli Kesme)                  |
|  * Çift Eksen Pan-Tilt MG996R Servoları (50 Hz PWM Donanımsal İntegratör)         |
|  * 2 Aşamalı Atış Sekansı: 1.0 s Volan (23.500 RPM) + 80 ms Solenoid Darbesi       |
+-----------------------------------------------------------------------------------+
```

---

## 🔄 Algoritmik Durum Akış Şeması (Flowchart)

Sistemin arama, tespit, takip, kilitlenme ve iki aşamalı atış sekansları [turret_fcs_flowchart.svg](docs/assets/turret_fcs_flowchart.svg) dosyasında modellenmiştir:

* **DURUM 1 (Devriye / Arama - PATROL):** Hedef yokken Lissajous parametrik tarama eğrileriyle (Pan: +/-35°, Tilt: +/-8°) hava sahası taranır; HUD yeşil renktedir.
* **DURUM 2 (Tespit & Mesafe - DETECT):** Hedef algılandığında TF-Luna LiDAR ile 100 Hz mesafe ve sinyal gücü doğrulanır; Menzil Kapılama ile duvar okumaları filtrelenir; hedef kimliği kilitlenir.
* **DURUM 3 (Takip & Lead - TRACK):** Kalman Filtresi hedef konum ve hızını kestirir, dinamik önleme noktası (Lead Reticle) hesaplanır; HUD açık mavi renge geçer.
* **DURUM 4 (P / RL Kontrolü):** Yumuşak Ölü Bölgeli Tip-1 P kontrolcü (hassas merkezleme) veya 11-boyutlu PPO RL (çevik takip) ile açısal takip hatası minimize edilir.
* **DURUM 5 (Hedefe Kilitlenme - LOCKED):** Açısal hata histerezis eşiğinin altına indiğinde ve 10 karelik kararlılık sayacı dolduğunda hedef kilitlenir (`TARGET LOCKED`), muharebe sadeleştirmesi devreye girer, operatörden `[Y]` atış yetkisi istenir.
* **DURUM 6 (İki Aşamalı Atış Protokolü):**
  * `SPINUP`: 1.0 s (1000 ms) volan motoru hızlanması ve kosinüs önleme manevrası (HUD koyu kırmızı).
  * `FIRING`: 80 ms kenar tetiklemeli solenoid darbesiyle dart fırlatma.
  * `RECOVER`: 350 ms namlu eksenine yumuşak geri dönüş.
  * `EVAL`: 1.5 saniye ekranda `ENGAGEMENT COMPLETE / ENGAGEMENT SUCCESSFUL` taktiksel bildirim afişi gösterilir (atış sonrası kilit muafiyeti 0.75 s / 22 kare olup EVAL'in ilk yarısında çalışır).

---

## 📐 Kontrol Teorisi ve Matematiksel Modelleme

### 1. Tip-1 Oransal (P) Kontrol ve Donanımsal Ayrık İntegratör
STM32 servo sürücüsü her döngüde gelen açısal artım komutunu mevcut açıya ekler:

`theta(k) = theta(k-1) + delta_theta(k)`

Z-düzleminde transfer fonksiyonu:

`H(z) = theta(z) / delta_theta(z) = 1 / (1 - z^(-1)) = z / (z - 1)`

Sistem z = 1 noktasında kutba sahip açık çevrim bir integratördür. Kapalı çevrimde oransal denetleyici:

`delta_theta(k) = Kp * e(k)`

Basamak hedef girdisi `R(z) = 1 / (1 - z^(-1))` uygulandığında Son Değer Teoremi uyarınca kalıcı durum hatası:

`e_ss = lim_{z -> 1} (1 - z^(-1)) * E(z) = 0`

Sistem doğası gereği Tip-1 olduğundan harici yazılımsal Ki integratörüne ihtiyaç duyulmaz. Rampa tipi hareketli hedef girdisinde oluşan sonlu gecikme hatası `e_ss = omega / Kv` ise Kalman filtreli hız ileri beslemesi ve atış sırasındaki doğrudan balistik slew ileri beslemesi (`Aim-Point Feedforward: u = delta_slew + u_P`) ile sıfır gecikmeyle tam zamanında telafi edilir.

### 2. Yumuşak Ölü Bölge (Soft Deadband) Formülasyonu
Mekanik dişli boşluğu ve aktüatör titreşimini önlemek için uygulanan parçalı sürekli fonksiyon:

`u = 0, e_abs <= db`

`u = sign(e) * Kp * (e_abs - db), e_abs > db`

Bu formülasyon sayesinde ölü bölgeden çıkış anında kontrol sinyali sıfırdan sürekli olarak başlar (sıfır jerk), mekanik vuruntu engellenir.

### 3. Kapalı Form 2B Gauss Vuruş Olasılığı (Marcum Q)
Hedef dairesi üzerinde dikey eksantriklik `r_ofset` ve silah-hareket varyansı `sigma_toplam` altında vuruş olasılığı non-central chi-squared kümülatif dağılım fonksiyonu (ncx2 CDF) ile hesaplanır:

`nc = (r_ofset / sigma_toplam)^2`

`p_vurus = ncx2.cdf(1, df=2, nc=nc) * (1 - exp(-theta_hedef^2 / (2 * sigma_toplam^2)))`

Burada:
* `theta_hedef`: Hedefin anlık açısal yarıçapı (YOLO kutusundan türetilen açısal boyut).
* `sigma_toplam = sqrt(sigma_silah^2 + sigma_hareket^2)`: Silah saçılımı (1.25°) ile hedef hareket belirsizliğinin geometrik toplamı.

### 4. PPO RL Kuadratik LQR Ödül Fonksiyonu
PPO ajanının eğitiminde simülasyon kararlılığını sağlayan normalize LQR ödülü:

`r = - (e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2) + r_bullseye`

Burada `lambda_u = 0.05` kontrol enerjisi regülasyonu, `mu_du = 0.02` eyleyici jerk regülasyonudur.

---

## 🛡️ Güvenlik ve Fail-Safe (Arıza Korumalı) Mimarisi

1. **300 ms Donanımsal Heartbeat Watchdog:** PC ile STM32 arasındaki seri iletişim 300 ms kesilirse, STM32 fırlatıcı motorları ve solenoid darbesini donanımsal olarak kapatır (`Launcher_EmergencyStop()`).
2. **Mandallı (Latched) Acil Durdurma (E-STOP):** Operatör `SPACE` tuşuna bastığında motorlar anında mekanik merkez (90°, 90°) konumuna kilitlenir. Tekrar `SPACE` tuşuna basılana kadar sistem hiçbir hedef komutunu işlemez.
3. **350 ms Kamera Donma Koruması:** Web kamerasından 350 ms boyunca kare gelmezse, sistem güvenli bekleme moduna geçer.
4. **Çift Atış Önleyici Yükselen Kenar Kilidi:** Solenoid darbesi 80 ms ile sınırlandırılmıştır ve tek yetkilendirmede birden fazla atış yapılması donanımsal mantıkla engellenmiştir.
5. **Açılışta Donanımsal PULL-DOWN Koruması:** STM32 ilk açıldığında ve hata durumlarında PB0/PB1 tetik hatları donanımsal dirençlerle kesin 0V seviyesinde tutulur.

---

## 📊 Karşılaştırmalı Performans Analizi (P Kontrolcü vs PPO RL)

Sistem hem gerçek donanım saha testlerinde hem de LQR simülasyon ortamında (`turret_fcs/rl_sim/compare_p_vs_rl.py`) kapsamlı olarak test edilmiştir:

![P Kontrolcü vs PPO RL Benchmark Karşılaştırması](analysis/figures/p_vs_rl_final_benchmark.png)

### 1. Gerçek Donanım Saha Testi (Fiziksel Taret, MG996R Servolar ve TF-Luna)

| Metrik / Performans Göstergesi | Klasik Saf Tip-1 P Kontrolcü | 11-Boyutlu PPO Pekiştirmeli Öğrenme | Akademik Üstünlük / Çıkarım |
| :--- | :---: | :---: | :---: |
| **Kilitlenme Oranı (Lock Ratio)** | %39.0 | **%50.2** | **+%11.2 Daha Yüksek Kilitlenme** |
| **Azami Kesintisiz Kilit Serisi** | 147 kare (4.9 saniye) | **1.061 kare (v1.2 donanım logunda 80.6 FPS ile 13.16 saniye; 30 FPS normalize tabanında 35.37 saniye)** | **Rekor Kilit Kararlılığı** |
| **Medyan Açısal Takip Hatası** | 2.22 derece | **1.56 derece** | **%29.7 Daha Yüksek Hassasiyet** |
| **Dikey Eksen Salınımı (Limit Cycle)** | %0.1 Yön Değişimi | **%0.0 Yön Değişimi** | **Mekanik Titreşimsiz Sıfır Jerk** |
| **Gerçek Zamanlı Çıkarım Süresi (CPU)**| **0.02 ms** | **0.17 ms** | **30 FPS'te %0.5, UART 100 Hz ayrı** |

### 2. Hedef Hızına Göre Dinamik Kapalı Çevrim Takip Başarımı (30 FPS, YOLO Gürültüsü)

| Hedef Hızı | P Takip Hatası | P Kilit Koruma | RL Takip Hatası | RL Kilit Koruma |
| :---: | :---: | :---: | :---: | :---: |
| **0.0 derece/s (Statik)** | 0.71 derece | %100 | 0.67 derece | %100 |
| **2.0 derece/s (Yavaş)** | 1.47 derece | %100 | 1.19 derece | %100 |
| **5.0 derece/s (Orta)** | 2.37 derece | %100 | 1.19 derece | %100 |
| **8.0 derece/s (Hızlı)** | 3.27 derece | %0 (Rampa Hatası) | **1.41 derece** | **%100 (Tam Kilit)** |
| **10.0 derece/s (Manevra)** | 3.87 derece | %0 (Kilit Koptu) | **1.85 derece** | **%100 (Tam Kilit)** |
| **12.0 derece/s (Sınır)** | 4.47 derece | %0 | 2.74 derece | %0 |

*Çıkarım:* Tip-1 P kontrolcü statik ve düşük hızlı hedeflerde açısal hatayı soft deadband bandı içine (Pan: ±0.85°, Tilt: ±0.95°) asimptotik olarak çekip sıfır motor titreşimi sunarken, 8-10 derece/s hızındaki çevik manevralarda PPO RL modeli rampa hatasını dinamik eylem geçmişiyle kapatıp kilidi %100 korumaktadır. Operatör `[M]` tuşu ile tehdidin dinamizmine göre iki mod arasında anlık geçiş yapabilir.

### 3. Çevrimdışı Sentetik Benzetim Doğrulaması (`compare_p_vs_rl.py`)
Üretim Tip-1 P kontrolcüsü (`compute_angles` + `compute_p`) ile 250k adımlık PPO modeli, 3 farklı manevra senaryosunda (Zikzak, Doğrusal Kaçış, Dinamik Evasive) birebir aynı tohum ve 80 ms taşıyıcı gecikmesi altında test edilmiştir:
* **P Kontrolcü Ortalama Takip Hatası:** 78.03 px
* **PPO RL Ortalama Takip Hatası:** 54.62 px
* **RL Başarım Kazancı:** %+30.0 daha düşük takip hatası

---

## 🚀 Hızlı Başlangıç ve Kurulum

### 1. Sistem Gereksinimleri
* Windows 10/11 x64
* Python 3.10 veya 3.12 (*"Add Python to PATH"* seçili olmalıdır)
* STM32 Nucleo-F401RE Kartı & Logitech C270 Web Kamerası
* Benewake TF-Luna ToF LiDAR Sensörü

### 2. Otomatik Ortam Kurulumu
Depoyu klonlayın ve ana dizindeki kurulum betiğini çalıştırın:

```bash
git clone https://github.com/brkysnturk/turret_fcs.git
cd turret_fcs
setup_env.bat
```

Bu betik Python sanal ortamını (`venv`) oluşturur, `pip`'i günceller ve [requirements.txt](requirements.txt) içerisindeki bağımlılıkları eksiksiz yükler.

### 3. Test Paketini Doğrulama
Kurulumun ve modül tutarlılığının doğrulanması için otomatik test paketini (19/19 test) çalıştırın:

```bash
pytest tests/ -v
```

### 4. Sistemi Başlatma
Donanım kablolarını taktıktan sonra başlatıcıyı çalıştırın:

```bash
main.bat
```

*(Webots simülatöründe çalıştırmak için `webots/` dizinindeki dünyayı açıp `webots_main.py` kontrolcüsünü başlatabilirsiniz).*

---

## 🎮 Operatör Kontrol Tuşları

| Tuş | Fonksiyon | Açıklama |
| :---: | :---: | :--- |
| **`ESC` / `Q`** | Güvenli Çıkış | Programı ve telemetri kaydını güvenli kapatır (`.csv`, `.meta.json`, `_events.csv`). |
| **`SPACE`** | Mandallı E-STOP | Motorları anında mekanik merkeze (90°, 90°) çeker ve kilitler. |
| **`M`** | Mod Değişimi | `[P CONTROLLER (Hassas Kilit)]` ile `[RL CONTROLLER (Çevik Takip)]` arasında geçiş yapar. |
| **`P`** | Devriye Taraması | Lissajous sonsuzluk eğrisi ile otonom hava sahası taramasını açar / kapatır. |
| **`Y`** | Atış Yetkisi | Hedef kilitlendiğinde (`TARGET LOCKED`) 2 aşamalı balistik atış sekansını onaylar. |
| **`N`** | Kilit İptali | Mevcut hedef kilidini iptal eder ve aramaya döner. |

---

## 📈 Akademik Analiz ve Simülasyon Araçları

Taret kontrol dinamiklerini, FOPDT sistem tanıma sonuçlarını ve P vs RL karşılaştırmalarını incelemek için `analysis/` araçları kullanılır:

```bash
# 1. P Kontrolcü vs PPO RL Akademik Benchmark Analizi (300 DPI Görsel)
python turret_fcs/rl_sim/compare_p_vs_rl.py

# 2. 6 Panelli Detaylı Dinamik ve Kontrol Kararlılık Analizi
python analysis/plot_tuned_system.py

# 3. 4 Panelli FOPDT Sistem Tanıma ve Kök-Yer Eğrisi Analizi
python analysis/system_identification.py

# 4. Donanım Telemetri Kütüklerinin Otomatik İncelemesi
python analysis/inspect_logs.py
```

---

## 📂 Dizin Yapısı

```text
turret_fcs/
├── .github/                   # GitHub Actions CI sürekli entegrasyon iş akışları (ci.yml)
├── tests/                     # 19 otomatik Pytest birim, SSOT ve duman testi
├── requirements.txt           # Python kütüphane bağımlılıkları (YOLO, Torch, SB3, Optuna, vb.)
├── pyproject.toml             # Python paket ve yapılandırma manifestosu (v1.3)
├── CHANGELOG.md               # Sürüm değişiklikleri ve sürüm geçmişi
├── CITATION.cff               # Akademik alıntı ve tez referans üstverisi
├── setup_env.bat              # Otomatik sanal ortam (venv) ve kütüphane kurulum aracı
├── main.bat                   # Tek tıkla gerçek zamanlı atış kontrol başlatıcısı
├── README.md                  # Proje mimarisi ve teknik dokümantasyon
├── docs/                      # Dokümantasyon, şemalar ve CAD varlıkları
│   ├── assets/                # Mimari şemalar, CAD renderları ve akış şemaları (.png, .jpg, .svg)
│   ├── cad/                   # 3D yazıcı üretim ve prototipleme dosyaları (.stl)
│   └── evidence/              # v1.2/v1.3 kıyaslama verileri (summary.md, 2 CSV test kaydı)
├── analysis/                  # Akademik kontrol ve sistem tanıma araçları
│   ├── inspect_logs.py        # Konsolide donanım telemetri test teftiş ve metrik analiz motoru
│   ├── generate_benchmark_plots.py # Yayın kalitesinde (300 DPI) grafik üretim motoru
│   ├── plot_tuned_system.py   # Python 6 panelli dinamik analiz motoru
│   ├── system_identification.py # FOPDT basamak eğri uyumu ve Padé kök-yer eğrisi
│   ├── turret_fcs_analysis.m  # MATLAB akademik analiz ikizi
│   └── figures/               # Yüksek çözünürlüklü (300 DPI) akademik grafikler ve metrics.json
├── runs/                      # YOLOv8 tespit modeli çıktıları
│   └── detect/balloon_detector/weights/best.pt # Özel eğitilmiş balon tespit model ağırlığı
├── turret_fcs/                # Gerçek donanım çalışma kodları
│   ├── config.py              # SSOT (Single Source of Truth) merkezi konfigürasyon modülü
│   ├── main.py                # Ana atış kontrol döngüsü ve hedef koordinasyonu
│   ├── telemetry.py           # v2 Telemetri kaydedicisi, JSON metadata sidecar ve event log
│   ├── fcs_manager.py         # Hedef seçim, kilit, balistik ve menzil kapılama motoru
│   ├── lock_manager.py        # Kapalı form 2B Gauss vuruş olasılığı (ncx2 CDF) ve histerezis kilit
│   ├── fcs_comm.py            # Ters kayan pencereli STM32 UART binary paket haberleşme sürücüsü
│   ├── kalman_filter.py       # CWNA adaptif Kalman filtresi ve hız kestirimi (Lead Pursuit)
│   ├── turret_hud.py          # Askeri standartta taktiksel HUD ve muharebe sadeleştirmesi
│   ├── rl_controller.py       # 11-boyutlu eylem geçmişli PPO RL çıkarım motoru
│   ├── p_controller.py        # Yumuşak ölü bölgeli Tip-1 P motor kontrolcüsü
│   ├── patrol_generator.py    # Lissajous eğrisi parametrik devriye motoru
│   ├── target_selector.py     # Öncelikli hedef filtreleme algoritması
│   ├── camera_thread.py       # Sıfır gecikmeli, watchdog korumalı kamera sürücüsü
│   └── rl_sim/                # Pekiştirmeli öğrenme ortamı ve eğitim araçları
│       ├── turret_env.py      # 11-D Gymnasium taret ortamı (LQR Reward & Action History)
│       ├── optimize_rl.py     # Optuna TPE Bayesyen Hiperparametre Optimizasyonu
│       ├── train_rl.py        # PPO model eğitim kodu (Stable-Baselines3, 250k steps)
│       ├── compare_p_vs_rl.py # P vs RL akademik benchmark karşılaştırması
│       └── models/turret_ppo_final.zip # Eğitilmiş üretim PPO politikası
├── stm32/                     # STM32F401RE Nucleo gömülü yazılım projesi (CubeIDE)
│   └── turret_main/           # Ana taret donanım projesi (.ioc, Core/Src, Core/Inc)
│       └── Core/Src/          # fcs_comm.c, servo_driver.c, luna_read.c, flywheel_solenoid.c, main.c
└── webots/                    # Webots dijital ikiz simülasyonu
    ├── controllers/           # Simülasyon kontrolcüleri (webots_main, webots_turret_hud, vb.)
    ├── worlds/                # 3D simülasyon dünyası ve aktörler
    └── analysis/              # Webots performans analiz motorları ve telemetri verileri
```

---

## 📜 Lisans & Akademik Referans

Bu proje Yıldız Teknik Üniversitesi Kontrol ve Otomasyon Mühendisliği Bölümü Bitirme Çalışması & Tasarım Projesi kapsamında geliştirilmiştir. Tüm hakları proje ekibine aittir.

Projenin akademik çalışmalarda veya yayınlarda referans gösterilmesi için:

```bibtex
@misc{turret_fcs_2026,
  author = {Senturk, Berkay and Project Team},
  title = {Autonomous Air Defense Turret Fire Control System (Turret FCS)},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/brkysnturk/turret_fcs}},
  version = {v1.3}
}
```
