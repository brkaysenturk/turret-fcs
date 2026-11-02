# Basket Challenge: Turret FCS Kulplu Kupaya Basket Atma Projesi

Bu proje; otonom pan-tilt tareti kullanarak masada duran kulplu bir kupaya (mug) dart ile basket atma gorevini gerceklestiren 2 asamali hiyerarsik RL sistemidir.

---

## 1. Hiyerarşik Mimari (2-Aşamalı Model Yapısı)

1. **1. Model (Görsel Kilitlenme - CupTracker):**
   * Dosya: [cup_tracker.py](file:///c:/YTU/turret-fcs/turret_basket/cup_tracker.py) & [cup_tracker_policy.npz](file:///c:/YTU/turret-fcs/turret_basket/rl/models/cup_tracker_policy.npz)
   * `turret-fcs` projesinde egitilen 9-64-64-2 NumPy PPO modelini kullanir.
   * Kameradan kupayı yakaladigi anda tareti kupanın merkezine optik olarak kilitler (`err_x -> 0`, `err_y -> 0`).
2. **2. Model (Balistik Açı ve Volan Hızı Karar Ajanı):**
   * Ortam: [cup_env.py](file:///c:/YTU/turret-fcs/turret_basket/rl/cup_env.py) (`BallisticBasketEnv`)
   * Egitim Betigi: [train_rl.py](file:///c:/YTU/turret-fcs/turret_basket/rl/train_rl.py)
   * Hedef kilitlendiginde LiDAR mesafesini (1.0m - 3.0m) alir; dartin kupanin agzindan iceriye basket olmasi icin gereken:
     * **Ekstra Tilt Atis Acisini (Süper-elevasyon: 1.0° ile 18.0° arasi)**
     * **Dinamik Volan Surucu Hizini (7.5 m/s ile 13.5 m/s arasi)**
     birlikte optimize eder.

---

## 2. Webots Dunyasi ve Kontrolcusu

* **Dunya Dosyasi:** [webots/worlds/turret_basket.wbt](file:///c:/YTU/turret-fcs/webots/worlds/turret_basket.wbt)
  * Ayni taret (MG996R servo torklari, C270 kamera, TF-Luna LiDAR, fircasiz cift volan motorlari ve solenoid).
  * Masanin karsisinda ici bos kulplu seramik kupa (`DEF CUP`).
  * Kupanin dibinde temas algilayici (`cup_basket_sensor TouchSensor`).
* **Kontrolcu:** [webots_turret_basket.py](file:///c:/YTU/turret-fcs/webots/controllers/webots_turret_basket/webots_webots_turret_basket.py)
  * 1. Model ile kupaya kilitlenir.
  * 2. Modelden (veya nominal balistikten) gelen aci ve volan hiziyla atis yapar.
  * Ekrana modern bilgilendirici [basket_hud.py](file:///c:/YTU/turret-fcs/turret_basket/basket_hud.py) arayuzunu canli basar.

---

## 3. Nasil Calistirilir?

### Adım 1: Webots'ta Test Etme
Webots R2025a uzerinden `webots/worlds/turret_basket.wbt` dosyasini acip simülasyonu baslatin.
* **SPACE:** Dart firlatir.
* **R:** Kupayi masada rastgele yeni bir mesafeye tasir (1.3m - 2.7m).

### Adım 2: 2. Modeli Eğitme
LiDAR mesafesine gore Tilt acisi ve volan surucu devrini ogrenen 2. Modeli egitmek icin:
```powershell
python turret_basket/rl/train_rl.py 50000
```

### Adım 3: Gerçek Donanımı Çalıştırma (Real Hardware Main)
Gerçek STM32 taretini ve C270 kamerayı bağlayıp masadaki kupaya basket atmak için ana giriş noktası:
```powershell
python turret_basket/main.py
```
* **SPACE / Y:** 2 Aşamalı Atışı Başlatır (Yükselme -> 600 ms Spin-up/Settle -> 80 ms Solenoid -> Geri Dönüş).
* **N:** Atışı İptal Eder.
* **E:** E-Stop Acil Fren.
* **R:** Hedef Takipçisini Sıfırlar.
* **Q / ESC:** Güvenli Çıkış.
