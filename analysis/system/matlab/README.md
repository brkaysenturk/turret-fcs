# Turret FCS — MATLAB & Simulink Kontrol ve Kararlılık Paketi

Bu dizin, Turret FCS (Yangın Kontrol Sistemi) projesinin akademik seviyede sistem tanımlama, klasik kontrol teorisi kararlılık analizi, Simulink dijital ikizi ve Hardware-in-the-Loop (HIL) doğrulama altyapısını içerir.

Tüm analiz betikleri gerçek saha telemetri verileriyle (`logs/telemetry_*.csv`) ve fiziksel taret parametreleriyle (MG996R servo, STM32F401RE, 30 FPS YOLO kamera gecikmesi) tam uyumlu olarak çalışır.

---

## 1. Dizin Mimarisi ve Dosya Rolleri

| Dosya | Görev / İşlev | İlgili Faz |
|---|---|:---:|
| `system_id_turret.m` | Gerçek telemetri CSV verisinden sistem tanımlama: Gecikme (theta), zaman sabiti (tau), dişli boşluğu (backlash) ve FOPDT/SOPDT transfer fonksiyonu çıkarma. | Madde 1 |
| `control_stability_analysis.m` | S-düzlemi kararlılık analizi: Kök-Yer Eğrisi (Root Locus), Bode/Margin (GM, PM), Nyquist diyagramı ve Ki=0 / Kd=0 kontrol teorisi ispatı. | Madde 2 |
| `build_turret_simulink.m` | `turret_digital_twin.slx` Simulink modelini programatik olarak sıfırdan inşa eden ve kaydeden betik. | Madde 5 |
| `run_turret_simulation.m` | Simulink dijital ikizini 10 saniye çalıştıran, kapalı çevrim takip hatasını (RMS, MAE, Kilitlenme Oranı) hesaplayan ve 300 DPI grafik üreten betik. | Madde 5 |
| `turret_hil_runner.m` | PC (MATLAB) ile STM32 mikrodenetleyici arasında 115200 Baud binary protokolüyle donanım döngüde (HIL) canlı takip ve arayüz betiği. | Madde 5 |
| `identified_model.mat` | `system_id_turret.m` tarafından üretilen ve diğer betiklerin doğrudan okuduğu doğrulanmış model parametreleri. | Çıktı |
| `analysis/figures/matlab/` | Tez ve makale için 300 DPI çözünürlükte kaydedilen yayın kalitesindeki MATLAB grafikleri (`matlab_fig1`–`matlab_fig6`). | Rapor |

---

## 2. Madde 1: Sistem Tanımlama (System Identification)

`system_id_turret.m` betiği, taretin fiziksel test kayıtlarını (`telemetry_*.csv`) okuyarak dinamik modeli çıkarır:

* **Taşıyıcı Gecikme (Transport Delay / theta):** YOLOv8 çıkarımı, kamera USB tamponu ve UART iletiminden kaynaklanan toplam gecikme (Ölçülen: `66.7 ms`, tam `2.0 kare @ 30 FPS`).
* **Mekanik Dişli Boşluğu (Backlash):** Yön değişimlerinde servonun harekete başlayana kadar yuttuğu açısal komut (Canlı telemetri ile ölçülen: `1.61 derece`).
* **Transfer Fonksiyonları (turret_fcs/config.py SSOT Uyumlu):**
  * **Tip-1 Açısal Konum Modeli:** `G_pos(s) = [ K / (s * (tau * s + 1)) ] * exp(-theta * s)`
    * Kazanç `K = 0.985`
    * Zaman sabiti `tau = 68.2 ms` (Optuna TPE optimal / `CONFIG.mount.servo_tau`)
    * Efektif ölü zaman `theta = 66.7 ms` (2 tam kare)
    * Uyum başarısı: `R^2 = 0.993`, `RMSE = 0.349 derece`.
  * **2. Derece Eyleyici Modeli (SOPDT):** `G_sopdt(s) = [ K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) ] * exp(-theta * s)` (`wn = 14.66 rad/s`, `zeta = 0.76`).

**Çalıştırma:**
```matlab
cd('analysis/system/matlab');
system_id_turret;
```
**Çıktı Grafiği:** `analysis/figures/matlab/matlab_fig1_system_id_result.png` (Komut, gerçek servo yanıtı, artık hata ve basamak tepkisi).

---

## 3. Madde 2: Klasik Kontrol Kararlılık Analizi

`control_stability_analysis.m` betiği, projedeki "Tip-1 Saf P Kontrolcü" kararını klasik kontrol teorisiyle ispatlar:

### Neden Ki = 0 Seçildi? (Teorik İspat)
* Taret motorunun açısal konumu ayrık zamanda `theta(k) = theta(k-1) + delta_theta` şeklindedir; yani motorun kendisi bir integratördür (`1/s`).
* Sisteme harici bir integratör (`Ki / s`) eklendiğinde açık çevrimde orijinde çifte kutup (`1/s^2`) oluşur.
* 66.7 ms taşıma gecikmesinin Padé yaklaşımıyla gelen sağ yarı düzlem (RHP) sıfırlarıyla etkileşen Kök-Yer Eğrisi (Root Locus) dalları, en ufak bir `Ki` değerinde bile doğrudan sağ yarı düzleme geçer.
* **Sonuç:** Klasik PI kontrolcünün faz erozyonu ve kutup yığılması sebebiyle dişli avlanması (hunting) oluşur. Saf Tip-1 P kontrolcüde ise (`Kp = 0.1953 derece/kare`, `Kv = 5.859 s^-1`):
  * **Faz Payı (Phase Margin - PM):** `49.04 derece` (Frekans: `5.41 rad/s` / `0.86 Hz`)
  * **Kazanç Payı (Gain Margin - GM):** `9.37 dB` (Frekans: `12.79 rad/s` / `2.04 Hz`)
  * Sistem klasik kontrol kriterlerini (`PM > 45 derece, GM > 6 dB`) fazlasıyla sağlamakta olup kritik sönümlü ve kararlıdır.

### Analiz Kapsamı
1. **Kök-Yer Eğrisi (`analysis/figures/matlab/matlab_fig2_stability_root_locus.png`):** P vs PI kök dalları ve kapalı çevrim kutupları.
2. **Bode ve Nyquist (`analysis/figures/matlab/matlab_fig3_stability_bode_nyquist.png`):** Kazanç Payı (GM), Faz Payı (PM) ve `(-1, j0)` kritik nokta mesafesi.
3. **Zaman Cevabı (`analysis/figures/matlab/matlab_fig4_stability_step_response.png`):** Yumuşak ölü bölgenin (`+/- 0.85 derece`) avlanmayı (hunting) nasıl sıfırladığı.

**Çalıştırma:**
```matlab
cd('analysis/system/matlab');
control_stability_analysis;
```

---

## 4. Madde 5: Simulink Dijital İkiz ve Hardware-in-the-Loop (HIL)

### A) Simulink Dijital İkiz Modeli (`turret_digital_twin.slx`)
Model, fiziksel sistemin tüm dinamik ve lineer olmayan bileşenlerini içerir:
* **Target Ref Switch:** Sinüs dalgası (`0.15 Hz`, `10 derece`) veya basamak hedef (`5 derece`).
* **FCS Controller:** Yumuşak ölü bölge (`Dead Zone: +/- 0.85 derece`), Oransal Kazanç (`Kp_cont = 5.859 s^-1`), Hız Satürasyonu (`+/- 45 derece/s`) ve İvme Sınırlayıcı (`Rate Limiter: +/- 353.1 derece/s^2`).
* **Hardware Delay:** `Transport Delay` bloğu (`66.7 ms` - 2 tam kare @ 30 FPS).
* **Mechanical Plant:** `Backlash` bloğu (`1.61 derece`), `Transfer Fcn` servo dinamiği (`tau = 68.2 ms`) ve `Integrator` (`1/s`, `0 - 180 derece` sınırları).
* **Vision Sensor:** `Zero-Order Hold` (`30 FPS / 33.3 ms` ayrık kamera örneklemesi).

**Modeli İnşa Etme ve Çalıştırma:**
```matlab
cd('analysis/system/matlab');
build_turret_simulink;   % turret_digital_twin.slx dosyasını üretir
run_turret_simulation;    % Benzetimi koşar ve performans metriklerini döker
```
**Çıktı Grafiği:** `analysis/figures/matlab/matlab_fig5_simulink_simulation_result.png`.

### B) Hardware-in-the-Loop (HIL) Canlı Testi (`turret_hil_runner.m`)
* MATLAB, sanal bir hedef yörüngesi (`0.25 Hz`, `12 derece` sinüs) üretir.
* 115200 Baud binary seri port üzerinden STM32 Nucleo kartına 7 baytlık komut paketi (`0xAA [Pan] [Tilt] [State] [Checksum]`) gönderilir.
* STM32'den gelen 10 baytlık telemetri paketi (`0x55 [Mesafe] [Sinyal] [Pan] [Tilt] [Checksum]`) okunur ve servo potansiyometre geri bildirimi canlı arayüzde çizdirilir.
* **Sanal Donanım Modu:** Eğer fiziksel STM32 bağlı değilse, betik kullanıcıyı engellemeden otomatik olarak sanal donanım yanıtı üreterek testi tamamlar.

**Çalıştırma:**
```matlab
cd('analysis/system/matlab');
turret_hil_runner;
```
**Çıktı Grafiği:** `analysis/figures/matlab/matlab_fig6_hil_test_result.png` ve `hil_test_result.mat`.

---

## 5. Bitirme Tezi ve Akademik Rapor İçin Kullanım Önerisi

1. **Bölüm: Sistem Modeli ve Dinamik Tanımlama:** `matlab_fig1_system_id_result.png` grafiğini ve `K = 0.985, tau = 68.2 ms, theta = 66.7 ms, backlash = 1.61 deg` tablosunu ekleyin.
2. **Bölüm: Kontrolcü Tasarımı ve Kararlılık Analizi:** `matlab_fig2_stability_root_locus.png` ve `matlab_fig3_stability_bode_nyquist.png` grafiklerini kullanarak `Ki = 0` ve `Kd = 0` seçiminin faz payı ve limit cycle açısından matematiksel zorunluluk olduğunu gösterin.
3. **Bölüm: Dijital İkiz Doğrulaması:** `matlab_fig5_simulink_simulation_result.png` ile Tip-1 P kontrolcünün sinüzoidal manevrada dinamik hız rampa gecikmesini (`e_ss = v / Kv = 1.61 deg`) teorik ve simüle olarak karşılaştırın.
4. **Bölüm: HIL Doğrulaması:** `matlab_fig6_hil_test_result.png` ile mikrodenetleyicinin döngüdeki tepki ve kilitlenme başarısını gösterin.
