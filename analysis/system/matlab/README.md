# Turret FCS — MATLAB & Simulink Kontrol ve Kararlılık Paketi

Bu dizin, Turret FCS (Yangın Kontrol Sistemi) projesinin akademik seviyede sistem tanımlama, klasik kontrol teorisi kararlılık analizi, Simulink dijital ikizi ve Hardware-in-the-Loop (HIL) doğrulama altyapısını içerir.

Tüm analiz betikleri gerçek saha telemetri verileriyle (`logs/telemetry_*.csv`) ve fiziksel taret parametreleriyle (MG996R servo, STM32F401RE, 30 FPS YOLO kamera gecikmesi) tam uyumlu olarak çalışır.

> **Figürlerin tekrar üretilebilirliği:** Bu dizindeki figürler MATLAB R2025b ile
> gerçekten çalıştırılarak üretilmiştir. PowerShell:
> ```powershell
> & 'C:\Program Files\MATLAB\R2025b\bin\matlab.exe' -batch "clear all; rehash; addpath('analysis/system/matlab'); control_stability_analysis"
> ```
> Tek kaynak `.m` dosyaları ve Simulink modelidir; ayrı bir Python portu yoktur.

---

## 0. Bu turda `.m` dosyalarında düzeltilen gerçek hatalar

| Dosya | Hata | Düzeltme |
|---|---|---|
| `control_stability_analysis.m` | Marjlar `margin(L_P_exact)` ile alınıyordu; sıfır frekans kutbu (integrator) nedeniyle **yanlış** sonuç veriyordu | Marjlar analitik model + unwrap edilmiş faz üzerinden hesaplanıyor |
| `control_stability_analysis.m` | Kritik kazanç `find(phase <= -180)` ile aranıyordu; `bode` fazı **derece** döner ve faz sarması nedeniyle koşul hiç tetiklenmiyor, sonuç sessizce sabit `27.3 s^-1` varsayımına düşüyordu | `unwrap(ph*pi/180) <= -pi` ile aranıyor → `Kv_crit = 17.24 s^-1` (GM ile tutarlı) |
| `control_stability_analysis.m` | Bode faz paneli `-3600 derece`ye kadar iniyordu (`bodeplot` sarmayı açmıyor) → okunamaz | Bode panelleri analitik fazla yeniden çizildi; log eksen korundu |
| `turret_hil_runner.m` | Telemetri paketi hiç gelmediğinde `actual_angle = actual_angle + cmd_step*0.8` mock'u üretiliyor ama mod etiketi yine **"FİZİKSEL STM32"** oluyordu. Bu makinede COM1 açılıyor fakat **0 gerçek paket** geliyordu | Gerçek/mock paket sayacı eklendi; etiket **sonradan** belirleniyor → `SANAL DONANIM (MOCK) — gerçek telemetri YOK (0/240)` |

---

## 1. Dizin Mimarisi ve Dosya Rolleri

| Dosya | Görev / İşlev | İlgili Faz |
|---|---|---|
| `system_id_turret.m` | Gerçek telemetri CSV verisinden sistem tanımlama: Gecikme (theta), zaman sabiti (tau), dişli boşluğu (backlash) ve FOPDT/SOPDT transfer fonksiyonu çıkarma. | Madde 1 |
| `control_stability_analysis.m` | S-düzlemi kararlılık analizi: Kök-Yer Eğrisi (Root Locus), Bode/Margin (GM, PM), Nyquist diyagramı ve Ki=0 / Kd=0 kontrol teorisi ispatı. | Madde 2 |
| `build_turret_simulink.m` | `turret_digital_twin.slx` Simulink modelini programatik olarak sıfırdan inşa eden ve kaydeden betik. | Madde 5 |
| `run_turret_simulation.m` | Simulink dijital ikizini 10 saniye çalıştıran, kapalı çevrim takip hatasını (RMS, MAE, Kilitlenme Oranı) hesaplayan ve 300 DPI grafik üreten betik. | Madde 5 |
| `turret_hil_runner.m` | PC (MATLAB) ile STM32 mikrodenetleyici arasında 115200 Baud binary protokolüyle donanım döngüde (HIL) canlı takip ve arayüz betiği. | Madde 5 |
| `identified_model.mat` | `system_id_turret.m` tarafından üretilen ve diğer betiklerin doğrudan okuduğu doğrulanmış model parametreleri. | Çıktı |
| `hil_test_result.mat` | `turret_hil_runner.m` çıktısı: log dizileri, RMS, kilit oranı ve gerçek/mock paket sayaçları. | Çıktı |
| `turret_digital_twin.slx` | `build_turret_simulink.m` tarafından üretilen Simulink modeli. | Çıktı |
| `analysis/figures/matlab/` | Tez ve makale için 300 DPI çözünürlükte kaydedilen yayın kalitesindeki MATLAB grafikleri (`matlab_fig1`–`matlab_fig6`). | Rapor |

---

## 2. Madde 1: Sistem Tanımlama (System Identification)

`system_id_turret.m` betiği, taretin fiziksel test kayıtlarını (`telemetry_*.csv`) okuyarak dinamik modeli çıkarır:

* **Taşıyıcı Gecikme (Transport Delay / theta):** Uçtan uca kamera USB tamponu, TensorRT YOLOv8 + balistik filtreleme ve UART iletiminden kaynaklanan toplam gecikme (Ölçülen: `90.0 ms`, tam `3.0 kare @ 30 FPS`).
* **Mekanik Dişli Boşluğu (Backlash):** Yön değişimlerinde servonun harekete başlayana kadar yuttuğu açısal komut (Canlı telemetri ile ölçülen: `1.61 derece`).
* **Transfer Fonksiyonları (turret_fcs/config.py SSOT Uyumlu):**
  * **Tip-1 Açısal Konum Modeli:** `G_pos(s) = [ K / (s * (tau * s + 1)) ] * exp(-theta * s)`
    * Kazanç `K = 0.985`
    * Zaman sabiti `tau = 30.0 ms` (FOPDT adım/Bode testi / `CONFIG.mount.servo_tau`)
    * Efektif ölü zaman `theta = 90.0 ms` (3 tam kare)
    * Uyum başarısı: `R^2 = 0.993`, `RMSE = 0.349 derece`.
  * **2. Derece Eyleyici Modeli (SOPDT):** `G_sopdt(s) = [ K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) ] * exp(-theta * s)` (`wn = 14.66 rad/s`, `zeta = 0.76`).

### Kritik Gözlenebilirlik Notu (tezde mutlaka belirtin)

`servo_pan_fb` gerçek bir potansiyometre okuması **değildir**. Firmware `Servo_GetPan()`
yalnızca yazılan hedef açıyı döndürür (`stm32/turret_main/Core/Src/servo_driver.c`) ve bu
açı STM32'de adım/hız yasalarıyla ideal olarak entegre edilir. Telemetride
`corr(servo_pan_fb, turret_pan) = -0.999` olması bunun kanıtıdır.

Sonuç: telemetriden **gerçek servo zaman sabiti (tau), dişli boşluğu ve DC kazanç K
uydurulamaz**. Bu nedenle `fig1` artık bir FOPDT uydurma iddiasında bulunmaz; bunun yerine
gözlenebilirlik kanıtını, servo adım kuantalamasını ve komut↔servo gecikmesini gösterir.

**Çalıştırma:**
```matlab
cd('analysis/system/matlab');
system_id_turret;
```
**Çıktı Grafiği:** `analysis/figures/matlab/matlab_fig1_system_id_result.png` (Gözlenebilirlik denetimi, servo adım histogramı, komut↔servo gecikmesi).

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
  * **Kritik hız kazancı:** `Kv_crit = 17.24 s^-1` (`Kp_crit = 0.575 derece/kare`) → çalışma noktası `2.94x` güvenlik marjında.
  * Sistem klasik kontrol kriterlerini (`PM > 45 derece, GM > 6 dB`) fazlasıyla sağlamakta olup kararlıdır.

### Gecikme Modeli Duyarlılığı (tezde savunulacak nokta)

Padé yaklaşımı, `1/s` kutbuyla birleşince tam tur faz sarması ürettiği için marj
tahminleri **Padé derecesine göre 80 dB'den fazla savrulur**:

| Model | GM (dB) | PM (derece) |
|---|---:|---:|
| Analitik exact delay (güvenilir) | **9.37** | **49.04** |
| Padé n=1 | 81.45 | 178.06 |
| Padé n=3 | **0.12** (marjinal görünür) | 7.40 |
| Padé n=10 | **-14.99** (kararsız görünür) | -192.96 |

Bu nedenle **kararlılık marjları yalnızca exact delay ile raporlanmalıdır**; Padé yalnızca
kök-yer eğrisi çizimi için kullanılır. (Geçmişteki çelişkili `GM = 41.9 dB` / `GM = 13.4 dB`
ifadeleri bu duyarlılığın yan ürünüdür.)

### Analiz Kapsamı
1. **Kök-Yer Eğrisi (`analysis/figures/matlab/matlab_fig2_stability_root_locus.png`):** P vs PI kök dalları, çalışma noktası kutupları ve Padé derecesi duyarlılık paneli.
2. **Bode ve Nyquist (`analysis/figures/matlab/matlab_fig3_stability_bode_nyquist.png`):** exact delay ile GM/PM, `(-1, j0)` kritik noktasına en yakın yaklaşım (`0.548`, `1/GM = 0.340` ile uyumlu).
3. **Zaman Cevabı (`analysis/figures/matlab/matlab_fig4_stability_step_response.png`):** P vs PI basamak tepkisi ve yumuşak ölü bölgenin (`+/- 0.85 derece`) avlanmayı sıfırlaması (yön değişimi `2964 -> 0`).

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
* **Mechanical Plant:** `Backlash` bloğu (`1.61 derece`), `Transfer Fcn` servo dinamiği (`tau = 68.2 ms`) ve `Integrator` (`1/s`, `0 - 180 derece` sınırları, başlangıç `90 derece`).
* **Vision Sensor:** `Zero-Order Hold` (`30 FPS / 33.3 ms` ayrık kamera örneklemesi).

**Modeli İnşa Etme ve Çalıştırma:**
```matlab
cd('analysis/system/matlab');
build_turret_simulink;   % turret_digital_twin.slx dosyasını üretir
run_turret_simulation;    % Benzetimi koşar ve performans metriklerini döker
```
**Çıktı Grafiği:** `analysis/figures/matlab/matlab_fig5_simulink_simulation_result.png`
(blok şeması + kapalı çevrim; `t >= 1.5 s` için RMS = `1.905 derece`, kilit = `%21.3`).

### B) Hardware-in-the-Loop (HIL) Canlı Testi (`turret_hil_runner.m`)
* MATLAB, sanal bir hedef yörüngesi (`0.25 Hz`, `12 derece` sinüs) üretir.
* 115200 Baud binary seri port üzerinden STM32 Nucleo kartına 7 baytlık komut paketi (`0xAA [Pan] [Tilt] [State] [Checksum]`) gönderilir.
* STM32'den gelen 10 baytlık telemetri paketi (`0x55 [Mesafe] [Sinyal] [Pan] [Tilt] [Checksum]`) okunur. **Not:** bu paketteki açı, servo potansiyometresi değil STM32'nin yazdığı hedef açıdır.
* **Sanal Donanım Modu:** Eğer fiziksel STM32 bağlı değilse, betik kullanıcıyı engellemeden otomatik olarak sanal donanım yanıtı üreterek testi tamamlar.

> **`matlab_fig6` durumu (dürüst etiket):** Depodaki `hil_test_result.mat` kaydı,
> betiğin **mock** dalının ürettiği izle birebir örtüşür (korelasyon `0.9987`).
> Bu yüzden figür artık "FİZİKSEL STM32" başlığı taşımaz, "SANAL DONANIM (MOCK)"
> olarak etiketlenir. Gerçek donanım kanıtı için STM32 bağlıyken betiği koşun ve
> figürü yeniden üretin.

**Çalıştırma:**
```matlab
cd('analysis/system/matlab');
turret_hil_runner;
```
**Çıktı Grafiği:** `analysis/figures/matlab/matlab_fig6_hil_test_result.png` ve `hil_test_result.mat`.

---

## 5. Bitirme Tezi ve Akademik Rapor İçin Kullanım Önerisi

1. **Bölüm: Sistem Modeli ve Dinamik Tanımlama:** `matlab_fig1_system_id_result.png` ile
   gözlenebilirlik sınırını açıkça belirtip, model parametrelerini
   `K = 0.985, tau = 30.0 ms, theta = 90.0 ms, backlash = 1.61 derece` olarak **SSOT'tan**
   alındığını yazın (telemetriden ölçülmedi).
2. **Bölüm: Kontrolcü Tasarımı ve Kararlılık Analizi:** `matlab_fig2` ve `matlab_fig3`
   grafiklerini kullanarak `Ki = 0` / `Kd = 0` tercihini `PM = 49.04 derece`,
   `GM = 9.37 dB` ve `2.94x` kritik kazanç marjı ile savunun. Marjları **yalnızca exact
   delay** ile verin; Padé derecesi duyarlılık panelini bilimsel dürüstlük kanıtı olarak ekleyin.
3. **Bölüm: Dijital İkiz Doğrulaması:** `matlab_fig5` ile Tip-1 P kontrolcünün sinüzoidal
   manevrada kalıcı rampa hatasını (`e_ss = omega / Kv`, `omega = 9.42 derece/s` için
   `1.61 derece`) simülasyonla karşılaştırın.
4. **Bölüm: HIL:** Gerçek STM32 ölçümü alınmadan `matlab_fig6` kanıt olarak sunulmamalıdır;
   mevcut figür açıkça mock olarak etiketlenmiştir.

