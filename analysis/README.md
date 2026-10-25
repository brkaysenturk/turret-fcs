# Analysis Tools & Verification Suite (Analiz ve Doğrulama Araçları)

Bu dizin; Turret FCS projesinin kontrol dinamiklerini, sistem tanıma modellerini, Kalman filtresi kestirim doğruluğunu, Webots simülasyon ve gerçek donanım telemetri loglarını incelemek için geliştirilen akademik analiz ve doğrulama araçlarını içerir.

Tüm grafikler `analysis/figures/` dizinine 300 DPI çözünürlükte kaydedilir.

---

## 📊 Betik ve Çıktı Haritası

| Kategori | Betik Dosyası | Açıklama & Ürettiği Çıktı | Çalıştırma Komutu |
| :--- | :--- | :--- | :--- |
| **Sistem Tanıma** | `plot_tuned_system.py` | 8 panelli analitik model, basamak yanıtı, faz portresi ve Bode kararlılık analizi (`fig1`–`fig8`). | `python analysis/plot_tuned_system.py` |
| **Sistem Tanıma** | `system_identification.py` | Taret motor gecikmesi, zaman sabiti (tau = 45 ms), transfer fonksiyonu kestirimi ve kök-yer eğrisi (`fig9_system_identification.png`). | `python analysis/system_identification.py` |
| **Kontrol & Benchmark** | `generate_benchmark_plots.py` | Saf PPO RL ile Tip-1 P kontrolcünün TurretEnv benchmark karşılaştırma görseli (`fig11_p_vs_rl_final_benchmark.png`). | `python analysis/generate_benchmark_plots.py` |
| **Kontrol & Benchmark** | `p_controller_benchmark.py` | Farklı hedef profillerinde (zikzak, kaçış, dairesel) hızlı kapalı çevrim P adım yanıtı benchmark'ı. | `python analysis/p_controller_benchmark.py` |
| **Kontrol & Benchmark** | `export_rl_policy.py` | PyTorch / SB3 politikasını hafif 11-64-64-2 NumPy MLP (`.npz`) formatına aktarır. | `python analysis/export_rl_policy.py` |
| **Kontrol & Benchmark** | `rl_diagnose.py` | PPO politikasının durum dağılımlarını, eylem sürekliliğini ve kapalı çevrim başarımını doğrular. | `python analysis/rl_diagnose.py` |
| **Kalman** | `generate_kalman_plots.py` | CV Kalman filtresi hız kestirimi ile yer gerçeği (ground truth) karşılaştırma grafiği (`fig10_kalman_tracking_performance.png`). | `python analysis/generate_kalman_plots.py` |
| **Mimari ve Diyagramlar** | `generate_readme_diagrams.py` | Güncel CONFIG değerleriyle mimari, akış ve donanım SVG şemalarını (`docs/assets/turret_fcs_*.svg`) üretir. | `python analysis/generate_readme_diagrams.py` |
| **Telemetri & Doğrulama** | `inspect_logs.py` | En son telemetri CSV kütüğünü otomatik okur; kilit süresi, açısal hata, salınım ve rezonansı raporlar. | `python analysis/inspect_logs.py` |
| **Telemetri & Doğrulama** | `p_controller_validation.py` | Webots telemetrisinden balonun yer gerçeği konumuna göre fiziksel nişan hatası ve kilit analizi. | `python analysis/p_controller_validation.py` |
| **Telemetri & Doğrulama** | `p_profile_sweep.py` | 6 farklı hedef uçuş profiline karşı Webots üzerinde P kazanç matrisi doğrulaması. | `python analysis/p_profile_sweep.py` |
| **Telemetri & Doğrulama** | `p_gain_sweep.py` | Webots ortamında farklı Kp kazançlarının taranması ve yön değiştirme doyum analizi. | `python analysis/p_gain_sweep.py` |
| **Telemetri & Doğrulama** | `plot_patrol_log.py` | Lissajous devriye tarama yörüngesini ve servo açı adımlarını çizer. | `python analysis/plot_patrol_log.py` |
| **Telemetri & Doğrulama** | `consolidate_telemetry_logs.py` | Parçalı saha telemetri dosyalarını tek bir özet veri setinde birleştirir ve KPI grafiği çizer. | `python analysis/consolidate_telemetry_logs.py` |

---

## 🚀 Hızlı Başlangıç

Tüm temel akademik analiz grafiklerini tek seferde üretmek için:

```bash
# 1. 8 Panelli Dinamik Kararlılık Figürleri (fig1 - fig8)
python analysis/plot_tuned_system.py

# 2. Sistem Tanıma ve Kök-Yer Eğrisi Figürü (fig9)
python analysis/system_identification.py

# 3. Kalman Filtresi Hız ve Doğrulama Figürü
python analysis/generate_kalman_plots.py

# 4. P vs RL Benchmark Karşılaştırma Figürü
python analysis/generate_benchmark_plots.py

# 5. En Son Telemetri Kütüğünün İncelenmesi
python analysis/inspect_logs.py
```
