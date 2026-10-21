# Turret FCS - Performans ve Kararlilik Metrikleri Kaniti (v1.3)

Bu dokuman, otonom hava savunma tareti ates kontrol sisteminin (Turret FCS) saha testlerinden elde edilen telemetri verilerinin ozetini ve karsilastirmali analizini icerir.

## 1. Temsili Telemetri Kayitlari

Depo boyutunu hafif ve hizli klonlanabilir tutmak amaciyla ham telemetri kayitlari yerine iki adet temsili referans ucus / hedef takip verisi saklanmistir:

- `v1.2_baseline.csv`: Ilk parametre optimizasyonu oncesi, temel P kontrolcu ile yapilan statik ve yavas hareketli hedef takip testi.
- `v1.3_benchmark.csv`: Histerezisli Kilit Yoneticisi (LockManager), 2 Boyutlu Gauss Atis Penceresi, Aim-Point Slew Ileri Beslemesi ve PPO Pekiştirmeli Ogrenme (RL) modelleriyle yapilan dinamik manevra testi.

## 2. Karsilastirmali Performans Ozeti

| Performans Parametresi | v1.2 Temel (Baseline) | v1.3 Optimize Edilmis (Benchmark) | Kazanim / Iyilestirme |
| :--- | :--- | :--- | :--- |
| **Pan RMS Takip Hatasi** | 1.84 derece | 0.42 derece | %77.2 hata azalmasi |
| **Tilt RMS Takip Hatasi** | 1.12 derece | 0.31 derece | %72.3 hata azalmasi |
| **Kilit Baslama Suresi (Lock T_acq)** | 0.65 saniye (20 kare) | 0.33 saniye (10 kare) | 2 kat daha hizli angajman |
| **Kilit Devam Orani (Lock Ratio)** | %61.4 | %92.8 | Kesintisiz atis penceresi |
| **Atis Aninda Aci Sapmasi** | +- 1.50 derece | +- 0.25 derece | Balistik slew ileri besleme |
| **Donanim Gecikmesi (Latency)** | 115 ms | 80 ms | Optimize Kalman ve atomik UART |

## 3. Anahtar Mimari Gelistirmeler

1. **Ayrilmis Atis ve Degerlendirme Durumlari:**
   - Fiziksel atis (SPINUP, FIRING, RECOVER: 1.43 saniye) sirasinda kilit durumu dondurulurken, EVAL asamasinda (1.50 saniye) kilit yoneticisi serbest birakilarak hedef kaybina karsi dinamik guvenlik saglanmistir.

2. **Aim-Point Direct Feedforward (Sifi-Gecikmeli Balistik Slew):**
   - Atis oncesi volan ivmelenmesi sirasinda uygulanan acisal slew, turevsel ileri besleme ile dogrudan motor komutuna aktarilmis, sistem gecikmelerine bagli hedef kacirma onlenmistir.

3. **2B Gauss Kilit Penceresi:**
   - Optik hedef kutu boyutlari (w, h) ve mesafe verisiyle olceklenen dinamik 2B Gauss integrali sayesinde, hedef buyuklugune gore atis olabilirligi dinamik olarak hesaplanmaktadir.
