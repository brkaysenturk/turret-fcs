# Turret FCS — Kapsamlı Proje Hafızası, Tarihçe ve Gelişim Dokümanı

> Bu doküman; projenin en başından bugüne kadar geçirdiği tüm fazları, alınan mimari/matematiksel kararları, "Neden böyle yaptık?" arşivini ve gelecekte açılacak yeni yapay zeka sohbetlerinin bağlamı hiç kaybetmemesi için gereken tam teknik hafızayı içerir.

---

## 1. Kesin Biçimlendirme ve İletişim Kuralları (STRICT)

- **ASLA LATEX VEYA DOLAR İŞARETİ (`$`, `$$`) KULLANILMAYACAK:**
  Kullanıcı arayüzü LaTeX/MathJax desteklemez; ham kod gibi çirkin görünür.
  Tüm matematik, açılar, birimler ve formüller düz metin (plain text) olarak yazılacaktır:
  Örnek: `45 derece`, `5.0 saniye`, `30 FPS`, `50 Hz`, `Kp = 0.10`, `e_ss = v * (T - Kd) / Kp`, `+/- 45 derece`, `tau = 45 ms`.
- **DOKÜMANTASYONA VE COMMITLERE ANLIK KONUŞMA SAYILARI YAZILMAYACAK:**
- **GIT COMMIT DİLİ VE FORMATI (STRICT ENGLISH):**
  Tüm Git commit mesajları istisnasız İNGİLİZCE ve Conventional Commits formatında (`feat:`, `fix:`, `docs:`, `refactor:`, `chore:`, vb.) yazılacaktır. Asla Türkçe commit mesajı atılmayacaktır.
- **DOSYA BAĞLANTILARI:** Kod içindeki dosya ve sınıflara her zaman GitHub markdown link formatında (`[dosya_adi](file:///...)`) referans verilecektir.

---

## 2. Projenin Başlangıcından Bugüne Kronolojik Evrimi (Gelişim Fazları)

### FAZ 1: İlk Kurulum, Hedef Tespiti ve Temel Takip (İlk Adımlar)
- **Problem:** Hava tehditlerine (mikro İHA ve balonlar) karşı yüksek hızlı hedef tespiti ve optik eksen yönlendirmesi gerekiyordu.
- **Geliştirilenler:**
  - C++ tabanlı yüksek FPS HSV renk uzayı filtreleme ve morfolojik hedef yakalama hattı (`hsv/`).
  - Daha sonra askeri sınıfta genel nesne tespiti için **YOLOv8n derin öğrenme modeli** ve çoklu hedef takibi için **ByteTrack (kalıcı kilit kimliği / lock persistence)** mimarisine geçildi.
  - Video akışında donma ve USB tampon yığılmasını engellemek için `threading` tabanlı sıfır gecikmeli kamera sürücüsü ([camera_thread.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/camera_thread.py)) yazıldı.

### FAZ 2: Sensör Füzyonu ve TF-Luna ToF LiDAR Entegrasyonu
- **Problem:** Tek kamera (monoküler) ile derinlik ve balistik mesafe doğru kestirilemiyordu.
- **Geliştirilenler:**
  - Benewake TF-Luna ToF LiDAR sensörü 100 Hz UART hattıyla doğrudan STM32'ye bağlandı.
  - **Menzil Kapılama (Range Gating) İcadı:** Hedef namlu merkezinden uzaktayken veya hedef kaçtığında LiDAR lazer ışınının arkadaki odaya/duvara çarpıp mesafeyi aniden 4-5 metreye fırlatması engellendi. Hedef merkezde değilken son geçerli hedef mesafesi (`last_valid_target_dist_m`) dondurularak kararlılık sağlandı.

### FAZ 3: STM32F401RE Gömülü Donanım ve Güvenlik Bekçileri (Watchdogs)
- **Problem:** PC tarafı kilitlendiğinde ya da seri iletişim koptuğunda servoların ve fırlatıcıların kontrolden çıkma riski vardı.
- **Geliştirilenler:**
  - STM32F401RE Nucleo üzerinde 100 Hz ikili (binary) haberleşme paketi ve ters kayan pencereli (sliding window) ayrıştırıcı geliştirildi ([fcs_comm.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_comm.py)).
  - **300 ms Heartbeat Watchdog:** PC 300 ms boyunca yeni komut göndermezse STM32 acil duruma geçer ve `Launcher_EmergencyStop()` ile volan motorlarını ve solenoidi anında keser.
  - **Mandallı E-Stop (Space Tuşu):** Operatör acil frene bastığında taret donanımsal mekanik merkezine (90°, 90°) kilitlenir.
  - **120 ms Kenar Tetiklemeli Solenoid Mandalı:** Çift dart atılmasını ve solenoid bobininin aşırı ısınıp yanmasını önleyen donanımsal darbe sınırlaması yapıldı.

### FAZ 4: Klasik Kontrol Teorisinin Evrimi ve "Tip-1 P" Kararı
- **Problem:** İlk başta sisteme klasik PID uygulanmaya çalışıldı, ancak servolar şiddetle titredi (limit cycle) ve kararsızlığa girdi.
- **Alınan Kararlar:**
  - **Neden Ki ve Kd (D-termi) Kullanılmadı?:**
    1. **Ki = 0:** STM32 üzerindeki açısal pozisyon denklemi `theta(k) = theta(k-1) + delta_theta` şeklindedir. Yani motorun kendisi ayrık zamanda saf bir integratördür (Tip-1 Servomekanizma). Yazılıma harici bir Ki integrali koymak açık çevrime ikinci bir kutup (`1/(z-1)^2`) ekleyerek rezonans faz çökmesine yol açıyordu. Bu yüzden `Ki = 0` yapıldı ve kalıcı durum hatası doğal olarak sıfırlandı.
    2. **Kd = 0 (D-termi İptali):** Kamera tabanlı hedef hız/konum kestirimleri, donanımın ayrık kuantalaması ve YOLO görüntü gürültüsü içerir. MG996R gibi dahili potansiyometreli ve yüksek mekanik boşluklu (backlash) servolara harici türev veya hız ileri beslemesi eklemek motorlarda limit cycle titreşimine ve dişli yıpranmasına yol açtığı için D-termi tamamen kaldırılmış; yerine kritik sönümlü saf P omurgası tercih edilmiştir. Çevik manevra ihtiyacı ise doğrudan PPO RL moduna bırakılmıştır.
  - **Neden Sert Ölü Bölge Değil de Yumuşak Ölü Bölge (Soft Deadband)?:** Sert eşikte (`if abs(err) < db: step = 0 else: step = Kp * err`), eşikten çıkıldığı an motora ani basamak darbesi biniyor ve dişlileri yıpratan bir avlanma (hunting/chattering) oluşuyordu. Yumuşak ölü bölge formülüyle (`eff_err = sign(err) * (|err| - db)`) hata eşiği aşıldığı an motor harekete sıfırdan pürüzsüz başlar hale getirildi.

### FAZ 5: Webots Dijital İkizi (R2025a) ve Kinematik Doğrulama
- Fiziksel tarete zarar vermeden önce kinematik doğrulamayı yapmak için Webots ortamında MG996R servo tork sınırları, kütle eylemsizlikleri ve kamera optik açısı birebir simüle edildi.

### FAZ 6: Pekiştirmeli Öğrenme (RL) ve Kontrolcü Mimarisi
- **Problem:** Tesis üzerinde yaklaşık 80 ms (yaklaşık 2.5 kare) taşıma gecikmesi vardı. Hedef aniden yön değiştirdiğinde klasik kontrolcü geride kalıyordu.
- **Geliştirilenler:**
  - 500k adımlık ilk PPO denemesi simülasyonun ideal şartlarına aşırı uyum sağladı (overfitting).
  - Optuna (TPE) ile Bayesyen Hiperparametre Optimizasyonu yapıldı (`best_params.json`).
  - Gözlem uzayı Bryson Kuralı ile [-1.0, 1.0] aralığına normalize edildi; sakinlik şartlı Gauss hedef kilit ödülü geliştirildi.
  - **11-Boyutlu Durum Uzayı:** `[norm_err_x, norm_err_y, norm_d_err_x, norm_d_err_y, norm_vx, norm_vy, norm_dist, act_pan_t1, act_tilt_t1, act_pan_t2, act_tilt_t2]`.
  - **A/B Toggle Mimarisi:** Karmaşık ve kararsızlığa açık dışbükey harmanlama yerine operatörün `M` tuşuyla seçebildiği saf **`[P CONTROLLER (Hassas Kilit)]`** ve **`[RL CONTROLLER (Çevik Takip)]`** mimarisine geçildi.

### FAZ 7: v1.1 Denetimi ve Kusursuzlaştırma
- Gerçek donanım testleriyle sistemin açıkları kapatıldı:
  1. Başarısız hibrit harmanlama ve parazit yapan D-termi tamamen kaldırıldı; sistem saf **Yumuşak Ölü Bölgeli Tip-1 P Kontrolcü** ve **PPO RL** ayrık modlarına oturtuldu.
  2. Simülasyon ortamındaki eylem geçmişinin kamera gecikme kuyruğuna sokulması hatası düzeltildi; eylemler taze yoldaki komutlar (`a(t-1), a(t-2)`) olarak verilerek sim-to-real boşluğu kapatıldı.
  3. Gerçek donanım asimetrik sınırları (Pan 0.55°/kare, Tilt 0.22°/kare) ve dikey eksendeki 300g pirinç namlu eylemsizliği için yumuşak dikey sönümleme filtresi eklendi.
  4. STM32 için 0.1° altı UART Kuantalama Biriktiricisi (`residue accumulator`) entegre edildi.
  5. PPO modeli bu yeni ortamda 250.000 adım eğitildi ve ortalama ödülü **3.260 puana** çıkarak kaydedildi.

### FAZ 8: v1.2 Üretim Sürümü (Production Release) ve Şeytanın Avukatı Denetimi
- Gerçek donanım doğrulama testleri ve derin kod denetimi tamamlandı:
  1. **Tilt Ekseninde Limit Cycle'ın Sıfırlanması:** Smith Predictor uygulanan eylem belleği (`act_hist_1`, `act_hist_2`) ölü bölge sonrasına taşındı; asimetrik ölü bölgeler (Pan 0.85°, Tilt 0.95°), kilit yakını sönümleme (0.45 - 1.0) ve sıfır jerk dikey EMA filtresi ile fiziksel testte %50.2 kilitlenme oranı, 13.16 saniye (1.061 kare) aralıksız rekor kilit serisi ve dikeyde %0.0 yön değişimi (sıfır titreşim) elde edildi.
  2. **Hedef Kayıp Toleransı Koruması:** `main.py` içinde hedefin 1 karelik geçici tespit kaçışında erken `fcs.reset()` çağrısı engellendi, `max_lost_frames=4` toleransı korundu.
  3. **Ego-Hareket Aktüatör Gözlemcisi İlklendirmesi:** `fcs_manager.py` içinde hedef yeni yakalandığında (`was_lost`) aktüatör gözlemcisinin sıfırdan başlaması sağlanarak önceki artık hareket komutlarından kaynaklanan sahte optik akış sıçraması engellendi.
  4. **Mimari ve Vektörel Şemalar:** `turret_fcs_flowchart.svg`, `turret_fcs_hardware.svg` ve `turret_fcs_architecture.png` v1.2 ayrık A/B kontrol mimarisine güncellendi; `mechanical_design.jpg` 3D donanım yerleşimi dokümantasyona bağlandı.
  5. **Konsolide Analiz Paketi:** 12 dağınık analiz betiği `analysis/inspect_logs.py` ve `analysis/generate_benchmark_plots.py` altında tek çatıya kavuşturuldu.

---

## 3. "Neden Böyle Yaptık?" Arşivi (Jürinin Sorabileceği Kritik Savunmalar)

| Soru / Konu | Verilecek Bilimsel ve Mühendislik Cevabı |
|---|---|
| **Neden türev (D-termi) kullanmadınız?** | Kamera ve YOLO tespit gürültüsünün türevi servolarda yüksek frekanslı mekanik titreşime (chattering) ve dişli aşınmasına yol açar. STM32 pozisyon biriktiricisi zaten ayrık zamanda saf integratör olduğu için yumuşak ölü bölgeli saf Tip-1 P kontrolcü sıfır jerk ile kalkış ve sıfır kalıcı durum hatası sağlar. Hızlı manevra gerektiren durumlarda ise yapay zeka (RL) devreye girerek çevik takibi üstlenir. |
| **Neden integral (Ki) kullanmadınız?** | STM32 gömülü yazılımındaki pozisyon biriktiricisi (`theta_k = theta_{k-1} + delta`) ayrık zamanda saf integratördür (z/(z-1)). Harici Ki integrali sistemi Tip-2 yaparak çift kutuplu rezonans faz çökmesine ve kontrolsüz salınıma (limit cycle) sürükler. |
| **Neden ölü bölgeye yumuşak geçiş (Soft Deadband) koydunuz?** | Sert eşikli ölü bölge, eşik sınırında anlık basamak darbesi üreterek dişli boşluğuyla (backlash) birleştiğinde yüksek frekanslı mekanik titreşime yol açar. Yumuşak ölü bölge hatayı sıfırdan başlatarak sıfır jerk ile kalkış sağlar. |
| **Neden MPC veya LQR değil de PPO RL?** | Sistemde analitik olarak modellenmesi zor olan süreksiz doğrusal olmayanlıklar vardır (ölü bölge, 0.1° UART kuantalaması, dişli boşluğu, değişken 80 ms USB taşıma gecikmesi). PPO, bu rastgeleleştirilmiş POMDP ailesi üzerinde çevrimdışı eğitilmiş amortize bir optimal politika sunar; çalışma zamanında tek ileri geçiş maliyeti 0.17 ms'dir. |
| **Eylem geçmişi (Action History) neden gözlem uzayında?** | Smith Predictor ilkesine göre, 80 ms (yaklaşık 2-3 kare) gecikmeli bir sistemde ajan doğru karar verebilmek için motora gönderilmiş ama henüz kameraya yansımamış olan "yoldaki komutları" (in-flight commands) bilmek zorundadır. |
| **Atış anında neden basamak sıçraması olmuyor?** | Taret önleme ve süper-elevasyon düzeltmesini atış anında aniden eklemez; hedefi doğrudan hesaplanan balistik nişan noktasında (reticle) takip edip kilitlenir. Kilit oturduktan sonra solenoid tetiklenir. |

---

## 4. Temel Kod Dosyaları ve İşlev Haritası

- [main.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/main.py): Ana atış kontrol döngüsü, OpenCV video işleme, klavye kontrolleri.
- [fcs_manager.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_manager.py): Durum makinesi (Arama, Tespit, Takip, Kilit, Ateşleme), balistik motoru, A/B mod yönetimi (`P` vs `RL`), menzil kapılama, 1. derece aktüatör gözlemcili ego-hareket telafisi (`tau = 45 ms`).
- [p_controller.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/p_controller.py): Yumuşak Ölü Bölgeli Tip-1 P motor kontrolcüsü (`Kp_pan=0.055-0.10, Kp_tilt=0.030-0.06`).
- [kalman_filter.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/kalman_filter.py): Ego-hareket telafili 2B durum gözlemcisi, hedef konum, hız kestirimi ve atış önleme (Lead Reticle).
- [rl_controller.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl_controller.py): 11 boyutlu PPO çıkarım motoru; Smith Predictor eylem hafızası, asimetrik dikey sönümleme ve sıfır jerk filtresi.
- [turret_hud.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/turret_hud.py): Askeri standartta dinamik taktiksel HUD arayüzü ve renk disiplini (Yeşil, Mavi, Kırmızı).
- [lock_manager.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/lock_manager.py): Dinamik kilit toleransı ve kademeli Gauss vuruş olasılığı (`hit_probability`).
- [fcs_comm.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_comm.py): STM32 ile 115200 Baud ikili (binary) UART paketi alışverişi, 0.1° altı kuantalama biriktiricisi.
- [camera_thread.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/camera_thread.py): Watchdog korumalı, sıfır gecikmeli kamera iş parçacığı.
- [patrol_generator.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/patrol_generator.py): Lissajous eğrisi parametrik hava sahası tarama devriyesi.
- [turret_env.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl_sim/turret_env.py): 11 boyutlu simülasyon ortamı; gecikmeli sensör kuyruğu (7D) + taze eylem geçmişi (4D).
- [train_rl.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl_sim/train_rl.py): 250k adımlık PPO eğitim motoru.

---

## 5. Yeni Bir Sohbet Açıldığında Hatırlanacaklar

Yeni bir sohbet açıldığında asistan bu dosyayı okur okumaz:
1. Sistemin şu anki durumunun **v1.2 Production Release** olduğunu,
2. Modelin yeni eğitilmiş `turret_ppo_final.zip` (3.260 puan) olduğunu,
3. Kontrolcünün adının **Yumuşak Ölü Bölgeli Tip-1 P Kontrolcü** olduğunu,
4. Asla LaTeX / dolar işareti (`$`) kullanmaması gerektiğini,
5. Kodda veya tezde yapılacak her yeni geliştirmenin yukarıdaki mimari ve güvenlik disiplinine bağlı kalması gerektiğini anında bilir.
