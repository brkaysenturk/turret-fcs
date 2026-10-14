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
  - **Neden Ki = 0?:** STM32 üzerindeki açısal pozisyon denklemi `theta(k) = theta(k-1) + delta_theta` şeklindedir. Yani motorun kendisi ayrık zamanda saf bir integratördür (Tip-1 Servomekanizma). Yazılıma harici bir Ki integrali koymak açık çevrime ikinci bir kutup (`1/(z-1)^2`) ekleyerek rezonans faz çökmesine yol açıyordu. Bu yüzden `Ki = 0` yapıldı ve kalıcı durum hatası doğal olarak sıfırlandı.
  - **Neden Sert Ölü Bölge Değil de Yumuşak Ölü Bölge (Soft Deadband)?:** Sert eşikte (`if abs(err) < db: step = 0 else: step = Kp * err`), eşikten çıkıldığı an motora ani basamak darbesi biniyor ve dişlileri yıpratan bir avlanma (hunting/chattering) oluşuyordu. Yumuşak ölü bölge formülüyle (`eff_err = sign(err) * (|err| - db)`) hata eşiği aşıldığı an motor harekete sıfırdan pürüzsüz başlar hale getirildi.
  - **Neden "D-Termi" Değil, 2-DOF Referans Hızı İleri Beslemesi (Velocity Feedforward)?:** Kodda türev hatanın türevi (`de/dt = v_hedef - v_taret`) değil; doğrudan Kalman durum gözlemcisinden gelen filtrelenmiş hedef açısal hızıdır (`u_ff = kd * omega_hat`). Bu terim bir sönümleme değil, **2 Serbestlik Dereceli Referans Hızı İleri Beslemesidir**. Tip-1 sistemde sabit hızlı hedef takibindeki rampa hız geride kalma hatasını (velocity lag) 7.5 dereceden 2.5 dereceye düşürür.

### FAZ 5: Webots Dijital İkizi (R2025a) ve Kinematik Doğrulama
- Fiziksel tarete zarar vermeden önce kinematik doğrulamayı yapmak için Webots ortamında MG996R servo tork sınırları, kütle eylemsizlikleri ve kamera optik açısı birebir simüle edildi.

### FAZ 6: Pekiştirmeli Öğrenme (RL) ve Hibrit Kontrolcü Mimarisi
- **Problem:** Tesis üzerinde yaklaşık 80 ms (yaklaşık 2.5 kare) taşıma gecikmesi vardı. Hedef aniden yön değiştirdiğinde klasik kontrolcü geride kalıyordu.
- **Geliştirilenler:**
  - 500k adımlık ilk PPO denemesi simülasyonun ideal şartlarına aşırı uyum sağladı (overfitting).
  - Optuna (TPE) ile Bayesyen Hiperparametre Optimizasyonu yapıldı (`best_params.json`).
  - Gözlem uzayı Bryson Kuralı ile [-1.0, 1.0] aralığına normalize edildi; sakinlik şartlı Gauss hedef kilit ödülü geliştirildi.
  - **11-Boyutlu Durum Uzayı:** `[norm_err_x, norm_err_y, norm_d_err_x, norm_d_err_y, norm_vx, norm_vy, norm_dist, act_pan_t1, act_tilt_t1, act_pan_t2, act_tilt_t2]`.
  - **Dinamik Dışbükey Harmanlama (Convex Hybrid Blending):** `u_blend = (1 - w)*u_P + w*u_RL`. Hedef manevra yaparken AI desteği azami %55'e kadar yükselir; kilit oturduğunda AI desteği sönümlenir ve kontrolü sakin P kontrolcüye bırakır.

### FAZ 7: v1.1 Denetimi ve Kusursuzlaştırma (Son Büyük Aşama)
- Claude Opus ile yapılan bağımsız kontrol denetimi sonucunda tespit edilen son açıklar kapatıldı:
  1. Terminoloji jüri standardına uygun olarak "Gözlemci D-termi"nden **"2-DOF Referans Hızı İleri Beslemesi"**ne çevrildi.
  2. Simülasyon ortamındaki eylem geçmişinin kamera gecikme kuyruğuna sokulması hatası düzeltildi; eylemler taze yoldaki komutlar (`a(t-1), a(t-2)`) olarak verilerek sim-to-real boşluğu kapatıldı.
  3. Hibrit modda RL hafızasına fiilen uygulanan harmanlanmış komut geri beslendi (`update_applied_action`).
  4. PPO modeli bu yeni ortamda 250.000 adım eğitildi ve ortalama ödülü **3.260 puana** çıkarak kaydedildi.
  5. MIT Lisansı, Hızlı Başlangıç rehberi ve mimari rozetler repoya eklendi.
  6. Webots R2025a dijital ikizi 11-boyutlu Bryson RL çıkarım hattı, hibrit eylem geri beslemesi ve runtime.ini sanal ortam yapılandırmasıyla tam senkronize edildi.

---

## 3. "Neden Böyle Yaptık?" Arşivi (Jürinin Sorabileceği Kritik Savunmalar)

| Soru / Konu | Verilecek Bilimsel ve Mühendislik Cevabı |
|---|---|
| **Neden D-termi formülünde taret hızı yok?** | Çünkü bu bir hata türevi sönümleyicisi değil, 2-DOF Referans Hızı İleri Beslemesidir (Velocity Feedforward). Kalman durum gözlemcisinden gelen hedef hızını kullanarak Tip-1 döngüdeki rampa takip hatasını (velocity lag) ortadan kaldırır. |
| **Neden integral (Ki) kullanmadınız?** | STM32 gömülü yazılımındaki pozisyon biriktiricisi (`theta_k = theta_{k-1} + delta`) ayrık zamanda saf integratördür (z/(z-1)). Harici Ki integrali sistemi Tip-2 yaparak çift kutuplu rezonans faz çökmesine ve kontrolsüz salınıma (limit cycle) sürükler. |
| **Neden ölü bölgeye yumuşak geçiş (Soft Deadband) koydunuz?** | Sert eşikli ölü bölge, eşik sınırında anlık basamak darbesi üreterek dişli boşluğuyla (backlash) birleştiğinde yüksek frekanslı mekanik titreşime yol açar. Yumuşak ölü bölge hatayı sıfırdan başlatarak sıfır jerk ile kalkış sağlar. |
| **Neden MPC veya LQR değil de PPO RL?** | Sistemde analitik olarak modellenmesi zor olan süreksiz doğrusal olmayanlıklar vardır (ölü bölge, 0.1° UART kuantalaması, dişli boşluğu, değişken 80 ms USB taşıma gecikmesi). PPO, bu rastgeleleştirilmiş POMDP ailesi üzerinde çevrimdışı eğitilmiş amortize bir optimal politika sunar; çalışma zamanında tek ileri geçiş maliyeti 0.17 ms'dir. Ayrıca RL tek başına değil, kararlı Tip-1 P omurgası üzerinde güvenli bir kompanzatör olarak çalışır. |
| **Eylem geçmişi (Action History) neden gözlem uzayında?** | Smith Predictor ilkesine göre, 80 ms (yaklaşık 2-3 kare) gecikmeli bir sistemde ajan doğru karar verebilmek için motora gönderilmiş ama henüz kameraya yansımamış olan "yoldaki komutları" (in-flight commands) bilmek zorundadır. |
| **Atış anında neden basamak sıçraması olmuyor?** | Taret önleme ve süper-elevasyon düzeltmesini atış anında aniden eklemez; hedefi doğrudan hesaplanan balistik nişan noktasında (reticle) takip edip kilitlenir. Kilit oturduktan sonra solenoid tetiklenir. |

---

## 4. Temel Kod Dosyaları ve İşlev Haritası

- [main.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/main.py): Ana atış kontrol döngüsü, OpenCV video işleme, klavye kontrolleri.
- [fcs_manager.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_manager.py): Durum makinesi (Arama, Tespit, Takip, Kilit, Ateşleme), balistik motoru, hibrit harmanlama (`(1-w)*P + w*RL`), menzil kapılama, 1. derece aktüatör gözlemcili ego-hareket telafisi (`tau = 45 ms`).
- [p_controller.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/p_controller.py): 2-DOF Referans Hızı İleri Beslemeli Tip-1 P motor kontrolcüsü (`Kp_pan=0.10, Kp_tilt=0.07, Kd_pan=0.025, Kd_tilt=0.020`).
- [kalman_filter.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/kalman_filter.py): Ego-hareket telafili 2B durum gözlemcisi, hedef konum ve açısal hız kestirimi.
- [rl_controller.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl_controller.py): 11 boyutlu PPO çıkarım motoru; `update_applied_action` ile hibrit komut hafızası.
- [turret_hud.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/turret_hud.py): Askeri standartta dinamik taktiksel HUD arayüzü ve renk disiplini (Yeşil, Mavi, Kırmızı).
- [lock_manager.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/lock_manager.py): Dinamik kilit toleransı ve kademeli vuruş olasılığı (`hit_probability`).
- [fcs_comm.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_comm.py): STM32 ile 115200 Baud ikili (binary) UART paketi alışverişi ve ters kayan pencere ayrıştırıcısı.
- [camera_thread.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/camera_thread.py): Watchdog korumalı, sıfır gecikmeli kamera iş parçacığı.
- [patrol_generator.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/patrol_generator.py): Lissajous eğrisi parametrik hava sahası tarama devriyesi.
- [turret_env.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl_sim/turret_env.py): 11 boyutlu simülasyon ortamı; gecikmeli sensör kuyruğu (7D) + taze eylem geçmişi (4D).
- [train_rl.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl_sim/train_rl.py): 250k adımlık PPO eğitim motoru.

---

## 5. Yeni Bir Sohbet Açıldığında Hatırlanacaklar

Yeni bir sohbet açıldığında asistan bu dosyayı okur okumaz:
1. Sistemin şu anki durumunun **v1.1** olduğunu,
2. Modelin yeni eğitilmiş `turret_ppo_final.zip` (3.260 puan) olduğunu,
3. Kontrolcünün adının **2-DOF Hız İleri Beslemeli Tip-1 P Kontrolcü** olduğunu,
4. Asla LaTeX / dolar işareti (`$`) kullanmaması gerektiğini,
5. Kodda veya tezde yapılacak her yeni geliştirmenin yukarıdaki mimari ve güvenlik disiplinine bağlı kalması gerektiğini anında bilir.
