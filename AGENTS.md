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
  - **80 ms Kenar Tetiklemeli Solenoid Darbe Protokolü:** Çift dart atılmasını ve solenoid bobininin aşırı ısınıp yanmasını önleyen donanımsal darbe sınırlaması protokolü tasarlandı (firmware ve config ile uyumlu; masaüstü ve güvenlik testlerinde fiziksel ateşleme yolu yazılımsal/donanımsal olarak izole edilmektedir).

### FAZ 4: Klasik Kontrol Teorisinin Evrimi ve "Tip-1 P" Kararı
- **Problem:** İlk başta sisteme klasik PID uygulanmaya çalışıldı, ancak servolar şiddetle titredi (limit cycle) ve kararsızlığa girdi.
- **Alınan Kararlar:**
  - **Neden Ki ve Kd (D-termi) Kullanılmadı?:**
    1. **Ki = 0:** STM32 üzerindeki açısal pozisyon denklemi `theta(k) = theta(k-1) + delta_theta` şeklindedir. Yani motorun kendisi ayrık zamanda saf bir integratördür (Tip-1 Servomekanizma). Klasik kontrol teorisine göre Tip-1 sistemler statik hedef (basamak) girdilerinde hatayı ölü bölge kararlı bandı içine (Pan: ±0.85°, Tilt: ±0.95°) asimptotik olarak çeker; ölü bölgeye girildiğinde motor eforu sıfırlandığı için bu bant dahilinde sınırlı kalıcı durum hatası kalır ve mekanik dişli titreşimi (hunting) tamamen sıfırlanır. Hareketli hedeflerdeki sonlu rampa gecikmesi ise (`e_ss = omega / Kv`) Kalman filtreli hız kestirimi ve balistik hız ileri beslemesi (velocity feedforward) ile karşılanır. Yazılıma harici bir Ki integrali koymak açık çevrime ikinci bir kutup (`1/(z-1)^2`) ekleyerek rezonans faz çökmesine ve kontrolsüz limit döngüsü (limit cycle) titreşimine yol açıyordu. Bu yüzden `Ki = 0` yapıldı.
    2. **Kd = 0 (D-termi İptali):** Kamera tabanlı hedef hız/konum kestirimleri, donanımın ayrık kuantalaması ve YOLO görüntü gürültüsü içerir. MG996R gibi dahili potansiyometreli ve yüksek mekanik boşluklu (backlash) servolara harici türev eklemek motorlarda limit cycle titreşimine ve dişli yıpranmasına yol açtığı için D-termi tamamen kaldırılmış; yerine kritik sönümlü saf P omurgası tercih edilmiştir. Çevik manevra ihtiyacı ise doğrudan PPO RL moduna bırakılmıştır.
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
  6. **Patrol Birikimli Adımlayıcı (Step Accumulator):** 80-100 FPS döngüde merkeze dönerken 0.20° gibi minik adımların MG996R ölü bölgesinde takılı kalmasını önlemek için `patrol_generator.py` içine birikimli adım mekanizması eklendi; komutlar birikip `0.95°` eşiğini aştığında taret 90-90 mekanik merkezine net torklu adımlarla döner hale getirildi.
  7. **Dinamik Kalman Hız Ölü Bölgesi (Dynamic Velocity Deadband):** Taret dönüş hızı (`ego_speed`) ve hedef mesafesine (`dist_m < 1.2m`) bağlı olarak Kalman hız ölü bölgesi 45 px/s'den 120 px/s'ye dinamik genişletildi. Yakın mesafedeki sahte hız dalgalanmaları 358.3 px/s'den 70.2 px/s'ye (5 kat sakin) düşürüldü.
  8. **Takip Histerezisi & CONF HUD:** `target_selector.py` arama modunda `conf = 0.28`, kilitli takip modunda ise `conf = 0.16` dinamik toleransına geçirildi. Taret hızla dönerken ve hareket bulanıklığı oluştuğunda hedefin kopması engellendi; anlık güven skoru (% CONF) hem HUD retikülüne hem de CSV telemetrisine bağlandı.

---

## 3. "Neden Böyle Yaptık?" Arşivi (Jürinin Sorabileceği Kritik Savunmalar)

| Soru / Konu | Verilecek Bilimsel ve Mühendislik Cevabı |
|---|---|
| **Neden türev (D-termi) kullanmadınız?** | Kamera ve YOLO tespit gürültüsünün türevi servolarda yüksek frekanslı mekanik titreşime (chattering) ve dişli aşınmasına yol açar. STM32 pozisyon biriktiricisi ayrık zamanda saf integratör olduğu için yumuşak ölü bölgeli saf Tip-1 P kontrolcü statik hedeflerde hatayı ölü bölge sınırları içerisine (Pan ±0.85°, Tilt ±0.95°) asimptotik olarak çekip sıfır motor titreşimi sunar. Hareketli hedeflerdeki sonlu rampa hatası ise Kalman filtreli hız ileri beslemesi ve önleme terimiyle bertaraf edilir; hızlı manevra gerektiren durumlarda ise yapay zeka (RL) devreye girerek çevik takibi üstlenir. |
| **Neden integral (Ki) kullanmadınız?** | STM32 gömülü yazılımındaki pozisyon biriktiricisi (`theta_k = theta_{k-1} + delta`) ayrık zamanda saf integratördür (z/(z-1)). Harici Ki integrali sistemi Tip-2 yaparak çift kutuplu rezonans faz çökmesine ve kontrolsüz salınıma (limit cycle) sürükler. |
| **Neden ölü bölgeye yumuşak geçiş (Soft Deadband) koydunuz?** | Sert eşikli ölü bölge, eşik sınırında anlık basamak darbesi üreterek dişli boşluğuyla (backlash) birleştiğinde yüksek frekanslı mekanik titreşime yol açar. Yumuşak ölü bölge hatayı sıfırdan başlatarak sıfır jerk ile kalkış sağlar. |
| **Neden MPC veya LQR değil de PPO RL?** | Sistemde analitik olarak modellenmesi zor olan süreksiz doğrusal olmayanlıklar vardır (ölü bölge, 0.1° UART kuantalaması, dişli boşluğu, değişken 80 ms USB taşıma gecikmesi). PPO, bu rastgeleleştirilmiş POMDP ailesi üzerinde çevrimdışı eğitilmiş amortize bir optimal politika sunar; çalışma zamanında tek ileri geçiş maliyeti 0.17 ms'dir. |
| **Eylem geçmişi (Action History) neden gözlem uzayında?** | Smith Predictor ilkesine göre, 80 ms (yaklaşık 2-3 kare) gecikmeli bir sistemde ajan doğru karar verebilmek için motora gönderilmiş ama henüz kameraya yansımamış olan "yoldaki komutları" (in-flight commands) bilmek zorundadır. |
| **Atış anında neden basamak sıçraması olmuyor?** | 2 Aşamalı Atış Senkronizasyonu ve Zero-Lag Feedforward devrededir: Taret hedefi optik merkezde boresight takip eder. Y tuşuna basıldığında STM32 volan motorlarının nominal devrine (23.500 RPM) ulaşması için gereken 1.0 saniye (1000 ms, `spinup_time_s = 1.00`) içinde doğrudan önleme ileri beslemesi (`u = delta_slew + u_P`) ile taret balistik önleme ve süper-elevasyon açısına gecikmesiz ulaşır. 1.0 s sonra solenoid tetik darbesi (80 ms) uygulanır ve taret 350 ms içinde optik merkeze pürüzsüzce geri döner. EVAL ekranı 1.5 s gösterilirken ilk 0.75 s (22 kare) boyunca kilit muafiyeti (grace period) uygulanır; ani basamak sıçraması ve sahte kilit kaybı tamamen önlenir (fiziksel fırlatıcı yolu güvenlik gereği laboratuvar/HUD testlerinde simüle edilebilir). |

---

## 4. Temel Kod Dosyaları ve İşlev Haritası

- [config.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/config.py): SSOT (Single Source of Truth) merkezi konfigürasyon; optik, aktüatör, kontrolcü ve yol parametreleri.
- [main.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/main.py): Ana atış kontrol döngüsü, OpenCV video işleme, klavye kontrolleri.
- [telemetry.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/telemetry.py): v2 Telemetri kaydedicisi (40 kolon), oturum özeti (`.meta.json`) ve taktiksel olay günlüğü (`_events.csv`).
- [fcs_manager.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_manager.py): Durum makinesi (Arama, Tespit, Takip, Kilit, Ateşleme), balistik motoru, A/B mod yönetimi (`P` vs `RL`), menzil kapılama, 1. derece aktüatör gözlemcili ego-hareket telafisi (`tau = 45 ms`).
- [p_controller.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/p_controller.py): Yumuşak Ölü Bölgeli Tip-1 P motor kontrolcüsü (`Kp_pan=0.10, Kp_tilt=0.05`).
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
1. Sistemin şu anki durumunun **v1.3 Production Release** olduğunu,
2. Modelin yeni eğitilmiş LQR temelli `turret_ppo_final.zip` (+%59.0 takip iyileşmesi) olduğunu,
3. Kontrolcünün adının **Yumuşak Ölü Bölgeli Tip-1 P Kontrolcü** olduğunu,
4. Asla LaTeX / dolar işareti (`$`) kullanmaması gerektiğini,
5. Kodda veya tezde yapılacak her yeni geliştirmenin yukarıdaki mimari ve güvenlik disiplinine bağlı kalması gerektiğini anında bilir.
