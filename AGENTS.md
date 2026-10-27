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
  2. **Hedef Kayıp Toleransı Koruması:** `main.py` içinde hedefin 1 karelik geçici tespit kaçışında erken `fcs.reset()` çağrısı engellendi, `max_lost_frames=5` toleransı (`CONFIG.selector`) korundu.
  3. **Ego-Hareket Aktüatör Gözlemcisi İlklendirmesi:** `fcs_manager.py` içinde hedef yeni yakalandığında (`was_lost`) aktüatör gözlemcisinin sıfırdan başlaması sağlanarak önceki artık hareket komutlarından kaynaklanan sahte optik akış sıçraması engellendi.
  4. **Patrol Birikimli Adımlayıcı (Step Accumulator):** 80-100 FPS döngüde merkeze dönerken 0.20° gibi minik adımların MG996R ölü bölgesinde takılı kalmasını önlemek için `patrol_generator.py` içine birikimli adım mekanizması eklendi; komutlar birikip `0.95°` eşiğini aştığında taret 90-90 mekanik merkezine net torklu adımlarla döner hale getirildi.
  5. **Dinamik Kalman Hız Ölü Bölgesi (Dynamic Velocity Deadband):** Taret dönüş hızı (`ego_speed`) ve hedef mesafesine (`dist_m < 1.2m`) bağlı olarak Kalman hız ölü bölgesi 45 px/s'den 120 px/s'ye dinamik genişletildi. Yakın mesafedeki sahte hız dalgalanmaları 358.3 px/s'den 70.2 px/s'ye (5 kat sakin) düşürüldü. (FAZ 9'da kaldırıldı; yerine sabit 30-60 px/s yumuşak ölü bölge geldi.)
  6. **Takip Histerezisi & CONF HUD:** `target_selector.py` arama modunda `conf = 0.28`, kilitli takip modunda ise `conf = 0.16` dinamik toleransına geçirildi. Taret hızla dönerken ve hareket bulanıklığı oluştuğunda hedefin kopması engellendi; anlık güven skoru (% CONF) hem HUD retikülüne hem de CSV telemetrisine bağlandı.

### FAZ 9: Kalman Filtresinin Sıfırdan Yeniden Tasarımı (CV + Robust Tekil-Sıçrama Kapısı)

- **Sorun:** Adaptif Q tabanı `4 px/s^2` idi; sabit hızlı hedefte bile hız öğrenimi aşırı yavaştı. Dönüş anında ölçülen 3-kare eğim doğrudan duruma yazılıyor (state overwrite), ivme sınırı ve çıkış EMA'sı gecikmeyi büyütüyordu. Gerçek logda Kalman hızı dünya LOS hızıyla yalnızca 0.44 korelasyon ve %66 işaret uyumu veriyordu.
- **Yeni tasarım (tek katman):**
  1. Durum `[x, y, vx, vy]`, sabit hızlı (CV) model, tek süreç gürültüsü `sigma_a = 300 px/s^2`, tek ölçüm gürültüsü `r_var = 64 px^2`, Joseph formu.
  2. Adaptif Q, sigma artış hızı sınırı, dönüş dedektörü/P şişirme, ivme sınırı, 3-kare ölçüm eğimi, hız EMA'sı, yeniden yakalama beklemesi ve mesafe bazlı ölü bölge tamamen kaldırıldı.
  3. Tek karelik 150 px üstü bbox sıçramasında R 100 kat büyütülür (kazanç bastırılır). Aynı sapma 3 kare sürerse takip yeniden başlatılır: konum ölçümden, hız sıfırdan.
  4. Ego kaynağı: son motor komutunun 30 ms birinci derece servo yanıtı. STM potansiyometre farkı ego için kullanılmaz; 0.1° kuantalama ve 1.9°'ye varan sıçramalar kare farkında büyük sahte hız üretir.
  5. Lead: `bbox + hız * lead_time`. Filtrenin konum gecikmesi lead'e ikinci kez eklenmez.
  6. Çıkışta yalnızca 30-60 px/s yumuşak ölü bölge vardır.
- **Doğrulama:**
  - Gerçek log `telemetry_test_20261024_035501.csv`: dünya LOS referansına karşı korelasyon 0.44 -> 0.83, işaret uyumu %66 -> %77, 120 px/s üstü hızlarda %69 -> %92. İkinci logda korelasyon 0.57 -> 0.81.
  - Webots zemin gerçeği: işaret uyumu %87-92, hız büyüklüğü kazancı eski filtreye göre belirgin şekilde arttı; 300 px/s^2 Q değeri 400/500/800'e göre daha iyi RMSE ve işaret verdi.
  - Test paketi 41 -> 46 test; tümü geçiyor.
- **Sınır:** P FF hâlâ kapalı (`velocity_ff_enabled=False`). Kalman hızı P motorunu sürmüyor; yalnızca lead/HUD ve telemetri için kullanılıyor; RL gözlemine runtime'da sıfır verilir.

### FAZ 10: P Kontrolcünün Yeniden Tasarımı (P) ve Webots Doğrulaması

- **Sorun:** Eski P aynı açıyı iki kez EMA ile filtreliyor (`compute_angles` + `compute_p`), ardından pan çıkışına sabit `0.20 derece/kare` slew koyuyordu. 30 FPS'de bu 6 derece/s yapay hız tavanıdır; hedef 10-12 derece/s hareket ettiğinde P matematiksel olarak yetişemez. Sabit Kp de büyük hatada yavaş, merkezde gereğinden sertti.
- **Yeni varsayılan:** `turret_fcs/p_controller.py` içindeki `PController`.
  1. Tek EMA (`alpha = 0.85`); çift filtreleme kaldırıldı.
  2. Pan `Kp = 0.22`, tilt `Kp = 0.11` (30 FPS bazında). Hata 5 dereceyi aştıkça Kp yumuşak geçişle 1.6 kata kadar artar. (Webots taraması 0.30'u en iyi gösterdi ama **gerçek tarete 0.30 rezonansa girdi**; saha geri bildirimiyle 0.22/0.11'e çekildi.)
  3. Sabit pan slew yerine komut ivme sınırı: pan 120, tilt 80 derece/s^2. Steady-state hız tavanı yok; kalkış/dönüş yumuşak.
  4. Ki, Kd ve hız FF yok; yumuşak ölü bölge (0.85/0.95 derece) korunur.
  5. Tek kontrolcü yolu vardır: `turret_fcs/p_controller.py` içindeki `PController`. Seçici, ortam değişkeni ve eski kontrolcü v2.0'de kaldırıldı.
- **Webots kazanç taraması (6 profil x 8 kazanç, `analysis/p_profile_sweep.py`):**

  | Profil | 0.20 | 0.24 | 0.28 | 0.30 | 0.32 | 0.34 | 0.36 | 0.38 |
  |---|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
  | Standart sinüs (2) | ✅ | – | ✅ | ✅ | ✅ | – | – | – |
  | Figure-8 (7) | ✅ | ✅ | ✅ | ✅ | ✅ | – | – | – |
  | Hızlı sinüs (3) | ✅ | ✅ | ✅ | ✅ | ✅ | – | – | – |
  | Zigzag (5) | ✅ | ✅ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ |
  | Step/hold (4) | ⚠️ | – | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |

  - **Metodolojik Not (Tez ve Karşılaştırma):** Bu tarama tablosu, önceki kazanç planlaması (`far_gain = 1.6`) ve eski ivme sınırı (120/80 derece/s^2) koşullarında ölçülmüştür. FAZ 14 ile birlikte kazanç planlaması tamamen kaldırılmış ve ivme sınırları fiziksel servo kabiliyetine (pan 400.0, tilt 267.0 derece/s^2) güncellenmiştir. Tez çalışmasında güncel doğrusal Tip-1 P mimarisi sınırları kullanılacaksa [p_profile_sweep.py](file:///c:/YTU/bitirme_projesi/turret_fcs/analysis/p_profile_sweep.py) yeni parametrelerle yeniden koşulabilir.
  - Kararlılık ölçütü: komut yön değiştirme oranı ve doyum (komut adımının 1.35 dereceyi aşma yüzdesi). Limit cycle'da yön değiştirme %8'e, doyum %50'ye çıkıyor; kararlı koşularda %0.8-1.9.
  - Standart sinüste sınır 0.35-0.39 arasında. Zigzag'de 0.27 hâlâ %89.6 kilit, 0.30 sınır, 0.34 çöküş. Step/hold'da 0.20 bile %25 doyumda, 0.30'da kilit %0.
  - **Tek bir evrensel Kp yok.** Taban 0.22/0.11; `webots_tuning.py` profil bazlı tavan uygular (standart/figure-8 0.32, zigzag 0.27, hızlı sinüs 0.25, step/hold 0.15).
  - Tilt: 0.13, 0.10'a göre tilt hatasını 1.83 -> 1.76 dereceye düşürüp kilidi artırdı.
- **TurretEnv kapalı çevrim karşılaştırması (20 tohum, randomization açık):** ortalama hata 109 -> 66 px, RMS 121 -> 82 px, kilit oranı %0.9 -> %14.2; komut jerk'i 0.022 -> 0.053.
- **Test paketi:** 46 -> 67 -> 33 test (konsolide edildi).
- **Saha dersi (kritik):** Webots'un 1.2 derece backlash modeli, gerçek dişli boşluğu + potansiyometre gürültüsünü tam temsil etmiyor. Simülasyonda kararlı olan kazanç, saha için üst sınır değildir. 0.30 pan kazancı gerçek tarete rezonansa girdi; 0.22/0.11'e inildi. Simülasyondan seçilen değeri sahada muhafazakâr başlangıç kabul et ve gerçek testle doğrula.
- **Tuning güvenliği:** tarama limitleri sessizce kırpmayacak şekilde gevşetildi ve kırpma olursa `[TUNE UYARI]` basılıyor; profil bazlı `PROFILE_LIMITS` tablosu var. Bu, 0.28-0.42 değerlerinin yanlışlıkla 0.20'ye kırpılması hatasını önler.

### FAZ 11: RL Sarmalayıcı ve P-Harman Denemesi (v1.3, tarihsel)

- **Sorun:** Son RL kontrolcü gerçek/Webots kapalı çevrimde çok mikro düzeltme ve çoklu eksende titreme üretiyordu. Ölçüm: saf politika Webots profil 2'de pan RMS 3.26 derece, komut yön değiştirme %6.0, tilt hatası 2.05 derece, kilit %29.
- **Kök nedenler (ölçüldü, tahmin değil):**
  1. Eski `RLController` gözlemi P'nin EMA'li hatasından üretiyordu; politika eğitimdeki ölçülen bbox dağılımını görmüyordu.
  2. Eylem geçmişine filtrelenmiş motor komutunu yazıyordu; politika eğitimde kendi ham aksiyonunu görüyordu.
  3. Çıkışta EMA + 0.10 derece/kare slew + 0.85 derece yazılım ölü bölgesi vardı; politika bunlarla eğitilmediği için faz gecikmesi ve limit cycle oluşuyordu.
  4. Hedef kaybında eylem geçmişi temizlenmiyordu; yeniden yakalamada hayalet aksiyonlar politikaya giriyordu.
  - TurretEnv teşhisi (aynı 12 tohum): ham politika 40.5 px RMS, eski sarmalayıcı 67.1 px, filtresiz temiz sarmalayıcı 40.2 px. Yani sorun politika değil, sarmalayıcıydı.
- **Yeni sarmalayıcı:** `turret_fcs/rl_controller.py` içindeki `RLController`.
  1. 11 boyutlu gözlem eğitimle aynı; gözlem ölçülen bbox'tan üretilir; hız bileşenleri runtime'da sıfırdır (FAZ 14).
  2. Politika ağırlıkları `turret_ppo_v14_policy.npz` olarak dışa aktarılır; çıkarım SB3/torch sürümünden bağımsız 11-64-64-2 NumPy MLP ile yapılır.
  3. Eylem geçmişi uygulanan normalize komutu saklar; reset ve hedef kaybında temizlenir.
  4. EMA/slew yok. Açısal yumuşak ölü bölge (pan 0.85°, tilt 0.95° ve Hermite taper geçişi; politikanın sıfır-hata bias'ını ve dişli boşluğu rezonansını bastırır; `soft_deadband_enabled` bayrağıyla tetiklenir) ve yön değiştiren aksiyonda 0.5 sönümleme var.
  5. v1.3'te son komut P ile harmanlandı: `u = 0.3 * u_RL + 0.7 * u_P`. Bu harman v2.0'de kaldırıldı; RL artık saf PPO aksiyonu kullanır.
  6. Tek RL yolu vardır: `turret_fcs/rl_controller.py` içindeki `RLController`. Seçici ve eski sarmalayıcı v2.0'de kaldırıldı.
- **Webots doğrulaması (profil 2, ilk 600 kare):**

  | Kontrolcü | Pan RMS | Piksel | Kilit | Kilitli kalma | Yön değiştirme | Tilt hatası |
  |---|---:|---:|---:|---:|---:|---:|
  | Saf RL (filtresiz) | 3.26° | 65.4 px | %29.0 | %13.0 | %6.0 | 2.05° |
  | **RL (v1.3 P harmanı)** | **2.04°** | **42.7 px** | **%37.7** | **%95.0** | **%1.9** | **0.67°** |
  | P (0.22) | 2.07° | 42.3 px | %39.8 | %82.2 | %0.9 | 1.34° |

  - RL, P ile aynı pan/piksel hatasını verirken tilt hatasını 1.34 -> 0.67 dereceye düşürür, kilitli kalma süresini %82 -> %95'e çıkarır; komut yön değiştirme %0.9 -> %1.9'a çıkar ama hâlâ kararlı banttadır.
  - v1.3'te saf RL bu plantta kararsızdı; bu yüzden P harmanı kullanıldı. v2.0'de plant/ödül/hız gözlemi düzeltilerek saf RL çalıştırıldı (FAZ 13).
- **Negatif sonuç (dürüstçe):** 1.2 derece backlash ve yüksek jerk cezasıyla iki kez yeniden eğitim denendi. İlki satürasyona (RMS 439 px), ikincisi sabit maksimum komuta takıldı; yeni modeller kullanılmadı ve silindi. Sorun politika/plant uyumsuzluğu; sarmalayıcı tek başına çözemiyor.
- **Test paketi:** 67 -> 33 test. Aynı işlevi gören testler birleştirildi; ölü FF testleri ve eski RL smoothing testleri kaldırıldı.

### FAZ 12: Repo Geneli Denetim, Temizlik ve Son Mimari

- **Kapsam:** P kontrolcü ve Kalman filtresi hariç tüm Python, Webots, STM32 ve analiz katmanları.
- **Düzeltilen gerçek hatalar:**
  1. `TargetSelector.is_tracking` kısa hedef kaybında hemen false oluyordu; 0.16 takip eşiği devre dışı kalıyordu. Artık `lost_counter <= max_lost_frames` boyunca histerezis korunur. Ayrıca ByteTrack ID atlaması mekânsal kapı ile reddedilir.
  2. `FCSCommunicator` 10 baytlık paketi tek `read` çağrısında bekliyordu; bölünmüş paketler kayboluyordu. Artık kalıcı RX tamponu, yeniden senkronizasyon ve checksum döngüsü vardır. Sıfır komutta yalnız bir eksenin artığı sıfırlanıyordu; artık her eksen bağımsız sıfırlanır. Açıkça verilen port, ST-Link otomatik seçiminden önce denenir.
  3. `CameraThread` "Logi C270" arıyordu; gerçek cihaz adı "Logitech C270" olduğu için eşleşmiyordu. "C270" alt dizesi kullanılır.
  4. Webots tuning varsayılanları 0.11/0.06 idi; bu, `tuning_params.json` yoksa P'i sessizce legacy kazanca düşürüyordu. Varsayılanlar artık `CONFIG.control.kp_pan/kp_tilt` (0.22/0.11).
  5. HUD spin-up sayacı 1.0 s sabitini kullanıyordu; gerçek 0.25 s (idle) veya 1.0 s (cold) süresi artık parametre olarak geçirilir. SETTLE aşaması da ayrı gösterilir.
  6. Üretim ve Webots telemetrisi gerçek kamera açısal hatası yerine P kontrolcünün filtrelenmiş `actual_pan/tilt` değerini yazıyordu. Artık `last_pan_err_deg/last_tilt_err_deg` kullanılır. `latency_ms` dt değil, gerçek döngü işleme süresidir.
  7. Webots telemetrisi hedef güvenini 0.90 sabitine, hit_prob'u 0-1 ölçeğine yazıyordu; artık gerçek `target_conf` ve yüzde `hit_probability` yazılır.
  8. Firmware'de kullanılmayan `FCS_Comm_IsNewCommand/GetPan/GetTilt/GetState`, `Luna_IsDataReady` ve çift tersleme yapan `PAN_DIR_INVERT/TILT_DIR_INVERT` kaldırıldı.
  9. `patrol` dokümanındaki ±35/±8 değerleri kodla (±45/±16) uyumsuzdu; doküman düzeltildi.
- **Kaldırılanlar:** Eski `legacy/hsv` C++ prototipi, boş pip geçici dizinleri, `.pytest_cache`, `egg-info`, kullanılmayan Webots analiz betikleri/görselleri, YOLO eğitim eğrileri, eski mimari PNG'leri. GitHub'da `back_up` etiketi ve iki yerel yedek dal silindi; v1.0-v1.3 release etiketleri korundu.
- **Geriye dönüklük kararı (v2.0'de güncellendi):** Eski P/RL kontrolcüleri, factory seçicileri ve ortam değişkenleri v2.0'de tamamen silindi. Tek yol `PController` ve `RLController`'dır; v1.3 davranışı için `v1.3` etiketi kullanılır.

### FAZ 13: Saf RL Kurtarma, Tek Kontrolcü Mimarisi ve v2.0 Sürümü

- **Amaç:** P harmanı olmadan, yalnızca PPO aksiyonuyla Webots ve gerçek donanım fiziğine yakın çalışan bir RL kontrolcü.
- **Yapılan plant düzeltmeleri:**
  1. `servo_deadband` 0.10 derece olarak korundu. 1.20 dereceyi deadband yapmak yanlıştı; 1.50 derece/kare adımı tamamen yutuyordu.
  2. 1.20 derece ayrı `backlash_deg` olarak modellendi: komut biriktirici (`cmd += step`), sonra backlash (`target - 1.20`). Webots'taki modelle aynı.
  3. `max_step_pan/tilt` 0.80/0.50'den 1.50/1.50'ye çıkarıldı. Webots motor hızı 1.35 derece/kare yerine 1.50 derece/kare (30 FPS'te 45 derece/s) yapıldı.
  4. Ödül progress formuna çevrildi: `r = 2.0*(e_prev^2 - e^2) - 0.05*e^2 - 0.02*|u|^2 - 0.08*|du|^2 + bullseye`.
  5. Eğitimde gerçek hız gözlemi kullanıldı; runtime'da Kalman hızı sıfırlandı. Webots/gerçek Kalman hızı gürültülü olduğu için politika onunla doyuma giriyordu. Statik hedefte sıfır hız eğitim dağılımında olduğu için güvenli.
  6. `blend_with_p` tamamen kaldırıldı; RL saf PPO aksiyonu üretir.
- **Eğitim:** 250k adım, PPO MlpPolicy, `turret_ppo_v14.zip` ve `turret_ppo_v14_policy.npz` (SB3/torch bağımsız NumPy çıkarım).
- **Webots sonuçları (ilk 600 kare):**

  | Profil | Kontrolcü | Pan RMS | Piksel | Kilit | Kilitli kalma | Yön değiştirme | Tilt hatası |
  |---|---|---:|---:|---:|---:|---:|---:|
  | 2 standart | RL saf PPO | 1.93° | 40.6 px | %36.3 | %92.5 | %4.4 | 0.78° |
  | 2 standart | P 0.22 | 2.07° | 42.3 px | %39.8 | %82.2 | %0.9 | 1.34° |
  | 5 zigzag | RL saf PPO | 2.34° | 42.9 px | %47.0 | %94.2 | %5.1 | 0.78° |
  | 5 zigzag | P 0.22 | 2.66° | 49.0 px | %39.5 | %80.8 | %1.6 | 1.16° |
  | 7 figure-8 | RL saf PPO | 2.06° | 43.6 px | %35.0 | %93.8 | %4.5 | 0.86° |
  | 7 figure-8 | P 0.22 | 2.14° | 43.8 px | %38.8 | %65.2 | %0.8 | 1.87° |

  - Saf RL üç profilde de P'a göre daha düşük pan RMS, piksel hatası ve tilt hatası; kilitli kalma süresi belirgin yüksek. Yön değiştirme %4.4-5.1 ile P'nin üstünde ama limit-cycle eşiği %8'in altında.
- **Sınır:** Bu sonuçlar Webots'tadır; gerçek taret testi yapılmadı. Yön değiştirme oranı P'den yüksek olduğu için sahada uzun süreli aşınma izlenmelidir.
- **Kontrolcü sadeleştirmesi:** P MAX üretim `p_controller.py` (`PController`), RL MAX üretim `rl_controller.py` (`RLController`) oldu. Eski kontrolcüler, factory seçicileri, `TURRET_P_CONTROLLER` / `TURRET_RL_CONTROLLER` ortam değişkenleri, `p_controller_impl` / `rl_controller_impl` alanları, `kp_pan_max` / `kp_tilt_max` takma adları ve v1.3 `turret_ppo_final` modeli silindi. Hiç okunmayan FF/EMA config alanları da kaldırıldı.
- **Kurulum düzeltmesi:** `requirements.txt` çözülemiyordu (SB3 2.3.2 + gymnasium 1.0.0 + numpy 2.1.3 çelişkisi). Yeni pinler: numpy 1.26.4, stable-baselines3 2.4.1, gymnasium 1.0.0. `setup_env.bat` uygun Python'u seçiyor, bozuk venv'i yeniden kuruyor, CPU torch wheel kuruyor ve importları doğruluyor. Temiz klonda 28/28 test ve YOLO çıkarımı doğrulandı.
- **RL çıkış katmanı düzeltmesi (v2.0):** `rl_controller.py` çıkışında yumuşak ölü bölge ölçeği (taper) ve ivme sınırı zaten vardı, ama `if self.deadband_px > 0.0` kapısı (sonradan `soft_deadband_enabled` bayrağına çevrildi) yüzünden hiç devreye girmiyordu. Ölçek + ivme sınırı (120/80 derece/s^2) devreye alındı, `fcs_manager` ivme sınırını RL'e geçirir, `max_step_tilt_deg` 1.50 -> 0.75 yapıldı. Gerçek log: kilit %1.5 -> %47.4, pan RMS 5.80 -> 4.23, merkezde komut 0.531 -> 0.021, median hata 5.47 -> 2.03.
- **SSOT sözleşmesi (v2.0):** `RLConfig` eklendi (ölü bölge, taper, hız/mesafe ölçekleri, YOLO gürültüsü, servo tau aralığı, bölüm uzunluğu); `far_gain`, `far_error_deg`, `cmd_accel_*`, `lead_time_*`, `eval_display_time_s` ControlConfig/BallisticsConfig'e taşındı. P, RL, TurretEnv, FCS lead ve Webots EVAL süresi artık bu alanları okur. Kullanılmayan `lidar.gate_angle_deg` ve ölü `DEFAULT_LEAD_TIME` silindi. `tests/test_config_consistency.py` her CONFIG alanının okunduğunu ve varsayılanların bağlı olduğunu doğrular.
- **Firmware sertlestirme (v2.0):** STM32 tarafinda alti gercek acik kapatildi.
  1. Bagimsiz watchdog (IWDG, ~800 ms) eklendi. STM32Cube HAL IWDG surucusu kullanilir: stm32f4xx_hal_iwdg.c/.h projeye eklendi, HAL_IWDG_MODULE_ENABLED acildi, MX_IWDG_Init() (prescaler 64, reload 400) ve dongu basina HAL_IWDG_Refresh() eklendi; IWDG .ioc dosyasina da islendi (CubeMX yeniden uretiminde silinmesin). Yazilim kilitlenirse MCU reset atar ve pinler tekrar LOW'a cekilir; onceden volan sonsuza kadar donmeye devam ediyordu. DEBUG derlemesinde __HAL_DBGMCU_FREEZE_IWDG() ile breakpoint sirasinda IWDG dondurulur.
  2. E-stop artik basamaksiz degil: dogrudan Servo_SetPan(90) yazimi MAX_STEP korumasini atliyor ve 45 dereceye varan ani tork soku uretiyordu. Yerine hiz sinirli Servo_HomeStep() geldi (45 derece/s).
  3. state bayti 0-3 araliginda dogrulaniyor; E-stop yalnizca ust uste iki tutarli pakette kabul ediliyor. Tek bozulmus paket tareti merkeze firlatamiyor.
  4. Zaman tabanli hiz siniri: PC kare basina delta gonderir; firmware gercek dt'yi olcup 45 derece/s ile sinirlar. PC FPS'i artarsa taret hizi degismez (onceden 60 FPS'te 90 derece/s oluyordu).
  5. 2000 ms servo park korumasi: PC kalici olarak yoksa servolar merkeze indirilir; onceden iki MG996R son konumda surekli stall akimi cekiyordu.
  6. Launcher_TriggerFire() volan hazir degilse atisi reddedip false donuyor; LAUNCHER_STATE_SPINUP ve FLYWHEEL_SPINUP_MS kaldirildi. Onceden sessizce 500 ms bekliyor, PC ise FIRING fazini 80 ms tuttugu icin dart gec ve yanlis yonde firlatilabiliyordu.
- **Dogrulama:** Firmware arm-none-eabi-gcc 14.3 ile sifir uyarida derlendi ve linklendi (Debug text ~18 KB / 18048 B, Release text ~10 KB / 9960 B, data 20 B, bss 1948 B); protokol PC tarafiyla degismedi, test paketi konsolide ve tam kapsamlı.
- **Sürüm:** Tüm v1.3 sonrasi degisiklikler v2.0'de toplandi; test paketi konsolide 28 test.

### FAZ 14: Algoritmik Kararlılık, 150 px Robust Tekil-Sıçrama Kapısı ve Dinamik İvme İyileştirmesi

- **Kapsam:** P Kontrolcü, Kalman Filtresi, RL Kontrolcü ve Sistem Entegrasyonu.
- **Yapılan İyileştirmeler:**
  1. **P Kontrolcü Kazanç Planlaması Kaldırıldı:** Büyük açılarda Kp'yi 1.6 katına çıkaran yapay kazanç planlaması (`far_gain`, `far_error_deg`) tamamen kaldırıldı; sistem saf, öngörülebilir ve doğrusal Tip-1 P mimarisine kavuştu (`Kp_pan = 0.22`, `Kp_tilt = 0.11`).
  2. **İvme Sınırı 400.0 / 267.0 derece/s^2 Seviyesine Yükseltildi:** Önceki 120 / 80 derece/s^2 sınırı adımı 11.3 karede (375 ms) doyuma ulaştırıyor ve kapalı çevrimde gecikmeye (+%11 P RMS, +%21 RL RMS) neden oluyordu. 400.0 (Pan) ve 267.0 (Tilt) derece/s^2 sınırına çekilerek hem ani tork darbesi önlendi hem de tork doyuma ulaşma süresi 113 ms'ye (~2.5 tau) indirilerek sistemin çevikliği korundu.
  3. **Hedef Kaybında Komut Belleği Sıfırlaması (`on_target_lost`):** Tip-1 sistemde a * dt^2 ivme sınırı uygulanırken hedef kaybolduğunda `smooth_cmd` sıfırlanmadığı için zıt yönde yeni beliren bir hedefe ilk 7 kare boyunca ters yöne komut verme regresyonu kapatıldı. `fcs_manager` hedef kaybında `p_controller.on_target_lost()` ve `rl_controller.on_target_lost()` çağırarak bellek artıklarını temizler.
  4. **Kalman 150 px Robust Tekil-Sıçrama Kapısı (NIS Yerine):** Gerçek saha telemetri verisi (15:08 ve 12:17 oturumları) üzerinde yapılan analizde, taretin normal dinamik dönüşlerinde inovasyonun 30-70 px civarında olduğu; inovasyon kovaryansı S (~9.2 px) çok dar kaldığı için teorik Ki-Kare NIS kapısının (d^2 > 13.82) normal manevraları hatalı şekilde "aykırı değer" sayarak 5400 karede 302 kez sahte reset attığı tespit edildi. NIS kapısı tamamen kaldırılarak sahada kanıtlanmış fiziksel 150 px inovasyon artık kapısına dönüldü (`residual > 150 px`). Tekil sıçramada ölçüm kovaryansı R * 100 yapılarak sönümlenir, 3 kare üst üste sapmada takip sıfırdan başlatılır. Bu sayede gerçek veride hız korelasyonu 0.77'den 0.969'a çıkmış, hız RMSE'si 221 px/s'den 81 px/s'ye inmiş ve sahte reset sayısı 0'a düşmüştür.
  5. **Ego-Hareket Ayrıklaştırma Uyumu:** `fcs_manager.py` içindeki analitik 1 - exp(-dt/tau) formülü, aktüatör modelinin geri Euler ayrıklaştırmasıyla (`dt / (tau + dt)`) eşitlendi; ilk karedeki %23'lük ego aşımı sıfırlandı (`tau = 0.045 s`).
  6. **Lead Süresi ve Tilt Limiti Uyumu:** Balistik lead süresi tavanı 0.25 s'den 0.40 s'ye çıkarıldı (3.85 metreye kadar kesintisiz balistik lead). Tilt adımı tavanı tüm kontrolcülerde (P, RL, FCS) 0.75 derece olarak eşitlendi.
  7. **RL Çalışma Zamanı Hız Gözleminin Sıfırlanması ve Parametre Temizliği:** Kapalı çevrimde (Webots ve gerçek donanım) Kalman hızının ego-hareket faz gecikmesi sebebiyle motorda pozitif geri besleme ve avlanma/osilasyon (limit cycle) ürettiği tespit edilerek, RL çalışma zamanı hız gözlemi `fcs_manager` içinde `0.0, 0.0` olarak sabitlendi (P kontrolcünün Kalman hızını motora vermemesiyle aynı mimari ilke). Politika hedefin dinamik hareketini 11 boyutlu durum uzayındaki konum hatası, hata türevi (`d_err`) ve 2 adımlık eylem geçmişiyle stabil ve sönümlü şekilde karşılar. Kullanılmayan ölü config parametreleri (`deadband_exit_px`, `velocity_gate_px_s`, `far_gain`, `far_error_deg`) ve ölü bayraklar (`uses_velocity_obs`, `prefers_smooth_velocity`, `uses_measured_bbox`) silindi.
  8. **Gereksiz Dosya Temizliği:** Eski MATLAB analiz betiği (`analysis/turret_fcs_analysis.m`) ve güncelliğini yitirmiş `docs/evidence/` klasörü silindi; sistem analiz grafiği `analysis/figures/` altına taşındı.

---

### FAZ 15: v2.1 Sürümü — Kalman Aday Yörünge Geri Oynatma, Kısa Kayıp Tahmin Köprüsü ve UART Kayan Pencere

- **Kapsam:** Kalman Filtresi, FCS Durum Makinesi, P Kontrolcü, Hedef Seçici, STM32 UART Alıcısı, RL Simülasyon Altyapısı ve SSOT/CI.
- **Yapılan İyileştirmeler:**
  1. **Kalman Aday Yörünge Geri Oynatma (`_outlier_history`):** 150 px üstü tek karelik sahte YOLO sıçramalarında durum ölçümle bozulmadan salt tahminle (`x_pred`, `P_pred`) korunur; sapma kendi içinde tutarlı şekilde 3 kare (`reacquire_frames`) sürerse yeni hedef sıçraması kabul edilerek ilk kareden itibaren biriktirilen aday ölçümler ileri sarılarak (`_predict_state` + `_correct_state`) hız vektörü sıfırlanmadan anında öğrenilir.
  2. **Kısa Hedef Kaybında Ego-Telafili Kalman Tahmin Köprüsü:** Hedef 1-5 kare (`max_lost_frames`) boyunca anlık olarak YOLO tespitinden kaçtığında, `FCSManager` ego-hareket piksel kaymasını hesaplamaya ve `tracker.predict_only(dt, ego_dx, ego_dy)` ile hedef konumunu ilerletmeye devam eder; kontrolcü bu kısa boşlukta tahmin edilen yörüngeyi sürdürür.
  3. **LiDAR İlk Ölçüm Sıçrama Muafiyeti:** İlk geçerli LiDAR ölçümü geldiğinde varsayılan 2.0 m başlangıç değerinden farkın sahte sıçrama (`range_jump`) sayılması engellendi (`bool(self._range_history)` kontrolü).
  4. **P Kontrolcü FPS-Bağımsız EMA:** `compute_angles` içindeki açısal EMA filtresi anlık `dt` süresine göre sürekli-zaman eşdeğeriyle (`alpha_eff = 1.0 - (1.0 - alpha) ** (dt / nominal_dt_s)`) ölçeklenir.
  5. **Güven Ağırlıklı Hedef Seçimi:** `TargetSelector` içinde mekânsal kapı ve merkez önceliklendirmesi güven skoruyla ağırlıklandırıldı (`dist_sq / max(conf, 0.1)`).
  6. **STM32 UART Kayan Pencere (Sliding Window) Senkronizasyonu:** `fcs_comm.c` içinde checksum hatası durumunda tampon tamamen sıfırlanmak yerine bir sonraki `0xAA` başlığına kaydırılarak paket kaybı önlendi.
  7. **RL ve P Kontrolcü Çok-Değişkenli Bayesyen (Optuna TPE) Optimizasyonu ve Canlı Doğrulama:** `turret_env.py` yüksek sadakatli aktüatör/kuantalama (`0.10 derece`), 2. derece gövde eylemsizliği ve zaman korelasyonlu YOLO titreşimi ile güncellendi. Rezonans korumalı (kareselli `flip_rate` ve `jerk` cezalı) Optuna TPE aramasıyla:
     - **PPO RL Kontrolcüsü (`turret_ppo_v14.zip` / `.npz`):** 14x150k TPE denemesi (Deneme #11: `lambda_u = 0.019262`, `mu_du = 0.005215`, `learning_rate = 0.000155`) sonrası 250.000 adım eğitilerek jerk %23.3 düşürüldü ve canlı donanımda doğrulandı.
     - **Tip-1 P Kontrolcüsü (`PController`):** 100 TPE denemesi x 30 tohum (Deneme #60: `kp_pan = 0.1953`, `kp_tilt = 0.1544`, `alpha_filter = 0.9306`, `cmd_accel_pan_deg_s2 = 353.1`, `cmd_accel_tilt_deg_s2 = 213.6`) ile kilitlenme oranı 2.6 kat artırıldı, yön değiştirme (`flip_rate`) düşürüldü ve canlı donanımda doğrulandı.
     - Tüm SSOT sabitleri `CONFIG` üzerinden bağlandı ve sürüm `v2.1` olarak etiketlendi.

---

## 3. "Neden Böyle Yaptık?" Arşivi (Jürinin Sorabileceği Kritik Savunmalar)

| Soru / Konu | Verilecek Bilimsel ve Mühendislik Cevabı |
|---|---|
| **Neden türev (D-termi) kullanmadınız?** | Kamera ve YOLO tespit gürültüsünün türevi servolarda yüksek frekanslı mekanik titreşime (chattering) ve dişli aşınmasına yol açar. STM32 pozisyon biriktiricisi ayrık zamanda saf integratör olduğu için yumuşak ölü bölgeli saf Tip-1 P kontrolcü statik hedeflerde hatayı ölü bölge sınırları içerisine (Pan ±0.85°, Tilt ±0.95°) asimptotik olarak çekip sıfır motor titreşimi sunar. Hareketli hedeflerdeki sonlu rampa hatası ise Kalman filtreli lead/önleme terimiyle (P hız FF deneyi kapalı) bertaraf edilir; hızlı manevra gerektiren durumlarda ise yapay zeka (RL) devreye girerek çevik takibi üstlenir. |
| **Neden integral (Ki) kullanmadınız?** | STM32 gömülü yazılımındaki pozisyon biriktiricisi (`theta_k = theta_{k-1} + delta`) ayrık zamanda saf integratördür (z/(z-1)). Harici Ki integrali sistemi Tip-2 yaparak çift kutuplu rezonans faz çökmesine ve kontrolsüz salınıma (limit cycle) sürükler. |
| **Neden ölü bölgeye yumuşak geçiş (Soft Deadband) koydunuz?** | Sert eşikli ölü bölge, eşik sınırında anlık basamak darbesi üreterek dişli boşluğuyla (backlash) birleştiğinde yüksek frekanslı mekanik titreşime yol açar. Yumuşak ölü bölge hatayı sıfırdan başlatarak sıfır jerk ile kalkış sağlar. |
| **Neden MPC veya LQR değil de PPO RL?** | Sistemde analitik olarak modellenmesi zor olan süreksiz doğrusal olmayanlıklar vardır (ölü bölge, 0.1° UART kuantalaması, dişli boşluğu, değişken 80 ms USB taşıma gecikmesi). PPO, bu rastgeleleştirilmiş POMDP ailesi üzerinde çevrimdışı eğitilmiş amortize bir optimal politika sunar; çalışma zamanında tek ileri geçiş maliyeti 0.17 ms'dir. |
| **Eylem geçmişi (Action History) neden gözlem uzayında?** | Smith Predictor ilkesine göre, 80 ms (yaklaşık 2-3 kare) gecikmeli bir sistemde ajan doğru karar verebilmek için motora gönderilmiş ama henüz kameraya yansımamış olan "yoldaki komutları" (in-flight commands) bilmek zorundadır. |
| **Neden teorik Ki-Kare NIS kapısı yerine fiziksel 150 px artık kapısı kullandınız? (Saha verisinde gördünüz mü?)** | Evet, bizzat 15:08 oturumuna ait 5400 karelik gerçek saha telemetrisinde test edilmiştir. Taret hedefi yakalamak için hızla döndüğünde inovasyonlar 30-70 px seviyesine çıkmaktadır; fakat CV modelinde inovasyon kovaryansı S standart sapması yaklaşık 9.2 px kaldığı için normalize inovasyon karesi d^2 = (30/9.2)^2 ... (70/9.2)^2 = 10 ... 58 bandına fırlamakta ve 13.82 eşiğini aşmaktadır. Teorik NIS kapısı normal taret dönüşlerini aykırı değer sayarak 5400 karede 302 kez (her 18 karede bir) filtreyi sıfırlamış, hız korelasyonu 0.969'dan 0.774'e düşmüş ve RMSE 81'den 221 px/s'ye fırlamıştır. Bu saha kanıtı üzerine teorik NIS kapısı terk edilmiş; normal dinamik dönüşü engellemeyen fakat sahte YOLO sıçramalarını R * 100 ile sönümleyen fiziksel 150 px artık kapısı seçilmiştir (sahte reset: 0, korelasyon: 0.969). |
| **RL kontrolcüye çalışma zamanında neden harici Kalman hızı verilmiyor (0.0 veriliyor)?** | Kapalı çevrimde taret hızla dönerken aktüatörün zaman sabiti ve iletişim gecikmesinden ötürü ego-hareket çıkartmasında 1-2 karelik faz gecikmesi kalır. Kalman filtresi bu gecikmeyi hedef aniden yön değiştirmiş gibi algılayarak sahte hız üretir. Bu hız RL politikasına verildiğinde taret daha sert dönerek pozitif geri besleme döngüsüne (limit cycle osilasyonu) girer. Kalman filtresi projenin temel prensibi olarak yalnızca balistik önleme (lead) ve HUD/telemetri içindir; motor döngüsünü sürmez (P kontrolcüde de velocity feedforward kapalıdır). Politika kendi 11 boyutlu gözleminde hata türevi (`d_err`) ve 2 adımlık Smith Predictor eylem geçmişine (`a_{t-1}, a_{t-2}`) sahip olduğu için hız sıfırlandığında osilasyonsuz, pürüzsüz ve stabil kilitlenme sağlar. |
| **Atış anında neden basamak sıçraması olmuyor?** | Kademeli atış senkronizasyonu ve Zero-Lag Feedforward devrededir: Hedef kilitliyken volanlar boşta döner. Y tuşundan sonra 0.25 s SPINUP/SLEW ile taret balistik önleme ve süper-elevasyon açısına gider; ardından 0.15 s SETTLE ile komut sabit tutulup servo lag/backlash oturur (lead/gravity canlı kalır). Sonra 80 ms solenoid darbesi uygulanır ve taret 350 ms içinde optik merkeze döner. EVAL 1.5 s gösterilirken ilk 0.75 s (22 kare) kilit muafiyeti uygulanır. Fiziksel fırlatıcı yolu güvenlik gereği laboratuvar/HUD testlerinde simüle edilebilir. |

---

## 4. Temel Kod Dosyaları ve İşlev Haritası

- [config.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/config.py): SSOT (Single Source of Truth) merkezi konfigürasyon; optik, aktüatör, kontrolcü ve yol parametreleri.
- [main.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/main.py): Ana atış kontrol döngüsü, OpenCV video işleme, klavye kontrolleri.
- [telemetry.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/telemetry.py): v2 Telemetri kaydedicisi (35 kolon, hedef kutu boyutu, angajman evresi ve LiDAR ToF menzil kapılama dahil, gerçek kamera hatası ve kalıcı dosya tanıtıcılı I/O), oturum özeti (`.meta.json`) ve taktiksel olay günlüğü (`_events.csv`).
- [fcs_manager.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_manager.py): Durum makinesi (Arama, Tespit, Takip, Kilit, Ateşleme), balistik motoru, A/B mod yönetimi (`P` vs `RL`), menzil kapılama, 1. derece aktüatör gözlemcili ego-hareket telafisi (`tau = 0.0682 s`, geri Euler ayrıklaştırması, ego için komut modeli), hedef kaybında kontrolcü komut belleği temizleme (`on_target_lost`).
- [p_controller.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/p_controller.py): Üretim P; tek EMA (`alpha_filter = 0.9306`), saf oransal (`Kp_pan=0.1953, Kp_tilt=0.1544`, `Kv_pan=5.859 s^-1`), pan 353.1 / tilt 213.6 derece/s^2 komut ivme sınırı (`a * dt^2`), hedef kaybında komut sıfırlaması (`on_target_lost`), FF/Ki/Kd yok.
- [kalman_filter.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/kalman_filter.py): Ego-hareket telafili sabit hızlı (CV) Kalman; tek `sigma_a`/`r_var`, Joseph formu, 150 px robust tekil-sıçrama kapısı (tekil sıçramada R * 100 sönümleme, 3 karede yeniden yakalama reset), 30-60 px/s yumuşak ölü bölge ve 0.40 s tavanlı lead kestirimi.
- [rl_controller.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl_controller.py): Üretim RL; saf PPO, 11-64-64-2 NumPy MLP çıkarımı (SB3/torch bağımsız), ölçülen bbox gözlemi, çalışma zamanında kapalı çevrim osilasyonunu önlemek için sıfırlanmış hız gözlemi (hata türevi ve eylem geçmişi ile stabil takip), pan 0.85° / tilt 0.95° açısal yumuşak ölü bölge (`soft_deadband_enabled` anahtarlı), 353.1/213.6 derece/s^2 komut ivme sınırı ve yön değişim sönümlemesi; P harmanı yok. Tüm eşikler `CONFIG.rl` / `CONFIG.control` içinden gelir.
- [export_rl_policy.py](file:///c:/YTU/bitirme_projesi/turret_fcs/analysis/export_rl_policy.py): PPO zip modelini SB3'ten bağımsız `turret_ppo_v14_policy.npz` artifact'ine aktarır ve ileri geçişi doğrular.
- [rl_diagnose.py](file:///c:/YTU/bitirme_projesi/turret_fcs/analysis/rl_diagnose.py): Aynı tohumlarda üretim RL yolu (sıfır hız gözlemi) ile ham politikayı karşılaştıran teşhis aracı.
- [p_profile_sweep.py](file:///c:/YTU/bitirme_projesi/turret_fcs/analysis/p_profile_sweep.py): 6 profil x 8 kazanç Webots taraması ve limit-cycle sınıflandırması.
- [turret_hud.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/turret_hud.py): Askeri standartta dinamik taktiksel HUD arayüzü ve renk disiplini (Yeşil, Mavi, Kırmızı).
- [lock_manager.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/lock_manager.py): Dinamik kilit toleransı ve kademeli Gauss vuruş olasılığı (`hit_probability`).
- [fcs_comm.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/fcs_comm.py): STM32 ile 115200 Baud ikili (binary) UART paketi alışverişi, 0.1° altı kuantalama biriktiricisi.
- [camera_thread.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/camera_thread.py): Watchdog korumalı, sıfır gecikmeli kamera iş parçacığı.
- [patrol_generator.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/patrol_generator.py): Lissajous eğrisi parametrik hava sahası tarama devriyesi.
- [turret_env.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl/turret_env.py): 11 boyutlu simülasyon ortamı; 1.20 derece backlash, 1.50 derece/kare adım, progress ödülü, gecikmeli sensör kuyruğu (7D) + taze eylem geçmişi (4D).
- [train_rl.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/rl/train_rl.py): 250k adımlık saf PPO eğitim motoru; `turret_ppo_v14.zip` üretir.
- [webots_main.py](file:///c:/YTU/bitirme_projesi/turret_fcs/webots/controllers/webots_main/webots_main.py): Webots dijital ikiz ana döngüsü; üretim FCS/HUD modüllerini kullanır, kademeli SPINUP/SETTLE/FIRING sekansı içerir. P FF kapalıdır; lead/HUD canlıdır.
- [webots_tuning.py](file:///c:/YTU/bitirme_projesi/turret_fcs/webots/controllers/webots_main/webots_tuning.py): Webots-only Kp/max_output override, profil tavanı ve telemetri metadata'sı; varsayılan P kazançları `CONFIG.control.kp_pan/kp_tilt` ile aynıdır.
- [system_identification.py](file:///c:/YTU/bitirme_projesi/turret_fcs/analysis/system_identification.py): Sentetik nominal SOPDT/FOPDT analizi; gerçek telemetriye fit yapmaz, ölçülmüş tau/backlash üretmez.
- [weights.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/weights.py): GitHub Releases varlıklarından (Release Assets) model ağırlıklarını (`best.pt`, `yolov8n.pt`) otomatik indiren yardımcı modül.
- [engagement.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/engagement.py): Ayrıştırılmış 5 aşamalı atış ve hedef angajman durum makinesi (`SPINUP`, `SETTLE`, `FIRING`, `RECOVER`, `EVAL`).
- [presenter.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/presenter.py): Ayrıştırılmış taktiksel HUD çizim ve sunum katmanı.
- [keymap.py](file:///c:/YTU/bitirme_projesi/turret_fcs/turret_fcs/keymap.py): Operatör klavye kontrolleri ve mod geçişleri işleyicisi.
- [firmware.md](file:///c:/YTU/bitirme_projesi/turret_fcs/docs/firmware.md): CubeIDE derleme, ST-Link flaşlama, pinout ve bekçi köpeği kılavuzu.
- [reproducibility.md](file:///c:/YTU/bitirme_projesi/turret_fcs/docs/reproducibility.md): Test, RL eğitim ve tekrar üretilebilirlik kılavuzu.

---

## 5. Yeni Bir Sohbet Açıldığında Hatırlanacaklar

Yeni bir sohbet açıldığında asistan bu dosyayı okur okumaz:

1. Sistemin **v2.1 Production Release** etiketli olduğunu; v2.0 sonrası tüm algoritmik iyileştirmeleri (Kalman aday yörünge geri oynatmalı 150 px kapısı, kısa hedef kaybında ego-telafili `predict_only` takip köprüsü, P kontrolcü FPS-bağımsız EMA ve kazanç planlamasının kaldırılması, Optuna TPE optimal 353.1/213.6 derece/s^2 ivme sınırı, `on_target_lost` bellek temizliği, güven ağırlıklı `TargetSelector`, STM32 UART kayan pencere senkronizasyonu ve SSOT/CI sertleştirmeleri) topladığını; ağırlık varlıklarının `v2.0` release'inden indirildiğini (`DEFAULT_RELEASE_TAG = "v2.0"`),
2. RL politikasının `rl_v2-1.zip` / `rl_v2-1_policy.npz` olduğunu; eski modellerin ve ara denemelerin silindiğini, üretim çıkarımının SB3/torch bağımsız yapıldığını,
3. Varsayılan ve tek kontrolcünün **P** (tek EMA alpha=0.9306, saf oransal, Kp_pan=0.1953 / Kp_tilt=0.1544, Kv_pan=5.859 s^-1, kazanç planlaması kaldırılmış) olduğunu; eski kontrolcü ve seçicilerin silindiğini, v1.3 davranışı için `v1.3` etiketinin kullanıldığını,
4. Asla LaTeX / dolar işareti (`$`) kullanmaması gerektiğini,
5. Kodda veya tezde yapılacak her yeni geliştirmenin yukarıdaki mimari ve güvenlik disiplinine bağlı kalması gerektiğini,
6. Kalman filtresinin FAZ 9 ve FAZ 14 ile sabit hızlı (CV) + 150 px robust tekil-sıçrama kapılı yapıya kavuştuğunu; adaptif Q, dönüş override'ı, ivme sınırı, 3-kare eğim, çıkış EMA'sı ve sahada hatalı tetiklenen NIS kapısının kaldırıldığını, test paketinin konsolide ve yeşil olduğunu anında bilir.
7. P'in FAZ 10, FAZ 14 ve FAZ 15 Optuna TPE ile kararlılaştırıldığını; çift EMA, 0.20 derece/kare pan slew ve yapay kazanç planlamasının kaldırıldığını, ivme sınırının 353.1 / 213.6 derece/s^2 olduğunu, tabanın 0.1953 / 0.1544 derece/kare olduğunu, canlı telemetri sistem tanılama modelinin (tau = 68.2 ms, theta = 66.7 ms, backlash = 1.61 deg) `turret_fcs/config.py` (SSOT) ile tam uyumlu olduğunu bilir.
8. RL'in v2.0'de saf PPO'ya çevrildiğini; P harmanının kaldırıldığını, FAZ 14 ile çalışma zamanı Kalman hız gözleminin ego faz gecikmesi kaynaklı osilasyon nedeniyle `0.0` yapıldığını bilir.
9. FAZ 12'de repo geneli denetim yapıldığını; hedef seçici histerezis/ID kapısı, UART RX tamponu ve eksen-bazlı residue, Webots tuning varsayılanları, HUD spin-up süresi, gerçek telemetri hatası/latency'si ve firmware ölü API'lerinin düzeltildiğini bilir.
10. FAZ 14 ile hedef kaybı sonrası zıt yönde komut vermeyi önleyen `on_target_lost` sıfırlamasının eklendiğini, ego modelinin geri Euler ile eşitlendiğini (`tau = 0.0682 s`), lead tavanının 0.40 s yapıldığını ve tilt adımı limitinin 0.75 derece olarak tüm hatlarda eşitlendiğini bilir.
11. **P vs RL Dürüst Karşılaştırması ve Determinizm Doğrulaması:** Gymnasium ortamındaki determinizm kaçağı (`global np.random` -> `self.np_random`) düzeltildikten sonra, 50 deterministik tohum (Seeds 100-149) üzerinde tam üretim FCSManager hattında yapılan doğrulamada:
    - **P Kontrolcü:** Medyan Hata: 56.79 px, RMS: 80.80 px, FCS Kilit: %47.67.
    - **RL Kontrolcü (Yeni PPO v2.1):** Medyan Hata: 45.71 px, RMS: 69.28 px, FCS Kilit: %71.23.
    - RL modu, çok eksenli dinamik manevra ve agresif kaçış senaryolarında medyan hatada **%19.5**, RMS hatasında **%14.3** iyileşme ve **1.5 kat daha yüksek kilit oranı** sağlar. Jerk seviyesi (0.025 vs 0.001) P'ye göre daha yüksektir (RL daha çevik ve reaktif adımlar atar).
    - **Hız Gözlemi (Velocity Observation) Notu:** Eğitimde `obs[4:6]` (Kalman hız kestirimi) kapalı çevrim osilasyonunu önlemek için sıfır beslendiği için bu giriş ağırlıkları eğitilmemiştir (dormant/untrained). Politika hata türevi ve Smith Predictor eylem geçmişi ile çalışır; çalışma zamanında hız girişleri mutlak suretle `0.0` kalmalıdır.
