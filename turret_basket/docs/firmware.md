# Basket Challenge — STM32F401RE Gömülü Donanım ve Firmware Kılavuzu

Bu doküman; Kulplu Kupaya Basket Atma (Basket Challenge) görevinin gömülü denetleyici katmanı olan **STM32F401RE Nucleo-64** kartının firmware mimarisini, pin bağlantılarını, ikili (binary) haberleşme protokol sözleşmesini, STM32CubeIDE derleme adımlarını ve çok katmanlı güvenlik sistemini açıklamaktadır.

---

## 🛠️ Donanım Özellikleri

- **Mikrodenetleyici:** STM32F401RET6 (ARM Cortex-M4 @ 84 MHz, donanımsal FPU destekli)
- **Hafıza:** 512 KB Flash, 96 KB SRAM
- **İletişim:** 2x USART (PC ile 115200 Baud ikili protokol ST-Link VCP, Benewake TF-Luna ToF LiDAR ile 115200 Baud UART)
- **Zamanlayıcılar (Timers):** 50 Hz Servo PWM (TIM3_CH1, TIM3_CH2)
- **Hata Güvenliği:** Donanımsal Bağımsız Bekçi Köpeği (IWDG - 800 ms), Kademeli PC İletişim Bekçisi (300 ms fırlatıcı E-stop + 2000 ms servo merkezleme), 80 ms kenar tetiklemeli solenoid durum makinesi ve B1 mandallı acil fren (E-stop).

---

## 📌 Doğrulanmış Pin Bağlantı Tablosu (Pinout)

Sistem, Turret FCS ana taret mimarisi ile donanımsal ve elektriksel olarak %100 uyumlu aynı fiziksel platformu kullanır:

| Fonksiyon | STM32 Pini | Donanım Modülü | Açıklama |
| :--- | :--- | :--- | :--- |
| **Pan Servo Sinyali** | `PB4` (TIM3_CH1) | MG996R / DS3218 | 50 Hz PWM, 0.57 - 2.47 ms darbe aralığı, 15.0° - 165.0° (±75° hareket alanı, merkez 90.0°) |
| **Tilt Servo Sinyali** | `PB5` (TIM3_CH2) | MG996R / DS3218 | 50 Hz PWM, 0.57 - 2.47 ms darbe aralığı, 65.0° - 115.0° (±25° hareket alanı, merkez 90.0°) |
| **Volan Sürücü (MOSFET)** | `PB0` (GPIO Output) | IRLZ44N N-MOSFET | Çift FF-180 6V fırlatıcı motor sürücüsü. Atış sekansında 600 ms ön devirlenme (spin-up) ile aktif edilir |
| **Solenoid Tetikleme** | `PB1` (GPIO Output) | JF-0530B İtici | Optokuplör yalıtımlı solenoid darbe çıkışı (80 ms itme + 150 ms geri çekilme) |
| **TF-Luna LiDAR RX** | `PA10` (USART1_RX) | Benewake TF-Luna | 100 Hz ToF mesafe telemetrisi alımı (115200 Baud UART) |
| **TF-Luna LiDAR TX** | `PA9` (USART1_TX) | Benewake TF-Luna | LiDAR konfigürasyon ve sorgulama hattı |
| **PC İletişim TX** | `PA2` (USART2_TX) | ST-Link VCP | PC'ye 10-baytlık durum ve LiDAR telemetrisi iletimi (`0x55` header) |
| **PC İletişim RX** | `PA3` (USART2_RX) | ST-Link VCP | PC'den 7-baytlık ikili komut paketi alımı (`0xAA` header + Pan + Tilt + State + CS) |
| **Kullanıcı Butonu (B1)** | `PC13` (GPIO Input) | Mavi Buton (B1) | Mandallı yazılım E-stop: ilk basışta volan/solenoid kesilir ve servolar merkeze iner; ikinci basışta mandal çözülür. Dahili pull-up + 250 ms debounce |
| **Durum LED'i (LD2)** | `PA5` (GPIO Output) | Yeşil LED (LD2) | Sistem hazır / ana döngü aktivite göstergesi |

---

## 📡 PC <-> STM32 İkili Protokol Sözleşmesi (SSOT)

PC tarafındaki [basket_comm.py](file:///c:/YTU/turret-fcs/turret_basket/basket_comm.py) ve [protocol.py](file:///c:/YTU/turret-fcs/turret_basket/protocol.py) ile firmware tarafındaki [protocol.h](file:///c:/YTU/turret-fcs/turret_basket/stm32/protocol.h) birebir senkronize çalışan Tek Doğruluk Kaynağı (SSOT) mimarisine sahiptir.

### 1. Komut Çerçevesi (PC -> STM32, 7 Bayt)

100 Hz frekansında PC'den mikrodenetleyiciye gönderilir:

```
[0xAA][pan_ticks_i16][tilt_ticks_i16][state_u8][checksum_u8]
```

- **Başlık (`0xAA`):** Paket başlangıç baytı (`PROTO_PC_TO_STM_HEADER`).
- **Pan / Tilt Adımı (`int16_t`):** 0.1 derece çözünürlüklü göreli servo hareket emri (`değer = açı_derece * 10.0`). 0.1 derecenin altındaki hassas komutlar PC tarafındaki artık biriktiricide (`_accum_pan`, `_accum_tilt`) saklanır; kuantalama kaybı önlenir.
- **Durum Baytı (`uint8_t`):**
  - `0 (STATE_TRACKING)`: Kupayı takip etme ve optik merkeze kilitlenme modu.
  - `1 (STATE_LOCKED)`: Kupa merkezine kararlı kilit sağlandı, atış için hazır.
  - `2 (STATE_FIRING)`: 80 ms kenar tetiklemeli solenoid darbesi ile dartı fırlatma emri.
  - `3 (STATE_ESTOP)`: Acil durum freni; motorları kes ve servoları kilitle.
- **Sağlama Toplamı (Checksum, `uint8_t`):** İlk 6 baytın 8-bit toplama toplamı (`sum & 0xFF`).

### 2. Telemetri Çerçevesi (STM32 -> PC, 10 Bayt)

STM32 tarafından 100 Hz frekansında PC'ye aktarılan durum paketi:

```
[0x55][distance_u16][strength_u16][pan_angle_i16][tilt_angle_i16][checksum_u8]
```

- **Başlık (`0x55`):** Telemetri başlangıç baytı (`PROTO_STM_TO_PC_HEADER`).
- **Mesafe (`uint16_t`):** TF-Luna LiDAR'dan okunan filtrelenmiş ham mesafe (santimetre cinsinden).
- **Sinyal Gücü (`uint16_t`):** ToF sensörünün sinyal kalitesi (güvenilirlik eşiği kontrolü için).
- **Pan / Tilt Açısı (`int16_t`):** Servoların 0.1 derece ölçeğinde güncel açıları (0..1800 aralığı; 0.0° - 180.0°). PC tarafı 90.0° merkez ofsetini çıkararak göreli açıyı elde eder.
- **Sağlama Toplamı (Checksum, `uint8_t`):** İlk 9 baytın 8-bit toplama toplamı (`sum & 0xFF`).

---

## 💻 STM32CubeIDE ile Derleme ve Yükleme

Fiziksel taret donanımı, [turret_fcs/stm32/turret_fcs](file:///c:/YTU/turret-fcs/turret_fcs/stm32/turret_fcs) dizinindeki doğrulanmış STM32CubeIDE firmware projesini çalıştırır.

1. **Projeyi İçe Aktarma (Import):**
   - STM32CubeIDE'yi açın: `File -> Import... -> General -> Existing Projects into Workspace`.
   - `Root Directory` olarak `turret_fcs/stm32/turret_fcs/` dizinini seçin ve `Finish` butonuna tıklayın.

2. **CubeMX Yapılandırması (.ioc):**
   - `turret_fcs.ioc` dosyası mikrodenetleyicinin saat frekansını (84 MHz PLL), zamanlayıcıları (TIM3 50 Hz PWM) ve UART çevre birimlerini (USART1 115200, USART2 115200) barındırır.
   - Değişiklik yapıldığında `Device Configuration Tool -> Code Generation` ile C kodları yeniden üretilebilir.

3. **Derleme (Build):**
   - `Project -> Build Project` (veya `Ctrl+B`) seçeneği ile projeyi derleyin.
   - İki yapılandırma hazırdır: `Debug` (geliştirme) ve `Release` (saha). Debug çıktısı `Debug/turret_fcs.elf` yaklaşık 18 KB, Release çıktısı `Release/turret_fcs.elf` yaklaşık 10 KB'dir; ikisi de sıfır uyarı ile derlenir.

4. **Karta Yükleme (Flash):**
   - STM32 Nucleo kartını USB kablosuyla bilgisayara bağlayın.
   - `Run -> Run As -> STM32 Cortex-M C/C++ Application` seçin.
   - ST-Link programlayıcı kodu mikrodenetleyiciye flaşlayacak ve sistemi başlatacaktır.

### Debug ve Release Farkı

- **Debug:** Kod `-O0` ile (optimizasyon kapalı) derlenir, `DEBUG` makrosu tanımlıdır. Kesme noktalarında bekleme yapılabilir. Kod boyutu yaklaşık 18 KB'dir.
- **Release:** Kod `-Os` (boyut optimizasyonu) ve `-g0` ile derlenir; sahada çalışacak nihai sürümdür. Kod boyutu yaklaşık 10 KB'dir.

---

## 🛡️ Basket Görevine Özel Güvenlik ve Fırlatma Katmanları

Basket atışı görevi yüksek süper-elevasyon açıları (+21.8° ile +22.6° arası ilave tilt) ve hızlı volan devirlenmesi gerektirdiğinden çok katmanlı güvenlik mekanizmaları devrededir:

1. **2-Aşamalı Balistik Fırlatma Durum Makinesi ([engagement.py](file:///c:/YTU/turret-fcs/turret_basket/engagement.py)):**
   - **Aşama 1 (SPINUP_ELEVATE - 600 ms):** Atış emri verildiğinde volan motorları ön devirlenmeye başlar ve tilt ekseni nominal görüş hattından çıkarak 2. Modelin belirlediği süper-elevasyon açısına yükselir.
   - **Aşama 2 (FIRING - 80 ms):** Taret hedeflenen balistik açıya oturduğunda ve volanlar hedef hıza ulaştığında `STATE_FIRING` paketi gönderilir; solenoid 80 ms darbe ile dartı fırlatır.
   - **Aşama 3 (RECOVER - 900 ms):** Fırlatma sonrası volan motorları kesilir; taret yumuşak bir sönümleme profili ile kupanın optik merkezine geri döner ve takibi sürdürür.

2. **Kademeli PC Heartbeat Watchdog:**
   - **300 ms Aşımı (`FCS_CMD_TIMEOUT_MS`):** PC komut akışı kesilirse fırlatıcı motorlar derhal enerjisiz bırakılır ve solenoid geri çekilir.
   - **2000 ms Aşımı (`SERVO_PARK_TIMEOUT_MS`):** PC bağlantısı kalıcı koparsa servoların zorlanıp aşırı akım çekmesini önlemek için taret güvenli merkez konumuna (90.0°, 90.0°) çekilir.

3. **800 ms Bağımsız Donanımsal Bekçi Köpeği (IWDG):**
   - STM32 dahili 32 kHz LSI osilatörü üzerinden çalışan bağımsız IWDG sayacı aktiftir. Ana döngü kilitlenirse mikrodenetleyici donanımsal reset atarak taretin kontrolden çıkmasını engeller.

4. **80 ms Yazılımsal Solenoid Durum Makinesi:**
   - Solenoid bobininin ısınarak yanmasını ve mekanizmaya çift dart sıkışmasını önlemek için itme süresi donanımsal zamanlayıcı ile 80 ms, yeniden doldurma beklemesi 150 ms ile sınırlandırılmıştır.

5. **B1 Mavi Buton ile Mandallı E-stop:**
   - Kart üzerindeki B1 butonu donanımsal acil frendir. Basıldığı anda volan motorları ve solenoid kapatılır; taret donanımsal merkeze park edilir. Tekrar basıldığında sistem normal çalışmaya döner.

6. **Fiziksel Ateşleme İzolasyonu (Güvenli Test Modu):**
   - Laboratuvar ve masaüstü yazılım testlerinde istenmeyen dart atışlarını önlemek için firmware içinde fiziksel solenoid pini yazılımsal güvenlik koşuluna bağlıdır:
     ```c
     #ifndef ENABLE_PHYSICAL_FIRE
     #define ENABLE_PHYSICAL_FIRE 0
     #endif
     ```
   - Bu bayrak `0` iken tüm durum makinesi, LED ve telemetri çalışır ancak solenoid fiziksel olarak tetiklenmez. Gerçek atış yapılacağı zaman `1` yapılarak derlenir.
