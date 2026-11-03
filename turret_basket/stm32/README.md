# Basket Challenge — STM32F401RE Gömülü Firmware Modülü

Bu dizin, **Kulplu Kupaya Dart Basket Challenge** görevi için özel olarak geliştirilmiş bağımsız STM32F401RE (Nucleo-64) gömülü yazılımını ve kaynak kodlarını içerir.

---

## 📌 Pin Bağlantı ve Çevre Birimleri Tablosu (Pinout)

STM32CubeMX (.ioc) yapılandırmasında kullanılacak doğrulanmış pin listesi:

| Fonksiyon | STM32 Pini | Çevre Birimi / Mod | Açıklama |
| :--- | :--- | :--- | :--- |
| **Pan Servo Sinyali** | `PB4` (D5) | TIM3_CH1 (PWM Generation) | 50 Hz PWM, 0.57 - 2.47 ms darbe aralığı, 15.0° - 165.0° (±75° hareket alanı) |
| **Tilt Servo Sinyali** | `PB5` (D4) | TIM3_CH2 (PWM Generation) | 50 Hz PWM, 0.57 - 2.47 ms darbe aralığı, 65.0° - 135.0° (-25° / +45° aşırtma süper-elevasyon) |
| **TB6612FNG Volan PWM** | `PB0` (D3) | TIM1_CH2N (PWM Generation) | FF-180 çift motor için 20 kHz ultrasonik sessiz PWM (PWMA+PWMB) |
| **Solenoid Tetikleme** | `PB1` (D6) | GPIO_Output (Pull-down) | 80 ms itme darbesi için optokuplör / MOSFET kapısı |
| **TF-Luna LiDAR RX** | `PA10` (D2) | USART1_RX | 100 Hz ToF LiDAR telemetrisi (115200 Baud, 8N1) |
| **TF-Luna LiDAR TX** | `PA9` (D8) | USART1_TX | TF-Luna sorgu ve yapılandırma hattı |
| **PC İletişim TX** | `PA2` (ST-Link) | USART2_TX | PC'ye 10-baytlık telemetri akışı (115200 Baud, 8N1) |
| **PC İletişim RX** | `PA3` (ST-Link) | USART2_RX | PC'den 8-baytlık dinamik hızlı komut akışı (115200 Baud, 8N1) |
| **Kullanıcı Butonu (B1)** | `PC13` (Mavi) | GPIO_EXTI13 (Pull-up) | Mandallı donanımsal/yazılımsal E-stop (250 ms debounce) |
| **Durum LED'i (LD2)** | `PA5` (Yeşil) | GPIO_Output | 500 ms kalp atışı göstergesi |

---

## ⚡ TB6612FNG & FF-180 Bağlantı Şeması

Çift FF-180 fırçalı DC volan motoru, TB6612FNG çift H-köprüsü ile 20 kHz PWM kullanılarak sessiz ve dinamik hız kontrollü sürülür:

```
[STM32 Nucleo-64]               [TB6612FNG Motor Sürücü]            [FF-180 Motorlar]
   PB0 (TIM1_CH2N) ------------> PWMA & PWMB (Birleştirilmiş)
   3.3V (VCC)       ------------> AIN1 & BIN1 (İleri Yön)
   GND              ------------> AIN2 & BIN2 (İleri Yön)
   3.3V (VCC)       ------------> STBY (Standby Aktif)
   3.3V (VCC)       ------------> VCC (Lojik Besleme)
   Ortak GND        ------------> GND
[Harici 6V-8.4V Güç] ----------> VM (Motor Güç Girişi)
                                  AO1 & AO2 ----------------------> Sol Volan Motoru
                                  BO1 & BO2 ----------------------> Sağ Volan Motoru
```

---

## 🛠️ STM32CubeMX (.ioc) Yapılandırma Kılavuzu

Kendi .ioc dosyanızı oluştururken aşağıdaki parametreleri birebir eşleştirmeniz yeterlidir:

### 1. Clock Configuration (RCC)
- **HCLK (Core Clock):** 84 MHz (HSI 16 MHz -> PLL /16 *336 /4 = 84 MHz)
- **APB1 Peripheral Clocks:** 42 MHz (TIMx Clock: 84 MHz)
- **APB2 Peripheral Clocks:** 84 MHz (TIMx Clock: 84 MHz)

### 2. Timers
- **TIM3 (PB4 - CH1 Pan, PB5 - CH2 Tilt - 50 Hz Servo PWM):**
  - Clock Source: Internal Clock
  - Channel 1: PWM Generation CH1
  - Channel 2: PWM Generation CH2
  - Prescaler (PSC): `83` (84 MHz / 84 = 1 MHz -> 1 us per tick)
  - Counter Period (ARR): `19999` (20000 us = 20 ms -> 50 Hz)
  - Pulse: `1520` (90° mekanik merkez)
- **TIM1 (PB0 - CH2N - 20 kHz TB6612FNG Volan PWM):**
  - Channel 2: PWM Generation CH2N (Pin: `PB0`)
  - Prescaler (PSC): `0` (84 MHz tick)
  - Counter Period (ARR): `4199` (84 MHz / 4200 = 20.000 Hz, sessiz ultrasonik PWM)
  - Pulse: `0` (Başlangıçta duruyor)

### 3. USART Configuration
- **USART1 (TF-Luna LiDAR):**
  - Mode: Asynchronous, Baudrate: `115200 Bits/s`, 8 Data Bits, None Parity, 1 Stop Bit
  - NVIC Settings: `USART1 global interrupt` ENABLED
- **USART2 (ST-Link PC İletişimi):**
  - Mode: Asynchronous, Baudrate: `115200 Bits/s`, 8 Data Bits, None Parity, 1 Stop Bit
  - NVIC Settings: `USART2 global interrupt` ENABLED

### 4. GPIO Ayarları
- **PB1 (Solenoid İtici):**
  - Mode: Output Push-Pull, Pull-up/Pull-down: `Pull-down`, Maximum Output Speed: `Low`
- **PC13 (B1 Mavi Buton):**
  - Mode: External Interrupt Mode with Falling edge trigger detection
  - Pull-up/Pull-down: `Pull-up`
  - NVIC Settings: `EXTI line[15:10] interrupts` ENABLED (Priority: 5)
- **PA5 (LD2 LED):**
  - Mode: Output Push-Pull, No pull-up/pull-down

### 5. IWDG (Bağımsız Bekçi Köpeği)
- IWDG counter clock prescaler: `64`
- IWDG down-counter reload value: `400` (~800 ms zaman aşımı)

---

## 📡 İkili Protokol Sözleşmesi (SSOT - 8 Bayt)

- **Komut Çerçevesi (PC -> STM32, 8 Bayt):**
  `[0xAA][pan i16][tilt i16][speed u8][state u8][checksum u8]`
  - `pan i16`: 0.1 derece çözünürlüklü göreli açı komutu
  - `tilt i16`: 0.1 derece çözünürlüklü göreli açı komutu
  - `speed u8`: 0 - 255 aralığında TB6612FNG 20 kHz PWM duty değeri (5.5 - 16.5 m/s balistik hız eşlemesi)
  - `state u8`: 0=TRACKING, 1=LOCKED, 2=FIRING, 3=ESTOP
  - `checksum u8`: İlk 7 baytın 8-bit toplamı
- **Telemetri Çerçevesi (STM32 -> PC, 10 Bayt):**
  `[0x55][mesafe u16][sinyal u16][pan i16][tilt i16][checksum u8]`
