# Basket Challenge — STM32 Gömülü Firmware Modülü

Bu dizin, Kulplu Kupaya Dart Basket Challenge görevi için özel olarak geliştirilecek bağımsız STM32F401RE gömülü yazılımını içerir.

## Mimari ve Gereksinimler:
1. **İletişim:** PC tarafındaki `basket/main.py` ile 115200 Baud ikili (binary) UART protokolü (`0xAA` komut, `0x55` telemetri).
2. **Servo Sürücüsü:** MG996R Pan/Tilt servoları (50 Hz PWM, 45.0 derece/saniye güvenlik hız limiti).
3. **Mesafe Sensörü:** TF-Luna ToF LiDAR (100 Hz UART okuma).
4. **Fırlatıcı:** Çift volan motoru PWM sürücüsü + 80 ms kenar tetiklemeli solenoid itici darbe kontrolü.
5. **Güvenlik Bekçisi:** 300 ms Heartbeat Watchdog ve E-Stop.
