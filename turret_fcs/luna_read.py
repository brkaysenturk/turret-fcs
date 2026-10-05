import threading
import time
import serial


class LunaDriver:
    """
    Benewake TF-Luna LiDAR sensorunden 100hz mesafe, sinyal gucu
    ve sicaklik verilerini thread uzerinden okuyan, baglanti
    kopmalarina dayanikli surucu sinifi.
    """

    def __init__(self, port="COM4", baud=115200, stale_timeout=0.25):
        self.port = port
        self.baud = baud
        self.distance = 0
        self.strength = 0
        self.temp = 0.0
        self.running = False
        self.ser = None
        self.thread = None
        self.last_valid_time = 0.0
        self.stale_timeout = stale_timeout

    def _open_port(self):
        """Seri portu acar."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            print(f"[OK] [TF-Luna] {self.port} portuna baglandi.")
            return True
        except Exception as e:
            self.ser = None
            return False

    def start(self):
        """Seri port baglantisini acar ve okuma thread'ini baslatir."""
        self.running = True
        self.last_valid_time = time.time()
        self._open_port()
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return True

    def _update(self):
        """Gecerli TF-Luna paketlerini okur, gecici I/O hatalarinda yeniden dener."""
        while self.running:
            # Port acik degilse baglanmayi dene
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                if self.running:
                    self._open_port()
                continue

            try:
                if self.ser.in_waiting < 9:
                    time.sleep(0.002)
                    continue

                header_1 = self.ser.read(1)
                if header_1 != b"\x59":
                    continue

                header_2 = self.ser.read(1)
                if header_2 != b"\x59":
                    continue

                data = self.ser.read(7)
                if len(data) != 7:
                    continue

                checksum = (0x59 + 0x59 + sum(data[:6])) & 0xFF
                if checksum != data[6]:
                    continue

                self.distance = data[0] + (data[1] << 8)
                self.strength = data[2] + (data[3] << 8)
                self.temp = (data[4] + (data[5] << 8)) / 8.0 - 256.0
                self.last_valid_time = time.time()

            except (serial.SerialException, OSError):
                # Anlik port kesilmesinde thread'i oldurme; portu kapat ve tekrar denemeye gec
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(0.1)
            except Exception:
                time.sleep(0.005)

    def get_data(self):
        """Guncel sensor verilerini dondurur. Eski veri, stale durumda sifira cekilir."""
        if time.time() - self.last_valid_time > self.stale_timeout:
            return 0, 0, 0.0
        return self.distance, self.strength, self.temp

    def stop(self):
        """Okuma thread'ini durdurur ve seri portu kapatir."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.3)
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            finally:
                self.ser = None


if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    print(f"[*] TF-Luna USB-TTL Dogrudan Baglanti Testi ({port})...")
    driver = LunaDriver(port=port)
    if driver.start():
        try:
            while True:
                dist, strength, _ = driver.get_data()
                print(f"[TF-LUNA] Mesafe: {dist:4d} cm | Sinyal: {strength:5d}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n[!] Test durduruldu.")
        finally:
            driver.stop()