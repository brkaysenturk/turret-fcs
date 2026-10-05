import struct
import threading
import time
import serial


class FCSCommunicator:
    """
    STM32 mikrodenetleyicisi ile tek bir seri port (COM) uzerinden Full-Duplex
    ve Saf Binary (ikili) paket protokolu ile calisan iletisim sinifi:
    
      1. TX (PC -> STM32) [7 Bayt]:
         [0xAA] [Pan: int16 (2B)] [Tilt: int16 (2B)] [State: uint8] [Checksum: uint8]
         
      2. RX (STM32 -> PC) [6 Bayt]:
         [0x55] [Mesafe: uint16 (2B)] [Sinyal: uint16 (2B)] [Checksum: uint8]
    """

    HEADER_TX = 0xAA
    HEADER_RX = b"\x55"

    def __init__(self, port="COM3", baudrate=115200, reconnect_interval=2.0, stale_timeout=0.3):
        self.port = port
        self.baudrate = baudrate
        self.reconnect_interval = reconnect_interval
        self.stale_timeout = stale_timeout

        self.last_reconnect_attempt = 0.0
        self.ser = None
        self._lock = threading.Lock()

        # TF-Luna telemetri verileri
        self.distance = 0
        self.strength = 0
        self.temp = 0.0
        self.last_valid_time = 0.0

        # Arka plan binary dinleyici
        self.running = True
        self.connect()
        self.thread = threading.Thread(target=self._binary_reader_worker, daemon=True)
        self.thread.start()

    def connect(self):
        """Seri port baglantisini baslatir veya yeniden baglanmayi dener."""
        self.last_reconnect_attempt = time.time()
        try:
            with self._lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=0.04,
                    write_timeout=0.04,
                )
            print(f"[OK] [STM32] {self.port} baglandi (Full-Duplex Binary Mod).")
            return True
        except (serial.SerialException, OSError):
            self.ser = None
            return False

    @property
    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def send(self, pan_angle: float, tilt_angle: float, fcs_state: int):
        """
        Taret hedef acilarini ve durumunu (0: Takip, 1: Kilit, 2: Ates) binary paket olarak gonderir.
        Paket: [0xAA, Pan_L, Pan_H, Tilt_L, Tilt_H, State, Checksum] (7 Bayt)
        """
        now = time.time()
        if not self.is_connected:
            if (now - self.last_reconnect_attempt) > self.reconnect_interval:
                self.connect()
            if not self.is_connected:
                return

        p = int(max(-32000, min(32000, int(pan_angle * 10))))
        t = int(max(-32000, min(32000, int(tilt_angle * 10))))
        state = int(fcs_state) & 0xFF

        try:
            payload = struct.pack("<BhhB", self.HEADER_TX, p, t, state)
            checksum = sum(payload) & 0xFF
            with self._lock:
                if self.ser and self.ser.is_open:
                    self.ser.write(payload + bytes([checksum]))
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            print("[ERR] [STM32] Paket gonderilemedi veya baglanti koptu.")
            self.close()

    def _binary_reader_worker(self):
        """
        STM32'den gelen 6 baytlik ikili telemetri paketlerini okur:
        [0x55] [Dist_L, Dist_H] [Str_L, Str_H] [Checksum]
        """
        while self.running:
            if not self.is_connected:
                time.sleep(0.5)
                continue

            try:
                # 1. Seri port durumu ve bekleyen bayt kontrolu (kilit kilitlemeden bekleme)
                with self._lock:
                    if not self.ser or not self.ser.is_open:
                        ser_ok = False
                    else:
                        ser_ok = True
                        in_w = self.ser.in_waiting

                if not ser_ok:
                    time.sleep(0.1)
                    continue

                if in_w < 6:
                    time.sleep(0.002)
                    continue

                # Tamponda 3 paketten fazla birikmisse en guncel pakete atla (sifir sensor gecikmesi)
                if in_w > 18:
                    with self._lock:
                        if self.ser and self.ser.is_open:
                            discard_count = (in_w // 6 - 1) * 6
                            self.ser.read(discard_count)

                # 2. Paket basligini oku (0x55)
                header = None
                with self._lock:
                    if self.ser and self.ser.is_open:
                        header = self.ser.read(1)

                if header != self.HEADER_RX:
                    continue

                # 3. Kalan 5 bayti oku (2B Mesafe + 2B Sinyal + 1B Checksum)
                payload = None
                with self._lock:
                    if self.ser and self.ser.is_open:
                        payload = self.ser.read(5)

                if not payload or len(payload) != 5:
                    continue

                # 4. Checksum dogrulama: (0x55 + payload[0..3]) & 0xFF == payload[4]
                calc_csum = (0x55 + sum(payload[:4])) & 0xFF
                if calc_csum != payload[4]:
                    continue

                # 5. Veriyi cozumle
                dist, strength = struct.unpack("<HH", payload[:4])
                self.distance = dist
                self.strength = strength
                self.last_valid_time = time.time()

            except (serial.SerialException, OSError):
                time.sleep(0.1)

    def get_luna_data(self):
        """
        Guncel sensor verilerini dondurur.
        0.3 saniye boyunca yeni veri gelmezse 0 cm dondurur.
        """
        if time.time() - self.last_valid_time > self.stale_timeout:
            return 0, 0, 0.0
        return self.distance, self.strength, self.temp

    def close(self):
        """Seri portu ve okuma thread'ini guvenli kapatir."""
        self.running = False
        with self._lock:
            if self.ser:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                    print("[OK] [STM32] Baglanti kapatildi.")
                except Exception:
                    pass
                finally:
                    self.ser = None
