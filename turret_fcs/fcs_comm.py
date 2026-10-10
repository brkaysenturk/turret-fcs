"""
================================================================================
MODUL: DONANIM HABERLESME VE SERI PORT PROTOKOLU (FCS COMMUNICATOR / UART)
================================================================================
Bu modul, PC (Python) ile gomulu sistem (STM32 mikrodenetleyici) arasindaki
tam cift yonlu (Full-Duplex), asenkron ve binary paketli iletisim katmanidir.

PROTOKOL VE PAKET YAPISI (115200 Baud):
1. TX Paketi (PC -> STM32) [7 Bayt]:
   [0xAA (Header)] [Pan Delta * 100: int16 (2B)] [Tilt Delta * 100: int16 (2B)] [State/Fire: uint8 (1B)] [Checksum: uint8 (1B)]
2. RX Paketi (STM32 -> PC) [6 Bayt]:
   [0x55 (Header)] [TF-Luna Mesafe: uint16 (2B)] [Sinyal Gucu: uint16 (2B)] [Checksum: uint8 (1B)]

GELISMIS OZELLIKLER:
- Otomatik Port Tespiti (Auto Plug-and-Play): Windows COM portlarini tarayarak
  ST-Link / STM32 USB VID '0483' kimligini otomatik tespit eder ve baglanir.
- Bagimsiz Okuma Thread'i: UART RX okumasini ayri thread'de calistirarak ana kontrol
  dongusunun bloklanmasini engeller.
- Checksum Dogrulama: Gurultulu seri hatta hatali baytlari tespit edip eler.
================================================================================
"""

import struct
import threading
import time
import serial


class FCSCommunicator:
    """
    STM32 mikrodenetleyicisi ile tek bir seri port uzerinden Full-Duplex
    ve Saf Binary (ikili) paket protokolu ile calisan iletisim sinifi:
    
      1. TX (PC -> STM32) [7 Bayt]:
         [0xAA] [Pan: int16 (2B)] [Tilt: int16 (2B)] [State: uint8] [Checksum: uint8]
         
      2. RX (STM32 -> PC) [6 Bayt]:
         [0x55] [Mesafe: uint16 (2B)] [Sinyal: uint16 (2B)] [Checksum: uint8]
         
    Baglanti: Herhangi bir sabit COM port girmeden ST-Link / STM32 USB VID (0483) 
    veya bagli tek USB seri portu otomatik olarak tarayip baglanir.
    """

    HEADER_TX = 0xAA
    HEADER_RX = b"\x55"

    def __init__(self, port=None, baudrate=115200, reconnect_interval=2.0, stale_timeout=0.3):
        self.port = port
        self.baudrate = baudrate
        self.reconnect_interval = reconnect_interval
        self.stale_timeout = stale_timeout

        self.last_reconnect_attempt = 0.0
        self.ser = None
        self._lock = threading.Lock()
        self._warned_offline = False

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

    def _find_target_port(self):
        """Mevcut seri portlari donanim kimligine (VID 0483 / ST-Link) gore tarar."""
        try:
            import serial.tools.list_ports
            available = list(serial.tools.list_ports.comports())
            if not available:
                return None

            # 1. Oncelik: ST-Link / STM32 donanim tanimi veya VID:0483
            for p in available:
                desc = (p.description or "").lower()
                hwid = (p.hwid or "").lower()
                if "0483:" in hwid or "stlink" in desc or "stm" in desc or "stmicroelectronics" in desc:
                    return p.device

            # 2. Tanimlanmis ozel port parametresi varsa kontrol et
            if self.port:
                for p in available:
                    if p.device.upper() == self.port.upper():
                        return p.device

            # 3. Bilgisayarda takili tek bir COM port varsa direkt onu sec
            if len(available) == 1:
                return available[0].device

            # 4. Genel USB-Serial donanimlari (CH340, CP210, FTDI vb.)
            for p in available:
                desc = (p.description or "").lower()
                if any(x in desc for x in ["ch340", "cp210", "ftdi", "usb serial", "usb-to-serial"]):
                    return p.device

        except Exception:
            pass
        return None

    def connect(self):
        """Seri port baglantisini otomatik tespit ile baslatir."""
        self.last_reconnect_attempt = time.time()
        target_port = self._find_target_port()
        if not target_port:
            if not self._warned_offline:
                print("[BILGI] [STM32] Bagli ST-Link/STM32 portu bulunamadi (Donanimsiz Mod). Takildiginda otomatik baglanacak.")
                self._warned_offline = True
            self.ser = None
            return False

        try:
            with self._lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    target_port,
                    self.baudrate,
                    timeout=0.04,
                    write_timeout=0.04,
                )
            self.port = target_port
            self._warned_offline = False
            print(f"[OK] [STM32] {self.port} otomatik baglandi (Full-Duplex Binary Mod).")
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
            self.close()

    def _binary_reader_worker(self):
        """STM32'den gelen 6 baytlik ikili telemetri paketlerini okur."""
        while self.running:
            if not self.is_connected:
                time.sleep(0.5)
                continue

            try:
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

                # Tamponda 3 paketten fazla birikmisse en guncel pakete atla (sifir gecikme)
                if in_w > 18:
                    with self._lock:
                        if self.ser and self.ser.is_open:
                            discard_count = (in_w // 6 - 1) * 6
                            self.ser.read(discard_count)

                # Paket basligi: 0x55
                header = None
                with self._lock:
                    if self.ser and self.ser.is_open:
                        header = self.ser.read(1)

                if header != self.HEADER_RX:
                    continue

                payload = None
                with self._lock:
                    if self.ser and self.ser.is_open:
                        payload = self.ser.read(5)

                if not payload or len(payload) != 5:
                    continue

                # Checksum dogrulama
                calc_csum = (0x55 + sum(payload[:4])) & 0xFF
                if calc_csum != payload[4]:
                    continue

                dist, strength = struct.unpack("<HH", payload[:4])
                self.distance = dist
                self.strength = strength
                self.last_valid_time = time.time()

            except (serial.SerialException, OSError):
                time.sleep(0.1)

    def get_luna_data(self):
        """Guncel sensor verilerini dondurur. 0.3 sn veri gelmezse 0 cm dondurur."""
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
                except Exception:
                    pass
                finally:
                    self.ser = None
