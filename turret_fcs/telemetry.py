"""
================================================================================
MODUL: GERCEK ZAMANLI TELEMETRI KAYIT MOTORU (TELEMETRY LOGGER / BLACKBOX)
================================================================================
Bu modul, otonom taretin calismasi sirasinda her 33 ms'lik karede olculen tum durum,
hata, eyleyici ve sensor parametrelerini zaman damgali olarak CSV formatinda kaydeder.

KAYDEDILEN 21 KRITIK PARAMETRE:
- Zaman ve Basarim: time_sec, dt, fps
- Hedef Geometrisi ve Durumu: target_detected, box_cx, box_cy
- Kontrol Hatalari: angular_err_pan, angular_err_tilt, angular_err_total
- Eyleyici Komutlari: cmd_pan, cmd_tilt (derece/kare)
- Kalman Kestirimleri: target_vel_x, target_vel_y, target_speed (px/s)
- TF-Luna LiDAR: dist_cm, signal_strength
- Durum Makinasi: fcs_state, control_mode (P/RL), rl_weight
- Atis ve Kilit Kriterleri: lock_counter, is_locked, hit_prob (%)
================================================================================
"""

import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path


class TelemetryLogger:
    """
    Turret FCS Gercek Donanim Test Telemetri Kaydedicisi (SSOT).
    
    Kamera, YOLO, Kalman Filtresi, LiDAR, dinamik olu bolge ve motor komutlarini 
    arka planda performansi etkilemeden (tamponlu CSV olarak) kaydeder.
    Elde edilen loglar MATLAB veya Python ile analiz edilip tez grafiklerine donusturulebilir.
    """

    def __init__(self, output_dir=None, filename_prefix="telemetry_test"):
        if output_dir is None:
            base_dir = Path(__file__).resolve().parents[1]
            output_dir = base_dir / "logs"
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_dir / f"{filename_prefix}_{timestamp_str}.csv"

        self.start_time = None
        self.buffer = []
        self.buffer_flush_size = 30  # Her 30 karede bir diske yazar
        self.total_records = 0
        self.is_active = True

        self.headers = [
            "time_sec",          # Test baslangicindan itibaren gecen sure (s)
            "dt",                # Kare delta zamani (s)
            "fps",               # Anlik FPS
            "target_detected",   # Hedef algilandi mi (1 / 0)
            "target_conf",       # Hedef YOLO guven skoru (0.00 - 1.00)
            "box_cx",            # Hedef merkez x (pixel)
            "box_cy",            # Hedef merkez y (pixel)
            "angular_err_pan",   # Aci hatasi Pan (derece)
            "angular_err_tilt",  # Aci hatasi Tilt (derece)
            "cmd_pan",           # Motora gonderilen Pan komutu (delta derece)
            "cmd_tilt",          # Motora gonderilen Tilt komutu (delta derece)
            "target_vel_x",      # Kalman hedef hizi Vx (px/s)
            "target_vel_y",      # Kalman hedef hizi Vy (px/s)
            "target_speed",      # Hedef bileske hizi (px/s)
            "dist_cm",           # TF-Luna mesafe (cm)
            "signal_strength",   # TF-Luna sinyal gucu
            "fcs_state",         # 0: Takip, 1: Kilitli, 2: Ates
            "control_mode",      # P / RL
            "rl_weight",         # Yapay zeka agirlik katsayisi (%0.0 - 1.0)
            "is_locked",         # Kilit durumu (1 / 0)
            "lock_counter",      # Kilit kararlilik sayaci (0 - 10)
            "hit_prob",          # Balistik isabet olasiligi (%0.0 - 100.0)
            "turret_pan",        # Taret dead-reckoning yatay yonelim acisi (derece)
            "turret_tilt",       # Taret dead-reckoning dikey yonelim acisi (derece)
            "patrol_status"      # Otonom devriye durum metni
        ]

        self._init_csv()

    def _init_csv(self):
        with open(self.csv_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)

    def log_frame(
        self,
        dt,
        fps,
        target_detected=False,
        best_box=None,
        fcs=None,
        pan_cmd=0.0,
        tilt_cmd=0.0,
        dist_cm=0,
        strength=0,
        fcs_state=0
    ):
        """Her kamera karesinde tek satir telemetri verisi ekler."""
        if not self.is_active:
            return

        now = time.time()
        if self.start_time is None:
            self.start_time = now
        elapsed_sec = round(now - self.start_time, 4)

        # Hedef merkez koordinatlari
        box_cx, box_cy = 0, 0
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            box_cx = int((x1 + x2) // 2)
            box_cy = int((y1 + y2) // 2)

        # FCS parametreleri
        err_pan = round(getattr(fcs, "actual_pan", 0.0), 3) if fcs else 0.0
        err_tilt = round(getattr(fcs, "actual_tilt", 0.0), 3) if fcs else 0.0
        target_vel = getattr(fcs, "target_velocity", [0.0, 0.0]) if fcs else [0.0, 0.0]
        vx = float(target_vel[0])
        vy = float(target_vel[1])
        target_speed = round(math.hypot(vx, vy), 2)

        control_mode = getattr(fcs, "control_mode", "P") if fcs else "P"
        rl_weight = round(getattr(fcs, "current_rl_weight", 0.0), 3) if fcs else 0.0
        is_locked_flag = 1 if getattr(fcs, "is_locked", False) else 0
        lock_counter = int(getattr(fcs, "lock_counter", 0)) if fcs else 0
        hit_prob = round(float(getattr(fcs, "hit_probability", 0.0)), 1) if fcs else 0.0
        turret_p = round(float(getattr(fcs, "turret_pan_deg", 0.0)), 2) if fcs else 0.0
        turret_t = round(float(getattr(fcs, "turret_tilt_deg", 0.0)), 2) if fcs else 0.0
        p_status = str(getattr(fcs, "patrol_status", "STANDBY")) if fcs else "STANDBY"
        target_conf = round(float(getattr(fcs, "target_conf", 0.0)), 3) if (fcs and target_detected) else 0.0

        row = [
            elapsed_sec,
            round(dt, 4),
            int(fps),
            1 if target_detected else 0,
            target_conf,
            box_cx,
            box_cy,
            err_pan,
            err_tilt,
            round(float(pan_cmd), 3),
            round(float(tilt_cmd), 3),
            round(vx, 2),
            round(vy, 2),
            target_speed,
            int(dist_cm or 0),
            int(strength or 0),
            int(fcs_state),
            str(control_mode),
            rl_weight,
            is_locked_flag,
            lock_counter,
            hit_prob,
            turret_p,
            turret_t,
            p_status
        ]

        self.buffer.append(row)
        self.total_records += 1

        if len(self.buffer) >= self.buffer_flush_size:
            self.flush()

    def flush(self):
        """Tampondaki verileri diske yazar."""
        if not self.buffer:
            return
        with open(self.csv_path, mode="a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(self.buffer)
        self.buffer.clear()

    def close(self):
        """Log dosyasini kapatir ve ozet basar."""
        if not self.is_active:
            return
        self.flush()
        self.is_active = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
