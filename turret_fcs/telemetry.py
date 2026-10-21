"""
================================================================================
MODUL: GERCEK ZAMANLI TELEMETRI V2 KAYIT MOTORU (TELEMETRY V2 / BLACKBOX)
================================================================================
Bu modul, otonom taretin calismasi sirasinda olculen tum durum, inovasyon, hata,
eyleyici, Kalman kestirimi ve sensor parametrelerini Schema v2 formatinda tamponlu
CSV olarak kaydeder. Ayni zamanda oturum basinda JSON metadata sidecar (.meta.json)
ve kritik taktiksel olay gunlugunu (_events.csv) uretir.

SCHEMA V2 KOLONLARI (37 KRITIK MÜHENDİSLİK PARAMETRESİ):
- Urun ve Kare Kimligi: schema_version, frame_id, time_sec, dt, latency_ms, fps
- Hedef Algilama ve Geometri: target_detected, target_conf, box_cx, box_cy
- Hata ve Eyleyici Komutlari: angular_err_pan, angular_err_tilt, cmd_pan, cmd_tilt
- Kalman ve Inovasyon: vx_raw, vy_raw, target_vel_x, target_vel_y, target_speed, innov_px, sigma_a
- Kontrolcu Dinamikleri: kp_eff_pan, kp_eff_tilt, db_eff_pan, db_eff_tilt
- Sensorler (LiDAR & Sicaklik): dist_cm, signal_strength, sensor_temp_c
- FCS Durum Makinasi: fcs_state, control_mode, rl_weight
- Kilit ve Balistik Kriterleri: is_locked, lock_counter, lock_thr, hit_prob
- Donanim Geri Besleme ve Devriye: turret_pan, turret_tilt, servo_pan_fb, servo_tilt_fb, patrol_status
================================================================================
"""

import csv
import json
import math
import subprocess
import time
from datetime import datetime
from pathlib import Path

try:
    from .config import CONFIG
except (ImportError, ValueError):
    from config import CONFIG


class TelemetryLogger:
    """
    Turret FCS Gercek Donanim Telemetri v2 Kaydedicisi (SSOT).
    Tamponlu disk yazimi, JSON metadata sidecar ve olay gunlugu ile
    sistemin tam gozlemlenebilirligini (full observability) saglar.
    """

    SCHEMA_VERSION = "2.0"

    def __init__(self, output_dir=None, filename_prefix="telemetry_test"):
        if output_dir is None:
            base_dir = Path(__file__).resolve().parents[1]
            output_dir = base_dir / "logs"

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_dir / f"{filename_prefix}_{self.session_id}.csv"
        self.meta_path = self.output_dir / f"{filename_prefix}_{self.session_id}.meta.json"
        self.events_path = self.output_dir / f"{filename_prefix}_{self.session_id}_events.csv"

        self.start_time = None
        self.buffer = []
        self.buffer_flush_size = 30  # Her 30 karede bir diske yazar
        self.total_records = 0
        self.locked_records = 0
        self.detected_records = 0
        self.frame_id_counter = 0
        self.is_active = True

        self.headers = [
            "schema_version",    # "2.0"
            "frame_id",          # Artan kare indeksi (0, 1, 2...)
            "time_sec",          # Test baslangicindan itibaren gecen sure (s)
            "dt",                # Kare delta zamani (s)
            "latency_ms",        # Kare dongu gecikmesi (ms)
            "fps",               # Anlik FPS
            "target_detected",   # Hedef algilandi mi (1 / 0)
            "target_conf",       # Hedef YOLO guven skoru (0.00 - 1.00)
            "box_cx",            # Hedef merkez x (pixel)
            "box_cy",            # Hedef merkez y (pixel)
            "angular_err_pan",   # Aci hatasi Pan (derece)
            "angular_err_tilt",  # Aci hatasi Tilt (derece)
            "cmd_pan",           # Motora gonderilen Pan komutu (delta derece)
            "cmd_tilt",          # Motora gonderilen Tilt komutu (delta derece)
            "vx_raw",            # Kalman oncesi olcum hedef hizi Vx (px/s)
            "vy_raw",            # Kalman oncesi olcum hedef hizi Vy (px/s)
            "target_vel_x",      # Filtrelenmis hedef hizi Vx (px/s)
            "target_vel_y",      # Filtrelenmis hedef hizi Vy (px/s)
            "target_speed",      # Hedef bileske hizi (px/s)
            "innov_px",          # Kalman inovasyon / kalinti buyuklugu (pixel)
            "sigma_a",           # Dinamik Kalman surec ivme gurultusu (px/s^2)
            "kp_eff_pan",        # Uygulanan efektif Pan Kp kazanci
            "kp_eff_tilt",       # Uygulanan efektif Tilt Kp kazanci
            "db_eff_pan",        # Efektif Pan olu bolgesi (derece)
            "db_eff_tilt",       # Efektif Tilt olu bolgesi (derece)
            "dist_cm",           # TF-Luna mesafe (cm)
            "signal_strength",   # TF-Luna sinyal gucu
            "sensor_temp_c",     # TF-Luna sensor sicakligi (°C)
            "fcs_state",         # 0: TRACKING, 1: LOCKED, 2: FIRING, 3: ESTOP
            "control_mode",      # P / RL
            "rl_weight",         # Yapay zeka agirlik katsayisi (0.0 - 1.0)
            "is_locked",         # Kilit durumu (1 / 0)
            "lock_counter",      # Kilit kararlilik sayaci (0 - 10)
            "lock_thr",          # Kilit esik degeri (orn. 10)
            "hit_prob",          # Balistik Gauss isabet olasiligi (%0.0 - 100.0)
            "turret_pan",        # Taret dead-reckoning yatay acisi (derece)
            "turret_tilt",       # Taret dead-reckoning dikey acisi (derece)
            "servo_pan_fb",      # Donanimsal servo geri besleme acisi Pan (derece)
            "servo_tilt_fb",     # Donanimsal servo geri besleme acisi Tilt (derece)
            "patrol_status",     # Otonom devriye durum metni
        ]

        self._init_csv()
        self._init_metadata()
        self._init_events()

    def _get_git_commit(self):
        try:
            res = subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"],
                capture_output=True,
                text=True,
                timeout=2,
            )
            return res.stdout.strip() if res.returncode == 0 else "unknown"
        except Exception:
            return "unknown"

    def _init_csv(self):
        with open(self.csv_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)

    def _init_events(self):
        with open(self.events_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["time_sec", "frame_id", "event_type", "message"])

    def _init_metadata(self):
        meta_data = {
            "schema_version": self.SCHEMA_VERSION,
            "session_id": self.session_id,
            "session_start_iso": datetime.now().isoformat(),
            "git_commit": self._get_git_commit(),
            "hardware": {
                "camera": "Logitech C270 (1280x720 @ 30 FPS)",
                "microcontroller": "STM32F401RE (UART 115200 8N1)",
                "lidar": "Benewake TF-Luna I2C/UART",
                "actuator": "Pan-Tilt SG90/MG90S Servomechanism",
            },
            "system_config": {
                "optics": {
                    "h_fov_deg": CONFIG.optics.h_fov_deg,
                    "v_fov_deg": CONFIG.optics.v_fov_deg,
                    "default_width": CONFIG.optics.default_width,
                    "default_height": CONFIG.optics.default_height,
                },
                "control": {
                    "kp_pan": CONFIG.control.kp_pan,
                    "kp_tilt": CONFIG.control.kp_tilt,
                    "deadband_pan_deg": CONFIG.control.deadband_pan_deg,
                    "deadband_tilt_deg": CONFIG.control.deadband_tilt_deg,
                    "max_output_deg": CONFIG.control.max_output_deg,
                    "alpha_filter": CONFIG.control.alpha_filter,
                },
                "actuator": {
                    "max_step_pan_deg": CONFIG.actuator.max_step_pan_deg,
                    "max_step_tilt_deg": CONFIG.actuator.max_step_tilt_deg,
                    "servo_deadband_deg": CONFIG.actuator.servo_deadband_deg,
                },
                "ballistics": {
                    "projectile_speed_mps": CONFIG.ballistics.projectile_speed_mps,
                    "gravity_accel": CONFIG.ballistics.gravity_accel,
                    "spinup_time_s": CONFIG.ballistics.spinup_time_s,
                    "firing_pulse_s": CONFIG.ballistics.firing_pulse_s,
                    "recovery_time_s": CONFIG.ballistics.recovery_time_s,
                },
                "lock": {
                    "lock_threshold_frames": CONFIG.lock.lock_threshold_frames,
                    "lock_exit_margin_deg": CONFIG.lock.lock_exit_margin_deg,
                    "target_radius_m": CONFIG.lock.target_radius_m,
                },
                "mount": {
                    "pan_limit_min": CONFIG.mount.pan_limit_min,
                    "pan_limit_max": CONFIG.mount.pan_limit_max,
                    "tilt_limit_min": CONFIG.mount.tilt_limit_min,
                    "tilt_limit_max": CONFIG.mount.tilt_limit_max,
                    "pan_invert": CONFIG.mount.pan_invert,
                    "tilt_invert": CONFIG.mount.tilt_invert,
                    "servo_tau": CONFIG.mount.servo_tau,
                },
            },
            "firmware_protocol": {
                "0": "TRACKING (Dynamic Search & Slew)",
                "1": "LOCKED (Target inside Boresight Window)",
                "2": "FIRING (Solenoid Engagement - Reserved)",
                "3": "ESTOP (Emergency Motor Power Off)",
            },
            "columns": self.headers,
        }
        with open(self.meta_path, "w", encoding="utf-8") as f:
            json.dump(meta_data, f, indent=4)

    def log_event(self, event_type, message):
        """Kritik sistem olaylarini zaman damgali olarak _events.csv dosyasina yazar."""
        if not self.is_active:
            return
        now = time.time()
        elapsed = round(now - (self.start_time or now), 4)
        with open(self.events_path, mode="a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([elapsed, self.frame_id_counter, str(event_type), str(message)])

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
        temp_c=0.0,
        fcs_state=0,
        servo_pan_fb=None,
        servo_tilt_fb=None,
        frame_id=None,
    ):
        """Her kamera karesinde tek satir Telemetri v2 verisi ekler."""
        if not self.is_active:
            return

        now = time.time()
        if self.start_time is None:
            self.start_time = now
        elapsed_sec = round(now - self.start_time, 4)

        if frame_id is not None:
            current_frame_id = int(frame_id)
        else:
            current_frame_id = self.frame_id_counter
        self.frame_id_counter += 1

        latency_ms = round(float(dt) * 1000.0, 1)

        # Hedef merkez koordinatlari
        box_cx, box_cy = 0, 0
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            box_cx = int((x1 + x2) // 2)
            box_cy = int((y1 + y2) // 2)

        # FCS parametreleri
        err_pan = round(getattr(fcs, "actual_pan", 0.0), 3) if fcs else 0.0
        err_tilt = round(getattr(fcs, "actual_tilt", 0.0), 3) if fcs else 0.0

        # Kalman Filtresi Inovasyon ve Hiz Metrikleri
        kalman = getattr(fcs, "kalman", None) if fcs else None
        if kalman is not None:
            target_vel = kalman.get_velocity()
            vx = float(target_vel[0])
            vy = float(target_vel[1])
            vx_raw = float(getattr(kalman, "raw_vx", vx))
            vy_raw = float(getattr(kalman, "raw_vy", vy))
            innov_px = round(float(getattr(kalman, "last_residual", 0.0)), 2)
            sigma_a = round(float(getattr(kalman, "current_sigma_a", kalman.sigma_a_min)), 1)
        else:
            vx, vy = 0.0, 0.0
            vx_raw, vy_raw = 0.0, 0.0
            innov_px = 0.0
            sigma_a = 0.0

        target_speed = round(math.hypot(vx, vy), 2)

        # Kontrolcu dinamikleri
        p_ctrl = getattr(fcs, "p_controller", None) if fcs else None
        if p_ctrl is not None:
            kp_eff_pan = round(float(getattr(p_ctrl, "kp_frame_pan", CONFIG.control.kp_pan)), 4)
            kp_eff_tilt = round(float(getattr(p_ctrl, "kp_frame_tilt", CONFIG.control.kp_tilt)), 4)
            db_eff_pan = round(float(getattr(p_ctrl, "deadband_pan", CONFIG.control.deadband_pan_deg)), 3)
            db_eff_tilt = round(float(getattr(p_ctrl, "deadband_tilt", CONFIG.control.deadband_tilt_deg)), 3)
        else:
            kp_eff_pan = CONFIG.control.kp_pan
            kp_eff_tilt = CONFIG.control.kp_tilt
            db_eff_pan = CONFIG.control.deadband_pan_deg
            db_eff_tilt = CONFIG.control.deadband_tilt_deg

        control_mode = getattr(fcs, "control_mode", "P") if fcs else "P"
        rl_weight = round(getattr(fcs, "current_rl_weight", 0.0), 3) if fcs else 0.0
        is_locked_flag = 1 if getattr(fcs, "is_locked", False) else 0
        lock_counter = int(getattr(fcs, "lock_counter", 0)) if fcs else 0
        lock_thr = int(CONFIG.lock.lock_threshold_frames)
        hit_prob = round(float(getattr(fcs, "hit_probability", 0.0)), 1) if fcs else 0.0
        turret_p = round(float(getattr(fcs, "turret_pan_deg", 0.0)), 2) if fcs else 0.0
        turret_t = round(float(getattr(fcs, "turret_tilt_deg", 0.0)), 2) if fcs else 0.0
        p_status = str(getattr(fcs, "patrol_status", "STANDBY")) if fcs else "STANDBY"
        target_conf = round(float(getattr(fcs, "target_conf", 0.0)), 3) if (fcs and target_detected) else 0.0

        # Donanimsal servo geri beslemesi
        pan_fb = round(float(servo_pan_fb), 2) if servo_pan_fb is not None else -999.0
        tilt_fb = round(float(servo_tilt_fb), 2) if servo_tilt_fb is not None else -999.0

        row = [
            self.SCHEMA_VERSION,
            current_frame_id,
            elapsed_sec,
            round(dt, 4),
            latency_ms,
            int(fps),
            1 if target_detected else 0,
            target_conf,
            box_cx,
            box_cy,
            err_pan,
            err_tilt,
            round(float(pan_cmd), 3),
            round(float(tilt_cmd), 3),
            round(vx_raw, 2),
            round(vy_raw, 2),
            round(vx, 2),
            round(vy, 2),
            target_speed,
            innov_px,
            sigma_a,
            kp_eff_pan,
            kp_eff_tilt,
            db_eff_pan,
            db_eff_tilt,
            int(dist_cm or 0),
            int(strength or 0),
            round(float(temp_c or 0.0), 1),
            int(fcs_state),
            str(control_mode),
            rl_weight,
            is_locked_flag,
            lock_counter,
            lock_thr,
            hit_prob,
            turret_p,
            turret_t,
            pan_fb,
            tilt_fb,
            p_status,
        ]

        self.buffer.append(row)
        self.total_records += 1
        if target_detected:
            self.detected_records += 1
        if is_locked_flag:
            self.locked_records += 1

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
        """Log dosyasini kapatir ve .meta.json dosyasina oturum ozetini yazar."""
        if not self.is_active:
            return
        self.flush()
        self.is_active = False

        duration_s = round(time.time() - (self.start_time or time.time()), 2)
        mean_fps = round(self.total_records / max(0.1, duration_s), 1)
        lock_ratio = round(self.locked_records / max(1, self.total_records), 3)
        detect_ratio = round(self.detected_records / max(1, self.total_records), 3)

        # Metadata sidecar guncelleme
        if self.meta_path.exists():
            try:
                with open(self.meta_path, "r", encoding="utf-8") as f:
                    meta = json.load(f)
                meta["session_summary"] = {
                    "session_end_iso": datetime.now().isoformat(),
                    "duration_seconds": duration_s,
                    "total_frames": self.total_records,
                    "mean_fps": mean_fps,
                    "target_detection_ratio": detect_ratio,
                    "locked_ratio": lock_ratio,
                    "csv_file": self.csv_path.name,
                    "events_file": self.events_path.name,
                }
                with open(self.meta_path, "w", encoding="utf-8") as f:
                    json.dump(meta, f, indent=4)
            except Exception:
                pass

        print(f"[TELEMETRY v2] Oturum kapandi: Toplam {self.total_records} kare ({duration_s}s, {mean_fps} FPS) -> {self.csv_path.name}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
