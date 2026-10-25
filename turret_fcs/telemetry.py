"""
================================================================================
MODUL: GERCEK ZAMANLI TELEMETRI V2 KAYIT MOTORU (TELEMETRY V2 / BLACKBOX)
================================================================================
Bu modul, otonom taretin calismasi sirasinda olculen tum durum, inovasyon, hata,
eyleyici, Kalman kestirimi ve sensor parametrelerini Schema v2 formatinda tamponlu
CSV olarak kaydeder. Ayni zamanda oturum basinda JSON metadata sidecar (.meta.json)
ve kritik taktiksel olay gunlugunu (_events.csv) uretir.

SCHEMA V2 KOLONLARI (35 KRITIK MUHENDISLIK PARAMETRESI):
- Urun ve Kare Kimligi: frame_id, time_sec, dt, latency_ms, fps
- Hedef Algilama ve Geometri: target_detected, target_conf, box_cx, box_cy, box_w, box_h
- Hata ve Eyleyici Komutlari: angular_err_pan, angular_err_tilt, cmd_pan, cmd_tilt
- Kalman ve Inovasyon: vx_raw, vy_raw, target_vel_x, target_vel_y, target_speed, innov_px
- Sensorler ve Menzil Kapilama: dist_cm, signal_strength, range_valid
- FCS Durum Makinasi ve Angajman: fcs_state, control_mode, fire_stage
- Kilit ve Balistik Kriterleri: is_locked, lock_counter, hit_prob
- Donanim Geri Besleme ve Devriye: turret_pan, turret_tilt, servo_pan_fb, servo_tilt_fb, patrol_status
================================================================================
"""

from dataclasses import dataclass, fields
import csv
import json
import math
import subprocess
import time
from datetime import datetime
from pathlib import Path

from .config import CONFIG


@dataclass
class TelemetryRow:
    """Schema v2 telemetri satiri veri yapisi (Tek Dogruluk Kaynagi - SSOT)."""
    frame_id: int = 0
    time_sec: float = 0.0
    dt: float = 0.0
    latency_ms: float = 0.0
    fps: int = 0
    target_detected: int = 0
    target_conf: float = 0.0
    box_cx: int = 0
    box_cy: int = 0
    box_w: int = 0
    box_h: int = 0
    angular_err_pan: float = 0.0
    angular_err_tilt: float = 0.0
    cmd_pan: float = 0.0
    cmd_tilt: float = 0.0
    vx_raw: float = 0.0
    vy_raw: float = 0.0
    target_vel_x: float = 0.0
    target_vel_y: float = 0.0
    target_speed: float = 0.0
    innov_px: float = 0.0
    dist_cm: int = 0
    signal_strength: int = 0
    range_valid: int = 0
    fcs_state: int = 0
    control_mode: str = "P"
    fire_stage: str = "STANDBY"
    is_locked: int = 0
    lock_counter: int = 0
    hit_prob: float = 0.0
    turret_pan: float = 0.0
    turret_tilt: float = 0.0
    servo_pan_fb: float = 0.0
    servo_tilt_fb: float = 0.0
    patrol_status: str = "STANDBY"

    def to_list(self):
        """Dataclass alanlarinin sirali deger listesini dondurur."""
        return [getattr(self, f.name) for f in fields(self)]


class TelemetryLogger:
    """
    Turret FCS Gercek Donanim Telemetri v2 Kaydedicisi (SSOT).
    Tamponlu ve kalici dosya taniticili (persistent file handle) disk yazimi,
    JSON metadata sidecar ve olay gunlugu ile tam gozlemlenebilirlik saglar.
    """

    SCHEMA_VERSION = "2.1"

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

        # Basliklar dogrudan TelemetryRow alanlarindan dinamik uretilir (pozisyon kaymasi imkansizdir)
        self.headers = [f.name for f in fields(TelemetryRow)]

        # Acik dosya taniticilari: Surekli open/close yapilmaz, I/O takilmasi (jitter) onlenir
        self.csv_file = open(self.csv_path, mode="w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.headers)
        self.csv_file.flush()

        self.events_file = open(self.events_path, mode="w", newline="", encoding="utf-8")
        self.events_writer = csv.writer(self.events_file)
        self.events_writer.writerow(["time_sec", "frame_id", "event_type", "message"])
        self.events_file.flush()

        self._init_metadata()

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
                "actuator": "Pan-Tilt MG996R servomechanism",
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
                "kalman": {
                    "sigma_a": CONFIG.kalman.sigma_a,
                    "r_var": CONFIG.kalman.r_var,
                    "velocity_deadband_low": CONFIG.kalman.velocity_deadband_low_px_s,
                    "velocity_deadband_high": CONFIG.kalman.velocity_deadband_high_px_s,
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
                "2": "FIRING (Solenoid Engagement)",
                "3": "ESTOP (Emergency Motor Power Off)",
            },
            "columns": self.headers,
        }
        with open(self.meta_path, "w", encoding="utf-8") as f:
            json.dump(meta_data, f, indent=4)

    def log_event(self, event_type, message):
        """Kritik sistem olaylarini zaman damgali olarak _events.csv dosyasina yazar."""
        if not self.is_active or self.events_file is None or self.events_file.closed:
            return
        now = time.time()
        elapsed = round(now - (self.start_time or now), 4)
        self.events_writer.writerow([elapsed, self.frame_id_counter, str(event_type), str(message)])
        self.events_file.flush()

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
        fcs_state=0,
        fire_stage=None,
        servo_pan_fb=None,
        servo_tilt_fb=None,
        frame_id=None,
        latency_ms=None,
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

        latency_val = round(float(latency_ms), 1) if latency_ms is not None else round(float(dt) * 1000.0, 1)

        # Hedef geometrisi (merkez, genislik, yukseklik)
        box_cx, box_cy, box_w, box_h = 0, 0, 0, 0
        if best_box is not None:
            x1, y1, x2, y2 = best_box[:4]
            box_cx = int((x1 + x2) // 2)
            box_cy = int((y1 + y2) // 2)
            box_w = int(abs(x2 - x1))
            box_h = int(abs(y2 - y1))

        # FCS parametreleri: kontrolcu ici filtre degil, gercek kamera acisal hatasi
        err_pan = round(float(getattr(fcs, "last_pan_err_deg", 0.0)), 3) if fcs else 0.0
        err_tilt = round(float(getattr(fcs, "last_tilt_err_deg", 0.0)), 3) if fcs else 0.0

        # Kalman Filtresi Inovasyon ve Hiz Metrikleri
        kalman = getattr(fcs, "kalman", None) if fcs else None
        if kalman is not None:
            target_vel = kalman.get_velocity()
            vx = float(target_vel[0])
            vy = float(target_vel[1])
            vx_raw = float(getattr(kalman, "raw_vx", vx))
            vy_raw = float(getattr(kalman, "raw_vy", vy))
            innov_px = round(float(getattr(kalman, "last_residual", 0.0)), 2)
        else:
            vx, vy = 0.0, 0.0
            vx_raw, vy_raw = 0.0, 0.0
            innov_px = 0.0

        target_speed = round(math.hypot(vx, vy), 2)

        # LiDAR menzil kapilama (range gating) durumu
        range_valid_flag = 1 if getattr(fcs, "range_valid", False) else 0

        # Durum, kontrol modu ve angajman asamasi
        control_mode = getattr(fcs, "control_mode", "P") if fcs else "P"
        if fire_stage is not None:
            stage_str = str(fire_stage)
        else:
            stage_str = "FIRING" if int(fcs_state) == 2 else ("TRACKING" if target_detected else "STANDBY")

        is_locked_flag = 1 if getattr(fcs, "is_locked", False) else 0
        lock_counter = int(getattr(fcs, "lock_counter", 0)) if fcs else 0
        hit_prob = round(float(getattr(fcs, "hit_probability", 0.0)), 1) if fcs else 0.0
        turret_p = round(float(getattr(fcs, "turret_pan_deg", 0.0)), 2) if fcs else 0.0
        turret_t = round(float(getattr(fcs, "turret_tilt_deg", 0.0)), 2) if fcs else 0.0
        p_status = str(getattr(fcs, "patrol_status", "STANDBY")) if fcs else "STANDBY"
        target_conf = round(float(getattr(fcs, "target_conf", 0.0)), 3) if (fcs and target_detected) else 0.0

        # Donanimsal servo geri beslemesi
        pan_fb = round(float(servo_pan_fb), 2) if servo_pan_fb is not None else -999.0
        tilt_fb = round(float(servo_tilt_fb), 2) if servo_tilt_fb is not None else -999.0

        row = TelemetryRow(
            frame_id=current_frame_id,
            time_sec=elapsed_sec,
            dt=round(dt, 4),
            latency_ms=latency_val,
            fps=int(fps),
            target_detected=1 if target_detected else 0,
            target_conf=target_conf,
            box_cx=box_cx,
            box_cy=box_cy,
            box_w=box_w,
            box_h=box_h,
            angular_err_pan=err_pan,
            angular_err_tilt=err_tilt,
            cmd_pan=round(float(pan_cmd), 3),
            cmd_tilt=round(float(tilt_cmd), 3),
            vx_raw=round(vx_raw, 2),
            vy_raw=round(vy_raw, 2),
            target_vel_x=round(vx, 2),
            target_vel_y=round(vy, 2),
            target_speed=target_speed,
            innov_px=innov_px,
            dist_cm=int(dist_cm or 0),
            signal_strength=int(strength or 0),
            range_valid=range_valid_flag,
            fcs_state=int(fcs_state),
            control_mode=str(control_mode),
            fire_stage=stage_str,
            is_locked=is_locked_flag,
            lock_counter=lock_counter,
            hit_prob=hit_prob,
            turret_pan=turret_p,
            turret_tilt=turret_t,
            servo_pan_fb=pan_fb,
            servo_tilt_fb=tilt_fb,
            patrol_status=p_status,
        )

        self.buffer.append(row.to_list())
        self.total_records += 1
        if target_detected:
            self.detected_records += 1
        if is_locked_flag:
            self.locked_records += 1

        if len(self.buffer) >= self.buffer_flush_size:
            self.flush()

    def flush(self):
        """Tampondaki verileri diske yazar."""
        if not self.buffer or self.csv_file is None or self.csv_file.closed:
            return
        self.csv_writer.writerows(self.buffer)
        self.csv_file.flush()
        self.buffer.clear()

    def close(self):
        """Log dosyalarini kapatir ve .meta.json dosyasina oturum ozetini yazar."""
        if not self.is_active:
            return
        self.flush()

        # Acik dosya taniticilarini guvenli sekilde kapat
        if self.csv_file is not None and not self.csv_file.closed:
            try:
                self.csv_file.close()
            except Exception:
                pass

        if self.events_file is not None and not self.events_file.closed:
            try:
                self.events_file.close()
            except Exception:
                pass

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

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

