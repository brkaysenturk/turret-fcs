"""
================================================================================
MODUL: TARET ATIS KONTROL SISTEMI KONFIGURASYONU (SSOT - CONFIG)
================================================================================
Tum alt sistemlerin (Optik, Kontrol, Balistik, Eyleyici, Kilit, Yollar ve Durum)
ortak fiziksel ve algoritmik parametrelerini tek bir kaynakta (Single Source of Truth)
toplayan, harici kutuphane bagimliligi olmayan saf yaprak moduldur.
================================================================================
"""

from dataclasses import dataclass
from enum import IntEnum
import math


class FCSState(IntEnum):
    """
    FCS durum makinesi ve STM32 donanim protokol durumlari.
    STM32 firmware main.c protokolu ile birebir eslesir:
      0: TRACKING  (Hedef arama / dinamik takip)
      1: LOCKED    (Hedef kilit alaninda)
      2: FIRING    (Atis / Solenoid tahrigi - firmware revizyonunda rezerve)
      3: ESTOP     (Acil durum durdurma / tum motorlar enerjisiz)
    """
    TRACKING = 0
    LOCKED = 1
    FIRING = 2
    ESTOP = 3


@dataclass(frozen=True)
class OpticsConfig:
    """Logitech C270 kamera optik parametreleri."""
    h_fov_deg: float = 48.8
    v_fov_deg: float = 28.6
    default_width: int = 1280
    default_height: int = 720

    def compute_fx(self, width: int = 1280) -> float:
        return float((width / 2.0) / math.tan(math.radians(self.h_fov_deg / 2.0)))

    def compute_fy(self, height: int = 720) -> float:
        return float((height / 2.0) / math.tan(math.radians(self.v_fov_deg / 2.0)))


@dataclass(frozen=True)
class ControlConfig:
    """P kontrolcu ve filtre parametreleri."""
    kp_pan: float = 0.10        # 30 FPS bazinda kare basi kazanc (Kp_s = 3.0 1/s)
    kp_tilt: float = 0.05       # 30 FPS bazinda kare basi kazanc (Kp_s = 1.5 1/s)
    deadband_pan_deg: float = 0.85
    deadband_tilt_deg: float = 0.95
    max_output_deg: float = 1.35
    alpha_filter: float = 0.75


@dataclass(frozen=True)
class ActuatorConfig:
    """Eyleyici ve servo mekanik adim / kuantalama limitleri."""
    max_step_pan_deg: float = 0.55     # Kare basi maksimum Pan adimi (+-0.55 deg/kare = 16.5 deg/s)
    max_step_tilt_deg: float = 0.22    # Kare basi maksimum Tilt adimi (+-0.22 deg/kare = 6.6 deg/s)
    servo_deadband_deg: float = 0.10   # STM32 0.1 deg UART kuantalama ve servo mekanik olu bolgesi


@dataclass(frozen=True)
class BallisticsConfig:
    """Dis balistik, yercekimi kompanzasyonu ve atesleme zamanlamalari."""
    projectile_speed_mps: float = 10.0
    gravity_accel: float = 9.80665
    default_distance_m: float = 2.0
    # Atis sekansi zamanlamalari (saniye)
    spinup_time_s: float = 1.00     # Volan motorlarinin maksimum devre ve balistik slew acisina ulasma suresi
    firing_pulse_s: float = 0.08    # Solenoid itici darbe suresi (80 ms - STM32 firmware ile uyumlu)
    recovery_time_s: float = 0.35   # Mekanik geri toplama suresi
    system_latency_s: float = 0.080 # 80 ms kamera-yolo-uart-solenoid toplam tasiyici gecikmesi


@dataclass(frozen=True)
class MountConfig:
    """Gimbal mekanik limitleri ve servo dinamikleri."""
    pan_limit_min: float = -75.0
    pan_limit_max: float = 75.0
    tilt_limit_min: float = -25.0
    tilt_limit_max: float = 25.0
    pan_invert: bool = True
    tilt_invert: bool = False
    servo_tau: float = 0.045        # 1. derece motor zaman sabiti (45 ms)


@dataclass(frozen=True)
class LockConfig:
    """Kilit kararlilik sayaci ve histerezis parametreleri."""
    lock_threshold_frames: int = 10
    lock_exit_margin_deg: float = 0.60
    target_radius_m: float = 0.15
    sigma_weapon_deg: float = 1.25   # DIY cift volan ve sunger dart gercekci ic balistik sacilmasi (derece)
    post_fire_grace_duration_frames: int = 22  # 0.75 saniye @ 30 FPS kilit muafiyeti (grace period)


@dataclass(frozen=True)
class PathConfig:
    """Agirlik ve yapay zeka model dosya yollari."""
    yolo_model: str = "runs/detect/balloon_detector/weights/best.pt"
    fallback_yolo_model: str = "weights/yolov8n.pt"
    rl_model: str = "turret_fcs/rl_sim/models/turret_ppo_final.zip"


@dataclass(frozen=True)
class TurretConfig:
    optics: OpticsConfig = OpticsConfig()
    control: ControlConfig = ControlConfig()
    actuator: ActuatorConfig = ActuatorConfig()
    ballistics: BallisticsConfig = BallisticsConfig()
    mount: MountConfig = MountConfig()
    lock: LockConfig = LockConfig()
    paths: PathConfig = PathConfig()


CONFIG = TurretConfig()
