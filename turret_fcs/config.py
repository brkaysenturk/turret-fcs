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
      2: FIRING    (Atis / Solenoid tahrigi - volan hazirsa solenoid itici tetiklenir)
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
    default_fps: int = 30

    def compute_fx(self, width: int = 1280) -> float:
        return float((width / 2.0) / math.tan(math.radians(self.h_fov_deg / 2.0)))

    def compute_fy(self, height: int = 720) -> float:
        return float((height / 2.0) / math.tan(math.radians(self.v_fov_deg / 2.0)))


@dataclass(frozen=True)
class ControlConfig:
    """P kontrolcu ve filtre parametreleri."""
    kp_pan: float = 0.1953      # P: 30 FPS bazinda Optuna TPE optimal kare basi kazanc
    kp_tilt: float = 0.1544     # P: 30 FPS bazinda Optuna TPE optimal kare basi kazanc
    deadband_pan_deg: float = 0.85
    deadband_tilt_deg: float = 0.95
    max_output_deg: float = 1.50
    max_command_step_deg: float = 1.50  # Firmware Servo_ApplyCorrection ust siniri
    cmd_accel_pan_deg_s2: float = 353.1  # P/RL pan komut ivme siniri (Optuna TPE optimal)
    cmd_accel_tilt_deg_s2: float = 213.6  # P/RL tilt komut ivme siniri (Optuna TPE optimal)
    lead_time_offset_s: float = 0.015    # Balistik lead'e eklenen sabit pay
    lead_time_min_s: float = 0.03        # Lead sure alt siniri
    lead_time_max_s: float = 0.40        # Lead sure ust siniri (dart ucus suresi olcegi)
    alpha_filter: float = 0.9306         # P giris EMA'sinda yeni deger agirligi (Optuna TPE optimal)
    ego_tau_s: float = 0.0682            # Ego icin komut adimi servo zaman sabiti (Optuna TPE optimal)


@dataclass(frozen=True)
class ActuatorConfig:
    """Eyleyici ve servo mekanik adim / kuantalama limitleri."""
    max_step_pan_deg: float = 1.50     # RL kare basi maksimum Pan adimi (30 FPS'te 45 deg/s)
    max_step_tilt_deg: float = 0.75    # RL kare basi maksimum Tilt adimi (30 FPS'te 22.5 deg/s - donanim eylemsizlik uyumu)
    servo_deadband_deg: float = 0.10   # STM32 0.1 deg UART kuantalama ve servo mekanik olu bolgesi
    backlash_deg: float = 1.61         # MG996R disli boslugu (canli telemetri Sistem Tanilama ile olculen 1.61 deg)


@dataclass(frozen=True)
class KalmanConfig:
    """Sabit hizli (CV) Kalman filtresi: tek Q/R cifti ve 150 px tekil-sicrama kapisi."""
    r_var: float = 63.21                 # Olcum gurultusu varyansi (px^2), Bayesyen Optuna TPE optimal
    sigma_a: float = 478.19              # Surec ivme gurultusu (px/s^2), Bayesyen Optuna TPE optimal
    initial_velocity_var: float = 90000.0  # Ilk hiz kovaryansi (px/s)^2, hizli yakinsama icin
    velocity_deadband_low_px_s: float = 30.0   # Bu hizin altinda cikis hizi sifir
    velocity_deadband_high_px_s: float = 60.0  # Bu hizdan itibaren cikis hizi tam gecerli
    outlier_jump_px: float = 150.0       # Tekil sahte kutu sicrama esigi (piksel inovasyon normu)
    reacquire_frames: int = 3            # Ust uste aykiri deger sonrasi yeni takip baslatma esigi


@dataclass(frozen=True)
class BallisticsConfig:
    """Dis balistik, yercekimi kompanzasyonu ve atesleme zamanlamalari."""
    projectile_speed_mps: float = 10.0
    gravity_accel: float = 9.80665
    default_distance_m: float = 2.0
    # Atis sekansi zamanlamalari (saniye; spin-up sureleri FlywheelConfig icindedir)
    settle_time_s: float = 0.12     # Slew sonrasi servo oturma (Monte Carlo optimal: 66.7ms tasiyici + 1.61 deg backlash)
    firing_pulse_s: float = 0.08    # Solenoid itici darbe suresi (80 ms - STM32 firmware ile uyumlu)
    recovery_time_s: float = 0.35   # Mekanik geri toplama suresi
    eval_display_time_s: float = 1.50  # Atis sonrasi EVAL gosterim suresi
    system_latency_s: float = 0.0667 # 66.7 ms (2.0 kare) kamera-yolo-uart-servo olculen tasiyici gecikmesi


@dataclass(frozen=True)
class RLConfig:
    """RL cikarim ve egitim ortaminin ortak sabitleri (tek kaynak)."""
    soft_deadband_enabled: bool = True   # Acisal yumusak olu bolge + taper anahtari
    reversal_damping: float = 0.2889        # Yon degistiren aksiyonun sonum orani (Optuna TPE optimal)
    reversal_hold_frames: int = 2        # Yon degisim sonumunu 2 kare (~66ms, gecikme+backlash) koru
    taper_pan_deg: float = 1.60           # Soft deadband gecis genisligi (pan - 1.61 deg backlash uyumlu)
    taper_tilt_deg: float = 1.60          # Soft deadband gecis genisligi (tilt - 1.61 deg backlash ile simetrik)
    nominal_dt_s: float = 0.03333        # Egitim kare periyodu (adim limiti olcekleme referansi)
    d_err_scale: float = 0.05            # Hata turevi gozlem olcegi (egitimle ayni)
    d_err_ema_alpha: float = 0.4386        # Hata turevi EMA katsayisi (Optuna TPE optimal)
    v_max_px_s: float = 300.0            # Hiz gozlemi Bryson olcegi
    dist_min_m: float = 0.5              # Mesafe gozlemi alt siniri
    dist_max_m: float = 5.0              # Mesafe gozlemi ust siniri
    yolo_noise_std: float = 3.5          # Ortam varsayilan YOLO kutu gurultusu (px)
    yolo_noise_std_min: float = 2.0      # Domain randomization alt siniri
    yolo_noise_std_max: float = 5.0      # Domain randomization ust siniri
    servo_tau_min_s: float = 0.035       # Domain randomization servo zaman sabiti
    servo_tau_max_s: float = 0.065
    episode_steps: int = 300             # Egitim bolumu uzunlugu
    lambda_u: float = 0.0104             # Kontrol eforu ceza katsayisi (Optuna TPE 14x150k optimal)
    mu_du: float = 0.0055                # Komut degisimi (jerk) ceza katsayisi (Optuna TPE 14x150k optimal)
    progress_gain: float = 2.0           # Hata azalma ilerleme odulu katsayisi
    error_gain: float = 0.05             # Kalici karesel hata ceza katsayisi
    bullseye_reward: float = 0.50        # Tam merkez (bullseye) arti odulu
    bullseye_threshold_norm: float = 0.05  # Bullseye normalize hata yaricapi esigi


@dataclass(frozen=True)
class LidarConfig:
    """TF-Luna menzil kapilama, sicrama reddi ve kararlilik filtreleri."""
    min_distance_m: float = 0.20
    max_distance_m: float = 8.00
    jump_threshold_m: float = 0.50       # Son gecerli mesafeden 50 cm'den fazla sapma = supheli
    stability_window: int = 3            # Ates icin gereken kararli okuma sayisi
    stability_tolerance_m: float = 0.10  # Bu pencerede izin verilen mesafe dalgalanmasi
    reacquire_frames: int = 3            # Gercek menzil degisimini kabul icin ust uste kare
    geometry_tolerance_ratio: float = 0.75  # Kutu boyutundan mesafe ile LiDAR uyumsuzlugu


@dataclass(frozen=True)
class FlywheelConfig:
    """Cift volan motoru: kilitliyken bosta donme ve ates boost parametreleri."""
    target_speed_rad_s: float = 2460.9   # 23.500 RPM nominal atis devri
    idle_speed_rad_s: float = 1600.0     # Kilitliyken bosta donme devri (~%65)
    idle_spinup_time_s: float = 0.25     # Bostan tam devre + kisa slew suresi
    cold_spinup_time_s: float = 1.00     # Bosta degilse emniyetli tam spin-up


@dataclass(frozen=True)
class MountConfig:
    """Gimbal mekanik limitleri ve servo dinamikleri."""
    pan_limit_min: float = -75.0
    pan_limit_max: float = 75.0
    tilt_limit_min: float = -25.0
    tilt_limit_max: float = 25.0
    pan_invert: bool = True
    tilt_invert: bool = False
    servo_tau: float = 0.0682       # 1. derece motor zaman sabiti (Optuna TPE optimal)


@dataclass(frozen=True)
class LockConfig:
    """Kilit kararlilik sayaci ve histerezis parametreleri."""
    lock_threshold_frames: int = 10
    lock_exit_margin_deg: float = 0.60
    target_radius_m: float = 0.15
    sigma_weapon_deg: float = 1.25   # DIY cift volan ve sunger dart gercekci ic balistik sacilmasi (derece)
    min_hit_probability_pct: float = 25.0  # Atis izni icin gereken minimum analitik isabet olasiligi (%)
    post_fire_grace_duration_frames: int = 22  # 0.75 saniye @ 30 FPS kilit muafiyeti (grace period)
    threshold_low_speed_px_s: float = 150.0
    threshold_high_speed_px_s: float = 350.0
    gate_slow_deg: float = 1.70
    gate_medium_deg: float = 2.25
    gate_fast_deg: float = 2.95


@dataclass(frozen=True)
class SelectorConfig:
    """Hedef secici ve filtreleme esikleri."""
    min_conf: float = 0.30
    track_min_conf: float = 0.16
    min_area: int = 400
    max_area: int = 400000
    min_dim: int = 18
    max_lost_frames: int = 5
    spatial_gate_px: float = 173.0
    min_aspect_ratio: float = 0.35
    max_aspect_ratio: float = 2.50
    yolo_imgsz: int = 640
    tracker_config: str = "bytetrack.yaml"


@dataclass(frozen=True)
class PatrolConfig:
    """Devriye, merkeze donus ve Figure-8 tarama parametreleri."""
    wait_delay_sec: float = 1.5
    scan_pan_amp: float = 45.0
    scan_tilt_amp: float = 16.0
    scan_period_sec: float = 8.0
    return_speed_dps: float = 20.0
    max_slew_per_frame: float = 1.5
    min_dispatch_deg: float = 0.95


@dataclass(frozen=True)
class HUDConfig:
    """MIL-STD HUD esik, sinyal, menzil ve olay gunlugu parametreleri."""
    hit_high_pct: int = 70
    hit_mid_pct: int = 40
    signal_strong: int = 1000
    signal_moderate: int = 100
    max_display_range_cm: int = 300
    min_healthy_fps: int = 25
    max_event_logs: int = 6
    default_event_lines: int = 4


@dataclass(frozen=True)
class PathConfig:
    """Agirlik ve yapay zeka model dosya yollari."""
    yolo_model: str = "weights/best.pt"
    fallback_yolo_model: str = "weights/yolov8n.pt"


@dataclass(frozen=True)
class TurretConfig:
    optics: OpticsConfig = OpticsConfig()
    control: ControlConfig = ControlConfig()
    actuator: ActuatorConfig = ActuatorConfig()
    kalman: KalmanConfig = KalmanConfig()
    ballistics: BallisticsConfig = BallisticsConfig()
    rl: RLConfig = RLConfig()
    flywheel: FlywheelConfig = FlywheelConfig()
    lidar: LidarConfig = LidarConfig()
    mount: MountConfig = MountConfig()
    lock: LockConfig = LockConfig()
    selector: SelectorConfig = SelectorConfig()
    patrol: PatrolConfig = PatrolConfig()
    hud: HUDConfig = HUDConfig()
    paths: PathConfig = PathConfig()
    telemetry_enabled: bool = True


CONFIG = TurretConfig()

