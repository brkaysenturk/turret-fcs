"""
================================================================================
BASKET CHALLENGE: PROJE KONFIGURASYONU (CONFIG)
================================================================================
Bu modul, kulplu kupaya (mug) dart atma gorevi icin ozel olarak sadelestirilmis
tum donanim, kamera, hedef, balistik ve RL parametrelerini icerir.
================================================================================
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List

# Proje Koku: C:\YTU\turret-fcs\turret_fcs
PROJECT_ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class CameraConfig:
    """Kamera ve optik kalibrasyon parametreleri."""
    device_id: int = 0
    camera_name_substr: str = "C270"
    width: int = 640
    height: int = 480
    fps: int = 30
    h_fov_deg: float = 48.8    # Logitech C270 yatay gorunur alan
    v_fov_deg: float = 38.0    # Logitech C270 dikey gorunur alan

    @property
    def fx(self) -> float:
        import math
        return float(self.width / (2.0 * math.tan(math.radians(self.h_fov_deg / 2.0))))

    @property
    def fy(self) -> float:
        import math
        return float(self.height / (2.0 * math.tan(math.radians(self.v_fov_deg / 2.0))))


@dataclass(frozen=True)
class HardwareConfig:
    """STM32 ve servo mekanik parametreleri."""
    baudrate: int = 115200
    stlink_vcp_keyword: str = "STMicroelectronics STLink Virtual COM Port"
    manual_port: str = "COM3"

    # Donanimsal Aci Limitleri (Mekanik 0 - 180 derece)
    pan_min_deg: float = 15.0
    pan_max_deg: float = 165.0
    pan_center_deg: float = 90.0

    tilt_min_deg: float = 65.0
    tilt_max_deg: float = 115.0
    tilt_center_deg: float = 90.0

    # Adim limitleri (kare basina max servo hareketi)
    max_step_deg: float = 1.50   # 30 FPS'te 45 derece/saniye

    # Yon Inversiyonu (turret_fcs ile %100 uyumlu)
    pan_invert: bool = True
    tilt_invert: bool = False


@dataclass(frozen=True)
class CupTargetConfig:
    """Hedef kupa / bardak fiziksel ve tespit parametreleri."""
    yolo_model_path: str = str(PROJECT_ROOT / "weights" / "yolov8n.pt")
    target_classes: List[int] = field(default_factory=lambda: [41, 40, 39, 44])  # 41: cup, 40: wine glass, 39: bottle, 44: bowl
    confidence_threshold: float = 0.25
    ema_alpha: float = 0.80      # Sabit bardak kutu titremesini onleyen EMA

    # Bardagin Fiziksel Boyutlari (metre)
    mouth_diameter_m: float = 0.090   # 9.0 cm agiz capi
    height_m: float = 0.106           # 10.6 cm yukseklik
    target_distance_min_m: float = 1.0
    target_distance_max_m: float = 3.0


@dataclass(frozen=True)
class BallisticsConfig:
    """Sunger dart firlatma ve dinamik volan hizi parametreleri (FF-180 6V 23500 RPM)."""
    default_speed_mps: float = 10.0      # Nominal cikis hizi
    min_speed_mps: float = 5.5           # FF-180 dusuk PWM devri (yumusak asirtma: 5.5 m/s)
    max_speed_mps: float = 16.5          # FF-180 23500 RPM tam devir (guclu firlatma: 16.5 m/s)
    projectile_mass_kg: float = 0.012     # 12 gram sunger dart
    drag_coefficient: float = 0.60       # Silindirik govde surukleme katsayisi
    cross_area_m2: float = 0.000132      # 13 mm cap kesit alani
    air_density: float = 1.225           # kg/m^3
    gravity: float = 9.80665
    firing_pulse_s: float = 0.080        # 80 ms solenoid itme darbesi


@dataclass(frozen=True)
class RLConfig:
    """2. Model: Balistik Aci ve Volan Hizi Ajan Sabitleri."""
    obs_dim: int = 2        # [norm_distance, norm_height_offset]
    act_dim: int = 2        # [tilt_offset_deg, flywheel_speed_mps]

    # Atis Karar Limitleri (Gercekci Parabolik Asirtma ve Motor Devri)
    tilt_offset_min_deg: float = 5.0     # Min ek yukselis acisi
    tilt_offset_max_deg: float = 24.5    # Max ek yukselis acisi (Taret mekanik limiti 25.0°)
    speed_min_mps: float = 3.0           # Min dart hizi (Yumusak asirtmali lob: 3.0 m/s)
    speed_max_mps: float = 12.0          # Max dart hizi (Uzak mesafe firlatma: 12.0 m/s)

    basket_reward: float = 100.0
    proximity_scale: float = 20.0
    speed_penalty_weight: float = 0.03   # Enerji tasarrufu: gereksiz yuksek devir cezasi


@dataclass(frozen=True)
class BasketAppConfig:
    """Ana konfigürasyon paketi."""
    camera: CameraConfig = field(default_factory=CameraConfig)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    cup: CupTargetConfig = field(default_factory=CupTargetConfig)
    ballistics: BallisticsConfig = field(default_factory=BallisticsConfig)
    rl: RLConfig = field(default_factory=RLConfig)


CONFIG = BasketAppConfig()
