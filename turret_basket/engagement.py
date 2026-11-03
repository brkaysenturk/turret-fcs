"""
================================================================================
MODUL: BASKET ATIS VE ANGAJMAN YONETICISI (ENGAGEMENT MANAGER)
================================================================================
Bu modul, kulplu kupaya dart basket atisinda 2 asamali firlatma durum makinesini
(SPINUP_ELEVATE -> FIRING -> RECOVER -> COMPLETED) yoneten durum makinesidir (FSM).
================================================================================
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

from turret_basket.config import CONFIG
from turret_basket.protocol import STATE_TRACKING, STATE_LOCKED, STATE_FIRING


@dataclass
class EngagementStep:
    """Anlik angajman adimi durumu."""
    is_active: bool
    phase: Optional[str]               # "SPINUP_ELEVATE", "FIRING", "RECOVER", None
    step_tilt: float
    cmd_state: int                     # STATE_TRACKING, STATE_LOCKED, STATE_FIRING
    just_fired: bool                   # Solenoid pulse ilk tetiklendiginde True
    is_finished: bool                  # Sekans bittiginde True
    active_fire_speed: float
    active_fire_tilt: float
    applied_super_elevation: float


class BasketEngagement:
    """Kulplu Kupa 2-Asamali Balistik Firlatma Durum Makinesi."""

    def __init__(
        self,
        spinup_time_s: float = CONFIG.ballistics.spinup_time_s,
        firing_pulse_s: float = CONFIG.ballistics.firing_pulse_s,
        recovery_time_s: float = CONFIG.ballistics.recovery_time_s,
    ):
        self.spinup_time_s = float(spinup_time_s)
        self.firing_pulse_s = float(firing_pulse_s)
        self.recovery_time_s = float(recovery_time_s)
        self.total_seq_time_s = self.spinup_time_s + self.firing_pulse_s + self.recovery_time_s

        self.is_active = False
        self.start_time = 0.0
        self.dart_fired = False

        self.tilt_at_lock = 0.0
        self.active_fire_speed = 4.4
        self.active_fire_tilt = 21.8
        self.applied_super_elevation = 22.6

    def trigger(
        self,
        now: float,
        tilt_at_lock: float,
        fire_speed: float,
        fire_tilt: float,
        super_elevation: float,
    ) -> Tuple[bool, str]:
        """Atis sekansini baslatir."""
        if self.is_active:
            return False, "THROW ALREADY IN PROGRESS"

        self.is_active = True
        self.start_time = float(now)
        self.dart_fired = False
        self.tilt_at_lock = float(tilt_at_lock)
        self.active_fire_speed = float(fire_speed)
        self.active_fire_tilt = float(fire_tilt)
        self.applied_super_elevation = float(super_elevation)
        return True, f"THROW INITIATED: +{self.applied_super_elevation:.1f} deg & spin-up"

    def abort(self):
        """Atis sekansini derhal sonlandirir."""
        self.is_active = False
        self.start_time = 0.0
        self.dart_fired = False

    def update(self, now: float, current_tilt: float) -> EngagementStep:
        """Anlik zamana ve tilt acisina gore atis FSM durumunu ilerletir."""
        if not self.is_active or self.start_time <= 0:
            return EngagementStep(
                is_active=False,
                phase=None,
                step_tilt=0.0,
                cmd_state=STATE_TRACKING,
                just_fired=False,
                is_finished=False,
                active_fire_speed=self.active_fire_speed,
                active_fire_tilt=self.active_fire_tilt,
                applied_super_elevation=self.applied_super_elevation,
            )

        elapsed = now - self.start_time
        t_spinup = self.spinup_time_s
        t_fire_end = t_spinup + self.firing_pulse_s
        t_total = self.total_seq_time_s

        # 1. Asama: Super-elevasyon acisina yuksel ve spin-up yap (0 - 0.60s)
        # Model 2 mutlak atis acisini (active_fire_tilt ~21-23 deg) hesaplar
        if elapsed < t_spinup:
            target_elevated_tilt = self.active_fire_tilt
            tilt_error = target_elevated_tilt - current_tilt
            step_tilt = float(np.clip(tilt_error * 0.40, -1.50, 1.50))
            return EngagementStep(
                is_active=True,
                phase="SPINUP_ELEVATE",
                step_tilt=step_tilt,
                cmd_state=STATE_LOCKED,
                just_fired=False,
                is_finished=False,
                active_fire_speed=self.active_fire_speed,
                active_fire_tilt=self.active_fire_tilt,
                applied_super_elevation=self.applied_super_elevation,
            )

        # 2. Asama: Solenoid darbesi ile dart firlat (0.60 - 0.70s)
        elif elapsed < t_fire_end:
            just_fired = False
            if not self.dart_fired:
                self.dart_fired = True
                just_fired = True

            return EngagementStep(
                is_active=True,
                phase="FIRING",
                step_tilt=0.0,
                cmd_state=STATE_FIRING,
                just_fired=just_fired,
                is_finished=False,
                active_fire_speed=self.active_fire_speed,
                active_fire_tilt=self.active_fire_tilt,
                applied_super_elevation=self.applied_super_elevation,
            )

        # 3. Asama: Geri toplama ve kilit acisina yumusak inis (0.70 - 1.60s)
        elif elapsed < t_total:
            rec_progress = (elapsed - t_fire_end) / max(1e-4, t_total - t_fire_end)
            target_tilt = self.active_fire_tilt + (self.tilt_at_lock - self.active_fire_tilt) * rec_progress
            tilt_error = target_tilt - current_tilt
            step_tilt = float(np.clip(tilt_error * 0.40, -1.50, 1.50))
            return EngagementStep(
                is_active=True,
                phase="RECOVER",
                step_tilt=step_tilt,
                cmd_state=STATE_TRACKING,
                just_fired=False,
                is_finished=False,
                active_fire_speed=self.active_fire_speed,
                active_fire_tilt=self.active_fire_tilt,
                applied_super_elevation=self.applied_super_elevation,
            )

        # 4. Asama: Toparlama tamamlandi
        else:
            self.is_active = False
            self.dart_fired = False
            return EngagementStep(
                is_active=False,
                phase=None,
                step_tilt=0.0,
                cmd_state=STATE_TRACKING,
                just_fired=False,
                is_finished=True,
                active_fire_speed=self.active_fire_speed,
                active_fire_tilt=self.active_fire_tilt,
                applied_super_elevation=self.applied_super_elevation,
            )

