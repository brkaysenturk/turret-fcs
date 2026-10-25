"""
================================================================================
MODUL: ATIS VE ANGAJMAN YONETICISI (ENGAGEMENT MANAGER)
================================================================================
Bu modul, otonom taretin atis guvenlik kapilarini, zaman cizelgesini ve
2 asamali firlatma durum makinesini (SPINUP -> SETTLE -> FIRING -> RECOVER -> EVAL)
tek bir merkezden (SSOT) yonetir.

ZAMAN VE ASAMA SOZLESMESI:
- SPINUP: Volan motorlari devirlenir; taret hedefe yumusakca rampalanir (slew_factor).
- SETTLE: 0.15 s boyunca taret oturur; kilit dogrulanir.
- FIRING: 80 ms boyunca solenoid tetiklenir (FCSState.FIRING).
- RECOVER: Namlu sarsintisi yumusatilip optik eksene geri donulur.
- EVAL: Atis sonucu taktik panelde gosterilir.
================================================================================
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

from .config import CONFIG, FCSState


@dataclass
class EngagementState:
    """Anlik angajman durumunu temsil eden veri yapisi."""
    phase: Optional[str]
    slew_factor: float
    spinup_progress: Optional[float]
    fcs_fire_cmd_state: int
    is_physical_firing: bool
    is_active: bool
    status_text: str
    status_color: Tuple[int, int, int]
    spinup_duration: float = 0.25


class EngagementManager:
    """2 Asamali Balistik Atis ve Angajman Yoneticisi (FSM)."""

    def __init__(
        self,
        idle_spinup_time_s: float = CONFIG.flywheel.idle_spinup_time_s,
        cold_spinup_time_s: float = CONFIG.flywheel.cold_spinup_time_s,
        settle_time_s: float = CONFIG.ballistics.settle_time_s,
        firing_pulse_s: float = CONFIG.ballistics.firing_pulse_s,
        recovery_time_s: float = CONFIG.ballistics.recovery_time_s,
        eval_display_time_s: float = CONFIG.ballistics.eval_display_time_s,
    ):
        self.idle_spinup_time_s = float(idle_spinup_time_s)
        self.cold_spinup_time_s = float(cold_spinup_time_s)
        self.settle_time_s = float(settle_time_s)
        self.firing_pulse_s = float(firing_pulse_s)
        self.recovery_time_s = float(recovery_time_s)
        self.eval_display_time_s = float(eval_display_time_s)

        self.fire_start_time = 0.0
        self.is_firing_sequence = False
        self.spinup_duration = self.cold_spinup_time_s

    def trigger(
        self,
        now: float,
        is_locked: bool,
        is_range_valid: bool,
        is_flywheel_idle: bool = False,
        hit_probability: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """Atis tetikleme istegi. Guvenlik ve minimum isabet olasiligi kapilarini kontrol eder."""
        if not is_locked:
            return False, "TARGET NOT LOCKED!"
        if hit_probability is not None and float(hit_probability) < CONFIG.lock.min_hit_probability_pct:
            return False, f"HIT PROBABILITY TOO LOW ({float(hit_probability):.1f}% < {CONFIG.lock.min_hit_probability_pct:.0f}%)"
        if not is_range_valid:
            return False, "LIDAR RANGE NOT STABLE/VALID"
        if self.is_firing_sequence:
            return False, "ENGAGEMENT ALREADY IN PROGRESS"

        self.spinup_duration = self.idle_spinup_time_s if is_flywheel_idle else self.cold_spinup_time_s
        self.fire_start_time = float(now)
        self.is_firing_sequence = True
        return True, "2-STAGE ENGAGEMENT: SPIN-UP & SLEW ENGAGED"

    def abort(self):
        """Atisi veya angajmani aninda iptal eder."""
        self.fire_start_time = 0.0
        self.is_firing_sequence = False

    def update(self, now: float, is_locked: bool) -> EngagementState:
        """Anlik zamana gore atis durum makinesini gunceller."""
        if not self.is_firing_sequence or self.fire_start_time <= 0:
            default_state = FCSState.LOCKED if is_locked else FCSState.TRACKING
            return EngagementState(
                phase=None,
                slew_factor=0.0,
                spinup_progress=None,
                fcs_fire_cmd_state=default_state,
                is_physical_firing=False,
                is_active=False,
                status_text="STATUS: TARGET LOCKED" if is_locked else "STATUS: TRACKING",
                status_color=(0, 0, 255) if is_locked else (0, 191, 255),
            )

        t_fire = float(now - self.fire_start_time)
        t_spinup = self.spinup_duration
        t_settle = t_spinup + self.settle_time_s
        t_pulse = t_settle + self.firing_pulse_s
        t_recover = t_pulse + self.recovery_time_s
        t_total = t_recover + self.eval_display_time_s

        if t_fire < t_spinup:
            progress = float(np.clip(t_fire / max(1e-3, t_spinup), 0.0, 1.0))
            ms_left = max(0, int((1.0 - progress) * t_spinup * 1000))
            slew_factor = float(0.5 * (1.0 - np.cos(np.pi * progress)))
            return EngagementState(
                phase="SPINUP",
                slew_factor=slew_factor,
                spinup_progress=progress,
                fcs_fire_cmd_state=FCSState.LOCKED if is_locked else FCSState.TRACKING,
                is_physical_firing=True,
                is_active=True,
                status_text=f"STATUS: SPIN-UP & SLEW ({ms_left:3d}ms)",
                status_color=(0, 215, 255),
                spinup_duration=self.spinup_duration,
            )
        elif t_fire < t_settle:
            return EngagementState(
                phase="SETTLE",
                slew_factor=1.0,
                spinup_progress=1.0,
                fcs_fire_cmd_state=FCSState.LOCKED if is_locked else FCSState.TRACKING,
                is_physical_firing=True,
                is_active=True,
                status_text="STATUS: SETTLING AIM",
                status_color=(0, 215, 255),
                spinup_duration=self.spinup_duration,
            )
        elif t_fire < t_pulse:
            return EngagementState(
                phase="FIRING",
                slew_factor=1.0,
                spinup_progress=1.0,
                fcs_fire_cmd_state=FCSState.FIRING,
                is_physical_firing=True,
                is_active=True,
                status_text="STATUS: ENGAGING (SOLENOID RELEASE)",
                status_color=(0, 0, 255),
                spinup_duration=self.spinup_duration,
            )
        elif t_fire < t_recover:
            rec_p = float((t_fire - t_pulse) / max(1e-3, self.recovery_time_s))
            slew_factor = float(0.5 * (1.0 + np.cos(np.pi * rec_p)))
            return EngagementState(
                phase="RECOVER",
                slew_factor=slew_factor,
                spinup_progress=1.0,
                fcs_fire_cmd_state=FCSState.LOCKED if is_locked else FCSState.TRACKING,
                is_physical_firing=True,
                is_active=True,
                status_text="STATUS: RECOVERING BORESIGHT",
                status_color=(0, 255, 128),
                spinup_duration=self.spinup_duration,
            )
        elif t_fire < t_total:
            return EngagementState(
                phase="EVAL",
                slew_factor=0.0,
                spinup_progress=1.0,
                fcs_fire_cmd_state=FCSState.LOCKED if is_locked else FCSState.TRACKING,
                is_physical_firing=False,
                is_active=True,
                status_text="STATUS: ENGAGEMENT COMPLETED",
                status_color=(0, 255, 0),
                spinup_duration=self.spinup_duration,
            )
        else:
            self.abort()
            default_state = FCSState.LOCKED if is_locked else FCSState.TRACKING
            return EngagementState(
                phase=None,
                slew_factor=0.0,
                spinup_progress=None,
                fcs_fire_cmd_state=default_state,
                is_physical_firing=False,
                is_active=False,
                status_text="STATUS: TARGET LOCKED" if is_locked else "STATUS: TRACKING",
                status_color=(0, 0, 255) if is_locked else (0, 191, 255),
            )
