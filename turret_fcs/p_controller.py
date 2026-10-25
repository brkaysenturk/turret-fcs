"""
================================================================================
MODUL: URETIM P KONTROLCUSU (P CONTROLLER)
================================================================================
Tip-1 P omurgasini koruyan, gereksiz cift filtrelemeyi ve sabit slew tavanini
kaldiran dogrusal Tip-1 P kontrolcusu:

1. ESKI SORUN - CIFT FILTRE VE SLEW TAVANI:
   Eski kontrolcu kameradan gelen aciyi compute_angles icinde EMA ile
   filtreliyor, compute_p icinde tekrar EMA'liyor ve ustune 0.20 derece/kare
   pan slew koyuyordu. 30 FPS'de bu 6 derece/s yapay tavan demektir.
   Yeni kontrolcu tek EMA (alpha = 0.85) uygular; sabit pan slew yerine komut
   ivme siniri (pan 400, tilt 267 derece/s^2) kullanarak steady-state hiz
   tavanini kaldirir, kalkis ve donuslerde mekanik darbeleri onler.

2. KONTROL STRUKTURU:
   - Tek EMA (tek filtre katmani), varsayilan alpha = 0.85.
   - Yumusak olu bolge korunur (pan 0.85, tilt 0.95 derece).
   - Sabit oransal kazanc (nominal 30 FPS: pan Kp=0.22, tilt Kp=0.11).
   - Pan/tilt icin ayri ivme (jerk) siniri; steady-state takip hizi kisitlanmaz.
   - Ki ve Kd yoktur. Hiz feedforward'i yoktur; Kalman hizi P motoruna girmez.

3. DOGRULAMA:
   Webots profil taramasi ve gercek taret geri bildirimi ile taban kazanc
   0.22/0.11 olarak belirlenmistir. (0.30 gercek donanimda rezonansa girmistir).
================================================================================
"""

import numpy as np

from .config import CONFIG


class PController:
    """Tek filtreli, ivme sinirli, yumusak olu bolgeli saf Tip-1 P kontrolcusu."""

    DEFAULT_DISTANCE_M = CONFIG.ballistics.default_distance_m
    PROJECTILE_SPEED_MPS = CONFIG.ballistics.projectile_speed_mps
    GRAVITY_ACCEL = CONFIG.ballistics.gravity_accel

    MAX_OUTPUT_DEG = CONFIG.control.max_output_deg
    MAX_OUTPUT_TILT_DEG = CONFIG.actuator.max_step_tilt_deg
    DEADBAND_PAN_DEG = CONFIG.control.deadband_pan_deg
    DEADBAND_TILT_DEG = CONFIG.control.deadband_tilt_deg
    ALPHA_FILTER = CONFIG.control.alpha_filter
    KP_PAN_FRAME = CONFIG.control.kp_pan
    KP_TILT_FRAME = CONFIG.control.kp_tilt
    CMD_ACCEL_PAN = CONFIG.control.cmd_accel_pan_deg_s2
    CMD_ACCEL_TILT = CONFIG.control.cmd_accel_tilt_deg_s2

    def __init__(
        self,
        kp_pan=KP_PAN_FRAME,
        kp_tilt=KP_TILT_FRAME,
        alpha=ALPHA_FILTER,
        max_output=MAX_OUTPUT_DEG,
        max_output_tilt=MAX_OUTPUT_TILT_DEG,
        deadband_pan=DEADBAND_PAN_DEG,
        deadband_tilt=DEADBAND_TILT_DEG,
        cmd_accel_pan_deg_s2=CMD_ACCEL_PAN,
        cmd_accel_tilt_deg_s2=CMD_ACCEL_TILT,
    ):
        self.kp_frame_pan = float(kp_pan)
        self.kp_frame_tilt = float(kp_tilt)

        self.alpha = float(alpha)
        self.max_output = float(max_output)
        self.max_output_tilt = float(max_output_tilt)
        self.deadband_pan = float(deadband_pan)
        self.deadband_tilt = float(deadband_tilt)

        self.cmd_accel_pan_deg_s2 = max(1.0, float(cmd_accel_pan_deg_s2))
        self.cmd_accel_tilt_deg_s2 = max(1.0, float(cmd_accel_tilt_deg_s2))

        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0

    def compute_angles(self, bx, by, center_x, center_y, fx, fy, dt=CONFIG.rl.nominal_dt_s):
        """Kamera merkezinden hedefin acisal sapmasini hesaplar; tek EMA uygular."""
        target_err_x = bx - center_x
        target_err_y = by - center_y
        raw_pan = np.degrees(np.arctan(target_err_x / max(fx, 1e-6)))
        raw_tilt = np.degrees(np.arctan(-target_err_y / max(fy, 1e-6)))

        if dt is None or float(dt) <= 0.0:
            alpha_eff = self.alpha
        else:
            dt_safe = float(np.clip(float(dt), 0.005, 0.200))
            base_retain = float(np.clip(1.0 - self.alpha, 0.0, 1.0))
            alpha_eff = float(1.0 - (base_retain ** (dt_safe / CONFIG.rl.nominal_dt_s)))
        self.actual_pan = alpha_eff * raw_pan + (1.0 - alpha_eff) * self.actual_pan
        self.actual_tilt = alpha_eff * raw_tilt + (1.0 - alpha_eff) * self.actual_tilt
        return self.actual_pan, self.actual_tilt

    def compute_gravity_comp(self, dist_cm):
        """Analitik yercekimi dususu kompanzasyonu: theta = arctan(g * D / (2 * v^2))."""
        min_dist_cm = CONFIG.lidar.min_distance_m * 100.0
        target_d_m = (dist_cm / 100.0) if (dist_cm is not None and dist_cm >= min_dist_cm) else self.DEFAULT_DISTANCE_M
        v_sq_2 = 2.0 * (self.PROJECTILE_SPEED_MPS ** 2)
        return float(np.degrees(np.arctan((self.GRAVITY_ACCEL * target_d_m) / v_sq_2)))

    def compute_lead_angles(self, d_lead_x, d_lead_y, fx, fy):
        """Hedefin gelecekteki konumuna dogru atis onleme acilarini hesaplar."""
        lead_pan = np.degrees(np.arctan(d_lead_x / max(fx, 1e-6)))
        lead_tilt = np.degrees(np.arctan(-d_lead_y / max(fy, 1e-6)))
        return float(lead_pan), float(lead_tilt)

    def max_step_for(self, dt, axis="pan"):
        """FPS'ten bagimsiz kare basi adim limiti (30 FPS'te pan: 1.50, tilt: 0.75)."""
        dt_safe = max(1e-3, float(dt))
        rate_scale = float(np.clip(dt_safe / CONFIG.rl.nominal_dt_s, 0.25, 3.0))
        if axis == "tilt":
            return float(min(self.max_output_tilt * rate_scale, CONFIG.actuator.max_step_tilt_deg))
        return float(min(self.max_output * rate_scale, CONFIG.control.max_command_step_deg))

    def _axis_command(self, error_deg, kp_frame, deadband, accel_deg_s2, previous, dt):
        abs_error = abs(float(error_deg))
        if abs_error <= deadband:
            desired = 0.0
        else:
            effective = float(np.sign(error_deg)) * (abs_error - deadband)
            # kp_frame 30 FPS nominal kazancidir; dt ile olceklenir.
            desired = kp_frame * effective * (dt / CONFIG.rl.nominal_dt_s)

        # Komut derece/kare cinsinden oldugu icin ivme siniri: a * dt^2 (derece/kare)
        max_delta = accel_deg_s2 * (dt ** 2)
        return float(np.clip(desired, previous - max_delta, previous + max_delta))

    def compute_p(self, active_pan, active_tilt, dt=0.033):
        """Tek filtreli, ivme sinirli, yumusak olu bolgeli saf Tip-1 P komutu."""
        effective_dt = max(0.010, min(float(dt), 0.100))

        u_pan = self._axis_command(
            active_pan, self.kp_frame_pan, self.deadband_pan,
            self.cmd_accel_pan_deg_s2, self.smooth_cmd_pan, effective_dt,
        )
        u_tilt = self._axis_command(
            active_tilt, self.kp_frame_tilt, self.deadband_tilt,
            self.cmd_accel_tilt_deg_s2, self.smooth_cmd_tilt, effective_dt,
        )

        max_step_pan = self.max_step_for(dt, axis="pan")
        max_step_tilt = self.max_step_for(dt, axis="tilt")
        u_pan = float(np.clip(u_pan, -max_step_pan, max_step_pan))
        u_tilt = float(np.clip(u_tilt, -max_step_tilt, max_step_tilt))
        self.smooth_cmd_pan = u_pan
        self.smooth_cmd_tilt = u_tilt
        return u_pan, u_tilt

    def reset(self):
        """Kontrolcunun ic durumlarini sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0

    def on_target_lost(self):
        """Hedef kaybedildiginde komut ivme rampasi hafizasini sifirlar."""
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0

