"""
================================================================================
MODUL: URETIM P KONTROLCUSU (P CONTROLLER)
================================================================================
Tip-1 P omurgasini koruyan, fiziksel hiz uzayinda (deg/s) asimetrik ivme
sinirlamali ve 3-kare yoldaki komut ongorulu (Smith Predictor) Tip-1 P kontrolcusu:

1. KONTROL STRUKTURU:
   - Tek EMA (tek filtre katmani), ilk tespit karesinde sifir gecikmeli baslatma.
   - Yumusak olu bolge korunur (pan 0.85, tilt 0.95 derece).
   - Sabit oransal kazanc (nominal 30 FPS: pan Kp=0.2439, tilt Kp=0.1487).
   - ONERI 1 (Asimetrik Kalkis / Serbest Frenleme): Kalkista ve sert yon
     degisimlerinde ivme siniri (cmd_accel) mekanik vuruntuyu onler; hedefe
     varista (ayni yonde yavaslama veya olu bolge icinde durma) ivme siniri
     kaldirilarak sifir asim (zero overshoot) ile aninda durus saglanir.
   - ONERI 2 (3-Kare Smith Predictor / In-Flight Damping): 90-96 ms (3 kare)
     goruntu gecikmesi + 30 ms servo lag suresince yolda olan son 3 karelik
     motor komutunun etkisi, taret hedefe yaklasirken hatadan dusulerek
     koseden girislerdeki ilk kilitlenme osilasyonu tamamen onlenir.
================================================================================
"""

import numpy as np

from .config import CONFIG
from .ballistics import compute_rk4_super_elevation


class PController:
    """Tek filtreli, asimetrik hiz-ivme sinirli ve 3-kare yoldaki komut ongorulu Tip-1 P kontrolcusu."""

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
        asymmetric_braking=CONFIG.control.asymmetric_braking,
        in_flight_damping=CONFIG.control.in_flight_damping,
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
        self.asymmetric_braking = bool(asymmetric_braking)
        self.in_flight_damping = max(0.0, float(in_flight_damping))

        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self._has_target = False
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0
        self.command_speed_pan_deg_s = 0.0
        self.command_speed_tilt_deg_s = 0.0
        self.cmd_hist_pan_1 = 0.0
        self.cmd_hist_pan_2 = 0.0
        self.cmd_hist_pan_3 = 0.0
        self.cmd_hist_tilt_1 = 0.0
        self.cmd_hist_tilt_2 = 0.0
        self.cmd_hist_tilt_3 = 0.0

    @staticmethod
    def normalize_dt(dt):
        """Gecerli pozitif dt degerini dondurur; gecersizse nominal periyoda doner."""
        if dt is None:
            return CONFIG.rl.nominal_dt_s
        dt = float(dt)
        return dt if np.isfinite(dt) and dt > 0.0 else CONFIG.rl.nominal_dt_s

    def compute_angles(self, bx, by, center_x, center_y, fx, fy, dt=CONFIG.rl.nominal_dt_s):
        """Kamera merkezinden hedefin acisal sapmasini hesaplar; tek EMA uygular."""
        target_err_x = bx - center_x
        target_err_y = by - center_y
        raw_pan = float(np.degrees(np.arctan(target_err_x / max(fx, 1e-6))))
        raw_tilt = float(np.degrees(np.arctan(-target_err_y / max(fy, 1e-6))))

        if not self._has_target:
            self.actual_pan = raw_pan
            self.actual_tilt = raw_tilt
            self._has_target = True
            return self.actual_pan, self.actual_tilt

        dt_safe = self.normalize_dt(dt)
        base_retain = float(np.clip(1.0 - self.alpha, 0.0, 1.0))
        alpha_eff = float(1.0 - (base_retain ** (dt_safe / CONFIG.rl.nominal_dt_s)))
        self.actual_pan = alpha_eff * raw_pan + (1.0 - alpha_eff) * self.actual_pan
        self.actual_tilt = alpha_eff * raw_tilt + (1.0 - alpha_eff) * self.actual_tilt
        return self.actual_pan, self.actual_tilt

    def compute_gravity_comp(self, dist_cm):
        """Yercekimi ve aerodinamik surukleme (drag) kompanzasyonu (RK4 veya analitik)."""
        min_dist_cm = CONFIG.lidar.min_distance_m * 100.0
        target_d_m = (dist_cm / 100.0) if (dist_cm is not None and dist_cm >= min_dist_cm) else self.DEFAULT_DISTANCE_M
        if CONFIG.ballistics.drag_enabled:
            theta_deg, _ = compute_rk4_super_elevation(
                target_d_m,
                v0=self.PROJECTILE_SPEED_MPS,
                g=self.GRAVITY_ACCEL,
                k=CONFIG.ballistics.drag_k,
            )
            raw_comp = float(theta_deg)
        else:
            v_sq_2 = 2.0 * (self.PROJECTILE_SPEED_MPS ** 2)
            raw_comp = float(np.degrees(np.arctan((self.GRAVITY_ACCEL * target_d_m) / v_sq_2)))

        # CAD v10 Paralaks Telafisi: Namlu kameranin camera_to_muzzle_z_m kadar yukarisindadir.
        # Kamera hedefe baktiginda namlu dogal olarak atan(dz / d) kadar yukari bakar.
        parallax_deg = float(np.degrees(np.arctan(CONFIG.ballistics.camera_to_muzzle_z_m / max(0.20, target_d_m))))
        return float(raw_comp - parallax_deg)

    def compute_lead_angles(self, d_lead_x, d_lead_y, fx, fy):
        """Hedefin gelecekteki konumuna dogru atis onleme acilarini hesaplar."""
        lead_pan = np.degrees(np.arctan(d_lead_x / max(fx, 1e-6)))
        lead_tilt = np.degrees(np.arctan(-d_lead_y / max(fy, 1e-6)))
        return float(lead_pan), float(lead_tilt)

    def max_step_for(self, dt, axis="pan"):
        """FPS'ten bagimsiz kare basi adim limiti (donanim ust sinirini korur)."""
        dt_safe = self.normalize_dt(dt)
        rate_scale = dt_safe / CONFIG.rl.nominal_dt_s
        if axis == "tilt":
            return float(min(self.max_output_tilt * rate_scale, CONFIG.actuator.max_step_tilt_deg))
        return float(min(self.max_output * rate_scale, CONFIG.control.max_command_step_deg))

    def _apply_in_flight_damping(self, error_deg, deadband, hist_1, hist_2, hist_3=0.0):
        """
        Oneri 2 (3-Kare Smith Predictor): 90-96 ms (3 kare) kamera gecikmesi
        boyunca yolda olan komutlarin toplam etkisini, taret hedefe yaklasirken
        (ayni isaret) gecikmeli kamera hatasindan duser; isaret terslenmesine izin vermez.
        """
        error_deg = float(error_deg)
        if self.in_flight_damping <= 0.0:
            return error_deg
        in_flight = self.in_flight_damping * (
            float(hist_1) + 0.85 * float(hist_2) + 0.65 * float(hist_3)
        )
        if (error_deg * in_flight) > 0.0:
            if abs(error_deg) > abs(in_flight):
                return error_deg - in_flight
            return 0.0
        return error_deg

    def _axis_command(self, error_deg, kp_frame, deadband, accel_deg_s2, previous_speed, dt, max_step, hist_1=0.0, hist_2=0.0, hist_3=0.0):
        pred_error = self._apply_in_flight_damping(error_deg, deadband, hist_1, hist_2, hist_3)
        effective = float(np.sign(pred_error)) * max(abs(pred_error) - deadband, 0.0)
        desired_speed = (kp_frame / CONFIG.rl.nominal_dt_s) * effective
        max_speed = max_step / dt
        desired_speed = float(np.clip(desired_speed, -max_speed, max_speed))

        # Oneri 1 (Asimetrik Frenleme): Ayni yonde yavaslarken veya olu bolge icinde
        # dururken (desired_speed == 0) ivme siniri uygulamadan aninda frenle.
        if (
            self.asymmetric_braking
            and desired_speed * previous_speed >= 0.0
            and abs(desired_speed) < abs(previous_speed)
        ):
            speed = desired_speed
        else:
            speed = float(np.clip(
                desired_speed,
                previous_speed - accel_deg_s2 * dt,
                previous_speed + accel_deg_s2 * dt,
            ))

        step = float(np.clip(speed * dt, -max_step, max_step))
        return step, step / dt

    def compute_p(self, active_pan, active_tilt, dt=CONFIG.rl.nominal_dt_s):
        """Asimetrik hiz-ivme sinirli ve 3-kare Smith Predictor ongorulu Tip-1 P komutu."""
        dt_safe = self.normalize_dt(dt)

        u_pan, self.command_speed_pan_deg_s = self._axis_command(
            active_pan, self.kp_frame_pan, self.deadband_pan,
            self.cmd_accel_pan_deg_s2, self.command_speed_pan_deg_s, dt_safe,
            self.max_step_for(dt_safe, axis="pan"),
            hist_1=self.cmd_hist_pan_1, hist_2=self.cmd_hist_pan_2, hist_3=self.cmd_hist_pan_3,
        )
        u_tilt, self.command_speed_tilt_deg_s = self._axis_command(
            active_tilt, self.kp_frame_tilt, self.deadband_tilt,
            self.cmd_accel_tilt_deg_s2, self.command_speed_tilt_deg_s, dt_safe,
            self.max_step_for(dt_safe, axis="tilt"),
            hist_1=self.cmd_hist_tilt_1, hist_2=self.cmd_hist_tilt_2, hist_3=self.cmd_hist_tilt_3,
        )

        self.cmd_hist_pan_3 = self.cmd_hist_pan_2
        self.cmd_hist_pan_2 = self.cmd_hist_pan_1
        self.cmd_hist_pan_1 = u_pan
        self.cmd_hist_tilt_3 = self.cmd_hist_tilt_2
        self.cmd_hist_tilt_2 = self.cmd_hist_tilt_1
        self.cmd_hist_tilt_1 = u_tilt

        self.smooth_cmd_pan = u_pan
        self.smooth_cmd_tilt = u_tilt
        return u_pan, u_tilt

    def reset(self):
        """Kontrolcunun ic durumlarini sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self._has_target = False
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0
        self.command_speed_pan_deg_s = 0.0
        self.command_speed_tilt_deg_s = 0.0
        self.cmd_hist_pan_1 = 0.0
        self.cmd_hist_pan_2 = 0.0
        self.cmd_hist_pan_3 = 0.0
        self.cmd_hist_tilt_1 = 0.0
        self.cmd_hist_tilt_2 = 0.0
        self.cmd_hist_tilt_3 = 0.0

    def on_target_lost(self):
        """Hedef kaybedildiginde komut ivme rampasi ve yoldaki komut hafizasini sifirlar."""
        self._has_target = False
        self.smooth_cmd_pan = 0.0
        self.smooth_cmd_tilt = 0.0
        self.command_speed_pan_deg_s = 0.0
        self.command_speed_tilt_deg_s = 0.0
        self.cmd_hist_pan_1 = 0.0
        self.cmd_hist_pan_2 = 0.0
        self.cmd_hist_pan_3 = 0.0
        self.cmd_hist_tilt_1 = 0.0
        self.cmd_hist_tilt_2 = 0.0
        self.cmd_hist_tilt_3 = 0.0
