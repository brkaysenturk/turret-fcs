import numpy as np


class PIDController:
    """
    Kamera gorus acilarini, balistik yercekimi kompanzasyonunu (super-elevation),
    onleme acilarini ve sizintili integral (leaky PID) motor kontrol komutlarini hesaplar.
    """

    DEFAULT_DISTANCE_M = 2.0
    PROJECTILE_SPEED_MPS = 10.0  # m/s
    GRAVITY_ACCEL = 9.81         # m/s^2
    MAX_OUTPUT_DEG = 10.0        # Maksimum motor duzeltme derecesi / komut limiti

    def __init__(self, kp=0.42, ki=0.18, kd=0.038, alpha=0.85, max_output=MAX_OUTPUT_DEG):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        self.max_output = max_output

        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0

        self.integral_pan = 0.0
        self.integral_tilt = 0.0
        self.last_pan_error = 0.0
        self.last_tilt_error = 0.0
        self.first_detection = True

    def compute_angles(self, bx, by, center_x, center_y, fx, fy):
        """Kamera merkezinden hedefin acisal sapmasini hesaplar ve filtreler."""
        target_err_x = bx - center_x
        target_err_y = by - center_y
        raw_pan = np.degrees(np.arctan(target_err_x / max(fx, 1e-6)))
        raw_tilt = np.degrees(np.arctan(-target_err_y / max(fy, 1e-6)))

        self.actual_pan = self.alpha * raw_pan + (1.0 - self.alpha) * self.actual_pan
        self.actual_tilt = self.alpha * raw_tilt + (1.0 - self.alpha) * self.actual_tilt
        return self.actual_pan, self.actual_tilt

    def compute_gravity_comp(self, dist_cm):
        """Analitik yercekimi dususu kompanzasyonu: theta = arctan(g * D / (2 * v^2))."""
        target_d_m = (dist_cm / 100.0) if (dist_cm is not None and dist_cm > 20) else self.DEFAULT_DISTANCE_M
        v_sq_2 = 2.0 * (self.PROJECTILE_SPEED_MPS ** 2)
        return float(np.degrees(np.arctan((self.GRAVITY_ACCEL * target_d_m) / v_sq_2)))

    def compute_lead_angles(self, d_lead_x, d_lead_y, fx, fy):
        """Hedefin gelecekteki konumuna dogru atis onleme acilarini hesaplar."""
        lead_pan = np.degrees(np.arctan(d_lead_x / max(fx, 1e-6)))
        lead_tilt = np.degrees(np.arctan(-d_lead_y / max(fy, 1e-6)))
        return float(lead_pan), float(lead_tilt)

    def compute_pid(self, active_pan, active_tilt, dt):
        """Sizintili integral (leaky integrator) destekli PID motor duzeltme komutu uretir."""
        self.filtered_pan = self.alpha * active_pan + (1.0 - self.alpha) * self.filtered_pan
        self.filtered_tilt = self.alpha * active_tilt + (1.0 - self.alpha) * self.filtered_tilt

        if self.first_detection:
            self.last_pan_error = self.filtered_pan
            self.last_tilt_error = self.filtered_tilt
            self.integral_pan = 0.0
            self.integral_tilt = 0.0
            self.first_detection = False

        d_pan = (self.filtered_pan - self.last_pan_error) / dt
        d_tilt = (self.filtered_tilt - self.last_tilt_error) / dt
        self.last_pan_error = self.filtered_pan
        self.last_tilt_error = self.filtered_tilt

        # Leaky integrator (%8 sizinti kalici durum hatasini ve windup'i onler)
        self.integral_pan = float(np.clip((0.92 * self.integral_pan) + (self.filtered_pan * dt), -1.0, 1.0))
        self.integral_tilt = float(np.clip((0.92 * self.integral_tilt) + (self.filtered_tilt * dt), -1.0, 1.0))

        u_pan = (self.kp * self.filtered_pan) + (self.ki * self.integral_pan) + (self.kd * d_pan)
        u_tilt = (self.kp * self.filtered_tilt) + (self.ki * self.integral_tilt) + (self.kd * d_tilt)

        u_pan = float(np.clip(u_pan, -self.max_output, self.max_output))
        u_tilt = float(np.clip(u_tilt, -self.max_output, self.max_output))

        return u_pan, u_tilt

    def reset(self):
        """Kontrolcu durumunu sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0
        self.integral_pan = 0.0
        self.integral_tilt = 0.0
        self.last_pan_error = 0.0
        self.last_tilt_error = 0.0
        self.first_detection = True

