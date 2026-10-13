"""
================================================================================
MODUL: ORANSAL (P / PID) MOTOR KONTROLCUSU (P CONTROLLER)
================================================================================
Bu modul, kamera gorus acilarini, balistik yercekimi kompanzasyonunu (super-elevation),
onleme acilarini ve gercek donanimda dogrulanmis yumusak olu bolgeli (Soft Deadband)
motor kontrol komutlarini hesaplar.

KONTROL PARAMETRELERI:
- Pan:  Kp = 0.10, Ki = 0.0, Kd = 0.0, Deadband = 0.90°
- Tilt: Kp = 0.07, Ki = 0.0, Kd = 0.0, Deadband = 0.95°
- Filtre: alpha = 0.75 (EMA)
- Komut Limiti: max_output = 2.75°/kare

NOT:
Ki ve Kd parametreleri hazir ve matematiksel olarak entegre edilmistir.
0.0 olarak baslatilir (servo ic donanimi ve STM32 Tip-1 dogal entegrasyonu icin).
Istenildigi anda parametre olarak verilerek PID modunda calistirilabilir.
================================================================================
"""

import numpy as np


class PController:
    """
    Kamera gorus acilarini, balistik telafiyi ve ayrik eksenli yumusak olu bolgeli
    oransal (ve opsiyonel I/D destekli) motor duzeltme komutlarini hesaplar.
    """

    DEFAULT_DISTANCE_M = 2.0
    PROJECTILE_SPEED_MPS = 10.0  # m/s (Mermi namlu cikis hizi)
    GRAVITY_ACCEL = 9.81         # m/s^2 (Yercekimi ivmesi)
    MAX_OUTPUT_DEG = 2.75        # Kare basi maksimum motor adim limiti (derece)
    DEADBAND_PAN_DEG = 0.90      # Yatay yumusak olu bolge (Pan)
    DEADBAND_TILT_DEG = 0.95     # Dikey yumusak olu bolge (Tilt)

    def __init__(
        self,
        kp_pan=0.10,
        kp_tilt=0.07,
        ki_pan=0.0,
        ki_tilt=0.0,
        kd_pan=0.0,
        kd_tilt=0.0,
        alpha=0.75,
        max_output=MAX_OUTPUT_DEG,
        deadband_pan=DEADBAND_PAN_DEG,
        deadband_tilt=DEADBAND_TILT_DEG,
        max_integral=1.5,
    ):
        # Oransal kazanclar (P)
        self.kp_pan = float(kp_pan)
        self.kp_tilt = float(kp_tilt)

        # Integral kazanclari (I)
        self.ki_pan = float(ki_pan)
        self.ki_tilt = float(ki_tilt)

        # Turev kazanclari (D)
        self.kd_pan = float(kd_pan)
        self.kd_tilt = float(kd_tilt)

        # Filtre ve sinirlar
        self.alpha = float(alpha)
        self.max_output = float(max_output)
        self.deadband_pan = float(deadband_pan)
        self.deadband_tilt = float(deadband_tilt)
        self.max_integral = float(max_integral)

        # Anlik ve filtrelenmis acilar
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0

        # Integral birikimleri ve gecmis hatalar (I ve D icin)
        self.integral_pan = 0.0
        self.integral_tilt = 0.0
        self.last_pan_error = 0.0
        self.last_tilt_error = 0.0
        self.first_run = True

    def compute_angles(self, bx, by, center_x, center_y, fx, fy):
        """Kamera merkezinden hedefin acisal sapmasini hesaplar ve EMA ile filtreler."""
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

    def compute_p(self, active_pan, active_tilt, dt=0.033, target_speed=0.0):
        """
        Yumusak olu bolgeli ve ayrik eksenli oransal (P) motor komutunu hesaplar.
        Eger ki veya kd 0'dan farkli ayarlanmissa, integral ve turev terimlerini de ekler.
        """
        dt = 0.033 if (dt is None or dt <= 0.0) else float(dt)

        self.filtered_pan = self.alpha * active_pan + (1.0 - self.alpha) * self.filtered_pan
        self.filtered_tilt = self.alpha * active_tilt + (1.0 - self.alpha) * self.filtered_tilt

        # =========================================================================
        # 1. PAN EKSENI (Yatay)
        # =========================================================================
        abs_pan = abs(self.filtered_pan)
        if abs_pan <= self.deadband_pan:
            eff_pan = 0.0
            # Olu bolge icindeyken integral szintisi (leaky integrator)
            self.integral_pan *= 0.90
        else:
            eff_pan = np.sign(self.filtered_pan) * (abs_pan - self.deadband_pan)

        # Oransal (P) terimi
        p_term_pan = self.kp_pan * eff_pan

        # Integral (I) terimi (Anti-windup sinirlamali)
        if self.ki_pan > 0.0:
            self.integral_pan += eff_pan * dt
            self.integral_pan = float(np.clip(self.integral_pan, -self.max_integral, self.max_integral))
            i_term_pan = self.ki_pan * self.integral_pan
        else:
            self.integral_pan = 0.0
            i_term_pan = 0.0

        # Turev (D) terimi
        if self.kd_pan > 0.0 and not self.first_run:
            d_err_pan = (eff_pan - self.last_pan_error) / dt
            d_term_pan = self.kd_pan * d_err_pan
        else:
            d_term_pan = 0.0

        self.last_pan_error = eff_pan
        u_pan = p_term_pan + i_term_pan + d_term_pan

        # =========================================================================
        # 2. TILT EKSENI (Dikey)
        # =========================================================================
        abs_tilt = abs(self.filtered_tilt)
        if abs_tilt <= self.deadband_tilt:
            eff_tilt = 0.0
            self.integral_tilt *= 0.90
        else:
            eff_tilt = np.sign(self.filtered_tilt) * (abs_tilt - self.deadband_tilt)

        # Oransal (P) terimi
        p_term_tilt = self.kp_tilt * eff_tilt

        # Integral (I) terimi (Anti-windup sinirlamali)
        if self.ki_tilt > 0.0:
            self.integral_tilt += eff_tilt * dt
            self.integral_tilt = float(np.clip(self.integral_tilt, -self.max_integral, self.max_integral))
            i_term_tilt = self.ki_tilt * self.integral_tilt
        else:
            self.integral_tilt = 0.0
            i_term_tilt = 0.0

        # Turev (D) terimi
        if self.kd_tilt > 0.0 and not self.first_run:
            d_err_tilt = (eff_tilt - self.last_tilt_error) / dt
            d_term_tilt = self.kd_tilt * d_err_tilt
        else:
            d_term_tilt = 0.0

        self.last_tilt_error = eff_tilt
        u_tilt = p_term_tilt + i_term_tilt + d_term_tilt

        self.first_run = False

        # Eyleyici mekanik sinirlamasi (Maksimum motor komutu)
        u_pan = float(np.clip(u_pan, -self.max_output, self.max_output))
        u_tilt = float(np.clip(u_tilt, -self.max_output, self.max_output))

        return u_pan, u_tilt

    def reset(self):
        """Kontrolcunun ic durumlarini sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0
        self.integral_pan = 0.0
        self.integral_tilt = 0.0
        self.last_pan_error = 0.0
        self.last_tilt_error = 0.0
        self.first_run = True
