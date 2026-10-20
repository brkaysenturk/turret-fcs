"""
================================================================================
MODUL: AYRIK EKSENLI YUMUSAK OLU BOLGELI TIP-1 P MOTOR KONTROLCUSU
================================================================================
Bu modul, kamera gorus acilarini, balistik yercekimi kompanzasyonunu (super-elevation),
atis onleme acilarini ve gercek donanimda dogrulanmis yumusak olu bolgeli (Soft Deadband)
saf oransal (Tip-1 P) motor kontrol komutlarini hesaplar.

KONTROL PARAMETRELERI:
- Pan:  Kp = 0.055 - 0.10, Deadband = 0.85 - 0.90 deg
- Tilt: Kp = 0.030 - 0.06, Deadband = 0.85 - 0.95 deg (Agir volan namlu sonumlu)
- Filtre: alpha = 0.75 (EMA)
- Komut Limiti: max_output = 1.35 - 2.75 deg/kare

KONTROL TEORISI GEREKCESI:
STM32 gometim yazilimindaki pozisyon biriktiricisi theta(k) = theta(k-1) + delta_theta
ayrik zamanda dogal bir integrator gibi davranir (Tip-1 Servomekanizma).
Bu sayede sistem harici bir integral (Ki) veya turev (Kd / D-termi) gerektirmeden
dogal olarak sifir kalici durum hatasi (zero steady-state error) ile hedefe kilitlenir.
Kamera gurultusunu motora tasimamak ve disli titresimini engellemek icin turev (D)
terimi kullanilmaz; sifir jerk ile kalkis icin Yumusak Olu Bolge (Soft Deadband) uygulanir.
===============================================================================
"""

import numpy as np


class PController:
    """
    Kamera gorus acilarini, balistik telafiyi ve ayrik eksenli yumusak olu bolgeli
    saf oransal (Tip-1 P) motor duzeltme komutlarini hesaplar.
    """

    DEFAULT_DISTANCE_M = 2.0
    PROJECTILE_SPEED_MPS = 10.0  # m/s (Mermi namlu cikis hizi)
    GRAVITY_ACCEL = 9.81         # m/s^2 (Yercekimi ivmesi)

    # --- KALIBRASYON DEGERLERI ---
    MAX_OUTPUT_DEG = 1.35        # Ev: 1.35 | Okul: 2.75
    DEADBAND_PAN_DEG = 0.85      # Ev: 0.85 | Okul: 0.85
    DEADBAND_TILT_DEG = 0.95     # Ev: 0.95 | Okul: 0.95

    def __init__(
        self,
        kp_pan=0.055,            # Ev: 0.055 | Okul: 0.10
        kp_tilt=0.030,           # Ev: 0.030 | Okul: 0.06
        alpha=0.75,
        max_output=MAX_OUTPUT_DEG,
        deadband_pan=DEADBAND_PAN_DEG,
        deadband_tilt=DEADBAND_TILT_DEG,
    ):
        # Oransal kazanclar (P)
        self.kp_pan = float(kp_pan)
        self.kp_tilt = float(kp_tilt)

        # Filtre ve sinirlar
        self.alpha = float(alpha)
        self.max_output = float(max_output)
        self.deadband_pan = float(deadband_pan)
        self.deadband_tilt = float(deadband_tilt)

        # Anlik ve filtrelenmis acilar
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0

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

    def compute_p(self, active_pan, active_tilt, dt=0.033, **kwargs):
        """
        Yumusak olu bolgeli (Soft Deadband) ve ayrik eksenli saf oransal (Tip-1 P)
        motor komutunu hesaplar. Sifir jerk ile puzursuz kalkis saglar.
        """
        self.filtered_pan = self.alpha * active_pan + (1.0 - self.alpha) * self.filtered_pan
        self.filtered_tilt = self.alpha * active_tilt + (1.0 - self.alpha) * self.filtered_tilt

        # =========================================================================
        # 1. PAN EKSENI (Yatay)
        # =========================================================================
        abs_pan = abs(self.filtered_pan)
        if abs_pan <= self.deadband_pan:
            u_pan = 0.0
        else:
            eff_pan = np.sign(self.filtered_pan) * (abs_pan - self.deadband_pan)
            u_pan = self.kp_pan * eff_pan

        # =========================================================================
        # 2. TILT EKSENI (Dikey)
        # =========================================================================
        abs_tilt = abs(self.filtered_tilt)
        if abs_tilt <= self.deadband_tilt:
            u_tilt = 0.0
        else:
            eff_tilt = np.sign(self.filtered_tilt) * (abs_tilt - self.deadband_tilt)
            u_tilt = self.kp_tilt * eff_tilt

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
