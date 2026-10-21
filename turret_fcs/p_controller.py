"""
================================================================================
MODUL: AYRIK EKSENLI YUMUSAK OLU BOLGELI TIP-1 P MOTOR KONTROLCUSU
================================================================================
Bu modul, kamera gorus acilarini, balistik yercekimi kompanzasyonunu (super-elevation),
atis onleme acilarini ve gercek donanimda dogrulanmis yumusak olu bolgeli (Soft Deadband)
saf oransal (Tip-1 P) motor kontrol komutlarini hesaplar.

KONTROL PARAMETRELERI:
- Pan:  Kp = 0.10 (30 FPS bazinda, dt normalizasyonu ile Kp_s = 3.0 1/s), Deadband = 0.85 deg
- Tilt: Kp = 0.05 (30 FPS bazinda, dt normalizasyonu ile Kp_s = 1.5 1/s), Deadband = 0.95 deg
- Filtre: alpha = 0.75 (EMA)
- Komut Limiti: max_output = 1.35 deg (Tek kare mekanik jerk ve adim tavani ile korunur)

KONTROL TEORISI GEREKCESI:
STM32 gomulu yazilimindaki pozisyon biriktiricisi theta(k) = theta(k-1) + delta_theta
ayrik zamanda dogal bir integrator gibi davranir (Tip-1 Servomekanizma).
Klasik kontrol teorisine gore (Ogata / Franklin):
1. Statik Hedefler (Konum Basamak Girisi): Tip-1 acik cevrim integratoru sayesinde
   harici bir yazilimsal integral (Ki) terimine ihtiyac duymadan sifir kalici durum hatasi (e_ss = 0) saglar.
2. Hareketli Hedefler (Acisal Hiz Rampa Girisi): Tip-1 sistemlerde rampa hatasi e_ss = omega / Kv olur.
   Sistemimiz bu sonlu rampa gecikmesini, Kalman filtresi hedef hiz kestirimi ve
   balistik atis onleme ofseti (kinematik lead offset) ile telafi eder;
   cevik manevralarda ise yapay zeka (RL) denetleyicisi devreye girer.
Yazilima harici bir Ki integrali konulmamasinin temel sebebi, acik cevrime ikinci bir kutup
ekleyerek (Tip-2 rezonansi) faz cokmesine ve servo limit dongusu titresimine yol acmasini engellemektir.
Kamera gurultusunu servolara tasimamak icin turev (D) terimi yerine Yumusak Olu Bolge (Soft Deadband) kullanilir.
===============================================================================
"""

import numpy as np

try:
    from .config import CONFIG
except (ImportError, ValueError):
    from config import CONFIG


class PController:
    """
    Kamera gorus acilarini, balistik telafiyi ve ayrik eksenli yumusak olu bolgeli
    saf oransal (Tip-1 P) motor duzeltme komutlarini hesaplar.
    """

    DEFAULT_DISTANCE_M = CONFIG.ballistics.default_distance_m
    PROJECTILE_SPEED_MPS = CONFIG.ballistics.projectile_speed_mps
    GRAVITY_ACCEL = CONFIG.ballistics.gravity_accel

    MAX_OUTPUT_DEG = CONFIG.control.max_output_deg
    DEADBAND_PAN_DEG = CONFIG.control.deadband_pan_deg
    DEADBAND_TILT_DEG = CONFIG.control.deadband_tilt_deg

    def __init__(
        self,
        kp_pan=CONFIG.control.kp_pan,
        kp_tilt=CONFIG.control.kp_tilt,
        alpha=CONFIG.control.alpha_filter,
        max_output=MAX_OUTPUT_DEG,
        deadband_pan=DEADBAND_PAN_DEG,
        deadband_tilt=DEADBAND_TILT_DEG,
    ):
        # 30 FPS bazinda girilen kazanci surekli zaman formuna (1/s) donusturur
        self.kp_frame_pan = float(kp_pan)
        self.kp_frame_tilt = float(kp_tilt)

        self.kp_pan = self.kp_frame_pan if self.kp_frame_pan >= 1.0 else (self.kp_frame_pan / 0.03333)
        self.kp_tilt = self.kp_frame_tilt if self.kp_frame_tilt >= 1.0 else (self.kp_frame_tilt / 0.03333)

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
        """Yumusak olu bolgeli ve dt-normalizasyonlu Tip-1 P motor komutunu hesaplar."""
        # Dongu gecikmelerinde maksimum dt sinirlamasi (50 ms)
        effective_dt = max(0.010, min(float(dt), 0.050))

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
            u_pan = self.kp_pan * eff_pan * effective_dt

        # =========================================================================
        # 2. TILT EKSENI (Dikey)
        # =========================================================================
        abs_tilt = abs(self.filtered_tilt)
        if abs_tilt <= self.deadband_tilt:
            u_tilt = 0.0
        else:
            eff_tilt = np.sign(self.filtered_tilt) * (abs_tilt - self.deadband_tilt)
            u_tilt = self.kp_tilt * eff_tilt * effective_dt

        # Tek karede verilebilecek maksimum mekanik adim siniri
        max_step = float(min(self.max_output, self.max_output * (effective_dt / 0.03333)))
        u_pan = float(np.clip(u_pan, -max_step, max_step))
        u_tilt = float(np.clip(u_tilt, -max_step, max_step))

        return u_pan, u_tilt

    def reset(self):
        """Kontrolcunun ic durumlarini sifirlar."""
        self.actual_pan = 0.0
        self.actual_tilt = 0.0
        self.filtered_pan = 0.0
        self.filtered_tilt = 0.0
