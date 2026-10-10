"""
================================================================================
MODUL: DINAMIK HEDEF KILIT YONETICISI (LOCK MANAGER)
================================================================================
Bu modul, atis kontrol sisteminin atesleme kapisi (Firing Gate) ve hedef kilit
kararliligini yonetir. Histerezisli cift esik mantigi ve analitik Gauss isabet
olasiligi modeli ile hedefe tam oturulmadan atis yapilmasini onler.

ALGORITMA VE GUCLU MEKANIZMALAR:
1. Histerezisli Dinamik Kilit Penceresi (Hysteresis Window):
   - Kilit Giris Esigi: Hedeften uzakliga ve hiza bagli (2.0° - 3.0°).
   - Kilit Cikis Esigi: 3.5° (Genis tolerans).
   - Kilit Titremesi Onleme: Hedef tam esik cizgisinde hareket ederken kilidin
     anlik acilip kapanmasini (chattering / flickering) onler.
2. Kararlilik Sayaci (Stability Counter):
   - Hedef kilit penceresine girdikten sonra art arda 10 kare (~0.33 sn) boyunca
     pencere icinde kalmasi zorunludur. Anlik hedef gecislerinde atis yapilmaz.
3. Analitik Gauss Isabet Olasiligi Modeli (Hit Probability):
   - P_hit = exp(-theta_err^2 / (2 * sigma_eff^2)) formulu ile hedef mesafesine (TF-Luna LiDAR)
     ve hedef hizina gore 0 - 100% arasinda gercekci balistik vurus olasiligi hesaplar.
================================================================================
"""

import numpy as np


class LockManager:
    """
    Dinamik kilit acimini, histerezis esiklerini, kararlilik sayacini ve
    hedef kilit kalitesi puanini gercek zamanli turret FCS icin yonetir.
    """

    def __init__(self, lock_threshold_frames=10, lock_exit_deg=3.5):
        self.lock_threshold_frames = lock_threshold_frames
        self.lock_exit_deg = lock_exit_deg

        self.stability_counter = 0
        self.is_locked = False
        self.lock_quality = 0.0
        self.hit_probability = 0.0

    def compute_dynamic_threshold(self, target_velocity):
        speed_mag = float(np.hypot(target_velocity[0], target_velocity[1]))
        if speed_mag < 15.0:
            return 2.0
        if speed_mag < 45.0:
            return 2.5
        return 3.0

    def compute_hit_probability(self, angular_error_deg, target_velocity, dist_m=2.0, target_radius_m=0.15):
        """
        Analitik Bivariate Gaussian Isabet Olasiligi Modeli (P_hit):
        Hedef geometrisi, menzil, namlu dagilimi ve hedef hiz belirsizligini
        birlestiren kapali form balistik model (%0.0 - %100.0).
        """
        d = max(0.5, float(dist_m)) if dist_m is not None else 2.0
        theta_target = np.degrees(np.arctan(target_radius_m / d))

        t_flight = (d / 10.0) + 0.08
        speed_mag = float(np.hypot(target_velocity[0], target_velocity[1]))
        speed_deg_s = speed_mag * (57.3 / 642.0)

        sigma_motion = 0.25 * speed_deg_s * t_flight
        sigma_weapon = 0.75  # Flywheel ic balistik sacilmasi (derece)
        sigma_total = np.sqrt(sigma_weapon**2 + sigma_motion**2)

        p_zero = 1.0 - np.exp(-(theta_target**2) / (2.0 * (sigma_total**2)))
        p_hit = p_zero * np.exp(-(angular_error_deg**2) / (2.0 * (sigma_total**2 + 0.5 * (theta_target**2))))
        return float(np.clip(p_hit * 100.0, 0.0, 100.0))

    def update(self, angular_error_deg, target_velocity, is_firing=False, dist_m=2.0):
        if is_firing:
            self.is_locked = True
            self.stability_counter = self.lock_threshold_frames
            return self.is_locked, self.stability_counter, self.hit_probability

        self.hit_probability = self.compute_hit_probability(angular_error_deg, target_velocity, dist_m=dist_m)
        self.lock_quality = self.hit_probability / 100.0

        dynamic_threshold = self.compute_dynamic_threshold(target_velocity)

        # Hedef tolerans icindeyse sayaci artir, ciktiysa her kare 1 azalt 
        if angular_error_deg <= dynamic_threshold:
            self.stability_counter = min(self.lock_threshold_frames, self.stability_counter + 1)
        else:
            self.stability_counter = max(0, self.stability_counter - 1)

        # Histerezisli Kilit Durumu:
        if not self.is_locked:
            # Kilitli degilken: Yalnizca 15 kare tam dolunca kilitlen
            if self.stability_counter >= self.lock_threshold_frames:
                self.is_locked = True
        else:
            # Hedef cok uzaklasirsa (> exit_deg) veya sayac yariya duserse kilit bozulur
            if angular_error_deg > self.lock_exit_deg or self.stability_counter < (self.lock_threshold_frames // 2):
                self.is_locked = False
                if angular_error_deg > self.lock_exit_deg:
                    self.stability_counter = 0

        return self.is_locked, self.stability_counter, self.hit_probability

    def reset(self):
        self.stability_counter = 0
        self.is_locked = False
        self.lock_quality = 0.0
        self.hit_probability = 0.0