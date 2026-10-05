import numpy as np


class WebotsLockManager:
    """Taretin hedefe kilitlenme durumunu ve kilit kalitesi skorunu yonetir."""

    def __init__(self, enter_deg=2.0, exit_deg=3.2, threshold_frames=15):
        self.LOCK_ENTER_DEG = enter_deg
        self.LOCK_EXIT_DEG = exit_deg
        self.LOCK_THRESHOLD_FRAMES = threshold_frames

        self.lock_counter = 0
        self.stability_counter = 0
        self.is_locked = False
        self.last_lock_quality = 0.0
        self.hit_probability = 0.0

    def compute_dynamic_threshold(self, target_velocity):
        """Hedef hizina bagli olarak kilitlenme tolerans esigini dinamik genisletir."""
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
        sigma_weapon = 0.75  # Webots dart balistik sacilmasi (derece)
        sigma_total = np.sqrt(sigma_weapon**2 + sigma_motion**2)

        p_zero = 1.0 - np.exp(-(theta_target**2) / (2.0 * (sigma_total**2)))
        p_hit = p_zero * np.exp(-(angular_error_deg**2) / (2.0 * (sigma_total**2 + 0.5 * (theta_target**2))))
        return float(np.clip(p_hit * 100.0, 0.0, 100.0))

    def update(self, actual_pan, actual_tilt, target_velocity, is_firing=False, dist_m=2.0):
        """Kilitlenme durumunu ve kilit kalitesini histerezisli olarak gunceller."""
        if is_firing:
            self.is_locked = True
            self.lock_counter = self.LOCK_THRESHOLD_FRAMES
            self.stability_counter = self.LOCK_THRESHOLD_FRAMES
            return self.is_locked, self.hit_probability

        target_angular_error = float(np.hypot(actual_pan, actual_tilt))
        self.hit_probability = self.compute_hit_probability(target_angular_error, target_velocity, dist_m=dist_m)
        self.last_lock_quality = self.hit_probability / 100.0

        dynamic_threshold = self.compute_dynamic_threshold(target_velocity)

        # Hedef tolerans icindeyse sayaci artir, ciktiysa her kare 1 azalt (leaky integrator)
        if target_angular_error <= dynamic_threshold:
            self.stability_counter = min(self.LOCK_THRESHOLD_FRAMES, self.stability_counter + 1)
        else:
            self.stability_counter = max(0, self.stability_counter - 1)

        self.lock_counter = self.stability_counter

        # Histerezisli Kilit Yonetimi:
        if not self.is_locked:
            # Henuz kilitli degilse: Yalnizca 15 kare tam doldugunda kilitlen
            if self.stability_counter >= self.LOCK_THRESHOLD_FRAMES:
                self.is_locked = True
        else:
            # Zaten kilitliyse: 1 kare cikti diye kilit ANINDA bozulmaz!
            # Kilit ancak hedef cok uzaklasirsa (> LOCK_EXIT_DEG) veya sayac yariya duserse (< 8) bozulur
            if target_angular_error > self.LOCK_EXIT_DEG or self.stability_counter < (self.LOCK_THRESHOLD_FRAMES // 2):
                self.is_locked = False
                if target_angular_error > self.LOCK_EXIT_DEG:
                    self.stability_counter = 0
                    self.lock_counter = 0

        return self.is_locked, self.hit_probability

    def reset(self):
        """Kilit durumunu ve sayaclari sifirlar."""
        self.is_locked = False
        self.lock_counter = 0
        self.stability_counter = 0
        self.last_lock_quality = 0.0
        self.hit_probability = 0.0