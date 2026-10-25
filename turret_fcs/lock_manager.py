"""
================================================================================
MODUL: DINAMIK HEDEF KILIT YONETICISI (LOCK MANAGER)
================================================================================
Bu modul, atis kontrol sisteminin atesleme kapisi (Firing Gate) ve hedef kilit
kararliligini yonetir. Histerezisli cift esik mantigi ve analitik Gauss isabet
olasiligi modeli ile hedefe tam oturulmadan atis yapilmasini onler.

ALGORITMA VE GUCLU MEKANIZMALAR:
1. Histerezisli Dinamik Kilit Penceresi (Hysteresis Window):
   - Kilit Giris Esigi: Hedeften uzakliga ve hiza bagli dinamik esik (1.60° - 3.00°).
   - Kilit Cikis Esigi: Dinamik esik + 0.60° (2.20° - 3.60°).
   - Kilit Titremesi Onleme: Hedef tam esik cizgisinde hareket ederken kilidin
     anlik acilip kapanmasini (chattering / flickering) onler.
2. Kararlilik Sayaci (Stability Counter):
   - Hedef kilit penceresine girdikten sonra art arda 10 kare (~0.33 sn) boyunca
     pencere icinde kalmasi zorunludur. Anlik hedef gecislerinde atis yapilmaz.
3. Analitik Gauss Isabet Olasiligi Modeli (Hit Probability):
   - Tam 2B Dairesel Gauss Integrali (Merkezi Olmayan Ki-Kare CDF / Marcum Q) ile
     hedef geometrisi, ucus suresi, gecikme ve sacilmaya gore %0.0 - %100.0 arasi hesaplar.
================================================================================
"""

import numpy as np
from scipy.stats import ncx2


from .config import CONFIG


class LockManager:
    """
    Dinamik kilit acimini, histerezis esiklerini, kararlilik sayacini ve
    hedef kilit kalitesi puanini gercek zamanli turret FCS icin yonetir.
    """

    def __init__(
        self,
        lock_threshold_frames=CONFIG.lock.lock_threshold_frames,
        lock_exit_margin_deg=CONFIG.lock.lock_exit_margin_deg,
        fx=None,
    ):
        self.lock_threshold_frames = lock_threshold_frames
        self.lock_exit_margin_deg = lock_exit_margin_deg
        self.fx = float(fx) if fx is not None else CONFIG.optics.compute_fx(CONFIG.optics.default_width)
        self.fy = CONFIG.optics.compute_fy(CONFIG.optics.default_height)

        self.stability_counter = 0
        self.is_locked = False
        self.lock_quality = 0.0
        self.hit_probability = 0.0
        self._prev_is_firing = False
        self._post_fire_grace_frames = 0
        self.post_fire_grace_duration_frames = CONFIG.lock.post_fire_grace_duration_frames  # 0.75 saniye @ 30 FPS kilit muafiyeti (grace period)

    def compute_dynamic_threshold(self, target_velocity):
        speed_mag = float(np.hypot(target_velocity[0], target_velocity[1]))
        if speed_mag < CONFIG.lock.threshold_low_speed_px_s:
            return CONFIG.lock.gate_slow_deg
        if speed_mag < CONFIG.lock.threshold_high_speed_px_s:
            return CONFIG.lock.gate_medium_deg
        return CONFIG.lock.gate_fast_deg

    def compute_hit_probability(
        self,
        angular_error_deg,
        target_velocity,
        dist_m=CONFIG.ballistics.default_distance_m,
        target_radius_m=CONFIG.lock.target_radius_m,
        fx=None,
        fy=None,
        box_dim=None,
        stability_progress=1.0,
        dynamic_threshold=None,
    ):
        """
        Tam 2B Dairesel Gauss Integrali (Non-Central Chi-Squared CDF / Marcum Q) + Takip Kararliligi:
        Hedefin olumcul etkin cekirdegi (lethal core), ucus suresi (t_flight), sistem gecikmesi,
        kalibre edilmis namlu sacilmasi ve taretin anlik kilitlenme oturma seviyesini
        birlestiren operasyonel atis kontrol isabet olasiligi (%0.0 - %100.0).
        """
        d = max(0.5, float(dist_m)) if dist_m is not None else CONFIG.ballistics.default_distance_m
        effective_fx = float(fx) if (fx is not None and fx > 100.0) else self.fx
        effective_fy = float(fy) if (fy is not None and fy > 100.0) else self.fy

        # 1. Hedef Acisal Yaricapi: Etkin olumcul cekirdek (kutu alaninin %60'i,
        # cunku balonun/hedefin en dis kenarina temas eden dartlar seker veya patlatmaz).
        # Fiziksel hedef yaricapi (target_radius_m) ile ustten sinirlanir.
        theta_max_geom = float(np.degrees(np.arctan(target_radius_m / d)))
        if box_dim is not None and len(box_dim) >= 2 and box_dim[0] > 0 and box_dim[1] > 0:
            bw, bh = float(box_dim[0]), float(box_dim[1])
            theta_pan = float(np.degrees(np.arctan((bw * 0.60 / 2.0) / effective_fx)))
            theta_tilt = float(np.degrees(np.arctan((bh * 0.60 / 2.0) / effective_fy)))
            theta_target = float(min(theta_max_geom, 0.5 * (theta_pan + theta_tilt)))
        else:
            theta_target = theta_max_geom

        theta_target = max(0.10, theta_target)

        # 2. Ucus Suresi ve Sistem Gecikmesi Ayrilimi
        v_proj = CONFIG.ballistics.projectile_speed_mps
        t_flight = d / max(1.0, v_proj)
        system_latency_s = CONFIG.ballistics.system_latency_s
        total_lead_time = t_flight + system_latency_s

        # 3. Anizotropik Acisal Hedef Hizi (pan ve tilt ayrik odak uzakliklariyla)
        vx_deg_s = float(np.degrees(target_velocity[0] / effective_fx))
        vy_deg_s = float(np.degrees(target_velocity[1] / effective_fy))
        speed_deg_s = float(np.hypot(vx_deg_s, vy_deg_s))

        # 4. Saçılma Bileşenleri
        sigma_motion = 0.35 * speed_deg_s * total_lead_time
        sigma_weapon = CONFIG.lock.sigma_weapon_deg
        sigma_total = float(np.sqrt(sigma_weapon**2 + sigma_motion**2))
        sigma_safe = max(0.01, sigma_total)

        # 5. Tam 2B Gauss Integrali: Merkezi Olmayan Ki-Kare (ncx2) CDF
        err_mag = max(0.0, float(angular_error_deg))
        nc_param = float((err_mag / sigma_safe) ** 2)
        x_cutoff = float((theta_target / sigma_safe) ** 2)

        p_ballistic = float(ncx2.cdf(x_cutoff, df=2, nc=nc_param))

        # 6. Takip ve Kilit Kararliligi Modulasyonu (FCS Operasyonel Doktrini):
        # Kilit henuz oturmamissa (stability_counter < 10) veya taret hedefe yakinsiyorsa,
        # anlik hata kucuk olsa bile isabet olasiligi sahte sekilde %99 gosterilmez.
        s_track = float(np.clip(stability_progress, 0.15, 1.0))

        # Dinamik kapi disina tasildiginda isabet olasiligi hizla sonumlenir
        s_gate = 1.0
        if dynamic_threshold is not None and dynamic_threshold > 0.0:
            if err_mag > dynamic_threshold:
                gate_ratio = (err_mag - dynamic_threshold) / max(0.5, dynamic_threshold * 0.5)
                s_gate = float(np.exp(-2.0 * (gate_ratio ** 2)))

        p_hit_ratio = p_ballistic * s_track * s_gate
        return float(np.clip(p_hit_ratio * 100.0, 0.0, 100.0))

    def update(self, angular_error_deg, target_velocity, is_firing=False, dist_m=2.0, fx=None, fy=None, box_dim=None):
        if is_firing:
            self._prev_is_firing = True
            # Atis ve geri toparlanma (recover) dongusu boyunca mevcut kilit durumunu dondurarak koru
            return self.is_locked, self.stability_counter, self.hit_probability

        # Atis sekansi henuz bittiyse (is_firing True -> False gecisi), 1 saniyelik grace period baslat
        if self._prev_is_firing:
            self._prev_is_firing = False
            self._post_fire_grace_frames = self.post_fire_grace_duration_frames

        in_grace_period = self._post_fire_grace_frames > 0
        if in_grace_period:
            self._post_fire_grace_frames -= 1

        dynamic_threshold = self.compute_dynamic_threshold(target_velocity)
        dynamic_exit_threshold = dynamic_threshold + self.lock_exit_margin_deg

        # Hedef tolerans icindeyse sayaci artir, ciktiysa her kare 1 azalt 
        if angular_error_deg <= dynamic_threshold:
            self.stability_counter = min(self.lock_threshold_frames, self.stability_counter + 1)
        else:
            self.stability_counter = max(0, self.stability_counter - 1)

        # Histerezisli Kilit Durumu:
        if not self.is_locked:
            # Kilitli degilken: Yalnizca esik karesi dolunca kilitlen
            if self.stability_counter >= self.lock_threshold_frames:
                self.is_locked = True
        else:
            # Atis sonrasi taret merkeze donerken (grace period) kilidi erken ve sahte sekilde dusurme
            if in_grace_period:
                self.stability_counter = max(self.lock_threshold_frames // 2, self.stability_counter)
            else:
                # Hedef histerezis esiginden uzaklasirsa (> dynamic_exit_threshold) veya sayac yariya duserse kilit bozulur
                if angular_error_deg > dynamic_exit_threshold or self.stability_counter < (self.lock_threshold_frames // 2):
                    self.is_locked = False
                    if angular_error_deg > dynamic_exit_threshold:
                        self.stability_counter = 0

        # Kilit oturmus ise s_track = 1.0, kilitlenme asamasindaysa ilerleme orani
        stability_progress = 1.0 if self.is_locked else (
            0.15 + 0.85 * ((self.stability_counter / max(1, self.lock_threshold_frames)) ** 1.5)
        )

        self.hit_probability = self.compute_hit_probability(
            angular_error_deg, target_velocity, dist_m=dist_m, fx=fx, fy=fy, box_dim=box_dim,
            stability_progress=stability_progress, dynamic_threshold=dynamic_threshold
        )
        self.lock_quality = self.hit_probability / 100.0

        return self.is_locked, self.stability_counter, self.hit_probability

    def reset(self):
        self.stability_counter = 0
        self.is_locked = False
        self.lock_quality = 0.0
        self.hit_probability = 0.0
        self._prev_is_firing = False
        self._post_fire_grace_frames = 0