import numpy as np
from webots_target_selector import WebotsTargetSelector
from webots_kalman_filter import WebotsKalmanFilter
from webots_lock_manager import WebotsLockManager
from webots_pid_controller import WebotsPIDController
from webots_patrol_generator import WebotsPatrolGenerator
from webots_rl_controller import WebotsRLController


class WebotsFCSManager:
    """Moduler ve yuksek hassasiyetli Ates Kontrol Sistemi (FCS Pipeline Yoneticisi)."""

    def __init__(self, kp=0.42, alpha=0.75, min_conf=0.30, control_mode="HYBRID"):
        self.selector = WebotsTargetSelector(min_conf=min_conf)
        self.tracker = WebotsKalmanFilter(dt=0.016)
        self.lock_mgr = WebotsLockManager(enter_deg=2.0, exit_deg=3.2, threshold_frames=15)
        self.aimer = WebotsPIDController(kp=kp, alpha=alpha)
        self.patroller = WebotsPatrolGenerator()
        self.rl_controller = WebotsRLController()
        self.control_mode = control_mode  # "HYBRID", "PID", "RL"

        self.pan_target = 0.0
        self.tilt_target = 0.0
        self.frames_since_lost = 0
        self.MAX_LOST_FRAMES = 5

        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.filtered_dist_m = 2.10

    @property
    def is_locked(self):
        return self.lock_mgr.is_locked

    @property
    def hit_probability(self):
        return self.lock_mgr.hit_probability

    @property
    def last_lock_quality(self):
        return self.lock_mgr.hit_probability

    @property
    def lock_quality(self):
        return self.lock_mgr.hit_probability

    @property
    def lock_counter(self):
        return self.lock_mgr.lock_counter

    @property
    def LOCK_THRESHOLD_FRAMES(self):
        return self.lock_mgr.LOCK_THRESHOLD_FRAMES

    @property
    def actual_pan(self):
        return self.aimer.actual_pan

    @property
    def actual_tilt(self):
        return self.aimer.actual_tilt

    @property
    def gravity_comp_deg(self):
        return self.aimer.gravity_comp_deg

    @property
    def is_searching(self):
        return self.patroller.is_searching

    @property
    def search_time(self):
        return self.patroller.search_time

    @search_time.setter
    def search_time(self, val):
        self.patroller.search_time = val

    def select_best_target(self, boxes, center_x, center_y):
        """Kutular arasindan en uygun hedefi secer."""
        return self.selector.select(boxes, center_x, center_y)

    def update(self, best_box, center_x, center_y, fx, fy, dt, is_firing=False, is_hud_firing=False, current_pan=None, current_tilt=None, dist_cm=None):
        """FCS ana kontrol cevrimi: tespit, takip, onleme, kilit ve motor hedefleme."""
        dt = float(dt) if (dt is not None and dt > 0.0) else 0.016

        if best_box is not None:
            self.frames_since_lost = 0
            self.patroller.is_searching = False

            # 1. Tespit koordinatlarini cikar
            bx, by, raw_p, raw_t, w_px, w_py = self.aimer.process_detection(
                best_box, center_x, center_y, fx, fy, current_pan, current_tilt
            )

            # 2. LiDAR Mesafe Filtresi
            if dist_cm is not None and 120 <= dist_cm <= 280:
                self.filtered_dist_m = 0.85 * self.filtered_dist_m + 0.15 * (dist_cm / 100.0)
            target_d_m = float(self.filtered_dist_m)
            lead_time = float(np.clip((target_d_m / 10.0) + 0.08, 0.10, 0.35))

            if not is_firing:
                _, _, kvx, kvy = self.tracker.update((w_px, w_py), dt=dt)
                self.target_velocity = np.array([kvx, kvy], dtype=float)

                d_lead_x = kvx * lead_time
                d_lead_y = kvy * lead_time
                self.pred_center = np.array([bx + d_lead_x, by + d_lead_y], dtype=float)

                # Takip hatasina bagli yumusak onleme gecisi
                angular_err = float(np.hypot(self.aimer.actual_pan, self.aimer.actual_tilt))
                lead_scale = float(np.clip((3.5 - angular_err) / 1.5, 0.0, 1.0))

                raw_lead_pan = float(np.clip(np.degrees(np.arctan(d_lead_x / max(fx, 1e-6))), -10.0, 10.0))
                raw_lead_tilt = float(np.clip(np.degrees(np.arctan(-d_lead_y / max(fy, 1e-6))), -6.0, 6.0))

                self.firing_lead_offset = np.array([raw_lead_pan * lead_scale, raw_lead_tilt * lead_scale], dtype=float)

            # 3. Kilitlenme durumunu ve isabet olasiligini guncelle
            self.lock_mgr.update(self.aimer.actual_pan, self.aimer.actual_tilt, self.target_velocity, is_firing=(is_firing or is_hud_firing), dist_m=target_d_m)

            # 4. Motor komutlarini hesapla (HYBRID, PID veya RL Modu)
            if self.control_mode == "HYBRID" and self.rl_controller.is_loaded:
                # 4a. Temel PID Motor Hedefini Hesapla (Puruzsuz & Kararli Omurga)
                pid_pan, pid_tilt = self.aimer.aim(
                    current_pan, current_tilt, raw_p, raw_t, target_d_m,
                    self.firing_lead_offset, self.pan_target, self.tilt_target, is_firing=is_firing
                )
                
                # 4b. RL Akilli Manevra & Onleme Asisini Hesapla
                curr_p_rad = current_pan if (current_pan is not None and not np.isnan(current_pan)) else self.pan_target
                curr_t_rad = current_tilt if (current_tilt is not None and not np.isnan(current_tilt)) else self.tilt_target
                screen_w = center_x * 2
                screen_h = center_y * 2
                
                d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                    bx, by, center_x, center_y, screen_w, screen_h,
                    self.target_velocity[0], self.target_velocity[1],
                    target_d_m, float(np.degrees(curr_p_rad)), float(np.degrees(curr_t_rad)), dt
                )
                
                # 4c. Cift Modlu Dinamik RL Agirligi (Hiz + Acisal Hata + Kilit Durumu Dengesi):
                # - Hedef kacarken / manevrada / kilit kirildiginda: Boost %70'e kadar firlar!
                # - Hedef kilitlendiginde (Lock Quality %80-%99): Boost otomatik olarak %5 - %10'a suzulur (Saf PID devralir).
                angular_err = float(np.hypot(self.aimer.actual_pan, self.aimer.actual_tilt))
                speed_mag = float(np.hypot(self.target_velocity[0], self.target_velocity[1]))
                lock_qual = float(self.lock_mgr.hit_probability)  # 0.0 ile 100.0 arasi
                
                # 1. Hiz ve Manevra Katkisi (%0 - %70 arasi)
                speed_ratio = float(np.clip(speed_mag / 200.0, 0.0, 1.0))
                weight_spd = 0.60 * speed_ratio
                weight_err = float(np.clip(angular_err / 2.5, 0.0, 0.25))
                raw_boost = weight_spd + weight_err
                
                # 2. Kilitlenme Sonumleme Carpani (Lock Suppression):
                # Kilit kalitesi %40'in altindayken carpan = 1.0 (Tam Guc RL).
                # Kilit kalitesi %95'e ciktiginda carpan = 0.10 (RL sessizlesir, Saf PID tutar).
                lock_factor = 1.0 - float(np.clip((lock_qual - 40.0) / 55.0, 0.0, 0.88))
                
                self.current_rl_weight = float(np.clip(raw_boost * lock_factor, 0.0, 0.75))
                
                # 4d. Hibrit Birlesim
                hybrid_pan = pid_pan - float(np.radians(d_pan_deg * self.current_rl_weight))
                hybrid_tilt = pid_tilt + float(np.radians(d_tilt_deg * self.current_rl_weight))
                
                self.pan_target = float(np.clip(hybrid_pan, self.aimer.PAN_LIMITS[0], self.aimer.PAN_LIMITS[1]))
                self.tilt_target = float(np.clip(hybrid_tilt, self.aimer.TILT_LIMITS[0], self.aimer.TILT_LIMITS[1]))

            elif self.control_mode == "RL" and self.rl_controller.is_loaded:
                self.current_rl_weight = 1.0
                curr_p_rad = current_pan if (current_pan is not None and not np.isnan(current_pan)) else self.pan_target
                curr_t_rad = current_tilt if (current_tilt is not None and not np.isnan(current_tilt)) else self.tilt_target
                
                curr_p_deg = float(np.degrees(curr_p_rad))
                curr_t_deg = float(np.degrees(curr_t_rad))
                screen_w = center_x * 2
                screen_h = center_y * 2
                
                d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                    bx, by, center_x, center_y, screen_w, screen_h,
                    self.target_velocity[0], self.target_velocity[1],
                    target_d_m, curr_p_deg, curr_t_deg, dt
                )
                
                new_pan_rad = curr_p_rad - float(np.radians(d_pan_deg))
                new_tilt_rad = curr_t_rad + float(np.radians(d_tilt_deg))
                
                self.pan_target = float(np.clip(new_pan_rad, self.aimer.PAN_LIMITS[0], self.aimer.PAN_LIMITS[1]))
                self.tilt_target = float(np.clip(new_tilt_rad, self.aimer.TILT_LIMITS[0], self.aimer.TILT_LIMITS[1]))
            else:
                self.pan_target, self.tilt_target = self.aimer.aim(
                    current_pan, current_tilt, raw_p, raw_t, target_d_m,
                    self.firing_lead_offset, self.pan_target, self.tilt_target, is_firing=is_firing
                )
        else:
            self.frames_since_lost += 1
            if self.frames_since_lost >= self.MAX_LOST_FRAMES:
                if not self.patroller.is_searching:
                    self.reset()
                self.pan_target, self.tilt_target = self.patroller.update(dt, self.pan_target, self.tilt_target)

        return self.pan_target, self.tilt_target

    def reset(self):
        """Tum alt modullerin hafizasini temizler."""
        self.lock_mgr.reset()
        self.aimer.reset()
        self.tracker.reset()
        self.patroller.reset()
        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.filtered_dist_m = 2.10
