"""
================================================================================
MODUL: ATIS KONTROL SISTEMI UST YONETICISI (FCS MANAGER)
================================================================================
Bu modul, otonom taretin en ust karar verici ve durum yoneticisidir (State Machine).
Tum alt kontrolculeri, filtreleri ve guvenlik mekanizmalarini koordine eder.

GOREVLER VE ALGORITMALAR:
1. Hedef Onceliklendirme: Coklu hedef arasindan merkeze en yakin/tehditkar olani secme.
2. Durum Kestirimi (Kalman Filter): Olcum gurultusunu filtreleyerek hedef hiz vektoru
   (vx, vy) ve gelecekteki konumunu kestirme.
3. Balistik Onleme ve Yercekimi Telafisi: Ucus suresi (Time-of-Flight) boyunca hedefin
   alacagi yolu hesaplayip atis onleme acisi (Lead Angle) ve irtifa dususunu kompanse etme.
4. Dinamik Kilit Yonetimi (Lock Manager): Histerezisli kilit penceresi ve analitik
   Gauss isabet olasiligi hesabi ile atis izni kontrolu.
5. Hibrit Kontrol Harmanlama: Hedef hizina ve kilit kalitesine gore P kontrolcu ile PPO RL
   arasinda dinamik agirliklandirma (%0 - %55 AI takviyesi).
6. Iki Asamali Akilli Devriye: Hedef kayboldugunda Grace Period, yerel ve genel arama.
================================================================================
"""

import numpy as np

from target_selector import TargetSelector
from kalman_filter import KalmanFilter
from lock_manager import LockManager
from p_controller import PController
from rl_controller import RLController
from patrol_generator import PatrolGenerator


class FCSManager:
    """
    Hedef secimi, Kalman durum takibi, dinamik kilit yonetimi, balistik ve tarete 
    atesleme koordinasyonunu yoneten ust seviye yonetici sinifi.
    """

    def __init__(self, control_mode="P", max_lost_frames=3, patrol_enabled=False):
        self.selector = TargetSelector()
        self.tracker = KalmanFilter()
        self.lock_mgr = LockManager()
        self.p_controller = PController()
        self.rl_controller = RLController()
        self.patrol = PatrolGenerator()

        self.control_mode = "P" if control_mode not in ["HYBRID", "RL"] else control_mode  # "P" (Varsayilan), "HYBRID", "RL"
        self.current_rl_weight = 0.0
        self.patrol_enabled = bool(patrol_enabled)
        self.patrol_status = "STANDBY"

        self.max_lost_frames = max_lost_frames
        self.frames_since_lost = 0

        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0

    @property
    def is_locked(self):
        return self.lock_mgr.is_locked

    @property
    def lock_counter(self):
        return self.lock_mgr.stability_counter

    @property
    def LOCK_THRESHOLD_FRAMES(self):
        return self.lock_mgr.lock_threshold_frames

    @property
    def hit_probability(self):
        return self.lock_mgr.hit_probability

    @property
    def last_lock_quality(self):
        return self.lock_mgr.hit_probability

    @property
    def actual_pan(self):
        return self.p_controller.actual_pan

    @property 
    def actual_tilt(self):
        return self.p_controller.actual_tilt

    def select_best_target(self, boxes, center_x, center_y):
        return self.selector.select_best_target(boxes, center_x, center_y)

    def update(self, best_box, center_x, center_y, fx, fy, dt, is_firing=False, is_hud_firing=False, dist_cm=None):
        dt = 0.033 if (dt is None or dt <= 0.0) else float(dt)

        if best_box is not None:
            self.frames_since_lost = 0
            x1, y1, x2, y2 = best_box
            bx = (x1 + x2) // 2
            by = (y1 + y2) // 2 

            # Devriye modulune hedefin son bilinen koordinatlarini bildir
            self.patrol.on_target_detected(self.actual_pan, self.actual_tilt)
            self.patrol_status = "TRACKING"

            # 1. Kamera merkezinden hedef acilari
            pan_deg, tilt_deg = self.p_controller.compute_angles(bx, by, center_x, center_y, fx, fy)

            # 2. Kalman durum takibi ve atis onleme hesaplamasi
            d_m = (dist_cm / 100.0) if (dist_cm is not None and dist_cm > 20) else self.p_controller.DEFAULT_DISTANCE_M
            lead_time = float(np.clip((d_m / self.p_controller.PROJECTILE_SPEED_MPS) + 0.015, 0.03, 0.25))

            if not is_firing:
                kwx, kwy, kvx, kvy = self.tracker.update((float(bx), float(by)), dt=dt)
                pred_px, pred_py, _, _ = self.tracker.predict_next_position(lead_time=lead_time)
                self.target_velocity = np.array([kvx, kvy], dtype=float)

                d_lead_x = pred_px - float(bx)
                d_lead_y = pred_py - float(by)
                raw_pred = np.array([bx + d_lead_x, by + d_lead_y], dtype=float)
                self.pred_center = 0.70 * raw_pred + 0.30 * self.pred_center

                lead_pan, lead_tilt = self.p_controller.compute_lead_angles(d_lead_x, d_lead_y, fx, fy)
                self.firing_lead_offset = np.array([lead_pan, lead_tilt], dtype=float)

            # 3. Balistik yercekimi telafisi
            self.gravity_comp_deg = self.p_controller.compute_gravity_comp(dist_cm)

            # 4. Kilit durumu kontrolu
            angular_err = float(np.hypot(pan_deg, tilt_deg))
            speed_mag = float(np.hypot(self.target_velocity[0], self.target_velocity[1]))
            self.lock_mgr.update(angular_err, self.target_velocity, is_firing=(is_firing or is_hud_firing), dist_m=d_m)

            # 5. Motor kontrol komutu (Hiza duyarli dinamik olu bolge ve adaptif kazanc hesaplamasi)
            active_pan = float(pan_deg + (self.firing_lead_offset[0] if (is_firing or is_hud_firing) else 0.0))
            active_tilt = float(tilt_deg + ((self.firing_lead_offset[1] + self.gravity_comp_deg) if (is_firing or is_hud_firing) else 0.0))

            u_p_pan, u_p_tilt = self.p_controller.compute_p(active_pan, active_tilt, dt, target_speed=speed_mag)

            if self.control_mode == "HYBRID" and self.rl_controller.is_loaded:
                screen_w = center_x * 2
                screen_h = center_y * 2
                d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                    bx, by, center_x, center_y, screen_w, screen_h,
                    self.target_velocity[0], self.target_velocity[1],
                    d_m, float(pan_deg), float(tilt_deg), dt
                )
                
                # Dinamik RL Agirligi (Hizli manevralarda gecikme telafisi icin %55'e kadar AI takviyesi)
                lock_qual = float(self.lock_mgr.hit_probability)
                speed_ratio = float(np.clip(speed_mag / 180.0, 0.0, 1.0))
                weight_spd = 0.65 * speed_ratio
                weight_err = float(np.clip(angular_err / 3.0, 0.0, 0.35))
                raw_boost = weight_spd + weight_err
                
                # Kilit saglamlastikca RL agirligini azalt, olu bolgede sifir titresim sagla
                lock_factor = 1.0 - float(np.clip((lock_qual - 35.0) / 60.0, 0.0, 0.85))
                self.current_rl_weight = float(np.clip(raw_boost * lock_factor, 0.0, 0.55))

                u_pan = float(np.clip(u_p_pan + (d_pan_deg * self.current_rl_weight), -self.p_controller.max_output, self.p_controller.max_output))
                u_tilt = float(np.clip(u_p_tilt + (d_tilt_deg * self.current_rl_weight), -self.p_controller.max_output, self.p_controller.max_output))
                return u_pan, u_tilt

            elif self.control_mode == "RL" and self.rl_controller.is_loaded:
                self.current_rl_weight = 1.0
                screen_w = center_x * 2
                screen_h = center_y * 2
                d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                    bx, by, center_x, center_y, screen_w, screen_h,
                    self.target_velocity[0], self.target_velocity[1],
                    d_m, float(pan_deg), float(tilt_deg), dt
                )
                u_pan = float(np.clip(d_pan_deg, -self.p_controller.max_output, self.p_controller.max_output))
                u_tilt = float(np.clip(d_tilt_deg, -self.p_controller.max_output, self.p_controller.max_output))
                return u_pan, u_tilt

            else:
                self.current_rl_weight = 0.0
                return u_p_pan, u_p_tilt

        else:
            self.frames_since_lost += 1
            if self.frames_since_lost >= 2:
                self.tracker.reset()
                self.lock_mgr.reset()
                self.target_velocity = np.array([0.0, 0.0], dtype=float)
                self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)

            # Hedef yokken: Devriye modu aciksa iki asamali akilli arama
            if self.patrol_enabled:
                d_p, d_t, self.patrol_status = self.patrol.update(dt, self.actual_pan, self.actual_tilt)
                self.p_controller.actual_pan += d_p
                self.p_controller.actual_tilt += d_t
                return d_p, d_t
            else:
                self.patrol_status = "STANDBY"
                return 0.0, 0.0

    def reset(self):
        self.frames_since_lost = 0
        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0
        self.tracker.reset()
        self.lock_mgr.reset()
        self.p_controller.reset()
        self.patrol.reset()
