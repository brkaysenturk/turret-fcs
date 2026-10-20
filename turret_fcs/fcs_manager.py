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
5. Mod Yonetimi ve Kontrolculer: Yumusak Olu Bolgeli Tip-1 P Kontrolcu (Hassas Kilit)
   veya 11-Boyutlu PPO RL Kontrolcu (Cevik Takip).
6. Sakin Dairesel Devriye: Hedef kayboldugunda 1.5 sn bekleme, merkeze donus ve 2 eksenli dairesel tarama.
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

    def __init__(self, control_mode="P", max_lost_frames=4, patrol_enabled=False):
        self.selector = TargetSelector()
        self.tracker = KalmanFilter()
        self.lock_mgr = LockManager()
        self.p_controller = PController()
        self.rl_controller = RLController()
        self.patrol = PatrolGenerator()

        self.control_mode = "RL" if control_mode == "RL" else "P"  # "P" (Precision) veya "RL" (Agile)
        self.current_rl_weight = 1.0 if self.control_mode == "RL" else 0.0
        self.patrol_enabled = bool(patrol_enabled)
        self.patrol_status = "STANDBY"

        self.max_lost_frames = max_lost_frames
        self.frames_since_lost = 0

        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0

        # Son karede tarete gonderilen motor komutlari (Ego-motion kompanzasyonu icin)
        self.last_cmd_pan = 0.0
        self.last_cmd_tilt = 0.0

        # Taret Açısal Konum Kestirimi (Dead-Reckoning Odometri - Devriye ve Emniyet Sinirlari)
        self.turret_pan_deg = 0.0
        self.turret_tilt_deg = 0.0
        self.PAN_LIMIT_MIN = -75.0
        self.PAN_LIMIT_MAX = 75.0
        self.TILT_LIMIT_MIN = -25.0
        self.TILT_LIMIT_MAX = 25.0

        # Aktuator Durum Gozlemcisi (Actuator State Observer - 1. derece motor dinamigi tau=45ms)
        self.servo_tau = 0.045
        self.actual_motor_pan = 0.0
        self.actual_motor_tilt = 0.0

        # Menzil Kapilama (Range Gating) icin dogrulanmis hedef mesafesi hafizasi
        self.last_valid_target_dist_m = float(self.p_controller.DEFAULT_DISTANCE_M)

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
            was_lost = (self.frames_since_lost > 0)
            self.frames_since_lost = 0
            x1, y1, x2, y2 = best_box
            bx = (x1 + x2) // 2
            by = (y1 + y2) // 2 

            # Devriye modulune hedefin son bilinen taret koordinatlarini bildir
            self.patrol.on_target_detected(self.turret_pan_deg, self.turret_tilt_deg)
            self.patrol_status = "TRACKING"

            # 1. Kamera merkezinden hedef acilari ve acisal hata
            pan_deg, tilt_deg = self.p_controller.compute_angles(bx, by, center_x, center_y, fx, fy)
            angular_err = float(np.hypot(pan_deg, tilt_deg))

            # 2. Menzil Kapilama (Range Gating):
            # TF-Luna dar isin acisina (~2 deg) sahiptir. Hedef vizor eksenine girmeden once
            # (angular_err > 3.0 deg), arka duvardan gelen yaniltici mesafeyi eler;
            # hedefin dogrulanmis son gercek mesafesini (veya varsayilan 2.0m) korur.
            if angular_err <= 3.0 and dist_cm is not None and dist_cm > 20:
                self.last_valid_target_dist_m = float(dist_cm / 100.0)

            d_m = self.last_valid_target_dist_m
            effective_dist_cm = float(d_m * 100.0)
            lead_time = float(np.clip((d_m / self.p_controller.PROJECTILE_SPEED_MPS) + 0.015, 0.03, 0.25))

            if not is_firing:
                # Taretin bir onceki karedeki donusunden kaynaklanan optik akis kaymasi (Ego-Motion)
                if not was_lost:
                    # 1. Derece aktüatör gecikme modeli (tau = 45 ms)
                    alpha_m = float(dt / (self.servo_tau + dt))
                    self.actual_motor_pan = (1.0 - alpha_m) * self.actual_motor_pan + alpha_m * self.last_cmd_pan
                    self.actual_motor_tilt = (1.0 - alpha_m) * self.actual_motor_tilt + alpha_m * self.last_cmd_tilt

                    ego_dx = - float(fx) * float(np.tan(np.radians(self.actual_motor_pan)))
                    ego_dy =   float(fy) * float(np.tan(np.radians(self.actual_motor_tilt)))
                else:
                    self.actual_motor_pan = 0.0
                    self.actual_motor_tilt = 0.0
                    ego_dx = 0.0
                    ego_dy = 0.0

                kwx, kwy, kvx, kvy = self.tracker.update((float(bx), float(by)), dt=dt, ego_dx=ego_dx, ego_dy=ego_dy)
                pred_px, pred_py, _, _ = self.tracker.predict_next_position(lead_time=lead_time)
                self.target_velocity = np.array([kvx, kvy], dtype=float)

                d_lead_x = pred_px - float(bx)
                d_lead_y = pred_py - float(by)
                raw_pred = np.array([bx + d_lead_x, by + d_lead_y], dtype=float)
                self.pred_center = 0.70 * raw_pred + 0.30 * self.pred_center

                lead_pan, lead_tilt = self.p_controller.compute_lead_angles(d_lead_x, d_lead_y, fx, fy)
                self.firing_lead_offset = np.array([lead_pan, lead_tilt], dtype=float)

            # 3. Balistik yercekimi telafisi (Dogrulanmis kapilanmis mesafe ile)
            self.gravity_comp_deg = self.p_controller.compute_gravity_comp(effective_dist_cm)

            # 4. Kilit durumu kontrolu
            self.lock_mgr.update(angular_err, self.target_velocity, is_firing=(is_firing or is_hud_firing), dist_m=d_m)

            # 5. Motor kontrol komutu (Yumusak olu bolgeli saf oransal Tip-1 P kontrolcu)
            active_pan = float(pan_deg + (self.firing_lead_offset[0] if (is_firing or is_hud_firing) else 0.0))
            active_tilt = float(tilt_deg + ((self.firing_lead_offset[1] + self.gravity_comp_deg) if (is_firing or is_hud_firing) else 0.0))

            u_p_pan, u_p_tilt = self.p_controller.compute_p(active_pan, active_tilt, dt=dt)

            if self.control_mode == "RL" and self.rl_controller.is_loaded:
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
                self.turret_pan_deg = float(np.clip(self.turret_pan_deg + u_pan, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
                self.turret_tilt_deg = float(np.clip(self.turret_tilt_deg + u_tilt, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))
                self.last_cmd_pan = u_pan
                self.last_cmd_tilt = u_tilt
                return u_pan, u_tilt

            else:
                self.current_rl_weight = 0.0
                self.turret_pan_deg = float(np.clip(self.turret_pan_deg + u_p_pan, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
                self.turret_tilt_deg = float(np.clip(self.turret_tilt_deg + u_p_tilt, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))
                self.last_cmd_pan = u_p_pan
                self.last_cmd_tilt = u_p_tilt
                return u_p_pan, u_p_tilt

        else:
            self.frames_since_lost += 1
            if hasattr(self.rl_controller, "on_target_lost"):
                self.rl_controller.on_target_lost()
            if self.frames_since_lost >= self.max_lost_frames:
                self.tracker.reset()
                self.lock_mgr.reset()
                self.target_velocity = np.array([0.0, 0.0], dtype=float)
                self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
                self.actual_motor_pan = 0.0
                self.actual_motor_tilt = 0.0

            # Hedef yokken: Devriye modu aciksa sakin dairesel tarama (odometri ile merkeze donus)
            if self.patrol_enabled:
                d_p, d_t, self.patrol_status = self.patrol.update(
                    dt, current_pan=self.turret_pan_deg, current_tilt=self.turret_tilt_deg
                )
                self.turret_pan_deg = float(np.clip(self.turret_pan_deg + d_p, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
                self.turret_tilt_deg = float(np.clip(self.turret_tilt_deg + d_t, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))
                self.p_controller.actual_pan = 0.0
                self.p_controller.actual_tilt = 0.0
                self.last_cmd_pan = d_p
                self.last_cmd_tilt = d_t
                return d_p, d_t
            else:
                self.p_controller.actual_pan = 0.0
                self.p_controller.actual_tilt = 0.0
                self.patrol_status = "STANDBY"
                self.last_cmd_pan = 0.0
                self.last_cmd_tilt = 0.0
                return 0.0, 0.0

    def reset(self):
        self.frames_since_lost = 0
        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0
        self.last_cmd_pan = 0.0
        self.last_cmd_tilt = 0.0
        self.turret_pan_deg = 0.0
        self.turret_tilt_deg = 0.0
        self.actual_motor_pan = 0.0
        self.actual_motor_tilt = 0.0
        self.tracker.reset()
        self.lock_mgr.reset()
        self.p_controller.reset()
        self.patrol.reset()
        self.selector.reset()

