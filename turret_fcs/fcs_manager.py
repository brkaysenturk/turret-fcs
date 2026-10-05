import numpy as np

from target_selector import TargetSelector
from kalman_filter import KalmanFilter
from lock_manager import LockManager
from pid_controller import PIDController
from rl_controller import RLController


class FCSManager:
    """
    Hedef secimi, Kalman durum takibi, dinamik kilit yonetimi, balistik ve tarete 
    atesleme koordinasyonunu yoneten ust seviye yonetici sinifi.
    """

    def __init__(
        self,
        kp=0.42,
        ki=0.18,
        kd=0.038,
        alpha=0.85,
        min_conf=0.30,
        lock_threshold_frames=15,
        max_lost_frames=12,
        control_mode="HYBRID",
    ):
        self.selector = TargetSelector(min_conf=min_conf)
        self.tracker = KalmanFilter(dt=0.033)
        self.lock_mgr = LockManager(lock_threshold_frames=lock_threshold_frames)
        self.aimer = PIDController(kp=kp, ki=ki, kd=kd, alpha=alpha)
        self.rl_controller = RLController()
        self.control_mode = control_mode  # "HYBRID", "PID", "RL"
        self.current_rl_weight = 0.0

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
        return self.aimer.actual_pan

    @property 
    def actual_tilt(self):
        return self.aimer.actual_tilt

    def select_best_target(self, boxes, center_x, center_y):
        return self.selector.select_best_target(boxes, center_x, center_y)

    def update(self, best_box, center_x, center_y, fx, fy, dt, is_firing=False, is_hud_firing=False, dist_cm=None):
        dt = 0.033 if (dt is None or dt <= 0.0) else float(dt)

        if best_box is not None:
            self.frames_since_lost = 0
            x1, y1, x2, y2 = best_box
            bx = (x1 + x2) // 2
            by = (y1 + y2) // 2 

            # 1. Camera center'dan hedef acilari
            pan_deg, tilt_deg = self.aimer.compute_angles(bx, by, center_x, center_y, fx, fy)

            # 2. Kalman hiz tahmini ve onleme hesaplamasi
            d_m = (dist_cm / 100.0) if (dist_cm is not None and dist_cm > 20) else self.aimer.DEFAULT_DISTANCE_M
            lead_time = float(np.clip((d_m / self.aimer.PROJECTILE_SPEED_MPS) + 0.015, 0.03, 0.25))

            if not is_firing:
                kwx, kwy, kvx, kvy = self.tracker.update((float(bx), float(by)), dt=dt)
                pred_px, pred_py, _, _ = self.tracker.predict_next_position(lead_time=lead_time)
                self.target_velocity = np.array([kvx, kvy], dtype=float)

                d_lead_x = pred_px - float(bx)
                d_lead_y = pred_py - float(by)
                raw_pred = np.array([bx + d_lead_x, by + d_lead_y], dtype=float)
                self.pred_center = 0.70 * raw_pred + 0.30 * self.pred_center

                lead_pan, lead_tilt = self.aimer.compute_lead_angles(d_lead_x, d_lead_y, fx, fy)
                self.firing_lead_offset = np.array([lead_pan, lead_tilt], dtype=float)

            # 3. Dusuk aci / yercekimi telafisi
            self.gravity_comp_deg = self.aimer.compute_gravity_comp(dist_cm)

            # 4. Kilit durumu kontrolu
            angular_err = float(np.hypot(pan_deg, tilt_deg))
            self.lock_mgr.update(angular_err, self.target_velocity, is_firing=(is_firing or is_hud_firing), dist_m=d_m)

            # 5. Motor kontrol komutu (HYBRID, PID veya RL Modu)
            active_pan = float(pan_deg + (self.firing_lead_offset[0] if (is_firing or is_hud_firing) else 0.0))
            active_tilt = float(tilt_deg + ((self.firing_lead_offset[1] + self.gravity_comp_deg) if (is_firing or is_hud_firing) else 0.0))

            u_pid_pan, u_pid_tilt = self.aimer.compute_pid(active_pan, active_tilt, dt)

            if self.control_mode == "HYBRID" and self.rl_controller.is_loaded:
                screen_w = center_x * 2
                screen_h = center_y * 2
                d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                    bx, by, center_x, center_y, screen_w, screen_h,
                    self.target_velocity[0], self.target_velocity[1],
                    d_m, float(pan_deg), float(tilt_deg), dt
                )
                
                # Cift Modlu Uyarlamali Agirlik (Kilitliyken %0-%10, Manevrada %65-%75)
                speed_mag = float(np.hypot(self.target_velocity[0], self.target_velocity[1]))
                lock_qual = float(self.lock_mgr.hit_probability)
                
                speed_ratio = float(np.clip(speed_mag / 200.0, 0.0, 1.0))
                weight_spd = 0.60 * speed_ratio
                weight_err = float(np.clip(angular_err / 2.5, 0.0, 0.25))
                raw_boost = weight_spd + weight_err
                
                lock_factor = 1.0 - float(np.clip((lock_qual - 40.0) / 55.0, 0.0, 0.88))
                self.current_rl_weight = float(np.clip(raw_boost * lock_factor, 0.0, 0.75))
                
                u_pan = float(np.clip(u_pid_pan + (d_pan_deg * self.current_rl_weight), -self.aimer.max_output, self.aimer.max_output))
                u_tilt = float(np.clip(u_pid_tilt + (d_tilt_deg * self.current_rl_weight), -self.aimer.max_output, self.aimer.max_output))
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
                u_pan = float(np.clip(d_pan_deg, -self.aimer.max_output, self.aimer.max_output))
                u_tilt = float(np.clip(d_tilt_deg, -self.aimer.max_output, self.aimer.max_output))
                return u_pan, u_tilt

            else:
                self.current_rl_weight = 0.0
                return u_pid_pan, u_pid_tilt

        else:
            self.frames_since_lost += 1
            if self.frames_since_lost >= self.max_lost_frames:
                self.reset()
            return 0.0, 0.0

    def reset(self):
        self.frames_since_lost = 0
        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0
        self.tracker.reset()
        self.lock_mgr.reset()
        self.aimer.reset()
