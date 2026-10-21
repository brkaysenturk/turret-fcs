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

try:
    from .target_selector import TargetSelector
    from .kalman_filter import KalmanFilter
    from .lock_manager import LockManager
    from .p_controller import PController
    from .patrol_generator import PatrolGenerator
    from .config import CONFIG
except (ImportError, ValueError):
    from target_selector import TargetSelector
    from kalman_filter import KalmanFilter
    from lock_manager import LockManager
    from p_controller import PController
    from patrol_generator import PatrolGenerator
    from config import CONFIG


class FCSManager:
    """
    Hedef secimi, Kalman durum takibi, dinamik kilit yonetimi, balistik ve tarete 
    atesleme koordinasyonunu yoneten ust seviye yonetici sinifi.
    """

    def __init__(self, control_mode="P", max_lost_frames=5, patrol_enabled=False):
        self.selector = TargetSelector()
        self.tracker = KalmanFilter()
        self.lock_mgr = LockManager()
        self.p_controller = PController()
        self.patrol = PatrolGenerator()

        self.control_mode = "RL" if control_mode == "RL" else "P"  # "P" (Precision) veya "RL" (Agile)
        self.rl_controller = None
        if self.control_mode == "RL":
            self._init_rl()

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
        self.PAN_LIMIT_MIN = CONFIG.mount.pan_limit_min
        self.PAN_LIMIT_MAX = CONFIG.mount.pan_limit_max
        self.TILT_LIMIT_MIN = CONFIG.mount.tilt_limit_min
        self.TILT_LIMIT_MAX = CONFIG.mount.tilt_limit_max

        # Aktuator Durum Gozlemcisi (Actuator State Observer - 1. derece motor dinamigi tau=45ms)
        self.servo_tau = CONFIG.mount.servo_tau
        self.actual_motor_pan = 0.0
        self.actual_motor_tilt = 0.0

        # Menzil Kapilama (Range Gating) icin dogrulanmis hedef mesafesi hafizasi
        self.last_valid_target_dist_m = float(CONFIG.ballistics.default_distance_m)

        # Balistik Slew ve Nisan Noktasi Ileri Beslemesi (Aim-Point Feedforward)
        self.prev_applied_lead_pan = 0.0
        self.prev_applied_lead_tilt = 0.0

    def _init_rl(self):
        """PPO / PyTorch agirligini yalnizca RL modu secildiginde tembel (lazy) yukler."""
        if self.rl_controller is None:
            try:
                from .rl_controller import RLController
            except (ImportError, ValueError):
                from rl_controller import RLController
            self.rl_controller = RLController()
        return self.rl_controller

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
    def kalman(self):
        """Kalman filtresi referansi (telemetri ve harici analitikler icin)."""
        return self.tracker

    @property
    def lock_quality(self):
        return self.lock_mgr.lock_quality

    @property
    def last_lock_quality(self):
        return self.lock_mgr.lock_quality

    @property
    def actual_pan(self):
        return self.p_controller.actual_pan

    @property 
    def actual_tilt(self):
        return self.p_controller.actual_tilt

    @property
    def target_conf(self):
        return getattr(self.selector, "last_target_conf", 0.0)

    @property
    def target_angular_velocity_deg_s(self):
        """Kalman filtresinden hesaplanan gurultusuz acisal hedef hizini (pan, tilt deg/s) doner."""
        fx = CONFIG.optics.compute_fx(CONFIG.optics.default_width)
        fy = CONFIG.optics.compute_fy(CONFIG.optics.default_height)
        return self.tracker.get_angular_velocity_deg_s(fx=fx, fy=fy)

    def select_best_target(self, boxes, center_x, center_y):
        return self.selector.select_best_target(boxes, center_x, center_y)

    def update(self, best_box, center_x, center_y, fx, fy, dt, is_firing=False, dist_cm=None, slew_factor=0.0):
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

            # Kalman filtresi ve balistik onleme atis esnasinda da canli akar (Lead dondurulmaz)
            _, _, kvx, kvy = self.tracker.update((float(bx), float(by)), dt=dt, ego_dx=ego_dx, ego_dy=ego_dy, dist_m=d_m)
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

            # 4. Kilit durumu kontrolu (Tam 2B Gauss ncx2 integrali ve hedef kutu boyutu ile)
            box_dim = (float(best_box[2] - best_box[0]), float(best_box[3] - best_box[1])) if best_box is not None else None
            self.lock_mgr.update(angular_err, self.target_velocity, is_firing=is_firing, dist_m=d_m, fx=fx, fy=fy, box_dim=box_dim)

            # 5. Balistik Yonelim ve Slew Hesaplamasi (2 Asamali Spin-up & Slew Modeli)
            # Normal takipte slew_factor = 0.0 (hedef optik merkezde boresight takip edilir).
            # Y basildiginda spin-up boyunca slew_factor 0.0 -> 1.0 puruzsuzce kayar.
            slew = float(np.clip(slew_factor, 0.0, 1.0))
            applied_lead_pan = float(slew * self.firing_lead_offset[0])
            applied_lead_tilt = float(slew * (self.firing_lead_offset[1] + self.gravity_comp_deg))

            # Aim-Point Ileri Besleme (Direct Feedforward):
            # Slew rampasinin anlik turevini P gecikmesine takilmadan dogrudan motorlara iletir.
            delta_slew_pan = float(applied_lead_pan - self.prev_applied_lead_pan)
            delta_slew_tilt = float(applied_lead_tilt - self.prev_applied_lead_tilt)
            self.prev_applied_lead_pan = applied_lead_pan
            self.prev_applied_lead_tilt = applied_lead_tilt

            active_pan = float(pan_deg + applied_lead_pan)
            active_tilt = float(tilt_deg + applied_lead_tilt)

            u_p_pan, u_p_tilt = self.p_controller.compute_p(active_pan, active_tilt, dt=dt)

            # Rampa adimini dogrudan motor komutuna ekle (Zero-Lag Slew)
            u_p_pan = float(np.clip(u_p_pan + delta_slew_pan, -self.p_controller.max_output, self.p_controller.max_output))
            u_p_tilt = float(np.clip(u_p_tilt + delta_slew_tilt, -self.p_controller.max_output, self.p_controller.max_output))

            if self.control_mode == "RL":
                if self.rl_controller is None:
                    self._init_rl()
                if self.rl_controller.is_loaded:
                    self.current_rl_weight = 1.0
                    screen_w = center_x * 2
                    screen_h = center_y * 2

                    # RL modelini balistik hedef sapmasiyla besleme (Birlestirilmis Balistik Hedef Noktasi)
                    eff_bx = center_x + float(np.tan(np.radians(active_pan)) * fx)
                    eff_by = center_y - float(np.tan(np.radians(active_tilt)) * fy)

                    # RL gozlem uzayina olu bolgesiz ham Kalman hizini (raw_vx, raw_vy) besle
                    raw_vx = getattr(self.tracker, "raw_vx", self.target_velocity[0])
                    raw_vy = getattr(self.tracker, "raw_vy", self.target_velocity[1])

                    d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                        eff_bx, eff_by, center_x, center_y, screen_w, screen_h,
                        raw_vx, raw_vy,
                        d_m, active_pan, active_tilt, dt
                    )
                    u_pan = float(np.clip(d_pan_deg + delta_slew_pan, -self.p_controller.max_output, self.p_controller.max_output))
                    u_tilt = float(np.clip(d_tilt_deg + delta_slew_tilt, -self.p_controller.max_output, self.p_controller.max_output))
                    self.turret_pan_deg = float(np.clip(self.turret_pan_deg + u_pan, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
                    self.turret_tilt_deg = float(np.clip(self.turret_tilt_deg + u_tilt, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))
                    self.last_cmd_pan = u_pan
                    self.last_cmd_tilt = u_tilt
                    return u_pan, u_tilt

            # P modu veya RL yuklu degilse:
            self.current_rl_weight = 0.0
            self.turret_pan_deg = float(np.clip(self.turret_pan_deg + u_p_pan, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
            self.turret_tilt_deg = float(np.clip(self.turret_tilt_deg + u_p_tilt, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))
            self.last_cmd_pan = u_p_pan
            self.last_cmd_tilt = u_p_tilt
            return u_p_pan, u_p_tilt

        else:
            self.frames_since_lost += 1
            if self.rl_controller is not None and hasattr(self.rl_controller, "on_target_lost"):
                self.rl_controller.on_target_lost()
            if self.frames_since_lost >= self.max_lost_frames:
                self.tracker.reset()
                self.lock_mgr.reset()
                self.target_velocity = np.array([0.0, 0.0], dtype=float)
                self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
                self.actual_motor_pan = 0.0
                self.actual_motor_tilt = 0.0
                self.prev_applied_lead_pan = 0.0
                self.prev_applied_lead_tilt = 0.0

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

    def sync_turret_pose(self, pan_deg, tilt_deg):
        """STM32 donanim telemetrisinden gelen gercek servo PWM acilari ile odometriyi senkronize eder."""
        if pan_deg is not None and tilt_deg is not None:
            self.turret_pan_deg = float(np.clip(pan_deg, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
            self.turret_tilt_deg = float(np.clip(tilt_deg, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))

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

