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
6. Sakin Figure-8 (Lissajous) Devriye: Hedef kayboldugunda 1.5 sn bekleme, merkeze donus ve 2 eksenli sekiz motifi tarama.
================================================================================
"""

import numpy as np
from collections import deque

from .target_selector import TargetSelector
from .kalman_filter import KalmanFilter
from .lock_manager import LockManager
from .p_controller import PController
from .patrol_generator import PatrolGenerator
from .config import CONFIG


class FCSManager:
    """
    Hedef secimi, Kalman durum takibi, dinamik kilit yonetimi, balistik ve tarete 
    atesleme koordinasyonunu yoneten ust seviye yonetici sinifi.
    """

    def __init__(self, control_mode="P", max_lost_frames=CONFIG.selector.max_lost_frames, patrol_enabled=False):
        self.selector = TargetSelector()
        self.tracker = KalmanFilter()
        self.lock_mgr = LockManager()
        self.p_controller = PController()
        self.patrol = PatrolGenerator()
        self.control_mode = "RL" if control_mode == "RL" else "P"  # "P" (Precision) veya "RL" (Agile)
        self.rl_controller = None
        if self.control_mode == "RL":
            self._init_rl()

        self.patrol_enabled = bool(patrol_enabled)
        self.patrol_status = "STANDBY"

        self.max_lost_frames = max_lost_frames
        self.frames_since_lost = 0

        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0
        self.last_pan_err_deg = 0.0
        self.last_tilt_err_deg = 0.0

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

        # Ego-motion: son komutun 1. derece servo yaniti (kare basi piksel kaymasi icin)
        self.ego_tau_s = CONFIG.control.ego_tau_s
        self.ego_step_pan_deg = 0.0
        self.ego_step_tilt_deg = 0.0

        # Menzil Kapilama (Range Gating) icin dogrulanmis hedef mesafesi hafizasi
        self.last_valid_target_dist_m = float(CONFIG.ballistics.default_distance_m)
        self.range_valid = False
        self.range_stable = False
        self.range_jump = False
        self.range_geom_m = None
        self.range_geom_mismatch = False
        self._range_history = deque(maxlen=max(3, int(CONFIG.lidar.stability_window)))
        self._range_candidate = None
        self._range_candidate_count = 0

        # Balistik Slew ve Nisan Noktasi Ileri Beslemesi (Aim-Point Feedforward)
        self.prev_applied_lead_pan = 0.0
        self.prev_applied_lead_tilt = 0.0

    def _integrate_command(self, u_pan, u_tilt):
        """Hesaplanan inkremental adimlari limitler dahilinde tarete uygular."""
        self.turret_pan_deg = float(np.clip(self.turret_pan_deg + u_pan, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
        self.turret_tilt_deg = float(np.clip(self.turret_tilt_deg + u_tilt, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))
        self.last_cmd_pan = u_pan
        self.last_cmd_tilt = u_tilt
        return u_pan, u_tilt

    def _compute_ego_pixel_shift(self, dt, fx, fy):
        """1. derece servo yanitini gunceller ve kare basi ego piksel kaymasini hesaplar."""
        alpha_m = float(dt / (self.ego_tau_s + dt))
        self.ego_step_pan_deg = (1.0 - alpha_m) * self.ego_step_pan_deg + alpha_m * self.last_cmd_pan
        self.ego_step_tilt_deg = (1.0 - alpha_m) * self.ego_step_tilt_deg + alpha_m * self.last_cmd_tilt
        ego_dx = -float(fx) * float(np.tan(np.radians(self.ego_step_pan_deg)))
        ego_dy = float(fy) * float(np.tan(np.radians(self.ego_step_tilt_deg)))
        return ego_dx, ego_dy

    def _estimate_geom_distance(self, box_dim, fx, fy):
        """Hedef kutusunun acisal boyutundan mesafe tahmini (0.15 m yaricap varsayimi)."""
        if not box_dim or fx is None or fy is None:
            return None
        bw, bh = float(box_dim[0]), float(box_dim[1])
        if bw <= 0.0 or bh <= 0.0 or fx <= 1.0 or fy <= 1.0:
            return None
        theta_pan = np.arctan((bw / 2.0) / fx)
        theta_tilt = np.arctan((bh / 2.0) / fy)
        theta = 0.5 * (theta_pan + theta_tilt)
        if theta <= 1e-4:
            return None
        d_geom = float(CONFIG.lock.target_radius_m / np.tan(theta))
        return d_geom if np.isfinite(d_geom) and d_geom > CONFIG.lidar.min_distance_m else None

    def _update_lidar_range(self, dist_cm, is_firing, box_dim, fx, fy):
        """
        TF-Luna menzil kapilama:
        - Ates sirasinda mesafeyi dondur (spin-up'ta duvar okumasini engelle).
        - Son gecerli mesafeden 50 cm'den fazla sicramayi reddet.
        - 5 kare boyunca tutarli yeni mesafeyi gercek degisim olarak kabul et.
        - Kutu geometrisiyle celisen okumayi (duvar) reddet.
        - Ates icin 3 kare / 10 cm kararlilik iste.
        """
        L = CONFIG.lidar
        self.range_geom_m = self._estimate_geom_distance(box_dim, fx, fy)
        self.range_geom_mismatch = False

        if dist_cm is None or float(dist_cm) <= 0.0:
            self.range_valid = False
            self.range_stable = False
            self.range_jump = False
            return

        d_new = float(dist_cm) / 100.0
        if not (L.min_distance_m <= d_new <= L.max_distance_m):
            self.range_valid = False
            self.range_stable = False
            self.range_jump = False
            return

        # Ates sirasinda LiDAR'in arka plani okumasina izin verme.
        if is_firing:
            self.range_jump = False
            return

        geom_bad = False
        if self.range_geom_m is not None and self.range_geom_m > 0.3:
            ratio = abs(d_new - self.range_geom_m) / self.range_geom_m
            geom_bad = ratio > L.geometry_tolerance_ratio
        self.range_geom_mismatch = geom_bad

        last = float(self.last_valid_target_dist_m)
        jump = bool(self._range_history) and (abs(d_new - last) > L.jump_threshold_m)
        self.range_jump = jump

        if jump:
            self.range_valid = False
            self.range_stable = False
            # Duvar/arka plan ise adayi hic kabul etme.
            if geom_bad:
                self._range_candidate = None
                self._range_candidate_count = 0
                return
            # Gercek hedef mesafesi degismis olabilir: 5 kare tutarliysa kabul et.
            if self._range_candidate is None or abs(d_new - self._range_candidate) > L.stability_tolerance_m:
                self._range_candidate = d_new
                self._range_candidate_count = 1
            else:
                self._range_candidate_count += 1
            if self._range_candidate_count >= L.reacquire_frames:
                self.last_valid_target_dist_m = d_new
                self._range_history.clear()
                self._range_history.append(d_new)
                self._range_candidate = None
                self._range_candidate_count = 0
                self.range_jump = False
                self.range_valid = True
                self.range_stable = False
            return

        # Normal, sicramasiz okuma.
        self._range_candidate = None
        self._range_candidate_count = 0
        # Geometri yalnizca teshis/uyari; normal hedef okumasini engellemez.
        self.last_valid_target_dist_m = d_new
        self._range_history.append(d_new)
        self.range_valid = True
        if len(self._range_history) >= L.stability_window:
            self.range_stable = (max(self._range_history) - min(self._range_history)) <= L.stability_tolerance_m
        else:
            self.range_stable = False

    def _reset_lidar_range(self):
        """Hedef kaybinda filtre gecmisini sifirla; son gecerli mesafeyi koru."""
        self.range_valid = False
        self.range_stable = False
        self.range_jump = False
        self.range_geom_mismatch = False
        self._range_history.clear()
        self._range_candidate = None
        self._range_candidate_count = 0

    def _init_rl(self):
        """PPO politika agirligini yalnizca RL modu secildiginde tembel (lazy) yukler."""
        if self.rl_controller is None:
            from .rl_controller import RLController
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
    def actual_pan(self):
        return self.p_controller.actual_pan

    @property
    def actual_tilt(self):
        return self.p_controller.actual_tilt

    @property
    def target_conf(self):
        return getattr(self.selector, "last_target_conf", 0.0)

    def select_best_target(self, boxes, center_x, center_y):
        return self.selector.select_best_target(boxes, center_x, center_y)

    def _update_tracking_and_ballistics(self, best_box, center_x, center_y, fx, fy, dt, is_firing, dist_cm):
        """Hedef acilari, menzil kapilama, ego-hareket, Kalman hiz kestirimi ve kilit durumunu gunceller."""
        self.frames_since_lost = 0
        x1, y1, x2, y2 = best_box
        bx = (x1 + x2) // 2
        by = (y1 + y2) // 2
        box_dim = (float(x2 - x1), float(y2 - y1))

        # Devriye modulune hedefin son bilinen taret koordinatlarini bildir
        self.patrol.on_target_detected(self.turret_pan_deg, self.turret_tilt_deg)
        self.patrol_status = "TRACKING"

        # 1. Kamera merkezinden hedef acilari ve acisal hata
        pan_deg, tilt_deg = self.p_controller.compute_angles(bx, by, center_x, center_y, fx, fy)
        # Telemetri icin ham kamera hatasi (kontrolcu ici EMA'dan bagimsiz).
        self.last_pan_err_deg = float(np.degrees(np.arctan((bx - center_x) / max(float(fx), 1e-6))))
        self.last_tilt_err_deg = float(np.degrees(np.arctan(-(by - center_y) / max(float(fy), 1e-6))))
        angular_err = float(np.hypot(pan_deg, tilt_deg))

        # 2. Menzil Kapilama (Range Gating)
        self._update_lidar_range(dist_cm, is_firing, box_dim, fx, fy)
        d_m = self.last_valid_target_dist_m
        effective_dist_cm = float(d_m * 100.0)
        lead_time = float(np.clip(
            (d_m / self.p_controller.PROJECTILE_SPEED_MPS) + CONFIG.control.lead_time_offset_s,
            CONFIG.control.lead_time_min_s,
            CONFIG.control.lead_time_max_s,
        ))

        # 3. Ego-Motion (son motor komutunun 1. derece servo yaniti)
        if self.tracker.initialized:
            ego_dx, ego_dy = self._compute_ego_pixel_shift(dt, fx, fy)
        else:
            self.ego_step_pan_deg = 0.0
            self.ego_step_tilt_deg = 0.0
            ego_dx = 0.0
            ego_dy = 0.0

        # 4. Kalman filtresi, balistik onleme ve yercekimi telafisi
        _, _, kvx, kvy = self.tracker.update((float(bx), float(by)), dt=dt, ego_dx=ego_dx, ego_dy=ego_dy)
        self.target_velocity = np.array([kvx, kvy], dtype=float)

        d_lead_x = kvx * lead_time
        d_lead_y = kvy * lead_time
        self.pred_center = np.array([bx + d_lead_x, by + d_lead_y], dtype=float)

        lead_pan, lead_tilt = self.p_controller.compute_lead_angles(d_lead_x, d_lead_y, fx, fy)
        self.firing_lead_offset = np.array([lead_pan, lead_tilt], dtype=float)
        self.gravity_comp_deg = self.p_controller.compute_gravity_comp(effective_dist_cm)

        # 5. Kilit durumu kontrolu (Tam 2B Gauss ncx2 integrali ve hedef kutu boyutu ile)
        self.lock_mgr.update(angular_err, self.target_velocity, is_firing=is_firing, dist_m=d_m, fx=fx, fy=fy, box_dim=box_dim)
        return bx, by, pan_deg, tilt_deg, d_m

    def _select_control_command(self, bx, by, center_x, center_y, fx, fy, pan_deg, tilt_deg, d_m, dt, slew_factor):
        """Slew rampasi ileri beslemesini ve P / RL kontrolcu komut secimini hesaplar."""
        slew = float(np.clip(slew_factor, 0.0, 1.0))
        applied_lead_pan = float(slew * self.firing_lead_offset[0])
        applied_lead_tilt = float(slew * (self.firing_lead_offset[1] + self.gravity_comp_deg))

        # Aim-Point Ileri Besleme (Direct Feedforward): Slew rampasinin anlik turevi
        delta_slew_pan = float(applied_lead_pan - self.prev_applied_lead_pan)
        delta_slew_tilt = float(applied_lead_tilt - self.prev_applied_lead_tilt)
        self.prev_applied_lead_pan = applied_lead_pan
        self.prev_applied_lead_tilt = applied_lead_tilt

        active_pan = float(pan_deg + applied_lead_pan)
        active_tilt = float(tilt_deg + applied_lead_tilt)

        u_p_pan, u_p_tilt = self.p_controller.compute_p(active_pan, active_tilt, dt=dt)
        step_limit_pan = self.p_controller.max_step_for(dt, axis="pan")
        step_limit_tilt = self.p_controller.max_step_for(dt, axis="tilt")

        u_p_pan = float(np.clip(u_p_pan + delta_slew_pan, -step_limit_pan, step_limit_pan))
        u_p_tilt = float(np.clip(u_p_tilt + delta_slew_tilt, -step_limit_tilt, step_limit_tilt))

        if self.control_mode == "RL":
            if self.rl_controller is None:
                self._init_rl()
            if self.rl_controller.is_loaded:
                screen_w = center_x * 2
                screen_h = center_y * 2
                eff_bx = float(bx) + float(np.tan(np.radians(applied_lead_pan))) * fx
                eff_by = float(by) - float(np.tan(np.radians(applied_lead_tilt))) * fy

                # RL hiz gozlemi kasitli olarak sifirlanir (Kalman gurultusunun limit cycle uretmesini onler)
                obs_vx, obs_vy = 0.0, 0.0
                d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                    eff_bx, eff_by, center_x, center_y, screen_w, screen_h,
                    obs_vx, obs_vy, d_m, active_pan, active_tilt, dt
                )
                u_pan = float(np.clip(d_pan_deg + delta_slew_pan, -step_limit_pan, step_limit_pan))
                u_tilt = float(np.clip(d_tilt_deg + delta_slew_tilt, -step_limit_tilt, step_limit_tilt))
                return self._integrate_command(u_pan, u_tilt)

        return self._integrate_command(u_p_pan, u_p_tilt)

    def _handle_target_lost(self, dt, fx=None, fy=None):
        """Hedef kaybinda tolerans sayacini, coasting kestirimini, sifirlamayi ve Figure-8 devriye hareketini yonetir."""
        if fx is None:
            fx = CONFIG.optics.compute_fx(CONFIG.optics.default_width)
        if fy is None:
            fy = CONFIG.optics.compute_fy(CONFIG.optics.default_height)
        self.frames_since_lost += 1
        self.last_pan_err_deg = 0.0
        self.last_tilt_err_deg = 0.0
        self._reset_lidar_range()
        self.p_controller.on_target_lost()
        if self.rl_controller is not None and hasattr(self.rl_controller, "on_target_lost"):
            self.rl_controller.on_target_lost()
        if self.frames_since_lost >= self.max_lost_frames:
            self.tracker.reset()
            self.lock_mgr.reset()
            self.target_velocity = np.array([0.0, 0.0], dtype=float)
            self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
            self.ego_step_pan_deg = 0.0
            self.ego_step_tilt_deg = 0.0
            self.prev_applied_lead_pan = 0.0
            self.prev_applied_lead_tilt = 0.0
        elif self.tracker.initialized:
            ego_dx, ego_dy = self._compute_ego_pixel_shift(dt, fx, fy)
            px, py, kvx, kvy = self.tracker.predict_only(dt=dt, ego_dx=ego_dx, ego_dy=ego_dy)
            self.target_velocity = np.array([kvx, kvy], dtype=float)
            self.pred_center = np.array([px, py], dtype=float)

        self.p_controller.actual_pan = 0.0
        self.p_controller.actual_tilt = 0.0
        if self.patrol_enabled:
            d_p, d_t, self.patrol_status = self.patrol.update(
                dt, current_pan=self.turret_pan_deg, current_tilt=self.turret_tilt_deg
            )
            return self._integrate_command(d_p, d_t)

        self.patrol_status = "STANDBY"
        self.last_cmd_pan = 0.0
        self.last_cmd_tilt = 0.0
        return 0.0, 0.0

    def update(self, best_box, center_x, center_y, fx, fy, dt, is_firing=False, dist_cm=None, slew_factor=0.0):
        """Hedef kutusuna gore takip, balistik, kilit ve motor komut adimini (d_pan, d_tilt) hesaplar."""
        dt = 0.033 if (dt is None or dt <= 0.0) else float(dt)
        if best_box is not None:
            bx, by, pan_deg, tilt_deg, d_m = self._update_tracking_and_ballistics(
                best_box, center_x, center_y, fx, fy, dt, is_firing, dist_cm
            )
            return self._select_control_command(
                bx, by, center_x, center_y, fx, fy, pan_deg, tilt_deg, d_m, dt, slew_factor
            )
        return self._handle_target_lost(dt, fx=fx, fy=fy)

    def sync_turret_pose(self, pan_deg, tilt_deg):
        """STM32 geri beslemesinden taretin mutlak pozunu senkronize eder.

        Bu poz yalnizca devriye, limitler ve HUD icindir; ego-motion hesabi
        komut adimi modelinden gelir (bkz. update).
        """
        if pan_deg is not None and tilt_deg is not None:
            self.turret_pan_deg = float(np.clip(pan_deg, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
            self.turret_tilt_deg = float(np.clip(tilt_deg, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))

    def reset(self):
        self.frames_since_lost = 0
        self.patrol_status = "STANDBY"
        self._reset_lidar_range()
        self.last_valid_target_dist_m = float(CONFIG.ballistics.default_distance_m)
        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0
        self.last_pan_err_deg = 0.0
        self.last_tilt_err_deg = 0.0
        self.last_cmd_pan = 0.0
        self.last_cmd_tilt = 0.0
        self.turret_pan_deg = 0.0
        self.turret_tilt_deg = 0.0
        self.ego_step_pan_deg = 0.0
        self.ego_step_tilt_deg = 0.0
        self.prev_applied_lead_pan = 0.0
        self.prev_applied_lead_tilt = 0.0
        self.tracker.reset()
        self.lock_mgr.reset()
        self.p_controller.reset()
        if self.rl_controller is not None:
            self.rl_controller.reset()
        self.patrol.reset()
        self.selector.reset()
