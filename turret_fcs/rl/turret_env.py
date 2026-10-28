"""
================================================================================
MODUL: GERCEK DONANIM FIZIKLI RL BENZETIM ORTAMI (TURRET ENV / GYMNASIUM)
================================================================================
Bu modul, gercek taret donaniminin olculmus fiziksel kisitlarini:
1. Donanim tasima gecikmesi (Transport Delay Buffer: 33-100 ms / 1-3 kare rastgelelestirme)
2. Gercek servo olu bolgesi (Deadband: 0.10 derece) ve 1.20 derece disli boslugu
3. Kare basi maksimum adim: 1.50 derece (30 FPS'te 45 derece/s)
4. 1. Derece viskoz motor gecikmesi (Actuator Time Constant: tau = 35-65 ms)
5. Logitech C270 gercek optik modeli (H_FOV = 48.8 derece, V_FOV = 28.6 derece)
6. Domain Randomization (YOLO gurultusu, degisken gecikme, oransal FPS dalgalanmasi)
7. Eylem Gecmisi (Action History - 2 adimlik boru hatti farkindaligi)
8. Progress odulu: r = k_p*(e_prev^2 - e^2) - k_e*e^2
   - lambda_u*|u|^2 - mu_du*|du|^2 + bullseye
birebir iceren ve Bryson Kurali ile normalize edilmis 11 boyutlu Gymnasium ortamidir.
================================================================================
"""

from collections import deque
import math
import gymnasium as gym
from gymnasium import spaces
import numpy as np

import sys
from pathlib import Path

_current_dir = Path(__file__).resolve().parent
_turret_fcs_dir = _current_dir.parent
_project_root = _turret_fcs_dir.parent
for _p in [str(_turret_fcs_dir), str(_project_root)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from turret_fcs.config import CONFIG
except (ImportError, ValueError):
    from config import CONFIG


class TurretEnv(gym.Env):
    """
    Gercek servo dinamigi, olu bolge, donanim gecikmesi, Domain Randomization,
    eylem gecmisi ve sakinlik sartli Bryson normalizasyonlu FCS ortami.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(
        self,
        render_mode=None,
        dt=0.033,
        latency_steps=3,
        lambda_u=CONFIG.rl.lambda_u,
        mu_du=CONFIG.rl.mu_du,
        progress_gain=CONFIG.rl.progress_gain,
        error_gain=CONFIG.rl.error_gain,
        backlash_deg=CONFIG.actuator.backlash_deg,
        use_velocity_obs=False,
        enable_randomization=True,
    ):
        super().__init__()
        self.render_mode = render_mode
        self.base_dt = float(dt)
        self.base_latency_steps = int(latency_steps)
        self.latency_steps = self.base_latency_steps
        self.obs_queue = deque(maxlen=self.latency_steps + 1)
        self.enable_randomization = enable_randomization

        # Progress odulu: r = k_p * (e_prev^2 - e^2) - k_e * e^2
        #                - lambda_u * |u|^2 - mu_du * |du|^2 + bullseye
        self.lambda_u = float(lambda_u)
        self.mu_du = float(mu_du)
        self.progress_gain = float(progress_gain)
        self.error_gain = float(error_gain)
        self.backlash_deg = max(0.0, float(backlash_deg))
        self.use_velocity_obs = bool(use_velocity_obs)
        self.lock_streak = 0

        # 1. Logitech C270 Gercek Donanim Optik Parametreleri (SSOT)
        self.screen_w = float(CONFIG.optics.default_width)
        self.screen_h = float(CONFIG.optics.default_height)
        self.center_x = self.screen_w / 2.0
        self.center_y = self.screen_h / 2.0
        self.fx = CONFIG.optics.compute_fx(int(self.screen_w))
        self.fy = CONFIG.optics.compute_fy(int(self.screen_h))

        # 2. Gercek Donanim Mekanik ve Servo Sinirlari (SSOT)
        self.max_pan_deg = CONFIG.mount.pan_limit_max
        self.max_tilt_deg = CONFIG.mount.tilt_limit_max
        self.max_step_pan_deg = CONFIG.actuator.max_step_pan_deg
        self.max_step_tilt_deg = CONFIG.actuator.max_step_tilt_deg
        self.servo_deadband = CONFIG.actuator.servo_deadband_deg
        self.servo_tau = CONFIG.mount.servo_tau
        self.yolo_noise_std = CONFIG.rl.yolo_noise_std

        # 3. Bryson Kurali ile Normalizasyon Olcekleri (RL cikarimi ile ayni kaynak)
        self.d_err_scale = CONFIG.rl.d_err_scale
        self.d_err_ema_alpha = CONFIG.rl.d_err_ema_alpha
        self.v_max_px_s = CONFIG.rl.v_max_px_s
        self.dist_min_m = CONFIG.rl.dist_min_m
        self.dist_max_m = CONFIG.rl.dist_max_m

        # 4. RL Sarmalayici / Cikis Katmani Parametreleri (SSOT)
        self.apply_rl_wrapper = False
        self.soft_deadband_enabled = CONFIG.rl.soft_deadband_enabled
        self.deadband_pan_deg = CONFIG.control.deadband_pan_deg
        self.deadband_tilt_deg = CONFIG.control.deadband_tilt_deg
        self.taper_pan_deg = CONFIG.rl.taper_pan_deg
        self.taper_tilt_deg = CONFIG.rl.taper_tilt_deg
        self.reversal_damping = CONFIG.rl.reversal_damping
        self.reversal_hold_frames = getattr(CONFIG.rl, "reversal_hold_frames", 2)
        self.cmd_accel_pan_deg_s2 = CONFIG.control.cmd_accel_pan_deg_s2
        self.cmd_accel_tilt_deg_s2 = CONFIG.control.cmd_accel_tilt_deg_s2
        self.prev_policy_action = np.zeros(2, dtype=np.float32)
        self._reversal_timer = np.zeros(2, dtype=int)
        self.prev_cmd_pan = 0.0
        self.prev_cmd_tilt = 0.0

        # Eylem Uzayı: [-1.0, 1.0] araliginda normalize motor adimi
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        # 9 Boyutlu Gozlem Uzayi (Bryson Kuraliyla [-1.0, 1.0] araliginda)
        # [err_x, err_y, d_err_x, d_err_y, dist, act_pan_t1, act_tilt_t1, act_pan_t2, act_tilt_t2]
        self.observation_space = spaces.Box(
            low=-np.ones(9, dtype=np.float32),
            high=np.ones(9, dtype=np.float32),
            dtype=np.float32,
        )

        self.turret_pan = 0.0
        self.turret_tilt = 0.0
        self.actual_pan_step = 0.0
        self.actual_tilt_step = 0.0
        self.cmd_pan_deg = 0.0
        self.cmd_tilt_deg = 0.0
        self.backlash_pan_out = 0.0
        self.backlash_tilt_out = 0.0
        self.prev_e_sq = None
        self.target_pos_3d = np.zeros(3, dtype=np.float32)
        self.target_vel_3d = np.zeros(3, dtype=np.float32)
        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.last_action = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.last_pixel_pos = (self.center_x, self.center_y)
        self.trajectory_type = "linear"
        self.step_count = 0
        self.max_steps = CONFIG.rl.episode_steps
        self.dt = self.base_dt

    def _get_sensor_obs(self):
        """Kamera ve sensor olcumlerinden olusan 7-boyutlu fiziksel gozlemi hesaplar."""
        # 3B hedef koordinatlarindan taret acilarina projeksiyon
        target_azimuth = math.degrees(math.atan2(self.target_pos_3d[0], max(0.1, self.target_pos_3d[2])))
        target_elevation = math.degrees(math.atan2(self.target_pos_3d[1], max(0.1, self.target_pos_3d[2])))

        rel_pan = target_azimuth - self.turret_pan
        rel_tilt = target_elevation - self.turret_tilt

        px = self.center_x + math.tan(math.radians(rel_pan)) * self.fx
        py = self.center_y - math.tan(math.radians(rel_tilt)) * self.fy

        # Domain Randomization: Saf Gauss YOLO tespit kutusu titresimi
        if self.enable_randomization and self.yolo_noise_std > 0:
            noise_x = float(self.np_random.normal(0.0, self.yolo_noise_std))
            noise_y = float(self.np_random.normal(0.0, self.yolo_noise_std))
        else:
            noise_x, noise_y = 0.0, 0.0

        px_noisy = px + noise_x
        py_noisy = py + noise_y
        self.last_pixel_pos = (float(px_noisy), float(py_noisy))

        # 1. Hata (Ekran yaricapina gore Bryson normalizasyonu)
        err_x_norm = np.clip((px_noisy - self.center_x) / (self.screen_w / 2.0), -1.5, 1.5)
        err_y_norm = np.clip((py_noisy - self.center_y) / (self.screen_h / 2.0), -1.5, 1.5)

        # 2. Hata Turevi (Dusuk geciren filtreli / EMA smoothed - gurultuden arindirilmis)
        raw_d_err_x = (err_x_norm - self.last_err[0]) / max(self.dt, 1e-4) * self.d_err_scale
        raw_d_err_y = (err_y_norm - self.last_err[1]) / max(self.dt, 1e-4) * self.d_err_scale
        self.d_err_filt[0] = self.d_err_ema_alpha * raw_d_err_x + (1.0 - self.d_err_ema_alpha) * self.d_err_filt[0]
        self.d_err_filt[1] = self.d_err_ema_alpha * raw_d_err_y + (1.0 - self.d_err_ema_alpha) * self.d_err_filt[1]
        d_err_x = np.clip(self.d_err_filt[0], -1.0, 1.0)
        d_err_y = np.clip(self.d_err_filt[1], -1.0, 1.0)
        self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        # 3. Mesafe Normalizasyonu
        target_dist = float(np.linalg.norm(self.target_pos_3d))
        dist_norm = np.clip((target_dist - self.dist_min_m) / (self.dist_max_m - self.dist_min_m), 0.0, 1.0) * 2.0 - 1.0

        return np.array(
            [
                err_x_norm,
                err_y_norm,
                d_err_x,
                d_err_y,
                dist_norm,
            ],
            dtype=np.float32,
        )

    def _get_obs(self):
        """
        9-Boyutlu Durum Gozlemi:
        Gecikmeli kamera sensor verisi (5 Boyut) + Gecikmesiz guncel eylem gecmisi (4 Boyut).
        Boylece Smith Predictor mantigina uygun olarak eylemler gecikme kuyruguna girmez,
        ajanan yoldaki (in-flight) taze komutlari dogru bir sekilde sunulur.
        """
        delayed_sensor = self.obs_queue[0] if (len(self.obs_queue) > 0) else self._get_sensor_obs()
        return np.concatenate(
            [
                delayed_sensor,
                self.act_hist_1,
                self.act_hist_2,
            ],
            dtype=np.float32,
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.lock_streak = 0
        self.turret_pan = 0.0
        self.turret_tilt = 0.0
        self.actual_pan_step = 0.0
        self.actual_tilt_step = 0.0
        self.cmd_pan_deg = 0.0
        self.cmd_tilt_deg = 0.0
        self.backlash_pan_out = 0.0
        self.backlash_tilt_out = 0.0
        self.prev_e_sq = None
        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.last_action = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)

        # Domain Randomization: 77.541 karelik gercek telemetri verisine dayali olasiliksal dagilim
        if self.enable_randomization:
            # 1. Donanim gecikmesi (%15 hafif yuk 2 kare / %70 nominal 3 kare / %15 yuk 4 kare)
            self.latency_steps = int(self.np_random.choice([2, 3, 4], p=[0.15, 0.70, 0.15]))
            self.obs_queue = deque(maxlen=self.latency_steps + 1)

            # 2. Gercek telemetri FPS dagilimi (%55 nominal 28.5-30 FPS, %35 yuk 22-28.5 FPS, %10 spike 18-22 FPS)
            fps_mode = str(self.np_random.choice(["nominal", "load", "drop"], p=[0.55, 0.35, 0.10]))
            if fps_mode == "nominal":
                self.dt = float(self.np_random.uniform(0.0333, 0.0350))
            elif fps_mode == "load":
                self.dt = float(self.np_random.uniform(0.0350, 0.0455))
            else:
                self.dt = float(self.np_random.uniform(0.0455, 0.0555))

            # 3. Servo dinamigi (olculen 30.0 ms etrafinda)
            self.servo_tau = float(self.np_random.uniform(CONFIG.rl.servo_tau_min_s, CONFIG.rl.servo_tau_max_s))
            self.servo_deadband = float(self.np_random.uniform(0.08, 0.12))

            # 4. Disli boslugu (Gercek donanim olcumu 1.61 deg etrafinda domain randomization)
            base_backlash = float(CONFIG.actuator.backlash_deg)  # 1.61 deg (System ID olcumu)
            self.backlash_deg = float(self.np_random.uniform(base_backlash * 0.85, base_backlash * 1.15))

            self.yolo_noise_std = float(self.np_random.uniform(CONFIG.rl.yolo_noise_std_min, CONFIG.rl.yolo_noise_std_max))
        else:
            self.latency_steps = self.base_latency_steps
            self.obs_queue = deque(maxlen=self.latency_steps + 1)
            self.servo_tau = float(CONFIG.mount.servo_tau)  # 0.0300 s (SSOT olculen zaman sabiti)
            self.servo_deadband = float(CONFIG.actuator.servo_deadband_deg)  # 0.10 deg
            self.dt = self.base_dt
            self.yolo_noise_std = 0.0

        scenarios = ["static", "linear", "zigzag", "evasive", "circular"]
        self.trajectory_type = str(self.np_random.choice(scenarios, p=[0.20, 0.25, 0.25, 0.15, 0.15]))

        start_x = float(self.np_random.uniform(-0.8, 0.8))
        start_y = float(self.np_random.uniform(-0.3, 0.3))
        # Yakin mesafe (0.8 m) avlanma bolgesi dahil 0.8 - 3.2 m genisletilmis dagilim
        start_z = float(self.np_random.uniform(0.8, 3.2))
        self.target_pos_3d = np.array([start_x, start_y, start_z], dtype=np.float32)

        if self.trajectory_type == "static":
            self.target_vel_3d = np.zeros(3, dtype=np.float32)
        elif self.trajectory_type == "linear":
            vx = float(self.np_random.uniform(-0.6, 0.6))
            vy = float(self.np_random.uniform(-0.2, 0.2))
            self.target_vel_3d = np.array([vx, vy, 0.0], dtype=np.float32)
        else:
            self.target_vel_3d = np.array([0.5 * float(self.np_random.choice([-1, 1])), 0.0, 0.0], dtype=np.float32)

        self.phase = float(self.np_random.uniform(0, 2 * math.pi))

        sensor_obs = self._get_sensor_obs()
        self.last_err = np.array(sensor_obs[:2], dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.prev_e_sq = float(sensor_obs[0] ** 2 + sensor_obs[1] ** 2)
        self.obs_queue.clear()
        for _ in range(self.latency_steps + 1):
            self.obs_queue.append(sensor_obs.copy())

        return self._get_obs(), {}

    def _apply_backlash(self, target_deg, last_output_deg):
        """Webots'taki 1.2 derece mekanik disli boslugu modeli."""
        if self.backlash_deg <= 0.0:
            return float(target_deg)
        delta = float(target_deg) - float(last_output_deg)
        if delta > self.backlash_deg:
            return float(target_deg) - self.backlash_deg
        if delta < -self.backlash_deg:
            return float(target_deg) + self.backlash_deg
        return float(last_output_deg)

    def step(self, action):
        self.step_count += 1
        # =====================================================================
        # 1. TEMIZ EYLEM ESLEME (Sarmalayicisiz dogrudan motor adimi)
        # =====================================================================
        cmd_pan_step = float(action[0]) * self.max_step_pan_deg
        cmd_tilt_step = float(action[1]) * self.max_step_tilt_deg
        applied_action = action.copy()

        # =====================================================================
        # 2. GERCEK SERVO AKTÜATÖR FİZİĞİ (Deadband + Asimetrik Eylemsizlik + Motor Lag)
        # =====================================================================
        # A) Servo Olu Bolge Modeli
        db_half = self.servo_deadband * 0.5
        eff_pan_step = math.copysign(max(0.0, abs(cmd_pan_step) - db_half), cmd_pan_step) if abs(cmd_pan_step) > 1e-5 else 0.0
        eff_tilt_step = math.copysign(max(0.0, abs(cmd_tilt_step) - db_half), cmd_tilt_step) if abs(cmd_tilt_step) > 1e-5 else 0.0

        # B) 1. Derece Viskoz Motor Dinamigi (Lag Filtresi)
        alpha_m = self.dt / (self.servo_tau + self.dt)
        self.actual_pan_step = (1.0 - alpha_m) * self.actual_pan_step + alpha_m * eff_pan_step
        self.actual_tilt_step = (1.0 - alpha_m) * self.actual_tilt_step + alpha_m * eff_tilt_step

        # C) Komut biriktirici, mekanik limitler, disli boslugu ve pozisyon
        self.cmd_pan_deg += self.actual_pan_step
        self.cmd_tilt_deg += self.actual_tilt_step
        self.cmd_pan_deg = float(np.clip(self.cmd_pan_deg, -self.max_pan_deg, self.max_pan_deg))
        self.cmd_tilt_deg = float(np.clip(self.cmd_tilt_deg, -self.max_tilt_deg, self.max_tilt_deg))

        # Donanim kuantalama ve surekli taret pozisyonu entegrasyonu (gradyan tikanmasini onler)
        target_pan = self.cmd_pan_deg
        target_tilt = self.cmd_tilt_deg
        if self.enable_randomization and self.servo_deadband > 0:
            q_half = float(self.servo_deadband) * 0.5
            target_pan += float(self.np_random.uniform(-q_half, q_half))
            target_tilt += float(self.np_random.uniform(-q_half, q_half))

        self.turret_pan = float(np.clip(self._apply_backlash(target_pan, self.backlash_pan_out), -self.max_pan_deg, self.max_pan_deg))
        self.turret_tilt = float(np.clip(self._apply_backlash(target_tilt, self.backlash_tilt_out), -self.max_tilt_deg, self.max_tilt_deg))
        self.backlash_pan_out = self.turret_pan
        self.backlash_tilt_out = self.turret_tilt

        # D) Eylem Gecmisi Kaydi (Sahadaki gibi: gercek uygulanan komut saklanir)
        self.act_hist_2 = self.act_hist_1.copy()
        self.act_hist_1 = applied_action.copy()

        # =====================================================================
        # 2. HEDEF KİNEMATİĞİ
        # =====================================================================
        t = self.step_count * self.dt
        if self.trajectory_type == "zigzag":
            self.target_vel_3d[0] = 0.6 * math.cos(2.2 * t + self.phase)
            self.target_vel_3d[1] = 0.2 * math.sin(2.4 * t + self.phase)
        elif self.trajectory_type == "circular":
            self.target_vel_3d[0] = 0.5 * math.cos(1.8 * t + self.phase)
            self.target_vel_3d[1] = 0.3 * math.sin(1.8 * t + self.phase)
        elif self.trajectory_type == "evasive":
            if self.step_count % 35 == 0:
                self.target_vel_3d[0] = float(self.np_random.uniform(-0.7, 0.7))
                self.target_vel_3d[1] = float(self.np_random.uniform(-0.3, 0.3))

        self.target_pos_3d += self.target_vel_3d * self.dt

        if abs(self.target_pos_3d[0]) > 1.8:
            self.target_vel_3d[0] *= -1.0
        if abs(self.target_pos_3d[1]) > 0.8:
            self.target_vel_3d[1] *= -1.0

        # =====================================================================
        # 3. GECIKMELI GOZLEM VE PROGRESS ODULU
        # =====================================================================
        sensor_obs = self._get_sensor_obs()
        self.obs_queue.append(sensor_obs.copy())
        obs = self._get_obs()

        err_x_norm, err_y_norm = float(sensor_obs[0]), float(sensor_obs[1])
        e_norm_sq = err_x_norm**2 + err_y_norm**2
        current_dist = math.sqrt(e_norm_sq)

        progress = 0.0 if self.prev_e_sq is None else (self.prev_e_sq - e_norm_sq)
        self.prev_e_sq = e_norm_sq

        u_norm_sq = float(action[0]**2 + action[1]**2)
        du_norm_sq = float((action[0] - self.last_action[0])**2 + (action[1] - self.last_action[1])**2)

        # 1. MEKANIZMA: Yon Degistirme (Reversal / Flip) Cezasi
        reversal_pen = sum(
            0.08 * abs(float(action[idx] - self.last_action[idx]))
            for idx in (0, 1)
            if float(action[idx]) * float(self.last_action[idx]) < -1e-3
        )
        self.last_action = action.copy()

        # 2. MEKANIZMA: Merkezde / Kilit Yakininda Ivme & Efor Olceklemesi
        near_lock_scale = 1.8 if current_dist < 0.08 else 1.0

        # 1. Takip ilerlemesi (Progress)
        r_tracking = self.progress_gain * float(np.clip(progress, -2.0, 2.0))
        # 2. Kalici durum hatasi cezasi (L2)
        r_error = -self.error_gain * e_norm_sq
        # 3. Kilit yakini olcekli efor ve jerk cezasi + reversal bariyeri
        r_effort = -self.lambda_u * near_lock_scale * u_norm_sq
        r_jerk = -self.mu_du * near_lock_scale * du_norm_sq - reversal_pen
        # 4. Sakinlik sartli Gauss hedef kilit ve hayatta kalma odulu
        calm_factor = float(math.exp(-3.0 * (u_norm_sq + du_norm_sq)))
        r_alive = 0.15 * float(math.exp(-2.5 * e_norm_sq))
        r_bullseye = (float(CONFIG.rl.bullseye_reward) * calm_factor if current_dist < float(CONFIG.rl.bullseye_threshold_norm) else 0.0) + r_alive

        # Kilit serisi (Lock Streak) sayaci (metrik/telemetri icin guncellenir)
        seri_esik = 0.030
        if current_dist < seri_esik:
            self.lock_streak += 1
        else:
            self.lock_streak = 0

        reward = float(r_tracking + r_error + r_effort + r_jerk + r_bullseye)

        terminated = False
        truncated = self.step_count >= self.max_steps

        # Hedef kadrajdan tamamen kacarsa bolum sonlanir
        if current_dist > 1.4:
            terminated = False
            reward -= 0.50

        return obs, reward, terminated, truncated, {"pixel_dist": current_dist, "lock_streak": self.lock_streak}

    def render(self):
        """Gorsel test modu (human veya rgb_array) icin 2B taktiksel kadraj cizer."""
        if self.render_mode not in ("human", "rgb_array"):
            return None
        import cv2

        w, h = int(self.screen_w), int(self.screen_h)
        cx, cy = int(self.center_x), int(self.center_y)
        canvas = np.zeros((h, w, 3), dtype=np.uint8)

        cv2.line(canvas, (cx - 40, cy), (cx + 40, cy), (80, 180, 80), 1)
        cv2.line(canvas, (cx, cy - 40), (cx, cy + 40), (80, 180, 80), 1)
        cv2.circle(canvas, (cx, cy), 30, (0, 200, 0), 1)

        tx = int(np.clip(self.last_pixel_pos[0], 0, w - 1))
        ty = int(np.clip(self.last_pixel_pos[1], 0, h - 1))
        cv2.circle(canvas, (tx, ty), 14, (0, 0, 255), -1)

        info_text = (
            f"Step: {self.step_count:03d} | Trajectory: {self.trajectory_type} | "
            f"Pan: {self.turret_pan:+.1f} deg | Tilt: {self.turret_tilt:+.1f} deg"
        )
        cv2.putText(canvas, info_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220, 220, 220), 2)

        if self.render_mode == "human":
            cv2.imshow("TurretEnv RL Simulation", canvas)
            return None
        return canvas[:, :, ::-1]

    def close(self):
        if self.render_mode == "human":
            import cv2
            cv2.destroyAllWindows()

