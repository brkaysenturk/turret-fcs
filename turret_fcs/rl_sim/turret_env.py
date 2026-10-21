"""
================================================================================
MODUL: GERCEK DONANIM FIZIKLI RL BENZETIM ORTAMI (TURRET ENV / GYMNASIUM)
================================================================================
Bu modul, gercek taret donaniminin olculmus fiziksel kisitlarini:
1. Donanim tasima gecikmesi (Transport Delay Buffer: 33-100 ms / 1-3 kare rastgelelestirme)
2. Gercek servo olu bolgesi (Deadband: Pan +-0.90 deg, Tilt +-0.95 deg)
3. Kare basi maksimum adim doyumu (Max Slew Limit: +-2.75 deg/kare)
4. 1. Derece viskoz motor gecikmesi (Actuator Time Constant: tau = 35-70 ms)
5. Logitech C270 gercek optik modeli (dFOV 55 deg -> H_FOV = 48.8 deg, V_FOV = 28.6 deg)
6. Domain Randomization (YOLO gurultusu, degisken gecikme, oransal FPS dalgalanmasi)
7. Eylem Gecmisi (Action History - 2 adimlik boru hatti farkindaligi)
8. LQR / Bryson Tabanli Kuadratik Odul: r = -(e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2)
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
        latency_steps=2,
        lambda_u=0.04,
        mu_du=0.03,
        enable_randomization=True,
        **kwargs,
    ):
        super().__init__()
        self.render_mode = render_mode
        self.base_dt = float(dt)
        self.base_latency_steps = int(latency_steps)
        self.latency_steps = self.base_latency_steps
        self.obs_queue = deque(maxlen=self.latency_steps + 1)
        self.enable_randomization = enable_randomization

        # LQR / Bryson Tabanli Odul Katsayilari:
        # r = -(e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2)
        self.lambda_u = float(kwargs.get("lambda_u", lambda_u))
        self.mu_du = float(kwargs.get("mu_du", mu_du))

        # 1. Logitech C270 Gercek Donanim Optik Parametreleri (SSOT)
        self.screen_w = float(CONFIG.optics.default_width)
        self.screen_h = float(CONFIG.optics.default_height)
        self.center_x = self.screen_w / 2.0
        self.center_y = self.screen_h / 2.0
        self.h_fov = CONFIG.optics.h_fov_deg
        self.v_fov = CONFIG.optics.v_fov_deg
        self.fx = CONFIG.optics.compute_fx(int(self.screen_w))
        self.fy = CONFIG.optics.compute_fy(int(self.screen_h))

        # 2. Gercek Donanim Mekanik ve Servo Sinirlari (SSOT)
        self.max_pan_deg = CONFIG.mount.pan_limit_max
        self.max_tilt_deg = CONFIG.mount.tilt_limit_max
        self.max_step_pan_deg = CONFIG.actuator.max_step_pan_deg
        self.max_step_tilt_deg = CONFIG.actuator.max_step_tilt_deg
        self.max_step_deg = self.max_step_pan_deg
        self.servo_deadband = CONFIG.actuator.servo_deadband_deg
        self.servo_tau = CONFIG.mount.servo_tau
        self.yolo_noise_std = 3.5        # YOLO merkez gurultusu (piksel)

        # 3. Bryson Kurali ile Normalizasyon Olcekleri
        self.v_max_px_s = 300.0          # Maksimum hedef piksel hizi (px/s)
        self.dist_min_m = 0.5            # TF-Luna minimum menzil (m)
        self.dist_max_m = 5.0            # TF-Luna maksimum calisma menzili (m)

        # Eylem Uzayı: [-1.0, 1.0] araliginda normalize motor adimi
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        # 11 Boyutlu Gozlem Uzayi (Bryson Kuraliyla [-1.0, 1.0] araliginda)
        # [err_x, err_y, d_err_x, d_err_y, vx, vy, dist, act_pan_t1, act_tilt_t1, act_pan_t2, act_tilt_t2]
        self.observation_space = spaces.Box(
            low=-np.ones(11, dtype=np.float32),
            high=np.ones(11, dtype=np.float32),
            dtype=np.float32,
        )

        # Durum Degiskenleri
        self.turret_pan = 0.0
        self.turret_tilt = 0.0
        self.actual_pan_step = 0.0
        self.actual_tilt_step = 0.0
        self.target_pos_3d = np.zeros(3, dtype=np.float32)
        self.target_vel_3d = np.zeros(3, dtype=np.float32)
        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.last_action = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.last_dist_val = 1.0
        self.last_pixel_pos = (self.center_x, self.center_y)
        self.trajectory_type = "linear"
        self.step_count = 0
        self.max_steps = 300
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

        # Domain Randomization: YOLO tespit kutusu titresimi
        noise_x = float(np.random.normal(0.0, self.yolo_noise_std)) if (self.enable_randomization and self.yolo_noise_std > 0) else 0.0
        noise_y = float(np.random.normal(0.0, self.yolo_noise_std)) if (self.enable_randomization and self.yolo_noise_std > 0) else 0.0
        px_noisy = px + noise_x
        py_noisy = py + noise_y
        self.last_pixel_pos = (float(px_noisy), float(py_noisy))

        # 1. Hata (Ekran yaricapina gore Bryson normalizasyonu)
        err_x_norm = np.clip((px_noisy - self.center_x) / (self.screen_w / 2.0), -1.5, 1.5)
        err_y_norm = np.clip((py_noisy - self.center_y) / (self.screen_h / 2.0), -1.5, 1.5)

        # 2. Hata Turevi (Dusuk geciren filtreli / EMA smoothed - gurultuden arindirilmis)
        raw_d_err_x = (err_x_norm - self.last_err[0]) / max(self.dt, 1e-4) * 0.05
        raw_d_err_y = (err_y_norm - self.last_err[1]) / max(self.dt, 1e-4) * 0.05
        self.d_err_filt[0] = 0.40 * raw_d_err_x + 0.60 * self.d_err_filt[0]
        self.d_err_filt[1] = 0.40 * raw_d_err_y + 0.60 * self.d_err_filt[1]
        d_err_x = np.clip(self.d_err_filt[0], -1.0, 1.0)
        d_err_y = np.clip(self.d_err_filt[1], -1.0, 1.0)
        self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        # 3. Hedef Piksel Hizi (3B hizdan piksele donusum ve Bryson normalizasyonu)
        target_dist = float(np.linalg.norm(self.target_pos_3d))
        target_dist_safe = max(0.5, target_dist)
        vx_px = (self.target_vel_3d[0] / target_dist_safe) * self.fx
        vy_px = -(self.target_vel_3d[1] / target_dist_safe) * self.fy

        vx_norm = np.clip(vx_px / self.v_max_px_s, -1.0, 1.0)
        vy_norm = np.clip(vy_px / self.v_max_px_s, -1.0, 1.0)

        # 4. Mesafe Normalizasyonu
        dist_norm = np.clip((target_dist - self.dist_min_m) / (self.dist_max_m - self.dist_min_m), 0.0, 1.0) * 2.0 - 1.0

        return np.array(
            [
                err_x_norm,
                err_y_norm,
                d_err_x,
                d_err_y,
                vx_norm,
                vy_norm,
                dist_norm,
            ],
            dtype=np.float32,
        )

    def _get_obs(self):
        """
        11-Boyutlu Durum Gozlemi:
        Gecikmeli kamera sensor verisi (7 Boyut) + Gecikmesiz guncel eylem gecmisi (4 Boyut).
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
        self.turret_pan = 0.0
        self.turret_tilt = 0.0
        self.actual_pan_step = 0.0
        self.actual_tilt_step = 0.0
        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.last_action = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.last_dist_val = 1.0

        # Domain Randomization: Bolum basi fiziksel parametre dagilimi
        if self.enable_randomization:
            self.latency_steps = int(np.random.choice([1, 2, 3]))  # 33 ms - 100 ms degisken donanim gecikmesi
            self.obs_queue = deque(maxlen=self.latency_steps + 1)
            self.servo_tau = float(np.random.uniform(0.035, 0.065))  # Servo gecikmesi (35 - 65 ms)
            self.servo_deadband = float(np.random.uniform(0.08, 0.14)) # STM32 0.1 deg kuantalamasi
            self.dt = float(np.random.uniform(0.028, 0.038))         # 26 - 36 FPS dalgalanmasi
            self.yolo_noise_std = float(np.random.uniform(2.0, 5.0))  # YOLO kutu titremesi (piksel)
        else:
            self.latency_steps = self.base_latency_steps
            self.obs_queue = deque(maxlen=self.latency_steps + 1)
            self.servo_tau = 0.045
            self.servo_deadband = 0.10
            self.dt = self.base_dt
            self.yolo_noise_std = 0.0

        scenarios = ["static", "linear", "zigzag", "evasive", "circular"]
        self.trajectory_type = np.random.choice(scenarios, p=[0.20, 0.25, 0.25, 0.15, 0.15])

        start_x = np.random.uniform(-0.8, 0.8)
        start_y = np.random.uniform(-0.3, 0.3)
        start_z = np.random.uniform(1.8, 3.2)
        self.target_pos_3d = np.array([start_x, start_y, start_z], dtype=np.float32)

        if self.trajectory_type == "static":
            self.target_vel_3d = np.zeros(3, dtype=np.float32)
        elif self.trajectory_type == "linear":
            vx = np.random.uniform(-0.6, 0.6)
            vy = np.random.uniform(-0.2, 0.2)
            self.target_vel_3d = np.array([vx, vy, 0.0], dtype=np.float32)
        else:
            self.target_vel_3d = np.array([0.5 * np.random.choice([-1, 1]), 0.0, 0.0], dtype=np.float32)

        self.phase = np.random.uniform(0, 2 * math.pi)

        sensor_obs = self._get_sensor_obs()
        self.obs_queue.clear()
        for _ in range(self.latency_steps + 1):
            self.obs_queue.append(sensor_obs.copy())

        return self._get_obs(), {}

    def step(self, action):
        self.step_count += 1
        action = np.clip(action, -1.0, 1.0)

        # =====================================================================
        # 1. GERCEK SERVO AKTÜATÖR FİZİĞİ (Deadband + Asimetrik Eylemsizlik + Motor Lag)
        # =====================================================================
        cmd_pan_step = float(action[0]) * self.max_step_pan_deg
        cmd_tilt_step = float(action[1]) * self.max_step_tilt_deg

        # A) STM32 Kuantalama ve Servo Olu Bolge Modeli (Yumusak Gecisli - Zero Jerk)
        if abs(cmd_pan_step) < self.servo_deadband:
            eff_pan_step = 0.0
        else:
            eff_pan_step = math.copysign(abs(cmd_pan_step) - self.servo_deadband * 0.5, cmd_pan_step)

        if abs(cmd_tilt_step) < self.servo_deadband:
            eff_tilt_step = 0.0
        else:
            eff_tilt_step = math.copysign(abs(cmd_tilt_step) - self.servo_deadband * 0.5, cmd_tilt_step)

        # B) 1. Derece Viskoz Motor Dinamigi (Lag Filtresi)
        alpha_m = self.dt / (self.servo_tau + self.dt)
        self.actual_pan_step = (1.0 - alpha_m) * self.actual_pan_step + alpha_m * eff_pan_step
        self.actual_tilt_step = (1.0 - alpha_m) * self.actual_tilt_step + alpha_m * eff_tilt_step

        # C) Mekanik Limitler ve Pozisyon Entegrasyonu
        self.turret_pan = float(np.clip(self.turret_pan + self.actual_pan_step, -self.max_pan_deg, self.max_pan_deg))
        self.turret_tilt = float(np.clip(self.turret_tilt + self.actual_tilt_step, -self.max_tilt_deg, self.max_tilt_deg))

        # D) Eylem Gecmisi Kaydi (Son 2 adimlik pipeline hafizasi)
        self.act_hist_2 = self.act_hist_1.copy()
        self.act_hist_1 = action.copy()

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
                self.target_vel_3d[0] = np.random.uniform(-0.7, 0.7)
                self.target_vel_3d[1] = np.random.uniform(-0.3, 0.3)

        self.target_pos_3d += self.target_vel_3d * self.dt

        if abs(self.target_pos_3d[0]) > 1.8:
            self.target_vel_3d[0] *= -1.0
        if abs(self.target_pos_3d[1]) > 0.8:
            self.target_vel_3d[1] *= -1.0

        # =====================================================================
        # 3. GECİKMELİ GÖZLEM VE LQR / BRYSON ÖDÜL FONKSİYONU
        # =====================================================================
        sensor_obs = self._get_sensor_obs()
        self.obs_queue.append(sensor_obs.copy())
        obs = self._get_obs()

        err_x_norm, err_y_norm = float(sensor_obs[0]), float(sensor_obs[1])
        e_norm_sq = err_x_norm**2 + err_y_norm**2
        current_dist = math.sqrt(e_norm_sq)

        # LQR Kuadratik Maliyet: r = -(e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2)
        u_norm_sq = float(action[0]**2 + action[1]**2)
        du_norm_sq = float((action[0] - self.last_action[0])**2 + (action[1] - self.last_action[1])**2)
        self.last_action = action.copy()

        r_tracking = - e_norm_sq
        r_effort = - self.lambda_u * u_norm_sq
        r_jerk = - self.mu_du * du_norm_sq

        # Hedef vizor merkezindeyse (%5 alan) pozitif kilit tesviki
        r_bullseye = 0.50 if current_dist < 0.05 else 0.0

        reward = float(r_tracking + r_effort + r_jerk + r_bullseye)

        terminated = False
        truncated = self.step_count >= self.max_steps

        # Hedef kadrajdan tamamen kacarsa bolum sonlanir
        if current_dist > 1.4:
            terminated = True
            reward -= 2.0

        return obs, reward, terminated, truncated, {"pixel_dist": current_dist}
