"""
================================================================================
MODUL: GERCEK DONANIM FIZIKLI RL BENZETIM ORTAMI (TURRET ENV / GYMNASIUM)
================================================================================
Bu modul, gercek taret donaniminin olculmus fiziksel kisitlarini:
1. Donanim tasima gecikmesi (Transport Delay Buffer: 33-100 ms / 1-3 kare rastgelelestirme)
2. Gercek servo olu bolgesi (Deadband: Pan +-0.90 deg, Tilt +-0.95 deg)
3. Kare basi maksimum adim doyumu (Max Slew Limit: +-2.75 deg/kare)
4. 1. Derece viskoz motor gecikmesi (Actuator Time Constant: tau = 35-70 ms)
5. Logitech C270 gercek optik parametreleri (H_FOV = 53.0 deg, V_FOV = 31.5 deg)
6. Domain Randomization (YOLO gurultusu, degisken gecikme, oransal FPS dalgalanmasi)
7. Eylem Gecmisi (Action History - 2 adimlik boru hatti farkindaligi)
8. Sakinlik Sartli Odul Fonksiyonu (Merkezde titresim ve savrulmayi onleyen ceza)
birebir iceren ve Bryson Kurali ile normalize edilmis 11 boyutlu Gymnasium ortamidir.
================================================================================
"""

from collections import deque
import math
import gymnasium as gym
from gymnasium import spaces
import numpy as np


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
        w_dist=3.5,
        w_dist_sq=5.0,
        w_bullseye=6.0,
        w_lock=4.0,
        w_smooth=0.75,
        enable_randomization=True,
    ):
        super().__init__()
        self.render_mode = render_mode
        self.base_dt = float(dt)
        self.base_latency_steps = int(latency_steps)
        self.latency_steps = self.base_latency_steps
        self.obs_queue = deque(maxlen=self.latency_steps + 1)
        self.enable_randomization = enable_randomization

        # Odul Fonksiyonu Agirliklari
        self.w_dist = float(w_dist)
        self.w_dist_sq = float(w_dist_sq)
        self.w_bullseye = float(w_bullseye)
        self.w_lock = float(w_lock)
        self.w_smooth = float(w_smooth)

        # 1. Logitech C270 Gercek Donanim Optik Parametreleri (1280x720)
        self.screen_w = 1280.0
        self.screen_h = 720.0
        self.center_x = self.screen_w / 2.0
        self.center_y = self.screen_h / 2.0
        self.h_fov = 53.0
        self.v_fov = 31.5
        self.fx = (self.screen_w / 2.0) / math.tan(math.radians(self.h_fov / 2.0))
        self.fy = (self.screen_h / 2.0) / math.tan(math.radians(self.v_fov / 2.0))

        # 2. Gercek Donanim Mekanik ve Servo Sinirlari
        self.max_pan_deg = 45.0          # Guvenli mekanik Pan siniri (+-45 deg)
        self.max_tilt_deg = 18.0         # Guvenli mekanik Tilt siniri (+-18 deg)
        self.max_step_pan_deg = 0.55     # Kare basi maksimum Pan adimi (+-0.55 deg/kare = 16.5 deg/s)
        self.max_step_tilt_deg = 0.22    # Dikey agir volan namlu adimi (+-0.22 deg/kare = 6.6 deg/s)
        self.max_step_deg = self.max_step_pan_deg
        self.servo_deadband = 0.10       # STM32 0.1 deg UART kuantalama ve servo mekanik olu bolgesi
        self.servo_tau = 0.045           # 1. derece motor zaman sabiti (45 ms)
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
        # 3. GECİKMELİ GÖZLEM VE SAKİNLİK ŞARTLI BRYSON ÖDÜL FONKSİYONU
        # =====================================================================
        sensor_obs = self._get_sensor_obs()
        self.obs_queue.append(sensor_obs.copy())
        obs = self._get_obs()

        err_x_norm, err_y_norm = sensor_obs[0], sensor_obs[1]
        dist_sq = err_x_norm**2 + err_y_norm**2
        current_dist = math.sqrt(dist_sq)

        # Bryson Kurali Tabanli Odul:
        # A) Mesafe Cezasi (Kuadratik ve dogrusal)
        r_dist = - self.w_dist * current_dist - self.w_dist_sq * dist_sq

        # B) Sakinlik / Hız Faktoru: Merkezde savrulmadan durabilme yetenegi
        action_mag = float(np.hypot(action[0], action[1]))
        calm_factor = math.exp(- 4.0 * (action_mag**2))

        # C) Hedef Vizor Merkezindeyken Gauss Kilit Bonusu (Sakinlikle carpanli)
        r_bullseye = self.w_bullseye * math.exp(- 20.0 * dist_sq) * (0.30 + 0.70 * calm_factor)

        # D) Kilit Kararlilik Odulu: Hedef vizor merkezinde (%4 alan) VE motor sakinse buyuk odul
        is_calm_lock = (current_dist < 0.04) and (action_mag < 0.20)
        r_lock = self.w_lock if is_calm_lock else (self.w_lock * 0.25 if current_dist < 0.04 else 0.0)

        # E) Yaklasma Ilerleme Odulu (Asiri hiz tesvikini onlemek icin olculu)
        r_progress = 3.5 * (self.last_dist_val - current_dist)
        self.last_dist_val = current_dist

        # F) Eylem Sarsinti / Jerk Cezasi (Motor titresimini sifirlar)
        action_diff = np.sum((action - self.last_action) ** 2)
        r_smooth = - self.w_smooth * action_diff
        self.last_action = action.copy()

        # G) Surekli Eylem Buyuklugu Cezasi (L2 Action Regularization - Asiri komut basmayi onler)
        r_act_reg = - 0.70 * (action[0]**2 + action[1]**2)

        # H) Merkezde gereksiz aksiyon alma cezasi (Hareketsiz sakin kilit tesviki)
        r_center_still = - 2.5 * action_mag if current_dist < 0.06 else 0.0

        # I) Kamera Hareket Bulanikligi (Motion Blur) Cezasi (Hizli savrulmada goruntu bozulur)
        step_mag = math.hypot(cmd_pan_step, cmd_tilt_step)
        r_blur = - 2.0 * (step_mag - 0.35) if step_mag > 0.35 else 0.0

        reward = r_dist + r_bullseye + r_lock + r_progress + r_smooth + r_act_reg + r_center_still + r_blur

        terminated = False
        truncated = self.step_count >= self.max_steps

        # Hedef ekrandan tamamen kacarsa bolum sonlanir
        if current_dist > 1.4:
            terminated = True
            reward -= 6.0

        return obs, reward, terminated, truncated, {"pixel_dist": current_dist}
