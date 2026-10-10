from collections import deque
import math
import cv2
import gymnasium as gym
from gymnasium import spaces
import numpy as np


class TurretEnv(gym.Env):
    """
    Turret FCS - Gercek Dunya Gecikmesi (Transport Delay Buffer) ve 
    Sarsinti Onleme (Action Smoothness Penalty) ile Guclendirilmis RL Ortami.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(self, render_mode=None, dt=0.016, latency_steps=2):
        super().__init__()
        self.render_mode = render_mode
        self.base_dt = dt
        self.latency_steps = latency_steps  # ~60-80 ms Donanim Gecikmesi (Kamera + YOLO + UART)
        self.obs_queue = deque(maxlen=self.latency_steps + 1)

        # Optik Parametreler (Webots C270)
        self.screen_w = 640
        self.screen_h = 360
        self.center_x = self.screen_w / 2.0
        self.center_y = self.screen_h / 2.0
        self.h_fov = 53.0
        self.v_fov = 31.5
        self.fx = (self.screen_w / 2.0) / math.tan(math.radians(self.h_fov / 2.0))
        self.fy = (self.screen_h / 2.0) / math.tan(math.radians(self.v_fov / 2.0))

        # Servo Limitleri
        self.max_pan_deg = 90.0
        self.max_tilt_deg = 45.0
        self.max_vel_deg_s = 140.0

        # Eylem Uzayı
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        # 9 Boyutlu Gözlem Uzayı
        self.observation_space = spaces.Box(
            low=-np.ones(9, dtype=np.float32),
            high=np.ones(9, dtype=np.float32),
            dtype=np.float32,
        )

        # Durum Değişkenleri
        self.turret_pan = 0.0
        self.turret_tilt = 0.0
        self.pan_vel = 0.0
        self.tilt_vel = 0.0
        self.target_pos_3d = np.zeros(3, dtype=np.float32)
        self.target_vel_3d = np.zeros(3, dtype=np.float32)
        self.last_err = np.zeros(2, dtype=np.float32)
        self.last_dist_val = 1.0
        self.last_action = np.zeros(2, dtype=np.float32)
        self.trajectory_type = "linear"
        self.step_count = 0
        self.max_steps = 450
        self.dt = self.base_dt

    def _get_obs(self):
        target_azimuth = math.degrees(math.atan2(self.target_pos_3d[0], self.target_pos_3d[2]))
        target_elevation = math.degrees(math.atan2(self.target_pos_3d[1], self.target_pos_3d[2]))

        rel_pan = target_azimuth - self.turret_pan
        rel_tilt = target_elevation - self.turret_tilt

        px = self.center_x + math.tan(math.radians(rel_pan)) * self.fx
        py = self.center_y - math.tan(math.radians(rel_tilt)) * self.fy

        err_x_norm = np.clip((px - self.center_x) / (self.screen_w / 2.0), -1.5, 1.5)
        err_y_norm = np.clip((py - self.center_y) / (self.screen_h / 2.0), -1.5, 1.5)

        # Hata türevi
        d_err_x = np.clip((err_x_norm - self.last_err[0]) / max(self.dt, 1e-4) * 0.08, -1.0, 1.0)
        d_err_y = np.clip((err_y_norm - self.last_err[1]) / max(self.dt, 1e-4) * 0.08, -1.0, 1.0)
        self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        vx_norm = np.clip(self.target_vel_3d[0] / 2.0, -1.0, 1.0)
        vy_norm = np.clip(self.target_vel_3d[1] / 2.0, -1.0, 1.0)

        dist = float(np.linalg.norm(self.target_pos_3d))
        dist_norm = np.clip((dist - 0.5) / 7.5, 0.0, 1.0) * 2.0 - 1.0

        p_vel_norm = np.clip(self.pan_vel / self.max_vel_deg_s, -1.0, 1.0)
        t_vel_norm = np.clip(self.tilt_vel / self.max_vel_deg_s, -1.0, 1.0)

        self.last_pixel_pos = (px, py)
        self.last_dist = dist

        return np.array(
            [
                err_x_norm,
                err_y_norm,
                d_err_x,
                d_err_y,
                vx_norm,
                vy_norm,
                dist_norm,
                p_vel_norm,
                t_vel_norm,
            ],
            dtype=np.float32,
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.turret_pan = 0.0
        self.turret_tilt = 0.0
        self.pan_vel = 0.0
        self.tilt_vel = 0.0
        self.last_err = np.zeros(2, dtype=np.float32)
        self.last_dist_val = 1.0
        self.last_action = np.zeros(2, dtype=np.float32)
        self.dt = np.random.uniform(0.016, 0.024)

        scenarios = ["static", "linear", "zigzag", "evasive", "circular"]
        self.trajectory_type = np.random.choice(scenarios, p=[0.15, 0.25, 0.30, 0.15, 0.15])

        start_x = np.random.uniform(-1.0, 1.0)
        start_y = np.random.uniform(-0.4, 0.4)
        start_z = np.random.uniform(1.8, 3.5)
        self.target_pos_3d = np.array([start_x, start_y, start_z], dtype=np.float32)

        if self.trajectory_type == "static":
            self.target_vel_3d = np.zeros(3, dtype=np.float32)
        elif self.trajectory_type == "linear":
            vx = np.random.uniform(-0.9, 0.9)
            vy = np.random.uniform(-0.25, 0.25)
            self.target_vel_3d = np.array([vx, vy, 0.0], dtype=np.float32)
        else:
            self.target_vel_3d = np.array([0.8 * np.random.choice([-1, 1]), 0.0, 0.0], dtype=np.float32)

        self.phase = np.random.uniform(0, 2 * math.pi)

        # Gecikme tamponunu ilklendir
        obs = self._get_obs()
        self.obs_queue.clear()
        for _ in range(self.latency_steps + 1):
            self.obs_queue.append(obs.copy())

        return self.obs_queue[0], {}

    def step(self, action):
        self.step_count += 1
        action = np.clip(action, -1.0, 1.0)

        # 1. Motor Dinamiği
        target_pan_vel = float(action[0] * self.max_vel_deg_s)
        target_tilt_vel = float(action[1] * self.max_vel_deg_s)

        self.pan_vel = 0.50 * self.pan_vel + 0.50 * target_pan_vel
        self.tilt_vel = 0.50 * self.tilt_vel + 0.50 * target_tilt_vel

        self.turret_pan = np.clip(self.turret_pan + self.pan_vel * self.dt, -self.max_pan_deg, self.max_pan_deg)
        self.turret_tilt = np.clip(self.turret_tilt + self.tilt_vel * self.dt, -self.max_tilt_deg, self.max_tilt_deg)

        # 2. Hedef Hareketi
        t = self.step_count * self.dt
        if self.trajectory_type == "zigzag":
            self.target_vel_3d[0] = 1.0 * math.cos(2.4 * t + self.phase)
            self.target_vel_3d[1] = 0.35 * math.sin(2.6 * t + self.phase)
        elif self.trajectory_type == "circular":
            self.target_vel_3d[0] = 0.8 * math.cos(2.0 * t + self.phase)
            self.target_vel_3d[1] = 0.5 * math.sin(2.0 * t + self.phase)
        elif self.trajectory_type == "evasive":
            if self.step_count % 30 == 0:
                self.target_vel_3d[0] = np.random.uniform(-1.2, 1.2)
                self.target_vel_3d[1] = np.random.uniform(-0.5, 0.5)

        self.target_pos_3d += self.target_vel_3d * self.dt

        if abs(self.target_pos_3d[0]) > 2.0:
            self.target_vel_3d[0] *= -1.0
        if abs(self.target_pos_3d[1]) > 1.0:
            self.target_vel_3d[1] *= -1.0

        # 3. Gercek Durum ve Gecikmeli Gozlem
        real_obs = self._get_obs()
        self.obs_queue.append(real_obs.copy())
        delayed_obs = self.obs_queue[0]  # Ajana 2-3 kare gecikmeli gercek dunya gozlemi sunulur

        err_x_norm, err_y_norm = real_obs[0], real_obs[1]
        dist_sq = err_x_norm**2 + err_y_norm**2
        current_dist = math.sqrt(dist_sq)

        # Odul Hesaplamasi
        r_dist_penalty = -4.0 * current_dist - 6.0 * dist_sq
        r_bullseye = 5.0 * math.exp(-12.0 * dist_sq)
        r_lock = 3.0 if current_dist < 0.04 else 0.0

        delta_dist = self.last_dist_val - current_dist
        r_progress = 12.0 * delta_dist
        self.last_dist_val = current_dist

        # Sarsinti ve Titreme Cezasi (Action Jerk Penalty) -> Donanim motorlarini korur
        action_diff = np.sum((action - self.last_action) ** 2)
        r_smooth = -0.15 * action_diff

        reward = r_dist_penalty + r_bullseye + r_lock + r_progress + r_smooth
        self.last_action = action.copy()

        terminated = False
        truncated = self.step_count >= self.max_steps

        if current_dist > 1.5:
            terminated = True
            reward -= 5.0

        return delayed_obs, reward, terminated, truncated, {"pixel_dist": current_dist}

    def render(self):
        if self.render_mode is None:
            return

        frame = np.zeros((self.screen_h, self.screen_w, 3), dtype=np.uint8)
        frame[:] = (20, 24, 28)

        cx, cy = int(self.center_x), int(self.center_y)
        tx, ty = int(self.last_pixel_pos[0]), int(self.last_pixel_pos[1])

        cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 30, 2)
        cv2.circle(frame, (cx, cy), 35, (0, 255, 255), 1)

        if 0 <= tx < self.screen_w and 0 <= ty < self.screen_h:
            is_locked = math.hypot(tx - cx, ty - cy) < 25
            color = (0, 255, 0) if is_locked else (0, 140, 255)
            cv2.circle(frame, (tx, ty), 18, color, 2)
            cv2.line(frame, (cx, cy), (tx, ty), (60, 60, 180), 1)

        if self.render_mode == "human":
            cv2.imshow("Turret FCS - Aggressive RL Simulation", frame)
            cv2.waitKey(int(self.dt * 1000))

    def close(self):
        cv2.destroyAllWindows()
