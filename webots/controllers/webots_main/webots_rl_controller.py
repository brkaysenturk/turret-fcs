from pathlib import Path
import numpy as np
from stable_baselines3 import PPO


class WebotsRLController:
    """
    Webots simulasyon ortami icin PPO tabanli (11-Boyutlu Bryson durum uzayi)
    ve gercek donanim modeliyle tam uyumlu hedef takip denetleyicisi.
    """

    def __init__(self, model_path=None):
        if model_path is None:
            base_dir = Path(__file__).resolve().parents[3]
            model_path = base_dir / "turret_fcs" / "rl_sim" / "models" / "turret_ppo_final.zip"

        self.model_path = Path(model_path)
        self.model = None
        self.is_loaded = False
        self.load_model()

        # Gercek Donanim Mekanik ve Bryson Olcekleri
        self.max_step_deg = 2.75         # Kare basi maksimum motor adimi (+-2.75 deg)
        self.v_max_px_s = 300.0          # Maksimum hedef piksel hizi (px/s)
        self.dist_min_m = 0.5
        self.dist_max_m = 5.0

        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)

    def load_model(self):
        """Egitilmis PPO zip modelini yukler."""
        try:
            if self.model_path.exists():
                self.model = PPO.load(str(self.model_path), device="cpu")
                self.is_loaded = True
            else:
                self.is_loaded = False
        except Exception:
            self.is_loaded = False

    def reset(self):
        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)

    def predict_angles(self, bx, by, center_x, center_y, screen_w, screen_h, vx, vy, dist_m, curr_pan_deg, curr_tilt_deg, dt):
        """
        11 boyutlu Bryson durum vektorunu olusturur ve motora iletilecek aci komutunu uretir.
        """
        if not self.is_loaded or self.model is None:
            return 0.0, 0.0

        dt_safe = max(0.005, float(dt))

        # 1. Hata ve Gurultuden Arindirilmis Filtreli Hata Turevi (EMA Smoothed)
        err_x_norm = np.clip((bx - center_x) / (screen_w / 2.0), -1.5, 1.5)
        err_y_norm = np.clip((by - center_y) / (screen_h / 2.0), -1.5, 1.5)

        raw_d_err_x = (err_x_norm - self.last_err[0]) / dt_safe * 0.05
        raw_d_err_y = (err_y_norm - self.last_err[1]) / dt_safe * 0.05
        self.d_err_filt[0] = 0.40 * raw_d_err_x + 0.60 * self.d_err_filt[0]
        self.d_err_filt[1] = 0.40 * raw_d_err_y + 0.60 * self.d_err_filt[1]
        d_err_x = np.clip(self.d_err_filt[0], -1.0, 1.0)
        d_err_y = np.clip(self.d_err_filt[1], -1.0, 1.0)
        self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        # 2. Kalman Hedef Hizi (Piksel/saniye -> Bryson Normalizasyonu: V_max = 300 px/s)
        vx_norm = np.clip(float(vx) / self.v_max_px_s, -1.0, 1.0)
        vy_norm = np.clip(float(vy) / self.v_max_px_s, -1.0, 1.0)

        # 3. TF-Luna Mesafe Normalizasyonu (0.5m - 5.0m)
        dist_val = dist_m if (dist_m is not None and dist_m > 0.2) else 2.0
        dist_norm = np.clip((dist_val - self.dist_min_m) / (self.dist_max_m - self.dist_min_m), 0.0, 1.0) * 2.0 - 1.0

        # 4. 11 Boyutlu Bryson Durum Vektoru (Eylem gecmisi dahil)
        obs = np.array(
            [
                err_x_norm,
                err_y_norm,
                d_err_x,
                d_err_y,
                vx_norm,
                vy_norm,
                dist_norm,
                self.act_hist_1[0],
                self.act_hist_1[1],
                self.act_hist_2[0],
                self.act_hist_2[1],
            ],
            dtype=np.float32,
        )

        action, _ = self.model.predict(obs, deterministic=True)

        # 5. Eylem Gecmisi Guncellemesi (Boru hatti hafizasi - Smith Predictor)
        self.act_hist_2 = self.act_hist_1.copy()
        self.act_hist_1 = np.array([action[0], action[1]], dtype=np.float32)

        # 6. Eylemden Fiziksel Motor Adimina Donusum (+-2.75 deg/kare doyumlu)
        d_pan_deg = float(np.clip(action[0] * self.max_step_deg, -self.max_step_deg, self.max_step_deg))
        d_tilt_deg = float(np.clip(action[1] * self.max_step_deg, -self.max_step_deg, self.max_step_deg))

        return d_pan_deg, d_tilt_deg

    def update_applied_action(self, applied_pan_deg, applied_tilt_deg):
        """
        Hibrit modda servoya fiilen gonderilen harmanlanmis komutla eylem hafizasini gunceller.
        """
        norm_pan = float(np.clip(applied_pan_deg / self.max_step_deg, -1.0, 1.0))
        norm_tilt = float(np.clip(applied_tilt_deg / self.max_step_deg, -1.0, 1.0))
        self.act_hist_1 = np.array([norm_pan, norm_tilt], dtype=np.float32)
