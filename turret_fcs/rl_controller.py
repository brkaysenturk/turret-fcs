"""
================================================================================
MODUL: PEKISDIRMELI OGRENME DENETLEYICISI (RL CONTROLLER / PPO AGENT)
================================================================================
Bu modul, gercek donanim kisitlari (80 ms tasiyici gecikmesi, 0.90 deg olu bolge,
2.75 deg/kare adim doyumu ve Logitech C270 optigi) altinda egitilmis Proximal Policy
Optimization (PPO) yapay zeka modelinin gercek zamanli cikarim motorudur.

DURUM VE EYLEM UZAYI (BRYSON KURALI ILE NORMALIZASYON):
- Gozlem Vektoru (9 Boyutlu):
  [err_x, err_y, d_err_x, d_err_y, vx_norm, vy_norm, dist_norm, pan_step, tilt_step]
- Eylem Uzayi (2 Boyutlu):
  [-1.0, 1.0] araliginda kare basi normalize motor adim komutlari.
================================================================================
"""

from pathlib import Path
import numpy as np
from stable_baselines3 import PPO


class RLController:
    """
    Gercek Donanim (STM32 / Kamera) icin PPO tabanli
    ve gercek servo fizigiyle uyumlu hedef takip denetleyicisi.
    """

    def __init__(self, model_path=None):
        if model_path is None:
            base_dir = Path(__file__).resolve().parent
            model_path = base_dir / "rl_sim" / "models" / "turret_ppo_final.zip"

        self.model_path = Path(model_path)
        self.model = None
        self.is_loaded = False
        self.load_model()

        # Gercek Donanim Mekanik ve Bryson Olcekleri
        self.max_step_deg = 2.75         # Kare basi maksimum motor adimi (+-2.75 deg)
        self.v_max_px_s = 300.0          # Maksimum hedef piksel hizi (px/s)
        self.dist_min_m = 0.5
        self.dist_max_m = 5.0

        self.actual_pan_step = 0.0
        self.actual_tilt_step = 0.0
        self.last_err = np.zeros(2, dtype=np.float32)

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
        self.actual_pan_step = 0.0
        self.actual_tilt_step = 0.0
        self.last_err = np.zeros(2, dtype=np.float32)

    def predict_angles(self, bx, by, center_x, center_y, screen_w, screen_h, vx, vy, dist_m, curr_pan_deg, curr_tilt_deg, dt):
        """
        9 boyutlu Bryson durum vektorunu olusturur ve motora iletilecek aci komutunu uretir.
        """
        if not self.is_loaded or self.model is None:
            return 0.0, 0.0

        dt_safe = max(0.005, float(dt))

        # 1. Hata ve Hata Turevi (Bryson Normalizasyonu)
        err_x_norm = np.clip((bx - center_x) / (screen_w / 2.0), -1.5, 1.5)
        err_y_norm = np.clip((by - center_y) / (screen_h / 2.0), -1.5, 1.5)

        d_err_x = np.clip((err_x_norm - self.last_err[0]) / dt_safe * 0.05, -1.0, 1.0)
        d_err_y = np.clip((err_y_norm - self.last_err[1]) / dt_safe * 0.05, -1.0, 1.0)
        self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        # 2. Kalman Hedef Hizi (Piksel/saniye -> Bryson Normalizasyonu: V_max = 300 px/s)
        vx_norm = np.clip(float(vx) / self.v_max_px_s, -1.0, 1.0)
        vy_norm = np.clip(float(vy) / self.v_max_px_s, -1.0, 1.0)

        # 3. TF-Luna Mesafe Normalizasyonu (0.5m - 5.0m)
        dist_val = dist_m if (dist_m is not None and dist_m > 0.2) else 2.0
        dist_norm = np.clip((dist_val - self.dist_min_m) / (self.dist_max_m - self.dist_min_m), 0.0, 1.0) * 2.0 - 1.0

        # 4. Onceki Motor Adimi Normalizasyonu
        p_step_norm = np.clip(self.actual_pan_step / self.max_step_deg, -1.0, 1.0)
        t_step_norm = np.clip(self.actual_tilt_step / self.max_step_deg, -1.0, 1.0)

        obs = np.array(
            [
                err_x_norm,
                err_y_norm,
                d_err_x,
                d_err_y,
                vx_norm,
                vy_norm,
                dist_norm,
                p_step_norm,
                t_step_norm,
            ],
            dtype=np.float32,
        )

        action, _ = self.model.predict(obs, deterministic=True)

        # 5. Eylemden Fiziksel Motor Adimina Donusum (+-2.75 deg/kare doyumlu)
        d_pan_deg = float(np.clip(action[0] * self.max_step_deg, -self.max_step_deg, self.max_step_deg))
        d_tilt_deg = float(np.clip(action[1] * self.max_step_deg, -self.max_step_deg, self.max_step_deg))

        self.actual_pan_step = d_pan_deg
        self.actual_tilt_step = d_tilt_deg

        return d_pan_deg, d_tilt_deg
