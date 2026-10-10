"""
================================================================================
YILDIZ TEKNIK UNIVERSITESI - KONTROL VE OTOMASYON MUHENDISLIGI BOLUMU
BITIRME PROJESI: OTONOM PAN-TILT TARET VE ATIS KONTROL SISTEMI (FCS)
--------------------------------------------------------------------------------
MODUL: PEKISDIRMELI OGRENME DENETLEYICISI (RL CONTROLLER / PPO AGENT)
================================================================================
Bu modul, donanimda 80 ms tasiyici gecikmesi altinda hareketli hedefleri izlemek
icin egitilmis Proximal Policy Optimization (PPO) yapay zeka modelinin gercek
zamanli cikarim motorudur.

DURUM VE EYLEM UZAYI:
- Gozlem Vektoru (9 Boyutlu):
  [err_x, err_y, d_err_x, d_err_y, vx_target, vy_target, dist_norm, pan_vel, tilt_vel]
- Eylem Uzayi (2 Boyutlu Surekli):
  [-1.0, 1.0] araliginda hedef pan ve tilt acisal hiz komutlari.

GECIKME TELAFISI (LATENCY COMPENSATION):
PPO modeli simulasyonda 80 ms transport delay buffer'i ile egitildiginden, hedefin
anlik konumuna degil, 80 ms sonra olacagi konuma dogru taret komutu uretir (Lead Pursuit).
Bu sayede yuksek hizli hedeflerde klasik P kontrolcunun arkada kalma (lag) problemini cozer.
================================================================================
"""

from pathlib import Path
import numpy as np
from stable_baselines3 import PPO


class RLController:
    """
    Gercek Donanim (STM32 / Kamera) icin PPO tabanli
    yuksek hassasiyetli ve cevik hedef takip denetleyicisi.
    """

    def __init__(self, model_path=None):
        if model_path is None:
            base_dir = Path(__file__).resolve().parent
            model_path = base_dir / "rl_sim" / "models" / "turret_ppo_final.zip"

        self.model_path = Path(model_path)
        self.model = None
        self.is_loaded = False
        self.load_model()

        self.max_vel_deg_s = 140.0
        self.max_pan_deg = 90.0
        self.max_tilt_deg = 45.0

        self.pan_vel = 0.0
        self.tilt_vel = 0.0
        self.last_err = np.zeros(2, dtype=np.float32)

    def load_model(self):
        """Egitilmis PPO zip modelini yukler."""
        try:
            if self.model_path.exists():
                self.model = PPO.load(str(self.model_path), device="cpu")
                self.is_loaded = True
                print(f"[OK] [RL Controller] Gercek Sistem Icin PPO Modeli Yuklendi: {self.model_path.name}")
            else:
                print(f"[UYARI] [RL Controller] Model bulunamadi: {self.model_path}")
        except Exception as e:
            print(f"[HATA] [RL Controller] Model yuklenirken hata: {e}")
            self.is_loaded = False

    def reset(self):
        self.pan_vel = 0.0
        self.tilt_vel = 0.0
        self.last_err = np.zeros(2, dtype=np.float32)

    def predict_angles(self, bx, by, center_x, center_y, screen_w, screen_h, vx, vy, dist_m, curr_pan_deg, curr_tilt_deg, dt):
        """
        9 boyutlu durum vektorunu olusturur ve akici hiz tabanli aci duzeltmesi uretir.
        """
        if not self.is_loaded or self.model is None:
            return 0.0, 0.0

        dt_safe = max(0.005, float(dt))

        # 1. Hata ve Hata Turevi
        err_x_norm = np.clip((bx - center_x) / (screen_w / 2.0), -1.5, 1.5)
        err_y_norm = np.clip((by - center_y) / (screen_h / 2.0), -1.5, 1.5)

        d_err_x = np.clip((err_x_norm - self.last_err[0]) / dt_safe * 0.08, -1.0, 1.0)
        d_err_y = np.clip((err_y_norm - self.last_err[1]) / dt_safe * 0.08, -1.0, 1.0)
        self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        vx_norm = np.clip(vx / 2.0, -1.0, 1.0)
        vy_norm = np.clip(vy / 2.0, -1.0, 1.0)

        dist_val = dist_m if (dist_m is not None and dist_m > 0.2) else 2.0
        dist_norm = np.clip((dist_val - 0.5) / 7.5, 0.0, 1.0) * 2.0 - 1.0

        p_vel_norm = np.clip(self.pan_vel / self.max_vel_deg_s, -1.0, 1.0)
        t_vel_norm = np.clip(self.tilt_vel / self.max_vel_deg_s, -1.0, 1.0)

        obs = np.array(
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

        action, _ = self.model.predict(obs, deterministic=True)

        # 2. Hizli & Cevik Hiz Entegrasyonu
        target_pan_vel = float(action[0] * self.max_vel_deg_s)
        target_tilt_vel = float(action[1] * self.max_vel_deg_s)

        self.pan_vel = 0.50 * self.pan_vel + 0.50 * target_pan_vel
        self.tilt_vel = 0.50 * self.tilt_vel + 0.50 * target_tilt_vel

        # Aci Degisimi (Derece)
        d_pan_deg = self.pan_vel * dt_safe
        d_tilt_deg = self.tilt_vel * dt_safe

        return d_pan_deg, d_tilt_deg
