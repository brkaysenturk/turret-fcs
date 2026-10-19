"""
================================================================================
MODUL: PEKISDIRMELI OGRENME DENETLEYICISI (RL CONTROLLER / PPO AGENT)
================================================================================
Bu modul, gercek donanim kisitlari (Domain Randomization, eylem gecmisi hafizasi,
0.90 deg olu bolge, 2.75 deg/kare adim doyumu ve Logitech C270 optigi) altinda
egitilmis Proximal Policy Optimization (PPO) yapay zeka modelinin gercek zamanli
cikarim motorudur.

DURUM VE EYLEM UZAYI (BRYSON KURALI ILE NORMALIZASYON):
- Gozlem Vektoru (11 Boyutlu):
  [err_x, err_y, d_err_x, d_err_y, vx_norm, vy_norm, dist_norm,
   act_pan_t1, act_tilt_t1, act_pan_t2, act_tilt_t2]
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
        self.max_step_pan_deg = 0.55     # Yatay servo cevik takip limiti (+-0.55 deg/kare = 16.5 deg/s)
        self.max_step_tilt_deg = 0.22    # Dikey agir volan namlu takip limiti (+-0.22 deg/kare = 6.6 deg/s)
        self.max_step_deg = self.max_step_pan_deg
        self.v_max_px_s = 300.0          # Maksimum hedef piksel hizi (px/s)
        self.dist_min_m = 0.5
        self.dist_max_m = 5.0

        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.smooth_d_tilt = 0.0
        self.has_prev_target = False

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
        self.smooth_d_tilt = 0.0
        self.has_prev_target = False

    def on_target_lost(self):
        """Hedef kadrajdan ciktiginda cagrilir; hedef yeniden belirdiginde sahte turev patlamasini onler."""
        self.has_prev_target = False
        self.smooth_d_tilt = 0.0

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

        if not self.has_prev_target:
            # Hedef yeni belirdi veya kaybolup geri geldi: Turev kesin olarak 0, gecmis hata anlik konuma esitlenir
            self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)
            self.d_err_filt = np.zeros(2, dtype=np.float32)
            d_err_x = 0.0
            d_err_y = 0.0
            self.has_prev_target = True
        else:
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

        # 5. Eylemden Fiziksel Motor Adimina Donusum (Gercek Eylemsizlik Sinirlamali)
        d_pan_deg = float(np.clip(action[0] * self.max_step_pan_deg, -self.max_step_pan_deg, self.max_step_pan_deg))
        d_tilt_deg = float(np.clip(action[1] * self.max_step_tilt_deg, -self.max_step_tilt_deg, self.max_step_tilt_deg))

        # 6. Kilit Alani Yumusak Olu Bolge ve Asimetrik Eylemsizlik Sonumu
        # Pan Ekseni: 0.85 derece (Yatay simetrik, dusuk eylemsizlik)
        deadband_pan = 0.85
        abs_err_p = abs(float(curr_pan_deg))
        if abs_err_p <= deadband_pan:
            d_pan_deg = 0.0
        else:
            scale_p = (abs_err_p - deadband_pan) / max(0.5, abs_err_p)
            d_pan_deg *= float(np.clip(scale_p, 0.0, 1.0))

        # Tilt Ekseni: 0.95 derece (300g pirinc volan namlu, yercekimi ve mekanik bosluk sonumu)
        deadband_tilt = 0.95
        abs_err_t = abs(float(curr_tilt_deg))
        if abs_err_t <= deadband_tilt:
            d_tilt_deg = 0.0
            self.smooth_d_tilt = 0.0
        else:
            # Kilit bolgesi yakininda (0.95 - 3.0 deg) stick-slip avlanmasini ve titresimi onleyen sonumlu gecis
            scale_t = (abs_err_t - deadband_tilt) / max(0.8, abs_err_t)
            damping_t = float(np.clip(0.45 + 0.55 * ((abs_err_t - deadband_tilt) / 3.0), 0.45, 1.0))
            d_tilt_deg *= float(np.clip(scale_t * damping_t, 0.0, 1.0))

            # Dikey eksen yumusatma filtresi (Sifir jerk / mekanik titresim onleyici)
            self.smooth_d_tilt = 0.70 * d_tilt_deg + 0.30 * self.smooth_d_tilt
            d_tilt_deg = self.smooth_d_tilt

        # 7. Eylem Gecmisi Guncellemesi (Fiilen uygulanan gercek motor komutlari - Smith Predictor Tutarliligi)
        applied_norm_pan = float(np.clip(d_pan_deg / self.max_step_pan_deg, -1.0, 1.0))
        applied_norm_tilt = float(np.clip(d_tilt_deg / self.max_step_tilt_deg, -1.0, 1.0))
        self.act_hist_2 = self.act_hist_1.copy()
        self.act_hist_1 = np.array([applied_norm_pan, applied_norm_tilt], dtype=np.float32)

        return d_pan_deg, d_tilt_deg

