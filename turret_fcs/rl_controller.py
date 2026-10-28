"""
================================================================================
MODUL: RL KONTROLCU (SAF PPO CIKARIM MOTORU)
================================================================================
Sinif adi RLController'dir. v2.1 politikasi `rl_v2-1.zip` /
`rl_v2-1_policy.npz` olarak egitilir; cikarim SB3/torch surumune
bagimli degildir.

TASARIM GEREKCESI:
Onceki RL sarmalayicisi politikayi egittigi ortamdan farkli bir kapali cevrime
sokuyordu:
1. Pan cikisina EMA + 0.10 derece/kare slew, tilt cikisina ayri EMA ve
   yakin kilit bolgesinde asimetrik sonum uyguluyordu. Politika bu filtrelerle
   egitilmedi; filtreler faz gecikmesi ekleyip mikro duzeltmeleri buyuttu.
2. Eylem gecmisi (Smith Predictor) filtrelenmis motor komutunu sakliyordu.
   Politika ise egitimde kendi ham aksiyonunu goruyordu.
3. Yazilim olu bolgesi (0.85 / 0.95 derece) egitimde yoktu.
4. Hedef kaybinda eylem gecmisi temizlenmiyordu.

TASARIM (v2.0, SAF RL):
- Gozlem vektoru egitimle ayni: 11 boyut, ayni Bryson olcekleri.
- Gozlem, P kontrolcunun EMA'li hatasindan degil, olculen bbox'tan uretilir.
- Hiz gozlemi (obs[4:6]): Calisma zamaninda (fcs_manager) olculen Kalman hiz kestirimi
  kamera gurultusu ve sensor kuantalamasi sebebiyle yuksek frekansli gurultu icerdiginden
  ve kapali cevrimde pozitif geri besleme/limit cycle (avlanma) osilasyonuna yol actigindan
  politikaya runtime'da sifir hiz (0.0, 0.0) olarak verilir. Politika; hedefin dinamik hareketini
  11 boyutlu durum uzayindaki normalize konum hatasi, hata turevi (d_err) ve 2 adimlik
  Smith Predictor eylem gecmisiyle gurultuden etkilenmeden stabil ve sonumlu sekilde karsilar.
- Cikarim, SB3/torch gerektirmeyen 11-64-64-2 NumPy MLP ile yapilir.
- Eylem gecmisi uygulanan normalize komutu saklar.
- Cikista EMA/slew yok; yalnizca:
  * Acisal yumusak olu bolge (pan 0.85, tilt 0.95 derece ve Hermite taper;
    soft_deadband_enabled bayragiyla aktiflesir, politikanin bias'ini bastirir),
  * Yon degistiren aksiyonda 0.5 sonum (ileri-geri mikro salinimi keser),
  * Komut ivme siniri: pan 400.0, tilt 267.0 derece/s^2 (P ile ayni dinamik limit).
- P harmani YOKTUR: son komut dogrudan RL aksiyonudur.
- Egitim plant modeli: 1.2 derece backlash, 1.5 derece/kare adim siniri,
  0.10 derece servo deadband, progress odulu (k_p=2.0, k_e=0.05,
  lambda_u=0.02, mu_du=0.08).
================================================================================
"""

from pathlib import Path
import numpy as np

from .config import CONFIG


def rate_scaled_step_limits(max_step_pan_deg, max_step_tilt_deg, dt,
                            nominal_dt=CONFIG.rl.nominal_dt_s, max_output_deg=None):
    """Kare basi adim limitini dt'ye gore olcekler (egitimle ayni semantik)."""
    dt_safe = max(1e-3, float(dt))
    scale = float(np.clip(dt_safe / max(1e-6, float(nominal_dt)), 0.25, 3.0))
    cap_pan = float(CONFIG.control.max_output_deg if max_output_deg is None else max_output_deg)
    cap_tilt = float(CONFIG.actuator.max_step_tilt_deg if max_output_deg is None else min(float(max_output_deg), CONFIG.actuator.max_step_tilt_deg))
    pan = min(float(max_step_pan_deg) * scale, cap_pan)
    tilt = min(float(max_step_tilt_deg) * scale, cap_tilt)
    return float(pan), float(tilt), scale


class _NumpyPolicy:
    """SB3/torch bagimliligi olmayan 9-64-64-2 (veya 11D'den kayipsiz indirgenmis 9D) MLP cikarimi."""

    def __init__(self, path, target_obs_dim=9):
        data = np.load(path, allow_pickle=False)
        self.w1 = data["w1"].astype(np.float32)
        self.b1 = data["b1"].astype(np.float32)
        self.w2 = data["w2"].astype(np.float32)
        self.b2 = data["b2"].astype(np.float32)
        self.w3 = data["w3"].astype(np.float32)
        self.b3 = data["b3"].astype(np.float32)
        loaded_obs_dim = int(data["obs_dim"][0])
        self.act_dim = int(data["act_dim"][0])

        # 11-D egitilmis modeldeki 0.0 olan sahte hiz sutunlarini (indeks 4 ve 5) kayipsiz dilimle
        if target_obs_dim == 9 and loaded_obs_dim == 11 and self.w1.shape[1] == 11:
            self.w1 = np.delete(self.w1, [4, 5], axis=1)
            self.obs_dim = 9
        else:
            self.obs_dim = loaded_obs_dim

        if self.w1.shape != (64, self.obs_dim) or self.w3.shape != (self.act_dim, 64):
            raise ValueError(f"npz katman boyutlari uyumsuz: w1={self.w1.shape}, obs_dim={self.obs_dim}")

    def predict(self, obs, deterministic=True):
        x = np.asarray(obs, dtype=np.float32).reshape(self.obs_dim)
        x = np.tanh(x @ self.w1.T + self.b1)
        x = np.tanh(x @ self.w2.T + self.b2)
        action = np.clip(x @ self.w3.T + self.b3, -1.0, 1.0)
        return action.astype(np.float32), None


class RLController:
    """PPO politikasini egitim semantigine sadik calistiran temiz cikarim motoru."""

    OBS_DIM = 9
    ACT_DIM = 2

    def __init__(self, model_path=None, auto_load=True,
                 soft_deadband_enabled=None,
                 reversal_damping=None,
                 deadband_pan_deg=None, deadband_tilt_deg=None,
                 taper_pan_deg=None, taper_tilt_deg=None,
                 cmd_accel_pan_deg_s2=None, cmd_accel_tilt_deg_s2=None,
                 v_max_px_s=None, dist_min_m=None, dist_max_m=None):
        R = CONFIG.rl
        soft_deadband_enabled = (
            R.soft_deadband_enabled if soft_deadband_enabled is None else soft_deadband_enabled
        )
        reversal_damping = R.reversal_damping if reversal_damping is None else reversal_damping
        taper_pan_deg = R.taper_pan_deg if taper_pan_deg is None else taper_pan_deg
        taper_tilt_deg = R.taper_tilt_deg if taper_tilt_deg is None else taper_tilt_deg
        cmd_accel_pan_deg_s2 = (
            CONFIG.control.cmd_accel_pan_deg_s2 if cmd_accel_pan_deg_s2 is None else cmd_accel_pan_deg_s2
        )
        cmd_accel_tilt_deg_s2 = (
            CONFIG.control.cmd_accel_tilt_deg_s2 if cmd_accel_tilt_deg_s2 is None else cmd_accel_tilt_deg_s2
        )
        v_max_px_s = R.v_max_px_s if v_max_px_s is None else v_max_px_s
        dist_min_m = R.dist_min_m if dist_min_m is None else dist_min_m
        dist_max_m = R.dist_max_m if dist_max_m is None else dist_max_m
        if model_path is None:
            model_path = Path(__file__).resolve().parent / "rl" / "models" / "rl_v2-1.zip"
        self.model_path = Path(model_path)
        self.model = None
        self.is_loaded = False
        self.policy_format = None
        self.load_error = None
        if auto_load:
            self.load_model()

        self.max_step_pan_deg = CONFIG.actuator.max_step_pan_deg
        self.max_step_tilt_deg = CONFIG.actuator.max_step_tilt_deg
        self.v_max_px_s = float(v_max_px_s)
        self.dist_min_m = float(dist_min_m)
        self.dist_max_m = float(dist_max_m)

        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.has_prev_target = False
        self.last_action = np.zeros(2, dtype=np.float32)
        self.prev_policy_action = np.zeros(2, dtype=np.float32)
        self.recurrent_hidden = np.zeros(64, dtype=np.float32)
        self._in_deadband = False
        self.soft_deadband_enabled = bool(soft_deadband_enabled)
        self.reversal_damping = float(np.clip(float(reversal_damping), 0.0, 1.0))
        self.reversal_hold_frames = int(getattr(CONFIG.rl, 'reversal_hold_frames', 2))
        self._reversal_timer = np.zeros(self.ACT_DIM, dtype=int)

        self.deadband_pan_deg = (
            float(deadband_pan_deg) if deadband_pan_deg is not None
            else float(CONFIG.control.deadband_pan_deg)
        )
        self.deadband_tilt_deg = (
            float(deadband_tilt_deg) if deadband_tilt_deg is not None
            else float(CONFIG.control.deadband_tilt_deg)
        )
        self.taper_pan_deg = max(0.1, float(taper_pan_deg))
        self.taper_tilt_deg = max(0.1, float(taper_tilt_deg))
        self.cmd_accel_pan_deg_s2 = float(cmd_accel_pan_deg_s2)
        self.cmd_accel_tilt_deg_s2 = float(cmd_accel_tilt_deg_s2)
        self.prev_cmd_pan = 0.0
        self.prev_cmd_tilt = 0.0

    def load_model(self):
        """NumPy politikasini (varsa) veya SB3 zip modelini yukler."""
        npz_candidates = [
            self.model_path if self.model_path.suffix == ".npz" else self.model_path.with_name(self.model_path.stem + "_policy.npz"),
            self.model_path.with_suffix(".npz"),
        ]
        for npz_path in npz_candidates:
            if npz_path.exists():
                try:
                    model = _NumpyPolicy(npz_path)
                    if model.obs_dim != self.OBS_DIM or model.act_dim != self.ACT_DIM:
                        raise ValueError(f"npz boyutu uyumsuz: obs={model.obs_dim}, action={model.act_dim}")
                    self.model = model
                    self.is_loaded = True
                    self.policy_format = "numpy"
                    self.load_error = None
                    return True
                except Exception as exc:
                    self.load_error = f"npz yuklenemedi: {exc}"
                    break

        try:
            from stable_baselines3 import PPO

            if not self.model_path.exists():
                self.load_error = self.load_error or f"model bulunamadi: {self.model_path}"
                return False
            model = PPO.load(str(self.model_path), device="cpu")
            obs_shape = tuple(getattr(model.observation_space, "shape", ()))
            act_shape = tuple(getattr(model.action_space, "shape", ()))
            if obs_shape != (self.OBS_DIM,) or act_shape != (self.ACT_DIM,):
                self.load_error = f"model boyutu uyumsuz: obs={obs_shape}, action={act_shape}"
                return False
            self.model = model
            self.is_loaded = True
            self.policy_format = "sb3"
            self.load_error = None
            return True
        except Exception as exc:  # model bozuksa FCS P moduna duser
            self.load_error = str(exc)
            self.model = None
            self.is_loaded = False
            return False

    def reset(self):
        self.last_err = np.zeros(2, dtype=np.float32)
        self.d_err_filt = np.zeros(2, dtype=np.float32)
        self.act_hist_1 = np.zeros(2, dtype=np.float32)
        self.act_hist_2 = np.zeros(2, dtype=np.float32)
        self.has_prev_target = False
        self.last_action = np.zeros(2, dtype=np.float32)
        self.prev_policy_action = np.zeros(2, dtype=np.float32)
        self.recurrent_hidden = np.zeros(64, dtype=np.float32)
        self._in_deadband = False
        self._reversal_timer = np.zeros(self.ACT_DIM, dtype=int)
        self.prev_cmd_pan = 0.0
        self.prev_cmd_tilt = 0.0

    def on_target_lost(self):
        """Hedef kaybinda gecmisi temizle; yeniden yakalamada hayalet aksiyon kalmasin."""
        self.reset()

    def _build_observation(self, bx, by, center_x, center_y, screen_w, screen_h,
                           dist_m, dt):
        err_x_norm = float(np.clip((bx - center_x) / (screen_w / 2.0), -1.5, 1.5))
        err_y_norm = float(np.clip((by - center_y) / (screen_h / 2.0), -1.5, 1.5))

        if not self.has_prev_target:
            self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)
            self.d_err_filt = np.zeros(2, dtype=np.float32)
            d_err_x = 0.0
            d_err_y = 0.0
            self.has_prev_target = True
        else:
            d_scale = CONFIG.rl.d_err_scale
            d_alpha = CONFIG.rl.d_err_ema_alpha
            raw_d_x = (err_x_norm - self.last_err[0]) / max(0.005, dt) * d_scale
            raw_d_y = (err_y_norm - self.last_err[1]) / max(0.005, dt) * d_scale
            self.d_err_filt[0] = d_alpha * raw_d_x + (1.0 - d_alpha) * self.d_err_filt[0]
            self.d_err_filt[1] = d_alpha * raw_d_y + (1.0 - d_alpha) * self.d_err_filt[1]
            d_err_x = float(np.clip(self.d_err_filt[0], -1.0, 1.0))
            d_err_y = float(np.clip(self.d_err_filt[1], -1.0, 1.0))
            self.last_err = np.array([err_x_norm, err_y_norm], dtype=np.float32)

        dist_val = float(dist_m) if (dist_m is not None and dist_m > 0.2) else 2.0
        dist_norm = float(np.clip(
            (dist_val - self.dist_min_m) / (self.dist_max_m - self.dist_min_m), 0.0, 1.0
        )) * 2.0 - 1.0

        return np.array(
            [
                err_x_norm,
                err_y_norm,
                d_err_x,
                d_err_y,
                dist_norm,
                self.act_hist_1[0],
                self.act_hist_1[1],
                self.act_hist_2[0],
                self.act_hist_2[1],
            ],
            dtype=np.float32,
        )

    def predict_angles(self, bx, by, center_x, center_y, screen_w, screen_h,
                       vx=0.0, vy=0.0, dist_m=2.0, curr_pan_deg=0.0, curr_tilt_deg=0.0, dt=0.033):
        """Politikadan 9-boyutlu durum ile egitimdeki gibi ham aksiyon alir ve fiziksel adima cevirir."""
        if not self.is_loaded or self.model is None:
            return 0.0, 0.0

        dt_safe = max(0.005, float(dt))
        obs = self._build_observation(
            bx, by, center_x, center_y, screen_w, screen_h, dist_m, dt_safe
        )
        action, _ = self.model.predict(obs, deterministic=True)
        action = np.asarray(action, dtype=np.float32).reshape(self.ACT_DIM)
        if not np.all(np.isfinite(action)):
            action = np.zeros(self.ACT_DIM, dtype=np.float32)

        max_pan_step, max_tilt_step, _ = rate_scaled_step_limits(
            self.max_step_pan_deg, self.max_step_tilt_deg, dt_safe
        )

        # Yon degistiren aksiyonu yumusat: 2 kare (~66ms, gecikme+backlash) boyunca
        # sonum uygulayarak mikro salinimi ve limit cycle rezonansini sonumler.
        damped = action.copy()
        for axis in range(self.ACT_DIM):
            if action[axis] * self.prev_policy_action[axis] < 0.0:
                self._reversal_timer[axis] = self.reversal_hold_frames
            if self._reversal_timer[axis] > 0:
                damped[axis] *= self.reversal_damping
                self._reversal_timer[axis] -= 1

        d_pan_deg = float(damped[0] * max_pan_step)
        d_tilt_deg = float(damped[1] * max_tilt_step)

        # 1. Acisal yumusak olu bolge (Soft Deadband Taper)
        # Hedef merkezdeyken politikanin sifir-hata gurultusunu bastirir,
        # MG996R mekanik disli boslugu (backlash) rezonansini ve avlanmayi (limit cycle) onler.
        # Gercek uygulanan olu bolge acisal bazdadir: Pan 0.85°, Tilt 0.95° ve Hermite taper.
        # soft_deadband_enabled bu bolgeyi ac/kapa yapar (testlerde ham politika icin kapatilir).
        if self.soft_deadband_enabled:
            fx = CONFIG.optics.compute_fx(screen_w)
            fy = CONFIG.optics.compute_fy(screen_h)
            err_pan = (
                float(curr_pan_deg) if (curr_pan_deg is not None and abs(float(curr_pan_deg)) > 1e-4)
                else float(np.degrees(np.arctan((bx - center_x) / max(1e-3, fx))))
            )
            err_tilt = (
                float(curr_tilt_deg) if (curr_tilt_deg is not None and abs(float(curr_tilt_deg)) > 1e-4)
                else float(np.degrees(np.arctan(-(by - center_y) / max(1e-3, fy))))
            )

            abs_pan = abs(err_pan)
            abs_tilt = abs(err_tilt)

            def _calc_taper(err_abs, db_deg, taper_deg):
                if err_abs <= db_deg:
                    return 0.0
                if err_abs >= (db_deg + taper_deg):
                    return 1.0
                x = (err_abs - db_deg) / max(1e-4, taper_deg)
                return float(x * x * (3.0 - 2.0 * x))

            taper_pan = _calc_taper(abs_pan, self.deadband_pan_deg, self.taper_pan_deg)
            taper_tilt = _calc_taper(abs_tilt, self.deadband_tilt_deg, self.taper_tilt_deg)

            d_pan_deg *= taper_pan
            d_tilt_deg *= taper_tilt

            self._in_deadband = (abs_pan <= self.deadband_pan_deg and abs_tilt <= self.deadband_tilt_deg)
            if self._in_deadband:
                d_pan_deg = 0.0
                d_tilt_deg = 0.0

        # 2. Ivme sinirlama (Slew Rate Limiter - P ile ayni: CONFIG.control.cmd_accel_*)
        max_delta_pan = self.cmd_accel_pan_deg_s2 * (dt_safe ** 2)
        d_pan_deg = float(np.clip(
            d_pan_deg, self.prev_cmd_pan - max_delta_pan, self.prev_cmd_pan + max_delta_pan
        ))
        max_delta_tilt = self.cmd_accel_tilt_deg_s2 * (dt_safe ** 2)
        d_tilt_deg = float(np.clip(
            d_tilt_deg, self.prev_cmd_tilt - max_delta_tilt, self.prev_cmd_tilt + max_delta_tilt
        ))

        d_pan_deg = float(np.clip(d_pan_deg, -max_pan_step, max_pan_step))
        d_tilt_deg = float(np.clip(d_tilt_deg, -max_tilt_step, max_tilt_step))
        self.prev_cmd_pan = d_pan_deg
        self.prev_cmd_tilt = d_tilt_deg

        # Egitimdeki gibi normalize komut gecmisi (deadband/reversal sonrasi).
        applied = np.array(
            [d_pan_deg / max(1e-6, max_pan_step), d_tilt_deg / max(1e-6, max_tilt_step)],
            dtype=np.float32,
        )
        self.act_hist_2 = self.act_hist_1.copy()
        self.act_hist_1 = applied
        self.last_action = applied
        self.prev_policy_action = action
        return d_pan_deg, d_tilt_deg
