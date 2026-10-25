"""Uretim PPO politikasinin TurretEnv uzerindeki kapali cevrim teshisi.

Ayni tohum ve hedef uzerinde iki varyant:
1. policy : `turret_fcs.rl_controller.RLController` (uretim cikarim yolu, hiz gozlemi 0.0)
2. raw    : egitimdeki gibi dogrudan ham politika aksiyonu (hiz gozlemi 0.0)
Iki yolun ayni sonucu vermesi, uretim sarmalayicisinin egitim semantigini
korudugunu gosterir.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.rl_controller import RLController
from turret_fcs.rl_sim.turret_env import TurretEnv

MODEL = Path(os.environ.get(
    "RL_MODEL",
    str(ROOT / "turret_fcs" / "rl_sim" / "models" / "turret_ppo_v14.zip"),
))


def sign_flip_rate(actions):
    actions = np.asarray(actions)
    moves = actions[np.abs(actions) > 1e-4]
    if moves.size < 3:
        return 0.0
    return float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0)


def run(seed, mode):
    np.random.seed(seed)
    env = TurretEnv(enable_randomization=False, use_velocity_obs=False)
    obs, _ = env.reset(seed=seed)
    model = PPO.load(str(MODEL), device="cpu")
    controller = RLController(auto_load=False)
    controller.model = model
    controller.is_loaded = True

    actions = []
    errors = []
    for _ in range(env.max_steps):
        az = math.degrees(math.atan2(env.target_pos_3d[0], max(0.1, env.target_pos_3d[2])))
        el = math.degrees(math.atan2(env.target_pos_3d[1], max(0.1, env.target_pos_3d[2])))
        rel_pan = az - env.turret_pan
        rel_tilt = el - env.turret_tilt
        eff_bx = env.center_x + math.tan(math.radians(rel_pan)) * env.fx
        eff_by = env.center_y - math.tan(math.radians(rel_tilt)) * env.fy

        if mode == "raw":
            action, _ = model.predict(obs, deterministic=True)
            action = np.asarray(action, dtype=np.float32)
        elif mode == "policy":
            target_dist = float(np.linalg.norm(env.target_pos_3d))
            # Hiz gozlemi bilincli olarak sifir (uretimdeki fcs_manager yolu gibi).
            d_pan, d_tilt = controller.predict_angles(
                eff_bx, eff_by, env.center_x, env.center_y, env.screen_w, env.screen_h,
                0.0, 0.0, target_dist, rel_pan, rel_tilt, env.dt,
            )
            action = np.array(
                [d_pan / env.max_step_pan_deg, d_tilt / env.max_step_tilt_deg],
                dtype=np.float32,
            )
        else:
            raise ValueError(mode)

        actions.append(np.asarray(action, dtype=np.float32))
        tp = env.last_pixel_pos
        errors.append(math.hypot(tp[0] - env.center_x, tp[1] - env.center_y))
        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            break

    a = np.array(actions)
    e = np.array(errors)
    return {
        "rms": float(np.sqrt(np.mean(e**2))),
        "pan_flip": sign_flip_rate(a[:, 0]),
        "tilt_flip": sign_flip_rate(a[:, 1]),
        "pan_jerk": float(np.mean(np.abs(np.diff(a[:, 0])))),
        "tilt_jerk": float(np.mean(np.abs(np.diff(a[:, 1])))),
        "pan_abs": float(np.mean(np.abs(a[:, 0]))),
        "tilt_abs": float(np.mean(np.abs(a[:, 1]))),
        "axis_corr": float(np.corrcoef(a[:, 0], a[:, 1])[0, 1]) if np.std(a[:, 0]) > 1e-6 and np.std(a[:, 1]) > 1e-6 else 0.0,
    }


def summarize(rows, label):
    keys = ("rms", "pan_flip", "tilt_flip", "pan_jerk", "tilt_jerk", "pan_abs", "tilt_abs", "axis_corr")
    print(f"\n{label} (n={len(rows)})")
    for key in keys:
        print(f"  {key:>9}: mean={np.mean([r[key] for r in rows]):.4f}")


if __name__ == "__main__":
    seeds = list(range(12))
    for mode in ("policy", "raw"):
        label = "policy (production controller path with zero velocity observation)" if mode == "policy" else "raw (direct policy observation)"
        summarize([run(s, mode) for s in seeds], label)
