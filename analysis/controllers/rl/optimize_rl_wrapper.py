#!/usr/bin/env python3
"""
================================================================================
MODUL: RL SARMALAYICI (WRAPPER) VE GOZLEM FILTRESI BAYESYEN OPTIMIZASYONU
================================================================================
Bu betik; donanimdan olculen gercek disli boslugu (backlash = 1.61 deg) ve
sensor dinamikleri altinda uretim RL sarmalayicisinin parametrelerini optimize eder:

Parametreler:
- reversal_damping  : Yon degisiminde aksiyon sonumleme katsayisi (0.20 - 0.80)
- taper_pan_deg     : Pan Hermite soft deadband gecis genisligi (0.80 - 3.50)
- taper_tilt_deg    : Tilt Hermite soft deadband gecis genisligi (1.00 - 4.50)
- d_err_ema_alpha   : Hata turevi EMA filtresi agirligi (0.15 - 0.65)

Hedef Fonksiyonu (Cost):
- Medyan ve RMS piksel takip hatasi
- Yon degistirme orani (sign flip rate)
- Komut degisim jerk'i (reversals around backlash)
================================================================================
"""

import json
import math
import sys
from pathlib import Path
import numpy as np
import optuna

optuna.logging.set_verbosity(optuna.logging.WARNING)

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from turret_fcs.rl_controller import RLController
from turret_fcs.rl.turret_env import TurretEnv
from turret_fcs.config import CONFIG

SEEDS = [11, 23, 42, 77, 88, 99, 105, 123, 202, 333]


def evaluate_wrapper(reversal_damping, taper_pan_deg, taper_tilt_deg, d_err_ema_alpha, backlash_deg=1.61):
    errors = []
    flip_rates = []
    jerks = []

    controller = RLController(
        auto_load=True,
        reversal_damping=reversal_damping,
        taper_pan_deg=taper_pan_deg,
        taper_tilt_deg=taper_tilt_deg,
    )

    for seed in SEEDS:
        np.random.seed(seed)
        env = TurretEnv(enable_randomization=False, use_velocity_obs=False)
        # Gercek donanimdan kestirilen disli boslugunu uygula
        env.backlash_deg = backlash_deg
        env.d_err_ema_alpha = d_err_ema_alpha

        controller.reset()
        obs, _ = env.reset(seed=seed)

        ep_errors = []
        ep_actions = []

        for _ in range(env.max_steps):
            az = math.degrees(math.atan2(env.target_pos_3d[0], max(0.1, env.target_pos_3d[2])))
            el = math.degrees(math.atan2(env.target_pos_3d[1], max(0.1, env.target_pos_3d[2])))
            rel_pan = az - env.turret_pan
            rel_tilt = el - env.turret_tilt
            eff_bx = env.center_x + math.tan(math.radians(rel_pan)) * env.fx
            eff_by = env.center_y - math.tan(math.radians(rel_tilt)) * env.fy

            target_dist = float(np.linalg.norm(env.target_pos_3d))
            d_pan, d_tilt = controller.predict_angles(
                eff_bx, eff_by, env.center_x, env.center_y, env.screen_w, env.screen_h,
                0.0, 0.0, target_dist, rel_pan, rel_tilt, env.dt,
            )
            action = np.array(
                [d_pan / env.max_step_pan_deg, d_tilt / env.max_step_tilt_deg],
                dtype=np.float32,
            )

            ep_actions.append(action)
            tp = env.last_pixel_pos
            ep_errors.append(math.hypot(tp[0] - env.center_x, tp[1] - env.center_y))

            obs, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                break

        a = np.array(ep_actions)
        e = np.array(ep_errors)
        errors.append(float(np.sqrt(np.mean(e ** 2))))

        # Flip rate
        moves_pan = a[:, 0][np.abs(a[:, 0]) > 1e-4]
        if len(moves_pan) > 2:
            flip_rates.append(float(np.mean(np.sign(moves_pan[1:]) != np.sign(moves_pan[:-1])) * 100.0))
        jerks.append(float(np.mean(np.abs(np.diff(a[:, 0])))))

    mean_rms = float(np.mean(errors))
    mean_flip = float(np.mean(flip_rates)) if flip_rates else 1.0
    mean_jerk = float(np.mean(jerks))

    # Rezonans korumali maliyet
    flip_penalty = max(0.0, mean_flip - 1.5) ** 2 * 10.0
    jerk_penalty = max(0.0, mean_jerk - 0.020) ** 2 * 1000.0
    cost = mean_rms + flip_penalty + jerk_penalty

    return {
        "cost": cost,
        "mean_rms": mean_rms,
        "mean_flip": mean_flip,
        "mean_jerk": mean_jerk,
    }


def run_rl_wrapper_optimization(n_trials: int = 50):
    print("=" * 70)
    print("  TURRET FCS // RL SARMALAYICI BAYESYEN OPTIMIZASYONU (OPTUNA TPE)")
    print("=" * 70)

    base_eval = evaluate_wrapper(
        reversal_damping=CONFIG.rl.reversal_damping,
        taper_pan_deg=CONFIG.rl.taper_pan_deg,
        taper_tilt_deg=CONFIG.rl.taper_tilt_deg,
        d_err_ema_alpha=CONFIG.rl.d_err_ema_alpha,
    )
    print("[BASLANGIC] Mevcut Parametreler:")
    print(f"    - reversal_damping: {CONFIG.rl.reversal_damping}")
    print(f"    - taper_pan_deg   : {CONFIG.rl.taper_pan_deg}")
    print(f"    - taper_tilt_deg  : {CONFIG.rl.taper_tilt_deg}")
    print(f"    - d_err_ema_alpha : {CONFIG.rl.d_err_ema_alpha}")
    print(f"    - RMS Hata        : {base_eval['mean_rms']:.2f} px")
    print(f"    - Yon Degistirme  : %{base_eval['mean_flip']:.2f}")
    print(f"    - Jerk            : {base_eval['mean_jerk']:.4f}")
    print(f"    - Toplam Maliyet  : {base_eval['cost']:.2f}")

    def objective(trial: optuna.Trial):
        reversal_damping = trial.suggest_float("reversal_damping", 0.20, 0.85)
        taper_pan_deg = trial.suggest_float("taper_pan_deg", 0.80, 3.50)
        taper_tilt_deg = trial.suggest_float("taper_tilt_deg", 1.00, 4.50)
        d_err_ema_alpha = trial.suggest_float("d_err_ema_alpha", 0.15, 0.65)

        res = evaluate_wrapper(reversal_damping, taper_pan_deg, taper_tilt_deg, d_err_ema_alpha)
        return res["cost"]

    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(direction="minimize", sampler=sampler)
    study.optimize(objective, n_trials=n_trials)

    best_p = study.best_params
    best_eval = evaluate_wrapper(
        reversal_damping=best_p["reversal_damping"],
        taper_pan_deg=best_p["taper_pan_deg"],
        taper_tilt_deg=best_p["taper_tilt_deg"],
        d_err_ema_alpha=best_p["d_err_ema_alpha"],
    )

    print("\n" + "=" * 70)
    print("  RL SARMALAYICI OPTUNA TPE EN IYI SONUCLARI")
    print("=" * 70)
    print(f"    - reversal_damping : {best_p['reversal_damping']:.4f} (varsayilan: {CONFIG.rl.reversal_damping})")
    print(f"    - taper_pan_deg    : {best_p['taper_pan_deg']:.4f} (varsayilan: {CONFIG.rl.taper_pan_deg})")
    print(f"    - taper_tilt_deg   : {best_p['taper_tilt_deg']:.4f} (varsayilan: {CONFIG.rl.taper_tilt_deg})")
    print(f"    - d_err_ema_alpha  : {best_p['d_err_ema_alpha']:.4f} (varsayilan: {CONFIG.rl.d_err_ema_alpha})")
    print("-" * 70)
    print(f"    - Maliyet (Cost)   : {best_eval['cost']:.2f} (Eski: {base_eval['cost']:.2f})")
    print(f"    - RMS Hata         : {best_eval['mean_rms']:.2f} px (Eski: {base_eval['mean_rms']:.2f} px)")
    print(f"    - Yon Degistirme   : %{best_eval['mean_flip']:.2f} (Eski: %{base_eval['mean_flip']:.2f})")
    print(f"    - Jerk             : {best_eval['mean_jerk']:.4f} (Eski: {base_eval['mean_jerk']:.4f})")
    print("=" * 70)

    out_json = BASE_DIR / "best_rl_wrapper_params.json"
    results = {
        "best_params": best_p,
        "best_metrics": best_eval,
        "baseline_metrics": base_eval,
    }
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"[+] Sonuclar kaydedildi: {out_json}")
    return results


if __name__ == "__main__":
    run_rl_wrapper_optimization(n_trials=50)
