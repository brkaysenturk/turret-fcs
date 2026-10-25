#!/usr/bin/env python3
"""
================================================================================
MODUL: KALMAN FILTRESI COK-DEGISKENLI BAYESYEN OPTIMIZASYONU (OPTUNA TPE)
================================================================================
Bu betik; gercek saha telemetri kayitlari uzerinde sabit hizli (CV) Kalman
filtresi ve ego-hareket gozlemcisi parametrelerini matematiksel olarak optimize eder:

Parametreler:
- sigma_a (px/s^2)        : Surec ivme gurultusu (100.0 - 600.0)
- r_var (px^2)            : Olcum gurultusu varyansi (4.0 - 100.0)
- deadband_low (px/s)     : Alt hiz olu bolgesi (15.0 - 50.0)
- deadband_high (px/s)    : Ust hiz olu bolgesi (35.0 - 90.0)
- ego_tau_s (s)           : Ego-hareket servo zaman sabiti (0.015 - 0.080)

Hedef Fonksiyonu (Cost):
- Konum inovasyon hatasi (RMSE)
- Hiz dalgalanmasi ve titreşimi (Velocity Jitter)
- Aykiri deger (outlier) sicrama direnci
- Duragan hedefte sifir-hiz kararliligi
================================================================================
"""

import csv
import json
import sys
from pathlib import Path
import numpy as np
import optuna

optuna.logging.set_verbosity(optuna.logging.WARNING)

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from turret_fcs.kalman_filter import KalmanFilter
from turret_fcs.config import CONFIG


def load_telemetry_frames(csv_path: Path):
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    frames = []
    for r in rows:
        if r["target_detected"] == "1":
            frames.append({
                "cx": float(r["box_cx"]),
                "cy": float(r["box_cy"]),
                "dt": max(0.010, min(0.100, float(r["dt"]))),
                "cmd_pan": float(r["cmd_pan"]),
                "cmd_tilt": float(r["cmd_tilt"]),
            })
    return frames


def evaluate_kalman_replay(frames, sigma_a, r_var, deadband_low, deadband_high, ego_tau_s):
    kf = KalmanFilter(
        sigma_a=sigma_a,
        r_var=r_var,
        deadband_low=deadband_low,
        deadband_high=deadband_high,
    )

    fx = CONFIG.optics.compute_fx(1280)
    fy = CONFIG.optics.compute_fy(720)

    ego_step_pan = 0.0
    ego_step_tilt = 0.0

    innovations = []
    vxs = []
    vys = []

    for fr in frames:
        cx, cy, dt = fr["cx"], fr["cy"], fr["dt"]
        cmd_p, cmd_t = fr["cmd_pan"], fr["cmd_tilt"]

        # Ego pixel shift
        alpha_m = float(dt / (ego_tau_s + dt))
        ego_step_pan = (1.0 - alpha_m) * ego_step_pan + alpha_m * cmd_p
        ego_step_tilt = (1.0 - alpha_m) * ego_step_tilt + alpha_m * cmd_t
        ego_dx = -float(fx) * float(np.tan(np.radians(ego_step_pan)))
        ego_dy = float(fy) * float(np.tan(np.radians(ego_step_tilt)))

        state = kf.update((cx, cy), dt=dt, ego_dx=ego_dx, ego_dy=ego_dy)

        # Innovation (inlier frames where innov < 150 px)
        innov = np.sqrt((cx - state[0]) ** 2 + (cy - state[1]) ** 2)
        if innov < 150.0:
            innovations.append(innov)
        vxs.append(state[2])
        vys.append(state[3])

    innovations = np.array(innovations) if innovations else np.array([50.0])
    vxs = np.array(vxs)
    vys = np.array(vys)

    # 1. Konum inovasyon RMSE (px)
    pos_rmse = float(np.sqrt(np.mean(innovations ** 2)))

    # 2. Hiz dalgalanmasi / titreşim (Jitter: median frame-to-frame diff)
    diff_vx = np.abs(np.diff(vxs))
    diff_vy = np.abs(np.diff(vys))
    jitter = float(np.median(diff_vx) + np.median(diff_vy))

    # 3. Maksimum hiz sicramasi (outlier basim kontrolu)
    max_jump = float(np.max(diff_vx) + np.max(diff_vy)) if len(diff_vx) > 0 else 0.0

    # 4. Duragan bolgelerde sifir hiz orani (deadband kalitesi)
    speeds = np.sqrt(vxs ** 2 + vys ** 2)
    zero_ratio = float(np.mean(speeds == 0.0))

    # Toplam Maliyet (Cost - minimize edilecek)
    cost = pos_rmse + 1.2 * jitter + 0.02 * max_jump - 5.0 * zero_ratio
    return {
        "cost": cost,
        "pos_rmse": pos_rmse,
        "jitter": jitter,
        "max_jump": max_jump,
        "zero_ratio": zero_ratio,
    }


def run_kalman_optimization(n_trials: int = 60):
    log_path = PROJECT_ROOT / "logs" / "telemetry_test_20261025_054723.csv"
    print(f"[*] Telemetri verisi yukleniyor: {log_path.name}")
    frames = load_telemetry_frames(log_path)
    print(f"[*] Toplam analiz edilecek hedefli kare sayisi: {len(frames)}")

    # Once mevcut varsayilanlari olcelim
    base_eval = evaluate_kalman_replay(
        frames,
        sigma_a=CONFIG.kalman.sigma_a,
        r_var=CONFIG.kalman.r_var,
        deadband_low=CONFIG.kalman.velocity_deadband_low_px_s,
        deadband_high=CONFIG.kalman.velocity_deadband_high_px_s,
        ego_tau_s=CONFIG.control.ego_tau_s,
    )
    print(f"[BASLANGIC] Mevcut Parametreler Maliyeti: {base_eval['cost']:.2f}")
    print(f"    - Inovasyon RMSE : {base_eval['pos_rmse']:.2f} px")
    print(f"    - Hiz Titresimi  : {base_eval['jitter']:.2f} px/s")
    print(f"    - Sifir Hiz Orani: %{base_eval['zero_ratio']*100:.1f}")

    def objective(trial: optuna.Trial):
        sigma_a = trial.suggest_float("sigma_a", 150.0, 500.0)
        r_var = trial.suggest_float("r_var", 4.0, 64.0)
        deadband_low = trial.suggest_float("deadband_low", 15.0, 45.0)
        deadband_high = trial.suggest_float("deadband_high", deadband_low + 10.0, 80.0)
        ego_tau_s = trial.suggest_float("ego_tau_s", 0.015, 0.070)

        res = evaluate_kalman_replay(frames, sigma_a, r_var, deadband_low, deadband_high, ego_tau_s)
        return res["cost"]

    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(direction="minimize", sampler=sampler)
    study.optimize(objective, n_trials=n_trials)

    best_p = study.best_params
    best_eval = evaluate_kalman_replay(
        frames,
        sigma_a=best_p["sigma_a"],
        r_var=best_p["r_var"],
        deadband_low=best_p["deadband_low"],
        deadband_high=best_p["deadband_high"],
        ego_tau_s=best_p["ego_tau_s"],
    )

    print("\n" + "=" * 70)
    print("  KALMAN OPTUNA TPE EN IYI SONUCLARI")
    print("=" * 70)
    print(f"    - sigma_a       : {best_p['sigma_a']:.2f} px/s^2 (varsayilan: {CONFIG.kalman.sigma_a})")
    print(f"    - r_var         : {best_p['r_var']:.2f} px^2 (varsayilan: {CONFIG.kalman.r_var})")
    print(f"    - deadband_low  : {best_p['deadband_low']:.2f} px/s (varsayilan: {CONFIG.kalman.velocity_deadband_low_px_s})")
    print(f"    - deadband_high : {best_p['deadband_high']:.2f} px/s (varsayilan: {CONFIG.kalman.velocity_deadband_high_px_s})")
    print(f"    - ego_tau_s     : {best_p['ego_tau_s']:.4f} s (varsayilan: {CONFIG.control.ego_tau_s})")
    print("-" * 70)
    print(f"    - Maliyet (Cost): {best_eval['cost']:.2f} (Eski: {base_eval['cost']:.2f})")
    print(f"    - Inovasyon RMSE: {best_eval['pos_rmse']:.2f} px (Eski: {base_eval['pos_rmse']:.2f} px)")
    print(f"    - Hiz Titresimi : {best_eval['jitter']:.2f} px/s (Eski: {base_eval['jitter']:.2f} px/s)")
    print(f"    - Sifir Hiz Oran: %{best_eval['zero_ratio']*100:.1f} (Eski: %{base_eval['zero_ratio']*100:.1f})")
    print("=" * 70)

    out_json = BASE_DIR / "best_kalman_params.json"
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
    run_kalman_optimization(n_trials=60)
