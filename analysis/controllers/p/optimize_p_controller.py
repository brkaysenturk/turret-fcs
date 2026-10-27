#!/usr/bin/env python3
"""
================================================================================
MODUL: P KONTROLCU COK DEGISKENLI (MULTIVARIATE) BAYESYEN TPE OPTIMIZASYONU
================================================================================
Bu betik; gercek donanim sistem tanilamasiyla (System ID) olculen tesis dinamikleri
altinda (backlash = 1.61 deg, gecikme = 2 kare / 66.7 ms, yolo_noise_std = 2.10 px,
tau = 45 ms, max_step_tilt = 0.75 deg) Tip-1 P kontrolcunun 5 temel parametresini
30 farkli hedef senaryosu uzerinde 100 denemelik Optuna TPE ile optimize eder:

Parametreler:
1. kp_pan               : Pan kare basi oransal kazanci (30 FPS bazinda)
2. kp_tilt              : Tilt kare basi oransal kazanci (30 FPS bazinda)
3. alpha_filter         : Giris EMA filtresi agirligi (dt-olcekli)
4. cmd_accel_pan_deg_s2 : Pan komut ivme siniri (deg/s^2)
5. cmd_accel_tilt_deg_s2: Tilt komut ivme siniri (deg/s^2)

Maliyet Fonksiyonu (Cost):
- RMS ve Medyan piksel takip hatasi
- Yon degistirme orani (sign flip rate) ve komut jerk'i (rezonans korumasi)
- Vizor kilitlenme orani (lock_pct)
================================================================================
"""

import json
import sys
import time
from pathlib import Path

import numpy as np
import optuna

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(PROJECT_ROOT / "turret_fcs" / "rl") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "turret_fcs" / "rl"))

from turret_fcs.config import CONFIG
from turret_fcs.p_controller import PController
from turret_fcs.rl.turret_env import TurretEnv

optuna.logging.set_verbosity(optuna.logging.WARNING)

EVAL_SEEDS = list(range(100, 130))  # 30 sabit tohum (5 farkli hedef senaryo tipi dagilimi)


def sign_flip_rate(arr: np.ndarray) -> float:
    """Komut dizisindeki yon degistirme yuzdesi (titresim ve avlanma olcutu)."""
    a = np.asarray(arr, dtype=np.float32)
    moves = a[np.abs(a) > 1e-4]
    if moves.size < 3:
        return 0.0
    return float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0)


def evaluate_p_params(
    kp_pan: float,
    kp_tilt: float,
    alpha: float,
    accel_pan: float,
    accel_tilt: float,
    seeds: list[int] = EVAL_SEEDS,
) -> dict[str, float]:
    """Verilen 5 kontrolcu parametresini gercekci TurretEnv uzerinde 30 tohumla test eder."""
    rms_list = []
    med_list = []
    lock_steps = 0
    total_steps = 0
    pan_flips = []
    tilt_flips = []
    pan_jerks = []
    tilt_jerks = []
    sat_steps = 0

    for seed in seeds:
        np.random.seed(seed)
        env = TurretEnv(
            render_mode=None,
            latency_steps=2,
            backlash_deg=CONFIG.actuator.backlash_deg,  # 1.61 deg (gercek donanim)
            enable_randomization=True,
        )
        obs, _ = env.reset(seed=seed)
        p_ctrl = PController(
            kp_pan=kp_pan,
            kp_tilt=kp_tilt,
            alpha=alpha,
            cmd_accel_pan_deg_s2=accel_pan,
            cmd_accel_tilt_deg_s2=accel_tilt,
        )

        ep_err_px = []
        ep_u_pan = []
        ep_u_tilt = []

        for _ in range(env.max_steps):
            # Gecikmeli kamera gozleminden piksel koordinatini geri coz (gercek 2-kare kamera gecikmesiyle)
            err_x_norm = float(obs[0])
            err_y_norm = float(obs[1])
            bx = env.center_x + err_x_norm * (env.screen_w / 2.0)
            by = env.center_y + err_y_norm * (env.screen_h / 2.0)

            pan_deg, tilt_deg = p_ctrl.compute_angles(
                bx, by, env.center_x, env.center_y, env.fx, env.fy, dt=env.dt
            )
            u_pan_deg, u_tilt_deg = p_ctrl.compute_p(pan_deg, tilt_deg, dt=env.dt)

            act_pan = u_pan_deg / env.max_step_pan_deg
            act_tilt = u_tilt_deg / env.max_step_tilt_deg
            action = np.array([act_pan, act_tilt], dtype=np.float32)

            obs, _, terminated, truncated, info = env.step(action)
            err_norm = float(info.get("pixel_dist", 0.0))
            err_px = err_norm * (env.screen_w / 2.0)
            ep_err_px.append(err_px)
            ep_u_pan.append(u_pan_deg)
            ep_u_tilt.append(u_tilt_deg)

            if err_norm < 0.05:
                lock_steps += 1
            if abs(u_pan_deg) >= 1.35:
                sat_steps += 1
            total_steps += 1

            if terminated or truncated:
                break

        e_arr = np.asarray(ep_err_px, dtype=np.float32)
        u_p = np.asarray(ep_u_pan, dtype=np.float32)
        u_t = np.asarray(ep_u_tilt, dtype=np.float32)
        rms_list.append(float(np.sqrt(np.mean(e_arr ** 2))))
        med_list.append(float(np.median(e_arr)))
        pan_flips.append(sign_flip_rate(u_p))
        tilt_flips.append(sign_flip_rate(u_t))
        pan_jerks.append(float(np.mean(np.abs(np.diff(u_p)))))
        tilt_jerks.append(float(np.mean(np.abs(np.diff(u_t)))))

    return {
        "rms_px": float(np.mean(rms_list)),
        "med_px": float(np.mean(med_list)),
        "lock_pct": float(lock_steps / max(1, total_steps) * 100.0),
        "pan_flip_pct": float(np.mean(pan_flips)),
        "tilt_flip_pct": float(np.mean(tilt_flips)),
        "pan_jerk_deg": float(np.mean(pan_jerks)),
        "tilt_jerk_deg": float(np.mean(tilt_jerks)),
        "sat_pct": float(sat_steps / max(1, total_steps) * 100.0),
    }


def main():
    print("=" * 76)
    print(" P KONTROLCU COK DEGISKENLI (MULTIVARIATE) BAYESYEN TPE OPTIMIZASYONU")
    print(f" Tesis Parametreleri: Backlash = {CONFIG.actuator.backlash_deg:.2f} deg | Gecikme = 2 kare (66.7 ms)")
    print("=" * 76)

    # 1. Mevcut Yapilandirma (Trial #0 referansi)
    cur = evaluate_p_params(
        CONFIG.control.kp_pan,
        CONFIG.control.kp_tilt,
        CONFIG.control.alpha_filter,
        CONFIG.control.cmd_accel_pan_deg_s2,
        CONFIG.control.cmd_accel_tilt_deg_s2,
    )
    print(f" MEVCUT CONFIG REFERANSI (#0: Kp={CONFIG.control.kp_pan:.4f}/{CONFIG.control.kp_tilt:.4f}, alpha={CONFIG.control.alpha_filter:.4f})")
    print(
        f"   RMS Hata: {cur['rms_px']:.2f} px | Medyan: {cur['med_px']:.2f} px | Kilit: %{cur['lock_pct']:.2f}\n"
        f"   Pan Flip: %{cur['pan_flip_pct']:.2f} | Tilt Flip: %{cur['tilt_flip_pct']:.2f} | "
        f"Pan Jerk: {cur['pan_jerk_deg']:.4f} deg | Doyum: %{cur['sat_pct']:.2f}"
    )
    print("-" * 76)

    max_allowed_pan_flip = cur["pan_flip_pct"] + 0.20
    max_allowed_tilt_flip = cur["tilt_flip_pct"] + 0.20

    def objective(trial: optuna.Trial) -> float:
        # Gercek donanim ve Webots rezonans sinirlari dahilinde arama araligi:
        kp_pan = trial.suggest_float("kp_pan", 0.160, 0.260)
        kp_tilt = trial.suggest_float("kp_tilt", 0.090, 0.170)
        alpha = trial.suggest_float("alpha_filter", 0.750, 0.960)
        accel_pan = trial.suggest_float("cmd_accel_pan_deg_s2", 280.0, 480.0)
        accel_tilt = trial.suggest_float("cmd_accel_tilt_deg_s2", 160.0, 320.0)

        m = evaluate_p_params(kp_pan, kp_tilt, alpha, accel_pan, accel_tilt)

        # Rezonans ve Titresim Koruma Bariyeri (Limit Cycle Guard):
        pan_flip_ex = max(0.0, m["pan_flip_pct"] - max_allowed_pan_flip)
        tilt_flip_ex = max(0.0, m["tilt_flip_pct"] - max_allowed_tilt_flip)
        jerk_ex = max(0.0, m["pan_jerk_deg"] - (cur["pan_jerk_deg"] * 1.20))
        res_penalty = 150.0 * (pan_flip_ex ** 2 + tilt_flip_ex ** 2) + 2000.0 * (jerk_ex ** 2)

        cost = (
            m["rms_px"]
            + 0.5 * m["med_px"]
            + 6.0 * (m["pan_flip_pct"] + m["tilt_flip_pct"])
            + 120.0 * (m["pan_jerk_deg"] + m["tilt_jerk_deg"])
            + res_penalty
            - 0.8 * m["lock_pct"]
        )

        for k, v in m.items():
            trial.set_user_attr(k, v)
        return cost

    sampler = optuna.samplers.TPESampler(seed=42, n_startup_trials=15, multivariate=True)
    study = optuna.create_study(direction="minimize", sampler=sampler)

    # Ilk denemeler olarak mevcut konfigrasyonu ve eski nominali onceden kaydet
    study.enqueue_trial(
        {
            "kp_pan": CONFIG.control.kp_pan,
            "kp_tilt": CONFIG.control.kp_tilt,
            "alpha_filter": CONFIG.control.alpha_filter,
            "cmd_accel_pan_deg_s2": CONFIG.control.cmd_accel_pan_deg_s2,
            "cmd_accel_tilt_deg_s2": CONFIG.control.cmd_accel_tilt_deg_s2,
        }
    )
    study.enqueue_trial(
        {
            "kp_pan": 0.2200,
            "kp_tilt": 0.1100,
            "alpha_filter": 0.8500,
            "cmd_accel_pan_deg_s2": 400.0,
            "cmd_accel_tilt_deg_s2": 267.0,
        }
    )

    t0 = time.time()
    study.optimize(objective, n_trials=100)
    elapsed = time.time() - t0

    best = study.best_trial
    bm = best.user_attrs

    print(f"\n[OK] 100 Multivariate Bayesyen Deneme Tamamlandi ({elapsed:.1f} saniye)")
    print(f"En Iyi Deneme: #{best.number} (Maliyet: {best.value:.2f} vs Referans #0: {study.trials[0].value:.2f})")
    print("-" * 76)
    print(" EN IYI P KONTROLCU PARAMETRELERI (1.61 deg Backlash Altinda):")
    for k, v in best.params.items():
        print(f"   - {k:24s}: {v:.4f}")
    print("-" * 76)
    print(" KARSILASTIRMA (MEVCUT vs OPTUNA YENI - 30 TOHUM):")
    print(f"   - RMS Piksel Hatasi : {cur['rms_px']:.2f} px  ->  {bm['rms_px']:.2f} px")
    print(f"   - Medyan Hata       : {cur['med_px']:.2f} px  ->  {bm['med_px']:.2f} px")
    print(f"   - Kilit Orani       : %{cur['lock_pct']:.2f}    ->  %{bm['lock_pct']:.2f}")
    print(f"   - Pan Flip Orani    : %{cur['pan_flip_pct']:.2f}    ->  %{bm['pan_flip_pct']:.2f}")
    print(f"   - Tilt Flip Orani   : %{cur['tilt_flip_pct']:.2f}    ->  %{bm['tilt_flip_pct']:.2f}")
    print(f"   - Pan Jerk (deg)    : {cur['pan_jerk_deg']:.4f}   ->  {bm['pan_jerk_deg']:.4f}")
    print(f"   - Tilt Jerk (deg)   : {cur['tilt_jerk_deg']:.4f}   ->  {bm['tilt_jerk_deg']:.4f}")
    print("=" * 76)

    out_path = Path(__file__).resolve().parent / "p_optuna_results.json"
    result_data = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "plant": {
            "backlash_deg": CONFIG.actuator.backlash_deg,
            "latency_frames": 2,
            "servo_tau_s": CONFIG.mount.servo_tau,
        },
        "best_trial_number": best.number,
        "best_cost": best.value,
        "best_params": best.params,
        "metrics_comparison": {
            "baseline": cur,
            "optimized": bm,
        },
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result_data, f, indent=2)
    print(f"Sonuclar kaydedildi: {out_path}")


if __name__ == "__main__":
    main()
