"""
================================================================================
MODUL: BAYESIAN OPTIMIZATION ILE ODUL HIPERPARAMETRE TARAMASI (OPTUNA TPE)
================================================================================
Bu modul, yuksek sadakatli TurretEnv ortaminda (2. derece eylemsizlik,
Webots harmonik YOLO salinimi, 0.10 derece kuantalama ve sakinlik sartli odul)
3 temel parametreyi:
1. lambda_u: Kontrol cabasi (effort / u^2) agirligi (serbest)
2. mu_du: Eylem sarsintisi / jerk (du^2) agirligi (serbest: 0.005 - 0.15)
3. learning_rate: PPO baslangic ogrenme orani (lineer takvim)
150.000 adim/deneme ufkunda optimize eder ve rezonans kapili (flip_rate + jerk)
hakem fonksiyonuyla degerlendirerek best_params.json dosyasina yazar.
================================================================================
"""

import json
import sys
import time
from pathlib import Path

current_dir = Path(__file__).resolve().parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

import numpy as np
import optuna
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from turret_env import TurretEnv
from turret_fcs.config import CONFIG

# CPU termal korumasi icin 2-thread kisiti
torch.set_num_threads(2)

EVAL_SEEDS = list(range(100, 120))
SCREEN_STEPS = 150_000


def _sign_flip_rate(actions_1d):
    arr = np.asarray(actions_1d, dtype=np.float32)
    moves = arr[np.abs(arr) > 1e-4]
    if moves.size < 3:
        return 0.0
    return float(np.mean(np.sign(moves[1:]) != np.sign(moves[:-1])) * 100.0)


def evaluate_agent(model, test_seeds=EVAL_SEEDS, max_steps=300):
    """
    Egitilmis modeli 20 sabit tohumda (calisma zamani gibi use_velocity_obs=False ile)
    degerlendirir: Medyan hata, IQR, Kilit orani, Medyan Jerk ve Yön Degistirme (%).
    """
    total_errors = []
    lock_steps = 0
    total_steps = 0
    total_jerk = []
    flip_rates = []

    for seed in test_seeds:
        eval_env = TurretEnv(
            render_mode=None,
            latency_steps=2,
            use_velocity_obs=False,
            enable_randomization=False,
        )
        obs, _ = eval_env.reset(seed=seed)
        prev_act = np.zeros(2, dtype=np.float32)
        ep_actions = []

        for _ in range(max_steps):
            action, _ = model.predict(obs, deterministic=True)
            action = np.asarray(action, dtype=np.float32)
            ep_actions.append(action)
            obs, _, terminated, truncated, info = eval_env.step(action)

            err_norm = float(info.get("pixel_dist", 0.0))
            err_px = err_norm * (eval_env.screen_w / 2.0)
            total_errors.append(err_px)

            if err_norm < 0.05:
                lock_steps += 1
            total_steps += 1

            jerk = float(np.mean(np.abs(action - prev_act)))
            total_jerk.append(jerk)
            prev_act = action.copy()

            if terminated or truncated:
                break

        a_mat = np.asarray(ep_actions, dtype=np.float32)
        flip_rates.append(0.5 * (_sign_flip_rate(a_mat[:, 0]) + _sign_flip_rate(a_mat[:, 1])))

    median_err_px = float(np.median(total_errors)) if total_errors else 500.0
    if total_errors:
        q75, q25 = np.percentile(total_errors, [75, 25])
        iqr_err_px = float(q75 - q25)
    else:
        iqr_err_px = 200.0

    lock_ratio = float(lock_steps / max(1, total_steps))
    median_jerk = float(np.median(total_jerk)) if total_jerk else 1.0
    mean_flip_pct = float(np.mean(flip_rates)) if flip_rates else 10.0

    return median_err_px, iqr_err_px, lock_ratio, median_jerk, mean_flip_pct


def objective(trial):
    # mu_du serbest (0.005 - 0.20 araliginda, 1.61 deg backlash sönümü)
    lambda_u = trial.suggest_float("lambda_u", 0.008, 0.08, log=True)
    mu_du = trial.suggest_float("mu_du", 0.005, 0.20, log=True)
    lr = trial.suggest_float("learning_rate", 1.5e-4, 4.2e-4, log=True)

    def env_fn():
        return TurretEnv(
            render_mode=None,
            latency_steps=2,
            lambda_u=lambda_u,
            mu_du=mu_du,
            progress_gain=2.0,
            error_gain=0.05,
            backlash_deg=CONFIG.actuator.backlash_deg,
            use_velocity_obs=True,
            enable_randomization=True,
        )

    vec_env = DummyVecEnv([env_fn])

    model = PPO(
        policy="MlpPolicy",
        env=vec_env,
        learning_rate=lambda progress: max(5e-5, lr * progress),
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.003,
        verbose=0,
        device="cpu",
    )

    t0 = time.time()
    model.learn(total_timesteps=SCREEN_STEPS)
    median_err_px, iqr_err_px, lock_ratio, median_jerk, mean_flip_pct = evaluate_agent(model)

    # Rezonans / Limit-Cycle Hakem Kriteri:
    # Yon degistirme %1.60'i veya sarsinti 0.024'u asarsa karesel rezonans cezasi uygulanir.
    flip_excess = max(0.0, mean_flip_pct - 1.60)
    jerk_excess = max(0.0, median_jerk - 0.024)
    resonance_penalty = 80.0 * (flip_excess ** 2) + 2500.0 * (jerk_excess ** 2)

    cost = (
        median_err_px
        + 1.0 * iqr_err_px
        + 300.0 * median_jerk
        + 8.0 * mean_flip_pct
        + resonance_penalty
        - 55.0 * lock_ratio
    )

    trial.set_user_attr("median_err_px", median_err_px)
    trial.set_user_attr("iqr_err_px", iqr_err_px)
    trial.set_user_attr("lock_ratio", lock_ratio)
    trial.set_user_attr("median_jerk", median_jerk)
    trial.set_user_attr("mean_flip_pct", mean_flip_pct)

    print(
        f"[Trial #{trial.number:02d}] ({time.time() - t0:.1f}s) "
        f"cost={cost:.2f} | med_err={median_err_px:.1f}px | iqr={iqr_err_px:.1f}px | "
        f"lock={lock_ratio * 100:.1f}% | flip={mean_flip_pct:.2f}% | jerk={median_jerk:.4f} | "
        f"lambda_u={lambda_u:.4f}, mu_du={mu_du:.4f}, lr={lr:.5f}",
        flush=True,
    )
    return cost


def run_optimization(n_trials=14):
    current_dir = Path(__file__).resolve().parent
    db_path = current_dir / "optuna_rl.db"
    storage_url = f"sqlite:///{db_path.as_posix()}"

    print("==================================================================", flush=True)
    print(f" TURRET FCS // REZONANS KORUMALI OPTUNA TPE TARAMASI ({n_trials} x 150K)   ", flush=True)
    print("==================================================================", flush=True)
    print(f"[Optuna DB] {db_path.name}", flush=True)
    print(f"[Deneme] {n_trials} deneme x {SCREEN_STEPS:,} adim (mu_du serbest: 0.005..0.15)", flush=True)
    print(f"[Referans Tohumlar] 20 Tohum: {EVAL_SEEDS[0]}..{EVAL_SEEDS[-1]}", flush=True)
    print("------------------------------------------------------------------", flush=True)

    sampler = optuna.samplers.TPESampler(seed=42, n_startup_trials=5, multivariate=True)
    study = optuna.create_study(
        study_name="turret_v21_resonance_guarded_150k",
        storage=storage_url,
        direction="minimize",
        sampler=sampler,
        load_if_exists=True,
    )

    if len(study.trials) == 0:
        study.enqueue_trial(
            {
                "lambda_u": 0.02,
                "mu_du": 0.08,
                "learning_rate": 0.0003,
            }
        )

    start_t = time.time()
    study.optimize(objective, n_trials=n_trials, show_progress_bar=False)
    elapsed = time.time() - start_t

    best = study.best_trial
    print("\n==================================================================", flush=True)
    print(" [OK] ODUL HIPERPARAMETRE TARAMASI TAMAMLANDI!", flush=True)
    print(f" Toplam Sure: {elapsed:.1f} saniye", flush=True)
    print(f" En Iyi Deneme Numarasi: #{best.number}", flush=True)
    print(f" En Iyi Maliyet (Score): {best.value:.2f}", flush=True)
    print("------------------------------------------------------------------", flush=True)
    print(" EN IYI ODUL & POLITIKA PARAMETRELERI:", flush=True)
    for k, v in best.params.items():
        print(f"   - {k:15s}: {v:.6f}" if isinstance(v, float) else f"   - {k:15s}: {v}", flush=True)
    print("------------------------------------------------------------------", flush=True)
    print(" PERFORMANS METRIKLERI (20 SEED MEDYAN & IQR):", flush=True)
    print(f"   - Medyan Piksel Hatasi   : {best.user_attrs['median_err_px']:.2f} px", flush=True)
    print(f"   - Piksel Hata IQR        : {best.user_attrs['iqr_err_px']:.2f} px", flush=True)
    print(f"   - Vizor Kilitlenme Orani : %{best.user_attrs['lock_ratio'] * 100:.2f}", flush=True)
    print(f"   - Yon Degistirme Orani   : %{best.user_attrs['mean_flip_pct']:.2f}", flush=True)
    print(f"   - Medyan Jerk / Titresim : {best.user_attrs['median_jerk']:.5f}", flush=True)
    print("==================================================================", flush=True)

    out_path = current_dir / "best_params.json"
    result_data = {
        "_comment": f"Optuna TPE optimization result ({SCREEN_STEPS:,} timesteps per trial, high-fidelity TurretEnv with resonance guard). Automatically consumed by train_rl.py.",
        "version": "2.0",
        "timesteps_per_trial": SCREEN_STEPS,
        "best_trial_number": best.number,
        "best_score": float(best.value),
        "params": best.params,
        "metrics": {
            "median_err_px": float(best.user_attrs["median_err_px"]),
            "iqr_err_px": float(best.user_attrs["iqr_err_px"]),
            "lock_ratio": float(best.user_attrs["lock_ratio"]),
            "mean_flip_pct": float(best.user_attrs["mean_flip_pct"]),
            "median_jerk": float(best.user_attrs["median_jerk"]),
        },
        "elapsed_seconds": elapsed,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result_data, f, indent=4)
    print(f"[Kaydedildi] {out_path.name}", flush=True)

    return best.params


if __name__ == "__main__":
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 14
    run_optimization(n_trials=trials)
