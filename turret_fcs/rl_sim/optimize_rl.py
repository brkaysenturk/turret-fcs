"""
================================================================================
MODUL: BAYESIAN OPTIMIZATION ILE HIPERPARAMETRE OPTIMIZASYONU (OPTUNA TPE)
================================================================================
Bu modul, TurretEnv odul agirliklarini (w_dist, w_dist_sq, w_bullseye, w_lock,
w_smooth) ve PPO ogrenme oranini (learning_rate) Tree-structured Parzen Estimator
(TPE) Bayesian optimizasyon algoritmasiyla arastirir.

Hedef Fonksiyonu (Objektif):
- Hedef takip piksel hatasini MINIMIZE et.
- Motor sarsintisini / jerk (eylem turevi) MINIMIZE et.
- Vizor kilitlenme oranini (Lock Ratio) MAKSIMIZE et.
================================================================================
"""

import json
import time
from pathlib import Path
import numpy as np
import optuna
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from turret_env import TurretEnv

# CPU asiri isinmasini onlemek icin 2-thread kisiti
torch.set_num_threads(2)


def evaluate_agent(model, test_seeds=[101, 202, 303, 404, 505], max_steps=300):
    """
    Egitilmis ajani 5 farkli zorlu yorunge tohumunda test eder.
    Metrikler: Ortalama piksel hatasi, vizor kilidi orani, servo jerk/sarsinti.
    """
    total_errors = []
    lock_steps = 0
    total_steps = 0
    total_jerk = []

    for seed in test_seeds:
        eval_env = TurretEnv(render_mode=None, latency_steps=2)
        obs, _ = eval_env.reset(seed=seed)
        prev_act = np.zeros(2, dtype=np.float32)

        for _ in range(max_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = eval_env.step(action)

            # Piksel hatasi (1280x720 ekranda merkezden uzaklik)
            err_norm = math_dist = float(info.get("pixel_dist", 0.0))
            err_px = err_norm * (eval_env.screen_w / 2.0)
            total_errors.append(err_px)

            # Eger hedef %5 vizor ici alandaysa kilitli sayilir
            if err_norm < 0.05:
                lock_steps += 1
            total_steps += 1

            # Motor sarsinti/jerk olcumu
            jerk = float(np.sum((action - prev_act) ** 2))
            total_jerk.append(jerk)
            prev_act = action.copy()

            if terminated or truncated:
                break

    mean_err_px = float(np.mean(total_errors)) if total_errors else 500.0
    lock_ratio = float(lock_steps / max(1, total_steps))
    mean_jerk = float(np.mean(total_jerk)) if total_jerk else 1.0

    return mean_err_px, lock_ratio, mean_jerk


def objective(trial):
    # 1. Bayesian Optimizasyon tarafindan orneklenen aday parametreler
    w_dist = trial.suggest_float("w_dist", 1.5, 6.0, step=0.5)
    w_dist_sq = trial.suggest_float("w_dist_sq", 2.0, 8.0, step=0.5)
    w_bullseye = trial.suggest_float("w_bullseye", 3.0, 10.0, step=0.5)
    w_lock = trial.suggest_float("w_lock", 2.0, 8.0, step=0.5)
    w_smooth = trial.suggest_float("w_smooth", 0.10, 0.50, step=0.05)
    lr = trial.suggest_float("learning_rate", 1.5e-4, 5.0e-4, log=True)

    def env_fn():
        return TurretEnv(
            render_mode=None,
            latency_steps=2,
            w_dist=w_dist,
            w_dist_sq=w_dist_sq,
            w_bullseye=w_bullseye,
            w_lock=w_lock,
            w_smooth=w_smooth,
        )

    vec_env = DummyVecEnv([env_fn])

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = PPO(
        policy="MlpPolicy",
        env=vec_env,
        learning_rate=lr,
        n_steps=1024,
        batch_size=64,
        n_epochs=6,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.003,
        verbose=0,
        device=device,
    )

    # Hizli ve etkin Bayesian degerlendirme adimi (16.000 adim / deneme)
    TRAIN_STEPS = 16_000
    model.learn(total_timesteps=TRAIN_STEPS)

    # 5 referans tohumda degerlendir
    mean_err_px, lock_ratio, mean_jerk = evaluate_agent(model)

    # Birlesik Maliyet Fonksiyonu (Cost = Minimization Goal)
    # Dusuk piksel hatasi (+), Dusuk motor sarsintisi (+), Yuksek kilitlenme orani (-)
    cost = (mean_err_px) + (25.0 * mean_jerk) - (40.0 * lock_ratio)

    trial.set_user_attr("mean_err_px", mean_err_px)
    trial.set_user_attr("lock_ratio", lock_ratio)
    trial.set_user_attr("mean_jerk", mean_jerk)

    return cost


def run_optimization(n_trials=10):
    print("==================================================================")
    print(" TURRET FCS // BAYESIAN OPTIMIZATION (OPTUNA TPE) MOTORU         ")
    print("==================================================================")
    print(f"[Optuna] Planlanan Deneme Sayisi: {n_trials}")
    print("[Hedef] Minimasyon: Ortalama Piksel Hatasi + 25*Jerk - 40*Kilitlenme")
    print("------------------------------------------------------------------")

    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(direction="minimize", sampler=sampler)

    start_t = time.time()
    study.optimize(objective, n_trials=n_trials, show_progress_bar=False)
    elapsed = time.time() - start_t

    best = study.best_trial
    print("\n==================================================================")
    print(" [OK] BAYESIAN OPTIMIZASYON TAMAMLANDI!")
    print(f" Toplam Sure: {elapsed:.1f} saniye")
    print(f" En Iyi Deneme Numarasi: #{best.number}")
    print(f" En Iyi Maliyet (Score): {best.value:.2f}")
    print("------------------------------------------------------------------")
    print(" EN IYI HIPERPARAMETRELER:")
    for k, v in best.params.items():
        if isinstance(v, float):
            print(f"   - {k:15s}: {v:.5f}")
        else:
            print(f"   - {k:15s}: {v}")
    print("------------------------------------------------------------------")
    print(" PERFORMANS METRIKLERI:")
    print(f"   - Ortalama Piksel Hatasi : {best.user_attrs['mean_err_px']:.1f} px")
    print(f"   - Vizor Kilitlenme Orani : %{best.user_attrs['lock_ratio'] * 100:.1f}")
    print(f"   - Motor Jerk / Sarsinti  : {best.user_attrs['mean_jerk']:.4f}")
    print("==================================================================")

    # Sonuclari JSON dosyasina kaydet
    out_path = Path(__file__).resolve().parent / "best_params.json"
    result_data = {
        "best_trial_number": best.number,
        "best_score": float(best.value),
        "params": best.params,
        "metrics": {
            "mean_err_px": float(best.user_attrs["mean_err_px"]),
            "lock_ratio": float(best.user_attrs["lock_ratio"]),
            "mean_jerk": float(best.user_attrs["mean_jerk"]),
        },
        "elapsed_seconds": elapsed,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result_data, f, indent=4)
    print(f"[Kaydedildi] {out_path.name}")

    return best.params


if __name__ == "__main__":
    run_optimization(n_trials=10)
