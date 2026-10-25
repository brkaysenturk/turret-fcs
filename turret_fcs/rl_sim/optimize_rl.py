"""
================================================================================
MODUL: BAYESIAN OPTIMIZATION ILE ODUL HIPERPARAMETRE TARAMASI (OPTUNA TPE)
================================================================================
Bu modul, TurretEnv ortaminda (v2.0 progress odulu) 3 temel parametreyi:
1. lambda_u: Kontrol cabasi (effort / u^2) agirligi
2. mu_du: Eylem sarsintisi / jerk (du^2) agirligi
3. learning_rate: PPO ogrenme orani
20 sabit test tohumu uzerinde medyan + IQR (Interquartile Range) metrigiyle
ve SQLite veritabani kaliciligi ile optimize eder.
NOT: Bu hizli tarama araci (10k adim/deneme) uretim egitimini (train_rl.py)
otomatik beslemez; train_rl.py sabit lambda_u=0.02, mu_du=0.08 kullanir.
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

# CPU termal korumasi icin 2-thread kisiti
torch.set_num_threads(2)

# Deterministik degerlendirme tohumlari (20 adet sabit seed)
EVAL_SEEDS = list(range(100, 120))


def evaluate_agent(model, test_seeds=EVAL_SEEDS, max_steps=300):
    """
    Egitilmis modeli 20 sabit tohumda test eder.
    Metrikler: Medyan piksel hatasi, Hata IQR (yayilim), Kilit orani, Medyan Jerk.
    """
    total_errors = []
    lock_steps = 0
    total_steps = 0
    total_jerk = []

    for seed in test_seeds:
        eval_env = TurretEnv(render_mode=None, latency_steps=2, enable_randomization=True)
        obs, _ = eval_env.reset(seed=seed)
        prev_act = np.zeros(2, dtype=np.float32)

        for _ in range(max_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = eval_env.step(action)

            # Piksel hatasi (1280x720 ekranda merkezden uzaklik)
            err_norm = float(info.get("pixel_dist", 0.0))
            err_px = err_norm * (eval_env.screen_w / 2.0)
            total_errors.append(err_px)

            if err_norm < 0.05:
                lock_steps += 1
            total_steps += 1

            # Motor sarsinti/jerk olcumu
            jerk = float(np.sum((action - prev_act) ** 2))
            total_jerk.append(jerk)
            prev_act = action.copy()

            if terminated or truncated:
                break

    if total_errors:
        median_err_px = float(np.median(total_errors))
        q75, q25 = np.percentile(total_errors, [75, 25])
        iqr_err_px = float(q75 - q25)
    else:
        median_err_px = 500.0
        iqr_err_px = 200.0

    lock_ratio = float(lock_steps / max(1, total_steps))
    median_jerk = float(np.median(total_jerk)) if total_jerk else 1.0

    return median_err_px, iqr_err_px, lock_ratio, median_jerk


def objective(trial):
    # 3 Temel LQR & PPO Skaleri
    lambda_u = trial.suggest_float("lambda_u", 0.01, 0.15, log=True)
    mu_du = trial.suggest_float("mu_du", 0.005, 0.10, log=True)
    lr = trial.suggest_float("learning_rate", 1.5e-4, 5.0e-4, log=True)

    def env_fn():
        return TurretEnv(
            render_mode=None,
            latency_steps=2,
            lambda_u=lambda_u,
            mu_du=mu_du,
            enable_randomization=True,
        )

    vec_env = DummyVecEnv([env_fn])

    model = PPO(
        policy="MlpPolicy",
        env=vec_env,
        learning_rate=lr,
        n_steps=1024,
        batch_size=64,
        n_epochs=5,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.003,
        verbose=0,
        device="cpu",
    )

    # Hizli ve etkin tarama adimi (10.000 adim / deneme)
    SCREEN_STEPS = 10_000
    model.learn(total_timesteps=SCREEN_STEPS)

    # 20 referans tohumda degerlendir
    median_err_px, iqr_err_px, lock_ratio, median_jerk = evaluate_agent(model)

    # Dayanıklı Maliyet Fonksiyonu (Robust Cost Function):
    # Düşük medyan hata + düşük yayılım (IQR) + düşük jerk - yüksek kilit oranı
    cost = median_err_px + 1.0 * iqr_err_px + 25.0 * median_jerk - 50.0 * lock_ratio

    trial.set_user_attr("median_err_px", median_err_px)
    trial.set_user_attr("iqr_err_px", iqr_err_px)
    trial.set_user_attr("lock_ratio", lock_ratio)
    trial.set_user_attr("median_jerk", median_jerk)

    return cost


def run_optimization(n_trials=12):
    current_dir = Path(__file__).resolve().parent
    db_path = current_dir / "optuna_rl.db"
    storage_url = f"sqlite:///{db_path.as_posix()}"

    print("==================================================================")
    print(" TURRET FCS // ODUL HIPERPARAMETRE TARAMASI (OPTUNA TPE) MOTORU    ")
    print("==================================================================")
    print(f"[Optuna DB] {db_path.name}")
    print(f"[Deneme] Planlanan Ek Deneme Sayisi: {n_trials}")
    print(f"[Referans Tohumlar] 20 Tohum: {EVAL_SEEDS[0]}..{EVAL_SEEDS[-1]}")
    print("------------------------------------------------------------------")

    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(
        study_name="turret_lqr_study",
        storage=storage_url,
        direction="minimize",
        sampler=sampler,
        load_if_exists=True,
    )

    start_t = time.time()
    study.optimize(objective, n_trials=n_trials, show_progress_bar=False)
    elapsed = time.time() - start_t

    best = study.best_trial
    print("\n==================================================================")
    print(" [OK] ODUL HIPERPARAMETRE TARAMASI TAMAMLANDI!")
    print(f" Toplam Sure: {elapsed:.1f} saniye")
    print(f" En Iyi Deneme Numarasi: #{best.number}")
    print(f" En Iyi Maliyet (Score): {best.value:.2f}")
    print("------------------------------------------------------------------")
    print(" EN IYI ODUL & POLITIKA PARAMETRELERI:")
    for k, v in best.params.items():
        if isinstance(v, float):
            print(f"   - {k:15s}: {v:.6f}")
        else:
            print(f"   - {k:15s}: {v}")
    print("------------------------------------------------------------------")
    print(" PERFORMANS METRIKLERI (20 SEED MEDYAN & IQR):")
    print(f"   - Medyan Piksel Hatasi   : {best.user_attrs['median_err_px']:.1f} px")
    print(f"   - Piksel Hata IQR        : {best.user_attrs['iqr_err_px']:.1f} px")
    print(f"   - Vizor Kilitlenme Orani : %{best.user_attrs['lock_ratio'] * 100:.1f}")
    print(f"   - Medyan Jerk / Titresim : {best.user_attrs['median_jerk']:.5f}")
    print("==================================================================")

    # Sonuclari JSON dosyasina kaydet
    out_path = current_dir / "best_params.json"
    result_data = {
        "_comment": "Optuna screening result (10,000 timesteps per trial). Informational only; the production model uses fixed reward weights from train_rl.py.",
        "version": "2.0",
        "best_trial_number": best.number,
        "best_score": float(best.value),
        "params": best.params,
        "metrics": {
            "median_err_px": float(best.user_attrs["median_err_px"]),
            "iqr_err_px": float(best.user_attrs["iqr_err_px"]),
            "lock_ratio": float(best.user_attrs["lock_ratio"]),
            "median_jerk": float(best.user_attrs["median_jerk"]),
        },
        "elapsed_seconds": elapsed,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result_data, f, indent=4)
    print(f"[Kaydedildi] {out_path.name}")

    return best.params


if __name__ == "__main__":
    run_optimization(n_trials=12)
