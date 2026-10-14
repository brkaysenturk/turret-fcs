"""
================================================================================
MODUL: PPO PEKISDIRMELI OGRENME EGITIM MOTORU (TRAIN RL / SB3 PPO)
================================================================================
Bu modul, TurretEnv benzetim ortami uzerinde Stable-Baselines3 PPO algoritmasiyla
otonom taret ajanini egitir, odul egrilerini kaydeder ve 'turret_ppo_final.zip'
modelini uretir.

EGITIM VE SISTEM GUVENLIGI PARAMETRELERI:
1. Donanim Termal Korumasi: torch.set_num_threads(2) kisitlamasi ile dizustu
   bilgisayarin asiri isinmasi ve termal kapanmasi engellenir.
2. Politika Hiperparametreleri:
   - learning_rate = 3e-4, n_steps = 1024, batch_size = 64, n_epochs = 10
   - gamma = 0.99, gae_lambda = 0.95, clip_range = 0.2
3. Gecikme Telafili Egitim: Ajan 80 ms tasiyici gecikmesi altinda hedefin yonunu
   tahmin etmeyi ogrenir ve hedefi vizor merkezinde tutarak +2000 uzeri odul toplar.
================================================================================
"""

import json
import os
import time
from pathlib import Path
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from turret_env import TurretEnv


def load_best_params(params_path):
    if params_path.exists():
        try:
            with open(params_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                return data.get("params", {})
        except Exception:
            pass
    return {}


def make_env(best_params=None):
    best_params = best_params or {}
    w_dist = best_params.get("w_dist", 3.5)
    w_dist_sq = best_params.get("w_dist_sq", 5.0)
    w_bullseye = best_params.get("w_bullseye", 6.0)
    w_lock = best_params.get("w_lock", 4.0)
    w_smooth = best_params.get("w_smooth", 1.0)

    env = TurretEnv(
        render_mode=None,
        latency_steps=2,
        w_dist=w_dist,
        w_dist_sq=w_dist_sq,
        w_bullseye=w_bullseye,
        w_lock=w_lock,
        w_smooth=w_smooth,
        enable_randomization=True,
    )
    return Monitor(env)


def train():
    current_dir = Path(__file__).resolve().parent
    models_dir = current_dir / "models"
    logs_dir = current_dir / "logs"
    params_file = current_dir / "best_params.json"
    models_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    # Islemci asiri isinmasini (thermal throttle / kapanma) onlemek icin thread limit
    torch.set_num_threads(2)

    best_params = load_best_params(params_file)

    print("==================================================")
    print(" TURRET FCS // BAYESIAN OPTIMIZE EDILMIS PPO EGITIMI")
    print("==================================================")
    if best_params:
        print("[Optuna] En Iyi Parametreler Dosyadan Yuklendi:")
        for k, v in best_params.items():
            print(f"  - {k}: {v}")
    else:
        print("[Bilgi] Standart Bryson parametreleri kullaniliyor.")

    # MlpPolicy ve vektor ortamlari icin CPU GPU'dan cok daha hizlidir (CUDA gecikmesi olmadan)
    device = "cpu"
    print(f"[Device] Donanim: {device.upper()} (Guvenli 2-Thread Modu)")

    env = DummyVecEnv([lambda: make_env(best_params)])

    base_lr = float(best_params.get("learning_rate", 3.97e-4))

    # Lineer ogrenme orani sonumleme (fine-tuning asamasinda motor titresimini sifirlar)
    def lr_schedule(progress_remaining):
        return max(5e-5, base_lr * progress_remaining)

    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=lr_schedule,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.003,
        verbose=1,
        tensorboard_log=str(logs_dir),
        device=device,
    )

    TOTAL_TIMESTEPS = 250_000
    print(f"\n[Eğitim Başlıyor] Toplam Adım: {TOTAL_TIMESTEPS:,} (Derin Yakınsama Modu)...")
    start_time = time.time()

    model.learn(
        total_timesteps=TOTAL_TIMESTEPS,
        progress_bar=False,
    )

    training_duration = time.time() - start_time
    final_model_path = models_dir / "turret_ppo_final.zip"
    model.save(str(final_model_path))

    print("\n==================================================")
    print(f"[OK] Egitim Tamamlandi! Sure: {training_duration:.1f} saniye")
    print(f"[OK] Yeni Guclendirilmis Model Kaydedildi: {final_model_path}")
    print("==================================================")


if __name__ == "__main__":
    train()
