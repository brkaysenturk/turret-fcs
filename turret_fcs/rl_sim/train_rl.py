import os
import time
from pathlib import Path
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from turret_env import TurretEnv


def make_env():
    # latency_steps=2: Gercek donanimin ~70-80 ms tasima gecikmesi (kamera + YOLO + UART)
    env = TurretEnv(render_mode=None, latency_steps=2)
    return Monitor(env)


def train():
    current_dir = Path(__file__).resolve().parent
    models_dir = current_dir / "models"
    logs_dir = current_dir / "logs"
    models_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    # Islemci asiri isinmasini (thermal throttle / kapanma) onlemek icin thread limit
    torch.set_num_threads(2)

    print("==================================================")
    print(" TURRET FCS // GECIKME TAMPONLU PPO RL EGITIMI   ")
    print("==================================================")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[Device] Donanım: {device.upper()} (Guvenli 2-Thread Modu)")

    env = DummyVecEnv([make_env])

    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=3e-4,
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

    TOTAL_TIMESTEPS = 150_000
    print(f"\n[Eğitim Başlıyor] Toplam Adım: {TOTAL_TIMESTEPS:,}...")
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
