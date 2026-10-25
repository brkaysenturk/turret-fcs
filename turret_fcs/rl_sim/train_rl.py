"""
================================================================================
MODUL: PPO EGITIM MOTORU (PROGRESS REWARD / v2.0)
================================================================================
TurretEnv uzerinde saf PPO politikasi egitir. v2.0'da odul, LQR cezasindan
progress formuna cevrildi:

    r = k_p * (e_prev^2 - e^2) - k_e * e^2
        - lambda_u * |u|^2 - mu_du * |du|^2 + bullseye

Plant modeli: 1.2 derece disli boslugu (komut biriktirici + backlash),
1.5 derece/kare pan-tilt adim siniri, 0.10 derece servo deadband,
domain randomization (gecikme, servo tau, YOLO gurultusu, FPS).
================================================================================
"""

import sys
import time
from pathlib import Path

current_dir = Path(__file__).resolve().parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from turret_env import TurretEnv


def make_env():
    return Monitor(
        TurretEnv(
            render_mode=None,
            latency_steps=2,
            lambda_u=0.02,
            mu_du=0.08,
            progress_gain=2.0,
            error_gain=0.05,
            backlash_deg=1.20,
            use_velocity_obs=True,
            enable_randomization=True,
        )
    )


def train(timesteps=250_000):
    models_dir = current_dir / "models"
    logs_dir = current_dir / "logs"
    models_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    torch.set_num_threads(2)
    print("==================================================")
    print(" TURRET FCS // SAF PPO + PROGRESS REWARD (v2.0)  ")
    print("==================================================")
    print("[Plant] backlash=1.20 deg, max_step=1.50 deg, deadband=0.10 deg")
    print("[Reward] progress=2.0, error=0.05, lambda_u=0.02, mu_du=0.08")

    env = DummyVecEnv([make_env])
    model = PPO(
        policy="MlpPolicy",
        env=env,
        learning_rate=lambda progress: max(5e-5, 3e-4 * progress),
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.003,
        verbose=1,
        tensorboard_log=str(logs_dir),
        device="cpu",
    )

    start = time.time()
    model.learn(total_timesteps=timesteps, progress_bar=False)
    output = models_dir / "turret_ppo_v14.zip"
    model.save(str(output))
    print(f"[OK] {timesteps:,} adim, {time.time() - start:.1f} s -> {output}")


if __name__ == "__main__":
    train(int(sys.argv[1]) if len(sys.argv) > 1 else 250_000)
