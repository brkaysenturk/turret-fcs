import math
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Proje ana dizinini ekle (p_controller import icin)
current_dir = Path(__file__).resolve().parent
parent_dir = current_dir.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

from stable_baselines3 import PPO
from p_controller import PController
from turret_env import TurretEnv


def run_comparison():
    model_path = current_dir / "models" / "turret_ppo_final.zip"
    if not model_path.exists():
        print(f"[HATA] Model bulunamadi: {model_path}")
        print("Lutfen once 'train_rl.py' ile modeli egitin.")
        return

    print("==================================================")
    print("  TURRET FCS // P CONTROLLER vs. RL KARŞILAŞTIRMA TESTİ   ")
    print("==================================================")

    model = PPO.load(str(model_path))

    # Test Senaryoları (Her ikisi de birebir aynı tohum/harekette test edilir)
    seeds = [42, 100, 2026]
    scenario_names = ["Zikzak Manevrası", "Doğrusal Kaçış", "Dinamik Evasive"]

    fig, axes = plt.subplots(len(seeds), 1, figsize=(10, 10), sharex=True)
    if len(seeds) == 1:
        axes = [axes]

    pid_total_errors = []
    rl_total_errors = []

    for idx, (seed, title) in enumerate(zip(seeds, scenario_names)):
        # 1. P Kontrolcu Testi
        env_pid = TurretEnv()
        obs_pid, _ = env_pid.reset(seed=seed)
        p_ctrl = PController(kp_pan=0.35, kp_tilt=0.35)

        pid_errors = []
        for _ in range(env_pid.max_steps):
            # Piksel sapmasini hesaplar
            px, py = env_pid.last_pixel_pos
            err_x = px - env_pid.center_x
            err_y = py - env_pid.center_y
            dist_px = math.hypot(err_x, err_y)
            pid_errors.append(dist_px)

            # Basit oransal kontrol eylemi
            action_pid = np.array(
                [
                    np.clip(err_x / (env_pid.screen_w / 4.0), -1.0, 1.0),
                    np.clip(-err_y / (env_pid.screen_h / 4.0), -1.0, 1.0),
                ],
                dtype=np.float32,
            )
            obs_pid, _, term, trunc, _ = env_pid.step(action_pid)
            if term or trunc:
                break

        # 2. RL (PPO) Testi (Birebir Ayni Seed)
        env_rl = TurretEnv()
        obs_rl, _ = env_rl.reset(seed=seed)

        rl_errors = []
        for _ in range(env_rl.max_steps):
            px, py = env_rl.last_pixel_pos
            dist_px = math.hypot(px - env_rl.center_x, py - env_rl.center_y)
            rl_errors.append(dist_px)

            action_rl, _ = model.predict(obs_rl, deterministic=True)
            obs_rl, _, term, trunc, _ = env_rl.step(action_rl)
            if term or trunc:
                break

        pid_total_errors.append(np.mean(pid_errors))
        rl_total_errors.append(np.mean(rl_errors))

        # Grafige Ciz
        time_axis = np.linspace(0, len(pid_errors) * env_pid.dt, len(pid_errors))
        ax = axes[idx]
        ax.plot(time_axis, pid_errors, label=f"P Kontrolcü (Ort Hata: {np.mean(pid_errors):.1f}px)", color="crimson", lw=1.8, ls="--")
        ax.plot(time_axis[:len(rl_errors)], rl_errors, label=f"PPO - RL Ajanı (Ort Hata: {np.mean(rl_errors):.1f}px)", color="teal", lw=2.0)
        ax.axhline(30, color="gray", linestyle=":", alpha=0.7, label="Hedef Kilit Alanı (30px)")
        ax.set_title(f"Senaryo {idx+1}: {title} (Seed: {seed})", fontweight="bold", fontsize=11)
        ax.set_ylabel("Piksel Hatası (px)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

    axes[-1].set_xlabel("Zaman (Saniye)", fontsize=11)
    plt.tight_layout()

    output_plot_path = current_dir / "pid_vs_rl_comparison.png"
    plt.savefig(str(output_plot_path), dpi=300)
    plt.close()

    print("\n==================================================")
    print("               TEST SONUÇ ÖZETİ                   ")
    print("==================================================")
    print(f"P Kontrolcü Ortalama Takip Hatası : {np.mean(pid_total_errors):.2f} px")
    print(f"PPO (RL)   Ortalama Takip Hatası : {np.mean(rl_total_errors):.2f} px")
    improvement = (np.mean(pid_total_errors) - np.mean(rl_total_errors)) / np.mean(pid_total_errors) * 100.0
    print(f"RL Performans İyileştirmesi      : %{improvement:+.1f}")
    print(f"\n[OK] Rapor ve Tez Grafiği Kaydedildi: {output_plot_path}")
    print("==================================================")


if __name__ == "__main__":
    run_comparison()
