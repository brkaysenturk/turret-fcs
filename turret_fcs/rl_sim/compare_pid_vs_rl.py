"""
================================================================================
MODUL: P KONTROLCU vs. REINFORCEMENT LEARNING (PPO) KARSILASTIRMA BENZETIMI
================================================================================
Bu betik, 80 ms tasiyici gecikmesi (Transport Delay) altinda calisan otonom taretin
klasik Hız-Uyarlamalı P Kontrolcüsü ile PPO Derin Takviyeli Öğrenme Ajanının (RL)
takip başarımını 3 kritik manevra senaryosunda (Zikzak, Doğrusal Kaçış, Evasive)
birebir aynı tohum (seed) ve dinamik koşullarda karşılaştırır ve tez grafiği üretir.
================================================================================
"""

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

    print("==================================================================")
    print("  YILDIZ TEKNIK UNIVERSITESI - KONTROL VE OTOMASYON MUHENDISLIGI   ")
    print("      TURRET FCS // P CONTROLLER vs. RL KARSILASTIRMA BENZETIMI    ")
    print("==================================================================")

    model = PPO.load(str(model_path), device="cpu")

    # Test Senaryolari (Her iki kontrolcu de birebir ayni tohum ve yorungede test edilir)
    seeds = [42, 100, 2026]
    scenario_names = ["Zikzak Manevrasi", "Dogrusal Kacis", "Dinamik Evasive (Kacis)"]

    fig, axes = plt.subplots(len(seeds), 1, figsize=(11, 10), sharex=True)
    if len(seeds) == 1:
        axes = [axes]

    p_total_errors = []
    rl_total_errors = []

    for idx, (seed, title) in enumerate(zip(seeds, scenario_names)):
        # 1. P Kontrolcu Testi
        env_p = TurretEnv()
        obs_p, _ = env_p.reset(seed=seed)
        p_ctrl = PController(kp_pan=0.35, kp_tilt=0.35)

        p_errors = []
        for _ in range(env_p.max_steps):
            px, py = env_p.last_pixel_pos
            err_x = px - env_p.center_x
            err_y = py - env_p.center_y
            dist_px = math.hypot(err_x, err_y)
            p_errors.append(dist_px)

            # Oransal kontrol eylemi (Normalize acisal hiz komutu)
            action_p = np.array(
                [
                    np.clip(err_x / (env_p.screen_w / 4.0), -1.0, 1.0),
                    np.clip(-err_y / (env_p.screen_h / 4.0), -1.0, 1.0),
                ],
                dtype=np.float32,
            )
            obs_p, _, term, trunc, _ = env_p.step(action_p)
            if term or trunc:
                break

        # 2. RL (PPO) Testi (Birebir Ayni Tohum ve Dinamik Kosul)
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

        p_total_errors.append(np.mean(p_errors))
        rl_total_errors.append(np.mean(rl_errors))

        # Alt Grafige Ciz
        time_axis = np.linspace(0, len(p_errors) * env_p.dt, len(p_errors))
        ax = axes[idx]
        ax.plot(
            time_axis,
            p_errors,
            label=f"P Kontrolcu (Ort Hata: {np.mean(p_errors):.1f} px)",
            color="#dc2626",
            lw=1.8,
            ls="--",
        )
        ax.plot(
            time_axis[:len(rl_errors)],
            rl_errors,
            label=f"PPO - RL Ajani (Ort Hata: {np.mean(rl_errors):.1f} px)",
            color="#0d9488",
            lw=2.2,
        )
        ax.axhline(30, color="gray", linestyle=":", alpha=0.75, label="Hedef Kilit Alani (30 px)")
        ax.set_title(f"Senaryo {idx+1}: {title} (Seed: {seed})", fontweight="bold", fontsize=11)
        ax.set_ylabel("Piksel Hatasi (px)", fontsize=10)
        ax.grid(True, alpha=0.35, linestyle="--")
        ax.legend(loc="upper right", framealpha=0.92)

    axes[-1].set_xlabel("Zaman (Saniye)", fontsize=11)

    fig.suptitle(
        "YILDIZ TEKNIK UNIVERSITESI - KONTROL VE OTOMASYON MUHENDISLIGI BOLUMU\n"
        "OTONOM PAN-TILT TARET: P KONTROLCU vs. PPO RL DERIN TAKVIYELI OGRENME KARSILASTIRMASI",
        fontsize=12,
        fontweight="bold",
        y=0.992,
    )
    plt.tight_layout(rect=[0, 0.02, 1, 0.94])

    p_plot_path = current_dir / "p_vs_rl_comparison.png"
    pid_plot_path = current_dir / "pid_vs_rl_comparison.png"
    plt.savefig(str(p_plot_path), dpi=300)
    plt.savefig(str(pid_plot_path), dpi=300)
    plt.close()

    p_avg = float(np.mean(p_total_errors))
    rl_avg = float(np.mean(rl_total_errors))
    improvement = (p_avg - rl_avg) / p_avg * 100.0

    print("\n==================================================================")
    print("                    BENZETIM TEST SONUC OZETI                     ")
    print("==================================================================")
    print(f"P Kontrolcu Ortalama Takip Hatasi : {p_avg:.2f} px")
    print(f"PPO (RL)    Ortalama Takip Hatasi : {rl_avg:.2f} px")
    print(f"RL Performans Iyilestirmesi       : %{improvement:+.1f}")
    print("------------------------------------------------------------------")
    print(f"[OK] 300 DPI Tez Grafigi Kaydedildi: {p_plot_path.name}")
    print(f"[OK] Karsilastirma Grafigi Kaydedildi: {pid_plot_path.name}")
    print("==================================================================")


if __name__ == "__main__":
    run_comparison()
