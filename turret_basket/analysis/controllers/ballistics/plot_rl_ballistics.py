#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET BASKET 2. MODEL RL BALISTIK POLITIKA KARAR MOTORU ANALIZI
================================================================================
Bu betik; cup_ballistic_policy.npz modelini calistirarak ToF LiDAR menziline
(0.8m - 3.2m) ve hedef dikey kot farkina (delta_z) gore:
  1. python_fig2_rl_ballistics_policy_decision.png:
     - Super-elevasyon Aci Egrisi (derece) vs Mesafe
     - Volan Firlatma Hiz Egrisi (m/s) vs Mesafe
     - STM32 TB6612FNG PWM Devir Haritasi (0-255) vs Mesafe
     - Tepe Noktasi (Apex) ve Kinetik Inis Enerjisi Analizi
================================================================================
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Font ve stil ayarlari (Akademik yayin formati)
plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 0.8

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[4]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

FIG_DIR = PROJECT_ROOT / "turret_basket" / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)

from turret_basket.rl_ballistics import RLBallistics


def plot_fig2_ballistics_policy():
    """Figure 2: 2. Model Balistik Politika Karar Motoru & Kestirim Analizi."""
    ballistics_ai = RLBallistics()

    distances = np.linspace(0.8, 3.2, 80)
    delta_z_nominal = -0.2287  # CAD v10 nominal masa-bardak kotu

    # 3 farkli kot seviyesi icin analiz (Nominal, +10cm yuksek masa, -10cm alcak masa)
    scenarios = [
        {"dz": delta_z_nominal + 0.10, "label": "Yuksek Kupa (delta_z = -0.13m)", "col": "#2563eb", "ls": "--"},
        {"dz": delta_z_nominal, "label": "Nominal Masa (delta_z = -0.23m)", "col": "#16a34a", "ls": "-"},
        {"dz": delta_z_nominal - 0.10, "label": "Alcak Kupa (delta_z = -0.33m)", "col": "#ea580c", "ls": "-."},
    ]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor("#ffffff")
    fig.suptitle("Turret Basket: 2. Model RL Balistik Politika Karar Motoru ve Dinamik Kestirim Analizi\n"
                 "ToF LiDAR Menziline Gore Super-elevasyon Acisi ve Volan Hiz Optimizasyonu",
                 fontsize=13, fontweight="bold", y=0.98, color="#0f172a")

    # 1. Super-elevasyon Aci Egrisi (Sol Ust)
    ax1 = axes[0, 0]
    ax1.set_facecolor("#fcfcfc")
    ax1.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    for sc in scenarios:
        angles = [ballistics_ai.compute_launch_parameters(d, sc["dz"]).tilt_offset_deg for d in distances]
        ax1.plot(distances, angles, color=sc["col"], linestyle=sc["ls"], linewidth=2.2, label=sc["label"])

    ax1.set_title("Super-elevasyon Tilt Atis Acisi vs Mesafe", fontsize=11, fontweight="bold", pad=8)
    ax1.set_xlabel("LiDAR Hedef Mesafesi [metre]", fontsize=9.5)
    ax1.set_ylabel("Atis Aci Ofseti [derece]", fontsize=9.5)
    ax1.set_ylim(16.0, 26.0)
    ax1.legend(loc="lower right", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    # 2. Volan Firlatma Hizi (Sag Ust)
    ax2 = axes[0, 1]
    ax2.set_facecolor("#fcfcfc")
    ax2.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    for sc in scenarios:
        speeds = [ballistics_ai.compute_launch_parameters(d, sc["dz"]).speed_mps for d in distances]
        ax2.plot(distances, speeds, color=sc["col"], linestyle=sc["ls"], linewidth=2.2, label=sc["label"])

    ax2.set_title("Volan Cikis Hizi (Muzzle Velocity) vs Mesafe", fontsize=11, fontweight="bold", pad=8)
    ax2.set_xlabel("LiDAR Hedef Mesafesi [metre]", fontsize=9.5)
    ax2.set_ylabel("Dart Cikis Hizi [m/s]", fontsize=9.5)
    ax2.set_ylim(2.5, 9.5)
    ax2.legend(loc="upper left", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    # 3. STM32 TB6612FNG PWM Haritasi (Sol Alt)
    ax3 = axes[1, 0]
    ax3.set_facecolor("#fcfcfc")
    ax3.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    # Hizi PWM'e donusturme modeli (3.0 m/s = 140 PWM, 10.0 m/s = 255 PWM lineer oranti)
    nominal_speeds = [ballistics_ai.compute_launch_parameters(d, delta_z_nominal).speed_mps for d in distances]
    pwms = [np.clip(int(140 + (s - 3.0) * 16.5), 130, 255) for s in nominal_speeds]

    ax3.plot(distances, pwms, color="#9333ea", linewidth=2.4, label="STM32 Motor PWM Komutu (0-255)")
    ax3.axhline(190, color="#64748b", linestyle=":", label="1.8m Nominal PWM Eşlenigi (~190)")
    ax3.set_title("Aktuator Surucu: TB6612FNG Volan PWM vs Mesafe", fontsize=11, fontweight="bold", pad=8)
    ax3.set_xlabel("LiDAR Hedef Mesafesi [metre]", fontsize=9.5)
    ax3.set_ylabel("PWM Duty Degeri [8-bit Ticks: 0 - 255]", fontsize=9.5)
    ax3.set_ylim(120, 260)
    ax3.legend(loc="upper left", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    # 4. Tepe Noktasi (Apex) Yuksekligi (Sag Alt)
    ax4 = axes[1, 1]
    ax4.set_facecolor("#fcfcfc")
    ax4.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    # Basit balistik tepe noktasi yaklasimi: y_apex = (v0 * sin(th))^2 / (2 * g)
    apexes = []
    for d, s in zip(distances, nominal_speeds):
        th = ballistics_ai.compute_launch_parameters(d, delta_z_nominal).tilt_offset_deg
        th_rad = np.radians(th)
        y_ap = (s * np.sin(th_rad)) ** 2 / (2 * 9.80665)
        apexes.append(y_ap * 100.0)  # cm cinsinden

    ax4.plot(distances, apexes, color="#0284c7", linewidth=2.4, label="Tepe Noktasi (Apex) Yukselisi (cm)")
    ax4.fill_between(distances, 0, apexes, color="#38bdf8", alpha=0.2)
    ax4.set_title("Dart Tepe Noktasi (Apex) vs Mesafe", fontsize=11, fontweight="bold", pad=8)
    ax4.set_xlabel("LiDAR Hedef Mesafesi [metre]", fontsize=9.5)
    ax4.set_ylabel("Namlu Ustu Maksimum Yukselis [cm]", fontsize=9.5)
    ax4.set_ylim(0, 35)
    ax4.legend(loc="upper left", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")

    plt.tight_layout()
    out_path = FIG_DIR / "python_fig2_rl_ballistics_policy_decision.png"
    plt.savefig(out_path, dpi=300, facecolor=fig.get_facecolor())
    plt.close()
    print(f"[FIG 2 OK] Kaydedildi: {out_path}")


if __name__ == "__main__":
    plot_fig2_ballistics_policy()
