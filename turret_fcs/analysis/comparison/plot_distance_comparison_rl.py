#!/usr/bin/env python3
"""
RL Kontrolcu Mesafe Bazli (1m, 2m, 3m) Karsilastirma Grafigi.
Eski ve Yeni Coklu Mesafe Optuna parametrelerinin metriklerini gorsellestirir.
"""

from __future__ import annotations

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT_IMG = ROOT / "turret_fcs" / "analysis" / "comparison" / "distance_comparison_rl.png"

distances = ["1.0 m", "2.0 m", "3.0 m"]

# 1. Kose Giris Asimi (Overshoot deg)
overshoot_old = [2.100, 1.208, 0.680]
overshoot_new = [2.019, 1.289, 0.680]

# 2. 1m Kose Giris Flip Rate (%) - Yon degistirme
flip_old = [8.74, 3.36, 1.92]
flip_new = [3.21, 3.12, 1.92]

# 3. Kilit Orani (Lock %) P8 Kose
lock_old = [77.36, 82.24, 3.45]
lock_new = [76.19, 82.24, 3.45]

# 4. Dinamik Takip (Figure-8) Kilit Orani (%)
fig8_lock_old = [39.89, 54.79, 60.26]
fig8_lock_new = [41.86, 56.38, 60.26]

x = np.arange(len(distances))
width = 0.35

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Turret FCS - RL Kontrolcu Mesafe Bazli Karsilastirma (1m vs 2m vs 3m)\nEski (2.0m Tekil) vs. Yeni (1m, 2m, 3m Coklu Mesafe Optuna)", fontsize=13, fontweight="bold")

# Panel 1: 1m Titreme / Salinim (Flip Rate)
ax = axes[0, 0]
ax.bar(x - width/2, flip_old, width, label="Eski RL (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, flip_new, width, label="Yeni RL (Coklu Mesafe)", color="#2ecc71", alpha=0.85)
ax.set_ylabel("Yon Degistirme Orani (%)", fontweight="bold")
ax.set_title("Motor Titreme / Salinim (Flip Rate) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, flip_old[i] + 0.2, f"%{flip_old[i]:.2f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, flip_new[i] + 0.2, f"%{flip_new[i]:.2f}", ha="center", fontsize=9, fontweight="bold")

# Panel 2: Figure-8 Dinamik Kilit Orani
ax = axes[0, 1]
ax.bar(x - width/2, fig8_lock_old, width, label="Eski RL (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, fig8_lock_new, width, label="Yeni RL (Coklu Mesafe)", color="#3498db", alpha=0.85)
ax.set_ylabel("Figure-8 Kilit Orani (%)", fontweight="bold")
ax.set_title("Figure-8 Dinamik Takip Kilit Orani [Yuksek = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, fig8_lock_old[i] + 1.2, f"%{fig8_lock_old[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, fig8_lock_new[i] + 1.2, f"%{fig8_lock_new[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

# Panel 3: Kose Giris Asimi (Overshoot)
ax = axes[1, 0]
ax.bar(x - width/2, overshoot_old, width, label="Eski RL (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, overshoot_new, width, label="Yeni RL (Coklu Mesafe)", color="#9b59b6", alpha=0.85)
ax.set_ylabel("Asim (derece)", fontweight="bold")
ax.set_title("Kose Giris Asimi (Overshoot) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, overshoot_old[i] + 0.05, f"{overshoot_old[i]:.2f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, overshoot_new[i] + 0.05, f"{overshoot_new[i]:.2f}", ha="center", fontsize=9, fontweight="bold")

# Panel 4: P8 Kose Giris Kilit Orani
ax = axes[1, 1]
ax.bar(x - width/2, lock_old, width, label="Eski RL (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, lock_new, width, label="Yeni RL (Coklu Mesafe)", color="#2ecc71", alpha=0.85)
ax.set_ylabel("Kilit Orani (%)", fontweight="bold")
ax.set_title("Kose Giris Ilk Kilit Orani [Yuksek = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, lock_old[i] + 1.5, f"%{lock_old[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, lock_new[i] + 1.5, f"%{lock_new[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

plt.tight_layout()
fig.savefig(OUT_IMG, dpi=200)
print(f"[OK] RL grafik kaydedildi: {OUT_IMG}")
