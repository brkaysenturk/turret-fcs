#!/usr/bin/env python3
"""
Mesafe bazli (1m, 2m, 3m) karsilastirma grafigi uretici.
Eski (tekil 2m) ve Yeni (coklu mesafe Optuna) parametrelerin
Lock %, Overshoot ve Jerk metriklerini gorsellestirir.
"""

from __future__ import annotations

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
RESULTS_FILE = ROOT / "turret_fcs" / "analysis" / "comparison" / "webots_multiprofile_p_results.json"
OUT_IMG = ROOT / "turret_fcs" / "analysis" / "comparison" / "distance_comparison_p.png"

# Veriler: Eski (2m-only) vs Yeni (Multi-distance Optuna)
# Metrikler P8 (Kose Giris) profilinde:
distances = ["1.0 m", "2.0 m", "3.0 m"]

# 1. Kose Giris Asimi (Overshoot deg) - Dusuk olmasi daha iyi
overshoot_old = [3.758, 2.626, 2.464]
overshoot_new = [2.302, 1.816, 0.924]

# 2. Yon Degistirme / Titreme Orani (Flip Rate %) - Dusuk olmasi daha iyi
flip_old = [3.41, 2.82, 1.92]
flip_new = [0.54, 2.52, 2.00]

# 3. Kilit Orani (Lock %) - Yuksek olmasi daha iyi
lock_old = [60.75, 54.21, 0.0]
lock_new = [79.44, 70.09, 3.45]

# 4. Motor Sarsintisi (Jerk deg) - Dusuk olmasi daha iyi
jerk_old = [0.0207, 0.0253, 0.0708]
jerk_new = [0.0231, 0.0147, 0.0654]

x = np.arange(len(distances))
width = 0.35

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Turret FCS - P Kontrolcu Mesafe Bazli Karsilastirma (1m vs 2m vs 3m)\nEski (2.0m Tekil Optuna) vs. Yeni (Coklu Mesafe Optuna)", fontsize=13, fontweight="bold")

# Panel 1: Asim (Overshoot)
ax = axes[0, 0]
ax.bar(x - width/2, overshoot_old, width, label="Eski (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, overshoot_new, width, label="Yeni (Coklu Mesafe)", color="#2ecc71", alpha=0.85)
ax.set_ylabel("Asim (derece)", fontweight="bold")
ax.set_title("Kose Giris Asimi (Overshoot) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, overshoot_old[i] + 0.1, f"{overshoot_old[i]:.2f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, overshoot_new[i] + 0.1, f"{overshoot_new[i]:.2f}", ha="center", fontsize=9, fontweight="bold")

# Panel 2: Titreme / Salinim (Flip Rate)
ax = axes[0, 1]
ax.bar(x - width/2, flip_old, width, label="Eski (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, flip_new, width, label="Yeni (Coklu Mesafe)", color="#3498db", alpha=0.85)
ax.set_ylabel("Yon Degistirme Orani (%)", fontweight="bold")
ax.set_title("Motor Titreme / Salinim Orani (Flip Rate) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, flip_old[i] + 0.1, f"%{flip_old[i]:.2f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, flip_new[i] + 0.1, f"%{flip_new[i]:.2f}", ha="center", fontsize=9, fontweight="bold")

# Panel 3: Kilit Basarisi (Lock %)
ax = axes[1, 0]
ax.bar(x - width/2, lock_old, width, label="Eski (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, lock_new, width, label="Yeni (Coklu Mesafe)", color="#2ecc71", alpha=0.85)
ax.set_ylabel("Kilit Orani (%)", fontweight="bold")
ax.set_title("Kose Giris Kilit Orani (Lock %) [Yuksek = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, lock_old[i] + 1.5, f"%{lock_old[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, lock_new[i] + 1.5, f"%{lock_new[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

# Panel 4: Motor Jerk (Sarsinti)
ax = axes[1, 1]
ax.bar(x - width/2, jerk_old, width, label="Eski (2m Tekil)", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, jerk_new, width, label="Yeni (Coklu Mesafe)", color="#9b59b6", alpha=0.85)
ax.set_ylabel("Jerk (derece / adim)", fontweight="bold")
ax.set_title("Motor Komut Sarsintisi (Jerk) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, jerk_old[i] + 0.002, f"{jerk_old[i]:.3f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, jerk_new[i] + 0.002, f"{jerk_new[i]:.3f}", ha="center", fontsize=9, fontweight="bold")

plt.tight_layout()
fig.savefig(OUT_IMG, dpi=200)
print(f"[OK] Grafik kaydedildi: {OUT_IMG}")

