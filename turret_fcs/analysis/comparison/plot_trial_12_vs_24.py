#!/usr/bin/env python3
"""
Ilk 12 Trial vs Ikinci 12 Trial (12 vs 24) Karsilastirma Grafigi.
Optuna'nin ilk turda buldugu katsayilar ile ikinci turdaki ek aramanin
metriklerini karsilastirir.
"""

from __future__ import annotations

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT_IMG = ROOT / "turret_fcs" / "analysis" / "comparison" / "trial_12_vs_24_comparison.png"

distances = ["1.0 m", "2.0 m", "3.0 m"]

# 1. Kose Giris Asimi (Overshoot deg)
overshoot_t12 = [2.302, 1.816, 0.924]
overshoot_t24 = [2.302, 1.816, 0.924]

# 2. 1m Kose Giris Flip Rate (%) - Yon degistirme
flip_t12 = [1.67, 2.52, 2.00]
flip_t24 = [0.54, 2.52, 2.00]

# 3. Kilit Orani (Lock %) P8 Kose
lock_t12 = [79.44, 70.09, 3.45]
lock_t24 = [79.44, 70.09, 6.90]

# 4. Toplam Senaryo Maliyeti (Cost)
cost_t12 = [82.47, 64.41, 186.69]
cost_t24 = [80.28, 64.41, 184.96]

x = np.arange(len(distances))
width = 0.35

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Turret FCS - Optuna Gelisimi: Ilk 12 Trial vs. 24 Trial (Ek 12 Trial)\nOptuna'nin Parametre Kararliligi ve Global Konverjans Analizi", fontsize=13, fontweight="bold")

# Panel 1: Toplam Maliyet (Cost)
ax = axes[0, 0]
ax.bar(x - width/2, cost_t12, width, label="12 Trial Sonu", color="#f39c12", alpha=0.85)
ax.bar(x + width/2, cost_t24, width, label="24 Trial Sonu", color="#27ae60", alpha=0.85)
ax.set_ylabel("Senaryo Maliyeti", fontweight="bold")
ax.set_title("Kose Giris Maliyeti [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, cost_t12[i] + 2, f"{cost_t12[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, cost_t24[i] + 2, f"{cost_t24[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

# Panel 2: 1m Flip Rate (Titreme)
ax = axes[0, 1]
ax.bar(x - width/2, flip_t12, width, label="12 Trial Sonu", color="#f39c12", alpha=0.85)
ax.bar(x + width/2, flip_t24, width, label="24 Trial Sonu", color="#2980b9", alpha=0.85)
ax.set_ylabel("Yon Degistirme Orani (%)", fontweight="bold")
ax.set_title("Motor Titreme / Salinim Orani (Flip Rate) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, flip_t12[i] + 0.05, f"%{flip_t12[i]:.2f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, flip_t24[i] + 0.05, f"%{flip_t24[i]:.2f}", ha="center", fontsize=9, fontweight="bold")

# Panel 3: Kilit Orani (Lock %)
ax = axes[1, 0]
ax.bar(x - width/2, lock_t12, width, label="12 Trial Sonu", color="#f39c12", alpha=0.85)
ax.bar(x + width/2, lock_t24, width, label="24 Trial Sonu", color="#27ae60", alpha=0.85)
ax.set_ylabel("Kilit Orani (%)", fontweight="bold")
ax.set_title("Kilitlenme Orani (Lock %) [Yuksek = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, lock_t12[i] + 1.5, f"%{lock_t12[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, lock_t24[i] + 1.5, f"%{lock_t24[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

# Panel 4: Overshoot (Asim)
ax = axes[1, 1]
ax.bar(x - width/2, overshoot_t12, width, label="12 Trial Sonu", color="#f39c12", alpha=0.85)
ax.bar(x + width/2, overshoot_t24, width, label="24 Trial Sonu", color="#8e44ad", alpha=0.85)
ax.set_ylabel("Asim (derece)", fontweight="bold")
ax.set_title("Kose Giris Asimi (Overshoot) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, overshoot_t12[i] + 0.05, f"{overshoot_t12[i]:.2f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, overshoot_t24[i] + 0.05, f"{overshoot_t24[i]:.2f}", ha="center", fontsize=9, fontweight="bold")

plt.tight_layout()
fig.savefig(OUT_IMG, dpi=200)
print(f"[OK] 12 vs 24 grafigi kaydedildi: {OUT_IMG}")
