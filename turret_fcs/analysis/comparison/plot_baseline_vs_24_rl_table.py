#!/usr/bin/env python3
"""
Turret FCS - RL Kontrolcu: Baseline vs. 24-Trial Optuna 8-Metrik Karsilastirma Grafigi.
"""

from pathlib import Path
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
OUT_IMG = ROOT / "turret_fcs" / "analysis" / "comparison" / "baseline_vs_24_rl_table_comparison.png"

metrics_data = [
    {
        "title": "Toplam Robust Cost",
        "unit": "Puan [Dusuk = Iyi]",
        "base": 184.83,
        "optuna": 139.00,
        "diff": "-%24.8",
        "fmt": "{:.2f}"
    },
    {
        "title": "Profil 8: 1.0 m RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 12.38,
        "optuna": 4.20,
        "diff": "-%66.1",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 8: 1.0 m Kilit Orani",
        "unit": "Yuzde [Yuksek = Iyi]",
        "base": 0.0,
        "optuna": 23.91,
        "diff": "+%23.9",
        "fmt": "%{:.1f}"
    },
    {
        "title": "Profil 8: 1.0 m Komut Jerk'i",
        "unit": "deg/adim [Dusuk = Iyi]",
        "base": 0.0874,
        "optuna": 0.0842,
        "diff": "-%3.7",
        "fmt": "{:.4f}"
    },
    {
        "title": "Profil 2: 2.0 m Sinus RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 2.18,
        "optuna": 2.14,
        "diff": "-0.04°",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 2: 3.0 m Sinus RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 1.79,
        "optuna": 1.80,
        "diff": "+0.01°",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 7: 1.0 m Figure-8 RMS",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 6.07,
        "optuna": 6.05,
        "diff": "-0.02°",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 7: 2.0 m Figure-8 RMS",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 2.53,
        "optuna": 2.45,
        "diff": "-0.08°",
        "fmt": "{:.2f}°"
    },
]

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
fig, axes = plt.subplots(4, 2, figsize=(15, 16))
fig.suptitle(
    "Turret FCS - RL Kontrolcu: Baseline vs. 24-Trial Optuna Dogrudan Tablo Karsilastirmasi\n"
    "Eski Tasarimin En Iyi Degerleri (Baseline) vs. Yeni CAD Fizigine Ozel 24-Trial Optuna Optimal (T12)",
    fontsize=14,
    fontweight="bold"
)

axes_flat = axes.flatten()

for idx, item in enumerate(metrics_data):
    ax = axes_flat[idx]
    bars = ax.bar(
        ["Baseline (Kirmizi)", "24-Trial Optuna (Yesil/Mavi)"],
        [item["base"], item["optuna"]],
        color=["#e74c3c", "#2980b9"],
        width=0.45,
        alpha=0.88,
        edgecolor=["#c0392b", "#1f618d"],
        linewidth=1.2
    )
    
    ax.set_title(f"{item['title']} - [{item['diff']}]", fontsize=11, fontweight="bold", pad=8)
    ax.set_ylabel(item["unit"], fontsize=9, fontweight="bold")
    
    # Deger etiketlerini barlarin uzerine ekle
    max_val = max(item["base"], item["optuna"])
    y_offset = max(0.001, max_val * 0.05)
    
    val_base_str = item["fmt"].format(item["base"])
    val_optuna_str = item["fmt"].format(item["optuna"])
    
    ax.text(0, item["base"] + y_offset, val_base_str, ha="center", va="bottom", fontsize=10, fontweight="bold", color="#922b21")
    ax.text(1, item["optuna"] + y_offset, val_optuna_str, ha="center", va="bottom", fontsize=10, fontweight="bold", color="#1b4f72")
    
    top_limit = max_val + (y_offset * 4.0) if max_val > 0 else 1.0
    ax.set_ylim(0, top_limit)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
OUT_IMG.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT_IMG, dpi=200)
print(f"[OK] RL Tablo Karsilastirma Grafigi kaydedildi: {OUT_IMG}")

