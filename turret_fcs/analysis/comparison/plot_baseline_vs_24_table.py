#!/usr/bin/env python3
"""
Baseline vs. 24-Trial Optuna Tablo Karsilastirma Grafigi Uretici.
"""

from pathlib import Path
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
OUT_IMG = ROOT / "turret_fcs" / "analysis" / "comparison" / "baseline_vs_24_table_comparison.png"

metrics_data = [
    {
        "title": "Toplam Robust Cost",
        "unit": "Puan [Dusuk = Iyi]",
        "base": 203.39,
        "optuna": 154.20,
        "diff": "-%24.2",
        "fmt": "{:.2f}"
    },
    {
        "title": "Profil 8: 1.0 m Kose Asimi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 6.37,
        "optuna": 3.35,
        "diff": "-%47.4",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 8: 1.0 m Titreme / Jerk",
        "unit": "deg/adim [Dusuk = Iyi]",
        "base": 0.085,
        "optuna": 0.047,
        "diff": "-%44.7",
        "fmt": "{:.3f}"
    },
    {
        "title": "Profil 2: 1.0 m Sinus RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 7.34,
        "optuna": 4.62,
        "diff": "-%37.0",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 2: 2.0 m Sinus Takip Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 2.80,
        "optuna": 2.99,
        "diff": "+0.19°",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 2: 3.0 m Sinus Takip Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 2.52,
        "optuna": 2.63,
        "diff": "+0.11°",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 7: 1.0 m Figure-8 RMS",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 6.50,
        "optuna": 6.58,
        "diff": "+0.08°",
        "fmt": "{:.2f}°"
    },
    {
        "title": "Profil 2: Sinus Komut Jerk'i (2m)",
        "unit": "deg/adim [Dusuk = Iyi]",
        "base": 0.0145,
        "optuna": 0.0113,
        "diff": "-%22.1",
        "fmt": "{:.4f}"
    },
]

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
fig, axes = plt.subplots(4, 2, figsize=(15, 16))
fig.suptitle(
    "Turret FCS - P Kontrolcu: Baseline vs. 24-Trial Optuna Dogrudan Tablo Karsilastirmasi\n"
    "Eski Tasarimin En Iyi Degerleri (Baseline) vs. Yeni CAD Fizigine Ozel 24-Trial Optuna Optimal (T23)",
    fontsize=14,
    fontweight="bold"
)

axes_flat = axes.flatten()

for idx, item in enumerate(metrics_data):
    ax = axes_flat[idx]
    bars = ax.bar(
        ["Baseline (Kirmizi)", "24-Trial Optuna (Yesil)"],
        [item["base"], item["optuna"]],
        color=["#e74c3c", "#27ae60"],
        width=0.45,
        alpha=0.88,
        edgecolor=["#c0392b", "#1e8449"],
        linewidth=1.2
    )
    ax.set_title(f"{item['title']} ({item['diff']})", fontsize=11, fontweight="bold", pad=10)
    ax.set_ylabel(item["unit"], fontsize=9, fontweight="bold")
    
    y_max = max(item["base"], item["optuna"])
    ax.set_ylim(0, y_max * 1.25)
    
    val_b_str = item["fmt"].format(item["base"])
    val_o_str = item["fmt"].format(item["optuna"])
    
    ax.text(0, item["base"] + y_max * 0.03, val_b_str, ha="center", va="bottom", fontsize=10, fontweight="bold", color="#c0392b")
    ax.text(1, item["optuna"] + y_max * 0.03, f"{val_o_str} ({item['diff']})", ha="center", va="bottom", fontsize=10, fontweight="bold", color="#1e8449")

plt.tight_layout()
fig.savefig(OUT_IMG, dpi=200)
print(f"[OK] Tablodaki 8 metrigin birebir karsilastirma grafigi kaydedildi: {OUT_IMG}")

