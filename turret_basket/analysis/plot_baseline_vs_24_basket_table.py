#!/usr/bin/env python3
"""
Baseline vs. 24-Trial Optuna Karşılaştırma Grafiği (Turret Basket - Cup Tracker).
"""

from pathlib import Path
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[2]
OUT_IMG = PROJECT_ROOT / "turret_basket" / "analysis" / "baseline_vs_24_basket_table_comparison.png"

metrics_data = [
    {
        "title": "Toplam Robust Cost",
        "unit": "Puan [Dusuk = Iyi]",
        "base": 3.60,
        "optuna": 2.99,
        "diff": "-%16.9",
        "fmt": "{:.2f}"
    },
    {
        "title": "Ortalama Kilitlenme RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 0.580,
        "optuna": 0.452,
        "diff": "-%22.1",
        "fmt": "{:.3f}°"
    },
    {
        "title": "Genel Kilitli Kalma Orani",
        "unit": "Yuzde [Yuksek = Iyi]",
        "base": 96.0,
        "optuna": 100.0,
        "diff": "+%4.0 (Tam Kilit)",
        "fmt": "%{:.1f}"
    },
    {
        "title": "2.6 m Zorlu Mesafe RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 0.904,
        "optuna": 0.349,
        "diff": "-%61.4 (Kritik Kazanc)",
        "fmt": "{:.3f}°"
    },
    {
        "title": "2.6 m Mesafe Kilit Orani",
        "unit": "Yuzde [Yuksek = Iyi]",
        "base": 76.1,
        "optuna": 100.0,
        "diff": "+%23.9",
        "fmt": "%{:.1f}"
    },
    {
        "title": "1.4 m Ideal Mesafe RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 0.353,
        "optuna": 0.279,
        "diff": "-%21.0",
        "fmt": "{:.3f}°"
    },
    {
        "title": "1.8 m Orta Mesafe RMS Hatasi",
        "unit": "Derece [Dusuk = Iyi]",
        "base": 0.507,
        "optuna": 0.448,
        "diff": "-%11.6",
        "fmt": "{:.3f}°"
    },
    {
        "title": "Ortalama Yerlesme Suresi (Settling)",
        "unit": "Saniye [Dusuk = Iyi]",
        "base": 0.406,
        "optuna": 0.404,
        "diff": "Dengeli Hizli",
        "fmt": "{:.3f} s"
    },
]

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
fig, axes = plt.subplots(4, 2, figsize=(15, 16))
fig.suptitle(
    "Turret Basket - Cup Tracker (1. Model): Baseline vs. 24-Trial Optuna Dogrudan Tablo Karsilastirmasi\n"
    "Simulatör Priors (Baseline - Trial 0) vs. Webots CAD Taretine Ozel Optimal (Trial 14)",
    fontsize=14,
    fontweight="bold"
)

axes_flat = axes.flatten()

for idx, item in enumerate(metrics_data):
    ax = axes_flat[idx]
    bars = ax.bar(
        ["Baseline (Kirmizi)", "24-Trial Optuna (Yesil/Mavi)"],
        [item["base"], item["optuna"]],
        color=["#e74c3c", "#16a085"],
        width=0.45,
        alpha=0.88,
        edgecolor=["#c0392b", "#117a65"],
        linewidth=1.2
    )
    
    ax.set_title(f"{item['title']} - [{item['diff']}]", fontsize=11, fontweight="bold", pad=8)
    ax.set_ylabel(item["unit"], fontsize=9, fontweight="bold")
    
    max_val = max(item["base"], item["optuna"])
    y_offset = max(0.001, max_val * 0.05)
    
    val_base_str = item["fmt"].format(item["base"])
    val_optuna_str = item["fmt"].format(item["optuna"])
    
    ax.text(0, item["base"] + y_offset, val_base_str, ha="center", va="bottom", fontsize=10, fontweight="bold", color="#922b21")
    ax.text(1, item["optuna"] + y_offset, val_optuna_str, ha="center", va="bottom", fontsize=10, fontweight="bold", color="#0e6251")
    
    top_limit = max_val + (y_offset * 4.0) if max_val > 0 else 1.0
    ax.set_ylim(0, top_limit)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
OUT_IMG.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT_IMG, dpi=200)
print(f"[OK] Basket Tablo Karsilastirma Grafigi kaydedildi: {OUT_IMG}")
