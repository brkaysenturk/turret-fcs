#!/usr/bin/env python3
"""
================================================================================
MODUL: BASELINE VS 24-TRIAL OPTUNA KARSILASTIRMA TABLOSU (TURRET BASKET)
================================================================================
Baseline (Trial 0) vs Optimal 24-Trial Webots CAD (Trial 14) parametrelerinin
tum metriklerdeki karsilastirmali bar grafik tablosunu uretir.
================================================================================
"""

from pathlib import Path
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[3]
FIG_DIR = PROJECT_ROOT / "turret_basket" / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)

OUT_IMG_FIG = FIG_DIR / "python_fig7_baseline_vs_24_table.png"
OUT_IMG_LEGACY = PROJECT_ROOT / "turret_basket" / "analysis" / "baseline_vs_24_basket_table_comparison.png"

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
    ax.set_title(item["title"], fontsize=12, fontweight="bold", pad=8)
    ax.set_ylabel(item["unit"], fontsize=10)
    
    # Değerleri ve etiketleri barların üzerine yazma
    for bar, val in zip(bars, [item["base"], item["optuna"]]):
        y_val = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            y_val / 2.0,
            item["fmt"].format(val),
            ha="center",
            va="center",
            fontsize=12,
            fontweight="bold",
            color="white"
        )
        
    # Fark / İyileşme Rozeti (Badge)
    ax.text(
        0.5, 0.88,
        f"Degisim: {item['diff']}",
        transform=ax.transAxes,
        ha="center",
        va="center",
        fontsize=11,
        fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#f1f2f6", edgecolor="#bdc3c7", linewidth=1.2)
    )

plt.tight_layout(rect=[0, 0.03, 1, 0.96])
plt.savefig(OUT_IMG_FIG, dpi=300)
plt.savefig(OUT_IMG_LEGACY, dpi=300)
plt.close()
print(f"[TABLO OK] Kaydedildi: {OUT_IMG_FIG} ve {OUT_IMG_LEGACY}")
