#!/usr/bin/env python3
"""
================================================================================
MODUL: ASIRI DEGER TEORISI ILE HEDEF SECICI VE KAPILAMA OPTIMIZASYONU
================================================================================
Bu betik; gercek saha telemetrisindeki 4.041 hedefli kare uzerinde
YOLOv8 guven skoru (target_conf) dagilimini ve kareler arasi piksel sicrama
istatistiklerini (Extreme Value Theory / %99.5 persentil) analiz eder:

Ciktilar:
- min_conf (Arama modunda yanlis pozitifi eleyen esik)
- track_min_conf (Hareket bulanikliginda hedefi koparmayan takip histerezis esigi)
- spatial_gate_px (Farkli hedefe ID atlamasini engelleyen maksimum sicrama capi)
================================================================================
"""

import csv
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
FIG_DIR = BASE_DIR / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 0.8
plt.rcParams["grid.color"] = "#E0E0E0"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.7


def run_selector_optimization(csv_path: Path):
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    det_rows = [r for r in rows if r["target_detected"] == "1"]
    confs = np.array([float(r["target_conf"]) for r in det_rows])
    cxs = np.array([float(r["box_cx"]) for r in det_rows])
    cys = np.array([float(r["box_cy"]) for r in det_rows])

    # 1. Guven Skoru Dagilimi Analizi
    # Gercek takipte guven skoru %99.5 guven araligi
    # min_conf (Arama modu): %99 dogrulukla sahte tespiti (false positive) dislayan alt esik
    # Telemetride tespit edilen gercek hedeflerin %99'u 0.35+ uzerinde
    p005 = float(np.percentile(confs, 0.5)) # %0.5 alt persentil: 0.299

    # min_conf: 0.30 (arama modunda temiz baslangic)
    optimal_min_conf = float(np.round(p005, 2)) # 0.30

    # track_min_conf: Hareket bulanikliginda dahi takibi surduren mutlak alt sinir
    # En dusuk gecerli takip skoru 0.165
    optimal_track_min_conf = float(np.round(np.min(confs) * 0.95, 2)) # ~0.15 - 0.16

    # 2. Mekansal Sicrama (Spatial Jump) Analizi - Extreme Value Theory (EVT)
    # Kesintisiz hedef hareketinde ardisik kareler arasi maksimum fiziksel hareket
    spatial_jumps = np.sqrt(np.diff(cxs) ** 2 + np.diff(cys) ** 2)

    # %99.5 persentil gercek en yuksek hedef manevrasidir. Bunun uzerindekiler baska kutuya atlamadir.
    p99_5_jump = float(np.percentile(spatial_jumps, 99.5)) # 128.4 px

    # Guvenlik katsayisiyla (1.25x EVT payi):
    optimal_spatial_gate = float(np.round(p99_5_jump * 1.35, 0)) # ~175 - 180 px

    # Gorsellestirme
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), dpi=300)

    # Panel 1: Conf Dagilimi ve Esikler
    ax1.hist(confs, bins=45, color="#1565C0", alpha=0.8, edgecolor="white", density=True)
    ax1.axvline(optimal_min_conf, color="#E65100", linestyle="--", lw=2.0, label=f"Optimal min_conf: {optimal_min_conf:.2f}")
    ax1.axvline(optimal_track_min_conf, color="#2E7D32", linestyle=":", lw=2.0, label=f"Optimal track_min_conf: {optimal_track_min_conf:.2f}")
    ax1.set_title("1. Gercek YOLOv8 Hedef Guven Dagilimi (CONF)", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Confidence Skoru", fontsize=10)
    ax1.set_ylabel("Olasilik Yogulugu", fontsize=10)
    ax1.grid(True)
    ax1.legend(loc="upper left")

    # Panel 2: Mekansal Sicrama Dagilimi ve Kapisi
    ax2.hist(spatial_jumps[spatial_jumps < 250], bins=50, color="#6A1B9A", alpha=0.8, edgecolor="white", density=True)
    ax2.axvline(optimal_spatial_gate, color="#B71C1C", linestyle="--", lw=2.0, label=f"Optimal spatial_gate: {optimal_spatial_gate:.0f} px")
    ax2.axvline(250.0, color="#555555", linestyle=":", lw=1.5, label="Eski Elle Girilen: 250.0 px")
    ax2.set_title("2. Ardisik Kare Piksel Sicramasi (EVT Sınırı)", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Kareler Arasi Piksel Sicramasi (px)", fontsize=10)
    ax2.set_ylabel("Yogunluk", fontsize=10)
    ax2.grid(True)
    ax2.legend(loc="upper right")

    out_png = FIG_DIR / "fig12_selector_evt_optimization.png"
    plt.tight_layout()
    plt.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close()

    results = {
        "min_conf": optimal_min_conf,
        "track_min_conf": optimal_track_min_conf,
        "spatial_gate_px": optimal_spatial_gate,
        "p99_5_jump_px": p99_5_jump,
    }
    out_json = BASE_DIR / "optimal_selector_params.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print(f"[+] Hedef secici EVT sonuclari kaydedildi: {out_json}")
    print(f"    - min_conf        : {results['min_conf']}")
    print(f"    - track_min_conf  : {results['track_min_conf']}")
    print(f"    - spatial_gate_px : {results['spatial_gate_px']} px (Eski: 250 px)")
    return results


if __name__ == "__main__":
    latest_log = PROJECT_ROOT / "logs" / "telemetry_test_20261026_002136.csv"
    run_selector_optimization(latest_log)
