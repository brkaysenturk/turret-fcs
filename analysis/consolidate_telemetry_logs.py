"""
================================================================================
MODUL: SAHA TESTLERI TELEMETRI BIRLESTIRME VE KARSILASTIRMALI ANALIZ MOTORU
================================================================================
Bu modul, projedeki tum gercek donanim test telemetri loglarini (logs/*.csv)
birlestirir, her oturum icin temel basarim gostergelerini (KPI) hesaplar ve
yayin kalitesinde karsilastirmali grafikler uretir.
================================================================================
"""

import csv
import glob
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
LOGS_DIR = PROJECT_ROOT / "logs"
FIG_DIR = BASE_DIR / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)


def analyze_single_log(filepath):
    with open(filepath, "r", encoding="utf-8", errors="ignore") as fp:
        reader = csv.DictReader(fp)
        rows = list(reader)
    if not rows:
        return None

    filename = os.path.basename(filepath)
    n = len(rows)
    try:
        dur = float(rows[-1]["time_sec"]) - float(rows[0]["time_sec"])
    except Exception:
        dur = 0.0

    fps_vals = [float(r["fps"]) for r in rows if float(r.get("fps", 0)) > 0]
    avg_fps = sum(fps_vals) / len(fps_vals) if fps_vals else 0.0

    det_rows = [r for r in rows if r.get("target_detected") == "1"]
    lock_rows = [r for r in rows if r.get("is_locked") == "1"]
    det_count = len(det_rows)
    lock_count = len(lock_rows)

    det_pct = (det_count / n * 100.0) if n > 0 else 0.0
    lock_tot_pct = (lock_count / n * 100.0) if n > 0 else 0.0
    lock_det_pct = (len([r for r in det_rows if r.get("is_locked") == "1"]) / det_count * 100.0) if det_count > 0 else 0.0

    tot_errs = [math.hypot(float(r["angular_err_pan"]), float(r["angular_err_tilt"])) for r in det_rows] if det_count else []
    pan_errs = [abs(float(r["angular_err_pan"])) for r in det_rows] if det_count else []
    tilt_errs = [abs(float(r["angular_err_tilt"])) for r in det_rows] if det_count else []

    cmd_pans = [abs(float(r["cmd_pan"])) for r in det_rows] if det_count else []
    cmd_tilts = [abs(float(r["cmd_tilt"])) for r in det_rows] if det_count else []

    sat_pan_count = sum(1 for c in cmd_pans if c >= 2.70)
    sat_tilt_count = sum(1 for c in cmd_tilts if c >= 2.70)
    sat_pct = ((sat_pan_count + sat_tilt_count) / (2 * det_count) * 100.0) if det_count else 0.0

    # Kilit kesintisizlik serileri (Lock streak)
    max_streak = 0
    cur_streak = 0
    streaks = []
    for r in rows:
        if r.get("is_locked") == "1":
            cur_streak += 1
        else:
            if cur_streak > 0:
                streaks.append(cur_streak)
                max_streak = max(max_streak, cur_streak)
                cur_streak = 0
    if cur_streak > 0:
        streaks.append(cur_streak)
        max_streak = max(max_streak, cur_streak)

    avg_streak = sum(streaks) / len(streaks) if streaks else 0.0

    # Modlar
    modes = list(set(r.get("control_mode", "P") for r in rows if r.get("control_mode")))
    mode_str = "+".join(sorted(modes)) if modes else "P"

    # Oturum siniflandirmasi
    if "failed" in filename:
        status = "BASARISIZ (Adaptif Asiri Salinim)"
    elif lock_det_pct > 30.0:
        status = "YUKSEK BASARIMLI KILIT"
    elif det_pct > 50.0:
        status = "STABIL TAKIP"
    else:
        status = "BEKLEME / SERBEST CALISMA"

    return {
        "filename": filename,
        "samples": n,
        "duration_sec": round(dur, 2),
        "avg_fps": round(avg_fps, 1),
        "detected_count": det_count,
        "detection_pct": round(det_pct, 1),
        "locked_count": lock_count,
        "lock_overall_pct": round(lock_tot_pct, 1),
        "lock_when_detected_pct": round(lock_det_pct, 1),
        "mean_error_deg": round(sum(tot_errs) / len(tot_errs), 2) if tot_errs else 0.0,
        "median_error_deg": round(float(np.median(tot_errs)), 2) if tot_errs else 0.0,
        "mean_pan_err_deg": round(sum(pan_errs) / len(pan_errs), 2) if pan_errs else 0.0,
        "mean_tilt_err_deg": round(sum(tilt_errs) / len(tilt_errs), 2) if tilt_errs else 0.0,
        "mean_cmd_pan_deg": round(sum(cmd_pans) / len(cmd_pans), 2) if cmd_pans else 0.0,
        "mean_cmd_tilt_deg": round(sum(cmd_tilts) / len(cmd_tilts), 2) if cmd_tilts else 0.0,
        "actuator_saturation_pct": round(sat_pct, 1),
        "max_lock_streak_frames": max_streak,
        "avg_lock_streak_frames": round(avg_streak, 1),
        "control_modes": mode_str,
        "status": status,
    }


def main():
    csv_files = sorted(glob.glob(str(LOGS_DIR / "*.csv")))
    summary_list = []

    print(f"Toplam {len(csv_files)} telemetri log dosyasi birlestiriliyor...")

    for f in csv_files:
        if "consolidated" in os.path.basename(f):
            continue
        res = analyze_single_log(f)
        if res:
            summary_list.append(res)

    # 1. Birlestirilmis Ozet CSV Yaz
    summary_csv_path = LOGS_DIR / "consolidated_runs_summary.csv"
    if summary_list:
        headers = list(summary_list[0].keys())
        with open(summary_csv_path, "w", newline="", encoding="utf-8") as fp:
            writer = csv.DictWriter(fp, fieldnames=headers)
            writer.writeheader()
            writer.writerows(summary_list)
        print(f"[OK] Birlestirilmis telemetri ozeti kaydedildi: {summary_csv_path}")

    # 2. 12 Ekim Testleri Karsilastirma Grafigi Uret
    oct12_runs = [r for r in summary_list if "20261012" in r["filename"]]
    if oct12_runs:
        plot_oct12_comparison(oct12_runs)


def plot_oct12_comparison(runs):
    labels = []
    lock_rates = []
    med_errors = []
    mean_errors = []
    sat_rates = []
    durations = []

    for r in runs:
        fn = r["filename"]
        if "145309" in fn:
            name = "Test 14:53\n(PID/RL/HYB)"
        elif "175926" in fn:
            name = "Test 17:59\n(Stabil P)"
        elif "180555" in fn:
            name = "Test 18:05\n(Stabil P)"
        elif "180748" in fn:
            name = "Test 18:07\n(Stabil P Uzun)"
        elif "failed" in fn:
            name = "Test 20:55\n(Adaptif Basarisiz)"
        else:
            name = fn[:18]
        labels.append(name)
        lock_rates.append(r["lock_when_detected_pct"])
        med_errors.append(r["median_error_deg"])
        mean_errors.append(r["mean_error_deg"])
        sat_rates.append(r["actuator_saturation_pct"])
        durations.append(r["duration_sec"])

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), dpi=300)
    fig.patch.set_facecolor("#FAFAFA")

    x = np.arange(len(labels))
    width = 0.55

    # 1. Kilitlenme Orani (%)
    ax = axes[0, 0]
    ax.set_facecolor("#FFFFFF")
    colors_lock = ["#2E7D32" if v > 30 else ("#1976D2" if v > 15 else "#D32F2F") for v in lock_rates]
    bars = ax.bar(x, lock_rates, width, color=colors_lock, edgecolor="#333", alpha=0.88)
    ax.set_title("Hedef Gorunurken Kilit Basarimi (%)", fontsize=12, fontweight="bold", pad=10)
    ax.set_ylabel("Kilit Orani (%)", fontsize=10)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylim(0, 60)
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    for bar in bars:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2.0, h + 1.0, f"%{h:.1f}", ha="center", va="bottom", fontsize=9, fontweight="bold")

    # 2. Medyan ve Ortalama Aci Hatasi (derece)
    ax = axes[0, 1]
    ax.set_facecolor("#FFFFFF")
    w2 = 0.30
    b1 = ax.bar(x - w2 / 2, med_errors, w2, label="Medyan Hata", color="#0288D1", edgecolor="#333", alpha=0.88)
    b2 = ax.bar(x + w2 / 2, mean_errors, w2, label="Ortalama Hata", color="#FB8C00", edgecolor="#333", alpha=0.88)
    ax.set_title("Acisal Takip Hatasi (Derece - Dusuk Olan Iyi)", fontsize=12, fontweight="bold", pad=10)
    ax.set_ylabel("Acisal Hata (°)", fontsize=10)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylim(0, 15)
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    ax.legend(loc="upper left")
    for bar in b1:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2.0, h + 0.3, f"{h:.1f}°", ha="center", va="bottom", fontsize=8)

    # 3. Motor Doyum Orani (%)
    ax = axes[1, 0]
    ax.set_facecolor("#FFFFFF")
    colors_sat = ["#D32F2F" if v > 20 else "#388E3C" for v in sat_rates]
    bars = ax.bar(x, sat_rates, width, color=colors_sat, edgecolor="#333", alpha=0.88)
    ax.set_title("Eyleyici Doyum Orani (|Komut| >= 2.70°)", fontsize=12, fontweight="bold", pad=10)
    ax.set_ylabel("Doyum Orani (%)", fontsize=10)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylim(0, 65)
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    for bar in bars:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2.0, h + 1.0, f"%{h:.1f}", ha="center", va="bottom", fontsize=9, fontweight="bold")

    # 4. Test Suresi ve Toplam Ornek
    ax = axes[1, 1]
    ax.set_facecolor("#FFFFFF")
    bars = ax.bar(x, durations, width, color="#5E35B1", edgecolor="#333", alpha=0.88)
    ax.set_title("Oturum Test Suresi (Saniye)", fontsize=12, fontweight="bold", pad=10)
    ax.set_ylabel("Sure (s)", fontsize=10)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.grid(axis="y", linestyle="--", alpha=0.5)
    for bar in bars:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2.0, h + 15.0, f"{int(h)}s", ha="center", va="bottom", fontsize=9, fontweight="bold")

    plt.suptitle("TURRET FCS // 12 EKIM 2026 SAHA TESTLERI KARSILASTIRMALI ANALIZ RAPORU", fontsize=14, fontweight="bold", y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    out_fig = FIG_DIR / "oct12_field_tests_analysis.png"
    plt.savefig(out_fig, dpi=300)
    plt.close()
    print(f"[OK] Karsilastirmali analiz grafigi olusturuldu: {out_fig}")


if __name__ == "__main__":
    main()
