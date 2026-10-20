#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET FCS OTONOM DEVIYE (PATROL) LOG VE KINEMATIK ANALIZ ARACI
================================================================================
Bu betik, gercek donanimda veya test ortaminda kaydedilen devriye telemetri
loglarini (logs/telemetry_test_*.csv) analiz eder ve gorsellestirir:

1. Taret Acisal Yonelimi (Odometri Pan vs Tilt 2D Hava Sahasi Deseni)
2. Zaman Boyunca Acisal Hareket (Bekleme -> Merkeze Donus -> Genis Tarama)
3. Acisal Hiz ve Ivme Profilleri (Pruzsuzluk, Jerk ve Titresim Denetimi)
4. Kinematik Limit ve Emniyet Sinirlari Kontrolu (+/-75° Pan, +/-25° Tilt)

KULLANIM:
  python analysis/plot_patrol_log.py                       # En son logu otomatik bulur ve inceler
  python analysis/plot_patrol_log.py logs/telemetry_...csv # Belirtilen logu inceler
================================================================================
"""

import argparse
import csv
import glob
import math
import os
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Estetik ve yayin kalitesinde grafik ayarlari
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "figure.titlesize": 13,
    "lines.linewidth": 1.8,
    "grid.alpha": 0.4,
    "grid.linestyle": "--",
})

COLOR_PAN = "#2980B9"     # Safir Mavi
COLOR_TILT = "#C0392B"    # Koyu Kirmizi
COLOR_VEL = "#27AE60"     # Zumrut Yesil
COLOR_ACCEL = "#8E44AD"   # Mor


def find_latest_log():
    patterns = [
        "logs/telemetry_test_*.csv",
        "../logs/telemetry_test_*.csv",
        "logs/*.csv",
        "../logs/*.csv",
    ]
    all_files = []
    for p in patterns:
        all_files.extend(glob.glob(p))
    if not all_files:
        return None
    return max(all_files, key=os.path.getctime)


def safe_float(val, default=0.0):
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def load_patrol_data(filepath):
    if not os.path.isfile(filepath):
        print(f"[HATA] Dosya bulunamadi: {filepath}")
        sys.exit(1)

    with open(filepath, mode="r", encoding="utf-8", errors="ignore") as f:
        reader = list(csv.DictReader(f))

    if not reader:
        print("[HATA] Log dosyasi bos!")
        sys.exit(1)

    t = []
    cmd_p = []
    cmd_t = []
    turret_p = []
    turret_t = []
    det = []
    status = []

    # Odometri alanlari yoksa cmd birikiminden hesapla
    cum_p = 0.0
    cum_t = 0.0

    for r in reader:
        time_s = safe_float(r.get("time_sec"))
        cp = safe_float(r.get("cmd_pan"))
        ct = safe_float(r.get("cmd_tilt"))
        d = int(safe_float(r.get("target_detected")))
        st = r.get("patrol_status", "STANDBY")

        tp_val = r.get("turret_pan")
        tt_val = r.get("turret_tilt")

        if tp_val is not None and tt_val is not None:
            tp = safe_float(tp_val)
            tt = safe_float(tt_val)
        else:
            cum_p += cp
            cum_t += ct
            tp = cum_p
            tt = cum_t

        t.append(time_s)
        cmd_p.append(cp)
        cmd_t.append(ct)
        turret_p.append(tp)
        turret_t.append(tt)
        det.append(d)
        status.append(st)

    return (
        np.array(t),
        np.array(cmd_p),
        np.array(cmd_t),
        np.array(turret_p),
        np.array(turret_t),
        np.array(det),
        status,
    )


def analyze_and_plot(filepath):
    print("=" * 78)
    print(f" TURRET FCS PATROL TELEMETRI VE KINEMATIK ANALIZI: {os.path.basename(filepath)}")
    print("=" * 78)

    t, cmd_p, cmd_t, pose_p, pose_t, det, status = load_patrol_data(filepath)

    N = len(t)
    duration = t[-1] - t[0] if N > 1 else 0.0
    dt_arr = np.diff(t)
    avg_dt = float(np.mean(dt_arr)) if len(dt_arr) > 0 else 0.033
    avg_fps = (1.0 / avg_dt) if avg_dt > 0 else 30.0

    # Hiz ve ivme hesabi (deg/s ve deg/s^2)
    vel_p = np.zeros(N)
    vel_t = np.zeros(N)
    for i in range(1, N):
        d_time = max(0.005, t[i] - t[i - 1])
        vel_p[i] = (pose_p[i] - pose_p[i - 1]) / d_time
        vel_t[i] = (pose_t[i] - pose_t[i - 1]) / d_time

    acc_p = np.zeros(N)
    acc_t = np.zeros(N)
    for i in range(1, N):
        d_time = max(0.005, t[i] - t[i - 1])
        acc_p[i] = (vel_p[i] - vel_p[i - 1]) / d_time
        acc_t[i] = (vel_t[i] - vel_t[i - 1]) / d_time

    # Istatistikler
    max_vp, max_vt = float(np.max(np.abs(vel_p))), float(np.max(np.abs(vel_t)))
    max_ap, max_at = float(np.max(np.abs(acc_p))), float(np.max(np.abs(acc_t)))
    span_p = float(np.max(pose_p) - np.min(pose_p))
    span_t = float(np.max(pose_t) - np.min(pose_t))

    print(f" Toplam Kayit Suresi     : {duration:.2f} saniye ({N} kare, ortalama {avg_fps:.1f} FPS)")
    print(f" Yatay (Pan) Kapsama     : [{np.min(pose_p):+.1f}°, {np.max(pose_p):+.1f}°] -> Toplam Acisal Genlik: {span_p:.1f}°")
    print(f" Dikey (Tilt) Kapsama    : [{np.min(pose_t):+.1f}°, {np.max(pose_t):+.1f}°] -> Toplam Acisal Genlik: {span_t:.1f}°")
    print(f" Tepe Hizlar (deg/s)     : Pan = {max_vp:.1f}°/s | Tilt = {max_vt:.1f}°/s")
    print(f" Tepe Ivmeler (deg/s^2)  : Pan = {max_ap:.1f}°/s² | Tilt = {max_at:.1f}°/s²")
    print("=" * 78)

    # 4 Panelli Analiz Grafigi
    fig = plt.figure(figsize=(14, 9))

    # Panel 1: 2D Hava Sahasi Tarama Yorungesi (Pan vs Tilt)
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(pose_p, pose_t, color=COLOR_PAN, linewidth=2.0, alpha=0.85, label="Otonom Tarama Yolu")
    if len(pose_p) > 0:
        ax1.plot(pose_p[0], pose_t[0], "go", markersize=8, label="Baslangic")
        ax1.plot(pose_p[-1], pose_t[-1], "bs", markersize=8, label="Bitis")
    ax1.plot(0, 0, "rx", markersize=10, markeredgewidth=2.2, label="Boresight (0°, 0°)")

    # Emniyet Limitleri
    ax1.axvline(-45, color="gray", linestyle=":", alpha=0.6)
    ax1.axvline(45, color="gray", linestyle=":", alpha=0.6)
    ax1.axhline(-16, color="gray", linestyle=":", alpha=0.6)
    ax1.axhline(16, color="gray", linestyle=":", alpha=0.6)

    ax1.set_xlabel("Pan Acisi (deg)", fontweight="bold")
    ax1.set_ylabel("Tilt Acisi (deg)", fontweight="bold")
    ax1.set_title(f"2D Hava Sahasi Kapsama Alani (Span: {span_p:.0f}° x {span_t:.0f}°)", fontweight="bold")
    ax1.grid(True)
    ax1.legend(loc="upper right", framealpha=0.9)

    # Panel 2: Zaman Boyunca Acisal Yonelim
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, pose_p, color=COLOR_PAN, linewidth=1.8, label="Pan Pose (deg)")
    ax2.plot(t, pose_t, color=COLOR_TILT, linewidth=1.8, label="Tilt Pose (deg)")
    ax2.set_xlabel("Zaman (s)", fontweight="bold")
    ax2.set_ylabel("Yonelim Acisi (deg)", fontweight="bold")
    ax2.set_title("Zaman Serisi Acisal Yonelim (Pan & Tilt)", fontweight="bold")
    ax2.grid(True)
    ax2.legend(loc="upper right", framealpha=0.9)

    # Panel 3: Acisal Hizlar (deg/s)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(t, vel_p, color=COLOR_PAN, linewidth=1.5, alpha=0.85, label=f"Pan Hizi (Max: {max_vp:.1f}°/s)")
    ax3.plot(t, vel_t, color=COLOR_TILT, linewidth=1.5, alpha=0.85, label=f"Tilt Hizi (Max: {max_vt:.1f}°/s)")
    ax3.axhline(0, color="black", linestyle="-", linewidth=0.8, alpha=0.5)
    ax3.set_xlabel("Zaman (s)", fontweight="bold")
    ax3.set_ylabel("Acisal Hiz (deg/s)", fontweight="bold")
    ax3.set_title("Eyleyici Hiz Profili (Pruzsuz Donus / Limit Kontrolu)", fontweight="bold")
    ax3.grid(True)
    ax3.legend(loc="upper right", framealpha=0.9)

    # Panel 4: Acisal Ivmeler (Jerk ve Eylemsizlik Sarsintisi Denetimi)
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(t, acc_p, color="#2980B9", linewidth=1.4, alpha=0.75, label=f"Pan Ivme (Tepe: {max_ap:.0f}°/s²)")
    ax4.plot(t, acc_t, color="#C0392B", linewidth=1.4, alpha=0.75, label=f"Tilt Ivme (Tepe: {max_at:.0f}°/s²)")
    ax4.axhline(0, color="black", linestyle="-", linewidth=0.8, alpha=0.5)
    ax4.set_xlabel("Zaman (s)", fontweight="bold")
    ax4.set_ylabel("Acisal Ivme (deg/s²)", fontweight="bold")
    ax4.set_title("Ivme ve Sarsinti (Jerk) Analizi (300g Namlu Eylemsizligi)", fontweight="bold")
    ax4.grid(True)
    ax4.legend(loc="upper right", framealpha=0.9)

    plt.tight_layout()

    out_dir = Path(filepath).parent.parent / "analysis" / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "patrol_kinematic_telemetry_analysis.png"
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[+] Analiz grafigi 300 DPI cozunurlukte kaydedildi: {out_path}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Turret FCS Patrol Telemetry Analysis")
    parser.add_argument("logfile", nargs="?", default=None, help="Incelenecek telemetri CSV dosyasi")
    args = parser.parse_args()

    logfile = args.logfile
    if logfile is None:
        logfile = find_latest_log()
        if logfile is None:
            print("[HATA] logs/ dizininde hic telemetri CSV dosyasi bulunamadi!")
            sys.exit(1)
        print(f"[*] Belirtilen dosya yok, en son log secildi: {logfile}")

    analyze_and_plot(logfile)


if __name__ == "__main__":
    main()
