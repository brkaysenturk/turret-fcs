#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET FCS BIRLESIK TELEMETRI VE LOG INCELEME MOTORU (INSPECT_LOGS)
================================================================================
Bu arac; taret atis kontrol sisteminin (FCS) gercek donanim veya simülasyon test
loglarini (logs/*.csv) ayrintili olarak inceler.

ONCEKI DAGINIK BETIKLERIN TEK BIR CATI ALTINDA TOPLANMIS HALIDIR:
- quick_inspect.py, analyze_test2.py, analyze_vertical.py
- find_resonance.py, find_tilt_osc.py, inspect_tilt.py, inspect_15s.py
- check_jitter.py, check_lock_streak.py, check_transition.py
- trace_kalman.py, trace_trajectory.py

KULLANIM:
  python inspect_logs.py                         # En son olusturulan log dosyasini otomatik inceler
  python inspect_logs.py logs/telemetry_...csv   # Belirtilen log dosyasini inceler
  python inspect_logs.py --trace 25 35           # 25-35. saniyeler arasindaki kareleri tablo olarak doker
  python inspect_logs.py --oscillations          # Yalnizca eksenel rezonans ve salinim analizi yapar
  python inspect_logs.py --streaks               # Kesintisiz kilit serilerini ve titresimi listeler
================================================================================
"""

import argparse
import csv
import glob
import math
import os
import sys


def mean(lst):
    return sum(lst) / len(lst) if lst else 0.0


def std(lst):
    if len(lst) < 2:
        return 0.0
    m = mean(lst)
    return math.sqrt(sum((x - m) ** 2 for x in lst) / len(lst))


def safe_float(val, default=0.0):
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def find_latest_log():
    patterns = [
        "logs/telemetry_test_*.csv",
        "../logs/telemetry_test_*.csv",
        "logs/*.csv",
        "../logs/*.csv",
    ]
    all_files = []
    for p in patterns:
        all_files.extend(
            f for f in glob.glob(p)
            if not f.endswith("_events.csv") and "consolidated_runs_summary" not in f
        )
    if not all_files:
        return None
    return max(all_files, key=os.path.getctime)


def load_telemetry(filepath):
    if not os.path.isfile(filepath):
        print(f"[HATA] Dosya bulunamadi: {filepath}")
        sys.exit(1)

    with open(filepath, mode="r", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows


def analyze_overview(rows, filepath):
    total_frames = len(rows)
    if total_frames == 0:
        print("[UYARI] Log dosyasi bos!")
        return

    t0 = safe_float(rows[0].get("time_sec", 0.0))
    t1 = safe_float(rows[-1].get("time_sec", 0.0))
    duration_sec = max(0.0, t1 - t0)

    fps_vals = [safe_float(r.get("fps")) for r in rows if safe_float(r.get("fps")) > 0]
    avg_fps = mean(fps_vals)

    det_rows = [r for r in rows if r.get("target_detected") in ["1", "True"]]
    locked_rows = [r for r in rows if r.get("is_locked") in ["1", "True"]]

    modes = {}
    for r in rows:
        m = r.get("control_mode", "UNKNOWN")
        modes[m] = modes.get(m, 0) + 1

    print("\n" + "=" * 78)
    print(f" TURRET FCS TELEMETRI RAPORU: {os.path.basename(filepath)}")
    print("=" * 78)
    print(f"Toplam Sure       : {duration_sec:.2f} saniye ({total_frames} kare)")
    print(f"Ortalama Hiz      : {avg_fps:.1f} FPS (dt: {1000.0/max(1.0, avg_fps):.1f} ms)")
    print(f"Hedef Tespiti     : {len(det_rows)}/{total_frames} kare (%{len(det_rows)/total_frames*100:.1f})")
    print(f"Kilit Basarimi    : {len(locked_rows)}/{total_frames} kare (%{len(locked_rows)/total_frames*100:.1f} genel | %{(len(locked_rows)/len(det_rows)*100 if det_rows else 0.0):.1f} tespit sirasinda)")

    mode_str = ", ".join([f"{k}: {v} kare (%{v/total_frames*100:.1f})" for k, v in modes.items()])
    print(f"Kontrol Modlari   : {mode_str}")
    print("-" * 78)


def analyze_axes_kinematics(rows):
    det_rows = [r for r in rows if r.get("target_detected") in ["1", "True"]]
    if not det_rows:
        print("[BILGI] Hedef tespit karesi yok.")
        return

    pan_errs = [safe_float(r.get("angular_err_pan")) for r in det_rows]
    tilt_errs = [safe_float(r.get("angular_err_tilt")) for r in det_rows]
    cmd_pans = [safe_float(r.get("cmd_pan")) for r in det_rows]
    cmd_tilts = [safe_float(r.get("cmd_tilt")) for r in det_rows]
    tot_errs = [math.hypot(p, t) for p, t in zip(pan_errs, tilt_errs)]
    hit_probs = [safe_float(r.get("hit_prob")) for r in det_rows]

    print("\n--- 1. EKSEN KINEMATIGI VE HATA ISTATISTIKLERI (Yalnizca Hedef Varken) ---")
    print(f"{'Metrik':<20} | {'Yatay (Pan)':<22} | {'Dikey (Tilt)':<22} | {'Bileske / Skor':<15}")
    print("-" * 78)
    print(f"{'Ortalama Hata':<20} | {mean(pan_errs):+6.2f} deg (std: {std(pan_errs):.2f} deg)     | {mean(tilt_errs):+6.2f} deg (std: {std(tilt_errs):.2f} deg)     | {mean(tot_errs):.2f} deg")
    print(f"{'Ort. Mutlak Hata':<20} | {mean([abs(x) for x in pan_errs]):6.2f} deg                 | {mean([abs(x) for x in tilt_errs]):6.2f} deg                 | -")
    print(f"{'Hata Araligi (Min/Max)':<20} | [{min(pan_errs):+5.1f} deg, {max(pan_errs):+5.1f} deg]        | [{min(tilt_errs):+5.1f} deg, {max(tilt_errs):+5.1f} deg]        | [0.0 deg, {max(tot_errs):.1f} deg]")
    print(f"{'Ort. Komut Adimi':<20} | {mean([abs(x) for x in cmd_pans]):6.4f} deg/kare           | {mean([abs(x) for x in cmd_tilts]):6.4f} deg/kare          | -")
    print(f"{'Maks. Komut Adimi':<20} | {max([abs(x) for x in cmd_pans]):6.4f} deg/kare           | {max([abs(x) for x in cmd_tilts]):6.4f} deg/kare          | -")
    print(f"{'Ortalama Vurus Olas.':<20} | {'-':<22} | {'-':<22} | %{mean(hit_probs):.1f} (Peak: %{max(hit_probs):.1f})")


def analyze_oscillations(rows):
    det_rows = [r for r in rows if r.get("target_detected") in ["1", "True"]]
    if len(det_rows) < 2:
        return

    cmd_p = [safe_float(r.get("cmd_pan")) for r in det_rows]
    cmd_t = [safe_float(r.get("cmd_tilt")) for r in det_rows]
    err_t = [safe_float(r.get("angular_err_tilt")) for r in det_rows]
    times = [safe_float(r.get("time_sec")) for r in det_rows]

    p_flips = sum(1 for i in range(len(cmd_p) - 1) if cmd_p[i] * cmd_p[i + 1] < -0.0001)
    t_flips = sum(1 for i in range(len(cmd_t) - 1) if cmd_t[i] * cmd_t[i + 1] < -0.0001)

    print("\n--- 2. KARARLILIK VE SALINIM (OSCILLATION / STICK-SLIP) ANALIZI ---")
    print(f"Yatay (Pan) Komut Yon Degisimi  : {p_flips}/{len(cmd_p)-1} (%{p_flips/max(1, len(cmd_p)-1)*100:.1f})")
    print(f"Dikey (Tilt) Komut Yon Degisimi : {t_flips}/{len(cmd_t)-1} (%{t_flips/max(1, len(cmd_t)-1)*100:.1f})")

    peaks = []
    for i in range(1, len(err_t) - 1):
        d1 = err_t[i] - err_t[i - 1]
        d2 = err_t[i + 1] - err_t[i]
        if d1 * d2 < 0 and abs(err_t[i]) > 0.5:
            peaks.append((times[i], err_t[i], cmd_t[i]))

    swings = []
    for j in range(len(peaks) - 1):
        dt_p = peaks[j + 1][0] - peaks[j][0]
        de_p = abs(peaks[j + 1][1] - peaks[j][1])
        if 0.15 < dt_p < 1.5 and de_p > 1.2:
            swings.append((peaks[j][0], peaks[j + 1][0], dt_p, peaks[j][1], peaks[j + 1][1]))

    print(f"Dikey Buyuk Salinim Sayisi (>1.2 deg genlikli) : {len(swings)}")
    if swings:
        print("  Son tespit edilen salinim ornekleri:")
        for s in swings[:5]:
            print(f"    t={s[0]:.2f}s -> {s[1]:.2f}s (dt={s[2]:.2f}s): Hata {s[3]:+5.2f} deg -> {s[4]:+5.2f} deg")
    else:
        print("  [KUSURSUZ] Sistemde limit cycle veya mekanik rezonans tespit edilmedi.")


def analyze_lock_streaks(rows):
    det_rows = [r for r in rows if r.get("target_detected") in ["1", "True"]]
    streaks = []
    curr = []

    for r in det_rows:
        if r.get("is_locked") in ["1", "True"]:
            curr.append(r)
        else:
            if len(curr) >= 10:
                streaks.append(curr)
            curr = []
    if len(curr) >= 10:
        streaks.append(curr)

    print("\n--- 3. KESINTISIZ KILIT SERILERI (LOCK PERSISTENCE & STABILITY) ---")
    if not streaks:
        print("  10 kareden uzun surdurulebilir kilit serisi bulunamadi.")
        return

    print(f"Toplam Kilit Blogu (>10 kare) : {len(streaks)}")
    longest = max(streaks, key=len)
    t_start = safe_float(longest[0].get("time_sec"))
    t_end = safe_float(longest[-1].get("time_sec"))
    print(f"En Uzun Kesintisiz Kilit      : {len(longest)} kare ({t_end - t_start:.2f} saniye, t={t_start:.1f}s -> {t_end:.1f}s)")

    cys = [safe_float(r.get("box_cy")) for r in longest]
    cxs = [safe_float(r.get("box_cx")) for r in longest]
    t_errs = [safe_float(r.get("angular_err_tilt")) for r in longest]
    p_errs = [safe_float(r.get("angular_err_pan")) for r in longest]
    h_probs = [safe_float(r.get("hit_prob")) for r in longest]

    print(f"  En Uzun Blokta Kutu Titremesi : std_cx={std(cxs):.2f}px | std_cy={std(cys):.2f}px")
    print(f"  En Uzun Blokta Aci Hatasi     : pan={mean(p_errs):+5.2f} deg (std: {std(p_errs):.2f} deg) | tilt={mean(t_errs):+5.2f} deg (std: {std(t_errs):.2f} deg)")
    print(f"  En Uzun Blokta Vurus Olasiligi: %{mean(h_probs):.1f}")


def trace_frames(rows, start_sec=None, end_sec=None, limit=40):
    det_rows = [r for r in rows if r.get("target_detected") in ["1", "True"]]
    if start_sec is not None or end_sec is not None:
        s0 = start_sec if start_sec is not None else 0.0
        s1 = end_sec if end_sec is not None else 9999.0
        sub = [r for r in det_rows if s0 <= safe_float(r.get("time_sec")) <= s1]
    else:
        sub = det_rows[:limit]

    print(f"\n--- 4. ZAMAN TABLOSU DETAYLI KARE DOKUMU (Gosterilen: {len(sub[:limit])} kare) ---")
    print(f"{'Zaman(s)':<8} | {'Mod':<4} | {'PanErr':<10} | {'CmdPan':<10} | {'TiltErr':<10} | {'CmdTilt':<10} | {'VelY(px/s)':<10} | {'Kilit':<5} | {'P_hit':<6}")
    print("-" * 92)

    for r in sub[:limit]:
        t = safe_float(r.get("time_sec"))
        m = r.get("control_mode", "P")[:4]
        pe = safe_float(r.get("angular_err_pan"))
        cp = safe_float(r.get("cmd_pan"))
        te = safe_float(r.get("angular_err_tilt"))
        ct = safe_float(r.get("cmd_tilt"))
        vy = safe_float(r.get("target_vel_y"))
        l = "EVET" if r.get("is_locked") in ["1", "True"] else "HAYIR"
        hp = safe_float(r.get("hit_prob"))
        print(f"{t:7.3f}s | {m:<4} | {pe:+6.2f} deg | {cp:+6.3f} deg | {te:+6.2f} deg | {ct:+6.3f} deg | {vy:+9.1f} | {l:<5} | %{hp:4.1f}")


def main():
    parser = argparse.ArgumentParser(
        description="Turret FCS Telemetri Log Inceleme ve Karsilastirma Araci",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("logfile", nargs="?", default=None, help="Incelenecek telemetri CSV dosyasi (varsayilan: en son olusturulan)")
    parser.add_argument("--trace", nargs=2, type=float, metavar=("START_SEC", "END_SEC"), help="Belirtilen zaman araligini (saniye) kare kare listeler")
    parser.add_argument("--oscillations", action="store_true", help="Yalnizca salinim ve rezonans analizini gosterir")
    parser.add_argument("--streaks", action="store_true", help="Yalnizca kilit serilerini ve kararlilik surelerini gosterir")
    args = parser.parse_args()

    filepath = args.logfile
    if filepath is None:
        filepath = find_latest_log()
        if filepath is None:
            print("[HATA] logs/ dizininde hic telemetri CSV dosyasi bulunamadi.")
            sys.exit(1)

    rows = load_telemetry(filepath)

    if args.oscillations:
        analyze_overview(rows, filepath)
        analyze_oscillations(rows)
    elif args.streaks:
        analyze_overview(rows, filepath)
        analyze_lock_streaks(rows)
    elif args.trace:
        analyze_overview(rows, filepath)
        trace_frames(rows, start_sec=args.trace[0], end_sec=args.trace[1], limit=100)
    else:
        analyze_overview(rows, filepath)
        analyze_axes_kinematics(rows)
        analyze_oscillations(rows)
        analyze_lock_streaks(rows)
        trace_frames(rows, limit=25)


if __name__ == "__main__":
    main()
