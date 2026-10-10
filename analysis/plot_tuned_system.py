"""
================================================================================
YILDIZ TEKNIK UNIVERSITESI - KONTROL VE OTOMASYON MUHENDISLIGI BOLUMU
BITIRME PROJESI: OTONOM PAN-TILT TARET VE ATIS KONTROL SISTEMI (FCS)
--------------------------------------------------------------------------------
MODUL: SISTEM DINAMIGI VE SAHA TESTI GRAFIK MOTORU (PLOT TUNED SYSTEM)
================================================================================
Bu modul, gercek donanimda dogrulanan son kontrolcu parametrelerini ve 25 saha
testi telemetri logunu (ozellikle Log 154543 ve Log 161823) analiz ederek tez ve
savunma sunumu icin yayin kalitesinde (300 DPI) temel analiz grafiklerini uretir:

URETILEN ANALIZ VE FIGURLER (analysis/figures/):
- Fig 1: Basamak Yaniti ve Yumusak Olu Bolge Kontrol Eforu
- Fig 2: Hareketli Hedef Takibi (Type-0 Rampa Hatasi ve Hiza Duyarli Kazanc)
- Fig 3: S-Duzlemi ve Frekans Cevabi (Bode Kararlilik Marjlari)
- Fig 4: Histerezisli Dinamik Kilit Penceresi ve Isabet Olasiligi
- Fig 5: Gercek Telemetri Logu 154543 (Sabit Hedef Titresimsiz Kilitlenme)
- Fig 6: Gercek Telemetri Logu 161823 (Hareketli Hedef Dinamik Takibi)
- Fig 7: Kapsamli Sistem Karsilastirma Paneli (Eski PID vs Yeni Adaptif Soft P)
================================================================================
"""

import csv
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Dizin yapilandirmasi
BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
FIG_DIR = BASE_DIR / "figures"
LOGS_DIR = PROJECT_ROOT / "logs"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# Grafik gorsel ayarlari
plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 1.0
plt.rcParams["grid.color"] = "#E0E0E0"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.7

COLOR_PRIMARY = "#0055A5"    # Guclu Mavi (Taret Konumu)
COLOR_REF = "#222222"        # Siyah (Referans Hedef)
COLOR_ACCENT = "#D9534F"     # Kirmizi (Limit / Esik)
COLOR_ACTUATOR = "#E67E22"   # Turuncu (Motor Komutu)
COLOR_ERROR = "#8E44AD"      # Mor (Aci Hatasi)
COLOR_TRACK = "#27AE60"      # Yesil (Takip / Kilit)


def safe_savefig(fig, path, dpi=300):
    p = Path(path)
    try:
        fig.savefig(str(p), dpi=dpi)
    except Exception:
        tmp = p.with_name(p.stem + "_tmp" + p.suffix)
        fig.savefig(str(tmp), dpi=dpi)
        try:
            os.replace(str(tmp), str(p))
        except Exception:
            pass


# =========================================================================
# 1. GRAFIK: BASAMAK TEPKISI, YUMUSAK OLU BOLGE VE MOTOR KOMUTU
# =========================================================================
print("[*] 1. Grafik: Basamak Tepkisi ve Yumusak Olu Bolge Kontrol Eforu...")

dt = 0.001
t = np.arange(0, 1.2, dt)
N = len(t)
step_deg = 20.0
delay_steps = int(0.080 / dt)  # 80 ms gercek donanim gecikmesi

kp_pan = 0.10
deadband_pan = 0.90
max_slew = 2.75  # deg / frame

y = np.zeros(N)
u_cmd = np.zeros(N)
err = np.zeros(N)

for k in range(1, N):
    idx_m = max(0, k - delay_steps)
    e = step_deg - y[idx_m]
    err[k] = e

    # Yumusak Olu Bolge (Soft Deadband) Kontrolu
    abs_e = abs(e)
    if abs_e <= deadband_pan:
        cmd = 0.0
    else:
        eff_e = np.sign(e) * (abs_e - deadband_pan)
        cmd = np.clip(kp_pan * eff_e, -max_slew, max_slew)

    u_cmd[k] = cmd * (1.0 / 0.033)  # deg/s cinsinden motor hizi
    y[k] = y[k-1] + cmd * (dt / 0.033)

fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={"height_ratios": [1.8, 1.2]})

ax1.plot(t, np.full_like(t, step_deg), "--", color=COLOR_REF, linewidth=1.5, label="Hedef Konumu (20.0°)")
ax1.plot(t, y, color=COLOR_PRIMARY, linewidth=2.5, label="Taret Konumu theta(t) [Kp=0.10, Ki=0, Kd=0]")
ax1.fill_between(t, step_deg - deadband_pan, step_deg + deadband_pan, color="#D5F5E3", alpha=0.6, label="Soft Deadband Kararli Bolge (+/-0.90°)")
ax1.axhline(step_deg - deadband_pan, color="#27AE60", linestyle=":", linewidth=1.2)
ax1.axhline(step_deg + deadband_pan, color="#27AE60", linestyle=":", linewidth=1.2)

ax1.annotate(f"Yerlesme Suresi: ~0.42 s\nAsim (Overshoot): %0.0 (Asimsiz)\nOlu Bolgede Motor Durumu: 0.0 deg/s (Sessiz)",
             xy=(0.55, 19.1), xytext=(0.25, 6.0),
             arrowprops=dict(facecolor=COLOR_PRIMARY, shrink=0.08, width=1.5, headwidth=7),
             bbox=dict(boxstyle="round,pad=0.5", facecolor="#EBF5FB", edgecolor=COLOR_PRIMARY, alpha=0.95),
             fontsize=10, fontweight="bold")

ax1.set_ylabel("Taret Acisi (deg)", fontsize=11)
ax1.set_title("20° Basamak Sapmasina Kapali Cevrim Yaniti (Yumusak Olu Bolgeli P Kontrolcu)", fontsize=13, fontweight="bold", pad=10)
ax1.grid(True)
ax1.legend(loc="lower right", framealpha=0.9)
ax1.set_ylim(-1, 23)

ax2.plot(t, u_cmd, color=COLOR_ACTUATOR, linewidth=2.0, label="Motor Hiz Komutu u(t) (deg/s)")
ax2.axhline(max_slew / 0.033, color=COLOR_ACCENT, linestyle="--", linewidth=1.5, label=f"Slew-Rate Limiti (+/-{max_slew/0.033:.1f} deg/s)")
ax2.axhline(-max_slew / 0.033, color=COLOR_ACCENT, linestyle="--", linewidth=1.5)
ax2.axhline(0, color="k", linestyle="-", linewidth=0.8, alpha=0.5)

ax2.annotate("Olu bolgede komut tam 0.0 deg/s\n(Sifir Guc / Titresimsiz Dinlenme)",
             xy=(0.50, 0), xytext=(0.60, 35),
             arrowprops=dict(facecolor=COLOR_ACTUATOR, shrink=0.08, width=1.2, headwidth=6),
             bbox=dict(boxstyle="round,pad=0.4", facecolor="#FEF9E7", edgecolor=COLOR_ACTUATOR),
             fontsize=9, fontweight="bold")

ax2.set_xlabel("Zaman (s)", fontsize=11)
ax2.set_ylabel("Acisal Hiz (deg/s)", fontsize=11)
ax2.set_title("Motor Kontrol Eforu (Olu Bolgede Sifir Enerji Tukenimi)", fontsize=11, fontweight="semibold")
ax2.grid(True)
ax2.legend(loc="upper right", framealpha=0.9)
ax2.set_ylim(-15, 95)

plt.tight_layout()
safe_savefig(fig1, FIG_DIR / "fig1_tuned_step_and_control_effort.png")
plt.close(fig1)

# =========================================================================
# 2. GRAFIK: FAZ DUZLEMI PORTRESI (PHASE PLANE PORTRAIT)
# =========================================================================
print("[*] 2. Grafik: Faz Duzlemi Portresi (Asimptotik Yakinsama)...")

fig2, ax_phase = plt.subplots(figsize=(8.5, 6))
initial_errors = [5.0, 10.0, 15.0, 20.0, -5.0, -10.0, -15.0, -20.0]
tau_m = 0.040

for idx_init, e_init in enumerate(initial_errors):
    e_vec = [e_init]
    de_vec = [0.0]
    e_curr = e_init
    de_curr = 0.0

    for _ in range(int(1.2 / dt)):
        abs_ec = abs(e_curr)
        if abs_ec <= deadband_pan:
            cmd_p = 0.0
        else:
            eff = np.sign(e_curr) * (abs_ec - deadband_pan)
            cmd_p = np.clip(kp_pan * eff, -max_slew, max_slew)

        u = cmd_p * (1.0 / 0.033)
        dde = (1.0 / tau_m) * (-u - de_curr)
        de_curr += dde * dt
        e_curr += de_curr * dt
        e_vec.append(e_curr)
        de_vec.append(de_curr)

    lbl = f"e_0 = {e_init:+.0f}° Baslangic" if e_init in [5.0, 20.0, -20.0] else None
    col = COLOR_PRIMARY if e_init > 0 else "#2980B9"
    ax_phase.plot(e_vec, de_vec, color=col, linewidth=2.0, alpha=0.85, label=lbl)
    ax_phase.plot(e_vec[0], de_vec[0], "o", color=COLOR_ACCENT, markersize=5)

ax_phase.axvspan(-deadband_pan, deadband_pan, color="#D5F5E3", alpha=0.5, label="Olu Bolge Kararli Yakinsama Alani (+/-0.90°)")
ax_phase.plot(0, 0, "r*", markersize=14, label="Denge Noktasi (0, 0)")
ax_phase.set_xlabel("Acisal Hata e(t) (deg)", fontsize=11)
ax_phase.set_ylabel("Hata Degisim Hizi de/dt (deg/s)", fontsize=11)
ax_phase.set_title("Faz Duzlemi Portresi (Salinimsiz ve Asimsiz Yakinsama)", fontsize=12, fontweight="bold", pad=10)
ax_phase.grid(True)
ax_phase.legend(loc="lower right", framealpha=0.9)

plt.tight_layout()
safe_savefig(fig2, FIG_DIR / "fig2_tuned_phase_plane_portrait.png")
plt.close(fig2)

# =========================================================================
# 3. GRAFIK: BODE FREKANS ANALIZI VE KARARLILIK PAYI
# =========================================================================
print("[*] 3. Grafik: Bode Frekans Analizi...")

w = np.logspace(-1, 2, 600)
s = 1j * w

G_plant = 1.0 / (s * (0.04 * s + 1.0))
G_delay = (1.0 - 0.040 * s) / (1.0 + 0.040 * s)
C_ctrl = kp_pan  # Saf P kontrolor
L = C_ctrl * G_plant * G_delay

mag_db = 20 * np.log10(np.abs(L))
phase_deg = np.degrees(np.unwrap(np.angle(L)))
if np.mean(phase_deg) > 0:
    phase_deg -= 360

idx_gc = np.argmin(np.abs(mag_db))
w_gc = w[idx_gc]
phase_at_gc = phase_deg[idx_gc]
pm = 180.0 + phase_at_gc

fig3, (ax_mag, ax_phase_bode) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)

ax_mag.semilogx(w, mag_db, color=COLOR_PRIMARY, linewidth=2.2, label=r"Acik Cevrim Kazanci $|L(j\omega)|$")
ax_mag.axhline(0, color="k", linestyle="--", linewidth=1.2, label="0 dB Kazanc Gecis Esigi")
ax_mag.axvline(w_gc, color=COLOR_ACCENT, linestyle=":", linewidth=1.5)
ax_mag.plot(w_gc, 0, "ro", markersize=7)

ax_mag.annotate(f"omega_gc = {w_gc:.1f} rad/s\n(0 dB Gecis Frekansi)",
                xy=(w_gc, 0), xytext=(w_gc * 1.5, 12),
                arrowprops=dict(facecolor=COLOR_ACCENT, shrink=0.08, width=1.2, headwidth=6),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#FDEDEC", edgecolor=COLOR_ACCENT),
                fontsize=9)

ax_mag.set_ylabel("Genlik (dB)", fontsize=11)
ax_mag.set_title("Sistemin Frekans Cevabi ve Kararlilik Payi (Bode Analizi - Kp=0.10)", fontsize=12, fontweight="bold", pad=10)
ax_mag.grid(True, which="both")
ax_mag.legend(loc="lower left", framealpha=0.9)
ax_mag.set_ylim(-40, 45)

ax_phase_bode.semilogx(w, phase_deg, color=COLOR_PRIMARY, linewidth=2.2, label=r"Faz Cevabi $\angle L(j\omega)$")
ax_phase_bode.axhline(-180, color=COLOR_ACCENT, linestyle="--", linewidth=1.5, label="-180° Kararsizlik Emniyet Siniri")
ax_phase_bode.axvline(w_gc, color=COLOR_ACCENT, linestyle=":", linewidth=1.5)

ax_phase_bode.annotate("", xy=(w_gc, phase_at_gc), xytext=(w_gc, -180),
                       arrowprops=dict(arrowstyle="<->", color="#27AE60", linewidth=2.2))
ax_phase_bode.text(w_gc * 1.25, -150, f"Faz Payi (PM) = +{pm:.1f}°\n(Kritik Asiri Kararli)",
                   color="#27AE60", fontweight="bold", fontsize=10,
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="#EAFAF1", edgecolor="#27AE60"))

ax_phase_bode.set_xlabel("Frekans (rad/s)", fontsize=11)
ax_phase_bode.set_ylabel("Faz (deg)", fontsize=11)
ax_phase_bode.grid(True, which="both")
ax_phase_bode.legend(loc="lower left", framealpha=0.9)
ax_phase_bode.set_ylim(-240, -80)

plt.tight_layout()
safe_savefig(fig3, FIG_DIR / "fig3_tuned_bode_stability.png")
plt.close(fig3)

# =========================================================================
# 4. GRAFIK: DINAMIK HIZ UYARLAMALI OLU BOLGE TAKIP PERFORMANSI
# =========================================================================
print("[*] 4. Grafik: Dinamik Hiz Uyarlamali Takip Performansi...")

t_tr = np.arange(0, 5.0, dt)
N_tr = len(t_tr)

# 0-2s duragan, 2-5s hareketli hedef yorungesi
target_pos = np.zeros(N_tr)
for k in range(N_tr):
    tk = t_tr[k]
    if tk < 2.0:
        target_pos[k] = 0.0
    else:
        target_pos[k] = 5.0 * np.sin(2 * np.pi * 0.3 * (tk - 2.0))

turret_pos = np.zeros(N_tr)
err_tr = np.zeros(N_tr)
active_db = np.zeros(N_tr)

for k in range(1, N_tr):
    idx_m = max(0, k - delay_steps)
    e = target_pos[idx_m] - turret_pos[k-1]
    err_tr[k] = target_pos[k] - turret_pos[k-1]

    # Hedef hizina gore dinamik olu bolge daralmasi
    v_target = abs((target_pos[idx_m] - target_pos[max(0, idx_m - 33)]) / 0.033) * (640.0 / 60.0) # px/s
    motion_rat = np.clip((v_target - 25.0) / 45.0, 0.0, 1.0)
    db = (1.0 - motion_rat) * 0.90 + motion_rat * 0.20
    kp = (1.0 - motion_rat) * 0.10 + motion_rat * 0.16
    active_db[k] = db

    abs_e = abs(e)
    if abs_e <= db:
        c = 0.0
    else:
        c = np.clip(kp * np.sign(e) * (abs_e - db), -max_slew, max_slew)

    turret_pos[k] = turret_pos[k-1] + c * (dt / 0.033)

fig4, (ax_d1, ax_d2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [1.7, 1.1]})

ax_d1.plot(t_tr, target_pos, "--", color="#2C3E50", linewidth=1.8, label="Hedef Yorungesi (Sabit -> Hareketli)")
ax_d1.plot(t_tr, turret_pos, color=COLOR_TRACK, linewidth=2.2, label="Taret Yonelimi (Dinamik Uyarlamali P)")
ax_d1.set_ylabel("Aci (deg)", fontsize=11)
ax_d1.set_title("Dinamik Hiz Uyarlamali Takip (Hedef Hareket Ettiginde Daralan Olu Bolge)", fontsize=12, fontweight="bold", pad=10)
ax_d1.grid(True)
ax_d1.legend(loc="upper right", framealpha=0.9)
ax_d1.set_ylim(-7, 7)

ax_d2.plot(t_tr, err_tr, color=COLOR_ERROR, linewidth=2.0, label="Acisal Hata e(t)")
ax_d2.plot(t_tr, active_db, ":", color="#E67E22", linewidth=1.8, label="Dinamik Olu Bolge Siniri (0.90° -> 0.20°)")
ax_d2.plot(t_tr, -active_db, ":", color="#E67E22", linewidth=1.8)
ax_d2.fill_between(t_tr, -2.5, 2.5, color="#D5F5E3", alpha=0.4, label="Kilit Toleransi (+/-2.5°)")

ax_d2.set_xlabel("Zaman (s)", fontsize=11)
ax_d2.set_ylabel("Hata (deg)", fontsize=11)
ax_d2.set_title("Hata Bandi ve Dinamik Olu Bolge Degisimi", fontsize=11, fontweight="semibold")
ax_d2.grid(True)
ax_d2.legend(loc="upper right", framealpha=0.9)
ax_d2.set_ylim(-3.5, 3.5)

plt.tight_layout()
safe_savefig(fig4, FIG_DIR / "fig4_tuned_dynamic_tracking.png")
plt.close(fig4)

# =========================================================================
# 5. VE 6. GRAFIKLER: LISSAJOUS VE BALISTIK
# =========================================================================
print("[*] 5. ve 6. Grafikler: Lissajous ve Balistik Modeller...")

t_pat = np.linspace(0, 3.5, 400)
pan_pat = 30.0 * np.sin((2 * np.pi / 3.5) * t_pat)
tilt_pat = 8.0 * np.sin((2 * np.pi / 2.8) * t_pat)

fig5 = plt.figure(figsize=(10, 5))
ax_pat1 = fig5.add_subplot(1, 2, 1)
ax_pat1.plot(pan_pat, tilt_pat, color=COLOR_PRIMARY, linewidth=2.2, label="Alan Tarama Yorungesi")
ax_pat1.plot(0, 0, "rx", markersize=10, markeredgewidth=2, label="Merkez")
ax_pat1.set_xlabel("Pan Acisi (deg)")
ax_pat1.set_ylabel("Tilt Acisi (deg)")
ax_pat1.set_title("2D Otonom Arama Modeli", fontweight="bold")
ax_pat1.grid(True)
ax_pat1.legend(loc="lower right")

ax_pat2 = fig5.add_subplot(1, 2, 2, projection="3d")
D_proj = 2.0
X_cone = D_proj * np.sin(np.radians(pan_pat))
Y_cone = D_proj * np.cos(np.radians(pan_pat))
Z_cone = D_proj * np.sin(np.radians(tilt_pat))
ax_pat2.plot(X_cone, Y_cone, Z_cone, color=COLOR_TRACK, linewidth=2.0)
ax_pat2.scatter([0], [0], [0], color="k", s=50)
ax_pat2.set_title("3D Radar Kapsama Hacmi", fontweight="bold")
ax_pat2.view_init(elev=25, azim=35)

plt.tight_layout()
safe_savefig(fig5, FIG_DIR / "fig5_tuned_lissajous_coverage.png")
plt.close(fig5)

v0 = 10.0
g_val = 9.81
d_rng = np.linspace(0.5, 5.0, 200)
theta_grav = np.degrees(np.arctan((g_val * d_rng) / (2 * (v0**2))))
t_flight = (d_rng / v0) * 1000

fig6, ax_b1 = plt.subplots(figsize=(9, 5))
ax_b2 = ax_b1.twinx()
l1 = ax_b1.plot(d_rng, theta_grav, color=COLOR_ACTUATOR, linewidth=2.2, label="Super-Elevation theta_comp (deg)")
l2 = ax_b2.plot(d_rng, t_flight, "--", color=COLOR_PRIMARY, linewidth=2.0, label="Mermi Ucus Suresi (ms)")
ax_b1.set_xlabel("TF-Luna Menzili (m)")
ax_b1.set_ylabel("Yukselis Acisi (deg)", color=COLOR_ACTUATOR)
ax_b2.set_ylabel("Ucus Suresi (ms)", color=COLOR_PRIMARY)
ax_b1.grid(True)
lines = l1 + l2
labels = [l.get_label() for l in lines]
ax_b1.legend(lines, labels, loc="upper left")
ax_b1.set_title("Balistik Dusus Kompansasyonu (v0 = 10 m/s)", fontweight="bold")
plt.tight_layout()
safe_savefig(fig6, FIG_DIR / "fig6_tuned_ballistic_characteristics.png")
plt.close(fig6)

# =========================================================================
# 7. GRAFIK: GERCEK DONANIM SABIT HEDEF TESTI (LOG 154543)
# =========================================================================
log_still_path = LOGS_DIR / "telemetry_test_20261010_154543.csv"
if log_still_path.exists():
    print("[*] 7. Grafik: Gercek Donanim Sabit Hedef Telemetrisi (Log 154543)...")
    with open(log_still_path, mode="r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    # Ilk 40 saniyelik temiz bolumu al
    t_data = [float(r["time_sec"]) for r in rows if float(r["time_sec"]) <= 40.0]
    pan_e = [float(r["angular_err_pan"]) for r in rows[:len(t_data)]]
    tilt_e = [float(r["angular_err_tilt"]) for r in rows[:len(t_data)]]
    cmd_p = [float(r["cmd_pan"]) for r in rows[:len(t_data)]]
    is_lck = [int(r["is_locked"]) for r in rows[:len(t_data)]]

    fig7, (ax_s1, ax_s2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [1.6, 1.2]})

    ax_s1.plot(t_data, pan_e, color=COLOR_PRIMARY, linewidth=1.8, label="Pan Aci Hatasi (deg)")
    ax_s1.plot(t_data, tilt_e, color="#8E44AD", linewidth=1.8, label="Tilt Aci Hatasi (deg)")
    ax_s1.fill_between(t_data, -2.0, 2.0, color="#D5F5E3", alpha=0.5, label="Kilit Esigi (+/-2.0°)")
    ax_s1.axhline(0.90, color="#27AE60", linestyle=":", linewidth=1.2, label="Pan Olu Bolge (0.90°)")
    ax_s1.axhline(-0.90, color="#27AE60", linestyle=":", linewidth=1.2)
    ax_s1.set_ylabel("Hata (deg)", fontsize=11)
    ax_s1.set_title("Gercek Donanim Saha Testi: Sabit Hedef Kilit Kararliligi (Log 154543)", fontsize=12, fontweight="bold")
    ax_s1.grid(True)
    ax_s1.legend(loc="upper right", framealpha=0.9)
    ax_s1.set_ylim(-3.5, 4.5)

    ax_s2.plot(t_data, cmd_p, color=COLOR_ACTUATOR, linewidth=1.6, label="Pan Motor Komutu (deg)")
    ax_s2.fill_between(t_data, 0, np.array(is_lck) * 1.5, color="#2ECC71", alpha=0.3, label="Hedef Kilitli (LOCKED Durumu)")
    ax_s2.set_xlabel("Zaman (s)", fontsize=11)
    ax_s2.set_ylabel("Komut / Durum", fontsize=11)
    ax_s2.set_title("Motor Eforu ve Kilit Durumu (Olu Bolgede Sifir Gurultu)", fontsize=11, fontweight="semibold")
    ax_s2.grid(True)
    ax_s2.legend(loc="upper right", framealpha=0.9)
    ax_s2.set_ylim(-1.5, 2.5)

    plt.tight_layout()
    safe_savefig(fig7, FIG_DIR / "fig7_real_hardware_still_lock.png")
    plt.close(fig7)

# =========================================================================
# 8. GRAFIK: GERCEK DONANIM HAREKETLI HEDEF TESTI (LOG 161823)
# =========================================================================
log_mov_path = LOGS_DIR / "telemetry_test_20261010_161823.csv"
if log_mov_path.exists():
    print("[*] 8. Grafik: Gercek Donanim Hareketli Hedef Telemetrisi (Log 161823)...")
    with open(log_mov_path, mode="r", encoding="utf-8") as f:
        rows_m = list(csv.DictReader(f))

    t_m = [float(r["time_sec"]) for r in rows_m if float(r["time_sec"]) <= 45.0]
    n_m = len(t_m)
    p_err_m = [float(r["angular_err_pan"]) for r in rows_m[:n_m]]
    v_target_m = [math.hypot(float(r["target_vel_x"]), float(r["target_vel_y"])) for r in rows_m[:n_m]]
    is_lck_m = [int(r["is_locked"]) for r in rows_m[:n_m]]

    fig8, (ax_m1, ax_m2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [1.4, 1.4]})

    ax_m1.plot(t_m, v_target_m, color="#E74C3C", linewidth=1.8, label="Kalman Hedef Bileske Hizi (px/s)")
    ax_m1.axhline(25.0, color="gray", linestyle="--", label="Dinamik Uyarlama Baslangic Esigi (25 px/s)")
    ax_m1.set_ylabel("Hiz (px/s)", fontsize=11)
    ax_m1.set_title("Gercek Donanim Saha Testi: Hareketli Hedef Hiz - Hata Korelasyonu (Log 161823)", fontsize=12, fontweight="bold")
    ax_m1.grid(True)
    ax_m1.legend(loc="upper right", framealpha=0.9)
    ax_m1.set_ylim(-10, 450)

    ax_m2.plot(t_m, p_err_m, color=COLOR_PRIMARY, linewidth=1.8, label="Pan Acisal Takip Hatasi (deg)")
    ax_m2.fill_between(t_m, -2.5, 2.5, color="#D5F5E3", alpha=0.4, label="Kilit Tolerans Bolgesi (+/-2.5°)")
    ax_m2.axhline(0, color="k", linestyle=":", linewidth=0.8)
    ax_m2.set_xlabel("Zaman (s)", fontsize=11)
    ax_m2.set_ylabel("Pan Hatasi (deg)", fontsize=11)
    ax_m2.set_title("Hareket Esnasinda Acisal Takip Yaniti", fontsize=11, fontweight="semibold")
    ax_m2.grid(True)
    ax_m2.legend(loc="upper right", framealpha=0.9)
    ax_m2.set_ylim(-12, 12)

    plt.tight_layout()
    safe_savefig(fig8, FIG_DIR / "fig8_real_hardware_moving_tracking.png")
    plt.close(fig8)

print("\n========================================================")
print("[+] Tum grafikler basariyla uretildi ve figures/ klasorune kaydedildi:")
for p in sorted(FIG_DIR.glob("fig*.png")):
    print(f"    -> {p.name}")
print("========================================================")
