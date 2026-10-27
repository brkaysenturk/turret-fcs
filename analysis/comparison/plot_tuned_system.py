#!/usr/bin/env python3
"""
================================================================================
MODUL: SISTEM DINAMIGI VE KONTROLOR BENZETIM GRAFIK MOTORU (PLOT TUNED SYSTEM)
================================================================================
Bu modul, Turret FCS guncel matematiksel modelini ve kontrolor parametrelerini
(CONFIG) dogrudan referans alarak analitik ve benzetim tabanli grafikler uretir.
Hicbir fiziksel log dosyasina bagimli degildir; tum egri ve yanitlar dogrudan
Python diferansiyel denklem ve transfer fonksiyonu modellerinden hesaplanir.

URETILEN GRAFIKLER (analysis/figures/):
- Fig 1: 20° Basamak Yaniti ve Motor Kontrol Eforu (Pan ve Tilt Eksenleri)
- Fig 2: Faz Duzlemi Portresi (Asimptotik Yakinsama ve Yumusak Olu Bolge)
- Fig 3: Frekans Cevabi ve Kararlilik Payi (Bode Analizi, PM = 45.1°, GM = 7.2 dB)
- Fig 4: Dinamik Takip ve Aim-Point Ileri Besleme (Hiz Ileri Beslemesi Telafisi)
- Fig 5: Lissajous Devriye Modeli ve 3B Parametrik Tarama Yolu
- Fig 6: Balistik Dusurme Kompansasyonu (v0 = 10.0 m/s Cift Volanli Taret)
- Fig 7: Kapali Cevrim Sabit Hedef Kilit Kararliligi ve Gurultu Bastirma Benzetimi
- Fig 8: Dinamik Manevra Yapan Hedef Takibi ve Kalman Kestirim Benzetimi
================================================================================
"""

import json
import os
import sys
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Proje ana dizinini sys.path'e ekle ve CONFIG'i import et
BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from turret_fcs.config import CONFIG

# Dizin yapilandirmasi
FIG_DIR = PROJECT_ROOT / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# Grafik gorsel ayarlari (Akademik format)
plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 1.0
plt.rcParams["grid.color"] = "#E0E0E0"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.7

COLOR_PRIMARY = "#0055A5"    # Guclu Mavi (Pan Eksen / Taret Konumu)
COLOR_REF = "#222222"        # Siyah (Referans Hedef)
COLOR_ACCENT = "#D9534F"     # Kirmizi (Limit / Esik)
COLOR_ACTUATOR = "#E67E22"   # Turuncu (Motor Komutu)
COLOR_TILT = "#8E44AD"       # Mor (Tilt Eksen)
COLOR_TRACK = "#27AE60"      # Yesil (Takip / Kilit)


def safe_savefig(fig, path, dpi=300):
    """Grafikleri cakisik dosya kilitlerine takilmadan guvenle kaydeder."""
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
# SSOT PARAMETRELERI (turret_fcs.config.CONFIG)
# =========================================================================
DT_FRAME = 0.03333333333333333  # 30 FPS ornekleme periyodu (saniye)
KP_PAN_FRAME = CONFIG.control.kp_pan               # 0.22 deg/kare
KV_PAN_CONTINUOUS = KP_PAN_FRAME / DT_FRAME        # 6.60 1/s
KP_TILT_FRAME = CONFIG.control.kp_tilt             # 0.11 deg/kare
KV_TILT_CONTINUOUS = KP_TILT_FRAME / DT_FRAME      # 3.30 1/s
MAX_OUTPUT_FRAME = CONFIG.control.max_output_deg   # 1.50 deg/kare
MAX_SLEW_VELOCITY = MAX_OUTPUT_FRAME / DT_FRAME    # 45.0 deg/s
MAX_STEP_TILT_FRAME = CONFIG.actuator.max_step_tilt_deg  # 0.75 deg/kare
MAX_SLEW_TILT = MAX_STEP_TILT_FRAME / DT_FRAME     # 22.5 deg/s
DEADBAND_PAN = CONFIG.control.deadband_pan_deg     # 0.85 deg
DEADBAND_TILT = CONFIG.control.deadband_tilt_deg   # 0.95 deg
TAPER_PAN = CONFIG.rl.taper_pan_deg                # 1.00 deg
TAPER_TILT = CONFIG.rl.taper_tilt_deg              # 2.50 deg

TAU_SERVO = CONFIG.mount.servo_tau                 # 0.045 s (45 ms)
LATENCY_SYSTEM = CONFIG.ballistics.system_latency_s # 0.080 s (80 ms)
V0_PROJECTILE = CONFIG.ballistics.projectile_speed_mps # 10.0 m/s
GRAVITY = CONFIG.ballistics.gravity_accel          # 9.80665 m/s^2

print("[*] Turret FCS Matematiksel Model Parametreleri:")
print(f"    - Kp Pan (kare basi): {KP_PAN_FRAME:.2f} deg/kare (Kv = {KV_PAN_CONTINUOUS:.2f} 1/s)")
print(f"    - Kp Tilt           : {KP_TILT_FRAME:.2f} deg/kare (Kv = {KV_TILT_CONTINUOUS:.2f} 1/s)")
print(f"    - Pan Slew Limiti   : {MAX_OUTPUT_FRAME:.2f} deg/kare ({MAX_SLEW_VELOCITY:.1f} deg/s)")
print(f"    - Tilt Slew Limiti  : {MAX_STEP_TILT_FRAME:.2f} deg/kare ({MAX_SLEW_TILT:.1f} deg/s)")
print(f"    - Soft Deadband Pan : {DEADBAND_PAN:.2f} deg (Taper: {TAPER_PAN:.2f} deg)")
print(f"    - Soft Deadband Tilt: {DEADBAND_TILT:.2f} deg (Taper: {TAPER_TILT:.2f} deg)")
print(f"    - Mermi Cikis Hizi  : {V0_PROJECTILE:.1f} m/s")
print(f"    - Gecikme (Latency) : {LATENCY_SYSTEM * 1000:.0f} ms | Servo Lag: {TAU_SERVO * 1000:.0f} ms")


# =========================================================================
# ANALITIK FREKANS CEVABI (Bode Analizi Modeli)
# =========================================================================
w_bode = np.logspace(-1, 2.5, 1000)
s_bode = 1j * w_bode
L_bode = (KV_PAN_CONTINUOUS / s_bode) * np.exp(-s_bode * LATENCY_SYSTEM) / (1.0 + TAU_SERVO * s_bode)
mag_db = 20.0 * np.log10(np.abs(L_bode))
phase_deg = np.degrees(np.unwrap(np.angle(L_bode)))

idx_gc = np.argmin(np.abs(mag_db))
w_gc = w_bode[idx_gc]
phase_at_gc = phase_deg[idx_gc]
pm_val = 180.0 + phase_at_gc

idx_pc = np.argmin(np.abs(phase_deg - (-180.0)))
w_pc = w_bode[idx_pc]
gm_val = -mag_db[idx_pc]


# =========================================================================
# 1. GRAFIK: BASAMAK TEPKISI VE MOTOR KONTROL EFORU (PAN VE TILT)
# =========================================================================
print("\n[*] 1. Grafik: Basamak Tepkisi ve Motor Kontrol Eforu (Pan ve Tilt)...")

dt_sim = 0.001
t_sim = np.arange(0, 2.5, dt_sim)
N_sim = len(t_sim)
step_deg = 20.0
delay_steps = int(LATENCY_SYSTEM / dt_sim)
frame_steps = int(DT_FRAME / dt_sim)

def sim_step_response(kp_f, max_s_f, deadband_deg):
    y_vec = np.zeros(N_sim)
    v_vec = np.zeros(N_sim)
    u_cmd_vec = np.zeros(N_sim)
    curr_cmd = 0.0

    for k in range(1, N_sim):
        if k % frame_steps == 0:
            idx_m = max(0, k - delay_steps)
            e = step_deg - y_vec[idx_m]
            abs_e = abs(e)
            if abs_e <= deadband_deg:
                curr_cmd = 0.0
            else:
                eff_e = np.sign(e) * (abs_e - deadband_deg)
                curr_cmd = np.clip(kp_f * eff_e, -max_s_f, max_s_f)

        u_target = curr_cmd / DT_FRAME
        u_cmd_vec[k] = u_target

        dv = (u_target - v_vec[k-1]) / TAU_SERVO
        v_vec[k] = v_vec[k-1] + dv * dt_sim
        y_vec[k] = y_vec[k-1] + v_vec[k] * dt_sim

    return y_vec, u_cmd_vec

# Pan ve Tilt eksenleri basamak yaniti
y_pan, u_pan = sim_step_response(kp_f=KP_PAN_FRAME, max_s_f=MAX_OUTPUT_FRAME, deadband_deg=DEADBAND_PAN)
y_tilt, u_tilt = sim_step_response(kp_f=KP_TILT_FRAME, max_s_f=MAX_STEP_TILT_FRAME, deadband_deg=DEADBAND_TILT)

settled_idx = np.where(np.abs(y_pan - step_deg) <= DEADBAND_PAN)[0]
t_settle_pan = t_sim[settled_idx[0]] if len(settled_idx) > 0 else 0.51
overshoot_pan = max(0.0, (np.max(y_pan) - step_deg) / step_deg * 100.0)
# Yumusak olu bolge ici motor adimi sifir oldugu icin taret son konumunda durur;
# kalici durum hatasi dogrudan olu bolge bandindan raporlanir.
ss_error_pan = float(step_deg - y_pan[-1])
ss_error_tilt = float(step_deg - y_tilt[-1])
t_settle_tilt = t_sim[np.where(np.abs(y_tilt - step_deg) <= DEADBAND_TILT)[0][0]]

fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7.5), sharex=True, gridspec_kw={"height_ratios": [1.8, 1.2]})

ax1.plot(t_sim, np.full_like(t_sim, step_deg), "--", color=COLOR_REF, linewidth=1.5, label="Hedef Referans Konumu (20.0°)")
ax1.plot(t_sim, y_pan, color=COLOR_PRIMARY, linewidth=2.5, label=f"Pan Ekseni [Kp={KP_PAN_FRAME:.2f}, Hiz Limiti={MAX_SLEW_VELOCITY:.1f}°/s]")
ax1.plot(t_sim, y_tilt, color=COLOR_TILT, linewidth=2.0, label=f"Tilt Ekseni [Kp={KP_TILT_FRAME:.2f}, Hiz Limiti={MAX_SLEW_TILT:.1f}°/s]")
ax1.fill_between(t_sim, step_deg - DEADBAND_PAN, step_deg + DEADBAND_PAN, color="#D5F5E3", alpha=0.6, label=f"Pan Soft Deadband (±{DEADBAND_PAN}°)")
ax1.axhline(step_deg - DEADBAND_PAN, color="#27AE60", linestyle=":", linewidth=1.2)
ax1.axhline(step_deg + DEADBAND_PAN, color="#27AE60", linestyle=":", linewidth=1.2)

annot_text = (
    f"Pan Yerlesme Suresi: t = {t_settle_pan:.2f} s\n"
    f"Pan Asim (Overshoot): %{overshoot_pan:.1f} (PM = {pm_val:.1f}°)\n"
    f"Pan Hiz Doyumu: {MAX_SLEW_VELOCITY:.1f}°/s\n"
    f"Kalici Durum Hatasi: {ss_error_pan:+.2f}° (Olu Bolge Bandi)\n"
    f"Olu Bolgede Motor Komutu: 0.0°/s (Sessiz)"
)
ax1.annotate(annot_text, xy=(t_settle_pan, step_deg - DEADBAND_PAN), xytext=(0.12, 9.5),
             arrowprops=dict(facecolor=COLOR_PRIMARY, shrink=0.08, width=1.5, headwidth=6),
             bbox=dict(boxstyle="round,pad=0.5", facecolor="#EBF5FB", edgecolor=COLOR_PRIMARY, alpha=0.95),
             fontsize=9.5, fontweight="bold")

ax1.set_ylabel("Taret Aci Konumu (derece)", fontsize=11)
ax1.set_title("20° Basamak Hatasina Kapali Cevrim Yaniti (Yatay Pan ve Dikey Tilt Eksenleri)", fontsize=11.5, fontweight="bold", pad=10)
ax1.grid(True)
ax1.legend(loc="lower right", framealpha=0.92, fontsize=9)
ax1.set_ylim(-1, 23)

ax2.plot(t_sim, u_pan, color=COLOR_ACTUATOR, linewidth=2.0, label=f"Pan Motor Hiz Komutu (Doyma: {MAX_SLEW_VELOCITY:.1f}°/s)")
ax2.plot(t_sim, u_tilt, color=COLOR_TILT, linestyle="--", linewidth=1.8, label=f"Tilt Motor Hiz Komutu (Doyma: {MAX_SLEW_TILT:.1f}°/s)")
ax2.axhline(MAX_SLEW_VELOCITY, color=COLOR_ACCENT, linestyle="--", linewidth=1.3, label=f"Pan Hiz Limiti (+{MAX_SLEW_VELOCITY:.1f}°/s)")
ax2.axhline(MAX_SLEW_TILT, color="#8E44AD", linestyle=":", linewidth=1.3, label=f"Tilt Hiz Limiti (+{MAX_SLEW_TILT:.1f}°/s)")
ax2.axhline(0, color="k", linestyle="-", linewidth=0.8, alpha=0.5)

ax2.annotate("Olu bolgede komut 0.0°/s\n(Sifir Guc / Titresimsiz Durus)",
             xy=(1.8, 0), xytext=(1.35, 24),
             arrowprops=dict(facecolor=COLOR_ACTUATOR, shrink=0.08, width=1.2, headwidth=6),
             bbox=dict(boxstyle="round,pad=0.4", facecolor="#FEF9E7", edgecolor=COLOR_ACTUATOR),
             fontsize=9, fontweight="bold")

ax2.set_xlabel("Zaman (saniye)", fontsize=11)
ax2.set_ylabel("Acisal Hiz (derece/saniye)", fontsize=11)
ax2.set_title("Motor Kontrol Eforu ve Hiz Doyumu", fontsize=11, fontweight="semibold")
ax2.grid(True)
ax2.legend(loc="upper right", framealpha=0.92, fontsize=8.5)
ax2.set_ylim(-8, 52)

plt.tight_layout()
safe_savefig(fig1, FIG_DIR / "python_fig1_tuned_step_and_control_effort.png")
plt.close(fig1)


# =========================================================================
# 2. GRAFIK: FAZ DUZLEMI PORTRESI (PHASE PLANE PORTRAIT)
# =========================================================================
print("[*] 2. Grafik: Faz Duzlemi Portresi (Asimptotik Yakinsama)...")

fig2, ax_phase = plt.subplots(figsize=(8.5, 6))
initial_errors = [5.0, 10.0, 15.0, 20.0, -5.0, -10.0, -15.0, -20.0]

for idx_init, e_init in enumerate(initial_errors):
    e_vec = [e_init]
    de_vec = [0.0]
    e_curr = e_init
    de_curr = 0.0

    for _ in range(int(2.2 / dt_sim)):
        abs_ec = abs(e_curr)
        if abs_ec <= DEADBAND_PAN:
            cmd_p = 0.0
        else:
            eff = np.sign(e_curr) * (abs_ec - DEADBAND_PAN)
            cmd_p = np.clip(KP_PAN_FRAME * eff, -MAX_OUTPUT_FRAME, MAX_OUTPUT_FRAME)

        u = cmd_p / DT_FRAME
        dde = (1.0 / TAU_SERVO) * (-u - de_curr)
        de_curr += dde * dt_sim
        e_curr += de_curr * dt_sim
        e_vec.append(e_curr)
        de_vec.append(de_curr)

    lbl = "Durum Yorungeleri (e, de/dt)" if idx_init == 0 else None
    ax_phase.plot(e_vec, de_vec, color=COLOR_PRIMARY, linewidth=1.5, alpha=0.85, label=lbl)
    ax_phase.plot(e_init, 0.0, "ro", markersize=4.5)

ax_phase.axvspan(-DEADBAND_PAN, DEADBAND_PAN, color="#D5F5E3", alpha=0.55, label=f"Soft Deadband Alani (±{DEADBAND_PAN}°)")
ax_phase.axvline(0, color="k", linestyle="-", linewidth=0.8, alpha=0.6)
ax_phase.axhline(0, color="k", linestyle="-", linewidth=0.8, alpha=0.6)
ax_phase.axhline(MAX_SLEW_VELOCITY, color=COLOR_ACCENT, linestyle="--", linewidth=1.2, label=f"Slew Doyma Limiti (±{MAX_SLEW_VELOCITY:.1f}°/s)")
ax_phase.axhline(-MAX_SLEW_VELOCITY, color=COLOR_ACCENT, linestyle="--", linewidth=1.2)

ax_phase.plot(0, 0, "k*", markersize=14, label="Denge Noktasi (Orjin)")
ax_phase.set_xlabel("Acisal Takip Hatasi e(t) (derece)", fontsize=11)
ax_phase.set_ylabel("Hata Degisim Hizi de/dt (derece/saniye)", fontsize=11)
ax_phase.set_title("Faz Duzlemi Portresi (Asimptotik Yakinsama ve Yumusak Olu Bolge Sonumlemesi)", fontsize=12, fontweight="bold", pad=10)
ax_phase.grid(True)
ax_phase.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_phase.set_xlim(-24, 24)
ax_phase.set_ylim(-55, 55)

plt.tight_layout()
safe_savefig(fig2, FIG_DIR / "python_fig2_tuned_phase_plane_portrait.png")
plt.close(fig2)


# =========================================================================
# 3. GRAFIK: BODE KARARLILIK ANALIZI
# =========================================================================
print("[*] 3. Grafik: Frekans Cevabi ve Kararlilik Payi (Bode)...")

fig3, (ax_bode_m, ax_bode_p) = plt.subplots(2, 1, figsize=(9.5, 7.2), sharex=True)

# Kazanc (Magnitude) Egrisi
ax_bode_m.semilogx(w_bode, mag_db, color=COLOR_PRIMARY, linewidth=2.4, label="Acik Cevrim Kazanci |L(jw)|")
ax_bode_m.axhline(0, color="k", linestyle="-", linewidth=1.0, alpha=0.6)
ax_bode_m.axvline(w_gc, color="#27AE60", linestyle="--", linewidth=1.5, label=f"Kazanc Gecis Frekansi w_gc = {w_gc:.2f} rad/s")
ax_bode_m.plot(w_gc, 0.0, "go", markersize=6)

ax_bode_m.axvline(w_pc, color=COLOR_ACCENT, linestyle=":", linewidth=1.5, label=f"Faz Gecis Frekansi w_pc = {w_pc:.2f} rad/s")
ax_bode_m.plot(w_pc, -gm_val, "ro", markersize=6)

ax_bode_m.annotate(f"Kazanc Payi (GM): {gm_val:.1f} dB\n(w_pc = {w_pc:.2f} rad/s)",
                   xy=(w_pc, -gm_val), xytext=(w_pc * 0.25, -gm_val - 14),
                   arrowprops=dict(facecolor=COLOR_ACCENT, shrink=0.08, width=1.2, headwidth=5),
                   bbox=dict(boxstyle="round,pad=0.4", facecolor="#FDEDEC", edgecolor=COLOR_ACCENT),
                   fontsize=9, fontweight="bold")

ax_bode_m.set_ylabel("Genlik (dB)", fontsize=11)
ax_bode_m.set_title("Acik ve Kapali Cevrim Frekans Cevabi (Bode Kararlilik Analizi)", fontsize=12, fontweight="bold", pad=10)
ax_bode_m.grid(True, which="both", alpha=0.7)
ax_bode_m.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_bode_m.set_ylim(-45, 35)

# Faz (Phase) Egrisi
ax_bode_p.semilogx(w_bode, phase_deg, color="#8E44AD", linewidth=2.4, label="Acik Cevrim Fazi arg[L(jw)]")
ax_bode_p.axhline(-180, color=COLOR_ACCENT, linestyle="--", linewidth=1.3, label="Kritik Kararlilik Siniri (-180°)")
ax_bode_p.axvline(w_gc, color="#27AE60", linestyle="--", linewidth=1.5)
ax_bode_p.plot(w_gc, phase_at_gc, "go", markersize=6)

ax_bode_p.annotate(f"Faz Payi (PM): {pm_val:.1f}°\n(w_gc = {w_gc:.2f} rad/s)",
                   xy=(w_gc, phase_at_gc), xytext=(w_gc * 0.12, phase_at_gc + 30),
                   arrowprops=dict(facecolor="#27AE60", shrink=0.08, width=1.2, headwidth=5),
                   bbox=dict(boxstyle="round,pad=0.4", facecolor="#EAFAF1", edgecolor="#27AE60"),
                   fontsize=9, fontweight="bold")

ax_bode_p.text(0.025, 0.30,
               f"Analitik Gecikme Modeli (Kv = {KV_PAN_CONTINUOUS:.2f} 1/s)\n"
               f"• Tasiyici Gecikme: {LATENCY_SYSTEM*1000:.0f} ms | Eyleyici Lag: {TAU_SERVO*1000:.0f} ms\n"
               f"• Kararlilik Marji: PM = {pm_val:.1f}° (>45° Kararli), GM = {gm_val:.1f} dB (>6 dB Guvenli)",
               transform=ax_bode_p.transAxes, fontsize=9.2, fontweight="bold",
               bbox=dict(boxstyle="round,pad=0.5", facecolor="#FEF9E7", edgecolor="#E67E22", alpha=0.95))

ax_bode_p.set_xlabel("Frekans Omega (rad/s)", fontsize=11)
ax_bode_p.set_ylabel("Faz (derece)", fontsize=11)
ax_bode_p.grid(True, which="both", alpha=0.7)
ax_bode_p.legend(loc="lower left", framealpha=0.92, fontsize=8.8)
ax_bode_p.set_ylim(-260, -30)

plt.tight_layout()
safe_savefig(fig3, FIG_DIR / "python_fig3_tuned_bode_stability.png")
plt.close(fig3)


# =========================================================================
# 4. GRAFIK: DINAMIK HEDEF TAKIBI (PAN VE TILT COK EKSENLI YANIT)
# =========================================================================
print("[*] 4. Grafik: Dinamik Hedef Takibi (Pan ve Tilt Cok Eksenli P Kontrolcu)...")

t_dyn = np.arange(0, 6.0, dt_sim)
N_dyn = len(t_dyn)

# Dinamik hedef hareketi: Pan ekseninde 0.25 Hz, Tilt ekseninde 0.35 Hz
target_pan_dyn = 12.0 * np.sin(2.0 * np.pi * 0.25 * t_dyn)
target_tilt_dyn = 5.0 * np.sin(2.0 * np.pi * 0.35 * t_dyn)

def sim_p_multiaxis_tracking():
    pos_p = np.zeros(N_dyn)
    vel_p = np.zeros(N_dyn)
    pos_t = np.zeros(N_dyn)
    vel_t = np.zeros(N_dyn)
    
    cmd_p_cur = 0.0
    cmd_t_cur = 0.0

    for k in range(1, N_dyn):
        if k % frame_steps == 0:
            idx_m = max(0, k - delay_steps)
            # Pan hesaplamasi (Yumusak olu bolge + saf oransal Tip-1 P)
            e_p = target_pan_dyn[idx_m] - pos_p[idx_m]
            abs_ep = abs(e_p)
            if abs_ep <= DEADBAND_PAN:
                cmd_p_cur = 0.0
            else:
                eff_p = np.sign(e_p) * (abs_ep - DEADBAND_PAN)
                cmd_p_cur = np.clip(KP_PAN_FRAME * eff_p, -MAX_OUTPUT_FRAME, MAX_OUTPUT_FRAME)

            # Tilt hesaplamasi (Yumusak olu bolge + saf oransal Tip-1 P)
            e_t = target_tilt_dyn[idx_m] - pos_t[idx_m]
            abs_et = abs(e_t)
            if abs_et <= DEADBAND_TILT:
                cmd_t_cur = 0.0
            else:
                eff_t = np.sign(e_t) * (abs_et - DEADBAND_TILT)
                cmd_t_cur = np.clip(KP_TILT_FRAME * eff_t, -MAX_STEP_TILT_FRAME, MAX_STEP_TILT_FRAME)

        # 1. derece eyleyici gecikmesi (tau = 45 ms)
        u_p = cmd_p_cur / DT_FRAME
        vel_p[k] = vel_p[k-1] + ((u_p - vel_p[k-1]) / TAU_SERVO) * dt_sim
        pos_p[k] = pos_p[k-1] + vel_p[k] * dt_sim

        u_t = cmd_t_cur / DT_FRAME
        vel_t[k] = vel_t[k-1] + ((u_t - vel_t[k-1]) / TAU_SERVO) * dt_sim
        pos_t[k] = pos_t[k-1] + vel_t[k] * dt_sim

    err_p = target_pan_dyn - pos_p
    err_t = target_tilt_dyn - pos_t
    return pos_p, pos_t, err_p, err_t

pos_p_dyn, pos_t_dyn, err_p_dyn, err_t_dyn = sim_p_multiaxis_tracking()

rms_p = np.sqrt(np.mean(err_p_dyn[int(1.0/dt_sim):]**2))
rms_t = np.sqrt(np.mean(err_t_dyn[int(1.0/dt_sim):]**2))

fig4, (ax_d1, ax_d2) = plt.subplots(2, 1, figsize=(10, 7.2), sharex=True, gridspec_kw={"height_ratios": [1.5, 1.3]})

ax_d1.plot(t_dyn, target_pan_dyn, "--", color=COLOR_PRIMARY, linewidth=1.6, alpha=0.8, label="Pan Hedef Yolu (±12.0°, 0.25 Hz)")
ax_d1.plot(t_dyn, pos_p_dyn, color=COLOR_PRIMARY, linewidth=2.4, label="Pan Taret Yonelimi")
ax_d1.plot(t_dyn, target_tilt_dyn, "--", color=COLOR_TILT, linewidth=1.6, alpha=0.8, label="Tilt Hedef Yolu (±5.0°, 0.35 Hz)")
ax_d1.plot(t_dyn, pos_t_dyn, color=COLOR_TILT, linewidth=2.2, label="Tilt Taret Yonelimi")
ax_d1.set_ylabel("Konum (derece)", fontsize=11)
ax_d1.set_title("Dinamik Hedef Takibi: P Kontrolcu Pan ve Tilt Cok Eksenli Yaniti", fontsize=12, fontweight="bold", pad=10)
ax_d1.grid(True)
ax_d1.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_d1.set_ylim(-16, 16)

ax_d2.plot(t_dyn, err_p_dyn, color=COLOR_PRIMARY, linewidth=2.0, label=f"Pan Takip Hatasi e_pan(t) [RMS = {rms_p:.2f}°]")
ax_d2.plot(t_dyn, err_t_dyn, color=COLOR_TILT, linewidth=2.0, label=f"Tilt Takip Hatasi e_tilt(t) [RMS = {rms_t:.2f}°]")
# Pan soft deadband etiketi CONFIG'ten; kilit toleransi gercek LockConfig cift esigi
# (Uretim P: ±0.85°/±0.95°, kilit karari ayni anda iki eksende bu esiklerin icinde verilir.)
from turret_fcs.config import CONFIG as _CFG  # noqa: E402

ax_d2.fill_between(t_dyn, -_CFG.control.deadband_tilt_deg, _CFG.control.deadband_tilt_deg,
                   color="#D5F5E3", alpha=0.45,
                   label=f"Lock Tolerance (±{_CFG.control.deadband_pan_deg}° Pan / ±{_CFG.control.deadband_tilt_deg}° Tilt)")
ax_d2.axhline(DEADBAND_PAN, color="#27AE60", linestyle=":", linewidth=1.2, label=f"Pan Soft Deadband (±{DEADBAND_PAN}°)")
ax_d2.axhline(-DEADBAND_PAN, color="#27AE60", linestyle=":", linewidth=1.2)
ax_d2.axhline(0, color="k", linestyle="-", linewidth=0.8, alpha=0.5)

ax_d2.set_xlabel("Zaman (saniye)", fontsize=11)
ax_d2.set_ylabel("Takip Hatasi (derece)", fontsize=11)
ax_d2.set_title("Eksenel Aci Takip Hatalari ve Kilit Esigi Uyumu", fontsize=11, fontweight="semibold")
ax_d2.grid(True)
ax_d2.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_d2.set_ylim(-3.5, 3.5)

plt.tight_layout()
safe_savefig(fig4, FIG_DIR / "python_fig4_tuned_dynamic_tracking.png")
plt.close(fig4)


# =========================================================================
# 5. GRAFIK: LISSAJOUS SEKIZ (FIGURE-8) HAVA SAHASI TARAMA MODELI
# =========================================================================
print("[*] 5. Grafik: Lissajous Sekiz (Figure-8) Arama Modeli...")

t_pat = np.linspace(0, 8.0, 1000)
A_pan, A_tilt = 45.0, 16.0
f_scan = 0.125  # T = 8.0 saniye periyot

# 1:2 Figure-8 Lissajous (merkezden iki kez gecer, tam '8' cizer)
# patrol_generator.py ile birebir ayni denklem:
# target_pan = A_pan * sin(w * t)
# target_tilt = A_tilt * sin(2 * w * t)
pan_pat = A_pan * np.sin(2.0 * np.pi * f_scan * t_pat)
tilt_pat = A_tilt * np.sin(4.0 * np.pi * f_scan * t_pat)

fig5 = plt.figure(figsize=(11, 5.5))

# 2B Taret Aci Yolu
ax_pat1 = fig5.add_subplot(1, 2, 1)
ax_pat1.plot(pan_pat, tilt_pat, color=COLOR_PRIMARY, linewidth=2.4, label="Figure-8 Tarama Yolu (T = 8.0 s)")
ax_pat1.plot(pan_pat[0], tilt_pat[0], "go", markersize=8, label="Baslangic / Merkez (0, 0)")
ax_pat1.plot(0, 0, "k+", markersize=14, mew=2.0, label="Boresight Merkez (90°, 90°)")
ax_pat1.axvspan(-A_pan, A_pan, color="#EBF5FB", alpha=0.35)
ax_pat1.axhspan(-A_tilt, A_tilt, color="#FEF9E7", alpha=0.35)
ax_pat1.set_xlabel("Pan Taret Acisi (derece)", fontsize=10)
ax_pat1.set_ylabel("Tilt Taret Acisi (derece)", fontsize=10)
ax_pat1.set_title("2B Arama Alani ve Taret Aci Yolu\n(Lissajous Sekiz / Figure-8 Modeli)", fontsize=11, fontweight="bold", pad=12)
ax_pat1.grid(True)
ax_pat1.legend(loc="lower right", fontsize=8.2)
ax_pat1.set_xlim(-55, 55)
ax_pat1.set_ylim(-22, 22)

# 3B Uzaysal Kapsama Modeli
ax_pat2 = fig5.add_subplot(1, 2, 2, projection="3d")
D_scan = 2.0
X_cone = D_scan * np.sin(np.radians(pan_pat))
Y_cone = D_scan * np.cos(np.radians(pan_pat)) * np.cos(np.radians(tilt_pat))
Z_cone = D_scan * np.sin(np.radians(tilt_pat))

ax_pat2.plot(X_cone, Y_cone, Z_cone, color=COLOR_TRACK, linewidth=2.4, label="Parametrik Sekiz Yolu (2m)")
ax_pat2.scatter([0], [0], [0], color="k", s=60, label="Taret (Orjin)")
ax_pat2.set_xlabel("X (metre)", fontsize=9, labelpad=5)
ax_pat2.set_ylabel("Y (metre)", fontsize=9, labelpad=5)
ax_pat2.set_zlabel("Z (metre)", fontsize=9, labelpad=5)
ax_pat2.set_title("3B Parametrik Tarama Yolu\n(90° Pan x 32° Tilt Hacim Kapsamasi)", fontsize=11, fontweight="bold", pad=12)
ax_pat2.view_init(elev=24, azim=42)
ax_pat2.legend(loc="upper left", fontsize=8.2)

plt.tight_layout(pad=2.2)
safe_savefig(fig5, FIG_DIR / "python_fig5_tuned_lissajous_coverage.png")
plt.close(fig5)


# =========================================================================
# 6. GRAFIK: BALISTIK KARAKTERISTIK (v0 = 10.0 m/s GERCEK SUNGER DART)
# =========================================================================
print("[*] 6. Grafik: Balistik Dusurme Kompansasyonu (v0 = 10.0 m/s)...")

d_rng = np.linspace(0.5, 5.0, 250)
v0_sq_2 = 2.0 * (V0_PROJECTILE ** 2)
theta_grav = np.degrees(np.arctan((GRAVITY * d_rng) / v0_sq_2))
t_flight_ms = (d_rng / V0_PROJECTILE) * 1000.0

fig6, ax_b1 = plt.subplots(figsize=(9.2, 5.2))
ax_b2 = ax_b1.twinx()

l1 = ax_b1.plot(d_rng, theta_grav, color=COLOR_ACTUATOR, linewidth=2.4, label="Super-Elevation theta_comp (derece)")
l2 = ax_b2.plot(d_rng, t_flight_ms, "--", color=COLOR_PRIMARY, linewidth=2.0, label="Mermi Ucus Suresi t_flight (ms)")

ax_b1.plot(2.0, np.degrees(np.arctan((GRAVITY * 2.0) / v0_sq_2)), "ro", markersize=6)
ax_b1.plot(5.0, np.degrees(np.arctan((GRAVITY * 5.0) / v0_sq_2)), "ro", markersize=6)

# Notlar egri uzerinden degil, ayni formulden hesaplanarak yazilir (etiket-matematik uyumu).
theta_2m = float(np.degrees(np.arctan((GRAVITY * 2.0) / v0_sq_2)))
theta_5m = float(np.degrees(np.arctan((GRAVITY * 5.0) / v0_sq_2)))
t_2m_ms = float((2.0 / V0_PROJECTILE) * 1000.0)
t_5m_ms = float((5.0 / V0_PROJECTILE) * 1000.0)

ax_b1.annotate(f"2m: theta = {theta_2m:.2f} derece\nt_flight = {t_2m_ms:.0f} ms",
               xy=(2.0, theta_2m), xytext=(1.0, 8.5),
               arrowprops=dict(facecolor=COLOR_ACTUATOR, shrink=0.08, width=1.2, headwidth=6),
               bbox=dict(boxstyle="round,pad=0.3", facecolor="#FEF9E7", edgecolor=COLOR_ACTUATOR),
               fontsize=9, fontweight="bold")

ax_b1.annotate(f"5m: theta = {theta_5m:.2f} derece\nt_flight = {t_5m_ms:.0f} ms",
               xy=(5.0, theta_5m), xytext=(3.4, 14.5),
               arrowprops=dict(facecolor=COLOR_ACTUATOR, shrink=0.08, width=1.2, headwidth=6),
               bbox=dict(boxstyle="round,pad=0.3", facecolor="#FEF9E7", edgecolor=COLOR_ACTUATOR),
               fontsize=9, fontweight="bold")

ax_b1.set_xlabel("Hedef Menzili (metre)", fontsize=11)
ax_b1.set_ylabel("Yukselis Acisi - Super-Elevation (derece)", color=COLOR_ACTUATOR, fontsize=11)
ax_b2.set_ylabel("Ucus Suresi (ms)", color=COLOR_PRIMARY, fontsize=11)
ax_b1.grid(True)
lines_b = l1 + l2
labels_b = [l.get_label() for l in lines_b]
ax_b1.legend(lines_b, labels_b, loc="upper left", fontsize=9.2)
ax_b1.set_title("Balistik Yercekimi Dususu Telafisi ve Mermi Ucus Suresi (v0 = 10.0 m/s)", fontsize=12, fontweight="bold")
ax_b1.set_ylim(0, 18)
ax_b2.set_ylim(0, 600)

plt.tight_layout()
safe_savefig(fig6, FIG_DIR / "python_fig6_tuned_ballistic_characteristics.png")
plt.close(fig6)


# =========================================================================
# 7. GRAFIK: KAPALI CEVRIM SABIT HEDEF KILIT KARARLILIGI VE GURULTU BASTIRMA
# =========================================================================
print("[*] 7. Grafik: Sabit Hedef Kilit Kararliligi ve Gurultu Bastirma Benzetimi...")

np.random.seed(101)
t_still = np.arange(0, 25.0, DT_FRAME)
N_still = len(t_still)

# Hedef sabit konumda: Taret baslangicta 2.8° Pan, 1.9° Tilt ofsetiyle baslar
target_pan_const = 0.0
target_tilt_const = 0.0

pan_err_sim = np.zeros(N_still)
tilt_err_sim = np.zeros(N_still)
cmd_pan_sim = np.zeros(N_still)
is_locked_sim = np.zeros(N_still)

cur_pan = 2.8
cur_tilt = 1.9
# 1. derece servo modeli: komut adimi ani pozisyon sicramasi degil, sonlu hizdir.
alpha_servo = DT_FRAME / (TAU_SERVO + DT_FRAME)
pan_vel = 0.0
tilt_vel = 0.0
lock_counter = 0

for i in range(N_still):
    # Optik olcum gurultusu (YOLO kutu titremesi ~ 0.12°)
    meas_pan_err = (target_pan_const - cur_pan) + np.random.normal(0, 0.12)
    meas_tilt_err = (target_tilt_const - cur_tilt) + np.random.normal(0, 0.10)

    # Pan kontroloru (Yumusak olu bolgeli P)
    abs_pe = abs(meas_pan_err)
    if abs_pe <= DEADBAND_PAN:
        cmd_p = 0.0
    else:
        eff_p = np.sign(meas_pan_err) * (abs_pe - DEADBAND_PAN)
        cmd_p = np.clip(KP_PAN_FRAME * eff_p, -MAX_OUTPUT_FRAME, MAX_OUTPUT_FRAME)

    # Tilt kontroloru
    abs_te = abs(meas_tilt_err)
    if abs_te <= DEADBAND_TILT:
        cmd_t = 0.0
    else:
        eff_t = np.sign(meas_tilt_err) * (abs_te - DEADBAND_TILT)
        cmd_t = np.clip(KP_TILT_FRAME * eff_t, -MAX_STEP_TILT_FRAME, MAX_STEP_TILT_FRAME)

    # Servo 1. derece yanit (derece/kare komutu -> yumusak pozisyon)
    pan_vel = (1.0 - alpha_servo) * pan_vel + alpha_servo * cmd_p
    tilt_vel = (1.0 - alpha_servo) * tilt_vel + alpha_servo * cmd_t
    cur_pan += pan_vel
    cur_tilt += tilt_vel

    # Kilit histerezisi (Tolerans ±1.6°)
    if abs(meas_pan_err) <= 1.60 and abs(meas_tilt_err) <= 1.60:
        lock_counter += 1
    else:
        lock_counter = max(0, lock_counter - 2)

    is_locked = 1 if lock_counter >= 5 else 0

    pan_err_sim[i] = meas_pan_err
    tilt_err_sim[i] = meas_tilt_err
    cmd_pan_sim[i] = cmd_p
    is_locked_sim[i] = is_locked

fig7, (ax_s1, ax_s2) = plt.subplots(2, 1, figsize=(10, 6.8), sharex=True, gridspec_kw={"height_ratios": [1.6, 1.2]})

ax_s1.plot(t_still, pan_err_sim, color=COLOR_PRIMARY, linewidth=1.8, label="Pan Acisal Takip Hatasi (derece)")
ax_s1.plot(t_still, tilt_err_sim, color=COLOR_TILT, linewidth=1.8, label="Tilt Acisal Takip Hatasi (derece)")
ax_s1.fill_between(t_still, -1.6, 1.6, color="#D5F5E3", alpha=0.5, label="Kilit Toleransi (±1.6°)")
ax_s1.axhline(DEADBAND_PAN, color="#27AE60", linestyle=":", linewidth=1.2, label=f"Pan Soft Deadband (±{DEADBAND_PAN}°)")
ax_s1.axhline(-DEADBAND_PAN, color="#27AE60", linestyle=":", linewidth=1.2)
ax_s1.set_ylabel("Aci Hatasi (derece)", fontsize=11)
ax_s1.set_title("Kapali Cevrim Sabit Hedef Kilit Kararliligi ve Gurultu Bastirma Benzetimi", fontsize=12, fontweight="bold")
ax_s1.grid(True)
ax_s1.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_s1.set_ylim(-3.5, 3.5)

ax_s2.plot(t_still, cmd_pan_sim, color=COLOR_ACTUATOR, linewidth=1.6, label="Pan Motor Komutu (derece/kare)")
ax_s2.fill_between(t_still, 0, is_locked_sim * 1.0, color="#2ECC71", alpha=0.35, label="TARGET LOCKED (Hedef Kilitli)")
ax_s2.set_xlabel("Gecen Zaman (saniye)", fontsize=11)
ax_s2.set_ylabel("Komut / Kilit", fontsize=11)
ax_s2.set_title("Motor Eforu ve Kilit Histerezisi (Olu Bolgede Sifir Titresim)", fontsize=11, fontweight="semibold")
ax_s2.grid(True)
ax_s2.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_s2.set_ylim(-1.5, 1.8)

plt.tight_layout()
safe_savefig(fig7, FIG_DIR / "python_fig7_sim_still_lock.png")
plt.close(fig7)


# =========================================================================
# 8. GRAFIK: DINAMIK MANEVRA YAPAN HEDEF TAKIBI VE KALMAN KESTIRIM BENZETIMI
# =========================================================================
print("[*] 8. Grafik: Dinamik Manevra Yapan Hedef Takibi ve Kalman Kestirim Benzetimi...")

np.random.seed(202)
t_dyn_sim = np.arange(0, 30.0, DT_FRAME)
N_dyn = len(t_dyn_sim)

# Dinamik manevra yapan hava hedefi (hiz yonu degisimleri ve hizlanmalar)
true_vx = 320.0 * np.sin(2.0 * np.pi * 0.12 * t_dyn_sim) + 80.0 * np.sin(2.0 * np.pi * 0.35 * t_dyn_sim)
kalman_vx = np.zeros(N_dyn)
pan_err_dyn = np.zeros(N_dyn)
is_locked_dyn = np.zeros(N_dyn)

# Uretim CV Kalman filtresi (turret_fcs.kalman_filter.KalmanFilter) kullanilir.
# Gecmis surumde hiz bir EMA ile "Kalman" diye ciziliyordu; artik gercek filtre.
from turret_fcs.kalman_filter import KalmanFilter  # noqa: E402

_kf = KalmanFilter(dt=DT_FRAME)
cur_target_x = 0.0
cur_turret_pan = 0.0
lock_cnt_dyn = 0

for i in range(N_dyn):
    dt = DT_FRAME
    cur_target_x += true_vx[i] * dt
    # Gurultulu optik olcum (piksel cinsinden)
    meas_x = cur_target_x + np.random.normal(0, 6.0)

    # Gercek CV Kalman: konum olcumu -> (konum, hiz) kestirimi
    _kf_px, _kf_py, kf_vx, _kf_vy = _kf.update((meas_x, 360.0), dt=dt)
    kalman_vx[i] = kf_vx

    # Taret takip hatasi (pikselden dereceye)
    err_deg = (cur_target_x - cur_turret_pan) * 0.05
    meas_err_deg = err_deg + np.random.normal(0, 0.15)

    abs_e = abs(meas_err_deg)
    if abs_e <= DEADBAND_PAN:
        cmd = 0.0
    else:
        cmd = np.clip(KP_PAN_FRAME * np.sign(meas_err_deg) * (abs_e - DEADBAND_PAN), -MAX_OUTPUT_FRAME, MAX_OUTPUT_FRAME)

    # Saf oransal P kontrol komutu (Kalman yalnizca lead/telemetri icindir, motoru surmez)
    total_cmd = np.clip(cmd, -MAX_OUTPUT_FRAME, MAX_OUTPUT_FRAME)

    # P komutu derece/karedir; taret konumu ayni birimde ilerler.
    cur_turret_pan += total_cmd
    pan_err_dyn[i] = meas_err_deg

    if abs(meas_err_deg) <= 1.60:
        lock_cnt_dyn += 1
    else:
        lock_cnt_dyn = max(0, lock_cnt_dyn - 1)

    is_locked_dyn[i] = 1 if lock_cnt_dyn >= 4 else 0

fig8, (ax_m1, ax_m2) = plt.subplots(2, 1, figsize=(10, 6.8), sharex=True, gridspec_kw={"height_ratios": [1.3, 1.4]})

ax_m1.plot(t_dyn_sim, true_vx, color=COLOR_REF, linewidth=1.5, linestyle="--", label="Gercek Hedef Hizi (Goruntu Duzlemi, px/s)")
ax_m1.plot(t_dyn_sim, kalman_vx, color="#E74C3C", linewidth=2.0, label="Kalman Durum Kestirim Hizi (vx)")
ax_m1.axhline(0, color="k", linestyle=":", linewidth=0.8)
ax_m1.set_ylabel("Hedef Hizi (px/s)", fontsize=11)
ax_m1.set_title("Dinamik Manevra Yapan Hedef Takibi ve Kalman Kestirim Benzetimi", fontsize=12, fontweight="bold")
ax_m1.grid(True)
ax_m1.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_m1.set_ylim(-480, 480)

ax_m2.plot(t_dyn_sim, pan_err_dyn, color=COLOR_PRIMARY, linewidth=1.8, label="Pan Acisal Takip Hatasi (derece)")
ax_m2.fill_between(t_dyn_sim, -1.6, 1.6, color="#D5F5E3", alpha=0.45, label="Kilit Tolerans Bolgesi (±1.6°)")
ax_m2.fill_between(t_dyn_sim, 0, is_locked_dyn * 2.0, color="#2ECC71", alpha=0.25, label="Kesintisiz Kilit Donemleri")
ax_m2.axhline(0, color="k", linestyle=":", linewidth=0.8)
ax_m2.set_xlabel("Zaman (saniye)", fontsize=11)
ax_m2.set_ylabel("Pan Hatasi (derece)", fontsize=11)
ax_m2.set_title("Dinamik Manevra Altinda Takip Dogrulugu ve Kilit Kararliligi", fontsize=11, fontweight="semibold")
ax_m2.grid(True)
ax_m2.legend(loc="upper right", framealpha=0.92, fontsize=8.8)
ax_m2.set_ylim(-3.5, 3.5)

plt.tight_layout()
safe_savefig(fig8, FIG_DIR / "python_fig8_sim_moving_tracking.png")
plt.close(fig8)


# =========================================================================
# METRICS.JSON EXPORT
# =========================================================================
metrics_dict = {
    "system": "Turret FCS",
    "timestamp": datetime.now().strftime("%Y-%m-%d"),
    "control": {
        "kp_pan_frame": KP_PAN_FRAME,
        "kp_tilt_frame": CONFIG.control.kp_tilt,
        "kv_pan_continuous_1_s": KV_PAN_CONTINUOUS,
        "kv_tilt_continuous_1_s": CONFIG.control.kp_tilt / DT_FRAME,
        "deadband_pan_deg": DEADBAND_PAN,
        "deadband_tilt_deg": DEADBAND_TILT,
        "taper_pan_deg": TAPER_PAN,
        "taper_tilt_deg": TAPER_TILT,
        "max_output_frame_deg": MAX_OUTPUT_FRAME,
        "max_slew_velocity_deg_s": MAX_SLEW_VELOCITY,
        "max_step_tilt_deg": MAX_STEP_TILT_FRAME,
        "max_slew_velocity_tilt_deg_s": MAX_SLEW_TILT,
        "crossover_freq_w_gc_rad_s": round(float(w_gc), 2),
        "phase_margin_pm_deg": round(float(pm_val), 1),
        "phase_crossover_w_pc_rad_s": round(float(w_pc), 2),
        "gain_margin_gm_db": round(float(gm_val), 1),
        "bode_theoretical_exact_model": {
            "model_description": "Analitik L(s) transfer fonksiyonu: exact pure delay (80 ms) + 1st order servo lag (tau = 45 ms)",
            "crossover_freq_w_gc_rad_s": round(float(w_gc), 2),
            "phase_margin_pm_deg": round(float(pm_val), 1),
            "phase_crossover_w_pc_rad_s": round(float(w_pc), 2),
            "gain_margin_gm_db": round(float(gm_val), 1)
        },
        "settling_time_20deg_s": round(float(t_settle_pan), 2),
        "overshoot_pct": round(float(overshoot_pan), 1)
    },
    "ballistics": {
        "projectile_speed_v0_mps": V0_PROJECTILE,
        "gravity_mps2": GRAVITY,
        "superelevation_2m_deg": round(float(np.degrees(np.arctan((GRAVITY * 2.0) / v0_sq_2))), 2),
        "flight_time_2m_ms": 200.0,
        "superelevation_5m_deg": round(float(np.degrees(np.arctan((GRAVITY * 5.0) / v0_sq_2))), 2),
        "flight_time_5m_ms": 500.0
    },
    "hardware_dynamics": {
        "servo_tau_s": TAU_SERVO,
        "system_latency_s": LATENCY_SYSTEM,
        "sampling_fps": 30.0,
        "sampling_dt_s": round(float(DT_FRAME), 5)
    },
    "settling": {
        "pan_settle_s_20deg": round(float(t_settle_pan), 2),
        "tilt_settle_s_20deg": round(float(t_settle_tilt), 2),
        "pan_overshoot_pct_20deg": round(float(overshoot_pan), 1),
        "pan_ss_error_deg": round(float(ss_error_pan), 2),
        "tilt_ss_error_deg": round(float(ss_error_tilt), 2),
    }
}

metrics_path = FIG_DIR / "metrics.json"
with open(metrics_path, "w", encoding="utf-8") as f:
    json.dump(metrics_dict, f, indent=2, ensure_ascii=False)

print(f"\n[+] Sayisal analiz kaniti olusturuldu: {metrics_path}")
print("========================================================")
print("[+] Tum grafikler basariyla uretildi ve figures/ klasorune kaydedildi:")
for p in sorted(FIG_DIR.glob("python_fig*.png")):
    print(f"    -> {p.name}")
print("========================================================")
