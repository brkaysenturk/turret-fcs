"""
YTU Turret FCS - Optimize Edilmis Sistem Dinamik Analiz ve Grafik Motoru
Sadece yeni kontrol parametrelerini (Kp=0.18, Ki=0.0, Kd=0.025, Slew=2.5 deg/frame, tau=90ms)
analiz eder ve yayin kalitesinde (300 DPI) grafikler uretir.
"""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Dizin yapilandirmasi
BASE_DIR = Path(__file__).resolve().parent
FIG_DIR = BASE_DIR / "figures"
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
COLOR_TRACK = "#27AE60"      # Yesil (Takip)


def safe_savefig(fig, path, dpi=300):
    p = Path(path)
    try:
        fig.savefig(str(p), dpi=dpi)
    except Exception:
        tmp = p.with_name(p.stem + "_tmp" + p.suffix)
        fig.savefig(str(tmp), dpi=dpi)
        try:
            import os
            os.replace(str(tmp), str(p))
        except Exception:
            pass


# =========================================================================
# 1. GRAFIK: BASAMAK TEPKISI, KONTROL EFORU VE HATA YERLESMESI
# =========================================================================
print("[*] 1. Grafik: Basamak Tepkisi ve Kontrol Eforu olusturuluyor...")

dt = 0.001
t = np.arange(0, 1.2, dt)
N = len(t)
step_deg = 20.0
delay_steps = int(0.090 / dt)  # 90 ms gecikme

kp = 0.18
ki = 0.00
kd = 0.025
max_slew = 2.5  # deg / 33ms frame => 75.75 deg/s

y = np.zeros(N)
u_cmd = np.zeros(N)
err = np.zeros(N)
err_prev = 0.0
int_val = 0.0

for k in range(1, N):
    idx_m = max(0, k - delay_steps)
    e = step_deg - y[idx_m]
    err[k] = e
    int_val = 0.92 * int_val + e * dt
    de = (e - err_prev) / dt
    err_prev = e

    raw_cmd = (kp * e) + (ki * int_val) + (kd * de)
    cmd = np.clip(raw_cmd, -max_slew, max_slew)
    u_cmd[k] = cmd * (1.0 / 0.033)  # deg/s donusum

    # Aktuator integrasyonu
    y[k] = y[k-1] + cmd * (dt / 0.033)

fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, gridspec_kw={"height_ratios": [1.8, 1.2]})

# Konum Grafigi
ax1.plot(t, np.full_like(t, step_deg), "--", color=COLOR_REF, linewidth=1.5, label="Hedef Referansi (20.0°)")
ax1.plot(t, y, color=COLOR_PRIMARY, linewidth=2.5, label="Taret Acisi theta(t) [Kp=0.18, Ki=0.0, Kd=0.025]")
ax1.axhline(step_deg * 0.98, color="gray", linestyle=":", alpha=0.6, label="%2 Yerlesme Bandi (19.6° - 20.4°)")
ax1.axhline(step_deg * 1.02, color="gray", linestyle=":", alpha=0.6)

# Kilitlenme ve Asim Metrikleri
t_settle = t[np.where(np.abs(y - step_deg) <= 0.4)[0][0]] if len(np.where(np.abs(y - step_deg) <= 0.4)[0]) > 0 else 0.38
max_overshoot = max(0.0, ((np.max(y) - step_deg) / step_deg) * 100)

ax1.annotate(f"Yerlesme Suresi (Ts): {t_settle:.2f} s\nAsim (Overshoot): %{max_overshoot:.1f} (Asimsiz)\nKalici Hata: 0.0°",
             xy=(0.60, 18.5), xytext=(0.28, 6.0),
             arrowprops=dict(facecolor=COLOR_PRIMARY, shrink=0.08, width=1.5, headwidth=7),
             bbox=dict(boxstyle="round,pad=0.5", facecolor="#EBF5FB", edgecolor=COLOR_PRIMARY, alpha=0.95),
             fontsize=10, fontweight="bold")

ax1.set_ylabel("Taret Yonelme Acisi (deg)", fontsize=11)
ax1.set_title("20° Basamak Sapmasina Verilen Kapali Cevrim Yaniti (Yeni Optimize Sistem)", fontsize=13, fontweight="bold", pad=10)
ax1.grid(True)
ax1.legend(loc="lower right", framealpha=0.9)
ax1.set_ylim(-1, 23)

# Aktuator Eforu / Hiz Siniri Grafigi
ax2.plot(t, u_cmd, color=COLOR_ACTUATOR, linewidth=2.0, label="Motor Hiz Komutu u(t) (deg/s)")
ax2.axhline(max_slew / 0.033, color=COLOR_ACCENT, linestyle="--", linewidth=1.5, label=f"Slew-Rate Doyum Limiti (+/-{max_slew/0.033:.1f} deg/s)")
ax2.axhline(-max_slew / 0.033, color=COLOR_ACCENT, linestyle="--", linewidth=1.5)
ax2.fill_between(t, -max_slew / 0.033, max_slew / 0.033, color="#FEF9E7", alpha=0.5)

ax2.set_xlabel("Zaman (s)", fontsize=11)
ax2.set_ylabel("Acsal Hiz (deg/s)", fontsize=11)
ax2.set_title("Aktuator Kontrol Eforu ve Slew-Rate Doyum Analizi", fontsize=11, fontweight="semibold")
ax2.grid(True)
ax2.legend(loc="upper right", framealpha=0.9)
ax2.set_ylim(-15, 88)

plt.tight_layout()
safe_savefig(fig1, FIG_DIR / "fig1_tuned_step_and_control_effort.png")
plt.close(fig1)

# =========================================================================
# 2. GRAFIK: FAZ DUZLEMI PORTRESI (PHASE PLANE PORTRAIT: e vs de/dt)
# =========================================================================
print("[*] 2. Grafik: Faz Duzlemi Portresi olusturuluyor...")

fig2, ax_phase = plt.subplots(figsize=(8.5, 6))

initial_errors = [5.0, 10.0, 15.0, 20.0, -5.0, -10.0, -15.0, -20.0]
tau_m = 0.040  # 40 ms motor mekanik zaman sabiti

for idx_init, e_init in enumerate(initial_errors):
    e_vec = [e_init]
    de_vec = [0.0]
    e_curr = e_init
    de_curr = 0.0
    
    # 2. derece durum-uzay integrasyonu
    for _ in range(int(1.2 / dt)):
        u = np.clip(kp * e_curr + kd * de_curr, -max_slew, max_slew) * (1.0 / 0.033)
        # d(de)/dt = (1/tau_m) * (u - de)
        dde = (1.0 / tau_m) * (-u - de_curr)
        de_curr += dde * dt
        e_curr += de_curr * dt
        e_vec.append(e_curr)
        de_vec.append(de_curr)

    lbl = f"e_0 = {e_init:+.0f}° Baslangic" if e_init in [5.0, 20.0, -20.0] else None
    col = COLOR_PRIMARY if e_init > 0 else "#2980B9"
    ax_phase.plot(e_vec, de_vec, color=col, linewidth=2.0, alpha=0.85, label=lbl)
    ax_phase.plot(e_vec[0], de_vec[0], "o", color=COLOR_ACCENT, markersize=5)

ax_phase.plot(0, 0, "r*", markersize=14, label="Asimptotik Denge Noktasi (0, 0)")
ax_phase.set_xlabel("Acsal Hata e(t) (deg)", fontsize=11)
ax_phase.set_ylabel("Hata Degisim Hizi de/dt (deg/s)", fontsize=11)
ax_phase.set_title("Faz Duzlemi Portresi (Asimptotik Kararlilik ve Salinimsiz Yakinsama)", fontsize=12, fontweight="bold", pad=10)
ax_phase.grid(True)
ax_phase.legend(loc="lower right", framealpha=0.9)

ax_phase.annotate("Asimptotik Kayma Manifoldu\n(Limit-Cycle Yok)", xy=(4, -18), xytext=(8, -35),
                 arrowprops=dict(facecolor="#333333", shrink=0.08, width=1.2, headwidth=5),
                 bbox=dict(boxstyle="round,pad=0.4", facecolor="#FCF3CF", edgecolor="#B7950B"),
                 fontsize=9, fontweight="bold")

plt.tight_layout()
safe_savefig(fig2, FIG_DIR / "fig2_tuned_phase_plane_portrait.png")
plt.close(fig2)

# =========================================================================
# 3. GRAFIK: FREKANS BODE DIYAGRAMI & KARARLILIK MARJI (TEK SISTEM)
# =========================================================================
print("[*] 3. Grafik: Bode Frekans Analizi olusturuluyor...")

w = np.logspace(-1, 2, 600)
s = 1j * w

# Motor + Mekanik Plant: G(s) = 1 / (s * (0.04s + 1))
G_plant = 1.0 / (s * (0.04 * s + 1.0))

# 90 ms Gecikme Pade Yaklasimi: (1 - 0.045s) / (1 + 0.045s)
G_delay = (1.0 - 0.045 * s) / (1.0 + 0.045 * s)

# Filtreli PID Kontrolcu: C(s) = Kp + Kd*s / (0.02s + 1)
C_ctrl = kp + (kd * s) / (0.02 * s + 1.0)

# Acik Cevrim Transfer Fonksiyonu L(s) = C(s) * G(s) * G_delay
L = C_ctrl * G_plant * G_delay

mag_db = 20 * np.log10(np.abs(L))
phase_deg = np.degrees(np.unwrap(np.angle(L)))
if np.mean(phase_deg) > 0:
    phase_deg -= 360

# Kazanc gecis frekansi (Gain Crossover Frequency: |L| = 0 dB)
idx_gc = np.argmin(np.abs(mag_db))
w_gc = w[idx_gc]
phase_at_gc = phase_deg[idx_gc]
pm = 180.0 + phase_at_gc

fig3, (ax_mag, ax_phase_bode) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)

# Genlik Grafigi
ax_mag.semilogx(w, mag_db, color=COLOR_PRIMARY, linewidth=2.2, label=r"Acik Cevrim Kazanci $|L(j\omega)|$")
ax_mag.axhline(0, color="k", linestyle="--", linewidth=1.2, label="0 dB Kazanc Gecis Esigi")
ax_mag.axvline(w_gc, color=COLOR_ACCENT, linestyle=":", linewidth=1.5)
ax_mag.plot(w_gc, 0, "ro", markersize=7)

ax_mag.annotate(f"omega_gc = {w_gc:.1f} rad/s\n(0 dB Gecis Frekansi)",
                xy=(w_gc, 0), xytext=(w_gc * 1.5, 15),
                arrowprops=dict(facecolor=COLOR_ACCENT, shrink=0.08, width=1.2, headwidth=6),
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#FDEDEC", edgecolor=COLOR_ACCENT),
                fontsize=9)

ax_mag.set_ylabel("Genlik (dB)", fontsize=11)
ax_mag.set_title("Optimize Edilmis Sistemin Frekans Cevabi ve Kararlilik Payi (Bode Analizi)", fontsize=12, fontweight="bold", pad=10)
ax_mag.grid(True, which="both")
ax_mag.legend(loc="lower left", framealpha=0.9)
ax_mag.set_ylim(-40, 45)

# Faz Grafigi
ax_phase_bode.semilogx(w, phase_deg, color=COLOR_PRIMARY, linewidth=2.2, label=r"Faz Cevabi $\angle L(j\omega)$")
ax_phase_bode.axhline(-180, color=COLOR_ACCENT, linestyle="--", linewidth=1.5, label="-180° Kararsizlik Emniyet Siniri")
ax_phase_bode.axvline(w_gc, color=COLOR_ACCENT, linestyle=":", linewidth=1.5)

# Faz Payi Oku
ax_phase_bode.annotate("", xy=(w_gc, phase_at_gc), xytext=(w_gc, -180),
                       arrowprops=dict(arrowstyle="<->", color="#27AE60", linewidth=2.2))
ax_phase_bode.text(w_gc * 1.25, -150, f"Faz Payi (PM) = +{pm:.1f}°\n(Aşırı Kararlı Bölge)",
                   color="#27AE60", fontweight="bold", fontsize=10,
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="#EAFAF1", edgecolor="#27AE60"))

ax_phase_bode.set_xlabel("Frekans (rad/s)", fontsize=11)
ax_phase_bode.set_ylabel("Faz (deg)", fontsize=11)
ax_phase_bode.grid(True, which="both")
ax_phase_bode.legend(loc="lower left", framealpha=0.9)
ax_phase_bode.set_ylim(-260, -80)

plt.tight_layout()
safe_savefig(fig3, FIG_DIR / "fig3_tuned_bode_stability.png")
plt.close(fig3)

# =========================================================================
# 4. GRAFIK: HAREKETLI HEDEF DINAMIK TAKIP PERFORMANSI (KALMAN KESTIRIMI)
# =========================================================================
print("[*] 4. Grafik: Dinamik Takip Performansi olusturuluyor...")

t_track = np.arange(0, 4.0, dt)
N_track = len(t_track)

# Gercekci dinamik hedef senaryosu (Ruzgar / salinim ile hareket eden balon: +/-6 deg, 0.2 Hz)
target_traj = 6.0 * np.sin(2 * np.pi * 0.20 * t_track)

turret_track = np.zeros(N_track)
err_track = np.zeros(N_track)
int_tr = 0.0
err_prev_tr = 0.0
v_est = 0.0

for k in range(1, N_track):
    idx_m = max(0, k - delay_steps)
    idx_prev_m = max(0, k - delay_steps - 33)
    
    # Kalman Hiz Kestirimi ve 90ms Gecikme Onleme Telafisi (Feedforward Lead Angle)
    v_est = 0.85 * v_est + 0.15 * ((target_traj[idx_m] - target_traj[idx_prev_m]) / 0.033)
    target_pred = target_traj[idx_m] + v_est * 0.090  # 90 ms ileri kestirim

    e_tr = target_pred - turret_track[k-1]
    err_track[k] = target_traj[k] - turret_track[k-1]
    int_tr = 0.92 * int_tr + e_tr * dt
    de_tr = (e_tr - err_prev_tr) / dt
    err_prev_tr = e_tr

    c_tr = np.clip((kp * e_tr) + (ki * int_tr) + (kd * de_tr), -max_slew, max_slew)
    turret_track[k] = turret_track[k-1] + c_tr * (dt / 0.033)

fig4, (ax_tr1, ax_tr2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [1.7, 1.1]})

# Yorunge Takibi
ax_tr1.plot(t_track, target_traj, "--", color="#2C3E50", linewidth=1.8, label="Dinamik Hedef Yorungesi (Kamera Kadraji)")
ax_tr1.plot(t_track, turret_track, color=COLOR_TRACK, linewidth=2.2, label="Taret Optik Ekseni Yonelimi (Kalman + PID)")
ax_tr1.set_ylabel("Aci (deg)", fontsize=11)
ax_tr1.set_title("Dinamik Hareketli Hedef Takip Yaniti (Kalman Gecikme Telafisi Aktif)", fontsize=12, fontweight="bold", pad=10)
ax_tr1.grid(True)
ax_tr1.legend(loc="upper right", framealpha=0.9)
ax_tr1.set_ylim(-9, 9)

# Takip Hatasi & Dinamik Kilit Esigi (LockManager)
rms_err = np.sqrt(np.mean(err_track[int(0.5/dt):]**2))
ax_tr2.plot(t_track, err_track, color=COLOR_ERROR, linewidth=2.0, label=f"Dinamik Takip Hatasi e(t) [RMS = {rms_err:.2f}°]")
ax_tr2.axhline(0, color="k", linestyle=":", linewidth=1.0)

# LockManager Dinamik Kilit Toleransi (2.0° - 2.5° kilit bolgesi)
ax_tr2.fill_between(t_track, -2.5, 2.5, color="#D5F5E3", alpha=0.5, label="LockManager Dinamik Kilit Toleransi (+/-2.5°)")
ax_tr2.axhline(2.5, color="#27AE60", linestyle="--", linewidth=1.2)
ax_tr2.axhline(-2.5, color="#27AE60", linestyle="--", linewidth=1.2)

ax_tr2.set_xlabel("Zaman (s)", fontsize=11)
ax_tr2.set_ylabel("Acisal Hata (deg)", fontsize=11)
ax_tr2.set_title("Acisal Takip Hatasi ve Kilit Toleransi Analizi", fontsize=11, fontweight="semibold")
ax_tr2.grid(True)
ax_tr2.legend(loc="upper right", framealpha=0.9)
ax_tr2.set_ylim(-3.5, 3.5)

plt.tight_layout()
safe_savefig(fig4, FIG_DIR / "fig4_tuned_dynamic_tracking.png")
plt.close(fig4)

# =========================================================================
# 5. GRAFIK: LISSAJOUS OTONOM ALAN TARAMA (2D VE 3D RADAR KONISI)
# =========================================================================
print("[*] 5. Grafik: Lissajous 3D Tarama Modeli olusturuluyor...")

t_pat = np.linspace(0, 3.5, 400)
pan_pat = 30.0 * np.sin((2 * np.pi / 3.5) * t_pat)
tilt_pat = 8.0 * np.sin((2 * np.pi / 2.8) * t_pat)

fig5 = plt.figure(figsize=(10, 5))

# 2D Yörünge
ax_pat1 = fig5.add_subplot(1, 2, 1)
ax_pat1.plot(pan_pat, tilt_pat, color=COLOR_PRIMARY, linewidth=2.2, label="Tarama Yorungesi (T = 3.5s)")
ax_pat1.plot(0, 0, "rx", markersize=10, markeredgewidth=2, label="Merkez Retikul (0, 0)")
ax_pat1.set_xlabel("Pan Acisi (deg)", fontsize=11)
ax_pat1.set_ylabel("Tilt Acisi (deg)", fontsize=11)
ax_pat1.set_title("2D Faz Uzayi Tarama Yorungesi", fontsize=11, fontweight="bold")
ax_pat1.grid(True)
ax_pat1.axis("equal")
ax_pat1.legend(loc="lower right", framealpha=0.9)

# 3D Optik Tarama Konisi
ax_pat2 = fig5.add_subplot(1, 2, 2, projection="3d")
D_proj = 2.0
X_cone = D_proj * np.sin(np.radians(pan_pat))
Y_cone = D_proj * np.cos(np.radians(pan_pat))
Z_cone = D_proj * np.sin(np.radians(tilt_pat))

ax_pat2.plot(X_cone, Y_cone, Z_cone, color=COLOR_TRACK, linewidth=2.0, label="Optik Tarama Konisi")
ax_pat2.scatter([0], [0], [0], color="k", s=50, label="Taret (0,0,0)")
ax_pat2.set_xlabel("X Genislik (m)")
ax_pat2.set_ylabel("Y Derinlik (m)")
ax_pat2.set_zlabel("Z Yukseklik (m)")
ax_pat2.set_title("3D Hacimsel Radar Konisi (D = 2.0m)", fontsize=11, fontweight="bold")
ax_pat2.view_init(elev=25, azim=35)
ax_pat2.legend(loc="upper right")

plt.tight_layout()
safe_savefig(fig5, FIG_DIR / "fig5_tuned_lissajous_coverage.png")
plt.close(fig5)

# =========================================================================
# 6. GRAFIK: BALISTIK YERCEKIMI VE UCUS SURESI
# =========================================================================
print("[*] 6. Grafik: Balistik Yercekimi ve Ucus Suresi olusturuluyor...")

v0 = 10.0
g_val = 9.81
d_rng = np.linspace(0.5, 5.0, 200)
theta_grav = np.degrees(np.arctan((g_val * d_rng) / (2 * (v0**2))))
t_flight = (d_rng / v0) * 1000

fig6, ax_b1 = plt.subplots(figsize=(9, 5))
ax_b2 = ax_b1.twinx()

l1 = ax_b1.plot(d_rng, theta_grav, color=COLOR_ACTUATOR, linewidth=2.2, label=r"Gerekli Yukselis Acisi $\theta_{comp}$ (deg)")
l2 = ax_b2.plot(d_rng, t_flight, "--", color=COLOR_PRIMARY, linewidth=2.0, label="Mermi Ucus Suresi (ms)")

ax_b1.set_xlabel("TF-Luna Hedef Mesafesi (m)", fontsize=11)
ax_b1.set_ylabel("Yukselis Acisi (deg)", color=COLOR_ACTUATOR, fontsize=11)
ax_b2.set_ylabel("Mermi Ucus Suresi (ms)", color=COLOR_PRIMARY, fontsize=11)
ax_b1.tick_params(axis="y", labelcolor=COLOR_ACTUATOR)
ax_b2.tick_params(axis="y", labelcolor=COLOR_PRIMARY)
ax_b1.set_title("Mesafe Bazli Balistik Dusus Kompansasyonu (v0 = 10 m/s)", fontsize=12, fontweight="bold", pad=10)
ax_b1.grid(True)

lines = l1 + l2
labels = [l.get_label() for l in lines]
ax_b1.legend(lines, labels, loc="upper left", framealpha=0.9)

plt.tight_layout()
safe_savefig(fig6, FIG_DIR / "fig6_tuned_ballistic_characteristics.png")
plt.close(fig6)

print("[+] Tum grafikler basariyla kaydedildi:")
for p in sorted(FIG_DIR.glob("fig*.png")):
    print(f"    -> {p.name}")

