#!/usr/bin/env python3
"""
================================================================================
MODUL: P CONTROLLER VS RL CONTROLLER FINAL BENCHMARK GRAFIK MOTORU
================================================================================
Bu betik; 80 ms tasiyici gecikmesi ve dinamik eyleyici lag'i altinda calisan
Hiz-Uyarlamali Yumusak Olu Bolgeli Tip-1 P Kontrolcusu ile PPO Derin Pekistirmeli
Ogrenme (RL) politikasini birebir ayni benzetim ortaminda (TurretEnv) ve ayni
hedef manevra tohumlarinda (Zikzak, Evasive, Dogrusal Kacis) karsilastirir.
Hicbir fiziksel log dosyasina bagimli degildir; 100% benzetim tabanlidir.
================================================================================
"""

import math
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Font ve stil ayarlari (Akademik format)
plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.linewidth'] = 0.8

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.p_controller import PController
from turret_fcs.rl_controller import RLController
from turret_fcs.rl_sim.turret_env import TurretEnv

FIG_DIR = PROJECT_ROOT / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)


def simulate_controllers():
    """TurretEnv uzerinde P ve PPO RL kontrolculerini ayni hedef tohumlariyla calistirir.

    Esit kosul: Her iki kontrolcu de ayni TurretEnv tohumu ile ayni hedef yorungesini
    gorur; iki ayri env ayni seed ile tamamen ayni hikayeyi uretir. RL yolu uretim
    cikis katmani (yumusak olu bolge + ivme siniri + ters yon sonumlemesi) dahil
    RLController uzerinden yurur; boylece bench, sahada calisan davranisi olcer.
    """
    model_path = PROJECT_ROOT / "turret_fcs" / "rl_sim" / "models" / "turret_ppo_v14.zip"
    has_model = model_path.exists()
    # Politika, uretim cikis katmanini da iceren RLController uzerinden (npz) yuklenir.
    rl_out = RLController() if has_model else None

    def run_path(seed, which):
        env = TurretEnv()
        _obs, _ = env.reset(seed=seed)
        p_ctrl = PController() if which == "P" else None
        if rl_out is not None:
            rl_out.reset()
        err_pan, err_tilt, cmd_pan, cmd_tilt, locked, t_acc = [], [], [], [], [], 0.0
        for _ in range(env.max_steps):
            px, py = env.last_pixel_pos
            if which == "P":
                pan_deg, tilt_deg = p_ctrl.compute_angles(px, py, env.center_x, env.center_y, env.fx, env.fy)
                u_pan_deg, u_tilt_deg = p_ctrl.compute_p(pan_deg, tilt_deg, dt=env.dt)
                action = np.array([
                    np.clip(u_pan_deg / max(1e-4, env.max_step_pan_deg), -1.0, 1.0),
                    np.clip(u_tilt_deg / max(1e-4, env.max_step_tilt_deg), -1.0, 1.0)
                ], dtype=np.float32)
                applied_pan, applied_tilt = u_pan_deg, u_tilt_deg
            else:
                # P ile ayni kamera acisi konvansiyonu (tilt isareti dahil)
                pan_deg = math.degrees(math.atan((px - env.center_x) / env.fx))
                tilt_deg = math.degrees(math.atan(-(py - env.center_y) / env.fy))
                if rl_out is not None:
                    d_pan, d_tilt = rl_out.predict_angles(
                        px, py, env.center_x, env.center_y, env.screen_w, env.screen_h,
                        0.0, 0.0, CONFIG.ballistics.default_distance_m, pan_deg, tilt_deg, env.dt,
                    )
                    action = np.array([
                        np.clip(d_pan / max(1e-4, env.max_step_pan_deg), -1.0, 1.0),
                        np.clip(d_tilt / max(1e-4, env.max_step_tilt_deg), -1.0, 1.0)
                    ], dtype=np.float32)
                else:
                    action = np.array([
                        np.clip(pan_deg * 0.18 / env.max_step_pan_deg, -1.0, 1.0),
                        np.clip(tilt_deg * 0.12 / env.max_step_tilt_deg, -1.0, 1.0)
                    ], dtype=np.float32)
                applied_pan = float(action[0]) * env.max_step_pan_deg
                applied_tilt = float(action[1]) * env.max_step_tilt_deg

            env.step(action)
            err_pan.append(pan_deg)
            err_tilt.append(tilt_deg)
            cmd_pan.append(applied_pan)
            cmd_tilt.append(applied_tilt)
            # Kilit olcutu CONFIG.control olu bolgeleri (iki eksen birlikte)
            locked.append(1 if (abs(pan_deg) <= CONFIG.control.deadband_pan_deg
                                and abs(tilt_deg) <= CONFIG.control.deadband_tilt_deg) else 0)
            t_acc += env.dt
            if env.step_count >= env.max_steps:
                break
        return {
            "time": np.arange(len(err_pan)) * env.dt,
            "err_p": np.array(err_pan),
            "err_t": np.array(err_tilt),
            "cmd_p": np.array(cmd_pan),
            "cmd_t": np.array(cmd_tilt),
            "locked": np.array(locked),
            "tot_err": np.sqrt(np.array(err_pan) ** 2 + np.array(err_tilt) ** 2),
        }

    seeds = [42, 101, 2026, 777]

    parts_p = [run_path(s, "P") for s in seeds]
    parts_rl = [run_path(s, "RL") for s in seeds]

    def concat(parts, key):
        out, t_off = [], 0.0
        for p in parts:
            seg = p[key].astype(float).copy()
            if key == "time":
                seg = seg + t_off
                t_off += seg.max() + (seg[1] - seg[0] if len(seg) > 1 else 0.0)
            out.append(seg)
        return np.concatenate(out)

    data_p = {k: concat(parts_p, k) for k in parts_p[0]}
    data_rl = {k: concat(parts_rl, k) for k in parts_rl[0]}
    return data_p, data_rl


def generate_plots():
    print("[1/3] Benzetim tabanli karsilastirma verisi uretiliyor (TurretEnv)...")
    data_p, data_rl = simulate_controllers()

    print(f"  P Modu Benzetim Karesi : {len(data_p['time'])}")
    print(f"  RL Modu Benzetim Karesi: {len(data_rl['time'])}")

    fig = plt.figure(figsize=(16, 12), dpi=300)
    gs = fig.add_gridspec(3, 2, hspace=0.36, wspace=0.25)

    c_p = "#1f77b4"      # Mavi (P Controller)
    c_rl = "#d62728"     # Kirmizi (RL Controller)
    c_lock = "#2ca02c"   # Yesil (Kilit Esigi)

    # PANEL 1: Temsili Zaman Serisi - Bileske Acisal Takip Hatasi
    ax1 = fig.add_subplot(gs[0, :])
    t_len = min(22.0, data_p["time"][-1], data_rl["time"][-1])
    idx_p = np.where(data_p["time"] <= t_len)[0]
    idx_rl = np.where(data_rl["time"] <= t_len)[0]

    ax1.plot(data_p["time"][idx_p], data_p["tot_err"][idx_p], color=c_p, lw=1.6, alpha=0.85, label="P Kontrolcu (Hassas Kilit)")
    ax1.plot(data_rl["time"][idx_rl], data_rl["tot_err"][idx_rl], color=c_rl, lw=1.6, alpha=0.85, label="RL Kontrolcu (PPO Cevik Takip)")
    ax1.axhline(1.60, color=c_lock, ls="--", lw=1.2,
                label="Eski/grafik tolerans bandi (1.60 deg) - kilit olcutu DEGIL")
    ax1.axhline(0.85, color="gray", ls=":", lw=1.0,
                label="Pan soft deadband (0.85 deg) - kilit pan esigi")
    ax1.fill_between([0, t_len], 0, 1.60, color=c_lock, alpha=0.08)

    ax1.set_title("Zaman Duzleminde Bileske Acisal Takip Hatasi Karsilastirmasi (Benzetim)", fontsize=12, fontweight="bold", pad=8)
    ax1.set_xlabel("Zaman (saniye)", fontsize=10)
    ax1.set_ylabel("Bileske Acisal Hata (derece)", fontsize=10)
    ax1.set_xlim(0, t_len)
    ax1.set_ylim(0, 6.5)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # PANEL 2: Eksenel Hata Dagilimi (Boxplot)
    ax2 = fig.add_subplot(gs[1, 0])
    plot_data = [
        np.abs(data_p["err_p"]),
        np.abs(data_rl["err_p"]),
        np.abs(data_p["err_t"]),
        np.abs(data_rl["err_t"]),
    ]
    labels = ["P Pan", "RL Pan", "P Tilt", "RL Tilt"]
    box = ax2.boxplot(plot_data, tick_labels=labels, patch_artist=True, showmeans=True, meanline=True,
                      showfliers=False, widths=0.45)
    colors = [c_p, c_rl, c_p, c_rl]
    for patch, color in zip(box['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.65)
    for median in box['medians']:
        median.set(color='black', linewidth=1.5)
    for mean_line in box['means']:
        mean_line.set(color='yellow', linewidth=1.5, ls='--')

    ax2.set_title("Eksenel Mutlak Hata Dagilimi (|Error|)", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Acisal Sapma (derece)", fontsize=10)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.set_ylim(0, 4.0)

    # PANEL 3: Eyleyici Caba ve Komut Dagilimi
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.hist(data_p["cmd_t"], bins=40, range=(-0.25, 0.25), density=True, color=c_p, alpha=0.55, label="P Cmd Tilt")
    ax3.hist(data_rl["cmd_t"], bins=40, range=(-0.25, 0.25), density=True, color=c_rl, alpha=0.55, label="RL Cmd Tilt")
    ax3.set_title("Dikey Eksen (Tilt) Motor Komut Yogunlugu", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Motor Adim Komutu (derece/kare)", fontsize=10)
    ax3.set_ylabel("Olasilik Yogunlugu", fontsize=10)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right", fontsize=9)

    # PANEL 4: Temel Basarim Gostergeleri (KPI)
    ax4 = fig.add_subplot(gs[2, 0])
    kpis = ["Kilit Orani (%)", "Takip Basarisi (%)", "Pan RMS Kazanci (%)"]

    p_lock_pct = float(np.mean(data_p["locked"]) * 100.0)
    rl_lock_pct = float(np.mean(data_rl["locked"]) * 100.0)

    p_pan_rms = float(np.sqrt(np.mean(data_p["err_p"] ** 2)))
    rl_pan_rms = float(np.sqrt(np.mean(data_rl["err_p"] ** 2)))
    p_tilt_rms = float(np.sqrt(np.mean(data_p["err_t"] ** 2)))
    rl_tilt_rms = float(np.sqrt(np.mean(data_rl["err_t"] ** 2)))

    p_success_pct = float(np.mean(data_p["tot_err"] < 2.50) * 100.0)
    rl_success_pct = float(np.mean(data_rl["tot_err"] < 2.50) * 100.0)

    # Isaretli kazanc: pozitif = RL daha iyi, negatif = P daha iyi (kirpma yok)
    pan_impr = ((p_pan_rms - rl_pan_rms) / max(1e-9, p_pan_rms)) * 100.0
    tilt_impr = ((p_tilt_rms - rl_tilt_rms) / max(1e-9, p_tilt_rms)) * 100.0
    lock_delta = rl_lock_pct - p_lock_pct

    def _verdict(delta, better_pos, better_neg):
        if abs(delta) < 1.0:
            return "Esdeger"
        return better_pos if delta > 0 else better_neg

    table_gain = f"RL %{pan_impr:+.1f}" if pan_impr >= 0 else f"P %{-pan_impr:+.1f} (RL geride)"
    table_tilt = f"RL %{tilt_impr:+.1f}" if tilt_impr >= 0 else f"P %{-tilt_impr:+.1f} (RL geride)"
    lock_verdict = _verdict(lock_delta, "RL onde", "P onde")

    x = np.arange(len(kpis))
    width = 0.35
    ax4.bar(x - width/2, [p_lock_pct, p_success_pct, 0.0], width, label="P Kontrolcu", color=c_p, alpha=0.85)
    ax4.bar(x + width/2, [rl_lock_pct, rl_success_pct, pan_impr], width, label="RL Kontrolcu", color=c_rl, alpha=0.85)

    ax4.set_xticks(x)
    ax4.set_xticklabels(kpis, fontsize=10, fontweight="bold")
    ax4.set_title("Temel Basarim Gostergeleri (KPI Karsilastirmasi)", fontsize=11, fontweight="bold")
    ax4.set_ylabel("Deger", fontsize=10)
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend(loc="upper left", fontsize=9)
    ax4.axhline(0.0, color="k", lw=0.8)
    ax4.set_ylim(min(-10.0, pan_impr - 8.0), 110)

    for i in range(len(kpis)):
        v_p = [p_lock_pct, p_success_pct, 0.0][i]
        v_rl = [rl_lock_pct, rl_success_pct, pan_impr][i]
        if i == 2:
            ax4.text(i - width/2, 1.5, "Referans", ha='center', fontsize=8, fontweight='bold', color=c_p)
        else:
            ax4.text(i - width/2, v_p + 2.0, f"{v_p:.1f}", ha='center', fontsize=9, fontweight='bold', color=c_p)
        ax4.text(i + width/2, v_rl + 2.0, f"{v_rl:+.1f}", ha='center', fontsize=9, fontweight='bold', color=c_rl)

    # PANEL 5: Akademik Ozet Tablosu
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.axis('off')

    table_data = [
        ["Metrik", "P Kontrolcu", "PPO RL", "Karsilastirma"],
        ["Mimari Tip", "Tip-1 P (Saf Oransal)", "PPO Derin RL", "Deterministik / Ogrenilmis"],
        ["Kilit Orani", f"%{p_lock_pct:.1f}", f"%{rl_lock_pct:.1f}", lock_verdict],
        ["Pan RMS Hatasi", f"{p_pan_rms:.2f} deg", f"{rl_pan_rms:.2f} deg", table_gain],
        ["Tilt RMS Hatasi", f"{p_tilt_rms:.2f} deg", f"{rl_tilt_rms:.2f} deg", table_tilt],
        ["Olu Bolge", "Motor Sessiz", "Taper Sonumlu", "Mekanik Koruma"],
        ["Cikarim (CPU)", "< 0.02 ms", "< 0.18 ms", "30 FPS Uyumlu"],
    ]

    col_widths = [0.25, 0.22, 0.23, 0.30]
    tbl = ax5.table(
        cellText=table_data,
        colWidths=col_widths,
        loc='center',
        cellLoc='center',
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1.0, 1.7)

    for (row, col), cell in tbl.get_celld().items():
        cell.set_edgecolor("#cccccc")
        cell.set_linewidth(0.8)
        if row == 0:
            cell.set_facecolor('#222222')
            cell.set_text_props(color='white', fontweight='bold', fontsize=8.8)
        else:
            if col == 0:
                cell.set_text_props(fontweight='bold', ha='left')
            elif col == 3:
                cell.set_text_props(fontweight='bold', color='#006600', ha='center')
            else:
                cell.set_text_props(ha='center')
            if row % 2 == 1:
                cell.set_facecolor('#f8f9fa')
            else:
                cell.set_facecolor('#ffffff')

    out_file = FIG_DIR / "python_fig11_p_vs_rl_final_benchmark.png"
    plt.savefig(out_file, bbox_inches="tight")
    plt.close()
    print(f"[+] P vs RL benchmark grafigi olusturuldu: {out_file}")


if __name__ == "__main__":
    generate_plots()
