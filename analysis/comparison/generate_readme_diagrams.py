#!/usr/bin/env python3
"""
Turret FCS README diyagramlarini (mimari, akis ve donanim) guncel CONFIG
degerleriyle yeniden uretir.

Cikti:
  docs/assets/turret_fcs_architecture.svg
  docs/assets/turret_fcs_flowchart.svg
  docs/assets/turret_fcs_hardware.svg

Diyagramlar harici bir cizim araci gerektirmeyen saf SVG'dir; GitHub ve
tarayicilarda dogrudan goruntulenir.
"""

from __future__ import annotations

import sys
from html import escape
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG  # noqa: E402

ASSETS = ROOT / "docs" / "assets"
FONT = "Segoe UI, Arial, sans-serif"

INK = "#0f172a"
MUTED = "#475569"
WHITE = "#ffffff"


class SVG:
    def __init__(self, width: int, height: int, background: str = "#f8fafc"):
        self.width = width
        self.height = height
        self.background = background
        self.parts: list[str] = []
        self.markers: dict[str, str] = {}

    def add(self, value: str) -> None:
        self.parts.append(value)

    def rect(self, x, y, w, h, fill, stroke="none", rx=10, sw=1.4, dash=None, opacity=1.0):
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" ry="{rx}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{dash_attr} opacity="{opacity}"/>'
        )

    def text(self, x, y, value, size=12, color=INK, weight="normal", anchor="start", opacity=1.0):
        self.add(
            f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" '
            f'fill="{color}" font-weight="{weight}" text-anchor="{anchor}" opacity="{opacity}">'
            f"{escape(str(value))}</text>"
        )

    def lines(self, x, y, values, size=12, color=INK, leading=17, weight="normal", anchor="start"):
        tspans = []
        for index, value in enumerate(values):
            dy = 0 if index == 0 else leading
            tspans.append(f'<tspan x="{x}" dy="{dy}">{escape(str(value))}</tspan>')
        self.add(
            f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" fill="{color}" '
            f'font-weight="{weight}" text-anchor="{anchor}">{"".join(tspans)}</text>'
        )

    def marker(self, color: str) -> str:
        key = color.replace("#", "")
        if key not in self.markers:
            self.markers[key] = color
        return f"arrow-{key}"

    def arrow(self, points, color, width=2.0, label=None, label_at=None, dash=None, curve=False):
        marker = self.marker(color)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        if curve and len(points) == 4:
            (x1, y1), (cx, cy), (x2, y2) = points
            path = f"M {x1} {y1} Q {cx} {cy} {x2} {y2}"
        else:
            path = "M " + " L ".join(f"{x} {y}" for x, y in points)
        self.add(
            f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{width}" '
            f'stroke-linecap="round" stroke-linejoin="round"{dash_attr} '
            f'marker-end="url(#{marker})"/>'
        )
        if label and label_at:
            self.badge(label_at[0], label_at[1], label, color)

    def badge(self, x, y, label, color):
        width = max(44, len(label) * 6.3 + 14)
        self.rect(x - width / 2, y - 10, width, 20, "#ffffff", color, rx=5, sw=1.0)
        self.text(x, y + 3.6, label, size=9.5, color=color, weight="bold", anchor="middle")

    def card(self, x, y, w, h, title, subtitle, bullets, accent, fill="#ffffff", tag=None):
        self.rect(x, y, w, h, fill, accent, rx=12, sw=1.6)
        self.rect(x, y, 6, h, accent, "none", rx=3, sw=0)
        self.text(x + 18, y + 27, title, size=14.5, color=INK, weight="bold")
        if subtitle:
            self.text(x + 18, y + 47, subtitle, size=10.5, color=accent, weight="bold")
        start = y + 68 if subtitle else y + 52
        for index, bullet in enumerate(bullets):
            self.text(x + 18, start + index * 17, f"- {bullet}", size=11.2, color=MUTED)
        if tag:
            self.text(x + w - 14, y + h - 10, tag, size=9, color="#94a3b8", anchor="end")

    def band(self, x, y, w, h, title, color, fill):
        self.rect(x, y, w, h, fill, color, rx=16, sw=1.3, dash="7 5")
        self.text(x + 18, y + 27, title, size=13.5, color=color, weight="bold")

    def diamond(self, cx, cy, w, h, lines, fill, stroke):
        points = f"{cx},{cy - h / 2} {cx + w / 2},{cy} {cx},{cy + h / 2} {cx - w / 2},{cy}"
        self.add(f'<polygon points="{points}" fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>')
        start_y = cy - (len(lines) - 1) * 9
        self.lines(cx, start_y, lines, size=11.5, color=INK, leading=18, weight="bold", anchor="middle")

    def save(self, path: Path):
        marker_defs = []
        for key, color in self.markers.items():
            marker_defs.append(
                f'<marker id="arrow-{key}" viewBox="0 0 10 10" refX="9" refY="5" '
                f'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{color}"/></marker>'
            )
        body = "\n".join(self.parts)
        output = (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" '
            f'viewBox="0 0 {self.width} {self.height}" role="img">\n'
            "<defs>\n" + "\n".join(marker_defs) + "\n</defs>\n"
            f'<rect width="{self.width}" height="{self.height}" fill="{self.background}"/>\n'
            f"{body}\n</svg>\n"
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(output, encoding="utf-8")
        print(f"[OK] {path.relative_to(ROOT)}")


def architecture() -> None:
    C = CONFIG
    svg = SVG(1600, 1120)
    svg.text(40, 34, "Turret FCS - Guncel Sistem Mimarisi", size=21, color=INK, weight="bold")
    svg.text(
        40, 57,
        "Kamera + LiDAR -> algilama -> kestirim/balistik -> P veya PPO kontrol -> STM32 -> servo/firlatma",
        size=12, color=MUTED,
    )

    # Katman 1
    svg.band(28, 78, 1544, 198, "1. ALGILAMA VE SENSOR FUSYONU", "#7c3aed", "#f5f3ff")
    svg.card(
        48, 112, 462, 146,
        "Logitech C270 Kamera",
        f"1280x720 @ 30 FPS | FOV {C.optics.h_fov_deg:.1f} x {C.optics.v_fov_deg:.1f} derece",
        [
            "camera_thread.py: ayri thread, en guncel kare",
            "Bayat kare korumasi ve kare kimligi",
            "Piksel -> aci donusumu (fx, fy)",
        ],
        "#7c3aed", tag="camera_thread.py",
    )
    svg.card(
        532, 112, 486, 146,
        "YOLOv8n + ByteTrack",
        "Hava hedefi tespiti ve kimlik surekliligi",
        [
            "Arama esigi conf >= 0.28",
            "Kilitli takipte conf >= 0.16",
            "Sinir kutusu + track ID + uzamsal kapi",
        ],
        "#7c3aed", tag="target_selector.py",
    )
    svg.card(
        1040, 112, 514, 146,
        "TF-Luna ToF LiDAR",
        f"850 nm | {C.lidar.min_distance_m:.1f} - {C.lidar.max_distance_m:.1f} m | 100 Hz UART",
        [
            "STM32 USART1 uzerinden dogrudan okuma",
            "Menzil kapilama: sicrama, geometri, kararlilik",
            "Ucus suresi ve balistik dusum girdisi",
        ],
        "#2563eb", tag="fcs_manager.py",
    )

    # Katman 2
    svg.band(28, 296, 1544, 254, "2. FCS CEKIRDEGI - KARAR, KESTIRIM VE BALISTIK", "#0f766e", "#ecfdf5")
    svg.card(
        48, 330, 310, 104,
        "Hedef Secici",
        "TargetSelector",
        [
            "Alan / en-boy / guven filtreleri",
            "ByteTrack ID ve 5 kare kayip toleransi",
        ],
        "#0f766e", tag="target_selector.py",
    )
    svg.card(
        48, 448, 310, 84,
        "Devriye Ureteci",
        "PatrolGenerator",
        [
            "1.5 s bekleme, merkeze donus",
            "8 s Lissajous figure-8 tarama",
        ],
        "#0f766e", tag="patrol_generator.py",
    )
    svg.card(
        382, 330, 388, 202,
        "CV Kalman + Ego Telafisi",
        "[x, y, vx, vy] | Joseph formu",
        [
            f"Tek Q/R: sigma_a {C.kalman.sigma_a:.0f} px/s^2, r_var {C.kalman.r_var:.0f} px^2",
            f"1. derece aktüator gozlemcisi (tau {C.control.ego_tau_s * 1000:.0f} ms)",
            f"{C.kalman.outlier_jump_px:.0f} px tekil-sicrama kapisi",
            f"{C.kalman.velocity_deadband_low_px_s:.0f}-{C.kalman.velocity_deadband_high_px_s:.0f} px/s cikis hiz olu bolgesi",
        ],
        "#0f766e", tag="kalman_filter.py",
    )
    svg.card(
        794, 330, 354, 202,
        "Balistik ve Lead",
        "Ucus suresi + yercekimi telafisi",
        [
            f"t_flight = mesafe / {C.ballistics.projectile_speed_mps:.1f} m/s + {C.control.lead_time_offset_s:.3f} s",
            f"Lead tavani {C.control.lead_time_max_s:.2f} s",
            "theta = arctan(g D / (2 v^2))",
            "Slew ileri beslemesi (delta_slew)",
        ],
        "#0f766e", tag="fcs_manager.py",
    )
    svg.card(
        1172, 330, 382, 202,
        "Kilit Yoneticisi",
        "LockManager",
        [
            f"{C.lock.lock_threshold_frames} kare / 0.33 s kararlilik",
            "Dinamik esik 1.60 - 3.00 derece",
            f"+{C.lock.lock_exit_margin_deg:.2f} derece cikis histerezisi",
            "2B Gauss vurus olasiligi (ncx2)",
        ],
        "#b45309", tag="lock_manager.py",
    )

    # Katman 3
    svg.band(28, 568, 1544, 230, "3. KONTROL KATMANI - IKI BAGIMSIZ MOD", "#b45309", "#fffbeb")
    svg.card(
        48, 604, 470, 166,
        "P Kontrolcu - Hassas Kilit",
        "Tip-1, tek EMA, yumusak olu bolge",
        [
            f"Kp_pan {C.control.kp_pan:.2f} | Kp_tilt {C.control.kp_tilt:.2f}",
            f"Olu bolge {C.control.deadband_pan_deg:.2f} / {C.control.deadband_tilt_deg:.2f} derece",
            f"Ivme siniri {C.control.cmd_accel_pan_deg_s2:.0f} / {C.control.cmd_accel_tilt_deg_s2:.0f} derece/s^2",
            f"Adim siniri {C.actuator.max_step_pan_deg:.2f} / {C.actuator.max_step_tilt_deg:.2f} derece",
        ],
        "#b45309", tag="p_controller.py",
    )
    svg.card(
        542, 604, 322, 166,
        "Operator Mod Secimi",
        "[M] tusu ile ayrik gecis",
        [
            "P: hassas, dusuk titresim",
            "RL: cevik manevra takibi",
            "Geciste kontrolcu durumu sifirlanir",
        ],
        "#64748b", tag="keymap.py",
    )
    svg.card(
        890, 604, 664, 166,
        "RL Kontrolcu - Cevik Takip",
        "Saf PPO | 11-64-64-2 NumPy MLP",
        [
            "11 boyut: hata, hata turevi, hiz, mesafe, 2 adim eylem",
            "P harmani yok; politika dogrudan motor adimi uretir",
            "Acisal taper, yon degistirme sonumu, ayni ivme siniri",
        ],
        "#7c3aed", tag="rl_controller.py",
    )

    # Katman 4
    svg.band(28, 818, 1544, 260, "4. GOMULU DENETLEYICI, EYLEME VE OPERATOR ARAYUZU", "#0e7490", "#ecfeff")
    svg.card(
        48, 854, 448, 198,
        "STM32F401RE",
        "ARM Cortex-M4 @ 84 MHz | 115200 Baud",
        [
            "7 bayt TX / 10 bayt RX ikili protokol",
            "100 Hz servo ve LiDAR telemetrisi",
            "300 ms heartbeat + 800 ms IWDG",
            "Zaman tabanli 45 derece/s servo siniri",
        ],
        "#0e7490", tag="stm32/turret_main",
    )
    svg.card(
        516, 854, 476, 198,
        "Eyleyiciler ve Firlatma",
        "2 eksen gimbal + cift volan + solenoid",
        [
            f"Pan {C.mount.pan_limit_min:.0f} / +{C.mount.pan_limit_max:.0f} derece | Tilt {C.mount.tilt_limit_min:.0f} / +{C.mount.tilt_limit_max:.0f} derece",
            "MG996R / DS3218, 50 Hz PWM",
            "Cift FF-180 volan, 23.500 RPM",
            f"{C.ballistics.firing_pulse_s * 1000:.0f} ms solenoid darbe + 150 ms reload",
        ],
        "#dc2626", tag="servo_driver.c / flywheel_solenoid.c",
    )
    svg.card(
        1012, 854, 542, 198,
        "HUD, Telemetri ve Kayit",
        "MIL-STD taktik arayuz",
        [
            "Nisangah, kilit bari, LEAD ibresi, vurus olasiligi",
            "P / RL mod banneri ve olay gunlugu",
            "35 kolon CSV + meta.json + events.csv",
            "Oturum ozeti ve KPI analizi",
        ],
        "#b45309", tag="turret_hud.py / telemetry.py",
    )

    # Oklar
    svg.arrow([(510, 185), (532, 185)], "#7c3aed", label="kare", label_at=(521, 170))
    svg.arrow([(775, 258), (775, 282), (203, 282), (203, 330)], "#7c3aed", label="bbox + ID", label_at=(470, 276))
    svg.arrow([(1297, 258), (1297, 282), (576, 282), (576, 330)], "#2563eb", label="mesafe", label_at=(920, 276))
    svg.arrow([(358, 382), (382, 382)], "#0f766e")
    svg.arrow([(770, 430), (794, 430)], "#0f766e", label="durum", label_at=(782, 414))

    svg.arrow([(1148, 430), (1172, 430)], "#0f766e", label="hata", label_at=(1160, 414))
    svg.arrow([(970, 532), (970, 556), (283, 556), (283, 604)], "#b45309", label="aci + lead", label_at=(620, 550))
    svg.arrow([(1060, 532), (1060, 556), (1112, 556), (1112, 604)], "#7c3aed", label="11D gozlem", label_at=(1140, 574))
    svg.arrow([(542, 687), (518, 687)], "#b45309", label="P", label_at=(530, 673))
    svg.arrow([(864, 687), (890, 687)], "#7c3aed", label="RL", label_at=(877, 673))
    svg.arrow([(283, 770), (283, 854)], "#b45309", label="adim", label_at=(283, 812))
    svg.arrow([(1112, 770), (1112, 800), (350, 800), (350, 854)], "#7c3aed", label="adim", label_at=(700, 794))
    svg.arrow([(1352, 532), (1352, 800), (1284, 800), (1284, 854)], "#b45309", label="kilit + p_hit", label_at=(1352, 760))
    svg.arrow([(496, 953), (516, 953)], "#dc2626", label="PWM", label_at=(506, 938))
    svg.arrow([(1352, 770), (1352, 800), (1100, 800), (1100, 854)], "#0e7490", label="UART telemetri", label_at=(1230, 794))

    svg.text(40, 1104, "Not: Tum sayisal degerler turret_fcs/config.py (CONFIG) ile ayni kaynaktan gelir.", size=10.5, color=MUTED)
    svg.save(ASSETS / "turret_fcs_architecture.svg")


def flowchart() -> None:
    svg = SVG(1180, 1860)
    svg.text(40, 36, "Turret FCS - Guncel Calisma Akisi", size=21, color=INK, weight="bold")
    svg.text(40, 58, "Her kamera karesinde tekrarlanan algilama, takip, kilit ve atis zinciri", size=12, color=MUTED)

    center = 590
    width = 560
    left = center - width // 2

    def step(y, h, title, bullets, color):
        svg.card(left, y, width, h, title, None, bullets, color)

    # Baslangic
    svg.rect(390, 84, 400, 52, "#ecfdf5", "#059669", rx=26, sw=1.6)
    svg.text(center, 116, "BASLANGIC: sistem ve modeller yuklenir", size=13, color="#065f46", weight="bold", anchor="middle")

    step(166, 86, "1. Donanim ve model baslatma", [
        "Kamera, STM32, LiDAR ve YOLO agirliklari hazirlanir",
        "Telemetri oturumu ve HUD olay gunlugu acilir",
    ], "#0ea5e9")
    step(282, 74, "2. Kare yakalama", [
        "camera_thread en guncel kareyi verir; FPS ve dt olculur",
    ], "#0ea5e9")
    step(386, 84, "3. YOLOv8n + ByteTrack", [
        "Hedef kutulari ve track ID'leri uretilir",
        "Arama / kilitli takip esikleri uygulanir",
    ], "#7c3aed")

    svg.diamond(center, 540, 300, 104, ["4. Guvenilir", "hedef var mi?"], "#f5f3ff", "#7c3aed")
    svg.arrow([(590, 252), (590, 282)], "#0ea5e9")
    svg.arrow([(590, 356), (590, 386)], "#0ea5e9")
    svg.arrow([(590, 470), (590, 488)], "#7c3aed")

    # Hedef yok -> devriye
    svg.card(54, 492, 310, 150, "HAYIR: Devriye", None, [
        "1.5 s son konumu koru",
        "Merkeze yumusak donus",
        "8 s figure-8 tarama",
        "Yeni karede tekrar ara",
    ], "#b45309", tag="patrol_generator.py")
    svg.arrow([(440, 540), (364, 540)], "#b45309", label="HAYIR", label_at=(402, 526))
    svg.arrow([(209, 492), (209, 330), (300, 330)], "#b45309", dash="6 5", label="döngü", label_at=(209, 410))

    # Hedef var -> secim ve kestirim
    step(620, 78, "5. Hedef secimi ve kimlik kapisi", [
        "Merkeze en yakin guvenilir aday; ID atlamasi reddedilir",
    ], "#0f766e")
    step(724, 92, "6. Kalman + menzil kapilama", [
        "CV Kalman: konum ve dunya hizi; ego hareketi cikarilir",
        "LiDAR: 50 cm sicrama reddi, 3 kare kararlilik",
    ], "#0f766e")
    step(842, 86, "7. Lead ve balistik", [
        "Ucus suresi, hedef hizi ve yercekimi super-elevasyonu",
        "Nişan noktasi = olculen nokta + hiz * lead_time",
    ], "#0f766e")
    step(954, 86, "8. Kontrolcu", [
        "[P] hassas kilit veya [RL] cevik takip; tek aktif mod",
        "Yumusak olu bolge + ivme siniri + adim siniri",
    ], "#b45309")
    svg.arrow([(590, 566), (590, 620)], "#7c3aed", label="EVET", label_at=(620, 596))
    svg.arrow([(590, 698), (590, 724)], "#0f766e")
    svg.arrow([(590, 816), (590, 842)], "#0f766e")
    svg.arrow([(590, 928), (590, 954)], "#0f766e")

    svg.diamond(center, 1100, 320, 112, ["9. Kilit", "dogrulandi mi?"], "#fffbeb", "#b45309")
    svg.arrow([(590, 1040), (590, 1044)], "#b45309")
    svg.arrow([(430, 1100), (355, 1100)], "#b45309", label="HAYIR", label_at=(392, 1086))
    svg.card(44, 1052, 300, 112, "Takibe devam", None, [
        "Kilit sayaci azaltilir",
        "Yeni karede tekrar dene",
    ], "#64748b")
    svg.arrow([(194, 1052), (194, 330), (300, 330)], "#64748b", dash="6 5", label="döngü", label_at=(194, 690))

    step(1214, 80, "10. Kilitli hedef - atis izni", [
        "HUD: LEAD ibresi, kilit bari, vurus olasiligi",
        "Operator [Y] ile atisi onaylar; [N] birakir",
    ], "#b45309")
    svg.diamond(center, 1390, 300, 104, ["11. [Y] ile", "atis onayi?"], "#fffbeb", "#b45309")
    svg.arrow([(590, 1156), (590, 1214)], "#b45309", label="EVET", label_at=(620, 1188))
    svg.arrow([(590, 1294), (590, 1338)], "#b45309")
    svg.arrow([(440, 1390), (360, 1390)], "#64748b", label="HAYIR", label_at=(400, 1376))
    svg.card(48, 1342, 300, 96, "Beklemede kal", None, [
        "Atis baslatilmaz",
        "Kilit ve telemetri canli",
    ], "#64748b")

    step(1476, 116, "12. Kademeli atis sekansi", [
        "SPINUP + SLEW: volan devri ve balistik aciya yonelme",
        "SETTLE 0.15 s -> FIRING 80 ms -> RECOVER 0.35 s",
        "EVAL 1.50 s; ilk 0.75 s kilit muafiyeti",
    ], "#dc2626", )
    svg.arrow([(590, 1442), (590, 1476)], "#b45309", label="EVET", label_at=(620, 1460))
    svg.arrow([(590, 1592), (590, 1632)], "#dc2626")

    svg.rect(330, 1632, 520, 64, "#f1f5f9", "#64748b", rx=16, sw=1.4)
    svg.text(center, 1660, "Yeni kare / yeni hedef: 2. adima don", size=13, color=INK, weight="bold", anchor="middle")
    svg.text(center, 1680, "E-STOP her asamada onceliklidir", size=11, color="#dc2626", weight="bold", anchor="middle")
    svg.arrow([(330, 1664), (260, 1664), (260, 330), (300, 330)], "#64748b", dash="6 5", label="döngü", label_at=(260, 980))

    svg.text(40, 1818, "Kilit olcutu: 10 kare kararlilik + dinamik esik + histerezis. Atis yalnizca kilitli, menzili dogrulanmis ve sekans bosken tetiklenir.", size=10.5, color=MUTED)
    svg.save(ASSETS / "turret_fcs_flowchart.svg")


def hardware() -> None:
    svg = SVG(1460, 900, background="#ffffff")
    svg.text(40, 38, "Turret FCS - Guc, Haberlesme ve Eyleme Mimarisi", size=21, color=INK, weight="bold")
    svg.text(40, 60, "12 V ana hat, 6 V motor hatti, izole solenoid surucusu ve STM32 cevre birimleri", size=12, color=MUTED)

    # Guc katmani
    svg.rect(28, 84, 660, 252, "#fff7ed", "#c2410c", rx=16, sw=1.3, dash="7 5")
    svg.text(46, 110, "GUC DAGITIMI", size=13, color="#9a3412", weight="bold")
    svg.card(48, 128, 236, 88, "12 V DC Adaptor", "Ana guc girisi", ["KLD022 / 12 V hatti"], "#c2410c")
    svg.card(320, 128, 300, 88, "XL4015 DC-DC", "12 V -> 6 V buck", ["1000 uF dusuk ESR filtre"], "#ea580c")
    svg.card(48, 236, 572, 78, "6 V motor hatti", "Servolar ve volan motorlari", [
        "Pan / tilt servolar + cift FF-180 volan",
        "Ortak ground yildiz noktasinda birlestirilir",
    ], "#ea580c")
    svg.card(710, 84, 340, 252, "Solenoid Surucu", "12 V darbe kolu", [
        "IRLZ44N logic-level N-MOSFET",
        "1N4007 ters EMK diyodu",
        "Optokuplor ile tetik izolasyonu",
        "80 ms darbe + 150 ms reload",
    ], "#dc2626", tag="flywheel_solenoid.c")
    svg.card(1080, 84, 332, 252, "Guvenlik ve Topraklama", "Donanim korumalari", [
        "Tum moduller ortak GND",
        "Flywheel / solenoid pinleri pull-down",
        "Acil durumda yazilim + donanim kesme",
        "12 V ve 6 V hatlari ayri sigortali",
    ], "#0f766e")

    # Haberlesme / islem
    svg.rect(28, 360, 1384, 192, "#eff6ff", "#1d4ed8", rx=16, sw=1.3, dash="7 5")
    svg.text(46, 386, "HABERLESME VE ISLEM", size=13, color="#1e40af", weight="bold")
    svg.card(48, 404, 350, 122, "PC / Host", "Python 3.10-3.12", [
        "YOLOv8n + ByteTrack",
        "Kalman, balistik, P / RL",
        "HUD ve telemetri",
    ], "#1d4ed8")
    svg.card(430, 404, 360, 122, "STM32F401RE", "ARM Cortex-M4 @ 84 MHz", [
        "115200 Baud ikili protokol",
        "100 Hz servo + LiDAR telemetrisi",
        "300 ms heartbeat / 800 ms IWDG",
    ], "#1d4ed8")
    svg.card(822, 404, 350, 122, "TF-Luna LiDAR", "USART1 | 100 Hz", [
        "0.2 - 8.0 m ToF mesafe",
        "Sinyal gucu ve taret acisi",
        "Menzil kapilama girdisi",
    ], "#0891b2")
    svg.card(1204, 404, 190, 122, "Kamera", "USB video", [
        "C270 1280x720",
        "30 FPS MJPG",
    ], "#7c3aed")

    # Eyleme
    svg.rect(28, 584, 1384, 246, "#f0fdf4", "#15803d", rx=16, sw=1.3, dash="7 5")
    svg.text(46, 610, "EYLEME", size=13, color="#166534", weight="bold")
    svg.card(48, 630, 330, 174, "Pan / Tilt Servolar", "MG996R / DS3218", [
        "50 Hz PWM (TIM3 CH1 / CH2)",
        "Pan +/-75 derece, tilt +/-25 derece",
        "Zaman tabanli 45 derece/s sinir",
        "0.1 derece cozunurluklu geri bildirim",
    ], "#15803d")
    svg.card(408, 630, 330, 174, "Cift Volan", "FF-180 x 2", [
        "23.500 RPM nominal atis devri",
        "Kilitliyken bosta donus",
        "MOSFET enable / PWM",
        "Atistan sonra kontrollu stop",
    ], "#15803d")
    svg.card(768, 630, 300, 174, "Solenoid", "JF-0530B", [
        "12 V darbe ile dart itisi",
        "80 ms enerjilenme",
        "150 ms mekanik reload",
        "Cift tetikleme kilidi",
    ], "#15803d")
    svg.card(1098, 630, 296, 174, "Operator Arayuzu", "HUD + klavye", [
        "Nisangah, kilit bari, LEAD",
        "Vurus olasiligi ve mod banneri",
        "[Y] atis, [N] birak, [Space] E-stop",
        "CSV + meta.json + events.csv",
    ], "#b45309")

    # Oklar: guc
    svg.arrow([(284, 172), (320, 172)], "#c2410c", label="12 V", label_at=(302, 158))
    svg.arrow([(470, 216), (470, 236)], "#ea580c", label="6 V", label_at=(486, 228))
    svg.arrow([(284, 172), (284, 292), (48, 292)], "#c2410c", label="12 V", label_at=(220, 286))
    svg.arrow([(330, 292), (700, 292), (700, 276)], "#c2410c", label="12 V darbe", label_at=(600, 286))

    # Oklar: haberlesme
    svg.arrow([(398, 465), (430, 465)], "#1d4ed8", label="USB", label_at=(414, 450))
    svg.arrow([(790, 465), (822, 465)], "#0891b2", label="UART", label_at=(806, 450))
    svg.arrow([(1204, 465), (1050, 465), (1050, 526), (630, 526), (630, 404)], "#7c3aed", dash="6 5", label="USB video", label_at=(1000, 520))

    # Oklar: eyleme
    svg.arrow([(630, 526), (630, 590), (213, 590), (213, 630)], "#1d4ed8", label="50 Hz PWM", label_at=(470, 584))
    svg.arrow([(700, 526), (700, 590), (573, 590), (573, 630)], "#15803d", label="PWM + EN", label_at=(620, 584))
    svg.arrow([(860, 526), (860, 590), (918, 590), (918, 630)], "#dc2626", label="tetik", label_at=(890, 584))
    svg.arrow([(918, 720), (1000, 720), (1000, 292), (710, 292)], "#dc2626", label="12 V darbe", label_at=(1000, 600))
    svg.arrow([(1350, 630), (1350, 560), (1210, 560), (1210, 526)], "#b45309", dash="6 5", label="HUD", label_at=(1350, 580))

    svg.text(40, 872, "Not: Sinyal hatlari ile guc hatlari ayri tutulur; solenoid ve volan donuslerinde ortak ground tek noktada toplanir.", size=10.5, color=MUTED)
    svg.save(ASSETS / "turret_fcs_hardware.svg")


if __name__ == "__main__":
    architecture()
    flowchart()
    hardware()
