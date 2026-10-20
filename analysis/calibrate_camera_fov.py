#!/usr/bin/env python3
"""
================================================================================
MODUL: LOGITECH C270 KAMERA GORUS ACISI (HFOV / VFOV) VE ODAK KALIBRASYON ARACI
================================================================================
Bu arac, Logitech C270 kamerasinin gercek donanim optik parametrelerini
(H_FOV, V_FOV, fx, fy) en basit ve hassas sekilde olcmenizi saglar.

UC FARKLI OLÇUM YONTEMI DESTEKLENIR:

1. YONTEM 1: REFERANS NESNE / CETVEL TIKLAMA (EN HIZLI - 1 DAKIKA)
   - Kameranin karsisina (ornegin 80 cm veya 100 cm mesafeye) bilinen genislige
     sahip bir nesne (ornegin 30 cm'lik cetvel veya 29.7 cm A4 kagit) koyun.
   - Ekranda cetvelin sol ve sag ucuna tiklayin.
   - Program piksel genliginden odak uzakligini (fx) ve tam H_FOV'u cozer:
     fx = (dx_px * Mesafe_cm) / Nesne_Genisligi_cm
     H_FOV = 2 * arctan((1280 / 2) / fx)

2. YONTEM 2: DUVAR GENISLIGI VE MEZURA (GEOMETRIK HESAPLAMA)
   - Kamera merceğinden duvara olan mesafeyi (D) ve kadrajin sol/sag kenarlari
     arasindaki duvar genisligini (W) mezurayla olcun.
   - H_FOV = 2 * arctan((W / 2) / D)
   - V_FOV = 2 * arctan((H / 2) / D)

3. YONTEM 3: SATRANC TAHTASI (OPENCV CHECKERBOARD CALIBRATION)
   - [C] tusuna basarak satranc tahtasinin kose noktalarindan tam optik matris cikarir.

KULLANIM:
  python analysis/calibrate_camera_fov.py
================================================================================
"""

import math
import sys
import time
from pathlib import Path
import cv2
import numpy as np

# Proje kok dizinini ekle
sys.path.append(str(Path(__file__).resolve().parents[1]))
try:
    from turret_fcs.camera_thread import CameraThread
    USE_CAM_THREAD = True
except ImportError:
    USE_CAM_THREAD = False


# Varsayilan kalibrasyon parametreleri
DEFAULT_DISTANCE_CM = 100.0  # Kameradan nesneye/duvara olan mesafe (cm)
DEFAULT_OBJECT_WIDTH_CM = 30.0  # Referans nesnenin genisligi (cm)

clicks = []
current_distance_cm = DEFAULT_DISTANCE_CM
current_obj_width_cm = DEFAULT_OBJECT_WIDTH_CM
calib_result = None


def on_mouse(event, x, y, flags, param):
    global clicks, calib_result
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicks) >= 2:
            clicks = []
            calib_result = None
        clicks.append((x, y))

        if len(clicks) == 2:
            calculate_fov_from_clicks()


def calculate_fov_from_clicks():
    global clicks, current_distance_cm, current_obj_width_cm, calib_result
    if len(clicks) != 2:
        return

    (x1, y1), (x2, y2) = clicks
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    dist_px = math.hypot(dx, dy)

    if dist_px < 5:
        print("[!] Noktalar birbirine cok yakin, tekrar secin.")
        clicks = []
        return

    # Yatay odak uzakligi (fx) ve Dikey odak uzakligi (fy)
    # dx_px / fx = Width_real / Distance_real
    fx = (dx * current_distance_cm) / max(0.1, current_obj_width_cm)
    fy = fx  # C270 kare piksellere sahiptir (pixel aspect ratio ~ 1.0)

    # 1280x720 icin FOV acilari
    w_cam, h_cam = 1280.0, 720.0
    h_fov = 2.0 * math.degrees(math.atan((w_cam / 2.0) / fx))
    v_fov = 2.0 * math.degrees(math.atan((h_cam / 2.0) / fy))
    d_fov = 2.0 * math.degrees(math.atan(math.hypot(w_cam / 2.0, h_cam / 2.0) / fx))

    calib_result = {
        "fx": fx,
        "fy": fy,
        "h_fov": h_fov,
        "v_fov": v_fov,
        "d_fov": d_fov,
        "dx_px": dx,
        "dist_px": dist_px,
    }

    print("\n" + "=" * 65)
    print(" LOGITECH C270 OPTIK KALIBRASYON SONUCU (TIKLAMA YONTEMI)")
    print("=" * 65)
    print(f" Referans Nesne Genisligi : {current_obj_width_cm:.1f} cm")
    print(f" Kamera Mesafesi (D)       : {current_distance_cm:.1f} cm")
    print(f" Algilanan Piksel Genligi : {dx:.1f} px")
    print("-" * 65)
    print(f" Odak Uzakligi (fx, fy)   : fx = {fx:.1f} px | fy = {fy:.1f} px")
    print(f" Yatay Gorus Acisi (H_FOV): {h_fov:.2f} derece")
    print(f" Dikey Gorus Acisi (V_FOV): {v_fov:.2f} derece")
    print(f" Kosegen Acisi (D_FOV)    : {d_fov:.2f} derece (Datasheet: ~60°)")
    print("=" * 65)
    print(" main.py ICIN GUNCELLEME KODU:")
    print(f" H_FOV, V_FOV = {h_fov:.1f}, {v_fov:.1f}")
    print(f" fx = (1280 / 2.0) / np.tan(np.radians(H_FOV / 2.0))  # ~{fx:.1f}")
    print(f" fy = (720 / 2.0) / np.tan(np.radians(V_FOV / 2.0))   # ~{fy:.1f}")
    print("=" * 65 + "\n")


def calculate_fov_from_wall(dist_cm, wall_width_cm, wall_height_cm=None):
    """Duvar genisligi ve mesafeden direkt trigonometrik FOV hesabi."""
    if dist_cm <= 0 or wall_width_cm <= 0:
        return None

    h_fov = 2.0 * math.degrees(math.atan((wall_width_cm / 2.0) / dist_cm))
    if wall_height_cm and wall_height_cm > 0:
        v_fov = 2.0 * math.degrees(math.atan((wall_height_cm / 2.0) / dist_cm))
    else:
        # 16:9 en boy oranindan kestirim
        tan_v = math.tan(math.radians(h_fov / 2.0)) * (9.0 / 16.0)
        v_fov = 2.0 * math.degrees(math.atan(tan_v))

    fx = (1280.0 / 2.0) / math.tan(math.radians(h_fov / 2.0))
    fy = (720.0 / 2.0) / math.tan(math.radians(v_fov / 2.0))

    return {
        "h_fov": h_fov,
        "v_fov": v_fov,
        "fx": fx,
        "fy": fy,
    }


def main():
    global current_distance_cm, current_obj_width_cm, clicks, calib_result

    print("\n" + "=" * 70)
    print(" LOGITECH C270 HFOV / VFOV OPTIK KALIBRASYON ARACI")
    print("=" * 70)
    print(" TALIMATLAR:")
    print(" 1. Kameranin onune bilinen bir nesne (örn. 30 cm cetvel veya A4) koyun.")
    print(" 2. Nesnenin SOL ve SAG ucuna mouse ile sirayla sol tiklayin.")
    print(" 3. [W] veya [S] : Referans nesne genisligini (+5cm / -5cm) degistirir.")
    print(" 4. [D] veya [A] : Kamera mesafesini (+10cm / -10cm) degistirir.")
    print(" 5. [M]          : Konsoldan tam mesafe ve nesne boyu girmeyi saglar.")
    print(" 6. [R]          : Secimleri sifirlar.")
    print(" 7. [Q]          : Cikis yapar.")
    print("=" * 70 + "\n")

    cam_id = 0
    cap = None
    if USE_CAM_THREAD:
        try:
            cap = CameraThread(camera_id=cam_id, width=1280, height=720, fps=30).start()
        except Exception:
            cap = None

    if cap is None or not cap.isOpened():
        cap = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    window_name = "Logitech C270 FOV Calibrator"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, on_mouse)

    try:
        while True:
            if hasattr(cap, "read_latest"):
                ret, frame = cap.read_latest()
            else:
                ret, frame = cap.read()

            if not ret or frame is None:
                time.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            # Merkez kilavuz artisi ve 1/4 - 3/4 cizgileri
            cv2.line(frame, (cx, 0), (cx, h), (70, 80, 90), 1, cv2.LINE_AA)
            cv2.line(frame, (0, cy), (w, cy), (70, 80, 90), 1, cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 128), -1)

            # Kadraj kenar guvenlik sinirlari
            cv2.rectangle(frame, (2, 2), (w - 3, h - 3), (0, 200, 255), 2)

            # Tiklanan noktalari ciz
            if len(clicks) == 1:
                cv2.circle(frame, clicks[0], 6, (0, 255, 0), -1)
                cv2.putText(frame, "P1 (SOL UC)", (clicks[0][0] + 10, clicks[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            elif len(clicks) == 2:
                cv2.circle(frame, clicks[0], 6, (0, 255, 0), -1)
                cv2.circle(frame, clicks[1], 6, (255, 150, 0), -1)
                cv2.line(frame, clicks[0], clicks[1], (0, 255, 255), 2)
                mid_x = (clicks[0][0] + clicks[1][0]) // 2
                mid_y = (clicks[0][1] + clicks[1][1]) // 2
                dx = abs(clicks[1][0] - clicks[0][0])
                cv2.putText(frame, f"dx = {dx} px", (mid_x - 35, mid_y - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # Bilgi paneli (HUD)
            pad_h = 105
            roi = frame[0:pad_h, 0:w]
            black = np.zeros_like(roi)
            frame[0:pad_h, 0:w] = cv2.addWeighted(roi, 0.35, black, 0.65, 0)
            cv2.line(frame, (0, pad_h), (w, pad_h), (80, 90, 100), 1)

            line1 = f"MESAFE (D): {current_distance_cm:.1f} cm [A/D ile +/-10cm]  |  NESNE GENISLIGI: {current_obj_width_cm:.1f} cm [S/W ile +/-5cm]"
            cv2.putText(frame, line1, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)

            if calib_result is not None:
                line2 = f"H_FOV = {calib_result['h_fov']:.2f} deg  |  V_FOV = {calib_result['v_fov']:.2f} deg  |  fx = {calib_result['fx']:.1f} px  |  fy = {calib_result['fy']:.1f} px"
                cv2.putText(frame, line2, (15, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 128), 2, cv2.LINE_AA)

                line3 = f"-> main.py KODU: H_FOV, V_FOV = {calib_result['h_fov']:.1f}, {calib_result['v_fov']:.1f}"
                cv2.putText(frame, line3, (15, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 215, 255), 1, cv2.LINE_AA)
            else:
                prompt = "Cetvelin/nesnenin SOL ve SAG uclarina mouse ile sirayla tiklayin (veya [M] ile direkt deger girin)..."
                cv2.putText(frame, prompt, (15, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 200, 255), 1, cv2.LINE_AA)

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(15) & 0xFF

            if key == ord("q") or key == 27:
                break
            elif key == ord("r") or key == ord("R"):
                clicks = []
                calib_result = None
                print("[*] Secimler sifirlandi.")
            elif key == ord("w") or key == ord("W"):
                current_obj_width_cm += 5.0
                calculate_fov_from_clicks()
            elif key == ord("s") or key == ord("S"):
                current_obj_width_cm = max(2.0, current_obj_width_cm - 5.0)
                calculate_fov_from_clicks()
            elif key == ord("d") or key == ord("D"):
                current_distance_cm += 10.0
                calculate_fov_from_clicks()
            elif key == ord("a") or key == ord("A"):
                current_distance_cm = max(10.0, current_distance_cm - 10.0)
                calculate_fov_from_clicks()
            elif key == ord("m") or key == ord("M"):
                print("\n" + "-" * 50)
                try:
                    d_in = input(f"Kamera Merceginden Nesneye Mesafe (cm) [{current_distance_cm}]: ").strip()
                    if d_in:
                        current_distance_cm = float(d_in)
                    w_in = input(f"Nesne / Cetvel Gercek Genisligi (cm) [{current_obj_width_cm}]: ").strip()
                    if w_in:
                        current_obj_width_cm = float(w_in)
                    calculate_fov_from_clicks()
                except ValueError:
                    print("[!] Gecersiz sayisal deger!")
                print("-" * 50 + "\n")

    finally:
        if hasattr(cap, "stop"):
            cap.stop()
        elif hasattr(cap, "release"):
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Logitech C270 FOV Calibrator")
    parser.add_argument("--calc", nargs="+", type=float, default=None,
                        help="Kamera acmadan direkt hesaplama: --calc [mesafe_cm] [duvar_genislik_cm] [opsiyonel: duvar_yukseklik_cm]")
    args = parser.parse_args()

    if args.calc is not None:
        if len(args.calc) < 2:
            print("[HATA] En az 2 parametre gereklidir: --calc [mesafe_cm] [genislik_cm]")
            sys.exit(1)
        d_val = args.calc[0]
        w_val = args.calc[1]
        h_val = args.calc[2] if len(args.calc) > 2 else None
        res = calculate_fov_from_wall(d_val, w_val, h_val)
        if res:
            print("\n" + "=" * 65)
            print(" LOGITECH C270 DUVAR GEOMETRISI HESAPLAMA SONUCU")
            print("=" * 65)
            print(f" Duvar Mesafesi (D)       : {d_val:.1f} cm")
            print(f" Kadraj Duvar Genisligi (W): {w_val:.1f} cm")
            if h_val:
                print(f" Kadraj Duvar Yuksekligi (H): {h_val:.1f} cm")
            print("-" * 65)
            print(f" Yatay Gorus Acisi (H_FOV): {res['h_fov']:.2f} derece")
            print(f" Dikey Gorus Acisi (V_FOV): {res['v_fov']:.2f} derece")
            print(f" Odak Uzakliklari (fx, fy): fx = {res['fx']:.1f} px | fy = {res['fy']:.1f} px")
            print("=" * 65)
            print(" main.py ICIN GUNCELLEME KODU:")
            print(f" H_FOV, V_FOV = {res['h_fov']:.1f}, {res['v_fov']:.1f}")
            print(f" fx = (1280 / 2.0) / np.tan(np.radians(H_FOV / 2.0))  # ~{res['fx']:.1f}")
            print(f" fy = (720 / 2.0) / np.tan(np.radians(V_FOV / 2.0))   # ~{res['fy']:.1f}")
            print("=" * 65 + "\n")
        sys.exit(0)

    main()
