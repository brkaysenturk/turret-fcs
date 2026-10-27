"""
=============================================================================
TURRET FCS - HIZLI VERI SETI TOPLAYICI (C270 KAMERA KAYIT VE KARE AYIKLAYICI)
=============================================================================
Kullanim:
  - [SPACE] : Manuel tek bir kare yakalar ve kaydeder.
  - [R]     : Otomatik kaydi baslatir/durdurur (Saniyede 3 kare alir, balonu sallayin!).
  - [B]     : MOD DEGISTIRIR: Balon Modu <-> Bos Arka Plan (Negatif Ornek) Modu.
  - [Q]/[ESC]: Cikis yapar.
=============================================================================
"""

import time
from pathlib import Path
import cv2


def main():
    base_dir = Path(__file__).resolve().parents[2]
    save_dir = base_dir / "datasets" / "custom_raw"
    balloon_dir = save_dir / "balloon"
    bg_dir = save_dir / "background"

    balloon_dir.mkdir(parents=True, exist_ok=True)
    bg_dir.mkdir(parents=True, exist_ok=True)

    from turret_fcs.camera_thread import CameraThread
    from turret_fcs.config import CONFIG

    cam_idx, cam_api = CameraThread.detect_camera()
    cap = cv2.VideoCapture(cam_idx, cam_api)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CONFIG.optics.default_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG.optics.default_height)

    if not cap.isOpened():
        print("[HATA] C270 kamerasi acilamadi! Baska bir program tarafindan kullaniliyor olabilir.")
        return

    is_auto_recording = False
    is_background_mode = False
    last_capture_time = 0.0
    CAPTURE_INTERVAL = 0.30  # Saniyede yaklasik 3-4 kare (fazla benzer kareleri onler)

    count_balloon = len(list(balloon_dir.glob("*.jpg")))
    count_bg = len(list(bg_dir.glob("*.jpg")))

    window_name = "C270 Dataset Capture Tool"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("\n" + "=" * 65)
    print(" [C270 VERI TOPLAYICI] BASLATILDI")
    print(f" Hedef Klasor: {save_dir}")
    print("   [SPACE]      : Tek kare yakala")
    print("   [R]          : Otomatik seri cekim (Ac/Kapa)")
    print("   [B]          : Mod Degistir (Hedef Balon <-> Bos Arka Plan)")
    print("   [Q] / [ESC]  : Kaydi bitir ve cik")
    print("=" * 65 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        now = time.time()
        display_frame = frame.copy()

        # Otomatik seri yakalama tetikleyicisi
        should_capture = False
        if is_auto_recording and (now - last_capture_time) >= CAPTURE_INTERVAL:
            should_capture = True

        key = cv2.waitKey(1) & 0xFF
        if key in [27, ord('q'), ord('Q')]:
            break
        elif key == 32:  # SPACE
            should_capture = True
        elif key in [ord('r'), ord('R')]:
            is_auto_recording = not is_auto_recording
            status_str = "BASLADI" if is_auto_recording else "DURDURULDU"
            print(f"[SERI KAYIT] {status_str}")
        elif key in [ord('b'), ord('B')]:
            is_background_mode = not is_background_mode
            mode_str = "BOS ARKA PLAN (NEGATIF ORNEK)" if is_background_mode else "HEDEF BALON"
            print(f"[MOD DEGISTI] Aktif Mod: {mode_str}")

        if should_capture:
            last_capture_time = now
            timestamp_str = int(now * 1000)
            if is_background_mode:
                count_bg += 1
                file_path = bg_dir / f"bg_{timestamp_str}.jpg"
                cv2.imwrite(str(file_path), frame)
            else:
                count_balloon += 1
                file_path = balloon_dir / f"balloon_{timestamp_str}.jpg"
                cv2.imwrite(str(file_path), frame)

        # Bilgi Arayuzu (Overlay)
        mode_text = "MOD: BOS ARKA PLAN (NEGATIF)" if is_background_mode else "MOD: HEDEF BALON"
        mode_color = (0, 165, 255) if is_background_mode else (0, 255, 0)
        rec_text = "SERI CEKIM: AKTIF (KAYITTA)" if is_auto_recording else "SERI CEKIM: DURDU (R'ye bas)"
        rec_color = (0, 0, 255) if is_auto_recording else (180, 180, 180)

        cv2.putText(display_frame, mode_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        cv2.putText(display_frame, rec_text, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, rec_color, 2)
        cv2.putText(display_frame, f"Balon Karesi: {count_balloon}  |  Arka Plan: {count_bg}", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display_frame, "[R]: Seri Kayit  [B]: Mod Degistir  [SPACE]: Tek Cekim  [Q]: Cikis", (20, 690), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        cv2.imshow(window_name, display_frame)

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[TAMAMLANDI] Toplanan Veri: {count_balloon} Balon, {count_bg} Arka Plan karesi.")
    print(f"Gorseller su klasorde: {save_dir}\n")


if __name__ == "__main__":
    main()
