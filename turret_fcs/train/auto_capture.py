"""
=============================================================================
TURRET FCS - OTOMATIK ZAMAN AYARLI BALON VERI TOPLAYICI
=============================================================================
- 5 saniye geri sayimla baslar (hazirlanmaniz icin).
- 40 saniye boyunca saniyede 4-5 kare yakalar (yaklasik 180-200 kare).
- Ilk 20 saniye: Mevcut los isikta balonu sallayin.
- Son 20 saniye: Isik/flas acarak sallamaya devam edin.
- Sure bitince otomatik kapanir.
=============================================================================
"""

import time
from pathlib import Path
import cv2


def main():
    base_dir = Path(__file__).resolve().parents[2]
    save_dir = base_dir / "datasets" / "custom_raw" / "balloon"
    save_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("[HATA] C270 kamerasi acilamadi!")
        return

    window_name = "Turret FCS - Otomatik Balon Kayit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    # 1. ASAMA: 5 Saniye Geri Sayim
    print("\n" + "=" * 60)
    print(" >>> KAMERA ACILDI - 5 SANIYE ICINDE BALONLA GECIN <<<")
    print("=" * 60)
    countdown_start = time.time()
    while time.time() - countdown_start < 5.0:
        ret, frame = cap.read()
        if not ret:
            break
        remaining = 5 - int(time.time() - countdown_start)
        cv2.putText(frame, f"BASLIYOR: {remaining}", (440, 360), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 0, 255), 5)
        cv2.putText(frame, "Balonu elinize alin ve kameraya gecin", (320, 440), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.imshow(window_name, frame)
        if cv2.waitKey(30) & 0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            return

    # 2. ASAMA: 40 Saniyelik Otomatik Cekim
    TOTAL_DURATION = 40.0
    CAPTURE_INTERVAL = 0.22  # Saniyede yaklasik 4.5 kare
    record_start = time.time()
    last_cap_time = 0.0
    saved_count = 0

    print("\n[KAYIT BASLADI!] Balonu kameraya yaklastirin, uzaklastirin, saga-sola sallayin!\n")

    while True:
        now = time.time()
        elapsed = now - record_start
        if elapsed >= TOTAL_DURATION:
            break

        ret, frame = cap.read()
        if not ret:
            break

        # Kare yakalama
        if (now - last_cap_time) >= CAPTURE_INTERVAL:
            last_cap_time = now
            saved_count += 1
            ts = int(now * 1000)
            file_name = save_dir / f"balloon_{ts}.jpg"
            cv2.imwrite(str(file_name), frame)

        # Bilgilendirme ekrani
        display = frame.copy()
        remaining_sec = int(TOTAL_DURATION - elapsed)

        if elapsed < 20.0:
            phase_text = "1. ASAMA: LOS ISIKTA SALLAYIN (YAKIN / UZAK / HIZLI)"
            phase_color = (0, 255, 255)
        else:
            phase_text = "2. ASAMA: VARSA FLAS / MASA LAMBASINI ACIP SALLAYIN!"
            phase_color = (0, 255, 0)

        # Kirmizi REC yanip sonme efekti
        if int(elapsed * 2) % 2 == 0:
            cv2.circle(display, (40, 40), 14, (0, 0, 255), -1)
            cv2.putText(display, "REC", (65, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Kalan Sure: {remaining_sec}s | Kaydedilen Kare: {saved_count}", (140, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display, phase_text, (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, phase_color, 2)

        cv2.imshow(window_name, display)
        if cv2.waitKey(1) & 0xFF == 27:
            print("[IPTAL] ESC ile erken sonlandirildi.")
            break

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 60)
    print(f" [TEBRIKLER!] Toplam {saved_count} adet yuksek kaliteli kare kaydedildi.")
    print(f" Kayit Yeri: {save_dir}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
