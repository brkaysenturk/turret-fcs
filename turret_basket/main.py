"""
================================================================================
BASKET CHALLENGE: GERCEK DONANIM ANA YONETICISI (MAIN)
================================================================================
turret_fcs mimarisi standartlarinda, gercek STM32F401RE tareti ve Logitech C270
kamerasini baglayarak kulplu kupaya kilitlenen ve 2-asamali yapay zeka ile basket
atan ana uretim betigi:

- BasketManager: Hedef tespiti, LiDAR menzil kapilama, RL takip ve balistik cikarim.
- BasketEngagement: 2-asamali firlatma durum makinesi (Spin-up -> Firing -> Recovery).
- BasketPresenter: Sivil robotik hiyerarsik bas-ustu gosterge sunucusu.
================================================================================
"""

import sys
import time
from pathlib import Path
import cv2

# Proje kokunu sys.path'e ekle
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from turret_basket.config import CONFIG
from turret_basket.protocol import STATE_TRACKING
from turret_basket.basket_comm import FCSCommunicator
from turret_basket.camera_thread import CameraThread
from turret_basket.yolo.cup_detector import CupDetector
from turret_basket.rl_tracker import RLTracker
from turret_basket.rl_ballistics import RLBallistics
from turret_basket.engagement import BasketEngagement
from turret_basket.basket_manager import BasketManager
from turret_basket.basket_hud import BasketHUD
from turret_basket.presenter import BasketPresenter
from turret_basket.keymap import process_key_event


def main():
    print("=" * 70)
    print("TURRET FCS - REAL CUP BASKET CHALLENGE (PRODUCTION MAIN)")
    print("=" * 70)

    models_dir = Path(__file__).resolve().parent / "rl" / "models"

    # 1. Donanim ve Yapay Zeka Alt Bilesenleri
    comm = FCSCommunicator(port=CONFIG.hardware.manual_port, baudrate=CONFIG.hardware.baudrate)
    cam = CameraThread(src=CONFIG.camera.device_id, width=CONFIG.camera.width, height=CONFIG.camera.height)
    detector = CupDetector()
    tracker = RLTracker(
        model_path=str(models_dir / "cup_tracker_policy.npz"),
        params_path=str(models_dir / "cup_tracker_params.json")
    )
    ballistics = RLBallistics(models_dir)
    engagement = BasketEngagement()

    # 2. Sistem Orkestratoru ve Arayuz Sunucusu
    manager = BasketManager(
        comm=comm,
        detector=detector,
        tracker=tracker,
        ballistics=ballistics,
        engagement=engagement,
    )
    hud = BasketHUD()
    presenter = BasketPresenter(hud)

    WINDOW_NAME = "Turret FCS - Real Cup Basket Challenge (Main)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 960, 540)

    print("\n[KONTROLLER]")
    print("  SPACE / Y : Atis Emri (2 Asamali Spin-up -> Yukselis -> Firlatma)")
    print("  N         : Atisi Iptal Et (Abort)")
    print("  R         : Hedef Takipcisini Sifirla")
    print("  E         : E-Stop Acil Durum Freni")
    print("  Q / ESC   : Guvenli Cikis\n")

    hud.log_event("[SYS] CUP BASKET FCS ONLINE", hud.C_GREEN)
    hud.log_event("[AI] 2-STAGE HIERARCHICAL AGENTS READY", hud.C_CYAN)

    prev_time = time.time()
    fps = 30.0

    try:
        while True:
            t_now = time.time()
            dt = max(1e-4, t_now - prev_time)
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
            prev_time = t_now

            grabbed, frame = cam.read()
            if not grabbed or frame is None:
                time.sleep(0.01)
                continue

            # 1. Tum Karar ve Kontrol Dongusunu Yonet
            status = manager.step(
                frame=frame,
                dt=dt,
                now=t_now,
                on_event_cb=hud.log_event,
                on_shot_cb=hud.record_shot,
            )

            # 2. Bas-Ustu Gostergesini (HUD) Ciz
            rendered = presenter.render(
                frame=frame,
                status=status,
                fps=fps,
            )

            cv2.imshow(WINDOW_NAME, rendered)
            key = cv2.waitKey(1) & 0xFF

            # 3. Operator Klavye Girdilerini Isle
            if process_key_event(key_code=key, manager=manager, hud=hud):
                break

    finally:
        print("[SYS] Donanim ve kamera guvenli kapatiliyor...")
        cam.stop()
        if comm:
            comm.send_command(0.0, 0.0, state=STATE_TRACKING)
            comm.close()
        cv2.destroyAllWindows()
        print("[SYS] Sistem kapatildi. Iyi calismalar!")


if __name__ == "__main__":
    main()
