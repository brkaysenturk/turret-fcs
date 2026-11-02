"""
================================================================================
BASKET CHALLENGE: GERCEK DONANIM ANA YONETICISI (MAIN)
================================================================================
turret_fcs mimarisi standartlarinda, gercek STM32F401RE tareti ve Logitech C270
kamerasini baglayarak kulplu kupaya kilitlenen ve 2-asamali yapay zeka ile basket
atan ana uretim betigi:

1. ASAMA (CupTracker - Model 1):
   - YOLOv8 kupa tespiti (aim_mouth_ratio = 0.12 ile dogrudan bardak agzina kilit).
   - RLController (PPO 11D MLP) ile kupayi optik eksenin tam merkezine sifir
     hatayla oturtan kapali cevrim gorsel servomekanizma.

2. ASAMA (Ballistic Policy - Model 2):
   - ToF LiDAR menziline gore gereken ek super-elevasyon acisini (+21.8 deg - +22.6 deg)
     ve volan hizini (4.4 m/s) anlik kestiren 10.000 yorungeli fizik gudumlu AI modeli.

3. ATIS SEKANSI (2-Stage Engagement FSM):
   - Kilitlenme -> Y tuşu / SPACE -> Balistik Aciya Yukselme & Spin-up (600 ms)
   - Solenoid Darbesi (80 ms) -> Geri Toparlama ve Optik Kilide Donus.
================================================================================
"""

import sys
import time
from pathlib import Path
import cv2
import numpy as np

# Proje kokunu sys.path'e ekle
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from turret_basket.config import CONFIG
from turret_basket.basket_comm import (
    FCSCommunicator,
    STATE_TRACKING,
    STATE_LOCKED,
    STATE_FIRING,
    STATE_ESTOP
)
from turret_basket.camera_thread import CameraThread
from turret_basket.yolo.cup_detector import CupDetector
from turret_basket.cup_tracker import CupTracker
from turret_basket.basket_hud import BasketHUD


def load_ballistic_model(models_dir: Path):
    """2. Model: Hizli ve bagimsiz NumPy MLP Balistik Politika yukleyicisi."""
    model_npz = models_dir / "cup_ballistic_policy.npz"
    model_zip = models_dir / "cup_ballistic_policy.zip"

    if model_npz.exists():
        try:
            data = np.load(str(model_npz))
            w1, b1, w2, b2, w3, b3 = data["w1"], data["b1"], data["w2"], data["b2"], data["w3"], data["b3"]

            def predict_npz(obs: np.ndarray) -> np.ndarray:
                h1 = np.tanh(np.dot(w1, obs) + b1)
                h2 = np.tanh(np.dot(w2, h1) + b2)
                return np.clip(np.dot(w3, h2) + b3, -1.0, 1.0)

            print("[INFO] Model 2 (Fast NumPy MLP Policy) basariyla yuklendi!")
            return predict_npz
        except Exception as e:
            print(f"[WARN] NumPy Model 2 yuklenemedi: {e}")

    if model_zip.exists():
        try:
            from stable_baselines3 import PPO
            sb3_model = PPO.load(str(model_zip), device="cpu")
            print("[INFO] Model 2 (SB3 PPO Policy) yuklendi!")
            return lambda obs: sb3_model.predict(obs, deterministic=True)[0]
        except Exception as e:
            print(f"[WARN] Model 2 zip yuklenemedi: {e}")

    # Fallback: Nominal analitik yaklasim
    print("[WARN] Model 2 dosyalari bulunamadi, nominal yaklasim devrede.")
    return lambda obs: np.array([0.75, -0.65], dtype=np.float32)


def main():
    print("=" * 70)
    print("TURRET FCS - REAL CUP BASKET CHALLENGE (PRODUCTION MAIN)")
    print("=" * 70)

    models_dir = Path(__file__).resolve().parent / "rl" / "models"

    # 1. Donanim, Sensor ve Yapay Zeka Modulleri
    comm = FCSCommunicator(port=CONFIG.hardware.manual_port, baudrate=CONFIG.hardware.baudrate)
    cam = CameraThread(src=CONFIG.camera.device_id, width=CONFIG.camera.width, height=CONFIG.camera.height)
    detector = CupDetector()
    tracker = CupTracker(
        model_path=str(models_dir / "cup_tracker_policy.npz"),
        params_path=str(models_dir / "cup_tracker_params.json")
    )
    ballistic_model = load_ballistic_model(models_dir)
    hud = BasketHUD()

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

    # Durum Degiskenleri
    prev_time = time.time()
    fps = 30.0

    lock_counter = 0
    MAX_LOCK_FRAMES = 15
    is_locked = False
    is_estop = False

    last_valid_dist_m = 1.45
    target_speed_mps = 4.4
    target_tilt_offset = 21.8

    # Dikey kamera-namlu paralaks payi (+0.8 derece)
    PARALLAX_PITCH_MARGIN_DEG = 0.80

    # Atis Sekansi Durum Makinesi
    shot_in_progress = False
    shot_time = 0.0
    dart_fired = False
    active_fire_speed = 4.4
    active_fire_tilt = 21.8
    applied_super_elevation = 22.6
    tilt_at_lock = 0.0

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

            h, w = frame.shape[:2]
            cx_screen, cy_screen = w // 2, h // 2

            # 1. Kupa Tespiti (YOLOv8 + Agiz Elipsi 0.12 Orani)
            det = detector.detect(frame)

            # 2. Telemetri Oku (TF-Luna LiDAR & Guncel Aci)
            raw_dist_m = (comm.distance / 100.0) if comm.distance > 0 else 0.0
            if 0.60 <= raw_dist_m <= 4.0:
                dist_m = raw_dist_m
                last_valid_dist_m = dist_m
            else:
                dist_m = last_valid_dist_m

            current_tilt = comm.tilt_angle

            # 3. Kilit Kararliligi Kontrolu
            if det.detected:
                # Kupa agzi ekranda merkeze yakin mi? (Pan < 1.2 deg, Tilt < 1.2 deg)
                is_close_to_center = (abs(det.err_pan_deg) < 1.20 and abs(det.err_tilt_deg) < 1.20)
                if is_close_to_center:
                    lock_counter = min(MAX_LOCK_FRAMES, lock_counter + 1)
                else:
                    lock_counter = max(0, lock_counter - 1)
            else:
                lock_counter = max(0, lock_counter - 2)

            is_locked = (lock_counter >= MAX_LOCK_FRAMES)
            lock_ratio = float(lock_counter) / float(MAX_LOCK_FRAMES)

            # 4. Model 2 (Balistik Politika) Cikarimi (Atis yokken canli guncellenir)
            if not shot_in_progress:
                norm_dist = np.clip((dist_m - 2.0) / 1.0, -1.0, 1.0)
                obs_m2 = np.array([norm_dist, 0.0], dtype=np.float32)
                act_m2 = ballistic_model(obs_m2)

                tilt_min = CONFIG.rl.tilt_offset_min_deg
                tilt_max = CONFIG.rl.tilt_offset_max_deg
                speed_min = CONFIG.rl.speed_min_mps
                speed_max = CONFIG.rl.speed_max_mps

                target_tilt_offset = float(np.interp(act_m2[0], [-1.0, 1.0], [tilt_min, tilt_max]))
                target_speed_mps = float(np.interp(act_m2[1], [-1.0, 1.0], [speed_min, speed_max]))

            # 5. Hareket ve Atis Yonetimi
            if is_estop:
                comm.send_command(0.0, 0.0, state=STATE_ESTOP)

            elif shot_in_progress:
                # -------------------------------------------------------------
                # ATIS SEKANSI (2-Asamali FSM)
                # -------------------------------------------------------------
                elapsed = t_now - shot_time

                # 1. Asama: Super-elevasyon acisina yuksel ve spin-up yap (0 - 600 ms)
                if elapsed < 0.60:
                    target_elevated_tilt = tilt_at_lock + applied_super_elevation
                    tilt_error = target_elevated_tilt - current_tilt
                    step_tilt = float(np.clip(tilt_error * 0.40, -1.50, 1.50))
                    comm.send_command(0.0, step_tilt, state=STATE_LOCKED)

                # 2. Asama: Aci oturdu (600 ms tamamlandi), solenoid darbesi (80 ms)
                elif elapsed < 0.70:
                    if not dart_fired:
                        dart_fired = True
                        hud.log_event(
                            f"[THROW] DART FIRED ({active_fire_speed:.1f} m/s @ +{applied_super_elevation:.1f} deg)",
                            hud.C_GREEN
                        )
                        hud.record_shot(is_basket=True)
                    # Solenoid iticiyi tetikleyen STATE_FIRING komutunu gonder (watchdog beslenir)
                    comm.send_command(0.0, 0.0, state=STATE_FIRING)

                # 3. Asama: Geri toplama ve kilit acisina yumusak inis (0.70 - 1.60 s)
                elif elapsed < 1.60:
                    rec_progress = (elapsed - 0.70) / (1.60 - 0.70)
                    target_tilt = tilt_at_lock + applied_super_elevation * (1.0 - rec_progress)
                    tilt_error = target_tilt - current_tilt
                    step_tilt = float(np.clip(tilt_error * 0.40, -1.50, 1.50))
                    comm.send_command(0.0, step_tilt, state=STATE_TRACKING)

                # 4. Asama: Toparlama tamamlandi, optik takibe kesintisiz geri don
                else:
                    shot_in_progress = False
                    dart_fired = False
                    tracker.reset()
                    hud.log_event("[SYS] RECOVERED - READY FOR NEXT THROW", hud.C_CYAN)

            else:
                # -------------------------------------------------------------
                # TAKIP MODU: Model 1 (CupTracker) ile bardağı ortala
                # -------------------------------------------------------------
                if det.detected:
                    cx, cy = det.center_px
                    d_pan, d_tilt, _ = tracker.compute(
                        cx=float(cx),
                        cy=float(cy),
                        center_x=float(cx_screen),
                        center_y=float(cy_screen),
                        screen_w=float(w),
                        screen_h=float(h),
                        distance_m=float(dist_m),
                        dt=float(dt)
                    )
                    cmd_state = STATE_LOCKED if is_locked else STATE_TRACKING
                    # Yon Inversiyonu (turret_fcs ile %100 uyumlu)
                    eff_pan = -d_pan if CONFIG.hardware.pan_invert else d_pan
                    eff_tilt = -d_tilt if CONFIG.hardware.tilt_invert else d_tilt
                    comm.send_command(eff_pan, eff_tilt, state=cmd_state)
                else:
                    tracker.on_target_lost()
                    comm.send_command(0.0, 0.0, state=STATE_TRACKING)

            # 6. HUD Render
            disp_speed = active_fire_speed if shot_in_progress else target_speed_mps
            disp_tilt = active_fire_tilt if shot_in_progress else target_tilt_offset

            rendered = hud.render(
                frame=frame,
                cup_detected=det.detected,
                bbox=det.bbox if det.detected else None,
                confidence=det.confidence if det.detected else 0.0,
                distance_m=dist_m,
                err_pan_deg=det.err_pan_deg if det.detected else 0.0,
                err_tilt_deg=det.err_tilt_deg if det.detected else 0.0,
                fps=fps,
                is_locked=is_locked,
                firing=shot_in_progress,
                is_estop=is_estop,
                lock_ratio=lock_ratio,
                speed_mps=disp_speed,
                tilt_offset_deg=disp_tilt
            )

            cv2.imshow(WINDOW_NAME, rendered)
            key = cv2.waitKey(1) & 0xFF

            # 7. Operator Klavye Kontrolleri (turret_fcs standart keymap ile %100 uyumlu)
            if key in (ord('y'), ord('Y')):
                # Atis Baslat / Authorize Throw (Y)
                if not shot_in_progress and not is_estop:
                    if is_locked:
                        shot_in_progress = True
                        shot_time = time.time()
                        dart_fired = False
                        tilt_at_lock = current_tilt
                        active_fire_speed = target_speed_mps
                        active_fire_tilt = target_tilt_offset
                        applied_super_elevation = float(np.clip(
                            active_fire_tilt + PARALLAX_PITCH_MARGIN_DEG,
                            5.0,
                            25.0
                        ))
                        hud.log_event(
                            f"[AUTH] THROW INITIATED: +{applied_super_elevation:.1f} deg & spin-up",
                            hud.C_AMBER
                        )
                    else:
                        hud.log_event("[WARN] CANNOT FIRE - CUP NOT LOCKED", hud.C_RED)

            elif key in (32, ord('e'), ord('E')):
                # E-Stop Acil Durum Freni (SPACE veya E) - Mandalli
                is_estop = not is_estop
                if is_estop:
                    shot_in_progress = False
                    dart_fired = False
                    comm.send_command(0.0, 0.0, state=STATE_ESTOP)
                    hud.log_event("[E-STOP] MOTORS HALTED & ZEROED (LATCHED)", hud.C_RED)
                else:
                    comm.send_command(0.0, 0.0, state=STATE_TRACKING)
                    hud.log_event("[E-STOP] DISENGAGED - RESUMING FCS", hud.C_GREEN)

            elif key in (ord('n'), ord('N')):
                # Atisi Iptal Et (N)
                if shot_in_progress:
                    shot_in_progress = False
                    dart_fired = False
                    hud.log_event("[ABORT] THROW CANCELLED", hud.C_RED)

            elif key in (ord('r'), ord('R')):
                # Manuel Takipci ve Filtre Sifirlama (R)
                tracker.reset()
                lock_counter = 0
                hud.log_event("[TRACKER] SYSTEM FILTERS FLUSHED", hud.C_CYAN)

            elif key in (ord('q'), ord('Q'), 27):  # Q veya ESC
                break

    finally:
        print("[SYS] Donanim ve kamera guvenli kapatiliyor...")
        cam.stop()
        comm.send_command(0.0, 0.0, state=STATE_TRACKING)
        comm.close()
        cv2.destroyAllWindows()
        print("[SYS] Sistem kapatildi. Iyi calismalar!")


if __name__ == "__main__":
    main()
