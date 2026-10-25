"""FCSManager duman ve entegrasyon testleri."""
import pytest

from turret_fcs.config import CONFIG
from turret_fcs.fcs_manager import FCSManager


def test_fcs_lifecycle_tracking_and_lock_recovery():
    fcs = FCSManager(control_mode="P")
    assert fcs.control_mode in ("P", "RL")
    assert fcs.PAN_LIMIT_MIN < fcs.PAN_LIMIT_MAX
    assert (fcs.target_velocity[0], fcs.target_velocity[1]) == (0.0, 0.0)

    # Taret açı senkronizasyonu
    fcs.sync_turret_pose(15.5, -5.2)
    assert (fcs.turret_pan_deg, fcs.turret_tilt_deg) == pytest.approx((15.5, -5.2))

    # Tekil hedef güncelleme ve komut sınırlandırma
    pan_cmd, tilt_cmd = fcs.update(
        [620, 340, 660, 380], 640, 360, 1100.0, 1100.0, 0.033,
        is_firing=False, dist_cm=200,
    )
    assert abs(pan_cmd) <= CONFIG.control.max_output_deg
    assert abs(tilt_cmd) <= CONFIG.control.max_output_deg

    # Kararlı takip ile kilitlenme (lock hysteresis)
    box = [630, 350, 650, 370]
    for _ in range(15):
        fcs.update(box, 640, 360, 1100.0, 1100.0, 0.033, is_firing=False, dist_cm=250)
    assert fcs.is_locked is True

    # Atış sırasında kilit korunumu
    fcs.update(box, 640, 360, 1100.0, 1100.0, 0.033, is_firing=True, dist_cm=250)
    assert fcs.is_locked is True

    # Hedef kaybı ve kilidin düşmesi
    fcs.update(None, 640, 360, 1100.0, 1100.0, 0.033)
    assert fcs.frames_since_lost > 0

    for _ in range(fcs.max_lost_frames + 2):
        fcs.update(None, 640, 360, 1100.0, 1100.0, 0.033)
    assert fcs.is_locked is False


def test_fcs_ballistic_slew_and_lead():
    fcs = FCSManager(control_mode="P")
    fx = fy = 1100.0
    box = [620, 340, 660, 380]

    # Balistik süper-elevasyon ve slew factor etkisi
    fcs.update(box, 640, 360, fx, fy, 0.033, dist_cm=300, slew_factor=0.0)
    assert fcs.prev_applied_lead_tilt == 0.0
    _, tilt_cmd = fcs.update(box, 640, 360, fx, fy, 0.033, is_firing=True, dist_cm=300, slew_factor=0.3)
    assert fcs.prev_applied_lead_tilt > 0.0
    assert tilt_cmd > 0.0

    # Hareketli hedef için Kalman lead kestirimi
    fcs = FCSManager(control_mode="P")
    for i in range(30):
        bx = 640.0 + 100.0 * 0.033 * i
        target_box = [bx - 20, 340, bx + 20, 380]
        fcs.update(target_box, 640, 360, 1411.0, 1411.0, 0.033, dist_cm=200)

    lead_time = min(0.25, max(0.03, fcs.last_valid_target_dist_m / CONFIG.ballistics.projectile_speed_mps + 0.015))
    measured_x = (target_box[0] + target_box[2]) // 2
    measured_y = (target_box[1] + target_box[3]) // 2
    assert fcs.pred_center == pytest.approx(
        [measured_x + fcs.target_velocity[0] * lead_time,
         measured_y + fcs.target_velocity[1] * lead_time],
        abs=1e-6,
    )

    # Kisa sureli (3 kare) hedef kaybinda coasting (predict_only) sayesinde yeniden yakalamada hiz sicramaz
    vel_before_dropout = float(fcs.target_velocity[0])
    for j in range(1, 4):
        fcs.update(None, 640, 360, 1411.0, 1411.0, 0.033, dist_cm=200)
    bx_reacquired = 640.0 + 100.0 * 0.033 * 33
    fcs.update([bx_reacquired - 20, 340, bx_reacquired + 20, 380], 640, 360, 1411.0, 1411.0, 0.033, dist_cm=200)
    assert fcs.target_velocity[0] == pytest.approx(vel_before_dropout, rel=0.08)
