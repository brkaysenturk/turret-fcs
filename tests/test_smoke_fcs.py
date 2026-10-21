"""
================================================================================
TEST: FCS MANAGER DUMAN VE ENTEGRASYON TESTLERI (FCS SMOKE TESTS)
================================================================================
FCSManager ana yoneticisinin calisma zamaninda hicbir beklenmedik istisna (AttributeError,
NoneType error, regression) firlatmadan hedef secimi, takip guncellemesi, devriye ve mod
gecislerini yurutup yurutmedigini test eder.
================================================================================
"""

import pytest
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.config import CONFIG, FCSState


def test_fcs_manager_initialization():
    fcs = FCSManager()
    assert hasattr(fcs, "frames_since_lost")
    assert hasattr(fcs, "PAN_LIMIT_MIN")
    assert hasattr(fcs, "PAN_LIMIT_MAX")
    assert fcs.control_mode in ["P", "RL"]


def test_fcs_manager_update_smoke():
    fcs = FCSManager(control_mode="P")
    center_x, center_y = 640, 360
    fx, fy = 1100.0, 1100.0
    dt = 0.033

    # Hedef algilandiginda guncelleme
    best_box = [620, 340, 660, 380]
    pan_cmd, tilt_cmd = fcs.update(
        best_box, center_x, center_y, fx, fy, dt,
        is_firing=False, dist_cm=200
    )

    assert isinstance(pan_cmd, float)
    assert isinstance(tilt_cmd, float)
    assert abs(pan_cmd) <= CONFIG.control.max_output_deg
    assert abs(tilt_cmd) <= CONFIG.control.max_output_deg


def test_fcs_manager_target_lost_and_recovery():
    fcs = FCSManager(control_mode="P")
    center_x, center_y = 640, 360
    fx, fy = 1100.0, 1100.0

    # Hedef kayboldugunda frames_since_lost artmali
    fcs.update(None, center_x, center_y, fx, fy, 0.033)
    assert fcs.frames_since_lost > 0


def test_fcs_sync_turret_pose():
    fcs = FCSManager()
    fcs.sync_turret_pose(15.5, -5.2)
    assert fcs.turret_pan_deg == 15.5
    assert fcs.turret_tilt_deg == -5.2


def test_fcs_angular_velocity_property():
    fcs = FCSManager()
    omega_pan, omega_tilt = fcs.target_angular_velocity_deg_s
    assert isinstance(omega_pan, float)
    assert isinstance(omega_tilt, float)
    assert omega_pan == 0.0  # Initialized filter has 0 velocity
    assert omega_tilt == 0.0


def test_fcs_ballistic_slew_feedforward():
    fcs = FCSManager(control_mode="P")
    center_x, center_y = 640, 360
    fx, fy = 1100.0, 1100.0
    dt = 0.033

    best_box = [620, 340, 660, 380]

    # Step 1: Normal boresight tracking (slew_factor = 0.0)
    pan_cmd1, tilt_cmd1 = fcs.update(
        best_box, center_x, center_y, fx, fy, dt,
        is_firing=False, dist_cm=300, slew_factor=0.0
    )
    assert fcs.prev_applied_lead_tilt == 0.0

    # Step 2: Spin-up baslasin, slew_factor 0.0 -> 0.3 yükselsin
    pan_cmd2, tilt_cmd2 = fcs.update(
        best_box, center_x, center_y, fx, fy, dt,
        is_firing=True, dist_cm=300, slew_factor=0.3
    )
    # Ileri besleme devrede: prev_applied_lead_tilt 0'dan buyuk olmali
    assert fcs.prev_applied_lead_tilt > 0.0
    # Tilt komutu yukari dogru (pozitif) hareket uretmeli
    assert tilt_cmd2 > 0.0


def test_fcs_lock_evaluation_after_physical_firing():
    fcs = FCSManager(control_mode="P")
    center_x, center_y = 640, 360
    fx, fy = 1100.0, 1100.0
    dt = 0.033

    # Hedefi optik merkezde sabit tutarak kilit olustur (stability_threshold adim)
    best_box = [630, 350, 650, 370]
    for _ in range(15):
        fcs.update(best_box, center_x, center_y, fx, fy, dt, is_firing=False, dist_cm=250)

    assert fcs.is_locked is True

    # Fiziksel atis sirasinda (is_firing=True) kilit donmali
    fcs.update(best_box, center_x, center_y, fx, fy, dt, is_firing=True, dist_cm=250)
    assert fcs.is_locked is True

    # EVAL asamasinda (is_firing=False): Hedef kayboldugunda veya uzaklastiginda kilit donmamali, dinamik guncellenmeli
    for _ in range(fcs.max_lost_frames + 2):
        fcs.update(None, center_x, center_y, fx, fy, dt, is_firing=False)

    assert fcs.is_locked is False


