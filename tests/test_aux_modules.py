"""Kilit, HUD, firmware durum kodu ve telemetri konsolide testleri."""
import csv
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import FCSState
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.lock_manager import LockManager
from turret_fcs.telemetry import TelemetryLogger
from turret_fcs.turret_hud import TurretHUD


def test_lock_hysteresis_and_firmware_states():
    """Kilit histerezisi, vurus olasiligi, atis sonrasi grace period ve HUD/durum kodlarini dogrular."""
    lm = LockManager()
    assert not lm.is_locked
    assert lm.stability_counter == 0

    # 1. Kilitlenme histerezisi (10+ kare kucuk hata)
    for _ in range(12):
        is_locked, counter, hit_prob = lm.update(angular_error_deg=0.8, target_velocity=[0.0, 0.0])
    assert is_locked
    assert counter >= lm.lock_threshold_frames
    assert hit_prob > 50.0

    # 2. Atis esnasinda ve hemen sonrasinda grace period: kilit dusmemeli
    lm.update(angular_error_deg=5.0, target_velocity=[0.0, 0.0], is_firing=True)
    assert lm.is_locked

    for _ in range(20):
        is_locked, _, _ = lm.update(angular_error_deg=3.5, target_velocity=[0.0, 0.0])
        assert is_locked, "Grace period esnasinda kilit erken dusmemeli"

    # 3. Grace suresi bitince buyuk hatada kilit dusmeli
    for _ in range(40):
        lm.update(angular_error_deg=4.0, target_velocity=[0.0, 0.0])
    assert not lm.is_locked

    # 4. HUD kutu etiketi ve STM32 protokol durum kodlari
    assert TurretHUD.get_box_label() == "TARGET"
    assert (FCSState.TRACKING, FCSState.LOCKED, FCSState.FIRING, FCSState.ESTOP) == (0, 1, 2, 3)


def test_telemetry_writes_columns(tmp_path):
    """Telemetri CSV kaydinda durum ve geometri sutunlarinin dogru yazildigini dogrular."""
    fcs = FCSManager(control_mode="P")
    logger = TelemetryLogger(output_dir=str(tmp_path), filename_prefix="test")
    for i in range(10):
        box = [630 + i, 350, 650 + i, 370]
        fcs.update(box, 640, 360, 1100.0, 1100.0, 0.033, dist_cm=200)
        logger.log_frame(
            0.033, 30, target_detected=True, best_box=box, fcs=fcs,
            pan_cmd=0.1, tilt_cmd=0.05, dist_cm=200, strength=1000,
            fcs_state=0,
        )
    logger.close()

    with open(logger.csv_path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)

    for col in (
        "box_w", "box_h", "fire_stage", "range_valid",
        "cmd_pan", "cmd_tilt", "target_detected",
    ):
        assert col in reader.fieldnames
    assert "los_pan_deg" not in reader.fieldnames
    assert "los_tilt_deg" not in reader.fieldnames
    assert "los_pan_rate_deg_s" not in reader.fieldnames
    assert "los_tilt_rate_deg_s" not in reader.fieldnames
    assert "kp_eff_pan" not in reader.fieldnames
    assert "schema_version" not in reader.fieldnames
    assert rows
    assert int(rows[-1]["box_w"]) == 20
    assert int(rows[-1]["box_h"]) == 20
