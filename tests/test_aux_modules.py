"""Kilit, HUD, firmware durum kodu ve telemetri konsolide testleri."""
import csv
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.camera_thread import CameraThread
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


def test_camera_thread_device_detection(monkeypatch):
    """CameraThread MSMF ve DirectShow bagimsiz indeksleme ve C270 secimini dogrular."""
    # 1. API fonksiyonlari hata vermeden liste donmeli
    msmf_devs = CameraThread._list_msmf_devices()
    dshow_devs = CameraThread._list_dshow_devices()
    assert isinstance(msmf_devs, list)
    assert isinstance(dshow_devs, list)

    # 2. Varsayilan tarama calismali
    idx_msmf, name_msmf = CameraThread.detect_camera(backend="msmf")
    idx_dshow, name_dshow = CameraThread.detect_camera(backend="dshow")
    assert isinstance(idx_msmf, int) and isinstance(name_msmf, str)
    assert isinstance(idx_dshow, int) and isinstance(name_dshow, str)

    # 3. Coklu kamera durumunda (laptop kamerasi 0, C270 1) C270'i secmeli
    mock_devices = ["HD User Facing Camera", "Logi C270 HD WebCam"]
    monkeypatch.setattr(CameraThread, "_list_msmf_devices", staticmethod(lambda: mock_devices))
    monkeypatch.setattr(CameraThread, "_list_dshow_devices", staticmethod(lambda: mock_devices))

    idx, name = CameraThread.detect_camera(preferred_name="C270", backend="msmf")
    assert idx == 1
    assert "c270" in name.lower()

    # Ters sira (C270 0, Laptop kamerasi 1) durumunda C270 index 0 olmali
    mock_reversed = ["Logitech HD Webcam C270", "Integrated Camera"]
    monkeypatch.setattr(CameraThread, "_list_msmf_devices", staticmethod(lambda: mock_reversed))
    idx_rev, name_rev = CameraThread.detect_camera(preferred_name="C270", backend="msmf")
    assert idx_rev == 0
    assert "c270" in name_rev.lower()


def test_rk4_aerodynamic_drag_ballistics():
    """RK4 aerodinamik drag balistik cozumleyicisini dogrular."""
    from turret_fcs.ballistics import (
        compute_drag_constant,
        compute_rk4_super_elevation,
        compute_rk4_drop,
    )
    k = compute_drag_constant()
    assert 0.020 < k < 0.060, f"Beklenmeyen drag katsayisi: {k}"

    # 2.0 metrede drag vs vakum karsilastirmasi
    theta_rk4, t_rk4 = compute_rk4_super_elevation(2.0, v0=10.0, k=k)
    theta_vac, t_vac = compute_rk4_super_elevation(2.0, v0=10.0, k=0.0)
    assert theta_rk4 > theta_vac
    assert t_rk4 > t_vac
    assert 0.15 < (theta_rk4 - theta_vac) < 0.50

    # 3.0 metrede dikey dusus karsilastirmasi
    drop_drag, _ = compute_rk4_drop(3.0, v0=10.0, k=k)
    drop_vac, _ = compute_rk4_drop(3.0, v0=10.0, k=0.0)
    assert drop_drag > drop_vac
    assert 0.02 < (drop_drag - drop_vac) < 0.06  # 2 ila 6 cm daha fazla dusus


def test_lock_manager_trig_derivative_and_chndtr():
    """LockManager chndtr optimizasyonu ve sec^2 tam turev duzeltmesini dogrular."""
    lm = LockManager()

    # 1. chndtr ile Gauss Marcum Q / Non-central chi-sq hesabi
    p_hit = lm.compute_hit_probability(
        angular_error_deg=0.5,
        target_velocity=[100.0, 50.0],
        dist_m=2.0,
        fx=1100.0,
        fy=1100.0,
        target_pos_deg=(0.0, 0.0),
    )
    assert 0.0 <= p_hit <= 100.0
    assert p_hit > 60.0

    # 2. Tam trigonometrik turev (kamera kenarinda sec^2 duzeltmesi)
    # Merkezde (0 deg): scale = 1.0
    vx_c, vy_c, sp_c = LockManager._compute_angular_speed([200.0, 0.0], 1000.0, 1000.0, target_pos_deg=(0.0, 0.0))
    # Kenarda (+/-24 deg): scale = cos^2(24 deg) ~ 0.8346
    vx_e, vy_e, sp_e = LockManager._compute_angular_speed([200.0, 0.0], 1000.0, 1000.0, target_pos_deg=(24.0, 0.0))

    assert vx_c == pytest.approx(np.degrees(200.0 / 1000.0), abs=1e-4)
    expected_ratio = np.cos(np.radians(24.0)) ** 2
    assert (vx_e / vx_c) == pytest.approx(expected_ratio, abs=1e-4)
    assert (sp_e / sp_c) == pytest.approx(expected_ratio, abs=1e-4)


