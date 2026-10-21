"""
================================================================================
TEST: KONTROLCU VE FILTRE BIRIM TESTLERI (CONTROLLER & FILTER UNIT TESTS)
================================================================================
PController, KalmanFilter ve LockManager modullerinin matematiksel ve mantiksal
dogrulugunu, olu bolge davranisini, filtre gecislerini ve histerezisi test eder.
================================================================================
"""

import math
import numpy as np
import pytest
from turret_fcs.p_controller import PController
from turret_fcs.kalman_filter import KalmanFilter
from turret_fcs.lock_manager import LockManager


def test_p_controller_deadband():
    ctrl = PController()
    # Olu bolge icindeki kucuk aci hatasi sifir komut uretmeli
    u_pan, u_tilt = ctrl.compute_p(0.50, 0.50, dt=0.033)
    assert u_pan == 0.0
    assert u_tilt == 0.0


def test_p_controller_large_error_clipping():
    ctrl = PController()
    # Cok buyuk aci hatasinda komut max_output sinirina doymali
    u_pan, u_tilt = ctrl.compute_p(30.0, 20.0, dt=0.033)
    assert abs(u_pan) <= ctrl.max_output
    assert abs(u_tilt) <= ctrl.max_output
    assert u_pan > 0.0
    assert u_tilt > 0.0


def test_kalman_filter_initialization_and_innovation():
    kf = KalmanFilter(dt=0.033)
    assert not kf.initialized

    # Ilk adim: Baslatma
    px, py, vx, vy = kf.update((640.0, 360.0), dt=0.033)
    assert kf.initialized
    assert math.isclose(px, 640.0, abs_tol=1e-3)
    assert math.isclose(py, 360.0, abs_tol=1e-3)

    # Saga dogru surekli hareket
    for x_pos in [670.0, 700.0, 730.0, 760.0]:
        px, py, vx, vy = kf.update((x_pos, 360.0), dt=0.033)

    assert kf.last_residual > 0.0
    assert kf.raw_vx > 0.0
    assert vx > 0.0  # Filtrelenmis hiz olu bolgeyi asip pozitif saga oturur


def test_lock_manager_hysteresis_and_counter():
    lm = LockManager()
    assert not lm.is_locked
    assert lm.stability_counter == 0

    # Hedef vizor merkezindeyken ardisik 10 kare guncelle
    for _ in range(12):
        is_locked, counter, hit_prob = lm.update(angular_error_deg=0.8, target_velocity=[0.0, 0.0])

    assert is_locked
    assert counter >= lm.lock_threshold_frames
    assert hit_prob > 50.0


def test_lock_manager_post_fire_grace_period():
    lm = LockManager()
    # 1. Once kilit sagla
    for _ in range(12):
        lm.update(angular_error_deg=0.8, target_velocity=[0.0, 0.0])
    assert lm.is_locked

    # 2. Atis dongusu baslasin (is_firing=True)
    lm.update(angular_error_deg=5.0, target_velocity=[0.0, 0.0], is_firing=True)
    assert lm.is_locked

    # 3. Atis bitti (is_firing=False). Hata buyuk (3.5 deg) olsa bile grace period (22 kare / 0.75 s) kilidi korumali
    for _ in range(20):
        is_locked, counter, _ = lm.update(angular_error_deg=3.5, target_velocity=[0.0, 0.0], is_firing=False)
        assert is_locked, "Grace period esnasinda kilit erken dusmemeli"

    # 4. Grace period bitip hata hala surerse kilit normal olarak dusmeli
    for _ in range(40):
        lm.update(angular_error_deg=4.0, target_velocity=[0.0, 0.0], is_firing=False)
    assert not lm.is_locked, "Grace period sonrasi hata devam ediyorsa kilit dusmeli"
