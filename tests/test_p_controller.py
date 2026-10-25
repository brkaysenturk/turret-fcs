"""Uretim P kontrolcusu konsolide testleri."""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.p_controller import PController


def test_p_controller_wiring_and_kinematics():
    """FCSManager baglantisi, piksel-aci donusumu ve sifirlama durumunu dogrular."""
    manager = FCSManager()
    assert isinstance(manager.p_controller, PController)
    assert math.isclose(manager.p_controller.kp_frame_pan, CONFIG.control.kp_pan, abs_tol=1e-6)
    assert math.isclose(manager.p_controller.kp_frame_tilt, CONFIG.control.kp_tilt, abs_tol=1e-6)

    p = PController()
    pan, tilt = p.compute_angles(740.0, 300.0, 640.0, 360.0, 1411.0, 1411.0)
    assert pan > 0.0
    assert tilt > 0.0
    p.compute_p(pan, tilt, dt=0.03333)
    p.reset()
    assert (p.actual_pan, p.actual_tilt, p.smooth_cmd_pan, p.smooth_cmd_tilt) == (0.0, 0.0, 0.0, 0.0)


def test_p_controller_deadband_proportional_and_limits():
    """Yumusak olu bolge, saf oransal komut, doyum siniri ve feedforward yoklugunu dogrular."""
    p = PController()
    # 1. Olu bolge ici (0.50 deg): sifir hareket
    assert p.compute_p(0.50, 0.50, dt=0.03333) == (0.0, 0.0)

    # 2. Buyuk basamak: doyum sinirina kadar kirpilir (Pan 1.50, Tilt 0.75)
    limit_pan = p.max_step_for(0.03333, axis="pan")
    limit_tilt = p.max_step_for(0.03333, axis="tilt")
    u_pan, u_tilt = p.compute_p(30.0, 20.0, dt=0.03333)
    assert 0.0 < u_pan <= limit_pan
    assert 0.0 < u_tilt <= limit_tilt
    assert limit_tilt == pytest.approx(0.75, abs=0.01)

    # 3. Saf oransal komut: buyuk hata daha yuksek komut uretir (ivme rampasi sonrasi)
    p_small = PController()
    p_large = PController()
    for _ in range(15):
        u_small, _ = p_small.compute_p(3.0, 0.0, dt=0.03333)
        u_large, _ = p_large.compute_p(8.0, 0.0, dt=0.03333)
    assert u_large > u_small > 0.0


def test_p_controller_acceleration_and_dt_scaling():
    """Komut ivme rampalamasi ve kare hizina gore adim olceklemeyi dogrular."""
    dt = 0.03333
    p = PController(cmd_accel_pan_deg_s2=10.0)
    u1, _ = p.compute_p(30.0, 0.0, dt=dt)
    u2, _ = p.compute_p(30.0, 0.0, dt=dt)
    assert u1 == pytest.approx(10.0 * (dt ** 2), abs=1e-6)
    assert u2 - u1 == pytest.approx(10.0 * (dt ** 2), abs=1e-6)

    # dt olceklemesi (ivme sinirina takilmadan saf oran testi)
    p_slow = PController(cmd_accel_pan_deg_s2=2000.0)
    p_fast = PController(cmd_accel_pan_deg_s2=2000.0)
    slow, _ = p_slow.compute_p(3.0, 0.0, dt=0.03333)
    fast, _ = p_fast.compute_p(3.0, 0.0, dt=0.01667)
    assert fast == pytest.approx(slow / 2.0, rel=0.02)

    # Frame rate takibi
    for test_dt, expected in ((0.03333, 1.50), (0.0622, 1.50), (0.01667, 0.75)):
        assert PController().max_step_for(test_dt) == pytest.approx(expected, abs=0.01)

    p_30 = PController()
    p_60 = PController()
    pan_30, _ = p_30.compute_angles(740.0, 360.0, 640.0, 360.0, 1411.0, 1411.0, dt=0.03333)
    p_60.compute_angles(740.0, 360.0, 640.0, 360.0, 1411.0, 1411.0, dt=0.016665)
    pan_60, _ = p_60.compute_angles(740.0, 360.0, 640.0, 360.0, 1411.0, 1411.0, dt=0.016665)
    assert pan_60 == pytest.approx(pan_30, rel=1e-4)


def test_p_controller_target_lost_reset_prevents_wrong_direction_motion():
    """Hedef kaybinda smooth_cmd sifirlanarak zit yondeki yeni hedefe ters hareket edilmesi engellenir."""
    p = PController()
    dt = 0.03333

    # 1. Saga dogru hareket eden hedef (pan = +10.0) -> smooth_cmd_pan pozitif yuklenir
    for _ in range(10):
        u_pan, _ = p.compute_p(10.0, 0.0, dt=dt)
    assert u_pan > 0.5
    assert p.smooth_cmd_pan > 0.5

    # 2. Hedef kayboldugunda on_target_lost cagirilir
    p.on_target_lost()
    assert p.smooth_cmd_pan == 0.0
    assert p.smooth_cmd_tilt == 0.0

    # 3. Hedef simdi solda belirdi (pan = -10.0) -> Ilk adim derhal negatif olmali!
    u_new, _ = p.compute_p(-10.0, 0.0, dt=dt)
    assert u_new < 0.0, "Hedef kaybi sonrasi ilk komut hedefin oldugu yone (negatif) olmali!"

    # 4. Karsilastirma: on_target_lost cagrilmasaydi pozitif bellek nedeniyle komut ters yone devam ederdi
    p_leaky = PController()
    for _ in range(10):
        p_leaky.compute_p(10.0, 0.0, dt=dt)
    # on_target_lost cagrilmadi, dogrudan -10.0 verildi:
    u_leaky, _ = p_leaky.compute_p(-10.0, 0.0, dt=dt)
    assert u_leaky > 0.0, "Bellek sifirlanmazsa ivme siniri nedeniyle komut ters yone (pozitif) devam eder"

