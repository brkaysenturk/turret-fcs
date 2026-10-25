"""Kalman filtresi: yakinsama, ego telafisi, aykiri deger reddi ve olu bolge konsolide testleri."""
import sys
from pathlib import Path

import pytest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.kalman_filter import KalmanFilter


def _constant(kf, speed_px_s, frames, dt=0.033, x0=640.0):
    step = speed_px_s * dt
    for i in range(frames):
        kf.update((x0 + step * i, 360.0), dt=dt)


def test_kalman_convergence_lead_and_dt_normalization():
    """Kalman baslatma, sabit hiz yakinsamasi, onleme kestirimi ve degisken dt normalizasyonunu dogrular."""
    kf = KalmanFilter()
    assert not kf.initialized

    # 1. Ilk olcumle baslatma
    px, py, vx, vy = kf.update((640.0, 360.0), dt=0.033)
    assert kf.initialized
    assert (px, py, vx, vy) == pytest.approx((640.0, 360.0, 0.0, 0.0))

    # 2. Sabit hiz yakinsamasi (100 px/s)
    _constant(kf, 100.0, 61)
    assert kf.raw_vx == pytest.approx(100.0, rel=0.03)
    assert kf.smooth_vx == pytest.approx(100.0, rel=0.03)

    # 3. Hiz ve onleme hesabi
    vel_x, vel_y = kf.get_velocity()
    assert vel_x == pytest.approx(kf.smooth_vx)
    assert vel_y == pytest.approx(0.0)

    # 4. Duzensiz dt orneklemesinde zaman normalizasyonu (200 px/s)
    kf_uneven = KalmanFilter()
    elapsed = 0.0
    for dt in [0.02, 0.05, 0.03, 0.06, 0.012, 0.04] * 10:
        elapsed += dt
        kf_uneven.update((640.0 + 200.0 * elapsed, 360.0), dt=dt)
    assert kf_uneven.raw_vx == pytest.approx(200.0, rel=0.03)


def test_kalman_ego_shift_observer():
    """Tareti dondururken hedefin sahte hiz uretmesini engelleyen 1. derece eyleyici gozlemcisini dogrular."""
    kf = KalmanFilter()
    for i in range(61):
        kf.update((640.0 - 5.0 * i, 360.0), dt=0.033, ego_dx=-5.0)
    assert abs(kf.raw_vx) < 5.0
    assert kf.smooth_vx == 0.0


def test_kalman_reversal_and_deadband():
    """Ani yon degisimi ve hiz olu bolgesini dogrular."""
    # 1. Ani ters yone hareket (150 px/s -> -150 px/s)
    kf_rev = KalmanFilter()
    _constant(kf_rev, 150.0, 41)
    assert kf_rev.smooth_vx > 100.0
    for i in range(21):
        kf_rev.update((640.0 - 150.0 * 0.033 * (i + 1), 360.0), dt=0.033)
    assert kf_rev.raw_vx < -100.0
    assert kf_rev.smooth_vx < -100.0

    # 4. Hiz olu bolgesi (titresim yok etme)
    slow = KalmanFilter()
    for i in range(61):
        slow.update((640.0 + 0.5 * i, 360.0), dt=0.033)
    assert slow.smooth_vx == 0.0

    fast = KalmanFilter()
    for i in range(61):
        fast.update((640.0 + 2.0 * i, 360.0), dt=0.033)
    assert fast.smooth_vx > CONFIG.kalman.velocity_deadband_high_px_s


def test_kalman_outlier_gating_and_reacquire():
    """Tekil sahte YOLO sicramalarini saf predict-only ile reddetmeyi ve 3 karede replay ile yeniden yakalamayi dogrular."""
    kf = KalmanFilter()
    dt = 0.033
    # 1. 100 px/s sabit hizla oturma
    _constant(kf, 100.0, 41, dt=dt)
    assert kf.smooth_vx == pytest.approx(100.0, rel=0.05)
    last_x = float(kf.x[0, 0])
    vx_before = float(kf.smooth_vx)

    # 2. Tekil 300 px sahte kutu sicramasi: predict-only sayesinde hiz hic kirlenmez
    jump_x = last_x + 300.0
    kf.update((jump_x, 360.0), dt=dt)
    assert kf.last_residual > kf.outlier_jump_px
    assert kf._outlier_frames == 1
    assert kf.smooth_vx == pytest.approx(vx_before, abs=1e-6)

    # 3. Sonraki kare gercek hedefe donus
    normal_x = last_x + 100.0 * dt * 2
    kf.update((normal_x, 360.0), dt=dt)
    assert kf.last_residual <= kf.outlier_jump_px
    assert kf._outlier_frames == 0

    # 4. Ust uste 3 kare tutarli duragan sapma: yeni hedef kabul edilip hiz sifir olmali
    jump_pos = 950.0
    for frame in range(1, 4):
        px, py, vx, vy = kf.update((jump_pos, 360.0), dt=dt)
        if frame < 3:
            assert kf._outlier_frames == frame
        else:
            assert kf._outlier_frames == 0
            assert px == pytest.approx(jump_pos)
            assert vx == 0.0
            assert vy == 0.0

    # 5. Birbiriyle tutarsiz (ekranin farkli koselerinde) 3 sahte sicrama yeni hedef sayilmamali
    for erratic_x in (200.0, 1150.0, 250.0):
        kf.update((erratic_x, 360.0), dt=dt)
        assert kf._outlier_frames == 1
    kf.update((jump_pos, 360.0), dt=dt)
    assert kf._outlier_frames == 0

    # 6. Ust uste 3 kare hareketli yeni hedef sapmasi: 3. karede replay ile hiz aninda yakinsar
    moving_start = 300.0
    new_speed = -150.0
    for frame in range(1, 4):
        cand_x = moving_start + new_speed * dt * (frame - 1)
        px, py, vx, vy = kf.update((cand_x, 360.0), dt=dt)
    assert kf._outlier_frames == 0
    assert px == pytest.approx(moving_start + new_speed * dt * 2, abs=2.0)
    assert vx < -100.0


def test_kalman_dynamic_slew_innovations_never_false_reset():
    """Taretin donusu ve hedef manevrasi sirasinda 30-70 px normal inovasyonlarin
    asla sahte outlier veya takip sifirlamasi tetiklemedigini dogrular.
    (Saha telemetrisi regresyon testi: NIS kapisi her 18 karede bir sahte reset atiyordu,
    150 px esigi ise 0 false reset uretir).
    """
    kf = KalmanFilter()
    dt = 0.033
    kf.update((640.0, 360.0), dt=dt)

    # 100 kare boyunca gercek saha dinamigi: hedef hızlanıyor, donuyor, ego farkiyla
    # inovasyonlar 30-70 px arasinda dalgalaniyor
    np.random.seed(42)
    pos = 640.0
    vel = 120.0

    for i in range(100):
        vel += float(np.sin(i * 0.1) * 30.0)
        pos += vel * dt
        # 40-60 px inovasyon olusturan dinamik kamera ve YOLO kutu kaymasi
        jitter = float(np.cos(i * 0.2) * 45.0)
        m_x = pos + jitter

        kf.update((m_x, 360.0), dt=dt, ego_dx=0.0)

        # Normal manevrada residual 150 px'in altinda kalmali ve sifirlama olmamali
        assert kf.last_residual < kf.outlier_jump_px
        assert kf._outlier_frames == 0, f"Kare {i}'de sahte outlier tetiklendi (residual={kf.last_residual:.1f})"

    assert abs(kf.smooth_vx) > 0.0
