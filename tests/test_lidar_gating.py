"""LiDAR menzil kapılama: sıçrama reddi, ateş dondurma, geometri ve yeniden yakalama."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from turret_fcs.fcs_manager import FCSManager


def _stable():
    fcs = FCSManager(control_mode="P")
    for _ in range(3):
        fcs._update_lidar_range(200, False, None, 1000.0, 1000.0)
    assert fcs.range_valid is True
    assert fcs.range_stable is True
    return fcs


def test_lidar_range_gating_and_freeze():
    # 1. Kararlı menzil başlatma
    fcs = _stable()

    # 2. Ani duvar sıçraması reddi
    fcs._update_lidar_range(408, False, None, 1000.0, 1000.0)
    assert fcs.range_jump is True
    assert fcs.range_valid is False
    assert fcs.last_valid_target_dist_m == 2.0

    # 3. Geometrik tutarsızlık reddi (kamera kutusu ile LiDAR menzili uyuşmazlığı)
    fcs_geom = FCSManager(control_mode="P")
    for _ in range(3):
        fcs_geom._update_lidar_range(200, False, (211, 211), 1410.0, 1412.0)
    fcs_geom._update_lidar_range(408, False, (211, 211), 1410.0, 1412.0)
    assert fcs_geom.range_geom_mismatch is True
    assert fcs_geom.range_valid is False
    assert fcs_geom.last_valid_target_dist_m == 2.0

    # 4. Atış esnasında LiDAR dondurma (flaş/duman gürültüsünden korunma)
    fcs_fire = _stable()
    fcs_fire._update_lidar_range(408, True, None, 1000.0, 1000.0)
    assert fcs_fire.last_valid_target_dist_m == 2.0
    assert fcs_fire.range_valid is True

    # 5. Atış bittikten sonra yeni mesafeyi kararlı yakalama
    for _ in range(5):
        fcs_fire._update_lidar_range(300, False, None, 1000.0, 1000.0)
    assert fcs_fire.last_valid_target_dist_m == 3.0
    assert fcs_fire.range_valid is True
