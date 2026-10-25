"""Hedef secici histerezisi, UART paket iletisimi ve simulasyon algoritmalari konsolide testleri."""
import struct
import threading

import numpy as np
import pytest

from turret_fcs.fcs_comm import FCSCommunicator
from turret_fcs.protocol import ANGLE_SCALE, STM_TO_PC_HEADER, TLM_FORMAT
from turret_fcs.rl_sim.turret_env import TurretEnv
from turret_fcs.target_selector import TargetSelector


class _Tensor:
    def __init__(self, value):
        self.value = value

    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self.value)


class _Box:
    def __init__(self, coords, conf=0.9, track_id=1):
        self.xyxy = [_Tensor(coords)]
        self.conf = [_Tensor(conf)]
        self.id = [_Tensor(track_id)] if track_id is not None else None


def _packet(distance_cm, strength, pan_deg, tilt_deg):
    p = int(pan_deg * ANGLE_SCALE)
    t = int(tilt_deg * ANGLE_SCALE)
    payload = struct.pack(TLM_FORMAT, distance_cm, strength, p, t)
    payload = bytes([STM_TO_PC_HEADER]) + payload
    checksum = sum(payload) & 0xFF
    return payload + bytes([checksum])


def test_target_selector_tracking_and_spatial_gating():
    """TargetSelector kisa sureli hedef kaybi histerezisi ve mekansal kapi reddini dogrular."""
    # 1. Histerezis ve kisa sureli kayip toleransi
    sel = TargetSelector(min_conf=0.28, track_min_conf=0.16, max_lost_frames=3, spatial_gate_px=250)
    first = sel.select_best_target([_Box([600, 340, 640, 380], track_id=1)], 640, 360)
    assert first == (600, 340, 640, 380)
    assert sel.min_conf == 0.16

    # 1 kare kayip: hala takip modunda kalmali
    sel.select_best_target([], 640, 360)
    assert sel.is_tracking is True
    assert sel.min_conf == 0.16

    # 4 kare kayip: arama moduna geri donmeli
    for _ in range(4):
        sel.select_best_target([], 640, 360)
    assert sel.is_tracking is False
    assert sel.min_conf == 0.28

    # 2. Mekansal kapilama: ayni ID uzakta, yeni ID son konumun yaninda ise mekansal aday secilmeli
    sel.select_best_target([_Box([600, 340, 640, 380], track_id=1)], 640, 360)
    chosen = sel.select_best_target(
        [
            _Box([980, 340, 1020, 380], track_id=1),
            _Box([610, 342, 650, 382], track_id=2),
        ],
        640,
        360,
    )
    assert chosen == (610, 342, 650, 382)
    assert sel.locked_track_id == 2


def test_uart_communication_rx_and_tx():
    """UART paket ayristirma (gurultu/parcalanma) ve eksen basina kalinti (residue) gonderimini dogrular."""
    # 1. RX paket cozumu ve bozuk veri ayiklama
    comm = object.__new__(FCSCommunicator)
    comm._rx_buffer = bytearray()
    comm.distance = 0
    comm.strength = 0
    comm.servo_pan_deg = 0.0
    comm.servo_tilt_deg = 0.0
    comm.has_servo_telemetry = False
    comm.last_valid_time = 0.0

    pkt1 = _packet(250, 1200, 100.5, 86.8)
    pkt2 = _packet(180, 900, 85.9, 92.0)

    # Parcali ve gurultulu veri
    comm._rx_buffer.extend(b"\x00\x13" + pkt1[:5])
    comm._consume_rx_buffer()
    assert comm.has_servo_telemetry is False

    # Kalan kisim ve 2. tam paket eklendiginde son gecerli paket okunmali
    comm._rx_buffer.extend(pkt1[5:] + pkt2)
    comm._consume_rx_buffer()
    assert comm.distance == 180
    assert comm.strength == 900
    assert comm.servo_pan_deg == pytest.approx(-4.1)
    assert comm.servo_tilt_deg == pytest.approx(2.0)
    assert comm.has_servo_telemetry is True

    # 2. TX eksen basina kalinti akumulasyonu
    comm.reconnect_interval = 999.0
    comm.last_reconnect_attempt = 0.0
    comm.residue_pan = 0.0
    comm.residue_tilt = 0.0

    class _Serial:
        is_open = True
        def write(self, payload):
            pass

    comm.ser = _Serial()
    comm._lock = threading.Lock()

    # 0.05 derece gonderilince 0.5 kalinti birikmeli
    comm.send(0.05, 0.0, 0)
    assert comm.residue_pan == pytest.approx(0.5)
    assert comm.residue_tilt == 0.0

    # Tekrar 0.05 gonderilince 1 tam birim tamamlanip kalinti sifirlanmali
    comm.send(0.05, 0.0, 0)
    assert comm.residue_pan == pytest.approx(0.0)
    assert comm.residue_tilt == 0.0


def test_turret_env_backlash_does_not_eat_deadband():
    """Simulasyon ortaminda mekanik disli boslugunun (backlash) olu bolgeyi bozmamasini dogrular."""
    env = TurretEnv(enable_randomization=False, backlash_deg=1.20)
    env.reset(seed=0)
    env.cmd_pan_deg = 0.0
    env.backlash_pan_out = 0.0
    env.turret_pan = 0.0
    env.actual_pan_step = 0.0
    env.last_action = np.zeros(2, dtype=np.float32)

    # Disli boslugu asilana kadar hareket cikmamali
    env.step(np.array([1.0, 0.0], dtype=np.float32))
    assert env.cmd_pan_deg < 1.20
    assert env.turret_pan == pytest.approx(0.0)

    # Bosluk tukenince gercek donus baslamali
    for _ in range(3):
        env.step(np.array([1.0, 0.0], dtype=np.float32))
    assert env.turret_pan > 0.0
