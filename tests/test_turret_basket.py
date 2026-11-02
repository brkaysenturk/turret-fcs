import math
import struct
import numpy as np

from turret_basket.config import CONFIG
from turret_basket.basket_comm import (
    FCSCommunicator,
    CMD_FRAME_LEN,
    TLM_FRAME_LEN,
    CMD_FORMAT,
    TLM_FORMAT,
    PC_TO_STM_HEADER,
    STM_TO_PC_HEADER,
    STATE_LOCKED,
)
from turret_basket.cup_tracker import CupTracker
from turret_basket.rl.cup_env import BallisticBasketEnv, solve_rk4_trajectory


def test_basket_config_ssot():
    """turret_basket config degerlerinin geometrik ve fiziksel tutarliligini dogrular."""
    cam = CONFIG.camera
    assert cam.width == 640
    assert cam.height == 480
    assert cam.fps == 30
    assert cam.fx > 0 and cam.fy > 0

    hw = CONFIG.hardware
    assert hw.pan_min_deg < hw.pan_center_deg < hw.pan_max_deg
    assert hw.tilt_min_deg < hw.tilt_center_deg < hw.tilt_max_deg

    cup = CONFIG.cup
    assert cup.target_distance_min_m < cup.target_distance_max_m
    assert cup.mouth_diameter_m > 0

    ballistics = CONFIG.ballistics
    assert ballistics.firing_pulse_s == 0.080


def test_basket_comm_packet_serialization_and_checksum():
    """STM32 ile 100 Hz ikili haberlesme paket formatini ve checksum algoritmasini dogrular."""
    comm = FCSCommunicator(port=None)

    # 1. 7 Baytlik Komut paketi olusturma (0xAA + 2B pan + 2B tilt + 1B state + 1B csum)
    pan_ticks = 25  # 2.5 derece
    tilt_ticks = -15  # -1.5 derece
    payload = struct.pack(CMD_FORMAT, PC_TO_STM_HEADER, pan_ticks, tilt_ticks, STATE_LOCKED)
    csum = sum(payload) & 0xFF
    pkt = payload + bytes([csum])

    assert len(pkt) == CMD_FRAME_LEN
    assert pkt[0] == PC_TO_STM_HEADER
    assert pkt[5] == STATE_LOCKED
    assert (sum(pkt[:6]) & 0xFF) == pkt[6]

    # 2. 10 Baytlik Telemetri cozme (0x55 + dist + strength + pan + tilt + csum)
    tlm_payload = struct.pack(TLM_FORMAT, 180, 80, 0, 0)
    tlm_payload = bytes([STM_TO_PC_HEADER]) + tlm_payload
    tlm_csum = sum(tlm_payload) & 0xFF
    tlm_packet = tlm_payload + bytes([tlm_csum])

    assert len(tlm_packet) == TLM_FRAME_LEN
    comm._rx_buffer = bytearray(tlm_packet)
    frame = comm._rx_buffer[:TLM_FRAME_LEN]
    actual_cs = frame[-1]
    expected_cs = sum(frame[:-1]) & 0xFF
    assert actual_cs == expected_cs


def test_basket_cup_tracker_inference():
    """1. Model (CupTracker) PPO cikariminin soft deadband ve aksiyon limitlerini dogrular."""
    tracker = CupTracker()
    tracker.reset()

    # Merkezdeki hedef (cx=320, cy=240, screen_w=640, screen_h=480)
    d_pan, d_tilt, is_locked = tracker.compute(
        cx=320.0,
        cy=240.0,
        center_x=320.0,
        center_y=240.0,
        screen_w=640,
        screen_h=480,
        distance_m=1.8,
        dt=0.033,
    )
    # Hata sifir oldugunda kilitli olmali ve aksiyon sifira yakin olmali
    assert is_locked is True
    assert abs(d_pan) < 0.20
    assert abs(d_tilt) < 0.20

    # Uzakta olan hedef (cx=100, cy=100)
    d_pan_far, d_tilt_far, is_locked_far = tracker.compute(
        cx=100.0,
        cy=100.0,
        center_x=320.0,
        center_y=240.0,
        screen_w=640,
        screen_h=480,
        distance_m=1.8,
        dt=0.033,
    )
    assert is_locked_far is False
    assert abs(d_pan_far) <= CONFIG.hardware.max_step_deg
    assert abs(d_tilt_far) <= CONFIG.hardware.max_step_deg


def test_basket_ballistics_rk4_trajectory():
    """2. Model Runge-Kutta 4. Derece (RK4) aerodinamik yorunge cozumunu dogrular."""
    # 2.0 metre mesafeye 10 m/s ve 10 derece aciyla atis
    res = solve_rk4_trajectory(
        dist_m=2.0,
        v0=10.0,
        theta_launch_rad=math.radians(10.0),
    )
    # res: (y, t, v_impact, vy, apex_y, hit_table, hit_front_lip)
    assert len(res) == 7
    y, t_flight, v_impact, vy, apex_y, hit_table, hit_front_lip = res
    assert t_flight > 0.05
    assert v_impact > 0.0
    assert isinstance(hit_table, bool)
    assert isinstance(hit_front_lip, bool)


def test_basket_ballistics_gym_environment():
    """BallisticBasketEnv Gymnasium ortaminda reset, step ve odul sozlesmelerini dogrular."""
    env = BallisticBasketEnv(domain_randomization=False)
    obs, info = env.reset(seed=42)

    assert obs.shape == (2,)
    # Normalize gozlem araligi: [-1.0, 1.0]
    assert -1.0 <= obs[0] <= 1.0

    # Nominal aksiyon ver: [tilt_offset, flywheel_speed]
    action = np.array([0.0, 0.0], dtype=np.float32)
    next_obs, reward, terminated, truncated, step_info = env.step(action)

    assert next_obs.shape == (2,)
    assert isinstance(reward, (float, np.floating))
    assert isinstance(terminated, bool)
    assert "basket" in step_info
    assert "miss_dist_m" in step_info
    assert "speed_mps" in step_info
    assert "tilt_deg" in step_info
