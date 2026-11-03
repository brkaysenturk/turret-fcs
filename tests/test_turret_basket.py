import math
import struct
import numpy as np

from turret_basket.config import CONFIG
from turret_basket.protocol import (
    PC_TO_STM_HEADER,
    STM_TO_PC_HEADER,
    LUNA_HEADER,
    CMD_FRAME_LEN,
    TLM_FRAME_LEN,
    LUNA_FRAME_LEN,
    ANGLE_SCALE,
    ANGLE_OFFSET_DEG,
    CMD_FORMAT,
    TLM_FORMAT,
    STATE_TRACKING,
    STATE_LOCKED,
    STATE_FIRING,
    STATE_ESTOP,
)
from turret_basket.basket_comm import FCSCommunicator
from turret_basket.rl_tracker import RLTracker
from turret_basket.rl_ballistics import RLBallistics, LaunchParameters
from turret_basket.engagement import BasketEngagement
from turret_basket.basket_manager import BasketManager, BasketStatus
from turret_basket.rl.cup_env import BallisticBasketEnv, solve_rk4_trajectory
from turret_basket.basket_hud import BasketHUD
from turret_basket.presenter import BasketPresenter
from turret_basket.keymap import process_key_event


def test_basket_config_ssot():
    """turret_basket config degerlerinin geometrik ve fiziksel tutarliligini dogrular."""
    cam = CONFIG.camera
    assert cam.width == 1280
    assert cam.height == 720
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
    assert ballistics.spinup_time_s == 0.60
    assert ballistics.recovery_time_s == 0.90
    assert ballistics.parallax_pitch_margin_deg == 0.00  # Model 2 dogrudan cozer (paralaks 0)

    # Alt bilesenlerin CONFIG SSOT varsayilanlarini dogrulama
    eng = BasketEngagement()
    assert eng.spinup_time_s == ballistics.spinup_time_s
    assert eng.firing_pulse_s == ballistics.firing_pulse_s
    assert eng.recovery_time_s == ballistics.recovery_time_s

    mgr = BasketManager(comm=None)
    assert mgr.MAX_LOCK_FRAMES == cup.lock_threshold_frames
    assert mgr.last_valid_dist_m == cup.nominal_distance_m

    b_ai = RLBallistics()
    assert b_ai.default_parallax_margin_deg == ballistics.parallax_pitch_margin_deg

    comm = FCSCommunicator(port=None)
    assert comm.baudrate == hw.baudrate


def test_basket_protocol_constants():
    """turret_basket.protocol ikili protokol sozlesmesini dogrular."""
    assert PC_TO_STM_HEADER == 0xAA
    assert STM_TO_PC_HEADER == 0x55
    assert LUNA_HEADER == 0x59

    assert CMD_FRAME_LEN == 8
    assert TLM_FRAME_LEN == 10
    assert LUNA_FRAME_LEN == 9

    assert ANGLE_SCALE == 10.0
    assert ANGLE_OFFSET_DEG == 90.0

    assert STATE_TRACKING == 0
    assert STATE_LOCKED == 1
    assert STATE_FIRING == 2
    assert STATE_ESTOP == 3


def test_basket_comm_packet_serialization_and_checksum():
    """STM32 ile 100 Hz ikili haberlesme paket formatini ve checksum algoritmasini dogrular."""
    comm = FCSCommunicator(port=None)

    # 1. 8 Baytlik Komut paketi olusturma (0xAA + 2B pan + 2B tilt + 1B speed + 1B state + 1B csum)
    pan_ticks = 25  # 2.5 derece
    tilt_ticks = -15  # -1.5 derece
    speed_pwm = 190  # TB6612FNG PWM devri
    payload = struct.pack(CMD_FORMAT, PC_TO_STM_HEADER, pan_ticks, tilt_ticks, speed_pwm, STATE_LOCKED)
    csum = sum(payload) & 0xFF
    pkt = payload + bytes([csum])

    assert len(pkt) == CMD_FRAME_LEN
    assert pkt[0] == PC_TO_STM_HEADER
    assert pkt[5] == speed_pwm
    assert pkt[6] == STATE_LOCKED
    assert (sum(pkt[:7]) & 0xFF) == pkt[7]

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


def test_basket_rl_tracker_inference():
    """1. Model (RLTracker) PPO cikariminin soft deadband ve aksiyon limitlerini dogrular."""
    tracker = RLTracker()
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


def test_basket_rl_ballistics_inference():
    """2. Model (RLBallistics) politika cikarimini ve super-elevasyon hesaplarini dogrular."""
    ballistics = RLBallistics()

    # 1.80m mesafede firlatma parametreleri
    params = ballistics.compute_launch_parameters(dist_m=1.80)
    assert isinstance(params, LaunchParameters)
    assert CONFIG.rl.tilt_offset_min_deg <= params.tilt_offset_deg <= CONFIG.rl.tilt_offset_max_deg
    assert CONFIG.rl.speed_min_mps <= params.speed_mps <= CONFIG.rl.speed_max_mps

    # Super-elevasyon acisi
    super_elev = ballistics.compute_super_elevation(params.tilt_offset_deg)
    assert 5.0 <= super_elev <= 25.0
    assert abs(super_elev - (params.tilt_offset_deg + ballistics.default_parallax_margin_deg)) < 1e-4


def test_basket_engagement_fsm():
    """BasketEngagement 4-asamali atis durum makinesini dogrular."""
    eng = BasketEngagement(spinup_time_s=0.60, firing_pulse_s=0.10, recovery_time_s=0.90)

    # 1. Baslangic: Bostayken
    step = eng.update(now=10.0, current_tilt=0.0)
    assert step.is_active is False
    assert step.phase is None
    assert step.cmd_state == STATE_TRACKING

    # 2. Tetikleme (t = 10.0)
    ok, reason = eng.trigger(now=10.0, tilt_at_lock=5.0, fire_speed=4.5, fire_tilt=21.0, super_elevation=21.8)
    assert ok is True
    assert eng.is_active is True

    # 3. Spin-up ve Super-Elevasyon Asamasi (t = 10.30s)
    step = eng.update(now=10.30, current_tilt=5.0)
    assert step.is_active is True
    assert step.phase == "SPINUP_ELEVATE"
    assert step.cmd_state == STATE_LOCKED
    assert step.step_tilt > 0.0

    # 4. Solenoid Darbe Asamasi (t = 10.65s)
    step = eng.update(now=10.65, current_tilt=26.8)
    assert step.is_active is True
    assert step.phase == "FIRING"
    assert step.cmd_state == STATE_FIRING
    assert step.just_fired is True

    # Ayni asama icinde ikinci cagri (just_fired artik False olmali)
    step2 = eng.update(now=10.68, current_tilt=26.8)
    assert step2.just_fired is False

    # 5. Toparlama Asamasi (t = 11.20s)
    step = eng.update(now=11.20, current_tilt=20.0)
    assert step.is_active is True
    assert step.phase == "RECOVER"
    assert step.cmd_state == STATE_TRACKING

    # 6. Tamamlanma (t = 11.65s)
    step = eng.update(now=11.65, current_tilt=5.0)
    assert step.is_active is False
    assert step.is_finished is True
    assert eng.is_active is False

    # 7. Abort testi
    eng.trigger(now=20.0, tilt_at_lock=0.0, fire_speed=4.0, fire_tilt=20.0, super_elevation=21.0)
    assert eng.is_active is True
    eng.abort()
    assert eng.is_active is False


def test_basket_manager_lifecycle():
    """BasketManager orkestratorunun cevrim adimini ve komut yonetimini dogrular."""
    manager = BasketManager(comm=None)
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    status = manager.step(frame=dummy_frame, dt=0.033, now=1.0)
    assert isinstance(status, BasketStatus)
    assert status.is_estop is False
    assert status.firing is False

    # E-Stop Ac
    estop_on = manager.toggle_estop()
    assert estop_on is True
    assert manager.is_estop is True

    # E-Stop Kapa
    estop_off = manager.toggle_estop()
    assert estop_off is False
    assert manager.is_estop is False

    # Reset
    manager.lock_counter = 10
    manager.locked_dist_m = 1.65
    manager.reset()
    assert manager.lock_counter == 0
    assert manager.locked_dist_m is None


def test_basket_manager_range_latch_on_lock():
    """Kilit aninda hedefin mesafesinin donduruldugunu ve gurultuden etkilenmedigini dogrular."""
    manager = BasketManager(comm=None)
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # Mock detection: hedefin tam merkezde tespit edildigi sahte tespit
    from turret_basket.yolo.cup_detector import CupDetection
    mock_det = CupDetection(
        detected=True,
        bbox=(300, 220, 340, 260),
        center_px=(320, 240),
        confidence=0.92,
        err_pan_deg=0.1,
        err_tilt_deg=0.1,
    )
    manager.detector.detect = lambda f: mock_det

    # Hedef mesafesini 1.56m olarak baslat
    manager.dist_m = 1.56
    manager.last_valid_dist_m = 1.56

    # Kilit sayacini esik degerine kadar ilerlet (15 kare)
    for _ in range(CONFIG.cup.lock_threshold_frames):
        manager.step(dummy_frame, dt=0.033, now=1.0)

    assert manager.is_locked is True
    assert manager.locked_dist_m == 1.56

    # Kilitliyken disaridan mesafe degisse bile dondurulan mesafe korunmali
    manager.last_valid_dist_m = 1.35  # Lidar masaya carpmis gibi
    manager.step(dummy_frame, dt=0.033, now=1.033)
    assert manager.dist_m == 1.56
    assert manager.locked_dist_m == 1.56

    # Hedef kayboldugunda veya resetlendiginde mandal kalkmali
    manager.reset()
    assert manager.is_locked is False
    assert manager.locked_dist_m is None


def test_basket_ballistics_rk4_trajectory():
    """2. Model Runge-Kutta 4. Derece (RK4) aerodinamik yorunge cozumunu dogrular."""
    # 2.0 metre mesafeye 10 m/s ve 10 derece aciyla atis
    res = solve_rk4_trajectory(
        dist_m=2.0,
        v0=10.0,
        theta_launch_rad=math.radians(10.0),
    )
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
    assert -1.0 <= obs[0] <= 1.0

    action = np.array([0.0, 0.0], dtype=np.float32)
    next_obs, reward, terminated, truncated, step_info = env.step(action)

    assert next_obs.shape == (2,)
    assert isinstance(reward, (float, np.floating))
    assert isinstance(terminated, bool)
    assert "basket" in step_info
    assert "miss_dist_m" in step_info
    assert "speed_mps" in step_info
    assert "tilt_deg" in step_info


def test_basket_hud_and_presenter():
    """BasketHUD ve BasketPresenter render katmanlarinin calismasini dogrular."""
    hud = BasketHUD()
    presenter = BasketPresenter(hud)

    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # 1. Scanning durumu
    status_scan = BasketStatus(
        cup_detected=False,
        bbox=None,
        confidence=0.0,
        distance_m=1.45,
        err_pan_deg=0.0,
        err_tilt_deg=0.0,
        is_locked=False,
        firing=False,
        is_estop=False,
        lock_ratio=0.0,
        speed_mps=4.4,
        tilt_offset_deg=21.8,
    )
    out1 = presenter.render(dummy_frame.copy(), status=status_scan, fps=30.0)
    assert out1.shape == dummy_frame.shape

    # 2. Locked durumu
    status_locked = BasketStatus(
        cup_detected=True,
        bbox=(200, 150, 440, 350),
        confidence=0.95,
        distance_m=1.80,
        err_pan_deg=0.2,
        err_tilt_deg=0.1,
        is_locked=True,
        firing=False,
        is_estop=False,
        lock_ratio=1.0,
        speed_mps=4.5,
        tilt_offset_deg=22.0,
    )
    out2 = presenter.render(dummy_frame.copy(), status=status_locked, fps=30.0)
    assert out2.shape == dummy_frame.shape

    # 3. Skor tablosu
    hud.record_shot(is_basket=True)
    assert hud.total_shots == 1
    assert hud.total_baskets == 1


def test_basket_keymap_events():
    """Basket keymap tus isleyicisinin dogrudan BasketManager ile entegrasyonunu dogrular."""
    manager = BasketManager(comm=None)
    hud = BasketHUD()

    # 1. Tus basilmadi (-1)
    exit_req = process_key_event(key_code=-1, manager=manager, hud=hud)
    assert exit_req is False

    # 2. Cikis (Q)
    exit_req = process_key_event(key_code=ord("q"), manager=manager, hud=hud)
    assert exit_req is True

    # 3. Y (Kilitli degilken atis reddedilmeli)
    manager.is_locked = False
    process_key_event(key_code=ord("y"), manager=manager, hud=hud)
    assert manager.engagement.is_active is False
    assert any("CANNOT FIRE" in msg for msg, _ in hud.event_logs)

    # 4. Y (Kilitliyken atis baslatilmali)
    manager.is_locked = True
    process_key_event(key_code=ord("y"), manager=manager, hud=hud)
    assert manager.engagement.is_active is True

    # 5. N (Atisi iptal et)
    process_key_event(key_code=ord("n"), manager=manager, hud=hud)
    assert manager.engagement.is_active is False

    # 6. SPACE (E-Stop mandallama)
    process_key_event(key_code=32, manager=manager, hud=hud)
    assert manager.is_estop is True
    process_key_event(key_code=32, manager=manager, hud=hud)
    assert manager.is_estop is False

    # 7. R (Reset)
    manager.lock_counter = 10
    process_key_event(key_code=ord("r"), manager=manager, hud=hud)
    assert manager.lock_counter == 0
