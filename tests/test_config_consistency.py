"""
================================================================================
TEST: KONFIGURASYON VE SSOT TUTARLILIK TESTLERI (CONFIG CONSISTENCY TESTS)
================================================================================
Tum alt sistemlerin (PController, LockManager, RLController, TurretEnv)
parametrelerinin tek gercek kaynak olan CONFIG modulu ile tam uyumlu oldugunu dogrular.
================================================================================
"""

import math
import pytest
from turret_fcs.config import CONFIG, FCSState
from turret_fcs.p_controller import PController
from turret_fcs.lock_manager import LockManager
from turret_fcs.rl_controller import RLController
from turret_fcs.rl_sim.turret_env import TurretEnv


def test_p_controller_config_consistency():
    p_ctrl = PController()
    assert math.isclose(p_ctrl.kp_frame_pan, CONFIG.control.kp_pan, abs_tol=1e-5)
    assert math.isclose(p_ctrl.kp_frame_tilt, CONFIG.control.kp_tilt, abs_tol=1e-5)
    assert math.isclose(p_ctrl.deadband_pan, CONFIG.control.deadband_pan_deg, abs_tol=1e-5)
    assert math.isclose(p_ctrl.deadband_tilt, CONFIG.control.deadband_tilt_deg, abs_tol=1e-5)
    assert math.isclose(p_ctrl.max_output, CONFIG.control.max_output_deg, abs_tol=1e-5)
    assert math.isclose(p_ctrl.alpha, CONFIG.control.alpha_filter, abs_tol=1e-5)


def test_lock_manager_config_consistency():
    lm = LockManager()
    assert lm.lock_threshold_frames == CONFIG.lock.lock_threshold_frames
    assert math.isclose(lm.lock_exit_margin_deg, CONFIG.lock.lock_exit_margin_deg, abs_tol=1e-5)


def test_actuator_config_consistency():
    rl_ctrl = RLController(auto_load=False)
    env = TurretEnv(enable_randomization=False)

    assert math.isclose(rl_ctrl.max_step_pan_deg, CONFIG.actuator.max_step_pan_deg, abs_tol=1e-5)
    assert math.isclose(rl_ctrl.max_step_tilt_deg, CONFIG.actuator.max_step_tilt_deg, abs_tol=1e-5)
    assert math.isclose(env.max_step_pan_deg, CONFIG.actuator.max_step_pan_deg, abs_tol=1e-5)
    assert math.isclose(env.max_step_tilt_deg, CONFIG.actuator.max_step_tilt_deg, abs_tol=1e-5)
    assert math.isclose(env.servo_deadband, CONFIG.actuator.servo_deadband_deg, abs_tol=1e-5)


def test_optics_config_computations():
    fx = CONFIG.optics.compute_fx(1280)
    fy = CONFIG.optics.compute_fy(720)
    assert fx > 1000.0
    assert fy > 1000.0
    # Logitech C270 dFOV 55 deg -> H_FOV ~48.8, V_FOV ~28.6
    assert CONFIG.optics.h_fov_deg > CONFIG.optics.v_fov_deg


def test_paths_config_consistency():
    assert CONFIG.paths.yolo_model.endswith(".pt")
    assert CONFIG.paths.fallback_yolo_model.endswith(".pt")
    assert CONFIG.paths.rl_model.endswith(".zip")
