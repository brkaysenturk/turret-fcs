"""SSOT configuration consistency: parametreler ve dosya yollari."""
import math

from turret_fcs.config import CONFIG
from turret_fcs.p_controller import PController
from turret_fcs.lock_manager import LockManager
from turret_fcs.kalman_filter import KalmanFilter
from turret_fcs.rl_controller import RLController
from turret_fcs.rl_sim.turret_env import TurretEnv


def test_config_ssot_and_component_defaults():
    """Tum alt bilesenlerin (P, RL, Kalman, Lock, Env, Optik) CONFIG SSOT degerlerini dogru okudugunu dogrular."""
    p = PController()
    lm = LockManager()
    kf = KalmanFilter()
    rl = RLController(auto_load=False)
    env = TurretEnv(enable_randomization=False)

    # 1. P Kontrolcu SSOT
    assert math.isclose(p.kp_frame_pan, CONFIG.control.kp_pan, abs_tol=1e-5)
    assert math.isclose(p.kp_frame_tilt, CONFIG.control.kp_tilt, abs_tol=1e-5)
    assert math.isclose(p.deadband_pan, CONFIG.control.deadband_pan_deg, abs_tol=1e-5)
    assert math.isclose(p.deadband_tilt, CONFIG.control.deadband_tilt_deg, abs_tol=1e-5)
    assert math.isclose(p.max_output, CONFIG.control.max_output_deg, abs_tol=1e-5)
    assert math.isclose(p.alpha, CONFIG.control.alpha_filter, abs_tol=1e-5)
    assert math.isclose(p.cmd_accel_pan_deg_s2, CONFIG.control.cmd_accel_pan_deg_s2, abs_tol=1e-6)
    assert math.isclose(p.cmd_accel_tilt_deg_s2, CONFIG.control.cmd_accel_tilt_deg_s2, abs_tol=1e-6)

    # 2. LockManager & Kalman SSOT
    assert lm.lock_threshold_frames == CONFIG.lock.lock_threshold_frames
    assert math.isclose(lm.lock_exit_margin_deg, CONFIG.lock.lock_exit_margin_deg, abs_tol=1e-5)
    assert math.isclose(kf.sigma_a, CONFIG.kalman.sigma_a, abs_tol=1e-5)
    assert math.isclose(kf.outlier_jump_px, CONFIG.kalman.outlier_jump_px, abs_tol=1e-5)
    assert kf.reacquire_frames == CONFIG.kalman.reacquire_frames
    assert math.isclose(CONFIG.kalman.outlier_jump_px, 150.0, abs_tol=1e-5)
    assert CONFIG.kalman.reacquire_frames == 3
    assert math.isclose(CONFIG.control.cmd_accel_pan_deg_s2, 353.1, abs_tol=1e-5)
    assert math.isclose(CONFIG.control.cmd_accel_tilt_deg_s2, 213.6, abs_tol=1e-5)
    assert math.isclose(CONFIG.control.ego_tau_s, CONFIG.mount.servo_tau, abs_tol=1e-5)
    assert math.isclose(CONFIG.control.lead_time_max_s, 0.40, abs_tol=1e-5)

    # 3. RL Kontrolcu & Egitim Ortami SSOT
    assert rl.soft_deadband_enabled == CONFIG.rl.soft_deadband_enabled
    assert math.isclose(rl.reversal_damping, CONFIG.rl.reversal_damping, abs_tol=1e-6)
    assert math.isclose(rl.taper_pan_deg, CONFIG.rl.taper_pan_deg, abs_tol=1e-6)
    assert math.isclose(rl.taper_tilt_deg, CONFIG.rl.taper_tilt_deg, abs_tol=1e-6)
    assert math.isclose(rl.cmd_accel_pan_deg_s2, CONFIG.control.cmd_accel_pan_deg_s2, abs_tol=1e-6)
    assert math.isclose(rl.cmd_accel_tilt_deg_s2, CONFIG.control.cmd_accel_tilt_deg_s2, abs_tol=1e-6)
    assert math.isclose(rl.v_max_px_s, CONFIG.rl.v_max_px_s, abs_tol=1e-6)
    assert math.isclose(rl.dist_min_m, CONFIG.rl.dist_min_m, abs_tol=1e-6)
    assert math.isclose(rl.dist_max_m, CONFIG.rl.dist_max_m, abs_tol=1e-6)
    assert math.isclose(rl.deadband_pan_deg, CONFIG.control.deadband_pan_deg, abs_tol=1e-6)

    for obj in (rl, env):
        assert math.isclose(obj.max_step_pan_deg, CONFIG.actuator.max_step_pan_deg, abs_tol=1e-5)
        assert math.isclose(obj.max_step_tilt_deg, CONFIG.actuator.max_step_tilt_deg, abs_tol=1e-5)
    assert math.isclose(env.d_err_scale, CONFIG.rl.d_err_scale, abs_tol=1e-6)
    assert math.isclose(env.d_err_ema_alpha, CONFIG.rl.d_err_ema_alpha, abs_tol=1e-6)
    assert math.isclose(env.v_max_px_s, CONFIG.rl.v_max_px_s, abs_tol=1e-6)
    assert math.isclose(env.dist_min_m, CONFIG.rl.dist_min_m, abs_tol=1e-6)
    assert math.isclose(env.dist_max_m, CONFIG.rl.dist_max_m, abs_tol=1e-6)
    assert math.isclose(env.yolo_noise_std, CONFIG.rl.yolo_noise_std, abs_tol=1e-6)
    assert env.max_steps == CONFIG.rl.episode_steps
    assert math.isclose(env.servo_deadband, CONFIG.actuator.servo_deadband_deg, abs_tol=1e-5)
    assert math.isclose(env.backlash_deg, CONFIG.actuator.backlash_deg, abs_tol=1e-5)
    assert math.isclose(env.lambda_u, CONFIG.rl.lambda_u, abs_tol=1e-6)
    assert math.isclose(env.mu_du, CONFIG.rl.mu_du, abs_tol=1e-6)
    assert math.isclose(env.progress_gain, CONFIG.rl.progress_gain, abs_tol=1e-6)
    assert math.isclose(env.error_gain, CONFIG.rl.error_gain, abs_tol=1e-6)

    # 4. Optik ve Model Yollari
    assert CONFIG.optics.compute_fx(1280) > 1000.0
    assert CONFIG.optics.compute_fy(720) > 1000.0
    assert CONFIG.optics.h_fov_deg > CONFIG.optics.v_fov_deg
    assert CONFIG.paths.yolo_model.endswith(".pt")
    assert CONFIG.paths.fallback_yolo_model.endswith(".pt")


def test_eval_display_duration_is_shared():
    """Webots EVAL gosterim suresi CONFIG'ten okunmali."""
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    webots_main = root / "webots" / "controllers" / "webots_main"
    if str(webots_main) not in sys.path:
        sys.path.insert(0, str(webots_main))
    import webots_virtual_ballistics as vball

    evaluator = vball.VirtualBallisticsEvaluator(bullet_speed=10.0)
    assert math.isclose(
        evaluator.DISPLAY_DURATION,
        CONFIG.ballistics.eval_display_time_s,
        abs_tol=1e-6,
    )


def test_every_config_field_is_read():
    """SSOT: her CONFIG alani uretim kodunda en az bir kez okunmali."""
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    sources = []
    for pattern in ("turret_fcs/**/*.py", "webots/controllers/**/*.py", "analysis/**/*.py"):
        for path in root.glob(pattern):
            if path.name == "config.py":
                continue
            sources.append(path.read_text(encoding="utf-8", errors="ignore"))
    blob = "\n".join(sources)

    cfg_text = (root / "turret_fcs" / "config.py").read_text(encoding="utf-8")
    unused = []
    section = None
    for line in cfg_text.splitlines():
        match = re.match(r"class (\w+)Config", line)
        if match:
            section = match.group(1).lower()
            continue
        field = re.match(r"\s+(\w+):\s", line)
        if field and section:
            name = field.group(1)
            if not re.search(r"\." + re.escape(name) + r"\b", blob):
                unused.append(f"{section}.{name}")
    assert unused == [], f"unused CONFIG fields: {unused}"
