# Changelog

All notable changes to the Turret FCS (Autonomous Air Defense Turret Fire Control System) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [v1.3] - 2026-10-21

### Added
- **Single Source of Truth (SSOT) Architecture (`turret_fcs/config.py`):**
  - Consolidated all optical, actuator, controller, kalman, lock, ballistics, and path configurations into a lightweight, pure-math leaf module.
  - Eliminated circular dependencies and heavy imports from config.
  - Added `ActuatorConfig` with exact hardware step limits (0.55 deg pan, 0.22 deg tilt) and servo deadband (0.10 deg).
- **Telemetry v2 Specification & Sidecar Metadata (`turret_fcs/telemetry.py`):**
  - Expanded telemetry schema from 25 columns to 40 columns including `schema_version`, `frame_id`, `latency_ms`, `vx_raw`, `vy_raw`, `innov_px`, `sigma_a`, `kp_eff_pan`, `kp_eff_tilt`, `db_eff_pan`, `db_eff_tilt`, `servo_pan_fb`, `servo_tilt_fb`, `sensor_temp_c`, and `lock_thr`.
  - Automatic JSON metadata sidecar (`.meta.json`) recording system configuration, git commit SHA, session timestamps, and full post-session metrics (FPS, lock ratio, tracking error median/IQR).
  - Tactical event sidecar (`_events.csv`) logging all state transitions, locks, fire authorizations, and mode switches with millisecond timestamps.
- **Redesigned Linear-Quadratic Regulator (LQR) Reward Formulation for RL:**
  - Formulated continuous state-action penalty reward: `r = -(e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2) + bullseye_bonus`.
  - Bayesian hyperparameter optimization with Optuna TPE over 20 fixed seeds using robust Median + IQR scoring.
  - Retrained 250,000 steps PPO policy (`turret_ppo_final.zip`) achieving +59.0% tracking accuracy improvement over baseline P controller in benchmark tests.
- **Automated Pytest Test Suite & Continuous Integration:**
  - Created 19 automated test cases in `tests/` covering SSOT consistency, path resolution, controller deadband behavior, Joseph form Kalman stability, FCSState firmware enum parity, aim-point slew feedforward, and full FCSManager smoke pipelines.
  - Added GitHub Actions workflow (`.github/workflows/ci.yml`) matrix-testing Python 3.10, 3.11, and 3.12.
- **Exact 2D Gaussian Hit Probability (`turret_fcs/lock_manager.py`):**
  - Closed-form Marcum Q / Non-central chi-squared CDF (`scipy.stats.ncx2.cdf`) replacing heuristic attenuation.
  - Dynamic target angular radius computed directly from YOLO bounding box dimensions (`bw, bh`) and camera focal lengths.
  - Explicit separation of projectile flight time (`t_flight = d / v_proj`) and 80 ms transport delay with realistic weapon dispersion (`sigma_weapon = 1.25 deg`).
  - Extended dynamic hysteresis lock window (1.60° - 3.00° entry, 2.20° - 3.60° exit) with 10-frame stability filter.
- **Ballistic Firing Synchronization, Slew Feedforward & Lock Grace Period:**
  - Extended flywheel spin-up to 1.0s (`spinup_time_s = 1.00`), integrated direct aim-point slew feedforward (`u = delta_slew + u_P`) eliminating steady-state ramp lag, and ensuring servo achieves full super-elevation angle before solenoid release.
  - Added 0.75s post-firing lock grace period (`post_fire_grace_duration_frames = 22`), preventing false lock drops as turret returns to boresight.
- **Combat Declutter & Encapsulated Tactical HUD (`turret_fcs/turret_hud.py`):**
  - Encapsulated status banner, footer legend, and event logs into `TurretHUD` methods following SRP.
  - Automatically hides status panels, event logs, and FPS counters during fire authorization and firing cycle.
  - English tactical post-firing engagement feedback (`ENGAGEMENT COMPLETE / ENGAGEMENT SUCCESSFUL`).
- **HUD Color Discipline Decoupling:**
  - Decoupled tactical status banner colors from aiming reticle and target bounding boxes.
  - Reticle, target bounding box, and `TARGET` label remain strictly high-contrast Red during lock and firing sequences, while only the status banner updates through stage colors.

### Changed
- Synchronized Webots digital twin camera FOV (48.8 deg horizontal, 28.6 deg vertical) directly to `CONFIG.optics`.
- Replaced all raw numerical E-stop literals (3, 0) with `FCSState.ESTOP` and `FCSState.TRACKING`.
- Aligned solenoid actuation timing across documentation, config, and firmware to 80 ms (0.08 s).
- Clarified benchmark lock duration metrics: 1,061 frames corresponds to 13.16 seconds at 80.6 FPS.

### Fixed
- Fixed broken image references in `README.md` pointing to root instead of `docs/assets/`.
- Deleted duplicate analysis graphic `analysis/system_identification_analysis.png`.
- Updated `CameraThread.read()` to support optional `return_id=True` and zero-latency hardware frame IDs.
- Connected LiDAR temperature reading `temp_c` into real-time telemetry logging instead of discarding.

---

## [v1.2.0] - 2026-10-10

### Added
- Cumulative step accumulator in patrol generator (`min_dispatch_deg = 0.95 deg`) to prevent servo deadband stalling.
- Dynamic Kalman velocity deadband expanding from 45 px/s to 120 px/s based on ego-motion speed and target proximity.
- Tracking confidence hysteresis with dynamic thresholding (0.28 acquisition, 0.16 tracking persistence).
- Asymmetric elevation damping and vertical zero-jerk EMA filter to eliminate stick-slip limit cycles under 300g barrel inertia.
- Smith predictor action memory in PPO environment for sim-to-real latency alignment.
- Consolidated telemetry inspection suite under `analysis/inspect_logs.py`.

---

## [v1.1.0] - 2026-09-20

### Added
- Type-1 Proportional control backbone with soft deadband for chattering-free servo response.
- Range gating protection filtering out background wall LiDAR reflections for errors > 3.0 deg.
- 11-dimensional action history PPO reinforcement learning agent with domain randomization.
- Reverse sliding-window binary UART parser for 115200 baud PC-STM32 telemetry.
- Multi-layer fail-safe architecture (300 ms STM32 heartbeat watchdog, 350 ms camera freeze guard, latched E-stop, 80 ms edge-triggered solenoid latch).
- Lissajous curve parametric air surveillance patrol mode (Pan: +/- 35 deg, Tilt: +/- 8 deg).
