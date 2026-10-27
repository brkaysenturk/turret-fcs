"""Closed-loop P controller benchmark on the fast TurretEnv plant.

Runs the production P controller (optionally with overridden gains) under
identical seeds, targets, latency, servo lag, quantization, and YOLO noise.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from turret_fcs.p_controller import PController
from turret_fcs.rl.turret_env import TurretEnv


def make_controller(name):
    """`default` veya `kp_pan=0.22,kp_tilt=0.11` gibi kazanc listesi."""
    if name == "default":
        return PController()
    kwargs = {}
    for item in name.split(","):
        key, _, raw = item.partition("=")
        if not raw:
            raise ValueError(f"unknown controller: {name}")
        kwargs[key.strip()] = float(raw)
    return PController(**kwargs)


def run_episode(controller_name, seed, randomize=False):
    # TurretEnv uses the global numpy RNG for scenario and target generation.
    # Seed it explicitly so every controller sees the same paired episode.
    np.random.seed(seed)
    env = TurretEnv(enable_randomization=randomize)
    env.reset(seed=seed)
    ctrl = make_controller(controller_name)

    errors = []
    delayed_errors = []
    commands = []
    for _ in range(env.max_steps):
        obs = env._get_obs()
        err_x_norm, err_y_norm = float(obs[0]), float(obs[1])
        px = env.center_x + err_x_norm * env.center_x
        py = env.center_y + err_y_norm * env.center_y
        pan_deg, tilt_deg = ctrl.compute_angles(px, py, env.center_x, env.center_y, env.fx, env.fy)
        u_pan, u_tilt = ctrl.compute_p(pan_deg, tilt_deg, dt=env.dt)

        true_px, true_py = env.last_pixel_pos
        errors.append(math.hypot(true_px - env.center_x, true_py - env.center_y))
        delayed_errors.append(math.hypot(px - env.center_x, py - env.center_y))
        commands.append((abs(u_pan), abs(u_tilt)))

        action = np.array(
            [
                np.clip(u_pan / max(1e-4, env.max_step_pan_deg), -1.0, 1.0),
                np.clip(u_tilt / max(1e-4, env.max_step_tilt_deg), -1.0, 1.0),
            ],
            dtype=np.float32,
        )
        _, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            break

    errors = np.array(errors)
    commands = np.array(commands)
    du = np.abs(np.diff(commands, axis=0)) if len(commands) > 1 else np.zeros((1, 2))
    return {
        "scenario": env.trajectory_type,
        "mean_err": float(errors.mean()),
        "rms_err": float(np.sqrt(np.mean(errors**2))),
        "p90_err": float(np.percentile(errors, 90)),
        "lock_ratio": float(np.mean(errors < 30.0)),
        "mean_u_pan": float(commands[:, 0].mean()),
        "mean_u_tilt": float(commands[:, 1].mean()),
        "jerk_pan": float(du[:, 0].mean()),
        "jerk_tilt": float(du[:, 1].mean()),
        "frames": int(errors.size),
    }


def summarize(results, label):
    keys = ("mean_err", "rms_err", "p90_err", "lock_ratio", "mean_u_pan", "mean_u_tilt",
            "jerk_pan", "jerk_tilt")
    print(f"\n{label} (n={len(results)})")
    for scenario in sorted(set(r["scenario"] for r in results)):
        rows = [r for r in results if r["scenario"] == scenario]
        vals = {k: np.mean([r[k] for r in rows]) for k in keys}
        print(
            f"  {scenario:>9}: mean={vals['mean_err']:6.1f}px rms={vals['rms_err']:6.1f} "
            f"p90={vals['p90_err']:6.1f} lock={vals['lock_ratio']*100:5.1f}% "
            f"u_pan={vals['mean_u_pan']:.3f} u_tilt={vals['mean_u_tilt']:.3f} "
            f"jerk={vals['jerk_pan']:.3f}/{vals['jerk_tilt']:.3f} n={len(rows)}"
        )
    overall = {k: np.mean([r[k] for r in results]) for k in keys}
    print(
        f"  OVERALL  : mean={overall['mean_err']:6.1f}px rms={overall['rms_err']:6.1f} "
        f"p90={overall['p90_err']:6.1f} lock={overall['lock_ratio']*100:5.1f}% "
        f"u_pan={overall['mean_u_pan']:.3f} u_tilt={overall['mean_u_tilt']:.3f} "
        f"jerk={overall['jerk_pan']:.3f}/{overall['jerk_tilt']:.3f}"
    )
    return overall


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("variants", nargs="*", default=["default"])
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--randomize", action="store_true")
    args = parser.parse_args()

    seeds = list(range(args.seeds))
    for variant in args.variants:
        results = [run_episode(variant, seed, randomize=args.randomize) for seed in seeds]
        summarize(results, variant + (" [randomized]" if args.randomize else ""))
