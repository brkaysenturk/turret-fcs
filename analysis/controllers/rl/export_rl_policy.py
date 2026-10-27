"""PPO politika agirliklarini SB3'ten bagimsiz NumPy artifact'ina aktarir.

Kullanim:
    python analysis/controllers/rl/export_rl_policy.py turret_fcs/rl/models/turret_ppo_v14.zip
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO


def export(model_path: Path) -> Path:
    model = PPO.load(str(model_path), device="cpu")
    policy = model.policy
    net = policy.mlp_extractor.policy_net
    weights = {
        "w1": net[0].weight.detach().cpu().numpy().astype(np.float32),
        "b1": net[0].bias.detach().cpu().numpy().astype(np.float32),
        "w2": net[2].weight.detach().cpu().numpy().astype(np.float32),
        "b2": net[2].bias.detach().cpu().numpy().astype(np.float32),
        "w3": policy.action_net.weight.detach().cpu().numpy().astype(np.float32),
        "b3": policy.action_net.bias.detach().cpu().numpy().astype(np.float32),
        "obs_dim": np.array([policy.observation_space.shape[0]], dtype=np.int32),
        "act_dim": np.array([policy.action_space.shape[0]], dtype=np.int32),
    }
    out = model_path.with_name(model_path.stem + "_policy.npz")
    np.savez_compressed(out, **weights)

    # Dogrulama: NumPy ileri gecisi SB3 deterministic cikisiyla ayni olmali.
    obs = np.random.RandomState(0).uniform(-1.0, 1.0, size=(16, weights["obs_dim"][0])).astype(np.float32)
    expected, _ = model.predict(obs, deterministic=True)
    got = np.tanh(np.tanh(obs @ weights["w1"].T + weights["b1"]) @ weights["w2"].T + weights["b2"])
    got = got @ weights["w3"].T + weights["b3"]
    got = np.clip(got, -1.0, 1.0)
    max_err = float(np.max(np.abs(got - expected)))
    print(f"[OK] {out} ({out.stat().st_size} bytes), max forward error={max_err:.3e}")
    if max_err > 1e-4:
        raise SystemExit("NumPy forward pass SB3 ile uyusmuyor")
    return out


if __name__ == "__main__":
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("turret_fcs/rl/models/rl_v2-1.zip")
    export(path)
