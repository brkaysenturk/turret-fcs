"""
Webots-only tuning parametreleri.

Bu modul uretim config'ini degistirmez; sadece Webots controller'inda
P ve RL kazanclarini / sarmalayici parametrelerini gecici olarak uygular.
`webots/tuning_params.json` veya `WEBOTS_TUNING_JSON` tarama araclariyla guncellenir.
"""

import json
import os
from pathlib import Path

from turret_fcs.config import CONFIG

DEFAULTS = {
    "label": "baseline",
    # P parametreleri
    "kp_pan": CONFIG.control.kp_pan,
    "kp_tilt": CONFIG.control.kp_tilt,
    "max_output_deg": CONFIG.control.max_output_deg,
    "alpha_filter": CONFIG.control.alpha_filter,
    "cmd_accel_pan_deg_s2": CONFIG.control.cmd_accel_pan_deg_s2,
    "cmd_accel_tilt_deg_s2": CONFIG.control.cmd_accel_tilt_deg_s2,
    "in_flight_damping": CONFIG.control.in_flight_damping,
    "deadband_pan_deg": CONFIG.control.deadband_pan_deg,
    "deadband_tilt_deg": CONFIG.control.deadband_tilt_deg,
    # RL spesifik parametreleri
    "rl_deadband_pan_deg": CONFIG.rl.deadband_pan_deg,
    "rl_deadband_tilt_deg": CONFIG.rl.deadband_tilt_deg,
    "reversal_damping": CONFIG.rl.reversal_damping,
    "taper_pan_deg": CONFIG.rl.taper_pan_deg,
    "taper_tilt_deg": CONFIG.rl.taper_tilt_deg,
    "d_err_ema_alpha": CONFIG.rl.d_err_ema_alpha,
}

LIMITS = {
    "kp_pan": (0.05, 0.80),
    "kp_tilt": (0.02, 0.30),
    "max_output_deg": (0.50, 2.00),
    "alpha_filter": (0.50, 0.99),
    "cmd_accel_pan_deg_s2": (50.0, 1000.0),
    "cmd_accel_tilt_deg_s2": (50.0, 1000.0),
    "in_flight_damping": (0.0, 1.20),
    "deadband_pan_deg": (0.20, 1.50),
    "deadband_tilt_deg": (0.20, 1.50),
    "rl_deadband_pan_deg": (0.20, 1.50),
    "rl_deadband_tilt_deg": (0.20, 1.50),
    "reversal_damping": (0.10, 1.00),
    "taper_pan_deg": (0.10, 3.50),
    "taper_tilt_deg": (0.10, 3.50),
    "d_err_ema_alpha": (0.10, 0.95),
}

PROFILE_LIMITS = {
    "2": {"kp_pan": (0.10, 0.32)},
    "3": {"kp_pan": (0.10, 0.25)},
    "4": {"kp_pan": (0.08, 0.15)},
    "5": {"kp_pan": (0.10, 0.27)},
    "7": {"kp_pan": (0.10, 0.32)},
    "8": {"kp_pan": (0.10, 0.35)},
}


def _profile_limits_for():
    """Aktif balon profilini okur ve profile ozel tarama sinirlarini dondurur."""
    if os.environ.get("WEBOTS_IGNORE_PROFILE_LIMITS") == "1":
        return {}
    profile = os.environ.get("WEBOTS_BALLOON_PROFILE")
    if profile is None:
        try:
            data = json.loads((Path(__file__).resolve().parents[2] / "balloon_profile.json").read_text())
            profile = str(data.get("active_profile", ""))
        except (OSError, ValueError):
            profile = ""
    return PROFILE_LIMITS.get(str(profile).strip(), {})


def _warn_if_clamped(key, requested, applied, lo, hi):
    """Kirpilan deger sessizce kaybolmasin: tarama/tez icin gorunur olsun."""
    if abs(float(requested) - float(applied)) > 1e-9:
        print(
            f"[TUNE UYARI] {key}={requested} degeri guvenli aralik "
            f"[{lo}, {hi}] disinda, {applied} olarak kirpildi.",
            flush=True,
        )


def tuning_path():
    env_path = os.environ.get("WEBOTS_TUNING_JSON")
    if env_path:
        return Path(env_path)
    return Path(__file__).resolve().parents[2] / "tuning_params.json"


def load_tuning():
    """JSON'dan tuning parametrelerini okur; eksik/bozuk alanda varsayilana doner."""
    values = dict(DEFAULTS)
    path = tuning_path()
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except (OSError, ValueError):
        raw = {}

    profile_limits = _profile_limits_for()
    for key in LIMITS:
        try:
            val = float(raw.get(key, DEFAULTS[key]))
        except (TypeError, ValueError):
            val = float(DEFAULTS[key])
        lo, hi = LIMITS[key]
        if key in profile_limits:
            p_lo, p_hi = profile_limits[key]
            lo, hi = max(lo, p_lo), min(hi, p_hi)
        values[key] = max(lo, min(hi, val))
        _warn_if_clamped(key, val, values[key], lo, hi)

    values["label"] = str(raw.get("label", DEFAULTS["label"]))
    return values


def apply_tuning(fcs, tuning):
    """Tuning degerlerini PController ve RLController'a uygular (uretim config'e yazmaz)."""
    p = fcs.p_controller
    if "kp_pan" in tuning:
        p.kp_frame_pan = float(tuning["kp_pan"])
    if "kp_tilt" in tuning:
        p.kp_frame_tilt = float(tuning["kp_tilt"])
    if "max_output_deg" in tuning:
        p.max_output = float(tuning["max_output_deg"])
    if "alpha_filter" in tuning:
        p.alpha = float(tuning["alpha_filter"])
    if "cmd_accel_pan_deg_s2" in tuning:
        p.cmd_accel_pan_deg_s2 = float(tuning["cmd_accel_pan_deg_s2"])
    if "cmd_accel_tilt_deg_s2" in tuning:
        p.cmd_accel_tilt_deg_s2 = float(tuning["cmd_accel_tilt_deg_s2"])
    if "in_flight_damping" in tuning:
        p.in_flight_damping = float(tuning["in_flight_damping"])
    if "deadband_pan_deg" in tuning:
        p.deadband_pan = float(tuning["deadband_pan_deg"])
    if "deadband_tilt_deg" in tuning:
        p.deadband_tilt = float(tuning["deadband_tilt_deg"])

    if fcs.rl_controller is not None:
        rl = fcs.rl_controller
        if "in_flight_damping" in tuning:
            rl.in_flight_damping = float(tuning["in_flight_damping"])
        if "rl_deadband_pan_deg" in tuning:
            rl.deadband_pan_deg = float(tuning["rl_deadband_pan_deg"])
        elif "deadband_pan_deg" in tuning and "rl_deadband_pan_deg" not in tuning:
            rl.deadband_pan_deg = float(tuning["deadband_pan_deg"])

        if "rl_deadband_tilt_deg" in tuning:
            rl.deadband_tilt_deg = float(tuning["rl_deadband_tilt_deg"])
        elif "deadband_tilt_deg" in tuning and "rl_deadband_tilt_deg" not in tuning:
            rl.deadband_tilt_deg = float(tuning["deadband_tilt_deg"])

        if "cmd_accel_pan_deg_s2" in tuning:
            rl.cmd_accel_pan_deg_s2 = float(tuning["cmd_accel_pan_deg_s2"])
        if "cmd_accel_tilt_deg_s2" in tuning:
            rl.cmd_accel_tilt_deg_s2 = float(tuning["cmd_accel_tilt_deg_s2"])
        if "reversal_damping" in tuning:
            rl.reversal_damping = float(tuning["reversal_damping"])
        if "taper_pan_deg" in tuning:
            rl.taper_pan_deg = float(tuning["taper_pan_deg"])
        if "taper_tilt_deg" in tuning:
            rl.taper_tilt_deg = float(tuning["taper_tilt_deg"])
        if "d_err_ema_alpha" in tuning:
            rl.d_err_ema_alpha = float(tuning["d_err_ema_alpha"])

    return tuning
