"""
Webots-only tuning parametreleri.

Bu modul uretim config'ini degistirmez; sadece Webots controller'inda
P kazanclarini ve max_output'u gecici olarak uygular.
`webots/tuning_params.json` tarama araclariyla guncellenir.
"""

import json
import os
from pathlib import Path

from turret_fcs.config import CONFIG

DEFAULTS = {
    "label": "baseline",
    "kp_pan": CONFIG.control.kp_pan,
    "kp_tilt": CONFIG.control.kp_tilt,
    "max_output_deg": CONFIG.control.max_output_deg,
}

LIMITS = {
    "kp_pan": (0.05, 0.80),
    "kp_tilt": (0.02, 0.30),
    "max_output_deg": (0.50, 2.00),
}

# Profil bazli pan kazanc tavanlari. Webots kazanc taramasi (analysis/p_profile_sweep.py)
# su sonucu verdi: ani yon degisimi iceren profiller kazanc artisina tolerans gostermiyor.
#   standart sinus         : 0.30-0.32 kararli, 0.39+ limit cycle
#   figure-8               : 0.32'ye kadar kararli
#   hizli sinus            : 0.20-0.32 ayirt edilemez, 0.25 tavan
#   step/hold              : 0.20 bile doyumda (%25), daha da dusurulur
#   zigzag                 : 0.24-0.27 en iyi (kilit %89), 0.30+ limit cycle
PROFILE_LIMITS = {
    "2": {"kp_pan": (0.10, 0.32)},
    "3": {"kp_pan": (0.10, 0.25)},
    "4": {"kp_pan": (0.08, 0.15)},
    "5": {"kp_pan": (0.10, 0.27)},
    "7": {"kp_pan": (0.10, 0.32)},
}


def _profile_limits_for():
    """Aktif balon profilini okur ve profile ozel tarama sinirlarini dondurur."""
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
    for key in ("kp_pan", "kp_tilt", "max_output_deg"):
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
    """Tuning degerlerini PController'a uygular (uretim config'e yazmaz)."""
    p = fcs.p_controller
    kp_pan_frame = float(tuning["kp_pan"])
    kp_tilt_frame = float(tuning["kp_tilt"])

    p.kp_frame_pan = kp_pan_frame
    p.kp_frame_tilt = kp_tilt_frame
    p.max_output = float(tuning["max_output_deg"])
    return tuning
