"""Webots tuning: kontrolcu override, kırpma uyarısı ve profil tavanları."""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "webots" / "controllers" / "webots_main"))

from turret_fcs.fcs_manager import FCSManager
from webots_tuning import LIMITS, apply_tuning, load_tuning


def _write_tuning(tmp_path, payload):
    path = tmp_path / "tuning.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


@pytest.fixture(autouse=True)
def _no_ambient_profile(monkeypatch):
    monkeypatch.delenv("WEBOTS_BALLOON_PROFILE", raising=False)


def test_tuning_application_and_clamping(tmp_path, monkeypatch, capsys):
    # 1. Kontrolcü kazançlarının canlı güncellenmesi
    fcs = FCSManager("P")
    apply_tuning(
        fcs,
        {"label": "test", "kp_pan": 0.22, "kp_tilt": 0.11, "max_output_deg": 1.2},
    )
    assert fcs.p_controller.kp_frame_pan == pytest.approx(0.22)
    assert fcs.p_controller.kp_frame_tilt == pytest.approx(0.11)
    assert fcs.p_controller.max_output == pytest.approx(1.2)
    assert not hasattr(fcs.p_controller, "velocity_ff_gain")

    # 2. Profil tavanı aşımında otomatik kırpma ve görünür uyarı
    path = _write_tuning(tmp_path, {"kp_pan": 0.35})
    monkeypatch.setenv("WEBOTS_TUNING_JSON", str(path))
    monkeypatch.setenv("WEBOTS_BALLOON_PROFILE", "2")
    assert load_tuning()["kp_pan"] == pytest.approx(0.32)
    assert "TUNE UYARI" in capsys.readouterr().out

    # 3. Global limit tavanı kırpması
    path = _write_tuning(tmp_path, {"kp_pan": 0.90})
    monkeypatch.setenv("WEBOTS_TUNING_JSON", str(path))
    monkeypatch.setenv("WEBOTS_BALLOON_PROFILE", "0")
    assert load_tuning()["kp_pan"] == pytest.approx(LIMITS["kp_pan"][1])


def test_profile_limits(tmp_path, monkeypatch):
    path = _write_tuning(tmp_path, {"kp_pan": 0.30, "kp_tilt": 0.13})
    monkeypatch.setenv("WEBOTS_TUNING_JSON", str(path))
    expected = {"0": 0.30, "2": 0.30, "3": 0.25, "4": 0.15, "5": 0.27, "7": 0.30}
    for profile, value in expected.items():
        monkeypatch.setenv("WEBOTS_BALLOON_PROFILE", profile)
        assert load_tuning()["kp_pan"] == pytest.approx(value)
