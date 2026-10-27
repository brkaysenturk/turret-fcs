"""
Test suite for turret_fcs.weights manager: SHA256 verification and path resolution.
"""

from pathlib import Path
import pytest

from turret_fcs.weights import (
    KNOWN_WEIGHTS,
    compute_file_sha256,
    ensure_weight_file,
)


def test_weights_metadata_and_release_contracts():
    """KNOWN_WEIGHTS sozluk yapisi ve GitHub Release v2.0 sozlesmelerini dogrular."""
    import turret_fcs.weights as weights

    # 1. Bilinen agirliklar ve SHA256 ozetleri
    assert "best.pt" in KNOWN_WEIGHTS
    assert "yolov8n.pt" in KNOWN_WEIGHTS
    for name, info in KNOWN_WEIGHTS.items():
        assert "sha256" in info
        assert len(info["sha256"]) == 64
        int(info["sha256"], 16)  # gecerli hex olmali

    # 2. Release surumu ve indirme URL sablonu
    assert weights.DEFAULT_RELEASE_TAG == "v2.0"
    assert not hasattr(weights, "LEGACY_RELEASE_TAG")
    assert weights.RELEASE_URL_TEMPLATE.format(
        owner=weights.REPO_OWNER,
        repo=weights.REPO_NAME,
        tag=weights.DEFAULT_RELEASE_TAG,
        filename="best.pt",
    ).endswith("/releases/download/v2.0/best.pt")


def test_weights_resolution_and_hashing(tmp_path: Path):
    """SHA256 ozetleme, mevcut agirlik dosyasini dondurme ve kayip dosyada hata firlatmayi dogrular."""
    import hashlib

    # 1. SHA256 ozet dogrulugu
    sample_file = tmp_path / "sample.bin"
    sample_file.write_bytes(b"Turret FCS SHA256 Test")
    expected = hashlib.sha256(b"Turret FCS SHA256 Test").hexdigest()
    assert compute_file_sha256(sample_file) == expected

    # 2. Mevcut dosyanin dogrudan dondurulmesi
    dummy_weight = tmp_path / "custom_model.pt"
    dummy_weight.write_bytes(b"0" * 2000)
    assert ensure_weight_file(dummy_weight, required=True) == dummy_weight

    # 3. Bulunamayan dosyada FileNotFoundError firlatilmasi
    non_existent = tmp_path / "unknown_model_12345.pt"
    with pytest.raises(FileNotFoundError):
        ensure_weight_file(non_existent, required=True)
