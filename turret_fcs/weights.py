"""
================================================================================
MODUL: MODEL AGIRLIKLARI YONETICISI (WEIGHTS MANAGER)
================================================================================
Bu modul, YOLOv8 balon tespit modeli (best.pt) ve temel nesne modeli (yolov8n.pt)
agirliklarinin varligini ve SHA256 butunlugunu kontrol eder. Agirliklar yerel diskte
bulunmadiginda, GitHub Releases varliklarindan (Release Assets) guvenli, zaman asimi
(timeout) korumali ve kriptografik hash dogrulamali olarak otomatik indirir.
================================================================================
"""

import hashlib
import sys
import urllib.request
from pathlib import Path
from typing import Optional

REPO_OWNER = "brkaysenturk"
REPO_NAME = "turret-fcs"
DEFAULT_RELEASE_TAG = "v2.0"
RELEASE_URL_TEMPLATE = "https://github.com/{owner}/{repo}/releases/download/{tag}/{filename}"
DEFAULT_TIMEOUT_SEC = 30.0

KNOWN_WEIGHTS = {
    "best.pt": {
        "desc": "Ozel egitilmis hava hedefi (balon) tespit modeli",
        "sha256": "3f842cf4384cafa1315a0b452d8024b54329c8696091b005f413b7ae8958ed93",
    },
    "yolov8n.pt": {
        "desc": "Ultralytics YOLOv8 Nano temel model",
        "sha256": "f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36",
    },
}


def compute_file_sha256(path: Path) -> str:
    """Belirtilen dosyanin SHA256 ozetini (hex string) hesaplar."""
    hasher = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            hasher.update(chunk)
    return hasher.hexdigest().lower()


def download_weight_file(
    filename: str,
    dest_path: Path,
    tag: str = DEFAULT_RELEASE_TAG,
    timeout: float = DEFAULT_TIMEOUT_SEC,
    verify_checksum: bool = True,
) -> bool:
    """GitHub Releases asset URL'sinden agirlik dosyasini guvenle indirir ve dogrular."""
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    url = RELEASE_URL_TEMPLATE.format(
        owner=REPO_OWNER,
        repo=REPO_NAME,
        tag=tag,
        filename=filename,
    )
    temp_path = dest_path.with_suffix(".tmp")
    print(f"[WEIGHTS] Agirlik dosyasi eksik: {filename}")
    print(f"[WEIGHTS] GitHub Releases'den indiriliyor: {url}")
    try:
        req = urllib.request.Request(
            url,
            headers={"User-Agent": "TurretFCS-WeightsDownloader/2.0"},
        )
        with urllib.request.urlopen(req, timeout=timeout) as response:
            total_size = int(response.headers.get("content-length", 0))
            downloaded = 0
            block_size = 65536
            with open(temp_path, "wb") as out_file:
                while True:
                    buffer = response.read(block_size)
                    if not buffer:
                        break
                    out_file.write(buffer)
                    downloaded += len(buffer)
                    if total_size > 0:
                        pct = min(100, int(downloaded * 100 / total_size))
                        mb_done = downloaded / (1024 * 1024)
                        mb_tot = total_size / (1024 * 1024)
                        sys.stdout.write(f"\r[WEIGHTS] Indiriliyor: %{pct} ({mb_done:.1f}/{mb_tot:.1f} MB)")
                        sys.stdout.flush()

        sys.stdout.write("\n")

        # SHA256 Dogrulamasi
        expected_sha = KNOWN_WEIGHTS.get(filename, {}).get("sha256")
        if verify_checksum and expected_sha:
            actual_sha = compute_file_sha256(temp_path)
            if actual_sha != expected_sha.lower():
                print("[WEIGHTS HATA] SHA256 ozet uyusmazligi! Dosya bozuk olabilir.")
                print(f"  Beklenen: {expected_sha}")
                print(f"  Alinan:   {actual_sha}")
                if temp_path.exists():
                    temp_path.unlink()
                return False

        temp_path.replace(dest_path)
        print(f"[WEIGHTS] Basariyla dogrulandi ve kaydedildi: {dest_path}")
        return True
    except Exception as e:
        if temp_path.exists():
            temp_path.unlink()
        print(f"\n[WEIGHTS HATA] Otomatik indirme basarisiz oldu: {e}")
        print(f"[WEIGHTS BILGI] Dosyayi elle indirmek icin: {url}")
        print(f"[WEIGHTS BILGI] Hedef dizin: {dest_path}")
        return False


def ensure_weight_file(
    model_path: str | Path,
    tag: str = DEFAULT_RELEASE_TAG,
    required: bool = True,
) -> Path:
    """
    Model dosyasinin varligini kontrol eder, yoksa indirir.
    Dosya indirilemez ve required=True ise FileNotFoundError firlatir.
    """
    path = Path(model_path)
    if path.exists() and path.stat().st_size > 1000:
        return path

    filename = path.name
    if filename in KNOWN_WEIGHTS:
        download_weight_file(filename, path, tag=tag)

    if not path.exists() or path.stat().st_size < 1000:
        if required:
            raise FileNotFoundError(
                f"[WEIGHTS] Zorunlu model dosyasi bulunamadi ve otomatik indirilemedi: {path}.\n"
                f"Lutfen internet baglantinizi kontrol edin veya en son surumden elle indirin: "
                f"{RELEASE_URL_TEMPLATE.format(owner=REPO_OWNER, repo=REPO_NAME, tag=tag, filename=filename)}"
            )
    return path


def ensure_all_weights(
    weights_dir: Optional[Path] = None,
    tag: str = DEFAULT_RELEASE_TAG,
) -> bool:
    """Tum bilinen agirliklarin weights/ altinda hazir olmasini saglar. Basari durumunu dondurur."""
    if weights_dir is None:
        weights_dir = Path(__file__).resolve().parents[1] / "weights"
    weights_dir.mkdir(parents=True, exist_ok=True)
    all_ok = True
    for filename, info in KNOWN_WEIGHTS.items():
        target = weights_dir / filename
        if not target.exists() or target.stat().st_size < 1000:
            ok = download_weight_file(filename, target, tag=tag)
            if not ok:
                all_ok = False
        else:
            actual_sha = compute_file_sha256(target)
            expected_sha = info.get("sha256")
            if expected_sha and actual_sha != expected_sha.lower():
                print(f"[WEIGHTS UYARI] {filename} dosyasi mevcut ancak SHA256 uyusmuyor. Yeniden indiriliyor...")
                ok = download_weight_file(filename, target, tag=tag)
                if not ok:
                    all_ok = False
            else:
                size_mb = target.stat().st_size / (1024 * 1024)
                print(f"[WEIGHTS] Mevcut ve dogrulandi: {filename} ({size_mb:.2f} MB)")
    return all_ok


if __name__ == "__main__":
    success = ensure_all_weights()
    sys.exit(0 if success else 1)
