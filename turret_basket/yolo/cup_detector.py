"""
================================================================================
BASKET CHALLENGE: YOLOV8 KUPA / BARDAK TESPIT MODULU (CUP DETECTOR)
================================================================================
YOLOv8 sinif 41 ('cup') agirliklarini kullanarak sahnede kulplu kupayi tespit eden,
kutu koordinatlarindaki titresimi EMA ile pürüzsüzleştiren ve taret nişan hatalarını
hesaplayan optik tespit motoru.
================================================================================
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple
import numpy as np
from ultralytics import YOLO

from turret_basket.config import CONFIG


@dataclass
class CupDetection:
    """Tek bir kupa tespit sonucu."""
    detected: bool = False
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)  # (x1, y1, x2, y2)
    center_px: Tuple[int, int] = (0, 0)             # (cx, cy)
    confidence: float = 0.0
    err_x_norm: float = 0.0     # [-1.0, 1.0] Ekran yatay merkez sapması
    err_y_norm: float = 0.0     # [-1.0, 1.0] Ekran dikey merkez sapması
    err_pan_deg: float = 0.0    # Açısal yatay nişan hatası (derece)
    err_tilt_deg: float = 0.0   # Açısal dikey nişan hatası (derece)
    class_name: str = "cup"


class CupDetector:
    """YOLOv8 tabanli hassas kupa tespit ve filtreleme sinifi."""

    def __init__(self, model_path: Optional[str] = None):
        path = model_path or CONFIG.cup.yolo_model_path
        print(f"[VISION] YOLOv8 modeli yukleniyor: {path}")
        self.model = YOLO(path)
        self.target_classes = CONFIG.cup.target_classes
        self.conf_thresh = CONFIG.cup.confidence_threshold
        self.alpha = CONFIG.cup.ema_alpha

        # EMA Kutu Filtresi (Titreme onleyici)
        self.smooth_box = None
        self.lost_frames = 0
        self.MAX_LOST_FRAMES = 8

        # Kupa agzina kilitlenme orani (A Yontemi: Kutu tepesinin %12 alti = tam agiz elipsi)
        params_file = Path(__file__).resolve().parent / "models" / "cup_tracker_params.json"
        self.aim_mouth_ratio = 0.12
        if params_file.exists():
            try:
                with open(params_file, "r", encoding="utf-8") as f:
                    p = json.load(f)
                    self.aim_mouth_ratio = float(p.get("aim_mouth_ratio", 0.12))
            except Exception:
                pass

    def detect(self, frame_bgr: np.ndarray) -> CupDetection:
        """Kare uzerinde kupayi tespit eder ve normalize acisal hatalari doner."""
        if frame_bgr is None:
            return CupDetection()

        h, w = frame_bgr.shape[:2]
        center_x, center_y = w // 2, h // 2
        fx = CONFIG.camera.fx
        fy = CONFIG.camera.fy

        # Takip histerezisi (Kilitliyken 0.12, ararken 0.18)
        current_conf = 0.12 if self.smooth_box is not None else 0.18

        # YOLOv8 Cikarimi (verbose=False)
        results = self.model.predict(
            source=frame_bgr,
            classes=self.target_classes,
            conf=current_conf,
            verbose=False,
            imgsz=w if w in (640, 1280) else 640
        )

        best_box = None
        best_conf = 0.0
        best_cls = "cup"

        if results and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                xyxy = boxes.xyxy[i].cpu().numpy().astype(int)

                if conf > best_conf:
                    best_conf = conf
                    best_box = xyxy
                    best_cls = self.model.names.get(cls_id, "cup")

        if best_box is not None:
            self.lost_frames = 0
            if self.smooth_box is None:
                self.smooth_box = best_box.astype(float)
            else:
                # Kutu koselerine EMA uygulayarak piksellerin titresimini yok et
                self.smooth_box = self.alpha * self.smooth_box + (1.0 - self.alpha) * best_box.astype(float)

            final_box = self.smooth_box.astype(int)
            x1, y1, x2, y2 = final_box
            cx = int((x1 + x2) / 2)
            # Dartin girecegi kupa agiz seviyesi (Optuna ile optimize edilmis oran)
            cy = int(y1 + (y2 - y1) * self.aim_mouth_ratio)

            # Normalize Hatalar [-1.0, 1.0]
            err_x_norm = float((cx - center_x) / (w / 2.0))
            err_y_norm = float((cy - center_y) / (h / 2.0))

            # Açısal Hatalar (Pinhole Kamera Modeli)
            err_pan_deg = math.degrees(math.atan2(cx - center_x, fx))
            err_tilt_deg = math.degrees(math.atan2(cy - center_y, fy))

            return CupDetection(
                detected=True,
                bbox=(x1, y1, x2, y2),
                center_px=(cx, cy),
                confidence=best_conf,
                err_x_norm=err_x_norm,
                err_y_norm=err_y_norm,
                err_pan_deg=err_pan_deg,
                err_tilt_deg=err_tilt_deg,
                class_name=best_cls
            )
        else:
            self.lost_frames += 1
            if self.lost_frames > self.MAX_LOST_FRAMES:
                self.smooth_box = None

            return CupDetection(detected=False)

