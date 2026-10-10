"""
================================================================================
MODUL: HEDEF SECICI VE ONCELIKLENDIRME MOTORU (TARGET SELECTOR)
================================================================================
Bu modul, kameradan gelen coklu nesne tespitleri (YOLOv8 Bounding Boxes) arasindan
en oncelikli ve vurulabilir birincil hedefi (Primary Threat) secer.

ELEME VE PUANLAMA KRITERLERI:
1. Guvenilirlik Esigi (Confidence Threshold): conf >= 0.45 altindaki sahte tespitler elenir.
2. Geometrik Alan ve Boyut Filtresi: Cok kucuk (area < 400 px^2) gurultuler ve asiri buyuk
   yanlis tespitler (area > 400.000 px^2) elenir.
3. En Boy Orani (Aspect Ratio): 0.25 - 4.0 arasindaki gecerli sekiller kabul edilir.
4. Nisangah Mesafesi (Crosshair Distance): Nisangah merkezine (reticle center) en yakin
   olan hedef en yuksek oncelikle secilir.
================================================================================
"""


class TargetSelector:
    """
    YOLO tespitlerinden en uygun hedef sinir kutusunu guvene, alana, aspect orana ve 
    nisangaha olan mesafeye gore secen sinif.
    """

    def __init__(self, min_conf=0.45, min_area=400, max_area=400000, min_dim=18):
        self.min_conf = min_conf
        self.min_area = min_area
        self.max_area = max_area
        self.min_dim = min_dim

    def select_best_target(self, boxes, center_x, center_y):
        if len(boxes) == 0:
            return None

        best_box = None
        min_dist_sq = float("inf")

        for box in boxes:
            try:
                conf = float(box.conf[0].cpu().numpy()) if hasattr(box.conf[0], "cpu") else float(box.conf[0])
            except Exception:
                conf = float(box.conf) if hasattr(box, "conf") else 0.0

            if conf < self.min_conf:
                continue

            try:
                raw_coords = box.xyxy[0].cpu().numpy() if hasattr(box.xyxy[0], "cpu") else box.xyxy[0]
                x1, y1, x2, y2 = map(int, raw_coords)
            except Exception:
                continue

            bw = x2 - x1
            bh = y2 - y1
            area = bw * bh

            if area < self.min_area or area > self.max_area or bw < self.min_dim or bh < self.min_dim:
                continue

            aspect_ratio = bw / float(bh)
            if not (0.35 < aspect_ratio < 2.50):
                continue

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            dist_sq = (cx - center_x) ** 2 + (cy - center_y) ** 2

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_box = (x1, y1, x2, y2)

        return best_box