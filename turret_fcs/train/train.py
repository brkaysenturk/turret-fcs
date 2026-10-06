from pathlib import Path
from ultralytics import YOLO


def main():
    base_dir = Path(__file__).resolve().parents[2]
    yaml_path = base_dir / "datasets" / "balloon" / "data.yaml"
    
    # Egitim baslangic agirligi: Temel yolov8n uzerinden temiz egitim
    weights_path = base_dir / "weights" / "yolov8n.pt"

    model = YOLO(str(weights_path))

    # RTX 5060 Ti 16GB VRAM icin optimize edilmis egitim parametreleri
    model.train(
        data=str(yaml_path),
        epochs=70,             # 70 epoch (yaklasik 2-3 dakika)
        patience=15,           # 15 epoch boyunca iyilesme olmazsa erken durdur
        imgsz=640,             # 640x640 optimum hiz/dogruluk orani
        batch=32,              # 16GB VRAM icin rahat ve stabil gradyan boyutu
        device=0,              # RTX 5060 Ti (CUDA)
        workers=4,             # 6-Core CPU icin optimum veri yukleme hatti
        
        # Veri Artirma (Augmentation)
        hsv_h=0.015,           # Ton esnekligi
        hsv_s=0.7,             # Doygunluk esnekligi
        hsv_v=0.6,             # Parlaklik esnekligi (los/karanlik ve parlak isik icin)
        scale=0.6,             # Yakin/uzak mesafe olceklemesi
        translate=0.15,        # Konum kaydirma
        fliplr=0.5,            # Yatay aynalama
        mosaic=1.0,            # 4 kareyi birlestirerek kucuk hedef tespitini guclendirme
        name="balloon_detector",
        exist_ok=True
    )


if __name__ == "__main__":
    main()