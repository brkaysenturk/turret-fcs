# Model Weights (Model Ağırlıkları)

Bu dizin, Turret FCS sistemi tarafından kullanılan yapay zeka ve bilgisayarlı görü modellerini barındırır:

- **`best.pt`**: Özel veri setiyle (hava hedefleri, mikro-İHA ve balonlar) ince ayar yapılmış YOLOv8 modeli.
- **`yolov8n.pt`**: Ultralytics YOLOv8 Nano temel nesne tespit modeli (fallback).

## Depolama ve İndirme Stratejisi

Git reposunun çalışma ağacı boyutunu küçük tutmak için büyük ikili `.pt` dosyaları aktif çalışma ağacında (HEAD) izlenmez. Bu dosyalar GitHub Releases varlıkları üzerinden dağıtılır ve SHA256 ile doğrulanır:

1. **Otomatik İndirme:** `turret_fcs` çalıştırıldığında dosya yerel diskte yoksa [turret_fcs/weights.py](../turret_fcs/weights.py) üzerinden otomatik olarak en son GitHub Release asset'inden indirilir ve SHA256 doğrulaması yapılır.
2. **Komut Satırından İndirme:**
   ```bash
   python -m turret_fcs.weights
   ```
3. **Manuel İndirme:** [GitHub Releases](https://github.com/brkaysenturk/turret-fcs/releases) sayfasındaki en son sürümden `best.pt` ve `yolov8n.pt` dosyalarını indirip bu `weights/` dizinine yerleştirebilirsiniz.
