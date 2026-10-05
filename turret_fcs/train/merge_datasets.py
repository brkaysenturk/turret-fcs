import shutil
from pathlib import Path


def merge_datasets():
    base_dir = Path(__file__).resolve().parents[2]
    old_dir = base_dir / "datasets" / "baloon_old"
    new_dir = base_dir / "datasets" / "baloon"
    out_dir = base_dir / "datasets" / "baloon_merged"

    if out_dir.exists():
        shutil.rmtree(out_dir)

    for split in ["train", "valid"]:
        (out_dir / split / "images").mkdir(parents=True, exist_ok=True)
        (out_dir / split / "labels").mkdir(parents=True, exist_ok=True)

    print("[MERGE] 1. Yeni gercek kamera veri seti kopyalaniyor...")
    for split in ["train", "valid"]:
        img_dir = new_dir / split / "images"
        lbl_dir = new_dir / split / "labels"
        if not img_dir.exists():
            continue
        for img_p in img_dir.glob("*"):
            if img_p.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                dst_img = out_dir / split / "images" / f"real_{img_p.name}"
                shutil.copy2(img_p, dst_img)

                lbl_p = lbl_dir / f"{img_p.stem}.txt"
                dst_lbl = out_dir / split / "labels" / f"real_{img_p.stem}.txt"
                if lbl_p.exists():
                    shutil.copy2(lbl_p, dst_lbl)
                else:
                    dst_lbl.touch()

    print("[MERGE] 2. Eski Roboflow veri seti kopyalaniyor ve siniflar (cls=0) birlestiriliyor...")
    for split in ["train", "valid"]:
        img_dir = old_dir / split / "images"
        lbl_dir = old_dir / split / "labels"
        if not img_dir.exists():
            continue
        for img_p in img_dir.glob("*"):
            if img_p.suffix.lower() in [".jpg", ".jpeg", ".png"]:
                dst_img = out_dir / split / "images" / f"old_{img_p.name}"
                shutil.copy2(img_p, dst_img)

                lbl_p = lbl_dir / f"{img_p.stem}.txt"
                dst_lbl = out_dir / split / "labels" / f"old_{img_p.stem}.txt"
                if lbl_p.exists():
                    with open(lbl_p, "r", encoding="utf-8") as f:
                        lines = f.readlines()
                    remapped_lines = []
                    for line in lines:
                        parts = line.strip().split()
                        if len(parts) >= 5:
                            # Sinif ID'sini 0 ('balloon') yap
                            remapped_lines.append(f"0 {' '.join(parts[1:])}\n")
                    with open(dst_lbl, "w", encoding="utf-8") as f:
                        f.writelines(remapped_lines)
                else:
                    dst_lbl.touch()

    # data.yaml olustur
    yaml_content = f"""path: {out_dir.as_posix()}
train: train/images
val: valid/images

nc: 1
names: ['balloon']
"""
    with open(out_dir / "data.yaml", "w", encoding="utf-8") as f:
        f.write(yaml_content)

    print("\n[OK] Veri setleri basariyla birlestirildi:")
    for split in ["train", "valid"]:
        n_img = len(list((out_dir / split / "images").glob("*")))
        n_lbl = len(list((out_dir / split / "labels").glob("*.txt")))
        print(f"  - {split}: {n_img} gorsel, {n_lbl} etiket dosyasi")


if __name__ == "__main__":
    merge_datasets()
