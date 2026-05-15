#!/usr/bin/env python3
"""
export_tensorrt.py — best.pt → best.engine (TensorRT FP16)

JETSON'DA çalıştırılır — TensorRT engine mimariye özgüdür.
x86/WSL'de üretilen engine Jetson'da ÇALIŞMAZ.

Kullanım:
    cd ~/ika_ws
    python3 scripts/export_tensorrt.py

    # Farklı parametrelerle:
    python3 scripts/export_tensorrt.py --model models/best.pt --imgsz 640 --workspace 4

Gereksinimler (Jetson'da pip ile kur):
    pip3 install ultralytics
    # TensorRT ve CUDA Jetson'da JetPack ile birlikte gelir
"""

import argparse
import os
import sys
import time

def parse_args():
    parser = argparse.ArgumentParser(description="YOLOv8 → TensorRT FP16 export")
    parser.add_argument("--model",     default="models/best.pt",    help="Giriş .pt dosyası")
    parser.add_argument("--out",       default="models/best.engine", help="Çıkış .engine dosyası")
    parser.add_argument("--imgsz",     type=int, default=640,        help="Giriş boyutu (kare)")
    parser.add_argument("--workspace", type=int, default=4,          help="TensorRT workspace GB")
    parser.add_argument("--fp16",      action="store_true", default=True, help="FP16 hassasiyet")
    parser.add_argument("--int8",      action="store_true", default=False, help="INT8 hassasiyet (kalibrasyon gerekir)")
    return parser.parse_args()


def main():
    args = parse_args()

    # ── Giriş dosyası kontrol ────────────────────────────────────────────────
    model_path = os.path.abspath(args.model)
    if not os.path.isfile(model_path):
        print(f"[HATA] Model bulunamadı: {model_path}")
        print("       'models/best.pt' mevcut değil — önce Git'ten veya eğitimden al.")
        sys.exit(1)

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"[export_tensorrt] Giriş  : {model_path} ({os.path.getsize(model_path)/1e6:.1f} MB)")
    print(f"[export_tensorrt] Çıkış  : {out_path}")
    print(f"[export_tensorrt] imgsz  : {args.imgsz}x{args.imgsz}")
    print(f"[export_tensorrt] FP16   : {args.fp16}")
    print(f"[export_tensorrt] INT8   : {args.int8}")
    print(f"[export_tensorrt] Workspace: {args.workspace} GB")
    print()

    # ── Ultralytics import ───────────────────────────────────────────────────
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[HATA] ultralytics kurulu değil.")
        print("       pip3 install ultralytics")
        sys.exit(1)

    # ── Export ──────────────────────────────────────────────────────────────
    print("[export_tensorrt] Export başlıyor (Jetson'da 5-15 dakika sürebilir)...")
    t0 = time.time()

    model = YOLO(model_path)
    exported = model.export(
        format="engine",
        imgsz=args.imgsz,
        half=args.fp16,
        int8=args.int8,
        workspace=args.workspace,
        verbose=True,
    )

    elapsed = time.time() - t0
    print(f"\n[export_tensorrt] Export tamamlandı ({elapsed:.0f}s)")

    # Ultralytics çıktı dosyasını bul ve istenen konuma taşı
    exported_str = str(exported)
    if os.path.isfile(exported_str) and exported_str != out_path:
        import shutil
        shutil.move(exported_str, out_path)
        print(f"[export_tensorrt] {exported_str} → {out_path}")
    elif os.path.isfile(out_path):
        pass
    else:
        # Aynı dizinde .engine oluşmuş olabilir
        candidate = model_path.replace(".pt", ".engine")
        if os.path.isfile(candidate):
            import shutil
            shutil.move(candidate, out_path)
            print(f"[export_tensorrt] {candidate} → {out_path}")
        else:
            print(f"[UYARI] Engine dosyası bulunamadı, manuel kontrol et: {exported_str}")

    if os.path.isfile(out_path):
        print(f"[export_tensorrt] ✓ {out_path} ({os.path.getsize(out_path)/1e6:.1f} MB)")
        print()
        print("Sonraki adım — node'u test et:")
        print("  ros2 run teknofest_ika yolo_detection_node")
    else:
        print("[HATA] Engine dosyası oluşturulamadı.")
        sys.exit(1)


if __name__ == "__main__":
    main()
