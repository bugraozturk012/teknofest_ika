#!/usr/bin/env python3
"""
yolo_adapter_node.py — Detection2DArray → /ika/detections JSON + /yolo/class_id köprüsü

Girdi : /detections/yolo  (vision_msgs/Detection2DArray)  — yolo_detection_node
Çıktı : /ika/detections   (std_msgs/String JSON)          — misyon_fsm
        /yolo/class_id    (std_msgs/UInt8)                — terrain_adapter

Tabela numarası → parkur aşaması (model ALFABETİK sırayla eğitildi, sequential değil):
  class_id= 0  Tabela_1       → SULU_YOL
  class_id= 1  Tabela_10      → DIK_EGIM_CIKIS
  class_id= 2  Tabela_11      → HIZLANMA başlangıcı (§6.11)
  class_id= 3  Tabela_11_son  → HIZLANMA sonu (§6.11)
  class_id= 4  Tabela_2       → TASLI_YOL
  class_id= 5  Tabela_3       → YAN_EGIM
  class_id= 6  Tabela_4       → DIK_ENGEL
  class_id= 7  Tabela_5       → KONİLİ_YOL
  class_id= 8  Tabela_6       → KAYAR_ENGEL
  class_id= 9  Tabela_7       → ENGEBELİ_ARAZİ
  class_id=10  Tabela_8       → DIK_EGIM
  class_id=11  Tabela_9       → ATIS_BOLGESI
  class_id=12  Tabela_stop    → STOP işareti (§6.10)
  class_id=13  hedef_tahtasi  → hedef_var=True
  class_id=14  trafik_huni    → koni_var=True

Notlar:
  - kayar_yon: YOLO'dan çıkarılamaz — kayar_engel_kalman node'undan /moving_obs/direction gelir,
    misyon_fsm bu topic'i ayrıca dinler (DetectionsStore.update_field).
  - bariyer_sol/sag_m: LiDAR geometrisi gerekli, şimdilik varsayılan 1.5m.
  - Aynı karede birden fazla tabela tespiti varsa en yüksek güvenilirlik olanı seçilir.
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String, UInt8

from teknofest_ika.otonomi.topics import (
    YOLO_CLASS_ID_TOPIC, DETECTIONS_TOPIC, YOLO_RAW_TOPIC,
    YOLO_CONFIDENCE_THRESHOLD,
)
from teknofest_ika.otonomi.pure_logic import ConsecutiveFrameFilter

# class_id → parkur aşama adı (sadece loglama için)
# Model alfabetik sırayla eğitildi:
#   Tabela_1, Tabela_10, Tabela_11, Tabela_11_son, Tabela_2, Tabela_3, ...
# Sıra topics.YOLO_CLASSES ile birebir aynı olmalıdır.
TABELA_SINIF = {
    0:  "SULU_YOL",        # Tabela_1
    1:  "DIK_EGIM_CIKIS",  # Tabela_10 — DIK_EGIM çıkış tabelası
    4:  "TASLI_YOL",       # Tabela_2
    5:  "YAN_EGIM",        # Tabela_3
    6:  "DIK_ENGEL",       # Tabela_4
    7:  "KONİLİ_YOL",     # Tabela_5
    8:  "KAYAR_ENGEL",     # Tabela_6
    9:  "ENGEBELİ_ARAZİ", # Tabela_7
    10: "DIK_EGIM",        # Tabela_8
    11: "ATIS_BOLGESI",    # Tabela_9
}

# class_id'ler yeni 15 sınıflı model (best.pt 2026-07-22) ile uyumlu —
# eski "Tabela_12" (index 4) kaldırıldı, index 4'ten sonrası bir kaydı.
CLASS_TABELA_11     = 2    # §6.11 hızlanma başlangıcı
CLASS_TABELA_11_SON = 3    # §6.11 hızlanma sonu
CLASS_TABELA_STOP   = 12   # §6.10 ramp STOP işareti
CLASS_HEDEF_TAHTASI = 13
CLASS_TRAFIK_HUNI   = 14
NO_DETECTION        = 255

# Yanlış pozitife karşı: STOP art arda bu kadar frame gelmeden tetiklenmez
STOP_CONSECUTIVE_FRAMES = 2

# Yanlış pozitife karşı: parkur-aşaması tabelaları (TABELA_SINIF) da art arda
# bu kadar frame aynı sınıfı vermeden FSM'e onaylı olarak iletilmez.
TABELA_CONSECUTIVE_FRAMES = 2


class YoloAdapterNode(Node):

    def __init__(self):
        super().__init__("yolo_adapter_node")

        self.declare_parameter("img_cx",         320.0)
        self.declare_parameter("img_cy",         320.0)
        self.declare_parameter("conf_threshold",   YOLO_CONFIDENCE_THRESHOLD)

        self._img_cx          = self.get_parameter("img_cx").value
        self._img_cy          = self.get_parameter("img_cy").value
        self._conf_threshold  = self.get_parameter("conf_threshold").value

        self._pub_json  = self.create_publisher(String, DETECTIONS_TOPIC,    10)
        self._pub_class = self.create_publisher(UInt8,  YOLO_CLASS_ID_TOPIC, 10)

        # Ardışık-frame doğrulayıcılar (pure_logic.ConsecutiveFrameFilter) —
        # Şartname §7 yanlış pozitife karşı tedbir. Önceden sadece STOP
        # tabelası bu şekilde doğrulanıyordu, diğer 9 parkur-aşaması tabelası
        # (SULU_YOL, TASLI_YOL, ...) tek kare ile anında kabul ediliyordu.
        self._stop_filter   = ConsecutiveFrameFilter(
            STOP_CONSECUTIVE_FRAMES, bos_deger=False, sticky=False)
        self._tabela_filter = ConsecutiveFrameFilter(
            TABELA_CONSECUTIVE_FRAMES, bos_deger=NO_DETECTION, sticky=True)

        self.create_subscription(
            Detection2DArray,
            YOLO_RAW_TOPIC,
            self._on_detections,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"YoloAdapterNode hazır | conf≥{self._conf_threshold} | "
            f"img_center=({self._img_cx},{self._img_cy}) | "
            f"{DETECTIONS_TOPIC} + {YOLO_CLASS_ID_TOPIC}"
        )

    def _on_detections(self, msg: Detection2DArray):
        tabela_id        = NO_DETECTION
        hedef_var        = False
        hedef_hata_x     = 0.0
        hedef_hata_y     = 0.0
        koni_var         = False
        hizlanma_bitti   = False
        stop_goruldu     = False
        best_tabela_conf = 0.0
        best_hedef_conf  = 0.0

        for det in msg.detections:
            if not det.results:
                continue
            hyp  = det.results[0]
            cid  = int(hyp.hypothesis.class_id)
            conf = float(hyp.hypothesis.score)

            if conf < self._conf_threshold:
                continue

            if cid in TABELA_SINIF:
                if conf > best_tabela_conf:
                    best_tabela_conf = conf
                    tabela_id = cid

            elif cid == CLASS_HEDEF_TAHTASI:
                if conf > best_hedef_conf:
                    best_hedef_conf  = conf
                    hedef_var        = True
                    hedef_hata_x     = det.bbox.center.x - self._img_cx
                    hedef_hata_y     = det.bbox.center.y - self._img_cy

            elif cid == CLASS_TRAFIK_HUNI:
                koni_var = True

            elif cid == CLASS_TABELA_11:
                self.get_logger().info(
                    '§6.11 Tabela_11 tespit → hızlanma parkuru başlıyor.',
                    throttle_duration_sec=2.0,
                )

            elif cid == CLASS_TABELA_11_SON:
                hizlanma_bitti = True
                self.get_logger().info(
                    '§6.11 Tabela_11_son tespit → hızlanma parkuru bitiyor.',
                    throttle_duration_sec=2.0,
                )

            elif cid == CLASS_TABELA_STOP:
                stop_goruldu = True

        # Ardışık frame filtreleri (pure_logic.ConsecutiveFrameFilter) —
        # test_birim.py bu sınıfı doğrudan test eder.
        stop_var = self._stop_filter.isle(stop_goruldu)
        if stop_var:
            self.get_logger().info(
                '§6.10 STOP işareti onaylandı → FSM durdurma.',
                throttle_duration_sec=1.0,
            )

        tabela_confirmed = self._tabela_filter.isle(tabela_id)

        payload = {
            "tabela":           tabela_confirmed,
            "hedef_var":        hedef_var,
            "hedef_hata_x":     round(hedef_hata_x, 1),
            "hedef_hata_y":     round(hedef_hata_y, 1),
            "koni_var":         koni_var,
            "hizlanma_bitti":   hizlanma_bitti,
            "stop_var":         stop_var,
            "bariyer_sol_m":    1.5,
            "bariyer_sag_m":    1.5,
            "fps":              0.0,
        }

        self._pub_json.publish(String(data=json.dumps(payload)))
        self._pub_class.publish(UInt8(data=tabela_confirmed))

        if tabela_id != NO_DETECTION:
            self.get_logger().debug(
                f"tabela={tabela_id}({TABELA_SINIF[tabela_id]}) "
                f"hedef={hedef_var}({hedef_hata_x:.0f},{hedef_hata_y:.0f}px) "
                f"koni={koni_var}",
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
