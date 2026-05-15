#!/usr/bin/env python3
"""
yolo_adapter_node.py — Detection2DArray → /ika/detections JSON + /yolo/class_id köprüsü

Girdi : /detections/yolo  (vision_msgs/Detection2DArray)  — yolo_detection_node
Çıktı : /ika/detections   (std_msgs/String JSON)          — misyon_fsm
        /yolo/class_id    (std_msgs/UInt8)                — terrain_adapter

Tabela numarası → parkur aşaması (class_id = Tabela_N - 1):
  class_id=0  Tabela_1  → SULU_YOL
  class_id=1  Tabela_2  → TASLI_YOL
  class_id=2  Tabela_3  → YAN_EGIM
  class_id=3  Tabela_4  → DIK_ENGEL
  class_id=4  Tabela_5  → KONİLİ_YOL
  class_id=5  Tabela_6  → KAYAR_ENGEL
  class_id=6  Tabela_7  → ENGEBELİ_ARAZİ
  class_id=7  Tabela_8  → DIK_EGIM
  class_id=8  Tabela_9  → ATIS_BOLGESI
  class_id=9  Tabela_10 → YAN_EGIM_2
  class_id=13           → trafik_huni  → koni_var=True
  class_id=14           → hedef_tahtasi → hedef_var=True

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

from teknofest_ika.otonomi.topics import YOLO_CLASS_ID_TOPIC, DETECTIONS_TOPIC, YOLO_RAW_TOPIC

# class_id → parkur aşama adı (sadece loglama için)
# Model alfabetik sırayla eğitildi:
#   Tabela_1, Tabela_10, Tabela_11, Tabela_11_son, Tabela_12, Tabela_2, ...
TABELA_SINIF = {
    0:  "SULU_YOL",        # Tabela_1
    1:  "DIK_EGIM_CIKIS",  # Tabela_10 — DIK_EGIM çıkış tabelası
    5:  "TASLI_YOL",       # Tabela_2
    6:  "YAN_EGIM",        # Tabela_3
    7:  "DIK_ENGEL",       # Tabela_4
    8:  "KONİLİ_YOL",     # Tabela_5
    9:  "KAYAR_ENGEL",     # Tabela_6
    10: "ENGEBELİ_ARAZİ", # Tabela_7
    11: "DIK_EGIM",        # Tabela_8
    12: "ATIS_BOLGESI",    # Tabela_9
}

CLASS_TABELA_11     = 2    # §6.11 hızlanma başlangıcı
CLASS_TABELA_11_SON = 3    # §6.11 hızlanma sonu
CLASS_TABELA_12     = 4    # Tabela_12 — görüntü ekibinden netleştirilecek
CLASS_TABELA_STOP   = 13   # §6.10 ramp STOP işareti
CLASS_TRAFIK_HUNI   = 15
CLASS_HEDEF_TAHTASI = 14
NO_DETECTION        = 255

# Yanlış pozitife karşı: STOP art arda bu kadar frame gelmeden tetiklenmez
STOP_CONSECUTIVE_FRAMES = 2


class YoloAdapterNode(Node):

    def __init__(self):
        super().__init__("yolo_adapter_node")

        self.declare_parameter("img_cx",         320.0)
        self.declare_parameter("img_cy",         320.0)
        self.declare_parameter("conf_threshold",   0.45)

        self._img_cx          = self.get_parameter("img_cx").value
        self._img_cy          = self.get_parameter("img_cy").value
        self._conf_threshold  = self.get_parameter("conf_threshold").value

        self._pub_json  = self.create_publisher(String, DETECTIONS_TOPIC,    10)
        self._pub_class = self.create_publisher(UInt8,  YOLO_CLASS_ID_TOPIC, 10)
        self._stop_sayac = 0   # ardışık STOP frame sayacı

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

            elif cid == CLASS_TABELA_12:
                self.get_logger().warn(
                    'Tabela_12 tespit edildi (class_id=4) — görüntü ekibinden netleştirilecek.',
                    throttle_duration_sec=5.0,
                )

        # Ardışık frame filtresi: STOP_CONSECUTIVE_FRAMES kez üst üste görülmeden tetiklenme
        if stop_goruldu:
            self._stop_sayac += 1
            if self._stop_sayac >= STOP_CONSECUTIVE_FRAMES:
                self.get_logger().info(
                    f'§6.10 STOP işareti tespit ({self._stop_sayac} frame) → FSM durdurma.',
                    throttle_duration_sec=1.0,
                )
        else:
            self._stop_sayac = 0

        stop_var = self._stop_sayac >= STOP_CONSECUTIVE_FRAMES

        payload = {
            "tabela":           tabela_id,
            "hedef_var":        hedef_var,
            "hedef_hata_x":     round(hedef_hata_x, 1),
            "hedef_hata_y":     round(hedef_hata_y, 1),
            "koni_var":         koni_var,
            "hizlanma_bitti":   hizlanma_bitti,
            "stop_var":         stop_var,
            "kayar_yon":        "bilinmiyor",
            "bariyer_sol_m":    1.5,
            "bariyer_sag_m":    1.5,
            "fps":              0.0,
        }

        self._pub_json.publish(String(data=json.dumps(payload)))
        self._pub_class.publish(UInt8(data=tabela_id))

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
