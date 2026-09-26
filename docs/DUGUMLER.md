# Paket yapısı ve düğümler

> LYDİA İKA otonom sürüş yazılımı — ayrıntılı belge.
> Genel bakış için [README](../README.md).

---

## ROS2 Paket Yapısı

```
~/lydia_ws/
├── teknofest_ika/
│   ├── otonomi/
│   │   ├── topics.py               # Topic sabit tanımları
│   │   ├── misyon_fsm.py           # SMACH görev durum makinesi
│   │   ├── mod_yoneticisi.py       # RC mod seçici + mux
│   │   ├── terrain_adapter.py      # YOLO → Nav2 parametre adaptörü
│   │   ├── ackermann_converter.py  # cmd_vel → AckermannDrive
│   │   ├── anti_rollback.py        # Rampa geri kayma önleme
│   │   ├── imu_guvenlik.py         # Roll/pitch e-stop tetikleyici
│   │   ├── veri_paketi.py          # Kamera video kaydedici
│   │   ├── e_stop_node.py          # E-stop yöneticisi
│   │   ├── watchdog.py             # Kritik topic izleyici (/e_stop dahil)
│   │   └── pure_logic.py           # rclpy-bağımsız kritik hesaplamalar (test_birim.py bunu kullanır)
│   ├── gomulu/
│   │   ├── seri_kopru.py           # Sürüş kartı (F767) binary seri köprüsü
│   │   ├── f767_protokol.py        # Kart arayüz sözleşmesi — paket ve bayrak tanımları
│   │   ├── taret_rc_koprusu.py     # Taret kartı seri köprüsü
│   │   └── bms_koprusu.py          # BMS BLE → HTTP köprüsü
│   └── gorsel/
│       ├── scan_relay.py           # Lidar timestamp/frame düzeltici
│       ├── preprocessing_node.py   # Kamera ön işleme
│       ├── yolo_detection_node.py  # YOLOv8 çıkarım
│       ├── yolo_adapter_node.py    # YOLO → ROS2 arayüz
│       ├── targeting_node.py       # Taret HSV+PID kontrolü
│       ├── servo_controller_node.py# PCA9685 servo sürücü
│       ├── kayar_engel_kalman.py   # Kalman filtreli engel takibi
│       ├── kayar_engel_costmap.py  # Dinamik costmap yayıncı
│       ├── cone_fusion_node.py     # Lidar + kamera koni füzyonu
│       ├── lane_detection_node.py  # Şerit tespiti
│       └── map_image_node.py       # /map → pano görüntüsü
│   └── utils/
│       ├── tensorrt_inferer.py     # TensorRT çıkarım motoru
│       ├── camera_model.py         # Pinhole projeksiyon
│       └── pid_controller.py       # Integral windup korumalı PID
├── launch/
│   ├── gercek_arac.launch.py       # Gerçek araç — tam stack
│   └── gercek_harita.launch.py     # Gerçek araç — harita alma
├── config/
│   ├── nav2_params.yaml            # Gerçek araç Nav2 parametreleri
│   ├── ekf.yaml                    # Gerçek araç EKF konfigürasyonu
│   ├── mapper_params_online_sync.yaml  # SLAM Toolbox konfigürasyonu
│   ├── waypoints.yaml              # Parkur aşamaları: mesafe, süre, arazi
│   └── bt/                         # Kendi davranış ağaçlarımız (Spin'siz)
├── urdf/
│   └── arac.urdf                   # Araç URDF (robot_state_publisher)
├── scripts/
│   ├── lydia_startup.sh            # Araçtaki açılış otoritesi (systemd buradan)
│   ├── web_dashboard.py            # Araç panosu
│   └── parkur_cad/                 # STEP ayrıştırma ve geçilebilirlik analizi
├── models/
│   └── best.pt                     # YOLO model (15 sınıf)
├── package.xml
└── setup.py
```

---

## Node'lar ve Görevleri

### 5.1 `scan_relay.py` — Lidar Düzeltici

YDLidar Tmini Pro'nun [0x202] hatasında ürettiği bozuk scan'leri (timestamp=0, frame=laser_frame, değişken nokta sayısı) düzelterek SLAM ve Nav2 costmap'e iletir.

Yalnızca `launch/gercek_arac.launch.py` yolunda kullanılır. Araçta otorite
`scripts/lydia_startup.sh` ve o, sürücüyü `/scan_raw`'a remap edip relay'i araya
koymak yerine `preprocessing_node`'u doğrudan `-r /scan_lidar:=/scan` ile
başlatır — aynı işi tek satırda yapar. Yani sahada koşan zincirde `scan_relay`
**yoktur**; `/scan_raw` topic'ine hiçbir şey yayın yapmaz.

```
/scan_raw (BEST_EFFORT) → filtre (300–1500 nokta) → timestamp fix → frame_id='lidar_link' → /scan_lidar (RELIABLE)
```

### 5.2 `seri_kopru.py` — Sürüş kartı (Nucleo-F767ZI) ↔ ROS 2 köprüsü

**Binary Protokol (8 byte):**
```
[0xAA][CMD][v0_H][v0_L][v1_H][v1_L][CRC][0x55]
CRC = CMD ^ v0_H ^ v0_L ^ v1_H ^ v1_L
```

| Yön | Komut | Açıklama |
|---|---|---|
| Jetson→kart | `0x01` PKT_J_SURUCU | hız [mm/s] + direksiyon [1/100°], ROS işareti (+ sol) |
| Jetson→kart | `0x02` PKT_J_DUR | gaz rölanti + tam fren; KİLİT kurar, direksiyon açısına dokunmaz |
| Jetson→kart | `0x03` PKT_J_LAZER | lazer aç/kapa |
| Jetson→kart | `0x04` PKT_J_HB | heartbeat (yan etkisi olmayan tek paket) |
| Jetson→kart | `0x05` / `0x06` | taret pan / tilt [0–180°], 90° = dur |
| Jetson→kart | `0x07` PKT_J_ESTOP | Jetson E-STOP ilan eder |
| Jetson→kart | `0x08` PKT_J_FREN | fren [‰ 0–1000], işaretsiz |
| kart→Jetson | `0x30` enkoder sayımı (int32) · `0x31` ileri hız [mm/s] |
| kart→Jetson | `0x32` yaw/roll · `0x33` pitch + kalibrasyon |
| kart→Jetson | `0x34` E-STOP · `0x35` `HATA_*` + çalışma süresi |
| kart→Jetson | `0x36` gaz [‰] + `DRM_*` · `0x37` gaz [mV] + fren [‰] |
| kart→Jetson | `0x38` kartın anladığı komut · `0x39` kip + `JDR_*` |
| kart→Jetson | `0x3A` ham CH1/CH9 · `0x3B` sürüm · `0x3C` alınan/bozuk paket |

**Davranış kuralları:** Heartbeat penceresi 700 ms ve XOR'u tutan **her** paket
onu tazeler; bozuk çerçeveler bilerek tazelemez. `PKT_J_DUR` bir kilittir,
yalnız taze bir `PKT_J_SURUCU` çözer. Kart otonom dalda hızı kırpar: tavan
1,50 m/s, taban 0,20 m/s (altındaki sıfır olmayan komutlar tabana yükseltilir),
0,01 altı rölanti. Fren kaynakları arasında **büyük olan** kazanır — operatör
bizim frenimizin üstüne basabilir ama çözemez.

### 5.3 `ackermann_converter.py` — Kinematik Dönüştürücü

```
κ = ω / v            eğrilik, |κ| ≤ 1/R_min'e kırpılır
δ = arctan(L × κ)    R_min = L / tan(δ_max) = 2.49 m   (L = 1.44 m)

Giriş : /cmd_vel  (geometry_msgs/Twist)
Çıkış : /ackermann_cmd  (ackermann_msgs/AckermannDriveStamped)
```

İstenen yay R_min'den darsa direksiyon doyar ve hız taşma oranında düşürülür
(`viraj_taban_hizi` = 0.45 m/s kalkış sürtünmesi tabanının altına inilmez).
Doyma her seferinde loga uyarı olarak basılır.

### 5.4 `terrain_adapter.py` — Arazi Parametre Adaptörü

YOLO tabela tespitine göre Nav2 hız ve costmap parametrelerini anlık günceller. 3 ardışık aynı tespit eşiği sonrası profil değişir.

| Profil | `desired_linear_vel` (m/s) | `inflation_radius` (m) |
|---|---|---|
| normal | 0.65 | 0.40 |
| wet | 0.50 | 0.55 |
| gravel | 0.50 | 0.45 |
| slope | 0.45 | 0.65 |
| obstacle | 0.60 | 0.50 |
| rough | 0.45 | 0.60 |
| slow | 0.45 | 0.40 |
| fast | 0.90 | 0.35 |

Hızların tavanı sürüş kartının sınırıdır (1.50 m/s). `normal` profilinin 0.65
değeri sahada doğrulanmış en yüksek otonom hız; aracın tam gaz karşılığı
1.94 m/s ölçüldü ama o hız otonomda hiç denenmedi.

### 5.5 `misyon_fsm.py` — Görev Durum Makinesi

```
[IDLE] → [NAVIGATE] → [SHOOT_APPROACH] → [SHOOT] → [NAVIGATE]
              ↓                                           ↓
         [HIZLANMA]                               [MISSION_COMPLETE]
              ↓
         [ERROR_RECOVERY]
```

### 5.6 YOLO Sınıf Tablosu (15 sınıf, alfabetik)

| class_id | Model Adı | Parkur Anlamı | Terrain |
|---|---|---|---|
| 0 | Tabela_1 | SULU_YOL | wet |
| 1 | Tabela_10 | DIK_EGIM_CIKIS | normal |
| 2 | Tabela_11 | HIZLANMA başı | fast |
| 3 | Tabela_11_son | HIZLANMA sonu | normal |
| 4 | Tabela_12 | PARKURDA YOK | normal |
| 5 | Tabela_2 | TASLI_YOL | gravel |
| 6 | Tabela_3 | YAN_EGIM | slope |
| 7 | Tabela_4 | DIK_ENGEL | obstacle |
| 8 | Tabela_5 | KONİLİ_YOL | normal |
| 9 | Tabela_6 | KAYAR_ENGEL | normal |
| 10 | Tabela_7 | ENGEBELİ_ARAZİ | rough |
| 11 | Tabela_8 | DIK_EGIM girişi | rough |
| 12 | Tabela_9 | ATIS_BOLGESI | slow |
| 13 | Tabela_stop | STOP işareti | normal |
| 14 | hedef_tahtasi | Atış hedefi | slow |
| 15 | trafik_huni | Trafik konisi | normal |

---
