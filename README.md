# LYDİA İKA — Teknofest Otonom Kara Aracı

**Takım:** MAGNESIA | **Başvuru ID:** 4908667
**Platform:** ROS2 Humble | Jetson Orin Nano | Ubuntu 22.04
**KTR:** 1 Haziran 2026 | **AKV:** 20 Temmuz 2026 | **Final:** Ağustos–Eylül 2026

---

## İçindekiler

1. [Sisteme Genel Bakış](#1-sisteme-genel-bakış)
2. [Donanım](#2-donanım)
3. [Yazılım Mimarisi](#3-yazılım-mimarisi)
4. [ROS2 Paket Yapısı](#4-ros2-paket-yapısı)
5. [Node'lar ve Görevleri](#5-nodelar-ve-görevleri)
6. [TF Zinciri](#6-tf-zinciri)
7. [Topic Arayüzleri](#7-topic-arayüzleri)
8. [Nav2 Konfigürasyonu](#8-nav2-konfigürasyonu)
9. [Sensör Füzyonu (EKF)](#9-sensör-füzyonu-ekf)
10. [Görev Durum Makinesi](#10-görev-durum-makinesi)
11. [Parkur Aşamaları](#11-parkur-aşamaları)
12. [Kurulum](#12-kurulum)
13. [Çalıştırma](#13-çalıştırma)
14. [Ölçümler ve bekleyen kalibrasyonlar](#14-ölçümler-ve-bekleyen-kalibrasyonlar)

---

## 1. Sisteme Genel Bakış

LYDİA, Teknofest İnsansız Kara Aracı yarışması için geliştirilmiş tam otonom bir kara aracıdır. SLAM ile eş zamanlı harita oluşturma ve lokalizasyon, Nav2 ile otonom navigasyon, YOLOv8 tabanlı tabela tespiti ve SMACH durum makinesi ile görev yönetimini entegre eder.

```
┌─────────────────────────────────────────────────────────────────┐
│                       Jetson Orin Nano                           │
│                                                                  │
│  ┌───────────┐   ┌──────────────┐   ┌─────────────────────────┐ │
│  │ SLAM      │   │ EKF          │   │ Nav2 Stack              │ │
│  │ Toolbox   │──▶│ (odom+IMU)   │──▶│ SmacPlanner + RPP       │ │
│  └───────────┘   └──────────────┘   └────────────┬────────────┘ │
│       ▲                                           │ /cmd_vel     │
│  ┌────┴──────┐   ┌──────────────┐                ▼             │
│  │ scan_relay│   │ misyon_fsm   │   ┌─────────────────────────┐ │
│  │ /scan_lidar   │ (SMACH FSM)  │   │ ackermann_converter     │ │
│  └───────────┘   └──────┬───────┘   └────────────┬────────────┘ │
│       ▲                 │                         │             │
│  ┌────┴──────┐          │           ┌─────────────────────────┐ │
│  │ YDLidar   │          │           │ seri_kopru              │ │
│  │ Tmini Pro │          │           │ (binary protokol)       │ │
│  └───────────┘          │           └────────────┬────────────┘ │
│                         │                        │ ST-LINK VCP │
└─────────────────────────┼────────────────────────┼─────────────┘
                          │                        │
                    Nav2 Action              ┌─────┴──────────────┐
                    NavigateToPose           │  Nucleo-F767ZI      │
                                             │  motor·fren·direksiyon │
                                             │  RC · BNO055 · enkoder │
                                             └────────────────────┘
```

---

## 2. Donanım

| Bileşen | Model / Değer |
|---|---|
| Ana işlemci | NVIDIA Jetson Orin Nano |
| Sürüş kartı | Nucleo-F767ZI — motor, fren, direksiyon, RC, IMU, enkoder |
| Motor sürücü | Pilmak 48 V 1200 W BLDC |
| Gaz DAC | MCP4725 (12 bit) |
| Fren | BTS7960B H-köprü, vidalı aktüatör |
| Direksiyon | Step motor + DM860H sürücü (açık çevrim, homing yok) |
| LiDAR | YDLidar Tmini Pro (2D, 360°, 10Hz) |
| Kamera | IMX258 OIS (×1 ön) + Microcase 720P (×3) |
| Derinlik | Orbbec Gemini 335L (karar verildi, sipariş edilmedi) |
| IMU | BNO055 (NDOF, kartta) |
| Enkoder | E6B2-CWZ6C 600 P/R kuadratür ×4 = 2400 sayım/tur, motor miline 1:1 |
| RC Kumanda | Flysky FS-i6X — CH1 direksiyon · CH2 gaz · CH3 fren · CH7 kesme · CH8 taret · CH9 mod · CH10 ışık |
| E-STOP | Schneider XB5AS84W3B5 — mekanik kontak 48 V'u keser, NC blok karta gider |
| Gövde | 1,90 × 1,16 × 0,76 m |
| Kinematik | Ackermann (otomobil tipi direksiyon) |

⚠ Jetson'ın GPIO'suna E-STOP hattı **bağlı değil**; durum karttan `0x34` ile
geliyor. Gaz voltajı, fren zamanlaması ve kalkış itişi kartın işi — köprüde
karşılıkları yok.

### Ackermann Parametreleri

```
Dingil arası (L)  : 1.44 m       mezürle ölçüldü
Maks. steer açısı : 30° (0.52 rad)  ⚠ mekanik uç ölçülmedi, muhafazakâr sınır
Min. dönüş yarıçapı: L / tan(δ_max) = 2.49 m
```

Direksiyon kolonu ile tekerlek arasındaki oran **13.091** olarak ölçüldü ve karta
`0x09` ayar paketiyle gönderiliyor. Bu oranla kolonun ±750° sınırı tekerde ±57°
demek; iki ölçümden biri tutarsız olduğu için `max_steering_angle` 30°'de
bırakıldı — kelepçenin rahat içinde kalıyor ve `minimum_turning_radius` geçerli.

---

## 3. Yazılım Mimarisi

### Teknoloji Yığını

| Katman | Teknoloji |
|---|---|
| İşletim sistemi | Ubuntu 22.04 LTS |
| Robot framework | ROS2 Humble |
| Navigasyon | Nav2 (SmacPlannerHybrid + RPP) |
| Lokalizasyon | SLAM Toolbox + EKF (robot_localization) |
| Görev yönetimi | SMACH FSM |
| Nesne tespiti | YOLOv8 + TensorRT (görüntü ekibi) |
| Veri kaydı | OpenCV + cv_bridge (MP4) |
| MCU haberleşme | Binary seri protokol (8-byte sabit paket) |

### Topic Akışı

```
YDLidar → /scan → preprocessing_node → /scan/filtered → SLAM, Nav2 costmap
         (ham /scan aracın arkasındaki gövde dönüşlerini de taşıyor)
sürüş kartı → /kart/kip → mod_yoneticisi → mod kararı (karar kartta)
Nav2 → /cmd_vel → mod_yoneticisi → /mux/cmd_vel → ackermann_converter → /ackermann_cmd → seri_kopru → sürüş kartı
/camera/image_raw → preprocessing_node → yolo_detection_node → /detections/yolo
/detections/yolo → yolo_adapter_node → /ika/detections (JSON) → misyon_fsm
                                      → /yolo/class_id (UInt8) → terrain_adapter → Nav2 param güncelle
/camera/taret/image_raw → preprocessing_node → /camera/taret/image_processed → targeting_node → /turret/cmd → servo_controller_node
misyon_fsm → Nav2 NavigateToPose action → otonom sürüş
```

### E-STOP Mimarisi

Üç katman var ve yalnız sonuncusu yazılımda: mantar butonun mekanik kontağı
48 V'u doğrudan kesiyor, NC blok sürüş kartının pinine gidiyor, kart durumu
`0x34` ile bildiriyor. Jetson'ın GPIO'suna hiçbir hat bağlı değil.

```
kart 0x34 (fiziksel buton) ──┐
imu_guvenlik (roll>20°)    ──► e_stop_node → /e_stop (20Hz) → watchdog izler
kumanda kesmesi (DRM_KESME)──┘   (OR mantığı — hepsi temiz demeden düşmez)
GCS komutu                 ──┘
```

### Mod Sistemi

| Kart kipi (`0x39`) | Mod | Davranış |
|---|---|---|
| 0 | MANUAL | Kart kumandadan doğrudan sürer, Jetson komutunu yok sayar |
| 1 | — | Sözleşmede tanımlı, kullanılmıyor |
| 2 | FULL_AUTO | Tam otonom |

Kip anahtarını (SwC/CH9) **sürüş kartı okuyor ve kararı kart veriyor**;
Jetson'ın oyu yok. Sıralama güvenlik > kumanda > Jetson. `mod_yoneticisi`
kararı `/kart/kip`'ten alır, ham kanal eşiklemez — anahtar üç konumlu ve
kullanılmayan orta konum eski eşiğin üstüne düşüyordu. Kip 0,5 saniye
gelmezse mod manuele döner.

---

## 4. ROS2 Paket Yapısı

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

## 5. Node'lar ve Görevleri

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

## 6. TF Zinciri

```
map
 └── odom              ← SLAM Toolbox yayınlar
      └── base_footprint    ← EKF (robot_localization) yayınlar
           └── base_link
                ├── lidar_link
                ├── imu_link
                ├── kamera_link
                └── os30a_link
```

---

## 7. Topic Arayüzleri

| Topic | Tip | Yayıncı | Abone |
|---|---|---|---|
| `/scan` | LaserScan | ydlidar_node | preprocessing_node (boot `-r /scan_lidar:=/scan`) |
| `/scan/filtered` | LaserScan | preprocessing_node | SLAM, Nav2 costmap, cone_fusion, kayar_engel |
| `/scan_raw` | LaserScan | ydlidar_node (yalnız launch yolunda) | scan_relay |
| `/scan_lidar` | LaserScan | scan_relay (yalnız launch yolunda) | preprocessing_node |
| `/cmd_vel` | Twist | Nav2 RPP | mod_yoneticisi |
| `/mux/cmd_vel` | Twist | mod_yoneticisi | ackermann_converter |
| `/ackermann_cmd` | AckermannDriveStamped | ackermann_converter | seri_kopru |
| `/odometry/filtered` | Odometry | EKF | Nav2, SLAM |
| `/rc_input` | Float32MultiArray | seri_kopru | mod_yoneticisi, taret_rc_koprusu, web_dashboard |
| `/e_stop` | Bool | e_stop_node | mod_yoneticisi, seri_kopru, misyon_fsm, watchdog |
| `/e_stop/gpio_fault` | Bool | e_stop_node | web_dashboard |
| `/kart/kip` | UInt8 | seri_kopru | mod_yoneticisi — sürüş kipi (`0x39`) |
| `/kart/hata` | UInt16 | seri_kopru | misyon_fsm, web_dashboard — `HATA_*` (`0x35`) |
| `/kart/durum` | UInt16 | seri_kopru | mod_yoneticisi, web_dashboard — `DRM_*` (`0x36`) |
| `/kart/link` | UInt16 | seri_kopru | web_dashboard — `JDR_*` (`0x39`) |
| `/kart/surus` | Int16MultiArray | seri_kopru | web_dashboard — gaz [mV], fren [‰] (`0x37`) |
| `/kart/kabul` | Int16MultiArray | seri_kopru | web_dashboard — kartın anladığı komut (`0x38`) |
| `/kart/surum` | UInt16MultiArray | seri_kopru | web_dashboard — protokol, yapı (`0x3B`) |
| `/kart/hat` | UInt16MultiArray | seri_kopru | web_dashboard — alınan, bozuk (`0x3C`) |
| `/kart/calisma_suresi` | UInt16 | seri_kopru | web_dashboard — kart reseti buradan görülür (`0x35`) |
| `/enkoder/ham` | Int32 | seri_kopru | web_dashboard, sensor_dogrula — ham sayım (`0x30`) |
| `/sensor/fault` | String | watchdog | web_dashboard |
| `/misyon/kalan_sure` | Float32 | misyon_fsm | web_dashboard (§6.12 koşu saati) |
| `/veri_paketi/kayit_durumu` | Bool | veri_paketi | web_dashboard |
| `/speed_limit` | Float32 | imu_guvenlik | terrain_adapter, mod_yoneticisi |
| `/yolo/class_id` | UInt8 | yolo_adapter | terrain_adapter |
| `/ika/detections` | String (JSON) | yolo_adapter | misyon_fsm |
| `/camera/taret/image_processed` | Image | preprocessing_node | targeting_node |
| `/shoot_command` | Bool | misyon_fsm, mod_yoneticisi (manuel) | seri_kopru, servo_controller_node |

---

## 8. Nav2 Konfigürasyonu

### Gerçek Araç (Ackermann)

```yaml
planner: SmacPlannerHybrid
  motion_model_for_search: REEDS_SHEPP   # İleri + geri yay (geri vites cezalı)
  allow_reverse_expansion: true          # REEDS_SHEPP'in geri yayı için gerekli
  minimum_turning_radius: 2.49           # L/tan(δ_max) = 1.44/tan(30°)

controller: RegulatedPurePursuitController
  use_rotate_to_heading: false   # Ackermann yerinde dönemez
  desired_linear_vel: 0.65       # terrain_adapter çalışma anında 0.45–0.90 yazar
```

### Davranış Ağacı — neden kendi ağacımız var

Nav2'nin stok ağaçları kurtarma dalında `<Spin>` kullanıyor. Ackermann araç
yerinde dönemediği için `nav2_spin_action_bt_node` yüklenmiyor; tanınmayan
düğüm ise ağacın ayrıştırılamamasına ve **bt_navigator'ın aktive olamamasına**
yol açıyor. Sonuç yalnız kurtarmanın değil Nav2'nin tamamının düşmesi:

```
[bt_navigator] Node not recognized: Spin
[lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.
```

Bu yüzden `config/bt/` altında Spin'siz iki ağaç var ve `nav2_params.yaml`
onlara **tam yolla** işaret ediyor. İkisi de gerekli: `navigators` yalnız
`navigate_to_pose` dese bile bt_navigator through_poses ağacını da yüklüyor.

Çalışma alanı taşınırsa güncellenecek yer: `nav2_params.yaml` içindeki iki
`default_nav_*_bt_xml` satırı.

### Kurtarma sırası ve geri gitme

`RoundRobin` her başarısızlıkta sıradaki davranışı dener:

1. **costmap temizle** — hayalet engel yüzünden plan üretilemiyorsa
2. **geri git** (`BackUp` 0,5 m @ 0,5 m/s) — poz planlanamaz hâle geldiyse.
   İleri-yönlü planlayıcıda R_min'den dar çıkış gerektiren bir pozdan hiçbir
   ileri yay çıkmaz; araç kımıldamadan tekrar denemek aynı sonucu verir.
3. **bekle** (3 s) — §6.8 kayar engel gibi geçici bir engel varsa

Geri gitme beklemeden önce geliyor: bekleme yalnız hareketli engelde işe yarar,
planlanamaz pozda geçen her saniye koşu saatinden gider.

Sayılar aracın ölçülen gerçeğinden: araç **0,45 m/s altında yerinden kalkmıyor**,
bu yüzden `backup_speed` 0,5 ve `velocity_smoother.min_velocity[0]` −0,50.
Stok değer (0,05 m/s) bu araçta iki kere ölüdür — smoother'ın deadband'i (0,05)
sıfırlar, kalkış sürtünmesi de geçirmez.

⚠️ **Otonom geri gitme YALNIZ bu kurtarmadır.** Planlayıcı geri vites kullanmaz
ve kullanmamalı: LiDAR gövdeye takıldığı için aracın arkası kör, §6.12 geri
kamera da takılı değil.

---

## 9. Sensör Füzyonu (EKF)

```
/odom (kart 0x31 hızı)   ──▶ EKF ──▶ /odometry/filtered
/imu/data (BNO055, kart) ──┘          child_frame: base_footprint
```

---

## 10. Görev Durum Makinesi

**FSM Durum Açıklamaları:**

| Durum | Açıklama |
|---|---|
| IDLE | `/mission_start True` bekler |
| NAVIGATE | sıradaki aşamayı sürer (hedefleme yolu aşağıda) |
| SHOOT_APPROACH | ATIS waypoint'ine yaklaşır, hedef onayı bekler |
| SHOOT | `hedef_hata ≤ ±5px` ise `/shoot_command True` |
| HIZLANMA | Nav2 bypass — direkt /cmd_vel, 10m/s, 30m mesafe |
| ERROR_RECOVERY | navigasyon hatası → 2 deneme, sonra sonraki aşama |

**Hedefleme Yolu (`hedefleme_modu`)**

NAVIGATE aşamayı iki yoldan biriyle sürer. Seçim koşu başında bir kez yapılır
ve gerekçesiyle loglanır.

| mod | hedef nereden | aşama ne zaman biter | ön koşul |
|---|---|---|---|
| `harita` | `waypoints.yaml` → `waypoint:` (map çerçevesi) | Nav2 hedefe varınca | `parkur_cad.donusum` ölçülmüş ve koordinatlar doldurulmuş olmalı |
| `kayan` | her 1,5 s'de LiDAR taramasından üretilir | `mesafe_m` kadar yol kat edilince | odometri (enkoder) |
| `oto` | waypoint'ler doluysa `harita`, hepsi (0,0) ise `kayan` | — | — |

Kayan hedef haritaya da waypoint koordinatına da ihtiyaç duymaz: hedef araç
çerçevesinde doğar, `odom` çerçevesine taşınıp Nav2'ye verilir ve bir sonraki
döngüde yenisiyle değiştirilir. Düz şeritte koridorun merkez çizgisi, U
dönüşünde aşamanın `viraj` alanında yazan iç duvar takip edilir; merkez çizgisi
hedef veremediği anda iç duvara düşülür.

Sahada değiştirmek için:

```bash
HEDEFLEME_MODU=kayan NAV2_AKTIF=1 ENKODER_AKTIF=1 ./scripts/lydia_startup.sh
```

**Özel Durumlar:**
- `DIK_EGIM_GIRIS/CIKIS`: STOP tabelasında 2s dur (Şartname §6.10). STOP
  tabelası kaçırılırsa waypoint'e varışta konum-tabanlı yedek 2s bekleme
  otomatik devreye girer — tek tetikleyici görüntü tespitine bağımlı değildir.
- `KAYAR_ENGEL`: Yön tespiti için max 10s bekle (hız bileşeniyle öngörülür)
- `HIZLANMA`: Tabela_11 → max hız, Tabela_11_son → dur (§6.11)
- `SHOOT`: ateşten sonra hareket kilidi en az `LASER_FIRE_DURATION` (1s)
  yazılım seviyesinde garanti edilir, ardından kilit açılır

---

## 11. Parkur Aşamaları

Aşama, waypoint koordinatına varışla değil **`mesafe_m` kadar yol kat edilmesiyle**
biter. Mesafeler parkur CAD'inden planlayıcıyla ölçüldü (kuş uçuşu değil, gerçek yol
uzunluğu); süreler o mesafelerden türetildi.

| Sıra | Aşama | Mesafe | Süre | Arazi | Özel |
|---|---|---|---|---|---|
| 1 | SULU_YOL | 10,0 m | 50 s | wet | — |
| 2 | TASLI_YOL | 7,2 m | 40 s | gravel | — |
| 3 | YAN_EGIM | 5,0 m | 25 s | slope | — |
| 4 | DIK_ENGEL | 22,0 m | 90 s | obstacle | Nav2 kaçınır |
| 5 | KONİLİ_YOL | 5,0 m | 25 s | normal | Nav2 costmap |
| 6 | KAYAR_ENGEL | 9,8 m | 65 s | normal | Yön beklemesi |
| 7 | ENGEBELİ_ARAZİ | 15,7 m | 65 s | rough | §6.9 koridor ortalaması |
| 8 | DIK_EGIM_GIRIS | 13,0 m | 65 s | rough | **2 s dur** · Nav2 baypas |
| 9 | ATIS_BOLGESI | 2,5 m | 25 s | slow | Nişan onayı → ateş |
| 10 | DIK_EGIM_CIKIS | 2,5 m | 25 s | normal | **2 s dur** · Nav2 baypas |
| 11 | HIZLANMA_PARKURU | 19,2 m | 65 s | fast | Nav2 baypas · 30 m puanlanan + 10 m durma payı |

Toplam 111,9 m yol, 540 s aşama bütçesi; koşu süresi tavanı 900 s.

> ⚠️ `waypoints.yaml`'daki `waypoint:` koordinatları 11/11 hâlâ (0,0). Kayan hedef
> modu koordinat kullanmadığı için bu bloker değil — aşamalar mesafeyle bitiyor.
> `waypoint_cad:` alanları ise parkur CAD'inden çıkarılmış durumda.

---

## 12. Kurulum

```bash
# ROS2 Humble bağımlılıkları
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-ackermann-msgs \
  ros-humble-ydlidar-ros2-driver

pip3 install pyserial smbus2 ultralytics --break-system-packages

# OS30A derinlik kamerası (eYs3D BMVM0S30A) — AYRI bir workspace'te kurulur,
# rosdep ile çözümlenemeyen üçüncü parti bir paket olduğu için package.xml'e
# <depend> olarak eklenmemiştir (eklenirse rosdep install hata verir):
git clone https://github.com/eYs3D/HD-DM-ROS2-SDK-Release.git ~/eys3d_ws/src/dm_preview
cd ~/eys3d_ws && rosdep install -i --from-path src -y
colcon build --symlink-install
echo "source ~/eys3d_ws/install/setup.bash" >> ~/.bashrc

# Workspace build
cd ~/lydia_ws
# --symlink-install kullanilmaz: aractaki install/ kopya tabanli, bayrakla
# derlemek maps/teknofest_harita.pgm uzerinde [Errno 2] verip yarida keser.
colcon build --packages-select teknofest_ika
source install/setup.bash
echo "source ~/lydia_ws/install/setup.bash" >> ~/.bashrc
```

---

## 13. Çalıştırma

### Gerçek Araç

```bash
# Harita alma (sahada, ilk çalıştırma)
ros2 launch teknofest_ika gercek_harita.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/lydia_ws/maps/gercek_harita

# Yarışma
ros2 launch teknofest_ika gercek_arac.launch.py
ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
```

### Faydalı Komutlar

```bash
# TF zinciri kontrolü
ros2 run tf2_ros tf2_echo map odom

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Lidar veri kalitesi
ros2 topic hz /scan_lidar

# Nav2 manuel hedef
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

---

## 14. Ölçümler ve bekleyen kalibrasyonlar

### Ölçülenler

| Ölçüm | Değer | Nerede |
|---|---|---|
| Dingil arası | **1,44 m** | `ackermann_converter.py`, `urdf/arac.urdf`, `nav2_params.yaml`, `gercek_arac.launch.py` — dördü senkron |
| Min. dönüş yarıçapı | **2,49 m** | `nav2_params.yaml` — dingil arasından türer |
| Gövde ölçüleri | **1,83 × 1,22 m** | Nav2 footprint (yarım en 0,610 + 0,05 padding) |
| Direksiyon kolon/teker oranı | **13,091** | karta `0x09` ayarıyla gönderiliyor |
| LiDAR tarama düzlemi | zeminden **0,60 m** | `lydia_startup.sh` statik TF, `urdf/arac.urdf` ile tutarlı |
| LiDAR montaj açısı | **93,3°** | sahada huniyle çift yönlü kalibre edildi |
| Aşama mesafeleri | 11/11 dolu | parkur CAD'inden planlayıcıyla ölçüldü |

### Bekleyenler

| Görev | Nerede | Not |
|---|---|---|
| 🔴 **Odometri ölçeği** | kart `0x09`: `tekerlek_cevre_mm` + `gosterge_darbe_tur` | **En kritik madde.** Kart hız alanını (`0x31`) ölçek girilmeden bilerek `0` basıyor; `/odom` → EKF → aşama mesafesi zinciri buna bağlı. Çevre **yük altında** ölçülmeli: havalı lastik çöktüğü için geometrik `π·d` gerçekten %2–4 büyük. ⚠ Kart hızı iki kaynaktan alıyor (mil enkoderi ya da gösterge ucu) ve canlı olanı kendisi seçiyor; hangisinin bağlı olduğu aracı iterek `/enkoder/ham` sayımından anlaşılır. Sayım dönerken hız alanı 0 kalıyorsa köprü **"ÖLÇEK UYGULANMIYOR"** basar |
| 🔴 **BNO055 yaw doğrulaması** | — | EKF'in tek yön kaynağı. Son ölçümde yaw bütün örneklerde tam `0,000` ve kalibrasyon eşiğin altındaydı; araç durduğu için meşru da olabilir, **çevirmeden ayırt edilemez**. `imu_guvenlik` bu yüzden kapalı |
| 🔴 **Direksiyon işareti** | kart `0x09`: `direksiyon_isaret` | Yalnız `+1` / `−1` kabul edilir, ölçülmedi. Ters işarette düzeltme hatayı büyütür; ilk deneme **tekerlekler yerden kesik** yapılmalı |
| 🟠 Maks. steer açısı | `ackermann_converter.py` | 30° muhafazakâr sınır. Mekanik uç ölçülmedi; kolon ±750° ile 13,091 oranı tekerde ±57° veriyor ve iki ölçümden biri tutarsız |
| 🟠 Rampa hızları | `waypoints.yaml` | Tırmanış 0,60 / iniş 0,40 m/s — ikisi de ölçülmedi |
| 🟠 Fren kalibrasyonu | `topics.py` | Fren oranı sabitleri ve tork oturma penceresi muhafazakâr yer tutucu |
| ⚪ Parkur waypoint koordinatları | `waypoints.yaml` | 11 nokta (0,0). Kayan hedef modu koordinat kullanmadığı için bloker değil |
| ⚪ TensorRT engine | `models/best.engine` | `scripts/export_tensorrt.py` ile üretilir; depoda dağıtılmıyor |

---

## Lisans

[Apache License 2.0](LICENSE). Kullanabilir, değiştirebilir ve dağıtabilirsiniz;
telif bildirimini ve [`NOTICE`](NOTICE) dosyasını korumanız gerekir.

Depoda dağıtılmayan iki şey var: sürüş kartının firmware kaynağı ve eğitilmiş
model ağırlıkları (`models/`). Çalışma zamanı bağımlılıkları (ROS 2, Nav2,
slam_toolbox, OpenCV ve diğerleri) ayrıca kurulur ve her biri kendi lisansıyla
dağıtılır. YOLO çıkarımının isteğe bağlı Ultralytics yolu AGPL-3.0'dır ve bu
depo onu dağıtmaz — ayrıntı [`requirements.txt`](requirements.txt) içinde.

## Katkıda bulunanlar

### Buğra Öztürk ([@bugraozturk012](https://github.com/bugraozturk012)) — otonom sürüş yazılımı

Jetson üzerinde koşan bütün karar katmanı: algı, füzyon, navigasyon, görev yönetimi ve
sürüş kartıyla konuşan köprü. Bu depodaki 23 ROS 2 düğümünün tamamı ve 1.053 birim test.

- **Görev durum makinesi** — [`misyon_fsm.py`](teknofest_ika/otonomi/misyon_fsm.py) (2.700 satır):
  SMACH ile dokuz durum. Kayan hedef sürüşü (hedefler her döngüde LiDAR taramasından
  üretilir; haritaya da waypoint koordinatına da ihtiyaç yok), §6.10 rampa ve §6.11 hızlanma
  için Nav2 baypasları, atış yaklaşma/atış zinciri, hata kurtarma ve koşu süresi bütçesi.
- **Saf mantık katmanı** — [`pure_logic.py`](teknofest_ika/otonomi/pure_logic.py) (1.500 satır):
  ROS'tan bağımsız karar fonksiyonları. Ackermann kinematiği, koridor merkez çizgisi ve iç
  duvar takibi, kayan hedef üretimi, geri kayma riski, kart ayarlarının kodlanması, sensör
  ve ölçek kapıları. Kararlar düğümlerden buraya taşındı ki davranışla sınanabilsinler.
- **Sürüş kartı köprüsü** — [`seri_kopru.py`](teknofest_ika/gomulu/seri_kopru.py) (1.200 satır):
  arayüz sözleşmesinin Jetson tarafı, `0x09` ayar paketleri ve kalıcılığı, E-STOP zinciri,
  IMU ve odometri yayını, enkoder sessizliği ile ölçek denetimleri.
- **Navigasyon** — [`config/nav2_params.yaml`](config/nav2_params.yaml): ölçülmüş gövde ayak izi,
  SmacPlannerHybrid + Regulated Pure Pursuit ve kendi davranış ağaçları
  ([`config/bt/`](config/bt)) — stok ağaçtaki `Spin` düğümü Ackermann araçta `bt_navigator`'ı
  düşürüyordu. [`config/ekf.yaml`](config/ekf.yaml) ile enkoder + IMU füzyonu.
- **Görüntü işleme** — tabela tespiti (YOLO / TensorRT), koni ve kayar engel costmap
  besleyicileri, Kalman takibi, şerit tespiti, yağmur damlası onarımı, taret nişan düğümü.
- **Emniyet düğümleri** — `mod_yoneticisi`, `e_stop_node`, `watchdog`, `anti_rollback`,
  `imu_guvenlik`: kip hakemliği, dört E-STOP kaynağının birleştirilmesi, düğüm gözcülüğü,
  eğimde geri kayma koruması.
- **Parkur analizi** — [`scripts/parkur_cad/`](scripts/parkur_cad): TEKNOFEST'in yayımladığı
  STEP dosyasından tabela ve bariyer konumlarının çıkarılması, aşama mesafelerinin
  planlayıcıyla ölçülmesi, U dönüşü ve koni slalomu geçilebilirlik analizi.
- **Açılış ve saha araçları** — [`scripts/lydia_startup.sh`](scripts/lydia_startup.sh)
  (1.265 satır): düğümlerin sıralı başlatılması, ortam değişkeni kapıları, ön koşul
  doğrulamaları ve süreç gözcüsü. Web tabanlı araç panosu
  ([`scripts/web_dashboard.py`](scripts/web_dashboard.py), 1.689 satır).
- **Birim testleri** — [`test_birim.py`](test_birim.py): 1.053 test. Yapılandırma ile kod
  arasındaki tutarlılık, açılış betiğinin gerçekten ne geçirdiği ve durum makinesinin
  geçiş grafiği dahil; testler mutasyonla doğrulanıyor.

### Ahmet Efe NEZLİ ([@dwifk0](https://github.com/dwifk0)) — araç elektroniği ve sürüş kartı

Aracın Jetson altındaki bütün katmanı: kumandadan tekerleğe giden zincir, güç, emniyet ve
Jetson'la konuşan arayüz.

- **Sürüş kartı firmware'i (Nucleo-F767ZI)** — gaz (DAC ile BLDC kontrolcü), fren (BTS7960),
  step motorlu direksiyon ve jog kipi, iBUS kumanda ve failsafe, kip hakemi
  (manuel / otonom / DUR), E-STOP ve emniyet mandalları, tekerlek enkoderi ile hız ölçümü,
  BNO055 IMU, taret sürüşü. Önceki dönemde Arduino Mega üzerinde röleli direksiyon + ESC frenli
  ilk sürüş firmware'i.
- **Jetson ↔ kart arayüz sözleşmesi** — [`donanim/PROTOKOL_F767.md`](donanim/PROTOKOL_F767.md),
  [`teknofest_ika/gomulu/f767_protokol.py`](teknofest_ika/gomulu/f767_protokol.py) ve
  `seri_kopru.py`'nin F767 desteği: paket çerçevesi, fiziksel birimle komut, telemetri ve
  sağlık bayrakları.
- **Araç elektroniği** — bağlantı haritası ve kablolama, ana batarya (16S LiFePO4 + JK BMS) ve
  ayrı elektronik bataryası (4S + DALY), sigorta ve güç dağıtımı, acil stop devresi,
  motor kontrolcüsü bağlantıları, malzeme listesi ve seçimleri.
- **Araç panosu ve saha araçları** — web tabanlı araç panosu (telemetri, BMS'lerin BLE ile
  okunması, kamera akışları), Jetson açılış servisleri (`scripts/lydia_startup.sh`),
  araç ağı ve ekip uzaktan erişimi, tezgâh test düzeni ve testleri.

Kart firmware'i bu depoda yer almaz. Karar katmanının platformdan bağımsız, birim testli
sürümü: [dwifk0/IKA-MAGNESIA-LYDIA](https://github.com/dwifk0/IKA-MAGNESIA-LYDIA).

---

*Takım: MAGNESIA | LYDİA İKA | Teknofest 2026*
