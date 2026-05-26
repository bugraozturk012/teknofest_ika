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
14. [Test Aracı](#14-test-aracı)
15. [Bekleyen Kalibrasyonlar](#15-bekleyen-kalibrasyonlar)

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
│                         │                        │ USB Seri    │
└─────────────────────────┼────────────────────────┼─────────────┘
                          │                        │
                    Nav2 Action              ┌─────┴──────────────┐
                    NavigateToPose           │  Arduino Mega 2560  │
                                             │  + Nano (IMU/servo) │
                                             │  VESC Flipsky 75100 │
                                             └────────────────────┘
```

---

## 2. Donanım

| Bileşen | Model / Değer |
|---|---|
| Ana işlemci | NVIDIA Jetson Orin Nano |
| Motor sürücü | VESC Flipsky 75100 |
| Ana MCU | Arduino Mega 2560 |
| IMU/Servo MCU | Arduino Nano |
| LiDAR | YDLidar Tmini Pro (2D, 360°, 10Hz) |
| Kamera | IMX258 OIS (×1 ön) + Microcase 720P (×3) |
| IMU | MPU9250 (6-eksen, Madgwick AHRS) |
| Enkoder | AS5600 manyetik enkoder (10-bit, 1024 tick/tur) |
| RC Kumanda | Flysky FS-i6X |
| E-STOP | Schneider XB5AS84W3B5 (NC kontağı → GPIO pin 7) |
| LoRa | LR02 433MHz → /dev/lora |
| Kinematik | Ackermann (otomobil tipi direksiyon) |

### Ackermann Parametreleri

```
Wheelbase        : ölçülecek [m]
Maks. steer açısı: ölçülecek [rad]
Min. dönüş yarı  : L / tan(δ_max)
```

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
YDLidar → /scan_raw → scan_relay → /scan_lidar → SLAM, Nav2 costmap
RC → /rc_input → mod_yoneticisi → mod kararı
Nav2 → /cmd_vel → mod_yoneticisi → /mux/cmd_vel → ackermann_converter → /ackermann_cmd → seri_kopru → Arduino
/camera/image_raw → preprocessing_node → yolo_detection_node → /detections/yolo
/detections/yolo → yolo_adapter_node → /ika/detections (JSON) → misyon_fsm
                                      → /yolo/class_id (UInt8) → terrain_adapter → Nav2 param güncelle
misyon_fsm → Nav2 NavigateToPose action → otonom sürüş
```

### E-STOP Mimarisi

```
Schneider GPIO (pin 7)    ──┐
imu_guvenlik (roll>20°)   ──► e_stop_node → /e_stop (20Hz)
seri_kopru PKT_ESTOP_IN   ──┘   (OR mantığı)
lora_gcs GCS komutu       ──┘
```

### Mod Sistemi

| RC ch5 | Mod | Davranış |
|---|---|---|
| < 1300µs | MANUAL (0) | RC doğrudan sürer |
| 1300–1700µs | SEMI_AUTO (1) | Nav2 + RC override |
| > 1700µs | FULL_AUTO (2) | Tam otonom |

---

## 4. ROS2 Paket Yapısı

```
~/ika_ws/
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
│   │   └── watchdog.py             # 10 kritik topic izleyici
│   ├── gomulu/
│   │   ├── seri_kopru.py           # Arduino binary seri köprü
│   │   └── lora_gcs.py             # LoRa GCS telemetri
│   └── gorsel/
│       ├── scan_relay.py           # Lidar timestamp/frame düzeltici
│       ├── preprocessing_node.py   # Kamera ön işleme
│       ├── yolo_detection_node.py  # YOLOv8 çıkarım
│       ├── yolo_adapter_node.py    # YOLO → ROS2 arayüz
│       ├── targeting_node.py       # Taret HSV+PID kontrolü
│       ├── servo_controller_node.py# PCA9685 servo sürücü
│       ├── kayar_engel_kalman.py   # Kalman filtreli engel takibi
│       ├── kayar_engel_costmap.py  # Dinamik costmap yayıncı
│       ├── koni_costmap.py         # Koni costmap yayıncı
│       └── cone_fusion_node.py     # Lidar + kamera koni füzyonu
├── launch/
│   ├── gercek_arac.launch.py       # Gerçek araç — tam stack
│   ├── gercek_harita.launch.py     # Gerçek araç — harita alma
│   └── test_arac.launch.py         # Test aracı (DC motor, diferansiyel)
├── config/
│   ├── nav2_params.yaml            # Gerçek araç Nav2 parametreleri
│   ├── nav2_params_test.yaml       # Test aracı Nav2 parametreleri
│   ├── ekf.yaml                    # Gerçek araç EKF konfigürasyonu
│   ├── ekf_test.yaml               # Test aracı EKF konfigürasyonu
│   ├── mapper_params_online_sync.yaml  # SLAM Toolbox konfigürasyonu
│   └── waypoints.yaml              # Parkur waypoint koordinatları
├── urdf/
│   └── arac.urdf                   # Araç URDF (robot_state_publisher)
├── meshes/
│   ├── govde.stl
│   └── tekerlek.stl
├── maps/                           # Kaydedilen SLAM haritaları
├── arduino/
│   └── firmware.c                  # Test aracı Arduino firmware
├── models/
│   └── best.pt                     # YOLOv8 model (16 sınıf)
├── package.xml
└── setup.py
```

---

## 5. Node'lar ve Görevleri

### 5.1 `scan_relay.py` — Lidar Düzeltici

YDLidar Tmini Pro'nun [0x202] hatasında ürettiği bozuk scan'leri (timestamp=0, frame=laser_frame, değişken nokta sayısı) düzelterek SLAM ve Nav2 costmap'e iletir.

```
/scan_raw (BEST_EFFORT) → filtre (300–1500 nokta) → timestamp fix → frame_id='lidar_link' → /scan_lidar (RELIABLE)
```

### 5.2 `seri_kopru.py` — Arduino ↔ ROS2 Köprüsü

**Binary Protokol (8 byte):**
```
[0xAA][CMD][v0_H][v0_L][v1_H][v1_L][CRC][0x55]
CRC = CMD ^ v0_H ^ v0_L ^ v1_H ^ v1_L
```

| Yön | Komut | Açıklama |
|---|---|---|
| Jetson→MCU | PKT_SURUCU (0x01) | hız [mm/s] + direksiyon [centideg] |
| Jetson→MCU | PKT_DUR (0x02) | Acil dur |
| Jetson→MCU | PKT_LAZER (0x03) | Lazer ateş |
| Jetson→MCU | PKT_HB (0x04) | Heartbeat |
| Jetson→MCU | PKT_SERVO_PAN (0x05) | Taret yatay |
| Jetson→MCU | PKT_SERVO_TLT (0x06) | Taret dikey |
| MCU→Jetson | PKT_ENC (0x10) | Enkoder (AS5600) |
| MCU→Jetson | PKT_IMU_YP (0x11) | Yaw + Pitch |
| MCU→Jetson | PKT_RC (0x20) | RC kanal verileri |
| MCU→Jetson | PKT_ESTOP_IN (0x22) | Fiziksel e-stop |

### 5.3 `ackermann_converter.py` — Kinematik Dönüştürücü

```
δ = arctan(L × ω / v)

Giriş : /cmd_vel  (geometry_msgs/Twist)
Çıkış : /ackermann_cmd  (ackermann_msgs/AckermannDriveStamped)
```

### 5.4 `terrain_adapter.py` — Arazi Parametre Adaptörü

YOLO tabela tespitine göre Nav2 hız ve costmap parametrelerini anlık günceller. 3 ardışık aynı tespit eşiği sonrası profil değişir.

| Profil | Hız (m/s) | Inflation (m) |
|---|---|---|
| normal | 2.0 | 0.40 |
| wet | 0.8 | 0.55 |
| gravel | 0.7 | 0.45 |
| slope | 0.5 | 0.65 |
| rough | 0.4 | 0.60 |
| slow | 0.3 | 0.40 |
| fast | 3.0 | 0.35 |

### 5.5 `misyon_fsm.py` — Görev Durum Makinesi

```
[IDLE] → [NAVIGATE] → [SHOOT_APPROACH] → [SHOOT] → [NAVIGATE]
              ↓                                           ↓
         [HIZLANMA]                               [MISSION_COMPLETE]
              ↓
         [ERROR_RECOVERY]
```

### 5.6 YOLO Sınıf Tablosu (16 sınıf, alfabetik)

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
| `/scan_raw` | LaserScan | ydlidar_node | scan_relay |
| `/scan_lidar` | LaserScan | scan_relay | SLAM, costmap |
| `/cmd_vel` | Twist | Nav2 RPP | mod_yoneticisi |
| `/mux/cmd_vel` | Twist | mod_yoneticisi | ackermann_converter |
| `/ackermann_cmd` | AckermannDriveStamped | ackermann_converter | seri_kopru |
| `/odometry/filtered` | Odometry | EKF | Nav2, SLAM |
| `/rc_input` | Joy | seri_kopru | mod_yoneticisi |
| `/e_stop` | Bool | e_stop_node | mod_yoneticisi, seri_kopru |
| `/yolo/class_id` | UInt8 | yolo_adapter | terrain_adapter |
| `/ika/detections` | String (JSON) | yolo_adapter | misyon_fsm |
| `/shoot_command` | Bool | misyon_fsm | seri_kopru |

---

## 8. Nav2 Konfigürasyonu

### Gerçek Araç (Ackermann)

```yaml
planner: SmacPlannerHybrid
  motion_model: DUBIN       # Sadece ileri — geri harekete izin vermez
  minimum_turning_radius: 1.75  # Ölçülecek

controller: RegulatedPurePursuitController
  use_rotate_to_heading: false  # Ackermann yerinde dönemez
  desired_linear_vel: 2.0
```

### Test Aracı (Diferansiyel)

```yaml
planner: NavfnPlanner (A*)

controller: RegulatedPurePursuitController
  use_rotate_to_heading: true   # Diferansiyel yerinde döner
  desired_linear_vel: 0.20
  transform_tolerance: 0.5
```

---

## 9. Sensör Füzyonu (EKF)

```
/odom (dead reckoning)   ──▶ EKF ──▶ /odometry/filtered
/imu/data (MPU9250)      ──┘          child_frame: base_footprint
```

Test aracında IMU devre dışı — yalnızca odometri kullanılır.

---

## 10. Görev Durum Makinesi

**FSM Durum Açıklamaları:**

| Durum | Açıklama |
|---|---|
| IDLE | `/mission_start True` bekler |
| NAVIGATE | waypoints.yaml'dan sıradaki hedefe Nav2 ile gider |
| SHOOT_APPROACH | ATIS waypoint'ine yaklaşır, hedef onayı bekler |
| SHOOT | `hedef_hata ≤ ±5px` ise `/shoot_command True` |
| HIZLANMA | Nav2 bypass — direkt /cmd_vel, 10m/s, 30m mesafe |
| ERROR_RECOVERY | Nav2 timeout/hata → geri dönüş |

**Özel Durumlar:**
- `DIK_EGIM_GIRIS/CIKIS`: STOP tabelasında 2s dur (Şartname §6.10)
- `KAYAR_ENGEL`: Yön tespiti için max 10s bekle
- `HIZLANMA`: Tabela_11 → max hız, Tabela_11_son → dur (§6.11)

---

## 11. Parkur Aşamaları

| Sıra | Waypoint | Arazi | Özel |
|---|---|---|---|
| 1 | SULU_YOL | wet | — |
| 2 | TASLI_YOL | gravel | — |
| 3 | YAN_EGIM | slope | — |
| 4 | DIK_ENGEL | obstacle | Nav2 kaçınır |
| 5 | KONİLİ_YOL | normal | — |
| 6 | KAYAR_ENGEL | normal | Yön bekle max 10s |
| 7 | ENGEBELİ_ARAZİ | rough | — |
| 8 | DIK_EGIM_GIRIS | rough | **2s dur** |
| 9 | DIK_EGIM_CIKIS | normal | **2s dur** |
| 10 | ATIS_BOLGESI | slow | Hedef ±5px → ateş |
| 11 | HIZLANMA | fast | 10m/s, 30m |

> ⚠️ `waypoints.yaml` koordinatları 0.0 placeholder — saha haritasından güncellenecek.

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

# Workspace build
cd ~/ika_ws
colcon build --packages-select teknofest_ika --symlink-install
source install/setup.bash
echo "source ~/ika_ws/install/setup.bash" >> ~/.bashrc
```

---

## 13. Çalıştırma

### Gerçek Araç

```bash
# Harita alma (sahada, ilk çalıştırma)
ros2 launch teknofest_ika gercek_harita.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/ika_ws/maps/gercek_harita

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

## 14. Test Aracı

DC motorlu diferansiyel test aracı — gerçek araç yazılımını doğrulamak için kullanılır.

| Özellik | Değer |
|---|---|
| MCU | Arduino UNO (ATmega328P) |
| Motor sürücü | L298N Mini |
| Baud rate | 500.000 bps |
| Planner | NavfnPlanner (A*) |
| Controller | RegulatedPurePursuitController |
| Hız | 0.20 m/s |

```bash
# Test aracı başlatma
ros2 launch teknofest_ika test_arac.launch.py

# Harita alma (test_harita.pgm yoksa otomatik mapping moduna girer)
# Harita varsa → localization modunda başlar

# Arduino firmware derleme ve yükleme (Jetson'da)
cd ~/ika_ws/arduino
avr-gcc -mmcu=atmega328p -std=gnu99 -Os firmware.c -o fw.elf
avr-objcopy -O ihex fw.elf fw.hex
avrdude -p atmega328p -c arduino -P /dev/ttyACM0 -b 115200 -U flash:w:fw.hex:i
```

---

## 15. Bekleyen Kalibrasyonlar

| Görev | Dosya | Parametre |
|---|---|---|
| Wheelbase ölçümü | `ackermann_converter.py` | `WHEELBASE` |
| Maks. steer açısı | `ackermann_converter.py` | `MAX_STEER_ANGLE` |
| Min. dönüş yarıçapı | `nav2_params.yaml` | `minimum_turning_radius` |
| Parkur waypoint koordinatları | `waypoints.yaml` | Tüm x/y değerleri |
| Enkoder ölçeği | `seri_kopru.py` | `WHEEL_RADIUS`, `TRACK_WIDTH` |
| TensorRT engine | `models/best.engine` | `export_tensorrt.py` ile üret |

---

*Takım: MAGNESIA | LYDİA İKA | Teknofest 2026*
