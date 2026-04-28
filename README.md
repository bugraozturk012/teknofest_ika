# LYDİA İKA — Teknofest Otomasyon Stack

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
7. [Topic ve Servis Arayüzleri](#7-topic-ve-servis-arayüzleri)
8. [Nav2 Konfigürasyonu](#8-nav2-konfigürasyonu)
9. [Sensör Füzyonu (EKF)](#9-sensör-füzyonu-ekf)
10. [Arazi Adaptörü](#10-arazi-adaptörü)
11. [Görev Durum Makinesi](#11-görev-durum-makinesi)
12. [Parkur Aşamaları](#12-parkur-aşamaları)
13. [Kurulum](#13-kurulum)
14. [Çalıştırma](#14-çalıştırma)
15. [Test Prosedürleri](#15-test-prosedürleri)
16. [Bekleyen Kalibrasyonlar](#16-bekleyen-kalibrasyonlar)

---

## 1. Sisteme Genel Bakış

LYDİA, Teknofest İnsansız Kara Aracı (İKA) yarışması için geliştirilmiş tam otonom bir kara aracıdır. Sistem; görüntü tabanlı tabela tespiti, SLAM ile eş zamanlı harita oluşturma ve lokalizasyon, Nav2 ile otonom navigasyon ve SMACH durum makinesi ile görev yönetimini entegre eder.

```
┌─────────────────────────────────────────────────────────────────┐
│                        JETson Orin Nano                          │
│                                                                  │
│  ┌───────────┐   ┌──────────────┐   ┌─────────────────────────┐ │
│  │ SLAM      │   │ EKF          │   │ Nav2 Stack              │ │
│  │ Toolbox   │──▶│ (odom+IMU)   │──▶│ SmacPlanner + RPP       │ │
│  └───────────┘   └──────────────┘   └────────────┬────────────┘ │
│       ▲                                           │ /cmd_vel     │
│       │                                           ▼             │
│  ┌────┴──────┐   ┌──────────────┐   ┌─────────────────────────┐ │
│  │ LiDAR     │   │ misyon_fsm   │   │ ackermann_converter     │ │
│  │ Tmini Pro │   │ (SMACH FSM)  │   │ cmd_vel→AckermannDrive  │ │
│  └───────────┘   └──────┬───────┘   └────────────┬────────────┘ │
│                         │                         │             │
│  ┌───────────┐   ┌──────┴───────┐   ┌─────────────────────────┐ │
│  │ terrain   │   │ veri_paketi  │   │ seri_kopru              │ │
│  │ adapter   │   │ (video kayıt)│   │ (binary protokol)       │ │
│  └───────────┘   └──────────────┘   └────────────┬────────────┘ │
│       ▲                                           │ USB Seri    │
│       │ /yolo/class_id                            ▼             │
└───────┼───────────────────────────────────────────┼─────────────┘
        │                                           │
   ┌────┴──────┐                          ┌─────────┴──────────┐
   │  YOLO     │                          │  Arduino Mega 2560  │
   │ (Görüntü  │                          │  + Nano (IMU/servo) │
   │  Ekibi)   │                          │  VESC Flipsky 75100 │
   └───────────┘                          └────────────────────┘
```

---

## 2. Donanım

| Bileşen | Model / Değer |
|---|---|
| Ana işlemci | NVIDIA Jetson Orin Nano |
| Motor sürücü | VESC Flipsky 75100 |
| Ana MCU | Arduino Mega 2560 |
| IMU/Servo MCU | Arduino Nano |
| LiDAR | YDLidar Tmini Pro (2D, 360°) |
| Kamera | IMX258 (×3: ileri, geri, nişan) |
| IMU | MPU9250 (6-eksen) |
| Enkoder | AS5600 manyetik enkoder (10-bit, 0–1023) |
| İletişim | USB Seri — 115200 baud, 8N1 |
| Kinematik | Ackermann (otomobil tipi direksiyon) |

### Ackermann Kinematik Parametreleri

```
Wheelbase (L)          : ölçülecek [m]  ← Araç fiziksel hazır olunca
Maks. direksiyon açısı : ölçülecek [rad]
TICKS_PER_REV          : 1024 (AS5600 10-bit ADC)
```

> ⚠️ Bu parametreler `ackermann_converter.py` ve `nav2_params.yaml` içindeki
> `min_turning_radius` hesabını doğrudan etkiler. Araç hazır olunca ölçülmeli.

---

## 3. Yazılım Mimarisi

### Teknoloji Yığını

| Katman | Teknoloji |
|---|---|
| İşletim sistemi | Ubuntu 22.04 LTS |
| Robot framework | ROS2 Humble |
| Simülasyon | Gazebo (diff_drive plugin) |
| Navigasyon | Nav2 (SmacPlannerHybrid + RPP) |
| Lokalizasyon | SLAM Toolbox (async modu) + EKF |
| Görev yönetimi | SMACH FSM |
| Nesne tespiti | YOLO (görüntü ekibi — ayrı pipeline) |
| Veri kaydı | OpenCV + cv_bridge (MP4) |
| MCU haberleşme | Binary seri protokol (8-byte sabit paket) |

### Simülasyon vs. Gerçek Araç

Gazebo'da diff_drive plugin kullanılır (Ackermann plugin mevcut değil). Gerçek araçta `ackermann_converter.py`, Nav2'nin ürettiği `Twist` komutlarını `AckermannDriveStamped`'e çevirir.

```
Simülasyon : Nav2 → /cmd_vel → Gazebo diff_drive
Gerçek araç: Nav2 → /cmd_vel → ackermann_converter → /ackermann_cmd → seri_kopru → Arduino
```

---

## 4. ROS2 Paket Yapısı

```
~/ika_ws/
└── src/
    └── teknofest_ika/
        ├── teknofest_ika/          # Python node'ları
        │   ├── topics.py           # Topic sabit tanımları
        │   ├── misyon_fsm.py       # SMACH görev durum makinesi
        │   ├── terrain_adapter.py  # YOLO → Nav2 parametre adaptörü
        │   ├── veri_paketi.py      # Kamera video kaydedici
        │   ├── ackermann_converter.py  # cmd_vel → AckermannDrive
        │   └── seri_kopru.py       # Arduino binary seri köprü
        ├── launch/
        │   ├── baslat.launch.py    # Simülasyon ana launch
        │   ├── gercek_arac.launch.py  # Gerçek araç launch
        │   ├── nav2.launch.py      # Nav2 stack launch
        │   └── slam.launch.py      # SLAM Toolbox launch
        ├── config/
        │   ├── nav2_params.yaml    # Nav2 planner/controller parametreleri
        │   ├── ekf_params.yaml     # EKF sensör füzyon parametreleri
        │   ├── ekf.yaml            # EKF ek konfigürasyon
        │   ├── waypoints.yaml      # Parkur waypoint koordinatları
        │   └── mapper_params_online_sync.yaml  # SLAM Toolbox konfigürasyonu
        ├── urdf/
        │   └── arac.urdf           # Araç model tanımı
        ├── maps/                   # Kaydedilen SLAM haritaları
        ├── package.xml
        └── setup.py
```

---

## 5. Node'lar ve Görevleri

### 5.1 `seri_kopru.py` — Arduino ↔ ROS2 Köprüsü

Jetson ile Arduino arasındaki binary haberleşmeyi yönetir.

**Binary Protokol (8 byte sabit paket):**
```
[0xAA] [CMD] [D0] [D1] [D2] [D3] [CRC] [0x55]
CRC = XOR(CMD ^ D0 ^ D1 ^ D2 ^ D3)
Baud: 115200, Big-endian
```

| Yön | Komut | Açıklama |
|---|---|---|
| Jetson → MCU | `PKT_SURUCU (0x01)` | int16 hız [mm/s] + int16 yaw [1/100°] |
| Jetson → MCU | `PKT_DUR (0x02)` | Acil dur |
| Jetson → MCU | `PKT_LAZER (0x03)` | Lazer aç/kapat |
| Jetson → MCU | `PKT_HB (0x04)` | Heartbeat |
| MCU → Jetson | `PKT_ENC (0x10)` | Sol/sağ enkoder (AS5600 ADC) |
| MCU → Jetson | `PKT_IMU_YP (0x11)` | Yaw + Pitch [1/10°] |
| MCU → Jetson | `PKT_IMU_R (0x12)` | Roll [1/10°] |

---

### 5.2 `ackermann_converter.py` — Kinematik Dönüştürücü

Nav2'nin ürettiği `geometry_msgs/Twist` mesajını Ackermann direksiyon açısına çevirir.

**Bisiklet modeli (single-track model):**
```
δ = arctan(L × ω / v)

v = Twist.linear.x   [m/s]   — ileri hız
ω = Twist.angular.z  [rad/s] — dönüş açısal hızı
L = dingil arası     [m]     — wheelbase (ölçülecek)
δ = direksiyon açısı [rad]
```

**Özel durum:** `v ≈ 0, ω ≠ 0` → Ackermann yerinde dönemez. Araç durur, direksiyon maksimuma alınır, Nav2 yeniden plan üretir.

**Topic'ler:**
```
Giriş : /cmd_vel          (geometry_msgs/Twist)
Çıkış : /ackermann_cmd    (ackermann_msgs/AckermannDriveStamped)
```

---

### 5.3 `terrain_adapter.py` — Arazi Parametre Adaptörü

YOLO'nun tabela tespitine göre Nav2 parametrelerini gerçek zamanlı günceller. Node yeniden başlatılmaz — `/set_parameters` RPC ile anlık etki eder.

**Ardışık Frame Filtresi:**
Yanlış pozitif tespitlere karşı: aynı `class_id` art arda 3 kez gelmeden profil uygulanmaz.

**Class ID → Arazi Eşleşmesi:**

| class_id | Tabela | Profil |
|---|---|---|
| 0 | su_gecisi | wet |
| 1 | tasli_yol | gravel |
| 2 | yan_egim | slope |
| 3 | dik_engel | obstacle |
| 4 | trafik_koni | normal |
| 5 | kayar_engel | normal |
| 6 | dik_egim | rough |
| 7 | atis | slow |
| 8 | hizlanma | fast |
| 255 | tespit yok | normal |

**Arazi Profilleri (Nav2 RPP Parametreleri):**

| Profil | Hız (m/s) | Lookahead (m) | Max Angular Accel | Inflation (m) |
|---|---|---|---|---|
| normal | 2.0 | 1.5 | 3.2 | 0.40 |
| wet | 0.8 | 1.0 | 1.0 | 0.55 |
| gravel | 0.7 | 0.9 | 1.5 | 0.45 |
| slope | 0.5 | 0.7 | 1.0 | 0.65 |
| obstacle | 1.2 | 1.2 | 2.0 | 0.50 |
| rough | 0.4 | 0.6 | 1.0 | 0.60 |
| slow | 0.3 | 0.5 | 0.8 | 0.40 |
| fast | 3.0 | 2.5 | 3.2 | 0.35 |

**Lookahead formülü (RPP temel denklemi):**
```
L_d = k_ld × v    (minimum lookahead: v × 0.5s)
```

**Topic'ler:**
```
Giriş : /yolo/class_id   (std_msgs/UInt8, BEST_EFFORT QoS)
Çıkış : /controller_server/set_parameters    (RPC)
        /local_costmap/local_costmap/set_parameters  (RPC)
```

---

### 5.4 `misyon_fsm.py` — Görev Durum Makinesi

SMACH tabanlı 6 durumlu FSM, parkur boyunca görev sırasını yönetir.

**Durum Diyagramı:**
```
[IDLE] ──▶ [NAVIGATE] ──▶ [SHOOT_APPROACH] ──▶ [SHOOT] ──▶ [NAVIGATE]
                │                                                 │
                ▼                                                 ▼
          [ERROR_RECOVERY] ◀───────────────────────────────────── ┘
                │
                ▼
          [MISSION_COMPLETE]
```

**Durum Açıklamaları:**

| Durum | Açıklama |
|---|---|
| IDLE | Başlangıç. `/mission_start True` bekler |
| NAVIGATE | `waypoints.yaml`'dan sıradaki hedefe Nav2 ile gider |
| SHOOT_APPROACH | ATIS waypoint'ine yaklaşır, görüntü ekibinden hedef onayı bekler |
| SHOOT | `hedef_hata_x/y ≤ ±5px` ise `/shoot_command True` yayınlar |
| ERROR_RECOVERY | Nav2 timeout/hata durumunda geri dönüş |
| MISSION_COMPLETE | Tüm aşamalar tamamlandı |

**Özel Durum Mantığı:**
- `DIK_EGIM_GIRIS / CIKIS`: Waypoint'e varınca **2 saniye dur** (Şartname gereği)
- `KAYAR_ENGEL`: `kayar_yon != 'bilinmiyor'` olana kadar max 10s bekle
- `ATIS`: Görüntü ekibinden `hedef_hata_x/y ±5px` toleransı gelmeden ateşleme

**Topic'ler:**
```
Giriş : /mission_start     (std_msgs/Bool)
Giriş : /ika/detections    (std_msgs/String — JSON)
Çıkış : Nav2 NavigateToPose action
Çıkış : /shoot_command     (std_msgs/Bool)
Çıkış : /misyon/aktif      (std_msgs/Bool)
```

**Görüntü ekibinden beklenen JSON formatı (`/ika/detections`):**
```json
{
  "tabela": 3,
  "hedef_var": true,
  "hedef_hata_x": 2.0,
  "hedef_hata_y": 1.0,
  "kayar_yon": "sol",
  "fps": 30
}
```

---

### 5.5 `veri_paketi.py` — Video Kaydedici (Şartname 6.14)

Parkur boyunca 3 kameradan eş zamanlı MP4 kaydı alır. Hakem heyeti için `meta.txt` dosyası oluşturur.

**Kamera Topic'leri (URDF ile eşleştirilmiş):**
```
/ileri_kamera/image_raw   — İleri sürüş kamerası
/geri_kamera/image_raw    — Geri sürüş kamerası
/nisan_kamera/image_raw   — Nişan kamerası
```

**Çıktı Dizini:**
```
~/ika_kayitlar/<YYYY-MM-DD_HH-MM-SS>/
    ileri.mp4
    geri.mp4
    nisan.mp4
    meta.txt    ← hakem heyeti için: süre, FPS, kare sayısı, topic adları
```

**Otomatik entegrasyon:** `misyon_fsm.py` `/misyon/aktif True` yayınladığında kayıt otomatik başlar.

---

## 6. TF Zinciri

```
map
 └── odom           ← SLAM Toolbox yayınlar
      └── base_footprint   ← EKF (robot_localization) yayınlar
           └── base_link
                ├── lidar_link     ← YDLidar Tmini Pro
                ├── imu_link       ← MPU9250
                ├── ileri_kamera_joint
                ├── geri_kamera_joint
                └── nisan_kamera_joint
```

**Kritik konfigürasyon:**
- `ekf_params.yaml`: `base_link_frame: base_footprint` (Nav2 uyumluluğu için)
- `slam_toolbox`: `base_frame: base_footprint`
- `nav2_params.yaml`: `robot_base_frame: base_footprint`

---

## 7. Topic ve Servis Arayüzleri

### Ekipler Arası Protokol

| Topic | Tip | Yayıncı | Abone |
|---|---|---|---|
| `/yolo/class_id` | `std_msgs/UInt8` | Görüntü Ekibi | terrain_adapter |
| `/ika/detections` | `std_msgs/String` (JSON) | Görüntü Ekibi | misyon_fsm |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 RPP | ackermann_converter, Gazebo |
| `/ackermann_cmd` | `AckermannDriveStamped` | ackermann_converter | seri_kopru |
| `/shoot_command` | `std_msgs/Bool` | misyon_fsm | seri_kopru |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF | Nav2, SLAM |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR sürücüsü | Nav2, SLAM |
| `/misyon/aktif` | `std_msgs/Bool` | misyon_fsm | veri_paketi |
| `/veri_paketi/kayit_durumu` | `std_msgs/Bool` | veri_paketi | — |

### QoS Profilleri

| Topic | QoS |
|---|---|
| `/yolo/class_id` | BEST_EFFORT / VOLATILE |
| `/scan` | BEST_EFFORT / VOLATILE |
| `/cmd_vel` | RELIABLE / VOLATILE |
| `/shoot_command` | RELIABLE |
| `/odometry/filtered` | RELIABLE |

---

## 8. Nav2 Konfigürasyonu

### Planner: SmacPlannerHybrid

```yaml
motion_model_for_search: "DUBIN"     # ÖNEMLI: REEDS_SHEPP → geri gider!
                                      # DUBIN: sadece ileri hareket
minimum_turning_radius: 1.75         # Ackermann min dönüş yarıçapı [m]
angle_quantization_bins: 72          # 5° çözünürlük
```

**DUBIN vs REEDS_SHEPP:**
- REEDS_SHEPP: {İleri, Geri} × {Sol, Düz, Sağ} — 48 yay tipi, geri harekete izin verir
- DUBIN: {İleri} × {Sol, Düz, Sağ} — CSC/CCC tipleri, yalnızca ileri
- Ackermann araç geri gidemeyeceği için DUBIN zorunludur

### Controller: RegulatedPurePursuit (RPP)

```yaml
desired_linear_vel: 2.0
lookahead_dist: 1.5
max_angular_accel: 3.2              # DIKKAT: max_accel/max_decel RPP'de yok!
use_rotate_to_heading: false        # Ackermann yerinde dönemez → false zorunlu
```

### Costmap

```yaml
obstacle_min_range: 0.25    # LiDAR kendi gövdesini algılamasın (min 0.10m + marj)
raytrace_min_range: 0.10
inflation_radius: 0.40      # terrain_adapter tarafından runtime'da güncellenir
```

---

## 9. Sensör Füzyonu (EKF)

`robot_localization` paketi, odometri ve IMU verilerini Kalman filtresi ile birleştirir.

```
/odom        (nav_msgs/Odometry)      ─┐
                                       ├──▶ EKF ──▶ /odometry/filtered
/imu/data    (sensor_msgs/Imu)        ─┘           (nav_msgs/Odometry)
                                                     child_frame: base_footprint
```

**Füzyon matrisleri:**
- Odometri: `[x, y, yaw, vx, vy, vyaw]` → pozisyon + hız
- IMU: `[roll, pitch, yaw, ax, ay, az, vyaw]` → oryantasyon + ivme

---

## 10. Arazi Adaptörü

Bkz. [5.3 terrain_adapter.py](#53-terrain_adapterpy--arazi-parametre-adaptörü)

**Test (Nav2 çalışıyorken):**
```bash
# WET profili test — 5 mesaj, 3'ü ardışık eşiği geçer
python3 -c "
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8
rclpy.init()
node = Node('test_pub')
qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
pub = node.create_publisher(UInt8, '/yolo/class_id', qos)
time.sleep(0.5)
for i in range(5):
    msg = UInt8(); msg.data = 0   # 0 = su_gecisi → wet
    pub.publish(msg); time.sleep(0.3)
node.destroy_node(); rclpy.shutdown()
"
```

**Beklenen çıktı:**
```
[INFO] [terrain_adapter]: class_id=0 → WET (önceki: normal)
[INFO] [terrain_adapter]: controller_server → wet ✓
[INFO] [terrain_adapter]: local_costmap/local_costmap → wet ✓
```

---

## 11. Görev Durum Makinesi

Bkz. [5.4 misyon_fsm.py](#54-misyon_fsmpy--görev-durum-makinesi)

**Test:**
```bash
# Görüntü ekibi olmadan FSM testi
ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
ros2 topic pub /ika/detections std_msgs/msg/String \
  'data: "{\"tabela\":9,\"hedef_var\":true,\"hedef_hata_x\":2.0,\"hedef_hata_y\":1.0,\"kayar_yon\":\"sol\",\"fps\":30}"' \
  --rate 10
```

---

## 12. Parkur Aşamaları

`waypoints.yaml` dosyasında tanımlıdır. Koordinatlar placeholder — harita çekildikten sonra RViz2 ile güncellenmeli.

| Sıra | Aşama | Arazi Profili | Pas Geçilebilir | Özel Durum |
|---|---|---|---|---|
| 1 | SU_GECISI | wet | Hayır | — |
| 2 | TASLI_YOL | gravel | Hayır | — |
| 3 | YAN_EGIM | slope | Hayır | — |
| 4 | DIK_ENGEL | obstacle | Evet | Nav2 kaçınır |
| 5 | TRAFIK_KON | normal | Hayır | — |
| 6 | KAYAR_ENGEL | normal | Evet | Görüntüden yön bekle (max 10s) |
| 7 | DIK_EGIM_GIRIS | rough | Hayır | **2s dur (Şartname)** |
| 8 | DIK_EGIM_CIKIS | rough | Hayır | **2s dur (Şartname)** |
| 9 | ATIS | slow | Hayır | Hedef ±5px tolerans → ateş |
| 10 | HIZLANMA | fast | Hayır | Maksimum hız |

**Waypoint koordinatlarını güncelleme:**
```bash
# RViz2'de haritayı aç → "Publish Point" aracıyla koordinatları oku
ros2 run rviz2 rviz2
# Fixed Frame: map | Add → By Topic → /map
# "Publish Point" → tıkla → terminal'de koordinatları gör
```

---

## 13. Kurulum

### Gereksinimler

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-ackermann-msgs \
  ros-humble-smach \
  ros-humble-smach-ros \
  ros-humble-cv-bridge

pip3 install pyserial opencv-python --break-system-packages
```

### Workspace Kurulumu

```bash
mkdir -p ~/ika_ws/src
cd ~/ika_ws/src
# Paketi buraya kopyala / git clone yap
cd ~/ika_ws
colcon build --symlink-install
source install/setup.bash

# Otomatik source için:
echo "source ~/ika_ws/install/setup.bash" >> ~/.bashrc
```

---

## 14. Çalıştırma

### Simülasyon (Gazebo)

```bash
# Terminal 1 — Gazebo + tüm node'lar
ros2 launch teknofest_ika baslat.launch.py

# Terminal 2 — Nav2
ros2 launch teknofest_ika nav2.launch.py use_sim_time:=true

# Terminal 3 — Misyon başlat
ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
```

### Gerçek Araç

```bash
# Terminal 1 — Tüm node'lar (gerçek araç modu)
ros2 launch teknofest_ika gercek_arac.launch.py

# Terminal 2 — Nav2 (gerçek zaman)
ros2 launch teknofest_ika nav2.launch.py use_sim_time:=false

# Terminal 3 — SLAM
ros2 launch teknofest_ika slam.launch.py use_sim_time:=false

# Terminal 4 — Misyon başlat
ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
```

### Tekil Node Çalıştırma

```bash
source ~/ika_ws/install/setup.bash

ros2 run teknofest_ika terrain_adapter
ros2 run teknofest_ika misyon_fsm
ros2 run teknofest_ika veri_paketi
ros2 run teknofest_ika ackermann_converter
ros2 run teknofest_ika seri_kopru
```

---

## 15. Test Prosedürleri

### Build ve Import Testi
```bash
cd ~/ika_ws && colcon build --symlink-install 2>&1 | grep -E "error:|warning:|Finished"
python3 -c "import teknofest_ika; print('OK')"
```

### TF Zinciri Testi
```bash
ros2 run tf2_tools view_frames
# Beklenen: map → odom → base_footprint → base_link → lidar_link
```

### EKF Testi
```bash
ros2 topic echo /odometry/filtered --once | grep child_frame_id
# Beklenen: child_frame_id: 'base_footprint'
```

### Ackermann Kinematik Testi
```bash
# v=1.0 m/s, ω=0.3 rad/s → δ = arctan(L × 0.3 / 1.0)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.3}}" --once
ros2 topic echo /ackermann_cmd --once
```

### Terrain Adapter + Nav2 Entegrasyon Testi
```bash
# Nav2 çalışıyorken:
ros2 run teknofest_ika terrain_adapter
# Ayrı terminalde WET class_id gönder (3+ kez):
# Beklenti: controller_server → wet ✓  |  local_costmap/local_costmap → wet ✓
```

### Video Kaydı Testi
```bash
ros2 run teknofest_ika veri_paketi
ros2 topic pub /veri_paketi/kayit_baslat std_msgs/msg/Bool "data: true" --once
# 5 saniye bekle
ros2 topic pub /veri_paketi/kayit_baslat std_msgs/msg/Bool "data: false" --once
ls ~/ika_kayitlar/
```

---

## 16. Bekleyen Kalibrasyonlar

Araç fiziksel olarak hazır olduğunda yapılması gereken ölçüm ve güncellemeler:

| Görev | Dosya | Parametre |
|---|---|---|
| Wheelbase ölçümü | `ackermann_converter.py` | `WHEELBASE` sabiti |
| Maks. direksiyon açısı | `ackermann_converter.py` | `MAX_STEER_ANGLE` sabiti |
| Min. dönüş yarıçapı | `nav2_params.yaml` | `minimum_turning_radius` |
| Parkur waypoint koordinatları | `waypoints.yaml` | Tüm x/y değerleri |
| Enkoder ölçeği | `seri_kopru.py` | `WHEEL_RADIUS`, `TRACK_WIDTH` |

**Wheelbase ölçüm formülü:**
```
L = ön dingil merkezi → arka dingil merkezi arası mesafe [m]
Min. dönüş yarıçapı ≈ L / tan(δ_max)
```

---

*Son güncelleme: 28 Nisan 2026 | Takım: MAGNESIA | LYDİA İKA*
