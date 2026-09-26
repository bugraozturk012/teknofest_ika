# Sistem mimarisi

> LYDİA İKA otonom sürüş yazılımı — ayrıntılı belge.
> Genel bakış için [README](../README.md).

---

## Sisteme Genel Bakış

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

## Yazılım Mimarisi

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

## TF Zinciri

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

## Topic Arayüzleri

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
