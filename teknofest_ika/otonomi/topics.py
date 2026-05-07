"""
LYDİA — Ortak ROS2 Topic Sabitleri
====================================
Takım    : MAGNESIA
Araç     : LYDİA
Yarışma  : TEKNOFEST 2026 İKA
Başvuru  : 4908667

Bu dosya üç ekip tarafından import edilir:
  - ika_otonomi  → SLAM, Nav2, EKF, FSM node'ları
  - ika_gorsel   → YOLO, HSV, Kalman node'ları
  - ika_gomulu   → Seri köprü node'ları (Jetson tarafı)

KURAL: Topic adı değiştirilecekse bu dosyada değiştirilir,
       node kodlarına dokunulmaz.
"""

# ─────────────────────────────────────────────
# SENSÖR TOPIC'LERİ
# Üretici: Gömülü Sistemler (Arduino → Jetson seri köprü)
# Tüketen: Otomasyon (EKF, SLAM, Nav2)
# ─────────────────────────────────────────────

# YDLidar Tmini Pro — 360° LaserScan
# Mesaj tipi : sensor_msgs/LaserScan
# Frekans    : ~10 Hz
# Frame      : laser_frame
SCAN_TOPIC = "/scan"

# YDLidar OS30A — 3D PointCloud2
# Mesaj tipi : sensor_msgs/PointCloud2
# Frekans    : ~10 Hz
# Frame      : os30a_frame
POINTCLOUD_TOPIC = "/pointcloud"

# MPU9250 IMU — 9 eksen (Madgwick filtreli)
# Mesaj tipi : sensor_msgs/Imu
# Frekans    : 50 Hz
# Frame      : imu_link
IMU_TOPIC = "/imu/data"

# AS5600 Enkoder — Tekerlek odometresi
# Mesaj tipi : nav_msgs/Odometry
# Frekans    : 30 Hz
# Frame      : odom → base_link
ODOM_TOPIC = "/odom"

# EKF çıktısı — Füzyon sonrası hassas konum
# Mesaj tipi : nav_msgs/Odometry
# Frekans    : 30 Hz
# Üretici    : robot_localization ekf_node
EKF_ODOM_TOPIC = "/odometry/filtered"

# ─────────────────────────────────────────────
# KAMERA TOPIC'LERİ
# Üretici: Gömülü Sistemler (USB bağlantı)
#          veya Görüntü İşleme (usb_cam node)
# ─────────────────────────────────────────────

# WaveShare IMX258 Ana Kamera — 13MP, OIS
# Mesaj tipi : sensor_msgs/Image
# Frekans    : 30 Hz
# Frame      : camera_link
CAMERA_IMAGE_TOPIC = "/camera/image_raw"

# Kamera kalibrasyon bilgisi
# Mesaj tipi : sensor_msgs/CameraInfo
CAMERA_INFO_TOPIC = "/camera/camera_info"

# Microcase 720P — Ön manevra kamerası
CAMERA_FRONT_TOPIC = "/camera/front/image_raw"

# Microcase 720P — Arka manevra kamerası
CAMERA_REAR_TOPIC = "/camera/rear/image_raw"

# Microcase 720P — Nişan kamerası (taret üzeri)
CAMERA_TARET_TOPIC = "/camera/taret/image_raw"

# ─────────────────────────────────────────────
# GÖRÜNTÜ İŞLEME TOPIC'LERİ
# Üretici: Görüntü İşleme (YOLO, HSV node'ları)
# Tüketen: Otomasyon (FSM, Nav2 costmap)
# ─────────────────────────────────────────────

# YOLO tespit sonuçları
# Mesaj tipi : vision_msgs/Detection2DArray
# Frekans    : ~25 Hz (TensorRT FP16)
DETECTIONS_TOPIC = "/detections"

# YOLO → FSM tetikleme sinyali
# Mesaj tipi : std_msgs/String
# İçerik     : "SU_GECISI" | "TASLI_YOL" | "YAN_EGIM" |
#              "DIK_ENGEL" | "TRAFIK_KON" | "KAYAR_ENGEL" |
#              "DIK_EGIM" | "ATIS" | "HIZLANMA"
YOLO_TRIGGER_TOPIC = "/yolo/trigger"

# Debug görüntüsü — YOLO bounding box overlay
# Mesaj tipi : sensor_msgs/Image
DEBUG_IMAGE_TOPIC = "/debug/image"

# Atış hedefi merkez koordinatı (piksel)
# Mesaj tipi : geometry_msgs/Point (x=px, y=py, z=0)
TARGET_CENTER_TOPIC = "/target/center_px"

# Trafik konisi 3D konumu (LiDAR füzyon sonrası)
# Mesaj tipi : geometry_msgs/PoseArray
CONE_POSITIONS_TOPIC = "/cone_positions"

# Kayar engel Kalman tahmini
# Mesaj tipi : geometry_msgs/Point (x=konum, y=hız, z=0)
MOVING_OBS_TOPIC = "/moving_obs/prediction"

# ─────────────────────────────────────────────
# NAVİGASYON TOPIC'LERİ
# Üretici: Otomasyon (Nav2, SLAM node'ları)
# ─────────────────────────────────────────────

# Nav2 hareket komutu → Arduino Mega → VESC
# Mesaj tipi : geometry_msgs/Twist
# Frekans    : 20 Hz
# Format     : linear.x [m/s], angular.z [rad/s]
CMD_VEL_TOPIC = "/cmd_vel"

# SLAM Toolbox haritası
# Mesaj tipi : nav_msgs/OccupancyGrid
MAP_TOPIC = "/map"

# Araç anlık konumu (SLAM lokalizasyon)
# Mesaj tipi : geometry_msgs/PoseWithCovarianceStamped
POSE_TOPIC = "/pose"

# Hız sınırı override (engebeli arazi, yan eğim)
# Mesaj tipi : std_msgs/Float32  (m/s)
SPEED_LIMIT_TOPIC = "/speed_limit"

# ─────────────────────────────────────────────
# FSM TOPIC'LERİ
# Üretici: Otomasyon (FSM node)
# Tüketen: Tüm ekipler (durum izleme)
# ─────────────────────────────────────────────

# FSM aktif state
# Mesaj tipi : std_msgs/String
# Değerler   : "IDLE" | "MANUAL" | "AUTONOMOUS" |
#              "SU_GECISI" | "TASLI_YOL" | "YAN_EGIM" |
#              "DIK_ENGEL" | "TRAFIK_KON" | "KAYAR_ENGEL" |
#              "DIK_EGIM" | "RAMP_STOP" | "ATIS" |
#              "HIZLANMA" | "FINISH" | "EMERGENCY"
FSM_STATE_TOPIC = "/fsm_state"

# ─────────────────────────────────────────────
# TARET TOPIC'LERİ
# Üretici: Otomasyon (Atış PID node)
# Tüketen: Gömülü Sistemler (Arduino Nano → MG958/MG996R)
# ─────────────────────────────────────────────

# Taret pan açısı — MG958 servo
# Mesaj tipi : std_msgs/Float32  (derece, -90..+90)
TARET_PAN_TOPIC = "/taret/pan"

# Taret tilt açısı — MG996R servo
# Mesaj tipi : std_msgs/Float32  (derece, -45..+45)
TARET_TILT_TOPIC = "/taret/tilt"

# Atış sonucu
# Mesaj tipi : std_msgs/Bool  (True = ateş edildi)
SHOOT_RESULT_TOPIC = "/shoot/result"

# ─────────────────────────────────────────────
# GÜVENLİK TOPIC'LERİ
# Üretici: Gömülü Sistemler (Schneider XB5, Flysky)
# Tüketen: Otomasyon (FSM → EMERGENCY state)
# ─────────────────────────────────────────────

# Acil stop butonu (Schneider XB5)
# Mesaj tipi : std_msgs/Bool  (True = aktif, dur!)
E_STOP_TOPIC = "/e_stop"

# Batarya durumu (INA219 + Daly BMS)
# Mesaj tipi : sensor_msgs/BatteryState
# Frekans    : 1 Hz
BATTERY_TOPIC = "/battery/status"

# Sensör arıza bildirimi
# Mesaj tipi : std_msgs/String
# İçerik     : "IMU" | "ENC" | "BMS" | "LASER" | "SERVO" | "LIDAR"
SENSOR_FAULT_TOPIC = "/sensor/fault"

# Eğim güvenlik durumu (yan eğim + engebeli arazi)
# Mesaj tipi : std_msgs/Bool  (True = güvensiz, yavaşla)
SLOPE_SAFETY_TOPIC = "/slope_safety"

# ─────────────────────────────────────────────
# KAYIT TOPIC'LERİ  §6.14
# Üretici: Otomasyon (kayıt servisi)
# Tüketen: Gömülü Sistemler (LED göstergesi)
# ─────────────────────────────────────────────

# Kayıt başlat/durdur servisi
# Servis tipi : std_srvs/SetBool
RECORD_START_SERVICE = "/record/start"
RECORD_STOP_SERVICE  = "/record/stop"

# ─────────────────────────────────────────────
# SERİ PORT ADRESLERI (udev kurallarıyla sabit)
# ─────────────────────────────────────────────

SERIAL_LIDAR   = "/dev/lidar"        # YDLidar Tmini Pro
SERIAL_IMU     = "/dev/imu_arduino"  # Arduino Nano (IMU + servo + lazer)
SERIAL_ODOM    = "/dev/odom_arduino" # Arduino Mega (enkoder + VESC)
SERIAL_BAUD    = 115200

# ─────────────────────────────────────────────
# FRAME ID'LERİ (TF tree)
# ─────────────────────────────────────────────

FRAME_MAP        = "map"
FRAME_ODOM       = "odom"
FRAME_BASE_LINK  = "base_link"
FRAME_LASER      = "lidar_link"
FRAME_CAMERA     = "camera_link"
FRAME_IMU        = "imu_link"

# ─────────────────────────────────────────────
# YOLO SINIF İSİMLERİ (tabela sınıfları)
# ─────────────────────────────────────────────

CLASS_SU_GECISI   = "su_gecisi"
CLASS_TASLI_YOL   = "tasli_yol"
CLASS_YAN_EGIM    = "yan_egim"
CLASS_DIK_ENGEL   = "dik_engel"
CLASS_TRAFIK_KON  = "trafik_konisi"
CLASS_KAYAR_ENGEL = "kayar_engel"
CLASS_DIK_EGIM    = "dik_egim"
CLASS_ATIS        = "atis"
CLASS_HIZLANMA    = "hizlanma"

YOLO_CLASSES = [
    CLASS_SU_GECISI,
    CLASS_TASLI_YOL,
    CLASS_YAN_EGIM,
    CLASS_DIK_ENGEL,
    CLASS_TRAFIK_KON,
    CLASS_KAYAR_ENGEL,
    CLASS_DIK_EGIM,
    CLASS_ATIS,
    CLASS_HIZLANMA,
]

# YOLO → FSM state eşleme tablosu
YOLO_TO_FSM = {
    CLASS_SU_GECISI   : "SU_GECISI",
    CLASS_TASLI_YOL   : "TASLI_YOL",
    CLASS_YAN_EGIM    : "YAN_EGIM",
    CLASS_DIK_ENGEL   : "DIK_ENGEL",
    CLASS_TRAFIK_KON  : "TRAFIK_KON",
    CLASS_KAYAR_ENGEL : "KAYAR_ENGEL",
    CLASS_DIK_EGIM    : "DIK_EGIM",
    CLASS_ATIS        : "ATIS",
    CLASS_HIZLANMA    : "HIZLANMA",
}

# ─────────────────────────────────────────────
# EŞIK DEĞERLERİ
# ─────────────────────────────────────────────

# YOLO güven eşiği — altında tetikleme olmaz
YOLO_CONFIDENCE_THRESHOLD = 0.75

# Kaç ardışık frame sonra FSM tetiklenir
YOLO_CONSECUTIVE_FRAMES = 3

# Taret açı limitleri (derece)
TARET_PAN_MIN  = -90.0
TARET_PAN_MAX  =  90.0
TARET_TILT_MIN = -45.0
TARET_TILT_MAX =  45.0

# IMU eşikleri (derece)
IMU_PITCH_ENGEL_THRESHOLD = 5.0   # Dik engel — tork artışı
IMU_PITCH_RAMP_THRESHOLD  = 15.0  # Rampa — yüksek tork
IMU_ROLL_WARN_THRESHOLD   = 8.0   # Yan eğim — hız düşür
IMU_ROLL_STOP_THRESHOLD   = 15.0  # Yan eğim — dur

# Batarya eşikleri (%)
BATTERY_WARN_SOC     = 20.0  # Uyarı
BATTERY_CRITICAL_SOC = 10.0  # Güvenli durdurma

# Lazer ateşleme süresi (saniye)
LASER_FIRE_DURATION = 0.5

# Dik eğim bekleme süresi — §6.10 (saniye)
RAMP_STOP_DURATION = 2.0

# YOLO integer class_id topic (terrain_adapter icin)
YOLO_CLASS_ID_TOPIC = "/yolo/class_id"  # std_msgs/UInt8

ACKERMANN_CMD_TOPIC = "/ackermann_cmd"  # AckermannDriveStamped
