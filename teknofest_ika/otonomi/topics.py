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

# YDLidar sürücüsünün doğrudan çıkışı (ydlidar_launch.py varsayılanı)
# Üretici: ydlidar_ros2_driver  |  Tüketen: scan_relay, reaktif sürüş betikleri
SCAN_TOPIC = "/scan"

# scan_relay çıkışı — timestamp/frame_id düzeltilmiş ham LiDAR
# Üretici: scan_relay  |  Tüketen: preprocessing_node
SCAN_LIDAR_TOPIC = "/scan_lidar"

# preprocessing_node filtrelenmiş LaserScan
# Açı kırpma + geçersiz okuma temizleme + hareketli ortalama uygulanmış
# Nav2 costmap ve kayar_engel_* bu topic'i kullanır
SCAN_FILTERED_TOPIC = "/scan/filtered"

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

# AS5600 ham ADC — kalibrasyon/teşhis
# Mesaj tipi : std_msgs/UInt16MultiArray  → [sol_adc, sag_adc], 0–1023
# Frekans    : 50 Hz (enkoder paketiyle aynı)
# Üretici    : seri_kopru (yalnız ham_enkoder=True iken)
# Tüketen    : scripts/sensor_dogrula.py --mod ham
# Hangi analog pinin bağlı olduğunu, tur başına düşen tick'i ve okuma
# gürültüsünü görmek için; normal sürüşte kapalıdır.
ENKODER_HAM_TOPIC = "/enkoder/ham"

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

# Derinlik kamerası (eYs3D OS30A) — apc_camera_node renklendirmeyi kendisi
# yapar, yayın rgb8'dir (yakın sarı/turuncu, uzak koyu). Ham mesafe için
# /apc/points/data_raw (PointCloud2) kullanılır.
DEPTH_IMAGE_TOPIC = "/apc/depth/image_raw"

# ─────────────────────────────────────────────
# GÖRÜNTÜ İŞLEME TOPIC'LERİ
# Üretici: Görüntü İşleme (YOLO, HSV node'ları)
# Tüketen: Otomasyon (FSM, Nav2 costmap)
# ─────────────────────────────────────────────

# YOLO tespit sonuçları
# Mesaj tipi : vision_msgs/Detection2DArray
# Frekans    : ~25 Hz (TensorRT FP16)
DETECTIONS_TOPIC = "/ika/detections"

# Ham YOLO tespit çıkışı — yolo_detection_node → yolo_adapter_node + cone_fusion_node
# Mesaj tipi : vision_msgs/Detection2DArray
YOLO_RAW_TOPIC = "/detections/yolo"

# YOLO debug görüntüsü — bounding box overlay
# Mesaj tipi : sensor_msgs/Image
YOLO_RAW_DEBUG_TOPIC = "/detections/yolo/debug"

# Debug görüntüsü — YOLO bounding box overlay (genel)
# Mesaj tipi : sensor_msgs/Image
DEBUG_IMAGE_TOPIC = "/debug/image"

# Koni füzyon costmap çıkışı — cone_fusion_node → Nav2 ObstacleLayer
# Mesaj tipi : sensor_msgs/PointCloud2
CONE_FUSION_CLOUD_TOPIC = "/costmap/cone_cloud"

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

# SLAM haritasının GCS dashboard için gri tonlamalı görüntü hali
# Üretici: map_image_node (MAP_TOPIC'i dinler)
# Mesaj tipi : sensor_msgs/Image (bgr8)
MAP_IMAGE_TOPIC = "/map/image"

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

# Servo komutları — servo_controller_node → seri_kopru → PKT_SERVO_PAN/TLT
# Mesaj tipi : std_msgs/Int16  (derece, 0-180)
TARET_PAN_TOPIC  = "/taret/pan"    # Pan (yatay) açısı
TARET_TILT_TOPIC = "/taret/tilt"   # Tilt (dikey) açısı

# Taret servo hareket komutu — targeting_node → servo_controller_node
# Mesaj tipi : geometry_msgs/Vector3  (x=yaw_offset_deg, y=pitch_offset_deg, z=0)
TURRET_CMD_TOPIC = "/turret/cmd"

# Taret hedefleme etkinleştirme — misyon_fsm → targeting_node
# Mesaj tipi : std_msgs/Bool  (True=aktif, False=standby/home)
TARGETING_ENABLE_TOPIC = "/targeting/enable"

# Taret hedefleme durumu — targeting_node → misyon_fsm
# Mesaj tipi : std_msgs/String
# Değerler   : "SEARCHING" | "LOCKED" | "ALIGNED" | "STANDBY" | "NO_IMAGE" | "STALE_IMAGE"
TARGETING_STATUS_TOPIC = "/targeting/status"

# Taret hizalama hatası — targeting_node yayınlar (piksel cinsinden)
# Mesaj tipi : geometry_msgs/Point  (x=hata_x_px, y=hata_y_px, z=0)
TARGETING_ERROR_TOPIC = "/targeting/error"

# Atış sonucu
# Mesaj tipi : std_msgs/Bool  (True = ateş edildi)
SHOOT_RESULT_TOPIC = "/shoot/result"

# ─────────────────────────────────────────────
# GÖRÜNTÜ ÖN İŞLEME TOPIC'LERİ
# Üretici: preprocessing_node
# Tüketen: yolo_detection_node, targeting_node, lane_detection_node
# ─────────────────────────────────────────────

# preprocessing_node çıkışı — normalize edilmiş, boyutlandırılmış kamera görüntüsü
# Mesaj tipi : sensor_msgs/Image
CAMERA_PROCESSED_TOPIC = "/camera/image_processed"

# preprocessing_node çıkışı — nişan kamerası (CAMERA_TARET_TOPIC) işlenmiş hâli.
# targeting_node bu topic'i kullanır — ana sürüş kamerasının taretle birlikte
# hareket etmemesi nedeniyle paralaks hatasını önler (Şartname §6.10/§7).
# Mesaj tipi : sensor_msgs/Image
CAMERA_TARET_PROCESSED_TOPIC = "/camera/taret/image_processed"

# Debug görüntüsü — targeting_node overlay
# Mesaj tipi : sensor_msgs/Image
TARGETING_DEBUG_TOPIC = "/targeting/debug"

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

# ─────────────────────────────────────────────
# SERİ PORT ADRESLERI (udev kurallarıyla sabit)
# ─────────────────────────────────────────────

SERIAL_LIDAR   = "/dev/lidar"        # YDLidar Tmini Pro
SERIAL_IMU     = "/dev/imu_arduino"  # Arduino Nano (IMU + servo + lazer) — artık kullanılmıyor, IMU Mega'ya entegre
SERIAL_ODOM    = "/dev/mega"         # Arduino Mega (enkoder + Karaşimşek + step motor + BMI160 IMU) — udev: 99-ika.rules
SERIAL_TARET   = "/dev/turret"       # Turret UNO (PCA9685 + BMI160) — udev symlink;
                                     # ham ttyCH341USBx isimleri hub her koptuğunda
                                     # yeniden numaralanıyor, Mega'nın portuna denk
                                     # gelme riski var
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
#
# Model ALFABETİK sırayla eğitildi — class_id Tabela numarasıyla örtüşmüyor!
# Gerçek eşleme (best.pt names, 15 sınıf):
#   0=Tabela_1       4=Tabela_2       9=Tabela_7
#   1=Tabela_10      5=Tabela_3      10=Tabela_8
#   2=Tabela_11      6=Tabela_4      11=Tabela_9
#   3=Tabela_11_son  7=Tabela_5      12=Tabela_stop
#                    8=Tabela_6      13=hedef_tahtasi  14=trafik_huni

CLASS_SULU_YOL       = "Tabela_1"   # class_id= 0
CLASS_DIK_EGIM_CIKIS = "Tabela_10"  # class_id= 1
CLASS_TASLI_YOL      = "Tabela_2"   # class_id= 4
CLASS_YAN_EGIM       = "Tabela_3"   # class_id= 5
CLASS_DIK_ENGEL      = "Tabela_4"   # class_id= 6
CLASS_KONILI_YOL     = "Tabela_5"   # class_id= 7
CLASS_KAYAR_ENGEL    = "Tabela_6"   # class_id= 8
CLASS_ENGEBELI_ARAZI = "Tabela_7"   # class_id= 9
CLASS_DIK_EGIM       = "Tabela_8"   # class_id=10
CLASS_ATIS_BOLGESI   = "Tabela_9"   # class_id=11
CLASS_TRAFIK_HUNI    = "trafik_huni"    # class_id=14
CLASS_HEDEF_TAHTASI  = "hedef_tahtasi"  # class_id=13

# class_id → label (alfabetik model sırası) — referans tablo
# yolo_adapter_node ve terrain_adapter kendi dict'lerini kullanır
# Sınıf sırası model ile birebir aynı olmalı (best.pt names, 2026-07-22).
# YENİ MODEL 15 sınıf — eski 16 sınıflı sürümdeki "Tabela_12" KALDIRILDI,
# index 4'ten sonrası bir kaydı: Tabela_stop 13→12, hedef_tahtasi 14→13,
# trafik_huni 15→14. Model değişirse bu liste model.names ile eşitlenmeli.
YOLO_CLASSES = [
    "Tabela_1",       # 0  → SULU_YOL
    "Tabela_10",      # 1  → DIK_EGIM_CIKIS
    "Tabela_11",      # 2  → HIZLANMA başlangıcı
    "Tabela_11_son",  # 3  → HIZLANMA sonu
    "Tabela_2",       # 4  → TASLI_YOL
    "Tabela_3",       # 5  → YAN_EGIM
    "Tabela_4",       # 6  → DIK_ENGEL
    "Tabela_5",       # 7  → KONİLİ_YOL
    "Tabela_6",       # 8  → KAYAR_ENGEL
    "Tabela_7",       # 9  → ENGEBELİ_ARAZİ
    "Tabela_8",       # 10 → DIK_EGIM
    "Tabela_9",       # 11 → ATIS_BOLGESI
    "Tabela_stop",    # 12 → STOP işareti
    "hedef_tahtasi",  # 13 → atış hedefi
    "trafik_huni",    # 14 → trafik konisi
]

# Tabela label → FSM waypoint eşlemesi
YOLO_TO_FSM = {
    "Tabela_1"   : "SULU_YOL",
    "Tabela_2"   : "TASLI_YOL",
    "Tabela_3"   : "YAN_EGIM",
    "Tabela_4"   : "DIK_ENGEL",
    "Tabela_5"   : "KONİLİ_YOL",
    "Tabela_6"   : "KAYAR_ENGEL",
    "Tabela_7"   : "ENGEBELİ_ARAZİ",
    "Tabela_8"   : "DIK_EGIM",
    "Tabela_9"   : "ATIS_BOLGESI",
    "Tabela_10"  : "DIK_EGIM_CIKIS",
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
IMU_ROLL_ESTOP_THRESHOLD  = 20.0  # Devrilme — E-STOP
IMU_PITCH_DOWN_THRESHOLD  = 15.0  # Yokuş aşağı fren modu

# Batarya eşikleri (%)
BATTERY_WARN_SOC     = 20.0  # Uyarı
BATTERY_CRITICAL_SOC = 10.0  # Güvenli durdurma

# Lazer ateşleme süresi (saniye) — Şartname §6.10: lazer aktif olduktan sonra
# EN AZ 1 saniye hedefte sabit kalmalı. ShootState bu süreyi yazılım
# seviyesinde de garanti eder (donanım/Nano zamanlamasına tek başına güvenilmez).
LASER_FIRE_DURATION = 1.0

# Dik eğim bekleme süresi — §6.10 (saniye)
RAMP_STOP_DURATION = 2.0

# YOLO integer class_id topic (terrain_adapter icin)
YOLO_CLASS_ID_TOPIC = "/yolo/class_id"  # std_msgs/UInt8

ACKERMANN_CMD_TOPIC = "/ackermann_cmd"  # AckermannDriveStamped

# Fren komutu — ackermann_converter, hedef hızdaki ani düşüşten otomatik
# hesaplar, seri_kopru bunu PKT_FREN olarak Arduino'ya iletir.
FREN_KOMUT_TOPIC = "/fren_komut"   # std_msgs/UInt16, ‰ (0-1000)

# Fren hesabı kalibrasyon sabitleri — PLACEHOLDER, yumuşak/muhafazakâr
# başlangıç değerleri, fiziksel testte ayarlanacak.
FREN_IVME_ESIK_MIN  = 1.5    # [m/s²] — altında fren yok, motor coasting yeterli
FREN_IVME_ESIK_MAX  = 5.0    # [m/s²] — üstünde tam fren (yüksek eşik = geç tetiklenir)
FREN_TAM_DUR_ORAN   = 0.3    # hedef hız tam 0 olsa bile en fazla bu oranda fren (0-1)
FREN_RAMP_PER_S     = 500.0  # [‰/s] fren yüzdesi değişim hızı sınırı — ani sıçramayı önler

# ─────────────────────────────────────────────
# KONTROL TOPIC'LERİ (node'lar arası iç protokol)
# ─────────────────────────────────────────────

# E-STOP besleme — her kaynak kendi topic'ine yayınlar, e_stop_node ayrı takip eder.
# Tek topic kullanılırsa imu_guvenlik True gönderip hemen False'a dönünce
# seri_kopru'nun True'su silinir (OR mantığı bozulur).
E_STOP_FORCE_TOPIC       = "/e_stop/force"            # geriye dönük uyumluluk (kullanılmamalı)
E_STOP_FORCE_IMU_TOPIC   = "/e_stop/force/imu"        # imu_guvenlik → devrilme
E_STOP_FORCE_SERIAL_TOPIC= "/e_stop/force/serial"     # seri_kopru  → fiziksel buton
E_STOP_FORCE_GCS_TOPIC   = "/e_stop/force/gcs"        # ika_dashboard (WiFi) → GCS komutu
E_STOP_FORCE_RC_TOPIC    = "/e_stop/force/rc"         # mod_yoneticisi → RC sinyal kaybı

# GPIO kurulumu/fiziksel buton donanımı başarısız olduğunda True yayınlanır.
# Ayrılık ilkesi (Şartname §6.13/§7.8): güvenlik alt sisteminin kendi iç hata
# durumu artık sadece log'da kalmıyor, ROS2 üzerinden gözlemlenebilir.
E_STOP_GPIO_FAULT_TOPIC = "/e_stop/gpio_fault"

# watchdog, kritik topic'lerden biri bayatladığında hangisinin sustuğunu
# yayınlar. Panoya bağlıdır — sessizce ölen bir sensör aksi halde ancak
# davranış bozulunca fark ediliyor.
SENSOR_FAULT_TOPIC = "/sensor/fault"   # String (arıza açıklaması)

# Mod yönetimi
MOD_KOMUT_TOPIC   = "/mod/komut"    # yazılımsal/GCS mod değiştirme (UInt8)
MOD_AKTIF_TOPIC   = "/mod/aktif"    # geçerli mod (UInt8)
MUX_CMD_VEL_TOPIC = "/mux/cmd_vel"  # muxlanmış Twist → ackermann_converter

# Misyon akışı
MISSION_START_TOPIC  = "/mission_start"   # FSM tetikleyici (Bool)
MISSION_STATUS_TOPIC = "/mission_status"  # görev sonucu (String)
MISYON_AKTIF_TOPIC   = "/misyon/aktif"   # kayıt + durum göstergesi (Bool)
SHOOT_CMD_TOPIC      = "/shoot_command"   # lazer tetikleyici (Bool)
MISYON_WP_INDEX_TOPIC = "/misyon/wp_index"  # mevcut waypoint indeksi (UInt8)
MISYON_KALAN_SURE_TOPIC = "/misyon/kalan_sure"  # koşu saatinde kalan süre (Float32)

# Koşu süresi — Şartname §6.12: her koşu, atış dahil, en fazla 15 dakikadır.
# Süre dolduğunda takım geçtiği aşamaların puanını korur ama parkurdan çıkmak
# zorundadır. Saat bir güvenlik sınırı değil doğrudan puan kaynağıdır: §9'da
# "koşu tamamlama süresi" tek başına en yüksek 100 puanlık kalemdir, atıştan
# (50) ve konilerden (50) büyüktür. Saat koşunun başladığı anda işler ve
# manuel moda geçilse de DURMAZ — hakem kronometresi de durmaz.
KOSU_SURESI_S = 900.0

# Pas hakkı — Şartname §9: her koşuda YALNIZCA 1 aşama pas geçilebilir ve pas
# geçilen aşamanın alınabilecek en yüksek puanı eksi olarak yazılır. Pas, aracın
# bir sonraki hedefe sürmesi değildir: takım üyeleri parkura girip aracı elle
# taşır. Yani bu bayrak otonom bir kaçış yolu değil, sahadaki insan kararının
# yazılıma bildirilmesidir.
PAS_HAKKI = 1

# Şartname §9: hızlanma parkuru ve otonom koşuda trafik konileri pas geçilemez.
# Konfigürasyon ne derse desin bu iki aşama atlanmaz.
PAS_GECILEMEZ = ("KONİLİ_YOL", "HIZLANMA_PARKURU")

# RC kumanda kanalları
# Float32MultiArray [ch1_gaz, ch2_direksiyon/pan, ch5_mod, ch3_aux/lazer,
#                     ch_tilt, ch_taret_aktif] µs
RC_INPUT_TOPIC = "/rc_input"

# Anti-rollback override
ANTI_ROLLBACK_CMD_TOPIC   = "/anti_rollback/cmd"    # Twist — override komutu
ANTI_ROLLBACK_AKTIF_TOPIC = "/anti_rollback/aktif"  # Bool — override aktif mi

# Kayar engel iç topic'leri
MOVING_OBS_DIR_TOPIC   = "/moving_obs/direction"  # String ('sol'/'sag'/'bilinmiyor')
MOVING_OBS_CLOUD_TOPIC = "/moving_obs_cloud"       # PointCloud2 → Nav2 ObstacleLayer

# Koni costmap çıkışı
CONE_CLOUD_TOPIC = "/cone_cloud"   # PointCloud2 → Nav2 ObstacleLayer

# Veri paketi kayıt kontrolü
KAYIT_BASLAT_TOPIC = "/veri_paketi/kayit_baslat"  # Bool (True=başlat)
KAYIT_DURUMU_TOPIC = "/veri_paketi/kayit_durumu"  # Bool (True=devam ediyor)
