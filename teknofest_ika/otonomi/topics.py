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

# BNO055 IMU — sürüş kartına bağlı, 0x32/0x33 paketlerinden
# Mesaj tipi : sensor_msgs/Imu
# Frekans    : 50 Hz (yalnız çip takılıyken akar)
# Frame      : imu_link
IMU_TOPIC = "/imu/data"

# Sürüş kartının ileri hızı (0x31) — konum ve yön EKF'te türetilir
# Mesaj tipi : nav_msgs/Odometry
# Frekans    : 30 Hz
# Frame      : odom → base_link
ODOM_TOPIC = "/odom"

# Kartın ham enkoder sayımı (0x30) — ölçekten bağımsız, hareket teşhisi
# Mesaj tipi : std_msgs/Int32
# Frekans    : 20 Hz
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

# Microcase 720P — Arka manevra kamerası.
# 🔑 Ad elektrik tarafının kamera gözcüsüyle eşleşmek zorunda: yayıncı o,
# ve kamera fişten çıkıp geri takıldığında cihazı port yolundan geri bağlayan
# da o. Ön kamera ayrı bir konu DEĞİL, CAMERA_IMAGE_TOPIC'ten geliyor (iki
# kamera aynı fiziksel cihaza indirgendi).
CAMERA_REAR_TOPIC = "/camera/arka/image_raw"

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

# Nav2 hareket komutu → ackermann_converter → seri_kopru → sürüş kartı
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

# Batarya durumu — üretici bms_koprusu (JK BMS, BLE üzerinden HTTP ucundan)
# Mesaj tipi : sensor_msgs/BatteryState
# Frekans    : 1 Hz
BATTERY_TOPIC = "/battery/status"

# Jetson'ı besleyen ikinci paket (DALY). Traksiyon paketinden AYRI konu:
# ikisi farklı kimyada, farklı eşikte ve biri bitince ötekinden haber
# gelmiyor. Kaynağı seri hat (0x3D), BLE değil — DALY BLE taramasında hiç
# görünmüyor ve 3 Eylül'de Jetson'ı düşüren paket buydu.
BATTERY_JETSON_TOPIC = "/battery/jetson"

# ─── JK BMS köprüsü ──────────────────────────────────────────────────────────
# Batarya gerilimi seri hattan gelmiyor ve gelmeyecek: araçta iki BMS var ve
# ikisi de gerilimi hücre bazında kendi ölçüyor, okuma yolu BLE. Elektrik
# tarafındaki servis paketi BLE'den okuyup düz bir JSON olarak yayınlıyor.
BMS_HTTP_URL = "http://127.0.0.1:8091/bms"

# 🔑 Tazelik ölçütü `bagli` DEĞİL `yas`: BLE koptuğunda ölçüm alanları
# silinmiyor, SON DEĞERDE DONUYOR. Bağlantı bayrağına bakan bir tüketici
# donmuş bir gerilimi canlı sanır — bu projede defalarca karşılaştığımız
# "veri var görünüyor ama bayat" arıza sınıfının aynısı.
BMS_YAS_ESIK_S = 10.0        # [s] üstünde okuma bayat sayılır

# Kesme ölçütü hücre dibinden okunur, BMS'in SOC tahmininden değil: LiFePO4'ün
# deşarj eğrisi düz olduğu için SOC yük altında zıplıyor. 16S paket.
BMS_HUCRE_DIP_MV    = 2800   # [mV] pratik dip — altı "ölü"
BMS_HUCRE_UYARI_MV  = 3000   # [mV] eğrinin dizi — altı uyarı
BMS_HUCRE_SAYISI    = 16

# Jetson paketinin uyarı eşiği ayrı, çünkü paket ayrı: 4S LiPo (nominal
# 3,7 V/hücre), traksiyon paketi ise 16S LiFePO4 (3,2 V/hücre).
#
# Eşik kimyaya bağlı ve iki yönlü: LiPo'da 2800 mV uyarmak çok geç, hücre o
# noktada kalıcı zarar görmüş olur. LiFePO4'te 3300 mV ise paket DOLUYKEN bile
# sürekli alarm verir — sürekli yanan bir uyarı gerçek uyarıyı gizlediği için
# hiç uyarmamaktan kötüdür. Bu yüzden "belirsizlikte yüksek eşik" diye bir
# kural yok; sayı kimyayla birlikte değişir.
#
# ⚠️ Kimya paketin etiketinden değil kayıttan geliyor; sahada doğrulanacak.
# LiFePO4 çıkarsa bu değer ~2900'e iner.
BMS_JETSON_UYARI_MV = 3300   # [mV] — 4S LiPo

# ─────────────────────────────────────────────
# SERİ PORT ADRESLERI (udev kurallarıyla sabit)
# ─────────────────────────────────────────────

SERIAL_LIDAR   = "/dev/lidar"        # YDLidar Tmini Pro
SERIAL_IMU     = "/dev/imu_arduino"  # Arduino Nano (IMU + servo + lazer) — artık kullanılmıyor, IMU sürüş kartında
# Sürüş kartı: Nucleo-F767ZI. Sabit symlink kartın kendi udev kuralıyla
# kuruluyor; ham ttyACM* numarası her takışta kayabildiği için o isme
# güvenilmez. Kart tek seri cihaz olduğundan by-path ayrımı gerekmiyor.
SERIAL_ODOM    = "/dev/f767"
SERIAL_TARET   = "/dev/turret"       # Eski Turret UNO yolu. Taret arayüzü sürüş
                                     # kartına taşındı (0x03/0x05/0x06) ve bu port
                                     # araçta artık yok; taret_rc_koprusu ile
                                     # birlikte yalnız eski kurulumlar için duruyor
# İkili telemetri (100 Hz x 9 paket x 8 B = 7.200 B/s) ile kartın ASCII teşhis
# akışı (10 Hz x ~500 B = 5.000 B/s) birlikte 12.200 B/s ediyor; 115200 8N1'in
# taşıyabildiği 11.520 B/s bunun altında kalıyordu. Taret ayrı bir cihaz ve
# kendi hızında konuşuyor — iki hattın ortak bir sabiti yok.
SERIAL_BAUD_KART  = 921600
SERIAL_BAUD_TARET = 115200

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

# Tabela görüş alanından çıktıktan sonra arazi profilinin normale dönmesi için
# beklenen süre. Sınırsız sticky, bir kez görülen tabelanın profilini koşunun
# sonuna kadar tutuyordu; 11 aşamanın yalnız birinin (Tabela_11_son) bitiş
# tabelası var, yani "sonraki tabelaya kadar sürsün" kuralı diğer onunda
# bölümü hiç bitirmiyor.
# 3 s, iki şeyin arasında seçildi: tabelanın yanından geçerken kameradan
# çıkması (kısa, tek kare kayıplarını zaten 2 karelik onay yutuyor) ve aynı
# bölüm içinde tabelayı bir süre görememek. Kart/kamera tarafında değil
# köprüde: ölçüt kare değil SÜRE, kamera hızı değişirse kural değişmesin.
TABELA_GORUS_ZAMAN_ASIMI_S = 3.0   # [s]

# ── Nişan halkası HSV kalibrasyonu ───────────────────────────────────────────
# 2026-07-19, kapalı alanda nişan kamerasıyla ölçüldü. Halka bu kamerada
# H=165-169 okunuyor; turuncu bant sahte tespit ürettiği için üst sınır 4'te
# kesildi ve V tabanı 70'e indirildi (karanlık sahteleri eler, loş ışıkta
# gerçek halkayı elemez).
#
# 🔑 Değerler burada, düğümün varsayılanı olarak duruyor: bir dönem yalnız
# launch dosyasında yazılıydı ve sahada koşan açılış betiği launch'u
# kullanmadığı için kalibrasyon araca hiç ulaşmıyordu.
#
# ⚠️ Yarışma günü GÜN IŞIĞINDA yeniden kalibre edilmeli; yöntem
# launch/taret_otonom.launch.py docstring'inde.
NISAN_HSV_ALT   = [0,   40, 70]
NISAN_HSV_UST   = [4,  255, 255]
NISAN_HSV_ALT2  = [150, 40, 70]    # kırmızının ton ekseninde sarması
NISAN_HSV_UST2  = [179, 255, 255]
# Halkanın piksel yarıçapı üst sınırı — yakın mesafede büyük görünüyor.
NISAN_HOUGH_MAX_YARICAP = 250

# Taret açı limitleri (derece)
TARET_PAN_MIN  = -90.0
TARET_PAN_MAX  =  90.0
TARET_TILT_MIN = -45.0
TARET_TILT_MAX =  45.0

# IMU eşikleri (derece)
IMU_PITCH_ENGEL_THRESHOLD = 5.0   # Dik engel — tork artışı
IMU_PITCH_RAMP_THRESHOLD  = 15.0  # Rampa — yüksek tork
# Şartname §6.5 %20 yan eğimden geçmeyi ZORUNLU tutuyor, §7.3 de aracın o
# eğimde stabil olmasını istiyor. %20 = arctan(0.20) = 11,31°, yani uyarı
# eşiği bunun altında kaldığı sürece zorunlu bir aşamada hız kısılır.
IMU_ROLL_WARN_THRESHOLD   = 13.0  # Yan eğim — §6.5'in 11,31°'si + pay
# ⚠️ Aşağıdaki iki eşik ÖLÇÜME BAĞLI, şu anki değerler devralınmış tahmindir.
# Statik devrilme açısı  θ = arctan(iz_genişliği / (2 × ağırlık_merkezi_yük.))
# formülüyle türetilir; iz genişliği §7.1 kapsamında hâlâ ölçülmedi (depoda
# 0,50 / 0,67 / 0,900 diye üç değer var) ve ağırlık merkezi yüksekliği hiç
# bilinmiyor. İkisi ölçülünce buradan yeniden hesaplanacak.
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

# Güvenlik dallarının (watchdog, E-STOP) yayınladığı fren değeri. SIFIR OLAMAZ:
# sürüş kartında 0 doğrudan "freni bırak" demek ve fren kaynakları arasında
# büyük olan seçildiği için sıfır hiçbir katkı yapmaz. §6.10'un zorunlu 2 sn
# duruşu %45 eğimde geçiyor; orada fren isteğini kesmek aracı kaydırır.
FREN_GUVENLI_DUR_BINDE = 1000

# Atış kilidinin ÜST SINIRI [s] — hem ackermann_converter'ın tam freni hem
# seri_kopru'nun hareket kilidi buna bağlı. /shoot_command bir nabız değil
# kenar sinyalidir (True ile False ayrı olaylar) ve arada isteği yayınlayan
# düğüm ölürse kilit sonsuza kadar kapalı kalır: fren basılı, her sürüş komutu
# sıfır hıza çevriliyor, araç bir daha hiç hareket etmiyor. Sınır bu yüzden
# bayatlığa değil isteğin TOPLAM süresine bakar.
# Meşru en uzun tutuş ShootState'in tek denemesi: TIMEOUT_S 8 s onay beklemesi
# + LASER_FIRE_DURATION tamamlaması, yani ~9 s; targeting_node'un kendi darbesi
# 1 s. 12 s ikisinin de üstünde, sonsuzun ise çok altında.
ATIS_AZAMI_S = 12.0

# ── §6.10 yokuş kalkışı — fren SIKILIYKEN gaz ────────────────────────────────
# Eğimde duran araçta motorun tutma torku yok: fren bırakılınca araç geri kaçar,
# rotor ters yöne döner ve Pilmak sürücü ters dönen rotora tork basmayı
# reddeder. Kalkış o yüzden fren hâlâ basılıyken başlar; tork oturduktan sonra
# fren rampayla bırakılır, böylece "fren gitti ama tork yok" boşluğu hiç oluşmaz.
# Kart tarafı buna açık: PKT_FREN ve PKT_SURUCU bağımsız paketler ve firmware'in
# gaz kesme koşulları arasında "fren basılı" YOK.
YOKUS_KALKIS_AKTIF_TOPIC = "/yokus_kalkis/aktif"  # Bool  — override aktif mi
YOKUS_KALKIS_FREN_TOPIC  = "/yokus_kalkis/fren"   # UInt16 — tutma freni [‰]

# Tutma freni: duruş boyunca uygulanan değerle aynı kalır ki araç kımıldamasın.
YOKUS_TUTMA_FREN_BINDE = FREN_GUVENLI_DUR_BINDE

# Tork oturma süresi — fren basılıyken gazın verildiği pencere. ÖLÇÜLMEDİ.
# Kısa tutuldu: motor frene karşı ne kadar uzun zorlanırsa akım ve ısı o kadar
# artar, sürücü aşırı akımdan atabilir.
YOKUS_TORK_SURESI_S = 0.4

# Fren bırakma hızı [‰/s]. Ani bırakmak aracı sıçratır, çok yavaş bırakmak
# motoru frene karşı gereksiz zorlar.
YOKUS_FREN_BIRAKMA_BINDE_PER_S = 2000.0

# Kalkış gazı [m/s] — tırmanış ve iniş AYRI, çünkü fizik farklı: tırmanışta
# yerçekimi geri çeker (gaz kaçışı önler), inişte ileri iter (gaz fazlaysa araç
# sıçrar). İkisi de ÖLÇÜLMEDİ, muhafazakâr başlangıç.
# NOT: alt sınır kaygısı yok — yeni sürüş kartı duruştan kalkış itişini kendisi
# uyguluyor, düşük komutta da araç kalkıyor.
YOKUS_KALKIS_HIZ_TIRMANIS = 0.35
YOKUS_KALKIS_HIZ_INIS     = 0.20

# Yokuş kalkışı bayrağının bayatlama süresi. RampaState bayrağı NABIZ gibi
# döngüde tekrar tekrar basıyor; ackermann_converter bu süre boyunca yeni
# bayrak görmezse override'ı bırakıp normal fren hesabına döner.
#
# Neden gerekli: RampaState süreç olarak ölürse `finally` çalışmaz ve bayrak
# ackermann_converter'da sonsuza kadar True kalır — otomatik fren tamamen ölür,
# fren son değerinde donar. Bayrağı tek sefer basıp güvenmek bu kusuru üretiyor.
#
# RampaState.CMD_HZ = 10 Hz → 0,1 s periyot. Bu değer ondan BÜYÜK olmalı,
# yoksa override normal çalışırken bayat sayılır. Beş kaçırma payı bırakıldı.
YOKUS_BAYATLAMA_S = 0.5

# Aynı kusur anti_rollback bayrağı için de geçerli: düğüm ölürse son True
# ackermann_converter'da sonsuza kadar kalır ve Nav2 komutu kalıcı olarak
# kurtarma komutuyla değiştirilir. anti_rollback bayrağı KONTROL_HZ = 20 Hz
# ile nabız basıyor (0,05 s periyot); değer ondan büyük, on kaçırma payı var.
ROLLBACK_BAYATLAMA_S = 0.5

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

# §6.9 tümsekli bölümde parkurun ortalanması isteniyor; aracın tamamının aynı
# tümsek dizisi üzerinden geçirilmesi §9'da −5 puan. Tümsek 20 cm taban /
# 5 cm yükseklik (Şekil 4) — LiDAR düzlemi zeminden 55 cm'de olduğu için
# tümseklerin kendisi HİÇ görünmüyor, tekerlek yerleştirme planlanamıyor.
# Ölçülebilen ve kuralın ilk cümlesinin istediği şey koridorda ortalanmak:
# §6.1 koridoru 3 m ve iki yanı 80 ± 10 cm bariyerli, ikisi de taramada var.
ENGEBELI_SAPMA_TOPIC = "/engebeli/sapma"   # Float32 [m], + ise sağa kaymış

KORIDOR_GENISLIGI_M     = 3.0    # §6.1 yol genişliği
KORIDOR_SAPMA_UYARI_M   = 0.50   # bu kadar kayma sürekliyse loga uyarı
KORIDOR_TOPLAM_TOLERANS = 0.60   # sol+sağ bu kadar sapabilir, fazlası koridor değil
# Yan pencerelerin merkezi ±90°, LiDAR'ın gövdeye göre montaj dönüklüğü
# (LIDAR_YAW_RAD) taramada değil TF'te uygulandığı için buradaki açılar
# tarama çerçevesinde okunur ve montaj açısıyla kaydırılır.
KORIDOR_PENCERE_RAD     = 0.26   # ±15° — duvarın düz kısmını yakalar

# ─────────────────────────────────────────────
# LİDAR MONTAJ AÇISI — TEK DOĞRULUK KAYNAĞI
# ─────────────────────────────────────────────
# LiDAR gövdeye 93,3° dönük monte (sahada huniyle çift yönlü kalibre edildi).
# Bu dönüş TF'te de basılıyor (lydia_startup.sh LIDAR_YAW_RAD, urdf
# lidar_joint) — Nav2 costmap yalnız TF'e baktığı için orada yeterli.
#
# ⚠️ AMA taramayı DİZİ OLARAK indeksleyen her düğüm bu dönüşü kendi
# uygulamak zorundadır: LaserScan.angle_min/increment tarama çerçevesindedir,
# araç çerçevesi değil. Bu atlandığı için bir dönem preprocessing aracın SAĞ
# yanını komple `inf` yapıyor, kayar engel arayıcısı aracın SOLUNU tarıyor ve
# koni füzyonu mesafeyi 93° yanlış yönden okuyordu — üçü de hata basmadan.
#
# Dönüşüm pure_logic.tarama_acisi_arac / arac_acisi_tarama'da.
LIDAR_MONTAJ_YAW_RAD = 1.6284   # [rad] 93,3° — urdf lidar_joint ile AYNI sayı

# Otonom sürüş komutunun bayatlama sınırı. mod_yoneticisi FULL_AUTO'da
# /cmd_vel'i mux'a geçirir; yayıncı susarsa komut BAYATLAR. Süre
# ackermann_converter'ın cmd_vel_timeout'uyla aynı tutuluyor.
NAV2_CMD_BAYATLAMA_S = 0.5

# ─────────────────────────────────────────────
# KAYAN HEDEF — haritasız sürüş (pure_logic.koridor_merkez_cizgisi / ic_duvar_hedefi)
# ─────────────────────────────────────────────
# Hedef `map` çerçevesinde sabit bir nokta değil, her döngüde taramadan
# yeniden üretilen bir nokta. Bu yüzden SLAM haritasının doğruluğuna da
# waypoint koordinatlarına da ihtiyaç duymaz; odometri yalnız "bu aşamada ne
# kadar yol gittim" için kullanılır ve hedefe HİÇ birikmez.
KAYAN_HEDEF_PERIYOT_S  = 1.5    # [s] yeniden hedefleme periyodu (~0,67 Hz)
# Periyot bilerek Nav2'nin planlama süresinden (max_planning_time 5 s) kısa
# değil, uzun tutuldu: her yeni hedef öncekini preempt ediyor, çok sık
# gönderilirse planlayıcı hiçbir planı bitiremeden yenisine başlar.
# Hedef taramadan araç çerçevesinde doğar ama Nav2'ye ODOM çerçevesinde
# gönderilir: araç çerçevesindeki bir hedefi Nav2 her yeniden planlamada
# o anki poza göre yeniden çözer, yani hedef araçla birlikte kayar ve
# asla varılmaz. odom sürüklenir ama sıçramaz; hedefin ömrü zaten bir
# periyot (1,5 s) olduğu için sürüklenme ölçülemeyecek kadar küçük kalır.
KAYAN_HEDEF_FRAME      = "odom"

# Nav2'nin GLOBAL ÇERÇEVESİ. Hedef hangi çerçevede gönderilirse gönderilsin
# bt_navigator onu buna dönüştürmek zorunda; dönüşüm yoksa hedef sessizce
# reddedilir ve log'da yalnız bir TF hatası kalır.
#
# ⚠️ nav2_params.yaml'daki `bt_navigator.global_frame` ile AYNI olmak zorunda.
#    SLAM kapalı olduğu için orada `odom` yazıyor ve `map` çerçevesini basan
#    HİÇ KİMSE YOK: `map`'te doğan bir hedef hiçbir zaman çözülemez.
#    test_birim.py iki dosyayı karşılaştırıyor.
NAV2_GLOBAL_FRAME      = "odom"
KAYAN_HEDEF_YOK_SINIR  = 8      # ardışık bu kadar döngüde hedef üretilemezse aşama başarısız
# Hedef bu kadar kaymadıysa YENİDEN GÖNDERİLMEZ.
#
# NEDEN: Nav2'nin kurtarma dalı `<GoalUpdated/>` ile korunuyor ve o düğüm
# hedefi header'ıyla birlikte eşitlikle karşılaştırıyor. Her gönderimde zaman
# damgası yenilendiği için araç taş gibi dursa bile hedef "değişti" okunuyor;
# `ReactiveFallback` çalışan kurtarmayı halt edip SUCCESS dönüyor ve
# `RecoveryNode` bunu "kurtarma başarılı" sayıp altı hakkın birini yiyor.
# Sonuç: 3 saniyelik Wait hiç bitmiyor, 0,5 m'lik BackUp sınırda kesiliyor —
# yani Ackermann aracı sıkıştığı pozdan çıkarabilecek TEK davranış.
#
# 0,25 m iki uçtan da uzak:
#   ALT — LiDAR gürültüsünün koridor merkezinde ürettiği oynama santimetre
#         mertebesinde; eşik onun belirgin üstünde olmalı, yoksa duran araçta
#         da gönderim sürer ve düzeltme hiçbir işe yaramaz.
#   ÜST — aracın yerinden kalkabildiği en düşük hız 0,45 m/s; bir periyotta
#         (1,5 s) en az 0,68 m yol eder ve hedef onunla birlikte kayar.
#         Eşik bunun altında kaldığı sürece GERÇEK hareket hiç bastırılmaz.
# Yani düzeltme yalnız araç kımıldamadığında devreye giriyor — tam da
# kurtarmaya ihtiyaç duyulan an.
#
# Yalnız KONUM'a bakılıyor, yaw'a değil: `use_rotate_to_heading: false` ve
# kayan modda hedef denetleyicisi hiç tetiklenmediği için hedefin açısı
# sürüşü belirlemiyor. Yaw'ı eşiğe katmak, açıdaki gürültünün bastırmayı
# bozmasına kapı açardı.
KAYAN_HEDEF_OLU_BANT_M = 0.25   # [m] hedef bundan az kaydıysa yeniden gönderilmez

# Hedefin araca en yakın kabul edilebilir uzaklığı. `kayan_hedef`'in kendi
# varsayılanıyla AYNI olmak zorunda (test_birim.py imzadan okuyup karşılaştırır);
# iç duvar yolunda böyle bir sınır hiç yoktu, kapı ikisine birden uygulanıyor.
KAYAN_HEDEF_MIN_ILERI_M = 2.0    # [m]

# İç duvar takibinin kendi tabanı. Merkez çizgisinin tabanından AYRI olmak
# zorunda: `ic_duvar_hedefi` hedefi tasarımı gereği YAKINA koyuyor — duvar
# noktası 2 m menzilde seçiliyor ve hedef ondan 1,4 m koridorun içine
# kaydırıldığı için araca olan uzaklık kısalıyor. Parkur CAD'ine karşı ölçülen
# uzaklıklar 1,32–2,38 m (medyan ~1,9). Merkez çizgisinin 2,0 tabanı buna
# uygulandığında U dönüşlerinde hedeflerin %59-62'si eleniyordu ve araç
# yönlendirmenin en çok gerektiği yerde 4,5 saniyeye kadar bayat hedefle
# sürüyordu (ölçüm: scripts/parkur_cad/koridor_dogrula.py makineleri).
#
# 1,2 m ölçüden türetildi: gövdenin yarısı 0,95 m + pay. Bundan yakın bir
# hedef aracın KENDİ AYAK İZİNİN içinde kalır; planlayıcı için anlamsızdır.
# CAD süpürmesi de aynı yerde buluşuyor — 1,2 tabanı hedefsiz poz sayısını
# kapı öncesi hâline (1 ve 0) geri getiriyor, 1,5 zaten bir hedef kaybediyor.
IC_DUVAR_MIN_ILERI_M = 1.2       # [m]

# Planlayıcının dönebildiği en küçük yarıçap. Hedefin ERİŞİLEBİLİR sayılıp
# sayılmadığını belirleyen tek sayı bu: araç bu yarıçapın altında yay
# çizemez.
#
# 🔑 Kapı GERİ MANEVRAYI hesaba katmaz, bilerek. Planlayıcı REEDS_SHEPP,
#    yani arkadaki bir hedefe geri yayla teorik olarak yol var; ama araç
#    ARKADAN KÖR (LiDAR'ın arkasını gövde kapatıyor) ve geri manevra
#    costmap'te boş görünen bir alana yapılır. Geri yay sıkışmadan çıkmak
#    için açık; kayan hedefin ÜRETTİĞİ hedefler ileride olmak zorunda.
#
# ⚠️ nav2_params.yaml'daki `minimum_turning_radius` ile AYNI olmak zorunda —
#    planlayıcı o sayıyla arıyor, kapı bu sayıyla eliyor; ayrışırlarsa kapı
#    planlayıcının çözemeyeceği bir hedefi geçirir ya da çözebileceğini eler.
#    test_birim.py iki dosyayı karşılaştırıyor.
#    Arkasındaki dingil arası 1,44 m (mezürle ölçüldü): 2,49 = 1,44/tan(30°).
#    🔴 δ_max = 30° hâlâ bir varsayım; mekanik uç ölçülünce bu sayı da
#    değişir. Değer dört ayrı yerde yaşadığı için (burası, nav2_params,
#    ackermann_converter parametresi, urdf) tek kaynağa indirilmesi ayrı
#    bir iş olarak duruyor.
PLANLAYICI_DONUS_YARICAPI_M = 2.49   # [m]
KAYAN_ODOM_BAYATLAMA_S = 1.0    # [s] /odometry/filtered bu süre gelmezse yol ölçülemiyor demektir
# Taramanın karşılığı. Odometri için bu kapı vardı, tarama için YOKTU:
# `_scan` son taramayı süresiz tutuyor ve hedef ölü veriden üretilmeye devam
# ediyordu. En sinsi yanı, `hedefsiz` sayacının hiç artmaması — hedef
# ÜRETİLİYOR, yalnız koridorun hafızasından. 1,0 s ölçülen tarama hızına
# (9,96 Hz) on taramalık pay bırakır.
KAYAN_TARAMA_BAYATLAMA_S = 1.0  # [s] /scan/filtered bu süre gelmezse kör sürülüyor demektir
# `_yol` her EKF örneğinde |Δkonum| topluyor; mutlak değer olduğu için poz
# gürültüsü NEGATİF katkı veremez, hep mesafe olarak birikir. EKF 50 Hz'de
# koştuğu için duran araçta bile aşamanın bitiş ölçütü sessizce büyür ve
# istasyona varılmadan sonrakine geçilir.
#
# Eşik hız cinsinden: örnek arası yer değiştirme bu hızdan yavaş bir hareket
# ima ediyorsa gürültü sayılır. Hız cinsinden olması EKF frekansından
# bağımsız kılıyor — örnekleme seyrelirse eşik kendiliğinden ölçekleniyor.
# Değer iki sınır arasında sıkışıyor:
#   ALT — elemesi beklenen gürültüyü geçirmemeli. 50 Hz'de örnek başına 1 mm
#         titreme 0,05 m/s eder; eşik 0,05 seçilirse tam sınırda kalır ve
#         hiçbir şey elenmez (ilk denemede böyle yazıldı, test yakaladı).
#   ÜST — aracın yerinden kalkabildiği ölçülmüş en düşük hızı (0,45 m/s,
#         imu_guvenlik.TABAN_HIZ) elememeli.
# 0,15 m/s ikisinin arasında: örnek başına 3 mm'ye kadar titremeyi eler,
# kalkış hızının üçte biri kalır.
#
# 🔑 Hata yönü bilerek seçildi: eşik yüksek kalırsa yol EKSİK sayılır ve aşama
# bütçesinde biter (araç istasyonu geçmez, timeout'a düşer); düşük kalırsa yol
# FAZLA sayılır ve araç istasyona varmadan sonrakine geçer. İkincisi puanı
# doğrudan götürüyor, o yüzden şüphede yüksek taraf tercih edildi.
#
# 🔴 Gürültünün GERÇEK büyüklüğü ölçülmedi. Ölçümü: araç dursun, 60 s
#    /odometry/filtered izlenip konumun ne kadar gezdiğine bakılsın.
KAYAN_YOL_TABAN_HIZ_MS = 0.15   # [m/s] bunun altını ima eden yer değiştirme yola sayılmaz

# Mod yönetimi
MOD_KOMUT_TOPIC   = "/mod/komut"    # yazılımsal/GCS mod değiştirme (UInt8)
MOD_AKTIF_TOPIC   = "/mod/aktif"    # geçerli mod (UInt8)

# /mod/aktif'in değerleri. Burada duruyorlar çünkü yayıncı (mod_yoneticisi) ve
# tüketici (ackermann_converter) ayrı düğümler; iki yerde tutulan bir sayı er
# geç ayrışır ve ayrıştığı gün kimse fark etmez.
# FULL_AUTO 1 değil 2: panoyu, misyon_fsm'i ve kayıtlı bag'leri bu değer
# bağlıyor, kaydırmak protokolü bozar.
MOD_MANUAL    = 0
MOD_FULL_AUTO = 2

# /mod/aktif 1 Hz nabız (mod_yoneticisi `create_timer(1.0, _mod_yayinla)`).
# Bundan eskisi "kip bilinmiyor" sayılır. Üç nabız payı bırakılıyor.
MOD_BAYATLAMA_S = 3.0
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
#
# Sürüş kartı ham kanalları göndermiyor: yalnız CH1 (direksiyon) ve CH9 (mod
# anahtarı) 0x3A ile geliyor, gaz/fren türetilmiş oranlar hâlinde 0x36/0x37'de.
# Dizinin şekli bozulmadı çünkü mod_yoneticisi, taret_rc_koprusu ve pano bu
# indislere göre yazılmış. seri_kopru alanları elindeki en yakın karşılıkla
# doldurur; ayrıntı için oradaki _rc_yayinla.
#
# ⚠️ data[2] ham CH9 DEĞİLDİR. CH9 üç konumlu ve orta konumu 1500 µs'e denk
# geliyor; bu değeri eşikleyen taraf kullanılmayan orta kipi otonom okurdu.
# Alan kartın çözdüğü kipten (KART_KIP_TOPIC) türetilir, ham CH9 teşhis
# için ayrı bir indiste taşınır.
RC_INPUT_TOPIC = "/rc_input"

# ─────────────────────────────────────────────
# SÜRÜŞ KARTI (Nucleo-F767ZI) DURUM YAYINLARI
# Üretici: seri_kopru — kartın 0x30–0x3C telemetri bloğundan
# ─────────────────────────────────────────────

# Kartın çözdüğü sürüş kipi (UInt8): 0 manuel · 1 kullanılmıyor · 2 otonom.
# Kip anahtarı (SwC/CH9) kartta okunuyor, kararı kart veriyor — Jetson'ın
# oyu yok. Sıralama: güvenlik > kumanda > Jetson.
KART_KIP_TOPIC = "/kart/kip"

# Kart arıza bayrakları (UInt16): 0x35 v0'daki HATA_* biti kümesi.
KART_HATA_TOPIC = "/kart/hata"

# Kart sürüş durum bayrakları (UInt16): 0x36 v1'deki DRM_* biti kümesi.
KART_DURUM_TOPIC = "/kart/durum"

# Kartın Jetson'a dair gördüğü bayraklar (UInt16): 0x39 v1'deki JDR_* kümesi.
# JDR_LINK düşükse kart bizi canlı görmüyordur ve komutlarımız yok sayılıyordur;
# "gönderiyorum ama dinlemiyor" durumunun tek görünür yeri burasıdır.
KART_LINK_TOPIC = "/kart/link"

# Kart protokol/yapı sürümü (UInt16MultiArray [protokol, yapi]) — 0x3B.
KART_SURUM_TOPIC = "/kart/surum"

# Kartın ürettiği sürüş çıkışı (Int16MultiArray [gaz_mv, fren_binde]) — 0x37.
# "Fren gerçekten sıkıldı mı" sorusunun cevabı burası: fren alanı işaretlidir,
# negatif değer aktüatörün açma yönünde AKTİF sürüldüğünü gösterir ve bunu
# yapan kumanda kolu olur. Bizim 0x08'imiz işaretsiz olduğu için negatif değer
# bizden gelmez.
KART_SURUS_TOPIC = "/kart/surus"

# Kartın ANLADIĞI sürüş komutu (Int16MultiArray [hiz_mms, aci_1_100_derece]) — 0x38.
# Gönderdiğimiz 0x01 ile farkı tek başına teşhistir: ölçek hatası, işaret hatası
# ve kayıp paket burada görünür. Manuel kipte de basılıyor, yani ölçek
# doğrulaması araç kımıldamadan yapılabilir.
KART_KABUL_TOPIC = "/kart/kabul"

# Kartın çalışma süresi [saniye] (UInt16) — 0x35 v1.
# Kart resetlendiğinde enkoder sayımı da sıfırlanıyor ve bunun başka görünür
# izi yok; sayaç sıçramasının gerçek mi reset mi olduğu buradan anlaşılır.
KART_CALISMA_TOPIC = "/kart/calisma_suresi"

# Hat sağlığı (UInt16MultiArray [alinan, bozuk]) — 0x3C, 1 Hz.
# Alanlar 32 bit sayaçların alt 16 biti; mutlak değer değil ARTIŞ okunur.
KART_HAT_TOPIC = "/kart/hat"

# Kartın o an hangi ayarlarla çalıştığı (0x3E). Kart ayarları flash'a
# yazmadığı için bu konu "kart ne hatırlıyor" sorusunun tek cevabı: reset
# sonrası buradaki değerler sıfırlanır ve hız alanı sessizce 0'a döner.
# İçerik: [kimlik, ham değer] — ölçek pure_logic.ayar_deger ile çözülür.
KART_AYAR_TOPIC = "/kart/ayar"

# ─── Sürüş kartı protokolü — bit ve sürüm sabitleri ──────────────────────────
#
# Kaynak: elektrik ekibinin arayüz sözleşmesi (4 Eylül 2026). Firmware kaynağı
# paylaşılmıyor; bu sabitlerin dayanağı sözleşme metnidir.

# 0x35 v0 — HATA_* arıza bayrakları
HATA_BNO_YOK        = 0x01   # IMU cevap vermiyor
HATA_BNO_KALIB      = 0x02   # IMU kalibrasyonu yetersiz
HATA_ENK_SESSIZ     = 0x04   # araç hareket ederken enkoder kımıldamıyor
HATA_GOST_SESSIZ    = 0x08   # gösterge ucu darbe basmıyor — ileri hız kaynağı sustu
HATA_ESTOP_UYUSMAZ  = 0x10   # iki E-STOP okuması 100 ms'den uzun çelişiyor
HATA_RC_YOK         = 0x20   # iBUS çerçevesi yok = KOPUK KABLO
HATA_FREN_STALL     = 0x40   # fren aynı yönde 4200 ms sürdü
HATA_GAZ_YOK        = 0x80   # gaz DAC'ına ulaşılamıyor

# 0x36 v1 — DRM_* sürüş durum bayrakları
DRM_KESME  = 0x01   # SwA düşük — kumandadan kesme
DRM_TARET  = 0x02   # SwB yüksek — taret kipi
DRM_GERI   = 0x04   # geri vites devrede
DRM_SSR    = 0x08   # donanımı söküldü — KULLANILMAZ
DRM_GECIS  = 0x10   # yön değiştirme sürüyor, gaz kilitli
DRM_ISIK   = 0x20   # aydınlatma açık
# Lazer komutunun uygulandığını bildirir — lazerin yandığını ÖLÇMEZ. Donanımda
# akım ya da foto geri beslemesi yok; "röle sürüldü" onayı, "ışık çıktı" onayı
# değildir. Atış zinciri bunu bilerek kurulmalı.
DRM_LAZER  = 0x40

# 0x39 v1 — JDR_* kartın Jetson'a dair gördükleri
JDR_LINK  = 0x01   # Jetson canlı — son paket 700 ms içinde
JDR_ESTOP = 0x02   # Jetson E-STOP ilan etti
JDR_DUR   = 0x04   # son komut PKT_J_DUR, taze sürüş komutu bekleniyor
JDR_ELLE  = 0x08   # elle kip açık — direksiyon tezgâh kipinde

# 0x3B v0 — beklenen protokol sürümü.
# Protokol numarası yalnız paket anlamları değişince artar (alan eklenir, ölçek
# değişir, bit kayar). Firmware yapı numarası (v1) davranış değiştiren HER
# yüklemede artıyor; onu karşılaştırmak ilk güncellemede sahte alarm verir.
BEKLENEN_PROTOKOL_SURUMU = 1

# Kartın otonom dalda uyguladığı hız kısıtları. Kart komutu reddetmez, kırpar;
# kırpma 0x38'de (kartın anladığı hız) görünür.
#   tavan  : üstü sessizce kırpılır
#   taban  : sıfır olmayan ama tabanın altındaki komutlar buna YÜKSELTİLİR —
#            o aralıkta motor dönüyor ama araç kalkmıyordu
#   ölü bölge: altındaki komutlar rölanti sayılır
# 🔑 Tabanın sonucu: yavaşlama rampasının son bölümü 0,20'de takılıyor, yani
# duruş 0,20 m/s'den sıfıra basamaktır. Duruş hassasiyeti bunu hesaba katmalı.
KART_HIZ_TAVAN     = 1.50   # [m/s]
KART_HIZ_TABAN     = 0.20   # [m/s]
KART_HIZ_OLU_BOLGE = 0.01   # [m/s]

# Kart kipleri (0x39 v0)
KART_KIP_MANUEL = 0
# SwC ORTA KADEME — yumuşak E-STOP. Kart gazı, freni, direksiyonu ve tareti
# aynı anda kilitliyor; araç bu kipte hiçbir komuta cevap vermez. Jetson
# tarafında otonom sayılmaz, ama "manuel" diye göstermek de yanlış: operatör
# panoda manuel görüp aracın neden sürmediğini arar.
KART_KIP_BOS    = 1
KART_KIP_OTONOM = 2

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
