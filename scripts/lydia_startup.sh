#!/bin/bash
# lydia_startup.sh — LYDİA açılış yığını (systemd: lydia_autostart.service)
#
# Ağ hazır olduğunda tüm algı, kontrol ve izleme düğümlerini sırayla başlatır.
# Sıra önemlidir: sensörler → algı → kontrol → izleme. Aradaki beklemeler USB
# cihazlarının numaralandırılması ve düğümlerin abonelik kurması içindir.
#
# DDS: paylaşılan bellek taşıması kapalı, yalnız UDP kullanılıyor. /dev/shm'de
# bayat fastrtps kilit dosyaları kaldığında düğümler birbirini bulamıyordu
# ("open_and_lock_file failed"); UDP-only profil bu sınıf hatayı tamamen
# ortadan kaldırıyor.

source /opt/ros/humble/setup.bash

# Derlenmemiş ya da yarım derlenmiş workspace sessizce geçilmemeli: overlay
# yoksa aşağıdaki her `ros2 run teknofest_ika ...` çağrısı "Package not found"
# ile kendi log dosyasına düşer, betik "başlatıldı" der ve dışarıdan görünen
# tablo boş topic listesi olur. Bir kez `rm -rf build install` + tek paket
# derlemesi bu duruma soktu ve DDS arızası sanıldı.
_OVERLAY=${OVERLAY:-/home/lydia/lydia_ws/install/setup.bash}   # kuru test için geçersiz kılınabilir
if [ ! -f "$_OVERLAY" ]; then
    echo "HATA: $_OVERLAY yok — workspace derlenmemiş." >&2
    echo "      cd ~/lydia_ws && colcon build --symlink-install" >&2
    exit 1
fi
source "$_OVERLAY"

# Dosyanın varlığı yetmez: --packages-select ile tek paket derlendiğinde
# overlay durur ama teknofest_ika içinde olmayabilir.
if ! ros2 pkg prefix teknofest_ika >/dev/null 2>&1; then
    echo "HATA: teknofest_ika paketi overlay'de yok — derleme eksik." >&2
    echo "      cd ~/lydia_ws && colcon build --symlink-install   (tam derleme)" >&2
    exit 1
fi

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/lydia/lydia_ortam/udp_only.xml
unset ROS_DISCOVERY_SERVER

# systemd servisi durdururken SIGTERM gönderir; alt süreçler yakalanmazsa
# hayatta kalıp servisi "deactivating" durumunda kilitliyorlar.
temizle() {
    trap - TERM INT
    kill $(jobs -p) 2>/dev/null
    sleep 2
    kill -9 $(jobs -p) 2>/dev/null
    exit 0
}
trap temizle TERM INT

# ── Yığın anahtarları ───────────────────────────────────────────────────────
# ENKODER_AKTIF: odom→base_footprint dönüşümünü kimin yayınlayacağını belirler.
#   0 → enkoder yok sayılır; dönüşümü statik yayıncı basar (araç hep başlangıç
#       noktasındaymış gibi görünür, SLAM tarama eşlemeyle yürür).
#   1 → dönüşümü seri_kopru'nun odometrisi basar; statik yayıncı başlatılmaz.
# NAV2_AKTIF=1 iken sahip üçüncü bir adaya, EKF'e geçer: seri_kopru /odom'u
# yalnız ölçüm olarak yayınlar, TF'i EKF basar (bkz. _TF_SAHIBI aşağıda).
# Aynı halkayı iki kaynak basarsa TF ağacı iki konum arasında titrer ve SLAM
# haritayı bozuk kapatır — bu yüzden seçim üç yönlü ve karşılıklı dışlamalı.
: "${ENKODER_AKTIF:=0}"
# HAM_ENKODER: /enkoder/ham üzerinden çiğ AS5600 ADC'si yayınlanır. Kalibrasyon
# ve teşhis içindir (scripts/sensor_dogrula.py -p test:=ham), sürüşte kapalı.
: "${HAM_ENKODER:=false}"
# ENKODER_KANALI: arka aks tek parça olduğu için tek enkoder yeterli; Mega yine
# iki analog kanal gönderdiğinden bağlı olan burada seçilir (sol=A0, sag=A1).
: "${ENKODER_KANALI:=sol}"
# NAV2_AKTIF burada erken okunuyor: TF sahibinin kim olduğu seri_kopru
# başlatılmadan önce bilinmeli, Nav2 bloğu ise betiğin çok sonrasında.
# Anahtarın ön koşulları o bloğun başında yazılı.
: "${NAV2_AKTIF:=0}"
# IMU_GUVENLIK_AKTIF: yatış açısına göre /speed_limit yayınlar ve 15°'de
# E-STOP zorlar. Kapalı çünkü eşiği IMU'nun montaj yönüne güveniyor: BMI160
# karta dönük lehimliyse araç düz dururken bile devrilmiş sanılır ve sürekli
# E-STOP basar. Açmadan önce araç düz dururken /imu/data'nın roll ve pitch'i
# ~0 okumalı (gerekirse IMU_ROLL/PITCH/YAW_RAD ile düzelt).
: "${IMU_GUVENLIK_AKTIF:=0}"
# TARET_AKTIF: nişan alma ve taret aktüasyonu. Kapalı çünkü /turret/cmd'yi
# tüketen İKİ aday var ve hangisinin araçta gerçek olduğu doğrulanmadı:
#   taret_rc_koprusu   → seri port → Turret UNO   (düğümün kendi belgesi
#                        servoların UNO'da olduğunu söylüyor)
#   servo_controller_node → Jetson I2C → PCA9685 → /taret/pan,/taret/tilt
#                        → seri_kopru → Mega
# İkisi birden koşarsa taret iki kaynaktan sürülür. TARET_YOLU ile seç.
: "${TARET_AKTIF:=0}"
: "${TARET_YOLU:=uno}"        # uno | pca9685
# KAYIT_AKTIF: veri_paketi rosbag kaydı. Kapalı — Jetson'ın diski dolarsa
# loglar ve harita da yazılamaz.
: "${KAYIT_AKTIF:=0}"

if [ "$ENKODER_AKTIF" != "1" ]; then
    _TF_SAHIBI=statik
elif [ "$NAV2_AKTIF" = "1" ]; then
    _TF_SAHIBI=ekf
else
    _TF_SAHIBI=seri_kopru
fi
echo "odom→base_footprint sahibi: $_TF_SAHIBI"

WS=${WS:-/home/lydia/lydia_ws/src/teknofest_ika_yazilim}
LOG=${LOG:-/home/lydia/lydia_log}
mkdir -p "$LOG"
cd "$WS" || exit 1

# Ağ arayüzü gelene kadar bekle (en fazla 60 s)
for _ in $(seq 1 30); do
    ip -4 addr show | grep -q '192\.168\.100\.' && break
    sleep 2
done

# Bayat DDS kilitleri temizlensin (önceki oturumdan kalmış olabilir)
rm -f /dev/shm/fastrtps* 2>/dev/null

# Önceki oturumdan artan düğümleri kapat. systemd yeniden başlatırken eski
# süreçler ölmezse port 8080 ve seri portlar meşgul kalıyor, yeni düğümler
# sessizce açılamıyordu.
#
# Liste bu betiğin başlattığı HER süreci kapsamalı. Temiz `systemctl stop`'ta
# SIGTERM trap'i çocukları zaten öldürüyor; buradaki tarama trap'in çalışmadığı
# hâller içindir — betiğin elle yeniden çalıştırılması, kill -9, ya da Jetson'ın
# kendiliğinden resetlenmesi. Eksik bırakılan bir ad yarı temizlenmiş bir yığın
# üretir: en tehlikelisi static_transform_publisher, çünkü ikinci bir kopya
# odom→base_footprint'i aynı anda basıp TF'i titretir.
for _p in web_dashboard.py yolo_detection_node preprocessing_node \
          usb_cam_node_exe apc_camera_node \
          static_transform_publisher \
          e_stop_node watchdog anti_rollback imu_guvenlik \
          targeting_node taret_rc_koprusu servo_controller_node veri_paketi \
          ekf_node yolo_adapter_node terrain_adapter cone_fusion_node \
          kayar_engel_kalman kayar_engel_costmap misyon_fsm \
          controller_server planner_server bt_navigator behavior_server \
          smoother_server velocity_smoother lifecycle_manager \
          seri_kopru mod_yoneticisi ackermann_converter \
          async_slam_toolbox_node map_image_node foxglove_bridge; do
    pkill -f "$_p" 2>/dev/null
done

# LiDAR ayrı ele alınıyor: sürücü kapanırken seri portu geç bırakıyor, hemen
# yeniden açılırsa "cannot bind to serial port" verip düşüyor.
pkill -f ydlidar_ros2_driver_node 2>/dev/null
sleep 8

# ── Sensörler ────────────────────────────────────────────────────────────────
# LiDAR: fixed_resolution=false şart. true iken sürücü 430 noktaya sabitlemeye
# çalışıyor, gerçek tarama 400-642 nokta geldiği için /scan hiç yayınlanmıyordu.
for _deneme in 1 2 3; do
    ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
        params_file:=/home/lydia/lydia_ortam/tmini_pro.yaml \
        > "$LOG/lidar.log" 2>&1 &
    sleep 14
    grep -q "Lidar has started" "$LOG/lidar.log" && break
    echo "LiDAR açılmadı (deneme $_deneme), port serbest bırakılıp tekrar denenecek"
    pkill -f ydlidar_ros2_driver_node 2>/dev/null
    sleep 8
done

# Derinlik kamerası (OS30A) — /apc/depth/image_raw renklendirilmiş derinlik,
# /apc/left/image_color stereo sol göz renkli görüntü.
ros2 launch ydlidar_os30a apc_camera_launch.py > "$LOG/apc.log" 2>&1 &
sleep 14

# Kameralar udev symlink'i üzerinden açılır; USB düğüm numaraları (video4/6…)
# takılma sırasına göre kayıyor, symlink cihaz kimliğine bağlı olduğu için
# sabit kalıyor.
#
# MJPEG şart: her iki kamera da YUYV modunda 640x480'i 1 Hz veriyor, usb_cam
# o modda veri gelmeden zaman aşımına düşüp çöküyor.

# Ön kamera — icSpring WebCamera (32e6:9211)
if [ -e /dev/kamera_on ]; then
    ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p camera_name:=on_kamera \
        -p video_device:=/dev/kamera_on \
        -p image_width:=640 -p image_height:=480 \
        -p pixel_format:=mjpeg2rgb -p framerate:=30.0 \
        -p qos_history_policy:=keep_last -p qos_history_depth:=1 \
        -r /image_raw:=/camera/image_raw \
        -r /camera_info:=/camera/camera_info \
        > "$LOG/on_cam.log" 2>&1 &
else
    echo "UYARI: ön kamera (/dev/kamera_on) bulunamadı"
fi
sleep 5

# Nişan kamerası — Microdia Hy-13M (0c45:6370)
if [ -e /dev/kamera_nisan ]; then
    ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p camera_name:=nisan_kamera \
        -p video_device:=/dev/kamera_nisan \
        -p image_width:=640 -p image_height:=480 \
        -p pixel_format:=mjpeg2rgb -p framerate:=30.0 \
        -p qos_history_policy:=keep_last -p qos_history_depth:=1 \
        -r /image_raw:=/camera/taret/image_raw \
        -r /camera_info:=/camera/taret/camera_info \
        > "$LOG/nisan_cam.log" 2>&1 &
else
    echo "UYARI: nişan kamerası (/dev/kamera_nisan) bulunamadı"
fi
sleep 6

# ── Gövde bağlantısı ve kontrol zinciri ──────────────────────────────────────
if [ "$_TF_SAHIBI" = "seri_kopru" ]; then
    _SERI_TF=true
else
    _SERI_TF=false
fi
ros2 run teknofest_ika seri_kopru --ros-args \
    -p port:=/dev/mega -p baud:=115200 \
    -p publish_tf:="$_SERI_TF" \
    -p enkoder_kanali:="$ENKODER_KANALI" \
    -p ham_enkoder:="$HAM_ENKODER" > "$LOG/seri_kopru.log" 2>&1 &
sleep 8
ros2 run teknofest_ika mod_yoneticisi      > "$LOG/mod_yoneticisi.log" 2>&1 &
sleep 3
ros2 run teknofest_ika ackermann_converter > "$LOG/ackermann.log" 2>&1 &
sleep 3
# E-STOP toplayıcı. Dört kaynağı (Arduino butonu, IMU devrilme, RC sinyal
# kaybı, panodan GCS komutu) tek /e_stop'ta birleştirir. Bu düğüm olmadan
# /e_stop'a HİÇ yayın yapılmaz; ona abone olan seri_kopru, mod_yoneticisi,
# ackermann_converter, misyon_fsm ve pano sürekli "E-STOP yok" okur ve
# panodaki durme düğmesi hiçbir şey yapmaz. Aracın kendi donanım kesmesi
# ayrı bir yolla çalışsa bile ROS tarafındaki bütün kilitler buna bağlı.
# Jetson.GPIO kurulu değilse düğüm gpio_mod'u kendi kapatır, yazılımsal
# kaynaklar çalışmaya devam eder.
ros2 run teknofest_ika e_stop_node         > "$LOG/e_stop.log" 2>&1 &
sleep 3
# Eğimde geri kaymayı yakalayıp karşı komut basar. /odom'a bağlı olduğu için
# ENKODER_AKTIF=0 iken ölçüm sabit kalır ve düğüm hiç tetiklenmez — zararsız,
# ama gerçek koruma ancak enkoderle gelir.
ros2 run teknofest_ika anti_rollback       > "$LOG/anti_rollback.log" 2>&1 &
sleep 3

# ── Algı zinciri ─────────────────────────────────────────────────────────────
# preprocessing ön kameradan (/camera/image_raw) besleniyor ve
# /camera/image_processed üretiyor; yolo_detection_node oradan okur.
# Nişan kamerası araca ters monte; görüntü burada çevriliyor ki hem panel hem
# targeting düz görsün. Ön kamera için -p flip_ana:=true eklenebilir.
#
# derinlik_isle kapalı: /depth/points/filtered'ın tek tüketicisi Nav2 costmap
# katmanıydı ve Nav2 kapalı. Açıkken kare başına KD-tree kurulup nokta bulutu
# Python'da geziliyor, çıktıyı kimse okumuyordu. Derinlik GÖRÜNTÜSÜ etkilenmez
# (/apc/depth/image_raw doğrudan OS30A'dan panele gider).
#
# /scan_lidar remap'i ZORUNLU: preprocessing_node tarama girişini bu adda
# bekliyor, ydlidar sürücüsü ise /scan basıyor. Remap olmadan cb_scan hiç
# tetiklenmez ve /scan/filtered ÜRETİLMEZ — Nav2'nin obstacle_layer scan
# kaynağı (iki costmap'te de), cone_fusion_node, kayar_engel_kalman ve
# kayar_engel_costmap hep birden susar, hiçbiri hata vermeden.
# (Launch dosyasındaki çözüm sürücüyü /scan_raw'a remap edip scan_relay'i
# araya koymaktı; relay sürücünün eski 0x202 zaman damgası hatası içindi,
# doğrudan remap aynı işi tek satırda yapıyor.)
ros2 run teknofest_ika preprocessing_node --ros-args \
    -r /scan_lidar:=/scan \
    -p flip_taret:=true \
    -p derinlik_isle:=false > "$LOG/preprocessing.log" 2>&1 &
sleep 5
ros2 run teknofest_ika yolo_detection_node > "$LOG/yolo.log" 2>&1 &
sleep 3

# ── SLAM ─────────────────────────────────────────────────────────────────────
# TF zincirinin odom→base_footprint halkasını yalnız _TF_SAHIBI basar.
if [ "$_TF_SAHIBI" = "statik" ]; then
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint \
        > "$LOG/tf_odom.log" 2>&1 &
fi
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link \
    > "$LOG/tf_base.log" 2>&1 &
# EKF /imu/data'yı imu_link'ten base_footprint'e çevirebilmek için bu halkaya
# muhtaç; bulamazsa ölçümü SESSİZCE düşürür (robot_localization'ın
# lookupTransformSafe uyarısı kaynakta yorum satırı). O durumda EKF yalnız vx
# ile koşar, yön hiç güncellenmez ve araç estimatörde hep düz gider.
#
# ⚠️ DÖNÜŞ AÇILARI DOĞRULANMADI — urdf/arac.urdf imu_joint'inden alındı
#    (rpy 0 0 0). Yön füzyonunda önemli olan öteleme değil dönüştür: BMI160
#    karta ters ya da 90° dönük lehimliyse roll/pitch/yaw da o kadar yanlış
#    gelir. Araç düz dururken /imu/data'nın roll ve pitch'i ~0 okumuyorsa
#    burayı düzelt.
: "${IMU_X_M:=0}"
: "${IMU_Y_M:=0}"
: "${IMU_Z_M:=0}"
: "${IMU_ROLL_RAD:=0}"
: "${IMU_PITCH_RAD:=0}"
: "${IMU_YAW_RAD:=0}"
ros2 run tf2_ros static_transform_publisher \
    "$IMU_X_M" "$IMU_Y_M" "$IMU_Z_M" \
    "$IMU_YAW_RAD" "$IMU_PITCH_RAD" "$IMU_ROLL_RAD" \
    base_link imu_link > "$LOG/tf_imu.log" 2>&1 &
# LiDAR gövdeye 93.3° dönük monte (sahada huniyle çift yönlü kalibre edildi).
# huni_reaktif bu düzeltmeyi kendi içinde (aci_offset_deg) ham /scan üzerinde
# yapar ve TF'ten etkilenmez; Nav2 costmap ise YALNIZ TF'e bakar — dönüş burada
# verilmezse tüm engelleri 93° kaydırarak yerleştirir.
: "${LIDAR_YAW_RAD:=1.6284}"   # 93.3° — huni_reaktif aci_offset_deg ile aynı
: "${LIDAR_Z_M:=0.55}"         # zeminden tarama düzlemine, ölçüldü
ros2 run tf2_ros static_transform_publisher 0 0 "$LIDAR_Z_M" "$LIDAR_YAW_RAD" 0 0 \
    base_link laser_frame > "$LOG/tf_laser.log" 2>&1 &
# OS30A derinlik kamerası ayrı bir TF adasında duruyor: kendi launch'ı
# dm_base_frame → {points_frame, depth_frame, ...} dönüşümlerini basıyor ama
# dm_base_frame'i hiçbir şey base_link'e bağlamıyor. Bu halka olmadan Nav2'nin
# os30a_cloud kaynağı tek nokta bile dönüştüremez (costmap sürekli "Transform
# failure" basar) ve derinlik kamerası costmap'e hiçbir katkı yapmaz.
#
# ⚠️ AŞAĞIDAKİ KONUM ÖLÇÜLMEDİ — urdf/arac.urdf os30a_joint'inden alındı
#    (x=0.30 ileri, z=0.20 yukarı, base_link'e göre); urdf'te o da placeholder.
#    Ön kameranın kamera_joint'i (x=0.55) BAŞKA bir sensördür, karıştırma.
#    OS30A menzili 0.02–2.5 m olduğu için 10 cm'lik hata bile engelleri gözle
#    görülür kaydırır. Şerit metreyle ölç ve OS30A_X_M / OS30A_Z_M ile geç.
: "${OS30A_TF_AKTIF:=1}"
: "${OS30A_X_M:=0.30}"    # ⚠️ PLACEHOLDER — base_link'ten kamera gövdesine ileri
: "${OS30A_Y_M:=0.0}"     # ⚠️ PLACEHOLDER — yanal kaçıklık
: "${OS30A_Z_M:=0.20}"    # ⚠️ PLACEHOLDER — base_link'ten kamera gövdesine yukarı
if [ "$OS30A_TF_AKTIF" = "1" ]; then
    ros2 run tf2_ros static_transform_publisher \
        "$OS30A_X_M" "$OS30A_Y_M" "$OS30A_Z_M" 0 0 0 \
        base_link dm_base_frame > "$LOG/tf_os30a.log" 2>&1 &
fi
sleep 4
ros2 launch slam_toolbox online_async_launch.py > "$LOG/slam.log" 2>&1 &
sleep 14
ros2 run teknofest_ika map_image_node > "$LOG/map_image.log" 2>&1 &
sleep 3

# ── Nav2 yolu (tabela → arazi profili → planlama) ────────────────────────────
# Zincir: ön kamera → yolo_detection → yolo_adapter → /yolo/class_id →
#         terrain_adapter → Nav2 controller parametreleri.
# Engel katmanları: /scan/filtered (preprocessing), /costmap/cone_cloud
# (cone_fusion), /moving_obs_cloud (kayar_engel_costmap), /apc/points/data_raw.
#
# NAV2_AKTIF=1 yapmadan önce ÜÇ ön koşul sağlanmalı, aksi halde Nav2 aracı
# yanlış konumlandırır:
#   1) ENKODER_AKTIF=1 ve enkoder ölçümü doğrulanmış olmalı — Nav2 konumu
#      /odometry/filtered'dan alır, o da enkoder odometrisinden türer.
#   2) LIDAR_YAW_RAD gerçek montaj açısına ayarlanmış olmalı (bkz. yukarısı).
#   3) config/waypoints.yaml'daki koordinatlar doldurulmuş olmalı; hepsi 0.0
#      iken misyon_fsm her istasyonu aynı noktaya gönderir.
if [ "$NAV2_AKTIF" = "1" ]; then
    if [ "$ENKODER_AKTIF" != "1" ]; then
        echo "UYARI: NAV2_AKTIF=1 ama ENKODER_AKTIF=0 — Nav2 sabit odometriyle" \
             "aracı hareketsiz sanar. Nav2 başlatılmadı."
    else
        # EKF: /odom + /imu/data → /odometry/filtered (Nav2'nin odom_topic'i)
        # Düğüm adı açıkça veriliyor: ekf.yaml'daki parametre bloğu
        # `ekf_filter_node:` anahtarının altında ve ROS parametreleri düğüm
        # adına göre eşleşir. Ad tutmazsa blok hiç yüklenmez, EKF sensörsüz
        # varsayılanlarla açılır ve /odometry/filtered boş kalır — hata
        # vermeden. `ros2 run ... ekf_node` bu adı garanti etmiyor.
        ros2 run robot_localization ekf_node --ros-args \
            -r __node:=ekf_filter_node \
            --params-file "$WS/config/ekf.yaml" > "$LOG/ekf.log" 2>&1 &
        sleep 4
        ros2 launch nav2_bringup navigation_launch.py \
            use_sim_time:=false \
            params_file:="$WS/config/nav2_params.yaml" > "$LOG/nav2.log" 2>&1 &
        sleep 12
        # Tabela → arazi profili zinciri
        ros2 run teknofest_ika yolo_adapter_node  > "$LOG/yolo_adapter.log" 2>&1 &
        sleep 2
        ros2 run teknofest_ika terrain_adapter    > "$LOG/terrain_adapter.log" 2>&1 &
        sleep 2
        # Costmap besleyicileri
        ros2 run teknofest_ika cone_fusion_node       > "$LOG/cone_fusion.log" 2>&1 &
        sleep 2
        ros2 run teknofest_ika kayar_engel_kalman     > "$LOG/kayar_kalman.log" 2>&1 &
        sleep 2
        ros2 run teknofest_ika kayar_engel_costmap    > "$LOG/kayar_costmap.log" 2>&1 &
        sleep 2
        ros2 run teknofest_ika misyon_fsm             > "$LOG/misyon_fsm.log" 2>&1 &
        sleep 2
        echo "Nav2 yolu başlatıldı (tabela → terrain_adapter → controller)"
    fi
fi

# ── Taret ────────────────────────────────────────────────────────────────────
if [ "$TARET_AKTIF" = "1" ]; then
    # HSV+Hough+PID hedef takibi → /turret/cmd, ve /targeting/status.
    # misyon_fsm'in ShootState'i nişan onayını YALNIZ bu topic'ten alır;
    # düğüm koşmazsa üç atış denemesi de zaman aşımına düşer.
    ros2 run teknofest_ika targeting_node  > "$LOG/targeting.log" 2>&1 &
    sleep 3
    case "$TARET_YOLU" in
        uno)
            ros2 run teknofest_ika taret_rc_koprusu \
                > "$LOG/taret_koprusu.log" 2>&1 &
            ;;
        pca9685)
            ros2 run teknofest_ika servo_controller_node \
                > "$LOG/servo_controller.log" 2>&1 &
            ;;
        *)
            echo "UYARI: TARET_YOLU='$TARET_YOLU' tanınmadı (uno|pca9685) —" \
                 "aktüasyon başlatılmadı, taret yalnız nişan alır."
            ;;
    esac
    sleep 2
    echo "Taret yolu başlatıldı: $TARET_YOLU"
fi

# ── İzleme ───────────────────────────────────────────────────────────────────
# Sensör canlılık bekçisi: /scan_lidar, /scan/filtered, /odom, /imu/data ve
# tespit akışını zaman aşımıyla izleyip /sensor/fault basar. Yığındaki sessiz
# kopuklukları ilk fark edecek düğüm budur — mesajları deserialize etmediği
# için maliyeti düşük.
ros2 run teknofest_ika watchdog          > "$LOG/watchdog.log" 2>&1 &
sleep 2
if [ "$IMU_GUVENLIK_AKTIF" = "1" ]; then
    ros2 run teknofest_ika imu_guvenlik  > "$LOG/imu_guvenlik.log" 2>&1 &
    sleep 2
fi
if [ "$KAYIT_AKTIF" = "1" ]; then
    ros2 run teknofest_ika veri_paketi   > "$LOG/veri_paketi.log" 2>&1 &
    sleep 2
fi
ros2 run foxglove_bridge foxglove_bridge > "$LOG/foxglove.log" 2>&1 &
sleep 2
# Nişan paneli işlenmiş görüntüyü alsın (çevirme preprocessing'de yapılıyor).
python3 "$WS/scripts/web_dashboard.py" --ros-args \
    -r /camera/taret/image_raw:=/camera/taret/image_processed \
    > "$LOG/web_dashboard.log" 2>&1 &

# ── Açılış doğrulaması ───────────────────────────────────────────────────────
# Betik buraya kadar yirmiden fazla süreç başlatıp hiçbirinin ayağa kalkıp
# kalkmadığına bakmıyordu; bir düğüm seri portu açamayınca ya da modeli
# yükleyemeyince tek belirti kendi log dosyasındaki satır oluyor ve arıza ancak
# araç komuta cevap vermeyince fark ediliyordu. Aşağısı ucuz bir varlık
# kontrolü: `ros2 node list` bir kez okunur, beklenen adlar aranır.
# Düğümlerin ÇALIŞTIĞINI değil AYAKTA olduğunu söyler — topic akışı için
# ros2 topic hz'e bakmak gerekir.
_BEKLENEN="seri_kopru mod_yoneticisi ackermann_converter e_stop_node
           anti_rollback preprocessing_node yolo_detection_node map_image_node
           watchdog web_dashboard"
if [ "$NAV2_AKTIF" = "1" ]; then
    _BEKLENEN="$_BEKLENEN ekf_filter_node controller_server yolo_adapter_node
               terrain_adapter cone_fusion_node kayar_engel_kalman
               kayar_engel_costmap misyon_fsm"
fi
[ "$TARET_AKTIF" = "1" ]        && _BEKLENEN="$_BEKLENEN targeting_node"
[ "$IMU_GUVENLIK_AKTIF" = "1" ] && _BEKLENEN="$_BEKLENEN imu_guvenlik"

sleep 10
_CANLI=$(ros2 node list 2>/dev/null)
_EKSIK=""
for _n in $_BEKLENEN; do
    echo "$_CANLI" | grep -qx "/$_n" || _EKSIK="$_EKSIK $_n"
done
if [ -n "$_EKSIK" ]; then
    echo "UYARI: ayağa kalkmayan düğümler:$_EKSIK"
    echo "       Sebebi kendi log dosyasında: ls -t $LOG | head"
else
    echo "Açılış doğrulaması: beklenen tüm düğümler ayakta."
fi

echo "LYDİA açılış yığını başlatıldı — loglar: $LOG"
wait
