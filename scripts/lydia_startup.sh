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
source /home/lydia/lydia_ws/install/setup.bash

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
# İkisi aynı anda yayınlarsa TF ağacı iki farklı konum arasında titrer ve SLAM
# haritayı bozuk kapatır — bu yüzden anahtar tek, seçim karşılıklı dışlamalı.
: "${ENKODER_AKTIF:=0}"
# HAM_ENKODER: /enkoder/ham üzerinden çiğ AS5600 ADC'si yayınlanır. Kalibrasyon
# ve teşhis içindir (scripts/sensor_dogrula.py -p test:=ham), sürüşte kapalı.
: "${HAM_ENKODER:=false}"
# ENKODER_KANALI: arka aks tek parça olduğu için tek enkoder yeterli; Mega yine
# iki analog kanal gönderdiğinden bağlı olan burada seçilir (sol=A0, sag=A1).
: "${ENKODER_KANALI:=sol}"

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
for _p in web_dashboard.py yolo_detection_node preprocessing_node \
          usb_cam_node_exe apc_camera_node \
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
if [ "$ENKODER_AKTIF" = "1" ]; then
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
ros2 run teknofest_ika preprocessing_node --ros-args \
    -p flip_taret:=true \
    -p derinlik_isle:=false > "$LOG/preprocessing.log" 2>&1 &
sleep 5
ros2 run teknofest_ika yolo_detection_node > "$LOG/yolo.log" 2>&1 &
sleep 3

# ── SLAM ─────────────────────────────────────────────────────────────────────
# TF zincirinin odom→base_footprint halkasını yalnız TEK bir kaynak basar:
# enkoder yokken statik yayıncı, varken seri_kopru'nun odometrisi.
if [ "$ENKODER_AKTIF" = "1" ]; then
    echo "odom→base_footprint: seri_kopru odometrisi (statik yayıncı atlandı)"
else
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint \
        > "$LOG/tf_odom.log" 2>&1 &
fi
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link \
    > "$LOG/tf_base.log" 2>&1 &
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
# ⚠️ AŞAĞIDAKİ KONUM ÖLÇÜLMEDİ — urdf/arac.urdf kamera_joint'inden alındı
#    (x=0.55 ileri, z=0.20 yukarı, base_link'e göre). OS30A menzili 0.02–2.5 m
#    olduğu için 10 cm'lik hata bile engelleri gözle görülür kaydırır.
#    Şerit metreyle ölç ve OS30A_X_M / OS30A_Z_M ile geç.
: "${OS30A_TF_AKTIF:=1}"
: "${OS30A_X_M:=0.55}"    # ⚠️ PLACEHOLDER — base_link'ten kamera gövdesine ileri
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
: "${NAV2_AKTIF:=0}"
if [ "$NAV2_AKTIF" = "1" ]; then
    if [ "$ENKODER_AKTIF" != "1" ]; then
        echo "UYARI: NAV2_AKTIF=1 ama ENKODER_AKTIF=0 — Nav2 sabit odometriyle" \
             "aracı hareketsiz sanar. Nav2 başlatılmadı."
    else
        # EKF: /odom + /imu/data → /odometry/filtered (Nav2'nin odom_topic'i)
        ros2 run robot_localization ekf_node --ros-args \
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

# ── İzleme ───────────────────────────────────────────────────────────────────
ros2 run foxglove_bridge foxglove_bridge > "$LOG/foxglove.log" 2>&1 &
sleep 2
# Nişan paneli işlenmiş görüntüyü alsın (çevirme preprocessing'de yapılıyor).
python3 "$WS/scripts/web_dashboard.py" --ros-args \
    -r /camera/taret/image_raw:=/camera/taret/image_processed \
    > "$LOG/web_dashboard.log" 2>&1 &

echo "LYDİA açılış yığını başlatıldı — loglar: $LOG"
wait
