# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
Gerçek Araç — Yarışma Launch Dosyası
======================================
Tüm stack: donanım + SLAM + Nav2 + uygulama node'ları

Senaryo A (önceden harita alındıysa):
  SLAM Toolbox localization modunda başlar — pre-built haritayı kullanır.
  Harita yolu: maps/gercek_harita.pgm  (gercek_harita.launch.py ile alınır)

Senaryo B (harita yoksa):
  SLAM mapping modunda başlar — sahayı haritalarken navigasyon yapar.

Modu maps/gercek_harita.pgm varlığına göre otomatik seçer.

Başlatma:
  ros2 launch teknofest_ika gercek_arac.launch.py

Sıralama:
  0s  → robot_state_publisher
  1s  → seri_kopru + YDLidar
  3s  → EKF
  5s  → SLAM Toolbox
  8s  → Nav2 navigation stack
  10s → uygulama node'ları (ackermann, fsm, güvenlik...)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _harita_olcusu(pgm_yolu: str) -> str:
    """PGM başlığından harita boyutu (metre). Okunamazsa '?'."""
    try:
        with open(pgm_yolu, 'rb') as f:
            alan = f.read(64).split()
        cozunurluk = 0.05   # maps/*.yaml resolution ile aynı varsayım
        return (f'{int(alan[1]) * cozunurluk:.1f} x '
                f'{int(alan[2]) * cozunurluk:.1f} m')
    except Exception:
        return '?'


def _dosya_tarihi(yol: str) -> str:
    import datetime
    try:
        return datetime.datetime.fromtimestamp(
            os.path.getmtime(yol)).strftime('%Y-%m-%d')
    except OSError:
        return '?'


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')
    nav2_pkg  = get_package_share_directory('nav2_bringup')

    with open(os.path.join(pkg_share, 'urdf', 'arac.urdf'), 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('package://teknofest_ika', 'file://' + pkg_share)

    # ── Robot State Publisher ─────────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen'
    )

    # ── YDLidar Tmini Pro ─────────────────────────────────────────────────────
    lidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[{
            'port':           '/dev/lidar',
            'frame_id':       'lidar_link',
            'ignore_array':   '',
            'baudrate':       230400,
            'lidar_type':     1,
            'device_type':    6,
            'sample_rate':    5,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
            'reversion':      False,
            'inverted':       False,
            'auto_reconnect': True,
            'isSingleChannel': False,
            'intensity':      False,
            'support_motor_dtr': False,
            'angle_max':      180.0,
            'angle_min':      -180.0,
            'range_max':      8.0,
            'range_min':      0.1,
            'frequency':      10.0,
            'invalid_range_is_inf': False,
        }],
        remappings=[('/scan', '/scan_raw')]
    )

    # ── Scan Relay — timestamp ve frame_id düzeltici ──────────────────────────
    scan_relay = Node(
        package='teknofest_ika',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
    )

    # ── OS30A Derinlik Kamerası (eYs3D BMVM0S30A) ────────────────────────────
    # Sürücü: eYs3D HD-DM-ROS2-SDK  →  package: dm_preview
    # Yayınlanan topic: /apc/points/data_raw (PointCloud2)
    # Kurulum: terminalden aşağıdaki adımları izle (bir kez yapılır):
    #   git clone https://github.com/eYs3D/HD-DM-ROS2-SDK-Release.git ~/eys3d_ws/src/dm_preview
    #   cd ~/eys3d_ws && rosdep install -i --from-path src -y
    #   colcon build --symlink-install
    #   echo "source ~/eys3d_ws/install/setup.bash" >> ~/.bashrc
    #
    # DEĞİŞİKLİK (2026-07-15, sahada doğrulandı): dm_preview paketinin
    # eys3d_ws derlemesi eksik/bozuk (vendor .so dosyası ve stereo_msgs
    # bağımlılığı eksik, "package 'dm_preview' not found" ile TÜM launch'ı
    # çökertiyordu). Aynı donanım için ÇALIŞAN bir alternatif paket var:
    # ydlidar_os30a (dev_ws'te ayrı kurulu). Elle kısaltılmış parametrelerle
    # değil, resmi launch dosyasıyla başlatılmalı — aksi halde derinlik
    # akışı hiç üretilmiyor (2026-07-10 oturumunda bulunan bilinen sorun).
    os30a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_os30a'),
                         'launch', 'apc_camera_launch.py')
        )
    )

    # ── E-STOP Node ───────────────────────────────────────────────────────────
    # Fiziksel buton Jetson'ın GPIO'suna değil sürüş kartına bağlı; durumu
    # 0x34 ile geliyor. gpio_mod kapalı — boştaki bir pini okumak 48 V'un
    # gürültüsü altında rastgele E-STOP üretir.
    e_stop = Node(
        package='teknofest_ika',
        executable='e_stop_node',
        name='e_stop_node',
        output='screen',
        parameters=[{
            'gpio_mod':   False,
            'publish_hz': 20.0,
        }]
    )

    # ── Donanım Köprüsü (seri_kopru → /odom + /imu/data) ─────────────────────
    seri_kopru = Node(
        package='teknofest_ika',
        executable='seri_kopru',
        name='seri_kopru',
        output='screen',
        parameters=[{'use_sim_time': False,
                     # Sürüş kartı Nucleo-F767ZI; scripts/lydia_startup.sh
                     # aynı iki değeri kendi başına geçiyor, ikisi birlikte
                     # güncellenmeli.
                     'port': '/dev/f767',
                     'baud': 921600,
                     # odom → base_footprint TF'i aşağıdaki EKF yayınlar
                     'publish_tf': False}]
    )

    # ── EKF (odom + IMU füzyon) ───────────────────────────────────────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': False}
        ]
    )

    # ── SLAM Toolbox — harita varsa localization, yoksa mapping ───────────────
    slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')
    gercek_harita = os.path.join(pkg_share, 'maps', 'teknofest_harita.pgm')

    if os.path.exists(gercek_harita):
        # Harita var → localization modu (haritayı yükle, yeni alan haritalama)
        # Hangi haritaya localize olunduğu EKRANA BASILIYOR: dosyanın varlığı
        # tek başına doğruluğunun kanıtı değil. Burada bir kez 16,05 x 1,85 m'lik
        # bir koridor testi haritası kalmış ve araç parkur yerine ona localize
        # olacak duruma gelmişti; kimse fark etmezdi çünkü mod sessizce seçiliyor.
        slam_exe = 'localization_slam_toolbox_node'
        slam_extra = {
            'use_sim_time': False,
            'map_file_name': os.path.splitext(gercek_harita)[0],
            'map_start_at_dock': True,
        }
        print(f'[SLAM] LOCALIZATION modu — harita: {gercek_harita}\n'
              f'[SLAM]   boyut: {_harita_olcusu(gercek_harita)}, '
              f'tarih: {_dosya_tarihi(gercek_harita)}\n'
              f'[SLAM]   Bu harita parkurun DEĞİLSE araç yanlış yere localize olur.')
    else:
        # Harita yok → mapping modu (sahayı haritala)
        slam_exe = 'async_slam_toolbox_node'
        slam_extra = {'use_sim_time': False}
        print('[SLAM] MAPPING modu — harita sıfırdan kuruluyor, map çerçevesinin '
              'orijini aracın şu anki yeri.')

    slam = Node(
        package='slam_toolbox',
        executable=slam_exe,
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, slam_extra]
    )

    # ── Nav2 (navigation stack) ───────────────────────────────────────────────
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':    'false',
            'params_file':     nav2_params,
            'use_composition': 'False',
        }.items()
    )

    # ── Uygulama Node'ları ────────────────────────────────────────────────────

    # ackermann_converter /mux/cmd_vel'i dinler — mod_yoneticisi mux çıkışı
    ackermann = Node(
        package='teknofest_ika', executable='ackermann_converter',
        name='ackermann_converter', output='screen',
        parameters=[{'use_sim_time': False, 'wheelbase': 1.44,
                     # max_speed burada verilmiyor: düğümün varsayılanı
                     # KART_HIZ_TAVAN'a bağlı ve kart zaten orada kırpıyor.
                     'max_steering_angle': 0.5236}]
    )
    veri_paketi = Node(
        package='teknofest_ika', executable='veri_paketi',
        name='veri_paketi', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    terrain_adapter = Node(
        package='teknofest_ika', executable='terrain_adapter',
        name='terrain_adapter', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    imu_guvenlik = Node(
        package='teknofest_ika', executable='imu_guvenlik',
        name='imu_guvenlik', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    anti_rollback = Node(
        package='teknofest_ika', executable='anti_rollback',
        name='anti_rollback', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    kayar_kalman = Node(
        package='teknofest_ika', executable='kayar_engel_kalman',
        name='kayar_engel_kalman', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    # Mod Yöneticisi — MANUAL/FULL_AUTO geçişleri ve cmd_vel mux
    mod_yoneticisi = Node(
        package='teknofest_ika', executable='mod_yoneticisi',
        name='mod_yoneticisi', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    # Taret RC Köprüsü — sağ stick MANUEL+SWB'de Turret UNO'ya pan/tilt iletir
    taret_rc_koprusu = Node(
        package='teknofest_ika', executable='taret_rc_koprusu',
        name='taret_rc_koprusu', output='screen',
        # port ve baud düğümün varsayılanından gelir (topics.py SERIAL_TARET,
        # SERIAL_BAUD_TARET); burada tekrarlanırsa iki yer ayrışır.
        parameters=[{'use_sim_time': False}]
    )
    # NOT: Koni tespiti costmap'e yalnız cone_fusion_node üzerinden girer
    # (aşağıda 'cone_fusion'), LiDAR+YOLO füzyonuyla /costmap/cone_cloud'a.
    # Ayrı bir PoseArray tabanlı koni kaynağı yok.

    # Kayar Engel Costmap — Kalman aktifken /scan → Nav2 ObstacleLayer
    kayar_costmap = Node(
        package='teknofest_ika', executable='kayar_engel_costmap',
        name='kayar_engel_costmap', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    misyon_fsm = Node(
        package='teknofest_ika', executable='misyon_fsm',
        name='misyon_fsm', output='screen',
        parameters=[{'use_sim_time': False}]
    )


    # ── Ana Kamera (YOLO'ya giden görüntü) ────────────────────────────────────
    # ESKİ VARSAYIM YANLIŞTI: burası bir CSI/IMX258 kamerası değil, gerçek
    # donanımda "WebCamera" (UVC) cihazı — /dev/video_imx258 hiçbir zaman
    # var olmayan bir udev symlink'iydi, v4l2_camera de bu kameranın tek
    # desteklediği MJPG formatını çözemiyordu (2026-07-14 oturumunda bulundu).
    # Ahmet'in oluşturduğu gerçek udev ismi: /dev/kamera_on (usb-2.3 portu).
    # Bu, ön kameranın TEK açıcısı — ayrı bir webcam_ileri node'u YOK
    # (2026-07-15 sahada doğrulandı: aynı cihazı iki node açınca çakışma
    # oluyordu). Ön kamera görüntüsüne ihtiyaç duyanlar (dashboard,
    # veri_paketi) CAMERA_IMAGE_TOPIC'e (bu node'un çıktısı) abone.
    imx258 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='kamera_ana',
        output='screen',
        parameters=[{
            'video_device':       '/dev/kamera_on',
            'image_width':        1280,
            'image_height':       720,
            'framerate':          30.0,
            'pixel_format':       'mjpeg2rgb',
            'camera_name':        'ana',
            'camera_info_url':    '',
            'auto_white_balance': True,
            'autoexposure':       True,
        }],
        remappings=[
            ('image_raw',   '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )

    # ── Görüntü Ön İşleme ─────────────────────────────────────────────────────
    # preprocessing_node topics.py sabitleriyle doğrudan /camera/image_raw ve
    # /camera/taret/image_raw'a abone olur — remap gerekmez (remap-bağımlı bir
    # tasarım, nişan kamerasının hiç remap edilmemesi nedeniyle hiç
    # işlenmemesine yol açmıştı).
    # /depth/points → OS30A derinlik kamerasından (/apc/points/data_raw)
    # /scan_lidar → preprocessing_node.py SCAN_LIDAR_TOPIC ile direkt abone, remap gerekmez
    preprocessing = Node(
        package='teknofest_ika', executable='preprocessing_node',
        name='preprocessing_node', output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/depth/points', '/apc/points/data_raw'),
        ]
    )

    # ── YOLOv8 TensorRT Dedeksiyonu ───────────────────────────────────────────
    # model_path: Jetson'da best.pt → scripts/export_tensorrt.py ile üretilir
    yolo_detection = Node(
        package='teknofest_ika', executable='yolo_detection_node',
        name='yolo_detection_node', output='screen',
        parameters=[{
            'use_sim_time':        False,
            'model_path':          'models/best.engine',
            'conf_thres':          0.75,  # topics.py YOLO_CONFIDENCE_THRESHOLD ile uyumlu
            'iou_thres':           0.45,
            'publish_debug_image': True,
        }]
    )

    # ── Koni Füzyon (YOLO + LiDAR → Nav2 costmap) ────────────────────────────
    cone_fusion = Node(
        package='teknofest_ika', executable='cone_fusion_node',
        name='cone_fusion_node', output='screen',
        parameters=[{
            'use_sim_time':      False,
            'camera_fov_deg':    60.0,
            'image_width':       1280,
            'cone_radius_m':     0.354,   # §6.7 azami taban 50 cm → çevrel yarıçap
            'cone_min_confidence':  0.45,
            'target_label':      '14',   # 14 = trafik_huni (alfabetik model sırası)
        }],
        remappings=[
            ('/ileri_kamera/camera_info', '/camera/camera_info'),
        ]
    )

    # ── Hedef Kilitleme (HSV + Hough + PID) ───────────────────────────────────
    targeting = Node(
        package='teknofest_ika', executable='targeting_node',
        name='targeting_node', output='screen',
        parameters=[{
            'use_sim_time':          False,
            'align_threshold_px':    10.0,
            'fire_lock_duration_sec': 0.5,
            'fire_cooldown_sec':     2.0,
            'publish_debug':         True,
            # HSV kalibrasyonu ve Hough yarıçapı artık düğümün varsayılanı
            # (topics.py NISAN_HSV_*). Burada tekrarlanmıyor: değerler yalnız
            # launch'ta durduğu sürece, açılış betiği launch'u kullanmadığı
            # için kalibrasyon araca hiç ulaşmıyordu.
            #
            # image_timeout_sec de düğümün varsayılanından gelir: 0,5 s üç
            # kameranın aynı USB2 hattını paylaştığı ölçülen ~7-9 Hz akışa
            # göre seçildi, buradaki 1,0 gerekçesizdi.
        }]
    )

    # ── Servo Kontrolcüsü (PCA9685 + GPIO ateş) ───────────────────────────────
    servo_controller = Node(
        package='teknofest_ika', executable='servo_controller_node',
        name='servo_controller_node', output='screen',
        parameters=[{
            'use_sim_time':   False,
            'use_pca9685':    True,
            'yaw_channel':    0,
            'pitch_channel':  1,
            'yaw_home_deg':   90.0,
            'pitch_home_deg': 90.0,
        }]
    )

    # ── SLAM Haritası → GCS Dashboard Görüntüsü ───────────────────────────────
    # /map (OccupancyGrid) → /map/image (Image) — slam_toolbox'tan sonra başlamalı
    map_image = Node(
        package='teknofest_ika', executable='map_image_node',
        name='map_image_node', output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ── Watchdog (Kritik topic sağlık izleme) ─────────────────────────────────
    # Bu launch dosyasında sürücü /scan_raw'a remap edilip scan_relay
    # /scan_lidar basıyor; Nav2 ve EKF de burada her zaman ayağa kalkıyor.
    # (Boot betiği ikisini de farklı kuruyor, orada parametreler farklı.)
    watchdog = Node(
        package='teknofest_ika', executable='watchdog',
        name='watchdog', output='screen',
        parameters=[{'use_sim_time': False,
                     'nav2_aktif': True,
                     'ham_tarama_topic': '/scan_lidar'}]
    )

    # ── YOLO Adapter — Detection2DArray → /ika/detections JSON köprüsü ───────
    # yolo_detection_node'dan sonra başlamalı (13s+)
    # img_cx/img_cy: YOLO giriş boyutuna göre (preprocessing_node 640x640 → 320,320)
    yolo_adapter = Node(
        package='teknofest_ika', executable='yolo_adapter_node',
        name='yolo_adapter_node', output='screen',
        parameters=[{
            'use_sim_time':   False,
            'img_cx':         320.0,
            'img_cy':         320.0,
            'conf_threshold': 0.75,  # topics.py YOLO_CONFIDENCE_THRESHOLD ile uyumlu
        }]
    )

    # ── Microcase 720P Webcam'ler — Şartname §6.12: 3 kamera zorunlu ────────────
    #
    # Gerçek udev isimleri Ahmet'in /etc/udev/rules.d/99-ika.rules kuralında
    # zaten oluşturulmuş (2026-07-13 doğrulandı) — aşağıdaki webcam_taret/
    # webcam_ileri/webcam_geri placeholder isimleri hiç var olmayan symlink'lerdi,
    # gerçek isimlerle (kamera_nisan, kamera_on, kamera_arka) değiştirildi.

    # Taret kamerası — nişan alma (taret üzeri)
    webcam_taret = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam_taret',
        output='screen',
        parameters=[{
            'video_device':       '/dev/kamera_nisan',
            'image_width':        1280,
            'image_height':       720,
            'framerate':          30.0,
            'pixel_format':       'mjpeg2rgb',
            'camera_name':        'taret',
            'camera_info_url':    '',
            'auto_white_balance': True,
            'autoexposure':       True,
        }],
        remappings=[
            ('image_raw',   '/camera/taret/image_raw'),
            ('camera_info', '/camera/taret/camera_info'),
        ]
    )

    # Ön kamera — AYRI BİR NODE YOK. kamera_ana (yukarıda) zaten /dev/kamera_on'u
    # açıp /camera/image_raw'a yayınlıyor — aynı fiziksel kamerayı iki node'un
    # açmaya çalışması cihaz çakışmasına yol açıyordu (sahada doğrulandı,
    # 2026-07-15: ikinci usb_cam_node_exe "terminate called after throwing an
    # instance of 'char*'" ile çöktü). Ön kamera görüntüsüne ihtiyaç duyan
    # tüketiciler (dashboard, veri_paketi) doğrudan CAMERA_IMAGE_TOPIC'e
    # (kamera_ana'nın çıktısı) abone.

    # Arka kamera — geri sürüş görüntüsü (§6.12 zorunlu)
    webcam_geri = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam_geri',
        output='screen',
        parameters=[{
            'video_device':       '/dev/kamera_arka',
            'image_width':        1280,
            'image_height':       720,
            'framerate':          30.0,
            'pixel_format':       'mjpeg2rgb',
            'camera_name':        'geri',
            'camera_info_url':    '',
            'auto_white_balance': True,
            'autoexposure':       True,
        }],
        remappings=[
            ('image_raw',   '/camera/arka/image_raw'),
            ('camera_info', '/camera/arka/camera_info'),
        ]
    )

    return LaunchDescription([
        rsp,
        TimerAction(period=0.5,  actions=[e_stop]),
        TimerAction(period=1.0,  actions=[seri_kopru, lidar, scan_relay, os30a,
                                          imx258,
                                          webcam_taret, webcam_geri]),
        TimerAction(period=2.0,  actions=[preprocessing]),
        TimerAction(period=3.0,  actions=[ekf]),
        TimerAction(period=5.0,  actions=[slam]),
        TimerAction(period=8.0,  actions=[nav2, map_image]),
        TimerAction(period=10.0, actions=[ackermann, imu_guvenlik,
                                          anti_rollback, mod_yoneticisi]),
        TimerAction(period=11.5, actions=[veri_paketi, terrain_adapter,
                                          kayar_kalman, kayar_costmap,
                                          taret_rc_koprusu]),
        TimerAction(period=12.0, actions=[watchdog]),
        TimerAction(period=13.0, actions=[yolo_detection, cone_fusion,
                                          targeting, servo_controller]),
        TimerAction(period=13.5, actions=[yolo_adapter]),
        TimerAction(period=16.0, actions=[misyon_fsm]),
    ])
