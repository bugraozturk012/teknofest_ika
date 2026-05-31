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
        remappings=[('/scan_lidar', '/scan')],
    )

    # ── OS30A Derinlik Kamerası (eYs3D BMVM0S30A) ────────────────────────────
    # Sürücü: eYs3D HD-DM-ROS2-SDK  →  package: dm_preview
    # Yayınlanan topic: /apc/points/data_raw (PointCloud2)
    # Kurulum: terminalden aşağıdaki adımları izle (bir kez yapılır):
    #   git clone https://github.com/eYs3D/HD-DM-ROS2-SDK-Release.git ~/eys3d_ws/src/dm_preview
    #   cd ~/eys3d_ws && rosdep install -i --from-path src -y
    #   colcon build --symlink-install
    #   echo "source ~/eys3d_ws/install/setup.bash" >> ~/.bashrc
    os30a = Node(
        package='dm_preview',
        executable='dm_preview_node',
        name='os30a_node',
        output='screen',
        parameters=[{
            'frame_id':        'os30a_link',
            'color_width':     640,
            'color_height':    360,
            'depth_width':     640,
            'depth_height':    360,
            'fps':             30,
            'enable_pointcloud': True,
        }]
    )

    # ── E-STOP Node ───────────────────────────────────────────────────────────
    # gpio_pin: Jetson BOARD pin numarası (varsayılan 7 → GPIO9)
    # gpio_mod: False yapılırsa GPIO kullanılmaz, sadece /e_stop/force çalışır
    e_stop = Node(
        package='teknofest_ika',
        executable='e_stop_node',
        name='e_stop_node',
        output='screen',
        parameters=[{
            'gpio_pin':   7,
            'gpio_mod':   True,
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
                     'port': '/dev/odom_arduino',
                     'baud': 115200}]
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
    gercek_harita = os.path.join(pkg_share, 'maps', 'gercek_harita.pgm')

    if os.path.exists(gercek_harita):
        # Harita var → localization modu (haritayı yükle, yeni alan haritalama)
        slam_exe = 'localization_slam_toolbox_node'
        slam_extra = {
            'use_sim_time': False,
            'map_file_name': os.path.splitext(gercek_harita)[0],
            'map_start_at_dock': True,
        }
    else:
        # Harita yok → mapping modu (sahayı haritala)
        slam_exe = 'async_slam_toolbox_node'
        slam_extra = {'use_sim_time': False}

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
        parameters=[{'use_sim_time': False, 'wheelbase': 0.55,
                     'max_steering_angle': 0.5236, 'max_speed': 3.0}],
        remappings=[('/cmd_vel', '/mux/cmd_vel')]
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
    # Mod Yöneticisi — MANUAL/SEMI_AUTO/FULL_AUTO geçişleri ve cmd_vel mux
    mod_yoneticisi = Node(
        package='teknofest_ika', executable='mod_yoneticisi',
        name='mod_yoneticisi', output='screen',
        parameters=[{'use_sim_time': False}]
    )
    # Koni Costmap — görüntü ekibinden gelen koni pozisyonlarını Nav2'ye iletir
    koni_costmap = Node(
        package='teknofest_ika', executable='koni_costmap',
        name='koni_costmap', output='screen',
        parameters=[{'use_sim_time': False}]
    )
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


    # ── Görüntü Ön İşleme ─────────────────────────────────────────────────────
    # /ileri_kamera/image_raw → IMX258 ana kameradan (/camera/image_raw)
    # /yardimci_kamera/image_raw → ön webcam'den (/camera/front/image_raw)
    preprocessing = Node(
        package='teknofest_ika', executable='preprocessing_node',
        name='preprocessing_node', output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/ileri_kamera/image_raw',    '/camera/image_raw'),
            ('/yardimci_kamera/image_raw', '/camera/front/image_raw'),
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
            'conf_thres':          0.45,
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
            'cone_safety_radius_m': 0.4,
            'cone_min_confidence':  0.45,
            'target_label':      '13',   # yolo_detection_node str(class_id) yayınlar
        }],
        remappings=[
            ('/ileri_kamera/camera_info', '/camera/front/camera_info'),
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

    # ── Watchdog (Kritik topic sağlık izleme) ─────────────────────────────────
    watchdog = Node(
        package='teknofest_ika', executable='watchdog',
        name='watchdog', output='screen',
        parameters=[{'use_sim_time': False}]
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
            'conf_threshold': 0.45,
        }]
    )

    # ── LR02 433MHz LoRa GCS Köprüsü ─────────────────────────────────────────
    # Udev (bir kez): /dev/lora symlink oluştur
    #   echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="lora"' \
    #       | sudo tee /etc/udev/rules.d/99-lora.rules
    #   sudo udevadm control --reload-rules && sudo udevadm trigger
    lora = Node(
        package='teknofest_ika',
        executable='lora_gcs',
        name='lora_gcs',
        output='screen',
        parameters=[{
            'port':       '/dev/lora',
            'baud':       9600,
            'publish_hz': 1.0,
            'at_init':    True,
        }]
    )

    # ── Microcase 720P Webcam'ler — Şartname §6.12: 3 kamera zorunlu ────────────
    #
    # 3 Microcase 720P aynı USB VID:PID paylaşır → udev'de by-path ile ayırt et.
    # Hangi kamera hangi USB portuna takılı → lsusb -t ile bak, ardından:
    #
    #   Taret (nişan):
    #     echo 'SUBSYSTEM=="video4linux", KERNELS=="<PORT_TARET>", SYMLINK+="webcam_taret"' \
    #         | sudo tee /etc/udev/rules.d/99-webcam-taret.rules
    #
    #   Ön (ileri):
    #     echo 'SUBSYSTEM=="video4linux", KERNELS=="<PORT_ILERI>", SYMLINK+="webcam_ileri"' \
    #         | sudo tee /etc/udev/rules.d/99-webcam-ileri.rules
    #
    #   Arka (geri):
    #     echo 'SUBSYSTEM=="video4linux", KERNELS=="<PORT_GERI>", SYMLINK+="webcam_geri"' \
    #         | sudo tee /etc/udev/rules.d/99-webcam-geri.rules
    #
    #   sudo udevadm control --reload-rules && sudo udevadm trigger
    #
    # Udev yoksa /dev/video0, /dev/video1, /dev/video2 olarak dene.

    # Taret kamerası — nişan alma (taret üzeri)
    webcam_taret = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam_taret',
        output='screen',
        parameters=[{
            'video_device':       '/dev/webcam_taret',
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

    # Ön kamera — ileri sürüş görüntüsü (§6.12 zorunlu)
    webcam_ileri = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam_ileri',
        output='screen',
        parameters=[{
            'video_device':       '/dev/webcam_ileri',
            'image_width':        1280,
            'image_height':       720,
            'framerate':          30.0,
            'pixel_format':       'mjpeg2rgb',
            'camera_name':        'ileri',
            'camera_info_url':    '',
            'auto_white_balance': True,
            'autoexposure':       True,
        }],
        remappings=[
            ('image_raw',   '/camera/front/image_raw'),
            ('camera_info', '/camera/front/camera_info'),
        ]
    )

    # Arka kamera — geri sürüş görüntüsü (§6.12 zorunlu)
    webcam_geri = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam_geri',
        output='screen',
        parameters=[{
            'video_device':       '/dev/webcam_geri',
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
            ('image_raw',   '/camera/rear/image_raw'),
            ('camera_info', '/camera/rear/camera_info'),
        ]
    )

    return LaunchDescription([
        rsp,
        TimerAction(period=0.5,  actions=[e_stop]),
        TimerAction(period=1.0,  actions=[seri_kopru, lidar, scan_relay, os30a,
                                          webcam_taret, webcam_ileri, webcam_geri,
                                          lora]),
        TimerAction(period=2.0,  actions=[preprocessing]),
        TimerAction(period=3.0,  actions=[ekf]),
        TimerAction(period=5.0,  actions=[slam]),
        TimerAction(period=8.0,  actions=[nav2]),
        TimerAction(period=10.0, actions=[ackermann, imu_guvenlik,
                                          anti_rollback, mod_yoneticisi]),
        TimerAction(period=11.5, actions=[veri_paketi, terrain_adapter,
                                          kayar_kalman, koni_costmap,
                                          kayar_costmap]),
        TimerAction(period=12.0, actions=[watchdog]),
        TimerAction(period=13.0, actions=[yolo_detection, cone_fusion,
                                          targeting, servo_controller]),
        TimerAction(period=13.5, actions=[yolo_adapter]),
        TimerAction(period=14.0, actions=[misyon_fsm]),
    ])
