"""
Test Aracı — Otonom Sürüş Test Launch Dosyası
===============================================
Kürşat'ın DC motorlu diferansiyel test aracı için minimal stack.

Gerçek araçtan FARKLAR:
  - seri_kopru         → test_arac_koprusu  (500k baud, 29B telemetri)
  - ackermann_converter → YOK  (test araç diferansiyel, Ackermann değil)
  - misyon_fsm          → YOK  (Nav2 hedef testi yapılıyor, görev yok)
  - OS30A / webcam'ler  → YOK  (test araçta bu donanım yok)
  - taret / servo       → YOK
  - lora                → YOK
  - yolo_detection      → YOK
  - e_stop gpio_mod     → False (test bilgisayarında Jetson GPIO yok)

Başlatma:
  ros2 launch teknofest_ika test_arac.launch.py

Test sırası:
  1. Seri port bağlantısını doğrula  →  ros2 topic echo /imu/data
  2. Elle sür                         →  ros2 run teleop_twist_keyboard teleop_twist_keyboard
                                          (topic: /cmd_vel → mod_yoneticisi → /mux/cmd_vel)
  3. RViz'den hedef ver               →  Nav2 Goal → otonom gidiş
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
            'angle_max':       180.0,
            'angle_min':      -180.0,
            'range_max':       8.0,
            'range_min':       0.1,
            'frequency':       10.0,
            'invalid_range_is_inf': False,
        }]
    )

    # ── E-STOP Node — GPIO'SUZ (test bilgisayarı) ────────────────────────────
    # gpio_mod=False: donanım butonu yok, sadece /e_stop/force yazılımsal kanal
    e_stop = Node(
        package='teknofest_ika',
        executable='e_stop_node',
        name='e_stop_node',
        output='screen',
        parameters=[{
            'gpio_pin':   7,
            'gpio_mod':   False,   # Test bilgisayarında Jetson GPIO yok
            'publish_hz': 20.0,
        }]
    )

    # ── Test Araç Köprüsü (seri_kopru yerine) ────────────────────────────────
    # Kürşat'ın UNO firmware'i: 500k baud, 29B telemetri (MPU9250+BMI160)
    # Yayınlar: /odom + /imu/data + /rc_input (dummy FULL_AUTO)
    # Dinler  : /mux/cmd_vel → PKT_SURUCU (ackermann_converter ATLANIR)
    test_kopru = Node(
        package='teknofest_ika',
        executable='test_arac_koprusu',
        name='test_arac_koprusu',
        output='screen',
        parameters=[{
            'port':        '/dev/ttyACM0',   # UNO USB portu (dağışırsa ttyACM1 dene)
            'baud':        500000,
            'cmd_timeout': 0.5,
            'sim_mode':    False,
        }]
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

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam_params   = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')
    test_harita   = os.path.join(pkg_share, 'maps', 'test_harita.pgm')

    if os.path.exists(test_harita):
        slam_exe   = 'localization_slam_toolbox_node'
        slam_extra = {
            'use_sim_time':     False,
            'map_file_name':    os.path.splitext(test_harita)[0],
            'map_start_at_dock': True,
        }
    else:
        # Harita yoksa önce harita al:
        #   ros2 launch teknofest_ika slam_haritalama.launch.py
        #   ros2 run nav2_map_server map_saver_cli -f ~/ika_ws/maps/test_harita
        slam_exe   = 'async_slam_toolbox_node'
        slam_extra = {'use_sim_time': False}

    slam = Node(
        package='slam_toolbox',
        executable=slam_exe,
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, slam_extra]
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────────
    # NOT: Test araç için robot_radius ve max hız düşürülmeli.
    # nav2_params.yaml'da veya aşağıdaki launch arg'larla override et:
    #   robot_radius: 0.10   (test araç ~10cm)
    #   max_vel_x:    0.30   (iç mekan güvenli hız)
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

    # ── Mod Yöneticisi ────────────────────────────────────────────────────────
    # /rc_input [ch5=1800µs] → FULL_AUTO modu okur → /mux/cmd_vel yayınlar
    # test_arac_koprusu zaten ch5=1800 dummy yayınlıyor → FULL_AUTO otomatik
    mod_yoneticisi = Node(
        package='teknofest_ika',
        executable='mod_yoneticisi',
        name='mod_yoneticisi',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rsp,
        TimerAction(period=0.5,  actions=[e_stop]),
        TimerAction(period=1.0,  actions=[lidar, test_kopru]),
        TimerAction(period=3.0,  actions=[ekf]),
        TimerAction(period=5.0,  actions=[slam]),
        TimerAction(period=8.0,  actions=[nav2]),
    ])
