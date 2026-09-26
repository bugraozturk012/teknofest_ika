"""
Gerçek Araç — Harita Alma Launch Dosyası
=========================================
Yarışma öncesi sahada harita almak için kullanılır.
Nav2 YOKTUR — sadece SLAM çalışır.

Başlatma:
  ros2 launch teknofest_ika gercek_harita.launch.py

Harita Kaydetme (ayrı terminalde, tur bitince):
  ros2 run nav2_map_server map_saver_cli -f ~/lydia_ws/maps/teknofest_harita
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/lydia/lydia_ws/maps/teknofest_harita'}"
  # ÖNEMLİ: gercek_arac.launch.py localization_slam_toolbox_node için .pgm DEĞİL,
  # yukarıdaki serialize_map servisinin ürettiği .posegraph + .data dosyalarını
  # okur — sadece map_saver_cli çalıştırılırsa (.pgm/.yaml) bu dosyalar hiç
  # oluşmaz ve localization modu tetiklenmeden araç sessizce mapping modunda kalır.
  cd ~/lydia_ws && colcon build --packages-select teknofest_ika --symlink-install

Sıralama:
  0s  → robot_state_publisher
  1s  → seri_kopru (odom + imu) + YDLidar
  3s  → EKF
  5s  → SLAM Toolbox (mapping modu)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')

    with open(os.path.join(pkg_share, 'urdf', 'arac.urdf'), 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('package://teknofest_ika', 'file://' + pkg_share)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen'
    )

    # YDLidar Tmini Pro — /scan topic'i yayınlar
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
            'lidar_type':     1,       # TYPE_TRIANGLE
            'device_type':    6,       # YDLIDAR_TYPE_SERIAL
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
        }]
    )

    seri_kopru = Node(
        package='teknofest_ika',
        executable='seri_kopru',
        name='seri_kopru',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'port': '/dev/f767',
                     'baud': 921600}]
    )

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

    slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}]
    )

    # /map (OccupancyGrid) → /map/image (Image) — GCS dashboard'un SLAM Haritası
    # panelinde canlı önizleme için (slam_toolbox'tan sonra başlamalı)
    map_image = Node(
        package='teknofest_ika', executable='map_image_node',
        name='map_image_node', output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rsp,
        TimerAction(period=1.0, actions=[seri_kopru, lidar]),
        TimerAction(period=3.0, actions=[ekf]),
        TimerAction(period=5.0, actions=[slam]),
        TimerAction(period=7.0, actions=[map_image]),
    ])
