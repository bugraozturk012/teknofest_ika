"""
SLAM Haritalama Launch Dosyası
Mevcut hiçbir dosyayı değiştirmeden çalışır.

Başlatma:
  ros2 launch /home/bugra16/ika_ws/launch/slam_haritalama.launch.py

Sıralama:
  0s  → Gazebo + Robot + SLAM + EKF  (baslat.launch.py)
  15s → Nav2 navigation stack        (sim=true)
  8s  → RViz2 (nav2 varsayılan config)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('teknofest_ika')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # --- Gazebo + Robot + SLAM + EKF (mevcut launch dosyası) ---
    baslat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'baslat.launch.py')
        )
    )

    # --- Nav2 navigation stack (sim=true) ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'sim': 'true'}.items()
    )

    # --- RViz2 (nav2 varsayılan görünüm) ---
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        # Gazebo hemen başlar
        baslat,
        # RViz 8 saniye sonra (Gazebo görünür olunca)
        TimerAction(period=8.0, actions=[rviz]),
        # Nav2 15 saniye sonra (SLAM + EKF hazır olduktan sonra)
        TimerAction(period=15.0, actions=[nav2]),
    ])
