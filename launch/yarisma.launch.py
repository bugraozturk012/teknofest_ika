"""
Yarışma Modu Launch Dosyası
SLAM yoktur — kaydedilmiş harita + AMCL lokalizasyon kullanılır.

Başlatma:
  ros2 launch teknofest_ika yarisma.launch.py

Sıralama:
  0s  → Gazebo + robot_state_publisher + env vars
  5s  → araç spawn (gazebo_ros)
  8s  → EKF (odom + IMU füzyon)
  12s → Nav2 bringup  (map_server + AMCL + navigation stack)
  10s → RViz2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')
    nav2_pkg  = get_package_share_directory('nav2_bringup')

    # ── Ortam değişkenleri ──────────────────────────────────────────────────────
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    set_model_path  = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(ws_root, 'models') + ':' + os.path.join(pkg_share, 'models')
    )
    set_plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(ws_root, 'plugins')
    )

    # ── Gazebo ─────────────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_share, 'worlds', 'yarisma.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    # ── Robot State Publisher ───────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_share, 'urdf', 'arac.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace("package://teknofest_ika", "file://" + pkg_share)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # ── Araç spawn ─────────────────────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'teknofest_araci',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # ── EKF — odom + IMU füzyon ─────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ]
    )

    # ── Nav2 bringup: map_server + AMCL + navigation ───────────────────────────
    map_yaml = os.path.join(pkg_share, 'maps', 'teknofest_harita.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time':  'true',
            'map':           map_yaml,
            'params_file':   nav2_params,
            'use_composition': 'False',
        }.items()
    )

    # ── RViz2 ──────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        gazebo,
        robot_state_publisher,
        TimerAction(period=5.0,  actions=[spawn_entity]),
        TimerAction(period=8.0,  actions=[ekf_node]),
        TimerAction(period=10.0, actions=[rviz]),
        TimerAction(period=12.0, actions=[nav2_bringup]),
    ])
