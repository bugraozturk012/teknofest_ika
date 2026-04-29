import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("teknofest_ika")

    with open(os.path.join(pkg_share, "urdf", "arac.urdf"), "r") as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace("package://teknofest_ika", "file://" + pkg_share)

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}], output="screen")

    ekf = Node(package="robot_localization", executable="ekf_node",
        name="ekf_filter_node", output="screen",
        parameters=[os.path.join(pkg_share, "config", "ekf.yaml"), {"use_sim_time": False}])

    seri_kopru = Node(package="teknofest_ika", executable="seri_kopru",
        name="seri_kopru", output="screen", parameters=[{"use_sim_time": False}])

    ackermann = Node(package="teknofest_ika", executable="ackermann_converter",
        name="ackermann_converter", output="screen",
        parameters=[{"use_sim_time": False, "wheelbase": 0.55,
                     "max_steering_angle": 0.5236, "max_speed": 3.0}])

    veri_paketi = Node(package="teknofest_ika", executable="veri_paketi",
        name="veri_paketi", output="screen", parameters=[{"use_sim_time": False}])

    misyon_fsm = Node(package="teknofest_ika", executable="misyon_fsm",
        name="misyon_fsm", output="screen", parameters=[{"use_sim_time": False}])

    terrain_adapter = Node(package="teknofest_ika", executable="terrain_adapter",
        name="terrain_adapter", output="screen", parameters=[{"use_sim_time": False}])

    imu_guvenlik = Node(package="teknofest_ika", executable="imu_guvenlik",
        name="imu_guvenlik", output="screen", parameters=[{"use_sim_time": False}])

    anti_rollback = Node(package="teknofest_ika", executable="anti_rollback",
        name="anti_rollback", output="screen", parameters=[{"use_sim_time": False}])

    kayar_kalman = Node(package="teknofest_ika", executable="kayar_engel_kalman",
        name="kayar_engel_kalman", output="screen", parameters=[{"use_sim_time": False}])

    return LaunchDescription([
        rsp,
        TimerAction(period=1.0, actions=[ekf, seri_kopru]),
        TimerAction(period=3.0, actions=[ackermann, veri_paketi, terrain_adapter,
                                         imu_guvenlik, anti_rollback, kayar_kalman]),
        TimerAction(period=5.0, actions=[misyon_fsm]),
    ])
