import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("teknofest_ika")
    slam_params = os.path.join(pkg_share, "config", "mapper_params_online_sync.yaml")

    sim_arg = DeclareLaunchArgument("sim", default_value="false",
        description="true=Gazebo, false=gercek robot")

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": LaunchConfiguration("sim")}]
    )
    return LaunchDescription([sim_arg, slam_node])
