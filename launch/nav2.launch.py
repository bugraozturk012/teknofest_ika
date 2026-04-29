import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("teknofest_ika")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    sim_arg = DeclareLaunchArgument(
        "sim", default_value="false",
        description="true=Gazebo, false=gercek robot"
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": nav2_params,
            "use_composition": "False",
        }.items()
    )

    return LaunchDescription([sim_arg, nav2])
