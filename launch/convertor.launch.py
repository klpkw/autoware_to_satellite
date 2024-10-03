import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "autoware_to_satellite"
    share_dir = get_package_share_directory(pkg_name)

    convertor_node = Node(
        package=pkg_name,
        executable="convertor.py",
        name="autoware_to_satellite",
        parameters=[
            LaunchConfiguration("params_path")
            ],
        remappings=[
            ('/vehicle/status/steering_status', '/vehicle/status/steering_status'),
            ('/vehicle/status/gear_status', '/vehicle/status/gear_status'),
            ('/vehicle/status/velocity_status', '/vehicle/status/velocity_status'),
            ('/vehicle/status/turn_indicators_status', '/vehicle/status/turn_indicators_status'),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_path", default_value=os.path.join(share_dir, "params", "convertor_params.yaml")),
            convertor_node
        ]
    )