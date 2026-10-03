"""Sobe o rosbridge (ponte WebSocket para o celular) e o piloto por teclado e mouse.

Uso:
    source /opt/ros/humble/setup.bash
    source install/setup.bash   # depois de um colcon build
    ros2 launch ponte_teclado_drone sistema.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            get_package_share_directory("rosbridge_server") + "/launch/rosbridge_websocket_launch.xml"
        )
    )
    return LaunchDescription([
        rosbridge_launch,
        Node(
            package="ponte_teclado_drone",
            executable="piloto_teclado_mouse",
            name="piloto_teclado_mouse",
            output="screen",
        ),
    ])
