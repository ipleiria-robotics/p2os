from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = [{"usb_port": "/dev/ttyUSB0"}]
    return LaunchDescription([
        Node(
            package='p2os_driver',
            executable='p2os_driver',
            parameters=parameters,
        ),
    ])
