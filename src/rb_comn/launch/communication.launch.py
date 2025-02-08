import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rm_serial_driver_node =Node(
            package='rb_comn',
            executable='communication_node',
            # name='',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud_rate': 115200,
            }]
        )
    return LaunchDescription([rm_serial_driver_node])