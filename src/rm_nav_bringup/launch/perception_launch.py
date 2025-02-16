import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    livox_driver = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('livox_ros_driver2'), 'launch'),
         '/msg_MID360_launch.py'])
      )
    
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
             get_package_share_directory('rm_description'), 'launch'),
             '/robot_description.launch.py'])
    )

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
             get_package_share_directory('fast_lio'), 'launch'),
             '/mapping.launch.py'])

    )

    pointcloud_to_laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
             get_package_share_directory('pointcloud_to_laserscan'), 'launch'),
             '/pointcloud_to_laserscan_launch.py'])
    )

    return LaunchDescription([
        livox_driver,
        robot_description,
        fast_lio,
        pointcloud_to_laserscan
    ])