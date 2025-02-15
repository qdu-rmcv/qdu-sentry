import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取包路径并加载地图配置文件
    map_config = os.path.join(
        get_package_share_directory('nav_package'), 'config', 'map.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_config, 'use_sim_time': False}],
        remappings=[('/map', '/map')]  # 发布地图到 /map 话题
    )

    # nav节点
    navigation_node = Node(
        package='nav2_bringup',
        executable='navigation_launch',
        name='navigation',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[('/scan', '/scan')]  # 订阅 /scan 话题
    )

    return LaunchDescription([map_server_node, navigation_node])
