import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取当前包的路径
    pkg_share = get_package_share_directory('description')

    # 加载URDF文件内容
    urdf_file = os.path.join(pkg_share, 'urdf', 'sentry.urdf')
    with open(urdf_file, 'r') as infp:
        description = infp.read()

    # 配置robot_state_publisher节点参数
    rsp_params = {'robot_description': description}

    # 定义并返回launch描述
    return LaunchDescription([
        # 启动robot_state_publisher节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params]
        ),
        # 启动rviz2节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot.rviz')], 
            parameters=[{'use_sim_time': False}]  # True使用仿真
        )
    ])