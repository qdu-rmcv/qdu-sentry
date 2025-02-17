# Copyright 2025 Lihan Chen
# Copyright 2024 Hongbiao Zhu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Original work based on sensor_scan_generation package by Hongbiao Zhu.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrain_analysis_node",
        name="terrain_analysis",
        output="screen",
        namespace="",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {
                # 传感器坐标系名称
                "sensor_frame": "chasis_odom",
                # 点云降采样的体素大小（米）
                "scan_voxel_size": 0.05,
                # 点云保留时间（秒）
                "decay_time": 0.1,
                # 车辆点云的有效检测距离（米）
                "no_decay_dis": 1.75,
                # 点云清除距离阈值（米）
                "clearing_dis": 8.0,
                # 是否使用排序处理点云
                "use_sorting": True,
                # 地面高度分位数阈值（0-1）
                "quantile_z": 0.5,
                # 是否考虑斜坡作为障碍物
                "consider_drop": False,
                # 是否限制地面抬升
                "limit_ground_lift": False,
                # 最大地面抬升高度（米）
                "max_ground_lift": 0.6,
                # 是否清除动态障碍物
                "clear_dy_obs": True,
                # 动态障碍物最小检测距离（米）
                "min_dy_obs_dis": 0.3,
                # 动态障碍物最小检测角度（度）
                "min_dy_obs_angle": 0.0,
                # 相对于车体的地平面高度（米）
                "min_dy_obs_rel_z": -0.3,
                # 动态障碍物绝对Z轴阈值（米）
                "abs_dy_obs_rel_z_thre": 0.2,
                # 最小垂直视场角（度）
                "min_dy_obs_vfov": -28.0,
                # 最大垂直视场角（度）
                "max_dy_obs_vfov": 33.0,
                # 动态障碍物最小点数
                "min_dy_obs_point_num": 1,
                # 是否将无数据区域视为障碍物
                "no_data_obstacle": False,
                # 无数据区域的评估次数
                "no_data_block_skip_num": 0,
                # 每个区块的最小有效点数
                "min_block_point_num": 10,
                # 车辆高度（米）
                "vehicle_height": 0.4,
                # 体素更新点数阈值
                "voxel_point_update_thre": 100,
                # 体素更新时间阈值（秒）
                "voxel_time_update_thre": 2.0,
                # 点云裁剪的最小相对高度（米）
                "min_rel_z": -1.5,
                # 点云裁剪的最大相对高度（米）
                "max_rel_z": 0.3,
                # 点云的高度与距离比
                "dis_ratio_z": 0.2,
            },
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(start_terrain_analysis_cmd)

    return ld
