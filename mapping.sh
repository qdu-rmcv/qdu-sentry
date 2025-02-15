#!/bin/bash

cmds=(
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	"ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml"
	"ros2 launch terrain_analysis terrain_analysis_launch.py"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source ./devel/setup.bash;source install/setup.bash;$cmd;exec bash;"
	sleep 0.7
done
