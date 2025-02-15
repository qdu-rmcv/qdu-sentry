# 青岛大学哨兵项目

青岛大学未来战队25赛季哨兵

本项目高度参照FYT与PB哨兵项目
使用全向移动小车，附加 Livox Mid360 雷达，在 RMUC/RMUL 地图进行导航算法

项目使用FASTLIO2进行建图定位,[TODO：调整成为PointLIO,以期待实现更好的鲁棒性] 
然后借助terrain_analysis实现点云分割，分割后的障碍物输入转换话题转化为2维对象并输入[Nav，amcl]

[TODO]
1. nav包的调用与调试

2. 使用icp与amcl的融合重定位

3. 决策的更改

QA:
注意海康相机MVS，具体见github issues
