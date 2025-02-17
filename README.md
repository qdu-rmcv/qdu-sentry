# 青岛大学哨兵项目

青岛大学未来战队 25 赛季哨兵

本项目高度参照 FYT 与 PB 哨兵项目，采用全向移动小车和 Livox Mid360 雷达，在 **RMUC/RMUL** 地图环境下实现导航算法。

## 系统架构

- 使用 **FASTLIO2** 进行建图与定位  
    > [TODO] 调整为 PointLIO 以期待实现更好的鲁棒性
- 利用 **terrain_analysis** 实现点云分割  
    分割后的障碍物会转换为二维对象，并通过相应话题输入给导航（Nav）和 AMCL 模块

## 后续开发计划

- [ ] 调试与调用 nav 包
- [ ] 实现 ICP 与 AMCL 融合的重定位
- [ ] 修改与优化决策模块

## 注意事项

- 海康 MVS 相机使用方法，详见 [GitHub Issues](https://github.com/orgs/qdu-rmcv/discussions/2)
- ICP 模块使用了绝对路径引用，请特别注意调整和维护
