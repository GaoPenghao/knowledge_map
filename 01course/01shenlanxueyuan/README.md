深蓝学院规划课程学习笔记

# 课程介绍

主讲人：高飞，2015年毕业于浙大，赴港科大读博，师从沈劭劼，2019年博士毕业，并返回浙大控制学院任教，本课程为2019年开课课程

算法以实用为主

相比于无人机在三维环境中的运动规划，无人车的运动规划大多在二维环境中实现，但是需要服从的规则更加复杂，特别是城市道路自动驾驶，要服从交规

## 对于运动规划的基本要求

- 安全性，能够躲避所有的障碍物
- 光滑性，舒适、节省能量
- 动力学可行性，可以被机器人执行

## old-school 流程

1. 前段路径搜索
   - 搜索一条初始的安全的路径
   - 低维的，没有时间维度，没有速度、加速度等机器人状态信息，所以称之为路径
   - 离散的空间，路径是不光滑的
2. 后端轨迹生成
   - 搜索一条可执行的轨迹
   - 高维的，包含有时间信息，具有速度、加速度等机器人状态信息，所以称之为轨迹
   - 连续的空间，是光滑的

## 地图结构

- 表示地图的数据结构
- 使用什么方法做地图中的信息融合

### 占据栅格地图/occupancy grid map

常用的、简单的地图表示方法

- 3D，将三维空间划分为网格
- 2D，将二维平面划分为网格
- 2.5D，在平面上进行网格划分，但是每个网格会有自己的高度信息，也成为海拔地图，https://github.com/ANYbotics/grid_map

特点：

- 网格最稠密，内存消耗大
- 结构化
- 使用网格索引直接查询

### 八叉树地图/octo-map

对于三维空间，每个大的栅格可以沿着三个维度二分，结果将产生8个小的栅格，便形成所谓的八叉树结构

特点：

- 网格稀疏
- 结构化
- 在树状结构中非直接查询

开源项目：https://octomap.github.io/

### voxel hashing

解决视觉建图过程中，显存消耗太多的问题

特点：

- 最稀疏
- 结构化
- 通过哈希表非直接查找

开源项目：https://www.robots.ox.ac.uk/~victor/infinitam/

### 点云地图

直接使用传感器的数据，无序的点堆叠出来的地图

特点：

- 无序的
- 无法使用索引查询

开源项目：http://pointclouds.org/

### TSDF map

truncated signed distance function 截断的有符号的距离函数，即在相机一个帧图片内，到达障碍物曲面之前的点与障碍物之间的距离为正，而曲面之后的点与障碍物之间的距离为负

开源项目：https://github.com/personalrobotics/OpenChisel

### ESDF map

euclidean signed distance function 欧氏有符号的距离函数，即在整个欧氏空间中，记录了每个点到障碍物的距离，相比于TSDF，相当于是全局地图。

开源项目：

- VoxBlox：https://github.com/ethz-asl/voxblox
- FIESTA：https://github.com/HKUST-Aerial-Robotics/FIESTA
- TRR's Local Map：https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan

### 一些工具

- free-space roadmap 生成可行凸空间，https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan
- Voronoi Diagram Map 对于ESDF地图，提取其骨架，形成稀疏的拓扑地图，从而可以在拓扑地图中搜索全局路径，提高效率，https://github.com/ethz-asl/mav_voxblox_planning