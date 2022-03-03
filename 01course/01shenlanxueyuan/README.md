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

# 基于搜索的路径规划

## 相关概念

- 配置空间（configuration space）

  - 机器人配置：对机器人上面所有点的位置描述
  - 机器人自由度：能够完整表达机器人一个配置的最少的坐标数量，记为n维
  - 机器人配置空间：在自由度维数空间中，记录机器人所有可能配置的空间，记为 C-space
  - 所以机器人任何一个位姿状态都是 C-space 中的一个点

- 图（graph）

  图由节点和边构成，边可以是有向的或无向的，也可以是有权重的或无权重的

## 图搜索的基本逻辑

- 声明一个容器，用于存储所有将被访问的节点
- 将已知的起始节点放入容器
- 循环
  - 弹出，根据预定义的规则，从容器中取出一个节点并将其从容器中移除
  - 扩展，找到该节点的所有相邻节点
  - 添加，将找到的相邻节点添加到容器中
- 结束循环

搜索方法分为广度优先搜索（breadth first search）和深度优先搜索（depth first search），已经证明 BFS 更加适用于路径搜索，所以路径搜索算法主要是在 BFS 框架的基础之上发展的

启发式搜索，使用启发函数决定下一个循环需要访问的节点，所以启发函数需要具备一下特点：

- 能够指引正确的方向
- 容易被计算

二维空间常用的启发函数有：

- 欧氏距离，即 $\sqrt{(\Delta x)^2 + (\Delta y)^2}$
- 曼哈顿距离，即 $\ |\Delta x| + |\Delta y|$ 
- 对角线 (Diagonal Heuristic)，$(\Delta x + \Delta y) + (\sqrt2 - 2)*min(\Delta x, \Delta y)$

## Dijkstra

- 策略：每次弹出的节点总是具有最小的累积代价 g(n)

- 伪代码

  > - 声明“优先级队列”，用于存储所有待访问的节点    // ==根据节点的 g 值排序==
  > - 将起始节点 Xs 放入优先级队列中
  > - 初始化节点 g 值，令 g(Xs) = 0, 其他的节点 g(n) = inf
  > - 主循环
  >   - 如果队列为空，则返回 FALSE
  >   - 从优先级队列中弹出 ==g 值==最小的节点 n
  >   - 将节点 n 标记为被访问 
  >   - 如果节点 n 是最终目标，则返回 TRUE
  >   - 对于每一个未被访问的、n 的邻居节点 m
  >     * 如果 g(m) == inf    // 说明节点 m 不在优先级队列中
  >       - g(m) = g(n) + Cnm
  >       - 将节点 m 放入优先级队列中
  >     * 如果 g(m) > g(n) + Cnm  // 说明节点 m 已经在优先级队列中，只是未被访问
  >       - g(m) = g(n) + Cnm   // 更新 g(m)
  >   - 本轮循环结束
  > - 结束循环

- 特点

  - 优点：完备的、最优的
  - 缺点：只能看到当前的累积代价，下次扩展向所有可能的方向进行；没有关于目标的信息

## A*

- 策略

  - 维护当前累积代价 g(n)
  - 维护启发项 h(n)，即估计从节点 n 到目标节点的最小的代价
  - 维护预估总代价 f(n) = g(n) + h(n)，即估计从起点开始，经过节点 n，最终到达目标点的最小代价
  - 每次弹出拥有最小 f 值的节点

- 伪代码

  >- 声明“优先级队列”，用于存储所有待访问的节点    // ==根据节点的 f 值排序==
  >- 定义并初始化所有节点的启发项 h(n)
  >- 将起始节点 Xs 放入优先级队列中
  >- 初始化节点 g 值，令 g(Xs) = 0, 其他的节点 g(n) = inf
  >- 主循环
  >  - 如果队列为空，则返回 FALSE
  >  - 从优先级队列中弹出 ==f 值==最小的节点 n    // 与Dijkstra 不同之处
  >  - 将节点 n 标记为被访问 
  >  - 如果节点 n 是最终目标，则返回 TRUE
  >  - 对于每一个未被访问的、n 的邻居节点 m
  >    * 如果 g(m) == inf    // 说明节点 m 不在优先级队列中
  >      - g(m) = g(n) + Cnm
  >      - 将节点 m 放入优先级队列中
  >    * 如果 g(m) > g(n) + Cnm  // 说明节点 m 已经在优先级队列中，只是未被访问
  >      - g(m) = g(n) + Cnm   // 更新 g(m)
  >  - 本轮循环结束
  >- 结束循环

- 保证最优性的条件
  - Admissible Heuristics，对于所有节点的启发项 h 的值需要小于该节点到达目标点的实际最小代价 h\*，即 h(n) <= h\*(n)

## Weighted A*

- 策略
  - f = g + $ \varepsilon$ h，$\varepsilon$ > 1
- 特点：
  - 结果是次优的，$\varepsilon$-suboptimal：cost(solution) <= $\varepsilon$ cost(optimal solution)
  - 搜索速度很快

## 一些问题的应对策略

- Tie Breaker
  - 在从优先级队列中弹出节点时，如果存在多个节点的 f 值相同，那么该怎么办？
  - 核心思想，打破路径中的对称性
  - 方法一，当 f 值相同时，比较 h 值
  - 方法二，给 h 值加上一项较小的引导项，如
    - h = h $\times$ (1.0 + p)，p < $\frac{minimum\ cost\ of\ one\ step}{expected\ maximum\ path\ cost}$ 
    - h = h + cross $\times$ 0.001，更倾向于沿起点到终点直线方向搜索
      - $cross = abs(dx1\ \times \ dy2\ -\ dx2 \times \ dy1)$
      - $dx1 = abs(node.x\ - \ goal.x),\ dy1 = abs(node.y\ - \ goal.y)$
      - $dx2 = abs(start.x\ -\ goal.x),\ dy2 = abs(start.y\ - \ goal.y)$

## Jump Point Search (JPS)

- 核心思想，打破路径搜索中的对称性

- 伪代码

  > - 声明“优先级队列”，用于存储所有待访问的节点    // ==根据节点的 f 值排序==
  > - 定义并初始化所有节点的启发项 h(n)
  > - 将起始节点 Xs 放入优先级队列中
  > - 初始化节点 g 值，令 g(Xs) = 0, 其他的节点 g(n) = inf
  > - 主循环
  >   - 如果队列为空，则返回 FALSE
  >   - 从优先级队列中弹出 ==f 值==最小的节点 n 
  >   - 将节点 n 标记为被访问 
  >   - 如果节点 n 是最终目标，则返回 TRUE
  >   - 对于每一个未被访问的、n 的邻居节点 m    // ==如何寻找相邻节点与A*大相径庭==
  >     * 如果 g(m) == inf    // 说明节点 m 不在优先级队列中
  >       - g(m) = g(n) + Cnm
  >       - 将节点 m 放入优先级队列中
  >     * 如果 g(m) > g(n) + Cnm  // 说明节点 m 已经在优先级队列中，只是未被访问
  >       - g(m) = g(n) + Cnm   // 更新 g(m)
  >   - 本轮循环结束
  > - 结束循环



## 开源工具

- 各种图搜索算法的仿真演示：http://qiao.github.io/PathFinding.js/visual/
- JPS搜索可视化演示：https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html
- JPS开源库：https://github.com/KumarRobotics/jps3d