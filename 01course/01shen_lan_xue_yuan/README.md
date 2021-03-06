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
  >
  >- 定义并初始化所有节点的启发项 h(n)
  >
  >- 将起始节点 Xs 放入优先级队列中
  >
  >- 初始化节点 g 值，令 g(Xs) = 0, 其他的节点 g(n) = inf
  >
  >- 主循环
  >
  >  - 如果队列为空，则返回 FALSE
  >  - 从优先级队列中弹出 ==f 值==最小的节点 n    // 与Dijkstra 不同之处
  >  - 将节点 n 标记为被访问 
  >  - 如果节点 n 是最终目标，则返回 TRUE
  >  - 对于每一个未被访问的、n 的邻居节点 m
  >    - 如果 g(m) == inf    // 说明节点 m 不在优先级队列中
  >      - g(m) = g(n) + Cnm
  >      - 将节点 m 放入优先级队列中
  >    - 如果 g(m) > g(n) + Cnm  // 说明节点 m 已经在优先级队列中，只是未被访问
  >      - g(m) = g(n) + Cnm   // 更新 g(m)
  >
  >  - 本轮循环结束
  >
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

# 基于采样的路径规划

## 相关概念

- Complete Planner: always answers a path planning query correctly in bounded time
- Probabilistic Complete Planner: if a solution exists, planner will eventually find it, using random sampling (e.g. Monte Carlo sampling)
- Resolution Complete Planner: same as above but based on a deterministic sampling (e.g. sampling on a fixed grid)

## 概率路图（PRM, Probabilistic Road Map）

- 本质上是一种图结构，使用随机采样的点抽象表示整个空间，比栅格地图更加稀疏、简化，可以认为是一种简化版的栅格地图
- 将规划分成两个阶段：学习阶段和查询阶段
- 学习阶段
  - 在配置空间中采样N个点
  - 删除落到障碍物中的点
  - 连接采样点，构成边，连线的基本规则
    - 太远的两点之间不连
    - 经过障碍物的不连
- 查询阶段
  - 使用A*或Dijkstra算法找到路径
- 在学习阶段中，collision check 比较费时，然后每个采样点都需要进行 collision check 是比较低效的，所以有研究者提出了 lazy collision-checking，主要思想是在学习阶段不进行 collision check，在查询阶段如果查询到的边或节点是 collision 的，则将其删掉。

## 快速搜索随机树（RRT, Rapidly-exploring Random Tree）

- 以增量式的方法在起点和终点之间构建树结构，区别于PRM，不需要将整个流程分成学习阶段和查询阶段，也不需要A*或者Dijkstra进行路径查找，而是一种新的框架，一旦终点被扩充至树结构中，则表示找到了一条路径，等于说在学习地图的过程中顺便找到了路径

- 伪代码

  > Input: $\mathcal{M}, x_{init}, x_{goal}$
  >
  > Result: A path $\mathcal{T}$ from $x_{init}$ to $x_{goal}$ 
  >
  > $\mathcal{T}.init()$;
  >
  > for $i = 1\ to\ n$ do
  >
  > ​	$x_{rand} \gets Sample(\mathcal{M})$;
  >
  > ​	$x_{near}\gets Near(x_{rand},\mathcal{T})$;
  >
  > ​	$x_{new} \gets Steer(x_{rand},x_{near},StepSize)$;
  >
  > ​	$E_i \gets Edge(x_{new},x_{near})$;
  >
  > ​	if $CollisionFree(\mathcal{M},E_i)$ then
  >
  > ​		$\mathcal{T}.addNode(x_{new})$;
  >
  > ​		$\mathcal{T}.addEdge(E_i)$;
  >
  > ​	if $x_{new}=x_{goal}$ then
  >
  > ​		$Success()$;

- 新增的子节点总是存在1个父节点，所以生成的树结构中每个节点存在以下特点，这两个特点决定了，一旦终点被扩展到树结构，则可以直接回溯得到一条路径：
  - 每个节点可能存在多个子节点
  - 每个节点仅存在一个父节点
- 提高效率的措施
  - 使用 Kd-tree 提高 $Near(x_{rand},\mathcal{T})$ 的效率
  - RRT Connect / Bidirectional RRT
    - 以起点和终点作为根节点分别构建树，每撒一次点，同时对两个树进行一次扩展
    - 当两棵树连接时，则表示找到了路径

## RRT*

- 伪代码

  > Input: $\mathcal{M}, x_{init}, x_{goal}$
  >
  > Result: A path $\mathcal{T}$ from $x_{init}$ to $x_{goal}$ 
  >
  > $\mathcal{T}.init()$;
  >
  > for $i = 1\ to\ n$ do
  >
  > ​	$x_{rand} \gets Sample(\mathcal{M})$;
  >
  > ​	$x_{near}\gets Near(x_{rand},\mathcal{T})$;
  >
  > ​	$x_{new} \gets Steer(x_{rand},x_{near},StepSize)$;
  >
  > ​	if $CollisionFree(x_{new})$ then
  >
  > ​		$X_{near} \gets NearC(\mathcal{T},x_{new})$;
  >
  > ​		$x_{min} \gets ChooseParent(X_{naer},x_{near},x_{new})$;
  >
  > ​		$\mathcal{T}.addNodeEdge(x_{min},x_{new})$;
  >
  > ​		$\mathcal{T}.rewire()$;

## Kinodynamic-RRT*

- 主要修改 $Steer(x_{rand},x_{near},StepSize)$ 函数，已符合机器人运动学约束

## Anytime-RRT*

- 实时以机器人位置为根节点进行树的更新，以适应环境的变化

## Informed RRT*

- 当使用找到一条路径之后，将采样范围限定到椭圆（以起点和终点为焦点，以路径长度作为到焦点的距离之和）中

## Cross-entropy motion planning

- 采样得到一条路径
- 以路径节点为中心，生成多高斯分布
- 采样得到多条路径
- 多条路径的均值为新的高斯分布，进入下一轮采样

## 一些应用

- Navigation stick - ROS，基于 move_base
  - Global planner, A\*, D\*, RRTs, etc
  - Local planner, Dwa, eband, Teb, etc 

# 满足动力学约束的路径规划

## State Lattice Planning

- 基本思想
  - 构建一个 graph，其中节点之间的连线是动力学可行的
  - 创建这样的 graph 有两个基本的思路
    - 正向操作，离散化控制（工作）空间
    - 反向操作，离散化状态空间，求两个状态之间的动力学可行性连接
- 对于一个机器人运动模型：$\dot{s}=f(s,u)$，已知初始状态 $s_0$
  - 在控制空间采样，选择一个 $u$，固定一个时间间隔 $T$，向前积分
    - 没有任务导向
    - 每一步都比较方便实施
    - 规划效率较低
  - 在状态空间采样，选择一个 $s_f$，计算 $s_0$ 和 $s_f$ 之间的一条连接（轨迹）
    - 需要计算 $u,\ T$
    - 有任务导向
    - 每一步比较复杂
    - 规划效率较高

## 边值问题（Boundary  Value Problem）

- 自由边界，则该状态量对应的协变量 $\lambda=0$

## 启发式函数

- 不考虑障碍物
- 不考虑动力学约束

## Hybrid A*



## Kinodynamic RRT*



# 轨迹生成

## 微分平坦（Differential Flatness）

机器人系统的状态总是可以由几个维度本身以及它们的微分的线性组合完全表出，那么称这些维度为机器人状态空间的微分平坦。

比如无人机12维状态 $X=[x,y,z,\phi,\theta,\varphi,\dot{x},\dot{y},\dot{z},\omega_x,\omega_y,\omega_z]^T$ 的微分平坦空间为 $\sigma=[x,y,z,\varphi]^T$ 

## Minimum-snap

- 规划的流程
  - 已知边界条件，开始和终止的状态
  - 获取路标点的状态
    - 可以使用路径搜索的结果作为路标点（A\*，RRT\*，等）
  - 生成轨迹
    - 根据生成的路标点求解分段的边值问题（OBVP）
    - 生成的轨迹要平滑，平滑的主要准则就是最小化输入的变化率
    - 以此求出路标点的最优速度或加速度信息，从而形成轨迹
- ==在设计分段轨迹多项式时，每段轨迹的时间间隔必须已知，这一点与求解 OBVP 问题有所出入。==所以引入了新的问题，如何合理的分配时间间隔
- 
