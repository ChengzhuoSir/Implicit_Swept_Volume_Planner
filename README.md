# iSweep Planner

[![ROS](https://img.shields.io/badge/ROS-Noetic-green.svg)](http://wiki.ros.org/noetic)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

`iSweep Planner` 是一个面向任意形状地面机器人的 ROS 全局规划器。它使用 implicit swept-volume SDF（SVSDF）做连续碰撞约束，在狭窄通道、贴边转角和车体外形不能简单近似为圆的场景中，生成满足速度、加速度和角速度约束的 `SE(2)` 轨迹。

当前仓库提供 ROS Noetic 包、示例地图、RViz 配置，以及一套可直接修改的多边形 footprint 和网格模型配置。

## 适用场景

- 机器人外形明显偏离圆形，膨胀圆近似过于保守。
- 地图中存在窄通道、贴边转角或需要精确姿态调整的区域。
- 需要连续碰撞检查，而不只是离散路径点避障。
- 希望在优化失败时保留搜索式回退方案。

## 方法概览

规划流程采用一个由粗到细的管线，而不是把所有约束一次性丢给单个优化器：

1. 在 `R^2` 平面先做拓扑搜索，生成候选路径。
2. 将候选路径补全为 `SE(2)` 位姿序列，并按风险分段。
3. 在后端同时优化位置、航向和时间分配，并加入连续 SVSDF 碰撞惩罚。
4. 遇到极端狭窄或非光滑接触情况时，使用 LMBM 和回退搜索提高可解性。

如果你更关心实现位置，核心代码主要在这些目录：

- `isweep_planner/src/global_search`：前端拓扑搜索、Hybrid A*、Swept A*
- `isweep_planner/src/optimization`：SE(2) 序列生成、后端优化、连续安全评估
- `isweep_planner/src/env`：地图、footprint、隐式 SDF、扫掠体积相关计算
- `isweep_planner/src/framework`：运行时调度和恢复逻辑
- `isweep_planner/src/ros_interface`：ROS 节点、话题桥接和可视化

## 仓库结构

```text
.
├── README.md
└── isweep_planner/
    ├── config/      # 规划参数和 RViz 配置
    ├── launch/      # demo.launch / planner.launch
    ├── maps/        # 示例栅格地图
    ├── meshes/      # 机器人网格模型
    ├── include/     # 头文件
    ├── src/         # 核心实现
    └── lib/         # 预编译依赖库（如 lmbm.so）
```

## 依赖与构建

推荐环境：`Ubuntu 20.04 + ROS Noetic`

安装常用依赖：

```bash
sudo apt-get update
sudo apt-get install \
  libeigen3-dev \
  libpcl-dev \
  ros-noetic-geometry-msgs \
  ros-noetic-map-server \
  ros-noetic-nav-msgs \
  ros-noetic-pcl-conversions \
  ros-noetic-roscpp \
  ros-noetic-sensor-msgs \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-visualization-msgs
```

在 catkin 工作空间中编译：

```bash
cd <catkin_ws>
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`CMakeLists.txt` 已经为 `x86_64` 和 `ARM64` 增加了只作用于本项目源码的编译优化选项，第三方静态库不会被一并改写。

## 快速开始

如果你想直接跑仓库自带示例，可以启动 demo：

```bash
roslaunch isweep_planner demo.launch use_map_server:=true
```

这会：

- 启动 `isweep_planner`
- 加载仓库自带地图
- 打开 RViz 并载入 `isweep_planner/config/demo.rviz`

启动后可以在 RViz 中使用 `2D Nav Goal` 指定目标点。

默认地图由 `demo.launch` 中的 `map_file` 指定。其他示例地图位于 `isweep_planner/maps/`。

如果不启用 `use_map_server`，节点会等待外部地图输入；这和 `isweep_planner/config/planner.yaml` 中的 `startup_wait_for_map` 配置一致。

## 配置入口

主要参数位于 `isweep_planner/config/planner.yaml`。通常优先关注下面几组：

- `footprint`：二维多边形外形，影响搜索和优化阶段的几何约束。
- `inputdata`：机器人网格模型，用于更高精度的 SVSDF 评估。
- `topology.*`：前端采样和候选路径数量。
- `se2.*`：位姿序列离散步长和障碍推离次数。
- `optimizer.*`：迭代次数、安全距离、运动学约束和代价权重。
- `hybrid_astar.*`：优化失败时的回退搜索参数。

如果你准备替换机器人模型，通常至少需要同时检查 `footprint`、`inputdata`、`optimizer.max_vel`、`optimizer.max_acc`、`optimizer.max_yaw_rate` 和 `hybrid_astar.wheel_base`。

## 说明

这个仓库更像一个可运行、可继续改造的规划器实现，而不是论文配套代码的直接镜像。README 只保留使用和结构信息；更细的算法推导、梯度细节和优化器说明，建议单独维护到技术文档中。

## License

项目在 `package.xml` 中声明为 `MIT`。当前仓库没有单独的 `LICENSE` 文件，如需对外发布，建议补上正式文本。

## Acknowledgements

项目中的 implicit swept-volume 思路和部分总体方向参考了 [HKUST-Aerial-Robotics/esv_planner](https://github.com/HKUST-Aerial-Robotics/esv_planner)。当前实现针对地面机器人场景做了重新组织和适配。
