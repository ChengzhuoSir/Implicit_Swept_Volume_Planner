# ESV Planner (Efficient Swept-Volume Planner) 🚗💨

[![ROS](https://img.shields.io/badge/ROS-Noetic-green.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)](https://isocpp.org/)

**ESV Planner** 是一款专为**任意形状地面机器人**（如差速车、阿克曼底盘、长轴距卡车等）设计的高性能全局运动规划器。它巧妙地结合了“粗到细（Coarse-to-Fine）”的搜索策略与**隐式扫掠体积（Swept-Volume SDF）**优化，能在极其狭窄、复杂的非凸障碍物环境中，光速计算出一条丝滑、动力学可行且绝对安全的 SE(2) 轨迹。

> **核心亮点**：不管你的机器人是方是扁，是挂车还是扫地机，只要给它一个多边形轮廓，它就能像泥鳅一样在障碍物堆里穿梭！

---

## 🌟 为什么选择 ESV Planner？

传统的路径规划往往把机器人简化为一个“圆”或者“质点”，这在厂房、仓库或停车场等极限场景下根本行不通！如果你强行用 A* 或 TEB 规划器处理长方形车体，要么在拐角处发生诡异的碰撞，要么为了安全膨胀地图导致根本找不到路。

ESV Planner 的超能力在于：
1. **降维打击与分段治之**：
   - 先在轻量级的 $R^2$ 空间铺开拓扑网络，快速找到逃生方向。
   - 对路径进行“风险评估”：空旷地带随便跑（低风险），极限夹缝精细算（高风险）。
2. **真正的 Any-Shape 防撞**：抛弃了粗糙的圆盘膨胀，直接在底层引入了基于机器人真实多边形的**隐式扫掠体积 SDF (SVSDF)**。
3. **光速求解**：得益于极其克制的 C++ 架构与自动梯度的精妙运用，即便在满是柱子和 U 型弯的极限场景，也能在百毫秒级输出轨迹。
4. **极致鲁棒的兜底机制**：独创了“弹性带张力推离”与“局部瓶颈 Hybrid A* 修复”机制，告别局部极小值死锁。

---

## 📂 领域驱动的代码架构

我们对工程进行了地毯式的清理与重构，彻底剥离了 ROS 依赖，让核心算法变成了一件纯粹的艺术品：

```text
src/esv_planner/
├── include/esv_planner/ & src/
│   ├── core/           # 核心数学、轨迹插值与 ESDF 梯度运算 (The Math)
│   ├── env/            # 物理环境建模，包含地图、网格外形与 SVSDF 计算 (The World)
│   ├── global_search/  # 拓扑搜索与前端降维探路算法 (The Pathfinder)
│   ├── optimization/   # 轨迹序列化、风险切分与高/低风险数值后端 (The Optimizer)
│   ├── framework/      # 运行主轴与异常恢复降级管理器 (The Brain)
│   ├── ros_interface/  # 隔离网关，负责所有 ROS Topic 与 Rviz Marker 发布 (The Bridge)
│   └── third_party/    # 外部高阶轮子 (IGL, nanoflann 等)
```

> **ARM 友好**：全部代码均未绑定 X86 特定指令集，支持在 Jetson Orin / 树莓派 等车载 ARM 计算平台上无缝交叉编译与稳定运行。

---

## 🚀 快速上手

### 1. 依赖安装

环境要求：**Ubuntu 20.04 + ROS Noetic**

```bash
# 安装基础 ROS 依赖
sudo apt-get install ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-tf2-ros ros-noetic-visualization-msgs
# 安装数学与点云库
sudo apt-get install libpcl-dev
```

### 2. 编译工程

```bash
mkdir -p ~/esv_ws/src
cd ~/esv_ws/src
# 将当前代码拉取或复制到此处
git clone <your_repo_url> esv_planner
cd ..
# 释放性能猛兽 (Release 模式极其重要！)
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 3. 在 Rviz 中见证魔法

我们在代码中内置了高保真的 Rviz Visualizer。当你启动节点并发起规划时：
- 🔴 **红色曲线**：优化后的最终连续轨迹
- 🟢 **绿色多边形序列**：机器人沿轨迹运行时的真实轮廓扫掠（Footprint Marker）

*(只需在 Rviz 中订阅 `/esv_planner/markers` 和 `/esv_planner/trajectory` 即可观看)*

---

## 🛠 高阶配置 (Tuning)

您可以在 ROS 参数服务器中定制属于你自己的“老司机”：

- **`footprint`**: 顺时针/逆时针定义的二维多边形顶点坐标 `[x1, y1, x2, y2...]`。
- **`optimizer/max_vel`** & **`max_acc`**: 车辆的动力学极值。
- **`se2/max_push_attempts`**: 遇到死角时，算法尝试把路点“拽”出来的最大次数。
- **`hybrid_astar/wheel_base`**: 你的车有多长（决定了转弯半径的极限）。

---

## 🤝 贡献与致谢

本项目涉及的核心理论来自于先进的连续碰撞避免（Continuous Collision Avoidance）学术论文。我们在工程化落地时，对底层数值优化、异常恢复逻辑（Recovery Manager）以及模块解耦做了大量的工业级加固。

欢迎提交 Issue 或 Pull Request！我们期待让更多奇形怪状的机器人跑得飞快！