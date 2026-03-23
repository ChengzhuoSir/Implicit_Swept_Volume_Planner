# iSweep Planner (Implicit Swept-Volume Planner)

<div align="center">

[![ROS](https://img.shields.io/badge/ROS-Noetic-green.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C++-14-blue.svg)](https://isocpp.org/)

**A High-Performance, Morphology-Aware Global Motion Planner for Arbitrarily-Shaped Robots**

</div>

<br>

**iSweep Planner** 是一款针对**任意多边形（Any-shape）地面机器人**设计的高效、高精度连续避障全局运动规划器。针对传统膨胀圆方法（如 A*, TEB）在复杂厂房、仓储和狭窄通道中易发生死锁或碰撞的问题，iSweep Planner 创造性地结合了 **粗到细的分层架构（Coarse-to-Fine Architecture）** 与 **隐式扫掠体积符号距离场（Implicit Swept-Volume SDF）**，能够在毫秒级内求解出严格满足动力学约束与精确几何边界的安全轨迹。

本项目在底层思想与连续避障理论上参考并致敬了前沿的 [ESV (Efficient Swept-Volume) Planner](https://github.com/HKUST-Aerial-Robotics/esv_planner) 的学术工作，并在此基础上进行了深度的**工业级重构与工程增强**。

## 🌟 核心特性 (Key Features)

* **精确的几何感知 (Morphology-Aware)**
  告别保守的“外接圆膨胀”。算法直接读取机器人的真实多边形 Footprint，使用多边形隐式距离场进行碰撞惩罚，完美适应矩形车体、差速车、长轴距卡车等任意非凸形状。
* **分段治之的 C2F 架构 (Coarse-to-Fine Pipeline)**
  * **前端**：在 $R^2$ 空间构建降维拓扑图网络，快速跨越死胡同。
  * **中端**：根据环境 ESDF 安全裕度进行轨迹风险分段（Risk-Level Segmentation）。
  * **后端**：在开阔区域使用超轻量的轨迹优化；在极端狭窄的“高风险”地段，触发带有边界形变约束的 Strict SE(2) SVSDF 严格求解器。
* **弹性张力防震荡机制 (Elastic Tension Recovery)**
  针对传统梯度下降法在 U 型死角容易陷入局部极小值的问题，创新性地引入了基于前后节点“橡皮筋张力”与“动量”的逃逸策略，极大提升了极限通道的贯穿率。
* **极佳的硬件部署亲和性 (Cross-Platform & Hardware Acceleration)**
  工程经过纯净重构，高度解耦。内置智能 CMake 探针，**支持在 x86 平台启用 AVX/SSE 加速，在 ARM 架构（如 NVIDIA Jetson, 树莓派）上自动启用 NEON 硬件加速**，同时规避了棘手的 Eigen 内存对齐崩溃问题。

---

## 📂 领域驱动架构 (Architecture)

通过对底层算法解耦，我们构建了高内聚的工业级 C++ 库：

```text
src/isweep_planner/
├── include/isweep_planner/ & src/
│   ├── core/           # 核心数学、轨迹插值与 ESDF 梯度分析工具
│   ├── env/            # 物理环境建模，包含地图、网格外形与 SVSDF 计算
│   ├── global_search/  # 拓扑搜索与前端降维探路算法 (Hybrid A*, Swept A*)
│   ├── optimization/   # 轨迹序列化、风险切分与高/低风险数值后端
│   ├── framework/      # 运行主轴与多层级异常恢复降级管理器 (Recovery Manager)
│   ├── ros_interface/  # 严格隔离的 ROS 通信网关与 Rviz Marker 发布层
│   └── third_party/    # 外部高阶依赖 (libigl, nanoflann, lmbm)
```

---

## 🚀 快速上手 (Quick Start)

### 1. 环境依赖
* **OS:** Ubuntu 20.04
* **ROS:** Noetic
* **库:** Eigen3, PCL, OpenMP

```bash
# 安装基础 ROS 与数学依赖
sudo apt-get update
sudo apt-get install ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-tf2-ros ros-noetic-visualization-msgs libpcl-dev
```

### 2. 编译工程

请务必使用 `Release` 模式编译，以释放 C++ 底层的矩阵加速性能：

```bash
mkdir -p ~/isweep_ws/src
cd ~/isweep_ws/src
git clone <your_repo_url> isweep_planner
cd ..

# 开启极速编译
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 3. 运行与可视化

启动内置的 Demo 场景与 RViz 监控节点：

```bash
roslaunch isweep_planner demo.launch
```

在 RViz 中，通过 `2D Nav Goal` 指定终点，您将看到：
* 🔴 **红色曲线**：后端生成的平滑且动力学可行的行驶轨迹。
* 🟢 **绿色多边形序列**：基于机器人真实外廓在 SE(2) 状态下生成的扫掠体积（Swept-Volume）动态展示。

---

## 🛠 配置指南 (Configuration)

您可以通过修改 `config/planner.yaml` 灵活定制您的底盘约束：

```yaml
# 机器人的几何多边形外廓 (基于后轴或几何中心的相对坐标)
footprint: [0.20, 0.25, 0.20, -0.25, -0.10, -0.25, -0.10, 0.25]

# 后端优化的运动学极限
optimizer:
  max_vel: 1.2      # 最大线速度 (m/s)
  max_acc: 2.0      # 最大加速度 (m/s^2)
  max_yaw_rate: 2.5 # 最大角速度 (rad/s)
  
# 兜底前端 (Hybrid A*) 的阿克曼运动学约束
hybrid_astar:
  wheel_base: 0.5   # 轴距，决定了车辆的最小转弯半径
  max_steer: 0.6    # 最大前轮转向角 (rad)
```

## 📜 许可证 (License)

本项目采用 [MIT License](LICENSE)。

## 🙏 致谢 (Acknowledgements)
The core implicit swept-volume formulation and initial pipeline logic are inspired by the outstanding work of [HKUST-Aerial-Robotics/esv_planner](https://github.com/HKUST-Aerial-Robotics/esv_planner). We sincerely thank the authors for their contributions to the community.