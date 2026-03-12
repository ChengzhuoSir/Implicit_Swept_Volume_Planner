# ESV Upstream Paper Rebuild Design

## Goal
在保留 ROS、地图输入、launch、RViz 和基础测试框架的前提下，删除当前偏离论文的核心实现，改用上游几何/优化算法库重建 `esv_planner` 的论文主链，使系统真正向以下三项论文核心靠拢：
- body-frame SDF / generalized winding 主导的前端几何推离
- 连续 swept-volume 风格的统一碰撞语义
- 以连续变量和时间分配为主变量的 MINCO 风格优化

## Why Rewrite
当前代码虽然已经把 fixed case 跑通，但仍存在结构性偏差：
- 前端仍混有启发式 shortcut 和局部修补逻辑，body-frame 几何没有成为主导。
- continuous collision 仍是 bounded approximation，不是统一的上游可复用内核。
- `TrajectoryOptimizer` 当前的 `CT + LBFGS++` 路线把系数当自由变量，并依赖有限差分和 continuity penalty，既慢又病态，和论文的 MINCO 变量组织不一致。

继续在现有 `trajectory_optimizer.cpp` 上打补丁只会继续积累缝合层。正确路线是保留工程外壳，重建算法内核。

## Architecture

### 1. Geometry Map Layer
输入仍是 `OccupancyGrid`，但运行时主表示改成：
- obstacle boundary contours
- 2D segment / polygon map
- KD-tree / BVH
- auxiliary continuous ESDF for lightweight queries

这层负责为前端 body-frame push、continuous feasibility、优化器安全项提供统一几何基础，禁止主链直接扫栅格。

### 2. Body-Frame Frontend
使用 `libigl` 和 body-frame SDF 查询重写前端 obstruction repair：
- body-frame signed distance / closest point / gradient
- generalized winding number sign
- 几何梯度主导的局部推离
- 失败即剪枝，不再做暴搜式 fallback

`TopologyPlanner` 仍可暂时保留 roadmap 骨架，但前端通过条件和 push-away 必须由几何主导，而非全局 ESDF 梯度主导。

### 3. Unified Continuous Feasibility
建立统一接口，供以下模块共用：
- `TopologyPlanner`
- `SE2SequenceGenerator`
- `TrajectoryOptimizer`
- final acceptance

接口语义必须统一：同一条连续轨迹/状态链在各阶段的“可行 / 不可行”判断不能分裂。

### 4. Continuous Collision Core
不再使用当前试验性的 sampled / bounded evaluator 主实现。新的主实现需要满足：
- 高频状态评估使用轻量连续查询
- 段级/轨迹级碰撞使用局部障碍几何 + 有界连续最小值搜索
- 支持稳定梯度或至少稳定的优化接口

这层先追求几何一致性和上界明确，再逐步逼近论文的 swept-volume 理念。

### 5. Sequence Layer
`SE2SequenceGenerator` 只保留论文语义：
- `SafeYaw`
- recursive `SegAdjust`
- `LOW/HIGH` labeling

当前历史遗留的 `compactSegments / enforceLowSegmentMargin / expandTightHighSegments` 这种后处理链将退场。sequence 的职责是按统一可行性规则划分问题，而不是承担 optimizer 的补锅逻辑。

### 6. Optimizer Layer
新的 `TrajectoryOptimizer` 分两步重建：
- `SE2`: 以 `q, yaw, log(T)` 等低维变量为优化变量，通过 MINCO 映射生成连续多项式系数，再用 `LBFGS++` 优化。
- `R2`: 复用同样的参数化方式，但变量和代价项按 `LOW` 语义简化。

禁止继续使用：
- full coefficient 直接做自由变量
- finite-difference 主梯度路径
- continuity penalty 作为主要连续性保证
- reduced/full 双变量空间切换

### 7. Stitch Layer
`stitch()` 只负责：
- 连接已优化段
- 连续性检查
- 轻量时间协调

它不能再作为第二个优化器或 guard/fallback 聚合器。

## Keep / Rewrite / Delete

### Keep
- `esv_planner_node.cpp`
- `grid_map.cpp`（保留输入层与 ESDF 预处理壳）
- `body_frame_sdf.cpp`（改造成 libigl 主路径）
- `topology_planner.cpp` 的 roadmap 外壳
- fixed case / geometry / topology 稳定测试

### Rewrite
- `se2_sequence_generator.cpp`
- `continuous_svsdf_evaluator.cpp`
- `trajectory_optimizer.cpp`
- 对应头文件和测试入口

### Delete or Retire
- 旧 `CT` solver 实验路径
- 仅用于旧 optimizer 调试的测试：
  - `test_optimizer_se2_ct_variables.cpp`
  - `test_optimizer_se2_maze_t_path1_seg3.cpp`
  - `test_optimizer_se2_maze_t_path1_seg3_context.cpp`
  - 其他只服务于旧 coefficient-space 试验的红灯测试

这些测试不会立刻物理删除，但会在新实现接管后逐步退役。

## Validation Strategy
主验收保持三层：
1. geometry / sequence / optimizer 专项测试
2. fixed case: `(1.06, 7.55, -1.57) -> (8.97, 3.63, 1.52)`
3. secondary case: `(0.72, 5.31, 0.29) -> (9.14, 5.74, 1.64)`
4. maze regression: `maze/T`, `maze/L`

在线 accepted trajectory 的默认门槛维持：
- `min_svsdf >= 0.1`
- 不回退到 `Hybrid A*`
- `used_guard == 0`

## Execution Rules
- 大规模代码重构允许发生在 `src/esv_planner/src` 和 `src/esv_planner/test`。
- 每个阶段必须独立提交到 git。
- 不把用户已有的 `docs/plans/*` 删除状态混入我们新的代码提交。
- 不再继续 patch 当前 coefficient-space `CT` solver。

## Recommended Next Step
第一步不是继续优化器，而是先建立新的重构边界：
1. 清理旧 optimizer 实验测试
2. 为新的 `MincoParameterization` 和 `UnifiedContinuousEvaluator` 建骨架
3. 再开始 `SE2SequenceGenerator` 和 `TrajectoryOptimizer` 的真正替换
