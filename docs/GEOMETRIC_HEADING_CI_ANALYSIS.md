# PR #6 最新 CI 与 main 的对比

> 本文保留上一轮版本的历史证据。本轮实施与本地测试见
> [GEOMETRIC_HEADING_LOCAL_OPTIMIZATION.md](GEOMETRIC_HEADING_LOCAL_OPTIMIZATION.md)。


最新有效仿真运行：[41034920183](https://github.com/zz990099/swerve_mppi/actions/runs/41034920183)，PR 算法版本 bb29044，记录脚本修正版 63c97b2，CI 实际测试 merge commit bda34e1f。
main 对照运行：[41021506828](https://github.com/zz990099/swerve_mppi/actions/runs/41021506828)，版本 35b082b。
两轮都是 seed 42、原始输入参考点完全相同、平移路径至少 12 m、1 m/s 上限、map 路径、原始轮式里程计、真值 map→odom。

| 指标 | main | PR |
|---|---:|---:|
| 轨迹/到点检查通过 | 32/32 | 27/32 |
| 动作完成 | 32/32 | 29/32 |
| 完整质量门槛通过 | 31/32 | 0/32 |
| 各案例 compute P99 的中位数 | 31.36 ms | 58.95 ms |

PR 的 32 例均超过现有 50 ms P99 门槛，因此完整 CI 为 0/32；这不等于 32 例均无法跟踪。计算时间比较来自两次 CI 运行，未控制 runner 负载，不能仅据此把增幅全归因于代码。

## 五个实际跟踪退化案例

| 场景 | main 最大偏差 cm | PR 最大偏差 cm | PR 动作结果 |
|---|---:|---:|---|
| crab | 1.08 | 14.85 | 完成，但越过位置门槛 |
| crab_right | 0.80 | 13.52 | 未完成 |
| reverse_diagonal | 0.35 | 12.05 | 未完成 |
| lateral_zigzag | 2.09 | 12.38 | 完成，但越过位置门槛 |
| figure_eight | 1.50 | 8.12 | 未完成 |

crab_right、reverse_diagonal、figure_eight 未完成动作。crab、lateral_zigzag 完成，但最大位置偏差超过 10 cm。
smooth_s、combined、reverse_diagonal 还报告发布请求斜率异常；slalom、cloverleaf 等有总耗时失败，均保留原始指标。

## 为什么退化

### 1. 任务改变后，旧的强朝向代价被直接套到新几何朝向上

main 的侧向和反向场景允许车身保持原来的 yaw，依靠横移或倒车完成。PR 将中间 yaw 改为几何走向，例如 crab_right 的途中期望 yaw 从 0 变为 -90°，同时沿用了 heading_scale=0.05 rad、heading_corridor=0.10 rad 及旧权重。
这不是轻微的方向偏好：朝向误差在所有预测步都被较强地惩罚。车身转向与侧向位置保持、舵轮预转向同时竞争。

证据：crab、crab_right、reverse_diagonal、lateral_zigzag 的最大偏差都出现在起步约 3.6～3.9 s，终点接管权重均为 0，距离终点仍约 11.8 m。它们的退化不能由终点阈值解释。

### 2. 采样引导和评分的“平滑接管”力度不一致

feedback_request 用同一个 alpha 对两个 yaw 误差线性加权；Critic 却使用不同的归一化尺度：途中 0.05 rad，终点 0.35 rad。默认 path_heading=0.5、goal_heading=1。
当 alpha=0.5、两个角度误差相同时，途中项系数为 100，终点项约 4.08，相差约 24.5 倍；途中朝向走廊还额外增加惩罚。
因此 alpha=0.5 不代表实际评分中各占一半。引导开始要求转向最终 yaw，而评分仍强烈偏好几何 yaw，可能使继续前进/转向的候选输给停住。
这是本次实现中可以直接从代码确认的不一致。数值反向案例也出现了 alpha 约 0.223 时停住的记录；各因素的贡献尚未做消融隔离。

### 3. 保留了旧恢复代码，但没有使整条参考链适配新策略

ReferenceProfile 的名义速度包络仍依据纯几何 yaw；反馈已混入最终 yaw；分支入口、stable_nominal 和部分预转向判断仍使用几何姿态。
BranchEvent 的创建要求出段 yaw 与当前车身差小于 heading_corridor（0.10 rad），因此它不能直接接管新的 90°/124° 起步大角度调整。
五个物理跟踪失败案例的 branch_phase 全程为 0。crab_right、reverse_diagonal 的执行流各累计约 68 s 预转向，说明恢复机制的代码虽然保留，却没有解决这些新工况。
这些代码条件和执行现象支持参考/恢复不匹配的判断，但本次未继续改代码以隔离唯一根因。

### 4. 终点出现“已经接近，但无法完成”的活性问题

figure_eight 最终位置误差 2.41 cm、yaw 误差约 1.01°，但动作因进度检查失败退出。
末尾记录：remaining=0、goal_heading_weight=1、reorientation_active=1、noise_scale=0，轮速近零，输出请求仍约 (-0.0083,-0.0266,0.0345)。
现有零输出完成门槛不满足；terminal_hold 又要求位置 <1.5 cm、yaw <0.01 rad（0.57°），因此该状态也进不了保持分支。
关闭完成保护并不能解决控制器无法收敛到零请求的问题；应先统一终点纠偏、预转向和完成状态之间的参考。

## 结论与范围

这次回退包含策略要求提高和实现不一致两部分。我之前认为保留旧恢复逻辑即可，判断过于乐观；当前实现没有完整适配新的朝向策略。
不能把所有失败归因于实时性，也不能据此认定“忽略中间 yaw、车头尽量沿路径”本身不可行。
按本轮要求，没有继续深入修复算法或调参。实现保留在草稿 PR #6，未合入 main。

## 可视化和数据

- main_vs_pr_heading_comparison.png：main/PR 最大偏差、起步轨迹与朝向对比。
- heading_gazebo_32_case_overview.png：有效 Gazebo 单轮 32 场景。
- main_vs_pr_metrics.csv：所有 32 例的逐项对照。
- heading_gazebo_valid_seed42.zip：CI 原始轨迹、命令、诊断、日志、总览及分组图。

另外，原始长度数值闭环为 24/32 完成、6 停滞、2 超时；它与至少 12 m 的 Gazebo 验证采用不同路径尺度和被控对象，不能直接比较通过率。
