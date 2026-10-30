# 几何朝向策略：实施与验证结果

实施分支：`feat/geometric-travel-heading`；[草稿 PR #6](https://github.com/zz990099/swerve_mppi/pull/6)。
基线 `35b082b`；实现 `bb2904453cf7a9d268640d3a4d70d85dcefc60d5`。

外部路径中间 yaw 已忽略；途中按 XY 几何生成朝向，末点 yaw 独立保留。
最终朝向使用实测有序剩余距离和实际终点距离，在 0.50～0.10 m 之间平滑接管；同批候选共用权重。
删除了普通终点预测剩余弧长 0.02 m 的评分开关。内部精确对齐、舵轮对齐和驶出动作保留自己的参考，发布重定时保存有效参考快照。
停止/稳定时间、零输出完成门槛、实时 TF 终点及已有物理检查继续保留。

## 数值闭环：24/32 完成，6 停滞，2 超时

**这不是 Gazebo/Nav2 实测。** 单轮 seed 42，采用既有 32 场景的原始长度（未拉长至 12 m）、20 Hz 请求、0.8 m/s 规划上限。
共享底盘执行器与独立 0.12 s 一阶舵机模型组成被控对象；控制器名义模型为 0.10 s。

passed 表示到点位置/yaw/停止/稳定时间/零请求满足要求，且全过程位置偏差不超过 0.10 m；不表示完整物理 CI、实时性或所有质量门槛通过。
每例最多 90 s 仿真时间；连续 15 s 位移不足 2 cm 且朝向变化不足 0.05 rad 记为 stalled。
所有部分轨迹均保留。

| 场景 | 结果 | 仿真时间 s | 最大偏差 cm | 最终位置误差 m | 最终 yaw 误差 ° |
|---|---|---:|---:|---:|---:|
| reverse | stalled | 20.60 | 5.61 | 0.378 | 179.0 |
| reverse_curve | stalled | 35.30 | 3.87 | 0.177 | 163.1 |
| lateral_zigzag | stalled | 58.25 | 4.72 | 1.055 | 44.8 |
| hairpin_fixed_yaw | stalled | 33.45 | 0.81 | 0.168 | 161.1 |
| figure_eight | stalled | 64.35 | 1.33 | 0.991 | 146.7 |
| slalom | stalled | 22.65 | 0.53 | 1.837 | 49.8 |
| rounded_rectangle | timeout | 89.95 | 0.65 | 1.001 | 147.0 |
| cloverleaf | timeout | 89.95 | 0.76 | 0.455 | 143.6 |

失败案例均未报告 NoSolution，多数仍接近参考 XY，但最终输出趋近零，任务没有完成。
reverse 和 hairpin 在最终朝向接管阶段停滞；figure_eight、slalom 在终点接管权重为零时也出现停滞，不能将所有失败归因于终点阈值。
这里只记录现象，没有进行完整根因判定或深入修复。

## 回归检查

- 核心、几何朝向契约、安装后消费者：3/3 通过。
- 路径/碰撞支持检查：通过。
- Python 记录与审计测试：18/18 通过。
- ROS 官方格式与 git diff --check：通过。
- GitHub CI：Debug 核心检查、Jazzy ROS 集成测试通过；Release 的 closed_loop/stagnation 回归失败，与本地一致。
- closed_loop：失败。首个失败为 plant=0、scenario=1（侧向起步），位置 peak=2.98 cm；没有放宽既有质量门槛。
- stagnation：前 3 例完成，cloverleaf 在规定预算内未完成，回归失败。

## Gazebo / Nav2 CI

初次尝试因新增记录脚本把 `branch_phase="0.000000"` 直接按 int 解析而中断。该记录缺陷已在 `63c97b2` 修正，新增小数形式诊断字段回归检查通过。初次尝试不作为控制效果验证结论。

[物理工作流](https://github.com/zz990099/swerve_mppi/actions/runs/41034920183)已启动：一轮 32 场景、seed 42、平移路径至少 12 m，map 路径 + 原始轮式里程计 + 真值 map→odom。
有效仿真已完成：27/32 轨迹检查通过、29/32 动作完成、完整质量检查 0/32（全部超过 50 ms P99）。main 同口径分别为 32/32、32/32、31/32。详细退化分析见 main_vs_pr_analysis.md / GEOMETRIC_HEADING_CI_ANALYSIS.md。

## 文件

- heading_32_case_overview.png：32 场景数值总览，包含失败。
- heading_handoff_and_failures.png：终点接管与保留故障曲线。
- metrics.csv、每例 CSV、references/：完整数值记录。
- core-tests.log、support-tests.log、python-tests.log：本地验证证据。

实现已保存在草稿 PR，回归仍有较大问题，尚不适合合入主分支。按本轮范围停止深入修复。
