# 2f49f13 功能迁移清单

> 历史基线（只读参考）：`2f49f1385d0ef3e41704a0bc359f600a4b69b805`
> 当前 HEAD：`6c93ba2`
> 只读对照 worktree：`/tmp/project-baseline-2f49`（`git worktree add`，不切换工作区）

## 0. 首要结论：代码结构无退化，退化在**调参**

对 4 个核心文件做函数集合比对（`mission_coordinator.py`、`task_navigator.py`、
`mission_state_machine.py`、`pick_place_moveit.py`）：

```
comm -23 <2f49 函数集> <当前函数集>   =>   空
```

**2f49 里没有任何一个函数在当前版本中丢失**，当前版本是 2f49 的严格超集
（相对 2f49 净增约 1996 行）。因此不存在"旧逻辑更完整"的情况，
迁移工作**不是搬函数**，而是找回被改坏的参数与判据。

## 1. 逐模块对照

| 功能模块 | 2f49 行为 | 当前行为 | 是否退化 | 处理方式 |
| --- | --- | --- | --- | --- |
| 任务解析 | `parse_task_plan_json` + LLM/rules 双路 | 同，另加 schema/counts/order 交叉校验 | 否，当前更强 | 保留当前 |
| 五轮循环 | `MissionStateMachine` 有序五轮 | 同，另加 `skip_after_departure` | 否，当前更强 | 保留当前 |
| 状态机复位 | `start()` 清 attempts/selected_target | 同，另加 `skip_after_departure` 复位 | 否 | 保留当前 |
| 导航 action 管理 | Nav2 action + 超时 + handoff | 同，另加两段式直靠 | 否，当前更强 | 保留当前 |
| **DWB 速度上限** | `max_vel_x: 1.60`、`max_speed_xy: 1.60` | `0.70` / `0.70` | **是** | **迁回并按当前布局取值** |
| **进度检查等待** | `movement_time_allowance: 5.0`，注释明确警告"等 15 s 会把一次本该成功的五件轮推过 300 s" | `30.0`（6 倍） | **是** | **迁回 5.0 起步** |
| **DWB 路径贴合权重** | `PathDist.scale: 24.0`、`PathAlign.scale: 24.0` | `200.0` / `100.0` | **是** | **迁回 24 / 24** |
| 障碍代价权重 | `BaseObstacle.scale: 0.15` | `0.1` | 轻微 | 迁回 0.15 |
| costmap 膨胀 | `inflation_radius: 0.60` | `0.45` | 否 —— 0.60 会封死当前分散布局的托盘通道 | **保留当前 0.45** |
| AMCL 初始位姿 | `(-2.55,-6.80,-1.5708)` 旧地图 | `(-2.0,-2.0,3.14159)` | 否 | 保留当前（旧坐标不可复用）|
| 视觉识别 | `color_target_detector` + HSV + TF | 同，另加期望槽位交叉校验 | 否，当前更强 | 保留当前 |
| 候选物块选择 | `cube_selection_cost`，共享 dock | 同函数，per-cube dock + 距离感知 | 否，当前更强 | 保留当前 |
| 抓取 | `validate_grasp_window` + 限位恢复 | 同，另加 carry 复检与限位守卫 | 否，当前更强 | 保留当前 |
| attach/detach | LinkAttacher + MoveIt attached object | 同，另加 detach 幂等与 PlanningScene 清理 | 否 | 保留当前 |
| 放置 | 真值校验 + `place_maximum_x: 0.360` | `0.350` | 轻微收紧，非退化 | 保留当前 |
| 失败恢复 | 有限重试 + 跳过 | 同，另加非携带跳过先退回 entry | 否，当前更强 | 保留当前 |
| GUI | 面板 + 路径 | 同，另加只显示当前路径、双障碍、布局解析 | 否，当前更强 | 保留当前 |
| 移动障碍 | `moving_obstacle.py` 三角波 | 同（当前为 2 个有效障碍）| 否 | 保留当前 |
| 场景配置读取 | world / mission / navigation 三份坐标 | 同，仍是三份 | 两版同样弱 | 本轮新增统一配置 + 一致性校验 |

## 2. 可以直接复用的逻辑

无需搬迁 —— 全部已在当前版本中，且多数已被增强。

## 3. 需要参数化的逻辑

`config/final_mission_layout.yaml` 作为唯一权威布局源，供 world 生成、任务、导航、
GUI、测试共同校验（见本轮新增的一致性测试）。

## 4. 与旧坐标耦合、**明确不迁移**的内容

| 旧内容 | 旧值（2f49） | 不迁移原因 |
| --- | --- | --- |
| 旧 `room.world` / 旧集中布局 | A/B/C 集中在 x=-1、y=1.40/0.40/-0.60 | 违反规则的分散要求（场景设计 10 分）|
| 旧物块集中布局 | 物块共享一个取货区 | 违反"货物距离不小于 2 m" |
| 旧 pickup dock | `(-3.30,-7.85,-1.5708)` / `(-1.80,-7.85,-1.5708)` | 旧地图坐标 |
| 旧 exit_clear | 红蓝共用 `(-2.55,-6.20,0.0)` | 旧地图坐标，且统一撤离点 |
| 旧 zone nav | `(-1.34, 1.40/0.40/-0.60, 0.0)` | 旧地图坐标，且三区共线 |
| 旧 AMCL 初始位姿 | `(-2.55,-6.80,-1.5708)` | 旧地图坐标 |
| 旧 `inflation_radius: 0.60` | 0.60 | 当前托盘间通道会被封死，实测 entry 净空 0.577~0.955 m |
| 旧地图 `mapn3.pgm` | 旧紧凑地图 | 当前地图已按分散布局重建 |

## 5. 迁移风险

- `max_vel_x` 提高会同时提高动态障碍的相对接近速度。规则要求可审计避障，
  因此必须在 `moving_obstacle:=True` 下实测最小净空，确认仍高于 0.05 m 计分余量。
- `movement_time_allowance` 降低会让 Nav2 更早触发 recovery。移动障碍挡路时
  "等待"本身是合理策略，降太低会导致无谓 recovery。需要取折中值而非直接回到 5.0。
- `PathDist.scale` 降低会让 DWB 允许更大横向偏离，靠近托盘时需确认不擦碰。

## 6. 本轮更正的一处此前记录错误

上一轮交接文档写"`originbot_nav2.yaml` 的 `FollowPath` 块里 `critics` 与 `min_vel_x`
各出现两次（重复键）"。**这条不成立** —— 实际 `grep -n` 只各出现一次
（`min_vel_x` 在 130 行、`max_vel_x` 在 134 行、`critics` 在 157 行）。
当时看到的 `0.55` 来自另一文件的 `dock_max_speed_zone`。已在本轮文档中删除该错误条目。
