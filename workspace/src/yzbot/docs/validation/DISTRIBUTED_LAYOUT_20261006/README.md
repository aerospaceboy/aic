> 历史中间记录，非当前运行指令。最新规则与48题复核见 [UNORDERED_290_20261006](../UNORDERED_290_20261006/README.md)；接手使用该目录的 HANDOFF_PROMPT.md。

# DISTRIBUTED_LAYOUT_20261006 轮次报告（资源重新分散布局）

**结论一句话**：五个资源已按候选方案重新分散部署（world/托盘/complex_layout/mission/zones/代码默认值/地图/风险区全部联动），
并顺手修掉三个已定位缺陷；在**同一最终构建**上，正序 3rA2bB 与反序 2bB3rA 两题都取得
**CASE_ACCEPTED=true + motion_functional_pass=true + motion_under_300=true**（224.5 s / 257.8 s）。
**48 题全矩阵尚未跑完，本报告不代表全部合格。**

## 1. 基线与保护

- 接手 HEAD = `0e4acf8994b1d51c438928c011722076cacc139f` = origin/master；接手时无 gzserver/无他人仿真。
- 用户未提交改动（`competition_navigation.launch.py` 支撑轮 TF、`gazebo_debug_gui_plugin.cpp`）原样保留、未提交；旧 stash 保留；
  未执行 reset --hard。
- 备份：VM `/home/ros/distributed-20261006-backup/`（修改前 13 个关键文件 + 用户补丁 + 哈希清单）、本地分支
  `backup/distributed-20261006`。

## 2. 布局改动（五个资源 + 对应托盘）

| 资源 | 旧中心 | 新中心 |
|---|---|---|
| red_cube_1 | (-3.0, 3.8) | **(-7.5, -2.8)** |
| red_cube_2 | (6.0, 3.8) | **(1.0, -5.4)** |
| red_cube_4 | (10.0, -6.0) | **(11.4, -9.8)** |
| blue_cube_1 | (11.8, -2.5) | **(7.4, 3.6)** |
| blue_cube_4 | (12.0, -6.6) | **(-0.5, -8.8)** |
| red_cube_3 / red_cube_5 / blue_cube_2 / blue_cube_3 / blue_cube_5 | 不变 | 不变 |
| zone A/B/C | (-5.4,3.8) / (11.7,-5.0) / (3.3,-12.1) | 不变 |

- 只改 world 中 10 个模型（5 cube + 5 tray）的模型级 XY；Z、姿态、子 link、anchor、墙、岩石、
  出生点、四个连续移动体、区域尺寸全部未动（脚本断言 + 复核）。
- 分散度（实测坐标计算）：**最小 cube 中心间距 3.059 m**（blue3–red2，≥2 m 规则）；
  **最小 cube→区域中心 4.809 m**（red4→B）。
- 同步链：`config/complex_layout.json` → `activate_complex_layout.py` →
  `competition_zones.yaml` / `tools_demo/config/mission.yaml` / `task_navigator` 三个默认表 / `mission_coordinator.CUBE_WORLD_SLOTS` /
  `config/final_mission_layout.yaml` / `bot_navigation/maps/mapn3.pgm` / `navigation_risk_zones.yaml`；
  另手工更新两处硬编码测试（`test_mission_contract.py`、`test_navigation_contract.py`）。
- 地图重新生成后差异只有 360 个像素，全部落在五个托盘旧/新足迹；占据总数不变（23403），无幽灵足迹，墙/岩石/区域未变。
- 三个开关实际生效值：`use_legacy_layout_routes=false`、`use_legacy_selection_costs=false`、`cube_order_policy_file=""`（运行时校验）。

## 3. 本轮修复的缺陷（都有前后证据）

1. **横向纠偏通道被静音（重要）**：Gazebo 底座是 planarmove（全向），`mission_coordinator` 的
   `_center_target_laterally` / dock realign 用 `linear.y` 闭环纠偏，但 velocity_smoother 的
   `max_velocity: [2.30, 0.0, 3.2]` 把 y 轴清零 → 纠偏一直无效（2026-10-06 实测 moved=9.7e-05 m）。
   改为 `[2.30, 0.22, 3.2]`（accel/decel y 也放开到 ±1.0），前后向与偏航限幅不变。
   修复后实测 `centering_result moved=0.0554 m residual_y=-0.0142`（原先几乎为 0），
   并消除了随后的抓取笛卡尔/IK 失败重试（曾导致重复抓同一块、审计判 order_verified=false）。
2. **放行穿越与安全限速器相位不一致**：导航器用预测窗口放行，安全限速器的东门只在**观测到 mover_1 南向下行**
   时释放；放行后机器人被压在门内，原 6 s 穿越计时照跑 → 腿失败、协调器换侧重试、丢掉第二段展示航点
   （反序题因此只记到 1 个响应）。改法：① 穿越计时只在**真正开始移动**后启动（等待另有 12 s 上界，覆盖一个 8 s 巡逻周期）；
   ② 导航器放行前用与限速器**同一个** `safe_crossing_phase` 谓词（`tools_demo.safety_math`）；
   ③ hold 超时 8→12 s。全程不放宽任何安全阈值、速度门或验收判据。
3. **等待点落在响应半径之外**：hold 触发距离 `MOVER_HOLD_MIN_TRIGGER 1.30` 恰好等于 mover 的
   `response_distance`，机器人停在 1.36–1.47 m 处，mover 超时撤退且不计响应。改为 1.05 + standoff 0.85，
   停止点落到约 1.0 m（组合半径 0.46 m，仍有 0.5 m 余量）；`mover_hold_entry_lateral` 1.20→1.45 覆盖 ±0.45 m 巡逻摆动。
4. **导航交接不减速**：`nav_handoff_distance=0.20` 在 1.4–1.65 m/s 上直接取消 Nav2，G 界面演示中滑行约 0.6 m，
   最终停靠 Y 偏差 0.509 m → `target_unavailable`。新增 `nav_handoff_max_speed=0.45` 与 2 s 宽限：
   先等控制器减速再交接（宽限到期仍交接，避免卡死）。

## 4. 验证（同一最终构建的物理运行）

| 运行 | 题目 | 结果 | 墙钟 | 净空 | 响应 | 备注 |
|---|---|---|---|---|---|---|
| r1 | 3red→A, 2blue→B | **全门通过** | 224.5 s | 0.4066 | 双原始 | 修复前构建，仅作参考 |
| r11 | 2blue→B, 3red→A | **全门通过** | 257.8 s | 0.5287 | 双原始 | 反序；14 次 manip 全成功、无重复抓取 |
| r12 | 3red→A, 2blue→B | 见下节/CSV | — | — | — | 最终构建复跑 |

其它过程运行（保留为失败证据）：r2（重复抓取 → order_verified=false）、r3/r5/r10（穿越等待超时导致腿失败），
r4/r6/r7/r8（5/5 安全完成但只记到 1 个响应），r9（探针诊断运行）。

## 5. 测试与静态门

- `colcon build --symlink-install --packages-select mybot_description mybot tools_demo bot_navigation` 通过。
- 行为回归：**705 passed, 3 deselected**（`-m "not linter"`）。
- 新增/修改代码 flake8 无新增问题；仓库既有 `task_navigator.py` 两处 F841 与若干长行仍存在，单列不称全绿。
- 地图/world/配置/默认值/测试一致性由 `test_layout_config.py` 等合同测试把关。

## 6. 未完成（不得当作已解决）

1. **48 题全矩阵尚未运行**；本报告只证明两题全门通过。
2. 五个搬移对象中的 red4 与 blue4 还没有被物理抓取过一次（需要 4 红 / 4 蓝题覆盖），blue1 的 east 侧也未验证。
3. 反序题的双响应在 r4/r6/r7/r8 里只出现 1 个（r11 才两个）——**相位敏感尚未消除**，需要多轮重复或更确定性的展示路线。
4. GUI 可视演示未复跑（交接减速修复未在 GUI 负载下验证）。
5. 真实 API/CoStudio/完整比赛墙钟未测。
