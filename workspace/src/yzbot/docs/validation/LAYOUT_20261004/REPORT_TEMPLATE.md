# 当前布局全验证报告模板

日期、执行人/Agent、分支、commit、工作树状态、源码/世界/地图/参数哈希、VM 环境、证据根目录、规则来源及页码均必填。

## 比赛要求对照

| 要求 | 判定 | 证据 | 限制/待办 |
|---|---|---|---|
| 原场景保留；物块间距；三个 zone 由墙隔开 | 通过/失败/未验证 | 世界文件与几何校验 | |
| 五块全部抓取并投放到正确区，零跳过 | | 各实体投放真值与任务日志 | |
| 抓取、放置颜色序列遵循题目；同色实体选择合理 | | 原题、解析计划、逐次实际动作 | |
| 至少两次有效动态避障；原两个障碍分别影响行为 | | 每次事件的时间、模型、位置、距离、前后速度/路径、bag | |
| 新两面移动墙实际巡逻，无穿墙 | | 全扫掠几何及多相位实体真值 | |
| 不碰撞、不侧翻，安全链全程正常 | | 接触/姿态/净空与生命周期记录 | |
| RGB-D 感知与 MoveIt 抓取真实执行 | | 图像、目标、TF、规划/执行状态 | |
| 官方出题 + 真实 API + 严格解析校验 | | 原题、source、模型、延迟与关联信息，去密钥 | |
| CoStudio 显示规定的信息 | | 真实截图/视频、topic 数据、连接信息 | |
| 完整比赛墙钟 ≤300 s | | 明确起止事件、单调时钟和视频 | |

判定只用“通过 / 失败 / 未验证”。运行环境故障另列，不能删掉或自动当通过。

## 每题结果表

CSV 至少包含：case_tag、round_id、attempt_id、commit、dirty_hash、layout_hash、question、parsed_color_order、actual_pick_order、actual_place_order、entity_side_sequence、expected_count、completed、skipped、truth_delivery_pass、mission_terminal、llm_source、model、api_ms、motion_sec、full_wall_sec、timer_start_event、timer_end_event、obstacles_seen、original_obstacle_1_response、original_obstacle_2_response、additional_wall_events、min_clearance_by_model、collision_contacts、max_pitch_roll、nav_aborts、recovery_count、failure_stage、motion_accepted、competition_qualified、evidence_path。

`full_wall_sec` 未记录则写空/未验证，不填运动时间。API 未执行则写 disabled/未验证，不伪填延迟。通过率分母包含全部计划题；记录全部尝试，最新同版本结果和最好历史结果分别统计。

## 动态避障事件明细

| round_id | 障碍模型 | 时刻/时间区间 | 障碍与机器人位置 | 最小表面净空 | 响应前后速度/路径 | yield/detour | bag/视频位置 | 审核结论 |
|---|---|---|---|---|---|---|---|---|

不要只列 `obstacles_seen=4`。有效响应需证明障碍先构成实际冲突/影响，机器人随后安全减速、等待或绕行。新增矩形墙按真实矩形几何算净空，不套用圆柱半径。区分自动监测器判定与人工复核结果。

## 汇总和剩余问题

列出计划/执行/闭环/安全通过/两次避障通过/真实 API 通过/完整300 s通过数量，运动和全墙钟的 min/p50/p95/max，困难题同版本重复结果。逐个失败给最小复现、根因证据与尚未证实的假设。最后列明未完成清单、备份路径、提交与远端 HEAD 状态。

只有完整计时及其他全部要求均通过的题才进入“比赛达标”列。全 48 题或不同障碍相位未完成时，不宣称全分布稳定。
