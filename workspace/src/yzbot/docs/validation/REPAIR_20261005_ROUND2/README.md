# 二次审查与修复报告（2026-10-05）

## 结论

本轮不执行全48题，只检查代码、修复确定缺陷并做定向验证。候选修复仍须完整验证，**不能宣称稳定300秒或问题全部解决**。

已完成的确定修复：补齐统一速度选择与安全链入口；修正缺失方法导致的导航器崩溃；红块默认仅east接近；避免无净空的超时放行；按正式规则计两个不同有效动态障碍；补强批次检查点和嵌套目录汇总。行为测试最终记录为 **667 passed**，两包构建通过。

定向3rA_2bB仍失败：两条原始mover车道均完成等待、对正、穿越，red_cube_1从east抓取成功；随后持块去A时，在约(-7.0,2.4)一带反复慢进度恢复、recent_trail恢复与对位超时，runner在420秒退出，完成0/5。这不是300秒通过，也不是原窗口问题已代表全题修好。

单红→B的最后定向冒烟结果与最终提交号见本文末尾“最终收尾记录”。

## 审查发现与修改依据

### P1：红色west进坞

历史证据为红west 0/14失败、east 77/77成功、蓝west 15/15成功；当前世界没动。保留上一位agent的候选：两个允许表均删除红west，蓝不变。它是合理规避，不是对物理卡死根因的证明。需要下一位检查所有红块east和跨区运输；不能把以往east成功率当本版本矩阵成功率。

历史cube_order_policy.json仍含57个red-west步骤。当前mission参数cube_order_policy_file为空，未启用该表，因此没有改写旧表；禁止新agent重新启用它或声称默认受该表优化。

### P2/P7：速度发布冲突与扫掠带中停车

实测运行图发现/cmd_vel除velocity_smoother外，还有behavior_server、协调器、抓放节点等发布端点；直接对位与Nav2也曾共用/cmd_vel_nav。仅改navigator一个发布者不能消除所有竞争。

新增velocity_selector，所有竞争来源先进入选择器，再进入原安全链：
Nav2 controller/behavior → /cmd_vel_nav；
navigator直接对位/穿越 → /competition/cmd_vel_direct（显式direct_control控制权）；
协调器微调 → /competition/cmd_vel_manipulation；
抓放worker停车脉冲 → /competition/cmd_vel_arm_stop；
selector → /competition/cmd_vel_selected → velocity_smoother → /cmd_vel → collision_monitor → safety_limiter → /cmd_vel_safe。

直接控制期间只有新鲜直接命令生效，失联输出零；不回落Nav2。协调器和机械臂输入使用0.35秒短租约，机械臂停止脉冲优先。运行图确认/cmd_vel只有velocity_smoother一个发布者，/cmd_vel_selected仅selector发布、smoother订阅。Nav2内部行为插件仍可能有多个DDS发布端点，不能等同多节点无序控制。

safety_limiter中的通行逻辑原先按x“无限长车道”推断，会作用到远离原障碍的区域甚至长墙。改为仅原两个圆柱、且机器人与当前障碍Y距离在既有caution范围内才使用该逻辑；不改危险距离/净空。**保留零指令不主动生成运动**：碰撞监控或人工停车不能被下游自动覆盖为前进。静止被外部障碍撞并未因此自动解决，必须在车道外准入。

### P4：准入预测与实际执行不一致

原预测按直线0.8m/s模型，但放行后交还Nav2可能重新转向、绕行、减速，预测不约束实际穿越。本轮在车道外等待并对正，准入后保持短直线穿越，清出车道才交还Nav2；没有“先前得分后不再防护”的豁免。

最后代码还修正了“窗口恰好已清就直接把Nav2放行”的分支，使所有被触发的原mover准入使用同一执行轮廓。穿越失败有界报错，不把超时当安全放行。一般障碍等待超时也不再resuming anyway。

预测仍依赖冻结原mover轨迹、0.8m/s设定与调度延迟；安全链减速/车辆偏差可能破坏模型。没有宣称形式安全保证。后续需检查实际速度、后续遭遇、停车点是否已在带内。

读取活动mapn3地图后放弃“永久封禁完整原mover扫掠带”：原障碍北侧紧邻静态墙，新增整带禁区有堵通道风险。现有两墙keepout未改，物理布局与障碍参数未改。

### 确定运行时崩溃

初轮1rA_4bB触发倒退分支时，TaskNavigator调用不存在的_segment_is_free并退出。修为现有_costmap_segment_clear，增加防回归测试。该轮人工停止，明确标记INTERRUPTED，不作为完成结果。

### P3：规则与实现口径

工作区竞赛规则原文要求至少两个有效动态障碍，没有要求只由原始obstacle_1/2提供。monitor现在计两个不同、真实遭遇且观测到yield/detour的动态实体；新增墙可计入。保留original_gate_events/original_pair_demonstrated及legacy dual_yield诊断。

离线审计进一步要求响应实体属于真实encountered与response_obstacles交集。新增two_dynamic_responses；motion_functional_pass按规则计分。safe仍要求四障碍、样本、无碰撞和原净空门。旧25题由严格11变宽15只是历史离线重算，**不是本修复版实测成绩**。

### 批处理和报告

run48检查点由CASE_ACCEPTED单一标志改为同时要求audit.motion_functional_pass。run_ordered_case清理先SIGINT并留退出窗口，再KILL，避免直接TERM引发孤儿节点。

summarize48_v2原先只读一层目录，漏读run48的tag/attempt结构。现在递归读question.txt、按最新尝试，安全审计失败不能计入under300；人工中断不能计入strict_motion_pass。retry列表按严格功能门生成。full48_report.py读另一种results.jsonl格式，本轮未混用。

## 验证范围与证据

- 构建：tools_demo、bot_navigation通过。
- 行为测试：667 passed。命令排除了已有flake8/pep257/copyright整仓测试；不等于全仓规范检查通过。
- 新增测试覆盖租约/控制权、机械臂停车优先、缺失方法、直线穿越/超时、已清窗口仍受控、嵌套汇总/最新失败/中断、不同障碍响应与缺少遭遇证据。
- 运行图：velocity-graph-final.txt。
- 初轮1rA_4bB：导航器崩溃后人工停止，不完整。
- 3rA_2bB：420秒runner失败，0/5，red1已抓但未放；elapsed_sec=300.6是最后一条稀疏mission状态的时间，不能当runner退出时完整耗时。该轮没有可验收净空终态，collision_faults=[]不能据此证明整轮安全。
- 运行时源文件固定；中途只调整离线汇总和测试，不改运行节点。最终单块轮前完成最后控制分支修改，版本差异保存在各轮worktree.patch，不合并轮次成绩。
- 原始bag/日志分别在VM experiments/repair-20261005-round2-{first,final,single}/。全部保留，不删除旧25题及先前失败。
- 世界SHA256：51f23aa60c51ac61c8d74543f37fcde2118eaba7cf185e534909a1a951fdc5d4。

## 未完成与下一步

1. **A西侧持块路线恢复卡住**：约(-7.0,2.4)附近，检查墙1实际矩形、keepout/inflation、global动态点、载荷mask、/plan与实际轨迹。不要把recent_trail日志直接当已证实的唯一根因；先带costmap/scan/odom/墙状态bag做局部复现。解决前不盲开48题耗时。
2. **原mover静止被撞、第二次及以后穿越**：新控制链和受控准入是候选修复，尚无全分布物理保证。保留所有负净空历史，不能重跑成功后隐藏。
3. **红块east-only整族、C族**：未完成本最终版本完整矩阵；旧轮不跨版本借成绩。
4. **机械臂首用/服务拥塞**：未改，实现与验收需证据驱动。
5. **运动与完整墙钟300秒**：未稳定证实；API/LLM计时与展示链本轮未执行。
6. **CoStudio/Windows IndexedDB**：本轮未操作，不声称连接已验收。
7. **48题全矩阵**：交给下一agent，以验证为主，仅小修；完整提示词见HANDOFF_FULL_VALIDATION.md。

## 备份与提交边界

VM /home/ros/repair-20261005-round2/source-before.tgz 和 preexisting.patch保存初始现场，包括上一agent候选允许表与用户工作树变更。旧stash不删除。
用户GUI刷新率、支撑轮TF和既有未跟踪文档/preview世界不纳入本轮提交。只推master，不向huxuyang推送。

## 最终收尾记录

- 最后单红→B：red_cube_5 east抓放完成，1/1、零跳过、105.3 s；两个原mover均yield，但最小净空0.0669 m（obstacle_1），CASE_ACCEPTED=false。**安全失败，不能算通过。**
- 据此最后补正准入模型：要求名义0.8 m/s和NearSlow下0.56 m/s、0.4/0.8 s延迟四种轮廓全部清净才放行。保持现有0.60 m预测中心距和0.10 m验收门不变；没有修改安全减速。这是对过于乐观单一速度预测的修正。
- 最后预测修改通过667项行为测试，**未再做物理复测**；源数据反映修改前候选，不能改写为最终版本通过。实际速度低于模型或失去窗口时仍可能失败，先预检再开矩阵。
- 新增Python模块/测试与mission launch限定flake8通过；既有整仓规范告警不等于全部修复。
- 当前已停止仿真，无完整48题运行。交接第一项必须是单红B净空回归，再检查3rA_2bB的A西侧卡住。只有预检无阻塞才能跑48题。
- 最终提交与远端核对记录见主机交付目录的COMMIT.txt；仓库提交由git log核实。
