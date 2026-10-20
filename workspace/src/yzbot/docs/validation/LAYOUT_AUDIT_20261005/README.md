# 本轮工作复审与必要修复（2026-10-05）

接手版本：`huxuyang@c1fe40b01afdc8109afb26a6f1479bd3b7487189`。本次限定为审查、必要修复和一次困难题短验证，未开展 48 题、付费 API 或 Windows CoStudio 全链路验收。

## 结论

上一轮修复有效改善了载荷导致的导航停滞，但仍存在确定的安全逻辑缺陷，不能认定已经稳定完成目标。本轮修复后，`4bC_1rA` 单次 232.5s 完成 5/5，零跳过，抓取/投放顺序与真值验收通过，原障碍 1、2 均有 yield，记录最小表面净空 0.1618m，`collision_detected=false`、`CASE_ACCEPTED=true`。

该结果仅为当前改动的一次 motion-only 验证。实体选择为 `blue_cube_3 → blue_cube_2 → blue_cube_4 → blue_cube_1 → red_cube_1`，路线不一定等于 r10 的失败实体序列；不能据此断言旧接触相位已经逐项物理复现并被彻底消除。重复穿越保护另有行为测试，完整物理稳定性须下一轮验证。

官方世界 SHA256 保持 `51f23aa60c51ac61c8d74543f37fcde2118eaba7cf185e534909a1a951fdc5d4`。物块、zone、四个障碍中心/尺寸/运动参数、footprint 和现有安全阈值未修改。用户 GUI/支撑轮 TF 改动仍未提交。

## 审查发现及修复

| 问题 | 代码证据与影响 | 本轮处理 |
|---|---|---|
| 记分后停止保护 | `_mover_yield_topic` 跳过 `mover_response_seen`，原障碍在后续穿越失去相位等待；r10 第四题已接触障碍 2 | 每次穿越独立进行风险判断，记分仅用于验收；单次 admission latch 离开车道后重新设防 |
| 障碍速度估计失效 | `_update_mover_track` 将当前 Y 写入锚点，再以同一 Y 相减，几乎得到零速度 | 用两个独立真值样本计算速度；采样过旧或未知方向不能放行 |
| 当前方向不足以预测回摆 | 原释放判据只看侧向距离和瞬时方向，不能证明整段穿越安全 | 加入冻结 Y 巡逻的反射运动预测，估算完整直线穿越窗口；安全距离包含采样余量。明确这是 admission 模型，不是安全证明 |
| 等待超时强行恢复 | 8s 超时分支无条件恢复且设置冷却，即使障碍仍冲突 | 不安全超时报告 `mover_crossing_unsafe_timeout` 并停止该腿；等待开始立即发送零速度，超时不再绕过判据 |
| 载荷屏蔽制造虚假自由空间 | 原返回 `inf`，Nav2 开启 `inf_is_valid` 时可按最大射程清除遮挡后方标记 | 屏蔽值改为 NaN，表示无观测；保留其他束和元数据，不把载荷后方当成空地 |
| 非法状态消息可使节点异常 | payload mask 对合法 JSON 列表/null/bool 调用 `.get` | 只接受对象中的字符串 state，非法消息保留原电平状态；navigator 同样过滤非法/非有限真值 |

当前激光在 base_link 上 XY 偏移为零、yaw 为零，激光系盒与底盘平面盒对应；换机器人或外参时必须重新核对。持块状态是可靠、TRANSIENT_LOCAL、事件发布，不能把“长时间没新消息”误判为空载。屏蔽框内无法区分的外部障碍也可能被屏蔽，这是传感器盲区，不能声称所有真实回波均保留。

## 测试和仿真证据

- 构建：`colcon build --packages-select tools_demo --symlink-install` 通过。
- 行为回归：`mybot/test` 与 `tools_demo/test` 排除包内 flake8/pep257/copyright 检查后 **649 passed**；新增 9 项行为用例，覆盖记分后再穿越、离开后重新设防、真实速度估计、未知/陈旧真值、回摆相位、不安全超时及非法 JSON。
- 定向测试：新增 crossing admission 与 payload mask 合计 **17 passed**。
- 全目录第一次检查：`tools_demo/test` 为 `598 passed, 1 skipped, 2 failed`，失败是 flake8/pep257。新增文件和 payload mask 本轮修正后单独检查通过；navigation_math/task_navigator 仍有基线样式问题，已与 HEAD 提取的文件比较，不能写成全库检查全绿。
- 仿真：`Move 4 blue cubes to zone C and 1 red cube to zone A.`；motion-only、LLM disabled、dashboard disabled，超时上限 600.0s 用于保留失败证据，300s 达标仍单独判断。
- 终态：completed=5，skipped=0，232.5s；`audit.json` 的 `order_verified=true`、`truth_precise=true`、`dual_yield=true`、`motion_functional_pass=true`、`motion_under_300=true`；`full_competition_acceptance=not_tested`。
- 四个障碍最小记录净空：原障碍 1 **0.1618m**，原障碍 2 **0.2855m**，新墙 1 **0.3244m**，新墙 2 **1.5476m**。原两障碍分别记录 yield；新墙 1 额外 yield，新墙 2 未记录响应，不能写成四障碍都避让。
- Bag 校对原始与屏蔽 scan：新增 **27295 个 NaN 束**，其他可匹配束保持原值；被屏蔽原始 range 约 0.38–0.4965m（盒判据使用前向/侧向投影，不等于 range 上限）。原始 `/scan` 与输出的 `range_min` 均为 float32 约 0.38，frame 为 laser_link。
- 运行中未替换代码、重建或启动并发仿真；结束后确认本轮 launch/gzserver/runner 已退出。物理验证后只进行了换行与文档修订，无行为变化。

复现行为检查（从仓库目录）：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
python3 -m pytest mybot/test tools_demo/test \
  --ignore=tools_demo/test/test_flake8.py \
  --ignore=tools_demo/test/test_pep257.py \
  --ignore=tools_demo/test/test_copyright.py \
  --ignore=mybot/test/test_flake8.py \
  --ignore=mybot/test/test_pep257.py \
  --ignore=mybot/test/test_copyright.py -q
```

## 旧报告修正

旧报告历史数据保留，但已修正：四类改动不能统称完全闭环；不同版本各题最好成绩不能组成同版本四题通过；r3 表格中 262.8s 实属 r6；从少量混合版本尝试推断“每 8–10 题接触一次”不成立；margin 0.10 加膨胀层不是精确物理净空证明；先跑 48、80% 通过率作为交付门禁不合理。

“RTF≈1 因此排除服务冻结/拥塞”、载荷回波 0.345–0.373m 与 `range_min=0.38` 的关系、10 次机械臂循环是否必然最大耗时，均需要原始时间轴/坐标含义/阶段数据支持，不应继续当成无条件事实。

## 尚未完成与下一步

1. 当前 admission 模型假设原 Y 巡逻边界 [-0.8,0.1]、标称速度 0.225m/s，底盘直线穿越速度 0.8m/s、派发延迟 0.4s。实际慢行、转向、重规划、停在巡逻线内、反向行驶和暂停后的行为尚未全部验证。不要将本轮一次通过升级为全场景保证。
2. 最终 safety limiter 的 swept-band transit 与旧通用 reverse/wait 超时恢复分支仍需联合审查；本轮未改这些既有策略。强行通行与停在扫掠带都可能不安全，需通过进入前预测和全过程保护解决，不能仅靠记分或固定冷却。
3. `4bC_1rA` 至少两种相位重复、其余三道代表题、48 题固定版本矩阵未跑；已知失败必须先过安全门禁再投入全矩阵。
4. recent_trail/lethal start、机械臂首用 CONTROL_FAILED、服务超时、恢复语义仍未根治；每类要拿证据，而非笼统称为间歇问题。
5. 完整比赛墙钟、真实 API、出题和 CoStudio Windows 展示未测；motion=232.5s 不等于完整比赛用时。
6. 新墙 2 本题绕行安全但没有独立响应，报告必须区分被看到、被接近、真实响应和未遭遇。

按用户最新要求，下一位 Agent 使用同目录 `HANDOFF_FULL_VALIDATION.md`，以现有短验证作为启动基线，直接后台跑 48 题，汇总后只处理可局部修复的问题，再检查现有全链路计时与展示；不开展大幅改造。48 题本身主要耗仿真时间；集中看摘要、按失败签名取日志可显著减少 token 消耗。

## 文件与备份

VM 原始证据：`/home/ros/dev_ws/experiments/layout-audit-20261005/20261005T022415Z-LoXFIp/`，含完整 launch、result、audit、bag、原题、HEAD、源配置 SHA256 和工作树 patch；这轮录包约 27.4MB，不入 Git。

本轮备份与命令日志：`/home/ros/layout-audit-20261005/`，包含 `source-before.tgz`、`preexisting.patch`、测试/构建日志、HEAD 样式基线、`run-focused.sh` 和启动 PID。原 r1–r10 及旧 stash 不删除、不覆盖。

仓库摘要：`docs/validation/LAYOUT_AUDIT_20261005/`；主机交付：`D:\user\Desktop\AIC\layout-audit-20261005\`。Git 只提交本轮代码、测试、审查文档和小型证据；用户 GUI/TF 原改动保留工作树。

## 补充检查与本轮任务收紧

用户明确要求下一位 Agent 以全验证和简单修复为主。本次补查仅进行只读检查和文档更新，没有改变运行代码、参数或布局，也没有启动新的仿真。

- VM huxuyang 与远端一致，运行代码基线 f1a56323cc3d6de7b38dc175936cd44dfb78cf46。
- ROS 环境实际 import 路径为 /home/ros/dev_ws/build/tools_demo/tools_demo/，三模块最终解析到仓库源文件，字节一致；不能用假定的 site-packages 路径判断部署。
- run48_v2.sh 和 run_ordered_case.sh 的 bash -n 通过；生成器 48 行、48 个唯一题目、48 个唯一标签。
- 无残留 launch/gzserver/runner；磁盘约剩 12GB。
- 断点 COMPLETED 根据 runner 成功标记；全验证仍须复核 audit.json 的顺序、真值和功能安全字段。完整计时仍未测。

新版提示词不要求从头重做方案或增加一轮大型代表题预检；直接固定版本跑矩阵，批次后集中分类、小修和定向回归。需要系统性改造的失败保留为未完成项，不扩大本轮工作。

## 交付分支更正

用户指定所有交付与后续推送使用 master。本轮已验证代码及文档通过快进同步到 master，VM 工作分支也为 master；前文 huxuyang 是历史运行来源。物理验证、代码和用户未提交改动均保持不变。新的全验证 Agent 以 master 最新 HEAD 为准，不向 huxuyang 推送。
