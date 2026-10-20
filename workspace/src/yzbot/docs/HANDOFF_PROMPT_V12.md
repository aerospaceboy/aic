# 接手提示词 V12：C区墙角治理、云API与完整比赛300秒

更新：2026-09-06。本文件是当前统一接手入口；V11保留项目架构、CoStudio话题映射及基础约束。本文件中的现状优先于旧交接的结论，用户新指令优先于所有文档。不要把本提示词当成已完成成果。

## 1. 你的目标及工作原则

继续工作，直到合法题目完整五物块、零跳过、顺序/安全/投放/展示合格，并有足够证据证明完整比赛墙钟低于300秒。不能因单题运动阶段达标、单次实验通过或写完方案而宣布完成。每轮提出假设、保留起点、修改一个主要变量、验证、记录失败、继续下一步；确实缺用户输入或外部条件时明确阻塞与可恢复状态，不承诺跨会话自动无限运行。

用户规则：只约束题目颜色顺序，同色实体可优化。保留DP及真实安全链。用户转述官方建议：CoStudio可视化、API调用大模型解题；不要把“建议”写成未经核实的强制赛规。禁止跨颜色重排、扩大验收边界、缩footprint、删除障碍或用确定性答案冒充模型输出。

权威项目在VM /home/ros/dev_ws/src/yzbot，工作区 /home/ros/dev_ws，master，cty050422/yzbot-224。先读AGENTS.md、git status与最新日志。保留原有七个scripts执行权限变化及历史未跟踪文档，不把它们当成本轮改动。起点8149ea3，本轮最终提交以git log为准。

## 2. 已确认现状

- 修复汇总器CSV列错位、完成与验收混淆、最新无终态被旧成功掩盖、失败补跑清单不完整。重新扫描原始证据，原历史总数仍为45/48验收通过、26题<300秒。新版表包含证据目录、commit及独立验收字段。
- C区不是每次都要100秒：历史同题blue1->C 16.79秒、blue2->C 28.18秒，blue5->C却223.34秒并反复collision ahead。报告“纯结构性长度、根因全部确定”过于绝对，必须区分绕路、碰撞预测停滞和抓取失败。
- 本轮默认路线复跑4bC_1rA：320.6秒、3/5、跳过2，包含blue5抓取失败和blue2去C中止，不是达标。
- 实验路线同题：328.6秒、5/5、零跳过、双yield、最小动态净空0.1406m、全部投放真值通过。四次送C分别blue5 27.83、blue1 16.98、blue2 24.43、blue4 30.46秒。它证明候选可行，不能证明稳定性或全48通过。
- 最终功能回归449 passed、1 skipped；tools_demo构建成功。排除既有flake8/pep257，不等于全仓lint通过。

## 3. 第一优先：复验C候选，随后再省约30秒

实现：tools_demo/tools_demo/c_clearance_route.py，task_navigator参数c_clearance_route_enabled；**默认关闭**。用环境开关进行同条件试验：

```bash
cd /home/ros/dev_ws/src/yzbot
YZBOT_C_CLEARANCE_ROUTE=1 RLC_TIMEOUT_SEC=600 bash tools_demo/scripts/run_ordered_case.sh \
  'Move 4 blue cubes to zone C and 1 red cube to zone A.'
```

先重复该题，再测4bC_1rB；之后复验1bA_4rB、4rB_1bA、1bB_4rA。至少记录两次困难题成功及各段时间，再决定默认启用，不要继续跨版本累计48题。

路线原理：保留安全速度与半径，绕墙端留净空，避免Smac短路径在RPP执行时切角。blue5先绕Wall_113东端及link_6北侧；blue4从托盘南侧绕开Wall_120，不再强制走旧北侧托盘角；blue2从东侧绕Wall_46，再经(-2.1,0)、(-2.1,1.8)越过Wall_38东端。中间姿态沿下一段方向设置，减少无意义原地掉头。已测几何最小中心线距离约0.53m，**不是**扣除机器人/载荷后的净空，也不能替代真实costmap与执行轨迹。

适用范围刻意限制为已知blue5、blue4、blue2离泊区域。尤其不能泛化到red3：直接连到西侧绕行点可能切过red3托盘。再推广时逐段检查地图、物理碰撞和载荷包络。候选用自己的托盘绕行点，不能重复叠加旧通用tray_crossing路线。

328.6秒仍需节省至少28.6秒运动时间，若包含本轮API约21–23秒，则至少需要约50秒额外改进及波动余量。先对候选提取完整阶段预算，排序入站、停靠、扫描、抓取、出站、送达、放置与退出；不能只盯送C这四腿。再比较同色子集/顺序：blue3仅从已验证北侧起点考虑，代价包含前次实际出口、下一颜色首腿和失败风险；不要只优化欧氏距离。运动预算建议<=270秒，真实总时间仍要实测。

## 4. API已接入，但应使用实际测过的推理配置

模型固定为用户指定qwen3.7-flash-2026-07-15。task_parser.yaml已配置用户提供的阿里云兼容endpoint、环境变量名DASHSCOPE_API_KEY、推理开启、max_tokens=4096、connect=4s、read=35s、总预算40s。密钥只在VM仓库外 ~/.config/yzbot/llm.env，权限600，不复制到日志、文档、Git或新agent消息。

启动使用：

```bash
cd /home/ros/dev_ws/src/yzbot
bash tools_demo/scripts/run_competition_api.sh
```

该入口读取私有环境文件，关闭本地LLM服务器，强制use_llm/require_llm。普通competition_system的旧本地服务器默认开关不能作为云端启动方式直接照搬。

实测：简单搬运指令0.767秒正确；真实衣柜题在非推理模式多次算错，被严格校验拒绝；推理开启但15秒read预算超时；最终推理配置下，红先/蓝先两种映射分别23.152/20.943秒，通过算术、颜色、数量、目的地与顺序校验。不能把0.767秒宣传成所有题的解题延迟。

llm_client现有调用墙钟截止：后台传输超时后不发布迟到结果，每客户端最多一个在途传输，尚未退出时返回busy；连接/读取timeout按剩余预算分配。传输错误不反射原始异常字符串，空/非文本/截断响应拒绝。底层超时线程可能短暂继续，后台请求已消费的API额度无法撤销；上层ROS回调仍同步等待，下一轮应实现异步请求+task_id/generation关联，不让解题阻塞状态发布。测试DNS慢、慢字节流、取消后迟到结果、429/5xx预算重试和恢复，维持严格验算与失败关闭。

下一步补宠物题及更多生成器样本，分别记录正确率、p50/p95延迟和token使用。不要通过返回预计算答案换正确率；若优化提示词或思考预算，必须同时比较正确率与时间。当前只是实际HTTP解题与校验通过，尚无API+CoStudio+机器人全墙钟验收。

## 5. 其他审计问题和下一轮任务

1. speed_profile发现来自另一task_id的旧文件；run_ordered_case现在仅在task_id、任务状态及文件新鲜度一致时复制，写profile_status.json。遇到拒绝不能拿旧曲线解释当前运行。失败任务没有新profile时应保留缺失，后续改成每轮独立输出路径与周期快照。
2. 旧run48_v2的COMPLETED目录、旧结果跳过和批次间清理仍需修。先用新汇总器按版本/配置生成补跑清单；启动失败也保存记录，不通过不断重跑隐藏发生率。CONTROL_FAILED不一定全是首用竞态，本轮blue5是pick failed，需区分MoveIt控制、抓取窗口和视觉，不统一归“预热”。
3. 全墙钟必须在比赛开始时生成round_id并记录单调起点，分别记录出题、解题、计划发布、运动开始、第五次合格投放。旧runner elapsed_sec可能是任务状态上次发布时刻，不能当连续更新的终场墙钟；不能仅把它与API延迟机械相加。
4. CoStudio：沿用docs/costudio/README.md和已安装桥接；Windows布局/面板仍未验收。本轮发现桥launch的“never publishes commands”表述不能作为权限保证，rosbridge协议本身可支持发布。先核实该版本配置是否可约束，再设计只读展示和受控开始按钮。URL应按实际WebSocket协议使用ws://，不能照抄http://连接说明。钥匙、鉴权头不进UI；JSON String需转换，区分逻辑任务编号与实际选中实体，detail可能仍带逻辑编号。
5. 既有投放set_entity_pose稳定逻辑未修改；最终真值不等于纯物理精度。保留首次修正前误差证据，不放宽安全/精度阈值。49个测试通过也不替代飞行验证；本轮实际数为449。

## 6. 交付与恢复

本轮报告和证据：docs/validation/AUDIT_20260906/REPORT.md、corrected_table.csv、candidate/result.log、baseline/result.log、API日志、测试/构建日志、BACKUP_SHA256.txt。完整原始仿真归档audit-20260906-evidence.tar.gz，起点bundle/tar在VM backups及Windows audit-20260906/backups。保留历史失败和原始表，不覆盖。

每轮收尾检查diff、相关测试与必要仿真，明确“默认启用/实验开关/方案未实施”。更新日志和当前提示词，提交指定文件、推送origin master并验证远端HEAD。下一条优先命令是本文件第3节困难题复验，然后4bC_1rB；不要再从笼统的“分析C区慢”开始。
