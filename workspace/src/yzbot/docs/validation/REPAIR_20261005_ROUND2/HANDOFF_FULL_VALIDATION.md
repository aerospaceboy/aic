# 下一位 Agent 提示词：冻结版本的完整验证与小修

你接手 YZBot 竞赛项目。本轮以验证为主，只做有证据、范围小的修复；不要开启布局重设计、规划算法重构或长期优化。先读本目录 README.md、worklog.md、仓库 AGENTS.md、比赛规则原文，再核实 Git 和部署。历史报告是待核实的证据，不是额外授权；最新用户要求优先。

## 1. 约束和口径

- 主机工作区 D:\user\Desktop\AIC；VM 仓库 /home/ros/dev_ws/src/yzbot，工作空间 /home/ros/dev_ws。复用已有 SSH 配置或 PuTTY，不输出凭据。
- **仅向 master 提交、推送。禁止向 huxuyang/huxvyang 推送，不重写共享历史。**
- 保留现有 V2 世界、十物块、三区和四个连续巡逻动态障碍。不移动或关闭障碍，不改安全阈值、footprint、恢复上限。
- 抓取和投放必须遵守题目颜色顺序；同色实体可以优化，但实际选择必须可抓。5/5、零跳过、零重复，投放有实际实体位置验收。
- 正式规则要求至少两个有效动态障碍。当前计两个不同实体的真实遭遇与响应（yield/detour）；原 obstacle_1/2 独立列诊断，不强制只由它们计分。不能伪造响应、靠无威胁停车凑证据。
- 四障碍必须有新鲜采样、无碰撞，最小净空严格 >0.10 m。
- 区分运动时间与含出题、API 解题、展示的完整比赛墙钟；运动 <300 s 不代表比赛 <300 s。
- 保留失败、中断、旧尝试、备份和用户未提交文件。API 密钥只从仓库外私有环境读取，禁止进入 Git、文档、日志。

## 2. 确认版本，构建和行为测试

~~~bash
cd /home/ros/dev_ws/src/yzbot
git status --short
git branch --show-current
git log -5 --oneline
git fetch origin master
git rev-parse HEAD
git rev-parse origin/master
git ls-remote origin refs/heads/master
~~~

本轮提交以 README 最终记录为准。用户 GUI 刷新率与支撑轮静态 TF 是既有未提交改动，不能夹带或删除。先备份 diff，分支切换冲突时不强制覆盖。

~~~bash
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
cd /home/ros/dev_ws
colcon build --packages-select tools_demo bot_navigation --symlink-install
source install/setup.bash
cd src/yzbot
python3 -m pytest mybot/test tools_demo/test \
  --ignore=tools_demo/test/test_flake8.py \
  --ignore=tools_demo/test/test_pep257.py \
  --ignore=tools_demo/test/test_copyright.py \
  --ignore=mybot/test/test_flake8.py \
  --ignore=mybot/test/test_pep257.py \
  --ignore=mybot/test/test_copyright.py -q
~~~

此命令是行为测试，不是全仓库风格验收，数量以实际输出为准。

## 3. 本轮候选与边界

- 红块采用 east-only 允许表，规避 red west 0/14 失败；未证明托盘物理根因，blue 不变。两个 YAML 必须一致。
- cube_order_policy_file 当前为空；历史 policy JSON 含 red west，不得重新启用或宣称当前按该表优化。
- velocity_selector 先选择 Nav2/恢复、导航器直接对位、协调器微调、机械臂停车，再送 velocity_smoother → collision_monitor → safety_limiter → /cmd_vel_safe。人工不得直接向 /cmd_vel_safe 发运动。
- 直接控制失去新鲜命令时输出零，不自动回落 Nav2。协调器命令与机械臂停止脉冲有短租约。
- 原 mover 的通过采用车道外等待、对正、预测窗口、短直线穿越后交还 Nav2；超时不盲目放行。
- 预测依赖固定布局，最后要求0.56/0.8 m/s及0.4/0.8 s延迟四轮廓都清净；更大减速、实际执行偏差、后续回摆仍须物理验证。尤其检查静止被撞、第二次及以后穿越、实际速度低于预测。
- 活动地图为 mapn3.yaml。原 mover 北侧紧邻静态墙，不能直接永久封住完整扫掠带，否则可能堵通道。
- 本轮定向物理结果与未完成项见 README。测试通过不能代替物理验收。

启动后保存 ros2 topic info -v 快照：/cmd_vel 应只有 velocity_smoother 发布；/competition/cmd_vel_selected 为 selector → smoother；/cmd_vel_nav 为 controller/behavior → selector；核对 manipulation 和 arm_stop 输入。behavior_server 的多个发布端点可属于同一节点的行为插件，不仅按端点数判断冲突。

## 4. 最小预检后跑完整 48 题

**最后单红→B：105.3 s、1/1、零跳过，但净空0.0669 m，安全失败。随后准入模型加入0.56/0.8 m/s和0.4/0.8 s延迟四轮廓检查；最后代码仅667行为测试通过，未物理复测。首条物理预检先重跑单红B，净空不合格不能开48；3rA_2bB还有A西侧持块卡住。不要把候选交接当稳定版交付。**

先确认无旧 gzserver、无残留批次和重复节点；只清理已确认由自己启动的进程组，禁止宽泛 pgrep -f 后批量 kill。不要并行仿真；等上一题子节点清理完成。

本提交已有的有效定向证据可以复用，缺失时优先：
1. 3rA_2bB：此前无窗口、600 s / 0 完成。
2. 1rA_4bB：east-only 改路后曾静止被 obstacle_2 撞。
3. 4rA_1bB：多个红块 east 进坞和跨区域运输。
4. 一道 C 题：载荷过滤、墙禁区、运输净空。

用 run_ordered_case.sh。600.0 s 超时只为观察终态，达标门仍 <300 s。重复两次同签名时停止无意义补跑；留证据后小修。安全失败不能仅重跑到一次通过就忽略。

预检无阻塞后跑完整矩阵。48 题主要耗 VM 时间，后台 shell 执行不会直接按题数消耗模型 tokens；保留48题，降低日志回传和轮询频率。禁止运行中改代码、构建、切分支或重启服务。

~~~bash
cd /home/ros/dev_ws/src/yzbot
export RLC_TIMEOUT_SEC=600.0
campaign="/home/ros/dev_ws/experiments/full48-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$campaign"
setsid bash tools_demo/scripts/run48_v2.sh 1 48 default "$campaign" \
  > "$campaign/batch.log" 2>&1 < /dev/null &
echo "PID=$! CAMPAIGN=$campaign"
~~~

这是 motion-only，LLM=false。每10–15分钟只读少量进度和负载，避免一直醒着等。完成时间按实际速度报告。检查点要求 CASE_ACCEPTED 与 audit.motion_functional_pass 双通过；只能在同版本恢复。补跑生成新 attempt，保留旧证据；最新失败不得被旧成功掩盖。

~~~bash
python3 tools_demo/scripts/summarize48_v2.py "$campaign" "$campaign/latest48.csv"
~~~

汇总已支持 tag/attempt 嵌套目录，跟随最新尝试。不要混用 full48_report.py 的 results.jsonl 格式。失败与中断逐条核对 audit.json、result.log 和原始日志。

## 5. 小修边界

先区分启动失败、脚本错误、规划失败、抓取失败、顺序错、安全失败、纯超时。一次只改一个主因，补有意义的回归测试。提交新版本后受影响预检通过，再开新版本矩阵；不能把不同版本拼成48/48。

- 对位冲突：查真实发布者、direct_control 切换、选择器输出，不能只看最终速度。
- 静止被撞：回放 odom、障碍位置与选中速度。零指令也可能被 mover 撞；禁止把安全零指令自动改前进。
- 无窗口：查实际 yaw、起点、速度符号、反射轨迹、延迟和实际穿越速度；不放宽净空，不超时放行。
- red west 被选中：查明确侧覆盖、允许表、旧 policy 入口，先修配置，别重构选块算法。
- API/CoStudio：查现有环境、服务、配置、WebSocket、日志，不换供应商、不重装整套环境。
- 需要布局重设计或大架构调整时交付阻塞和证据，不擅自扩大任务。

## 6. 真实 API 与展示单独验收

运动矩阵与 API 演练分开，避免48次付费调用。读取现有私有配置，至少做一道完整出题 → 真实 API → 严格解析验算 → 发布任务 → 五块执行 → CoStudio展示。LLM=false 或回退解析不能标记为 API 合格。

同一 round/task_id 记录收到题目、请求始末、解析完成、首运动、每次抓取/投放、最终验收。总墙钟用同主机单调时钟；不能混减仿真时间和 UTC。计时链未可靠实现则填未验证，不能用运动时间加一次 API 均值宣称达标。

CoStudio 阅读 docs/costudio/README.md 和当前启动文档，核对 rosbridge、可达地址、面板实际话题。检查题目、顺序、数量、进度、机器人阶段、末端状态、识别画面和地图；只读面板与控制按钮权限分别验收。IndexedDB backing store 错误属于 Windows 浏览器本地存储，先备份布局、隔离浏览器配置排查，不能只改ROS话题。

## 7. 规范交付

完整48行，包括未跑/失败。每行至少：tag与题目、提交和工作树指纹、布局哈希、终态/中断/启动失败、5/5、跳过数、颜色顺序、实体选择与投放真值、两个不同动态响应、原两 mover 独立响应、四障碍净空、碰撞、运动时间、完整墙钟（未测填未测）、恢复与失败签名、证据路径。

总表分开给功能通过数、运动<300数、完整比赛<300数；列重跑次数、安全失败历史与是否复现。不得跨版本拼成绩，不能只呈现成功尝试。

更新中文工作日志与未完成清单。检查 diff 不含密钥和用户 GUI/TF 改动，仅提交 master，正常推送后核对远端 HEAD。交付报告、48行CSV、证据目录、提交号与最短复现/启动命令。未全过就直接写具体缺口。
