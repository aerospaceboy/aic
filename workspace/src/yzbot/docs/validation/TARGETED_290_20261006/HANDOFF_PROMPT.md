# 下一位Agent接手提示词：定向修复之后完成48题290秒验收

你从零接手YZBot机器人竞赛项目。当前任务是小步优化并完成全48题验证，直到同一冻结运行版本的48个原始题目全部符合任务和安全要求、运动墙钟严格小于290秒。先读当前事实，避免重做已经修好的问题；不要大改布局或架构。不能停在启动后台批次、几题通过、旧版48通过或只写建议。收到人类停止要求必须停止；确有外部阻塞时保留现场、说明阻塞，不能宣称已完成。

一、规则和范围

最新人类通知取消颜色先后顺序要求：“每抓取/放置一个货物得4分，总分为20分。未按照出题要求抓取不得分。”允许跨颜色交错，但原题的颜色、数量、目标区域不变。五件正确抓取并精确投放货物才是20分，不能抓取/投放重复计算成40分；实体不能重复使用。原题TaskPlan和解析校验保留，仅重排执行jobs。legacy order_verified可以false，新requirements_verified、pickup_delivery_consistent必须true。

保留现有分散布局和四个移动体的连续巡逻。合格门：5/5、skipped=0、goods_score=20、真值valid/precise、零碰撞、全程最小体表净空严格>0.10m、四移动体在线、有真实遭遇和至少两次不同移动体的真实响应证据。当前工程还保留original_pair_demonstrated原始双障碍证据，单独报告并先继续满足；未经新明确规则不要放宽该门。不能把actor在线、人工置yield、删除障碍或放宽安全阈值当作避障通过。

290秒是工程余量门槛；比赛300秒完整墙钟必须另外验收，包含出题/API/展示等实际起止点，不能用motion-only成绩宣称全链路达标。诊断runner的600秒上限不改变290秒通过门。

二、环境和保护

主机D:/user/Desktop/AIC；VM ros@192.168.153.128，Ubuntu22.04/ROS2 Humble。工作区/home/ros/dev_ws，仓库/home/ros/dev_ws/src/yzbot。复用主机sprint300-20261005/scripts/Invoke-VM.ps1和既有私有SSH配置，不打印其中密码。不将API密钥写入代码、Git、日志和文档，也不修改未经授权的模型配置。

先读AGENTS.md、README.md、PROJECT_HANDOFF.md、docs/validation/TARGETED_290_20261006/及上一轮UNORDERED_290_20261006报告。核实git status --short、git log -8 --oneline、git rev-parse HEAD、git ls-remote origin refs/heads/master，再读最新审查报告、逐腿表与运行证据。只提交推送master；不动huxuyang，不force push。

保留两项用户未提交修改：bot_navigation/launch/competition_navigation.launch.py的支撑轮静态TF、gazebo_debug_gui/src/gazebo_debug_gui_plugin.cpp。保留stash、未跟踪旧文件和历史失败证据，禁止reset --hard/git clean/广泛清空VM。只git add明确属于自己的文件。修改前备份patch与基线引用。确认没有别人仿真，清理时仅处理自己拥有的PID/进程组，确认子节点退出，不能广泛pkill误杀自己或其他任务。

三、代码和本轮事实

新规则实现提交0326585，文档交付3f1e698；本轮运行代码c124b24，最终文档HEAD以远端核对和部署收据为准。旧8a0f538的48题是同版本复跑收敛、旧<300秒全部通过；按新290秒仅46/48。不是一次性全过或稳定性证明。新规则与新评分版本尚未完成物理48题。

当前unordered_schedule.py在最多5任务、10实体和当前允许接近侧中做有界DP，保留每个(color,target_zone)配额与实体唯一性；显式operator override优先。评分失败退回正常执行，不许以跳过货物兜底。

本轮确认并修复了首腿估价漏洞：导航器从出生点先经过双动态障碍展示路线到(0,0)集结点，再去首件；旧DP却按出生点直线到物块评分。staged_cube_delivery_cost复用导航器同一个dynamic_gate_waypoints，把既有集结路段纳入代价；非出生段保持旧评分不变。只改navigation_math.py、mission_coordinator.py、新测试三文件。未改实际导航航点、布局、速度、机械臂和任何安全参数。行为测试734 passed，3个linter deselected；tools_demo构建通过，新测试style干净，修改文件相比基线零新增诊断。

本轮修正后：1rC_4bB270.6秒/净空0.2060m；4bB_1rA288.5秒/0.1943m；3rA_2bB237.2秒/0.3778m，三题均5/5、零跳过、20分、四移动体在线、原始双yield、零碰撞。修正前本轮前两题250.6/276.7秒，因此没有证明提速；4bB_1rA仅1.5秒余量，最先重复复测并校准实际路线代价。若证实负优化，最小还原coordinator调用，保留新规则、审计和备份，不整体reset。实测成绩及是否提速以本轮审查报告case-results.csv为准，不能用更小的代价数值代替计时，不能挑旧最慢记录宣称稳定收益。before/after不是同障碍相位的严格A/B。旧1bC_4rB294.2秒在上一轮新规则单次243.3秒通过；本轮重点复测1rC_4bB、4bB_1rA，并回归3rA_2bB。这些仅是定向单次，不能替代完整矩阵。

证据：/home/ros/dev_ws/experiments/targeted290-before-20261006、targeted290-after-20261006；上一轮unordered-smoke-20261006；旧完整distributed48-r7-20261006。完整launch日志留VM，主机与仓库有audit/result/question/commit/manifest及逐腿摘要。沿着证据读，不先猜“必是动态障碍”或“必是假障碍”。

四、尽快达到48题的执行路径

1. 核对部署、运行模块与配置确实来自当前源码，运行受影响测试和必要构建。当前行为734不是忽略linter后宣称仓库style全绿；既有债单独列出。
2. 先补最脆弱的定向题：3rB_2bA、3rB_2bC（旧净空0.1002/0.1020m），曾碰撞的4bA_1rC；本轮最慢题若余量不足，改变启动相位再复测一次。只在失败有证据时修；避免无目的调参。
3. 定向无真实安全失败后，冻结最新代码、安装、配置、用户patch，马上串行跑完整48题。原题措辞两种颜色先后共48条都保留；虽然配额等价，解析与相位覆盖仍有价值。禁止拿旧运行结果填新版本表。
4. 让仿真后台运行，模型只读简短计数和失败窗口；仿真本身运行时长不等于持续消耗推理token。无需每几秒读全日志。每次检查只输出已完成/失败/当前tag、时间和签名，失败再读取限定窗口。
5. 同版本只补失败/缺证据题，保留所有attempt，最新失败覆盖旧成功。若出现真实碰撞，安全停止本轮自己的批次、保留证据、修根因；不要同版本无限赌相位。改运行代码或配置后新建根目录，完整48重新跑，受影响题优先。
6. 48最新证据全合格后，再重复耗时接近290和净空接近0.10的题，报告首轮通过率、补跑次数、最差时间/净空，区分“全部有合格证据”与“重复稳定通过”。

VM Bash启动范例，先确认现有进程归属，source时不要提前set -u：

source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
cd /home/ros/dev_ws/src/yzbot
python3 -m pytest -q -m 'not linter' tools_demo/test mybot/test
cd /home/ros/dev_ws
colcon build --symlink-install --packages-select tools_demo
source install/setup.bash
cd src/yzbot
ROOT=/home/ros/dev_ws/experiments/targeted290-full48-$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "$ROOT"
RLC_TIMEOUT_SEC=600.0 setsid bash tools_demo/scripts/run48_v2.sh 1 48 ordered "$ROOT" > "$ROOT/campaign.log" 2>&1 < /dev/null &
echo "campaign_pid=$! root=$ROOT"

记录实际PID、HEAD、工作树patch hash、world/map/config/install hash和命令。用不依赖SSH会话的包装运行；不得并发仿真。批次期间不改源码、不构建、不改参数、不commit；文档先写仓库外，否则版本锁可能中止。只有测量结束、清理确认后才应用下一版。

python3 tools_demo/scripts/summarize48_v2.py "$ROOT" "$ROOT/final_table.csv"
python3 tools_demo/scripts/check48_290.py "$ROOT"

必须expected=48、accepted=48、failing=[]、退出0。当前run48标记含290门；check48_290强制固定48题并检查新货物门。checker仍需补充人工/脚本核对每题版本、用户patch、source/config manifest、响应和净空。不要使用experiments/converge48.py或主机旧副本直接关单，它们仍有旧规则/旧枚举口径。每题缺证据、启动失败、中断都必须进入48行，不能从分母删除。

五、出现问题时的具体解决路径

A. 5/5但>290：先生成逐腿表，区分正常长路径、控制器零速、避障等待、机械臂、恢复。每轮只改主要因素。DP已修首腿集结，但其他评分仍可能忽略静态墙/强制航点，且使用zone nav pose而非真正退出位姿；B东侧镜像退出也不是固定状态。先用日志提取actual exit与forced waypoint链，针对错估边加入估价；若加入地图路径缓存，用world/map/allowlist/route-profile哈希做键，不把旧布局测量复用新图。不靠再加巡航速度解决窄路摆动，不先大改布局。至少测难题和一道原稳定题后才默认启用。

B. 同点重试、collision ahead零速：同步看/plan、各costmap、cmd_vel_nav/selected/safe、里程计、控制器状态。recent_trail只是标签，不能当假障碍实锤。自身格膨胀cost不等于致死，不能简单把cost>=50都当lethal并任意倒车。识别“原地无进展且同目标重复”的有界条件，选择经几何检查的安全回退/集结点，取消旧goal并确认结束后请求新plan；保存剩余任务和已交付状态。恢复计数/时长有上界，不能重复抓已投放实体或从头重启题目。

C. 停车被移动体扫中：不是停车就安全。看扫掠包络、车身形状、侧向间距、障碍速度和回摆时相，等待点必须在扫掠区外。安全检查贯穿全部穿越，包括该障碍yield计分之后。恢复/退出路线也检查移动体；不放宽collision/clearance阈值，不把真实障碍当载荷滤掉。禁行带若推远导致失去真实响应，只能调整合法安全的遭遇/闸门路径，不伪造事件。

D. 抓取/附着服务异常：区分MoveIt规划失败、CONTROL_FAILED、控制器未ready、Gazebo实体/附着服务超时。已有payload_scan_mask处理持块自身回波，不重复破坏稀疏状态锁存；真实障碍和盒外点保持。附着后服务不响应时，先查询实体/夹具状态并有界重试，未知状态不能直接继续放置或重复计分。诊断日志标启动环境类/操作类/导航类，不把全部失败叫环境偶发。

E. 题未接管/STALE/残留节点：核对唯一栈、生命周期、navigate_to_pose可服务、传感器与任务订阅ready。保存任务文本与启动失败证据，停止自己进程组并确认全部子进程退出，再重启。不得在旧批次还存活时开新批次，不使用可能匹配当前SSH shell的泛化pgrep -f清理。

F. 顺序字段false：先核对新配额、实体唯一性和pick→truth一致性；取消颜色顺序不代表可以忽略错颜色/错区域、抓取失败后假成功。保留legacy字段作为诊断，改错调用者，不能直接把order_verified写true。

布局只有在物理可达性/停车空间确证不足、路径修复无效时才动，且需人类授权范围内最小改动。同步world及托盘、zones/mission、导航默认几何、allowlist、策略表、地图/风险/keepout生成器、合同测试与文档；检查所有实体接近侧停车footprint、扫掠体净空和抓放位姿。改布局就冻结新版本重新全48，绝不能只动world。

六、API/可视化与最终交付

motion48通过后，接入当前用户授权的真实API配置，测试至少混色/跨区域/含数量算术的题，记录解析配额正确率、超时/重试、API延迟与完整墙钟。密钥只用仓库外私有环境；401/403立即失败，旧任务迟到响应不得覆盖新generation。未测字段填“未测”，不能填0。用CoStudio连接实际rosbridge WebSocket并验证任务/数量/进度/抓放/地图/图像话题，RViz地图/TF正常。只读与控制按钮权限分开；手动速度走安全输入链，不直接向/cmd_vel_safe发布绕过监控。

报告固定48行表，列：tag、原题、运行commit/patch/manifest、完成/跳过、20分与配额/精确投放、四移动体、原始双yield/两次动态响应、碰撞、最小净空、motion秒、<290、API秒/全墙钟或未测、attempt次数、失败签名、证据路径、最终结论。另有首轮统计/历史失败表，不只展现最后成功。

收尾更新审查报告、完整工作日志（含失败和未完成项）、操作手册和下一位提示词。检查diff与密钥，精确提交master、推送、核对origin/master。只在新版本48最新证据全门通过后宣布该运动矩阵目标完成；完整比赛和重复稳定性另列。保持工作直至完成，不以写报告或启动批次替代实际结果。
