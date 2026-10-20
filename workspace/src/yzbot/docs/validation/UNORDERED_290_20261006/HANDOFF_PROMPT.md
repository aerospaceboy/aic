# 下一位 Agent 接手提示词：新计分规则与 48 题 <290 秒

你从零接手 YZBot ROS 2 机器人竞赛项目。任务是继续小步优化、跑全验证、修复实际失败，直到同一最终运行版本的48个原始题目全部合格且运动墙钟严格小于290秒。不要停在启动批次、部分通过、旧版48通过或几道冒烟通过。不要自行创建无限重试来掩盖真实碰撞；修复碰撞后必须用新版本重新验证受影响题，再完成新版本的全部矩阵。

## 1. 最新人类规则优先

最新通知取消“先做完一种颜色、再做另一种颜色”的要求。原文：“每抓取/放置一个货物得4分，总分为20分。未按照出题要求抓取不得分。”

执行顺序可以跨颜色交错，但原题的颜色、数量和目的区必须严格保持。同一个实体不能重复计分；抓到、放错区域或没有可靠投放验收都不能作为完整得分货物。当前工程按一件正确抓取并交付货物4分统计，五件20分，不把抓取和放置分别重复加成40分。不要根据历史顺序字段伪造新规则通过。

四个移动体继续连续巡逻。碰撞、净空>0.10米、精确投放、真实动态响应、安全链和机器人实际尺寸等要求均保留。290秒是本轮工程门槛，不是把比赛官方300秒规则改写。motion与含出题/API/展示的全墙钟分别记录，不能拿运动数据宣称全链路已达标。

## 2. 环境与仓库保护

主机工作区 D:/user/Desktop/AIC；VM ros@192.168.153.128，Ubuntu22.04/ROS2 Humble。VM工作区 /home/ros/dev_ws，仓库 /home/ros/dev_ws/src/yzbot。SSH使用主机已有私有配置，可复用 sprint300-20261005/scripts/Invoke-VM.ps1；不得打印它包含的密码。凭据不可写入Git、交接文档、命令回显或运行日志。

先读当前 AGENTS.md、docs/validation/UNORDERED_290_20261006/、本提示词旁的审查报告和工作日志。核对 git status --short、git log -8 --oneline、git rev-parse HEAD、git ls-remote origin refs/heads/master，确认本地/VM/远端版本。

只向master提交推送，不动huxuyang，不force push。保留用户两个未提交文件：competition_navigation.launch.py的支撑轮静态TF、gazebo_debug_gui_plugin.cpp；保留stash、历史失败证据和其他已有未跟踪文件。禁止reset --hard或广泛清空VM。每次提交精确列出本轮文件。

## 3. 事实基线，不要重复大改布局

旧运行版本8a0f538，同版本、同用户补丁，48题的最新证据均满足旧300秒motion门槛。这是复跑收敛成绩，不是首轮48/48，也不是稳定性保证。

按新290秒门槛，旧证据仅46/48：1bC_4rB=294.2秒，1rC_4bB=293.5秒。最小净空最低仅0.1002米，阈值余量很小。其中3rB_2bA净空0.1002米、3rB_2bC净空0.1020米，应换启动相位复测，不能把刚过线当稳定。旧失败碰撞、服务超时、302.2/336.0秒等证据全部保留；不得只呈现最后一次成功来声称零故障。

旧完整证据根：/home/ros/dev_ws/experiments/distributed48-r7-20261006。
本轮首题3rA_2bB新顺序单次239.8秒通过，旧记录205.9秒；不是同相位A/B实验，但足以说明不能宣称估算优化就一定省时。

本轮第二题1bC_4rB：243.3秒，5/5零跳过、20分、原始双响应、净空0.3344米，旧记录294.2秒；仅单次验证。两道均CASE与新290门通过。

本轮规则修改与两道冒烟：/home/ros/dev_ws/experiments/unordered-smoke-20261006。
新版本48题尚未跑；API、CoStudio、GUI和完整比赛墙钟不能引用旧motion通过来证明。

现有布局已按之前授权分散并同步，不要为省时间把物块挪回投放桌旁。现有小车仿真底座为planarmove，允许有界linear.y纠偏；不要继续沿用“必为差速、不能横移”的旧结论。

## 4. 本轮代码如何工作

- mission.yaml 的 enforce_color_order=false。TaskPlan保持原题结构，parser仍校验题目颜色、数量、区域和算术；只重排状态机的执行jobs。
- unordered_schedule.py：有界动态规划，在最多5个任务、10个实体、已允许接近侧中选择执行顺序和实体，严格保留每个(color,target_zone)配额和实体唯一性。通过当前 cube_delivery_cost估算代价。估算不是实际地图最短路或实际耗时保证；后续按实际腿表修正代价，不要先做大型重构。
- coordinator记录unordered_schedule日志并使用任务内override；operator显式override优先。候选无完整合法解时退回原执行，不跳过货物。
- audit新增 requirements_verified、pickup_delivery_consistent、goods_score、color_order_required、motion_under_290。order_verified仍描述是否符合旧文字顺序，允许false，不作为新规则门槛。
- run48_v2.sh 的COMPLETED标记现在要求 motion_under_290；summarize48_v2.py包含under290与对应重跑清单；check48_290.py枚举生成器全部48题，以最新证据判定，缺证据即失败。
- VM experiments/converge48.py及主机旧副本仍是旧规则工具，不要直接拿它们做新290秒关单。

## 5. 最快验证路径

先运行与本次有关的behavior tests、构建tools_demo，检查安装目录实际加载新模块及部署配置。全部测试有历史linter债，报告行为测试和style分别列出，不能把deselected写成全绿。

在最新冒烟基础上优先跑两道旧超290题、4bB_1rA、曾碰撞及接近净空阈值的3rB_2bA/3rB_2bC/4bA_1rC。若其中失败，先修最小问题，避免立即耗费6小时全矩阵；若通过，立即冻结版本跑完整48题。

不要删除原48题中颜色措辞相反的24题，虽然新规则下货物配额等价，原始题目、parser和启动相位仍须覆盖。仿真自己运行通常不消耗模型推理token；控制额度的办法是后台串行运行、只读简短进度和失败窗口，不是删题。motion批次use_llm=false；真实API另做有代表性的端到端验收并单独记API开销。

## 6. 冻结并运行全矩阵

下面是VM Bash命令；先核实已有进程归属，无他人仿真和批次，禁止直接广泛pkill。

```bash
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
cd /home/ros/dev_ws/src/yzbot
python3 -m pytest -q -m 'not linter' tools_demo/test mybot/test
cd /home/ros/dev_ws
colcon build --symlink-install --packages-select tools_demo
source install/setup.bash
cd src/yzbot
ROOT=/home/ros/dev_ws/experiments/unordered290-$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "$ROOT"
RLC_TIMEOUT_SEC=600.0 setsid bash tools_demo/scripts/run48_v2.sh 1 48 ordered "$ROOT" > "$ROOT/campaign.log" 2>&1 < /dev/null &
echo "campaign_pid=$! root=$ROOT"
```

600秒仅诊断运行上限，不是放宽290秒合格门槛。source ROS时不要先开set -u，ROS环境脚本可能访问未定义变量。

把PID、HEAD、用户补丁哈希、world/map/config/install哈希和完整启动命令记入campaign manifest。批次期间不改源码、不构建、不改参数、不commit，文档也先写在仓库外，否则现有版本锁会让批次停止。出现真实碰撞，应安全停止自己拥有的批次并留证，再修复；不要继续同版本赌相位。

```bash
python3 tools_demo/scripts/summarize48_v2.py "$ROOT" "$ROOT/final_table.csv"
python3 tools_demo/scripts/check48_290.py "$ROOT"
```

必须expected=48、accepted=48、failing为空，且逐题原始证据与版本哈希可复核。该checker不替代人工核实版本、四移动体和响应细节。同版本补跑只针对失败tag，保留全部旧attempt与失败率；最新失败必须覆盖旧成功。改代码后建立新root，旧版本成功不得填入新版本成绩。确认第一轮缺题也会报失败，不能按campaign.log已经出现的tag数量关单。

## 7. 具体问题与最小解决路径

| 现象 | 先查的证据 | 小修路径与限制 |
|---|---|---|
| 新顺序反而更慢 | 每腿起终点、实际/估算路径长度、等待、航点链、选择侧、放置槽位 | 先补真实路线代价/区域退出姿态估计，缓存用world/map/allowlist版本键；先调整少数误估边，保留基线方案作为候选；不得宣称DP估算是Nav2真实最短路 |
| 双动态响应缺一 | 首腿展示是否因重试丢剩余航点；响应距离；安全层实际放行与navigator一致性 | 保留同task/generation尚未完成的展示路段，修复真遭遇与安全减速；不得手工记yield、放宽响应判据或只凭actor在线判通过 |
| 车道内停车被撞 | actor矩形/圆形、底盘体表净空、受控等待点、回摆预测、cmd_vel_safe | 等待点退到扫掠带外，按行进线横向间隔/制动距离统一闸门；任何逃逸段也复用mover几何安全检查；不能只在首次计分前保护 |
| collision ahead/recent_trail卡停 | 持块激光屏蔽状态、TF、start cost、RPP零速与安全层零速区别 | 排除自己载荷假障碍；稀疏持块状态不能靠2秒超时自动解除屏蔽；有界取消并从当前真实位姿重新规划；禁止无上限重发同目标或全局清空障碍 |
| 抓取重试/重复实体 | 对接误差、纠偏实际位移、规划/执行错误、请求ID、pick与truth时间线 | 延续比例降速纠偏、准确停靠和恢复预检；失败尝试不是有效货物，但新审计的保守pick/delivery一致性会要求复核，不能仅删日志解决 |
| 接近侧不可达 | allowlist、实际托盘/墙/keepout、入口到dock完整扫掠 | 优先禁用未验证侧或加有依据的局部引导；world/slots/mission/zones/defaults/map/risk/policy/tests必须同步，当前任务不默认改布局 |
| waiting_for_task/服务超时 | 当前task_id、ROS进程归属、Nav2 action真可服务、Gazebo服务与RTF | 只清理该harness拥有的进程组，预检后发布新task；区分环境故障与运动回归，不能把“重跑恢复”写成根因已修 |
| GUI慢/滑行 | 交接速度、停靠TF、实时因子、终端truth | 做一次最终版本可视复跑，保留速度门和有界纠偏，不调低投放/抓取精度阈值 |
| 290motion过但总墙钟超300 | round_id单调起点、出题/API/校验/动作/展示分段 | 关thinking、有界重试、减少可消除的固定等待；10秒只是预留目标，不保证API和展示都在10秒内 |

## 8. 标准报告：表格为主

逐题48行CSV/Markdown至少包含以下列，不适用写未测，不能填预计通过：

| tag | 原题 | 运行commit/patch | 实际抓取→放置序列 | 完成/跳过 | 货物分/20 | requirements_verified | truth_precise | 四移动体在线/连续 | 动态响应对象与类别 | 原始双响应 | 碰撞 | 最小净空m | motion秒 | <290 | API秒 | 完整墙钟秒 | CASE_ACCEPTED | 最终结论 | attempt次数 | 证据目录 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|

另外给汇总表：48题覆盖、首次通过率、最终通过数、重试/碰撞次数、motion最大/p50/p95、净空最小、全墙钟测了几题、未完成项。新规则下order_verified可作历史诊断列，不写“顺序错误”导致失败；实际要求符合才是门。

终态要求：同版本48最新attempt全为5/5、skip0、goods_score20、要求和精确投放正确、四移动体连续、至少两种有真实遭遇和响应的动态体（原始双响应另列并尽量保持）、零碰撞、净空>0.10、CASE_ACCEPTED与functional为true、0<motion<290。反复碰撞的版本即使最后复跑通过，也必须报告次数并优先修复，不能称稳定。

## 9. 收尾

在48全合格之后补一次GUI与真实API/CoStudio代表性完整链演练，报告实测全墙钟；代表题不能证明全部48的全墙钟。如宣称48全比赛达标，须补齐对应全链计时证据。

更新仓库和主机审查报告、逐题CSV、工作日志、最新交接提示词及启动操作手册，明确历史基线与本轮结果、代码提交与文档提交差异。复核diff无密钥与用户改动，提交推送master并核对远端HEAD。不要仅写“下一位继续验证”结束当前已授权的48收敛工作；真实外部阻塞或用户叫停时，记录准确卡点和恢复命令。
