# 从零接手 YZBot：全分布验证与必要修复

你接手的是 ROS 2 Humble + Gazebo Classic + Nav2 + MoveIt 的机器人竞赛项目。你的任务是：先核实当前交付基线，按同一版本完成全部48题运动验证，再根据失败证据做必要的小修并补跑；最后检查真实 API / CoStudio 比赛链，给出诚实、可复现的交付报告。不要一上来改架构、换布局、提高速度或重做已验证功能。

## 1. 项目如何工作

系统有10个资源物块（5红5蓝）、A/B/C三个投放区、两个原始连续巡逻障碍和两个连续移动隔板。任务要求取放5块。任务文本经解析和校验后交给协调器；协调器按题目颜色序列选择同色实体及允许的接近侧，导航器完成入站/对位/离站/运输，机械臂执行扫描、抓取、投放；验收器核对实际实体投放、顺序、避障响应和安全数据。

不要混淆三类坐标：world物块/zone中心、Nav2入口与停靠点、机械臂作业目标。移动实体后必须同步托盘、地图、配置和代码默认值，不能只改world。

主要入口：
- AGENTS.md：仓库约束；PROJECT_HANDOFF.md：当前状态。
- docs/validation/SPRINT300_20261005/README.md、case-results.csv、worklog.md：本轮事实、失败与证据。
- mybot_description/worlds/offic_room.world：现用物理世界。
- config/complex_layout.json、config/final_mission_layout.yaml：布局元数据及生成结果。
- tools_demo/scripts/activate_complex_layout.py：同步布局的既有工具；修改前读清输出范围。
- tools_demo/config/mission.yaml、bot_navigation/config/competition_zones.yaml：协调器/导航器配置。
- tools_demo/tools_demo/mission_coordinator.py：颜色序列、同色实体选择、抓放闭环。
- tools_demo/tools_demo/task_navigator.py、navigation_math.py：入口、停靠、分段路线、恢复。
- bot_navigation/param/originbot_nav2.yaml：现用 SmacPlanner2D + RotationShim + RPP。
- payload_scan_mask.py、safety_limiter.py、avoidance_monitor.py：载荷激光过滤、安全链、响应证据。
- tools_demo/scripts/gen48_questions.py、run48_v2.sh、run_ordered_case.sh、summarize48_v2.py：现行48题与验收工具。
- docs/competition/competition_rules_2026_summary.md 和工作区官方规则原件：以原件核实规则，不把项目文档当作官方新增规则。
- docs/costudio/、docs/guide/：界面与人工启动说明。历史报告属于历史布局，不是当前通过证明。

## 2. 已知交付状态，必须先核实

本轮基线原提交365c0af，历史目标“3 red→A, 2 blue→B”耗时516.9s。本轮最终目标两次205.4s、190.3s、5/5、零跳过、颜色正确、投放真值 precise、原始双障碍响应、最小动态净空0.385m；最终重复/反序结果以本轮README和CSV为准。不要只引用这一最快成绩宣布48题或完整比赛通过。

目前布局调整：
- red1=(-3.0,3.8)，red3=(0.0,3.8)；
- blue1=(11.8,-2.5)，blue4=(12.0,-6.6)；
- A模型中心=(-5.4,3.8)，东侧导航停靠=(-5.0,3.8,π)。
- red2/red4/red5、blue2/blue3/blue5及B/C不动。
- 固定隔墙、岩石、四个连续移动体没有挪动或删除。
- 最终 world SHA256：da2a9d9377548470243ea4514c9ba22c54ff83e13e241aba3812b639481800c4。
- 旧证书已作废，geometry-audit.json / scope-audit.json 为新布局静态检查。静态连通性不等于所有运动任务已通过。

已修：
1. use_legacy_selection_costs=false时，基础送达评分仍引用旧zone路线锚点；实体和接近侧现在均使用当前目标评分。这仍是距离/转向启发式，不是完整路径最优证明。
2. 新A误用旧西侧A_sweep路线；旧路线现在仅适用于旧A位置。
3. y≈-0.445的巡逻车道整个摆动周期都没有0.60m侧向通行窗口，等待无法解决；特定左上↔右下运输改走(4.4,-2.5)，必要时经(13.4,-2.5)。不是全局关闭动态避障。
4. /global_costmap/costmap是OccupancyGrid：raw254发布为100，raw253为99，unknown为-1。旧>=253探测永远无效；现仅100提前拒绝致命起点，99/71不能直接当“必定卡死”。负边界索引使用floor。
5. 红块走廊出现高频停车/旋转：RPP预瞄min/max/time=.60/1.20/.80；内层rotate-to-heading=false，外层RotationShim保留。速度、footprint、碰撞预测及安全门未放宽。

## 3. 接手前提与不可变要求

- 只提交、推送master；不要推送或修改huxuyang，不要force push。
- 题目颜色/目的地/数量/抓取和投放序列不变；同色实体可优化。
- 不放宽0.10m净空门、不缩footprint、不移除障碍、不关安全链、不修改验收让失败变通过。
- 四个移动体必须仍连续运行；真实减速/等待/绕行证据不能用日志造事件或改计分制造。
- 预存GUI和支撑轮静态TF改动以及stash要保留，不混入你的提交，也不要为了“干净工作区”强制清理。
- 每次试验用新目录，保留失败、中断和旧轮次；最新失败不得被历史最好成绩遮住。
- 密钥只读取仓库外私有环境文件；不打印、不写文档或Git。现用API配置以当前文件及AGENTS为准，不凭历史聊天里的型号覆盖。
- 不并行启动两个Gazebo；只停止自己拥有的进程组，先确认无孤儿节点再开下一题。
- 工程输出明确“运动口径”与“完整比赛墙钟”；600s只是诊断上限，不是比赛达标门。

实际连接：主机工作区为AIC目录，既有SSH配置对应ros用户的Ubuntu22 VM。先读取主机现有连接脚本/配置确定IP与密钥，不猜密码，不要求用户重复提供已有凭据。旧Invoke-VM.ps1可能受PuTTY0.74引号影响；优先用plink -m命令文件或已有稳定包装脚本。不要把凭据写入命令日志。

在VM中（工作区不在默认位置时，先修正WS）：
~~~bash
WS="$HOME/dev_ws"
REPO="$WS/src/yzbot"
cd "$REPO"
git status --short
git branch --show-current
git log -5 --oneline
git rev-parse HEAD
git remote -v
git ls-remote origin refs/heads/master
pgrep -a -x gzserver || true
~~~
读AGENTS和当前报告，核对未提交项与运行归属。先fetch确认远端；工作树有用户改动时不要盲目pull/reset。确认测试和安装树对应交付代码，而非只相信旧报告：
~~~bash
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mybot_description mybot tools_demo bot_navigation
source install/setup.bash
python3 -m pytest -q src/yzbot/mybot/test src/yzbot/tools_demo/test -m "not linter"
~~~
本轮行为测试699 passed, 3 deselected；上述命令有意分开风格门。全目录pytest曾得到2个风格门失败、699 passed、1 skipped，不能写成整个仓库测试全绿。修改文件和新增测试的flake8诊断与旧版比较无新增。若数量不同，说明差异原因，不改测试迎合数字。

### 先修一项已知反向验收缺口，再开全48

最终反向回归2bB3rA：229.3s、5/5、零跳过、投放正确、safe、min0.3593m，但只有obstacle_1响应，CASE_ACCEPTED=false，不能算通过。证据见本轮r5目录。

已定位的链条：初始7段universal展示路线在穿越obstacle_1时被取消；导航器与安全限速器释放时机不同，6s crossing期限内未清出，报“Admitted crossing did not clear the patrol”；协调器随后重试并改接近侧，因机器人已离开出生区，新的动态门判断返回空，直接去blue1，未走剩余obstacle_2展示航点。后续安全运输从下走廊返回，因此不会补齐第二次响应。

最小解决路径：
1. 对齐navigator与safety_limiter的真实准入/释放条件，分析6s中哪些时间仍被安全层零速截断；不得单纯加速、延长穿越后放行或关闭安全门。
2. 为同一任务的首腿保留“尚未完成的展示航点/阶段”，失败或切换接近侧后继续未完成安全路段；不能仅凭当前坐标不在spawn就认为展示已完成。新任务/取消必须正确清理，迟到回调不能恢复旧路线。
3. 这应保留真实遭遇/响应，不能直接置yield事件或改验收。可用轨迹证据确认通过两个巡逻带且均真实减速。
4. 加一个故障注入测试：首个巡逻穿越失败→重试切换侧→第二个展示目标仍在剩余队列；再定向跑2bB3rA和已过3rA2bB。两者功能门均过才启动同版48。
5. 如果根因需要大改，在报告说明证据和最小影响面；不要以全48反复重跑代替这一小范围复现。
## 4. 快速进入全验证，控制时间与额度

先用现行run_ordered_case复跑3rA2bB，必要时补反序2bB3rA或1rA4bB；若交付报告已含这些最终同版本回归，可以只复跑目标作为安装确认。不要重复整套历史研究。

确认五块、顺序、投放真值、安全、双响应达标后冻结版本，完整48一次串行后台跑。48次仿真主要消耗机器时间；无需让模型逐帧读取日志，额度主要花在读长日志和反复推理。不要因为有48题就降为少量题宣称全覆盖。日常只读终态摘要、汇总CSV和失败窗口；bag仅给定向失败复现录制。

~~~bash
cd "$REPO"
ROOT="$WS/experiments/full48-sprint300-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$ROOT"
export RLC_TIMEOUT_SEC=600.0
setsid bash tools_demo/scripts/run48_v2.sh 1 48 ordered "$ROOT" > "$ROOT/batch.log" 2>&1 < /dev/null &
echo "$!" > "$ROOT/batch.pid"
printf '%s\n' "$ROOT"
~~~

启动后确认RUN及首题真实受理，不把“进程已启动”当作有效验证。运行期间不改任何受版本哈希覆盖的文件，包括已跟踪文档；进度写到实验目录，不提交、不重建、不插入另一个run。若发现阻断整个矩阵的确定性错误，停止自己的批次，保留中断原因，小修后以新版本/新目录重启，不混合冒充同版48。

~~~bash
python3 "$REPO/tools_demo/scripts/summarize48_v2.py" "$ROOT" "$ROOT/final_table.csv"
~~~
COMPLETED只表示功能验收，不保证<300s。汇总器retry清单主要筛功能失败；还要另提所有>=300s题。必须报告每题最新有效终态与历史尝试数，不拿旧成功覆盖新失败，不拿缺证据当通过。

## 5. 明确失败路径：先看证据，再做最小修复

A. Nav2一直零速/原地旋转：
- 对比cmd_vel_nav、selected、cmd_vel_safe、odom，确定零速来自控制器还是安全层。
- 记录自身格/局部层代价、实际scan/载荷状态、plan和入口姿态。
- cost100：追查占据源、footprint清除和载荷过滤；不能盲清整张地图或扩大屏蔽框。
- cost71/99：只结合真实停滞、collision-ahead、路径曲率判断；不要仅按cost>=50硬倒车。已有有界恢复需看后方净空、动作所有权、恢复次数。
- 若再次高频转向，检查RPP/RotationShim阈值与路径曲率，先用已知版本回归；不要直接提高巡航速度。

B. 通道等待循环：
- 把机器人等待点、四障碍全周期包络、车体和安全余量画在同一坐标系。
- 若整个周期都不存在安全窗口，换静态连通的走廊/入口，不继续8s等待重试。
- 若只相位不合，安全等待点必须在完整扫掠带外，并核对穿越所需时间而非只看瞬时中心距。
- 计分后安全保护仍要持续，不能“已经yield一次”就解除防撞。
- 停在带内即使速度0也可能被撞；检查等待点和释放/再次接近逻辑，不能把零指令当安全证明。

C. red west卡死或两路指令打架：
- 当前red实体allowlist是east-only，生成后的canonical布局必须保持这一限制。
- 若旧cube_order_policy或手动命令仍强选west，修策略/侧选择与运行时allowlist一致，不先改物理安全限制。
- 用ros2 topic info -v和状态机确认导航、停靠、恢复的指令所有权；交接时取消动作、等待确认，再让另一控制器发指令。
- 不沿用历史坐标下的west停靠“已验证”标签。

D. 载荷假障碍与机械臂首用/附着失败：
- 检查payload_scan_mask的当前任务/实际持块状态、TRANSIENT_LOCAL稀疏发布语义，不能用2s状态TTL随意解除载荷屏蔽。
- 只屏蔽已知自身载荷区域，真实墙/移动体/已放置物块仍进入安全链；不扩大range_min来隐藏现场障碍。
- MoveIt -4、实体服务超时、附着不稳要区分；核对controller就绪、服务延迟、真实attached状态。先做有界预检/重试，禁止已经持块又重复抓取或释放失败却记投放成功。

E. 新布局导致C题/4蓝题超时：
- 分开导航、入站/离站、10次机械臂、等待和恢复耗时，找最大腿。
- 优先现有入口、同色实体选择和墙角航点小修；当前评分仍是启发式，C族可能需要真实静态可达路径代价。
- 必须考虑连续四次投放的空间占用和返回方向，不能只优化首腿。
- 只有当前布局确定无法满足停车/连通/预算时才再动布局；同步world模型+保存state、托盘、layout元数据、final layout、mission/zones、默认字典、地图/risk、合同测试、旧证书和备份。重新检查10块间距、zone隔墙、至少一侧可停车、完整扫掠包络。改变布局后旧全48不能沿用。

F. harness启动竞态/STALE/无终态：
- 查看就绪门、DDS参与者、唯一launch与节点、runner环境；这不自动证明机器人运动故障。
- 只清理己方进程，保留INTERRUPTED/失败状态。
- 600.0参数必须double；不把启动过程耗时混进纯运动的600s截断解释。
- 复跑使用新目录且同源版本。中途改代码后停止旧版批次，不继续采集混合结果。

每次修复只围绕一个已证实故障，先定向复现/对比，再跑受影响题和一个原本稳定题。真正故障来自接口/状态交接时，修接口；不把全局恢复程序当作所有问题的替代。

## 6. 完整比赛链、CoStudio和API

运动矩阵完成后，用当前手动/出题入口开展至少一轮真实比赛链。依据docs/costudio和现用launch确认rosbridge WebSocket地址、ROS_DOMAIN_ID、话题消息类型、面板配置与发布权限；界面能打开不等于订阅有数据，也不等于开始按钮触发正确任务。

API私有环境复用现有配置；thinking=false。核对一次请求对应一次round_id，迟到响应不能覆盖新任务，输出颜色/数量/zone/算术一致性须验算。不为节省时间把真实API测量替换成直接发布运动题。

记录题目受理、API开始/结束、解析校验完成、首导航、每次抓/放、第五块验收、最终终态的单调时钟。当前运动脚本不包含出题/API/界面时间，因此即使48题运动都<300，也要单列完整比赛墙钟是否<300。若尚无这些字段，最小实现计时后验证，不根据历史API延迟估算宣称通过。

人工发速度命令必须走当前文档确认的安全输入，不直接发/cmd_vel_safe绕过链路。API密钥、私有凭据不要出现在截图、终端录屏或仓库中。

## 7. 报告、备份与推送

每题至少记录：题目原文、解析/颜色序列、实体和接近侧、源码版本+差异指纹、5/5与skipped、实际投放truth/precise、运动耗时、完整墙钟是否测量、最小净空及对应障碍、碰撞、原始两障碍各自响应、新增隔板响应、CASE_ACCEPTED、恢复次数、失败阶段/证据路径。不要把“四个被监测”写成“四个都yield”。

最后交付README审查报告、48行CSV、失败/超时待办、关键证据、完整worklog、从零接手提示词。保留本轮所有失败及旧布局备份，更新PROJECT_HANDOFF和AGENT_HISTORY，不覆盖历史成绩。

提交前检查diff/密钥/无关文件；只提交自己的改动。提交和构建不在运行批次中进行。推送origin master并核对远端HEAD：
~~~bash
git push origin HEAD:master
git rev-parse HEAD
git ls-remote origin refs/heads/master
~~~
不得强推或改huxuyang。回退只能回退本轮自己的提交/整套布局资产，保留用户GUI/TF等改动，不能用git reset --hard清空工作树。

最终目标是同版48题的完整、诚实成绩及真实比赛链验证。优先完成验证和小修；若仍有失败，给出准确定位、未完成清单和下一条可执行命令，不能把“根因猜测”“某次最快”写成稳定完成。
