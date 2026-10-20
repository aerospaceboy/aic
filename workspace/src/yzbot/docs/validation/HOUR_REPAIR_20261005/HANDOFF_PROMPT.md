# 下一位 Agent 接手提示词

你接手的是 YZBot ROS 2 Humble 竞赛项目。本轮主线是完成可复现的全分布验收并修小问题，不做大规模重构，不重设计布局。

## 先确认事实，再开始运行

1. 阅读仓库 AGENTS.md、docs/validation/HOUR_REPAIR_20261005/README.md、worklog.md、case-results.csv，以及 docs/competition/competition_rules_2026_summary.md。历史成绩不能代表当前版本。
2. VM 工作空间是 /home/ros/dev_ws，仓库是其中 src/yzbot。复用主机已有 SSH 配置/密钥，勿复制凭据到文档。
3. 核对 git status、分支、HEAD、origin/master。只能提交、推送 master；不要改写 huxuyang 或其他分支。
4. 保留用户未提交的 GUI 刷新和支撑轮 TF 改动，以及旧 stash、失败证据。不要 reset --hard，不要整目录 git add。
5. 查看本轮最终结果表：若定向题仍有安全失败或未完成，先用同一配置带 bag 复现最小问题，修复后过该题及一道已过题，再开始48题。不得因为单元测试通过就直接宣称物理安全。

## 项目及规则口径

系统包括出题/大模型解析、颜色顺序与实体选择、任务状态机、Nav2、RGB-D、MoveIt 抓放、速度仲裁与安全链、四个连续移动障碍、CoStudio/rosbridge 和证据验收。

必须完成五块、零跳过、颜色抓取及放置顺序符合题目，同色实体可优化。物块间距不小于2米，放置区隔墙，原有场景元素不能删减。至少两个不同的有效移动障碍必须真正影响路线并有响应证据；原始两个障碍的响应也单列，不能混淆。

安全验收关注全部四个动态物体：零碰撞、真值最小表面净空超过0.10米、话题和位置新鲜、安全链正常。不能缩小机器人、降低阈值、删除障碍或只放宽统计来通过。

比赛300秒指准备完成后的完整墙钟，包含出题/解题/显示和运动；加载仿真可在准备阶段完成。48题脚本是 motion-only，不能用运动时间冒充完整比赛时间。抓取与放置按件分别计分；项目交付目标仍是安全5/5，不用部分得分掩盖未完成。

## 本轮改动及需要重点看的边界

- 红色物块仅启用 east 接近：red1 west 落入墙体，red2 west 空间狭窄；旧顺序表的 west 项不能重新启用。blue 接近侧继续服从当前 allowlist。cube_order_policy_file 保持禁用，除非重新生成并独立验证符合当前布局与颜色顺序。
- 出生段穿越两个原始巡逻障碍保持 y=0.45 的引导线，增加跨越后清场航点。Nav2仍可能提早到点，必须看实测轨迹与净空。
- 障碍等待超时后，新有效导航命令会清理旧控制状态；同侧等待目标不能被误认为跨越目标。
- 主动等待/受控穿越有自己的有界超时，不再消耗 Nav2 进度恢复；恢复后重置进度基准。
- A区西北墙角使用局部通道引导；东侧去A先避开移动隔板1东角。备用路线仅在既有恢复流程中调用一次，不无限重启，不在持块失败时盲目回出生点。
- 移动隔板扫掠禁区规划 margin 为0.15：0.10验收余量加一格0.05栅格余量；机器人0.28半径仍由膨胀层提供。不得重新把半径叠加成0.38，也不得解释为“必然具有某个净空”；实际执行须验收。
- 抓放 worker 汇报 base_released/cube_attached，底盘释放失败或失败抓取仍挂着物块时协调器直接报失败，不能继续空车流程。释放最多两次、每次5秒；服务端明确确认 joint不存在才视作幂等释放成功。
- 曾出现抓取附着后Gazebo服务/夹爪请求不响应。已修正错误恢复及错误日志，底层间歇不响应根因尚未充分确认。若复现，区分 /clock、gz stats 是否停止，与ROS回调/服务是否阻塞，勿仅归咎布局。
- 当前仍须覆盖原障碍二次遭遇、不同巡逻相位、blue3/4东侧、B东出口、A通道、四蓝去C，以及载荷回波屏蔽和持块失败分支。

## 快速验证流程和额度控制

先运行构建与行为测试，再读本轮最终题表。不要重复已冻结版本的全部检查；新修改才触发相关复测。既有lint错误与新增错误分开报告，不能声称所有检查全绿。

48题保留：仿真小时数不会直接等于模型token；高成本通常来自频繁轮询、反复贴长日志、不断重跑。采用一个后台串行批次，不运行多个gzserver；只在启动、阶段变化、批次收尾时读摘要。日志和bag保存在磁盘，失败只提取必要上下文，禁止把整份bag/长日志送入对话。

在VM执行（先核对脚本路径与参数）：

~~~bash
source /opt/ros/humble/setup.bash
source /home/ros/dev_ws/install/setup.bash
cd /home/ros/dev_ws
colcon build --symlink-install --packages-select mybot tools_demo
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

规则摘要用于定位要求，原PDF冲突时以原PDF为准。具体地图/实体位置以当前world和运行真值为准。

48题启动前确认没有别人的仿真进程。使用新的批次根目录，例如：

~~~bash
cd /home/ros/dev_ws/src/yzbot
campaign="/home/ros/dev_ws/experiments/full48-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$campaign"
RLC_TIMEOUT_SEC=600.0 setsid bash tools_demo/scripts/run48_v2.sh \
  1 48 default "$campaign" >"$campaign/batch.log" 2>&1 < /dev/null &
printf '%s\n' "$!" >"$campaign/owner.pid"
~~~

600秒是诊断上限，不是比赛达标线。脚本有源码版本守卫、断点和残留仿真守卫；批次期间不编辑源码、不构建替换运行包。中断时先记录INTERRUPTED，再停止自己拥有的进程组，等清理结束；不使用可能匹配自身shell的广泛pkill/pgrep杀进程。

脚本正常返回后：

~~~bash
python3 tools_demo/scripts/summarize48_v2.py \
  "$campaign" "$campaign/final-table.csv"
~~~

检查表中每题的最新一次尝试，不能取最好成功覆盖最新失败。缺证据、启动失败、被主动中断、安全失败、运动超300分别分类。修复若改变版本，用新根目录，不能把旧成功计入新版本完成率。

如额度不足以跟进48题，至少完成8题诊断矩阵（A/B/C、两种颜色先后、blue3/4和原失败路线都覆盖）并明确标为“部分验证”；后台48题仍可串行运行并在结束后一次汇总。不能用8题替代48题而宣称全覆盖。

## 修小问题的方法

先查任务状态、指令所有权、base_hold、载荷附着、地图代价和障碍真值；定位后一次改一个主变量。区分“正在合理等待”“真的无路径”“动作/服务超时”“底盘仍固定”。
布局问题只有拿到停靠/出站扫掠包络证据才调整位置。必须同步world、map、zones、mission、导航默认值、选择器、生成器与测试，保留旧布局；不要仅修改一个坐标。
优先修状态生命周期、错误交接、缺失话题和局部引导航点。不要增加无限重试、把失败伪装完成、强行脱离安全链。

## API与CoStudio的独立验收

用户曾明确指定 qwen3.7-flash-2026-07-15，而仓库默认为 qwen3.8-max。先核对部署与用户最后有效指定是否一致，不要悄悄改用仓库默认；若端点不支持，明确报告后再处理。密钥只读私有环境，绝不写入仓库或日志，thinking保持关闭。
至少运行一次真实出题→API解题→数量/目的地/颜色顺序验算→CoStudio展示→5块运行。记录API开始/结束、任务发布、运动开始、第5块投放、最终状态，用同一round_id及单调墙钟。异步旧响应不能覆盖新题。
CoStudio确认WebSocket实际地址、VM端rosbridge监听、Windows端访问、布局话题、TF、map、plan、任务与末端状态；导入成功不等于数据连接成功。浏览器IndexedDB错误先排查本地应用存储，勿误改ROS地图。控制按钮走现有安全链，不能直接发布cmd_vel_safe。

## 必交付结果

逐题CSV包含题号/文本、版本及布局哈希、5/5与跳过、颜色顺序、取放验收、四个障碍各自遭遇及响应、两个有效响应、原始双响应、碰撞次数、每个模型最小净空、运动时间、API时间、完整墙钟、CASE_ACCEPTED、失败原因和证据目录。
另交简短中文报告、工作日志、失败补跑清单和下一步。没有测试的项目写未测；一次通过写单次通过，不能写稳定300秒。
仅按明确文件清单提交master，推送后核对origin/master与本地HEAD。保留用户工作树、失败日志和所有旧证据。
