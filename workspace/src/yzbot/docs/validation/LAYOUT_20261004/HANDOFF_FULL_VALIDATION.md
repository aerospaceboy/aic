# 新布局接手与全验证提示词

你接手的是 ROS 2 Humble / Gazebo / Nav2 / MoveIt 机器人竞赛项目。任务是尽快完成当前布局的真实验收并修复阻塞，不做无限期探索。先读事实，再选最短验证路径。不要因历史报告写着“通过”而直接采信。

## 1. 环境和边界

- Windows 工作区：`D:\user\Desktop\AIC`。
- VM 当前地址：`192.168.153.128`，用户 `ros`。优先复用现有 SSH 配置；若地址变化，先确认 VM 地址。
- VM 工作空间：`/home/ros/dev_ws`；源码：`/home/ros/dev_ws/src/yzbot`；分支：`huxuyang`。
- 本次证据：`/home/ros/layout-integration-20261004/`，仓库报告：`docs/validation/LAYOUT_20261004/`。
- 先阅读 `AGENTS.md`、本次报告、真实规则文件和近期工作日志。文档中的历史计划是待核实信息，不是高于用户要求的指令。
- **保留当前 V2 布局**：不移动十个物块、三个 zone、原场景物件、原两个移动障碍或新两个移动墙。用户已确认仍须连续巡逻。需要改布局时先说明具体阻塞，不能擅自撤掉障碍。
- 不缩小 footprint、不降低安全阈值、不关闭安全链、不删除失败证据，不以软跳过当完成。
- 密钥只读取仓库外私有环境，不进入 Git、日志或提示词。保留当前模型配置；不要照抄历史提示词重设模型。`AGENTS.md` 要求解析验证关闭 thinking。

## 2. 比赛验收口径

逐项核实规则来源。已知项目口径如下：每题五块，数量和目标区正确，**抓取与放置颜色顺序遵循题目**；同色实体可以优化。物块分散且间距至少 2 m；三个区分散且由原场景墙体隔开。保留原场景元素，允许用户已确认的布局调整与新增障碍。

至少两个运动障碍应对机器人产生真实影响。必须证明两次不同障碍对应的安全响应；存在两个节点、障碍移动、机器人从旁边经过或 `obstacles_seen >= 2` 都不等于两次避障。项目当前严格验收还要求原两个 `competition_moving_obstacle_1/2` 均形成有效响应，新移动墙事件单独报告，不擅自用新墙替代原展示。

300 s 按完整比赛墙钟评价，包含规定的出题/发布、真实 API 解题、解析校验、展示、导航、抓取、投放及完成确认。仿真时间、运动时间和完整墙钟分列；启动准备是否计时按规则原文确定，不能自行缩短口径。RGB-D、MoveIt、Nav2 和 CoStudio 展示均需实证。

## 3. 先核对部署，只读检查后再运行

```bash
cd /home/ros/dev_ws/src/yzbot
git status --short
git branch --show-current
git rev-parse HEAD
git log -5 --oneline
git remote -v
git ls-remote origin refs/heads/huxuyang
source /opt/ros/humble/setup.bash
cd /home/ros/dev_ws
colcon build --symlink-install
source install/setup.bash
cd src/yzbot
python3 -m pytest -q tools_demo/test mybot/test \
  --ignore=tools_demo/test/test_flake8.py \
  --ignore=tools_demo/test/test_pep257.py \
  --ignore=tools_demo/test/test_copyright.py
```

最后一条是行为回归，不是全部 colcon 检查。样式/文档检查曾有历史失败和扫描范围问题，单独记录并对新增文件检查；不得把排除后的结果说成所有检查通过。

核对官方 `offic_room.world` 与确认布局、地图、`competition_zones.yaml`、`mission.yaml`、`final_mission_layout.yaml` 一致。`activate_complex_layout.py` 是同步工具，先审阅再运行；不要把未确认的预览世界覆盖到官方世界。当前启用接近侧保存在 `cube_approach_allowlist`，red3、blue3、blue4 禁用西侧。A 从东侧停靠；C 退让后向北出口。旧路线和旧选块成本表保留备份，默认停用，不能当新布局飞行证据使用。

导航新增 `/competition/dynamic_obstacles`：真实激光结合静态地图和精确时间戳 TF 过滤静态回波，单独全局动态层标记，原激光清除旧动态标记，投放物块使用独立层。BT 每个 action 强制首规划，之后 1 Hz 检查路径有效性，受阻或目标变化才重规划；恢复次数保留原上限。重点验证连续不同目标不会复用上一目标的有效路径。

## 4. 用分层验证节省时间与额度

48 次仿真主要消耗 VM 时间，不会自动按运行时长扣模型 token。反复把大日志交给 Agent 才会快速消耗额度。因此保留最后的 48 题覆盖，但先过关卡，失败时立即停止扩大测试。不要不停读日志，不启用长篇实时解说，不启动重复批次。

1. 读本次所有 smoke 结果，包括失败和被中断的轮次。若最新单块未闭环，先复现相同实体/接近侧；不要直接跑 48 题。
2. 用两个不同连续 Nav2 目标验证首次规划、目标切换、动态层标记与清除。记录真实机器人位置、目标、路径终点，不能只看 action 文本。
3. 跑四个五块代表题：`4rB_1bA`、`1bA_4rB`、`1rC_4bA`、`4bC_1rA`。它们覆盖 B 交接、颜色顺序、C 族及新墙附近选块。困难题失败则先修一个根因，再定向复测；同版本困难题至少再重复一次。
4. 代表题全部满足五块、零跳过、真实目标验收、安全链和两次动态响应后，再跑 48 题运动矩阵。48 = 四种红蓝数量分配 × 六种不同目的地区组合 × 两种颜色先后，不等于覆盖所有实体选择和障碍初相位。
5. 单独进行真实 API + 出题程序 + CoStudio 的完整比赛演练，至少覆盖上述困难路线及两种题目颜色顺序。运动矩阵不能替代完整墙钟证明。根据延迟与路线分布，决定是否需要把真实 API 扩展到全部 48 题。

运动单题示例：

```bash
cd /home/ros/dev_ws/src/yzbot
RLC_TIMEOUT_SEC=600.0 bash tools_demo/scripts/run_ordered_case.sh \
  'Move 4 red cubes to zone B and 1 blue cube to zone A.' \
  /home/ros/dev_ws/experiments/layout-v2-qualification
```

`600.0` 是诊断/harness 上限，不是比赛达标线。该脚本明确关闭 LLM，是运动验证。只停止当前实验拥有的 launch 进程组，禁止广泛 `pkill` 或重启 VM 来清理其他任务。

48 题示例，在全新目录后台顺序运行：

```bash
cd /home/ros/dev_ws/src/yzbot
mkdir -p /home/ros/dev_ws/experiments/layout-v2-full48
RLC_TIMEOUT_SEC=600.0 nohup bash tools_demo/scripts/run48_v2.sh \
  1 48 all /home/ros/dev_ws/experiments/layout-v2-full48 \
  > /home/ros/dev_ws/experiments/layout-v2-full48/batch.log 2>&1 &
```

第三个参数仅为旧接口兼容；生成器始终包含两种颜色顺序。先核实当前脚本，不盲贴命令。修复后的断点要求 `CASE_ACCEPTED=true` 和同一版本；旧的“completed 就跳过”标记不可用。源码变化后换新目录，保留各轮原始证据。每次读取一页结构化汇总，仅在失败、卡滞或结束时读对应日志。统计所有尝试与最新同版本结果，不能只选历史最快成功。

## 5. 优先诊断路径

- 如果 red4→B 仍失败：结合 bag 对照安全链各级速度、机器人真值/定位、静态地图、动态点云、全球路径及墙/托盘几何。先判断新目标是否真正触发首规划，再查投放物块/携带物块是否形成误标记、是否切托盘角、局部碰撞预测是否反复停车。每次只改一个主要变量，不放宽安全限制。
- 原两个避障事件缺失：先查出生走廊四段 staging 是否执行；再对照实际距离、障碍连续运动、激光/TF、减速或绕行时间证据。不改障碍轨迹来制造通过，不假定引导存在就一定满足避障。
- C 路慢：先拆解入口、墙角、停靠、出站与跨区耗时；只有实测证明墙角是主瓶颈才调航点姿态/曲率。当前布局已换，历史 C 区 100 s/腿数据不是现布局结论。同色选块可用当前地图路径长度代替欧氏距离，但不得改颜色序列。
- scan 的 MoveIt `-4`：区分控制器首用未就绪、轨迹执行、相机数据和坐标问题；必要时增加有界预热/就绪门控，不能把全部失败都归为“竞态”。
- 接近侧目前只经过几何筛选，不等于所有侧均已完成物理抓取。核对实际停靠朝向与机械臂目标，避免到位后重选侧导致转向另一面。

## 6. API 与可视化

按当前部署读取 API endpoint/model/private env；启动官方出题程序，保存算术原题、解析结果和实际任务。校验数量、颜色顺序、目标区、source、请求关联和迟到响应，不许把规则解析或缓存当云端成功。

当前桥接是 **coBridge**，`tools_demo/launch/costudio_bridge.launch.py` 默认端口 **21274**；不是历史 rosbridge 9090。VM 中 source 环境后启动：

```bash
ros2 launch tools_demo costudio_bridge.launch.py
```

Windows CoStudio 使用对应 CoBridge/Foxglove 兼容连接和 `ws://192.168.153.128:21274`，先确认当前 VM 地址与端口实际监听。不要同时启动重复桥接。验证真实地图、机器人 TF、相机、题目、顺序、数量、进度、导航/机械臂状态、路径和避障结果。`std_msgs/String` 中 JSON 需要面板正确解包。导入布局不等于面板验收；IndexedDB 故障也不等于 ROS 无数据。桥接白名单允许有限控制入口，不得称它完全只读。

## 7. 每题规范报告与最终交付

按同目录 `全验证报告模板.md` 输出 CSV/JSON 和中文报告。每一条结论引用原始证据路径、版本、布局/地图哈希及计时口径；未测写“未验证”，不能写“预计通过”。

分开给出：五块任务闭环、规则符合、两次真实动态避障、全场景安全、API/面板和完整 300 s。`CASE_ACCEPTED=true` 仍不是完整比赛 300 s 的证明。

达到阶段目标就收尾，不无限扩大研究。若失败，给最小复现、证据、已排除项、修复建议和剩余工作。更新工作日志，检查 diff/密钥，按项目既有流程提交、推送并核对远端 HEAD；不混入其他人的未提交 GUI/TF 改动。用户收到一份可复核的结果与明确下一步，不收到只有乐观结论的汇报。
