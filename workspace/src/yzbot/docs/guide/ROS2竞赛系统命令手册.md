# ROS 2 竞赛系统操作手册

更新：2026-09-06。面向实际操作者：从启动 VM 到出题、API 解题、机器人执行、可视化、调试和停止。

本文依据当前源码、安装包 launch 参数和官方出题程序核对。核查时源码 HEAD 为 `379af2c`，选块策略表有既有未提交改动。本文不代表新一轮运动或 300 秒验收通过。

**最常用流程：第 1 节准备终端 → 第 2 节启动等待 → 第 3 节出题并提交 → 第 4 节观察 → 第 9 节停止。** 要自己开车看第 6 节；只测试 API 看第 7 节；CoStudio 看第 5 节。

## 0. 先分清几种操作

| 想做什么 | 用哪个入口 | 是否调用 API | 会不会让机器人动 |
| --- | --- | --- | --- |
| 官方出题程序生成一道应用题 | `~/dev_ws/TMSCQtest_x86_x64.bin` | 否，只输出文字 | 不会 |
| 应用题 + 映射规则 → 解题并搬运 | 第 2 节云模式 + 网页提交 | 是 | 解析通过后自动动 |
| 自己指定“几红去 A、几蓝去 B” | 第 3.4 节 `/competition/task/question` | 取决于启动模式 | 解析通过后自动动 |
| 随机生成直接搬运指令 | ROS 的 `task_question` | 取决于启动模式 | 接入整套系统后会动 |
| 单题运动基准 | `run_ordered_case.sh` | 否 | 会，且自动启动/停止仿真 |
| 只验证 API，不启动机器人 | 第 7 节独立解析模式 | 是 | 不会启动机器人 |
| 自己开车或去某个区域 | 第 6 节调试模式 | 否 | 会 |
| 观察网页/CoStudio | 第 4、5 节 | 观察本身不调用 | 观察不会，提交按钮会 |

注意：`gen48_questions.py` 是 48 种直接搬运指令清单，不是官方应用题程序。`clean_launch.sh` 会先清理再启动一套**关闭 API**的系统，不是“只停止”工具。

同一时刻只运行一套仿真。每轮完成后物块已经被移动；开始独立新比赛应重启整套仿真恢复场景，不能只再次发送题目。

## 1. 开机和每个终端的准备

### 1.1 在哪里输入命令

除特别标注“Windows PowerShell”，本文代码块都在 **Ubuntu VM 的终端（Bash）**执行。桌面按 `Ctrl+Alt+T` 打开终端。终端 A 留给主系统，终端 B 用于发题和检查，终端 C 可运行桥接或记录。

项目源码在 `~/dev_ws/src/yzbot`；已经构建的环境在 `~/dev_ws/install`。日常运行不需要每次重新编译。

每打开一个终端，先执行下面整段：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/dev_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/dev_ws/install/tools_demo/share/tools_demo/config/fastdds_udp.xml"
cd "$HOME/dev_ws/src/yzbot"
```

本项目 DDS 配置限制到 VM 本机回环地址。Windows 通过网页/WebSocket 连接 VM，不要求 Windows 直接发现 ROS 节点。VM 里的 CLI、桥接和主系统需要使用相同域及上述配置。

### 1.2 启动前看看有没有旧系统

```bash
pgrep -a -x gzserver
pgrep -a -x gzclient
ps -eo pid,ppid,args | grep -E '[r]os2 launch tools_demo competition_system|[c]ompetition_mission_coordinator|[c]ompetition_task_navigator'
```

有旧系统时先按第 9 节停止它。不要叠加启动。没有 Gazebo 也不代表没有残留任务节点。

### 1.3 核对 IP 和私有配置

```bash
hostname -I
test -r "$HOME/.config/yzbot/llm.env" && echo '私有 API 环境文件可读取'
```

核查时 VM IP 为 `192.168.153.128`，以后以实际输出为准。第二条只确认文件可读，不代表密钥有效。不要用 `cat`、截图或日志展示密钥内容。

## 2. 日常推荐：启动云 API 模式，等待你提交题目

### 2.1 终端 A：启动主系统

完成第 1.1 节后执行：

```bash
YZBOT_RUN_DIR="$HOME/dev_ws/experiments/manual/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$YZBOT_RUN_DIR"
printf '%s\n' "$YZBOT_RUN_DIR" > "$HOME/dev_ws/experiments/manual/latest-run.txt"
git rev-parse HEAD > "$YZBOT_RUN_DIR/commit.txt"
git status --short > "$YZBOT_RUN_DIR/worktree-status.txt"
git diff HEAD --binary > "$YZBOT_RUN_DIR/worktree.patch"
set -o pipefail
bash tools_demo/scripts/run_competition_api.sh \
  headless:=False generate_question:=False dashboard:=True \
  moving_obstacle:=True \
  2>&1 | tee "$YZBOT_RUN_DIR/system.log"
```

这个终端会持续输出日志，留着不要关闭。`Ctrl+C` 是停止主系统，不是切换窗口。

- `headless:=False` 打开 Gazebo 窗口。希望降低 VM 图形负载时，在下次启动改为 `headless:=True`；此时仍有仿真及相机，Ubuntu 图形会话也应保持可用。
- `generate_question:=False` 防止启动后自动发布随机搬运题。推荐保持这个值，等你自己提交官方应用题。
- 脚本读取私有环境，关闭本地 llama 服务启动选项，启用 `use_llm=True` 和 `require_llm=True`。
- **不要再额外启动 `competition_system.launch.py`、`mission.launch.py`、MoveIt 或第二套 Nav2。主系统已包含这些模块。**

### 2.2 等到真正就绪，再出题

系统分阶段启动：MoveIt 约在 15 秒，Nav2 约在 45 秒，任务模块约在 65 秒，启动检查约在 70 秒开始。这些是延迟配置，不能当成必然就绪时间。

终端 B 完成第 1.1 节后检查：

```bash
ros2 node list
ros2 lifecycle get /collision_monitor
ros2 action list -t
ros2 control list_controllers
ros2 topic info /competition/task/problem -v
ros2 topic echo /competition/mission/state --once --qos-durability transient_local
```

应看到：

- `collision_monitor` 为 `active`；不要通过反复 deactivate/activate 尝试修复它。
- 导航 action `/navigate_to_pose` 存在，任务导航器和协调器各一套。
- 机械臂/夹爪对应控制器已激活；主日志出现 `persistent pick-place serving requests`。
- 任务状态为等待/空闲，没有已有任务正在执行；`/competition/task/problem` 有解析器订阅。

进一步确认关键传感器：

```bash
timeout 6 ros2 topic hz /scan
timeout 6 ros2 topic hz /camera_rgbd/camera_rgbd/image_raw
timeout 6 ros2 topic hz /joint_states
```

`timeout` 到时返回 124 是正常结束观察，不等于传感器失败。没有频率输出时先检查话题、图形渲染和 DDS。此时不要反复发题。

### 2.3 核对实际模型

```bash
ros2 param get /competition_task_parser model
ros2 param get /competition_task_parser use_llm
ros2 param get /competition_task_parser require_llm
ros2 param get /competition_task_parser llm_enable_thinking
```

目标模型是 `qwen3.8-max`，后三项应为 `True`。当前真实应用题有约 21–23 秒的历史解题记录，不能保证每次相同。

API 端点和超时配置在 `tools_demo/config/task_parser.yaml`；密钥值在仓库外 `~/.config/yzbot/llm.env`，环境变量名为 `DASHSCOPE_API_KEY`。修改这些客户端参数后需要重启解析器/系统，不能假定 `ros2 param set` 会重建已创建的客户端。

## 3. 出题、API 解析和正式下发

### 3.1 官方出题程序在哪里

VM 已有三个平台版本，Ubuntu VM 使用 x86_64 版本：

```bash
cd "$HOME/dev_ws"
./TMSCQtest_x86_x64.bin
```

程序每次输出一道应用题然后退出，官方练习说明表示测试程序只有两类示例题。它**不会自动连接 ROS、不会自动调用 API，也不会自动开始搬运**。

若文件失去执行权限，先执行 `chmod u+x "$HOME/dev_ws/TMSCQtest_x86_x64.bin"`。`TMSCQtest_arm.bin` 给 ARM 平台用；`TMSCQtest.exe` 在 Windows 运行，不能在 VM 当 Linux 程序执行。

Windows 原始压缩包位于 AIC 下“全球校园人工智能算法精英大赛 大模型技术创新赛 比赛练习平台/2026/题目生成器.zip”。解压后在 Windows PowerShell 中进入实际解压目录，运行 `./TMSCQtest.exe`。原始使用说明是同目录的《出题程序使用说明.pdf》。

### 3.2 最容易操作：网页粘贴题目 + 映射规则

浏览器打开：

- VM 内：`http://127.0.0.1:8080`
- Windows：`http://192.168.153.128:8080`，IP 按第 1.3 节实际值替换。

依次操作：

1. 把官方出题程序输出的完整题目复制到题目输入框。
2. 把本轮规定的映射规则填入映射框。**网页有默认示例，必须确认它与本轮规则一致。**
3. 确认机器人空闲、场景为新一轮初始状态。
4. 点击一次“解析并下发任务”。此按钮不是只解析预览：解析通过后协调器会自动执行。
5. 观察 LLM 状态，再观察计划、顺序和机器人。不要因为模型还没返回而连点。

映射规则示例：

```text
x代表红色,y代表蓝色,数量大于等于3的去A区,其他颜色去C区
```

衣柜示例的校验预期是 x=4、y=1，对应先 4 红去 A，再 1 蓝去 C。这只是检查示例，不代表所有题目都用这个答案或映射。x 对应颜色在前，y 对应颜色在后；同色实体可由策略优化。

网页显示提交成功或 HTTP 返回 `accepted`，只说明服务器接收了请求。还应看到 `/competition/task/llm_status` 中 `available=true`、`source=llm`、`detail=validated_model_problem_solution`，以及新的合法 plan 和任务状态，才能确认模型解题通过。

当前网页确实提供 `POST /api/task`，不是纯只读系统。它没有替你完成所有忙碌检查；运行中不提交新题，否则可能产生额外 API 请求、拒绝执行或展示错轮。

### 3.3 命令行一键衔接：官方出题 → HTTP 提交

前提：第 2 节主系统已就绪，网页服务已开，机器人空闲。下面脚本会生成一道题，显示题目和映射，输入“提交”后才发送。

终端 B：

```bash
python3 - <<'PY'
import json
import pathlib
import subprocess
import urllib.request
import uuid

program = pathlib.Path.home() / 'dev_ws/TMSCQtest_x86_x64.bin'
question = subprocess.check_output([str(program)], text=True, timeout=10).strip()
rules = input('粘贴本轮完整映射规则：').strip()
if not rules:
    raise SystemExit('没有映射规则，未提交。')
task_id = str(uuid.uuid4())
payload = {'question': question, 'mapping_rules': rules, 'task_id': task_id}
print(json.dumps(payload, ensure_ascii=False, indent=2))
if input('解析成功后机器人将开始搬运；输入 提交 继续：').strip() != '提交':
    raise SystemExit('未提交。')
folder = pathlib.Path.home() / 'dev_ws/experiments/manual-submissions' / task_id
folder.mkdir(parents=True, exist_ok=False)
(folder / 'request.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
request = urllib.request.Request(
    'http://127.0.0.1:8080/api/task',
    data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
    headers={'Content-Type': 'application/json'}, method='POST')
with urllib.request.urlopen(request, timeout=10) as response:
    print(response.read().decode('utf-8'))
print('请求已保存到', folder)
print('接收成功不等于解题成功，请继续查看 LLM、plan 和 mission 状态。')
PY
```

这里的 HTTP 请求发给 VM 网页服务；真正的云 API 请求由解析器完成。脚本不接触或输出密钥。HTTP 超时后先看状态，不要立即重复提交。

也可以把题目和规则写成 JSON 文件，再通过下面形式发送：

```bash
curl --fail-with-body -sS -X POST http://127.0.0.1:8080/api/task \
  -H 'Content-Type: application/json' --data-binary @"$HOME/problem.json"
```

`~/problem.json` 需预先创建，内容含 `question`、`mapping_rules` 和本轮唯一 `task_id`；不要把密钥放进这个文件。复杂中文/引号用 JSON 文件或上面的 Python，避免手拼 shell 转义。

### 3.4 自己指定搬运数量、颜色和区域

这条入口用于直接搬运指令，不带官方应用题的算术解题过程。云模式中仍调用模型解析；第 6 节练习模式中使用规则解析。

先确保第 2.2 节解析器已就绪，再执行：

```bash
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /competition/task/question std_msgs/msg/String \
  "{data: 'Move 4 red cubes to zone B and 1 blue cube to zone A.'}"
```

中文写法：

```bash
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /competition/task/question std_msgs/msg/String \
  "{data: '请先将1个蓝色方块送到B区，再将4个红色方块送到A区。'}"
```

只选一条发。总数按比赛要求为 5。官方衣柜/宠物应用题要走第 3.2/3.3 节的 `problem` 入口并带映射规则，不能只扔到 `question`。

短生命周期的 `--once` 发布器退出后不会永久保存题目，所以要先确认订阅者就绪。需要稳定的晚加入展示，优先使用网页 problem 提交或保持出题节点运行。

### 3.5 随机直接搬运题

同样先启动等待模式，终端 B 执行：

```bash
ros2 run tools_demo task_question --ros-args \
  -p use_sim_time:=True -p random_seed:=42 \
  -p publish_delay_sec:=2.0 -p period_sec:=0.0
```

它只发一题，进程会继续运行以保留消息；`Ctrl+C` 停止出题节点，不会取消已交给机器人的任务。固定 seed 用于复现，改成 `-1` 随机。不要设置连续出题周期来压测正在搬运的机器人。

直接题目有去重逻辑，重复原文可能不会再执行；网页相同题目和规则也可能被去重。重复一轮最清楚的办法是停止旧发布器、重启整个场景后再提交。只修改 task_id 不会复原被搬走的物块。

## 4. 网页、状态和怎样判断成功

### 4.1 快速观察命令

以下命令一条占用一个终端，`Ctrl+C` 结束观察：

```bash
ros2 topic echo /competition/task/llm_status --qos-durability transient_local
ros2 topic echo /competition/task/plan --qos-durability transient_local
ros2 topic echo /competition/task/order --qos-durability transient_local
ros2 topic echo /competition/mission/state --qos-durability transient_local
ros2 topic echo /competition/navigation/state --qos-durability transient_local
ros2 topic echo /competition/mission/error
ros2 topic echo /competition/avoidance/state
ros2 topic echo /competition/gripper/state
```

只看当前保留消息时加 `--once`。JSON 是 `std_msgs/String.data` 的字符串内容，不是 ROS 消息原生嵌套字段。

### 4.2 关键字段的含义

| 字段 | 怎么理解 |
| --- | --- |
| `source=llm` | 当前计划来自通过校验的模型输出；练习模式的 `rules` 不能当 API 验收 |
| `completed` | 已完成投放的数量 |
| `current_index` | 当前逻辑任务进度，不等于已经完成几个 |
| `cube_name` | 实际选择实体；不能仅从 detail 的编号推断实体 |
| `skipped` | 跳过数；大于 0 不能称五块零跳过完成 |
| `state=completed` | 任务状态机完成，还需核对数量、顺序、安全和精度 |
| `elapsed_sec` | 任务阶段时间；不是官方全比赛连续墙钟 |
| `CASE_ACCEPTED` / `audit.json` | 自动基准的验收证据；普通手工启动不一定产生这些文件 |

当前配置的 `mission_timeout_sec` 为诊断用 700 秒，单题 runner 默认 480 秒。它们都不是比赛允许时长。300 秒是否达标要用比赛实际起止事件、API、展示和全部运动的真实总墙钟确认。

### 4.3 网页连不上

VM 先检查：

```bash
curl -fsS http://127.0.0.1:8080/healthz
ss -ltnp | grep ':8080'
```

预期返回 `ok`。VM 可访问而 Windows 不行时，检查实际 IP、VM 网络和端口，不要先重装 ROS。

Windows PowerShell：

```powershell
Test-NetConnection 192.168.153.128 -Port 8080
```

网页浏览器刷新不等于重新启动机器人。网页默认映射不会自动变成老师当轮的映射。

## 5. CoStudio 可视化：协议和步骤分开

核查状态：VM 安装了 rosbridge，`cobridge` 包未找到；Windows CoStudio 具体版本和完整布局尚未在本轮验收。下面是明确的操作路径，不把“写了文档”当成面板已配置完成。

### 5.1 推荐的原生 coBridge 路径

官方连接方式是 CoStudio 的 **coScene Websocket**，配合 VM 的 **coBridge**，默认端口 **21274**。官方步骤见：[通过 coBridge 连接机器](https://docs.coscene.cn/docs/client/connect-by-cobridge/)。

首次准备：

1. Windows 从 [CoStudio 官方下载入口](https://www.coscene.cn/download) 获取适合系统的安装包；若页面地址调整，从官网进入下载页。
2. VM 按 [官方 APT 安装说明](https://docs.coscene.cn/docs/client/apt-source-install/) 添加软件源及签名。不要在计时仿真中安装。
3. 添加源后，在 VM 执行：

```bash
sudo apt update
sudo apt install ros-humble-cobridge
```

终端 C 完成第 1.1 节后执行：

```bash
ros2 pkg prefix cobridge
ros2 launch cobridge cobridge_launch.xml
```

第一条仍报 `Package not found` 就先处理安装，不要继续猜连接地址。连接时保持此终端运行。

Windows 打开 CoStudio → 打开连接 → coScene Websocket → 输入：

```text
ws://192.168.153.128:21274
```

VM 内的 CoStudio 可用 `ws://127.0.0.1:21274`。Windows 的 localhost 指 Windows 自己，不能用来代替 VM IP。

### 5.2 已装 rosbridge 的备用路径

**仅当 CoStudio 当前版本有明确的 rosbridge 连接选项时使用。** 不要把 rosbridge 填进 coScene Websocket 连接器。

终端 C 完成第 1.1 节后执行已安装的标准入口：

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090
```

对应客户端的 **rosbridge** 连接地址是：

```text
ws://192.168.153.128:9090
```

仓库另有 `tools_demo/launch/costudio_bridge.launch.py`，但它实质上是 rosbridge 封装，并不启动 coBridge。文件名不能证明协议兼容。旧说明中的 `http://<IP>:9090` 应改为 WebSocket 地址；也不要把 8080 网页地址当作桥接地址。

如使用仓库封装，先确认本地版本的端口参数类型再运行；日常优先上面的标准 rosbridge XML 入口。本次只核对封装参数声明，未启动桥接做运行验收。

### 5.3 面板逐个添加

先加低带宽“原始消息”面板验证话题，再加图像和三维。不同 CoStudio 版本的中文面板名可能略有区别。

| 展示内容 | 话题 | 面板/说明 |
| --- | --- | --- |
| 官方提交题目和映射 | `/competition/task/problem` | 原始消息，JSON 在 data 内；其中可能包含 task_id |
| 直接搬运题 | `/competition/task/question` | 原始消息；官方 problem 入口未必同步到这里 |
| 颜色数量 | `/competition/task/counts` | 数组 data[0] 为红，data[1] 为蓝 |
| 顺序和目的地 | `/competition/task/order` | 先展示可读字符串 |
| API 状态 | `/competition/task/llm_status` | source、model、latency_sec、detail |
| 完整任务 | `/competition/task/plan` | items 的原顺序、task_id |
| 任务进度 | `/competition/mission/state` | completed、total、skipped、current_index |
| 正在做什么 | `/competition/navigation/state` | state、phase、route |
| 末端状态 | `/competition/gripper/state` | 末端状态字符串 |
| 避障 | `/competition/avoidance/state` | 响应和最小净空 |
| 原始 RGB | `/camera_rgbd/camera_rgbd/image_raw` | 图像面板 |
| 识别标注图 | `/competition/perception/debug_image` | 图像面板；无检测/无帧时检查实际发布 |
| 识别目标 | `/competition/perception/targets` | 原始消息，检查坐标系和目标 |
| 三维/地图 | `/map`、`/tf`、`/tf_static`、`/plan`、`/local_plan` | 三维面板，Fixed frame 先选 map |

显示机器人模型还需加载与当前工程一致的 URDF/模型资源；仅订阅 TF 不保证自动出现完整机器人模型。先让地图和路径正确显示。

推荐布局：顶部题目、颜色数量、目标区域；左侧两张图像和三维；右侧进度、工作状态、夹爪和 API 状态。

JSON String 不能直接按 `/competition/mission/state.completed` 取字段。未做转换前先显示原始 data，之后用 CoStudio 扩展/消息转换解析，再建立数值卡片。当前未交付已验证的转换扩展和导出布局，不要把此步骤说成已经做好。

不是每条状态都有 task_id；跨轮需按实际消息关联，缺少标识或过期时显示未知，不能沿用旧完成数。完整比赛总计时尚未实现统一可信话题时，也不能给面板上的任务 elapsed_sec 改名“完整比赛用时”。

开始按钮暂时用已有网页的受控题目入口；不要配置速度按钮直接发布 `/cmd_vel_safe`。桥接通常具备发布或调用服务的能力，“我只拿来看”不等于服务端自动只读。

### 5.4 连接失败怎么查

VM：

```bash
ss -ltnp | grep -E ':21274|:9090'
ros2 topic list -t
```

Windows PowerShell（按实际桥接选端口）：

```powershell
Test-NetConnection 192.168.153.128 -Port 21274
Test-NetConnection 192.168.153.128 -Port 9090
```

端口不通：查桥接进程、绑定地址、VM IP/网络。端口通但没有话题：先查连接器协议，再查 VM 内 ROS 域和 DDS 配置。话题有但卡片空：先看消息类型及 JSON String。不要同时调整三处来碰运气。

## 6. 自己发导航命令、开车和调试机械臂

### 6.1 启动不自动出题的练习模式

先停止 API 系统。终端 A 完成第 1.1 节后执行：

```bash
ros2 launch tools_demo competition_system.launch.py \
  headless:=False generate_question:=False \
  start_llm_server:=False use_llm:=False require_llm:=False \
  moving_obstacle:=True dashboard:=True
```

它保留导航、机械臂、感知和安全链，只关闭模型。先按第 2.2 节检查就绪。练习成绩不能当 API 全链比赛成绩。

### 6.2 去区域或某个物块泊位

前提：没有自动任务，没有其他导航或遥控命令正在运行。下面每条都会让底盘运动，逐条选择，不要一起粘贴。

```bash
ros2 topic pub --once /competition/navigation/goal std_msgs/msg/String "{data: 'A'}"
```

把 `A` 换成 `B` 或 `C` 可去对应区域。去实体泊位示例：

```bash
ros2 topic pub --once /competition/navigation/goal std_msgs/msg/String "{data: 'BLUE_CUBE_4:east'}"
```

语义导航会使用项目路线/泊位逻辑，只导航，不自动抓取。泊位中心和物块坐标不是同一个位置，不能直接把物块坐标当底盘目标。其他实体的接近侧必须查已验证配置，不随意改 east/west。

观察：

```bash
ros2 topic echo /competition/navigation/state --qos-durability transient_local
```

仅取消这次手动导航：

```bash
ros2 topic pub --once /competition/navigation/cancel std_msgs/msg/Bool '{data: true}'
```

自动搬运中需要停整轮时用第 9.1 节的 mission cancel，不能只取消导航后让协调器继续重试。

### 6.3 键盘遥控：走完整安全链

源码与安装的 Nav2 launch 对应链路：

```text
Nav2/键盘 /cmd_vel_nav
  → velocity_smoother → /cmd_vel
  → collision_monitor → /cmd_vel_collision_safe
  → competition_safety_limiter → /cmd_vel_safe
  → Gazebo 底盘
```

任务停靠的部分直接控制使用 `/cmd_vel`，仍经过碰撞监控。人工键盘推荐从 `/cmd_vel_nav` 进入，以包含速度平滑。

先确认所有自动任务/导航已停止，机器人无载荷并位于有净空的位置，再查：

```bash
ros2 lifecycle get /collision_monitor
ros2 lifecycle get /velocity_smoother
ros2 node info /competition_safety_limiter
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_safe -v
```

相关生命周期为 active，安全限速器连接正确，无其他正在下发的运动任务，才能开车。`Publisher count` 不等于某个发布者正在运动，必要时看实时消息。

VM 已安装键盘遥控包。在 VM 的交互终端运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.10 -p turn:=0.25 \
  -r cmd_vel:=/cmd_vel_nav
```

按终端显示的键位操作：`i` 前进、`,` 后退、`j/l` 转向、`k` 停止；保持终端有焦点。先低速点动，不用大写组合做横移。`Ctrl+C` 退出遥控。软件链路不是绝对防撞保证，不要故意顶墙或挤进狭窄处。

已安装版本按键时才发布，没有 `repeat_rate` / `key_timeout` 参数。不要误以为设置这些参数就有松键急停。当前速度平滑器的 velocity_timeout 为 1 秒，操作停止仍应主动按 `k`，并核对实际底盘停下。

简单零速请求（前提是其他运动发布者已经停止）：

```bash
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

零速一帧会被其他发布者覆盖，因此不是可靠的整系统急停。**不要直发 `/cmd_vel_safe` 或 `/cmd_vel_collision_safe` 来让车“终于动起来”。** 被安全链挡停时应该找原因。

### 6.4 RViz、TF 和感知检查

VM 图形终端：

```bash
rviz2 -d "$HOME/dev_ws/src/yzbot/bot_navigation/rviz/navigation.rviz"
```

地图固定坐标系选 `map`。使用 RViz 的导航目标前，先确保任务协调器空闲；它可能绕开项目语义导航附加航点，所以日常优先第 6.2 节。

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /competition/perception/targets --once
ros2 topic info /camera_rgbd/camera_rgbd/depth/image_raw -v
ros2 control list_controllers
ros2 topic echo /joint_states --once --qos-reliability best_effort
```

机械臂可打开仅 RViz 的现有入口：

```bash
ros2 launch mybot moveit_rviz.launch.py
```

先只观察模型、状态和规划。Execute 会实际驱动机械臂，不能与常驻抓放 worker 同时控制。不要在完整系统旁再运行一个默认 `pick_place_moveit`：默认 operation 是 cycle，可能执行整段动作并准备场景。

底层 `/competition/manipulation/request` 使用包含位姿、四元数和 request_id 的完整 JSON；只发 `operation=pick` 不够。日常抓放通过协调器进行。需要独立机械臂实验时，先隔离自动控制，核对目标坐标、场景和完整请求，再单步验证，不能把其他泊位的抓取坐标直接复制过来。

## 7. 只验证 API 解题，机器人不运行

用于检查密钥、模型和题目映射。先停止完整系统，下面使用独立 `ROS_DOMAIN_ID=77`；不要在此域启动机器人协调器。

终端 A：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/dev_ws/install/setup.bash"
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/dev_ws/install/tools_demo/share/tools_demo/config/fastdds_udp.xml"
source "$HOME/.config/yzbot/llm.env"
export DASHSCOPE_API_KEY
ros2 run tools_demo task_parser --ros-args \
  --params-file "$HOME/dev_ws/install/tools_demo/share/tools_demo/config/task_parser.yaml" \
  -p use_sim_time:=False -p use_llm:=True -p require_llm:=True
```

终端 B 先执行第 1.1 节，然后改域并启动网页：

```bash
export ROS_DOMAIN_ID=77
ros2 run tools_demo competition_dashboard --ros-args \
  --params-file "$HOME/dev_ws/install/tools_demo/share/tools_demo/config/dashboard.yaml" \
  -p use_sim_time:=False
```

打开 8080 网页，按第 3.2 节提交官方题目。这里只启动解析器和网页，plan 会产生但没有机器人执行。机器人/地图区域空白是预期现象。

终端 C：先第 1.1 节，再执行：

```bash
export ROS_DOMAIN_ID=77
ros2 topic echo /competition/task/llm_status --qos-durability transient_local
```

结束时关闭这两个节点，回到真实运行前重新执行第 1.1 节恢复域 0。不要同时在两个域启动 8080 网页，它们仍共用同一个系统端口。

此模式会真实消耗云 API 调用，只有模拟测试才不调用云端。认证失败检查私有环境和供应商授权；不要把 `require_llm` 关掉后声称问题修好了。

## 8. 运动基准、48 题与日志保存

### 8.1 单题自动基准

前提：先停止所有手动仿真。脚本自己启动和清理一套系统，不要先执行第 2 节。

```bash
cd "$HOME/dev_ws/src/yzbot"
RLC_TIMEOUT_SEC=600 bash tools_demo/scripts/run_ordered_case.sh \
  'Move 4 red cubes to zone B and 1 blue cube to zone A.'
```

这是 headless、关闭 LLM、关闭网页的运动基准，保留动态障碍。`600` 是诊断等待上限，不是比赛时限。脚本目前会转成浮点数再传给 ROS。

结束看 `EVIDENCE_DIRECTORY=...`。证据目录默认在 `~/dev_ws/experiments/ordered-cases/`，主要有：

- `question.txt`、`commit.txt`、`worktree.patch`：任务和运行基线。
- `launch.log`、`result.log`：过程和 runner 结果。
- `audit.json`：额外严格运动审查。旧手册的 `case_audit.json` 文件名不适用于当前脚本。
- `profile_status.json`：耗时曲线是否属于本轮。
- `speed_profile.json`：仅在 task_id、状态和新鲜度匹配时复制，缺失不应拿旧文件替代。

```bash
YZBOT_CASE_DIR=$(cat "$HOME/dev_ws/experiments/ordered-cases/latest-run.txt")
cat "$YZBOT_CASE_DIR/result.log"
cat "$YZBOT_CASE_DIR/audit.json"
```

`YZBOT_C_CLEARANCE_ROUTE=1` 是实验路线开关；代码默认关闭，旧布局几何仍需核验。不把它列入日常默认启动命令。

### 8.2 查看 48 种直接题，不运行仿真

```bash
cd "$HOME/dev_ws/src/yzbot"
python3 tools_demo/scripts/gen48_questions.py
```

每行是标签和题目。生成器已包含两种颜色先后顺序；旧批次脚本的 variant 注释不应当作额外题集依据。

### 8.3 运行少量定向补跑

```bash
cd "$HOME/dev_ws/src/yzbot"
COVERAGE_ROOT="$HOME/dev_ws/experiments/manual-regression-$(date +%Y%m%d-%H%M%S)" \
RLC_TIMEOUT_SEC=600 bash tools_demo/scripts/retry48_tags.sh 4rB_1bA 4bC_1rA
```

它逐题启动仿真，任务之间有等待。保留输出中的目录路径。日志 `RETRY_DONE` 不足以证明严格验收，需要检查各题 audit。

### 8.4 48 题全量入口及已知限制

仅在版本冻结、无人进行其他仿真时使用。长批次有较大墙钟成本，日常调试不要每次跑。

```bash
cd "$HOME/dev_ws/src/yzbot"
YZBOT_COVERAGE_DIR="$HOME/dev_ws/experiments/manual-coverage-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$YZBOT_COVERAGE_DIR"
python3 tools_demo/scripts/gen48_questions.py | while read -r tag question; do
  mkdir -p "$YZBOT_COVERAGE_DIR/$tag"
done
RLC_TIMEOUT_SEC=600 bash tools_demo/scripts/run48_v2.sh \
  1 48 redfirst "$YZBOT_COVERAGE_DIR" \
  2>&1 | tee "$YZBOT_COVERAGE_DIR/batch.log"
```

这里提前创建标签目录，避免已有脚本写 COMPLETED 标记失败。仍有两项限制：背靠背清理可能触发 STALE_SIM；COMPLETED 标记仅判断完成状态，不能代替完整验收。不要复用旧批次目录拼接新配置成绩，不能把这个入口称为已修好的可靠全自动验收系统。

汇总（保持同一终端的变量，或填入真实目录）：

```bash
python3 tools_demo/scripts/summarize48_v2.py \
  "$YZBOT_COVERAGE_DIR" "$YZBOT_COVERAGE_DIR/table.csv"
```

保留所有尝试。分别看五块完成、case_accepted、strict_motion_pass 和 under300，不把列混用。只做文档更新不会重新运行 48 题。

### 8.5 录 bag 复现故障

终端 C 完成第 1.1 节，在发题前开始：

```bash
YZBOT_BAG_DIR="$HOME/dev_ws/experiments/manual-bags/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$(dirname "$YZBOT_BAG_DIR")"
ros2 bag record -o "$YZBOT_BAG_DIR" \
  /clock /tf /tf_static /odom /scan /joint_states \
  /cmd_vel_nav /cmd_vel /cmd_vel_collision_safe /cmd_vel_safe \
  /collision_monitor_state /plan /local_plan \
  /competition/task/problem /competition/task/question /competition/task/plan \
  /competition/task/llm_status /competition/mission/state /competition/mission/error \
  /competition/navigation/state /competition/avoidance/state \
  /competition/obstacles/obstacle_1 /competition/obstacles/obstacle_2 \
  /competition/manipulation/request /competition/manipulation/result \
  /competition/gripper/state /competition/perception/targets
```

先用 `ros2 topic list -t` 核对实际话题。上面是基础记录集，定位 Nav2 action 还需补实际 action 的 feedback/status；定位视觉问题再加图像和 camera_info，避免无目的录全话题拖慢 VM。同时保留主系统文本日志，bag 不替代所有内部日志。

结束录制按 `Ctrl+C`，等 metadata 写完。回放必须在停止实体控制的隔离环境中进行，不能把含速度/任务的 bag 直接播放到正在运行的机器人域。

## 9. 取消、停止和重新开始

### 9.1 取消正在执行的自动任务

```bash
ros2 topic pub --once /competition/mission/cancel std_msgs/msg/Bool '{data: true}'
```

观察任务变为 canceled，导航停止。该取消流程可能请求机械臂 release，因此可能放下/释放当前物块，不是“冻结保持姿态”，也不是硬件急停。

如果只有手动语义导航，使用第 6.2 节的 navigation cancel。键盘遥控先按 `k` 停止，再退出其终端。

### 9.2 正常停止整套系统

1. 停止出题程序、手动控制和其他命令发布者。
2. 需要时取消任务并观察停止。
3. 录 bag 的终端按 `Ctrl+C` 并等写盘完成。
4. 主系统终端 A 按一次 `Ctrl+C`，等待 launch 结束子进程。
5. 桥接、独立网页/API、RViz 等终端分别 `Ctrl+C`。
6. 执行第 1.2 节，确认没有剩余系统后再启动下一轮。

只关闭 Gazebo 窗口、只杀 gzserver 或只刷新网页，都不能可靠地重置整轮任务。

### 9.3 找不到原启动终端时

先只列出进程：

```bash
ps -eo pid,ppid,pgid,args | grep -E '[r]os2 launch tools_demo competition_system|[g]zserver|[c]ompetition_mission_coordinator|[c]ompetition_task_navigator'
```

找到明确属于本轮的 `ros2 launch tools_demo competition_system.launch.py` 主进程，确认后发送 INT：

```bash
read -r -p '输入刚核对的主 launch PID：' YZBOT_LAUNCH_PID
ps -p "$YZBOT_LAUNCH_PID" -o pid,ppid,args
# 再核对上行确实是本轮主 launch，确认后执行下一行
kill -INT "$YZBOT_LAUNCH_PID"
```

等待正常退出。残留时先保存日志并找明父子关系，不对所有 Python、所有 ROS 或全部 launch_params 进程进行无差别清理。若无法确认残留归属，保存材料后重启 VM 比随意杀进程更容易恢复到可理解状态。

`clean_launch.sh` 是历史“强制清理 + 立刻重启练习系统”的脚本，含较宽的进程清理，不作为日常停止按钮使用。

### 9.4 下一轮如何复位

完整停止后重新执行第 2 节或第 6.1 节。新 Gazebo 进程重新加载世界，恢复物块及机器人初始布局。关闭上一轮残留的题目发布器，防止 retained 旧题触发新启动系统。

不建议在运行中调用 Gazebo reset_world/reset_simulation 代替整套重启：物理时间、TF、Nav2、任务状态和已搬运库存可能不同步。

## 10. 故障速查

| 现象 | 先查什么 | 下一步 |
| --- | --- | --- |
| `ros2: command not found` | 终端是否 source ROS | 重做 1.1 |
| 节点列表空、网页有但 ROS 查不到 | 域、DDS 配置、是否到任务启动阶段 | 重做 1.1，等待发现；核对进程，不先杀系统 |
| 相机无帧，headless 下抓取全失败 | Ubuntu 图形会话、DISPLAY、Gazebo 相机日志 | 保持 VM 桌面登录，查渲染失败和传感器，不直接归为机械臂竞态 |
| `Missing private LLM credentials` | 私有环境文件路径和可读性 | 使用既有私有文件；不把 key 写进 YAML |
| 401/403 | 当前密钥权限、端点和供应商状态 | 修正授权，不反复重试，不关闭 require_llm 冒充成功 |
| 解题超时/校验拒绝 | llm_status、模型、thinking、题目和映射 | 等一次请求结束；检查完整内容，记录失败后再重试 |
| `awaiting mapping rules` | 官方应用题误发到 question | 使用 problem/网页入口并带完整映射 |
| HTTP accepted 但车不动 | LLM 是否失败、plan 是否产生、任务是否忙碌 | 按 3.2 的状态链查，不重复点按钮 |
| 同题再次提交没有动作 | 去重及物理场景是否复位 | 完整停止重启后再发，不仅换 task_id |
| `already active` | 自动任务/语义导航尚未结束 | 等待或取消后再下发 |
| `STALE_SIM_DETECTED` | 自动基准前已有 gzserver | 先停原系统，不同时启动两套 |
| out-of-bounds、抖动 | 重复节点、地图/定位、costmap | 按日志定位，不能一概认定幽灵节点 |
| `scan failed` / CONTROL_FAILED | 控制器、joint_states、MoveIt 执行、图像/TF | 分开排查规划、控制、感知；不是固定“重跑就好” |
| 安全链 active 但车不动 | 各级速度和激光、TF、障碍 | 看在哪一层变零；禁止直发安全下游绕过 |
| 单块跳过后任务继续 | 入站/抓取/投放失败日志 | 记录实体和阶段；有跳过即不满足零跳过要求 |
| C 区耗时长 | 墙角等待/转向/规划、实际实体、路线开关 | 保留 bag 和分段时间，不先全面提速 |
| 侧翻 | 碰撞、速度发布者、载荷姿态和控制阶段 | 停止运动、保留证据、重启场景；不能仅凭姿态断言人工原因 |
| bridge 端口通但 CoStudio 空白 | 连接器类型、ROS 域、消息类型 | 按 5.4 分层检查 |

## 11. 改配置、编译与检查（维护人员）

日常运行不需要 git pull 或重新编译。修改前确认没有计时仿真，记录 `git status`；尤其不要覆盖既有策略表改动。

主要文件：

| 内容 | 源码路径（相对仓库） |
| --- | --- |
| 云 API 模型和预算 | tools_demo/config/task_parser.yaml |
| 协调、抓放和任务参数 | tools_demo/config/mission.yaml |
| 选块表 | tools_demo/config/cube_order_policy.json |
| 路线、泊位、动态障碍 | bot_navigation/config/competition_zones.yaml |
| Nav2 和碰撞监控 | bot_navigation/param/originbot_nav2.yaml |
| 导航地图 | bot_navigation/maps/mapn3.yaml 及对应图像 |
| 世界与物块 | mybot_description/worlds/offic_room.world |
| 网页配置 | tools_demo/config/dashboard.yaml |
| 感知与相机话题 | tools_demo/config/perception.yaml |

只改 tools_demo 时：

```bash
cd "$HOME/dev_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tools_demo
source install/setup.bash
```

涉及多个包时选择实际修改包；首次建立整个工作区或依赖改变才做完整构建。配置文件是否实时链接要检查，不要假定修改源码就自动改变 install：

```bash
ros2 pkg prefix tools_demo
readlink -f "$HOME/dev_ws/install/tools_demo/share/tools_demo/config/mission.yaml"
ros2 param get /competition_mission_coordinator mission_timeout_sec
ros2 param get /competition_mission_coordinator cube_order_policy_file
ros2 param get /competition_task_navigator c_clearance_route_enabled
```

参数查询需要节点运行。单纯查 launch 参数不会启动仿真：

```bash
ros2 launch tools_demo competition_system.launch.py --show-args
```

功能检查示例：

```bash
cd "$HOME/dev_ws"
python3 -m pytest -q src/yzbot/mybot/test src/yzbot/tools_demo/test \
  --ignore=src/yzbot/mybot/test/test_flake8.py \
  --ignore=src/yzbot/mybot/test/test_pep257.py \
  --ignore=src/yzbot/tools_demo/test/test_flake8.py \
  --ignore=src/yzbot/tools_demo/test/test_pep257.py
```

上述明确排除了四个风格检查，不能称全仓测试全绿。历史 449 passed、1 skipped 不代替本次实际结果。代码行为变化仍需对应的仿真验证。

## 12. 本版核查与尚未完成部分

本版已核对：源码及安装 launch 参数、入口脚本、题目和取消接口、网页 HTTP 路由、速度链重映射、官方出题程序可运行、rosbridge/键盘遥控安装情况。官方出题程序本次实际输出了衣柜应用题。

本版没有启动机器人、发送运动/抓取任务、调用云 API、安装 coBridge 或配置 Windows CoStudio；操作示例按接口核对，不能替代动态验收。

仍需项目后续解决：新配置 4rB_1bA 回归、C 区耗时、首用执行失败、全比赛计时、CoStudio 布局与字段转换、批次可靠恢复。文档修订没有把这些项目问题标为完成。

历史手册有多份且部分内容过时。本版作为日常入口，旧版保留备份；遇到“C 实验路线默认开启”“网页只读”“http 连接 rosbridge”“700 秒是比赛时限”等旧说法，以本版纠正内容及实际配置为准。
