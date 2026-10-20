# YZBot 224：省赛 U 盘系统交付与操作手册

更新：2026-10-12。目标环境：**x86_64 / Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11**。最新冻结版本及计时口径见 `版本冻结流程.md` 和 `FROZEN_BASELINE.json`；旧验证结论保留其日期和覆盖边界。

本包在原生 Linux 中重新构建运行，用户名不限，不依赖虚拟机 IP 或 VMware。它是项目源码、安装工具及操作说明，**不是可以直接引导启动的系统镜像，也不是完整离线 ROS 镜像**。首次安装缺失的系统依赖和真实 API 解题需要联网。U 盘系统尚未收到，所以实际硬件驱动、桌面渲染、持久化和 Linux CoStudio GUI 兼容性还需上机确认。

## 1. 先看版本和已知边界

原提交及导入状态在 `SOURCE_VERSION.json`；`SOURCE_USER_CHANGES.patch` 单独保存原有支撑轮 TF 和 Gazebo GUI 用户修改。源码已包含这两个运行所需修改，不能重复 apply 该补丁。已在不同路径、仅加载系统ROS的干净环境中重新构建全部7个包；构建日志随源码验收文档保留。包内另将两份历史GBK文件名文档转为UTF-8名称，文档内容字节不变，映射见SOURCE_FILENAME_MAP.json；运行代码没有因此更改。包内 `SHA256SUMS` 覆盖源文件、安装包和文档。Git 完整历史、模型解题服务 GGUF、build/install/log、私有凭据和大体积 rosbag 均不打包。机器人模型、网格、世界、地图、配置及比赛源码全部保留。

原交付改动：正式映射规则支持；官方生成器的新题型算术校验；真实模型解题提示；颜色顺序无要求后的等价结果校验；Nav2 自定义行为树按安装目录解析；凭据按当前用户 HOME 查找。

20261012 新基准改动：CoStudio 1.4.5 的点击会话计时、后台终态时间戳、刷新/断连/演示隔离、可读布局；机械臂放置与抬升检查碰撞，回位成功后才确认放置完成；官方书箱题“本／借阅”词汇补全。细节见 `LOCAL_CHANGES_20261012.md`，最终逐轮结论以 `acceptance/验证报告.md` 为准。没有改变运行场景布局、模型、速度、安全阈值、障碍轨迹或载荷滤波。

旧 `f8074d4` 的 48/48 小于290秒结论只属于旧冻结版本。包含 A 墙角修复的历史五蓝→A 单次293.9秒、5/5、墙体接触0、动态最小净空0.5122m；历史报告仍有取件托盘非夹爪接触，本次不冒充已消除。新基准按 PDF 三规则抽样完整比赛墙钟复验，不等于修改后所有 24/48 组合重跑；**48题、全部接触类型和U盘实机均未全验收**。这些边界不因“可部署”而改变。

## 2. 收到 U 盘系统后的准备

1. 通过电脑启动菜单选择主办方 U 盘系统并登录图形桌面。不要格式化或覆盖主办方系统，不要更换其机器人模型/比赛平台。
2. 将本交付文件夹复制到 Linux 的“下载”目录，**先解压 ZIP，再执行脚本**。如通过另一存储盘带文件，先复制到 Linux 持久化分区；不要在 Windows NTFS/exFAT 文件系统直接构建。
3. 必须确认该系统是可保存文件的安装系统或有持久化的 Live USB。若只是 Live USB RAM 临时系统，HOME 中源码、构建及 Key 会在重启后消失。完成部署后重启一次，核对 `~/yzbot_usb`、API 私有配置和 CoStudio 扩展仍在。
4. 系统或架构不同，保留原镜像并先确认主办方兼容要求；不要强行把 Humble 的包安装到 Ubuntu 24.04/Jazzy 或 ARM。包内 ARM 出题程序仅是官方工具，不代表机器人项目已经支持 ARM。

终端进入刚解压的文件夹（可在文件管理器中“在终端打开”），运行：

```bash
bash scripts/00_system_check.sh
sha256sum --quiet -c SHA256SUMS
```

检查会显示系统、架构、HOME 文件系统、ROS 和依赖。默认运行目标是 `~/yzbot_usb`。保留至少约20GB可用空间用于依赖、构建和证据；系统已具备依赖时实际增量较小。

## 3. 首次部署：按此顺序

在解压后的交付文件夹执行：

```bash
# 系统已预装全部依赖也可先检查；缺少时联网安装，没有 apt upgrade
bash scripts/02_install_dependencies.sh

# 复制到持久化HOME、初始化导入快照Git、重新构建并跑行为测试
bash scripts/01_deploy.sh

# 后续切到安装好的交付目录
cd "$HOME/yzbot_usb"
bash scripts/03_configure_api.sh
bash scripts/04_install_costudio.sh
bash scripts/08_api_check.sh
```

`01_deploy.sh` 遇到非空目标目录会停止，不会覆盖旧工程。要保留旧版本，可传一个新的绝对目录，如 `bash scripts/01_deploy.sh "$HOME/yzbot_usb_trial"`；后续命令随之在这个新目录执行。包里不复制 VM 的 install 目录，而是重新构建全部7个ROS包。导入的本地Git提交只是快照，不能冒称原始远端提交，原版本见 `SOURCE_VERSION.json`。

### API 地址、模型和 Key

`03_configure_api.sh` 不回显 Key，保存到 `~/.config/yzbot/llm.env`（目录700、文件600），旧私有文件会另行备份。**不要把该文件放进 Git、公开交付、网盘作品或录屏**。地址使用供应商业务空间提供的 OpenAI 兼容根地址，例如包含 `/compatible-mode/v1`，不要填 DashScope 原生 `/api/v1` 或重复 `/chat/completions`。模型默认沿用当前配置 `qwen3.8-max`、thinking关闭；如改模型需填真实可用模型名称，并重新执行检查。

非秘密的地址/模型配置存于 `workspace/src/yzbot/tools_demo/config/task_parser.yaml`，脚本修改后重建tools_demo。`08_api_check.sh` 在独立ROS域93验证一题真实API，不发布搬运计划、不启动机器人，**会产生一次正常API调用费用**。结果必须 `source=llm`；失败时解决网络、Key、权限或模型问题，不改成非模型规则模式冒充正式比赛。

### CoStudio

随包有官方 Linux amd64 安装包及原生 coBridge 1.1.2 deb，来源和校验见 `vendor/linux/installer-origin.json`。`04_install_costudio.sh` 会保留已存在的客户端，仅在找不到客户端命令时安装随包客户端；安装扩展前把同名旧版本移动到用户备份目录，不删除旧布局或 IndexedDB。

项目扩展为 `aerospaceboy.yzbot-competition` 1.4.5，随包带编译好的 `dist/extension.js`，普通操作无需 Node/npm。安装位置是当前桌面用户 `~/.coStudio/extensions/`；不能用 root 启动 CoStudio。本机 Linux CoStudio 的真实点击、计时及刷新已验证，U 盘目标系统仍需确认 SDK 和 GUI 兼容性；优先保留主办方已可用的客户端，不因本包存在就盲目升级。

依赖参考：[coScene 官方APT说明](https://docs.coscene.cn/docs/client/apt-source-install/)、[原生coBridge连接说明](https://docs.coscene.cn/docs/client/connect-by-cobridge/)、[扩展本地安装说明](https://docs.coscene.cn/en/docs/viz/extensions/local-development)。ROS本体缺失时先按 [ROS 2 Humble官方Ubuntu安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 处理，再执行项目依赖脚本。

## 4. 每次比赛启动

本次通信修复统一默认 `rmw_cyclonedds_cpp`（本机环回 UDP），需要已安装 `ros-humble-rmw-cyclonedds-cpp`。通过 coBridge WebSocket 可从其他电脑观看，但 ROS 节点默认仍在本机。调试也请先 `source scripts/lib.sh; load_ros`，不要混用旧 daemon 或其他 DDS 配置。仅诊断对照时显式设置 `YZBOT_RMW=rmw_fastrtps_cpp`；该覆盖不是已验收默认环境。入口先清旧栈再重启 daemon，启动子进程故障立即拒绝提交；PASS 前必须收到递增时钟及两帧有效扫描。

在 Linux 图形桌面打开终端：

```bash
cd "$HOME/yzbot_usb"
bash scripts/05_start_competition.sh
```

入口启动 Gazebo GUI、RViz、完整任务/导航/安全/机械臂系统、两路相机、coBridge、网页面板和SLAM展示，并在系统就绪后尝试打开CoStudio客户端。**不会自动生成题目、不会自动搬运**。等到输出 `MANUAL_PROBLEM_MODE=PASS` 后才能提交题目。API参数未配置或安全链未就绪会停止启动。

**任务进行中不能再次执行05启动脚本**，该入口会清理上一轮项目栈。要诊断连接请用07脚本。默认Gazebo显示60帧；性能不足可以仅降低渲染帧率，不加速仿真：

```bash
YZBOT_GAZEBO_GUI_RENDER_RATE=30 bash scripts/05_start_competition.sh
```

CoStudio 没自动打开时从 Linux 应用菜单启动，或执行已安装的 `costudio` / `coStudio`。随包0.4.0客户端实际入口为 `/opt/coStudio/honeybee`，05脚本也支持此入口。程序完成后各界面仍开启，录制结束再停止系统。启动脚本会打印本轮 `runs/manual-时间/launch.log` 与桥接日志路径。

## 5. CoStudio连接与导入布局

1. 打开连接，选择 **coScene WebSocket**，填 **`ws://127.0.0.1:21274`**。这是同一个原生 Linux 系统，不使用旧 VM 地址。
2. 布局管理 → 从本地文件导入，选择 `costudio/YZBot比赛布局.json`。扩展必须先装好并重启客户端。
3. 题目面板选择**实时 LIVE**，状态应显示ROS2已连接。不要选择演示模式或用“开始演示”代替真实任务。
4. 3D/地图坐标系选择 `map`；地图 `/competition/map`、场景 `/competition/scene_markers`，机器人状态、两路相机和数量卡应持续更新。**黄色粗线是当前目标规划路线，青线是实际行驶轨迹**；黄线保持至目标到达/终止，下一目标换线，青线保留历史。
5. 检查腕部和底盘相机画面不同：`/camera_rgbd/camera_rgbd/image_raw` 与 `/camera/image_raw`。任务进度、当前任务、夹爪状态应来自真实数据。

如果另一台电脑观看，Linux执行 `hostname -I`，客户端填 `ws://<这台Linux的实际局域网IP>:21274`，两台电脑在可互通网络。**localhost始终表示客户端所在那台电脑**。HTTP备用面板为 `http://127.0.0.1:8080`（远端换真实IP），用于状态观察及必要排错，不能把它当CoStudio WebSocket地址。9090 rosbridge与21274原生coBridge不是同一协议。

## 6. 正式出题、填规则、比赛计时

附件规定：正式出题程序附加 **`11470507`**，题目每30分钟变化。省赛队伍选择一套映射规则，国赛计时前由裁判挑选。题目与映射可以计时前输入/粘贴，须与裁判确认好，再开始任务并计时。

第二个终端执行：

```bash
cd "$HOME/yzbot_usb"
bash scripts/06_generate_question.sh
```

这等价于官方原生Linux程序 `TMSCQtest_x86_x64.bin 11470507`。不要把内部gen48、练习题或不带参数的随机题当省赛正式题。脚本保留原始stdout，不修改系统时钟，也不会直接发布ROS任务。复制**“题目:”之后的完整正文**，不复制种子、分隔线和说明。正式计时前不要跨过30分钟题目切换窗口；若切换需与裁判重新核对。

CoStudio“题目与大模型解析”面板中：上框贴题目，下框贴选定规则。三套逐字原文另存 `examples/规则1.txt`～`规则3.txt`：

| 规则 | 可直接粘贴的正式文本 |
|---|---|
| 1 | x 代表红色,y 代表蓝色;数值大于等于 3 的去 A 区,其他颜色去 C 区 |
| 2 | xy 中,数值大于等于 3 值的代表红色，否则代表蓝色；x 去 B 区,y 去 A 区 |
| 3 | 蓝色去 B 区,红色去 C 区; xy 中,数值大的是蓝色，数值少的是红色 |

**可以预填，不能提前点“确定并执行”**。该按钮会启动真实API解析并在校验通过后直接下发搬运计划，不是“只预览答案”。裁判确认计时开始时点击一次，等待“大模型解析成功 · 指令已发布”。不连续点、不重发，任务中不用旧“开始任务”按钮另发指令。

例如练习题的x=3、y=2：规则1为3红→A、2蓝→C；规则2为3红→B、2蓝→A；规则3为3蓝→B、2红→C。**第二、第三套不能假定x恒为红色**。规则3相等时原文未定义“大/小”，系统应拒绝猜测，向裁判确认。正式新规则不要求颜色执行先后顺序，但颜色、数量、目的区必须正确，每件得分以实际规则验收为准。

解析过程中可以观察 `parse_result` / `llm_status`。API不可用或答案与校验不一致会显示失败并停止下发，不冒充模型成功。正式计时从裁判指定起点计到五件最终完成，**包括API解题时间**，不能只看运动elapsed；完整墙钟必须现场录制/记录。

## 7. 比赛演示与拍摄流程

1. 提前完成系统部署、网络/API检查、CoStudio布局及相机检查，做一题练习确认机器人实际动作。随后干净重启场景，恢复出生位和所有物块，再开始正式演示。
2. 开始录屏或外部拍摄，展示完整地图、10个物块、A/B/C区、四个连续移动体、任务面板。不要在正式运行过程中重启系统或手动给底盘发速度。
3. 展示官方程序运行命令及当前题目，展示所选原文映射规则。填入CoStudio后与裁判确认，开始计时并点击“确定并执行”。
4. 连续记录模型解析、规划黄线与实际青线、抓取/运输/投放、至少两次不同移动体的真实避障、最终5/5/零跳过和真实投放。发生失败如实记录，不拼接失败与成功为同一轮。
5. 最后一件验收完成后再停止计时并展示结果与安全证据。保存题目stdout、所选规则、源码版本、API耗时、运动耗时、完整墙钟和日志。

按用户提供的赛程截图，创新赛视频要求竞赛任务全程一镜到底，不能加速、删减、拼接或做混淆视听的后期；作品通过百度网盘提交视频、**Word技术文档**和全部算法源码。本包附Word版《交付与部署说明.docx》，内容是安装和操作说明，不能自动替代完整作品技术说明；仍需结合队伍作品补足技术文档并按本省截止时间提交。赛事调整以本省组委会为准，包中保留附件原图供核对。

## 8. 常见故障按顺序排查

```bash
cd "$HOME/yzbot_usb"
bash scripts/07_check_connection.sh
```

| 现象 | 处理 |
|---|---|
| CoStudio连不上 | 同机用127.0.0.1:21274与coScene WebSocket；查本轮cobridge.log、`ss -ltnp`，确认实际21274监听 |
| 有连接无数据 | 确认05输出PASS，检查status/scene_markers有发布者，客户端LIVE；所有ROS终端用scripts/lib.sh加载同一域及DDS配置 |
| Unknown panel type | 确认1.4.5扩展已装到当前桌面用户的~/.coStudio/extensions，仅保留一个同名版本，重启客户端再导入配套布局 |
| IndexedDB backing store错误 | 退出全部CoStudio，检查HOME空间、目录归属与可写持久化分区，避免sudo启动；备份确认过的应用用户配置后再建新profile，不删除整个~/.config或扩展/作品 |
| RViz没有地图 | 固定坐标map、use_sim_time=true，查看/map、TF和map_server启动日志；不是靠改坐标名掩盖未激活的代价图 |
| 401/403/模型不存在 | 检查供应商业务空间Key、根地址、模型权限；使用03修改后08独立复核。不要展示Key，不自动退回规则模式 |
| 出题随机且只有两个练习题 | 必须带11470507；06默认正式模式。`--practice`仅明确练习时使用 |
| 场景加载慢/卡死 | 先检查GPU驱动、内存和实时因子；降低渲染帧率或录屏负载，不改物理步长、移动障碍和安全阈值 |
| 任务卡住/有疑似碰撞 | 保留launch.log与bag/contacts证据，不盲目重复点执行或重启。当前托盘接触风险未全部闭环，应先解决再声称满分验收 |

只重启桥接时，**在21274确实无监听**后执行（不会重置机器人场景）：

```bash
source scripts/lib.sh
load_ros
ros2 launch tools_demo costudio_bridge.launch.py address:=0.0.0.0 port:=21274 camera_streams:=True
```

调试终端也用上述环境，再执行 `ros2 topic echo /competition/mission/state` 或 `/competition/task/parse_result`。不要直接发布 `/cmd_vel_safe`，这会绕过正常安全入口；地图点击导航及额外控制话题的限制不为消除提示而放开。

## 9. 文件速查

| 位置 | 内容 |
|---|---|
| workspace/src/yzbot | 当前完整受版本管理的工程源码与资产，另含两份受保护用户修改 |
| scripts | 系统检查、依赖安装、部署构建、API配置检查、CoStudio扩展安装、启动、正式出题、连接排查 |
| costudio | 配套JSON布局、编译扩展与连接说明 |
| vendor/linux | 已校验的Linux CoStudio与coBridge安装包；操作系统依赖仍由APT解决 |
| vendor/question_generator | 官方x64/ARM/Linux与Windows出题器及说明；比赛使用11470507 |
| examples | 三套正式原文规则、明确标为练习的示例题 |
| rules | 用户提供的省赛PDF和赛程要求截图，作为输入资料保存 |
| SOURCE_VERSION.json / SOURCE_USER_CHANGES.patch | 上游版本、保护修改与快照说明 |
| SHA256SUMS | 交付完整性校验；部署后修改API非秘密参数会改变对应文件哈希 |

## 10. U盘首次上机验收清单

- [ ] 重启后源码、构建、扩展和私有凭据仍存在；系统/架构与本包匹配。
- [ ] 原生Linux重新构建成功，源码/配置没有旧VM工作区依赖。
- [ ] API预检source=llm，三套正式映射各抽查一次正确数量和目的区。
- [ ] Gazebo、RViz、CoStudio地图/路径、两路相机、状态全部真实更新。
- [ ] 裁判确认后的官方题目完成5/5、零跳过，真实投放、动态响应、安全净空、墙/托盘接触均独立核对。
- [ ] 完整比赛墙钟按现场口径达标，视频连续、源码版本与展示版本一致。

本包未在尚未收到的U盘系统上运行，以上未勾选项不能提前填“通过”。
