# CoStudio 连接与展示说明

更新：2026-09-21。当前分支 `huxuyang`，下面说明适用于原生 coBridge。

## 当前状态与启动

- VM 已安装 coBridge 1.1.2，入口 `costudio_bridge.launch.py` 使用 `cobridge`，不是 rosbridge。
- CoStudio 必须安装与本分支配套的 `aerospaceboy.yzbot-competition` 1.4.4 扩展。旧 1.5.0 仅注册 `mission-overview`，不能配合当前布局，会报 `Unknown panel type`。切换前保留旧扩展到 extensions 目录之外。
- 启动完整系统：在 VM 的 ROS 工作区仓库执行 `bash tools_demo/scripts/start_manual_problem_mode.sh`。等待 `MANUAL_PROBLEM_MODE=PASS`，再从 Windows 连接 VM。此命令会关闭上一轮项目进程，任务执行中不要重复运行。
- 该入口默认关闭网页面板；需要网页时，在启动命令前加 `COSTUDIO_DASHBOARD=True`，不能因为文档列出 8080 就认为服务已启动。
- 只恢复桥接（主系统已运行，且 21274 没有监听）：先 source ROS 与当前工作区环境，使用相同的 ROS_DOMAIN_ID、RMW 和 Fast DDS 配置，再运行 `ros2 launch tools_demo costudio_bridge.launch.py`。桥接本身不会启动相机或任务系统。
- 题目面板的“确定并执行”会在解析成功后下发任务。旧 `/competition/mission/start` 按钮没有当前后端订阅者，不应作为比赛入口；部署布局已移除此按钮。

## 正确的协议对应关系

| VM 服务 | 客户端连接器 | 地址示例 |
| --- | --- | --- |
| coBridge | coScene Websocket | ws://192.168.153.128:21274 |
| rosbridge | 明确支持 rosbridge 的连接器 | ws://192.168.153.128:9090 |
| 项目网页 HTTP | 普通浏览器 | http://192.168.153.128:8080 |

IP 以 VM 的 `hostname -I` 为准。不能给 coScene Websocket 填 rosbridge 地址，也不能把 http://...:9090 当成正确的 WebSocket URL。

coBridge 的安装与原生连接步骤依据 [官方连接文档](https://docs.coscene.cn/docs/client/connect-by-cobridge/) 和 [官方 APT 说明](https://docs.coscene.cn/docs/client/apt-source-install/)。所有 VM ROS 终端应使用与主系统一致的域和 Fast DDS 配置，详见主手册第 1 节。

已有 rosbridge 的标准入口：

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090
```

## 显示内容

- 官方题目及映射：`/competition/task/problem`。直接搬运题：`/competition/task/question`。两者不能互相假定自动同步。
- 数量、顺序、计划：`/competition/task/counts`、`/competition/task/order`、`/competition/task/plan`。
- 进度、导航、模型：`/competition/mission/state`、`/competition/navigation/state`、`/competition/task/llm_status`。
- 解析证据：`/competition/task/parse_result`。携带本次提交的 `request_id`、解析状态（`parsing`/`published`/`failed`）、模型名、题目、映射规则与完整计划；面板据此显示“大模型解析成功 · 指令已发布”或解析失败原因。契约与实测证据见 [PARSE_DISPLAY.md](PARSE_DISPLAY.md)。
- 场景标记：`/competition/scene_markers`（物块、A/B/C 区、小车、动态障碍），底图别名 `/competition/map`。
- 两路物理相机：底盘前视相机 `/camera/image_raw`，机械臂腕部 RGB-D 相机 `/camera_rgbd/camera_rgbd/image_raw`。两路图像默认随一键启动一起放行；仅当设置 `COSTUDIO_CAMERA_STREAMS=False` 时 coBridge 才不转发图像话题，此时两个画面会一直显示“等待图片信息中”。
- 感知调试：`/competition/perception/targets`、`/competition/perception/debug_image`。其中 `debug_image` 是腕部 RGB-D 彩色图的标注派生流，不是第三台相机。
- 地图/轨迹/变换：`/map`、`/plan`、`/local_plan`、`/tf`、`/tf_static`。
- 夹爪/避障：`/competition/gripper/state`、`/competition/avoidance/state`。

`plan`、`mission` 等消息为 JSON 包在 String.data 内。先用原始消息确认数据，再做 JSON 转换，不能直接按 ROS 原生嵌套字段配置卡片。

面板与状态卡由仓库内扩展 `tools_demo/costudio_extension/yzbot-competition` 提供；改动后需重新构建 `dist/extension.js` 并覆盖已安装扩展，再重启 coStudio 才会生效。

completed 是已投放数；current_index 是逻辑任务位置；cube_name 是实体。不是每条消息都有 task_id。跨轮关联、断连、旧数据和错误 JSON 均要明确处理，不显示上一轮成功值冒充本轮状态。

## 权限与开始按钮

桥接协议可以提供发布和服务调用能力。“只用于观察”的注释不是权限保证。不要通过可视化绕过机器人安全链。

现有网页确实有 `POST /api/task`；“解析并下发任务”会在解析成功后驱动机器人。HTTP accepted 不代表任务已执行或验收。运行中不重复提交；当前网页也没有完整的 round_id 和忙碌请求控制，不能将其描述为已完成的严格比赛控制台。

## 尚需验收

- 连接协议、晚加入消息、断线与重新连接。
- 官方题目与直接题目显示、颜色顺序和数量。
- JSON 转换、跨轮标识和失效数据。
- RGB/识别图/地图/TF/机器人模型与实际一致。
- 两个视觉面板必须分别订阅 `/camera/image_raw` 与 `/camera_rgbd/camera_rgbd/image_raw`，不得用 `/competition/perception/debug_image` 冒充第二台相机。
- 按钮不会重复启动或绕过安全链。
- 真实总墙钟与展示开销；当前任务 elapsed_sec 不代表全比赛计时。
- 导出布局和真实操作截图，保留失败记录。

## 2026-09-21 连接故障排查

1. `ss -ltnp | grep 21274`：无监听先启动系统；不要更换为 rosbridge 的 9090。
2. 能连接但没有话题：核对 ROS 域与 Fast DDS 环境，确认完整系统已就绪。
3. `Unknown panel type`：扩展与布局不匹配，安装仓库配套扩展并重启 CoStudio。
4. 图像只有一路：导入当前布局，两路绑定分别是 `/camera/image_raw` 与 `/camera_rgbd/camera_rgbd/image_raw`。
5. 3D 面板可能自动尝试注册 `/move_base_simple/goal`、`/clicked_point`、`/initialpose`；服务器白名单拒绝这些发布会产生 `not whitelisted` 提示。这不是连接失败，不影响地图/TF/图像订阅。不要为消除提示而开放控制权限。
6. `/competition/task/parse_result` 与 `/competition/navigation/current_goal` 已加入只读订阅白名单，便于核对解析证据与当前目标；客户端可发布的话题仍只允许题目分片。

本轮验证范围是部署、启动、数据连接与可视化；未重新运行完整搬运比赛，不能据此宣称本次完成 48 题或 300 秒回归。
