# Gazebo 一键启动窗口异常修复（2026-10-23）

## 现象与定位

接手时 `runs/manual-20261023T020713` 的仿真服务、导航和协调器已运行，但 gzclient 仍只有 576×320 的启动窗口，没有 Gazebo 主窗口，也没有 GUI 插件初始化记录。前一启动 `manual-20261023T020634` 同样没有 GUI 初始化记录。

原场景 launch 使用 `gazebo` 包装程序同时启动 gzserver 和 gzclient。图形客户端存在独立的连接/场景初始化阶段，服务器的 ROS 服务可用不代表窗口初始化完成。原一键就绪检查只证明 ROS 图、进程及传感器健康，因此不能区分这个状态。

诊断中，在场景服务器启动后独立打开客户端，能正常创建主窗口并加载场景。已采用事件驱动的启动顺序规避并发初始化窗口；此处并未把具体 Gazebo 内部锁或网络故障判定为唯一根因。

## 修改

- `mybot/launch/gazebo_world.launch.py`：所有模式统一由 launch 直接管理一个 gzserver，服务端保留场景文件与原有 ROS 插件；仅在 `spawn_entity.py` 成功生成 `six_arm` 后打开一个 gzclient。失败时不打开误导性的空场景窗口。
- 标准 GUI、受限 GUI、headless 的条件保持独立；headless 只有服务器。
- 两类客户端开启 `emulate_tty=True`，避免后台 stdout 缓冲导致 GUI 已就绪但当前启动日志迟迟没有标记。
- `tools_demo/scripts/start_manual_problem_mode.sh`：默认受限 GUI 模式要求当前启动日志中出现 `YZBot low-load GUI active:`，该标记在客户端场景及相机可用后产生。超过 90 秒仍未出现时明确报告图形窗口未就绪，禁止宣告启动成功。可通过 `GAZEBO_GUI_TIMEOUT_SEC` 配置诊断时限。
- 保留原来所有 ROS 图、导航 action、碰撞保护输出、RGB-D/激光与活时钟检查。图形就绪不能替代仿真/导航就绪；headless 不要求客户端标记。标准非受限 GUI 不使用专用插件标记。

## 验证

使用项目原入口：

```bash
cd <项目目录>
bash 启动本版本.sh
```

最终版本连续两次完整冷启动，均打开 1100×700 的 Gazebo 主窗口并输出 `MANUAL_PROBLEM_MODE=PASS`。日志顺序为：机器人生成成功 → gzclient 启动 → GUI 场景/相机初始化完成。每次一个服务器、一个客户端；RViz 保持可用。

第一次最终启动独立监听 4 秒：时钟 41 条、激光 40 条、RGB 13 条、深度 15 条，所有时间戳均持续推进；不是只有进程而没有仿真数据。

- 最终启动日志：`runs/gazebo-startup-fix-20261023/one-click-final-1.log`、`one-click-final-2.log`。
- 对应栈日志：`runs/manual-20261023T021433/launch.log`、`runs/manual-20261023T021644/launch.log`。
- 验证汇总及源码摘要：`runs/gazebo-startup-fix-20261023/validation-summary.json`。
- 主窗口截图：`runs/gazebo-startup-fix-20261023/after-final-1.png`。
- 构建 mybot、tools_demo 成功；Shell 语法检查通过。
- 全量非 linter 回归：1758 passed、3 deselected；新测试文件 flake8 通过。
- 新增测试实际求值四种 GUI/headless 条件，检查失败 spawn 不开客户端；另执行原 Shell 图形门禁，验证只有启动画面时不能通过、场景完成后可通过、headless/标准模式不被专用插件标记误挡。

早期 `one-click-1/2` 是诊断服务器尚未退出时入口按原规则拒绝的记录；`one-click-3` 是修复日志缓冲前的诊断启动，已主动清理，不能算最终完整验收。最后两次 final 启动单独记录。没有出题或执行搬运任务；修改范围仅为启动行为，没有改变运动、评分或冻结基准目录。
