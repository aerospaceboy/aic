# 优化验证入口

正式比赛继续使用项目根目录的 `bash 启动本版本.sh`。它会启动比赛后端和 CoStudio 客户端。

批量调试使用独立入口，启动同一真实模型、视觉、导航、机械臂、安全链和 coBridge 服务，不自动打开 CoStudio 客户端：

```bash
cd /home/ros/ceshi2/ceshi
bash scripts/09_start_validation.sh
```

默认打开 Gazebo 和 RViz，启用网页状态面板、摄像头流和 SLAM 证据。入口拒绝与已有 Gazebo/机器人栈重叠启动。先保存上一轮结果，再正常清理：

```bash
source scripts/lib.sh
load_ros
CLEAN_ONLY=True bash workspace/src/yzbot/tools_demo/scripts/clean_launch.sh
```

等待新日志中的完整行 `MANUAL_PROBLEM_MODE=PASS` 后再提交新题目。每轮使用新的输出目录和 task_id，不能复用上一轮的就绪日志。状态面板为 `http://127.0.0.1:8080`；coBridge 服务仍为 `ws://127.0.0.1:21274`。

需要无 Gazebo 图形客户端的实验时，可显式设置：

```bash
YZBOT_VALIDATION_HEADLESS=True YZBOT_VALIDATION_RVIZ=False bash scripts/09_start_validation.sh
```

当前后端仍检查本机图形会话。无界面与有界面成绩分开记录；最终验收包含有界面运行。不要用 `scripts/05_start_debug.sh` 替代正式验收，该入口关闭真实模型要求。

源码变更后先构建再启动；机械臂程序和世界文件即使使用 symlink-install 也可能安装为副本。行为测试：

```bash
source scripts/lib.sh
load_ros
python3 -m pytest -q -p no:cacheprovider -m 'not linter' \
  workspace/src/yzbot/tools_demo/test workspace/src/yzbot/mybot/test
```

完整验收还需独立检查真实模型解析、五件视觉抓取与正确落区、300 秒墙钟、当前任务两次不同障碍有效响应及碰撞记录。单元测试或仅显示 completed 不代表完整通过。
