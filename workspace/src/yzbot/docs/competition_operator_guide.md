# 比赛操作说明

完整部署命令见项目根目录 [README](../../../../README.md)。

1. 在 Ubuntu 图形桌面进入项目目录，执行 `bash 启动本版本.sh`。
2. 等待 `MANUAL_PROBLEM_MODE=PASS`，确认 Gazebo 主窗口、RViz、CoStudio 和网页面板正常。
3. CoStudio 使用 coScene WebSocket 连接 `ws://127.0.0.1:21274`，导入根目录 `costudio/YZBot比赛布局.json`。
4. 正式出题执行 `bash scripts/06_generate_question.sh`，练习时追加 `--practice`。
5. 填写题目正文及对应映射规则，检查后确认提交。API 校验失败、题目不一致或安全链未就绪时，应处理故障后重新开始。
6. 观察视觉目标、导航路径、夹取、运输和区域投放；机器人自动选择合法物块与任务顺序。
7. 结束后检查五件物块全部完成、无跳过、真实模型解析、视觉取件来源、实际落区、用时和避障记录。没有自然触发的有效避障指标应如实保留。

网页面板地址：`http://127.0.0.1:8080/`。

任务执行中不要重复启动仿真。查看当前日志使用启动输出中的路径；环境连接检查使用 `bash scripts/07_check_connection.sh`。停止本项目栈可在项目目录、正确加载环境后执行：

```bash
source scripts/lib.sh
load_ros
CLEAN_ONLY=True bash workspace/src/yzbot/tools_demo/scripts/clean_launch.sh
```

该清理脚本会终止机器人仿真栈，不能在其他任务执行中使用。

API Key 位于用户私有目录，不应出现在源码、提交 ZIP、截图或视频中。发布包不包含 Key，目标机器需单独配置。
