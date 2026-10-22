# YZBot ROS 2 工作区源码

环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11，x86_64。

项目完整部署、一键启动、出题和打包说明见项目根目录的 [README](../../../README.md)。

| 包 | 职责 |
|---|---|
| `mybot_description` | 机器人 URDF/Xacro、Gazebo 场景与资源 |
| `mybot` | MoveIt 配置、关节控制、视觉抓取与放置 |
| `bot_navigation` | Nav2 地图、路径控制、动态预测与安全区域 |
| `tools_demo` | 题目解析、视觉检测、任务协调、状态展示和验收 |
| `gazebo_debug_gui` | Gazebo 客户端显示与窗口设置 |
| `IFRA_LinkAttacher/linkattacher_msgs` | 抓取连接服务接口 |
| `IFRA_LinkAttacher/ros2_LinkAttacher` | Gazebo 夹持连接插件 |

从工作区目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

正式运行使用项目根目录的 `启动本版本.sh`，避免跳过凭据、图形窗口和安全链就绪检查。部署后配置当前用户 API Key，并使用 CoStudio 填写题目与映射规则。
