# CoStudio 实时场景与导航路径

桥接 launch 会启动只读节点 `competition_scene_markers`，在地图上显示机器人、十个物块、ABC 区、两个走廊移动障碍和两个移动隔墙。输出为 `/competition/scene_markers`（MarkerArray），地图别名为 `/competition/map`。

## 导航显示

- **黄色线** `navigation/planned`：Nav2 `/plan` 最新的全局规划路线，随重规划更新；导航到达目标、失败、取消或任务进入终态时清除；不会因 Nav2 暂时不重发规划而消失。切换目标时先清除旧线，新的规划到来后更新。
- **青色线** `navigation/traveled`：Gazebo 中 `six_arm` 的实际行驶轨迹，最多保留最近 1500 个点。它是仿真真值展示，不作为导航反馈。
- 新的 `task_id` 清空上一题轨迹；瞬移超过 2 m 或场景源中断超过 2 秒时清理旧轨迹，避免把重置前后的位置连起来。
- 场景最多 5 Hz 刷新，实际轨迹移动不足 0.06 m 不增加点；规划路线最多 1000 个点并保留终点。两条线随完整快照发布，避免 DELETEALL 误删路径。黄色线宽 0.14 m、完全不透明，青色线宽 0.035 m、透明度 0.7；黄线高于青线 0.08 m，重合时仍能分辨。

当前竞赛 world、地图和 odom 共享原点，map→odom 为恒等变换。路径仅接受 map/odom 坐标；其他坐标系不显示，不能直接改 frame_id 来冒充转换。将来换平台或地图原点时需要真实坐标变换。

## 在 CoStudio 中使用

沿用比赛启动脚本即可，无需再启动一套桥接。连接现有 WebSocket（主机客户端使用 VM IP，端口由启动日志确认；当前默认 21274），在三维地图面板勾选 `/competition/map` 和 `/competition/scene_markers`，固定坐标系选 `map`。已有布局启用了 scene_markers，新路径会自动出现。

新导入布局可选 `tools_demo/config/costudio_deployment_layout.json` 或 `costudio_reference_layout.json`。布局隐藏重复的 `/plan` 直接显示；局部规划 `/local_plan` 可以按需关闭。标题标明黄线规划、青线已走轨迹。场景标记的 namespaces 中可单独隐藏 `navigation/planned` 或 `navigation/traveled`。

启动后先输入题目和映射规则，确认解析内容，再执行任务。导航开始后观察黄线随规划变化、青线随机器人移动增长。完成后青线保留供回看，黄线清除；提交下一题后青线重置。

## 检查与边界

单元检查覆盖路线与轨迹区分、点数上限、终点保留、新题重置、瞬移、终态及错误坐标系。运行验证另见本轮 GUI_CORRIDOR_PATH_20261010 报告；ROS 数据和桥接连通验证不能替代人工确认客户端渲染。

该节点不发布速度、目标、控制服务，也不改变地图、碰撞体、导航参数或真实障碍输入。墙钟 2 秒无 Gazebo 状态时清空标记，避免显示冻结位置。
