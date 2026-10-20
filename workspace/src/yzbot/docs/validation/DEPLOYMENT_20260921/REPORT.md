# 2026-09-21 VM 部署与 CoStudio 修复

从 huxuyang 335bbbb 快进到 304c86d；不是不存在的 huxvyang。
部署前完整源码与未提交修改已备份到主机工作区 deployment-20260921。

## 修复

- 安装分支配套 1.4.4 扩展，原 1.5.0 只提供 mission-overview，导致当前布局 Unknown panel type；原扩展备份保留。
- 一键启动完整系统与 coBridge，21274 恢复监听；入口直接解析当前工作区环境，去除仓库外 scripts/00_env.sh 依赖。
- 启动输出不再在网页未启用时宣称 8080 可访问。
- coBridge 订阅白名单加入 parse_result、current_goal，客户端发布权限保持仅 problem_chunk。
- 增加 costudio_deployment_layout.json，使用两路物理相机、实时模式、场景标记，移除无后端订阅者的旧 mission/start 按钮，增大题目面板。
- 修正扩展小面板输入区顶部被裁切，以及未解析时目标区卡虚显 A/C；保留 dist 并新增依赖锁文件。
- 更新 CoStudio README 与 VM 桌面操作说明和启动入口。

## 验证

7 个 ROS 包构建成功；Python 555 passed, 1 skipped, 2 deselected（原有 flake8/pep257 未运行）。
修复入口启动通过 MANUAL_PROBLEM_MODE=PASS，collision_monitor active。
Windows 实际 TCP 会话连到 21274；CoStudio 状态、地图、动态场景标记和双相机图像已收到。
完整比赛、API 真请求与 48 题性能本轮未重跑，不以连接恢复宣称比赛验收。

## 清理与保留

旧备份/打包解压副本/早期实验归档副本/过期 Gazebo 和 ROS 日志先归档到 Windows，SHA256 校验后移出 VM。
清理 639 个明确目录及 pip 缓存；保留当前活动 server-11345 日志。系统 journal 仅清理超过 14 天的归档。
可用磁盘从约 9.3 GB 增至 12 GB；~/runs 与 dev_ws/experiments 包括 coverage-48 全部保留。
代码、依赖、模型、官方出题程序、API 凭据、Git 历史和个人文件均保留。
原 GUI 默认帧率 60 的本地修改及四份未跟踪旧交接文档保留，未混入本提交。

## 已知边界

3D 面板自动 advertise 导航/点选话题会被白名单拒绝，提示 not whitelisted；这是控制权限限制，不影响订阅。未放开任何导航/重定位控制。
Windows 旧 224 交付包本轮未替换。系统当前等待题目，不会自动搬运。
