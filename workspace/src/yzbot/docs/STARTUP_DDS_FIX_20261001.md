# 2026-10-01 启动后题目无响应：DDS配置修复

## 实测原因

`policy_20261001T211326`及单例`policy_20261001T224135`中，启动器返回
PASS，但测试进程每60秒重发题目，始终收不到mission、plan和avoidance。
启动器在子Shell中导出Fast DDS loopback UDP配置，不能改变Python父进程
的环境。默认DDS查询仅发现7个基础话题；使用项目UDP配置后发现完整竞赛图。
因此这不是证明大模型API请求失败的日志，而是测试端通信配置不一致。

发现并核对22个属于当前workspace、父PID为1的旧轮次节点，已定向TERM，
未删除数据或终止其他项目进程。残留节点是额外风险，不替代DDS根因证据。

## 修改

- `tools_demo/tools_demo/runtime_environment.py`：验证安装的DDS配置存在，
  保留用户ROS_DOMAIN_ID，设置与启动器一致的RMW和DDS配置。
- `tools_demo/scripts/run_zone9_campaign.py`：在导入rclpy前初始化上述环境；
  不依赖外部手工export，不修改任务策略、290秒/0.10米验收及GUI设置。
- `tools_demo/test/test_campaign_transport.py`：测试导入顺序、真实workspace路径、
  配置匹配/域保留及缺少配置时提前失败。

## 验证

新增测试修复前3失败；修复后与startup_race_contract共6通过。
tools_demo的colcon build --symlink-install成功，新增模块/测试flake8、
Python编译及git diff --check通过。
功能套件（排除flake8/pep257）566通过、5失败、1跳过、2取消选择；
5失败为b21已有BC策略契约，未放宽断言或更改策略。

实际GUI单例刻意unset FASTRTPS_DEFAULT_PROFILES_FILE和RMW_IMPLEMENTATION后
运行修复入口：`POLICY_CASES=3rA_2bB COSTUDIO_HEADLESS=False COSTUDIO_RVIZ=True`。
116秒启动成功，提交约1秒后任务接收，202.9秒完成5/5，原始passed=True。
原始证据：`/home/ros/huxuyang/YZBot-224/runs/policy_20261001T232305/`；
驱动日志：`runs/startup_dds_smoke_20261001.log`。

这是一次启动通信修复和1例物理回归，不是完整48/48认证。
当前HEAD仍b21f8ad；以上为未提交改动，不提交推送，保留其他用户文件。
