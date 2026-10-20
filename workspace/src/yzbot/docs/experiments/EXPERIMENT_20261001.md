# 2026-10-01 安全修复候选归档（未在当前版本生效）

这是失败实验及代码保存说明，不是当前系统配置，也不是48/48验收报告。
用户已要求回退到b21f8ad并做一次固定版本GUI48例；未经新的实施授权，
仅分析本文件，保持活动测试代码原样。

## 代码保存与归属

`20261001_safety_candidate.patch`保存回退前选定安全/导航源码、已有测试差异，
及8个新增回归测试。其源码基线是b21f8ad，目标是回退前stash849c5aeb。
该增量含回退前已有的部分导航差异，不能把所有行都归为本Agent原创。
它是候选归档，不是在当前测试版本上合入的补丁；未证明端到端可靠性。

原完整tracked/untracked改动仍可在VM恢复：

- stash对象：849c5aebf6726024cb1583454b59194f9262f98c。
- 原分支huxuyang：25bb81fab3df3e2f52f19d8d9b11d479b14c1769。
- 完整源码备份：/home/ros/huxuyang/YZBot-224/runs/rollback_b21f8ad_20261001T141656/source_before_rollback.tar.gz。
- 目标源码归档：同目录target_source.tar.gz；SHA256为
  d2bb0c88a3f6e5230dd73df0e7709db80610e4eea2cf8feecd6be0db2ea666e8。

## 实际新增修复及回归

1. `safety_limiter.py`缓存原始上游命令，20Hz重新评估、0.25秒过期丢弃，
   hold清空缓存，移除门控放行提高安全比例的下限。
2. `mission.launch.py`删除旧安全参数覆盖；候选采用0.82米/70度/0.35秒。
3. `avoidance_monitor.py`独立obstacle_hold与base_hold，先聚合最近障碍再更新hold。
4. `task_navigator.py`初始地图/action等待有60秒墙钟期限及非阻塞清图。
5. `navigation_math.py`穿越线y0.15改0.40，增加x7.2出口；速度不提。
6. 第二障碍提前退让相位门控；第一障碍限定下降y[-0.50,-0.35]放行。
7. 有界恢复候选过滤完整y[-0.8,0.1]扫掠范围；0.60米中心距保守底线。
8. 辅助倒退在取消Nav2前验证路径；不安全则保留Nav2控制，不发倒退。
9. 第一门控无条件承诺边界x9.5收紧x9.15，缓慢接近错过窗口时重新等待。

新增测试：test_safety_command_refresh、test_navigation_prerequisite_deadline、
test_obstacle_hold_isolation、test_spawn_crossing_replay、test_second_mover_crossing、
test_first_mover_phase、test_obstacle_reverse_admission、test_delayed_gate_commitment。
复现均先红再绿；最后候选功能套件648 passed,1 skipped,2 deselected，
tools_demo编译及选定lint/diff-check通过。恒速回放不覆盖Nav2曲线/接触/延迟。
BC旧契约还曾按两次实测修正（测试改动，不改运行策略），该项未包含本安全补丁。

## 实際GUI短批：全部未达到稳定两例通过

以下目录均位于/home/ros/huxuyang/YZBot-224/runs，顺序固定为1rA_4bB、2rA_3bC。

| 批次 | 第一例（秒/净空米） | 第二例（秒/净空米） | 通过 |
| --- | --- | --- | --- |
| policy_20261001T114841 | 303.2/-0.1481 | 235.5/-0.0324 | 0/2 |
| policy_20261001T120316 | 368.8/-0.0801 | watchdog400s/-0.1451 | 0/2 |
| policy_20261001T123537 | 293.7/0.0039 | 239.7/0.0574 | 0/2 |
| policy_20261001T125522 | 270.9/0.5017 | 223.8/-0.1606 | 1/2 |
| policy_20261001T131534 | watchdog400s/0.0109 | 207.7/0.5106 | 1/2 |
| policy_20261001T133405 | 336.4/-0.1008 | 191.2/0.4919 | 1/2 |
| policy_20261001T135243 | 677.5/-0.0277 | 152.4/0.2815 | 0/2 |

watchdog项的最后mission.elapsed是过时发布值，不当作真实总耗时。
短批source_guard.tar.gz只保存指定文件；不当作所有历史部署文件的完整冻结。

## 失败路径与结论

- T133405门放行后8秒仍停在(9.229,0.002)，进入曲线/慢行后Nav2 spin/backup，
  最终在(8.142,0.078)被第一障碍扫到；辅助倒退拒绝是有效的，但未解决闭环穿越。
- T135243仍在(8.539,0.530)停入第一障碍扫掠，净空-0.0277；之后BLUE_CUBE_2
  导航202.09秒失败、zoneB导航203.27秒，多个lethal start，不能全归因于相位。
- T135243第二例安全但RED_CUBE_2两次最终停靠航向漂移失败，最后completed1。
- 接受后的辅助倒退仍缺独立期限；当前版本也没有候选的准入修复。
- 仅扩大出口/盲调相位/恒速模拟通过不足以保证实际Gazebo安全。
- 临时只读观测脚本/tmp/yzbot_crossing_trace_20261001.py已准备但从未启动；
  用户回退请求覆盖了诊断重测计划，不能宣称取得速度/里程计延迟证据。

后续须先完成当前固定版本48例；需要实施修复时，先捕获实际cmd_vel链路、odom、
障碍真值、接触及停靠姿态，建立覆盖真实闭环的失败回归，再单变量修复/GUI重测。
