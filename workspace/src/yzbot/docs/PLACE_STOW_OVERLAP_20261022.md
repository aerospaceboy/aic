# 放置后底盘后退与空载收臂并行（2026-10-22）

## 版本快照

在修改前保存了只读开发快照：

- `YZBot-224-DEV-20261022T072006`
- 目录：`/home/ros/ceshi2/项目分析/版本快照/YZBot-224-DEV-20261022T072006`
- 包：同目录下 `YZBot-224-DEV-20261022T072006.tar.gz`
- SHA256：`8a5ed4055d8ff131294b0cc668a6b0c11e83fb1412a4ae8fd8ff69d9a4add041`

这是“修改前源码开发快照”，标记为 `development_snapshot_NOT_accepted_baseline`。官方 `scripts/09_freeze_baseline.sh` 要求当前源码对应的 R1/R2/R3 三套规则证据、通信验收和 `validated_sources.sha256`；现有收据属于更早源码，因此没有伪造官方冻结基准。

## 改动

`mybot/pick_place_moveit.py`：

- 新增 `defer_place_stow` 参数；persistent worker 在 `mission.launch.py` 中启用。
- `run_place_only` 在“放置证明完成、夹爪打开、cube 已 detach、已释放 base”之后，先发布 `place_ready:<cube>` 并返回成功。
- 持久服务循环在发布 `persistent_result` 后立即调用 `finish_deferred_place_stow()`，执行空载收臂。
- 收臂期间仍持有 operation lock，因此下一次 `scan/pick/place/release` 请求不会与 MoveIt 轨迹并发。
- 收臂只包含 joint-limit guard 和 `return_home_after_world_slot`，不会重跑夹爪、detach 或放置路径。
- 收臂超时仍执行原有回退和 deferred stow 语义；已完成的交付不会因为收臂失败被改判为失败。

`tools_demo/launch/mission.launch.py`：persistent pick-place 节点设置 `"defer_place_stow": True`。

## 验证

- 构建：`mybot`、`tools_demo`、`bot_navigation` 全部成功。
- 回归：`1748 passed, 3 deselected`。
- 完整 AB 规则轮 `OVERLAP-1`：
  - 5/5，0 跳过
  - **236.705 秒**
  - `safe=true`，`collision_detected=false`，最小净空 **0.1428 米**
  - 3 个有效响应

五次放置的时序：

| 放置 | `place_ready` | 收臂耗时 | 底盘退出启动延迟 |
|---|---:|---:|---:|
| red_cube_1 | 0.00 s | 2.520 s | 0.005 s |
| red_cube_4 | 33.42 s | 2.473 s | 0.005 s |
| blue_cube_3 | 77.58 s | 2.530 s | 0.005 s |
| blue_cube_1 | 124.83 s | 2.523 s | 0.006 s |
| red_cube_2 | 162.73 s | 2.483 s | 最终放置，无退出段 |

底盘在 `place_ready` 后约 5 毫秒开始退出，收臂约 2.5 秒后完成，说明两者确实并行。

与修改前 AB8 八轮均值 **258.646 秒**相比，这一轮少 **21.941 秒（约 8.5%）**；与 AB8 最快轮 251.725 秒相比少 **15.020 秒**。这是单轮对比，障碍相位和后续路径仍有波动，不能把 21.9 秒全部归因于本次改动；还需要多轮复测。

## 边界

- 第二次完整复跑在启动阶段被大模型预检超时挡住，属于外部 API 问题，没有进入任务；保留失败日志。
- 当前只完成 1 个成功的完整并行恢复轮，需继续多轮确认收臂失败、下一次抓取排队和不同放置区下的稳定性。
- 若收臂失败，下一次操作仍依赖原有 joint-limit guard 和扫描位恢复；本条未新增“收臂失败后禁止下一次抓取”的新硬门。
