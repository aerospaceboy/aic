# 最终地图保护记录（2f49 逻辑迁移前）

> 建立时间：2026-08-04
> 目的：在把历史提交 `2f49f13` 的成熟任务逻辑迁移进来之前，锁定当前最终地图与场景布局。

## 受保护的状态

| 项 | 值 |
| --- | --- |
| 分支 | `aerospaceboy` |
| 当前 HEAD | `60054c8521f55a5766cc884b6667f253c87acf30` |
| HEAD 提交 | `feat(nav): restore per-cube docking for the 2026 distributed layout` |
| 保护标签 | `backup/final-map-20260804-60054c8` |
| 保护分支 | `backup/final-map-60054c8` |
| 历史基线（只读参考） | `2f49f1385d0ef3e41704a0bc359f600a4b69b805` |
| 历史基线提交 | `Harden navigation fallback and placement precision`（2026-08-03 12:27:54 +0800）|

## 正式加载的 world 与地图

| 文件 | SHA256 |
| --- | --- |
| `mybot_description/worlds/offic_room.world` | `8df2c5ebc35ffdd624c4a44a6c49c16410c8887966396e3fdebf5328e8b415d9` |
| `bot_navigation/maps/mapn3.pgm` | `2e7dfc3fa2a4766fcbafe45aaed5ef7166f00ee2bf60677fa437c5577edaf6dd` |
| `bot_navigation/maps/mapn3.yaml` | `8c54c38861e57a7179645c3775323ec3090f8d8a34e20fbd8d6bdae91d7d3e52` |

world 由 `mybot/launch/gazebo_world.launch.py` 的 `world_file_path = 'offic_room.world'`
加载，经 `mybot_description` 的 share 目录解析。

world 副本另存为 `world_backups/offic_room.world.60054c8`（仅作对照，**不参与构建**）。

## 当前最终布局（不可回退到旧集中布局）

- 红物块 x=-3，y = -2 / -5 / 1 / 3 / -7
- 蓝物块 x=-1，y = -2 / -4 / 1 / 3 / -6
- 每个物块一个独立托盘（`0.55 x 0.24 x 0.10`，yaw 90°）
- A(-10,-6)、B(4,-10)、C(-10,2) —— 三区分散且隔墙
- 机器人出生 (-2,-2)，yaw 1.153
- 两个移动障碍：obstacle_1 沿 (-4,0)<->(0,0)，obstacle_3 沿 (-5,-8)<->(-0.5,-8)

## 迁移期间的红线

`2f49f13` 只能作为**只读功能参考**。禁止：

- `git reset --hard` 到 2f49，或把工作区整体 checkout 成旧版本；
- 用旧 `room.world` 覆盖当前 world；
- 恢复旧的 A/B/C 集中布局或物块集中布局；
- 直接复用旧地图里的固定 x / y / yaw；
- 为了让旧逻辑跑起来而破坏当前最终地图，或删除固定场景元素；
- 绕开两个移动障碍物，或用减少任务步骤的方式伪造提速。

## 回滚方式

```bash
git checkout backup/final-map-60054c8    # 或 git checkout backup/final-map-20260804-60054c8
sha256sum mybot_description/worlds/offic_room.world   # 应等于上表值
```
