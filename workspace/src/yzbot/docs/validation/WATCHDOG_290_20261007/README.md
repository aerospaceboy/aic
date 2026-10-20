# 290 秒门槛 · 全 48 题串行验收（冻结版本 f8074d4）

本目录是**新比赛规则**（取消颜色执行顺序、跨色可交错、配额与实体唯一性、每件正确取放 4 分、5 件 20 分、
同一实体不得复用、取放不得重复计分）下，在**单一冻结版本**上串行完成 48 题的证据与规范报告。

## 0. 结论摘要

- `check48_290.py` 判定（本机复跑，退出码 0）：

```
{
  "expected": 48,
  "rows": 48,
  "accepted": 48,
  "failing": []
}
EXIT=0
```

- **48/48 全部达标**：每题 5/5、`skipped=0`、`goods_score=20`、
  `requirements_verified` / `pickup_delivery_consistent` / `truth_precise` 均为 true；
  四移动体全程在线（4/4 位姿更新）；真实相遇且报告**两个不同移动体**的响应、并单独给出**原始双让行**（`dual_yield`）；
  零碰撞；最小体表净空严格 > 0.10 m；`motion_under_290` / `motion_functional_pass` / `CASE_ACCEPTED` 均 true。
- **首轮通过率 46/48（95.8%）**；两次未过均可归类，同版本复跑即通过（见 §5）。
- **最差用时 284.3 s**，用时中位数 220.6 s（范围 172.8–284.3 s）；**最小净空 0.1427 m**（范围 0.1427–0.5806 m）。
- 全程 **零 SAFETY 失败**（无净空 ≤ 0.10 m、无碰撞）。

## 1. 冻结版本与保护

| 项 | 值 |
|---|---|
| commit | `f8074d4`（master） |
| 前序修复 | `c9154fd`（车道放行下限 1.00 m + 斜墙三条规划禁行带）、`f47e45c`（卡死更早升级到有界恢复、retry 预算 2→1） |
| backup ref | `refs/backup/pre-290-watchdog-20261007` → `f8074d4` |
| 冻结 patch | `/home/ros/frozen-290-watchdog-20261007T031757Z.patch`，sha256 `aedee84946e154a5b2f32af92a80441f9828a992ac2e8eb9fa3d33f18ef3106c` |
| 源码 manifest sha256 | `9604ad03fee7e4937f80c0aaad7baca82e4dd573de622ad11937d3709ecc2a12` |
| 版本指纹（HEAD + worktree diff） | `ffdfdec42df22fa004b4851c8280d4bd5bec6d40e859a944e195b94e1d6041c8` |
| 证据根 | `/home/ros/dev_ws/experiments/full48-290-20261007T032500Z` |
| 运行参数 | `run48_v2.sh 1 48 ordered`（串行）+ 同版本收敛链 `run_converge_290_v2.sh`，`RLC_TIMEOUT_SEC=600`（仅诊断用外层上限） |
| 测试 | `python3 -m pytest tools_demo/test -q -m 'not linter'` → **681 passed, 3 deselected** |
| 构建 | `colcon build --packages-select tools_demo` → 成功 |

> 用户未提交的两个文件（`bot_navigation/launch/competition_navigation.launch.py` 支撑轮静态 TF、
> `gazebo_debug_gui/src/gazebo_debug_gui_plugin.cpp`）全程未动，其内容即冻结 patch 的 sha256 来源。
> `refs/backup/...`、`/home/ros/*.patch` 与 `/home/ros/dev_ws/experiments/` 位于虚拟机内，不随仓库分发；
> 逐题证据目录中的 `commit.txt` / `worktree.patch` / `source-config.sha256` 可用于核对。

## 2. 本版本为 290 s 门槛做的三处修复（均有定向证据，未放宽任何安全/净空/响应门槛）

1. **车道放行下限 0.75 → 1.00 m**（`c9154fd`）：`2rB_3bA` 曾出现 0.0892 m 净空（8.1 折放行）→ 复测 231.7 s / 0.221 m 全门通过。
2. **斜墙 `(5.5,-0.7)→(9.0,-1.4)` 三条全局规划禁行带**（`c9154fd`）：`1rA_4bB` 因 33.2 s 卡死跑到 295.8 s → 复测 279.3 s 无卡死。
3. **进度看门狗不再受 retry 预算门控 + 卡死升级到有界恢复**（`f8074d4`，本轮新增）：
   `timeout_callback` 原判定带 `self.nav_retries < self.nav_retry_attempts`；在 `nav_retry_attempts: 1` 下，
   第一次卡死耗尽唯一 retry 后看门狗对整条腿失效，第二次卡死只能盲等 Nav2 放弃。
   逐腿画像（旧批次 `4bB_1rA` 315.2 s 尝试）：前 14 条腿全部 succeeded，最后两条腿 `timed_out:65.9` + `failed:33.9`
   （100 s 盲等）才触发一次有界恢复。现改为：看门狗全程有效；retry 用尽后 `result_callback` 直接升级到与
   aborted/failed 分支同一套 `_try_bounded_recovery()`。边界不变：retry 仍 1 次、`max_recovery_attempts` 仍 2、
   脱困段仍须通过 `_recovery_segment_clear_of_movers`（不得横穿移动体车道）。

**未放宽的安全边界**：未删除任何障碍、未缩小 footprint、未放宽 0.10 m 净空 / 两次响应 / 原始双让行 / 290 s 任何门槛；
`tools_demo/test/test_navigation_contract.py` 相应断言"看门狗不受 retry 预算门控"且"retry → 有界恢复"的顺序。

## 3. 48 行规范报告

字段：tag、题目、完成/跳过、商品分、5/5+精确投放、四移动体在线、两次响应/原始双让行、碰撞、最小净空(m)、
运动秒数、<290 标记、尝试次数、失败原因；另有 `requirements_verified` / `pickup_delivery_consistent` /
`truth_precise` / `motion_functional_pass` / `CASE_ACCEPTED`。机器可读版本见 `report48_full.csv`。

| # | tag | 题目 | 完成/跳过 | 商品分 | 5/5+精确 | 四移动体在线 | 两次响应/原始双让行 | 碰撞 | 最小净空(m) | 运动秒数 | <290 | 尝试 | 失败原因 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 1rA_4bB | Move 1 red cube to zone A and 4 blue cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2461 | 284.3 | True | 1 | — |
| 2 | 4bB_1rA | Move 4 blue cubes to zone B and 1 red cube to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3312 | 281.4 | True | 1 | — |
| 3 | 2rA_3bB | Move 2 red cubes to zone A and 3 blue cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2350 | 257.3 | True | 1 | — |
| 4 | 3bB_2rA | Move 3 blue cubes to zone B and 2 red cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2563 | 248.1 | True | 1 | — |
| 5 | 3rA_2bB | Move 3 red cubes to zone A and 2 blue cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.4169 | 228.5 | True | 1 | — |
| 6 | 2bB_3rA | Move 2 blue cubes to zone B and 3 red cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.4376 | 235.0 | True | 1 | — |
| 7 | 4rA_1bB | Move 4 red cubes to zone A and 1 blue cube to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3697 | 215.2 | True | 1 | — |
| 8 | 1bB_4rA | Move 1 blue cube to zone B and 4 red cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.4746 | 220.6 | True | 1 | — |
| 9 | 1rA_4bC | Move 1 red cube to zone A and 4 blue cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5342 | 188.2 | True | 1 | — |
| 10 | 4bC_1rA | Move 4 blue cubes to zone C and 1 red cube to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5585 | 192.8 | True | 1 | — |
| 11 | 2rA_3bC | Move 2 red cubes to zone A and 3 blue cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3116 | 201.0 | True | 1 | — |
| 12 | 3bC_2rA | Move 3 blue cubes to zone C and 2 red cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3117 | 184.0 | True | 1 | — |
| 13 | 3rA_2bC | Move 3 red cubes to zone A and 2 blue cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3435 | 172.8 | True | 1 | — |
| 14 | 2bC_3rA | Move 2 blue cubes to zone C and 3 red cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5291 | 182.4 | True | 1 | — |
| 15 | 4rA_1bC | Move 4 red cubes to zone A and 1 blue cube to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2950 | 191.2 | True | 1 | — |
| 16 | 1bC_4rA | Move 1 blue cube to zone C and 4 red cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.4395 | 208.2 | True | 1 | — |
| 17 | 1rB_4bA | Move 1 red cube to zone B and 4 blue cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3041 | 232.6 | True | 1 | — |
| 18 | 4bA_1rB | Move 4 blue cubes to zone A and 1 red cube to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2310 | 251.1 | True | 1 | — |
| 19 | 2rB_3bA | Move 2 red cubes to zone B and 3 blue cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.1842 | 260.7 | True | 1 | — |
| 20 | 3bA_2rB | Move 3 blue cubes to zone A and 2 red cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2491 | 255.2 | True | 1 | — |
| 21 | 3rB_2bA | Move 3 red cubes to zone B and 2 blue cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3797 | 247.4 | True | 1 | — |
| 22 | 2bA_3rB | Move 2 blue cubes to zone A and 3 red cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.1427 | 264.6 | True | 2 | — |
| 23 | 4rB_1bA | Move 4 red cubes to zone B and 1 blue cube to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.4665 | 256.4 | True | 1 | — |
| 24 | 1bA_4rB | Move 1 blue cube to zone A and 4 red cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2536 | 255.5 | True | 1 | — |
| 25 | 1rB_4bC | Move 1 red cube to zone B and 4 blue cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3442 | 188.6 | True | 1 | — |
| 26 | 4bC_1rB | Move 4 blue cubes to zone C and 1 red cube to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3029 | 189.9 | True | 1 | — |
| 27 | 2rB_3bC | Move 2 red cubes to zone B and 3 blue cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3590 | 193.2 | True | 2 | — |
| 28 | 3bC_2rB | Move 3 blue cubes to zone C and 2 red cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2699 | 190.6 | True | 1 | — |
| 29 | 3rB_2bC | Move 3 red cubes to zone B and 2 blue cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2103 | 217.7 | True | 1 | — |
| 30 | 2bC_3rB | Move 2 blue cubes to zone C and 3 red cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.1849 | 219.8 | True | 1 | — |
| 31 | 4rB_1bC | Move 4 red cubes to zone B and 1 blue cube to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5806 | 256.0 | True | 1 | — |
| 32 | 1bC_4rB | Move 1 blue cube to zone C and 4 red cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.4426 | 242.3 | True | 1 | — |
| 33 | 1rC_4bA | Move 1 red cube to zone C and 4 blue cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2107 | 223.6 | True | 1 | — |
| 34 | 4bA_1rC | Move 4 blue cubes to zone A and 1 red cube to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2058 | 239.2 | True | 1 | — |
| 35 | 2rC_3bA | Move 2 red cubes to zone C and 3 blue cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5692 | 217.1 | True | 1 | — |
| 36 | 3bA_2rC | Move 3 blue cubes to zone A and 2 red cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3307 | 220.4 | True | 1 | — |
| 37 | 3rC_2bA | Move 3 red cubes to zone C and 2 blue cubes to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5379 | 190.2 | True | 1 | — |
| 38 | 2bA_3rC | Move 2 blue cubes to zone A and 3 red cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2916 | 188.0 | True | 1 | — |
| 39 | 4rC_1bA | Move 4 red cubes to zone C and 1 blue cube to zone A. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2794 | 200.2 | True | 1 | — |
| 40 | 1bA_4rC | Move 1 blue cube to zone A and 4 red cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.1977 | 215.0 | True | 1 | — |
| 41 | 1rC_4bB | Move 1 red cube to zone C and 4 blue cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3080 | 263.1 | True | 1 | — |
| 42 | 4bB_1rC | Move 4 blue cubes to zone B and 1 red cube to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3720 | 255.3 | True | 1 | — |
| 43 | 2rC_3bB | Move 2 red cubes to zone C and 3 blue cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2476 | 230.5 | True | 1 | — |
| 44 | 3bB_2rC | Move 3 blue cubes to zone B and 2 red cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5269 | 232.2 | True | 1 | — |
| 45 | 3rC_2bB | Move 3 red cubes to zone C and 2 blue cubes to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.3131 | 220.7 | True | 1 | — |
| 46 | 2bB_3rC | Move 2 blue cubes to zone B and 3 red cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2132 | 207.1 | True | 1 | — |
| 47 | 4rC_1bB | Move 4 red cubes to zone C and 1 blue cube to zone B. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.2331 | 212.6 | True | 1 | — |
| 48 | 1bB_4rC | Move 1 blue cube to zone B and 4 red cubes to zone C. | 5/0 | 20 | True/True | 4/4 | True/True | False | 0.5305 | 235.1 | True | 1 | — |

## 4. 用时与净空分布

| 指标 | 值 |
|---|---|
| 运动用时中位数 | 220.6 s |
| 运动用时范围 | 172.8 – 284.3 s |
| 超过 270 s 的题 | `1rA_4bB` 284.3 s、`4bB_1rA` 281.4 s（均在 290 内） |
| 最小净空 | `2bA_3rB` 0.1427 m |
| 净空 < 0.20 m 的题 | `2bA_3rB` 0.1427、`2rB_3bA` 0.1842、`2bC_3rB` 0.1849、`1bA_4rC` 0.1977（均 > 0.10） |
| 尝试次数分布 | 1 次：46 题；2 次：2 题（`2bA_3rB`、`2rB_3bC`） |

## 5. 全部失败记录

**本轮证据根（f8074d4）共 50 次尝试、48 次合格，2 次不合格尝试全部保留原始目录（`evidence-failures/`）**：

| # | tag | 尝试 | 失败类别 | 用时 | 净空 | 原始证据 |
|---|---|---|---|---|---|---|
| 1 | 2bA_3rB | 1 | TIME_ONLY：任务完成但 302.3 s 超 290 s 门槛（CASE_ACCEPTED=true、双响应、无碰撞） | 302.3 s | 0.5140 | 2bA_3rB/20261007T051424Z-xTM6lW |
| 2 | 2rB_3bC | 1 | INFRA：`move_group` 段错误导致 launch 全灭、任务从未启动（elapsed 0.0、state idle） | 0.0 s | 4.2425（未接近） | 2rB_3bC/20261007T054310Z-LJ3F6s |

两次复跑结果：`2bA_3rB` → 264.6 s / 0.1427 m 通过；`2rB_3bC` → 193.2 s / 0.3590 m 通过。

**上一证据根（`c9154fd`，`full48-290-20261006T213708Z`）的失败记录**（完整清单见 `previous-root-failures.log`）：
`1rA_4bB` 与 `2rC_3bB` 为 INFRA（任务未启动），`3bA_2rC` 与 `4bB_1rA` 为 ROBOT/TIME_ONLY；该根收敛到 47/48
（仅 `4bB_1rA` 未过）后被本轮版本取代——`4bB_1rA` 在新版本**首次尝试**即 281.4 s / 0.3312 m 通过。

## 6. 「48 题都有合格证据」与「重复稳定通过」分别说明

### 6.1 48 题都有合格证据（逐题可核）

- 每个 tag 在 `evidence/<tag>/` 下都有完整一套：`audit.json`（评分与验收判据）、`result.log`
  （`CASE_ACCEPTED` / `MISSION_TERMINAL` / `AVOIDANCE_EVIDENCE` / `MINIMUM_SURFACE_CLEARANCE`）、
  `commit.txt`、`question.txt`、`source-config.sha256`（源码与配置清单）、`worktree.patch`、`runner-exit-code.txt`。
- 逐题 14 项判据全部通过（`report48_attempts.py` 输出 `contract_problems: []`）：5/5、skipped=0、goods_score=20、
  requirements_verified、pickup_delivery_consistent、truth_precise、四移动体在线（`obstacle_updates` 四个模型计数均 > 0）、
  two_dynamic_responses、dual_yield、无碰撞、净空 > 0.10 m、motion_under_290、motion_functional_pass、CASE_ACCEPTED。
- 汇总判据 `check48_290.py`：`{"expected": 48, "rows": 48, "accepted": 48, "failing": []}`，退出码 0。

### 6.2 重复稳定通过（同版本多次运行）

**定向验证批次（`targeted290-watchdog-20261007`，同源码 manifest，5 次运行）**：

| tag#n | el | clr | u290 | func | accepted | resp2 | dual |
|---|---|---|---|---|---|---|---|
| 1rA_4bB#1 | 269.8 | 0.2589 | True | True | True | True | True |
| 2rB_3bA#1 | 256.7 | 0.3361 | True | True | True | True | True |
| 3bA_2rC#1 | 264.3 | 0.3898 | True | True | True | True | True |
| 4bB_1rA#1 | 265.3 | 0.2868 | True | True | True | True | True |
| 4bB_1rA#2 | 269.6 | 0.2151 | True | True | True | True | True |

**正式批次内**的同版本重复通过：`2bA_3rB`（302.3 → 264.6 s）与 `2rB_3bC`（INFRA → 193.2 s）各两次尝试后通过。

**第三方独立重复跑（`repeat290-frozen-20261007`，同版本指纹 `ffdfdec4…`，6 次运行）**：

| tag#n | el | clr | u290 | func | accepted | resp2 | dual |
|---|---|---|---|---|---|---|---|
| 1rA_4bB#1 | 266.8 | 0.2503 | True | True | True | True | True |
| 2bA_3rB#1 | 247.4 | 0.2626 | True | True | True | True | True |
| 2bA_3rB#2 | 254.3 | 0.2875 | True | True | True | True | True |
| 2rB_3bC#1 | 200.5 | 0.2924 | True | True | True | True | True |
| 2rB_3bC#2 | 199.1 | 0.4924 | True | True | True | True | True |
| 4bB_1rA#1 | 284.3 | 0.5412 | True | True | True | True | True |

结论：**48 题全部有合格证据**（§6.1）；**关键题与最紧的题在同版本上重复通过**（§6.2），不是靠放宽判据或删除障碍得到的。

## 7. 尚未验证（如实标注）

- 真实 LLM API（题目生成与评分）端到端：**未测**（本批次用固定 48 题，`full_competition_acceptance: "not_tested"`）；
- CoStudio 展示链路：**未测**；
- GUI 可见运行（录屏/人工观察）：**未测**；
- 竞赛全程真实墙钟（含出题、展示、评分、裁判交互）：**未测**。
- 290 s 为工程余量，运动计时不含出题/API/展示；运行器 600 s 仅作诊断。

## 8. 目录内容

| 文件 | 说明 |
|---|---|
| `README.md` | 本文件 |
| `report48_full.csv` / `report48_full.md` / `report48_full.json` | 48 行规范报告（含全部要求字段） |
| `report48_attempts.csv` / `report48_attempts.json` | 50 次尝试逐条记录、失败清单、逐题 14 项判据自检 |
| `report48_290.csv` / `final_table.csv` | 批次脚本原始报告与汇总表 |
| `check48-290-verdict.txt` | `check48_290.py` 判定与退出码 |
| `campaign.log` | 收敛链完整日志（两轮 PASS 的 SKIP/DONE/FAILED 与裁决） |
| `runtime-receipt.json` | 冻结版本运行收据（HEAD/patch/manifest/证据根/参数） |
| `tests.log` / `build.log` | 测试与构建记录 |
| `evidence/<tag>/` | 48 题逐题证据（判定所依据的那一次） |
| `evidence-failures/` | 2 次不合格尝试的原始证据（未删除） |
| `evidence-targeted/` | 5 例定向验证证据（看门狗修复） |
| `previous-root-failures.log` | 上一证据根的失败记录与收敛尾部 |
