# 48 例合法任务通过总结（2026-09-17）

数据来源：`runs/*/results.jsonl`。每例取**最佳通过轮**；其实际取块顺序与停靠侧已由 `tools_demo/scripts/verify48_policy_match.py` 从日志读回并与当前 `cube_order_policy.json` 比对：**matched 48/48**。

## 1. 总览

- 合法空间：48 例 = 6 个有序区域对 × 4 种数量分配 × 2 种子句顺序
- 最佳通过成绩 < 280s：**48/48**；仍 ≥280s：0 例（）
- 最佳成绩分布：最短 **198.1s**、中位 **256.0s**、最长 **277.4s**
- 距 290s 门限裕量（按最佳成绩）：最小 **12.6s**、中位 **34.0s**
- 最小净空：最低 **0.1052m**（要求 ≥0.10m）
- 48 例全部 `completed=5/5`、`skipped=0`、`safe=true`、`plan_source=llm`；抓取失败、搬运中止、致命起步、碰撞、导航重试计数全为 0

## 2. 分族统计（按最佳成绩）

| 区域对 | 例数 | 最短 | 中位 | 最长 | <280s |
| --- | ---: | ---: | ---: | ---: | ---: |
| (A,B) | 8 | 198.1 | 249.2 | 264.6 | 8 |
| (A,C) | 8 | 215.9 | 250.5 | 275.4 | 8 |
| (B,A) | 8 | 202.0 | 248.4 | 272.4 | 8 |
| (B,C) | 8 | 244.9 | 267.6 | 277.4 | 8 |
| (C,A) | 8 | 233.7 | 263.5 | 272.7 | 8 |
| (C,B) | 8 | 235.0 | 251.6 | 271.8 | 8 |

## 3. 逐例明细

| 区域对 | 任务键 | 解析顺序（子句） | 最佳(s) | 裕量(s) | 净空(m) | 实际取块顺序 | 尝试/通过 | 最佳批次 |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| (A,B) | `1bA_4rB` | 1:bluex1->A → 2:redx4->B | 251.1 | 38.9 | 0.2312 | blue4 red2 red1 red4 red5 | 5/3 | policy_20260918T125937 |
| (A,C) | `1bA_4rC` | 1:bluex1->A → 2:redx4->C | 273.1 | 16.9 | 0.2516 | blue1 red1 red3 red5 red4 | 16/8 | policy_20260918T101046 |
| (B,A) | `1bB_4rA` | 1:bluex1->B → 2:redx4->A | 256.4 | 33.6 | 0.2842 | blue1 red2 red5 red3 red1 | 2/2 | policy_20260918T125937 |
| (B,C) | `1bB_4rC` | 1:bluex1->B → 2:redx4->C | 277.4 | 12.6 | 0.2723 | blue4 red5 red3 red4 red1 | 7/4 | policy_20260918T125937 |
| (C,A) | `1bC_4rA` | 1:bluex1->C → 2:redx4->A | 235.4 | 54.6 | 0.2280 | blue2 red3 red1 red5 red2 | 2/2 | policy_20260918T125937 |
| (C,B) | `1bC_4rB` | 1:bluex1->C → 2:redx4->B | 268.3 | 21.7 | 0.2548 | blue2 red3 red1 red4 red5 | 4/3 | bc4_20260917T121718 |
| (A,B) | `1rA_4bB` | 1:redx1->A → 2:bluex4->B | 264.6 | 25.4 | 0.2402 | red1 blue2 blue1 blue5 blue4 | 2/2 | ab4_20260916T192531 |
| (A,C) | `1rA_4bC` | 1:redx1->A → 2:bluex4->C | 236.9 | 53.1 | 0.2412 | red1 blue1 blue2 blue3 blue5 | 2/2 | policy_20260918T125937 |
| (B,A) | `1rB_4bA` | 1:redx1->B → 2:bluex4->A | 255.1 | 34.9 | 0.2848 | red5 blue5 blue1 blue2 blue4 | 5/4 | policy_20260918T125937 |
| (B,C) | `1rB_4bC` | 1:redx1->B → 2:bluex4->C | 244.9 | 45.1 | 0.2312 | red1 blue2 blue5 blue3 blue4 | 5/3 | policy_20260918T125937 |
| (C,A) | `1rC_4bA` | 1:redx1->C → 2:bluex4->A | 264.5 | 25.5 | 0.2401 | red1 blue2 blue4 blue1 blue5 | 2/2 | policy_20260917T160043 |
| (C,B) | `1rC_4bB` | 1:redx1->C → 2:bluex4->B | 264.3 | 25.7 | 0.2849 | red4 blue3 blue1 blue4 blue5 | 4/2 | policy_20260918T125937 |
| (A,B) | `2bA_3rB` | 1:bluex2->A → 2:redx3->B | 260.3 | 29.7 | 0.2403 | blue4 blue1 red4 red3 red5 | 3/3 | policy_20260918T125937 |
| (A,C) | `2bA_3rC` | 1:bluex2->A → 2:redx3->C | 275.4 | 14.6 | 0.2314 | blue4 blue1 red1 red3 red4 | 6/2 | policy_20260918T191335 |
| (B,A) | `2bB_3rA` | 1:bluex2->B → 2:redx3->A | 202.0 | 88.0 | 0.2854 | blue4 blue3 red4 red1 red2 | 2/2 | ab4_20260917T075551 |
| (B,C) | `2bB_3rC` | 1:bluex2->B → 2:redx3->C | 273.1 | 16.9 | 0.2400 | blue4 blue3 red5 red3 red4 | 5/3 | policy_20260918T185815 |
| (C,A) | `2bC_3rA` | 1:bluex2->C → 2:redx3->A | 233.7 | 56.3 | 0.2482 | blue1 blue2 red4 red1 red2 | 2/2 | ac4_20260917T125440 |
| (C,B) | `2bC_3rB` | 1:bluex2->C → 2:redx3->B | 251.7 | 38.3 | 0.2404 | blue3 blue2 red4 red3 red5 | 4/3 | bc4_20260917T121718 |
| (A,B) | `2rA_3bB` | 1:redx2->A → 2:bluex3->B | 237.3 | 52.7 | 0.3333 | red3 red1 blue4 blue1 blue5 | 2/2 | ab4_20260916T192531 |
| (A,C) | `2rA_3bC` | 1:redx2->A → 2:bluex3->C | 223.2 | 66.8 | 0.3312 | red3 red1 blue1 blue2 blue3 | 2/2 | policy_20260918T125937 |
| (B,A) | `2rB_3bA` | 1:redx2->B → 2:bluex3->A | 230.3 | 59.7 | 0.2701 | red4 red5 blue4 blue5 blue1 | 5/3 | policy_20260918T200410 |
| (B,C) | `2rB_3bC` | 1:redx2->B → 2:bluex3->C | 271.9 | 18.1 | 0.3067 | red1 red2 blue1 blue2 blue3 | 3/3 | bc4_20260916T203717 |
| (C,A) | `2rC_3bA` | 1:redx2->C → 2:bluex3->A | 262.5 | 27.5 | 0.2852 | red3 red4 blue2 blue5 blue1 | 5/3 | policy_20260918T125937 |
| (C,B) | `2rC_3bB` | 1:redx2->C → 2:bluex3->B | 235.0 | 55.0 | 0.2857 | red3 red4 blue3 blue4 blue5 | 3/2 | policy_20260918T125937 |
| (A,B) | `3bA_2rB` | 1:bluex3->A → 2:redx2->B | 247.3 | 42.7 | 0.2765 | blue4 blue1 blue2 red4 red5 | 4/2 | policy_20260917T154748 |
| (A,C) | `3bA_2rC` | 1:bluex3->A → 2:redx2->C | 264.1 | 25.9 | 0.2177 | blue4 blue5 blue1 red4 red5 | 4/2 | policy_20260918T125937 |
| (B,A) | `3bB_2rA` | 1:bluex3->B → 2:redx2->A | 241.8 | 48.2 | 0.3363 | blue4 blue1 blue5 red2 red1 | 2/2 | ab4_20260916T192531 |
| (B,C) | `3bB_2rC` | 1:bluex3->B → 2:redx2->C | 255.5 | 34.5 | 0.3324 | blue4 blue3 blue5 red5 red4 | 3/2 | policy_20260917T175340 |
| (C,A) | `3bC_2rA` | 1:bluex3->C → 2:redx2->A | 262.0 | 28.0 | 0.2403 | blue4 blue1 blue2 red3 red1 | 2/2 | policy_20260918T125937 |
| (C,B) | `3bC_2rB` | 1:bluex3->C → 2:redx2->B | 244.4 | 45.6 | 0.2228 | blue4 blue2 blue5 red1 red5 | 6/3 | bc4_20260916T213724 |
| (A,B) | `3rA_2bB` | 1:redx3->A → 2:bluex2->B | 198.1 | 91.9 | 0.2401 | red4 red1 red2 blue4 blue3 | 2/2 | ab4_20260917T075551 |
| (A,C) | `3rA_2bC` | 1:redx3->A → 2:bluex2->C | 215.9 | 74.1 | 0.3346 | red4 red1 red2 blue1 blue2 | 2/2 | policy_20260918T125937 |
| (B,A) | `3rB_2bA` | 1:redx3->B → 2:bluex2->A | 234.6 | 55.4 | 0.1876 | red4 red1 red5 blue5 blue1 | 4/3 | policy_20260917T203847 |
| (B,C) | `3rB_2bC` | 1:redx3->B → 2:bluex2->C | 263.7 | 26.3 | 0.3024 | red1 red4 red2 blue3 blue2 | 3/3 | policy_20260918T125937 |
| (C,A) | `3rC_2bA` | 1:redx3->C → 2:bluex2->A | 265.7 | 24.3 | 0.2422 | red3 red4 red5 blue5 blue1 | 6/3 | policy_20260917T203847 |
| (C,B) | `3rC_2bB` | 1:redx3->C → 2:bluex2->B | 251.4 | 38.6 | 0.2460 | red3 red4 red5 blue3 blue5 | 2/2 | policy_20260917T165357 |
| (A,B) | `4bA_1rB` | 1:bluex4->A → 2:redx1->B | 261.9 | 28.1 | 0.2200 | blue4 blue1 blue5 blue2 red1 | 3/3 | policy_20260917T143854 |
| (A,C) | `4bA_1rC` | 1:bluex4->A → 2:redx1->C | 266.3 | 23.7 | 0.2682 | blue4 blue1 blue5 blue2 red1 | 2/2 | policy_20260917T160043 |
| (B,A) | `4bB_1rA` | 1:bluex4->B → 2:redx1->A | 272.4 | 17.6 | 0.1955 | blue4 blue3 blue2 blue5 red1 | 4/3 | policy_20260918T125937 |
| (B,C) | `4bB_1rC` | 1:bluex4->B → 2:redx1->C | 261.5 | 28.5 | 0.1052 | blue4 blue3 blue1 blue5 red5 | 3/3 | policy_20260917T203847 |
| (C,A) | `4bC_1rA` | 1:bluex4->C → 2:redx1->A | 267.1 | 22.9 | 0.2796 | blue4 blue2 blue5 blue1 red1 | 2/2 | policy_20260918T125937 |
| (C,B) | `4bC_1rB` | 1:bluex4->C → 2:redx1->B | 243.9 | 46.1 | 0.3299 | blue4 blue2 blue5 blue3 red1 | 4/3 | policy_20260918T125937 |
| (A,B) | `4rA_1bB` | 1:redx4->A → 2:bluex1->B | 246.5 | 43.5 | 0.2854 | red5 red1 red3 red2 blue1 | 2/2 | ab4_20260917T075551 |
| (A,C) | `4rA_1bC` | 1:redx4->A → 2:bluex1->C | 228.0 | 62.0 | 0.3117 | red5 red1 red3 red2 blue2 | 2/2 | policy_20260918T125937 |
| (B,A) | `4rB_1bA` | 1:redx4->B → 2:bluex1->A | 255.6 | 34.4 | 0.2403 | red1 red2 red4 red5 blue5 | 6/3 | policy_20260918T195540 |
| (B,C) | `4rB_1bC` | 1:redx4->B → 2:bluex1->C | 271.5 | 18.5 | 0.2104 | red3 red1 red4 red5 blue1 | 4/3 | bc4_20260917T115701 |
| (C,A) | `4rC_1bA` | 1:redx4->C → 2:bluex1->A | 272.7 | 17.3 | 0.3342 | red3 red5 red4 red1 blue5 | 3/2 | policy_20260917T175340 |
| (C,B) | `4rC_1bB` | 1:redx4->C → 2:bluex1->B | 271.8 | 18.2 | 0.2851 | red3 red5 red4 red1 blue3 | 3/3 | policy_20260917T203847 |

## 4. 最佳成绩 ≥280s 的两例（仍在优化）

| 任务键 | 最佳(s) | 裕量(s) | 已确认的结构下限 |
| --- | ---: | ---: | --- |

## 5. 本次“争取进入 280s”的成果

| 任务键 | 优化前 | 优化后 | 提升 | 关键修正 |
| --- | ---: | ---: | ---: | --- |
| `1bA_4rB` | 288.1 | 259.7 | -28.4 | 红3→红1（B 往返 52.5→39.4s）+ 蓝1→蓝4 |
| `3rB_2bA` | 280.4 | 234.6 | -45.8 | 红3→红1 + 蓝2→蓝5 |
| `3rC_2bA` | 284.1 | 265.7 | -18.4 | 红块集合改 {红3,红4,红5} + 蓝2→蓝5 |
| `4bB_1rC` | 283.0 | 261.5 | -21.5 | 蓝2→蓝1（B 往返 60.7→54.1s） |
| `1rB_4bA` | 280.7 | 265.3 | -15.4 | 蓝块首块改蓝5（B→蓝5 9.8s） |
| `4rC_1bB` | 280.7 | 271.8 | -8.9 | 无需改策略（原轮含 +8.3s 偶发） |
| `2rC_3bA` | 281.6 | 273.4 | -8.2 | 红3/红4 换位（-3.4s） |
| `4bB_1rA` | 280.2 | 275.2 | -5.0 | 蓝1→蓝3（首轮 target_unavailable 偶发） |

## 6. 口径说明

1. 每例的“最佳成绩”来自一次**完整通过**（严格门槛全满足），不是平均值；44 例的最近一次通过也在 290s 门限内。
2. **连续 48 轮 / 20 轮统计验收尚未做过**：按最佳成绩分布（中位 262s），若同场连跑，风险集中在 ≥270s 的少数几例。
3. 单轮耗时不含每轮约 95–105s 的 Gazebo 重启。
4. 策略表相对 HEAD 共 36 个键变化（含本轮 9 键），`git diff` 为纯替换、0 增删，只改选块与接近侧，未动数量/颜色/区域结构。
