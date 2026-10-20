# 剩余任务优化空间分析（2026-09-21）

数据来源：`audit_policy.py`、`coverage48_summary.py`、`mission_timing_analysis.py`（高墙钟通过轮日志）、`leg_side_table` 实测中位数。

当前状态：**18 键**已应用并单轮验证；`verify48` matched 48/48；audit ≥8s 剩余 9 条（含已失败回滚的 1rA_4bB）。

---

## 1. 结论总览

| 类别 | 数量 | 能否再优化 | 手段 |
| --- | --- | --- | --- |
| A. 集合仍有 gain 且墙钟偏高 | 6 | **可以** | 单键改集合 + 接近侧 |
| B. 集合 gain 小/已失败，墙钟 ≥265 | 8 | **策略层收益有限** | 接近侧微调 + **导航/运行时** |
| C. 已优化 18 键 | 18 | 集合已贴 audit 最优 | 可选：侧 residual / 复验波动 |
| D. 已优化且墙钟 <230 | 多数 | 不必再动策略 | 连续验收即可 |

---

## 2. 类别 A：仍建议策略优化（优先实测）

| tag | latest | audit gain | audit 建议集合 | 风险 | 建议 |
| --- | --- | --- | --- | --- | --- |
| **4bB_1rC** | 274.3 | 10.2 | blue2,3,5,4 + red5 | 低 | 可改；blue 优先 west/east 按侧表 |
| **1bC_4rB** | 270.3 | 11.7 | blue3 + red4,1,2,5 | 中（B 族） | 可改；避免 blue2 spawn east 长腿 |
| **2bA_3rB** | 260.3 | 17.4 | blue4,5 + red2,4,5 | 低 | 推荐 |
| **3bA_2rB** | 251.4 | 17.4 | blue4,1,5 + red4,5 | 低 | 推荐 |
| **2rC_3bA** | 262.5 | 9.2 | red3,4 + blue4,1,5 | 低 | 可试 |
| **4bC_1rB** | 243.9 | 15.0 | blue4,3,2,5 + red5 | 低 | 可试 |
| 3rA_2bB | 202.8 | 13.5 | red1,2,4 + blue5,4 | 低 | 已较快，优先级低 |
| 1rA_4bC | 236.9 | 9.9 | red1 + blue5,2,3,4 | 低 | 优先级低 |

**接近侧安全规则（实测）**：`blue3/blue5→west`，`blue4→B 用 west`，`red2/red4→west`，`red1/3/5→east`；`blue5→B` 必须 west（east 曾 lethal start）。

---

## 3. 类别 B：高墙钟、集合 gain≈0 —— 瓶颈在导航几何

### 3.1 阶段拆分（代表性通过轮）

| tag | wall | pickup nav | dest nav | pick | place | 其他 | nav占比 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1bA_4rC | 273.1 | 94.5 | 102.4 | 29.0 | 22.6 | ~24 | **~72%** |
| 4rC_1bA | 272.7 | 100.5 | 101.3 | 29.8 | 23.3 | ~18 | **~74%** |
| 4rC_1bB | 271.8 | 92.5 | 102.9 | 34.5 | 23.0 | ~19 | **~72%** |
| 1bB_4rC | 277.4 | 100.9 | 99.5 | 31.5 | 23.2 | ~22 | **~72%** |
| 3rC_2bA | 265.7 | 93.1 | 93.9 | 32.4 | 22.9 | ~23 | **~70%** |
| 2bA_3rC | 275.4 | 91.7 | 92.8 | 36.2 | 23.1 | ~32* | **~67%** |

\* 2bA_3rC 的 waiting_perception 11.6s + zone_exit 14.3s 偏高。

### 3.2 最慢导航腿（C 族共性）

- **红→C 投放腿**：19–25 s × 4 次 ≈ **76–100 s**
- **C 出口→下一托盘**：18–26 s × 3–4 次 ≈ **54–100 s**
- 出生点→首个蓝/红托盘：25–34 s（仅 1 次）

例：`1bA_4rC` 最慢腿 spawn→BLUE 31.8s，C 投放 24.2/24.1/19.5s，托盘 18.9/18.7s。

### 3.3 为何集合改不动

audit 对 `1bA_4rC`/`4rC_1bA`/`4rC_1bB`/`3rC_2bA`/`1bB_4rC` 的 gain 仅 **0.0–0.7 s**（当前集合已贴实测最优）。  
合法物块位置固定，**C 区与红托盘的往返距离是结构性成本**。

### 3.4 运行时可挖的方向（非策略表）

| 方向 | 预期 | 说明 |
| --- | --- | --- |
| Nav2 到 C 的巡航速度 / 停靠预转 | 每腿 2–5 s × 8 腿 → **15–40 s** | 对照 dev_ws 预转实现；禁止为过门限放宽净空 |
| 红托盘 corridor 接近惩罚调优 | 部分 west 接近绕远 | `CORRIDOR_APPROACH_*`；单键 A/B |
| 抓放时序 | pick 6–8s、place ~4.5s ×5 ≈ 50–60s 固定 | 反升余弦已做；再压需谨慎（放置抽搐历史） |
| 感知重试 | 2bA_3rC perception 11.6s | 缓存槽位/扫描窗口 |
| 运行器看门狗 | 不提速，但降失败浪费 | 交接 §12.1 |
| 负载控制 | 高 load 放大超时 | 跑分前 `uptime` |

**不要**：为 C 族强行改集合到未实测托盘（曾出现 lethal / 导航失败）。

---

## 4. 类别 C：仅侧优化候选（集合不动）

| tag | 现状 sides 问题 | 建议 |
| --- | --- | --- |
| **1rA_4bB** | blue1/2/4 用 east；B 实测 west 更快 | **仅改侧**：blue1/4→west（集合勿再动，已失败过） |
| **2bB_3rC** | gain 7.3 接近门槛；blue4 east | 试 blue4→west 或蓝组换 blue5 |
| **1rC_4bA** | gain 6.5；blue2/4 east 送 A | 试 blue→A 路径 west（A→blue1 west 7.7s） |
| 1bA_4rC | red3 west（corridor） | 试 red3→east 对照一轮 |

侧改动预期 **5–15 s**，须同样单轮验证。

---

## 5. 不建议再投入策略时间的任务

- 已优化且 latest **≤220 s** 的 18 键：余量已够 300 s 比赛门限。
- audit gain **&lt;8 s** 且 wall **&lt;255 s**：期望收益 &lt; 竞赛波动。
- `1rA_4bB` 集合方案：已实测失败，禁止原样重试。

---

## 6. 建议执行顺序

```
1) 类别 A 中 2bA_3rB、3bA_2rB、4bB_1rC（收益/风险比最好）→ 单轮验证
2) 1rA_4bB 仅侧：blue1/blue4 → west → 单轮
3) 1bC_4rB / 2rC_3bA / 4bC_1rB → 单轮
4) 类别 B：导航运行时（预转 / C 腿速度）A/B 对照 1bA_4rC 或 4rC_1bA
5) 低负载 ≥20 轮连续验收 + 看门狗
6) 全量 verify48 + coverage48 收口
```

---

## 7. 验证命令（不变）

```bash
cd /home/ros/huxuyang/YZBot-224/workspace
CLEAN_ONLY=True bash src/yzbot/tools_demo/scripts/clean_launch.sh
DISPLAY=:0 COSTUDIO_HEADLESS=True POLICY_CASES=<tag> \
  python3 src/yzbot/tools_demo/scripts/run_zone9_campaign.py
python3 src/yzbot/tools_demo/scripts/verify48_policy_match.py
python3 src/yzbot/tools_demo/scripts/audit_policy.py --min-gain 8
```
