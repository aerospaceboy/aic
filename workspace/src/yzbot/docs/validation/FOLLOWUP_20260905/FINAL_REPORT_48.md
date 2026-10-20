# 48 题覆盖与 300 秒冲刺 — 收尾报告(2026-09-06)

起点:`efbbc8a` 后的 V11 交接;本轮终点提交见 git log(4e6905b..本轮末尾)。
运行环境:VM u22 / ROS2 Humble / headless / 关 LLM 与 dashboard / 双障碍开启;
计时口径:**运动墙钟**(competition_case_runner 的 elapsed_sec),出题与
LLM 解题耗时尚未计入(见"全墙钟"一节)。

## 一、结果总览(全部 48 题,每题独立清洁重启)

- **45/48 题完成 5/5、零跳过、双障碍 yield、CASE_ACCEPTED=true**
- 其中 **26/45 运动墙钟 < 300 s**;300–330 s 11 题;330–360 s 7 题;≥360 s 1 题
- **3 题未完成 5/5**(详见第三节,根因均已定位)
- 逐题数据:`/home/ros/dev_ws/experiments/coverage-48/final_table.csv`
  (证据目录按时间戳保留全部轮次,含失败轮,未挑选)

## 二、本轮修复(全部带测试,回归 435 passed / 1 skipped)

1. **入站导航失败误触发托盘出发**(1bA_4rB 失败链):
   pickup_navigation 失败后状态机仍假设"已停靠"而执行 RED_EXIT,
   在半路位置把机器人往托盘推 → 32 s 超时×2 → 任务死亡。
   修复:未到泊位的失败直接跳过该块,下一腿由 Nav2 从当前位置规划;
   先写故障注入测试(红)再修(绿)。commit 4e6905b。
2. **red3 东侧墙角走廊**:成功 plan 距墙尖 0.503 m、失败执行 0.354 m,
   RPP 碰撞预测间歇触发。修复:RED_CUBE_3 接近航点 (-4.45,-1.60),
   两侧段净空 ≥0.6 m(证据取自成功轮 /plan)。commit 9069f68。
3. **B 东侧停靠出口**:确认沿用 2e529aa 的朝向判据(我方临时 dock_pose
   判据与已合入修复冲突,已回退),B 东停靠后反向退让 0.45 m 由 Nav2
   接管——本轮批次内 B 东停靠路线全部通过。commit 4a58ad4(回退)。
4. **LLM API 生产化**:tools_demo/llm_client.py——端点规范化(带/不带
   /v1)、环境变量鉴权(密钥值不进配置/日志/状态)、连接与读取超时受
   单调总预算约束、429/5xx 有界重试、401/403/404 立即显式失败;
   parser 两处调用统一走该客户端;9 项注入式单测 + 3 项本地 HTTP
   集成测试。真实云供应商待用户提供凭据后接入。commit eeab22c/dfef86e。
5. **CoStudio 准备**:rosbridge-suite 已安装并验证
   costudio_bridge.launch.py;docs/costudio/README.md 给出已核实话题
   表(String 封装 JSON 的解析警告)、面板布局、验收清单。Windows 端
   CoStudio 安装与面板截图验收待用户侧执行。

## 三、未完成 3 题与根因

| 题 | 现象 | 根因类 |
| --- | --- | --- |
| 4bC_1rA / 4bC_1rB | 每次蓝→C 送达约 100 s(B 区约 25 s),四腿必超 300 s;600 s 超时下 3/5 | 结构性路径长度(南绕行走廊)——E6 类 |
| 1rC_4bA | 两次 4/5,blue5 入站 patience 超时后软跳过 | 边缘走廊中段(西簇)|

瞬态两类(全轮次统计):机械臂首用 MoveIt CONTROL_FAILED(code=-4)
约 5%;teardown 后 STALE_SIM 竞态(已在补跑脚本加 10 s settle)。

## 四、与 300 s 的差距分解(运动口径)

- 已达标 26 题,覆盖 1+4/2+3/3+2/4+1 的多数色序/区域组合;
- 300–330 s 共 11 题:差距 1–30 s,属于转向与边缘走廊的连续小损耗,
  与 E1(转向)和走廊航点化直接相关;
- 330–360 s 共 7 题、≥360 s 1 题:全部是 **C 目的族**(蓝→C 或红→C),
  C 湾进出走廊长、必经南绕行——需要 E6 类"锚点间路径改善"或
  C 族专用走廊航点;
- 3 个未完成题修复后,预计落入 330 s+ 档(C 族为结构性路径长度),
  尚不能直接计为达标。

## 五、全墙钟(比赛口径)还差什么

现场计时 = 出题 → LLM 解题 → 展示 → 五次合格投放,当前只测运动段:

1. LLM:本地 qwen2.5-coder 路径已可复跑;云 API 需用户配置
   `base_url + api_key_env`,llm_status 话题记录 source/latency;
2. CoStudio:桥与话题就绪,面板验收按 docs/costudio 清单执行;
3. 计时字段:运动(已有 elapsed_sec)+ 解题(llm_status.latency_sec)
   + 总墙钟(待在 case_runner 增加 TASK_PUBLISHED 前的起点)。
   建议运动段预算 ≤270 s,为解题与展示留余量。

## 六、下一步(按优先级)

1. pass5 结果并入;对 4bC 双题做专项复现(录 costmap/plan/避障相位);
2. 边缘走廊治理:从 Gazebo 真值提取高于激光平面的几何做净空图
   (静态地图含低家具,不能直接用),对 330 s+ 档逐段加航点;
3. 机械臂预热预检(消除首用 CONTROL_FAILED);
4. 全墙钟计时三字段落地 + LLM/CoStudio 全链路演练;
5. 慢族优化后重跑 48 题全分布验证。
