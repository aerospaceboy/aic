# 大模型指令解析与面板显示证据

本页说明评分项“成功解析大模型生成题目指令，成功发布大模型指令并将指令解析到可视化面板中显示”的实现契约、判定方式与实测证据。

## 数据链

1. coStudio 题目面板把题目与映射规则分片发布到 `/competition/task/problem_chunk`。
2. `competition_task_parser` 重组分片，调用真实大模型求解，并与确定性求解结果逐项交叉校验。
3. 校验通过后发布 `/competition/task/plan`（颜色、数量、目标区、执行顺序），同时发布 `task_id`。
4. 解析证据发布到 `/competition/task/parse_result`，包含本次提交的唯一 `request_id`、状态、模型名、题目、映射规则与完整计划。
5. `legacy_costudio_status` 读取该证据，把解析结果写入 `/competition/status`，面板据此显示。

## 解析状态

| state | 含义 | 面板显示 |
| --- | --- | --- |
| `parsing` | 已收到题目，正在请求模型 | 已接收题目 · 大模型解析中 |
| `published` 且 `source=llm` | 模型结果通过校验并已发布 | 大模型解析成功 · 指令已发布 |
| `published` 且 `source=rules` | 非大模型结果（练习模式） | 规则解析结果 · 非大模型结果 |
| `failed` | 模型不可用、超时或结果与确定性解不一致 | 解析失败 · 未发布本题指令 |

失败时不会发布任何指令，并且面板会清空上一题的指令内容与执行顺序，避免把旧结果显示成当前解析结果。

## 面板显示内容

- 比赛题目原文。
- 解析状态、模型名与任务编号（`request_id`）。
- 指令内容：如“红 4个→A区 · 蓝 1个→C区”。
- 任务规则：题目给出的映射规则。
- 执行顺序：逐件展开的顺序，如“1.红→A · 2.红→A · 3.红→A · 4.红→A · 5.蓝→C”。
- 状态卡同步显示红蓝数量与目标区。

## 实测证据

真实题目（衣柜题，`x=4`、`y=1`，规则 `x→红、y→蓝，数量≥3去A区，其他去C区`）：

- 模型由 `tools_demo/config/task_parser.yaml` 的 `model` 参数决定，当前为 `qwen3.8-max`。
- `/competition/task/parse_result`：`state=published`、`source=llm`、`model=qwen3.8-max`、`task_id=plus-proof-151845`、`plan.items=[red 4→A, blue 1→C]`、`error=""`（运行中系统真实调用）。
- coStudio 面板实际渲染文本：`比赛题目：<原题>`、`大模型解析成功 · 指令已发布`、`模型：qwen3.8-max · 任务：plus-proof-151845`、`指令内容：红 4个→A区 · 蓝 1个→C区`、`执行顺序：1.红→A · 2.红→A · 3.红→A · 4.红→A · 5.蓝→C`。
- 隔离域验证（`ROS_DOMAIN_ID=91`，真实模型）：成功路径 `published`；无效映射与模拟超时均 `failed`，且 `plan_publications=1`（失败不发布指令），面板对象与顺序被清空。

## 回归测试

```bash
python3 -m pytest -q tools_demo/test/test_parse_result_evidence.py
```

覆盖：成功证据字段、失败不发布且带原因、失败后可重试、面板载荷携带指令、失败后清空旧指令。
