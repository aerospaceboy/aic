# 策略优化状态（2026-09-22 收口）

## 策略键
- 约 26 键已应用并单轮验证；verify48 matched 48/48。
- 1rA_4bB 集合勿改（blue1 停靠导航不稳定）；侧与历史通过证据保持 east。

## C 通路
- `YZBOT_C_CLEARANCE_ROUTE=1` 在 4rC_1bA 上更慢（286 vs 280），**保持关闭**。

## 运行器
- `run_zone9_campaign.py` 已加无进展看门狗 `NO_PROGRESS_SEC=150`。

## 下一步
1. 低负载 ≥20 轮连续验收
2. C 族短预转/导航运行时（需谨慎 A/B）
3. 提交 git
