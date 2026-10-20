# YZBot 当前交接状态（2026-10-06）

只向master提交推送，不动huxuyang，保留用户GUI/支撑轮静态TF修改、stash和历史证据。先读AGENTS.md、[最新报告](docs/validation/TARGETED_290_20261006/README.md)、[详细提示词](docs/validation/TARGETED_290_20261006/HANDOFF_PROMPT.md)、[日志](docs/validation/TARGETED_290_20261006/worklog.md)。

最新人类通知取消颜色执行顺序；数量、颜色、目标区、实体唯一性和精确投放不变，5件正确交付20分。旧order_verified允许false，新requirements_verified和pickup_delivery_consistent必须true。motion<290与完整比赛300秒分开记录。

## 当前实测

| 运行版本 | 题目/时间 | 安全与边界 |
|---|---|---|
| c124b24 | 1rC_4bB270.6s | 5/5零跳过20分，原始双yield，零碰撞，净空0.2060m |
| c124b24 | 4bB_1rA288.5s | 同上，净空0.1943m；仅1.5秒余量 |
| c124b24 | 3rA_2bB237.2s | 同上，净空0.3778m |

三题运行commit、用户patch、source/config manifest分别相同。实际运行记录是c124b24，之后只补文档，最终HEAD以Git/部署收据为准；下一次物理矩阵须重新冻结最终HEAD。

本轮before：1rC_4bB250.6、4bB_1rA276.7，故未证明提速；不同相位单次不作稳定因果结论。修的是DP漏计出生点经动态展示路线到origin的估价，staged_cube_delivery_cost复用同一导航helper；没有改实际导航或安全。其他静态绕路、强制航点和zone实际退出位姿仍可能误估。

734行为通过，3linter排除；tools_demo构建；48静态排程配额/唯一实体/allowlist合法，非物理48。当前after脚本结束且gzserver0；本轮未启动全48。

旧8a0f538同版本补跑收敛48/48<300、46/48<290；旧1bC_4rB在上一轮0326585单次243.3通过，但没有在c124重跑，不能混到新矩阵。旧3rB_2bA/3rB_2bC净空0.1002/0.1020，应优先不同相位复测。

## 下一步

先重复288.5秒题并读逐腿表，再复测近阈值安全题。按实际地图/强制路段/退出姿态校准代价，不先加速或大改布局；若证实负优化，最小撤回本轮coordinator评分调用，保留规则/审计，不整体reset。

定向无安全失败后冻结新版本，串行run48_v2.sh 1 48 ordered；summarize48_v2输出固定48行，check48_290必须expected48 accepted48 failing=[]。人工复核版本、patch、manifest与动态响应；新代码用新root，保留失败尝试和首次成功率，不混版本。碰撞停自己的批次留证后修，不能无限赌相位。

API、GUI、CoStudio、完整比赛墙钟未测。凭据仓库外、thinking=false，模型按当前人类授权配置，不默默覆盖。手动速度走安全输入链，不直接发/cmd_vel_safe。

备份backup/before-targeted290-20261006→3f1e698；更早backup/before-unordered-20261006→8a0f538。完整证据在VM experiments/targeted290-{before,after}-20261006，主机targeted290-20261006，仓库摘要见最新报告。
