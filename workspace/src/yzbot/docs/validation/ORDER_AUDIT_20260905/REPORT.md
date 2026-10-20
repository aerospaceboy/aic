# 2026-09-05 晚间顺序规则与下午成果审计

起点：75d9f58d6fe2af465f3a3a165a71bb00ee0b0ae1；最终代码以本报告所在提交为准。用户确认只约束题目颜色顺序，同色实体可优化。保留 DP 表、出生点前移及全部既有安全配置。

## 结论

修复有序策略键、跨题自动覆盖污染、失败重试消费未来颜色、无效策略校验和 B 东侧退出生命周期错误。未宣布全部48题或完整比赛已达300秒。新入口见 HANDOFF_PROMPT_V10.md。

| 完整运动题目 | 秒 | 完成/跳过 | 顺序审核 | 双 yield | 最小净空 |
|---|---:|---|---|---|---:|
| 4 red B -> 1 blue A | 261.4 | 5/0 | 通过 | 通过 | 0.1313 m |
| 1 blue B -> 4 red A | 259.7 | 5/0 | 通过 | 通过 | 0.1348 m |

两轮 collision_faults=[]，五次 delivery_truth 均 valid=True、precise=True。红先实体顺序 red3、red1、red4、red2、blue1；蓝先 blue1、red2、red5、red3、red1。red4 从 B 东侧投放后，实际底盘 (9.422,3.031,yaw=-3.103)，B_EXIT 1.90s 成功；下一次东侧退出1.80s成功。此处证明了 dock_pose 被清除且 yaw 为负pi附近时的真实修复效果。

最终构建 tools_demo 成功；功能回归 **415 passed, 1 skipped**。排除仓库已有 flake8/pep257 测试，不代表全仓库 lint 通过。新增行为验证覆盖全部48策略行、两题连续播种、手工覆盖快照、未来颜色保留、无效条目整体回退、正负pi出口、功能成功与300秒分离及缺失/错误安全证据拒绝。新 bash 入口通过语法检查并实际跑完第二轮。

两次仿真运行时的控制逻辑与最终一致；随后只整理测试/辅助审核器格式及文档。第一轮原始目录 `/tmp/evening_audit_20260905/redfirst_B_exit`；第二轮目录由 `/tmp/evening_audit_20260905/reproducible/latest-run.txt` 指向，已全量封存。目录内 audit.json 与 result.log 随仓库保存，完整 launch.log、gz_stats 和 profile 在证据压缩包中。

## 对下午结论的纠正

- 246.9/252.0s 是蓝先题真实成功，不因本轮发现而否定。但两次实体顺序不同；不能把它们当同配置重复试验，也不能据此证明48题模型普适可信。
- 红先键始终误查蓝先行，旧红先覆盖证明的是旧混合选择行为，不能作为修复后红先DP的验收。
- 旧B修复看dock_pose，而完成导航后该变量为空；原测试保留假dock姿态，未覆盖实际生命周期。本轮用实际朝向并经过完整任务验证。
- 旧批量脚本的completed跳过语义会掩盖超时与版本变化，且有覆盖目录和统计进程问题；新run_ordered_case.sh每次保留新目录。
- 下午覆盖快照29题5/5通过、38个终局中的15题<300，仍有失败、慢题与未完成项。不得在新版本继续累计成“已全覆盖”。

## 备份及恢复

VM `/home/ros/dev_ws/backups/` 内：

- `yzbot-pre-order-rule-20260905.bundle`：改动前完整Git历史。
- `yzbot-pre-order-rule-20260905.tar.gz`：改动前工作树，含DP表及历史未跟踪资料。
- `afternoon-experiments-20260905.tar.gz`：下午整个 exp_e0_20260905，含生成器、策略计算、覆盖结果和原始实验。
- `order-audit-evidence-20260905.tar.gz`：本轮两次完整原始仿真证据。

SHA256见 BACKUP_SHA256.txt。Windows备份目录为 `D:\user\Desktop\AIC\audit-evening-20260905\backups`；改动前源码另已解包在其父目录用于审计。备份分支 `backup/pre-order-audit-20260905-75d9f58` 指向起点并随本轮推送。

恢复时在空的新目录执行 `git clone <bundle路径> <新目录>`；需要只查看历史策略，用 `git show backup/pre-order-audit-20260905-75d9f58:tools_demo/config/cube_order_policy.json` 导出到新的文件。不要把tar覆盖当前工作区，也不要reset --hard当前未保存工作。

## 验证边界及后续

当前两轮是关闭LLM、dashboard、SLAM展示的运动benchmark，不含出题与推理耗时。既有投放位置稳定/设置逻辑未改，最终真值不能证明纯物理精度。web problem_callback 强制允许规则回退属于后续需处理的比赛模式问题。本轮未改变上述范围以免把未验证的修改混入稳定控制版本。

下一步按V10：历史失败/红先24题重测；真实ROS连续两题；完整48覆盖；困难族同机A/B；路径与动作预算校准；完整LLM和显示流程计时。保留用户规则和安全边界，不用跨颜色重排、缩碰撞体或放宽验收换成绩。
