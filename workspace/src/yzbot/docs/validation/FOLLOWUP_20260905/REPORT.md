# 追加15分钟：比赛模式约束、提交重试与历史失败复现

起点70397eb6c2664043516d31f29b74f0f517e78e54。当前提交修改仅涉及parser语义、测试和文档；导航/策略/安全配置没有新增修改。

## 修复与验证

web提交不再强制require_llm=False；必须使用模型时拒绝断线、非法JSON和错误顺序的结果，显式练习模式保留fallback。失败请求不写入去重缓存；网页去重包含task_id，允许同题不同轮次及失败后重试。direct question同样只缓存成功发布结果。

新增9项真实回调方法测试：旧代码7失败2通过，修复后全通过；全功能424 passed、1 skipped，仍排除既有flake8/pep257。构建结果和测试前后输出附于本目录。未验证真实模型服务、HTTP请求品质或比赛全过程计时。

## 本轮失败证据与工程结论

追加完整题目1 blue A -> 4 red B：290.4s，2/5、skipped=3、failed、CASE_ACCEPTED=false；双yield，净空0.1475m，collision_faults=[]。不得将290.4秒记成达标。

成功blue1->A和red1->B后，B西侧退出2.81s成功；从(7.539,3.002)取red3:east在(-4.525,-1.269)附近aborted(32.75s)。red4入站随后aborted，RED_EXIT触发多次32s超时，red2/red5后续也失败。失败链已定位，但还没有证明具体静态几何或恢复状态假设是根因。下一轮按V10第8节收集该路段路径/costmap与载荷状态，先复现再修复，禁止把未经复验的候选当稳定代码合入。

该完整仿真由70397eb启动，其parser进程早于本轮parser修复；运动代码与当前一致。新parser改动通过独立回调测试，不能把这次仿真当作新parser完整比赛验证。

## 备份与交接

追加前Git全历史：VM backups/yzbot-pre-followup-20260905.bundle；其中包含原始下午备份分支和上轮代码。完整新增原始失败仿真：backups/followup-evidence-20260905.tar.gz。本地副本在Windows audit-evening-20260905/backups，校验值见BACKUP_SHA256.txt。

当前交接仍为HANDOFF_PROMPT_V10.md，已追加第8节覆盖之前“尚未修复大模型开关”的状态。上一轮两次运动成功证据仍保留；全部48题与完整比赛300秒目标仍未实现。
