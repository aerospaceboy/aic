# YZBot Competition coStudio extension

正式比赛可视化扩展，提供题目与大模型解析、任务状态卡、双相机图像、三维场景标记和
比赛开始按钮。

- `topic-area`：输入题目与区域映射规则并提交；提交后显示解析状态（解析中 / 大模型解析成功·指令已发布 / 解析失败·未发布本题指令）、模型名、任务编号、指令内容与执行顺序。
- `status-card`：资源数量、目标区、任务进度、机械臂抓取状态和本轮完成情况。
- `vision-image`：底盘前视相机与机械臂腕部 RGB-D 相机两路独立画面。
- `start-button`：向开始门控发布 `std_msgs/msg/Bool`。

题目面板把两项输入编码为 JSON 分片，发布到 `/competition/task/problem_chunk`；
后端完成大模型解析与确定性校验后发布 `/competition/task/plan` 和
`/competition/task/parse_result`，`legacy_costudio_status` 再汇入
`/competition/status` 供面板显示。显示契约见 `docs/costudio/PARSE_DISPLAY.md`。

实时模式下面板不会用演示数据补齐缺失字段：未收到解析结果时显示等待状态，
解析失败时清空本题指令，避免把旧结果或示例数字显示为当前解析结果。

## 构建与安装

```bash
npm ci
npm run build            # 开发构建
npm run build -- --mode production
```

把 `dist/extension.js` 覆盖到已安装扩展目录后重启 coStudio：

```
%USERPROFILE%\.coStudio\extensions\aerospaceboy.yzbot-competition-<version>\dist\extension.js
```

`displayName` 必须保持 `aerospaceboy.yzbot-competition`，以匹配正式布局中的
`aerospaceboy.yzbot-competition.<panel>` 面板类型。
