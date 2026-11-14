# 解题模型配置

当前默认模型为 `qwen3.8-flash`，原 `qwen3.8-max` 的完整配置保存在同目录的 `task_parser.max.yaml`。配置路径均在 `tools_demo/config/`：

- `task_parser.yaml`：实际启动读取的配置。
- `task_parser.flash.yaml`：Flash 配置。
- `task_parser.max.yaml`：切换前保留的 Max 配置。

两份配置只在 model 字段上不同，API 地址、超时、重试、思考模式和话题保持一致。API Key 继续从用户私有环境文件读取。

停下当前任务和仿真后，在项目根目录执行以下命令恢复 Max：

```bash
cp workspace/src/yzbot/tools_demo/config/task_parser.max.yaml \
   workspace/src/yzbot/tools_demo/config/task_parser.yaml
source scripts/lib.sh
load_ros
(cd workspace && colcon build --symlink-install --packages-select tools_demo)
bash scripts/08_api_check.sh
```

切回 Flash 时把第一条命令的 `task_parser.max.yaml` 换成 `task_parser.flash.yaml`。切换后需要重新启动解析节点，并检查状态中的实际 model。测试允许完整切换到这两份配置；选择 Max 做对照时，应按该配置身份单独记录结果，不把两种模型的成绩合并为同一版本的全量验收。

模型输出仍经过现有确定性解题与任务校验；不会因模型切换自动关闭 require_llm、放宽答案检查或悄悄回退到另一付费模型。
