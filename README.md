# YZBot 移动抓取项目

基于 ROS 2 Humble、Gazebo Classic、Nav2、MoveIt 2 和 RGB-D 的移动抓取系统。系统将题目与映射规则解析为任务，识别红蓝物块，完成导航、抓取、运输和 A/B/C 区域投放，并在 CoStudio 与网页面板显示实时状态。

## 环境

- Ubuntu 22.04，x86_64，ROS 2 Humble，Gazebo Classic 11。
- 图形桌面用于 Gazebo、RViz 和 CoStudio；首次安装依赖及云模型调用需要联网。
- 当前模型为 `qwen3.8-flash`，thinking 关闭，实际调用以每题解析证据为准；原 Max 配置保留在用户私有配置中。API Key 保存在当前用户的 `~/.config/yzbot/llm.env`，不能放进源码或提交包。
- 部署场景默认开启 dense 分布式障碍：世界文件内 4 个原有移动障碍物，加运行时由 `competition_dense_actors` 生成的 13 个 `transport_mover_01..13`，合计 **17 个移动障碍物**（`expected_obstacle_count: 17`）。`YZBOT_DENSE_SCENE=0` 只保留为 4 障碍物回退回归模式，不作为验收场景。
- 本项目是源码工程，不是可引导的系统镜像。应复制到 Linux 持久化文件系统，不能直接在 NTFS/exFAT 中构建符号链接工作区。

## 目录

| 路径 | 用途 |
|---|---|
| `workspace/src/yzbot/` | ROS 包、机器人与场景资源、配置及测试 |
| `scripts/` | 环境检查、安装、启动、出题、打包工具 |
| `costudio/` | 比赛布局和扩展源码、已编译扩展 |
| `examples/` | 练习题与三条映射规则示例 |
| `rules/` | 比赛规则原文件 |
| `vendor/linux/` | CoStudio、coBridge 安装包及来源信息（本机可选资源） |
| `vendor/question_generator/` | 官方出题工具 |

机器人场景、URDF、地图及碰撞配置是运行资源，应随源码提交。大模型权重、私有凭据、历史日志、AI 交接记录、旧版本备份和构建缓存不属于提交源码。

## 首次部署

在项目目录执行：

```bash
bash scripts/00_system_check.sh
# 系统缺少依赖时执行；需要 sudo 权限和网络
bash scripts/02_install_dependencies.sh

# 在当前项目目录重新构建，不需要 Git 元数据
bash 部署本版本.sh

# 设置当前用户的私有 API Key
bash scripts/03_configure_api.sh
# 安装比赛面板扩展；保留系统已有的 CoStudio 客户端
bash scripts/04_install_costudio.sh
bash scripts/08_api_check.sh
```

`部署本版本.sh` 构建全部 ROS 包并运行行为测试。若使用发布 ZIP，先完整解压；部署脚本会校验包内 `SHA256SUMS`。源码工作区可不含该发布清单。

如需复制到一个新目录再部署：

```bash
bash scripts/01_deploy.sh "$HOME/yzbot_competition"
```

目标目录已存在且非空时，脚本会停止，避免覆盖其他工程。

依赖安装脚本优先安装包内的 coBridge deb；已安装 coBridge 时不重复安装。若尚未安装 CoStudio，需使用包含安装包的发布包或自行准备与环境匹配的客户端。

## 一键启动

```bash
bash 启动本版本.sh
```

入口打开 Gazebo、RViz、coBridge、比赛节点、网页面板，并尝试打开 CoStudio。等待 `MANUAL_PROBLEM_MODE=PASS` 后提交题目。启动不会自动出题或移动机器人。

系统已在运行时，顶层入口会拒绝重复启动；先正常停止上一轮。图形客户端在场景成功生成机器人后打开；只有 ROS 就绪而图形窗口未就绪时，不会报告启动成功。

需要较低显示负载时：

```bash
YZBOT_GAZEBO_GUI_RENDER_RATE=30 bash 启动本版本.sh
```

CoStudio 使用 **coScene WebSocket** 连接 `ws://127.0.0.1:21274`，导入 `costudio/YZBot比赛布局.json`。网页面板为 `http://127.0.0.1:8080/`。

该布局为项目自定义扩展，国赛须核对主办方指定的官方布局后再使用；完整字段清单和待确认事项见 `workspace/src/yzbot/docs/submission_checklist.md`。研发闭环使用 `bash scripts/09_start_validation.sh`，该入口保留 Gazebo/RViz 等验证环境并省去 CoStudio 客户端自动打开。

## 出题与提交

```bash
bash scripts/06_generate_question.sh             # 正式出题工具
bash scripts/06_generate_question.sh --practice  # 练习
```

完整复制正式出题输出，保留“题目种子：...”与题目正文，并在比赛面板填写对应映射规则后提交。若通过 JSON 接口提交，可另附真实 `seed` 或 `question_seed` 字段；缺失时界面明确提示，不自行补造。三个规则示例位于 `examples/规则1.txt`、`规则2.txt`、`规则3.txt`。比赛实际题目与规则以裁判要求为准。

## 检查与测试

```bash
source scripts/lib.sh
load_ros
python3 -m pytest -q -p no:cacheprovider -m 'not linter' \
  workspace/src/yzbot/tools_demo/test workspace/src/yzbot/mybot/test
bash scripts/07_check_connection.sh
```

运行时会重新生成 `runs/`，保存当前启动日志和必要故障信息；这些内容由打包工具排除。评分、感知失败、导航错误及碰撞保护的诊断信息应保留，不能通过删除日志或改写计数改变验收结果。

## 生成省赛源码提交包

```bash
python3 scripts/package_submission.py --output "$HOME/YZBot-省赛源码.zip"
# 目标机器尚未安装 CoStudio 时，可包含完整离线安装包
python3 scripts/package_submission.py --with-installers \
  --output "$HOME/YZBot-省赛部署包.zip"
```

源码包包含运行源码、测试、地图与模型、操作文档、比赛面板、规则示例，以及本机已有的 x86_64 出题器和 coBridge 安装包。`--with-installers` 额外加入 CoStudio 安装包。缺失的可选安装资源会明确提示，不能据此认为包已具备离线安装能力。

打包工具拒绝打包识别到的私有凭据，排除 Git 历史、旧备份、缓存和运行日志，并在 ZIP 内生成对应内容的 `SHA256SUMS`。不会复制用户 HOME 内的 API Key。

技术说明与验收项目见 `workspace/src/yzbot/docs/`。保留各组件许可证和第三方版权声明。
