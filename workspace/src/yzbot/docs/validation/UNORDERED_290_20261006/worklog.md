# 工作日志：2026-10-06 新货物计分规则

| 阶段 | 操作与证据 | 结果 / 边界 |
|---|---|---|
| 接手核对 | VM与origin/master均8a0f538；用户GUI/支撑轮TF、stash和既有未跟踪文件保留 | 无仿真运行；建立backup/before-unordered-20261006与仓库外before.patch |
| 旧证据复核 | 按48个最新-run读取audit/result/commit/worktree.patch/source-config.sha256 | 同commit、同用户patch、同source/config manifest；旧300秒48/48，新290秒46/48 |
| 新规则复核 | 48份旧证据用新要求审计重算，未覆盖旧audit.json | 全部货物配额符合、每题20分；原始双yield48/48 |
| 文档审查 | 仓库README、PROJECT_HANDOFF及DISTRIBUTED_LAYOUT报告仍指向早期未完成状态；真正完整报告在主机；修正后的converge工具仅在实验目录 | 收尾同步最新说明；新的check48_290按全部48题判定，不复用旧收敛口径 |
| 最小规则改造 | 原题plan不变；有界DP调整执行jobs；保留显式override；要求/实体/精度审计保留 | 允许跨颜色；旧order_verified留作诊断；new goods_score与motion_under_290 |
| 测试 | targeted 26；全部行为730、linter三项deselected；构建tools_demo | 首次测试因安装包未包含新模块失败，补PYTHONPATH与构建后通过；一项合成图最优值修正为3；运行时接入发现不存在zone_a_dock，改用已有_zone_nav_pose后通过 |
| 排程静态集成 | ROS_DOMAIN_ID=73，当前部署参数；完整48题排程配额、唯一实体、接近侧allowlist核对 | 全部合法，最大23.1ms；没有启动48题物理矩阵 |
| 空矩阵检查 | check48_290读取不存在证据根 | 返回expected48、accepted0、exit1，不会缺题假收敛 |
| 代码冻结 | 0326585，只有本轮9文件 | 用户两个改动未提交；两道物理运行期间源码、参数、Git版本保持不动 |
| 冒烟启动 | 脱离会话顺序跑两题 | 首次仓库外包装脚本在source前set -u导致AMENT_TRACE_SETUP_FILES未定义，没有启动仿真；修包装脚本后重启，未记为运动失败 |
| 物理运行1 | 3rA_2bB，蓝1→B、蓝5→B、红2→A、红1→A、红3→A | 239.8s，5/5/skip0，20分，safe，双yield，净空0.4055；旧顺序false、新要求true；较旧205.9s记录慢33.9s，不能声称稳定提速 |
| 物理运行2 | 1bC_4rB | 结果见审查报告与case-results.csv |
| style与收尾 | 完成两道后只整理新增style、文档及证据；对比用户patch、运行哈希，推送master | 最终结果及提交号见部署收据；不改世界、地图、速度、障碍或安全参数 |

未完成：新版本物理48矩阵与稳定重复、1rC_4bB等难题定向复测、GUI可视重放、真实API/CoStudio、完整比赛墙钟。下一位Agent按提示词冻结版本、继续优化和验证至48全部<290秒，不得用旧48表代替。
