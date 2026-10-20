# 定向慢题审查与首腿评分修复

日期：2026-10-06。修改基线master `3f1e698`；实际修正后运行代码 `c124b24300f7e441bca8c9bab628bbcc07c3ea74`。最终文档提交与远端核对见部署收据。

## 结论和实测

修复一处确定的排程估价遗漏：机器人从出生点必须经过双动态障碍展示路线到(0,0)后才去首件，旧排程却按出生点直接到物块计价。新增staged_cube_delivery_cost复用导航器的dynamic_gate_waypoints，补入现有路线；其他腿的既有估价保持不变。

此修复没有改实际导航路线、布局、物块、zone、四移动体连续模式、巡航速度、机械臂动作或安全参数。三道修正后定向题均全门<290，但本轮不能宣称已提速或稳定达标：两道有本轮before的题都变慢，第三道仅较上一轮单次略快。before/after并非同相位严格对照，既不能证明稳定收益，也不足以确定因果回归。候选估价仍需校准，4bB_1rA仅1.5秒余量。

| 题目 | 对照来源/秒 | 修正后秒 | 差值秒 | 5/5/跳过 | 分数 | 要求/精确投放 | 四移动体/原始双yield | 碰撞 | 最小净空m | <290 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1rC_4bB | 本轮3f1e698：250.6 | 270.6 | +20.0 | 5/5、0 | 20 | True/True | 4/True | False | 0.2060 | True |
| 4bB_1rA | 本轮3f1e698：276.7 | 288.5 | +11.8 | 5/5、0 | 20 | True/True | 4/True | False | 0.1943 | True |
| 3rA_2bB | 上一轮0326585：239.8 | 237.2 | −2.6 | 5/5、0 | 20 | True/True | 4/True | False | 0.3778 | True |

三道均CASE_ACCEPTED、motion_functional_pass、requirements_verified、truth_precise、original_pair_demonstrated为true。旧文字颜色顺序false符合新通知，不伪造该字段。三道修正后运行commit、用户patch哈希、source/config manifest分别一致。

历史8a0f538同版本补跑收敛48/48<300，按290门为46/48；旧1rC_4bB=293.5、4bB_1rA=285.3。本轮不能只用旧最慢值宣称修正后大幅提速。上一轮1bC_4rB243.3s证据仍有效于其运行版本，未在c124b24重跑，不能填入本版本48表。

## 修改范围与验证

| 项目 | 事实 |
|---|---|
| 修改文件 | navigation_math.py、mission_coordinator.py、test_staged_selection_cost.py，仅3文件 |
| 非出生段兼容性 | 没有dynamic_gate_waypoints时返回原cube_delivery_cost精确值 |
| 输入校验 | 先调用原评分校验，长度不足/NaN输入不会被集结位姿掩盖 |
| 单元/行为测试 | 定向27 passed；全部734 passed，3个linter deselected |
| 构建 | tools_demo成功；新模块与现有安装环境用于物理运行 |
| Style | 新测试flake8干净，修改文件相对基线零新增诊断；既有style债未修 |
| 48静态排程 | 隔离ROS域73，全部48题配额/唯一实体/allowlist合法，最大计算146.778ms；不是48题物理验收或严格耗时基准 |
| 物理运行 | 修正前2题，修正后3题；headless、use_llm=false、诊断600s上限，实际通过门<290 |
| 失败签名 | 五道定向运行未出现patience exceeded、bounded_recovery、CONTROL_FAILED等提取签名；所有导航结果成功，不等于未来无失败 |
| 版本保护 | backup/before-targeted290-20261006指向3f1e698；用户两文件patch逐字节未变；stash及未跟踪文件保留 |
| 运行状态 | after脚本AFTER_DONE；采证时gzserver为0；未启动全48批次 |

新函数修的是评分逻辑，未修改Nav2路线。DP依然使用路线估计：其他强制航点、静态墙绕行和真实zone退出位姿尚未完整计价；B东侧镜像出口也不能仅靠zone名称近似。不要将估计代价最优当作物理秒数最优。

## 具体下一步与未完成

| 优先级 | 问题/风险 | 下一位Agent执行路径 |
|---|---|---|
| P0 | 4bB_1rA288.5秒仅1.5秒余量，且本轮比before慢11.8秒 | 保留当前/基线引用和各自证据；不同相位复测并读逐腿表，定位首腿/北侧物块到B的真实绕路，不靠反复赌一次通过 |
| P0 | 评分遗漏已补，但两道before对照未体现收益 | 校准实际强制航点长度、地图绕路及zone退出位姿；小步比较，若证实负优化，最小撤回coordinator对新评分的调用，保留新规则/审计/备份，不整体reset |
| P0 | 旧3rB_2bA、3rB_2bC净空0.1002/0.1020m | 优先复测不同相位，检查全程扫掠区准入/等待/安全链一致性，不能放宽0.10门 |
| P1 | 新规则+新评分版本全48未跑 | 定向风险排除后冻结代码/安装/config/patch，用新root全48；最新失败覆盖旧成功，不混版本 |
| P1 | 接近290题可能有方差 | 报首轮通过率、补跑次数、最差时间/净空；48有合格证据与稳定重复分别报告 |
| P2 | GUI、真实API、CoStudio、完整比赛墙钟未测 | motion矩阵后补代表题端到端计时，密钥私有；未测填未测，不填0或声称比赛300已完成 |

不大改当前分散布局；只有物理可达性/停车净空确证不足时才成套同步world、托盘、zones/mission、默认几何、allowlist、地图与生成器，改布局后新版本全48重新验证。真实碰撞需停止本轮拥有的仿真并保留证据，不无限同版本碰运气。

## 证据

- 本报告旁case-results.csv、results.json：全部before2+after3台账，commit/patch/manifest与证据路径。
- evidence/{before,after}/{tag}/：原始audit、result、question、运行commit、模式、source/config hash、逐腿表和日志摘要。
- 完整原始launch日志仍在VM /home/ros/dev_ws/experiments/targeted290-{before,after}-20261006/，未覆盖历史。
- behavior-tests.log、build.log、style-comparison.json、schedule48-static.json、schedule-comparison.json。
- runtime-receipt.json证明三道同运行manifest、受保护patch一致；部署收据记录最终文档HEAD及origin/master。
- 主机交付目录D:/user/Desktop/AIC/targeted290-20261006；仓库镜像docs/validation/TARGETED_290_20261006。
