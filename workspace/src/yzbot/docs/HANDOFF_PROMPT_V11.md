> 当前接手入口：[HANDOFF_PROMPT_V12.md](HANDOFF_PROMPT_V12.md)。V11保留历史架构与方案；状态以V12及其审查报告为准。

# 项目接手提示词 V11：有序任务、CoStudio、API 大模型与300秒目标

状态日期：2026-09-05。代码基线 efbbc8ace53f3259fcd9125762eed1a17f229f3c，本文所在后续提交仅整理交接资料。本文是当前统一入口，V8/V9/V10保存历史，不再按旧章节逐轮覆盖来判断现状。当前用户请求优先于文档建议。

## 一、任务目标与规则来源

继续推进完整比赛300秒目标：五物块、零跳过、颜色/目的地顺序正确、真实安全链与投放验证有效，并包含出题、大模型解题和展示的全过程墙钟。不要因单题运动阶段<300秒就宣布整体完成。每次交接保存已执行证据、未完成项和下一条可执行实验。

用户明确：**只约束题目颜色顺序，同色实体可优化。** DP同色实体选择与接近侧优化继续保留。不得跨颜色重排或改变任务目的地，也不要强制同色实体编号。

本轮新增方向：用户转述官方建议可视化使用 **coStudio（CoStudio）**，大模型通过 **API接入解题**，下一位agent应按这两个方向落地。推荐并非已核实的强制赛规；不要把本提示词中的工程建议冒充官方要求。

用户截图直接支持的展示内容：任务题目、抓取顺序、每色数量、当前抓取数量/进度、识别资源/前往资源点/移动到放置区等工作状态、抓取/放置等末端状态；示例还有两幅识别图、三维视图、开始按钮。截图本身没有显示API协议、软件版本或强制指定软件的文字。

原始工作区竞赛规则PDF第4页规定每轮五分钟、计时后生成题目；第6页规定抓放遵循大模型规则。保留图像、SLAM与任务展示，不把运动计时等同完整比赛计时。

## 二、项目理解与已验证现状

权威工程：VM /home/ros/dev_ws/src/yzbot，工作区 /home/ros/dev_ws，master，仓库 cty050422/yzbot-224。先读AGENTS.md、git status、git log；使用现有连接配置，不在文档或仓库写密码/密钥。保留已有未跟踪AGENT_HISTORY_20260823及V5/V6/V7。

这是ROS2 Humble、Gazebo、Nav2、MoveIt组成的任务执行系统：题目 -> 有序TaskPlan -> MissionStateMachine逻辑作业 -> coordinator选择当前允许的同色实体 -> navigator安全锚点/停靠/退出 -> 视觉与机械臂抓放 -> 真值验证。CoStudio应消费这条真实数据链，不能自行推算出另一份比赛状态。API只负责解题和结构化解析，不能直接绕过任务校验控制机器人。

关键源码：tools_demo/tools_demo 下 task_protocol.py、task_parser_node.py、mission_state_machine.py、mission_coordinator.py、pick_order_policy.py、task_navigator.py、navigation_math.py、competition_dashboard.py、speed_profile_recorder.py、competition_case_runner.py、evidence_audit.py。DP表在tools_demo/config/cube_order_policy.json；导航配置在bot_navigation；抓放在mybot包，先rg确认文件位置。

已修复：实际颜色顺序策略键；自动覆盖跨题污染；失败重试吞未来颜色；无效实体/接近侧/重复和不完整序列校验；dock_pose清空后B东侧退出识别；web必须使用模型时禁止规则回退；失败提交可重试，同题不同task_id可作为新轮。

已验证的运动结果：

| 题目 | 时间 | 结果 | 双yield/净空 |
|---|---:|---|---|
| 4 red B -> 1 blue A | 261.4s | 5/5，零跳过，顺序正确 | 通过/0.1313m |
| 1 blue B -> 4 red A | 259.7s | 5/5，零跳过，顺序正确 | 通过/0.1348m |
| 1 blue A -> 4 red B | 290.4s | **失败，2/5，跳过3** | 通过/0.1475m |

最终功能回归424 passed、1 skipped，排除既有flake8/pep257，不声称全仓lint通过。前两题与失败题都未覆盖真实API和完整展示计时；全部48题稳定300秒尚未实现。已通过题保留为回归，失败题不能算超时目标成功。

## 三、第一优先级：复现已知失败，再改运动

失败证据：docs/validation/FOLLOWUP_20260905；完整launch日志在backups/followup-evidence-20260905.tar.gz。1bA_4rB完成blue1->A、red1->B，B西侧退出2.81s成功；随后从(7.539,3.002,yaw0.023)选red3:east，32.75s后在(-4.525,-1.269,yaw-2.037)附近aborted。red4又失败，RED_EXIT多次32s超时，后续red2/red5失败。

落地顺序：

1. 重跑同题并保存red3入站的nav_plan、实际Smac路径、costmap起点/终点占用、controller停机原因、底盘/载荷姿态。先判断静态托盘碰撞预测、路径弯折或恢复逻辑，不能把所有B族失败都归到已修好的B出口。
2. 检查“未到取块泊位、尚未抓取”后的nav abort为何触发RED_EXIT。以状态机事件及真实载荷证明是否误套停靠后出口假设；加入故障注入测试，再改恢复分支。检查迟到action结果、取消确认与新目标身份，避免旧动作污染新动作。
3. 仅在隔离候选中调整B->red3安全锚点/已验证接近侧，或同色顺序风险成本。先同题5/5，再回归上表两道成功题；没有复验就不合入。禁止扩大超时、缩碰撞体或放宽验收遮住问题。

## 四、CoStudio落地：先真实数据接入，再布局与计时

先确认官网下载的CoStudio实际版本和支持的数据源，不要把CoStudio、Foxglove、CodeScene当同一产品。官方入口：https://www.coscene.cn/ ，安装资料：https://docs.coscene.cn/docs/client/apt-source-install/ 。官方扩展模板：https://github.com/coscene-io/create-coscene-extension 。上述来源只证明产品/安装/扩展能力，不证明竞赛强制要求。

实施分四步：

1. 在Windows使用CoStudio，ROS VM提供受控的本地网络桥接。按实际版本支持选择桥：先验证rosbridge或官方cobridge兼容性，记录协议、版本、启动命令、绑定地址和端口，不能凭名称混接协议。先只订阅低带宽std_msgs/String题目，证明连接、断线重连和晚加入订阅有效，再接图像/TF/地图。桥接方案确定后再写可复用launch与使用说明。
2. 复用已有真实话题。题目 /competition/task/question；计划 /competition/task/plan；每色数量 /competition/task/counts（UInt32MultiArray，顺序red、blue）；可读顺序 /competition/task/order；模型状态 /competition/task/llm_status；任务 /competition/mission/state；导航 /competition/navigation/state；视觉 /competition/perception/targets；夹爪 /competition/gripper/state；避障 /competition/avoidance/state。实际类型必须用ros2 topic info确认，图像、map、tf按实机发现，不臆造主题。
3. 计划和状态目前有JSON封装在String.data中，不能假定面板直接能按嵌套字段取值。先测试现有数据转换能力；必要时编写只读适配节点输出明确字段的话题，或使用官方TypeScript扩展解析，异常JSON显示未知/错误而非沿用旧值。completed、正在操作的current_index、已抓取和已投放是不同语义；截图“第4个”不能简单当成完成4个。为消息增加task_id关联、时间戳/数据新鲜度，不把上轮状态显示成当前结果。
4. 保存布局导出文件与复现说明到docs/costudio/：顶栏题目；红/蓝数量和目的地；抓取顺序；双识别图；地图/机器人/路径三维视图；任务进度、工作状态、末端状态；API来源和延迟；比赛剩余墙钟。开始按钮只发一次具有新task_id的题目/开始请求，沿用校验与运行中拒绝重复逻辑，不发布cmd_vel。先用回放验证状态映射，再现场运行；断线/旧数据必须醒目标注。保留现有dashboard作为迁移对照，CoStudio全部验收后再决定默认展示入口。

验收：截图各栏目随真实任务更新；蓝先/红先均正确；重复开始不创建第二个任务；跨轮计数清零；相机重连不显示旧帧；TF/地图一致；图像仅在可视化链做适当降采样，不降低机器人感知/安全输入。对比开启前后CPU、网络、RTF和任务墙钟，记录展示开销。

## 五、API接入大模型：已有HTTP客户端，补齐生产接口

本轮源码审计：task_parser_node.py 已通过requests.post向base_url追加 /v1/chat/completions，默认本地8081；只有Content-Type，没有Authorization配置；timeout_sec默认45s。直接把含/v1的供应商base_url填入会重复拼路径。当前只能认定“已有本地兼容API调用”，不能声称云API已接好。

具体实施：

1. 按用户可用供应商选择兼容API，先核对其官方鉴权、endpoint、model、JSON输出和限流文档。不指定不存在的模型，不要求把key贴入对话。从进程环境变量或本地不入库凭据读取key，缺失时明确报错；配置中仅存环境变量名。认证头、日志、llm_status和保存响应都必须脱敏。
2. 抽离一个共享API客户端供request_llm和request_llm_problem使用：明确定义完整endpoint或规范化base_url，测试根路径与/v1路径；支持供应商指定鉴权方式。拆分连接/读超时，并加整个解题阶段的单调墙钟预算。对429/5xx只做预算内有限重试；401/403、模型不存在、schema/顺序错误立即显式失败。避免45s超时乘多次请求吞掉全场预算。
3. 真正的题目与映射规则一起发给模型，返回严格有序items。保留现有schema、颜色、数量、区域和确定性答案交叉检查；计算器仅作验算，require_llm=True时不能代替模型解题。失败不发布可执行计划，UI显示具体状态；模型恢复后原提交可重试，成功同task_id去重，新轮用新task_id。
4. 在线请求不能长期阻塞ROS状态/安全回调。将I/O放到受控工作线程或异步任务，由ROS执行线程发布结果；每个请求绑定task_id和generation，取消/新轮后迟到响应不得覆盖新题。限制只有一个在途解题任务，禁止重复按钮并发收费。必要实现先以模拟HTTP服务测试，不直接用收费API做故障轰炸。
5. 验证清单：正常两种颜色顺序；非法JSON/颠倒顺序/错误数量；缺key、401、429、500、连接和读取超时；用户取消后迟到响应；恢复后重试；同题新轮。最后少量真实API请求，记录供应商/model、请求id、延迟、tokens（若提供）与验算结果；不记录密钥。

当前还缺用户可用供应商与凭据配置，先实现可配置接口、脱敏和本地模拟测试；真正在线调用需要已有有效配置，不能假装完成。完整比赛同时启动真实API与CoStudio，从出题前比赛开始计时到第五次合格投放；保留运动耗时、推理耗时和总墙钟三个字段。运动<=270s仅为建议余量，最终以实测API延迟和波动确定。

## 六、300秒持续优化与交付标准

正确性修好后，按固定颜色顺序做同色子集/顺序/已验证接近侧DP；成本涵盖实际出口侧、目标物块、带载目的地与投放槽位、下一颜色首腿。缓存带地图/配置哈希。统计真实规划长度、转角、停转、抓放和重试耗时，不能把waypoint链下界当Smac路径。先困难族同机交替A/B至少三组，再按版本单独覆盖48题，记录所有失败，不跨版本跳过completed任务。

保持原安全边界、真实物理、墙体、双障碍、RGB-D/激光与Collision Monitor；保留出生点(13.2,-0.95)与匹配AMCL。既有抓放set_entity_pose会影响最终真值，需另记首次修正前物理误差，不能据修正后坐标宣称纯物理精度。未获得证据前不要并行带载伸臂和行走来冒险提速。

复现入口：先source ROS与workspace，再运行 bash tools_demo/scripts/run_ordered_case.sh 'Move 1 blue cube to zone A and 4 red cubes to zone B.'（从仓库根目录）。脚本发现已有gzserver会退出，只清自己进程组，每轮独立目录。CASE_ACCEPTED与motion_under_300不同；full_competition_acceptance目前not_tested。旧run48_batch2脚本覆盖证据/跨版本跳过的问题未修，不直接恢复它。

每轮完成：相关行为测试、全功能回归、必要完整仿真；更新AGENT44_WORKLOG与验证报告、当前提示词；备份原件与原始日志；明确文件提交推送并核对远端HEAD。备份在VM /home/ros/dev_ws/backups 与Windows audit-evening-20260905/backups，现有bundle/tar及SHA256勿覆盖。恢复在新目录，不reset当前未保存工作。最终报告区分已修复、已验证、未完成，不把建议当成果。
