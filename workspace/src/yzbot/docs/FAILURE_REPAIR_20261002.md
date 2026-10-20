# GUI48失败逐项修复状态（2026-10-02）

## 固定原始结果

批次`policy_20261001T233650`于04:49:24结束，38/48通过，10失败。
源目录：`/home/ros/huxuyang/YZBot-224/workspace/src/yzbot`。
运行时存在其他开发者脏改动，不能将全部差异归为本次修复，也不能保证
原48轮源码始终冻结。原始results/progress/launch日志保留。
开始本轮修改前全tracked/untracked源码保存至
`runs/repair_20261002T045354/source_before.tar.gz`。
保留用户物理步长、轮关节、传感器、策略及其他代码变动，不提交推送。

## 全部失败矩阵

launch日志目录均为`/home/ros/huxuyang/YZBot-224/runs/`下对应manual目录。

| 轮 | 标签 | 日志目录 | 原因/证据 | 当前状态 |
| --- | --- | --- | --- | --- |
| 5 | 3rA_2bB | manual-20261002T000018 | 障碍1净空-0.1411；放行后2.99s在(8.207,0.058)辅助倒退 | 短批通过174.3s/净空0.1956；偶发根因未证明消除 |
| 9 | 1rA_4bC | manual-20261002T002145 | 障碍1净空-0.0216；安全停后在(8.443,0.269)倒退 | 短批通过194.4s/净空0.2851；偶发根因未证明消除 |
| 13 | 3rA_2bC | manual-20261002T004425 | 搬运3/5；BLUE_CUBE_5离场两次回退后cost99，holding_cube失败 | 待离场位置、真实costmap/回退距离分析及修复 |
| 19 | 2rB_3bA | manual-20261002T013036 | 障碍1净空0.0235 | 待完整轨迹验证 |
| 20 | 3bA_2rB | manual-20261002T013646 | BLUE_CUBE_4导航lethal start7次、重试3次；253.3s完成但验收失败 | 待导航起点/离场栅格分析及修复 |
| 22 | 2bA_3rB | manual-20261002T014940 | 障碍1净空-0.0667；在(8.344,0.218)倒退 | 倒退准入回归通过，物理待验 |
| 24 | 1bA_4rB | manual-20261002T020250 | 341.9s；两次预对齐超时、RED_CUBE_4重试；启动还有Gazebo服务超时 | 待耗时分解，不全归为启动/GUI负载 |
| 26 | 4bC_1rB | manual-20261002T021612 | 离场卡住2/5；BLUE_EXIT lethal start1及重试2；随后bounded recovery | 待回退执行/失败期限分析及修复 |
| 35 | 2rC_3bA | manual-20261002T031113 | 障碍1净空0.0640 | 待完整轨迹验证 |
| 48 | 1bB_4rC | manual-20261002T044326 | 障碍1净空-0.1519；在(8.301,0.048)倒退 | 倒退准入回归通过，物理待验 |

## 第一处修复：辅助倒退先检查完整扫掠路径

新增`navigation_math.mover_reverse_segment_clear`，以2cm取样和1cm保守缓冲
检查整个计划倒退段到既有障碍巡逻线的距离，最低中心距0.60m。
`task_navigator._begin_obstacle_reverse`先检查动态扫掠区及静态栅格，
通过后才取消Nav2和发倒退；拒绝时不计已执行次数、不改goal、不发倒退。
不是禁用恢复，安全位置的恢复依然可用；场景和290s/0.10m评分未改。
也不表示Nav2保留控制即可保证物理安全，仍需闭环轨迹验证。

`test_mover_reverse_sweep.py`使用四个记录位置调用真实恢复方法，修复前4失败，
修复后通过，追加安全/危险端点和第二障碍检查。
导航契约与初版回归107通过；功能回归578 passed,1 skipped,2 deselected
（排除flake8/pep257）；tools_demo colcon build成功。
旧BC契约的现有用户改动已在本轮前存在，不将其归为本修复，也没有盲改断言。

## 未采用的方案

将整条穿越线抬到y0.70：虽然障碍扫掠净空0.14m，但mapn3北墙距中心
仅0.276m，静态风险过大，因此未修改路线。
过往单纯调相位/恒速回放不足以覆盖Nav2慢行/停车/恢复，本轮不以此替代物理测试。

## 已完成两例实验

`runs/repair_20261002T045354/launch_probe.sh`仅跑3rA_2bB及1rA_4bC两例GUI，
源码快照`reverse_guard_source.tar.gz`，日志`driver.log`，只读20Hz轨迹
`crossing_trace.jsonl`包含odom、原始/碰撞安全/最终速度及障碍1真值到达时刻。
批次`policy_20261002T050003`于05:12:05完成2/2通过，探针和runner已退出。
3rA_2bB为174.3s/净空0.1956；1rA_4bC为194.4s/净空0.2851。
两例都没有抓取失败、lethal start、导航重试。
轨迹记录了第一例从东向西穿越时约1.65m/s、障碍退到南侧；
两例日志没有触发新增倒退拒绝，故通过不能证明修复是唯一原因，
也不能证明慢行/停车时相位窗口充分。必须继续覆盖其他碰撞标签。
新增测试格式E128/E501已在短批结束后修正，flake8通过。

## 当前剩余失败集合实验及下一步

`launch_failure_set.sh`覆盖其余8个原始失败，控制代码不再变动：
2rB_3bA、2bA_3rB、2rC_3bA、1bB_4rC（碰撞/净空）在前，
3rA_2bC、3bA_2rB、1bA_4rB、4bC_1rB（其他失败）在后。
新增只读`mission_trace.py`采集全地图位置、速度三级链路、base_hold、
mission/navigation状态及local/global costmap沿底盘前后0.6m的栅格探测。
代价地图探测以odom坐标取值，必须检查frame/TF一致性后解释，
不能把不同时刻或不同轮次的缓存数据当实时证据。
输出`failure_set_trace.jsonl`，源码`failure_set_source.tar.gz`，
驱动日志`failure_set_driver.log`。从日志读取实际policy目录及活跃PID。
不启动第二批、不在运行中修改文件；这仍是诊断/失败覆盖，不是完整48重测。
第13轮BLUE_CUBE_5两次离场只移动约2cm并航向从0.019漂到0.375，起点cost99；
第26轮BLUE_CUBE_2在cost83交给Nav2后立即lethal start。
下一步区分安全链路抑制、底盘接触/姿态漂移、代价地图/TF不一致三种原因，
避免仅通过放宽阈值或增大退让距离使测试表面通过。
必须根据实际轨迹定位碰撞发生阶段，进一步处理门控和侧后方保护。
然后逐例修复矩阵其余问题，GUI覆盖全部失败集合；全部候选修复验证后
冻结源码/参数，再启动用户授权的完整48重测。当前尚未满足该启动条件。

## 06:10失败集合验证结束（仍非完整48）

批次`policy_20261002T051939`于06:10:52结束8/8通过，runner146369和
探针146368已退出，无第二个campaign。源码快照SHA256：
`dd8820f55820187a949e50974b0c22c465dae9b0a5550ad1e0a7b98f4eb88c48`。

| 标签 | 耗时s | 最小净空m |
| --- | --- | --- |
| 2rB_3bA | 227.7 | 0.2400 |
| 2bA_3rB | 235.8 | 0.3752 |
| 2rC_3bA | 222.9 | 0.2498 |
| 1bB_4rC | 282.3 | 0.3301 |
| 3rA_2bC | 176.0 | 0.2452 |
| 3bA_2rB | 209.1 | 0.3343 |
| 1bA_4rB | 240.4 | 0.3683 |
| 4bC_1rB | 224.0 | 0.2404 |

这8例lethal start、抓取失败均为0；1bB_4rC导航重试2次、carry_invalid8次，
并在large_rock附近执行bounded recovery，耗时距290s仅7.7s，仍有尾部风险。
其他7例导航重试0。日志均未出现新增obstacle_reverse拒绝/执行，不能证明
通过由倒退准入修复造成。原10标签在两个短批通过，不等于单批48/48。

新轨迹的成功离场对照：3rA_2bC的BLUE_CUBE_5从(2.233,-5.799)
退到(1.867,-5.801)，航向约0.011至-0.005；4bC_1rB的BLUE_CUBE_2
从(-9.630,-2.999)退到(-9.261,-2.998)。新离场非零原始速度样本中未观察到
新鲜最终速度压零，说明这次实际执行了约0.37m退离；这不能排除原失败的
安全压零或物理接触，因为原失败没有三级速度轨迹。
探针文件混有ROS非JSON行，分析时必须跳过；障碍数据的新鲜度与odom不同，
不能用仅保留0.3s新鲜样本算全程最小净空（会漏穿越），验收以原始results为准。

下一步仍先构造离场失败正确调用点回归，核对真实footprint成本及TF，
分析原24耗时分段和新48标签的large_rock恢复。尚未修复的偶发问题保留在
上方矩阵，不因一次通过移入已解决；暂未启动完整48重测。

## 06:15根因校正及耗时分段

原24总341.9s对照新同标签240.4s：RED_CUBE_1取块导航原125.9到181.0，
55.1s；新109.9到131.7，21.8s。RED_CUBE_4原213.4到275.8，62.4s；
新159.3到186.6，27.3s。两段额外68.4s，占总差101.5s的约67.4%。
RED_CUBE_5原23.2s，新5.5s，另多17.7s。主要尾部在取块导航，
而非归因于Gazebo启动耗时（启动在任务elapsed之外）。需对照两次
预导航对齐超时与RED_CUBE_4重试的时间戳，再锁定速度链/规划恢复原因。
新1bB_4rC最后取RED_CUBE_1为196.5到253.4，56.9s；large_rock恢复
发生在该段，不能把这例282.3s通过当作性能风险消除。

重要校正：当前配置为SmacPlanner2D，不是通用多边形footprint规划器。
Humble上游node_2d.cpp调用索引版inCollision，collision_checker.cpp索引版
按中心原始cost>=INSCRIBED判定，不逐点检查footprint。
因此“cost83因footprint边缘致命”目前不是正确结论；不据此修改阈值。
来源：https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/node_2d.cpp
及https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/collision_checker.cpp。
具体安装补丁版本仍需核验。现有trace仅OccupancyGrid沿线探测，未记录raw
Costmap及map->odom TF；不能据此证明原第26轮缓存cost83与规划器当时中心
原始成本相同。下一步正确回归/探针须覆盖缓存更新时间、raw成本和TF，
而不是构造任意cost83就应禁止Nav2的测试。原失败缺三级速度证据，离场接触
原因尚未定论。本轮未改控制代码，也未启动新测试批次。

## 06:20时间戳核对及精密复现

预对齐不是32秒：源码单独参数默认4秒，原24日志1790878030.142到
1790878034.121约3.98秒；另一处1790877997到1790878001也约4秒。
32秒是直接dock/离场的期限，不能混用，未修改预对齐参数。
RED_CUBE_1真正Nav2段8034.175到8081.689约47.5秒，无导航重试；
RED_CUBE_4在慢进展旋转后两次重试，位置分别约(0.78,-6.06)及
(0.65,-5.97)，第二次Nav2却立即记录(1.21,-5.60)，相差约0.67m。
这表明位置/TF时效值得复现，但原日志尚不能确定是哪一路滞后。

新增实验目录外只读`precision_trace.py`：同时记录map<-base_link、
map<-odom的时间戳和位姿，以及local/global原始Costmap中心格成本（以
odom及TF两种位置分别取值）。保留旧探针和日志，不修改控制源。
静止场景6秒smoke得到94条记录、两种raw地图及TF，map<-odom为恒等；
早先head截断stdout造成的BrokenPipe是诊断命令输出截断，不是ROS启动失败。
`launch_precision.sh`仅3rA_2bC、4bC_1rB、1bA_4rB三例GUI复现，
源码precision_source.tar.gz，轨迹precision_trace.jsonl，输出precision_driver.log。
启动后读取输出确认实际policy目录/PID，保持代码冻结直到结束。
它用于复现离场/导航尾部，不是完整48；没有宣称新的运行修复已完成。

## 精密批次完成及第二处修复：应用代价地图增量更新

`policy_20261002T062402`于06:43:39完成3/3，通过耗时分别215.8、239.1、
236.5s，最小净空0.2817、0.2857、0.1985m；第二例导航重试2次，其余0。
离场失败未复现，不以再次通过宣称原接触/压零原因已确定。

但复现了全图缓存错误：1790893992.768附近odom(0.284,-0.659)，
导航器完整OccupancyGrid缓存约152秒，中心值99；同期raw全局成本234
（对应非致命膨胀），原始图刚收到约0.04秒。导航器没有订阅
`/global_costmap/costmap_updates`，使避障/离场/恢复检查使用旧成本。
缓存可错误阻止已清空起点，也可遗漏新增加的致命格；这是双向安全风险。
不能用改99阈值或清空地图掩盖。

`test_costmap_updates.py`复现全图83收到增量0/99/100仍返回83及两行stride，
修复前4失败4通过。新增导航器增量订阅和校验回调，逐行应用完整有效patch；
无完整图、错误frame/尺寸/越界/数据长度拒绝，不改变原安全判断阈值。
`tools_demo/package.xml`补map_msgs运行依赖。
修复后导航契约及增量111通过；功能586 passed,1 skipped,2 deselected；
tools_demo colcon build通过，新增测试flake8及git diff --check通过。
此修复尚未完成真实Gazebo部署验证，下一步启动短GUI失败集合前冻结源码。

探针限制：独立precision_trace使用墙钟TF Buffer，不随Gazebo仿真时间
倒退清缓存，第二/三轮曾保留上一轮更大的TF时间戳，产生十几米假差异。
这不是TaskNavigator已证实的TF漂移（每轮重启）；不能使用那些map_tf成本
或最大距离归因。第一轮TF差约0.031m；原运行日志0.67m差仍待复现。
后续探针必须按轮重建TF缓存，或使用仿真时钟/时间跳变清缓存并验证；
odom/raw图仍须按task、接收年龄与仿真时间区分。旧文件全部保留。
当前无活跃campaign，新完整48仍未启动，不提交推送。

## 第二处修复GUI部署验证（08:40完成，3/4通过）

只读`costmap_guard_trace.py`通过/clock倒退显式清除动态TF、地图和状态缓存，
恢复静态TF，输出generation；/clock按实际BEST_EFFORT订阅。
同时调用真实TaskNavigator完整图/增量回调，镜像缓存成本和更新计数，
对照raw中心成本及运行日志（不把原始0..255和OccupancyGrid0..100直接比较）。
时钟10->1调用点验证generation1和清缓存；5秒无信号spin循环验证收到增量、
时钟并正常退出。timeout向rclpy发INT时曾在take_message输出转换异常，
干净退出无异常，不能与运行时崩溃混淆。
`launch_costmap_guard.sh`冻结costmap_guard_source.tar.gz，四例
3rA_2bC、4bC_1rB、1bA_4rB、1bB_4rC，覆盖离场、超时和碰撞代表，
驱动costmap_guard_driver.log、轨迹costmap_guard_trace.jsonl。
启动后以驱动日志核对真实policy/PID，运行中不再改文件。
这仍是针对性GUI验证，不是完整48，所有未定根因仍保留。

批次`policy_20261002T081417`于08:40:14结束；runner175104及probe175103
已退出。冻结源码costmap_guard_source.tar.gz SHA256：
`796f366bf4ed858e67e3dcc0064fcb11e4078a888a631e5cd1cc7353300ef2a0`。

| 标签 | 耗时s | 最小净空m | 原始判定 |
| --- | --- | --- | --- |
| 3rA_2bC | 183.2 | 0.2269 | 通过，重试0 |
| 4bC_1rB | 271.3 | 0.2404 | 失败，lethal start1，重试4 |
| 1bA_4rB | 241.2 | 0.2729 | 通过，重试0，carry_invalid1 |
| 1bB_4rC | 262.3 | 0.2497 | 通过，重试0 |

四例均5/5完成，无抓取失败/障碍碰撞，但不得忽略lethal判定。
关键阶段校正：新4bC_1rB失败发生在取BLUE_CUBE_2途中，不是BLUE_EXIT
离场。1790900719.605从C离场后开始取块导航；首次RPP碰撞预测
1790900734.369，两次controller patience及3秒wait后，1790900741.935
Smac报告起点致命。当时机器人在(-12.237,-4.783)，目标(-9.24,-3.00)。
旧08:31消息误记为离场，已以原始日志纠正，不据误判修改离场代码。

正确轮次generation2，map<-odom恒等，map<-base与odom一致；新鲜raw
全局中心253、增量镜像99持续约7秒一致，而local原始中心206。
全局清图后raw234与镜像91同步。因此本次失败不能归因于旧全图缓存。
probe镜像使用真实回调，但不是runtime内部缓存直接导出，不夸大部署证明。
clock generation0/1/2/3/4均有样本，跨轮旧未来TF问题未再次用于归因。

外置只读inspect_lethal_trace.py过滤失败时间窗并断言不得存在新鲜致命中心：
实际运行得到139/139新鲜致命样本、AssertionError（退出1，约1.3秒）。
这是已捕获症状的快速红信号，不是候选修复可转绿的闭环控制回归；
尚需捕获实际Nav2 path并建立正确调用点/闭环实验，不能靠修改过滤条件变绿。
后续三个可证伪方向依次为高速转弯跟踪偏差、全局/局部障碍覆盖差异、
规划路径绕行与局部路径处理。当前trace没有Nav2完整path，无法判断切角。
本轮没有再改运行代码，没有启动新48；保留两处修复及全部原始证据。

## 08:48只读路径复现准备

当前运行节点实际订阅/global_costmap/costmap_updates已由ros2 node info验证。
新增外置path_trace.py：保留时钟回退/速度/raw地图探针，订阅实际存在的
/plan、/plan_smoothed、/received_global_plan；路径消息独立记录完整点列及
同期raw成本，20Hz样本记录到最近线段的偏差和路径年龄。BLUE_CUBE_2
中心raw>=200时每秒记录两地图2.4m局部裁剪，不修改任何控制/场景参数。
map/odom坐标换算只在本场景恒等TF证据下比较，不泛化到任意地图。
L形路径调用点验证最近线段偏差0.5m；5秒真实spin收到7个地图增量，
PATH_PROBE_SMOKE_PASS、正常退出。启动前无其他campaign/probe活跃。
launch_path_probe.sh仅4bC_1rB单例GUI，冻结path_probe_source.tar.gz；
driver为path_probe_driver.log，轨迹path_trace.jsonl。该实验用于补缺失证据，
不是新控制修复或完整48；实际PID/policy目录以driver启动输出为准。

## 09:00路径单例结束及诊断坐标校正

policy_20261002T084832于08:56:44结束1/1通过：230.0s，净空0.2406m，
抓取失败/致命起点/碰撞均0，导航重试2。runner187582/probe187581退出。
源码path_probe_source.tar.gz SHA256：
`511a6a88a607665b3196b4ccdc874d19650849efcca45f06f7f9b381ed670d3e`。
这次未复现新lethal，不能据此将偶发问题列为已解决。

轨迹9563条sample、1713条path（/plan18，/received_global_plan1695），
9条ROI；/plan_smoothed未收到消息。BLUE_CUBE_2的全局/plan于
1790902459.317发布，map帧178点，最西x=-12.685，同期raw全局采样
最高成本43。真实取块Nav2 stage1的同一map路径最大偏差0.269m；
不存在足够证据把该成功路径当作上一失败批当时的规划路径。

重要探针校正：/received_global_plan为base_footprint帧，旧probe直接将
该局部点列与map坐标odom比较，导致12~13m假偏差，且同坐标raw路径
成本无效，不能拿来归因。真实调用点测试先红（12.924m假差），外置
path_trace_frame_safe.py拒绝未转换帧的偏差/成本推断后绿；map帧L形
0.5m断言仍绿。旧轨迹保留，原/plan map指标不受此错误影响。
这仅修复诊断探针，不是机器人控制修复。下一步若使用局部路径需在
消息时间戳下做坐标转换后比较；未转换帧一律标记不可解释，不再产生假差。
目前无新控制修改，无新完整48，余下所有原失败根因仍需逐项闭环验证。

## 09:10第三处单变量修复：门控不得在不安全扫掠区等待

修改前源码备份gate_wait_before.tar.gz。新增test_east_gate_wait_sweep.py
调用真实SafetyLimiter.east_gate_blocked：x=9.00/9.02/9.04、y=0.10时
非安全相位会令门控停车，且放行后相位失效会重新锁门。
这些等待位置到actor1整个[-0.8,0.1]巡逻线的中心距离仅0.50~0.54m，
扣0.28+0.18两半径后净空0.04~0.08m，不能满足原0.10m标准。
实际测试4 failed,3 passed，耗时0.26s；这证明一个真实门控不变量缺陷，
并非证明原全部碰撞都发生在该位置（原倒退位置x8.2~8.44）。

仅将safety_limiter门控等待/重新锁门下界x9.0恢复为x9.3；上界9.75、
y范围、相位释放、cone、限速和290s/0.10m评分不变。在更靠近巡逻线
的位置不再追加门控等待，独立方向性安全缩放仍保留。安全等待位置
缺失真值仍fail-closed。侧后保护、相位失效后的穿越仍不能视为已解决。

新回归7通过，结合旧safety及reverse回归23通过；功能593 passed,
1 skipped,2 deselected；tools_demo构建、新测试flake8、git diff --check通过。
尚未证明物理碰撞消除，需覆盖全部6个原碰撞/不足净空标签。
launch_gate_wait_probe.sh将冻结gate_wait_source.tar.gz并运行六例GUI：
3rA_2bB、1rA_4bC、2rB_3bA、2bA_3rB、2rC_3bA、1bB_4rC。
driver gate_wait_driver.log，轨迹gate_wait_trace.jsonl，使用帧安全只读probe；
不改导航路径/参数。活动期间冻结所有文件，实际目录/PID以driver为准。

## 09:50六例结束：安全等待边界仍不足以消除碰撞

policy_20261002T090938于09:49:25结束3/6；runner191684/probe191683退出。
3rA_2bB为236.9s/净空-0.0062，2rC_3bA为224.2s/-0.035，均障碍1碰撞。
1rA_4bC、2rB_3bA、2bA_3rB通过194.3、243.1、234.2s。
1bB_4rC完成5/5但304.3s超时，净空0.2891，导航重试4，无lethal。
不能将完成搬运等同安全通过，全部原10项仍按矩阵跟进，暂未开启完整48。

第一例轨迹：1790903573.376门控释放前机器人(9.712,0.066,yaw=-2.883)，
原始v=0.774/w=-1.833却最终v=0/w=0；放行后边走边转，3574.52在
(9.007,0.315)原始v=0/w=1.533，3575.52在(8.910,0.241)再停。
3576.076方向安全停，3576.745/3577.750/3578.750新增倒退准入拒绝，
未执行危险倒退；3578.02最终v=0而原始v=1.65，actor回到北端碰撞。
最小点actor(8.5,0.055)/robot(8.919,0.229)。这证明倒退guard实际触发，
但保留Nav2仍会停车被撞；不是所有原碰撞已修。
第五例最小点actor(8.5,-0.215)/robot(8.343,0.18)，重试2/carry_invalid8。

## 第四处单变量候选：安全相位等待允许上游安全转向

修改前保存gate_steering_before.tar.gz。候选排序：等待角速度被清零导致
延迟对齐；穿越路径跟踪停转消耗相位；方向安全停车后滞留动态扫掠区。
先检验第一项，不将其认定唯一碰撞根因。
test_east_gate_steering.py调用真实cmd_vel_callback及真实east_gate_blocked，
安全等待点x9.6输入记录v0.774/w-1.833和反向w1.533，两例均角速度
错误清零，修前2 failed/2 passed。仅闭门分支保留输入angular.z，平移
仍清零；base_hold和上游角速度零保持零，不绕过laser monitor。
功能597 passed/1 skipped/2 deselected，tools_demo构建、新测试flake8、
git diff --check通过。仍须GUI验证，不是已证明物理修复。
launch_gate_steering_probe.sh将冻结gate_steering_source.tar.gz，测试
3rA_2bB、2rC_3bA、1bB_4rC；轨迹gate_steering_trace.jsonl，驱动
gate_steering_driver.log，核对等待朝向、放行后穿越停顿和尾部耗时。
BLUE5离场、BLUE4/BLUE2 lethal及RED1/4尾部仍未定论，不因这次候选跳过。

## 10:15第四候选GUI三例完成，仍不等于全部根因已修

policy_20261002T095336于10:12:32结束3/3，runner206352/probe206351退出。
3rA_2bB：181.6s/净空0.1900，重试0；2rC_3bA：274.5s/0.2009，
重试2；1bB_4rC：222.2s/0.2420，重试0；三例抓取失败/lethal/碰撞均0。
冻结源码gate_steering_source.tar.gz SHA256：
2278302846c67a45465a530774455e9883cef3b7d6950e2c5352af841542a769。

第一例1790906157/6159时安全等待在x9.68，最终平移0且角速度与上游
一致；yaw从2.543转至3.057。6161.736释放前yaw2.915、输入角速度近0。
6162.175机器人(9.069,0.242)，6162.675已(8.240,0.181)，速度1.65；
6163.175到(7.411,0.121)，实际通过时没有旧轨迹停转。该证据支持
门控转向修复，但单次成功仍不保证任意相位/全部6碰撞标签安全。

第二例RED_CUBE_4在(-1.813,-3.744,yaw2.091)被RPP预测碰撞停止，
1790906523原始cmd已全零，不是最终安全链另行压零；同期global raw215、
local206。之前路径跟踪偏差0.236m。6528/6539慢进展重试两次，原始
规划重新开始后仍碰撞；6541第二重试路径朝向变化后恢复，通过。
此尾部是新的可观测导航停顿，不能以本例274.5<290宣称性能问题修复。

下一批仅验证剩余7个原失败标签，控制代码无新增修改：1rA_4bC、
2rB_3bA、2bA_3rB、3rA_2bC、3bA_2rB、1bA_4rB、4bC_1rB。
外置remaining_failure_trace.py保留同frame路径/clock清缓存，扩大只读ROI
捕获至任意active导航global中心raw>=150（1秒一次），用于区分路径
跟踪偏差、局部/全局覆盖与起步朝向碰撞；不改变导航安全和验收。
remaining_failure_source.tar.gz、remaining_failure_driver.log及
remaining_failure_trace.jsonl保留。尚未开启完整48，不拼两短批为48/48。

## 11:05剩余七例结束：均通过，非碰撞偶发根因仍未定论

policy_20261002T101848于11:01:08结束7/7，runner213766/probe213765退出。
冻结remaining_failure_source.tar.gz SHA256：
3430eb196a263f9b3b126edcd22e691f7ebb6ad94da4201c675e1d81360eba6a。

| 标签 | 耗时s | 最小净空m | 导航重试/致命起点/抓取失败 |
| --- | --- | --- | --- |
| 1rA_4bC | 188.5 | 0.2248 | 0/0/0 |
| 2rB_3bA | 191.3 | 0.2852 | 0/0/0 |
| 2bA_3rB | 213.9 | 0.2303 | 0/0/0 |
| 3rA_2bC | 187.5 | 0.2850 | 0/0/0 |
| 3bA_2rB | 205.0 | 0.2352 | 0/0/0 |
| 1bA_4rB | 231.3 | 0.2401 | 0/0/0 |
| 4bC_1rB | 201.5 | 0.2425 | 0/0/0 |

均5/5完成；3bA_2rB carry_invalid1、4bC_1rB carry_invalid2，其余0。
探针50328个sample、9657个path、107个ROI，跨轮generation0..6，旧证据保留。
原10标签已在当前四处候选修复版本的两个短批通过，但不能拼为完整48；
离场失效和lethal本批没复现，不能据此证明根因被第四门控修复消除。
特别是095336第二例RED4仍两次RPP预测碰撞停顿，需要正确控制回放。
当前无活跃campaign，不提交推送，也未满足“全部根因修复”后完整48条件。

下一诊断优先建立实际Nav2控制器输入回放：使用真实已捕获map帧path、
机器人pose/yaw/速度及同一时刻地图，不修改过滤器或阈值制造绿结果。
需区分控制器原始碰撞预测、速度平滑曲率变化、路径与地图覆盖差异；
不能凭参数注释直接改速度或切换控制器。若地图/路径时刻不够，明确缺口，
使用已捕获ROI及正确frame继续补证，不把成功路径当作失败当时路径。
原BLUE5/BLUE2离场原三级速度证据缺失仍未确认，全部矩阵保留未定论状态。

## 11:15原生速度平滑器调用及控制器输入补证

外置native_replay由CMake链接已安装nav2_velocity_smoother 1.1.18共享库，
在隔离ROS_DOMAIN_ID=77配置但不激活，不向Gazebo发速度。
实际调用findEtaConstraint/applyConstraints使用运行20Hz及accel/decel。
输入当前(v1.65,w0)、目标(v0,w2.5)，独立轴输出(v1.425,w0.25)，
同比例eta0.1输出(v1.485,w0.25)。目标(v0.75,w1.9)时独立(v1.425,w0.25)，
同比例(v1.53158,w0.25)。这不支持盲目开启scale_velocities可消除残余平移。
只是算法诊断，不是原失败的真实controller输入回放；参数不修改。
初版harness在configure前set_parameter出现未声明参数，改用NodeOptions
overrides后构建/调用成功；该外置错误不是机器人启动失败。

旧trace的/cmd_vel已是平滑后速度，且095336停顿未捕获完整local raw图。
仅有ROI和后平滑输出不能准确复现RPP碰撞预测，禁止制造假红绿。
新增外置controller_inputs_trace.py订阅/cmd_vel_nav，保存odom角速度、
map路径完整四元数、高成本位置完整local Costmap及同步前后平滑速度。
原有clock回退清缓存和frame保护保留，不修改控制代码。
4秒真实spin smoke确认/cmd_vel_nav存在、odom6字段、local图数据长度一致，
CONTROLLER_INPUT_PROBE_SMOKE_PASS。一次/秒高成本快照限制额外负载。
仅诊断2rC_3bA、3bA_2rB、4bC_1rB三例，补控制器/地图正确输入；
controller_inputs_source.tar.gz、controller_inputs_driver.log、
controller_inputs_trace.jsonl保留，启动后全部文件冻结。仍非完整48。
磁盘剩余约2.2GB，未删除用户数据/备份；后续完整48启动前须重查空间。

## 11:50控制输入三例结束及原生回放边界

policy_20261002T111629于11:39:42结束3/3，runner231344、probe231343及
parent231315均退出。2rC_3bA为225.4s/净空0.3301，3bA_2rB为
242.5s/0.2155，4bC_1rB为250.1s/0.2549；三例均5/5完成，抓取失败、
lethal、碰撞、carry_invalid均0，但各有2次导航重试，均发生在BLUE_CUBE_4。
故非碰撞尾部仍复现，不能因任务通过跳过根因。冻结源码SHA256：
92b9019dbd0d8dc50386074d7649b905e4958e185516520559307788860d4395。
探针27639条sample、5597条path、137条ROI含完整local raw地图。

第一例1790911440.649，机器人(-6.257,-6.597,yaw=-0.129)，实际odom
v=0.975/w=0.550；平滑前cmd_vel_nav已经v=0/w=0.633，平滑后仍
v=0.975/w=0.550。1440.661开始RPP碰撞预测，1441.050起机器人
停在(-6.164,-6.608,yaw=-0.096)，前后速度均0；1448/1456慢进展重试。
第二例停在(0.441,-0.953,yaw=-2.094)，1743/1751两次重试；
第三例2159/2167两次BLUE4重试。新的真实前后平滑速度可验证过渡，
但尚不能将残余平移认定唯一根因或直接开启scale_velocities。

外置native_replay/rpp_replay.cpp/CMakeLists已构建，加载安装Nav2 1.1.18
原生RPP及pluginlib RotationShim；ROS_DOMAIN_ID=77隔离、不激活发布器。
使用同generation最新map路径四元数、捕获odom位姿/速度、完整local raw图
和实际FollowPath参数，map<-odom恒等由该轮TF证据支持；只将时间戳重基准。
单帧1441.050与1760.049直接RPP均输出v0/w0.25，并未重现原碰撞异常。
重新初始化RotationShim在1760.049输出v0/w2.5，1440.649输出v0/w-2.5，
与真实当时cmd_vel_nav不匹配；插件版本原生回放还在清理阶段退出1，未证明
该单帧harness可用作红绿回归。不是机器人运行错误，也不能称回放已绿。

该程序明确是debug目录中的未完成诊断原型：重新初始化控制器不能还原
之前的路径接收、shim状态及控制周期。下一步须连续输入/状态回放并核对
控制器实际接收的路径及地图时刻，先得到正确的红信号，再作单变量修复；
不能用任意构造角速度、过滤失败快照或改安全阈值制造红绿。
当前控制代码/参数无第五处修改，无新GUI批次、无新完整48；全部原10矩阵
仍保留未定论项。磁盘11:31为2.1GB，完整48前仍需空间检查。

## 11:56真实RPP过渡状态已得到快速红信号

运行节点只读参数核对：FollowPath实际plugin为RotationShim、primary为
RPP、angular_dist_threshold=0.45、use_rotate_to_heading=true、
max_angular_accel=5.0，与回放源参数一致，未作参数修改。
此前停稳单帧未复现；改为原始记录的1790911440.649过渡状态（非构造速度）
直接调用原生RPP，得到与1440.661运行日志相同的
RegulatedPurePursuitController detected collision ahead异常。
程序在捕获该异常时明确返回1，这是红信号，不是清理错误。
重新初始化RotationShim的结果仍不能等同完整运行链，保留其状态缺口。

已由程序原样导出同generation最新map路径与完整ROI到外置
native_replay/blue4_transition_capture.jsonl，原始trace全部保留。
夹具SHA256：247b4f58f3a776d5eb757bae66579fe34f16fc1be6f2eb6f7e06da0f25460da3。
命令（先source /opt/ros/humble/setup.bash，实验目录为
/home/ros/huxuyang/YZBot-224/runs/repair_20261002T045354）：
ROS_DOMAIN_ID=77 native_replay/build/rpp_replay
native_replay/blue4_transition_capture.jsonl
/home/ros/huxuyang/YZBot-224/workspace/src/yzbot/bot_navigation/param/originbot_nav2.yaml
1790911440.649。
两次分别0.178/0.175秒、均退出1并输出相同碰撞异常；无激活发布器。
该反馈环锁定真实primary控制器的碰撞预测，不表示已复现整个物理碰撞、
lethal或所有原失败，也还没有候选修复绿结果。

待按单变量检验的排序及预测：
1. 若残余前进/速度相关lookahead是触发因素，保持同一捕获地图、路径、
   pose，仅对当前线速度作明确反事实诊断应改变碰撞判定；这不等于真实修复。
2. 若shim接收路径及切换状态是关键，相同捕获输入的连续状态回放应匹配
   真实转向正负与角速度，而重新初始化单帧不匹配；不能用其绿输出替代。
3. 若局部地图与规划路径覆盖差异是关键，保留原图时原生碰撞预测的位置
   应落在原路径/未来弧中的局部致命格；不能通过清图或禁用碰撞制造绿。
先最小化真实红夹具/补连续状态，再选择修复。控制代码仍四候选，完整48未启动。

跟进提示中旧历史段经本地JSON传输出现乱码，已通过app工具重新保存可读完整
提示；未改评分、任务代码或历史数据。后续读取自动化配置用ASCII转义JSON
避免终端编码再次损坏提示。

## 12:04两个反事实候选否证及真实碰撞投影

只改外置rpp_replay诊断选项，不改机器人源参数。保持原捕获路径、pose、
局部地图及当前角速度0.55，仅把当前线速度0.975设0（linear-zero），
仍抛出相同碰撞异常、退出1。因此残余平移/速度相关lookahead不是这个
捕获状态的唯一触发因素；它对进入危险位置的影响仍需连续链路验证。

另一个单变量primary-no-rotate只在外置NodeOptions关闭RPP朝向旋转，
RotationShim运行配置及碰撞检查不改，也仍抛出相同异常。不部署这个候选，
不声称红转绿或已修复。两个选项明确标记COUNTERFACTUAL，原夹具不修改。

外置collision_projection.gdb在隔离ROS_DOMAIN77原生程序的
isCollisionImminent/inCollision入口读取x86_64 ABI实际参数，标记
[DEBUG-rpp-capture]，不是附加到真实机器人进程。原配置预测v0/w0.8、
carrot0.53625；检查当前(-6.257423,-6.596787,yaw=-0.129168)，随后
同一位置yaw=0.040613就报碰撞（约0.1698rad纯旋转）。关闭primary旋转
后预测v0.805489/w1.833333，第一投影到(-6.207839,-6.603228,
yaw=-0.015366)，约5cm平移即报碰撞。调试器命令退出0不等于通过；
inferior仍退出01并捕获碰撞异常。符号PLT/实现断点双命中导致相同记录重复，
不把重复当两次独立事件。

该捕获local中心格成本151，最近map路径点距离0.2175m，最近致命格中心
(-6.125,-6.875)距机器人0.3081m；不能以中心151非致命就断言轮廓可旋转。
实际runtime local radius0.28、footprint空圆形配置、padding约0.01，与harness
一致。Humble RPP源码inCollision调用footprintCostAtPose并以>=254判断，
isCollisionImminent也投影原地旋转；这与Smac2D中心>=253判定是不同检查，
不反过来用RPP轮廓解释原Smac lethal。来源：
https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp
（安装1.1.18函数入口参数亦由本次gdb实测，不把分支源码当安装二进制全部等同）。

下一步优先重放到该位置之前的真实路径接收、shim状态、原始及平滑输入，
定位跟踪偏差进入轮廓受限位置的链路。单帧固定图连续诊断若地图时刻不足
须明确限制，不冒充全程精确物理回放。不能靠禁用轮廓检查/缩小半径或清图
消除红信号。无第五控制修改，无新GUI批次/完整48，所有原失败矩阵仍跟进。

## 12:13回放频率纠正与连续状态诊断

发现外置生命周期节点未声明server拥有的controller_frequency，仅传入
NodeOptions override未被RPP读取，导致旧harness沿用20Hz默认值；实际runtime
为15Hz。现明确按源YAML声明该参数，输出REPLAY_CONTROLLER_FREQUENCY 15。
这只修正debug程序，不改机器人代码或参数。15Hz原捕获夹具仍红；linear-zero
和primary-no-rotate也各重跑仍红，不部署这两个候选。gdb重新实测原预测
v0/w0.883333/carrot0.536250，纯旋转第二位置仍yaw0.040613即异常。
12:04记录w0.8属旧20Hz外置结果，不是实际运行角速度；几何投影结果未改变。

新增sequence诊断从同generation唯一map计划1790911437.655498接收后，
向同一RotationShim/RPP实例连续输入36个捕获odom pose/速度，至1440.649470。
调用真实插件、原参数15Hz、不激活发布器；清理正常。末帧捕获到同一
detected collision ahead异常并返回1，早期转向正负大体恢复，部分命令一致
（1437.949787 w-0.366667、1438.199630 w0.616667、1439.999748
v1.616409/w-1.833333），但其他帧仍不同。例如1437.699566预测w-0.124586
而捕获w-0.522422；1437.849085预测v0而捕获旧cmdnav v1.65。

明确限制：全段使用1440.649捕获的固定local图，不能冒充逐时刻地图回放；
采样墙钟与odom/cmd接收时间不同，命令可能跨控制周期持有。按odom接收时间
筛选15Hz近似步长不等于实际controller计算时刻，部分命令差异尚不能归为
控制器bug。没有通过改图、速度、轮廓或安全阈值制造绿。下一步优先对齐
cmd_vel_nav消息更新时刻及对应odom/路径接收，检查此前进到受限轮廓位置的
真实链路，再考虑单变量修复。无第五控制修复、无新campaign/完整48。

## 12:18命令接收配对诊断：32/44输出一致，尚非绿修复

只在外置rpp_replay加入sequence-command：按/cmd_vel_nav收到时间去重，
从已捕获样本选择该命令收到之前最近的odom，打印receive_age_ms。
不是插值或构造速度，也不是精确controller计算时刻。原先同一sample的
odom有时比命令晚到约44ms，会把后续状态拿来比较旧命令。

15Hz真实插件/固定1440.649 local图下，共44条捕获命令，32条输出v/w
在1e-5以内与实际相同（输出只保留6位）。例如1437.799515
v1.65/w1.552397，1438.799584 v1.442683/w1.833333，
1439.949611 v0.941115/w-1.833333，1440.599419 v0/w0.633333一致。
这支持先前若干差异来自输入时间配对，不支持任意改变shim参数。

其余12条仍不匹配；所选odom接收年龄最长86.25ms，说明20Hz快照未保留
所有独立odom事件，不能保证选中实际server使用的同一输入。固定地图时刻
及初始路径接收状态限制保留。新模式末命令为1440.599419；1440.661异常
没有发新cmd_vel_nav，因此按命令更新采样没有执行该异常帧。没有异常输出
不算修复绿，也没有改变原捕获过渡1440.649的稳定红结果。该次shell报告
退出1，不能仅凭退出码当新增碰撞；日志中无RPP_REPLAY_EXCEPTION且清理
正常，外置退出状态原因仍待核对。原工具输出保留。

下一步在关键过渡处需要所有odom/TF消息事件和controller计算时刻，而非
只有周期快照；先评估当前证据可否进一步对齐，必要时只读捕获这些边界。
没有第五控制修复，没有新GUI或完整48，全部原10矩阵缺口继续保留。

## 12:22外置插件生命周期退出缺陷修正

gdb明确捕获旧harness在RotationShim cleanup/map销毁之后、node.reset期间
SIGSEGV，回溯LifecycleNode析构；不是运行机器人故障。故12:18无异常的
sequence-command退出1不代表预测红；前文单帧shim清理异常疑点得到确认。
上游shim cleanup会释放内部primary实例，外置生命周期节点仍管理发布器
实体。诊断程序新增未configure的primary插件库持有实例，令其库保持加载
直至node析构后再释放，不激活发布器、不改任何预测输入或计算参数。
这是外置对象生命周期修复，不是第五处机器人控制修复。

修正后同一原捕获过渡1790911440.649 shim仍报告detected collision ahead，
正常清理并打印RPP_REPLAY_VERDICT 1，退出1；sequence-command同一44条
命令正常清理、打印RPP_REPLAY_VERDICT 0并退出0。这只证明退出信号已可
区分捕获异常和外置析构错误，不能把未计算异常帧的序列0当机器人修复绿。
捕获输入未改，原native primary红保持。插件卸载/节点实体寿命的具体内部
关系不作超出实测的断言。

上游参考（不代替安装二进制实测）：
https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp
cleanup释放primary控制器；本次gdb实际析构回溯与修后两个退出结果保留。
下一步继续原始/平滑速度导致轨迹进入轮廓受限位置的链路及完整事件捕获，
不因harness修好就跳过原10失败根因或启动完整48。

## 12:27真实减速链路原生平滑器15/15匹配

外置smoother_replay新增捕获输入模式，调用安装库实际inputCommandCallback
和smootherTimer，通过派生类读取last_cmd_。不激活发布器，不发Gazebo速度；
OPEN_LOOP/20Hz/max_accel[5,0,5]/max_decel[-4.5,0,-4]、速度上限2.30/3.2
及下限-0.80/-3.2与源配置一致。初始状态仅seed首个真实平滑输出，其后连续
按真实平滑输出接收时间选之前最新cmd_vel_nav，未每步重置，也未构造速度。

命令ROS_DOMAIN_ID=77 native_replay/build/smoother_replay
controller_inputs_trace.jsonl，1790911440.349883首帧seed到1441.099294：
SMOOTHER_CAPTURED_MATCH 15/15，逐分量1e-8内匹配，正常退出0。
关键减速输出1440.549 v1.425/w.166667、1440.599 v1.2/w.3、
1440.649 v.975/w.55、1440.703 v.75/w.35、1440.749 v.525/w.15、
1440.799 v.3/w0、1440.849 v.075/w0、1440.899 v0/w0全部一致。
这验证残余前进符合当前平滑参数和实际输入，不是平滑器没有执行；
绿结果仅表示捕获输出重现成功，不是原碰撞/停顿修复绿。

RPP要求原地转向时，平滑器按既有减速界保留平移，真实controller与实际
base速度方向并不立即相同；捕获异常前后位置从(-6.257,-6.597)继续到
(-6.164,-6.608)约0.094m再停。其对提前进入轮廓受限位置的因果仍须
同刻地图/事件及闭环证明，不能据此盲开scale_velocities、瞬时停车或修改
加减速度；前两个counterfactual仍红。此实验无空间/物理传播，因此不能
声称验证运动安全。全部原10根因及完整48启动条件仍未满足。

## 12:35只读消息边界单例准备与烟测

外置boundary_events.py逐消息记录odom完整四元数/速度/header_stamp、
底盘map/odom TF与static TF、四级cmd速度、三个path完整pose/stamp/frame、
每次local raw完整地图/header/update时间、mission/navigation状态。
使用同一进程wall/monotonic接收时刻及clock回退generation，不将接收时刻
当controller内部计算时间。过滤无关动态手臂TF，不过滤底盘/导航事件；
BEST_EFFORT odom/动态TF/命令depth1000仍不能保证DDS无丢失，后续按stamp
间隔核查。完整local地图按原发布频率记录，不再用1Hz抽样固定图冒充全程。
无publisher/service/控制修改；gzip1流式压缩，仅保存新的捕获输出。

4秒实际spin完成BOUNDARY_EVENT_SMOKE_PASS：odom85、底盘TF85、localraw11、
staticTF2；所有local raw长度等于size_x*size_y，gzip正常闭合并可完整读取。
烟测机器人静止没有新cmd/path，不能把这项当完整控制链物理验证。
检查无活跃campaign，df约2.1GB可用，未删任何备份/用户数据。
准备通过launch_boundary_events.sh只启动4bC_1rB GUI单例，启动前冻结
全部源码到boundary_events_source.tar.gz；driver boundary_events_driver.log、
只读消息记录boundary_events_trace.jsonl.gz。准确启动policy/PID/SHA以driver
实际输出及跟进记录为准，不预填不存在的运行结果。
启动后冻结所有文件、不开第二批、不重启；仍非完整48和新增控制修复验证。

## 12:40消息级单例完成：通过但未复现长停顿

准确启动为12:32:52 policy_20261002T123252（上方准备标题12:35不是实际
启动时间），parent245095/runner245166/probe245165均已退出。12:39:24结束
1/1通过，4bC_1rB 5/5完成241.9秒、最小净空0.2629；原始results.jsonl
lethal_start/pick_failed/nav_retry/nav_timeout/carry_invalid/collision全0。
源码SHA256ff246a93644112107d1dbf0d9a4a368ccdadac1641253b723d807e003903c278。
没有控制改动，不把本次通过等同上轮三个BLUE4两次重试的根因消除。

boundary_events_trace.jsonl.gz完整读至EOF并有capture_counts尾记录，7.1MB。
逐消息计数：odom10088、底盘TF10088、local raw1072、staticTF4、plan15、
received_global_plan1821、cmdnav2335、smooth3736、collision_safe3657、
safe4145、navstate247、mission45；generation0/1一次clock回退。
同generation odom stamp最大间隔0.036秒，无>0.1秒间隔；不因此声明DDS
所有topic零丢失或server输入时间已经完全确定。

仍有RPP预测异常。首两次1790915727.561905/5727.626556位于BLUE4
stage3（目标9.8,0.15，起12.6,-0.1）的东侧路径，机器人约(11.991,-.323)，
而不是上次(-6.257,-6.597)重试位置。对应cmdnav0/0，前消息1.65/-1.661；
实际odom1.65/-1.279，local图stamp365.252、odom365.272。
随后5727.694749恢复cmdnav1.116543/-1.724863，stage3在5730.789562
handoff，未nav_retry。BLUE4第五段5745.961->5755.546本次未长停。
其余5783/5936短警告位置尚待对齐，不将瞬时安全拒绝等同原故障。

下一步用新消息级数据检验同刻TF/local map/controller命令的可重放性，
并与此前持续受限位置区分；长停顿偶发缺口仍保留，不自动再开批或直接
完整48。本次仅完成补证据单例，没有第五控制修复。

## 12:49消息级瞬时拒绝/恢复原生回放对照（不是修复绿）

外置export_boundary_fixture.py按目标墙钟之前最后收到的同generation
odom/local raw/map计划导出原样输入，并校验捕获的map<-odom恒等TF。
接收边界不是server实际compute时刻，保留该限制。
transient_boundary_capture.jsonl SHA256
0099cdaeaeb0113f28b32bd95d62463535299c0c79c6025b5e9380ae24d68a3c，
1790915727.561，odom stamp365.272、local stamp365.252，真实pose
(11.991110,-.322740,yaw3.136120)、v1.65/w-1.279247。
ROS_DOMAIN77、不activate的实际15Hz原生RPP报detected collision ahead，
VERDICT1正常清理exit1，与运行5727.561905瞬时警告相符。

transient_recovered_capture.jsonl SHA256
4e1d4e1f3fc9a37bdae4b014f62b8a448d24a763ecd5aef0bd8b14e0e81a913c，
1790915727.694749，odom stamp365.416、pose(11.791867,-.310245,
yaw2.985554)、v.975/w-.679247。两夹具完整计划和local图逐字段相同。
该真实后继状态回放输出v1.100864/w-1.700641、VERDICT0/exit0；实际
cmdnav为1.116543/-1.724863，接收配对/连续状态仍不完全相等。

约134ms后同地图/同计划上的不同真实位姿可恢复，不是通过修改地图、
速度或控制参数造绿，也不是候选修复验证。说明单次预测异常不能充当
原持续长停/两次重试的独立判据。此前(-6.257,-6.597)真实红夹具仍保留，
持续受限因果与原BLUE5/BLUE2证据缺口未解决。无第五控制修改、无新批、
未启动完整48；下一步比较受限状态持续时间/同刻轮廓与实际平滑位移链路。

## 12:53长停后的真实零速帧仍持续受限

用controller_inputs_trace.jsonl各自ROI完整local图、同generation最新map
计划和真实odom，原生15Hz RPP分别回放准确目标墙钟：
1790911441.0498843、1443.0494306、1445.049306、1447.050265。
四次均detected collision ahead、VERDICT1正常清理exit1，无参数改变。
四帧真实v/w均0/0，pose约(-6.16356,-6.60763,yaw-.096178)，local
stamp分别657.352/659.452/661.552/663.352，非全段固定1440.649图。
先前笼统目标1441的结果不能替代这些准确ROI目标；本项具体调用为
build/rpp_replay controller_inputs_trace.jsonl originbot_nav2.yaml <准确墙钟>。

这支持停住后至少六秒仍处于RPP受限状态，与新捕获同图134ms恢复不同；
不支持把原长停仅解释为平滑减速未完成或普通瞬时警告。仍是捕获输入的
重初始化primary回放，不完全替代server连续shim状态和compute输入。
约.094m真实减速位移与进入受限位置的因果尚未证明，不能据此盲改平滑
加减速/缩轮廓/禁用碰撞。原10矩阵缺口保留，无第五运行修复/新批/完整48。

## 12:58原生轮廓检查：当前位姿未致命不等于预测运动安全

仅外置rpp_replay新增footprint-audit，调用安装Nav2的
FootprintCollisionChecker::footprintCostAtPose和实际Costmap2DROS生成的
16点轮廓，半径.28/padding.01不变；没有修改运行插件或参数。
构建通过，ROS_DOMAIN77 configure-only，原红夹具1440.64947的当前
轮廓成本253；0.75m范围内29个捕获路径位姿成本0..253，没有>=254。
最近路径点index38(-6.275,-6.38003,yaw.187126)，距实际位置.217464m，
成本253。仍按原样compute得到collision异常/VERDICT1/正常exit1。

准确停车ROI1441.0498843各自local图的当前轮廓成本仍253；附近29个
路径位姿0..253，最近index39距离.244900m、成本253，RPP仍拒绝预测运动。
本审计仅离散路径位姿/当前轮廓，不等于路径整段扫掠或可转向证明。
因此当前轮廓未达254和中心非致命不能否定实际预测弧碰撞；亦不能据此
认定Smac2D路径本身非法。下一步定位预测弧触及的原始格及跟踪/平滑
入场链路；不缩半径/清图/禁安全造绿。没有第五运行修复、新GUI或完整48。

## 13:09预测旋转原始致命格定位（外置诊断）

外置arc-audit使用安装的FootprintCollisionChecker、LineIterator及相同
Costmap2DROS轮廓。对原1440.649夹具仅扫描固定位置正向yaw，每步.001rad，
不是改controller输入/地图，也不是真实compute投影tick回放。
首次采样成本254在yaw-.03516843355（比捕获yaw多.094rad），轮廓边12/13
均触及同一原始格(50,33)，raw254，中心(-6.124999962,-6.874999975)。
两条边记录同一格，不算两次独立碰撞。原样compute仍collision/VERDICT1。

该.05m格方形范围x[-6.149999963,-6.099999962]，
y[-6.899999975,-6.849999975]。实际机器人中心到格方形最近距离.275057m，
小于未加padding的半径.28；此前格中心距.308120m不代表方形格不重叠。
因此不将这次旋转拒绝解释成仅16边轮廓误报，也不缩半径/清图解除保护。
栅格格内254不等于Gazebo真实障碍完整占据格方形；此结论仅证明原生
检查有捕获地图依据，不完成入场跟踪/平滑因果或全部原10失败根因修复。
无第五运行修改/新批/完整48，下一步对齐该格与入场路径和实际运动链。

## 13:13长停入场链：转向请求与平滑输出符号滞后

只读controller_inputs_trace原sample、最新同generation map计划1437.655498。
以下路径距离为最近离散顶点（不是最近线段），格方形距离以1440.649
捕获的致命格世界范围回看，不能据此断言早期地图格已经致命。
1440.199392 pose(-6.96851,-6.52529,yaw.05558)，真实v1.5/w-1.23333，
cmdnav已1.65/+.587799，路径顶点距.05522m。
1440.349883 pose(-6.73833,-6.52754,yaw-.09318)，真实1.65/-.63333，
cmdnav1.65/+1.101615，路径距.06301m。
1440.449284 pose(-6.56137,-6.54823,yaw-.15196)，真实1.65/-.23333，
cmdnav1.65/+1.364308，路径距.12109m。
1440.549248真实v1.425/w+.166667，cmdnav0/+.300，路径距.17348m，
距未来捕获格方形.36293m。
1440.649470真实v.975/w+.550，cmdnav0/+.633333，路径距.21746m、
距捕获致命格方形.27506m；1440.661碰撞拒绝。
1440.899713停止，路径距.24490m、格方形距.24274m。

角速度请求已为正而实际平滑仍为负期间，实际机器人向路径南侧偏移；
这与此前真实smoother15/15重现一致，不是平滑节点未执行。该链支持
旋转后开始平移时的转向响应滞后参与跟踪偏差，不是仅最后.094m余量。
仍需可验证候选闭环区分转向切换/线角独立约束/速度与路径曲率，不能
盲改scale_velocities或加减速。接收快照与servercompute时刻差异保留，
没有第五控制修改/新GUI/完整48，全部原10矩阵缺口仍保留。
