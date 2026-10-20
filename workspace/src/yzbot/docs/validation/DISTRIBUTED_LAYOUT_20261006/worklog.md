# 工作日志（DISTRIBUTED_LAYOUT_20261006）

- 接手核对：HEAD 0e4acf8 = origin/master；无 gzserver；用户未提交 GUI/TF 改动与 stash 原样保留。
- 备份：/home/ros/distributed-20261006-backup（13 文件 + 用户补丁 + 哈希）+ 本地 ref backup/distributed-20261006。
- 发现关键事实：**world 的墙/岩石以 <state> 里的绝对位姿为准**（Gazebo 载入时恢复），
  地图生成器也用 state；model 定义里的墙位姿比实际偏 -1.75 m。后续所有几何判断改用 state/map。
- 离线核对：候选布局最小间距 3.059 m、最小 cube→区域 4.809 m；新托盘足迹逐个检查入口净空 ≥0.80 m、与区域全连通。
- 搬移：world 只改 5 cube + 5 tray 的模型级 XY（断言 10 处、其余 30 个模型逐字节未变、cube/tray 对齐、Z/姿态保留）。
- 同步：complex_layout.json → activate_complex_layout.py → zones/mission/两个 py 默认表/地图/风险区；
  手工修两处硬编码测试坐标；地图差异 360 像素全部对应托盘，占据总数不变。
- 静态门：colcon 4 包构建通过；行为回归 699→705 passed（新增 6 个测试）。
- 物理运行（每轮都留 result.log/launch.log/bag/audit.json）：
  r1 正序全门通过 224.5 s（修复前构建）；r2 反序 5/5 但重复抓取导致审计不通过；
  r3/r5/r10 穿越等待超时腿失败；r4/r6/r7/r8 安全 5/5 但只记到 1 个响应；
- 修复：① velocity_smoother 放开有界横向通道（纠偏实测生效 0.055 m）；② 穿越计时改为“起跑后计时 + 12 s 等待上界”
  并与限速器共享 safe_crossing_phase 谓词；③ hold 触发 1.30→1.05、standoff 1.05→0.85、entry lateral 1.20→1.45；
  ④ 交接增加 0.45 m/s 速度门 + 2 s 宽限（防 GUI 负载下滑行 0.6 m）。
- r11 反序全门通过：257.8 s、5/5、零跳过、双原始响应、净空 0.5287、无碰撞；r12 正序复跑见 case-results.csv。
- 未完成：48 全矩阵、red4/blue4/blue1-east 定向覆盖、GUI 复跑、API/CoStudio/完整墙钟。
