# 车间多工位配送仿真工作空间

本工程使用 **ROS 2 Humble + Gazebo Classic 11**。当前主线是单机器人车间配送：
六台设备分为两排，每排三台，配套六个工位和一个物料站。
已完成场景、实际 SLAM 建图、AMCL 定位、Nav2 导航和固定顺序配送闭环。
软件包继续使用现有 `gasrobot_` 前缀；当前说明以车间配送仿真为准。

## 本次完成了什么

| 部分 | 实现与文件 | 验证情况 |
|---|---|---|
| 车间场景 | [workshop_delivery_v1.world](src/gasrobot_simulation/worlds/workshop_delivery_v1.world)，含六设备、六工位标记及物料架 | 场景加载、通行验证完成 |
| 环境地图 | [workshop_delivery_v1.yaml](src/gasrobot_gas_mapping/maps/workshop_delivery_v1.yaml) 及同名 PGM | 实际 SLAM 建图，地图重载通过 |
| 统一启动 | [delivery_sim.launch.py](src/gasrobot_bringup/launch/delivery_sim.launch.py) | Gazebo、AMCL、Nav2 和配送节点正常运行 |
| 工位标定 | [stations.yaml](src/gasrobot_delivery/config/stations.yaml) | map 坐标标定，地图哈希/净空校验，六工位往返及跨排通过 |
| 配送执行 | [gasrobot_delivery](src/gasrobot_delivery/README.md) | 开始、暂停、继续、取消、超时重试、卸货等待和回程闭环 |
| 运动修正 | 车轮惯量轴、接触摩擦、仿真专用 RPP 和障碍膨胀参数 | 修复转向停滞及设备边缘通行失败后重新验收 |
| 工程整理 | 配置校验、中文注释、配送文档及运行日志 | 运行记录放在 `runs/`，验收摘要保存在配送包 `docs/` |

无窗口 Gazebo 实测结果：短路线成功；完整六工位路线连续 **3 批成功**，
耗时分别为 **419.3、420.2、430.7 ROS 秒**，零重试、无人工接管，未检测到碰撞。
18 次卸货等待均为 5 秒；25 项配置/状态机测试通过。

[完整验收记录与限制](src/gasrobot_delivery/docs/acceptance.md) ·
[机器可读结果](src/gasrobot_delivery/docs/acceptance_results.json) ·
[三批轨迹图](src/gasrobot_delivery/docs/delivery_trajectories.png)

## 启动车间配送

在本工作空间根目录执行：

```bash
cd /home/iceice/ROS_Project/study/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to gasrobot_bringup --symlink-install
source install/setup.bash
ros2 launch gasrobot_bringup delivery_sim.launch.py
```

系统加载现有车间地图，机器人出生在物料站，默认自动初始化 AMCL。
启动后保持待命，**不会自动配送**。无窗口运行时追加 `gui:=false rviz:=false`。
修改 Python、Launch、配置或模型后应重新编译；模型及启动参数修改后重启系统。

另开终端，加载环境后确认装货并开始：

```bash
cd /home/iceice/ROS_Project/study/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /delivery_manager/start std_srvs/srv/Trigger '{}'
```

默认完整路线：`物料站 → S1 → S2 → S3 → S6 → S5 → S4 → 物料站`。
每站到达并停车后卸货 5 秒，返回物料站成功才将整批标记为完成。
任务执行期间由配送节点独占导航目标，不同时用键盘或 RViz 发送运动命令。

```bash
# 查看状态；该话题内容为 JSON 字符串。
ros2 topic echo /delivery_manager/status
# 暂停、继续、取消。
ros2 service call /delivery_manager/pause std_srvs/srv/Trigger '{}'
ros2 service call /delivery_manager/resume std_srvs/srv/Trigger '{}'
ros2 service call /delivery_manager/cancel std_srvs/srv/Trigger '{}'
```

暂停导航先取消旧目标，状态变为 PAUSED 后才能继续；暂停卸货保留剩余服务时间。
取消后等待 CANCELLED 确认停车。导航单次超时 180 ROS 秒，失败最多重试 1 次，
仍失败则终止任务。Gazebo 暂停时任务计时停止；重启节点不会自动续跑。
短路线、批次切换、参数和日志格式见 [配送包操作说明](src/gasrobot_delivery/README.md)。

## 三个启动入口的区别

| 入口 | 用途 | 是否启动定位/配送 |
|---|---|---|
| `gasrobot_bringup delivery_sim.launch.py` | 完整车间配送演示 | AMCL + Nav2 + 配送节点，等待显式开始 |
| `gasrobot_simulation workshop_delivery.launch.py` | 只查看车间、手动驾驶或配合独立建图入口 | 只启动仿真、机器人和控制器 |
| `gasrobot_simulation gazebo_sim.launch.py` | 通用仿真，默认旧 `L_model.world`，可指定 world | 只启动仿真、机器人和控制器 |

定位模式不同时启动 SLAM。重新建图的方法及已有地图来源见
[建图说明](src/gasrobot_gas_mapping/README.md) 和
[车间建图记录](src/gasrobot_gas_mapping/docs/workshop_delivery_v1_mapping.md)。

## 配置、坐标系与运动链路

- `stations.yaml` 保存 **map** 坐标和固定朝向，不直接使用 Gazebo 的 world 坐标。
- `batch.yaml` 保存工位顺序、服务时间、导航超时和重试次数；回程由执行器追加。
- `nav2_workshop.yaml` 保存独立仿真参数，采用 Navfn + RPP。
- 更换地图或出生点时，关闭自动初始定位并重新核对工位标定；不能只修改地图哈希绕过校验。

TF 为 `map → odom → base_footprint`：AMCL 发布前一段，差速控制器发布后一段。
`odom` 原点不需要跟着机器人移动；机器人在 odom 中的位姿会随行驶改变。
判断定位是否异常应结合激光/地图、TF 连续性及位姿误差。

速度链路为 `Nav2 控制器/恢复行为 → /cmd_vel_nav → velocity_smoother → /cmd_vel → 差速控制器`。
全部参与仿真的节点使用仿真时间。Gazebo 真值仅用于验收，不参与配送控制。

## 代码阅读顺序

1. [场景与基础启动](src/gasrobot_simulation/README.md)：了解 world、出生位姿、模型生成和控制器加载顺序。
2. [车轮模型](src/gasrobot_description/urdf/gasrobot/wheel/wheel.urdf.xacro)：了解轮轴、惯量及接触摩擦方向。
3. [统一启动](src/gasrobot_bringup/launch/delivery_sim.launch.py)：了解仿真、定位、导航和任务层的组合。
4. [配置校验](src/gasrobot_delivery/gasrobot_delivery/config.py)：了解地图绑定、净空及任务合法性检查。
5. [状态机](src/gasrobot_delivery/gasrobot_delivery/engine.py)：了解到站/卸货完成的区别和取消屏障。
6. [ROS 适配层](src/gasrobot_delivery/gasrobot_delivery/manager.py)：了解 Action 异步回调、服务、停车确认和工位可视化。
7. [运行记录](src/gasrobot_delivery/gasrobot_delivery/logger.py)：了解配置快照、事件、轨迹和终态汇总。
8. [验收脚本](src/gasrobot_delivery/scripts/validate_sim.py)：了解真实仿真导航、异常操作及批次验收。

## 结果与测试

每批独立保存在 `runs/`：配置快照、`events.csv`、`trajectory.csv`、`summary.json`。
该目录已加入 `.gitignore`；关键验收结果与轨迹图保存在配送包 `docs/` 中。

```bash
colcon test --packages-select gasrobot_delivery
colcon test-result --test-result-base build/gasrobot_delivery --verbose
```

完整仿真验收命令见 [配送包 README](src/gasrobot_delivery/README.md#验证)。
0.10 m / 0.10 rad 是停车后的 **map 估计位姿**容差，不代表真实装卸精度；
实际定位差异、GUI 复测范围和协议测试边界见验收记录。

2026-09-22 再次完成一批实际配送，并记录七段行程各自的**途中截图**。
整批轨迹由 8,479 条 Gazebo 真值采样分成七段后依次合并，合并 CSV 与原始 CSV 的
SHA256 完全一致；RViz 也实时显示累计真值轨迹。耗时 423.3 ROS 秒，零重试。
[最新截图、逐段图与来源校验](artifacts/workshop_delivery_gui_20260922/README.md) ·
[最新完整素材 ZIP](artifacts/workshop_delivery_gui_20260922.zip) ·
[完成时的实际累计轨迹截图](artifacts/workshop_delivery_gui_20260922/14_completed.png)

2026-09-21 补做一批 GUI 完整配送：431.5 ROS 秒、32.71 m、零重试；
保存 15 张同时包含 RViz 和 Gazebo 的 1920×1080 原始截图。
[截图索引与图注](artifacts/workshop_delivery_gui_20260921/README.md) ·
[完整素材 ZIP](artifacts/workshop_delivery_gui_20260921.zip) ·
[自动截图脚本](src/gasrobot_delivery/scripts/capture_delivery.py)

## 后续阶段

- [导航说明](src/gasrobot_navigation/README.md) · [建图说明](src/gasrobot_gas_mapping/README.md) · [启动包](src/gasrobot_bringup/README.md)。
- 下一阶段依次建立 7×7 点间代价矩阵、距离/累计迟交模型、基线算法及 NSGA-II 对比实验。
- 多机器人、动态插单、机械臂装卸未在本轮实现。
