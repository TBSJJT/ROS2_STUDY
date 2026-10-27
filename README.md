# 车间多工位配送机器人

基于 ROS 2 Humble、Gazebo Classic 的单机器人车间配送工程，保留 STM32 底盘、
IMU 和真实激光雷达接入能力。场景包含一个物料站、两排六台设备与六个工位；
现有功能为建图、定位导航、固定顺序配送、任务控制、真实轨迹记录及静态规划距离矩阵。

工程统一使用 `delivery_*` ROS 包名，气体检测接口和旧巡检实现已移除。
现已进入点间代价阶段：实现 Nav2 静态有向距离矩阵、独立时间采样工具、可编辑截止时间
与精确枚举目标模型。NSGA-II 尚未实现；时间缓存是否可用取决于实际标定完整性。
资料、历史实验记录和地图内容保持原样；
历史资料内的旧命令仅代表当时版本，当前命令以本 README 和各包 README 为准。

## 当前包结构

| 包 | 职责 |
|---|---|
| [delivery_bringup](src/delivery_bringup/README.md) | 仿真配送与实车组合启动 |
| [delivery_description](src/delivery_description/README.md) | 共用 URDF/Xacro、传感器和车轮模型 |
| [delivery_simulation](src/delivery_simulation/README.md) | Gazebo 场景、仿真控制器、机器人生成 |
| [delivery_mapping](src/delivery_mapping/README.md) | SLAM 参数和已保存地图 |
| [delivery_navigation](src/delivery_navigation/README.md) | 通用 Nav2 启动与实车导航参数 |
| [delivery_tasks](src/delivery_tasks/README.md) | 工位、批次、配送状态机、日志、截图和验收工具 |
| [delivery_planner](src/delivery_planner/README.md) | 有向距离矩阵、独立时间标定、可编辑迟交模型与精确枚举 |
| [delivery_base](src/delivery_base/README.md) | STM32 串口协议、底盘速度、里程计和 IMU |
| `src/vendor/lslidar_driver`、`lslidar_msgs` | 原有真实雷达驱动和消息，保持上游包名 |

## 编译与环境

迁移后请使用新终端，避免沿用旧安装目录的环境变量。本轮旧 `build/` 和 `install/`
已移至 `runs/restructure_20261011/previous_build`、`previous_install`，当前安装为重新构建。
不要 source 这些旧缓存中的 setup 文件。

```bash
cd /home/iceice/ROS_Project/study/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

同一次运行的终端使用一致的 `ROS_DOMAIN_ID` 与 `ROS_LOCALHOST_ONLY`。
旧包名不再提供兼容别名，所有启动命令使用下面的新名称。

## 仿真配送

```bash
ros2 launch delivery_bringup delivery_sim.launch.py
# 无窗口运行：在上述命令后追加 gui:=false rviz:=false
```

该入口加载车间地图和物料站出生位姿，启动 AMCL/Nav2，保持待命而不自动配送。
另开终端并加载 ROS 与工作空间环境后：

```bash
ros2 lifecycle get /bt_navigator
# 确认 active，且机器人在物料站停车后，开始任务表示装货已确认。
ros2 service call /delivery_manager/start std_srvs/srv/Trigger '{}'
ros2 topic echo /delivery_manager/status

# 按需执行，而非顺序全部调用：
ros2 service call /delivery_manager/pause std_srvs/srv/Trigger '{}'
ros2 service call /delivery_manager/resume std_srvs/srv/Trigger '{}'
ros2 service call /delivery_manager/cancel std_srvs/srv/Trigger '{}'
```

固定顺序为 `depot → S1 → S2 → S3 → S6 → S5 → S4 → depot`，每站卸货 5 秒。
导航超时默认 180 ROS 秒，最多重试一次；返回物料站成功才完成整批。
地图、工位、批次及仿真专用导航数值参数沿用此前已验证配置。

只加载场景与机器人：

```bash
ros2 launch delivery_simulation workshop_delivery.launch.py
```

建图时，在基础场景启动后另开终端运行：

```bash
ros2 launch delivery_mapping mapping.launch.py use_sim_time:=true
```

建图入口不与定位配送入口同时运行，避免两者争用 `map → odom`。

## 静态规划距离矩阵

```bash
# 新增包需要先构建，再 source install/setup.bash。
colcon build --symlink-install --packages-up-to delivery_planner
source install/setup.bash
ros2 launch delivery_planner cost_matrix.launch.py
```

无需启动 Gazebo，工具使用隔离的 Nav2 地图服务器和规划器，计算物料站及六工位的
42 个有向点对。输出在 `runs/cost_matrix_*/`，包含原始规划路径、米制距离表、
实际代价栅格、输入快照和哈希。采集结束自动关闭专用节点。
这些是规划路径，不能当作实际行驶轨迹；时间矩阵暂为空。
原配送导航仍使用实时激光障碍层，本静态距离模型只使用保存地图和膨胀层。
详细缓存定义、使用边界和下一阶段说明见 [规划包 README](src/delivery_planner/README.md)。

## 时间标定与离线目标模型

```bash
# 使用已有距离归档；默认五次短路线往返，自动启动并关闭独立仿真。
ros2 launch delivery_planner calibration_sim.launch.py \
  distance_file:=/绝对路径/距离归档/matrix.json pilot:=true
# 完整42个有向点对各五次：去掉 pilot:=true。不要与已有仿真同时启动。
```

截止时间和卸货时间在 `src/delivery_planner/config/instance_manual.yaml` 中逐工位编辑。
其中数值只是演示；该配置目前用于离线评价，不修改配送执行器现有批次。
完整时间缓存生成后，可用 `evaluate_routes.py` 枚举六工位全部720种顺序，
计算包含回程的距离、完成卸货后的累计迟交和精确Pareto集合。
采样失败和不足次数的结果会保留，但评价器拒绝将其当作完整时间矩阵。
操作与口径见 [规划包说明](src/delivery_planner/README.md)。

## 保留的实车入口

根据实际串口、雷达网络和地图设置参数；本轮整理不代表进行了真实硬件验证。

```bash
# 仅启动底盘和雷达，首次检查底盘可按需加 enable_lidar:=false。
ros2 launch delivery_bringup hardware.launch.py serial_port:=/dev/ttyUSB0

# 三选一：实车模型+硬件，或在此基础上建图、定位导航。
ros2 launch delivery_bringup delivery_robot.launch.py mode:=hardware
ros2 launch delivery_bringup delivery_robot.launch.py mode:=slam
ros2 launch delivery_bringup delivery_robot.launch.py mode:=nav map:=/绝对路径/实车地图.yaml
```

`hardware.launch.py` 与 `delivery_robot.launch.py` 不同时启动，后者已包含硬件。
实车组合入口默认 `use_sim_time:=false`，不加载 Gazebo 或旧巡检任务。
实车导航使用 `delivery_navigation/config/nav2_params.yaml`；仿真配送使用
`delivery_tasks/config/nav2_workshop.yaml`，两套参数分别维护。
实车工位尚需按实际地图重新标定，不能直接照搬仿真工位执行配送。

## 轨迹与验证工具

工具既可从 `src/delivery_tasks/scripts/` 运行，也已安装为 `ros2 run` 入口。

```bash
# 在开始新批次前记录；需同屏摆放 RViz/Gazebo，输出目录必须为空。
ros2 run delivery_tasks record_delivery_evidence.py --output runs/新的截图目录

# 在独占仿真任务的终端中执行一批完整配送。
ros2 run delivery_tasks validate_sim.py --mode batch --repeat 1

# 等待记录结束后，按七段实际采样融合轨迹。
ros2 run delivery_tasks plot_delivery_evidence.py runs/新的截图目录
```

Gazebo 实体名称已统一为 `delivery_robot`，验收和截图订阅代码同步迁移。
控制接口 `/cmd_vel`、`/odom`、`/scan`、`/delivery_manager/*` 和 TF 名称保持不变。
真值仅用于记录和评估，不参与导航控制。详细说明见配送包 README。

## 关闭与重复启动

在启动 launch 的终端按 Ctrl+C 并等待子进程退出，再重新启动。
仅关闭 Gazebo/RViz 窗口可能留下后台 `gzserver`。
若提示 `Address already in use`，先检查旧仿真；不要重复启动第二套系统。
`Nav2 lifecycle is not active` 时先检查 Gazebo、控制器、`odom` TF 和生命周期，
不要通过删除任务就绪检查绕过问题。

## 改名对应表

| 原包 | 当前包 |
|---|---|
| `gasrobot_base` | `delivery_base` |
| `gasrobot_bringup` | `delivery_bringup` |
| `gasrobot_description` | `delivery_description` |
| `gasrobot_simulation` | `delivery_simulation` |
| `gasrobot_gas_mapping` | `delivery_mapping` |
| `gasrobot_navigation` | `delivery_navigation` |
| `gasrobot_delivery` | `delivery_tasks` |

`gasrobot_gas`、`gasrobot_interfaces` 已删除；后者仅包含气体和旧巡检自定义接口。
当前配送使用 Nav2 和标准 ROS 消息/服务。历史实验资料未重新编写或冒充本轮测试。

## 本轮工程迁移验证（2026-10-11）

- 9 个包全新编译通过；最终 7 个业务包再次构建通过。
- 48 项底盘/配送功能单元测试通过；配送包 colcon 测试为 26 项、零失败。
- 配送异步协议测试通过：重复开始、暂停继续、取消、卸货暂停、失败重试。
- 新命令实际完成 `depot → S1 → depot`，两次导航成功且停车；未检测到障碍重叠。
- 96 个资料、地图、第三方驱动文件哈希未变；六份关键配置解析后的数值未变。
- 实车只验证构建、模型展开和启动参数，未连接设备实测；本轮未重跑完整三批配送。
- 详细日志与文件迁移清单位于 `runs/restructure_20261011/`。

当前用户环境已补装 `pyserial 3.5`，包清单仍声明系统依赖 `python3-serial`。
雷达驱动构建提示 pcap 文件 I/O 未启用；未为本轮改名修改其源码或第三方依赖配置。

## 点间距离阶段验证（2026-10-11）

新增 `delivery_planner`，当前共有 10 个 ROS 包（含两个原有雷达包）。
新包及依赖构建通过，矩阵与配送功能测试共 36 项通过。
两次独立启动 Nav2，均完成 42 个有向点对，原始规划路径和距离矩阵完全一致。
结果保存在 `runs/cost_matrix_validation_20261011/`，未进行车辆行驶或时间标定。
后续独立时间标定、可编辑截止时间及迟交模型的实现与实测进展见规划包 README。

## 冻结实验输入协议（2026-10-27）

优化读取冻结的规划距离、独立标定时间和任务实例；实际导航数据只用于验证。
每对距离重复规划三次，正式时间标定默认五次，报告P95、成功率与样本数。
冻结工具拒绝失败或不完整矩阵；当前完整时间采集仍保留S6→S1超时，尚不能发布。
详见 [T4修订、统计边界与运行命令](src/delivery_planner/EXPERIMENT_PROTOCOL.md)。
