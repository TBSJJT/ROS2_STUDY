# GasRobot ROS 2 工作空间

本工程是基于 ROS 2 Humble 的气体泄漏巡检移动机器人工作空间。当前阶段已经完成
麦克纳姆底盘通信、机器人模型、激光雷达、SLAM、AMCL、Nav2 和中间直走廊的
之字形正常巡检框架。

PicoPC 上的工作空间固定为：

```text
/userdata/iceice/gasrobot_ws
```

## 软件包结构

```text
gasrobot_ws/
└── src/
    ├── gasrobot_interfaces       # 自定义消息、服务和 Action
    ├── gasrobot_description      # URDF/Xacro、传感器和底盘坐标系
    ├── gasrobot_base             # STM32 串口、IMU、里程计和底盘 TF
    ├── gasrobot_gas              # 气体采集与预处理，当前仍是功能骨架
    ├── gasrobot_gas_mapping      # SLAM、地图和后续气体分布图
    ├── gasrobot_navigation       # Nav2 参数和巡检任务管理
    ├── gasrobot_bringup          # 整机统一启动
    └── vendor                    # 第三方雷达驱动
```

## 当前正常巡检逻辑

当前默认路线只测试 gasrobot_map.yaml 中间的直走廊：

- 路线范围约为 x=-4.0～6.0 米；
- y 在 -0.45 米和 0.45 米之间交替；
- 向东 6 个点、向西 6 个点，共 12 个之字形航点；
- 默认执行一圈往返；
- 线速度上限为 0.10 m/s；
- 所有航点的 dwell_sec 均为 0，到点后立即发送下一目标；
- Nav2 禁止规划进入未知区域；
- 启动前要求每个航点周围至少 0.30 米为已知自由栅格。

航点只负责控制 Nav2 的覆盖路线，不负责触发气体采样。gasrobot_gas 后续必须在
机器人运动、转弯和短暂停车期间持续发布带时间戳的 /gas/readings。实际采样位置
应通过数据时间戳查询历史 TF，不能使用当前目标航点坐标代替。

## 编译

在 PicoPC 终端执行：

```bash
cd /userdata/iceice/gasrobot_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to gasrobot_bringup

source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash
```

每个新终端都必须执行：

```bash
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash
```

## 首次启动正常巡检

### 1. 启动整机、Nav2、RViz 和巡检管理器

```bash
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash

ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav \
  map:=/userdata/iceice/gasrobot_ws/src/gasrobot_gas_mapping/maps/gasrobot_map.yaml \
  enable_inspection:=true \
  inspection_route_file:=/userdata/iceice/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml \
  default_route:=standard_route \
  auto_set_initial_pose:=false \
  auto_start_inspection:=false \
  enable_rviz:=true \
  use_sim_time:=false
```

首次调试必须保持 auto_set_initial_pose 和 auto_start_inspection 为 false，防止错误
初始位姿覆盖人工定位，或者节点启动后机器人立即运动。

### 2. 在 RViz 中确认定位

1. RViz 的 Fixed Frame 应为 map；
2. 使用 2D Pose Estimate 设置机器人真实位置和朝向；
3. 等待 LaserScan 与地图墙体重合；
4. 确认局部、全局代价地图没有把机器人包在障碍物内；
5. 确认现场没有建图后新增的障碍物。

新开终端检查 TF、Nav2 和巡检节点：

```bash
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash

ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 node list | grep inspection_manager
```

TF 检查完成后按 Ctrl+C 停止 tf2_echo。

### 3. 启动一圈之字形巡检

```bash
ros2 service call /inspection_manager/start_default \
  std_srvs/srv/Trigger '{}'
```

任务管理器一次只向 Nav2 发送一个 NavigateToPose。目标成功后立即发送下一点，
不会等到航点后才让传感器采样。

## 状态和任务控制

查看状态：

```bash
ros2 topic echo /inspection_manager/state
ros2 topic echo /inspection_manager/current_waypoint
ros2 topic echo /inspection_manager/active
```

常见状态：

- IDLE：等待任务；
- WAITING_NAV2：等待 Nav2 Action；
- NAVIGATING：正在前往当前巡检点；
- DWELLING：执行可选的静止观察，当前默认路线不会进入该状态；
- PAUSED：任务暂停；
- COMPLETED：本轮路线完成；
- FAILED：任务失败；
- CANCELLED：任务已取消；
- SAFETY_STOP：严重风险事件触发停机。

暂停：

```bash
ros2 service call /inspection_manager/pause \
  std_srvs/srv/SetBool '{data: true}'
```

继续：

```bash
ros2 service call /inspection_manager/pause \
  std_srvs/srv/SetBool '{data: false}'
```

取消或紧急停止当前巡检：

```bash
ros2 service call /inspection_manager/cancel \
  std_srvs/srv/Trigger '{}'
```

## 修改和重新加载路线

路线文件：

```text
/userdata/iceice/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml
```

修改航点前必须先取消正在运行的任务：

```bash
ros2 service call /inspection_manager/cancel std_srvs/srv/Trigger '{}'

nano /userdata/iceice/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml

ros2 service call /inspection_manager/reload_routes std_srvs/srv/Trigger '{}'
```

重新加载时会再次检查：

- YAML 字段是否合法；
- 航点 ID 是否唯一；
- 坐标系是否为 map；
- 航点是否位于 gasrobot_map.yaml 的已知自由区域；
- 航点周围 0.30 米是否具有足够净空。

安全检查失败时禁止通过降低净空参数强行运行，应在 RViz 中重新选择点位。

## 在 RViz 获取坐标

```bash
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash
ros2 topic echo /clicked_point
```

在 RViz 顶部选择 Publish Point，再点击地图。输出的 frame_id 必须为 map，x 和 y
可以填写到 inspection_routes.yaml。Publish Point 不包含目标朝向，yaw_deg 需要
根据下一段路线方向单独设置。

## 连续气体采样检查

gasrobot_gas 实现后，使用以下命令确认它与巡检点无关地持续发布：

```bash
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash

ros2 topic hz /gas/readings
ros2 topic echo /gas/readings
```

正确表现是启动巡检前和机器人前往第一个航点途中都能持续收到数据。当前
gasrobot_gas 仍是功能骨架，因此巡检导航可以运行，但暂时不会产生真实气体数据。

## 测试

```bash
cd /userdata/iceice/gasrobot_ws
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash

colcon test --packages-select gasrobot_base gasrobot_navigation
colcon test-result --verbose
```

## 安全要求

1. 首次运行必须打开 RViz，并保持 0.10 m/s 低速；
2. 启动任务前确认地图、LaserScan、TF 和机器人实际位置一致；
3. 机器人运动范围内不得站人，并确保取消命令随时可用；
4. 定位跳变、路径异常或现场出现新增障碍物时立即取消任务；
5. 不得直接使用未经地图校验和实车确认的航点；
6. 正式气体实验必须符合实验室规范并获得导师批准。

## 详细文档

- gasrobot_base/README.md：STM32 串口协议、IMU 和里程计；
- gasrobot_navigation/README.md：正常巡检配置、启动和控制；
- gasrobot_navigation/docs/active_gas_inspection_architecture.md：后续气体主动搜索框架；
- gasrobot_bringup/README.md：整机启动参数。
