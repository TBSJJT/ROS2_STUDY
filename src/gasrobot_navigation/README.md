# gasrobot_navigation

GasRobot 的 Nav2 配置与自主气体巡检任务软件包。机器人完成建图和实车导航调参后，
日常巡检不再依赖操作员在 RViz 中逐个点击目标点，而是由本包读取命名路线并自动执行。

## 工程边界

本包分成两层：

- Nav2 能力层：AMCL 定位、全局规划、局部避障、恢复行为和速度平滑；
- 巡检任务层：初始化位姿、命名巡检点、路线循环、失败重试、停留采样、暂停、取消、
  任务状态以及严重气体风险停机。

任务层调用标准 `NavigateToPose` Action，不重新实现路径规划和避障算法。气体采集、
响应延迟补偿、风险事件生成和后端上传仍分别属于 `gasrobot_gas`、
`gasrobot_gas_mapping` 和后端适配模块。

## 文件结构

```text
gasrobot_navigation/
├── config/
│   ├── nav2_params.yaml          # AMCL、控制器和代价地图实车参数
│   ├── inspection_routes.yaml    # 初始化位姿、命名路线与巡检策略
│   └── inspection_manager.yaml   # 任务管理节点运行参数
├── gasrobot_navigation/
│   ├── route_config.py           # YAML 数据模型、读取和严格校验
│   └── inspection_manager.py     # Nav2 Action 与巡检任务编排
├── launch/
│   ├── navigation.launch.py      # 仅启动 Nav2
│   └── inspection.launch.py      # 仅启动巡检任务层
└── test/                         # 路线校验和 Python 规范测试
```

## 第一次配置巡检点

先使用当前已经可用的 RViz 多点导航完成现场验证。对于每一个固定巡检位置，记录
`map` 坐标系中的：

- `x`：地图横坐标，单位米；
- `y`：地图纵坐标，单位米；
- `yaw_deg`：机器人目标朝向，单位度，逆时针为正；
- `dwell_sec`：到点后的静止采样时间。

将这些值填写到 `config/inspection_routes.yaml`。初始化位姿应当是机器人每次上电
实际停靠的位置，不应随意写为地图原点。完成实地核验后，把：

```yaml
site_configured: false
```

改为：

```yaml
site_configured: true
```

这个保护可以防止尚未配置的模板坐标让机器人意外运动。

如果机器人并非每次都从固定停靠位启动，请在启动时使用
`auto_set_initial_pose:=false`，并先在 RViz 中人工确认初始位姿。

## 路线字段

```yaml
standard_route:
  target_gas: methane
  alarm_threshold: 1000.0
  stop_on_critical_risk: true
  repeat_count: 1
  default_dwell_sec: 5.0
  max_retries: 2
  continue_on_failure: false
  navigation_timeout_sec: 180.0
```

- `target_gas`：本次巡检关注的气体类型，应与 `GasReading.gas_type` 一致；
- `alarm_threshold`：任务反馈使用的异常参考阈值，最终风险等级仍由气体算法确定；
- `stop_on_critical_risk`：收到严重 `RiskEvent` 时是否立即停止机器人；
- `repeat_count`：整条路线执行圈数；
- `default_dwell_sec`：未单独配置时的到点停留采样时间；
- `max_retries`：一个目标点导航失败后的最大重试次数；
- `continue_on_failure`：重试仍失败后是否跳到下一个巡检点；
- `navigation_timeout_sec`：单次前往巡检点的最长时间。

## 启动

完成地图和路线配置后，统一启动导航与巡检：

```bash
source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash

ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav \
  enable_inspection:=true \
  enable_rviz:=false
```

调试时可保留 RViz：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav \
  enable_inspection:=true \
  enable_rviz:=true
```

## 日常任务控制

启动配置文件中的默认路线：

```bash
ros2 service call /inspection_manager/start_default std_srvs/srv/Trigger '{}'
```

暂停任务：

```bash
ros2 service call /inspection_manager/pause std_srvs/srv/SetBool '{data: true}'
```

继续任务：

```bash
ros2 service call /inspection_manager/pause std_srvs/srv/SetBool '{data: false}'
```

取消任务：

```bash
ros2 service call /inspection_manager/cancel std_srvs/srv/Trigger '{}'
```

重新发送配置的 AMCL 初始位姿：

```bash
ros2 service call /inspection_manager/set_initial_pose std_srvs/srv/Trigger '{}'
```

现场修改路线 YAML 后，在空闲状态重新加载：

```bash
ros2 service call /inspection_manager/reload_routes std_srvs/srv/Trigger '{}'
```

查看任务状态和当前巡检点：

```bash
ros2 topic echo /inspection_manager/state
ros2 topic echo /inspection_manager/current_waypoint
ros2 topic echo /inspection_manager/active
```

## 后端 Action 接口

`gasrobot_interfaces/action/ExecuteInspection` 支持两种方式：

1. 填写 `route_name`，执行机器人本地经过现场审核的固定路线；
2. `route_name` 留空并填写 `waypoints`，由后端临时下发实验路线。

Action 反馈包含当前进度、目标气体浓度、风险等级和风险事件数量。任务结果包含完成
巡检点数量与风险事件总数，适合作为后端任务记录和实验统计入口。

## 与气体课题的结合

机器人导航期间持续订阅 `/gas/readings` 和 `/gas/risk_event`：

- 当前命名巡检点通过 `/inspection_manager/current_waypoint` 发布；
- 气体节点可把巡检点名称、采样时间和历史 TF 位姿关联保存；
- 严重风险事件可以取消当前 Nav2 目标并停止本轮巡检；
- 后续相机节点和后端节点可根据同一 `RiskEvent.event_id` 关联图片与报警记录。

六轴 IMU 和轮式里程计都会累积误差，初始化位姿只是定位起点，不代替 AMCL 的激光
匹配。正式巡检前仍应确认地图、激光、TF、代价地图和机器人实际位置一致。
