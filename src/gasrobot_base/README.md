# gasrobot_base：底盘通信组件

本包保存串口通信、反馈解析、里程计与 IMU 转换等底盘组件。
当前车间仿真由 Gazebo 的 ros2_control 插件和差速控制器提供运动接口，
`delivery_sim.launch.py` 不启动本包的串口桥接节点。

## 与配送仿真的关系

- 仿真使用 `/cmd_vel` 接收速度指令，使用 `/odom` 提供轮式里程计。
- odom → base_footprint 的 TF 由仿真差速控制器发布。
- 调整仿真轮径、轮距和控制器限制，应修改
  `gasrobot_simulation/config/gasrobot_ros2_controller.yaml`。
- 不要在同一仿真中额外启动另一个里程计或底盘 TF 发布者。

## 源码范围

`gasrobot_base/` 下的协议、串口、参数和里程计实现仍保留，便于后续底盘接入。
本 README 不提供当前仿真不使用的设备部署流程。

车间使用入口见 [工作空间 README](../../README.md)，
运动仿真说明见 [gasrobot_simulation](../gasrobot_simulation/README.md)。
