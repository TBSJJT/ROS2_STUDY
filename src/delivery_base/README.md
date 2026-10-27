# delivery_base：底盘通信组件

本包保存串口通信、反馈解析、里程计与 IMU 转换等底盘组件。
当前车间仿真由 Gazebo 的 ros2_control 插件和差速控制器提供运动接口，
`delivery_sim.launch.py` 不启动本包的串口桥接节点。

## 与配送仿真的关系

- 仿真使用 `/cmd_vel` 接收速度指令，使用 `/odom` 提供轮式里程计。
- odom → base_footprint 的 TF 由仿真差速控制器发布。
- 调整仿真轮径、轮距和控制器限制，应修改
  `delivery_simulation/config/delivery_ros2_controller.yaml`。
- 不要在同一仿真中额外启动另一个里程计或底盘 TF 发布者。

## 源码范围

`delivery_base/` 下的协议、串口、参数和里程计实现仍保留，便于后续底盘接入。
实车使用 `ros2 launch delivery_bringup hardware.launch.py serial_port:=/dev/ttyUSB0`
接入底盘与雷达；串口参数见 `config/stm32_bridge.yaml`。本轮未连接硬件验证。

车间使用入口见 [工作空间 README](../../README.md)，
运动仿真说明见 [delivery_simulation](../delivery_simulation/README.md)。
