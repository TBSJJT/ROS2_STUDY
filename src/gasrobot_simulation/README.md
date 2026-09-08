# gasrobot_simulation

ROS 2 Humble / Gazebo Classic 的四轮差速仿真运行包。共用模型由
`gasrobot_description` 提供；本包负责场景、仿真启动和控制器配置。

## 启动命令

```bash
cd /home/book/Study_BackUp/ros2_study/gasrobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to gasrobot_simulation --symlink-install
source install/setup.bash
ros2 launch gasrobot_simulation gazebo_sim.launch.py
```

不显示 Gazebo 窗口时，在启动命令末尾添加 `gui:=false`；指定其他场景时添加
`world:=/绝对路径/场景.world`。修改机器人模型后需要重新启动仿真。

另一终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/book/Study_BackUp/ros2_study/gasrobot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0
```

英文输入法下，`i` 前进、`,` 后退、`j/l` 原地转向、`k` 停止。
键盘终端必须获得焦点。检查控制器：

```bash
ros2 control list_controllers
ros2 topic echo /odom --once
```

关节状态和差速控制器默认激活；力矩控制器加载但不激活。
力矩模式与差速模式不同时使用，零力矩表示不驱动，不能当作主动制动。

## 目录与阅读顺序

1. `launch/gazebo_sim.launch.py`：启动模型发布、Gazebo、模型生成与顺序加载控制器。
2. `config/gasrobot_ros2_controller.yaml`：四轮关节分组、全轮距 0.266 米、半径 0.04 米与速度限制。
3. `gasrobot_description/urdf/gasrobot/gas_robot.urdf.xacro`：共用几何及仿真开关。
4. `gasrobot_description/urdf/gasrobot/plugins/gasrobot_ros2_control.xacro`：仿真硬件接口及话题映射。
5. `gasrobot_description/urdf/gasrobot/plugins/gazebo_sensor_plugin.xacro`：二维雷达、IMU 和 RGB 相机。
6. `worlds/L_model.world`、`models/L_model/`：现有场景和模型资源。

## 模块边界

仿真启动显式传入 `simulation:=true` 和控制器配置路径。默认展开共用模型时
不会生成 ros2_control 或 Gazebo 传感器插件，实车模型发布不依赖 Gazebo 运行库。
共用模型中的接触摩擦等 Gazebo 扩展标签仍保留，普通 URDF 消费者会忽略它们。

本包只启动运动和传感器仿真，不自动启动实车串口、Nav2 或巡检任务。
后续接入导航时应共用业务包，所有参与节点使用仿真时钟，并使用与仿真场景匹配的地图、初始位姿和路线。
气体扩散、气源定位与后端回传尚不能因本包启动成功而视为已经实现。

旧命令 `ros2 launch gasrobot_description gazebo_sim.launch.py` 保留兼容转发；
仍需构建本包。历史手写模型移至 description 的 `docs/legacy_models/`，不参与安装。
