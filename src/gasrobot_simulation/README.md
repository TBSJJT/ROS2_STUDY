# gasrobot_simulation

ROS 2 Humble / Gazebo Classic 的四轮差速仿真运行包。共用模型由
`gasrobot_description` 提供；本包负责场景、仿真启动和控制器配置。

## 先选对入口

| 目的 | 命令 |
|---|---|
| 完整车间定位与配送 | `ros2 launch gasrobot_bringup delivery_sim.launch.py` |
| 只加载车间、机器人和控制器 | `ros2 launch gasrobot_simulation workshop_delivery.launch.py` |
| 通用仿真，默认旧 L_model 场景 | `ros2 launch gasrobot_simulation gazebo_sim.launch.py` |

当前已完成车间场景、实际建图及固定顺序配送。完整操作见
[配送包说明](../gasrobot_delivery/README.md)，最新结果见
[验收记录](../gasrobot_delivery/docs/acceptance.md)。下文的初版验证保留为历史记录。

## 基础仿真启动命令

```bash
cd /home/iceice/ROS_Project/study/ros_ws
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
source /home/iceice/ROS_Project/study/ros_ws/install/setup.bash
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

## 启动文件的执行过程

`workshop_delivery.launch.py` 只覆盖 world、出生位姿和仿真时钟，然后复用
`gazebo_sim.launch.py`。通用启动文件负责以下步骤：

1. 查找包安装目录，补齐 Gazebo 场景/模型资源路径。
2. 展开 Xacro，启用仿真传感器和 ros2_control 插件，发布 robot_description。
3. 启动 Gazebo，spawn_entity 从 robot_description 生成机器人。
4. 等待插件创建 controller_manager，依次加载关节状态广播器、差速控制器和未激活的力矩控制器。

加载请求通过进程退出事件串行化，但退出不等于成功；启动异常时检查 spawner 日志和
`ros2 control list_controllers`。基础入口不负责 AMCL、Nav2 或配送任务的启动。
`spawn_x/y/z` 单位为米，`spawn_yaw` 单位为弧度，均属于 world 坐标系。
0.1 m 的出生高度用于自然落地，不是稳定运行时的坐标系高度。

## 目录与阅读顺序

1. `launch/gazebo_sim.launch.py`：启动模型发布、Gazebo、模型生成与顺序加载控制器。
2. `config/gasrobot_ros2_controller.yaml`：四轮关节分组、全轮距 0.266 米、半径 0.04 米与速度限制。
3. `gasrobot_description/urdf/gasrobot/gas_robot.urdf.xacro`：共用几何及仿真开关。
4. `gasrobot_description/urdf/gasrobot/plugins/gasrobot_ros2_control.xacro`：仿真硬件接口及话题映射。
5. `gasrobot_description/urdf/gasrobot/plugins/gazebo_sensor_plugin.xacro`：二维雷达、IMU 和 RGB 相机。
6. `worlds/L_model.world`、`models/L_model/`：现有场景和模型资源。

## 模块边界

仿真启动显式传入 `simulation:=true` 和控制器配置路径。默认展开共用模型时
不会生成 ros2_control 或 Gazebo 传感器插件，仅发布模型时不依赖 Gazebo 运行库。
共用模型中的接触摩擦等 Gazebo 扩展标签仍保留，普通 URDF 消费者会忽略它们。

本包只启动运动和传感器仿真；定位、导航和配送任务由 bringup 入口组合。
定位配送使用 bringup 的 `delivery_sim.launch.py` 组合入口，所有参与节点使用仿真时钟，并加载车间地图、初始位姿和工位配置。

旧命令 `ros2 launch gasrobot_description gazebo_sim.launch.py` 保留兼容转发；
仍需构建本包。历史手写模型移至 description 的 `docs/legacy_models/`，不参与安装。

## 车间多工位配送场景

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to gasrobot_simulation --symlink-install
source install/setup.bash
ros2 launch gasrobot_simulation workshop_delivery.launch.py
```

使用 `gui:=false` 可无窗口运行。出生点默认为物料站 `(1.5, 1.5, 0.1)`，
朝向 +X；可通过 `spawn_x/y/z/yaw` 覆盖，yaw 单位为弧度。
通用 `gazebo_sim.launch.py` 仍默认加载旧场景及原出生位姿。

`worlds/workshop_delivery_v1.world` 使用独立 SDF 几何，无外部模型下载。
墙内净范围 x=0～12、y=0～10 米；墙厚 0.15 米、高 1.5 米，向作业区外延伸。
六台设备分两排，每排三台，尺寸均为 2×1.6×1 米：
第一排 A(3.5,3)、B(6.5,3)、C(9.5,3)；第二排 D(3.5,7)、E(6.5,7)、F(9.5,7)。
同排设备间净宽 1 米、两排间净宽 2.4 米，上下环道净宽 2.2 米。
左下角物料站由绿色三层料架、橙色料箱和绿色地面停靠区组成；
料架中心 `(1.5,0.45)`，停靠/出生点 `(1.5,1.5)`，料架不占用停靠区。
黄色地标为六个工位，与 A～F 一一对应；箭头表示固定停靠朝向。
地标没有碰撞体，料架和设备具有碰撞体。
Gazebo 模型树可通过 `material_station_rack`、`dock_depot`、`dock_S1`～`dock_S6` 识别。

| 停靠点 | world 坐标 (x,y)，米 | yaw |
|---|---|---|
| 物料站 | (1.5,1.5) | 0 |
| S1 / S2 / S3 | (3.5,1.5) / (6.5,1.5) / (9.5,1.5) | +π/2 |
| S4 / S5 / S6 | (3.5,8.5) / (6.5,8.5) / (9.5,8.5) | −π/2 |

这些是场景设计坐标。建图后需在 RViz 的 map 坐标系重新标定目标并验证导航；
此入口只加载 Gazebo、机器人与控制器，不启动配送任务。

### 初版验证（2026-09-19，四设备版本）

- Gazebo Classic 11.10.2：`gz sdf -k` 通过。
- `colcon build --packages-up-to gasrobot_simulation --symlink-install` 成功。
- 无窗口启动成功加载新 world，并成功生成 gasrobot；关节状态广播器和差速
  控制器激活，力矩控制器加载但未激活。
- 尚未验证 GUI 视觉效果、手动行驶至各工位、SLAM 和 Nav2 到点导航。

### 六设备修订验证

`gz sdf -k` 通过；已检查六台设备、六个无碰撞工位标记及带碰撞体的物料架。
安装目录通过符号链接同步新 world；本次尚未重新验证 GUI 和导航。

### 已完成车间 SLAM 建图

已驾驶机器人覆盖外围与中央通道并返回物料站，生成
`gasrobot_gas_mapping/maps/workshop_delivery_v1.yaml` 和 `.pgm`。
地图服务器重载验证通过，详见 [建图记录](../gasrobot_gas_mapping/docs/workshop_delivery_v1_mapping.md)。
此验证补充上文初版记录；最新 Nav2 工位测试进度以 [配送验收记录](../gasrobot_delivery/docs/acceptance.md) 为准。

### 定位导航与固定顺序配送

统一入口：`ros2 launch gasrobot_bringup delivery_sim.launch.py`。
操作说明见 [配送包](../gasrobot_delivery/README.md)。车间 world 增加了 Gazebo 状态发布插件，
仅供验收记录真实位姿；几何布局未变化，配送控制仍完全依赖 AMCL 和 Nav2。
