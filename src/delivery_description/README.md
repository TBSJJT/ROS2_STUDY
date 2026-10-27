# delivery_description

车间配送机器人的模型与可视化资源包，提供底盘、四个车轮、雷达、相机和 IMU 的模型。

## 内容

- `urdf/delivery_robot/delivery_robot.urdf.xacro`：主 Xacro 模型。
- `urdf/delivery_robot/base.urdf.xacro`：底盘模型。
- `urdf/delivery_robot/wheel/`：轮组模型。
- `urdf/delivery_robot/sensor/`：相机、IMU 和雷达模型。
- `launch/description.launch.py`：启动 `robot_state_publisher` 和可选的
  `joint_state_publisher`。
- `config/show_robot_model.rviz`：模型显示配置。

## 使用

```bash
ros2 launch delivery_description description.launch.py
```

检查模型：

```bash
xacro src/delivery_description/urdf/delivery_robot/delivery_robot.urdf.xacro > /tmp/delivery_robot.urdf
check_urdf /tmp/delivery_robot.urdf
```

传感器或机械结构变更应在本包修改，运行算法和硬件通信不应放入本包。

## 仿真资源归属

Gazebo 启动、控制器 YAML 和场景已迁入 `delivery_simulation`，请阅读该包 README。
本包默认 `simulation:=false`，仅发布模型；仿真入口负责启用模型插件并注入配置。
模型包内的 `gazebo_sim.launch.py` 仅为兼容转发入口，需要另外构建仿真包。
`docs/legacy_models/` 保存历史手写模型，不作为运行入口，也不安装到软件包。

历史 `docs/` 保留原始记录，其中的旧包名反映当时版本；当前运行使用本 README 的 `delivery_*` 命令。
