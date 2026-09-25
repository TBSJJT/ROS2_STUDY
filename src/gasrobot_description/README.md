# gasrobot_description

车间配送机器人的模型与可视化资源包，提供底盘、四个车轮、雷达、相机和 IMU 的模型。

## 内容

- `urdf/gasrobot/gas_robot.urdf.xacro`：主 Xacro 模型。
- `urdf/gasrobot/base.urdf.xacro`：底盘模型。
- `urdf/gasrobot/wheel/`：轮组模型。
- `urdf/gasrobot/sensor/`：相机、IMU 和雷达模型。
- `launch/description.launch.py`：启动 `robot_state_publisher` 和可选的
  `joint_state_publisher`。
- `config/show_robot_model.rviz`：模型显示配置。

## 使用

```bash
ros2 launch gasrobot_description description.launch.py
```

检查模型：

```bash
xacro src/gasrobot_description/urdf/gasrobot/gas_robot.urdf.xacro > /tmp/gasrobot.urdf
check_urdf /tmp/gasrobot.urdf
```

传感器或机械结构变更应在本包修改，运行算法和硬件通信不应放入本包。

## 仿真资源归属

Gazebo 启动、控制器 YAML 和场景已迁入 `gasrobot_simulation`，请阅读该包 README。
本包默认 `simulation:=false`，仅发布模型；仿真入口负责启用模型插件并注入配置。
旧 `gazebo_sim.launch.py` 仅为兼容转发入口，需要另外构建仿真包。
`docs/legacy_models/` 保存历史手写模型，不作为运行入口，也不安装到软件包。
