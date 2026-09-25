# gasrobot_delivery

固定顺序车间配送：一台机器人、一个物料站、六个工位，人工装货以“开始任务”确认，
卸货以 ROS 仿真时间等待模拟。Nav2 负责全部运动控制。

已通过六工位往返、跨排、真实暂停/取消、短路线及连续三批完整配送验收，
结果与限制见 [验收记录](docs/acceptance.md)。

## 一键启动

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to gasrobot_bringup --symlink-install
source install/setup.bash
ros2 launch gasrobot_bringup delivery_sim.launch.py
```

先启动 Gazebo，8 秒后开始 AMCL 和 Nav2 初始化，同时启动 RViz 和配送管理器；**不会自动开始配送**。
服务器可加 `gui:=false rviz:=false`。RViz 使用 map 固定坐标系，显示工位文字和朝向。
定位模式加载已有地图，使用 Gazebo 的雷达和里程计，不同时启动 SLAM。

默认物料站出生配置自动初始化 AMCL。更改出生位姿或地图时须加
`auto_initial_pose:=false`，通过 RViz 的 2D Pose Estimate 初始化。
自定义地图还需提供针对该地图标定的 `stations_file`。

## 开始与控制

另一个终端加载相同 ROS 环境：

```bash
ros2 service call /delivery_manager/start std_srvs/srv/Trigger '{}'
ros2 topic echo /delivery_manager/status
```

开始表示装货确认；必须在物料站附近停车、定位可用且任务空闲。
默认完整路线：`depot → S1 → S2 → S3 → S6 → S5 → S4 → depot`。
每站停车卸货 5 秒，返回物料站才结束整批。执行期间不要通过 RViz、键盘或其他客户端发送额外导航/速度命令。

```bash
ros2 service call /delivery_manager/pause std_srvs/srv/Trigger '{}'
ros2 service call /delivery_manager/resume std_srvs/srv/Trigger '{}'
ros2 service call /delivery_manager/cancel std_srvs/srv/Trigger '{}'
```

暂停导航先取消旧目标，状态到 PAUSED 后才能继续；继续重发未完成目标。
暂停卸货保留剩余服务时间。取消请求不等于已停车，应等待 CANCELLED；取消期间拒绝新任务。
180 秒导航超时后取消旧目标，最多重试一次；仍失败则 FAILED，不跳过失败工位。
取消确认超时会进入 FAILED 并锁住新任务，直到旧目标真正结束。
Gazebo 暂停不推进导航超时或服务时间，仿真时钟回退会终止当前任务。
重启不会自动续跑；取消或失败后如果不在物料站，需要先通过 Nav2 返回物料站。

## 短路线与配置

首次启动可指定短批次：

```bash
ros2 launch gasrobot_bringup delivery_sim.launch.py \
  batch_file:="$(ros2 pkg prefix gasrobot_delivery)/share/gasrobot_delivery/config/batch_short.yaml"
```

短路线为 `depot → S1 → S3 → S5 → depot`。同一系统内，任务空闲时可切换下一批配置：

```bash
ros2 param set /delivery_manager batch_file \
  "$(ros2 pkg prefix gasrobot_delivery)/share/gasrobot_delivery/config/batch.yaml"
```

每次 start 重新读取批次文件；运行时拒绝切换。工位和地图等其他参数须重启更改。

- `config/stations.yaml`：地图坐标中的固定工位位姿、地图图像哈希和标定依据。
- `config/batch.yaml`：批次标识、唯一工位顺序、服务时间、导航超时、重试次数。
- `config/nav2_workshop.yaml`：车间仿真专用 AMCL、Navfn 和 RPP 参数。

工位首先与保存地图的设备边界做刚体配准，再用实际导航验收；绝不是直接复制 world 坐标。
加载时检查地图哈希、有限数值及每个停靠点周围 0.30 m 的已知自由空间。

## 日志与接口

启动参数 `output_dir` 默认是当前目录下的 `runs/`，建议始终从工作空间根目录启动。
每批日志目录具有唯一时间戳，保存：

```text
metadata.json / stations.yaml / batch.yaml / map.yaml / 地图图像 / nav2_params.yaml
simulation_*    # 世界、车轮接触参数和控制器文件快照
events.csv       # 出发、导航结果、到站、卸货完成、重试、暂停、返回、失败
trajectory.csv   # map 位姿、odom 位姿和速度
summary.json     # 最终状态、已交付工位、耗时、轮式里程计距离
```

任务耗时包含业务暂停，里程按 odom 的连续位置增量积分，不把 AMCL 跳变算进行驶距离。
轮式里程计距离可能包含滑移误差，不当作 Gazebo 真值距离。失败日志同样保留。
状态话题 `/delivery_manager/status` 为持久化 QoS 的 JSON String；
`/delivery_manager/stations` 为 MarkerArray。四个控制服务均使用 std_srvs/Trigger。

Gazebo `/gazebo/model_states` 只用于验收脚本的穿模检查和定位误差评估，配送节点不订阅真值。

## 验证

```bash
colcon test --packages-select gasrobot_delivery
colcon test-result --test-result-base build/gasrobot_delivery --verbose
ROS_DOMAIN_ID=42 python3 src/gasrobot_delivery/scripts/validate_manager_protocol.py
python3 src/gasrobot_delivery/scripts/validate_sim.py --mode navigation
python3 src/gasrobot_delivery/scripts/validate_sim.py --mode controls \
  --batch-file src/gasrobot_delivery/config/batch_short.yaml
python3 src/gasrobot_delivery/scripts/validate_sim.py --mode batch \
  --batch-file src/gasrobot_delivery/config/batch_short.yaml
python3 src/gasrobot_delivery/scripts/validate_sim.py --mode batch --repeat 3 \
  --batch-file src/gasrobot_delivery/config/batch.yaml
```

导航验证默认遍历六工位往返和一条跨排路线；批次验证使用当前管理器加载的批次。
`controls` 模式会实际开始、暂停和取消测试任务，结束后返回物料站；
它还短暂暂停 Gazebo 以检查仿真时钟。验证脚本要求独占仿真任务，输出在 `runs/validation_*`；到点后检查停车、地图位姿误差、
以及机器人包络与墙体、设备、料架是否重叠。它不会把真值传给导航。
正式验收结果见 [验收记录](docs/acceptance.md) 与 [机器可读记录](docs/acceptance_results.json)。

可用 `scripts/plot_runs.py` 将相同地图的多个批次画到一张图上（需要 python3-matplotlib）：

```bash
python3 src/gasrobot_delivery/scripts/plot_runs.py runs/批次目录1 runs/批次目录2 \
  --output runs/trajectories.png
```

图中轨迹来自 AMCL 的 map 位姿；总距离仍采用日志中的轮式里程计统计。

### 双窗口过程截图

本次已完成一批 GUI 复测并保存 15 张原始双窗口截图：
[素材索引、图注及本次结果](../../artifacts/workshop_delivery_gui_20260921/README.md)。

在同一 X11 桌面并排显示 RViz 地图和 Gazebo 场景，隐藏不需要的配置面板，
确认六工位、物料站和机器人可见。截图期间不要用其他窗口遮挡，也不要手动发送导航目标。
截图程序依赖系统的 `python3-pyqt5`，仅订阅配送状态，不改变机器人运动。

终端一启动图形仿真，终端二启动截图记录器，终端三开始验收；每个终端均需先
加载 ROS 和工作空间环境，并设置相同的 `ROS_LOCALHOST_ONLY=1`：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

# 终端一：启动后完成窗口布局。
ros2 launch gasrobot_bringup delivery_sim.launch.py gui:=true rviz:=true

# 终端二：必须选择一个新的空目录，避免覆盖已有实验素材。
python3 src/gasrobot_delivery/scripts/capture_delivery.py --output runs/screenshots_新批次

# 终端三：截图器就绪后，执行一批完整六工位配送。
python3 src/gasrobot_delivery/scripts/validate_sim.py --mode batch --repeat 1 \
  --batch-file src/gasrobot_delivery/config/batch.yaml
```

每张原始 PNG 同名附带 JSON，记录配送状态、目标、已完成工位、地图估计位姿、
ROS 时间和实际截图时间。状态采样与图形渲染是异步的，不能将截图用作毫秒级时序证明；
卸货时间和批次成功以事件日志及验收结果为准。`SERVICING` 表示正在模拟卸货，
并不表示该工位已经交付完成。记录器在完成或失败终态截图后退出。

### 连续真值记录、逐段融合与途中截图（推荐用于实验）

[2026-09-22 实际复测素材与七段途中截图](../../artifacts/workshop_delivery_gui_20260922/README.md)
已保存；整批真值轨迹由七段原始 CSV 融合，并通过完整行序列及 SHA256 一致性校验。

`plot_runs.py` 的旧图读取 `trajectory.csv` 中的 AMCL 估计位置，不能当作 Gazebo 真值。
需要可逐段追溯的实际运行图时，使用下面的只读采集流程。先在同一屏幕摆好 RViz 和
Gazebo，再启动记录器，最后调用完整批次验收（命令见上节）。输出目录必须为空。

```bash
# 终端二：需先 source ROS、工作空间，并与仿真使用相同的 ROS 域/localhost 设置。
python3 src/gasrobot_delivery/scripts/record_delivery_evidence.py \
  --output artifacts/新的实验目录

# 终端三：开始一批配送；不要同时运行其他导航或验收脚本。
python3 src/gasrobot_delivery/scripts/validate_sim.py --mode batch --repeat 1 \
  --batch-file src/gasrobot_delivery/config/batch.yaml

# 等待采集器保存 COMPLETED 截图并退出后，逐段导出、合并和绘图。
python3 src/gasrobot_delivery/scripts/plot_delivery_evidence.py artifacts/新的实验目录
```

- `actual_samples.csv` 连续保存 `/gazebo/model_states` 中机器人的实际位置，
  同时旁录 AMCL 的 map 估计位置。ModelStates 无时间戳，记录使用**接收时的 ROS 时间**。
- `plans.jsonl` 保存各段收到的 Nav2 规划路径。规划和实际位置分别记录，绝不将规划线
  当作已执行轨迹；`planned_segments.png` 只展示每段第一条有效规划。
- `segments/` 中七份 CSV 按原始顺序拼接成 `merged_actual_trajectory.csv`；
  绘图脚本强制校验合并后的所有行与原始连续记录完全相等，并保存 SHA256。
- `actual_trajectory_merged.png/.pdf` 使用上述七段真值采样绘制；不平滑、不按工位
  生成轨迹，超过 0.25 ROS 秒的采样间隔会断线。真值叠加到地图时只使用已有的固定
  world→map 标定，不根据 AMCL 或轨迹形状重新拟合。
- 途中截图以“实际已走距离达到该段首条有效规划长度的 50%，且离目标超过 0.6 m”触发，
  原地转向不会算作行驶；共七段，包含 depot→S1 和 S4→depot。额外记录六次卸货和完成。
  若某段没有满足触发条件，就保留缺失事实，不用到站截图冒充途中截图。
- RViz 配置新增 `/delivery_evidence/actual_path`：紫红色是累计 Gazebo 真值轨迹，
  绿色是当前规划线。记录器只做观察与可视化，不参与定位导航控制。
- `trajectory_provenance.json` 保存采样数量、缺口、逐段行号、哈希和合并一致性。
  `truth_vs_amcl.png` 用于查看真值配准轨迹与定位估计的差异。

### 仿真模型与控制器说明

四轮底盘采用滑移转向。车轮惯量使用 Y 轴版本，摩擦主方向使用 collision 局部轮轴，
避免随轮子滚动产生不稳定的摩擦方向。控制器选择 RPP，原地转向角速度为 0.4 rad/s，
线速度上限仍为 0.15 m/s。障碍膨胀半径为 0.55 m，机器人接近障碍时减速。实际速度受动力学影响，因此不能仅把指令积分当作真实行程。

协议测试在 ROS 域 42 使用受控假导航服务器，验证的是异步时序，不能替代 Gazebo 导航验收。
所有实际验收、调参失败和未完成项均在验收记录中单独列明。
