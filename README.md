GasRobot 自主巡检操作指南
==========================
1.在 RViz 中获取巡检点坐标
打开终端 ：
source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash
ros2 topic echo /clicked_point
然后：
点击 RViz 顶部的 Publish Point

1. 启动基础导航与 RViz

打开终端1，执行：

cd /home/book/ros2_study/gasrobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav \
  map:=/home/book/ros2_study/gasrobot_ws/src/gasrobot_gas_mapping/maps/home_map.yaml \
  enable_inspection:=false \
  enable_rviz:=true \
  use_sim_time:=false

2. 在 RViz 中设置初始位姿

3. 检查定位 TF

打开终端2，执行：

source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash
ros2 run tf2_ros tf2_echo map base_footprint

如果持续输出 Translation 和 Rotation，说明定位 TF 已建立。检查完成后按 Ctrl+C。

二、配置自主巡检路线

编辑巡检路线
/home/book/ros2_study/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml
 朝向定义
yaw_deg = 0：面向地图 X 正方向
yaw_deg = 90：面向地图 Y 正方向
yaw_deg = 180：面向地图 X 负方向
yaw_deg = -90：面向地图 Y 负方向
4. 检查配置
cat /home/book/ros2_study/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml
必须确认：
site_configured: true
两个巡检点的 x、y 正确
每个 id 唯一
YAML 缩进正确，只使用空格，不使用 Tab

cd /home/book/ros2_study/gasrobot_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select \
  gasrobot_interfaces \
  gasrobot_navigation \
  gasrobot_bringup \
  --symlink-install

source install/setup.bash

正常结果应包含：
Summary: 3 packages finished

只修改 inspection_routes.yaml 并使用 --symlink-install 时，通常不需要重新编译；运行中的巡检节点可通过 reload_routes 重新读取配置。


四、启动完整导航和巡检系统
==========================

打开终端1，执行：

cd /home/book/ros2_study/gasrobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav \
  map:=/home/book/ros2_study/gasrobot_ws/src/gasrobot_gas_mapping/maps/home_map.yaml \
  enable_inspection:=true \
  inspection_route_file:=/home/book/ros2_study/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml \
  default_route:=standard_route \
  auto_set_initial_pose:=false \
  auto_start_inspection:=false \
  enable_rviz:=true \
  use_sim_time:=false

参数说明：
mode:=nav：使用已有地图导航。
map:=...home_map.yaml：指定当前地图。
enable_inspection:=true：启动巡检管理节点。
default_route:=standard_route：选择标准巡检路线。
auto_set_initial_pose:=false：使用 RViz 人工设置初始位姿。
auto_start_inspection:=false：启动后不立即运动。
enable_rviz:=true：显示 RViz，便于首次调试。
use_sim_time:=false：使用实车系统时间。

该终端需要保持运行，不要关闭。


五、设置并检查初始定位
======================

1. 在 RViz 使用“2D Pose Estimate”设置机器人真实位置和朝向。
2. 等待激光扫描与地图墙体重合。
3. 打开终端2并检查：

source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash

ros2 run tf2_ros tf2_echo map base_footprint

看到持续坐标输出后按 Ctrl+C。

检查 Nav2：

ros2 action list | grep navigate_to_pose
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server

预期结果：
/navigate_to_pose
bt_navigator：active [3]
controller_server：active [3]

检查巡检节点和接口：

ros2 node list | grep inspection_manager
ros2 service list | grep inspection_manager
ros2 action list -t | grep execute_inspection

应当存在：
/inspection_manager
/inspection_manager/start_default
/inspection_manager/cancel
/inspection_manager/pause
/inspection_manager/reload_routes
/inspection_manager/set_initial_pose
/execute_inspection [gasrobot_interfaces/action/ExecuteInspection]


六、启动自动巡检

在终端2执行：

ros2 service call \
  /inspection_manager/start_default \
  std_srvs/srv/Trigger \
  '{}'

正常返回：
success: true
message: 默认路线启动请求已提交：standard_route

执行顺序：
inspection_point_01
到点停留 8 秒
inspection_point_02
到点停留 8 秒
任务完成


七、查看巡检状态
================

打开终端3：

source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash

查看任务状态：

ros2 topic echo /inspection_manager/state

查看当前巡检点：

ros2 topic echo /inspection_manager/current_waypoint

查看任务是否活动：

ros2 topic echo /inspection_manager/active

常见状态：
IDLE：等待任务
WAITING_NAV2：等待 Nav2
NAVIGATING：正在前往巡检点
DWELLING：正在巡检点停留采样
PAUSED：任务暂停
COMPLETED：任务完成
FAILED：任务失败
CANCELLED：任务已取消
SAFETY_STOP：严重气体风险触发停机


八、暂停、继续和取消
====================

暂停：

ros2 service call \
  /inspection_manager/pause \
  std_srvs/srv/SetBool \
  '{data: true}'

继续：

ros2 service call \
  /inspection_manager/pause \
  std_srvs/srv/SetBool \
  '{data: false}'

紧急取消：

ros2 service call \
  /inspection_manager/cancel \
  std_srvs/srv/Trigger \
  '{}'


九、运行中修改巡检点
====================

1. 如果任务正在运行，先取消：

ros2 service call /inspection_manager/cancel std_srvs/srv/Trigger '{}'

2. 修改配置：

nano /home/book/ros2_study/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml

3. 任务停止后重新加载：

ros2 service call /inspection_manager/reload_routes std_srvs/srv/Trigger '{}'

正常返回：
success: true
message: 巡检路线已重新加载

4. 再次启动：

ros2 service call /inspection_manager/start_default std_srvs/srv/Trigger '{}'

说明：reload_routes 只重新读取路线文件，不会重新启动 Nav2，也不会让机器人立即运动。巡检执行期间不允许重新加载。


十、继续添加新的巡检点
====================

1. 新开终端：

source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash
ros2 topic echo /clicked_point

2. 在 RViz 点击“Publish Point”，然后点击地图中的目标位置。
3. 记录输出中的 frame_id、x 和 y，frame_id 必须是 map。
4. 将新的点追加到 waypoints，例如：

      - id: inspection_point_03
        description: 第三个气体巡检位置
        x: 实际X坐标
        y: 实际Y坐标
        yaw_deg: 实际目标朝向
        dwell_sec: 8.0

5. 保存后调用 reload_routes。




十二、日常运行的最短流程
========================

1. 启动完整系统：

cd /home/book/ros2_study/gasrobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav \
  map:=/home/book/ros2_study/gasrobot_ws/src/gasrobot_gas_mapping/maps/home_map.yaml \
  enable_inspection:=true \
  inspection_route_file:=/home/book/ros2_study/gasrobot_ws/src/gasrobot_navigation/config/inspection_routes.yaml \
  default_route:=standard_route \
  auto_set_initial_pose:=false \
  auto_start_inspection:=false \
  enable_rviz:=true \
  use_sim_time:=false

2. 在 RViz 设置“2D Pose Estimate”。

3. 检查定位：

ros2 run tf2_ros tf2_echo map base_footprint

4. 启动巡检：

ros2 service call /inspection_manager/start_default std_srvs/srv/Trigger '{}'

5. 紧急取消：

ros2 service call /inspection_manager/cancel std_srvs/srv/Trigger '{}'


安全注意事项
============

1. 第一次测试必须打开 RViz 并降低机器人速度。
2. 机器人运动范围内不得站人。
3. 不要把未经验证的坐标直接用于自动巡检。
4. 启动巡检前必须确认激光与地图重合。
5. 定位漂移时立即取消，不要让机器人继续执行路线。
6. 正式气体实验必须遵守实验室安全规范和导师批准的测试条件。


## 模块边界

- 硬件通信与底盘状态管理归入 `gasrobot_base`。
- 气体检测算法归入 `gasrobot_gas`。
- SLAM、地图资源、气体地图及历史位姿补偿归入 `gasrobot_gas_mapping`。
- AMCL、Nav2 参数及巡检路线启动归入 `gasrobot_navigation`。
- 跨软件包启动编排归入 `gasrobot_bringup`。
- 第三方源码统一隔离在 `src/vendor`。
