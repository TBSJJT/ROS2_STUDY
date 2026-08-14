cmake_minimum_required(VERSION 3.8)
project(gasrobot_interfaces)

# 该虚拟机曾保留 Python 3.6 的旧 CMake 缓存。接口类型支持必须与当前 Humble
# 使用的 Python 3.10 ABI 一致，否则 rclpy 无法导入生成的 Action/Message 模块。
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH
  "rosidl_generator_py 使用的 Python 解释器" FORCE)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 接口按“采样—位姿关联—风险事件—任务与后端交互”的数据链组织。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GasReading.msg"
  "msg/LocatedGasReading.msg"
  "msg/RiskEvent.msg"
  "msg/GasSensorArray.msg"
  "srv/CalibrateGasSensor.srv"
  "srv/ReportRiskEvent.srv"
  "action/ExecuteInspection.action"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

# 下游运行时需要由 rosidl 生成的类型支持库。
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  # 自动检查接口包的 CMake、XML 和版权规范。
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
