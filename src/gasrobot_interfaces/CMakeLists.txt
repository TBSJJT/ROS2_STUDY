cmake_minimum_required(VERSION 3.8)
project(gasrobot_interfaces)

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
