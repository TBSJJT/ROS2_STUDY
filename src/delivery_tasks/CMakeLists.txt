cmake_minimum_required(VERSION 3.8)
project(delivery_tasks)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 纯 Python 状态机与 ROS 适配层共用一个包；配送管理器与辅助工具分别安装为可执行入口。
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS delivery_tasks/manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME delivery_manager)
# 每个工具保留 .py 名称，源码运行与安装后运行使用同一实现。
install(PROGRAMS
  scripts/capture_delivery.py
  scripts/record_delivery_evidence.py
  scripts/plot_delivery_evidence.py
  scripts/plot_runs.py
  scripts/validate_sim.py
  scripts/validate_manager_protocol.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# 仿真验收脚本需要独占运行中的 Gazebo，因此不在 colcon test 中自动驱动车辆。
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(delivery_tests test)
endif()

ament_package()
